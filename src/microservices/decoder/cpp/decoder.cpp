#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "decoder.h"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <queue>
#include <vector>
#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <rte_cycles.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_mbuf.h>
#include <rte_udp.h>

struct decoder_frame_context {
    bool initialized = false;
    bool input_closed = false;
    bool decoded = false;
    bool metadata_valid = true;

    struct cam_hdr cam = {};
    struct enc_hdr enc = {};

    uint32_t expected_packet_id = 0;
    uint32_t missing_packets = 0;
    uint32_t duplicate_packets = 0;

    uint32_t rx_packets = 0;
    uint32_t rx_media_bytes = 0;
    uint32_t max_codec_backlog = 0;

    uint64_t first_arrival = 0;
    uint64_t last_arrival = 0;
};

struct codec_packet_job {
    uint32_t frame_id = 0;
    uint16_t media_len = 0;
    uint8_t media[ MTU_PAYLOAD_SIZE ] = { 0 };
};

struct decoded_frame_job {
    uint32_t frame_id = 0;
    uint8_t slot = 0;
    uint64_t service_start_cycles = 0;
    uint64_t ready_cycles = 0;
};

struct decoder_pose_state {
    float yaw = 0.0f;
    float pitch = 0.0f;
    float zoom = 1.0f;
    uint64_t timestamp = 0;
    uint64_t generation = 0;
    bool pending = false;
};

struct decoder_pose_snapshot {
    float yaw = 0.0f;
    float pitch = 0.0f;
    float zoom = 1.0f;
    uint64_t timestamp = 0;
    uint64_t generation = 0;
    bool measure_latency = false;
};

// Global application state
static struct rte_mempool *mbuf_pool;

static const struct rte_ether_addr decoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x04, 0x01 } };
static const struct rte_ether_addr sff2_decoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x04, 0x02 } };

static struct decoder_frame_context frame_contexts[ K_FRAMES ];
static struct telemetry_csv telemetry_log[ K_FRAMES ];

static std::vector< uint8_t > decoded_i420_buffers[ I420_BUFFER_COUNT ];
static bool decoded_i420_slot_free[ I420_BUFFER_COUNT ] = { true, true, true };
static struct decoded_frame_job decoded_frame_jobs[ I420_BUFFER_COUNT ];

static uint32_t decoded_frame_head = 0;
static uint32_t decoded_frame_tail = 0;
static uint32_t decoded_frame_count = 0;

static std::vector< struct host_point > reconstructed_points;

static std::queue< uint32_t > decode_frame_order;

static uint32_t highest_frame_id = 0;
static uint32_t last_frame_id = 0;

static uint32_t target_frame_id = 0;

static bool eos_received = false;
static bool decoder_eos_sent = false;
static bool csv_written = false;

static std::atomic< bool > ffmpeg_output_eof( false );

static std::atomic< bool > ffmpeg_preroll_complete( false );
static uint32_t ffmpeg_preroll_outputs = 0;

static struct decoder_pose_state current_pose;

static uint64_t session_first_arrival = 0;
static uint32_t session_first_frame = 0;
static uint64_t previous_frame_arrival = 0;
static uint32_t previous_frame_id = 0;
static double jitter_ms = 0.0;

static int ffmpeg_in[ 2 ];
static int ffmpeg_out[ 2 ];
static pid_t ffmpeg_pid = -1;

static const size_t outer_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

static struct codec_packet_job codec_queue[ QUEUE_SIZE ];

static uint32_t codec_head = 0;
static uint32_t codec_tail = 0;
static uint32_t codec_count = 0;

static pthread_t writer_thread;
static pthread_mutex_t codec_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t codec_not_empty = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t decoded_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t decoded_slot_released = PTHREAD_COND_INITIALIZER;

static bool writer_started = false;

static int ffmpeg_read_slot = -1;
static size_t ffmpeg_read_filled = 0;
static uint64_t ffmpeg_start_cycles = 0;

static std::atomic< bool > codec_eos_requested( false );
static std::atomic< bool > codec_input_closed( false );
static std::atomic< bool > codec_failed( false );

static std::atomic< uint64_t > codec_start_cycles[ K_FRAMES ];
static std::atomic< uint32_t > frame_ffmpeg_calls[ K_FRAMES ];
static std::atomic< uint32_t > frame_ffmpeg_failures[ K_FRAMES ];
static std::atomic< uint32_t > codec_drops[ K_FRAMES ];

bool pose_notification_printed = false;

// Data path & support routines
static inline void process_network_stream();
static inline void drain_codec_output();
static inline void process_node_reception();
static inline void drain_ready_frames( uint64_t timer_hz );

static inline uint32_t float_to_be( float value ) {
    uint32_t bits;

    memcpy( &bits, &value, sizeof( bits ) );

    return rte_cpu_to_be_32( bits );
}

static inline float be_to_float( uint32_t value ) {
    uint32_t bits = rte_be_to_cpu_32( value );
    float result;

    memcpy( &result, &bits, sizeof( result ) );

    return result;
}

static inline void dispatch_postroll_signal() {

    // Purpose: It registers a completion marker utilizing "POSIX" filesystem descriptors

    FILE *done = fopen( POSTROLL_PATH, "w" );

    if ( done != NULL )
        fclose( done );
}

static inline uint32_t frame_queue_depth() {
    
    // Purpose: It safely assesses the quantity of entirely populated frames awaiting "DPDK" processing synchronization
    
    pthread_mutex_lock( &decoded_mutex );
    uint32_t depth = decoded_frame_count;
    
    pthread_mutex_unlock( &decoded_mutex );
    return depth;
}

static inline void enqueue_frame_id( uint32_t frame_id ) {
    
    // Purpose: It commits a shot identifier reference to ensure linear tracking across decoupled logic pipelines
    
    pthread_mutex_lock( &decoded_mutex );
    decode_frame_order.push( frame_id );
    pthread_mutex_unlock( &decoded_mutex );
}

static inline bool dequeue_frame_id( uint32_t *frame_id ) {
    
    // Purpose: It retrieves the ensuing frame key intended for reconstruction
    
    if ( frame_id == NULL )
        return false;

    pthread_mutex_lock( &decoded_mutex );

    if ( decode_frame_order.empty() ) {
        pthread_mutex_unlock( &decoded_mutex );
        return false;
    }

    *frame_id = decode_frame_order.front();
    decode_frame_order.pop();
    pthread_mutex_unlock( &decoded_mutex );

    return true;
}

static inline bool enqueue_codec_packet( uint32_t frame_id, const uint8_t *media, uint16_t media_len, uint32_t *depth_after_enqueue ) {
    
    // Purpose: It introduces a compressed sequence chunk into the "codec" "worker" context
    
    if ( media == NULL || media_len == 0 || media_len > MTU_PAYLOAD_SIZE )
        return false;

    pthread_mutex_lock( &codec_mutex );

    if ( codec_count >= QUEUE_SIZE ) {
        pthread_mutex_unlock( &codec_mutex );
        return false;
    }

    struct codec_packet_job *job = &codec_queue[ codec_tail ];
    job -> frame_id = frame_id;
    job -> media_len = media_len;
    rte_memcpy( job -> media, media, media_len );

    codec_tail = ( codec_tail + 1 ) % QUEUE_SIZE;
    codec_count++;

    if ( depth_after_enqueue )
        *depth_after_enqueue = codec_count;

    pthread_cond_signal( &codec_not_empty );
    pthread_mutex_unlock( &codec_mutex );

    return true;
}

static inline int port_init( uint16_t port, struct rte_mempool *pool ) {
    struct rte_eth_conf port_conf = { 0 };
    int retval;

    if ( !rte_eth_dev_is_valid_port( port ) )
        return -1;

    retval = rte_eth_dev_configure( port, 1, 1, &port_conf );

    if ( retval != 0 )
        return retval;

    retval = rte_eth_rx_queue_setup( port, 0, 4096, rte_eth_dev_socket_id( port ), NULL, mbuf_pool );

    if ( retval < 0 )
        return retval;

    retval = rte_eth_tx_queue_setup( port, 0, 4096, rte_eth_dev_socket_id( port ), NULL );

    if ( retval < 0 )
        return retval;

    retval = rte_eth_dev_start( port );

    if ( retval < 0 )
        return retval;

    return 0;
}

static inline void main_header_init( struct rte_ether_hdr *eth, struct rte_ipv4_hdr *ipv4, struct rte_udp_hdr *udp, uint16_t udp_payload_len ) {
    memset( eth, 0, outer_len );

    rte_ether_addr_copy( &decoder_mac, &eth -> src_addr );
    rte_ether_addr_copy( &sff2_decoder_mac, &eth -> dst_addr );

    eth -> ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    ipv4 -> version_ihl = 0x45;
    ipv4 -> time_to_live = 64;
    ipv4 -> next_proto_id = IPPROTO_UDP;
    ipv4 -> src_addr = rte_cpu_to_be_32( DECODER_IP );
    ipv4 -> dst_addr = rte_cpu_to_be_32( SFF2_DECODER_IP );
    ipv4 -> total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + udp_payload_len );

    udp -> src_port = rte_cpu_to_be_16( DECODER_PORT );
    udp -> dst_port = rte_cpu_to_be_16( SFF2_DECODER_PORT );
    udp -> dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + udp_payload_len );
    udp -> dgram_cksum = 0;

    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );
}

static void ffmpeg_init() {
    if ( pipe( ffmpeg_in ) < 0 || pipe( ffmpeg_out ) < 0 ) {
        perror( "[SYSTEM] Error: Failed to create \"FFmpeg\" pipes...\n" );
        exit( 1 );
    }

    ffmpeg_pid = fork();

    if ( ffmpeg_pid == 0 ) {
        cpu_set_t cpuset;

        CPU_ZERO( &cpuset );
        CPU_SET( FFMPEG_CPU, &cpuset );
        sched_setaffinity( 0, sizeof( cpu_set_t ), &cpuset );

        int err_fd = open( STDERR_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666 );

        if ( err_fd >= 0 ) {
            dup2( err_fd, STDERR_FILENO );
            close( err_fd );
        }

        dup2( ffmpeg_in[ 0 ], STDIN_FILENO );
        dup2( ffmpeg_out[ 1 ], STDOUT_FILENO );

        close( ffmpeg_in[ 1 ] );
        close( ffmpeg_out[ 0 ] );

        execlp( "ffmpeg", "ffmpeg", "-hide_banner", "-loglevel", "error", "-probesize", PROBE_SIZE, "-analyzeduration", DURATION, "-flags", "low_delay", "-hwaccel", "cuda", "-c:v", "hevc_cuvid", "-vstats_file", FFMPEG_PATH, "-f", "mpegts", "-i", "-", "-f", "rawvideo", "-pix_fmt", "yuv420p", "-", NULL );
        exit( 1 );
    }

    close( ffmpeg_in[ 0 ] );
    close( ffmpeg_out[ 1 ] );

    int output_flags = fcntl( ffmpeg_out[ 0 ], F_GETFL, 0 );

    if ( output_flags >= 0 )
        fcntl( ffmpeg_out[ 0 ], F_SETFL, output_flags | O_NONBLOCK );
}

static void telemetry_to_csv() {
    struct stat st = { 0 };

    if ( stat( TELEMETRY_FOLDER, &st ) == -1 )
        if ( mkdir( TELEMETRY_FOLDER, 0777 ) == -1 ) {
            printf( "[SYSTEM] Error: Failed to create directory \"%s\"...\n", TELEMETRY_FOLDER );
            return;
        }

    FILE *f = fopen( TELEMETRY_PATH, "w" );

    if ( !f ) {
        printf( "[SYSTEM] Error: Could not open \".csv\" file for writing...\n" );
        return;
    }

    fprintf( f, "frame_id;rx_complete;tx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_media_bytes;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbps;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;arrived_points;eroded_points;valid_points;erosion_ms;reconstruction_ms;pose_ms;reconstruction_pipeline_ms;tx_duration_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;pose_control_ms;codec_queue_ms;frame_queue_ms;codec_backlog;decode_service_ms;decode_h265_ms;ffmpeg_write_calls;ffmpeg_write_failures;codec_queue_drops;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation\n" );

    for ( uint32_t i = 0; i < K_FRAMES; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        uint32_t idx = t -> frame_id - 1;

        t -> ffmpeg_write_calls = frame_ffmpeg_calls[ idx ].load( std::memory_order_relaxed );
        t -> ffmpeg_write_failures = frame_ffmpeg_failures[ idx ].load( std::memory_order_relaxed );
        t -> codec_queue_drops = codec_drops[ idx ].load( std::memory_order_relaxed );

        fprintf( f, "%u;%u;%u;%u;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%llu;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%.3f;%.3f;%u;%u;%u;%u;%u;%u;%u;%u\n", t -> frame_id, t -> rx_complete, t -> tx_complete, t -> current_skip, t -> yaw, t -> pitch, t -> zoom, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> original_points, t -> rx_media_bytes, t -> tx_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, ( unsigned long long )t -> reference_size_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> reference_throughput_mbps, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> reference_bitrate_mbps, t -> arrived_points, t -> eroded_points, t -> valid_points, t -> erosion_ms, t -> reconstruction_ms, t -> pose_ms, t -> reconstruction_pipeline_ms, t -> tx_duration_ms, t -> active_tx_ms, t -> active_process_ms, t -> reference_process_ms, t -> total_processing_ms, t -> total_residency_ms, t -> reference_residency_ms, t -> node_efficiency_pct, t -> reference_efficiency_pct, t -> gpu_transfer_ms, t -> gpu_copyback_ms, t -> host_overhead_ms, t -> camera_node_ms, t -> e2e_latency_ms, t -> schedule_delay_ms, t -> instant_jitter_ms, t -> desynced_jitter_ms, t -> pose_control_ms, t -> codec_queue_ms, t -> frame_queue_ms, t -> codec_backlog, t -> decode_service_ms, t -> decode_h265_ms, t -> ffmpeg_write_calls, t -> ffmpeg_write_failures, t -> codec_queue_drops, t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets, t -> mbuf_starvation );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static void *ffmpeg_writer_loop( void *arg ) {
    cpu_set_t cpuset;
    CPU_ZERO( &cpuset );
    CPU_SET( FFMPEG_CPU, &cpuset );
    pthread_setaffinity_np( pthread_self(), sizeof( cpu_set_t ), &cpuset );
    pthread_setname_np( pthread_self(), "decoder_ffmpeg_writer" );

    uint8_t batch[ WRITE_BATCH_SIZE ];

    while ( 1 ) {
        uint32_t batch_frame_id = 0;
        size_t batch_size = 0;
        bool close_input = false;

        pthread_mutex_lock( &codec_mutex );

        while ( codec_count == 0 && !codec_eos_requested.load( std::memory_order_acquire ) )
            pthread_cond_wait( &codec_not_empty, &codec_mutex );

        if ( codec_count == 0 && codec_eos_requested.load( std::memory_order_acquire ) )
            close_input = true;
        else {
            batch_frame_id = codec_queue[ codec_head ].frame_id;

            while ( codec_count > 0 ) {
                struct codec_packet_job *job = &codec_queue[ codec_head ];

                if ( batch_size > 0 && job -> frame_id != batch_frame_id )
                    break;

                if ( batch_size + job -> media_len > WRITE_BATCH_SIZE )
                    break;

                rte_memcpy( batch + batch_size, job -> media, job -> media_len );
                batch_size += job -> media_len;

                codec_head = ( codec_head + 1 ) % QUEUE_SIZE;
                codec_count--;
            }
        }

        pthread_mutex_unlock( &codec_mutex );

        if ( close_input ) {
            close( ffmpeg_in[ 1 ] );
            codec_input_closed.store( true, std::memory_order_release );
            break;
        }

        if ( batch_size == 0 )
            continue;

        bool preroll = batch_frame_id == FRAME_ID;

        if ( unlikely( !preroll && ( batch_frame_id == 0 || batch_frame_id > K_FRAMES ) ) )
            continue;

        uint32_t idx = 0;
        uint64_t write_start = rte_get_timer_cycles();

        if ( !preroll ) {
            idx = batch_frame_id - 1;
            uint64_t expected_zero = 0;

            codec_start_cycles[ idx ].compare_exchange_strong( expected_zero, write_start, std::memory_order_release, std::memory_order_relaxed );
        }

        size_t written_total = 0;

        while ( written_total < batch_size ) {
            if ( !preroll )
                frame_ffmpeg_calls[ idx ].fetch_add( 1, std::memory_order_relaxed );

            ssize_t written = write( ffmpeg_in[ 1 ], batch + written_total, batch_size - written_total );

            if ( written > 0 ) {
                written_total += ( size_t )written;
                continue;
            }

            if ( written < 0 && errno == EINTR )
                continue;

            if ( !preroll )
                frame_ffmpeg_failures[ idx ].fetch_add( 1, std::memory_order_relaxed );

            codec_failed.store( true, std::memory_order_release );
            break;
        }

        if ( codec_failed.load( std::memory_order_acquire ) )
            break;
    }

    if ( !codec_input_closed.load( std::memory_order_acquire ) ) {
        close( ffmpeg_in[ 1 ] );
        codec_input_closed.store( true, std::memory_order_release );
    }

    return NULL;
}

static inline void drain_codec_output() {
    while ( !ffmpeg_output_eof.load( std::memory_order_acquire ) ) {
        if ( ffmpeg_read_slot < 0 ) {
            pthread_mutex_lock( &decoded_mutex );

            for ( uint8_t slot = 0; slot < I420_BUFFER_COUNT; slot++ ) {
                if ( decoded_i420_slot_free[ slot ] ) {
                    decoded_i420_slot_free[ slot ] = false;
                    ffmpeg_read_slot = ( int )slot;
                    break;
                }
            }

            pthread_mutex_unlock( &decoded_mutex );

            if ( ffmpeg_read_slot < 0 )
                return;

            ffmpeg_read_filled = 0;
        }

        size_t request = TOTAL_YUV_SIZE - ffmpeg_read_filled;

        if ( ffmpeg_read_filled == 0 && ffmpeg_preroll_complete.load( std::memory_order_acquire ) && ffmpeg_start_cycles == 0 )
            ffmpeg_start_cycles = rte_get_timer_cycles();
        
        ssize_t bytes_read = read( ffmpeg_out[ 0 ], decoded_i420_buffers[ ffmpeg_read_slot ].data() + ffmpeg_read_filled, request );

        if ( bytes_read > 0 ) {
            ffmpeg_read_filled += ( size_t )bytes_read;
       
            process_network_stream();

            if ( ffmpeg_read_filled < TOTAL_YUV_SIZE )
                continue;

            uint8_t completed_slot = ( uint8_t )ffmpeg_read_slot;

            ffmpeg_read_slot = -1;
            ffmpeg_read_filled = 0;

            uint32_t frame_id = 0;

            if ( unlikely( !dequeue_frame_id( &frame_id ) ) ) {
                codec_failed.store( true, std::memory_order_release );

                pthread_mutex_lock( &decoded_mutex );
                decoded_i420_slot_free[ completed_slot ] = true;
                pthread_cond_signal( &decoded_slot_released );
                pthread_mutex_unlock( &decoded_mutex );

                return;
            }

            if ( frame_id == FRAME_ID ) {
                if ( !ffmpeg_preroll_complete.load( std::memory_order_acquire ) ) {
                    ffmpeg_preroll_outputs++;

                    if ( ffmpeg_preroll_outputs >= FRAMES )
                        ffmpeg_preroll_complete.store( true, std::memory_order_release );
                }

                ffmpeg_start_cycles = 0;

                pthread_mutex_lock( &decoded_mutex );
                decoded_i420_slot_free[ completed_slot ] = true;
                pthread_cond_signal( &decoded_slot_released );
                pthread_mutex_unlock( &decoded_mutex );

                return;
            }

            struct decoded_frame_job job;
            job.frame_id = frame_id;
            job.slot = completed_slot;
            job.service_start_cycles = ffmpeg_start_cycles;
            job.ready_cycles = rte_get_timer_cycles();

            ffmpeg_start_cycles = 0;

            pthread_mutex_lock( &decoded_mutex );
            
            decoded_frame_jobs[ decoded_frame_tail ] = job;
            decoded_frame_tail = ( decoded_frame_tail + 1 ) % I420_BUFFER_COUNT;
            decoded_frame_count++;
            
            pthread_mutex_unlock( &decoded_mutex );

            return;
        }

        if ( bytes_read < 0 && errno == EINTR )
            continue;

        if ( bytes_read < 0 && ( errno == EAGAIN || errno == EWOULDBLOCK ) )
            return;

        if ( bytes_read == 0 ) {
            if ( ffmpeg_read_slot >= 0 ) {
                pthread_mutex_lock( &decoded_mutex );
                
                decoded_i420_slot_free[ ffmpeg_read_slot ] = true;
                
                pthread_cond_signal( &decoded_slot_released );
                pthread_mutex_unlock( &decoded_mutex );

                if ( ffmpeg_read_filled != 0 )
                    codec_failed.store( true, std::memory_order_release );
            }

            ffmpeg_read_slot = -1;
            ffmpeg_read_filled = 0;
            ffmpeg_start_cycles = 0;
            ffmpeg_output_eof.store( true, std::memory_order_release );
            
            return;
        }

        codec_failed.store( true, std::memory_order_release );
        
        return;
    }
}

static void ffmpeg_writer_start() {
    if ( writer_started )
        return;

    int retval = pthread_create( &writer_thread, NULL, ffmpeg_writer_loop, NULL );

    if ( retval != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"FFmpeg\" thread creation failed...\n" );

    writer_started = true;
}

static void ffmpeg_preroll() {
    printf( "[SYSTEM] Pre-rolling \"NVDEC\" components with %d frames...\n\n", FRAMES );

    while ( !ffmpeg_preroll_complete.load( std::memory_order_acquire ) ) {
        process_node_reception();

        if ( codec_failed.load( std::memory_order_acquire ) )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"Codec\" path failed during pre-roll...\n" );

        rte_pause();
    }
}

static void request_codec_eos() {

    // Purpose: It notifies a termination signal broadcasting synchronization release mechanisms on completion

    pthread_mutex_lock( &codec_mutex );
    codec_eos_requested.store( true, std::memory_order_release );
    pthread_cond_broadcast( &codec_not_empty );
    pthread_mutex_unlock( &codec_mutex );
}

static void ffmpeg_writer_stop() {
    if ( !writer_started )
        return;

    request_codec_eos();
    pthread_join( writer_thread, NULL );
    writer_started = false;
}

static inline void process_pose_control( const struct pose_payload *pose ) {
    
    // Purpose: It registers asynchronous user-driven alterations dynamically adjusting downstream view logic constraints
    
    uint64_t timestamp = rte_be_to_cpu_64( pose -> timestamp );
    
    float yaw = be_to_float( pose -> yaw );
    float pitch = be_to_float( pose -> pitch );
    float zoom = be_to_float( pose -> zoom );

    if ( unlikely( timestamp == 0 || !std::isfinite( yaw ) || !std::isfinite( pitch ) || !std::isfinite( zoom ) || zoom <= 0.0f ) )
        return;

    if ( timestamp == current_pose.timestamp )
        return;

    if ( unlikely( current_pose.timestamp > 0 && timestamp < current_pose.timestamp ) )
        return;

    current_pose.yaw = yaw;
    current_pose.pitch = pitch;
    current_pose.zoom = zoom;
    current_pose.timestamp = timestamp;
    current_pose.generation++;
    current_pose.pending = true;

    printf( "[SYSTEM] Pose controller updated stance ( \"Yaw\": %.2f, \"Pitch\": %.2f, \"Zoom\": %.2f ).\n", yaw, pitch, zoom );
    pose_notification_printed = true;
}

static inline void process_network_stream() {
    struct rte_mbuf *bufs[ BURST_SIZE ];

    while ( 1 ) {
        uint16_t nb_rx = rte_eth_rx_burst( PORT_SFF2, 0, bufs, BURST_SIZE );

        if ( nb_rx == 0 )
            break;

        for ( uint16_t i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];

            const size_t minimum_packet_size = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct pose_payload );

            if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < minimum_packet_size ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

            if ( unlikely( !rte_is_same_ether_addr( &eth -> src_addr, &sff2_decoder_mac ) || !rte_is_same_ether_addr( &eth -> dst_addr, &decoder_mac ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

            if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( SFF2_DECODER_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( DECODER_IP ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

            if ( unlikely( udp -> src_port != rte_cpu_to_be_16( SFF2_DECODER_PORT ) || udp -> dst_port != rte_cpu_to_be_16( DECODER_PORT ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
            uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );

            if ( unlikely( udp_length < sizeof( struct rte_udp_hdr ) || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length || rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + ipv4_length ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t udp_payload_length = udp_length - sizeof( struct rte_udp_hdr );

            if ( udp_payload_length == sizeof( struct pose_payload ) ) {
                struct pose_payload *pose = ( struct pose_payload * )( udp + 1 );

                process_pose_control( pose );
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( udp_payload_length < sizeof( struct cam_hdr ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint64_t packet_arrival_cycles = rte_get_timer_cycles();
            struct cam_hdr *cam = ( struct cam_hdr * )( udp + 1 );
            uint32_t frame_id = rte_be_to_cpu_32( cam -> frame_id );

            if ( frame_id == FRAME_ID ) {
                if ( unlikely( eos_received || udp_payload_length < sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                struct enc_hdr *enc = ( struct enc_hdr * )( cam + 1 );
                uint16_t media_len = udp_payload_length - sizeof( struct cam_hdr ) - sizeof( struct enc_hdr );
                uint32_t packet_id = rte_be_to_cpu_32( enc -> packet_id );

                if ( unlikely( rte_be_to_cpu_32( enc -> frame_id ) != FRAME_ID || cam -> sequence_number != 0 || rte_be_to_cpu_32( cam -> original_points ) != 0 || rte_be_to_cpu_32( cam -> points_in_packet ) != 0 || media_len == 0 || media_len > MTU_PAYLOAD_SIZE || media_len % TS_PACKET_SIZE != 0 ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                if ( packet_id == 0 ) {
                    if ( highest_frame_id != 0 ) {
                        if ( target_frame_id == 0 ) {
                            target_frame_id = highest_frame_id;
                            frame_contexts[ target_frame_id - 1 ].input_closed = true;

                            if ( frame_contexts[ target_frame_id - 1 ].decoded )
                                dispatch_postroll_signal();
                        }
                    }

                    enqueue_frame_id( FRAME_ID );
                }

                if ( unlikely( !enqueue_codec_packet( FRAME_ID, ( uint8_t * )( enc + 1 ), media_len, NULL ) ) )
                    codec_failed.store( true, std::memory_order_release );

                rte_pktmbuf_free( m );
                continue;
            }

            if ( frame_id == END_OF_STREAM ) {
                if ( unlikely( udp_payload_length != sizeof( struct cam_hdr ) ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                if ( highest_frame_id > 0 )
                    frame_contexts[ highest_frame_id - 1 ].input_closed = true;

                eos_received = true;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( eos_received ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( frame_id == 0 || frame_id > K_FRAMES || udp_payload_length < sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( cam -> sequence_number != 0 || rte_be_to_cpu_32( cam -> points_in_packet ) != 0 ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t source_points = rte_be_to_cpu_32( cam -> original_points );

            if ( unlikely( source_points == 0 || source_points > MAX_SOURCE_POINTS ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct enc_hdr *enc = ( struct enc_hdr * )( cam + 1 );

            float global_scale = be_to_float( enc -> global_scale );
            float bbox_center_x = be_to_float( enc -> box_center_x );
            float bbox_center_y = be_to_float( enc -> box_center_y );
            float bbox_center_z = be_to_float( enc -> box_center_z );
            float encoder_yaw = be_to_float( enc -> yaw );
            float encoder_pitch = be_to_float( enc -> pitch );
            float final_scale = be_to_float( enc -> final_scale );
            float centroid_x = be_to_float( enc -> centroid_x );
            float centroid_y = be_to_float( enc -> centroid_y );
            float centroid_z = be_to_float( enc -> centroid_z );

            if ( unlikely( rte_be_to_cpu_32( enc -> frame_id ) != frame_id || !std::isfinite( global_scale ) || global_scale <= 0.0f || !std::isfinite( bbox_center_x ) || !std::isfinite( bbox_center_y ) || !std::isfinite( bbox_center_z ) || !std::isfinite( encoder_yaw ) || !std::isfinite( encoder_pitch ) || !std::isfinite( final_scale ) || final_scale <= 0.0f || !std::isfinite( centroid_x ) || !std::isfinite( centroid_y ) || !std::isfinite( centroid_z ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t media_len = udp_payload_length - sizeof( struct cam_hdr ) - sizeof( struct enc_hdr );

            if ( unlikely( media_len == 0 || media_len > MTU_PAYLOAD_SIZE || media_len % TS_PACKET_SIZE != 0 ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( highest_frame_id > 0 && frame_id < highest_frame_id ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct decoder_frame_context &ctx = frame_contexts[ frame_id - 1 ];
            uint32_t packet_id = rte_be_to_cpu_32( enc -> packet_id );

            if ( !ctx.initialized ) {
                if ( highest_frame_id > 0 && frame_id > highest_frame_id )
                    frame_contexts[ highest_frame_id - 1 ].input_closed = true;

                ctx = decoder_frame_context();
                ctx.initialized = true;
                ctx.metadata_valid = true;
                ctx.cam = *cam;
                ctx.enc = *enc;
                ctx.first_arrival = packet_arrival_cycles;
                ctx.last_arrival = packet_arrival_cycles;

                highest_frame_id = std::max( highest_frame_id, frame_id );
                enqueue_frame_id( frame_id );
            }
            else {
                bool camera_metadata_valid = ctx.cam.frame_id == cam -> frame_id && ctx.cam.timestamp == cam -> timestamp && ctx.cam.yaw == cam -> yaw && ctx.cam.pitch == cam -> pitch && ctx.cam.zoom == cam -> zoom && ctx.cam.temporal_skip == cam -> temporal_skip && ctx.cam.original_points == cam -> original_points;

                bool encoder_metadata_valid = ctx.enc.frame_id == enc -> frame_id && ctx.enc.global_scale == enc -> global_scale && ctx.enc.box_center_x == enc -> box_center_x && ctx.enc.box_center_y == enc -> box_center_y && ctx.enc.box_center_z == enc -> box_center_z && ctx.enc.yaw == enc -> yaw && ctx.enc.pitch == enc -> pitch && ctx.enc.final_scale == enc -> final_scale && ctx.enc.centroid_x == enc -> centroid_x && ctx.enc.centroid_y == enc -> centroid_y && ctx.enc.centroid_z == enc -> centroid_z;

                if ( unlikely( ctx.decoded || !camera_metadata_valid || !encoder_metadata_valid ) ) {
                    ctx.metadata_valid = false;
                    rte_pktmbuf_free( m );
                    continue;
                }
            }

            if ( packet_id < ctx.expected_packet_id ) {
                ctx.duplicate_packets++;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( packet_id > ctx.expected_packet_id ) {
                uint32_t missing = packet_id - ctx.expected_packet_id;
                ctx.missing_packets += missing;
            }

            ctx.expected_packet_id = packet_id + 1;

            uint8_t *media = ( uint8_t * )( enc + 1 );
            uint32_t queue_depth_after = 0;

            if ( unlikely( !enqueue_codec_packet( frame_id, media, media_len, &queue_depth_after ) ) ) {
                codec_drops[ frame_id - 1 ].fetch_add( 1, std::memory_order_relaxed );
                rte_pktmbuf_free( m );
                continue;
            }

            ctx.max_codec_backlog = std::max( ctx.max_codec_backlog, queue_depth_after );
            ctx.rx_packets++;
            ctx.rx_media_bytes += media_len;
            ctx.last_arrival = packet_arrival_cycles;

            rte_pktmbuf_free( m );
        }
    }
}

static inline void process_node_reception() {
    
    // Purpose: It uniformly coordinates data-path polling and asynchronous pipe drainage ensuring no execution deadlock occurs
    
    process_network_stream();
    drain_codec_output();
}

static inline bool flush_tx_burst( struct rte_mbuf **tx_bufs, uint16_t *tx_points_buf, int *burst_idx, struct telemetry_csv *t, uint64_t *first_tx_cycles, uint64_t *last_tx_cycles, uint64_t *active_tx_cycles ) {
    if ( *burst_idx == 0 )
        return true;

    uint16_t sent = 0;
    uint16_t zero_accept_streak = 0;
    bool is_resubmission = false;
    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint16_t requested_packets = *burst_idx - sent;

        if ( is_resubmission && t != NULL ) {
            t -> tx_resubmit_calls++;
            t -> tx_resubmitted_packets += requested_packets;
        }

        uint64_t t_tx_start = rte_get_timer_cycles();

        if ( first_tx_cycles && *first_tx_cycles == 0 )
            *first_tx_cycles = t_tx_start;

        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF2, 0, &tx_bufs[ sent ], requested_packets );
        uint64_t t_tx_end = rte_get_timer_cycles();

        if ( active_tx_cycles )
            *active_tx_cycles += t_tx_end - t_tx_start;

        if ( last_tx_cycles )
            *last_tx_cycles = t_tx_end;

        if ( nb_tx > 0 ) {

            if ( t != NULL ) {
                t -> tx_packets += nb_tx;

                for ( uint16_t j = 0; j < nb_tx; j++ )
                    t -> tx_points += tx_points_buf[ sent + j ];

                if ( nb_tx < requested_packets )
                    t -> tx_partial_accepts++;
            }

            sent += nb_tx;
            zero_accept_streak = 0;
            is_resubmission = nb_tx < requested_packets;
        }
        else {
            if ( t != NULL )
                t -> tx_zero_accepts++;

            is_resubmission = true;

            if ( ++zero_accept_streak > MAX_ZERO_ACCEPTS ) {
                for ( int k = sent; k < *burst_idx; k++ )
                    rte_pktmbuf_free( tx_bufs[ k ] );

                *burst_idx = 0;
                return false;
            }

            uint16_t pause_count = ( zero_accept_streak < pause_window ) ? zero_accept_streak : pause_window;

            for ( uint16_t p = 0; p < pause_count; p++ )
                rte_pause();
        }
    }

    *burst_idx = 0;
    return true;
}

static inline bool dispatch_reconstructed_frame( uint32_t frame_id, const struct decoder_frame_context &ctx, const struct decoder_pose_snapshot &pose, const struct host_point *points, uint32_t point_count, struct telemetry_csv *t, uint64_t *first_tx_cycles, uint64_t *last_tx_cycles, uint64_t *active_tx_cycles ) {
    
    // Purpose: It serializes output arrays into standard network blocks transmitting continuous elements up to the sequence limit
    
    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    uint16_t tx_points_buf[ BURST_SIZE ];
    int burst_idx = 0;

    bool success = true;

    struct dec_hdr output_dec = { 0 };
    output_dec.frame_id = rte_cpu_to_be_32( frame_id );
    output_dec.timestamp = ctx.cam.timestamp;
    output_dec.yaw = float_to_be( pose.yaw );
    output_dec.pitch = float_to_be( pose.pitch );
    output_dec.zoom = float_to_be( pose.zoom );
    output_dec.temporal_skip = ctx.cam.temporal_skip;
    output_dec.original_points = ctx.cam.original_points;
    output_dec.arrived_points = rte_cpu_to_be_32( t -> arrived_points );
    output_dec.eroded_points = rte_cpu_to_be_32( t -> eroded_points );
    output_dec.valid_points = rte_cpu_to_be_32( point_count );
    output_dec.padding = 0;

    if ( point_count == 0 ) {
        struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );

        if ( m == NULL ) {
            t -> mbuf_starvation++;
            return false;
        }

        uint16_t packet_len = outer_len + sizeof( struct dec_hdr );
        uint8_t *data = ( uint8_t * )rte_pktmbuf_append( m, packet_len );

        if ( data == NULL ) {
            rte_pktmbuf_free( m );
            t -> mbuf_starvation++;
            return false;
        }

        struct rte_ether_hdr *eth = ( struct rte_ether_hdr * )data;
        struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );
        struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );
        struct dec_hdr *dec = ( struct dec_hdr * )( udp + 1 );

        main_header_init( eth, ipv4, udp, sizeof( struct dec_hdr ) );
        output_dec.sequence_number = 0;
        output_dec.points_in_packet = 0;
        *dec = output_dec;

        tx_bufs[ 0 ] = m;
        tx_points_buf[ 0 ] = 0;
        burst_idx = 1;

        return flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, t, first_tx_cycles, last_tx_cycles, active_tx_cycles );
    }

    uint32_t points_prepared = 0;
    uint32_t sequence_number = 0;

    while ( points_prepared < point_count ) {
        uint32_t batch = std::min( ( uint32_t )POINTS_PER_PACKET, point_count - points_prepared );
        uint16_t point_payload_len = ( uint16_t )( batch * sizeof( struct point_tx ) );
        uint16_t udp_payload_len = sizeof( struct dec_hdr ) + point_payload_len;
        uint16_t packet_len = outer_len + udp_payload_len;

        struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );

        if ( unlikely( m == NULL ) ) {
            t -> mbuf_starvation++;
            success = false;
            break;
        }

        uint8_t *data = ( uint8_t * )rte_pktmbuf_append( m, packet_len );

        if ( unlikely( data == NULL ) ) {
            rte_pktmbuf_free( m );
            t -> mbuf_starvation++;
            success = false;
            break;
        }

        struct rte_ether_hdr *eth = ( struct rte_ether_hdr * )data;
        struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );
        struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );
        struct dec_hdr *dec = ( struct dec_hdr * )( udp + 1 );
        struct point_tx *wire_points = ( struct point_tx * )( dec + 1 );

        main_header_init( eth, ipv4, udp, udp_payload_len );

        output_dec.sequence_number = rte_cpu_to_be_32( sequence_number );
        output_dec.points_in_packet = rte_cpu_to_be_32( batch );
        *dec = output_dec;

        for ( uint32_t point_id = 0; point_id < batch; point_id++ ) {
            const struct host_point &source = points[ points_prepared + point_id ];

            wire_points[ point_id ].x = float_to_be( source.x );
            wire_points[ point_id ].y = float_to_be( source.y );
            wire_points[ point_id ].z = float_to_be( source.z );
            wire_points[ point_id ].r = source.r;
            wire_points[ point_id ].g = source.g;
            wire_points[ point_id ].b = source.b;
            wire_points[ point_id ].padding = 0;
        }

        tx_bufs[ burst_idx ] = m;
        tx_points_buf[ burst_idx ] = ( uint16_t )batch;
        burst_idx++;

        points_prepared += batch;
        sequence_number++;

        if ( burst_idx == BURST_SIZE || points_prepared == point_count ) {
            if ( unlikely( !flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, t, first_tx_cycles, last_tx_cycles, active_tx_cycles ) ) ) {
                success = false;
                break;
            }

            process_node_reception();
        }
    }

    if ( burst_idx > 0 )
        if ( unlikely( !flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, t, first_tx_cycles, last_tx_cycles, active_tx_cycles ) ) )
            success = false;

    return success && points_prepared == point_count;
}

static inline bool dispatch_node_eos() {

    // Purpose: It issues the terminal condition forwarding completion intent to upstream endpoints

    struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );

    if ( m == NULL )
        return false;

    uint16_t packet_len = outer_len + sizeof( struct dec_hdr );
    uint8_t *data = ( uint8_t * )rte_pktmbuf_append( m, packet_len );

    if ( data == NULL ) {
        rte_pktmbuf_free( m );
        return false;
    }

    struct rte_ether_hdr *eth = ( struct rte_ether_hdr * )data;
    struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );
    struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );
    struct dec_hdr *dec = ( struct dec_hdr * )( udp + 1 );

    main_header_init( eth, ipv4, udp, sizeof( struct dec_hdr ) );
    memset( dec, 0, sizeof( *dec ) );
    dec -> frame_id = rte_cpu_to_be_32( END_OF_STREAM );

    struct rte_mbuf *tx_bufs[ 1 ] = { m };
    uint16_t tx_points_buf[ 1 ] = { 0 };
    int burst_idx = 1;

    return flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, NULL, NULL, NULL, NULL );
}

static inline void compile_input_metrics( uint32_t frame_id, const struct decoder_frame_context &ctx, struct telemetry_csv *t, uint64_t timer_hz ) {
    
    // Purpose: It populates fundamental latency extents parsing structural requirements
    
    uint16_t temporal_skip = rte_be_to_cpu_16( ctx.cam.temporal_skip );

    if ( temporal_skip == 0 )
        temporal_skip = 1;

    uint64_t camera_tx_cycles = rte_be_to_cpu_64( ctx.cam.timestamp );

    t -> frame_id = frame_id;
    t -> current_skip = temporal_skip;
    t -> camera_send_timestamp = ( double )camera_tx_cycles / timer_hz;
    t -> recv_start_timestamp = ( double )ctx.first_arrival / timer_hz;
    t -> original_points = rte_be_to_cpu_32( ctx.cam.original_points );
    t -> rx_media_bytes = ctx.rx_media_bytes;
    t -> rx_packets = ctx.rx_packets;
    t -> payload_bytes = ctx.rx_media_bytes;

    uint32_t expected_packets = ctx.rx_packets + ctx.missing_packets;
    t -> data_integrity_pct = ( expected_packets > 0 ) ? ( ( double )ctx.rx_packets / expected_packets ) * 100.0 : 0.0;

    double receive_sec = ( ctx.last_arrival >= ctx.first_arrival ) ? ( double )( ctx.last_arrival - ctx.first_arrival ) / timer_hz : 0.0;
    uint64_t logical_rx_bytes = ( uint64_t )ctx.rx_media_bytes + ( ctx.rx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );

    t -> internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_rx_bytes / 1000000.0 ) / receive_sec : 0.0;

    if ( session_first_arrival == 0 ) {
        session_first_arrival = ctx.first_arrival;
        session_first_frame = frame_id;
    }

    t -> camera_node_ms = ( ctx.first_arrival >= camera_tx_cycles ) ? ( ( double )( ctx.first_arrival - camera_tx_cycles ) / timer_hz ) * 1000.0 : 0.0;

    // Same jitter definition used by SFF1 / SFF2 / Encoder: instantaneous inter-arrival error plus a 1 / 16 EWMA.
    if ( previous_frame_arrival > 0 && frame_id > previous_frame_id ) {
        double real_interval_sec = ( double )( ctx.first_arrival - previous_frame_arrival ) / timer_hz;
        double expected_interval_sec = ( double )( frame_id - previous_frame_id ) / TARGET_FPS;
        double diff_sec = real_interval_sec - expected_interval_sec;

        t -> instant_jitter_ms = std::abs( diff_sec ) * 1000.0;
        jitter_ms += ( t -> instant_jitter_ms - jitter_ms ) / 16.0;
    }
    else
        t -> instant_jitter_ms = 0.0;

    t -> desynced_jitter_ms = jitter_ms;

    previous_frame_arrival = ctx.first_arrival;
    previous_frame_id = frame_id;

    uint64_t codec_write_start = codec_start_cycles[ frame_id - 1 ].load( std::memory_order_acquire );

    if ( codec_write_start >= ctx.first_arrival )
        t -> codec_queue_ms = ( ( double )( codec_write_start - ctx.first_arrival ) / timer_hz ) * 1000.0;

    t -> codec_backlog = ctx.max_codec_backlog;
    t -> ffmpeg_write_calls = frame_ffmpeg_calls[ frame_id - 1 ].load( std::memory_order_relaxed );
    t -> ffmpeg_write_failures = frame_ffmpeg_failures[ frame_id - 1 ].load( std::memory_order_relaxed );
    t -> codec_queue_drops = codec_drops[ frame_id - 1 ].load( std::memory_order_relaxed );

    t -> rx_complete = ( ctx.metadata_valid && ctx.rx_packets > 0 && ctx.missing_packets == 0 ) ? 1 : 0;

    uint64_t reference_frame_bytes = ( uint64_t )ctx.rx_media_bytes + ( ( uint64_t )ctx.rx_packets * ( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) );

    t -> reference_size_bytes = reference_frame_bytes;
    t -> reference_throughput_mbps = ( receive_sec > 0.0 ) ? ( ( double )reference_frame_bytes * 8.0 / 1000000.0 ) / receive_sec : 0.0;
    t -> reference_bitrate_mbps = ( reference_frame_bytes * 8.0 * TARGET_FPS ) / 1000000.0;
}

static inline void evaluate_decoded_frame( uint32_t frame_id, uint64_t timer_hz, const uint8_t *decoded_i420, uint64_t decode_service_start, uint64_t decoded_frame_ready ) {
    
    // Purpose: It organizes the "CUDA" point reconstruction orchestrating hardware transfers before preparing final metadata diagnosis
    
    if ( frame_id == 0 || frame_id > K_FRAMES )
        return;

    struct decoder_frame_context &ctx = frame_contexts[ frame_id - 1 ];
    struct telemetry_csv *t = &telemetry_log[ frame_id - 1 ];
    uint64_t frame_service_start = rte_get_timer_cycles();
    memset( t, 0, sizeof( *t ) );

    compile_input_metrics( frame_id, ctx, t, timer_hz );

    if ( frame_service_start >= decoded_frame_ready )
        t -> frame_queue_ms = ( ( double )( frame_service_start - decoded_frame_ready ) / timer_hz ) * 1000.0;

    uint64_t codec_write_start = codec_start_cycles[ frame_id - 1 ].load( std::memory_order_acquire );

    if ( decode_service_start > 0 && decoded_frame_ready >= decode_service_start )
        t -> decode_service_ms = ( ( double )( decoded_frame_ready - decode_service_start ) / timer_hz ) * 1000.0;

    if ( codec_write_start > 0 && decoded_frame_ready >= codec_write_start )
        t -> decode_h265_ms = ( ( double )( decoded_frame_ready - codec_write_start ) / timer_hz ) * 1000.0;

    struct decoder_pose_snapshot pose;
    pose.yaw = current_pose.yaw;
    pose.pitch = current_pose.pitch;
    pose.zoom = current_pose.zoom;
    pose.timestamp = current_pose.timestamp;
    pose.generation = current_pose.generation;
    pose.measure_latency = current_pose.pending && current_pose.timestamp > 0;

    t -> yaw = pose.yaw;
    t -> pitch = pose.pitch;
    t -> zoom = pose.zoom;

    uint32_t arrived_points = 0;
    uint32_t eroded_points = 0;
    uint32_t valid_points = 0;
    double gpu_metrics[ 5 ] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    uint64_t pose_apply_end_cycles = 0;
    uint64_t pipeline_end_cycles = 0;

    uint64_t pipeline_start_cycles = rte_get_timer_cycles();

    run_reconstruction_pipeline( decoded_i420, &ctx.enc, pose.yaw, pose.pitch, pose.zoom, reconstructed_points.data(), &arrived_points, &eroded_points, &valid_points, gpu_metrics, &pose_apply_end_cycles, &pipeline_end_cycles, process_node_reception );

    if ( pipeline_end_cycles < pipeline_start_cycles )
        pipeline_end_cycles = rte_get_timer_cycles();

    t -> reconstruction_pipeline_ms = ( ( double )( pipeline_end_cycles - pipeline_start_cycles ) / timer_hz ) * 1000.0;
    t -> gpu_transfer_ms = gpu_metrics[ 0 ];
    t -> erosion_ms = gpu_metrics[ 1 ];
    t -> reconstruction_ms = gpu_metrics[ 2 ];
    t -> pose_ms = gpu_metrics[ 3 ];
    t -> gpu_copyback_ms = gpu_metrics[ 4 ];

    double measured_gpu_ms = gpu_metrics[ 0 ] + gpu_metrics[ 1 ] + gpu_metrics[ 2 ] + gpu_metrics[ 3 ] + gpu_metrics[ 4 ];
    t -> host_overhead_ms = ( t -> reconstruction_pipeline_ms > measured_gpu_ms ) ? t -> reconstruction_pipeline_ms - measured_gpu_ms : 0.0;

    t -> arrived_points = arrived_points;
    t -> eroded_points = eroded_points;
    t -> valid_points = valid_points;

    if ( pose.measure_latency && valid_points > 0 && pose_apply_end_cycles >= pose.timestamp )
        t -> pose_control_ms = ( ( double )( pose_apply_end_cycles - pose.timestamp ) / timer_hz ) * 1000.0;

    if ( pose.measure_latency && valid_points > 0 && current_pose.generation == pose.generation )
        current_pose.pending = false;

    uint64_t first_tx_cycles = 0;
    uint64_t last_tx_cycles = 0;
    uint64_t active_tx_cycles = 0;

    bool tx_success = dispatch_reconstructed_frame( frame_id, ctx, pose, reconstructed_points.data(), valid_points, t, &first_tx_cycles, &last_tx_cycles, &active_tx_cycles );

    double tx_duration_sec = ( first_tx_cycles > 0 && last_tx_cycles >= first_tx_cycles ) ? ( double )( last_tx_cycles - first_tx_cycles ) / timer_hz : 0.0;

    t -> tx_duration_ms = tx_duration_sec * 1000.0;
    t -> active_tx_ms = ( ( double )active_tx_cycles / timer_hz ) * 1000.0;

    uint32_t expected_tx_packets = ( valid_points > 0 ) ? ( valid_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET : 1;
    t -> tx_complete = ( tx_success && t -> tx_points == valid_points && t -> tx_packets == expected_tx_packets && t -> mbuf_starvation == 0 ) ? 1 : 0;

    double reference_pipeline_ms = ( t -> reconstruction_pipeline_ms > t -> pose_ms ) ? t -> reconstruction_pipeline_ms - t -> pose_ms : 0.0;
    t -> reference_process_ms = t -> decode_service_ms + reference_pipeline_ms;
    t -> reference_residency_ms = ( pipeline_end_cycles >= ctx.first_arrival ) ? ( ( double )( pipeline_end_cycles - ctx.first_arrival ) / timer_hz ) * 1000.0 : 0.0;
    t -> reference_efficiency_pct = ( t -> reference_residency_ms > 0.0 ) ? ( t -> reference_process_ms / t -> reference_residency_ms ) * 100.0 : 0.0;

    uint64_t node_exit_cycles = ( last_tx_cycles > 0 ) ? last_tx_cycles : pipeline_end_cycles;
    t -> node_exit_timestamp = ( double )node_exit_cycles / timer_hz;

    double residency_sec = ( node_exit_cycles >= ctx.first_arrival ) ? ( double )( node_exit_cycles - ctx.first_arrival ) / timer_hz : 0.0;
    t -> total_residency_ms = residency_sec * 1000.0;

    if ( session_first_arrival > 0 && frame_id >= session_first_frame && node_exit_cycles >= session_first_arrival ) {
        double real_exit_sec = ( double )( node_exit_cycles - session_first_arrival ) / timer_hz;
        double ideal_exit_sec = ( double )( frame_id - session_first_frame ) / TARGET_FPS;

        t -> schedule_delay_ms = ( real_exit_sec - ideal_exit_sec ) * 1000.0;
    }
    else
        t -> schedule_delay_ms = 0.0;

    t -> active_process_ms =  t -> decode_service_ms + t -> reconstruction_pipeline_ms + t -> tx_duration_ms;
    t -> total_processing_ms = ( codec_write_start > 0 && node_exit_cycles >= codec_write_start ) ? ( ( double )( node_exit_cycles - codec_write_start ) / timer_hz ) * 1000.0 : 0.0;
    t -> node_efficiency_pct = ( t -> total_residency_ms > 0.0 ) ? ( t -> active_process_ms / t -> total_residency_ms ) * 100.0 : 0.0;

    uint64_t logical_output_bytes = ( uint64_t )t -> tx_points * sizeof( struct point_tx ) + ( t -> tx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );
    uint64_t network_output_bytes = ( uint64_t )t -> tx_points * sizeof( struct point_tx ) + ( ( uint64_t )t -> tx_packets * ( outer_len + sizeof( struct dec_hdr ) ) );
    
    uint16_t temporal_skip = t -> current_skip;
    
    if ( temporal_skip == 0 )
        temporal_skip = 1;

    double effective_fps = TARGET_FPS / temporal_skip;
    t -> logical_bitrate_mbps = ( logical_output_bytes * 8.0 * effective_fps ) / 1000000.0;
    t -> network_bitrate_mbps = ( network_output_bytes * 8.0 * effective_fps ) / 1000000.0;

    uint64_t camera_tx_cycles = rte_be_to_cpu_64( ctx.cam.timestamp );
    t -> e2e_latency_ms = ( node_exit_cycles >= camera_tx_cycles ) ? ( ( double )( node_exit_cycles - camera_tx_cycles ) / timer_hz ) * 1000.0 : 0.0;

    ctx.decoded = true;
    last_frame_id = std::max( last_frame_id, frame_id );

    if ( target_frame_id > 0 && frame_id == target_frame_id )
        dispatch_postroll_signal();
}

static inline void finalize_undecoded_frame( uint32_t frame_id, uint64_t timer_hz ) {
    
    // Purpose: It terminates instances discarded by error mechanisms maintaining continuity among subsequent analytical elements
    
    if ( frame_id == 0 || frame_id > K_FRAMES )
        return;

    struct decoder_frame_context &ctx = frame_contexts[ frame_id - 1 ];
    struct telemetry_csv *t = &telemetry_log[ frame_id - 1 ];

    if ( t -> frame_id != 0 )
        return;

    memset( t, 0, sizeof( *t ) );
    compile_input_metrics( frame_id, ctx, t, timer_hz );

    t -> yaw = current_pose.yaw;
    t -> pitch = current_pose.pitch;
    t -> zoom = current_pose.zoom;
    t -> node_exit_timestamp = ( double )rte_get_timer_cycles() / timer_hz;

    last_frame_id = std::max( last_frame_id, frame_id );
}

static inline void drain_ready_frames( uint64_t timer_hz ) {
    
    // Purpose: It extracts comprehensively converted subsets "offloading" structural computations inside the core functional loop
    
    while ( 1 ) {
        struct decoded_frame_job job;
        bool have_frame = false;

        pthread_mutex_lock( &decoded_mutex );

        if ( decoded_frame_count > 0 ) {
            job = decoded_frame_jobs[ decoded_frame_head ];
            decoded_frame_head = ( decoded_frame_head + 1 ) % I420_BUFFER_COUNT;
            decoded_frame_count--;
            have_frame = true;
        }

        pthread_mutex_unlock( &decoded_mutex );

        if ( !have_frame )
            break;

        evaluate_decoded_frame( job.frame_id, timer_hz, decoded_i420_buffers[ job.slot ].data(), job.service_start_cycles, job.ready_cycles );

        pthread_mutex_lock( &decoded_mutex );
        decoded_i420_slot_free[ job.slot ] = true;
        pthread_cond_signal( &decoded_slot_released );
        pthread_mutex_unlock( &decoded_mutex );
    }
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();

    ffmpeg_preroll();

    printf( "[SYSTEM] Reconstruction is about to begin at %.1f FPS...\n\n", TARGET_FPS );

    FILE *ready = fopen( READY_PATH, "w" );

    if ( ready == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to to open file \"%s\"...\n", READY_PATH );

    fclose( ready );

    while ( 1 ) {
        if ( csv_written ) {
            rte_delay_us_sleep( 1000 );
            continue;
        }

        process_node_reception();
        drain_ready_frames( timer_hz );

        if ( eos_received && !codec_eos_requested.load( std::memory_order_acquire ) )
            request_codec_eos();

        if ( codec_failed.load( std::memory_order_acquire ) ) {
            printf( "[SYSTEM] Error: Persistent input course failed...\n" );
            request_codec_eos();
        }

        if ( codec_input_closed.load( std::memory_order_acquire ) && ffmpeg_output_eof.load( std::memory_order_acquire ) && frame_queue_depth() == 0 ) {
            uint32_t undecoded_frame_id = 0;

            while ( dequeue_frame_id( &undecoded_frame_id ) )
                finalize_undecoded_frame( undecoded_frame_id, timer_hz );

            if ( !decoder_eos_sent ) {
                decoder_eos_sent = dispatch_node_eos();

            }

            if ( pose_notification_printed )
                printf( "\n" );

            telemetry_to_csv();
            csv_written = true;

            printf( "\n[SYSTEM] End of stream detected. Changing to \"idle\" state...\n" );
        }

        rte_pause();
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It enacts standard microservice boot patterns registering Decoder allocations prior to mapping threads to isolated processors
    
    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"Decoder\" microservice...\n\n" );

    unlink( POSTROLL_PATH );

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF2, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF2-facing virtual port configuration failed...\n" );

    for ( uint8_t slot = 0; slot < I420_BUFFER_COUNT; slot++ )
        decoded_i420_buffers[ slot ].resize( TOTAL_YUV_SIZE );

    reconstructed_points.resize( MAX_RECONSTRUCTED_POINTS );

    cuda_memory_init();
    cuda_memory_warmup();

    for ( uint8_t slot = 0; slot < I420_BUFFER_COUNT; slot++ )
        cuda_memory_register( decoded_i420_buffers[ slot ].data(), TOTAL_YUV_SIZE );

    cuda_memory_register( reconstructed_points.data(), reconstructed_points.size() * sizeof( struct host_point ) );

    for ( uint32_t i = 0; i < K_FRAMES; i++ ) {
        codec_start_cycles[ i ].store( 0, std::memory_order_relaxed );
        frame_ffmpeg_calls[ i ].store( 0, std::memory_order_relaxed );
        frame_ffmpeg_failures[ i ].store( 0, std::memory_order_relaxed );
        codec_drops[ i ].store( 0, std::memory_order_relaxed );
    }

    ffmpeg_init();
    ffmpeg_writer_start();

    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE )
        worker_loop( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    ffmpeg_writer_stop();

    if ( ffmpeg_out[ 0 ] >= 0 )
        close( ffmpeg_out[ 0 ] );

    if ( ffmpeg_pid > 0 )
        waitpid( ffmpeg_pid, NULL, 0 );

    cuda_memory_unleash( reconstructed_points.data() );

    for ( uint8_t slot = 0; slot < I420_BUFFER_COUNT; slot++ )
        cuda_memory_unleash( decoded_i420_buffers[ slot ].data() );

    cuda_memory_free();

    rte_eal_cleanup();

    return 0;
}