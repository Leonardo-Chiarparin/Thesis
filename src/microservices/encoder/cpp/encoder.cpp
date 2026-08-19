#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "encoder.h"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <memory>
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

// Global application state
static struct rte_mempool *mbuf_pool;

static struct rte_ether_addr encoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x02 } };
static struct rte_ether_addr sff2_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x01 } };

static int ffmpeg_in[ 2 ];
static int ffmpeg_out[ 2 ];
static pid_t ffmpeg_pid;

static struct telemetry_csv telemetry_log[ K_FRAMES ];
static bool csv_written = false;
static bool eos_received = false;

static struct enc_hdr encoder_metadata[ K_FRAMES ];
static struct cam_hdr camera_metadata[ K_FRAMES ];
static uint32_t current_out_frame_id = 0;
static uint32_t last_processed_frame_id = 0;
static uint32_t current_mpeg_packet_id = 0;

static std::vector< uint8_t > ts_pending;
static std::vector< uint8_t > mpeg_chunk;

static const size_t outer_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

struct frame_buffer {
    std::unique_ptr< struct host_point[] > points;
    std::vector< uint8_t > packet_received;

    uint32_t original_points = 0;
    uint32_t expected_packets = 0;
    uint32_t received_points = 0;
    uint32_t rx_packets = 0;
    uint32_t payload_bytes = 0;

    uint64_t conversion_cycles = 0;

    uint64_t camera_tx = 0;
    uint64_t first_arrival = 0;
    uint64_t last_arrival = 0;
    uint64_t frame_ready = 0;

    struct cam_hdr cam = { 0 };
    struct geo_agg_hdr geo = { 0 };
};

struct geometry_result {
    float centroid_x = 0.0f;
    float centroid_y = 0.0f;
    float centroid_z = 0.0f;

    float extent_x = 0.0f;
    float extent_y = 0.0f;
    float extent_z = 0.0f;

    float bbox_center_x = 0.0f;
    float bbox_center_y = 0.0f;
    float bbox_center_z = 0.0f;

    float max_r = 0.0f;

    double geometry_aggregation_ms = 0.0;
    double max_r_ms = 0.0;
};

struct workload_controller_state {
    double ewma_ms = 0.0;
    uint16_t requested_skip = 1;
    uint32_t observations = 0;
    uint32_t overload_streak = 0;
    uint32_t recovery_streak = 0;
    uint32_t warmup_stable_streak = 0;
    uint32_t last_control_observation = 0;
    bool armed = false;
};

struct yuv_job {
    uint32_t frame_id = 0;
    uint32_t frame_offset = 0;
    uint8_t slot = 0;

    uint64_t first_arrival = 0;
    uint64_t projection_end_cycles = 0;
    uint64_t session_start_cycles = 0;

    double slot_wait_ms = 0.0;
    double global_clock_offset_sec = 0.0;

    bool preroll = false;
};

static std::map< uint32_t, frame_buffer > frame_buffers;
static std::queue< uint32_t > mpeg_frame_queue;
static workload_controller_state workload_controller;
static bool controller_notification_printed = false;

static uint32_t ffmpeg_preroll_outputs = 0;
static uint16_t mpeg_video_pid = 0xFFFF;

static std::vector< uint8_t > yuv_buffers[ YUV_BUFFER_COUNT ];
static bool yuv_slot_free[ YUV_BUFFER_COUNT ] = { true, true, true };
static struct yuv_job writer_jobs[ YUV_BUFFER_COUNT ];

static uint32_t writer_head = 0;
static uint32_t writer_tail = 0;
static uint32_t writer_count = 0;

static bool writer_active = false;
static bool writer_stop_requested = false;
static bool writer_started = false;

static pthread_t writer_thread;
static pthread_mutex_t writer_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t writer_not_empty = PTHREAD_COND_INITIALIZER;
static pthread_cond_t writer_slot_released = PTHREAD_COND_INITIALIZER;

static std::atomic< uint64_t > frame_encode_start_cycles[ K_FRAMES ];
static uint64_t frame_first_arrival_cycles[ K_FRAMES ] = { 0 };
static uint64_t frame_last_egress_cycles[ K_FRAMES ] = { 0 };

// Data path & support routines
static inline void poll_network_rx();
static inline void drain_ffmpeg( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz );

static inline uint32_t float_to_be_32( float value ) {
    uint32_t bits;

    memcpy( &bits, &value, sizeof( bits ) );

    return rte_cpu_to_be_32( bits );
}

static inline float be_32_to_float( uint32_t value ) {
    uint32_t bits = rte_be_to_cpu_32( value );
    float result;

    memcpy( &result, &bits, sizeof( result ) );

    return result;
}

static inline int port_init( uint16_t port, struct rte_mempool *mbuf_pool ) {
    struct rte_eth_conf port_conf = { 0 };
    int retval;

    if ( ! rte_eth_dev_is_valid_port( port ) ) 
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

static inline void header_init( struct rte_ether_hdr *eth, struct rte_ipv4_hdr *ipv4, struct rte_udp_hdr *udp, uint16_t udp_payload_len ) {
    memset( eth, 0, outer_len );

    rte_ether_addr_copy( &encoder_mac, &eth -> src_addr );
    rte_ether_addr_copy( &sff2_mac, &eth -> dst_addr );

    eth -> ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    ipv4 -> version_ihl = 0x45;
    ipv4 -> time_to_live = 64;
    ipv4 -> next_proto_id = IPPROTO_UDP;
    ipv4 -> src_addr = rte_cpu_to_be_32( ENCODER_SFF2_IP );
    ipv4 -> dst_addr = rte_cpu_to_be_32( SFF2_ENCODER_IP );
    ipv4 -> total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + udp_payload_len );

    udp -> src_port = rte_cpu_to_be_16( ENCODER_SFF2_PORT );
    udp -> dst_port = rte_cpu_to_be_16( SFF2_ENCODER_PORT );
    udp -> dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + udp_payload_len );
    udp -> dgram_cksum = 0;

    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );
}

static inline bool is_preroll_frame( uint32_t frame_id ) {
    
    // Purpose: It determines whether the current frame belongs to the designated pre-roll initialization sequence
    
    return frame_id == FRAME_ID;
}

static inline uint16_t mpeg_ts_pid( const uint8_t *ts ) {
    
    // Purpose: It extracts the 13-bit "Packet Identifier" ( "PID" ) from an "MPEG-TS" packet header
    
    return ( ( uint16_t )( ts[ 1 ] & 0x1F ) << 8 ) | ts[ 2 ];
}

static inline bool ts_starts_video_pes( const uint8_t *ts ) {

    // Purpose: It identifies a video "Packetized Elementary Stream" ( "PES" ) boundary by combining the "MPEG-TS" payload-unit-start indicator with the elementary-sequence start code.
    //          Such parser is employeed only for source-frame association. Complete transport packets remain byte-preserved & the encoded "H.265" components are never rewritten

    if ( ts[ 0 ] != 0x47 )
        return false;

    if ( ( ts[ 1 ] & 0x40 ) == 0 )
        return false;

    uint8_t adaptation_control = ( ts[ 3 ] >> 4 ) & 0x03;

    if ( adaptation_control == 0 || adaptation_control == 2 )
        return false;

    size_t payload_offset = 4;

    if ( adaptation_control == 3 ) {
        uint8_t adaptation_length = ts[ 4 ];
        payload_offset += 1 + adaptation_length;
    }

    if ( payload_offset + 4 > TS_PACKET_SIZE )
        return false;

    return ts[ payload_offset ] == 0x00 && ts[ payload_offset + 1 ] == 0x00 && ts[ payload_offset + 2 ] == 0x01 && ( ts[ payload_offset + 3 ] & 0xF0 ) == 0xE0;
}

static void ffmpeg_init() {

    // Purpose: It creates operating-system inter-process communication "pipes" & spawns the persistent hardware-accelerated "H.265" subprocess before timed frame processing.
    //          The input element remains blocking only for the dedicated writer thread, while "FFmpeg" applies the configured queue argument & low-delay "NVENC" / "muxer" settings.
    //          This function isolates "codec"-side management from the "DPDK" "worker" while keeping the session startup outside the measured application stream
    
    if ( pipe( ffmpeg_in ) < 0 || pipe( ffmpeg_out ) < 0 ) {
        perror( "[SYSTEM] Error: Failed to create \"FFmpeg\" pipes...\n" );
        exit( 1 );
    }

    int output_flags = fcntl( ffmpeg_out[ 0 ], F_GETFL, 0 );

    if ( output_flags >= 0 )
        fcntl( ffmpeg_out[ 0 ], F_SETFL, output_flags | O_NONBLOCK );

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

        char res[ 32 ];
        snprintf( res, sizeof( res ), "%dx%d", ENCODER_W, ENCODER_H );

        execlp( "ffmpeg", "ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo", "-s", res, "-pix_fmt", "yuv420p", "-r", "30", "-thread_queue_size", QUEUE_SIZE, "-i", "-", "-c:v", "hevc_nvenc", "-preset", "p2", "-tune", "ull", "-rc", "cbr", "-b:v", TARGET_BITRATE_MBPS, "-maxrate", TARGET_BITRATE_MBPS, "-bufsize", TARGET_BUFFER_SIZE, "-rc-lookahead", LOOKAHEAD, "-delay", DELAY, "-zerolatency", ZERO_LATENCY, "-bf", "0", "-g", "15", "-forced-idr", "1", "-flush_packets", FLUSH_PACKETS, "-muxdelay", "0", "-muxpreload", "0", "-vstats_file", FFMPEG_PATH, "-f", "mpegts", "-", NULL );
        exit( 1 );
    }

    close( ffmpeg_in[ 0 ] );
    close( ffmpeg_out[ 1 ] );
}

static inline uint32_t writer_pending_frames() {

    // Purpose: It calculates the number of pending frames for "FFmpeg" elaboration

    pthread_mutex_lock( &writer_mutex );

    uint32_t pending = writer_count + ( writer_active ? 1 : 0 );

    pthread_mutex_unlock( &writer_mutex );

    return pending;
}

static inline int acquire_yuv_slot( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz, double *slot_wait_ms ) {

    // Purpose: It acquires a preallocated "I420" memory slot, maintaining network service processing whenever all three slots are occupied.
    //          During contention, raw input & currently available encoded output continue to be drained prior to the wait condition is re-evaluated

    uint64_t wait_start = rte_get_timer_cycles();

    while ( 1 ) {
        pthread_mutex_lock( &writer_mutex );

        for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
            if ( yuv_slot_free[ slot ] ) {
                yuv_slot_free[ slot ] = false;
                pthread_mutex_unlock( &writer_mutex );

                uint64_t wait_end = rte_get_timer_cycles();

                if ( slot_wait_ms )
                    *slot_wait_ms = ( ( double )( wait_end - wait_start ) / timer_hz ) * 1000.0;

                return slot;
            }
        }

        pthread_mutex_unlock( &writer_mutex );

        poll_network_rx();
        drain_ffmpeg( tx_bufs, burst_idx, timer_hz );
        rte_pause();
    }
}

static inline bool enqueue_yuv_job( const struct yuv_job &job ) {

    // Purpose: It registers a new projection job into the "FFmpeg" writer queue & signals the serialization thread

    pthread_mutex_lock( &writer_mutex );

    if ( writer_stop_requested || writer_count >= YUV_BUFFER_COUNT ) {
        pthread_mutex_unlock( &writer_mutex );
        return false;
    }

    writer_jobs[ writer_tail ] = job;
    writer_tail = ( writer_tail + 1 ) % YUV_BUFFER_COUNT;
    writer_count++;

    mpeg_frame_queue.push( job.frame_id );

    pthread_cond_signal( &writer_not_empty );
    pthread_mutex_unlock( &writer_mutex );

    return true;
}

static void *ffmpeg_writer_loop( void *arg ) {
    ( void )arg;

    // Purpose: It publishes resulting "I420" frames into "FFmpeg" via a dedicated blocking pipe, mitigating its backpressure on the primary "DPDK" "worker" 

    cpu_set_t cpuset;

    CPU_ZERO( &cpuset );
    CPU_SET( FFMPEG_CPU, &cpuset );
    pthread_setaffinity_np( pthread_self(), sizeof( cpu_set_t ), &cpuset );
    pthread_setname_np( pthread_self(), "encoder_ffmpeg_writer" );

    uint64_t timer_hz = rte_get_timer_hz();

    while ( 1 ) {
        struct yuv_job job;

        pthread_mutex_lock( &writer_mutex );

        while ( writer_count == 0 && !writer_stop_requested )
            pthread_cond_wait( &writer_not_empty, &writer_mutex );

        if ( writer_count == 0 && writer_stop_requested ) {
            pthread_mutex_unlock( &writer_mutex );
            break;
        }

        job = writer_jobs[ writer_head ];
        writer_head = ( writer_head + 1 ) % YUV_BUFFER_COUNT;
        writer_count--;
        writer_active = true;

        pthread_mutex_unlock( &writer_mutex );

        bool preroll = job.preroll;
        uint32_t idx = 0;
        struct telemetry_csv *t = NULL;
        uint64_t write_start = rte_get_timer_cycles();

        if ( !preroll ) {
            idx = ( job.frame_id - 1 ) % K_FRAMES;
            t = &telemetry_log[ idx ];
            frame_encode_start_cycles[ idx ].store( write_start, std::memory_order_release );
        }

        size_t written_total = 0;

        while ( written_total < TOTAL_YUV_SIZE ) {
            if ( !preroll )
                t -> ffmpeg_write_calls++;

            ssize_t written = write( ffmpeg_in[ 1 ], yuv_buffers[ job.slot ].data() + written_total, TOTAL_YUV_SIZE - written_total );

            if ( written > 0 ) {
                written_total += ( size_t )written;
                continue;
            }

            if ( written < 0 && errno == EINTR )
                continue;

            if ( written < 0 && ( errno == EAGAIN || errno == EWOULDBLOCK ) ) {
                if ( !preroll )
                    t -> ffmpeg_write_eagain++;

                sched_yield();
                continue;
            }

            if ( preroll )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"FFmpeg\" / \"NVENC\" pre-roll write failed...\n" );

            t -> status = 0;
            snprintf( t -> event, sizeof( t -> event ), "INVALID" );
            break;
        }

        uint64_t write_end = rte_get_timer_cycles();

        if ( !preroll ) {
            t -> tx_duration_ms = ( ( double )( write_end - write_start ) / timer_hz ) * 1000.0;
            t -> wait_render_queue_ms = ( write_start >= job.projection_end_cycles ) ? ( ( double )( write_start - job.projection_end_cycles ) / timer_hz ) * 1000.0 : 0.0;

            t -> total_processing_ms = t -> conversion_ms + t -> geometry_aggregation_ms + t -> max_r_ms + t -> projection_ms + t -> tx_duration_ms;
            t -> active_process_ms = t -> total_processing_ms;
        }

        pthread_mutex_lock( &writer_mutex );

        yuv_slot_free[ job.slot ] = true;
        writer_active = false;

        pthread_cond_broadcast( &writer_slot_released );
        pthread_mutex_unlock( &writer_mutex );
    }

    return NULL;
}

static void ffmpeg_writer_start() {
    if ( writer_started )
        return;

    writer_stop_requested = false;

    int retval = pthread_create( &writer_thread, NULL, ffmpeg_writer_loop, NULL );

    if ( retval != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"FFmpeg\" thread creation failed...\n" );

    writer_started = true;
}

static void ffmpeg_preroll() {

    // Purpose: It activates the persistent "FFmpeg" / "NVENC" session with an initial private blank "I420" "Group of Pictures" ( "GOP" ) before application telemetry commences.
    //          Pre-roll frames reuse the normal blocking writer & parser but never allocate network packets or populate per-frame metrics

    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    int burst_idx = 0;
    uint64_t timer_hz = rte_get_timer_hz();

    ffmpeg_preroll_outputs = 0;

    printf( "[SYSTEM] Pre-rolling \"FFmpeg\" & \"NVENC\" components with %d frames...\n", FRAMES );

    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
        memset( yuv_buffers[ slot ].data(), 16, SIZE_Y );
        memset( yuv_buffers[ slot ].data() + SIZE_Y, 128, 2 * SIZE_UV );
    }

    for ( uint32_t frame = 0; frame < FRAMES; frame++ ) {
        int yuv_slot = -1;

        while ( yuv_slot < 0 ) {
            pthread_mutex_lock( &writer_mutex );

            for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
                if ( yuv_slot_free[ slot ] ) {
                    yuv_slot_free[ slot ] = false;
                    yuv_slot = slot;
                    break;
                }
            }

            pthread_mutex_unlock( &writer_mutex );

            if ( yuv_slot < 0 ) {
                drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );
                rte_pause();
            }
        }

        struct yuv_job job;

        job.frame_id = FRAME_ID;
        job.slot = ( uint8_t )yuv_slot;
        job.preroll = true;

        if ( unlikely( !enqueue_yuv_job( job ) ) ) {
            pthread_mutex_lock( &writer_mutex );
            yuv_slot_free[ yuv_slot ] = true;
            pthread_cond_broadcast( &writer_slot_released );
            pthread_mutex_unlock( &writer_mutex );

            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to enqueue \"FFmpeg\" / \"NVENC\" pre-roll frame...\n" );
        }
    }

    while ( writer_pending_frames() > 0 || ffmpeg_preroll_outputs < FRAMES ) {
        drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );
        rte_pause();
    }

    drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );
}

static void ffmpeg_writer_stop() {

    // It terminates the dedicated "FFmpeg" writer thread, ensuring completion of all remaining tasks

    if ( !writer_started )
        return;

    pthread_mutex_lock( &writer_mutex );
    writer_stop_requested = true;
    pthread_cond_broadcast( &writer_not_empty );
    pthread_mutex_unlock( &writer_mutex );

    pthread_join( writer_thread, NULL );
    writer_started = false;
}

static inline void wait_for_idle( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {
    
    // It halts execution until all pending writer elements are processed, actively draining the network & "FFmpeg" outputs
    
    while ( writer_pending_frames() > 0 ) {
        poll_network_rx();
        drain_ffmpeg( tx_bufs, burst_idx, timer_hz );
        rte_pause();
    }
}

static void telemetry_to_csv() {
    struct stat st = { 0 };
    uint64_t timer_hz = rte_get_timer_hz();

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

    fprintf( f, "frame_id;status;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;clock_offset_ms;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;conversion_ms;geometry_aggregation_ms;max_r_ms;projection_ms;tx_duration_ms;active_process_ms;total_processing_ms;total_residency_ms;node_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_to_node_latency_ms;end_to_end_latency_ms;schedule_delay_ms;network_jitter_ms;wait_raw_queue_ms;wait_render_queue_ms;workload_ewma_ms;workload_ratio;frame_backlog;codec_backlog;encode_h265_ms;mpeg_bytes_generated;ffmpeg_write_calls;ffmpeg_write_eagain;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation\n" );

    uint32_t limit = ( last_processed_frame_id < K_FRAMES ) ? last_processed_frame_id : K_FRAMES;

    for ( uint32_t i = 0; i < limit; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        uint64_t first_arrival_cycles = frame_first_arrival_cycles[ i ];
        uint64_t last_egress_cycles = frame_last_egress_cycles[ i ];

        if ( first_arrival_cycles > 0 && last_egress_cycles >= first_arrival_cycles ) {
            double residency_sec = ( double )( last_egress_cycles - first_arrival_cycles ) / timer_hz;

            t -> total_residency_ms = residency_sec * 1000.0;
            t -> node_exit_timestamp = ( double )last_egress_cycles / timer_hz;
            t -> node_efficiency_pct = ( t -> total_residency_ms > 0.0 ) ? ( t -> active_process_ms / t -> total_residency_ms ) * 100.0 : 0.0;

            if ( t -> camera_send_timestamp > 0.0 ) {
                double clock_offset_sec = t -> clock_offset_ms / 1000.0;

                t -> end_to_end_latency_ms = ( t -> node_exit_timestamp - t -> camera_send_timestamp - clock_offset_sec ) * 1000.0;
            }
        }

        uint16_t temporal_skip = t -> current_skip;

        if ( temporal_skip == 0 )
            temporal_skip = 1;

        double effective_fps = TARGET_FPS / temporal_skip;
        uint64_t logical_frame_bytes = ( uint64_t )t -> mpeg_bytes_generated + ( t -> tx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );
        uint64_t network_frame_bytes = ( uint64_t )t -> mpeg_bytes_generated + ( ( uint64_t )t -> tx_packets * ( sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) );

        t -> logical_bitrate_mbps = ( logical_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
        t -> network_bitrate_mbps = ( network_frame_bytes * 8.0 * effective_fps ) / 1000000.0;

        fprintf( f, "%u;%u;%u;%s;%.3f;%.3f;%.3f;", t -> frame_id, t -> status, t -> current_skip, t -> event, t -> yaw, t -> pitch, t -> zoom );
        fprintf( f, "%.6f;%.6f;%.6f;%.3f;", t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> clock_offset_ms );
        fprintf( f, "%u;%u;%u;%u;%u;%u;%.2f;%.3f;%.3f;%.3f;", t -> original_points, t -> rx_points, t -> tx_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps );
        fprintf( f, "%.3f;%.3f;%.3f;%.6f;%.3f;%.3f;%.3f;%.3f;%.3f;", t -> conversion_ms, t -> geometry_aggregation_ms, t -> max_r_ms, t -> projection_ms, t -> tx_duration_ms, t -> active_process_ms, t -> total_processing_ms, t -> total_residency_ms, t -> node_efficiency_pct );
        fprintf( f, "%.3f;%.3f;%.3f;%.3f;%.3f;", t -> gpu_transfer_ms, t -> gpu_kernel_ms, t -> gpu_packing_ms, t -> gpu_copyback_ms, t -> host_overhead_ms );
        fprintf( f, "%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;", t -> camera_to_node_latency_ms, t -> end_to_end_latency_ms, t -> schedule_delay_ms, t -> network_jitter_ms, t -> wait_raw_queue_ms, t -> wait_render_queue_ms );
        fprintf( f, "%.3f;%.3f;%u;%u;", t -> workload_ewma_ms, t -> workload_ratio, t -> frame_backlog, t -> codec_backlog );
        fprintf( f, "%.3f;%u;%u;%u;", t -> encode_h265_ms, t -> mpeg_bytes_generated, t -> ffmpeg_write_calls, t -> ffmpeg_write_eagain );
        fprintf( f, "%u;%u;%u;%u;%u\n", t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets, t -> mbuf_starvation );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static inline void flush_tx_burst( struct rte_mbuf **tx_bufs, int *burst_idx, uint32_t *tx_packets, uint32_t *tx_zero_accepts, uint32_t *tx_partial_accepts, uint32_t *tx_resubmit_calls, uint32_t *tx_resubmitted_packets, uint64_t *last_egress_cycles = NULL ) {
    if ( *burst_idx == 0 )
        return;

    uint16_t sent = 0;
    uint16_t retries = 0;

    bool is_resubmission = false;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint16_t requested_packets = *burst_idx - sent;

        if ( is_resubmission ) {
            if ( tx_resubmit_calls )
                ( *tx_resubmit_calls )++;

            if ( tx_resubmitted_packets )
                *tx_resubmitted_packets += requested_packets;
        }

        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF2, 0, &tx_bufs[ sent ], requested_packets );

        uint64_t tx_end_cycles = rte_get_timer_cycles();

        if ( nb_tx > 0 ) {
            if ( last_egress_cycles != NULL )
                *last_egress_cycles = tx_end_cycles;

            if ( tx_packets )
                *tx_packets += nb_tx;

            if ( nb_tx < requested_packets && tx_partial_accepts )
                ( *tx_partial_accepts )++;

            sent += nb_tx;
            retries = 0;
            is_resubmission = nb_tx < requested_packets;
        }
        else {
            if ( tx_zero_accepts )
                ( *tx_zero_accepts )++;

            is_resubmission = true;

            if ( ++retries > MAX_ZERO_ACCEPTS ) {
                for ( int k = sent; k < *burst_idx; k++ )
                    rte_pktmbuf_free( tx_bufs[ k ] );

                break;
            }

            uint16_t pause_count = ( retries < pause_window ) ? retries : pause_window;

            for ( uint16_t p = 0; p < pause_count; p++ )
                rte_pause();
        }
    }

    *burst_idx = 0;
}

static inline bool send_temporal_control( uint32_t source_frame_id, uint16_t requested_skip ) {

    // Purpose: It originates the plain 8-byte temporal control message consumed by the SFF2 classifier, deliberately omitting "NSH" encapsulation

    if ( requested_skip == 0 )
        requested_skip = 1;

    struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );

    if ( m == NULL )
        return false;

    const size_t packet_len = outer_len + sizeof( struct temporal_payload );
    uint8_t *data = ( uint8_t * )rte_pktmbuf_append( m, packet_len );

    if ( data == NULL ) {
        rte_pktmbuf_free( m );
        return false;
    }

    memset( data, 0, packet_len );

    struct rte_ether_hdr *eth = ( struct rte_ether_hdr * )data;
    struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );
    struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );
    struct temporal_payload *temporal = ( struct temporal_payload * )( udp + 1 );

    rte_ether_addr_copy( &encoder_mac, &eth -> src_addr );
    rte_ether_addr_copy( &sff2_mac, &eth -> dst_addr );
    eth -> ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    uint16_t udp_length = sizeof( struct rte_udp_hdr ) + sizeof( struct temporal_payload );

    ipv4 -> version_ihl = 0x45;
    ipv4 -> time_to_live = 64;
    ipv4 -> next_proto_id = IPPROTO_UDP;
    ipv4 -> src_addr = rte_cpu_to_be_32( ENCODER_SFF2_IP );
    ipv4 -> dst_addr = rte_cpu_to_be_32( SFF2_ENCODER_IP );
    ipv4 -> total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + udp_length );

    udp -> src_port = rte_cpu_to_be_16( ENCODER_SFF2_PORT );
    udp -> dst_port = rte_cpu_to_be_16( SFF2_ENCODER_PORT );
    udp -> dgram_len = rte_cpu_to_be_16( udp_length );
    udp -> dgram_cksum = 0;

    temporal -> frame_id = rte_cpu_to_be_32( source_frame_id );
    temporal -> skip = rte_cpu_to_be_16( requested_skip );
    temporal -> padding = 0;

    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );

    struct rte_mbuf *control_bufs[ 1 ] = { m };
    int control_burst_idx = 1;

    flush_tx_burst( control_bufs, &control_burst_idx, NULL, NULL, NULL, NULL, NULL );

    return true;
}

static inline bool geometry_from_sff1( const frame_buffer &fb, const struct host_point *active_points, uint32_t active_point_count, geometry_result *result, uint64_t timer_hz ) {

    // Purpose: It utilizes the latest frame-local geometry snapshot provieded by SFF1 exclusively when the active point count matches the assembled set.
    //          Whole frames exploit centroid, extents, bounding-box centre & "max_r". An "EOS"-finalized partial component may apply the progressive mathematics but recomputes the exact radius over its active collection

    uint32_t metadata_active_points = rte_be_to_cpu_32( fb.geo.active_point_count );

    if ( active_points == NULL || active_point_count == 0 || metadata_active_points == 0 || metadata_active_points != active_point_count )
        return false;

    result -> centroid_x = be_32_to_float( fb.geo.centroid_x );
    result -> centroid_y = be_32_to_float( fb.geo.centroid_y );
    result -> centroid_z = be_32_to_float( fb.geo.centroid_z );

    result -> extent_x = be_32_to_float( fb.geo.extent_x );
    result -> extent_y = be_32_to_float( fb.geo.extent_y );
    result -> extent_z = be_32_to_float( fb.geo.extent_z );

    result -> bbox_center_x = be_32_to_float( fb.geo.bbox_center_x );
    result -> bbox_center_y = be_32_to_float( fb.geo.bbox_center_y );
    result -> bbox_center_z = be_32_to_float( fb.geo.bbox_center_z );

    bool valid_geometry = std::isfinite( result -> centroid_x ) && std::isfinite( result -> centroid_y ) && std::isfinite( result -> centroid_z ) && std::isfinite( result -> extent_x ) && std::isfinite( result -> extent_y ) && std::isfinite( result -> extent_z ) && std::isfinite( result -> bbox_center_x ) && std::isfinite( result -> bbox_center_y ) && std::isfinite( result -> bbox_center_z ) && result -> extent_x >= 0.0f && result -> extent_y >= 0.0f && result -> extent_z >= 0.0f;

    if ( !valid_geometry )
        return false;

    result -> geometry_aggregation_ms = 0.0;

    bool frame_complete = fb.received_points == fb.original_points;

    if ( frame_complete ) {
        result -> max_r = be_32_to_float( fb.geo.max_r );
        result -> max_r_ms = 0.0;

        return std::isfinite( result -> max_r ) && result -> max_r >= 0.0f;
    }

    uint64_t t_max_r_start = rte_get_timer_cycles();
    float max_r2 = 0.0f;

    for ( uint32_t i = 0; i < active_point_count; i++ ) {
        if ( i > 0 && i % 4096 == 0 )
            poll_network_rx();

        float dx = active_points[ i ].x - result -> centroid_x;
        float dy = active_points[ i ].y - result -> centroid_y;
        float dz = active_points[ i ].z - result -> centroid_z;
        float r2 = dx * dx + dy * dy + dz * dz;

        if ( r2 > max_r2 )
            max_r2 = r2;
    }

    result -> max_r = std::sqrt( max_r2 );

    uint64_t t_max_r_end = rte_get_timer_cycles();

    result -> max_r_ms = ( ( double )( t_max_r_end - t_max_r_start ) / timer_hz ) * 1000.0;

    return std::isfinite( result -> max_r );
}

static inline bool geometry_recompute_local( const struct host_point *active_points, uint32_t active_point_count, geometry_result *result, uint64_t timer_hz ) {
    if ( active_points == NULL || active_point_count == 0 )
        return false;

    uint64_t t_geometry_start = rte_get_timer_cycles();

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float min_z = FLT_MAX;
    float max_x = -FLT_MAX;
    float max_y = -FLT_MAX;
    float max_z = -FLT_MAX;

    for ( uint32_t i = 0; i < active_point_count; i++ ) {
        if ( i > 0 && i % 1024 == 0 )
            poll_network_rx();

        const struct host_point &point = active_points[ i ];

        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;

        min_x = std::min( min_x, point.x );
        min_y = std::min( min_y, point.y );
        min_z = std::min( min_z, point.z );
        max_x = std::max( max_x, point.x );
        max_y = std::max( max_y, point.y );
        max_z = std::max( max_z, point.z );
    }

    result -> centroid_x = ( float )( sum_x / active_point_count );
    result -> centroid_y = ( float )( sum_y / active_point_count );
    result -> centroid_z = ( float )( sum_z / active_point_count );

    result -> extent_x = max_x - min_x;
    result -> extent_y = max_y - min_y;
    result -> extent_z = max_z - min_z;

    result -> bbox_center_x = ( min_x + max_x ) * 0.5f;
    result -> bbox_center_y = ( min_y + max_y ) * 0.5f;
    result -> bbox_center_z = ( min_z + max_z ) * 0.5f;

    uint64_t t_geometry_end = rte_get_timer_cycles();

    result -> geometry_aggregation_ms = ( ( double )( t_geometry_end - t_geometry_start ) / timer_hz ) * 1000.0;

    uint64_t t_max_r_start = rte_get_timer_cycles();
    float max_r2 = 0.0f;

    for ( uint32_t i = 0; i < active_point_count; i++ ) {
        if ( i > 0 && i % 4096 == 0 )
            poll_network_rx();

        float dx = active_points[ i ].x - result -> centroid_x;
        float dy = active_points[ i ].y - result -> centroid_y;
        float dz = active_points[ i ].z - result -> centroid_z;
        float r2 = dx * dx + dy * dy + dz * dz;

        if ( r2 > max_r2 )
            max_r2 = r2;
    }

    result -> max_r = std::sqrt( max_r2 );

    uint64_t t_max_r_end = rte_get_timer_cycles();

    result -> max_r_ms = ( ( double )( t_max_r_end - t_max_r_start ) / timer_hz ) * 1000.0;

    return std::isfinite( result -> max_r );
}

static inline bool resolve_geometry( const frame_buffer &fb, const struct host_point *active_points, uint32_t active_point_count, geometry_result *result, uint64_t timer_hz ) {

    // Purpose: It resolves the optimal geometry source based on the global "OFFLOAD_MODE" configuration & metadata validity

    if ( OFFLOAD_MODE ) {
        if ( geometry_from_sff1( fb, active_points, active_point_count, result, timer_hz ) )
            return true;
    }

    return geometry_recompute_local( active_points, active_point_count, result, timer_hz );
}

static inline void update_workload_controller( uint32_t frame_id, uint16_t active_skip, double service_ms, double wait_raw_queue_ms, uint32_t frame_backlog, uint32_t codec_backlog, struct telemetry_csv *telemetry ) {

    // Purpose: It applies one-level temporal adaptation following the establishment of steady-state conditions in the raw data path
    //          Samples remain observable in telemetry but are discarded from the decision "EWMA" when the controller becomes armed

    if ( active_skip == 0 )
        active_skip = 1;

    workload_controller.observations++;

    if ( workload_controller.observations == 1 )
        workload_controller.ewma_ms = service_ms;
    else
        workload_controller.ewma_ms = EWMA_ALPHA * service_ms + ( 1.0 - EWMA_ALPHA ) * workload_controller.ewma_ms;

    double base_budget_ms = 1000.0 / TARGET_FPS;
    double active_budget_ms = base_budget_ms * active_skip;
    double workload_ratio = ( active_budget_ms > 0.0 ) ? workload_controller.ewma_ms / active_budget_ms : 0.0;

    telemetry -> workload_ewma_ms = workload_controller.ewma_ms;
    telemetry -> workload_ratio = workload_ratio;
    telemetry -> frame_backlog = frame_backlog;
    telemetry -> codec_backlog = codec_backlog;

    snprintf( telemetry -> event, sizeof( telemetry -> event ), "IDLE" );

    if ( !TEMPORAL_ADAPTATION ) {
        workload_controller.requested_skip = active_skip;
        workload_controller.overload_streak = 0;
        workload_controller.recovery_streak = 0;
        workload_controller.warmup_stable_streak = 0;
        workload_controller.armed = true;
        return;
    }

    if ( !workload_controller.armed ) {
        workload_controller.requested_skip = active_skip;
        workload_controller.overload_streak = 0;
        workload_controller.recovery_streak = 0;

        bool minimum_window_complete = workload_controller.observations >= MIN_FRAMES;
        bool startup_stable = frame_backlog == 0 && wait_raw_queue_ms <= active_budget_ms * RECOVERY_FRACTION;

        if ( minimum_window_complete && startup_stable )
            workload_controller.warmup_stable_streak++;
        else
            workload_controller.warmup_stable_streak = 0;

        bool stable_window_complete = workload_controller.warmup_stable_streak >= STABLE_STREAK;
        bool maximum_window_reached = workload_controller.observations >= MAX_FRAMES;

        snprintf( telemetry -> event, sizeof( telemetry -> event ), "WARMUP" );

        if ( !stable_window_complete && !maximum_window_reached )
            return;

        workload_controller.armed = true;
        workload_controller.warmup_stable_streak = 0;
        workload_controller.ewma_ms = service_ms;
        workload_controller.overload_streak = 0;
        workload_controller.recovery_streak = 0;

        workload_ratio = ( active_budget_ms > 0.0 ) ? workload_controller.ewma_ms / active_budget_ms : 0.0;
        telemetry -> workload_ewma_ms = workload_controller.ewma_ms;
        telemetry -> workload_ratio = workload_ratio;

        return;
    }

    if ( workload_controller.requested_skip != active_skip ) {
        workload_controller.overload_streak = 0;
        workload_controller.recovery_streak = 0;

        if ( workload_controller.observations >= workload_controller.last_control_observation + RETRY_FRAMES ) {
            if ( send_temporal_control( frame_id, workload_controller.requested_skip ) ) {
                workload_controller.last_control_observation = workload_controller.observations;
                snprintf( telemetry -> event, sizeof( telemetry -> event ), "RETRY" );
            }
        }

        return;
    }

    bool overloaded = workload_ratio >= OVERLOAD_RATIO || wait_raw_queue_ms > active_budget_ms * OVERLOAD_FRACTION || frame_backlog >= 2;

    double lower_budget_ms = ( active_skip > 1 ) ? base_budget_ms * ( active_skip - 1 ) : 0.0;
    double lower_ratio = ( lower_budget_ms > 0.0 ) ? workload_controller.ewma_ms / lower_budget_ms : 1.0;
    bool recoverable = active_skip > 1 && lower_ratio <= RECOVERY_RATIO && wait_raw_queue_ms <= active_budget_ms * RECOVERY_FRACTION && frame_backlog == 0;

    workload_controller.overload_streak = overloaded ? workload_controller.overload_streak + 1 : 0;
    workload_controller.recovery_streak = recoverable ? workload_controller.recovery_streak + 1 : 0;

    if ( workload_controller.overload_streak >= OVERLOAD_STREAK && active_skip < MAX_SKIP ) {
        uint16_t requested_skip = active_skip + 1;

        if ( requested_skip > MAX_SKIP )
            requested_skip = MAX_SKIP;

        if ( send_temporal_control( frame_id, requested_skip ) ) {
            workload_controller.requested_skip = requested_skip;
            workload_controller.last_control_observation = workload_controller.observations;
            workload_controller.overload_streak = 0;
            workload_controller.recovery_streak = 0;

            snprintf( telemetry -> event, sizeof( telemetry -> event ), "SKIP+1" );
            printf( "[SYSTEM] Workload controller increased skip at frame %u: %u -> %u ( \"Backlog\": %u, \"EWMA\": %.2f ms, Ratio: %.2f, Wait: %.2f ).\n", frame_id, active_skip, requested_skip, frame_backlog, workload_controller.ewma_ms, workload_ratio, wait_raw_queue_ms );
            controller_notification_printed = true;
        }

        return;
    }

    if ( workload_controller.recovery_streak >= RECOVERY_STREAK && active_skip > 1 ) {
        uint16_t requested_skip = active_skip - 1;

        if ( send_temporal_control( frame_id, requested_skip ) ) {
            workload_controller.requested_skip = requested_skip;
            workload_controller.last_control_observation = workload_controller.observations;
            workload_controller.overload_streak = 0;
            workload_controller.recovery_streak = 0;

            snprintf( telemetry -> event, sizeof( telemetry -> event ), "SKIP-1" );
            printf( "[SYSTEM] Workload controller decreased skip at frame %u: %u -> %u ( \"Backlog\": %u, \"EWMA\": %.2f ms, Ratio: %.2f, Wait: %.2f ).\n", frame_id, active_skip, requested_skip, frame_backlog, workload_controller.ewma_ms, lower_ratio, wait_raw_queue_ms );
            controller_notification_printed = true;
        }
    }
}

static inline void begin_mpeg_frame( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {

    // Purpose: It associates a newly detected video "PES" boundary with the oldest frame submitted to "FFmpeg", settling its encoding latency interval

    if ( current_out_frame_id > 0 && !is_preroll_frame( current_out_frame_id ) && *burst_idx > 0 ) {
        uint32_t old_idx = ( current_out_frame_id - 1 ) % K_FRAMES;

        flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ old_idx ].tx_packets, &telemetry_log[ old_idx ].tx_zero_accepts, &telemetry_log[ old_idx ].tx_partial_accepts, &telemetry_log[ old_idx ].tx_resubmit_calls, &telemetry_log[ old_idx ].tx_resubmitted_packets, &frame_last_egress_cycles[ old_idx ] );
    }

    if ( mpeg_frame_queue.empty() )
        return;

    current_out_frame_id = mpeg_frame_queue.front();
    mpeg_frame_queue.pop();
    current_mpeg_packet_id = 0;

    if ( is_preroll_frame( current_out_frame_id ) ) {
        ffmpeg_preroll_outputs++;
        return;
    }

    uint32_t idx = ( current_out_frame_id - 1 ) % K_FRAMES;
    uint64_t encode_start = frame_encode_start_cycles[ idx ].load( std::memory_order_acquire );

    if ( encode_start > 0 && telemetry_log[ idx ].encode_h265_ms == 0.0 )
        telemetry_log[ idx ].encode_h265_ms = ( ( double )( rte_get_timer_cycles() - encode_start ) / timer_hz ) * 1000.0;
}

static inline void emit_mpeg_payload( const uint8_t *mpeg_data, uint16_t mpeg_len, struct rte_mbuf **tx_bufs, int *burst_idx ) {

    // Purpose: It transmits a bounded "UDP" "MPEG-TS" media chunk to the SFF2 proxy, attaching reconstruction metadata without exposing service-chain state

    if ( current_out_frame_id == 0 || is_preroll_frame( current_out_frame_id ) || mpeg_len == 0 )
        return;

    uint32_t idx = ( current_out_frame_id - 1 ) % K_FRAMES;

    telemetry_log[ idx ].mpeg_bytes_generated += mpeg_len;

    struct rte_mbuf *m_out = rte_pktmbuf_alloc( mbuf_pool );

    if ( m_out == NULL ) {
        telemetry_log[ idx ].mbuf_starvation++;
        return;
    }

    size_t headers_len = outer_len + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr );
    char *data = ( char * )rte_pktmbuf_append( m_out, headers_len + mpeg_len );

    if ( data == NULL ) {
        rte_pktmbuf_free( m_out );
        telemetry_log[ idx ].mbuf_starvation++;
        return;
    }

    struct rte_ether_hdr *eth_out = ( struct rte_ether_hdr * )data;
    struct rte_ipv4_hdr *ipv4_out = ( struct rte_ipv4_hdr * )( eth_out + 1 );
    struct rte_udp_hdr *udp_out = ( struct rte_udp_hdr * )( ipv4_out + 1 );

    uint16_t udp_payload_len = sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) + mpeg_len;

    header_init( eth_out, ipv4_out, udp_out, udp_payload_len );

    uint8_t *payload_ptr = ( uint8_t * )( udp_out + 1 );

    struct cam_hdr out_cam = camera_metadata[ idx ];
    struct enc_hdr out_enc = encoder_metadata[ idx ];

    out_enc.packet_id = htonl( current_mpeg_packet_id++ );

    rte_memcpy( payload_ptr, &out_cam, sizeof( struct cam_hdr ) );
    payload_ptr += sizeof( struct cam_hdr );

    rte_memcpy( payload_ptr, &out_enc, sizeof( struct enc_hdr ) );
    payload_ptr += sizeof( struct enc_hdr );

    rte_memcpy( payload_ptr, mpeg_data, mpeg_len );

    tx_bufs[ ( *burst_idx )++ ] = m_out;

    if ( *burst_idx == BURST_SIZE )
        flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ idx ].tx_packets, &telemetry_log[ idx ].tx_zero_accepts, &telemetry_log[ idx ].tx_partial_accepts, &telemetry_log[ idx ].tx_resubmit_calls, &telemetry_log[ idx ].tx_resubmitted_packets, &frame_last_egress_cycles[ idx ] );
}

static inline void process_mpeg_bytes( const uint8_t *data, size_t data_len, struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {

    // Purpose: It reconstructs complete 188-byte "MPEG-TS" units from arbitrary "pipe" reads, detects video-"PES" frontiers & emits limited conveyance groups

    ts_pending.insert( ts_pending.end(), data, data + data_len );

    size_t consumed = 0;

    while ( ts_pending.size() - consumed >= TS_PACKET_SIZE ) {
        const uint8_t *ts = ts_pending.data() + consumed;

        if ( ts[ 0 ] != 0x47 ) {
            consumed++;
            continue;
        }

        if ( ts_starts_video_pes( ts ) ) {
            uint16_t video_pid = mpeg_ts_pid( ts );

            if ( mpeg_video_pid == 0xFFFF )
                mpeg_video_pid = video_pid;

            bool entering_real_stream = is_preroll_frame( current_out_frame_id ) && !mpeg_frame_queue.empty() && !is_preroll_frame( mpeg_frame_queue.front() );
            std::vector< uint8_t > transition_prefix;

            if ( current_out_frame_id > 0 && !mpeg_chunk.empty() ) {
                if ( entering_real_stream && mpeg_video_pid != 0xFFFF ) {
                    for ( size_t offset = 0; offset + TS_PACKET_SIZE <= mpeg_chunk.size(); offset += TS_PACKET_SIZE ) {
                        const uint8_t *pending_ts = mpeg_chunk.data() + offset;

                        if ( mpeg_ts_pid( pending_ts ) != mpeg_video_pid )
                            transition_prefix.insert( transition_prefix.end(), pending_ts, pending_ts + TS_PACKET_SIZE );
                    }
                }
                else if ( !is_preroll_frame( current_out_frame_id ) )
                    emit_mpeg_payload( mpeg_chunk.data(), ( uint16_t )mpeg_chunk.size(), tx_bufs, burst_idx );

                mpeg_chunk.clear();
            }

            begin_mpeg_frame( tx_bufs, burst_idx, timer_hz );

            if ( entering_real_stream ) {
                for ( size_t offset = 0; offset + TS_PACKET_SIZE <= transition_prefix.size(); offset += TS_PACKET_SIZE ) {
                    mpeg_chunk.insert( mpeg_chunk.end(), transition_prefix.data() + offset, transition_prefix.data() + offset + TS_PACKET_SIZE );

                    if ( mpeg_chunk.size() == MTU_PAYLOAD_SIZE ) {
                        emit_mpeg_payload( mpeg_chunk.data(), MTU_PAYLOAD_SIZE, tx_bufs, burst_idx );
                        mpeg_chunk.clear();
                    }
                }
            }
        }

        mpeg_chunk.insert( mpeg_chunk.end(), ts, ts + TS_PACKET_SIZE );
        consumed += TS_PACKET_SIZE;

        if ( current_out_frame_id > 0 && !is_preroll_frame( current_out_frame_id ) && mpeg_chunk.size() == MTU_PAYLOAD_SIZE ) {
            emit_mpeg_payload( mpeg_chunk.data(), MTU_PAYLOAD_SIZE, tx_bufs, burst_idx );
            mpeg_chunk.clear();
        }
    }

    if ( consumed > 0 )
        ts_pending.erase( ts_pending.begin(), ts_pending.begin() + consumed );
}

static inline void drain_ffmpeg( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {

    // Purpose: It asynchronously drains hardware-encoded "MPEG-TS" bytes, associating the first outcome with its source frame & relaying the media chunk without blocking the data path "worker"

    while ( 1 ) {
        uint8_t read_buffer[ FFMPEG_READ_SIZE ];
        int bytes_read = read( ffmpeg_out[ 0 ], read_buffer, FFMPEG_READ_SIZE );

        if ( bytes_read > 0 )
            process_mpeg_bytes( read_buffer, bytes_read, tx_bufs, burst_idx, timer_hz );
        else if ( bytes_read < 0 && ( errno == EAGAIN || errno == EWOULDBLOCK ) ) {
            rte_pause();
            break;
        }
        else
            break;
    }

    if ( *burst_idx > 0 && current_out_frame_id > 0 && !is_preroll_frame( current_out_frame_id ) ) {
        uint32_t idx = ( current_out_frame_id - 1 ) % K_FRAMES;

        flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ idx ].tx_packets, &telemetry_log[ idx ].tx_zero_accepts, &telemetry_log[ idx ].tx_partial_accepts, &telemetry_log[ idx ].tx_resubmit_calls, &telemetry_log[ idx ].tx_resubmitted_packets, &frame_last_egress_cycles[ idx ] );
    }
}

static inline void poll_network_rx() {

    // Purpose: It drains the SFF2-facing standard "UDP" receive queue, situating valid point packet at deterministic offsets derived from "sequence_number" values.
    //          Geometry metadata are supplied by the SFF2 proxy as application-side context, while "NSH" elaboration remain outside Encoder

    struct rte_mbuf *bufs[ BURST_SIZE ];

    while ( 1 ) {
        uint16_t nb_rx = rte_eth_rx_burst( PORT_SFF2, 0, bufs, BURST_SIZE );

        if ( nb_rx == 0 )
            break;

        for ( uint16_t i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];

            const size_t minimum_packet_size = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct geo_agg_hdr ) + sizeof( struct cam_hdr );

            if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < minimum_packet_size ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

            if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

            if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( SFF2_ENCODER_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( ENCODER_SFF2_IP ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

            if ( unlikely( udp -> src_port != rte_cpu_to_be_16( SFF2_ENCODER_PORT ) || udp -> dst_port != rte_cpu_to_be_16( ENCODER_SFF2_PORT ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
            uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );

            if ( unlikely( udp_length < sizeof( struct rte_udp_hdr ) + sizeof( struct geo_agg_hdr ) + sizeof( struct cam_hdr ) || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length || rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + ipv4_length ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t udp_payload_length = udp_length - sizeof( struct rte_udp_hdr );
            struct geo_agg_hdr *geo = ( struct geo_agg_hdr * )( udp + 1 );
            struct cam_hdr *cam = ( struct cam_hdr * )( geo + 1 );

            size_t application_bytes = udp_payload_length - sizeof( struct geo_agg_hdr );
            uint32_t frame_id = rte_be_to_cpu_32( cam -> frame_id );
            uint32_t points_in_packet = rte_be_to_cpu_32( cam -> points_in_packet );
            uint32_t original_points = rte_be_to_cpu_32( cam -> original_points );
            uint64_t packet_arrival = rte_get_timer_cycles();

            if ( unlikely( frame_id == END_OF_STREAM ) ) {
                if ( unlikely( application_bytes != sizeof( struct cam_hdr ) || points_in_packet != 0 || original_points != 0 ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                eos_received = true;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( frame_id == 0 || frame_id > K_FRAMES || frame_id <= last_processed_frame_id || original_points == 0 || original_points > MAX_POINTS || points_in_packet == 0 || points_in_packet > POINTS_PER_PACKET ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            size_t point_payload_size = application_bytes - sizeof( struct cam_hdr );

            if ( unlikely( point_payload_size != ( size_t )points_in_packet * sizeof( struct point_tx ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t incoming_active_points = rte_be_to_cpu_32( geo -> active_point_count );

            if ( unlikely( incoming_active_points > original_points || incoming_active_points < points_in_packet ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            frame_buffer &fb = frame_buffers[ frame_id ];

            if ( fb.rx_packets == 0 ) {
                fb.cam = *cam;
                fb.original_points = original_points;
                fb.expected_packets = ( original_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET;
                fb.camera_tx = rte_be_to_cpu_64( cam -> timestamp );
                fb.first_arrival = packet_arrival;
                fb.points.reset( new struct host_point[ original_points ] );
                fb.packet_received.assign( fb.expected_packets, 0 );
            }
            else if ( unlikely( fb.original_points != original_points || fb.cam.temporal_skip != cam -> temporal_skip ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t sequence_number = rte_be_to_cpu_32( cam -> sequence_number );

            if ( unlikely( sequence_number >= fb.expected_packets ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t point_offset = sequence_number * POINTS_PER_PACKET;
            uint32_t expected_points = std::min( ( uint32_t )POINTS_PER_PACKET, fb.original_points - point_offset );

            if ( unlikely( points_in_packet != expected_points ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( fb.packet_received[ sequence_number ] != 0 ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t stored_active_points = ( fb.rx_packets == 0 ) ? 0 : rte_be_to_cpu_32( fb.geo.active_point_count );

            if ( fb.rx_packets == 0 || incoming_active_points >= stored_active_points )
                fb.geo = *geo;

            struct point_tx *points = ( struct point_tx * )( ( uint8_t * )cam + sizeof( struct cam_hdr ) );
            uint64_t t_conversion_start = rte_get_timer_cycles();

            for ( uint32_t point_id = 0; point_id < points_in_packet; point_id++ ) {
                struct host_point &point = fb.points[ point_offset + point_id ];

                point.x = be_32_to_float( points[ point_id ].x );
                point.y = be_32_to_float( points[ point_id ].y );
                point.z = be_32_to_float( points[ point_id ].z );
                point.r = points[ point_id ].r;
                point.g = points[ point_id ].g;
                point.b = points[ point_id ].b;
                point.padding = 0;
            }

            uint64_t t_conversion_end = rte_get_timer_cycles();

            fb.packet_received[ sequence_number ] = 1;
            fb.received_points += points_in_packet;
            fb.conversion_cycles += t_conversion_end - t_conversion_start;
            fb.last_arrival = packet_arrival;
            fb.payload_bytes += point_payload_size;
            fb.rx_packets++;

            if ( fb.received_points == fb.original_points )
                fb.frame_ready = t_conversion_end;

            rte_pktmbuf_free( m );
        }
    }
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();
    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    int burst_idx = 0;

    uint64_t t_session_start = 0;
    uint32_t frames_received = 0;
    bool is_first_frame = true;
    double global_clock_offset_sec = 0.0;
    uint64_t prev_arrival_cyc = 0;
    uint32_t prev_arrival_f_id = 0;
    uint32_t first_arrival_f_id = 0;

    printf( "\n[SYSTEM] Conversion is about to begin at %.1f FPS...\n\n", TARGET_FPS );

    while ( 1 ) {
        if ( csv_written ) {
            rte_pause();
            continue;
        }

        poll_network_rx();
        drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );

        uint32_t frame_to_process = 0;

        if ( !frame_buffers.empty() ) {
            auto oldest = frame_buffers.begin();
            frame_buffer &candidate = oldest -> second;
            bool candidate_complete = candidate.original_points > 0 && candidate.received_points == candidate.original_points;

            if ( candidate_complete || eos_received )
                frame_to_process = oldest -> first;
        }

        if ( frame_to_process > 0 ) {
            frame_buffer &fb = frame_buffers[ frame_to_process ];
            bool frame_complete = fb.original_points > 0 && fb.received_points == fb.original_points;
            uint32_t idx = ( frame_to_process - 1 ) % K_FRAMES;
            uint16_t active_temporal_skip = rte_be_to_cpu_16( fb.cam.temporal_skip );

            if ( active_temporal_skip == 0 )
                active_temporal_skip = 1;

            struct telemetry_csv *t = &telemetry_log[ idx ];
            memset( t, 0, sizeof( *t ) );

            frame_first_arrival_cycles[ idx ] = fb.first_arrival;
            frame_last_egress_cycles[ idx ] = 0;

            t -> frame_id = frame_to_process;
            t -> status = frame_complete ? 1 : 0;
            t -> current_skip = active_temporal_skip;

            snprintf( t -> event, sizeof( t -> event ), "IDLE" );

            t -> yaw = 0.0f;
            t -> pitch = 0.0f;
            t -> zoom = 1.0f;
            t -> camera_send_timestamp = ( double )fb.camera_tx / timer_hz;
            t -> recv_start_timestamp = ( double )fb.first_arrival / timer_hz;
            t -> original_points = fb.original_points;
            t -> rx_points = fb.received_points;
            t -> tx_points = fb.received_points;
            t -> rx_packets = fb.rx_packets;
            t -> payload_bytes = fb.payload_bytes;
            t -> data_integrity_pct = ( fb.original_points > 0 ) ? ( ( double )fb.received_points / fb.original_points ) * 100.0 : 0.0;
            t -> conversion_ms = ( ( double )fb.conversion_cycles / timer_hz ) * 1000.0;

            if ( is_first_frame ) {
                global_clock_offset_sec = ( ( double )fb.first_arrival / timer_hz ) - ( ( double )fb.camera_tx / timer_hz );
                is_first_frame = false;
            }

            t -> clock_offset_ms = global_clock_offset_sec * 1000.0;
            t -> camera_to_node_latency_ms = ( ( ( double )fb.first_arrival / timer_hz ) - ( ( double )fb.camera_tx / timer_hz ) - global_clock_offset_sec ) * 1000.0;

            if ( prev_arrival_cyc > 0 && frame_to_process > prev_arrival_f_id ) {
                double real_interval_sec = ( double )( fb.first_arrival - prev_arrival_cyc ) / timer_hz;
                double expected_interval_sec = ( double )( frame_to_process - prev_arrival_f_id ) / TARGET_FPS;

                t -> network_jitter_ms = std::abs( real_interval_sec - expected_interval_sec ) * 1000.0;
            }

            prev_arrival_cyc = fb.first_arrival;
            prev_arrival_f_id = frame_to_process;

            double receive_sec = ( fb.last_arrival >= fb.first_arrival ) ? ( double )( fb.last_arrival - fb.first_arrival ) / timer_hz : 0.0;
            uint64_t logical_rx_frame_bytes = ( uint64_t )fb.payload_bytes + ( fb.rx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );

            t -> internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_rx_frame_bytes / 1000000.0 ) / receive_sec : 0.0;

            std::vector< struct host_point > compact_points;
            const struct host_point *active_points = fb.points.get();
            uint32_t active_point_count = fb.received_points;

            if ( !frame_complete ) {
                compact_points.reserve( fb.received_points );

                for ( uint32_t sequence_number = 0; sequence_number < fb.expected_packets; sequence_number++ ) {
                    if ( fb.packet_received[ sequence_number ] == 0 )
                        continue;

                    uint32_t point_offset = sequence_number * POINTS_PER_PACKET;
                    uint32_t point_count = std::min( ( uint32_t )POINTS_PER_PACKET, fb.original_points - point_offset );

                    compact_points.insert( compact_points.end(), fb.points.get() + point_offset, fb.points.get() + point_offset + point_count );
                }

                active_points = compact_points.data();
                active_point_count = ( uint32_t )compact_points.size();
            }

            if ( frames_received == 0 ) {
                t_session_start = fb.first_arrival;
                first_arrival_f_id = frame_to_process;
            }

            uint32_t frame_offset = frame_to_process - first_arrival_f_id;
            uint64_t service_start = rte_get_timer_cycles();

            double actual_start_sec = ( service_start >= t_session_start ) ? ( double )( service_start - t_session_start ) / timer_hz : 0.0;
            double ideal_start_sec = ( double )frame_offset / TARGET_FPS;

            t -> schedule_delay_ms = ( actual_start_sec - ideal_start_sec ) * 1000.0;

            uint64_t ready_reference = fb.frame_ready > 0 ? fb.frame_ready : fb.last_arrival;

            t -> wait_raw_queue_ms = ( ready_reference > 0 && service_start >= ready_reference ) ? ( ( double )( service_start - ready_reference ) / timer_hz ) * 1000.0 : 0.0;

            geometry_result geometry;

            if ( unlikely( !resolve_geometry( fb, active_points, active_point_count, &geometry, timer_hz ) ) ) {
                t -> status = 0;
                snprintf( t -> event, sizeof( t -> event ), "INVALID" );

                last_processed_frame_id = frame_to_process;
                frame_buffers.erase( frame_to_process );
                continue;
            }

            t -> geometry_aggregation_ms = geometry.geometry_aggregation_ms;
            t -> max_r_ms = geometry.max_r_ms;

            float target_radius = CAMERA_DISTANCE * 0.2f;
            float final_scale = ( geometry.max_r > 0.0f ) ? target_radius / geometry.max_r : 1.0f;

            double slot_wait_ms = 0.0;
            int yuv_slot = acquire_yuv_slot( tx_bufs, &burst_idx, timer_hz, &slot_wait_ms );

            uint64_t t_projection_start = rte_get_timer_cycles();
            float global_scale = 1.0f;
            float bbox_center_x = 0.0f;
            float bbox_center_y = 0.0f;
            float bbox_center_z = 0.0f;
            uint64_t projection_end_cycles = 0;
            double gpu_metrics[ 4 ] = { 0.0, 0.0, 0.0, 0.0 };

            run_projection_pipeline( active_points, active_point_count, geometry.centroid_x, geometry.centroid_y, geometry.centroid_z, geometry.extent_x, geometry.extent_y, geometry.extent_z, geometry.bbox_center_x, geometry.bbox_center_y, geometry.bbox_center_z, final_scale, CAMERA_DISTANCE, yuv_buffers[ yuv_slot ].data(), gpu_metrics, &global_scale, &bbox_center_x, &bbox_center_y, &bbox_center_z, &projection_end_cycles, poll_network_rx );

            t -> projection_ms = ( ( double )( projection_end_cycles - t_projection_start ) / timer_hz ) * 1000.0;
            t -> gpu_transfer_ms = gpu_metrics[ 0 ];
            t -> gpu_kernel_ms = gpu_metrics[ 1 ];
            t -> gpu_packing_ms = gpu_metrics[ 2 ];
            t -> gpu_copyback_ms = gpu_metrics[ 3 ];

            double measured_gpu_ms = gpu_metrics[ 0 ] + gpu_metrics[ 1 ] + gpu_metrics[ 2 ] + gpu_metrics[ 3 ];
            t -> host_overhead_ms = ( t -> projection_ms > measured_gpu_ms ) ? t -> projection_ms - measured_gpu_ms : 0.0;

            encoder_metadata[ idx ].frame_id = rte_cpu_to_be_32( frame_to_process );
            encoder_metadata[ idx ].final_scale = float_to_be_32( final_scale );
            encoder_metadata[ idx ].global_scale = float_to_be_32( global_scale );
            encoder_metadata[ idx ].box_center_x = float_to_be_32( bbox_center_x );
            encoder_metadata[ idx ].box_center_y = float_to_be_32( bbox_center_y );
            encoder_metadata[ idx ].box_center_z = float_to_be_32( bbox_center_z );
            encoder_metadata[ idx ].yaw = float_to_be_32( 0.0f );
            encoder_metadata[ idx ].pitch = float_to_be_32( 0.0f );
            encoder_metadata[ idx ].centroid_x = float_to_be_32( geometry.centroid_x );
            encoder_metadata[ idx ].centroid_y = float_to_be_32( geometry.centroid_y );
            encoder_metadata[ idx ].centroid_z = float_to_be_32( geometry.centroid_z );

            camera_metadata[ idx ] = fb.cam;
            camera_metadata[ idx ].yaw = float_to_be_32( 0.0f );
            camera_metadata[ idx ].pitch = float_to_be_32( 0.0f );
            camera_metadata[ idx ].zoom = float_to_be_32( 1.0f );
            camera_metadata[ idx ].sequence_number = 0;
            camera_metadata[ idx ].points_in_packet = 0;

            frames_received++;

            struct yuv_job job;

            job.frame_id = frame_to_process;
            job.frame_offset = frame_offset;
            job.slot = ( uint8_t )yuv_slot;
            job.first_arrival = fb.first_arrival;
            job.projection_end_cycles = projection_end_cycles;
            job.session_start_cycles = t_session_start;
            job.slot_wait_ms = slot_wait_ms;
            job.global_clock_offset_sec = global_clock_offset_sec;

            if ( unlikely( !enqueue_yuv_job( job ) ) ) {
                pthread_mutex_lock( &writer_mutex );
                yuv_slot_free[ yuv_slot ] = true;
                pthread_cond_broadcast( &writer_slot_released );
                pthread_mutex_unlock( &writer_mutex );

                t -> status = 0;
                snprintf( t -> event, sizeof( t -> event ), "INVALID" );
            }

            uint64_t service_end = rte_get_timer_cycles();
            double workload_service_ms = ( ( double )( service_end - service_start ) / timer_hz ) * 1000.0;

            uint32_t frame_backlog = frame_buffers.size() > 0 ? ( uint32_t )frame_buffers.size() - 1 : 0;
            uint32_t codec_backlog = std::max( writer_pending_frames(), ( uint32_t )mpeg_frame_queue.size() );

            update_workload_controller( frame_to_process, active_temporal_skip, workload_service_ms, t -> wait_raw_queue_ms, frame_backlog, codec_backlog, t );

            drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );

            last_processed_frame_id = frame_to_process;
            frame_buffers.erase( frame_to_process );
        }

        if ( eos_received && frame_buffers.empty() && !csv_written ) {
            wait_for_idle( tx_bufs, &burst_idx, timer_hz );
            ffmpeg_writer_stop();
            close( ffmpeg_in[ 1 ] );

            int ffmpeg_flags = fcntl( ffmpeg_out[ 0 ], F_GETFL, 0 );

            if ( ffmpeg_flags >= 0 )
                fcntl( ffmpeg_out[ 0 ], F_SETFL, ffmpeg_flags & ~O_NONBLOCK );

            drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );

            if ( current_out_frame_id > 0 && !mpeg_chunk.empty() ) {
                emit_mpeg_payload( mpeg_chunk.data(), ( uint16_t )mpeg_chunk.size(), tx_bufs, &burst_idx );
                mpeg_chunk.clear();
            }

            ts_pending.clear();

            if ( burst_idx > 0 ) {
                if ( current_out_frame_id > 0 ) {
                    uint32_t retry_idx = ( current_out_frame_id - 1 ) % K_FRAMES;

                    flush_tx_burst( tx_bufs, &burst_idx, &telemetry_log[ retry_idx ].tx_packets, &telemetry_log[ retry_idx ].tx_zero_accepts, &telemetry_log[ retry_idx ].tx_partial_accepts, &telemetry_log[ retry_idx ].tx_resubmit_calls, &telemetry_log[ retry_idx ].tx_resubmitted_packets, &frame_last_egress_cycles[ retry_idx ] );
                }
                else
                    flush_tx_burst( tx_bufs, &burst_idx, NULL, NULL, NULL, NULL, NULL );
            }

            close( ffmpeg_out[ 0 ] );
            waitpid( ffmpeg_pid, NULL, 0 );

            struct rte_mbuf *m_eos = rte_pktmbuf_alloc( mbuf_pool );

            if ( m_eos != NULL ) {
                size_t headers_len = outer_len + sizeof( struct cam_hdr );
                char *data = ( char * )rte_pktmbuf_append( m_eos, headers_len );

                if ( data != NULL ) {
                    struct rte_ether_hdr *eth_out = ( struct rte_ether_hdr * )data;
                    struct rte_ipv4_hdr *ipv4_out = ( struct rte_ipv4_hdr * )( eth_out + 1 );
                    struct rte_udp_hdr *udp_out = ( struct rte_udp_hdr * )( ipv4_out + 1 );
                    struct cam_hdr *cam_out = ( struct cam_hdr * )( udp_out + 1 );

                    uint16_t udp_payload_len = sizeof( struct cam_hdr );

                    header_init( eth_out, ipv4_out, udp_out, udp_payload_len );

                    memset( cam_out, 0, sizeof( struct cam_hdr ) );
                    cam_out -> frame_id = rte_cpu_to_be_32( END_OF_STREAM );

                    tx_bufs[ burst_idx++ ] = m_eos;
                    flush_tx_burst( tx_bufs, &burst_idx, NULL, NULL, NULL, NULL, NULL );
                }
                else
                    rte_pktmbuf_free( m_eos );
            }

            if ( controller_notification_printed )
                printf( "\n" );

            telemetry_to_csv();
            csv_written = true;
            printf( "\n[SYSTEM] End of stream detected. Changing to \"idle\" state...\n" );
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It starts the Encoder service function, persistent hardware "H.265" subprocess, "CUDA" / "I420" resources, & "FFmpeg" components before delegating data-path processing to "DPDK" "worker"

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"Encoder\" microservice...\n\n" );

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF2, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF2-facing virtual port configuration failed...\n" );

    cuda_memory_init( MAX_POINTS );
    cuda_memory_warmup();

    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
        yuv_buffers[ slot ].resize( TOTAL_YUV_SIZE );
        cuda_memory_register( yuv_buffers[ slot ].data(), TOTAL_YUV_SIZE );
    }

    ffmpeg_init();

    for ( uint32_t i = 0; i < K_FRAMES; i++ )
        frame_encode_start_cycles[ i ].store( 0, std::memory_order_relaxed );

    ffmpeg_writer_start();
    ffmpeg_preroll();

    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE )
        worker_loop( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    ffmpeg_writer_stop();

    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ )
        cuda_memory_unleash( yuv_buffers[ slot ].data() );

    cuda_memory_free();
    rte_eal_cleanup();

    return 0;
}