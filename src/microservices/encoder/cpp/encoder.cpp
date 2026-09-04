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

#define MBUF_DATA_SIZE ( RTE_PKTMBUF_HEADROOM + NETWORK_MTU + sizeof( struct rte_ether_hdr ) + 64 )

struct encode_service_sample {
    uint32_t frame_id = 0;
    uint64_t start_cycles = 0;
};

struct frame_buffer {
    std::unique_ptr< struct host_point[] > points;
    std::vector< uint8_t > packet_received;

    uint32_t original_points = 0;
    uint32_t expected_packets = 0;
    uint16_t points_per_packet = POINTS_PER_PACKET;
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

    float final_scale = 1.0f;
    float global_scale = 1.0f;
    float projected_bbox_x = 0.0f;
    float projected_bbox_y = 0.0f;
    float projected_bbox_z = 0.0f;
    bool projection_geometry_ready = false;

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

    uint32_t previous_codec_backlog = 0;
    bool codec_backlog_initialized = false;

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

    bool preroll = false;
};

// Global application state
static struct rte_mempool *mbuf_pool;

static const struct rte_ether_addr encoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x01 } };
static const struct rte_ether_addr sff2_encoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x02 } };

static int ffmpeg_in[ 2 ];
static int ffmpeg_out[ 2 ];
static pid_t ffmpeg_pid;

static struct telemetry_csv telemetry_log[ K_FRAMES ];
static bool csv_written = false;
static bool eos_received = false;

static struct enc_hdr encoder_metadata[ K_FRAMES ];
static struct cam_hdr camera_metadata[ K_FRAMES ];
static uint32_t current_frame_id = 0;
static uint32_t last_frame_id = 0;
static uint32_t highest_frame_id = 0;
static uint32_t current_mpeg_packet = 0;

static std::vector< uint8_t > ts_pending;
static std::vector< uint8_t > mpeg_chunk;
static size_t mpeg_chunk_ts_offset = 0;

static const size_t outer_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

static std::map< uint32_t, frame_buffer > frame_buffers;
static std::queue< uint32_t > mpeg_frame_queue;

static std::queue< struct encode_service_sample > encode_service_queue;
static size_t encode_service_bytes = 0;

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

static pthread_mutex_t encode_service_mutex = PTHREAD_MUTEX_INITIALIZER;

static std::atomic< uint64_t > frame_start_cycles[ K_FRAMES ];
static uint64_t frame_arrival_cycles[ K_FRAMES ] = { 0 };
static uint64_t frame_ready_cycles[ K_FRAMES ] = { 0 };
static uint64_t frame_egress_cycles[ K_FRAMES ] = { 0 };
static uint64_t frame_active_tx_cycles[ K_FRAMES ] = { 0 };
static uint32_t frame_created_packets[ K_FRAMES ] = { 0 };

static std::vector< uint8_t > quality_reference;
static std::vector< uint8_t > quality_stream;

static std::vector< uint8_t > debug_snapshot;
static bool debug_snapshot_ready = false;

static size_t quality_stream_size = 0;
static bool quality_failed = false;
static bool quality_capture_enabled = true;

static uint32_t quality_frame_ids[ K_FRAMES ] = { 0 };
static uint32_t quality_frame_count = 0;

// Function prototypes
static inline void drain_codec_output( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz );
static inline void process_network_stream();
static inline bool flush_tx_burst( struct rte_mbuf **tx_bufs, int *burst_idx, uint32_t *tx_packets, uint32_t *tx_zero_accepts, uint32_t *tx_partial_accepts, uint32_t *tx_resubmit_calls, uint32_t *tx_resubmitted_packets, uint64_t *last_egress_cycles = NULL, uint64_t *active_tx_cycles = NULL );
static void quality_capture_init();
static inline void write_reference( uint32_t frame_id, const uint8_t *yuv );
static inline void write_stream( const uint8_t *data, size_t size );
static void quality_capture_close();
static void evaluate_quality_capture();

// Data path & support routines
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

static inline bool is_preroll_frame( uint32_t frame_id ) {
    
    // Purpose: It determines whether the current shot belongs to the designated initialization sequence
    
    return frame_id == FRAME_ID;
}

static inline uint16_t mpeg_ts_pid( const uint8_t *ts ) {
    
    // Purpose: It extracts the 13-bit "Packet Identifier" ( "PID" ) from an "MPEG-TS" header
    
    return ( ( uint16_t )( ts[ 1 ] & 0x1F ) << 8 ) | ts[ 2 ];
}

static inline bool ts_starts_video_pes( const uint8_t *ts ) {

    // Purpose: It identifies a video "Packetized Elementary Stream" ( "PES" ) boundary by combining the "MPEG-TS" payload-unit-start indicator with the elementary-progression start code.
    //          Such a parser is employed only for source-frame association. Full transport packets remain byte-preserved, & the encoded "H.265" components are never rewritten

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

static void quality_capture_init() {

    // Purpose: It establishes bounded in-memory capture queues whose persistent file writes are delegated to an asynchronous component outside the measured real-time path

    struct stat st = { 0 };

    quality_failed = false;
    quality_frame_count = 0;
    quality_stream_size = 0;

    if ( stat( TELEMETRY_FOLDER, &st ) == -1 )
        if ( mkdir( TELEMETRY_FOLDER, 0777 ) == -1 ) {
            printf( "[SYSTEM] Error: Failed to create directory \"%s\"...\n", TELEMETRY_FOLDER );
            quality_failed = true;
            return;
        }

    try {
        quality_reference.resize( ( size_t )SIZE_Y * K_FRAMES );
        quality_stream.resize( QUALITY_STREAM_SIZE );
    }
    catch ( ... ) {
        printf( "[SYSTEM] Error: Unable to allocate quality buffers...\n" );

        quality_reference.clear();
        quality_stream.clear();

        quality_failed = true;
        return;
    }

}

static inline void write_reference( uint32_t frame_id, const uint8_t *yuv ) {

    // Purpose: It copies each real "Y" reference into a storage position & returns immediately, leaving persistent serialization to the involved writer

    if ( !quality_capture_enabled || quality_failed || yuv == NULL || frame_id == 0 || frame_id > K_FRAMES || quality_frame_count >= K_FRAMES )
        return;

    size_t offset = ( size_t )quality_frame_count * SIZE_Y;

    if ( offset + SIZE_Y > quality_reference.size() ) {
        quality_failed = true;
        return;
    }

    rte_memcpy( quality_reference.data() + offset, yuv, SIZE_Y );

    quality_frame_ids[ quality_frame_count++ ] = frame_id;
}

static inline void write_stream( const uint8_t *data, size_t size ) {

    // Purpose: It copies byte-preserved effective "MPEG-TS" chunks into a fixed-capacity buffer without issuing file-system writes from the "DPDK" transmission path

    if ( !quality_capture_enabled || quality_failed || data == NULL || size == 0 )
        return;

    if ( quality_stream_size + size > quality_stream.size() ) {
        printf( "[SYSTEM] Error: Quality buffer exhausted...\n" );
        quality_failed = true;
        return;
    }

    rte_memcpy( quality_stream.data() + quality_stream_size, data, size );
    quality_stream_size += size;
}

static void quality_capture_close() {

    // Purpose: It stops the asynchronous producer-consumer session only after all queued samples have been marshaled, thereby preserving post-session metric completeness

    if ( !quality_capture_enabled || quality_failed )
        return;

    FILE *reference_file = fopen( REFERENCE_PATH, "wb" );
    FILE *encoded_file = fopen( ENCODED_PATH, "wb" );

    if ( reference_file == NULL || encoded_file == NULL ) {
        printf( "[SYSTEM] Error: Unable to initialize quality serialization...\n" );
        quality_failed = true;

        if ( reference_file != NULL )
            fclose( reference_file );

        if ( encoded_file != NULL )
            fclose( encoded_file );

        return;
    }

    setvbuf( reference_file, NULL, _IOFBF, 4 * 1024 * 1024 );
    setvbuf( encoded_file, NULL, _IOFBF, 1024 * 1024 );

    size_t reference_size = ( size_t )quality_frame_count * SIZE_Y;

    if ( reference_size > 0 &&
         fwrite( quality_reference.data(), 1, reference_size, reference_file ) != reference_size )
        quality_failed = true;

    if ( !quality_failed &&
         quality_stream_size > 0 &&
         fwrite( quality_stream.data(), 1, quality_stream_size, encoded_file ) != quality_stream_size )
        quality_failed = true;

    fflush( reference_file );
    fflush( encoded_file );

    fclose( reference_file );
    fclose( encoded_file );

    if ( quality_failed )
        printf( "[SYSTEM] Error: Quality serialization failed...\n" );

    std::vector< uint8_t >().swap( quality_reference );
    std::vector< uint8_t >().swap( quality_stream );
}

static bool ffmpeg_quality_run( const char *filter_graph, uint32_t frame_count ) {
    
    // Purpose: It creates an isolated subprocess to carry out a designated filter graph, systematically comparing the compressed result against the initial reference to compute proper error metrics

    if ( filter_graph == NULL || frame_count == 0 )
        return false;

    char resolution[ 32 ];
    char frames[ 16 ];

    snprintf( resolution, sizeof( resolution ), "%dx%d", ENCODER_W, ENCODER_H );
    snprintf( frames, sizeof( frames ), "%u", frame_count );

    pid_t pid = fork();

    if ( pid < 0 )
        return false;

    if ( pid == 0 ) {
        execlp( "ffmpeg", "ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", ENCODED_PATH, "-f", "rawvideo", "-pix_fmt", "gray", "-s", resolution, "-r", "30", "-i", REFERENCE_PATH, "-lavfi", filter_graph, "-frames:v", frames, "-f", "null", "-", NULL );
        exit( 127 );
    }

    int status = 0;

    if ( waitpid( pid, &status, 0 ) < 0 )
        return false;

    return WIFEXITED( status ) && WEXITSTATUS( status ) == 0;
}

static uint32_t process_quality_metrics() {

    // Purpose: It extracts the "Mean Squared Error" ( "MSE" ) & "Peak Signal-to-Noise Ratio" ( "PSNR" ) from the analysis trails, correlating each discrete "luma" calculation with its original frame

    FILE *f = fopen( PSNR_PATH, "r" );

    if ( f == NULL )
        return 0;

    char line[ 512 ];
    uint32_t parsed = 0;

    while ( fgets( line, sizeof( line ), f ) != NULL ) {
        uint32_t n = 0;
        double mse_avg = 0.0;
        double mse_y = 0.0;
        double psnr_avg = 0.0;
        double psnr_y = 0.0;

        if ( sscanf( line, "n:%u mse_avg:%lf mse_y:%lf psnr_avg:%lf psnr_y:%lf", &n, &mse_avg, &mse_y, &psnr_avg, &psnr_y ) != 5 )
            continue;

        if ( n == 0 || n > quality_frame_count )
            continue;

        uint32_t frame_id = quality_frame_ids[ n - 1 ];

        if ( frame_id == 0 || frame_id > K_FRAMES )
            continue;

        telemetry_log[ frame_id - 1 ].mse_y = mse_y;
        telemetry_log[ frame_id - 1 ].psnr_y = psnr_y;
        parsed++;
    }

    fclose( f );
    return parsed;
}

static uint32_t process_resemblance_indicator() {

    // Purpose: It parses the "Structural Similarity Index Measure" ( "SSIM" ) logs, mapping the component-level structural condition evaluations back to their corresponding telemetry records

    FILE *f = fopen( SSIM_PATH, "r" );

    if ( f == NULL )
        return 0;

    char line[ 512 ];
    uint32_t parsed = 0;

    while ( fgets( line, sizeof( line ), f ) != NULL ) {
        uint32_t n = 0;
        double ssim_y = 0.0;

        if ( sscanf( line, "n:%u Y:%lf", &n, &ssim_y ) != 2 )
            continue;

        if ( n == 0 || n > quality_frame_count )
            continue;

        uint32_t frame_id = quality_frame_ids[ n - 1 ];

        if ( frame_id == 0 || frame_id > K_FRAMES )
            continue;

        telemetry_log[ frame_id - 1 ].ssim_y = ssim_y;
        parsed++;
    }

    fclose( f );
    return parsed;
}

static void evaluate_quality_capture() {

    // Purpose: It automates the comprehensive objective assessment, invoking the computational distortion algorithms strictly after sequence termination to preserve runtime temporal fidelity

    if ( !quality_capture_enabled )
        return;

    quality_capture_close();

    if ( quality_failed || quality_frame_count == 0 ) {
        printf( "[SYSTEM] Error: Quality metrics are unavailable...\n" );
        return;
    }

    printf( "\n[SYSTEM] Waiting for parameter assessments to complete...\n" );
    
    char psnr_filter[ 512 ];
    char ssim_filter[ 512 ];

    snprintf( psnr_filter, sizeof( psnr_filter ), "[0:v]setpts=PTS-STARTPTS,extractplanes=y[dist];" "[1:v]setpts=PTS-STARTPTS[ref];" "[dist][ref]psnr=stats_file=%s", PSNR_PATH );
    snprintf( ssim_filter, sizeof( ssim_filter ), "[0:v]setpts=PTS-STARTPTS,extractplanes=y[dist];" "[1:v]setpts=PTS-STARTPTS[ref];" "[dist][ref]ssim=stats_file=%s", SSIM_PATH );

    bool psnr_ok = ffmpeg_quality_run( psnr_filter, quality_frame_count );
    bool ssim_ok = ffmpeg_quality_run( ssim_filter, quality_frame_count );

    uint32_t psnr_rows = psnr_ok ? process_quality_metrics() : 0;
    uint32_t ssim_rows = ssim_ok ? process_resemblance_indicator() : 0;

    if ( psnr_rows == quality_frame_count && ssim_rows == quality_frame_count ) {
        unlink( REFERENCE_PATH );
        unlink( ENCODED_PATH );
        unlink( PSNR_PATH );
        unlink( SSIM_PATH );
        printf( "[SYSTEM] Quality indicators computed for %u elements.\n\n", quality_frame_count );
    }
    else
        printf( "[SYSTEM] Error: Quality analysis recovered \"PSNR\" / \"MSE\" on %u / %u frames & \"SSIM\" across %u / %u shots...\n", psnr_rows, quality_frame_count, ssim_rows, quality_frame_count );
}

static inline int port_init( uint16_t port, struct rte_mempool *mbuf_pool ) {
    struct rte_eth_conf port_conf = { 0 };
    int retval;

    if ( ! rte_eth_dev_is_valid_port( port ) ) 
        return -1;

    retval = rte_eth_dev_configure( port, 1, 1, &port_conf );
    
    if ( retval != 0 ) 
        return retval;

    if ( NETWORK_MTU != 1500 ) {
        retval = rte_eth_dev_set_mtu( port, NETWORK_MTU );

        if ( retval < 0 )
            return retval;
    }

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

    rte_ether_addr_copy( &encoder_mac, &eth -> src_addr );
    rte_ether_addr_copy( &sff2_encoder_mac, &eth -> dst_addr );

    eth -> ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    ipv4 -> version_ihl = 0x45;
    ipv4 -> time_to_live = 64;
    ipv4 -> next_proto_id = IPPROTO_UDP;
    ipv4 -> src_addr = rte_cpu_to_be_32( ENCODER_IP );
    ipv4 -> dst_addr = rte_cpu_to_be_32( SFF2_ENCODER_IP );
    ipv4 -> total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + udp_payload_len );

    udp -> src_port = rte_cpu_to_be_16( ENCODER_PORT );
    udp -> dst_port = rte_cpu_to_be_16( SFF2_ENCODER_PORT );
    udp -> dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + udp_payload_len );
    udp -> dgram_cksum = 0;

    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );
}

static void debug_write_plane( const char *path, const uint8_t *plane, uint32_t width, uint32_t height ) {

    // Purpose: It exports a given diagnostic element as a directly viewable grayscale image

    FILE *f = fopen( path, "wb" );

    if ( f == NULL )
        return;

    fprintf( f, "P5\n%u %u\n255\n", width, height );
    fwrite( plane, 1, ( size_t )width * height, f );
    fclose( f );
}

static inline void debug_capture_frame( uint32_t frame_id, const uint8_t *i420 ) {

    // Purpose: It preserves the selected Encoder "I420" "Super-Frame" in memory & postpones persistent visual serialization until the streaming session has completed

    if ( DEBUG_VISUALS != DEBUG_VISUALS_ENABLED || frame_id != DEBUG_FRAME_ID || i420 == NULL || debug_snapshot.size() != TOTAL_YUV_SIZE )
        return;

    rte_memcpy( debug_snapshot.data(), i420, TOTAL_YUV_SIZE );
    debug_snapshot_ready = true;
}

static void debug_dump_frame() {

    // Purpose: It serializes the retained snapshot & its "Geometry", "Texture" & "Occupancy" planes after post-roll, optional quality assessment & sequence termination

    if ( DEBUG_VISUALS != DEBUG_VISUALS_ENABLED || !debug_snapshot_ready || debug_snapshot.size() != TOTAL_YUV_SIZE )
        return;

    mkdir( TELEMETRY_FOLDER, 0777 );
    mkdir( DEBUG_FOLDER, 0777 );

    char path[ 256 ];
    const uint8_t *i420 = debug_snapshot.data();

    snprintf( path, sizeof( path ), "%s/frame_%u_input.i420", DEBUG_FOLDER, ( unsigned int )DEBUG_FRAME_ID );
    FILE *raw = fopen( path, "wb" );

    if ( raw != NULL ) {
        fwrite( i420, 1, TOTAL_YUV_SIZE, raw );
        fclose( raw );
    }

    const uint8_t *y = i420;

    snprintf( path, sizeof( path ), "%s/frame_%u_geometry.pgm", DEBUG_FOLDER, ( unsigned int )DEBUG_FRAME_ID );
    debug_write_plane( path, y, ENCODER_W, CROSS_H );

    snprintf( path, sizeof( path ), "%s/frame_%u_texture_y.pgm", DEBUG_FOLDER, ( unsigned int )DEBUG_FRAME_ID );
    debug_write_plane( path, y + ( size_t )ENCODER_W * CROSS_H, ENCODER_W, CROSS_H );

    snprintf( path, sizeof( path ), "%s/frame_%u_occupancy.pgm", DEBUG_FOLDER, ( unsigned int )DEBUG_FRAME_ID );
    debug_write_plane( path, y + ( size_t )ENCODER_W * CROSS_H * 2, ENCODER_W, CROSS_H );

    debug_snapshot_ready = false;
    std::vector< uint8_t >().swap( debug_snapshot );
}

static void ffmpeg_init() {

    // Purpose: It creates operating-system inter-process communication "pipes" & spawns the persistent hardware-accelerated "H.265" subprocess before timed frame processing.
    //          The input element remains blocking only for the dedicated writer thread, while "FFmpeg" applies the configured queue argument & low-delay "NVENC" / "muxer" settings.
    //          This function segregates "codec"-side management from the "DPDK" "worker", thereby keeping session startup outside the measured application stream
    
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

        execlp( "ffmpeg", "ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo", "-s", res, "-pix_fmt", "yuv420p", "-r", "30", "-i", "-", "-c:v", "hevc_nvenc", "-preset", "p2", "-tune", "ull", "-delay", DELAY, "-rc", "cbr", "-b:v", TARGET_BITRATE_MBPS, "-maxrate", TARGET_BITRATE_MBPS, "-bufsize", TARGET_BUFFER_SIZE, "-g", GOP, "-forced-idr", FORCED_IDR, "-vstats_file", FFMPEG_PATH, "-flush_packets", FLUSH_PACKETS, "-f", "mpegts", "-", NULL );
        exit( 1 );
    }

    close( ffmpeg_in[ 0 ] );
    close( ffmpeg_out[ 1 ] );
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

    fprintf( f, "frame_id;rx_complete;tx_complete;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;codec_exit_time;node_exit_timestamp;original_points;rx_points;processed_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;conversion_ms;geometry_aggregation_ms;max_r_ms;projection_ms;codec_write_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;reference_jitter_ms;raw_queue_ms;render_queue_ms;workload_ewma_ms;workload_ratio;frame_backlog;codec_backlog;encode_service_ms;encode_h265_ms;mse_y;psnr_y;ssim_y;mpeg_bytes_generated;ffmpeg_write_calls;ffmpeg_write_eagain;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation\n" );

    uint32_t limit = ( last_frame_id < K_FRAMES ) ? last_frame_id : K_FRAMES;

    uint64_t schedule_start_cycles = 0;
    uint32_t schedule_frame_id = 0;

    for ( uint32_t i = 0; i < limit; i++ ) {
        if ( telemetry_log[ i ].frame_id > 0 && frame_arrival_cycles[ i ] > 0 ) {
            schedule_start_cycles = frame_arrival_cycles[ i ];
            schedule_frame_id = telemetry_log[ i ].frame_id;
            break;
        }
    }

    for ( uint32_t i = 0; i < limit; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        uint64_t first_arrival_cycles = frame_arrival_cycles[ i ];
        uint64_t frame_ready = frame_ready_cycles[ i ];
        uint64_t last_egress_cycles = frame_egress_cycles[ i ];

        t -> active_tx_ms = ( ( double )frame_active_tx_cycles[ i ] / timer_hz ) * 1000.0;
        t -> tx_complete = ( t -> mpeg_bytes_generated > 0 && frame_created_packets[ i ] > 0 && t -> tx_packets == frame_created_packets[ i ] && t -> mbuf_starvation == 0 ) ? 1 : 0;

        if ( first_arrival_cycles > 0 && last_egress_cycles >= first_arrival_cycles ) {
            double residency_sec = ( double )( last_egress_cycles - first_arrival_cycles ) / timer_hz;

            t -> total_residency_ms = residency_sec * 1000.0;
            t -> total_processing_ms = ( frame_ready > 0 && last_egress_cycles >= frame_ready ) ? ( ( double )( last_egress_cycles - frame_ready ) / timer_hz ) * 1000.0 : 0.0;
            t -> node_exit_timestamp = ( double )last_egress_cycles / timer_hz;
            t -> node_efficiency_pct = ( t -> total_residency_ms > 0.0 ) ? ( t -> active_process_ms / t -> total_residency_ms ) * 100.0 : 0.0;

            uint64_t camera_tx_cycles = rte_be_to_cpu_64( camera_metadata[ i ].timestamp );

            if ( camera_tx_cycles > 0 && last_egress_cycles >= camera_tx_cycles )
                t -> e2e_latency_ms = ( ( double )( last_egress_cycles - camera_tx_cycles ) / timer_hz ) * 1000.0;

            if ( schedule_start_cycles > 0 && t -> frame_id >= schedule_frame_id && last_egress_cycles >= schedule_start_cycles ) {
                uint32_t schedule_frame_offset = t -> frame_id - schedule_frame_id;
                double real_exit_sec = ( double )( last_egress_cycles - schedule_start_cycles ) / timer_hz;
                double ideal_exit_sec = ( double )schedule_frame_offset / TARGET_FPS;

                t -> schedule_delay_ms = ( real_exit_sec - ideal_exit_sec ) * 1000.0;
            }
            else
                t -> schedule_delay_ms = 0.0;
        }
        else
            t -> schedule_delay_ms = 0.0;

        uint16_t temporal_skip = t -> current_skip;

        if ( temporal_skip == 0 )
            temporal_skip = 1;

        double effective_fps = TARGET_FPS / temporal_skip;
        uint64_t logical_frame_bytes = ( uint64_t )t -> mpeg_bytes_generated + ( t -> tx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );
        uint64_t network_frame_bytes = ( uint64_t )t -> mpeg_bytes_generated + ( ( uint64_t )t -> tx_packets * ( sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) );

        t -> logical_bitrate_mbps = ( logical_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
        t -> network_bitrate_mbps = ( network_frame_bytes * 8.0 * effective_fps ) / 1000000.0;

        fprintf( f, "%u;%u;%u;%u;%s;%.3f;%.3f;%.3f;%.6f;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%.3f;%.3f;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%u\n", t -> frame_id, t -> rx_complete, t -> tx_complete, t -> current_skip, t -> event, t -> yaw, t -> pitch, t -> zoom, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> codec_exit_time, t -> node_exit_timestamp, t -> original_points, t -> rx_points, t -> processed_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> reference_size_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> reference_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> reference_bitrate_mbps, t -> conversion_ms, t -> geometry_aggregation_ms, t -> max_r_ms, t -> projection_ms, t -> codec_write_ms, t -> active_tx_ms, t -> active_process_ms, t -> reference_process_ms, t -> total_processing_ms, t -> total_residency_ms, t -> reference_residency_ms, t -> node_efficiency_pct, t -> reference_efficiency_pct, t -> gpu_transfer_ms, t -> gpu_kernel_ms, t -> gpu_packing_ms, t -> gpu_copyback_ms, t -> host_overhead_ms, t -> camera_node_ms, t -> e2e_latency_ms, t -> schedule_delay_ms, t -> inter_arrival_ms, t -> instant_jitter_ms, t -> desynced_jitter_ms, t -> reference_jitter_ms, t -> raw_queue_ms, t -> render_queue_ms, t -> workload_ewma_ms, t -> workload_ratio, t -> frame_backlog, t -> codec_backlog, t -> encode_service_ms, t -> encode_h265_ms, t -> mse_y, t -> psnr_y, t -> ssim_y, t -> mpeg_bytes_generated, t -> ffmpeg_write_calls, t -> ffmpeg_write_eagain, t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets, t -> mbuf_starvation );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static inline bool geometry_from_sff1( const frame_buffer &fb, const struct host_point *active_points, uint32_t active_point_count, geometry_result *result, uint64_t timer_hz ) {

    // Purpose: It exploits the latest frame-local geometry snapshot provided by SFF1 exclusively when the active point count matches the assembled set.
    //          Whole frames apply centroid, extents, bounding-box centre & "max_r". An "EOS"-finalized partial component may reuse the progressive mathematics but recomputes the precise radius over its active collection

    uint32_t metadata_active_points = rte_be_to_cpu_32( fb.geo.active_point_count );

    if ( active_points == NULL || active_point_count == 0 || metadata_active_points == 0 || metadata_active_points != active_point_count )
        return false;

    result -> centroid_x = be_to_float( fb.geo.centroid_x );
    result -> centroid_y = be_to_float( fb.geo.centroid_y );
    result -> centroid_z = be_to_float( fb.geo.centroid_z );

    result -> extent_x = be_to_float( fb.geo.extent_x );
    result -> extent_y = be_to_float( fb.geo.extent_y );
    result -> extent_z = be_to_float( fb.geo.extent_z );

    result -> bbox_center_x = be_to_float( fb.geo.bbox_center_x );
    result -> bbox_center_y = be_to_float( fb.geo.bbox_center_y );
    result -> bbox_center_z = be_to_float( fb.geo.bbox_center_z );

    bool valid_geometry = std::isfinite( result -> centroid_x ) && std::isfinite( result -> centroid_y ) && std::isfinite( result -> centroid_z ) && std::isfinite( result -> extent_x ) && std::isfinite( result -> extent_y ) && std::isfinite( result -> extent_z ) && std::isfinite( result -> bbox_center_x ) && std::isfinite( result -> bbox_center_y ) && std::isfinite( result -> bbox_center_z ) && result -> extent_x >= 0.0f && result -> extent_y >= 0.0f && result -> extent_z >= 0.0f;

    if ( !valid_geometry )
        return false;

    result -> geometry_aggregation_ms = 0.0;

    bool frame_complete = fb.received_points == fb.original_points;

    if ( frame_complete ) {
        result -> max_r = be_to_float( fb.geo.max_r );
        result -> final_scale = be_to_float( fb.geo.final_scale );
        result -> global_scale = be_to_float( fb.geo.global_scale );
        result -> projected_bbox_x = be_to_float( fb.geo.projected_bbox_x );
        result -> projected_bbox_y = be_to_float( fb.geo.projected_bbox_y );
        result -> projected_bbox_z = be_to_float( fb.geo.projected_bbox_z );
        result -> max_r_ms = 0.0;

        result -> projection_geometry_ready =
            std::isfinite( result -> final_scale ) && result -> final_scale > 0.0f &&
            std::isfinite( result -> global_scale ) && result -> global_scale > 0.0f &&
            std::isfinite( result -> projected_bbox_x ) &&
            std::isfinite( result -> projected_bbox_y ) &&
            std::isfinite( result -> projected_bbox_z );

        return std::isfinite( result -> max_r ) && result -> max_r >= 0.0f;
    }

    result -> projection_geometry_ready = false;

    uint64_t t_max_r_start = rte_get_timer_cycles();
    float max_r2 = 0.0f;

    for ( uint32_t i = 0; i < active_point_count; i++ ) {
        if ( i > 0 && i % 4096 == 0 )
            process_network_stream();

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

static inline bool compute_geometry_locally( const struct host_point *active_points, uint32_t active_point_count, geometry_result *result, uint64_t timer_hz ) {
    
    // Purpose: It constructs the full geometry metrics locally via "CPU" if hardware offloading is disabled or upstream states are incorrect
    
    if ( active_points == NULL || active_point_count == 0 )
        return false;

    result -> projection_geometry_ready = false;

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
            process_network_stream();

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
        if ( i > 0 && i % 1024 == 0 )
            process_network_stream();

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

    // Purpose: It resolves the optimal spatial source predicated on the global "OFFLOAD_MODE" configuration & metadata validity

    if ( OFFLOAD_MODE ) {
        if ( geometry_from_sff1( fb, active_points, active_point_count, result, timer_hz ) )
            return true;
    }

    return compute_geometry_locally( active_points, active_point_count, result, timer_hz );
}

static inline bool dispatch_temporal_control( uint32_t source_frame_id, uint16_t requested_skip ) {

    // Purpose: It originates the plain 16-byte message consumed by SFF2, deliberately omitting "NSH" encapsulation

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
    rte_ether_addr_copy( &sff2_encoder_mac, &eth -> dst_addr );
    eth -> ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    uint16_t udp_length = sizeof( struct rte_udp_hdr ) + sizeof( struct temporal_payload );

    ipv4 -> version_ihl = 0x45;
    ipv4 -> time_to_live = 64;
    ipv4 -> next_proto_id = IPPROTO_UDP;
    ipv4 -> src_addr = rte_cpu_to_be_32( ENCODER_IP );
    ipv4 -> dst_addr = rte_cpu_to_be_32( SFF2_ENCODER_IP );
    ipv4 -> total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + udp_length );

    udp -> src_port = rte_cpu_to_be_16( ENCODER_PORT );
    udp -> dst_port = rte_cpu_to_be_16( SFF2_ENCODER_PORT );
    udp -> dgram_len = rte_cpu_to_be_16( udp_length );
    udp -> dgram_cksum = 0;

    temporal -> frame_id = rte_cpu_to_be_32( source_frame_id );
    temporal -> skip = rte_cpu_to_be_16( requested_skip );
    temporal -> padding = 0;

    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );

    temporal -> timestamp = rte_cpu_to_be_64( rte_get_timer_cycles() );

    struct rte_mbuf *control_bufs[ 1 ] = { m };
    int control_burst_idx = 1;

    return flush_tx_burst( control_bufs, &control_burst_idx, NULL, NULL, NULL, NULL, NULL );
}

static inline void update_workload_controller( uint32_t frame_id, uint16_t active_skip, double service_ms, double raw_queue_ms, uint32_t frame_backlog, uint32_t codec_backlog, struct telemetry_csv *telemetry ) {

    // Purpose: It applies one-level temporal adaptation following the establishment of steady-state conditions within the raw data path
    //          Samples remain observable in telemetry but are consistently discarded from the decision "EWMA" until the controller becomes armed

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
        bool startup_stable = frame_backlog == 0 && raw_queue_ms <= active_budget_ms * RECOVERY_FRACTION;

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
            if ( dispatch_temporal_control( frame_id, workload_controller.requested_skip ) ) {
                workload_controller.last_control_observation = workload_controller.observations;
                snprintf( telemetry -> event, sizeof( telemetry -> event ), "RETRY" );
            }
        }

        return;
    }

    bool codec_growing = false;
    bool codec_not_increasing = true;

    if ( workload_controller.codec_backlog_initialized ) {
        codec_growing = codec_backlog > workload_controller.previous_codec_backlog;
        codec_not_increasing = codec_backlog <= workload_controller.previous_codec_backlog;
    }

    workload_controller.previous_codec_backlog = codec_backlog;
    workload_controller.codec_backlog_initialized = true;

    bool overloaded = workload_ratio >= OVERLOAD_RATIO || raw_queue_ms > active_budget_ms * OVERLOAD_FRACTION || frame_backlog >= 2 || codec_growing;

    double lower_budget_ms = ( active_skip > 1 ) ? base_budget_ms * ( active_skip - 1 ) : 0.0;
    double lower_ratio = ( lower_budget_ms > 0.0 ) ? workload_controller.ewma_ms / lower_budget_ms : 1.0;
    
    bool recoverable = active_skip > 1 && lower_ratio <= RECOVERY_RATIO && raw_queue_ms <= active_budget_ms * RECOVERY_FRACTION && frame_backlog == 0 && codec_not_increasing;

    workload_controller.overload_streak = overloaded ? workload_controller.overload_streak + 1 : 0;
    workload_controller.recovery_streak = recoverable ? workload_controller.recovery_streak + 1 : 0;

    if ( workload_controller.overload_streak >= OVERLOAD_STREAK && active_skip < MAX_SKIP ) {
        uint16_t requested_skip = active_skip + 1;

        if ( requested_skip > MAX_SKIP )
            requested_skip = MAX_SKIP;

        if ( dispatch_temporal_control( frame_id, requested_skip ) ) {
            workload_controller.requested_skip = requested_skip;
            workload_controller.last_control_observation = workload_controller.observations;
            workload_controller.overload_streak = 0;
            workload_controller.recovery_streak = 0;
            workload_controller.codec_backlog_initialized = false;

            snprintf( telemetry -> event, sizeof( telemetry -> event ), "SKIP+1" );
            printf( "[SYSTEM] Workload controller increased skip at frame %u: %u -> %u ( \"Backlog\": %u, \"EWMA\": %.2f ms, Ratio: %.2f, Wait: %.2f ms ).\n", frame_id, active_skip, requested_skip, frame_backlog, workload_controller.ewma_ms, workload_ratio, raw_queue_ms );
            controller_notification_printed = true;
        }

        return;
    }

    if ( workload_controller.recovery_streak >= RECOVERY_STREAK && active_skip > 1 ) {
        uint16_t requested_skip = active_skip - 1;

        if ( dispatch_temporal_control( frame_id, requested_skip ) ) {
            workload_controller.requested_skip = requested_skip;
            workload_controller.last_control_observation = workload_controller.observations;
            workload_controller.overload_streak = 0;
            workload_controller.recovery_streak = 0;
            workload_controller.codec_backlog_initialized = false;

            snprintf( telemetry -> event, sizeof( telemetry -> event ), "SKIP-1" );
            printf( "[SYSTEM] Workload controller decreased skip at frame %u: %u -> %u ( \"Backlog\": %u, \"EWMA\": %.2f ms, Ratio: %.2f, Wait: %.2f ms ).\n", frame_id, active_skip, requested_skip, frame_backlog, workload_controller.ewma_ms, lower_ratio, raw_queue_ms );
            controller_notification_printed = true;
        }
    }
}

static inline uint32_t writer_pending_frames() {

    // Purpose: It retrieves the number of frames currently awaiting "FFmpeg" serialization

    pthread_mutex_lock( &writer_mutex );

    uint32_t pending = writer_count + ( writer_active ? 1 : 0 );

    pthread_mutex_unlock( &writer_mutex );

    return pending;
}

static inline int acquire_yuv_slot( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz, double *slot_wait_ms ) {

    // Purpose: It acquires a preallocated "I420" memory slot, maintaining network service elaboration whenever all spots are occupied.
    //          During contention, raw input & currently available encoded output are actively drained before the wait condition is re-evaluated

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

        process_network_stream();
        drain_codec_output( tx_bufs, burst_idx, timer_hz );
        rte_pause();
    }
}

static inline bool enqueue_yuv_job( const struct yuv_job &job ) {

    // Purpose: It registers a new projection job into the "FFmpeg" writer queue & signals the corresponding thread

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
    
    // Purpose: It publishes resulting "I420" frames into "FFmpeg" via a dedicated blocking "pipe", mitigating backpressure on the primary "DPDK" "worker" 

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
            frame_start_cycles[ idx ].store( write_start, std::memory_order_release );
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
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"NVENC\" pre-roll write failed...\n" );

            t -> tx_complete = 0;
            snprintf( t -> event, sizeof( t -> event ), "INVALID" );
            break;
        }

        uint64_t write_end = rte_get_timer_cycles();

        if ( !preroll ) {
            if ( written_total == TOTAL_YUV_SIZE ) {
                struct encode_service_sample service_sample;

                service_sample.frame_id = job.frame_id;
                service_sample.start_cycles = write_start;

                pthread_mutex_lock( &encode_service_mutex );
                encode_service_queue.push( service_sample );
                pthread_mutex_unlock( &encode_service_mutex );
            }
            
            t -> codec_write_ms = ( ( double )( write_end - write_start ) / timer_hz ) * 1000.0;
            t -> codec_exit_time = ( double )write_end / timer_hz;
            t -> render_queue_ms = ( write_start >= job.projection_end_cycles ) ? ( ( double )( write_start - job.projection_end_cycles ) / timer_hz ) * 1000.0 : 0.0;

            t -> active_process_ms = t -> conversion_ms + t -> geometry_aggregation_ms + t -> max_r_ms + t -> projection_ms + t -> codec_write_ms;

            t -> reference_process_ms = t -> conversion_ms + t -> projection_ms + t -> codec_write_ms;
            t -> reference_residency_ms = ( frame_arrival_cycles[ idx ] > 0 && write_end >= frame_arrival_cycles[ idx ] ) ? ( ( double )( write_end - frame_arrival_cycles[ idx ] ) / timer_hz ) * 1000.0 : 0.0;
            t -> reference_efficiency_pct = ( t -> reference_residency_ms > 0.0 ) ? ( t -> reference_process_ms / t -> reference_residency_ms ) * 100.0 : 0.0;

            if ( written_total == TOTAL_YUV_SIZE )
                write_reference( job.frame_id, yuv_buffers[ job.slot ].data() );

            debug_capture_frame( job.frame_id, yuv_buffers[ job.slot ].data() );
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

    // Purpose: It initiates the threaded execution for publishing uncompressed shots into the pipeline

    if ( writer_started )
        return;

    writer_stop_requested = false;

    int retval = pthread_create( &writer_thread, NULL, ffmpeg_writer_loop, NULL );

    if ( retval != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"FFmpeg\" thread creation failed...\n" );

    writer_started = true;
}

static void ffmpeg_preroll() {

    // Purpose: It activates the persistent "FFmpeg" session with an initial private blank "I420" "Group of Pictures" ( "GOP" ) prior to commencing sequence evaluation.
    //          Encoded pre-roll components are relayed with the reserved "FRAME_ID" so the Decoder can warm the persistent "NVDEC" session without populating application telemetry

    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    int burst_idx = 0;
    uint64_t timer_hz = rte_get_timer_hz();

    const uint64_t frame_period_cycles = ( uint64_t )( ( double )timer_hz / TARGET_FPS );

    uint64_t next_frame_cycles = rte_get_timer_cycles();

    uint32_t preroll_inputs = 0;
    bool decoder_ready_seen = false;

    ffmpeg_preroll_outputs = 0;

    printf( "[SYSTEM] Pre-rolling \"NVENC\" components with %d frames...\n", FRAMES );

    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
        memset( yuv_buffers[ slot ].data(), 16, SIZE_Y );
        memset( yuv_buffers[ slot ].data() + SIZE_Y, 128, 2 * SIZE_UV );
    }

    while ( 1 ) {
        if ( !decoder_ready_seen && access( PREROLL_PATH, F_OK ) == 0 )
            decoder_ready_seen = true;

        if ( decoder_ready_seen && preroll_inputs > 0 && ( preroll_inputs % FRAMES ) == 0 )
            break;

        while ( rte_get_timer_cycles() < next_frame_cycles ) {
            drain_codec_output( tx_bufs, &burst_idx, timer_hz );

            if ( !decoder_ready_seen && access( PREROLL_PATH, F_OK ) == 0 )
                decoder_ready_seen = true;

            if ( decoder_ready_seen && preroll_inputs > 0 && ( preroll_inputs % FRAMES ) == 0 )
                break;

            rte_pause();
        }

        if ( decoder_ready_seen && preroll_inputs > 0 && ( preroll_inputs % FRAMES ) == 0 )
            break;

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
                drain_codec_output( tx_bufs, &burst_idx, timer_hz );
                
                if ( !decoder_ready_seen && access( PREROLL_PATH, F_OK ) == 0 )
                    decoder_ready_seen = true;

                if ( decoder_ready_seen && preroll_inputs > 0 && ( preroll_inputs % FRAMES ) == 0 )
                    break;

                rte_pause();
            }
        }

        if ( decoder_ready_seen && preroll_inputs > 0 && ( preroll_inputs % FRAMES ) == 0 ) {
            if ( yuv_slot >= 0 ) {
                pthread_mutex_lock( &writer_mutex );
                yuv_slot_free[ yuv_slot ] = true;
                pthread_cond_broadcast( &writer_slot_released );
                pthread_mutex_unlock( &writer_mutex );
            }

            break;
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

            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to insert \"NVENC\" pre-roll frame...\n" );
        }

        preroll_inputs++;
        drain_codec_output( tx_bufs, &burst_idx, timer_hz );

        uint64_t now = rte_get_timer_cycles();
        next_frame_cycles += frame_period_cycles;

        if ( next_frame_cycles < now )
            next_frame_cycles = now + frame_period_cycles;
    }

    while ( writer_pending_frames() > 0 ) {
        drain_codec_output( tx_bufs, &burst_idx, timer_hz );
        rte_pause();
    }

    drain_codec_output( tx_bufs, &burst_idx, timer_hz );
}

static void ffmpeg_postroll( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {
    
    // Purpose: It flushes pending components across the chain signaling the culmination of mapping
    
    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
        memset( yuv_buffers[ slot ].data(), 16, SIZE_Y );
        memset( yuv_buffers[ slot ].data() + SIZE_Y, 128, 2 * SIZE_UV );
    }

    uint32_t postroll_frames = 0;

    const uint64_t frame_period_cycles = ( uint64_t )( ( double )timer_hz / TARGET_FPS );

    uint64_t next_frame_cycles = rte_get_timer_cycles();

    while ( access( POSTROLL_PATH, F_OK ) != 0 && postroll_frames < FRAMES) {
        int yuv_slot = acquire_yuv_slot( tx_bufs, burst_idx, timer_hz, NULL );

        struct yuv_job job;

        job.frame_id = FRAME_ID;
        job.slot = ( uint8_t )yuv_slot;
        job.preroll = true;

        if ( unlikely( !enqueue_yuv_job( job ) ) ) {
            pthread_mutex_lock( &writer_mutex );
            yuv_slot_free[ yuv_slot ] = true;
            pthread_cond_broadcast( &writer_slot_released );
            pthread_mutex_unlock( &writer_mutex );

            break;
        }

        postroll_frames++;

        next_frame_cycles += frame_period_cycles;

        while ( rte_get_timer_cycles() < next_frame_cycles ) {
            drain_codec_output( tx_bufs, burst_idx, timer_hz );

            if ( access( POSTROLL_PATH, F_OK ) == 0 )
                break;

            rte_pause();
        }
    }
}

static void ffmpeg_writer_stop() {

    // Purpose: It terminates the dedicated "FFmpeg" writer thread, ensuring robust completion of all residual tasks

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
    
    // Purpose: It halts execution until all elements are completely processed, vigorously draining the network & "FFmpeg" outcomes
    
    while ( writer_pending_frames() > 0 ) {
        process_network_stream();
        drain_codec_output( tx_bufs, burst_idx, timer_hz );
        rte_pause();
    }
}

static inline bool flush_tx_burst( struct rte_mbuf **tx_bufs, int *burst_idx, uint32_t *tx_packets, uint32_t *tx_zero_accepts, uint32_t *tx_partial_accepts, uint32_t *tx_resubmit_calls, uint32_t *tx_resubmitted_packets, uint64_t *last_egress_cycles, uint64_t *active_tx_cycles ) {
    if ( *burst_idx == 0 )
        return true;

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

        uint64_t tx_start_cycles = rte_get_timer_cycles();

        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF2, 0, &tx_bufs[ sent ], requested_packets );

        uint64_t tx_end_cycles = rte_get_timer_cycles();

        if ( active_tx_cycles != NULL )
            *active_tx_cycles += tx_end_cycles - tx_start_cycles;

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

                *burst_idx = 0;

                return false;
            }

            uint16_t pause_count = ( retries < pause_window ) ? retries : pause_window;

            for ( uint16_t p = 0; p < pause_count; p++ )
                rte_pause();
        }
    }

    *burst_idx = 0;

    return true;
}

static inline bool begin_mpeg_frame( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz, uint64_t output_cycles ) {

    // Purpose: It associates a newly detected video "PES" boundary with the oldest frame submitted to "FFmpeg"

    if ( current_frame_id > 0 && *burst_idx > 0 ) {
        if ( is_preroll_frame( current_frame_id ) )
            flush_tx_burst( tx_bufs, burst_idx, NULL, NULL, NULL, NULL, NULL, NULL );
        else {
            uint32_t old_idx = ( current_frame_id - 1 ) % K_FRAMES;

            flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ old_idx ].tx_packets, &telemetry_log[ old_idx ].tx_zero_accepts, &telemetry_log[ old_idx ].tx_partial_accepts, &telemetry_log[ old_idx ].tx_resubmit_calls, &telemetry_log[ old_idx ].tx_resubmitted_packets, &frame_egress_cycles[ old_idx ], &frame_active_tx_cycles[ old_idx ] );
        }
    }

    uint32_t next_frame_id = 0;

    pthread_mutex_lock( &writer_mutex );

    if ( !mpeg_frame_queue.empty() ) {
        next_frame_id = mpeg_frame_queue.front();
        mpeg_frame_queue.pop();
    }

    pthread_mutex_unlock( &writer_mutex );

    if ( next_frame_id == 0 )
        return false;

    current_frame_id = next_frame_id;
    current_mpeg_packet = 0;

    if ( is_preroll_frame( current_frame_id ) ) {
        ffmpeg_preroll_outputs++;
        return true;
    }

    uint32_t idx = ( current_frame_id - 1 ) % K_FRAMES;
    uint64_t encode_start = frame_start_cycles[ idx ].load( std::memory_order_acquire );

    if ( encode_start > 0 && output_cycles >= encode_start && telemetry_log[ idx ].encode_h265_ms == 0.0 )
        telemetry_log[ idx ].encode_h265_ms = ( ( double )( output_cycles - encode_start ) / timer_hz ) * 1000.0;

    return true;
}

static inline void emit_mpeg_payload( const uint8_t *mpeg_data, uint16_t mpeg_len, struct rte_mbuf **tx_bufs, int *burst_idx ) {

    // Purpose: It transmits a restricted "UDP" "MPEG-TS" media chunk to SFF2, affixing reconstruction details without exposing service-chain state

    if ( current_frame_id == 0 || mpeg_len == 0 )
        return;

    bool preroll = is_preroll_frame( current_frame_id );
    uint32_t idx = 0;

    if ( !preroll ) {
        idx = ( current_frame_id - 1 ) % K_FRAMES;
        telemetry_log[ idx ].mpeg_bytes_generated += mpeg_len;
        write_stream( mpeg_data, mpeg_len );
    }

    struct rte_mbuf *m_out = rte_pktmbuf_alloc( mbuf_pool );

    if ( m_out == NULL ) {
        if ( !preroll )
            telemetry_log[ idx ].mbuf_starvation++;

        return;
    }

    size_t headers_len = outer_len + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr );
    char *data = ( char * )rte_pktmbuf_append( m_out, headers_len + mpeg_len );

    if ( data == NULL ) {
        rte_pktmbuf_free( m_out );

        if ( !preroll )
            telemetry_log[ idx ].mbuf_starvation++;

        return;
    }

    struct rte_ether_hdr *eth_out = ( struct rte_ether_hdr * )data;
    struct rte_ipv4_hdr *ipv4_out = ( struct rte_ipv4_hdr * )( eth_out + 1 );
    struct rte_udp_hdr *udp_out = ( struct rte_udp_hdr * )( ipv4_out + 1 );

    uint16_t udp_payload_len = sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) + mpeg_len;

    main_header_init( eth_out, ipv4_out, udp_out, udp_payload_len );

    uint8_t *payload_ptr = ( uint8_t * )( udp_out + 1 );

    struct cam_hdr out_cam = { 0 };
    struct enc_hdr out_enc = { 0 };

    if ( preroll ) {
        out_cam.frame_id = rte_cpu_to_be_32( FRAME_ID );
        out_cam.temporal_skip = rte_cpu_to_be_16( 1 );
        out_cam.zoom = float_to_be( 1.0f );

        out_enc.frame_id = rte_cpu_to_be_32( FRAME_ID );
    }
    else {
        out_cam = camera_metadata[ idx ];
        out_enc = encoder_metadata[ idx ];
    }

    out_enc.packet_id = rte_cpu_to_be_32( current_mpeg_packet++ );

    rte_memcpy( payload_ptr, &out_cam, sizeof( struct cam_hdr ) );
    payload_ptr += sizeof( struct cam_hdr );

    rte_memcpy( payload_ptr, &out_enc, sizeof( struct enc_hdr ) );
    payload_ptr += sizeof( struct enc_hdr );

    rte_memcpy( payload_ptr, mpeg_data, mpeg_len );

    if ( !preroll )
        frame_created_packets[ idx ]++;

    tx_bufs[ ( *burst_idx )++ ] = m_out;

    if ( *burst_idx == BURST_SIZE ) {
        if ( preroll )
            flush_tx_burst( tx_bufs, burst_idx, NULL, NULL, NULL, NULL, NULL, NULL );
        else
            flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ idx ].tx_packets, &telemetry_log[ idx ].tx_zero_accepts, &telemetry_log[ idx ].tx_partial_accepts, &telemetry_log[ idx ].tx_resubmit_calls, &telemetry_log[ idx ].tx_resubmitted_packets, &frame_egress_cycles[ idx ], &frame_active_tx_cycles[ idx ] );
    }
}

static inline void consume_mpeg_chunk_prefix( size_t bytes ) {
    if ( bytes == 0 )
        return;

    if ( bytes >= mpeg_chunk.size() ) {
        mpeg_chunk.clear();
        mpeg_chunk_ts_offset = 0;
        return;
    }

    if ( bytes <= mpeg_chunk_ts_offset )
        mpeg_chunk_ts_offset -= bytes;
    else {
        size_t displaced = bytes - mpeg_chunk_ts_offset;
        size_t remainder = displaced % TS_PACKET_SIZE;
        mpeg_chunk_ts_offset = ( remainder == 0 ) ? 0 : TS_PACKET_SIZE - remainder;
    }

    mpeg_chunk.erase( mpeg_chunk.begin(), mpeg_chunk.begin() + bytes );
}

static inline void emit_ready_mpeg_payloads( struct rte_mbuf **tx_bufs, int *burst_idx, bool flush_all ) {
    while ( mpeg_chunk.size() >= MEDIA_PAYLOAD_SIZE ) {
        emit_mpeg_payload( mpeg_chunk.data(), MEDIA_PAYLOAD_SIZE, tx_bufs, burst_idx );
        consume_mpeg_chunk_prefix( MEDIA_PAYLOAD_SIZE );
    }

    if ( flush_all && !mpeg_chunk.empty() ) {
        emit_mpeg_payload( mpeg_chunk.data(), ( uint16_t )mpeg_chunk.size(), tx_bufs, burst_idx );
        mpeg_chunk.clear();
        mpeg_chunk_ts_offset = 0;
    }
}

static inline void process_mpeg_bytes( const uint8_t *data, size_t data_len, struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {

    // Purpose: It reconstructs complete 188-byte "MPEG-TS" units from arbitrary "pipe" reads, detecting video-"PES" frontiers & emitting appropriately limited conveyance groups

    ts_pending.insert( ts_pending.end(), data, data + data_len );

    size_t consumed = 0;

    while ( ts_pending.size() - consumed >= TS_PACKET_SIZE ) {
        const uint8_t *ts = ts_pending.data() + consumed;

        if ( ts[ 0 ] != 0x47 ) {
            consumed++;
            continue;
        }

        if ( ts_starts_video_pes( ts ) ) {
            uint64_t output_cycles = rte_get_timer_cycles();
            uint16_t video_pid = mpeg_ts_pid( ts );

            if ( mpeg_video_pid == 0xFFFF )
                mpeg_video_pid = video_pid;

            uint32_t queued_frame_id = 0;

            pthread_mutex_lock( &writer_mutex );

            if ( !mpeg_frame_queue.empty() )
                queued_frame_id = mpeg_frame_queue.front();

            pthread_mutex_unlock( &writer_mutex );

            bool entering_real_stream = is_preroll_frame( current_frame_id ) && queued_frame_id != 0 && !is_preroll_frame( queued_frame_id );
            std::vector< uint8_t > transition_prefix;

            if ( current_frame_id > 0 && !mpeg_chunk.empty() ) {
                if ( entering_real_stream && mpeg_video_pid != 0xFFFF ) {
                    for ( size_t offset = mpeg_chunk_ts_offset; offset + TS_PACKET_SIZE <= mpeg_chunk.size(); offset += TS_PACKET_SIZE ) {
                        const uint8_t *pending_ts = mpeg_chunk.data() + offset;

                        if ( pending_ts[ 0 ] == 0x47 && mpeg_ts_pid( pending_ts ) != mpeg_video_pid )
                            transition_prefix.insert( transition_prefix.end(), pending_ts, pending_ts + TS_PACKET_SIZE );
                    }
                }

                emit_ready_mpeg_payloads( tx_bufs, burst_idx, true );
            }

            if ( !begin_mpeg_frame( tx_bufs, burst_idx, timer_hz, output_cycles ) )
                break;

            if ( entering_real_stream ) {
                for ( size_t offset = 0; offset + TS_PACKET_SIZE <= transition_prefix.size(); offset += TS_PACKET_SIZE ) {
                    mpeg_chunk.insert( mpeg_chunk.end(), transition_prefix.data() + offset, transition_prefix.data() + offset + TS_PACKET_SIZE );
                    emit_ready_mpeg_payloads( tx_bufs, burst_idx, false );
                }
            }
        }

        mpeg_chunk.insert( mpeg_chunk.end(), ts, ts + TS_PACKET_SIZE );
        consumed += TS_PACKET_SIZE;

        if ( current_frame_id > 0 )
            emit_ready_mpeg_payloads( tx_bufs, burst_idx, false );
    }

    if ( consumed > 0 )
        ts_pending.erase( ts_pending.begin(), ts_pending.begin() + consumed );
}

static inline void drain_codec_output( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {

    // Purpose: It asynchronously drains hardware-encoded "MPEG-TS" bytes, associating the first result with its source frame without imposing blockage constraints

    while ( 1 ) {
        uint8_t read_buffer[ FFMPEG_READ_SIZE ];

        int bytes_read = read( ffmpeg_out[ 0 ], read_buffer, FFMPEG_READ_SIZE );

        if ( bytes_read > 0 ) {
            uint64_t service_end = rte_get_timer_cycles();

            struct encode_service_sample service_sample;

            bool service_sample_available = false;

            pthread_mutex_lock( &encode_service_mutex );

            if ( encode_service_queue.empty() )
                encode_service_bytes = 0;
            else {
                encode_service_bytes += ( size_t )bytes_read;

                if ( encode_service_bytes >= FFMPEG_READ_SIZE ) {

                    service_sample = encode_service_queue.front();
                    encode_service_queue.pop();
                    encode_service_bytes -= ( size_t )FFMPEG_READ_SIZE;
                    service_sample_available = true;
                }
            }

            pthread_mutex_unlock( &encode_service_mutex );

            if ( service_sample_available && service_sample.frame_id > 0 && service_sample.frame_id <= K_FRAMES && service_end >= service_sample.start_cycles ) {
                uint32_t service_idx = ( service_sample.frame_id - 1 ) % K_FRAMES;

                if ( telemetry_log[ service_idx ].encode_service_ms == 0.0 )
                    telemetry_log[ service_idx ].encode_service_ms = ( ( double )( service_end - service_sample. start_cycles ) / timer_hz ) * 1000.0;
            }

            process_mpeg_bytes( read_buffer, bytes_read, tx_bufs, burst_idx, timer_hz );
        }
        else if ( bytes_read < 0 && ( errno == EAGAIN || errno == EWOULDBLOCK ) ) {
            rte_pause();
            break;
        }
        else
            break;
    }

    if ( *burst_idx > 0 && current_frame_id > 0 ) {
        if ( is_preroll_frame( current_frame_id ) )
            flush_tx_burst( tx_bufs, burst_idx, NULL, NULL, NULL, NULL, NULL, NULL );
        else {
            uint32_t idx = ( current_frame_id - 1 ) % K_FRAMES;

            flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ idx ].tx_packets, &telemetry_log[ idx ].tx_zero_accepts, &telemetry_log[ idx ].tx_partial_accepts, &telemetry_log[ idx ].tx_resubmit_calls, &telemetry_log[ idx ].tx_resubmitted_packets, &frame_egress_cycles[ idx ], &frame_active_tx_cycles[ idx ] );
        }
    }
}

static inline void process_network_stream() {

    // Purpose: It drains the SFF2-facing standard "UDP" receive queue, situating valid point packets at deterministic offsets derived from "sequence_number" parameters.
    //          Geometry metadata is furnished by SFF2 serving as application-side context, while "NSH" elaboration remains isolated outside the Encoder domain

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

            if ( unlikely( !rte_is_same_ether_addr( &eth -> src_addr, &sff2_encoder_mac ) || !rte_is_same_ether_addr( &eth -> dst_addr, &encoder_mac ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

            if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( SFF2_ENCODER_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( ENCODER_IP ) ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

            if ( unlikely( udp -> src_port != rte_cpu_to_be_16( SFF2_ENCODER_PORT ) || udp -> dst_port != rte_cpu_to_be_16( ENCODER_PORT ) ) ) {
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
            uint16_t points_per_packet = rte_be_to_cpu_16( cam -> padding );

            if ( points_per_packet == 0 )
                points_per_packet = POINTS_PER_PACKET;

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

            if ( unlikely( frame_id == 0 || frame_id > K_FRAMES || frame_id <= last_frame_id || original_points == 0 || original_points > MAX_POINTS || points_per_packet == 0 || points_in_packet == 0 || points_in_packet > points_per_packet ) ) {
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

            if ( frame_id > highest_frame_id )
                highest_frame_id = frame_id;

            frame_buffer &fb = frame_buffers[ frame_id ];

            if ( fb.rx_packets == 0 ) {
                fb.cam = *cam;
                fb.original_points = original_points;
                fb.points_per_packet = points_per_packet;
                fb.expected_packets = ( original_points + fb.points_per_packet - 1 ) / fb.points_per_packet;
                fb.camera_tx = rte_be_to_cpu_64( cam -> timestamp );
                fb.first_arrival = packet_arrival;
                fb.points.reset( new struct host_point[ original_points ] );
                fb.packet_received.assign( fb.expected_packets, 0 );
            }
            else if ( unlikely( fb.original_points != original_points || fb.points_per_packet != points_per_packet || fb.cam.temporal_skip != cam -> temporal_skip ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t sequence_number = rte_be_to_cpu_32( cam -> sequence_number );

            if ( unlikely( sequence_number >= fb.expected_packets ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t point_offset = sequence_number * fb.points_per_packet;
            uint32_t expected_points = std::min( ( uint32_t )fb.points_per_packet, fb.original_points - point_offset );

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

                point.x = be_to_float( points[ point_id ].x );
                point.y = be_to_float( points[ point_id ].y );
                point.z = be_to_float( points[ point_id ].z );
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
    uint64_t prev_arrival_cyc = 0;
    uint32_t prev_arrival_f_id = 0;
    uint32_t first_arrival_f_id = 0;

    double jitter_ms = 0.0;

    printf( "\n[SYSTEM] Conversion is about to begin at %.1f FPS...\n\n", TARGET_FPS );

    FILE *ready = fopen( READY_PATH, "w" );

    if ( ready == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to to open file \"%s\"...\n", READY_PATH );

    fclose( ready );

    while ( 1 ) {
        if ( csv_written ) {
            rte_delay_us_sleep( 1000 );
            continue;
        }

        process_network_stream();
        drain_codec_output( tx_bufs, &burst_idx, timer_hz );

        uint32_t frame_to_process = 0;

        if ( !frame_buffers.empty() ) {
            auto oldest = frame_buffers.begin();
            frame_buffer &candidate = oldest -> second;

            bool candidate_complete = candidate.original_points > 0 && candidate.received_points == candidate.original_points;
            bool candidate_closed = eos_received || highest_frame_id > oldest -> first;

            if ( candidate_complete || candidate_closed )
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

            t -> mse_y = std::nan( "" );
            t -> psnr_y = std::nan( "" );
            t -> ssim_y = std::nan( "" );

            frame_arrival_cycles[ idx ] = fb.first_arrival;
            frame_ready_cycles[ idx ] = ( fb.frame_ready > 0 ) ? fb.frame_ready : fb.last_arrival;
            frame_egress_cycles[ idx ] = 0;
            frame_active_tx_cycles[ idx ] = 0;
            frame_created_packets[ idx ] = 0;

            t -> frame_id = frame_to_process;
            t -> rx_complete = ( frame_complete && fb.rx_packets == fb.expected_packets ) ? 1 : 0;
            t -> tx_complete = 0;
            t -> current_skip = active_temporal_skip;

            snprintf( t -> event, sizeof( t -> event ), "IDLE" );

            t -> yaw = 0.0f;
            t -> pitch = 0.0f;
            t -> zoom = 1.0f;
            t -> camera_send_timestamp = ( double )fb.camera_tx / timer_hz;
            t -> recv_start_timestamp = ( double )fb.first_arrival / timer_hz;
            t -> original_points = fb.original_points;
            t -> rx_points = fb.received_points;
            t -> rx_packets = fb.rx_packets;
            t -> payload_bytes = fb.payload_bytes;
            t -> data_integrity_pct = ( fb.original_points > 0 ) ? ( ( double )fb.received_points / fb.original_points ) * 100.0 : 0.0;
            t -> conversion_ms = ( ( double )fb.conversion_cycles / timer_hz ) * 1000.0;
            t -> camera_node_ms = ( fb.first_arrival >= fb.camera_tx ) ? ( ( double )( fb.first_arrival - fb.camera_tx ) / timer_hz ) * 1000.0 : 0.0;

            if ( prev_arrival_cyc > 0 && frame_to_process > prev_arrival_f_id ) {
                double real_interval_sec = ( double )( fb.first_arrival - prev_arrival_cyc ) / timer_hz;
                double expected_interval_sec = ( double )( frame_to_process - prev_arrival_f_id ) / TARGET_FPS;

                double diff_sec = real_interval_sec - expected_interval_sec;

                t -> inter_arrival_ms = real_interval_sec * 1000.0;
                t -> instant_jitter_ms = std::abs( diff_sec ) * 1000.0;

                double reference_interval_sec = ( double )active_temporal_skip / TARGET_FPS;
                t -> reference_jitter_ms = std::abs( real_interval_sec - reference_interval_sec ) * 1000.0;

                jitter_ms += ( t -> instant_jitter_ms - jitter_ms ) / 16.0;
            }
            else {
                t -> inter_arrival_ms = 0.0;
                t -> instant_jitter_ms = 0.0;
                t -> reference_jitter_ms = 0.0;
            }

            t -> desynced_jitter_ms = jitter_ms;

            prev_arrival_cyc = fb.first_arrival;
            prev_arrival_f_id = frame_to_process;

            double receive_sec = ( fb.last_arrival >= fb.first_arrival ) ? ( double )( fb.last_arrival - fb.first_arrival ) / timer_hz : 0.0;
            uint64_t logical_rx_frame_bytes = ( uint64_t )fb.payload_bytes + ( fb.rx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );

            t -> internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_rx_frame_bytes / 1000000.0 ) / receive_sec : 0.0;

            t -> reference_size_bytes = fb.original_points * sizeof( struct point_tx ) + sizeof( struct cam_hdr );
            t -> reference_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )t -> reference_size_bytes / 1000000.0 ) / receive_sec : 0.0;
            t -> reference_bitrate_mbps = ( t -> reference_size_bytes * 8.0 * ( TARGET_FPS / active_temporal_skip ) ) / 1000000.0;

            std::vector< struct host_point > compact_points;
            const struct host_point *active_points = fb.points.get();
            uint32_t active_point_count = fb.received_points;

            if ( !frame_complete ) {
                compact_points.reserve( fb.received_points );

                for ( uint32_t sequence_number = 0; sequence_number < fb.expected_packets; sequence_number++ ) {
                    if ( fb.packet_received[ sequence_number ] == 0 )
                        continue;

                    uint32_t point_offset = sequence_number * fb.points_per_packet;
                    uint32_t point_count = std::min( ( uint32_t )fb.points_per_packet, fb.original_points - point_offset );

                    compact_points.insert( compact_points.end(), fb.points.get() + point_offset, fb.points.get() + point_offset + point_count );
                }

                active_points = compact_points.data();
                active_point_count = ( uint32_t )compact_points.size();
            }

            t -> processed_points = active_point_count;

            if ( frames_received == 0 ) {
                t_session_start = fb.first_arrival;
                first_arrival_f_id = frame_to_process;
            }

            uint32_t frame_offset = frame_to_process - first_arrival_f_id;
            uint64_t service_start = rte_get_timer_cycles();

            uint64_t ready_reference = fb.frame_ready > 0 ? fb.frame_ready : fb.last_arrival;

            t -> raw_queue_ms = ( ready_reference > 0 && service_start >= ready_reference ) ? ( ( double )( service_start - ready_reference ) / timer_hz ) * 1000.0 : 0.0;

            geometry_result geometry;

            if ( unlikely( !resolve_geometry( fb, active_points, active_point_count, &geometry, timer_hz ) ) ) {
                t -> tx_complete = 0;
                snprintf( t -> event, sizeof( t -> event ), "INVALID" );

                last_frame_id = frame_to_process;
                frame_buffers.erase( frame_to_process );
                continue;
            }

            t -> geometry_aggregation_ms = geometry.geometry_aggregation_ms;
            t -> max_r_ms = geometry.max_r_ms;

            float target_radius = CAMERA_DISTANCE * 0.2f;
            float final_scale = geometry.projection_geometry_ready ? geometry.final_scale : ( ( geometry.max_r > 0.0f ) ? target_radius / geometry.max_r : 1.0f );

            double slot_wait_ms = 0.0;
            int yuv_slot = acquire_yuv_slot( tx_bufs, &burst_idx, timer_hz, &slot_wait_ms );

            uint64_t t_projection_start = rte_get_timer_cycles();
            float global_scale = 1.0f;
            float bbox_center_x = 0.0f;
            float bbox_center_y = 0.0f;
            float bbox_center_z = 0.0f;
            uint64_t projection_end_cycles = 0;
            double gpu_metrics[ 4 ] = { 0.0, 0.0, 0.0, 0.0 };

            run_projection_pipeline( active_points, active_point_count, geometry.centroid_x, geometry.centroid_y, geometry.centroid_z, geometry.extent_x, geometry.extent_y, geometry.extent_z, geometry.bbox_center_x, geometry.bbox_center_y, geometry.bbox_center_z, final_scale, CAMERA_DISTANCE, geometry.projection_geometry_ready, geometry.global_scale, geometry.projected_bbox_x, geometry.projected_bbox_y, geometry.projected_bbox_z, yuv_buffers[ yuv_slot ].data(), gpu_metrics, &global_scale, &bbox_center_x, &bbox_center_y, &bbox_center_z, &projection_end_cycles, process_network_stream );

            t -> projection_ms = ( ( double )( projection_end_cycles - t_projection_start ) / timer_hz ) * 1000.0;
            t -> gpu_transfer_ms = gpu_metrics[ 0 ];
            t -> gpu_kernel_ms = gpu_metrics[ 1 ];
            t -> gpu_packing_ms = gpu_metrics[ 2 ];
            t -> gpu_copyback_ms = gpu_metrics[ 3 ];

            double measured_gpu_ms = gpu_metrics[ 0 ] + gpu_metrics[ 1 ] + gpu_metrics[ 2 ] + gpu_metrics[ 3 ];
            t -> host_overhead_ms = ( t -> projection_ms > measured_gpu_ms ) ? t -> projection_ms - measured_gpu_ms : 0.0;

            encoder_metadata[ idx ].frame_id = rte_cpu_to_be_32( frame_to_process );
            encoder_metadata[ idx ].final_scale = float_to_be( final_scale );
            encoder_metadata[ idx ].global_scale = float_to_be( global_scale );
            encoder_metadata[ idx ].box_center_x = float_to_be( bbox_center_x );
            encoder_metadata[ idx ].box_center_y = float_to_be( bbox_center_y );
            encoder_metadata[ idx ].box_center_z = float_to_be( bbox_center_z );
            encoder_metadata[ idx ].yaw = float_to_be( 0.0f );
            encoder_metadata[ idx ].pitch = float_to_be( 0.0f );
            encoder_metadata[ idx ].centroid_x = float_to_be( geometry.centroid_x );
            encoder_metadata[ idx ].centroid_y = float_to_be( geometry.centroid_y );
            encoder_metadata[ idx ].centroid_z = float_to_be( geometry.centroid_z );

            camera_metadata[ idx ] = fb.cam;
            camera_metadata[ idx ].yaw = float_to_be( 0.0f );
            camera_metadata[ idx ].pitch = float_to_be( 0.0f );
            camera_metadata[ idx ].zoom = float_to_be( 1.0f );
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

            if ( unlikely( !enqueue_yuv_job( job ) ) ) {
                pthread_mutex_lock( &writer_mutex );
                yuv_slot_free[ yuv_slot ] = true;
                pthread_cond_broadcast( &writer_slot_released );
                pthread_mutex_unlock( &writer_mutex );

                t -> tx_complete = 0;
                snprintf( t -> event, sizeof( t -> event ), "INVALID" );
            }

            uint64_t service_end = rte_get_timer_cycles();
            double workload_service_ms = ( ( double )( service_end - service_start ) / timer_hz ) * 1000.0;

            uint32_t frame_backlog = 0;

            for ( const auto &entry : frame_buffers ) {

                if ( entry.first == frame_to_process )
                    continue;

                const frame_buffer &candidate = entry.second;

                bool candidate_complete = candidate.original_points > 0 && candidate.received_points == candidate.original_points;

                if ( candidate_complete )
                    frame_backlog++;
            }

            uint32_t codec_backlog = 0;

            pthread_mutex_lock( &writer_mutex );

            uint32_t writer_backlog = writer_count + ( writer_active ? 1 : 0 );
 
            uint32_t mpeg_backlog = ( uint32_t )mpeg_frame_queue.size();

            codec_backlog = std::max( writer_backlog, mpeg_backlog );

            pthread_mutex_unlock( &writer_mutex );

            update_workload_controller( frame_to_process, active_temporal_skip, workload_service_ms, t -> raw_queue_ms, frame_backlog, codec_backlog, t );

            drain_codec_output( tx_bufs, &burst_idx, timer_hz );

            last_frame_id = frame_to_process;
            frame_buffers.erase( frame_to_process );
        }

        if ( eos_received && frame_buffers.empty() && !csv_written ) {
            wait_for_idle( tx_bufs, &burst_idx, timer_hz );
            
            ffmpeg_postroll( tx_bufs, &burst_idx, timer_hz );
            
            wait_for_idle( tx_bufs, &burst_idx, timer_hz );
            ffmpeg_writer_stop();
            close( ffmpeg_in[ 1 ] );

            int ffmpeg_flags = fcntl( ffmpeg_out[ 0 ], F_GETFL, 0 );

            if ( ffmpeg_flags >= 0 )
                fcntl( ffmpeg_out[ 0 ], F_SETFL, ffmpeg_flags & ~O_NONBLOCK );

            drain_codec_output( tx_bufs, &burst_idx, timer_hz );

            if ( current_frame_id > 0 && !mpeg_chunk.empty() )
                emit_ready_mpeg_payloads( tx_bufs, &burst_idx, true );

            ts_pending.clear();

            if ( burst_idx > 0 ) {
                if ( current_frame_id > 0 && !is_preroll_frame( current_frame_id ) ) {
                    uint32_t retry_idx = ( current_frame_id - 1 ) % K_FRAMES;

                    flush_tx_burst( tx_bufs, &burst_idx, &telemetry_log[ retry_idx ].tx_packets, &telemetry_log[ retry_idx ].tx_zero_accepts, &telemetry_log[ retry_idx ].tx_partial_accepts, &telemetry_log[ retry_idx ].tx_resubmit_calls, &telemetry_log[ retry_idx ].tx_resubmitted_packets, &frame_egress_cycles[ retry_idx ], &frame_active_tx_cycles[ retry_idx ] );
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

                    main_header_init( eth_out, ipv4_out, udp_out, udp_payload_len );

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

            if ( quality_capture_enabled ) {
                evaluate_quality_capture();
                telemetry_to_csv();
            }

            debug_dump_frame();

            csv_written = true;
            printf( "\n[SYSTEM] End of stream detected. Changing to \"idle\" state...\n" );
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It commences the Encoder service function, contiguous hardware "H.265" subprocesses, "CUDA" resources, & "FFmpeg" components before delegating data-path processing

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"Encoder\" microservice...\n\n" );

    uint32_t media_ipv4_len = sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) + MEDIA_PAYLOAD_SIZE;

    if ( media_ipv4_len > NETWORK_MTU )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Packetization exceeded \"MTU\" size ( %u > %u )...\n", media_ipv4_len, ( unsigned int )NETWORK_MTU );

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, MBUF_DATA_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF2, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF2-facing virtual port configuration failed...\n" );

    const char *quality_env = getenv( "QUALITY_CAPTURE" );

    if ( quality_env != NULL )
        quality_capture_enabled = strcmp( quality_env, "0" ) != 0;

    if ( quality_capture_enabled )
        quality_capture_init();

    if ( DEBUG_VISUALS == DEBUG_VISUALS_ENABLED )
        debug_snapshot.resize( TOTAL_YUV_SIZE );
    
    cuda_memory_init( MAX_POINTS );
    cuda_memory_warmup();

    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ ) {
        yuv_buffers[ slot ].resize( TOTAL_YUV_SIZE );
        cuda_memory_register( yuv_buffers[ slot ].data(), TOTAL_YUV_SIZE );
    }

    ffmpeg_init();

    for ( uint32_t i = 0; i < K_FRAMES; i++ )
        frame_start_cycles[ i ].store( 0, std::memory_order_relaxed );

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

    if ( quality_capture_enabled && !csv_written )
        quality_capture_close();

    for ( uint8_t slot = 0; slot < YUV_BUFFER_COUNT; slot++ )
        cuda_memory_unleash( yuv_buffers[ slot ].data() );

    cuda_memory_free();
    rte_eal_cleanup();

    return 0;
}  