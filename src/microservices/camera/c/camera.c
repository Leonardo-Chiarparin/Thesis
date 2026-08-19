#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_cycles.h>

// Runtime & experimental configuration variables
#define SEQUENCE_FOLDER "/shared/data/loot/bin"
#define TELEMETRY_FOLDER "/shared/log/camera"
#define TELEMETRY_PATH "/shared/log/camera/telemetry_camera.csv"

#define K_FRAMES 300

#define TARGET_FPS 30.0 

#define BURST_SIZE 32
#define MAX_ZERO_ACCEPTS 2048 // local zero-accept bound. For retry index "r", the backoff executes "min( r, BURST_SIZE / 2 )" pause instructions.
//                               Consequently, the total rest cycles equate to "N_pause( R ) = sum_{ r = 1 }^{ R } min( r, BURST_SIZE / 2 )". Wall-clock delay additionally incorporates each "rte_eth_tx_burst()" invocation

// Loading strategies for I / O sensitivity analysis 
#define CACHE_MODE_BEST 0 // application-resident dataset. Pre-transmission "malloc" + "fread" + "mlock"
#define CACHE_MODE_MIDDLE 1 // reusable staging allocation. Per-frame reads kept within the isochronous source path
#define CACHE_MODE_WORST  2 // no a-priori "malloc". Element-wise allocation & "fread", followed by post-streaming release ( "free" )

#define CACHE_MODE CACHE_MODE_MIDDLE

// File-backed page residency conditions
#define WARM_MODE_DISABLED 0 // standard buffered document I / O without explicit locking
#define WARM_MODE_ENABLED 1 // maps source components prior to the timed sequence, preserving natural read semantics 

#define WARM_MODE WARM_MODE_ENABLED

#define END_OF_STREAM 0xFFFFFFFF
#define WAITING_TIME 5000

// "DPDK" packet-buffer pool settings
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// Sending bonds & networking parameters
#define PORT_SFF1 0
#define CAMERA_IP RTE_IPV4( 10, 0, 0, 2 )
#define SFF1_CAMERA_IP RTE_IPV4( 10, 0, 1, 254 ) // Camera-facing endpoint of SFF1, functioning as the "Geometry-Aware Classifier" ( "GAC" )
#define CAMERA_PORT 49432
#define SFF1_CAMERA_PORT 5001

// Packetization & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80 // a 16-byte point representation yields 1280 payload bytes for 80 points
#define POINT_SIZE_BYTES 16 // Such organization ensures the complete datagram remains below the traditional 1500-byte "Eth" broadcast component

// Wire-format structures utilized by the "DPDK" data path
struct cam_hdr { // application header placed immediately after the "UDP" section. It conveys frame details, such as identity, sequence ordering, source timing & relevant downstream control metadata
    uint32_t frame_id;
    uint32_t sequence_number;
    uint64_t timestamp;
    uint32_t yaw; // 0.0f, static upstream pose
    uint32_t pitch; // 0.0f
    uint32_t zoom; // 1.0f
    uint16_t temporal_skip; // sampling factor active for the current element
    uint32_t original_points;
    uint32_t points_in_packet;
    uint16_t padding; // maintains a 40-byte strict alignment
} __attribute__((__packed__)); // suppresses compiler-inserted padding to ensure the in-memory representation pairs with the network protocol layout

struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__, __aligned__(2)));

struct main_hdr {
    struct net_hdr net;
    struct cam_hdr cam;
} __attribute__((__packed__, __aligned__(2)));

struct point_tx {
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t padding; // completes the fixed 16-byte point-record stride
} __attribute__((__packed__));

struct temporal_payload {
    uint32_t frame_id; // source frame associated with the control decision
    uint16_t skip; // requested temporal adjustment
    uint16_t padding; // guarantees a native 8-byte format
} __attribute__((__packed__));

// Frame & diagnostic content abstractions
struct frame_data {
    uint8_t *buffer;
    void *locked_mapping;
    size_t size;
    uint32_t point_count;
    char file_path[ 128 ];
};

struct telemetry_csv {
    uint32_t frame_id;
    uint8_t status;
    uint16_t current_skip;
    uint32_t last_control_frame;

    double timestamp_start_tx;

    uint32_t tx_points;
    uint32_t tx_packets;
    uint32_t payload_bytes;
    double internal_throughput_mbs;

    double logical_bitrate_mbps;
    double network_bitrate_mbps;

    double disk_io_ms;
    double serialization_ms;
    double tx_duration_ms;
    double active_tx_ms;
    double active_process_ms;
    double total_residency_ms;
    double node_efficiency_pct;
    
    uint32_t tx_zero_accepts; 
    uint32_t tx_partial_accepts;
    uint32_t tx_resubmit_calls;
    uint32_t tx_resubmitted_packets;

    uint32_t mbuf_starvation;
};

// Global application state
struct frame_data frames[ K_FRAMES ];
struct telemetry_csv telemetry_log[ K_FRAMES ];
int loaded_frames = 0;
bool csv_written = false;
bool temporal_notification_printed = false;

struct rte_mempool *mbuf_pool;
struct main_hdr template_hdr;

uint16_t current_temporal_skip = 1;
uint32_t last_control_frame = 0;

uint8_t *staging_buffer = NULL;
size_t staging_buffer_size = 0;

// Data path & support routines
static inline uint32_t float_to_be_32( float value ) {

    // Purpose: It reinterprets an "IEEE-754" single-precision float as a 32-bit integer, converting it to network byte order without altering the underlying bit pattern

    uint32_t bits;

    memcpy( &bits, &value, sizeof( bits ) );

    return rte_cpu_to_be_32( bits );
}

static inline void serialize_points( uint8_t *buffer, uint32_t point_count ) {

    // Purpose: It converts pre-loaded "XYZ" coordinates to network byte order, preserving "RGB" components & padding bytes unchanged

    struct point_tx *points = ( struct point_tx * )buffer;

    for ( uint32_t i = 0; i < point_count; i++ ) {
        float x, y, z;

        memcpy( &x, &points[ i ].x, sizeof( float ) );
        memcpy( &y, &points[ i ].y, sizeof( float ) );
        memcpy( &z, &points[ i ].z, sizeof( float ) );

        points[ i ].x = float_to_be_32( x );
        points[ i ].y = float_to_be_32( y );
        points[ i ].z = float_to_be_32( z );
    }
}

static void lock_persistent_pages() {

    // Purpose: It freezes source-file pages in memory prior to streaming, ensuring stable residency during warm-mode experiments to minimize latency variability

    printf( "[SYSTEM] Locking collection pages into \"RAM\"...\n\n" );

    for ( int i = 0; i < loaded_frames; i++ ) {
        int fd = open( frames[ i ].file_path, O_RDONLY );

        if ( fd < 0 )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to open file \"%s\" for page locking: %s\n", frames[ i ].file_path, strerror( errno ) );

        void *mapping = mmap( NULL, frames[ i ].size, PROT_READ, MAP_PRIVATE, fd, 0 );

        if ( mapping == MAP_FAILED ) {
            close( fd );

            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"mmap\" failed about \"%s\"...\n", frames[ i ].file_path );
        }

        close( fd );

        if ( mlock( mapping, frames[ i ].size ) != 0 ) {
            munmap( mapping, frames[ i ].size );

            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"mlock\" failed about \"%s\"...\n", frames[ i ].file_path );
        }

        frames[ i ].locked_mapping = mapping;
    }
}

static void unlock_persistent_pages() {

    // Purpose: It releases every mapping jam established via the initialization phase following transmission completion

    for ( int i = 0; i < loaded_frames; i++ ) {
        if ( frames[ i ].locked_mapping != NULL ) {
            munlock( frames[ i ].locked_mapping, frames[ i ].size );
            munmap( frames[ i ].locked_mapping, frames[ i ].size );

            frames[ i ].locked_mapping = NULL;
        }
    }
}

static inline int port_init( uint16_t port, struct rte_mempool *mbuf_pool ) {

    // Purpose: It configures the bidirectional SFF1-facing "DPDK" port, arranging one Rx / Tx queue couple sustained by the overall "mbuf" pool

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

static void header_init( struct main_hdr *hdr ) {

    // Purpose: It initializes the reusable Camera-to-SFF1 network template ( "Eth" / "IPv4" / "UDP" ) alongside static pose values & default temporal sampling factor

    memset( hdr, 0, sizeof( struct main_hdr ) );
    
    struct rte_ether_addr src_mac = { { 0x00, 0x00, 0x00, 0x00, 0x01, 0x01 } };
    struct rte_ether_addr dst_mac = { { 0x00, 0x00, 0x00, 0x00, 0x01, 0x02 } };
    
    rte_memcpy( &hdr -> net.ethernet.src_addr, &src_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> net.ethernet.dst_addr, &dst_mac, RTE_ETHER_ADDR_LEN );
    
    hdr -> net.ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> net.ipv4.version_ihl = 0x45;
    hdr -> net.ipv4.time_to_live = 64;
    hdr -> net.ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> net.ipv4.src_addr = rte_cpu_to_be_32( CAMERA_IP );
    hdr -> net.ipv4.dst_addr = rte_cpu_to_be_32( SFF1_CAMERA_IP );

    hdr -> net.udp.src_port = rte_cpu_to_be_16( CAMERA_PORT );
    hdr -> net.udp.dst_port = rte_cpu_to_be_16( SFF1_CAMERA_PORT );
    hdr -> net.udp.dgram_cksum = 0;

    hdr -> cam.yaw = float_to_be_32( 0.0f );
    hdr -> cam.pitch = float_to_be_32( 0.0f );
    hdr -> cam.zoom = float_to_be_32( 1.0f );
    hdr -> cam.temporal_skip = htons( 1 );
    hdr -> cam.padding = 0;
}

static void telemetry_to_csv() {

    // Purpose: It serializes buffered per-frame telemetry after sequence ended, thereby isolating file I / O operations from the timing-sensitive source data path

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

    fprintf( f, "frame_id;status;current_skip;last_control_frame;timestamp_start_tx;tx_points;tx_packets;payload_bytes;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;disk_io_ms;serialization_ms;tx_duration_ms;active_tx_ms;active_process_ms;total_residency_ms;node_efficiency_pct;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation\n" );

    for ( int i = 0; i < loaded_frames; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];
        fprintf( f, "%u;%u;%u;%u;%.6f;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u;%u;%u\n", t -> frame_id, t -> status, t -> current_skip, t -> last_control_frame, t -> timestamp_start_tx, t -> tx_points, t -> tx_packets, t -> payload_bytes, t -> internal_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> disk_io_ms, t -> serialization_ms, t -> tx_duration_ms, t -> active_tx_ms, t -> active_process_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets, t -> mbuf_starvation );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static inline void poll_temporal_control() {

    // Purpose: It validates plain 8-byte control datagrams received from SFF1 on the reverse primary flow link.
    //          The latest admissible request updates the temporal skip coefficient before the next frame selection, guaranteeing an invariant sampling state per transmitted component

    struct rte_mbuf *control_bufs[ BURST_SIZE ];

    uint16_t nb_rx = rte_eth_rx_burst( PORT_SFF1, 0, control_bufs, BURST_SIZE );

    for ( uint16_t i = 0; i < nb_rx; i++ ) {
        struct rte_mbuf *m = control_bufs[ i ];

        size_t min_packet_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct temporal_payload );

        if ( unlikely( rte_pktmbuf_pkt_len( m ) < min_packet_len ) ) {
            rte_pktmbuf_free( m );

            continue;
        }

        struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

        if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
            rte_pktmbuf_free( m );
            continue;
        }

        struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

        if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( SFF1_CAMERA_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( CAMERA_IP ) ) ) {
            rte_pktmbuf_free( m );
            continue;
        }

        struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

        if ( unlikely( udp -> src_port != rte_cpu_to_be_16( SFF1_CAMERA_PORT ) || udp -> dst_port != rte_cpu_to_be_16( CAMERA_PORT ) ) ) {
            rte_pktmbuf_free( m );
            continue;
        }

        uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
        uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );
        uint16_t expected_udp_length = sizeof( struct rte_udp_hdr ) + sizeof( struct temporal_payload );

        if ( unlikely( udp_length != expected_udp_length || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length ) ) {
            rte_pktmbuf_free( m );
            continue;
        }

        if ( unlikely( rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + ipv4_length ) ) {
            rte_pktmbuf_free( m );
            continue;
        }

        struct temporal_payload *temporal = ( struct temporal_payload * )( udp + 1 );

        uint32_t source_frame = rte_be_to_cpu_32( temporal -> frame_id );
        uint16_t requested_skip = rte_be_to_cpu_16( temporal -> skip );

        if ( requested_skip == 0 )
            requested_skip = 1;

        if ( source_frame != END_OF_STREAM && source_frame >= last_control_frame ) {
            last_control_frame = source_frame;

            if ( requested_skip != current_temporal_skip ) {
                uint16_t previous_skip = current_temporal_skip;
                current_temporal_skip = requested_skip;

                const char *event = ( current_temporal_skip > previous_skip ) ? "increased" : "decreased";

                printf( "[SYSTEM] Temporal controller %s skip at frame %u: %u -> %u ( %.1f FPS ).\n", event, last_control_frame, previous_skip, current_temporal_skip, TARGET_FPS / current_temporal_skip );
                temporal_notification_printed = true;
            }
        }

        rte_pktmbuf_free( m );
    }
}

static inline void flush_tx_burst( struct rte_mbuf **tx_bufs, uint16_t *tx_points_buf, int *burst_idx, uint32_t *frame_tx_packets, uint32_t *frame_tx_points, uint32_t *frame_tx_zero_accepts, uint32_t *frame_tx_partial_accepts, uint32_t *frame_tx_resubmit_calls, uint32_t *frame_tx_resubmitted_packets, uint64_t *frame_active_tx_cycles ) {
    
    // Purpose: It submits a single transmission burst, locally re-presenting only those "mbufs" rejected by the egress queue. 
    //          This mechanism handles interface backpressure rather than providing transport-level re-issuance
    
    if ( *burst_idx == 0 )
        return;

    uint16_t sent = 0;
    uint16_t retries = 0;

    bool is_resubmission = false;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint16_t requested_packets = *burst_idx - sent;

        if ( is_resubmission ) {
            ( *frame_tx_resubmit_calls )++;
            *frame_tx_resubmitted_packets += requested_packets;
        }

        uint64_t t_active_tx_start = rte_get_timer_cycles();

        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF1, 0, &tx_bufs[ sent ], requested_packets );

        uint64_t t_active_tx_end = rte_get_timer_cycles();

        *frame_active_tx_cycles += t_active_tx_end - t_active_tx_start;

        if ( nb_tx > 0 ) {
            *frame_tx_packets += nb_tx;

            for ( uint16_t j = 0; j < nb_tx; j++ )
                *frame_tx_points += tx_points_buf[ sent + j ];

            if ( nb_tx < requested_packets )
                ( *frame_tx_partial_accepts )++;

            sent += nb_tx;
            retries = 0;

            is_resubmission = nb_tx < requested_packets;
        }
        else {
            ( *frame_tx_zero_accepts )++;

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

static int worker_loop( __rte_unused void *arg ) {

    // Purpose: It executes the frame-level isochronous source pipeline by sampling temporal control signals before loading, packetizing & transmitting each selected frame.
    //          Forwarding deadlines are computed as "t_expected( k ) = t_start + k * ( F_timer / TARGET_FPS )" with choice determined by "( frame_id - 1 ) mod temporal_skip == 0"

    uint64_t timer_hz = rte_get_timer_hz();
    uint64_t frame_cycles = ( uint64_t )( timer_hz / TARGET_FPS );
    
    struct rte_mbuf *burst_buffer[ BURST_SIZE ];

    uint16_t burst_points[ BURST_SIZE ];

    uint16_t standard_payload_size = POINTS_PER_PACKET * POINT_SIZE_BYTES;
    template_hdr.net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + standard_payload_size );
    template_hdr.net.ipv4.hdr_checksum = 0;
    template_hdr.net.ipv4.hdr_checksum = rte_ipv4_cksum( &template_hdr.net.ipv4 );
    template_hdr.net.udp.dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + standard_payload_size );

    const size_t outer_len = sizeof( struct main_hdr );

    printf( "[SYSTEM] Streaming is about to begin at %.1f FPS...\n\n", TARGET_FPS );
    
    while ( 1 ) {   
        uint64_t start_time = rte_get_timer_cycles();

        for ( int frame = 0; frame < loaded_frames; frame++ ) {
            uint32_t frame_id = frame + 1;

            uint64_t expected_time = start_time + ( ( frame + 1 ) * frame_cycles );
            
            poll_temporal_control();

            uint16_t frame_temporal_skip = current_temporal_skip;
            uint32_t frame_control_source = last_control_frame;

            if ( unlikely( frame_temporal_skip == 0 ) )
                frame_temporal_skip = 1;

            bool frame_selected = ( ( ( frame_id - 1 ) % frame_temporal_skip ) == 0 );

            if ( unlikely( !frame_selected ) ) {
                if ( !csv_written ) {
                    telemetry_log[ frame ].frame_id = frame_id;
                    telemetry_log[ frame ].status = 0;
                    telemetry_log[ frame ].current_skip = frame_temporal_skip;
                    telemetry_log[ frame ].last_control_frame = frame_control_source;
                }

                if ( CACHE_MODE == CACHE_MODE_BEST && frames[ frame ].buffer != NULL ) {
                    munlock( frames[ frame ].buffer, frames[ frame ].size );
                    free( frames[ frame ].buffer );
                    
                    frames[ frame ].buffer = NULL;
                }

                while ( rte_get_timer_cycles() < expected_time )
                    rte_pause();

                continue;
            }

            uint64_t t_start_residency = rte_get_timer_cycles();

            double current_disk_io_ms = 0.0;
            double serialization_sec = 0.0;

            uint8_t *frame_buffer = NULL;

            if ( CACHE_MODE == CACHE_MODE_BEST )
                frame_buffer = frames[ frame ].buffer;
            else if ( CACHE_MODE == CACHE_MODE_MIDDLE )
                frame_buffer = staging_buffer;

            if ( CACHE_MODE != CACHE_MODE_BEST ) {
                uint64_t t_start_io = rte_get_timer_cycles();
                
                if ( CACHE_MODE == CACHE_MODE_WORST ) {
                    frame_buffer = malloc( frames[ frame ].size );
                    
                    if ( frame_buffer == NULL ) 
                        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Buffer allocation failed...\n" );
                }

                FILE *fp = fopen( frames[ frame ].file_path, "rb" );

                if ( fp ) {
                    if ( fread( frame_buffer, 1, frames[ frame ].size, fp ) != frames[ frame ].size )
                        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: I / O anomaly while reading file \"%s\"...\n", frames[ frame ].file_path );
                    
                    fclose( fp );
                }
                else
                    rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to open file \"%s\"...\n", frames[ frame ].file_path );
                
                uint64_t t_end_io = rte_get_timer_cycles();
                current_disk_io_ms = ( double )( t_end_io - t_start_io ) * 1000.0 / timer_hz;
                
                uint64_t t_start_serialization = rte_get_timer_cycles();

                serialize_points( frame_buffer, frames[ frame ].point_count );
                
                uint64_t t_end_serialization = rte_get_timer_cycles();
                serialization_sec = ( double )( t_end_serialization - t_start_serialization ) / timer_hz;
            }

            uint32_t total_points = frames[ frame ].point_count;
            uint32_t points_sent = 0;
            uint32_t sequence_number = 0;
            int burst_idx = 0;

            uint32_t frame_tx_zero_accepts = 0;
            uint32_t frame_tx_partial_accepts = 0;
            uint32_t frame_tx_resubmit_calls = 0;
            uint32_t frame_tx_resubmitted_packets = 0;

            uint32_t frame_mbuf_drops = 0;

            uint32_t frame_tx_packets = 0;
            uint32_t frame_tx_points = 0;

            uint64_t frame_active_tx_cycles = 0;

            uint8_t *raw_points_ptr = frame_buffer;

            template_hdr.net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + standard_payload_size );
            template_hdr.net.ipv4.hdr_checksum = 0;
            template_hdr.net.ipv4.hdr_checksum = rte_ipv4_cksum( &template_hdr.net.ipv4 );
            template_hdr.net.udp.dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + standard_payload_size );

            template_hdr.cam.temporal_skip = rte_cpu_to_be_16( frame_temporal_skip );
            
            uint64_t t_send_start = rte_get_timer_cycles();
            
            while ( points_sent < total_points ) {
                uint32_t batch = ( total_points - points_sent > POINTS_PER_PACKET ) ? POINTS_PER_PACKET : ( total_points - points_sent );
                uint16_t payload_size = batch * POINT_SIZE_BYTES;

                struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );
                if ( likely( m != NULL ) ) {
                    uint16_t pkt_len = outer_len + payload_size;
                    uint8_t *pkt_data = ( uint8_t * )rte_pktmbuf_append( m, pkt_len );

                    if ( unlikely( pkt_data == NULL ) ) {
                        rte_pktmbuf_free( m );
                        break;
                    }

                    template_hdr.cam.frame_id = htonl( frame_id );
                    template_hdr.cam.sequence_number = htonl( sequence_number );
                    template_hdr.cam.timestamp = rte_cpu_to_be_64( t_send_start );
                    template_hdr.cam.original_points = htonl( total_points );
                    template_hdr.cam.points_in_packet = htonl( batch );

                    if ( unlikely( batch != POINTS_PER_PACKET ) ) {
                        template_hdr.net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + payload_size );
                        template_hdr.net.ipv4.hdr_checksum = 0;
                        template_hdr.net.ipv4.hdr_checksum = rte_ipv4_cksum( &template_hdr.net.ipv4 );
                        template_hdr.net.udp.dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + payload_size );
                    }

                    rte_memcpy( pkt_data, &template_hdr, outer_len );
                    rte_memcpy( pkt_data + outer_len, raw_points_ptr + ( points_sent * POINT_SIZE_BYTES ), payload_size );

                    burst_buffer[ burst_idx ] = m;
                    burst_points[ burst_idx ] = batch;
                    burst_idx++;
                }
                else {
                    frame_mbuf_drops++;
                    break;
                }

                points_sent += batch;
                sequence_number++;

                if ( burst_idx == BURST_SIZE || points_sent == total_points )
                    flush_tx_burst( burst_buffer, burst_points, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_tx_zero_accepts, &frame_tx_partial_accepts, &frame_tx_resubmit_calls, &frame_tx_resubmitted_packets, &frame_active_tx_cycles );
            }

            uint64_t t_send_end = rte_get_timer_cycles();

            if ( CACHE_MODE == CACHE_MODE_BEST && frames[ frame ].buffer != NULL ) {
                munlock( frames[ frame ].buffer, frames[ frame ].size );
                free( frames[ frame ].buffer );
                
                frames[ frame ].buffer = NULL;
            }
            else if ( CACHE_MODE == CACHE_MODE_WORST && frame_buffer != NULL ) {
                free( frame_buffer );
                frame_buffer = NULL;
            }

            if ( !csv_written ) {
                uint64_t send_cycles = t_send_end - t_send_start;

                double send_duration_sec = ( double )send_cycles / timer_hz;
                double residency_sec = ( double )( t_send_end - t_start_residency ) / timer_hz;

                double active_tx_sec = ( double )frame_active_tx_cycles / timer_hz;

                uint64_t transmitted_payload_bytes = ( uint64_t )frame_tx_points * POINT_SIZE_BYTES;
                uint64_t logical_frame_bytes = transmitted_payload_bytes + ( frame_tx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );
                uint64_t network_frame_bytes = transmitted_payload_bytes + ( ( uint64_t )frame_tx_packets * sizeof( struct main_hdr ) ) ;

                double effective_fps = TARGET_FPS / frame_temporal_skip;

                telemetry_log[ frame ].frame_id = frame_id;
                telemetry_log[ frame ].status = ( frame_tx_points == total_points && frame_mbuf_drops == 0 ) ? 1 : 0;
                telemetry_log[ frame ].current_skip = frame_temporal_skip;
                telemetry_log[ frame ].last_control_frame = frame_control_source;
                telemetry_log[ frame ].timestamp_start_tx = ( double )t_send_start / timer_hz;
                telemetry_log[ frame ].tx_duration_ms = send_duration_sec * 1000.0;
                telemetry_log[ frame ].payload_bytes = transmitted_payload_bytes;
                telemetry_log[ frame ].disk_io_ms = current_disk_io_ms;
                telemetry_log[ frame ].serialization_ms = serialization_sec * 1000.0;
                telemetry_log[ frame ].internal_throughput_mbs = ( send_duration_sec > 0 ) ? ( ( double )logical_frame_bytes / 1000000.0 ) / send_duration_sec : 0.0;
                telemetry_log[ frame ].logical_bitrate_mbps = ( logical_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
                telemetry_log[ frame ].network_bitrate_mbps = ( network_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
                telemetry_log[ frame ].total_residency_ms = residency_sec * 1000.0;
                telemetry_log[ frame ].tx_packets = frame_tx_packets;
                telemetry_log[ frame ].tx_points = frame_tx_points;

                telemetry_log[ frame ].tx_zero_accepts = frame_tx_zero_accepts;
                telemetry_log[ frame ].tx_partial_accepts = frame_tx_partial_accepts;
                telemetry_log[ frame ].tx_resubmit_calls = frame_tx_resubmit_calls;
                telemetry_log[ frame ].tx_resubmitted_packets = frame_tx_resubmitted_packets;
                telemetry_log[ frame ].mbuf_starvation = frame_mbuf_drops;

                telemetry_log[ frame ].active_tx_ms = active_tx_sec * 1000.0;

                double disk_io_sec = current_disk_io_ms / 1000.0;
                double active_process_sec = disk_io_sec + serialization_sec + send_duration_sec; 

                telemetry_log[ frame ].active_process_ms = active_process_sec * 1000.0;

                if ( residency_sec > 0.0 )
                    telemetry_log[ frame ].node_efficiency_pct = ( active_process_sec / residency_sec ) * 100.0;
                else
                    telemetry_log[ frame ].node_efficiency_pct = 0.0;
            }

            while ( rte_get_timer_cycles() < expected_time ) 
                rte_pause();
        }

        struct rte_mbuf *eos_burst[ BURST_SIZE ];
        int eos_count = 0;
        
        for ( int i = 0; i < BURST_SIZE; i++ ) {
            struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );
            
            if ( likely( m != NULL ) ) {
                uint16_t pkt_len = sizeof( struct net_hdr ) + sizeof( struct cam_hdr );
                uint8_t *pkt_data = ( uint8_t * )rte_pktmbuf_append( m, pkt_len );
                
                if ( unlikely( pkt_data == NULL ) ) {
                    rte_pktmbuf_free( m );
                    break;
                }

                template_hdr.net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) );
                template_hdr.net.ipv4.hdr_checksum = 0;
                template_hdr.net.ipv4.hdr_checksum = rte_ipv4_cksum( &template_hdr.net.ipv4 );
                template_hdr.net.udp.dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr) + sizeof( struct cam_hdr ) );
                
                template_hdr.cam.frame_id = htonl( END_OF_STREAM );
                template_hdr.cam.temporal_skip = rte_cpu_to_be_16( current_temporal_skip );
                template_hdr.cam.points_in_packet = 0;
                template_hdr.cam.original_points = 0;
                
                rte_memcpy( pkt_data, &template_hdr, outer_len );
                
                eos_burst[ eos_count++ ] = m;
            }
            else
                break;
        }
        
        if ( eos_count > 0 ) {
            uint16_t sent = 0;
            uint16_t zero_accept_streak = 0;

            while ( sent < eos_count ) {
                uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF1, 0, &eos_burst[ sent ], eos_count - sent );
                sent += nb_tx;

                if ( nb_tx == 0 ) {
                    if ( ++zero_accept_streak > MAX_ZERO_ACCEPTS ) {
                        for ( uint16_t i = sent; i < eos_count; i++ )
                            rte_pktmbuf_free( eos_burst[ i ] );

                        break;
                    }

                    rte_pause();
                } 
                else
                    zero_accept_streak = 0;
            }
        }

        if ( !csv_written ) {
            if ( temporal_notification_printed )
                printf( "\n" );

            telemetry_to_csv();
            csv_written = true;
        }
        
        printf( "\n[SYSTEM] Sequence completed. Shutting down...\n" );
        rte_delay_ms( WAITING_TIME );
        break;
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It instantiates the volumetric acquisition node, configures the "DPDK" endpoint, & streams pre-converted point-cloud frames according to the target schedule.
    //          "Temporal" requests dictate fount selection exclusively. Geometry & coding remain downstream responsibilities. 
    //          Diagnostic metrics are shielded in memory until the conveyance terminates

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );
    
    if ( ret < 0 ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"Camera\" microservice...\n" );

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );
    
    if ( mbuf_pool == NULL ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF1, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF1-facing virtual port configuration failed...\n" );

    printf( "\n[SYSTEM] Preparing volumetric data for modality \"%s\"...\n", ( CACHE_MODE == CACHE_MODE_BEST ) ? "Best" : ( CACHE_MODE == CACHE_MODE_MIDDLE ) ? "Middle" : ( CACHE_MODE == CACHE_MODE_WORST ) ? "Worst" : "" );
    
    for ( int i = 0; i < K_FRAMES; i++ ) {
        snprintf( frames[ i ].file_path, sizeof( frames[ i ].file_path ), "%s/loot_vox10_%d.bin", SEQUENCE_FOLDER, i + 1000 );
        
        FILE *fp = fopen( frames[ i ].file_path, "rb" );
        
        if ( !fp )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Unable to open file \"%s\"...\n", frames[ i ].file_path );

        fseek( fp, 0, SEEK_END );
        size_t file_size = ftell( fp );
        rewind( fp );

        if ( file_size % POINT_SIZE_BYTES != 0 )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: File size did not match about \"%s\"...\n", frames[ i ].file_path );

        frames[ i ].size = file_size;
        frames[ i ].point_count = file_size / POINT_SIZE_BYTES;

        if ( file_size > staging_buffer_size )
            staging_buffer_size = file_size;

        if ( CACHE_MODE == CACHE_MODE_BEST ) {
            frames[ i ].buffer = malloc( file_size );

            if ( frames[ i ].buffer == NULL )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Buffer allocation failed...\n" );

            if ( fread( frames[ i ].buffer, 1, file_size, fp ) != file_size )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: I / O anomaly while reading file \"%s\"...\n", frames[ i ].file_path );

            serialize_points( frames[ i ].buffer, frames[ i ].point_count );
            
            if ( mlock( frames[ i ].buffer, frames[ i ].size ) != 0 )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"mlock\" failed about buffer...\n" );
        }

        fclose( fp );
        loaded_frames++;
    }

    printf( "[SYSTEM] Reference elements structurally mapped: %d.\n\n", loaded_frames );

    if ( CACHE_MODE == CACHE_MODE_MIDDLE ) {
        staging_buffer = malloc( staging_buffer_size );

        if ( staging_buffer == NULL )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Buffer allocation failed...\n" );

        memset( staging_buffer, 0, staging_buffer_size );
        
        if ( WARM_MODE == WARM_MODE_ENABLED )
            if ( mlock( staging_buffer, staging_buffer_size ) != 0 )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"mlock\" failed about buffer...\n" );
    }

    if ( WARM_MODE == WARM_MODE_ENABLED && CACHE_MODE != CACHE_MODE_BEST )
        lock_persistent_pages();
    
    header_init( &template_hdr );

    printf( "[SYSTEM] Waiting time for initial stability...\n\n" );
    rte_delay_ms( WAITING_TIME );

    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE ) 
        worker_loop( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    if ( WARM_MODE == WARM_MODE_ENABLED && CACHE_MODE != CACHE_MODE_BEST )
        unlock_persistent_pages();

    if ( staging_buffer != NULL ) {
        if ( WARM_MODE == WARM_MODE_ENABLED )
            munlock( staging_buffer, staging_buffer_size );
        
        free( staging_buffer );
        staging_buffer = NULL;
        staging_buffer_size = 0;
    }

    rte_eal_cleanup();

    return 0;
}