#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_cycles.h>

// Configuration variables
#define TELEMETRY_FOLDER "/shared/log/sff1"
#define TELEMETRY_PATH "/shared/log/sff1/telemetry_sff1.csv"

#define K_FRAMES 300

#define TARGET_FPS 30.0 

#define BURST_SIZE 32
#define MAX_RETRIES 2048

#define END_OF_STREAM 0xFFFFFFFF
// Memory pool settings for "mbufs"
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// "SFP" identifiers
#define PRIMARY_SPI 100
#define PRIMARY_SI 255

#define FEEDBACK_SPI 200
#define FEEDBACK_SI 252

// Transmission bonds ( networking parameters )
#define PORT_RX 0
#define PORT_TX 1
#define SRC_IP RTE_IPV4( 10, 0, 2, 1 )
#define DST_IP RTE_IPV4( 10, 0, 2, 2 ) // SFF2
#define SRC_PORT 5001
#define DST_PORT 6633 // standard "IETF" port for "NSH" encapsulated over "UDP"

struct nsh_hdr {
    uint16_t base_flags_ttl_len; 
    uint8_t  md_type;
    uint8_t  next_protocol;
    uint32_t serv_path_hdr; // "SPI" ( 24 bits ), "SI" ( "8 bits" )
} __attribute__((__packed__)); 

// "In-band Network Telemetry" ( "INT" ) metadata appended dynamically by SFF1 to support Encoder. This structure contains running extents ( "Bounding Box" ) & barycenter sums
struct int_hdr { // computational "offloading" to provide a frame-based accumulator working at "line-rate"
    double sum_x, sum_y, sum_z;         
    float min_x, min_y, min_z; 
    float max_x, max_y, max_z; 
    uint32_t active_point_count;  
    uint32_t original_point_count;        
} __attribute__((__packed__));

struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__));

// Outer header structure for SFF1
struct full_hdr {
    struct net_hdr net;
    struct nsh_hdr nsh;
    struct int_hdr meta;
} __attribute__((__packed__));

struct cam_hdr { // per-packet control information for the current node
    uint32_t frame_id;
    uint32_t sequence_number;
    uint64_t timestamp;
    float yaw; 
    float pitch;
    float zoom;
    uint16_t temporal_skip; 
    uint16_t padding; 
    uint32_t original_points;
    uint32_t points_in_packet;
} __attribute__((__packed__));

struct point_tx { 
    float x, y, z; 
    uint8_t r, g, b; 
    uint8_t padding;
} __attribute__((__packed__));

struct feedback_payload {
    uint16_t skip;
    float yaw;
    float pitch;
    float zoom;
} __attribute__((__packed__));

struct telemetry_csv {
    uint32_t frame_id;
    uint8_t status;
    uint16_t current_skip;

    double camera_send_timestamp;
    double recv_start_timestamp;
    double node_exit_timestamp;

    uint32_t rx_points;      
    uint32_t tx_points;
    uint32_t rx_packets;      
    uint32_t tx_packets;
    uint32_t payload_bytes;
    double internal_throughput_mbs;
    double network_bitrate_mbps;

    double tx_duration_ms;
    double active_process_ms;
    double cycle_ms;
    double header_wait_ms;
    double total_residency_ms;
    double node_efficiency_pct;

    double camera_to_node_latency_ms;
    double schedule_delay_ms;
    double network_jitter_ms;

    uint32_t tx_retries;
};

struct full_hdr template_hdr;
struct telemetry_csv telemetry_log[ K_FRAMES ];
int logged_frames = 0;
bool csv_written = false;

uint16_t current_dynamic_skip = 0; // 0 means fallback to Camera's original temporal skip, otherwise it is dynamically adjusted by the "Feedback" mechanism

static inline int port_init( uint16_t port, struct rte_mempool *mbuf_pool ) {
    struct rte_eth_conf port_conf = { 0 };
    int retval;

    if ( ! rte_eth_dev_is_valid_port( port ) ) 
        return -1;

    retval = rte_eth_dev_configure( port, 1, 1, &port_conf );

    if ( retval != 0 ) 
        return retval;

    if ( port == PORT_RX ) 
        retval = rte_eth_rx_queue_setup( port, 0, 4096, rte_eth_dev_socket_id( port ), NULL, mbuf_pool );
    else 
        retval = rte_eth_rx_queue_setup( port, 0, 1024, rte_eth_dev_socket_id( port ), NULL, mbuf_pool );

    if ( retval < 0 ) 
        return retval;

    retval = rte_eth_tx_queue_setup( port, 0, 1024, rte_eth_dev_socket_id( port ), NULL );
        
    if ( retval < 0 ) 
        return retval;
    

    retval = rte_eth_dev_start( port );

    if ( retval < 0 ) 
        return retval;

    return 0;
}

static void header_init( struct full_hdr *hdr ) {
    memset( hdr, 0, sizeof( struct full_hdr ) );
    
    struct rte_ether_addr src_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x01 } };
    struct rte_ether_addr dst_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x02 } };
    
    rte_memcpy( &hdr -> net.ethernet.src_addr, &src_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> net.ethernet.dst_addr, &dst_mac, RTE_ETHER_ADDR_LEN );
    
    hdr -> net.ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> net.ipv4.version_ihl = 0x45;
    hdr -> net.ipv4.time_to_live = 64;
    hdr -> net.ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> net.ipv4.src_addr = rte_cpu_to_be_32( SRC_IP );
    hdr -> net.ipv4.dst_addr = rte_cpu_to_be_32( DST_IP );

    hdr -> net.udp.src_port = rte_cpu_to_be_16( SRC_PORT );
    hdr -> net.udp.dst_port = rte_cpu_to_be_16( DST_PORT );
    hdr -> net.udp.dgram_cksum = 0;

    hdr -> nsh.base_flags_ttl_len = htons( 0x0006 ); // "TTL" = 63, "Len" = 6 ( 4-byte words for "NSH" + "INT" )
    hdr -> nsh.md_type = 0x02; // context headers present
    hdr -> nsh.next_protocol = 0x03; // "IPv4" inner
    hdr -> nsh.serv_path_hdr = htonl( ( PRIMARY_SPI << 8 ) | PRIMARY_SI );
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

    fprintf( f, "frame_id;status;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;internal_throughput_mbs;network_bitrate_mbps;tx_duration_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;camera_to_node_latency_ms;schedule_delay_ms;network_jitter_ms;tx_retries\n" );

    for ( int i = 0; i < logged_frames; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        fprintf( f, "%u;%u;%u;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u\n", t -> frame_id, t -> status, t -> current_skip, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> rx_points, t -> tx_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> internal_throughput_mbs, t -> network_bitrate_mbps, t -> tx_duration_ms, t -> active_process_ms, t -> cycle_ms, t -> header_wait_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> camera_to_node_latency_ms, t -> schedule_delay_ms, t -> network_jitter_ms, t -> tx_retries );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static inline void flush_tx_burst( struct rte_mbuf **tx_bufs, uint32_t *tx_points_buf, int *burst_idx, uint32_t *frame_tx_packets, uint32_t *frame_tx_points, uint32_t *frame_tx_retries, uint64_t *frame_first_tx_cycles, uint64_t *frame_last_tx_cycles ) {
    
    // Purpose: It implements several "Hot Path" optimizations avoiding unnecessary function calls & branching when burst is empty
    
    if ( *burst_idx == 0 ) 
        return;
    
    uint16_t sent = 0; 
    uint16_t retries = 0;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint64_t t_tx_call_start = rte_get_timer_cycles();

        if ( *frame_first_tx_cycles == 0 )
            *frame_first_tx_cycles = t_tx_call_start;

        uint16_t nb_tx = rte_eth_tx_burst( PORT_TX, 0, &tx_bufs[ sent ], *burst_idx - sent );
        
        uint64_t t_tx_call_end = rte_get_timer_cycles();

        if ( nb_tx > 0 ) {
            *frame_last_tx_cycles = t_tx_call_end;

            *frame_tx_packets += nb_tx;

            for ( uint16_t j = 0; j < nb_tx; j++ )
                *frame_tx_points += tx_points_buf[ sent + j ];

            sent += nb_tx;
            retries = 0;
        }
        else {
            ( *frame_tx_retries )++;
            
            if ( ++retries > MAX_RETRIES ) {
                for( int k = sent; k < *burst_idx; k++ ) 
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
    uint64_t timer_hz = rte_get_timer_hz();
    
    struct rte_mbuf *bufs[ BURST_SIZE ];
    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    struct rte_mbuf *fb_bufs[ BURST_SIZE ];

    uint32_t tx_points_buf[ BURST_SIZE ];

    uint32_t current_frame_id = END_OF_STREAM;
    double frame_sum_x = 0.0, frame_sum_y = 0.0, frame_sum_z = 0.0;
    float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
    float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
    uint32_t frame_point_count = 0;
    uint32_t frame_rx_points = 0;
    uint32_t frame_original_points = 0;

    uint64_t t_frame_arrival = 0;

    uint64_t t_cycle_start = rte_get_timer_cycles();

    uint64_t frame_active_process_cycles = 0;

    uint32_t frame_tx_retries = 0;
    uint32_t frame_tx_packets = 0;
    uint32_t frame_rx_packets = 0;

    uint32_t frame_tx_points = 0;

    uint64_t frame_completion_cycles = 0;
    uint64_t frame_last_activity_cycles = 0;

    uint64_t frame_first_tx_cycles = 0;
    uint64_t frame_last_tx_cycles = 0;

    int burst_idx = 0;

    uint64_t t_session_start = 0;
    uint32_t frames_received = 0;
    double current_frame_latency_ms = 0.0;
    double current_jitter_ms = 0.0;

    uint64_t prev_arrival_cyc = 0;

    uint32_t first_arrival_f_id = 0;
    
    uint16_t active_temporal_skip = 1;
    uint16_t frame_temporal_skip = 1;

    uint8_t frame_status = 1;

    const size_t outer_len = sizeof( struct full_hdr );

    printf( "[SYSTEM] Listening on \"UDP\" port %u...\n\n", SRC_PORT );

    while ( 1 ) {
        // Reverse chain ( "Feedback" on "PORT_TX" ) to dynamically adjust the temporal skip 
        uint16_t nb_fb = rte_eth_rx_burst( PORT_TX, 0, fb_bufs, BURST_SIZE );
        
        for ( int i = 0; i < nb_fb; i++ ) {
            struct rte_mbuf *fm = fb_bufs[ i ];

            size_t min_fb_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct feedback_payload );
            
            if ( unlikely( fm -> data_len < min_fb_len ) ) { 
                rte_pktmbuf_free( fm ); 
                continue; 
            }

            struct rte_ether_hdr *eth = rte_pktmbuf_mtod( fm, struct rte_ether_hdr * );

            if ( eth -> ether_type == rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) {
                struct rte_ipv4_hdr *ipv4 = (struct rte_ipv4_hdr *)(eth + 1);
                
                if ( likely( ipv4 -> next_proto_id == IPPROTO_UDP ) ) {
                    struct rte_udp_hdr *udp = (struct rte_udp_hdr *)( ipv4 + 1 );
                        
                    if ( likely( udp->dst_port == rte_cpu_to_be_16( DST_PORT ) ) ) {
                        struct nsh_hdr *nsh = ( struct nsh_hdr * )( udp + 1 );

                        uint32_t sph = ntohl( nsh -> serv_path_hdr );
                        uint32_t spi = sph >> 8;
                        uint8_t  si  = sph & 0xFF;

                        if ( spi == FEEDBACK_SPI && si == FEEDBACK_SI ) {
                            struct feedback_payload *fb_cmd = ( struct feedback_payload * )( nsh + 1 );
                            uint16_t requested_skip = ntohs( fb_cmd -> skip );

                            if ( requested_skip != current_dynamic_skip ) {
                                current_dynamic_skip = requested_skip;

                                if ( current_dynamic_skip == 0 )
                                    printf( "[SYSTEM] Temporal controller reverted to \"Camera\"'s native skip ( %.1f FPS ).\n", TARGET_FPS );
                                else
                                    printf( "[SYSTEM] Temporal controller updated to: %u ( %.1f FPS ).\n", current_dynamic_skip, TARGET_FPS / current_dynamic_skip );
                            }
                        }
                    }
                }
            }

            rte_pktmbuf_free( fm ); 
        }

        // Main interaction ( video on "PORT_RX" ) to receive "Point Cloud" frames
        uint16_t nb_rx = rte_eth_rx_burst( PORT_RX, 0, bufs, BURST_SIZE );
        
        if ( unlikely( nb_rx == 0 ) ) 
            continue;

        for ( int i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];

            // Validating inner packet structure ( "eth" + "ipv4" + "udp" + "cam_hdr" )
            size_t min_req = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr );

            if ( unlikely( m -> data_len < min_req ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }

            struct rte_ether_hdr *old_eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );
            
            if ( unlikely( old_eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }

            struct rte_ipv4_hdr *old_ipv4 = ( struct rte_ipv4_hdr * )( old_eth + 1 );
            
            if ( unlikely( old_ipv4 -> next_proto_id != IPPROTO_UDP ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }

            struct rte_udp_hdr *old_udp = ( struct rte_udp_hdr * )( old_ipv4 + 1 );
            
            if ( unlikely( old_udp -> dst_port != rte_cpu_to_be_16( SRC_PORT ) ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }

            struct cam_hdr *old_cam = ( struct cam_hdr * )( ( char * )old_udp + sizeof( struct rte_udp_hdr ) );
            uint32_t f_id = ntohl( old_cam -> frame_id );

            uint64_t packet_arrival_cycles = rte_get_timer_cycles();

            active_temporal_skip = ( current_dynamic_skip > 0 ) ? current_dynamic_skip : ntohs( old_cam -> temporal_skip );
            
            if ( unlikely( active_temporal_skip == 0 ) ) 
                active_temporal_skip = 1;

            // Frame boundary shift
            if ( unlikely( f_id != current_frame_id ) ) {
                if ( frame_status == 1 && current_frame_id != END_OF_STREAM && burst_idx > 0 ) {
                    uint64_t t_final_flush_start = rte_get_timer_cycles();

                    flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_tx_retries, &frame_first_tx_cycles, &frame_last_tx_cycles );
                
                    uint64_t t_final_flush_end = rte_get_timer_cycles();
                    frame_active_process_cycles += t_final_flush_end - t_final_flush_start;

                    frame_last_activity_cycles = t_final_flush_end;
                }

                if ( current_frame_id != END_OF_STREAM && logged_frames < K_FRAMES && !csv_written ) {
                    uint64_t t_now = rte_get_timer_cycles();
                    uint64_t t_frame_end;

                    if ( frame_completion_cycles > 0 )
                        t_frame_end = frame_completion_cycles;  
                    else if ( frame_last_activity_cycles > 0 )
                        t_frame_end = frame_last_activity_cycles;
                    else
                        t_frame_end = t_now;
                    
                    double duration_sec = 0.0;

                    if ( frame_status == 1 && frame_first_tx_cycles > 0 && frame_last_tx_cycles >= frame_first_tx_cycles )
                        duration_sec = ( double )( frame_last_tx_cycles - frame_first_tx_cycles ) / timer_hz; 
                    
                    double residency_sec = ( double )( t_frame_end - t_frame_arrival ) / timer_hz;

                    double cycle_sec = ( t_frame_end >= t_cycle_start ) ? ( double )( t_frame_end - t_cycle_start ) / timer_hz : 0.0;
                    double header_wait_sec = ( cycle_sec > residency_sec ) ? cycle_sec - residency_sec : 0.0;

                    double active_process_sec = ( double )( frame_active_process_cycles ) / timer_hz;
                    
                    uint64_t logical_payload_bytes = ( uint64_t )frame_rx_points * sizeof( struct point_tx );
                    uint64_t logical_frame_bytes = logical_payload_bytes + ( frame_rx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );

                    telemetry_log[ logged_frames ].frame_id = current_frame_id;
                    telemetry_log[ logged_frames ].status = frame_status;
                    telemetry_log[ logged_frames ].current_skip = frame_temporal_skip;
                    telemetry_log[ logged_frames ].node_exit_timestamp = ( double )t_frame_end / timer_hz;
                    
                    telemetry_log[ logged_frames ].tx_duration_ms = duration_sec * 1000.0;
                    telemetry_log[ logged_frames ].payload_bytes = logical_payload_bytes;
                    
                    telemetry_log[ logged_frames ].rx_packets = frame_rx_packets;
                    telemetry_log[ logged_frames ].rx_points = frame_rx_points;

                    telemetry_log[ logged_frames ].tx_packets = frame_tx_packets;
                    telemetry_log[ logged_frames ].tx_points = frame_tx_points;

                    telemetry_log[ logged_frames ].tx_retries = frame_tx_retries;
                    
                    telemetry_log[ logged_frames ].active_process_ms = active_process_sec * 1000.0;
                    telemetry_log[ logged_frames ].total_residency_ms = residency_sec * 1000.0;

                    telemetry_log[ logged_frames ].cycle_ms = cycle_sec * 1000.0;
                    telemetry_log[ logged_frames ].header_wait_ms = header_wait_sec * 1000.0;
                    
                    telemetry_log[ logged_frames ].camera_to_node_latency_ms = current_frame_latency_ms;
                    
                    uint32_t schedule_frame_offset = current_frame_id - first_arrival_f_id;

                    double real_elapsed_sec = ( double ) ( t_frame_end - t_session_start ) / timer_hz;
                    double ideal_elapsed_sec = ( double )schedule_frame_offset / TARGET_FPS;

                    telemetry_log[ logged_frames ].schedule_delay_ms = ( real_elapsed_sec - ideal_elapsed_sec ) * 1000.0;
                    
                    telemetry_log[ logged_frames ].network_jitter_ms = current_jitter_ms;

                    telemetry_log[ logged_frames ].internal_throughput_mbs = ( residency_sec > 0 ) ? ( ( double )logical_frame_bytes / 1000000.0 ) / residency_sec : 0.0;
                    telemetry_log[ logged_frames ].network_bitrate_mbps = ( logical_frame_bytes * 8.0 * ( TARGET_FPS / frame_temporal_skip ) ) / 1000000.0;

                    if ( cycle_sec > 0.0 )
                        telemetry_log[ logged_frames ].node_efficiency_pct = ( residency_sec / cycle_sec ) * 100.0;
                    else
                        telemetry_log[ logged_frames ].node_efficiency_pct = 0.0;
                    
                    t_cycle_start = t_frame_end;
                    logged_frames++;
                }

                uint64_t cam_tx_cycles = rte_be_to_cpu_64( old_cam -> timestamp );
                uint64_t arrival_cycles = packet_arrival_cycles;

                // Network latency ( pure trip )
                if ( arrival_cycles >= cam_tx_cycles )
                    current_frame_latency_ms = ( ( double )( arrival_cycles - cam_tx_cycles ) / timer_hz ) * 1000.0;
                else
                    current_frame_latency_ms = 0.0;

                // Schedule delay ( deviation against ideal cadence )
                if ( frames_received == 0 ) {
                    t_session_start = arrival_cycles;
                    first_arrival_f_id = f_id;

                    t_cycle_start = arrival_cycles;
                }

                // Jitter
                if ( prev_arrival_cyc > 0 ) {
                    double real_interval_sec = ( double )( arrival_cycles - prev_arrival_cyc ) / timer_hz;

                    double expected_interval_sec = 1.0 / TARGET_FPS;

                    double diff = real_interval_sec - expected_interval_sec;

                    current_jitter_ms = ( diff < 0 ) ? -diff * 1000.0 : diff * 1000.0;
                }
                else
                    current_jitter_ms = 0.0;

                prev_arrival_cyc = arrival_cycles;
                frames_received++;

                // Updating aggregators
                current_frame_id = f_id;
                frame_temporal_skip = active_temporal_skip;
                t_frame_arrival = arrival_cycles;

                if ( logged_frames < K_FRAMES ) {
                    telemetry_log[ logged_frames ].camera_send_timestamp = ( double )cam_tx_cycles / timer_hz;
                    telemetry_log[ logged_frames ].recv_start_timestamp = ( double )arrival_cycles / timer_hz;
                }
                
                if ( f_id != END_OF_STREAM )
                    frame_status = ( ( ( f_id - 1 ) % frame_temporal_skip ) == 0 ) ? 1 : 0;
                else
                    frame_status = 0;

                frame_rx_points = 0; frame_tx_retries = 0; frame_tx_packets = 0; frame_rx_packets = 0;
                frame_active_process_cycles = 0;
                frame_tx_points = 0;
                frame_completion_cycles = 0;
                frame_last_activity_cycles = 0;
                frame_first_tx_cycles = 0;
                frame_last_tx_cycles = 0;
                frame_sum_x = 0.0; frame_sum_y = 0.0; frame_sum_z = 0.0;
                frame_point_count = 0; frame_original_points = 0;
                min_x = FLT_MAX; min_y = FLT_MAX; min_z = FLT_MAX;
                max_x = -FLT_MAX; max_y = -FLT_MAX; max_z = -FLT_MAX;
                csv_written = false;
            }

            // "EOS" detection
            if ( unlikely( f_id == END_OF_STREAM ) ) {
                if ( likely( rte_pktmbuf_headroom( m ) >= outer_len ) ) {
                    size_t old_net_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );
                    
                    if ( unlikely( rte_pktmbuf_adj( m, old_net_len ) == NULL ) ) { 
                        rte_pktmbuf_free( m ); 
                        continue; 
                    }
                    
                    char *new_hdr_start = rte_pktmbuf_prepend( m, outer_len );
                    
                    if ( likely( new_hdr_start != NULL ) ) {
                        struct full_hdr *hdr = ( struct full_hdr * )new_hdr_start;
                        rte_memcpy( hdr, &template_hdr, outer_len );
                        
                       memset( &hdr -> meta, 0, sizeof( struct int_hdr ) );
                        
                        uint32_t inner_pkt_len = m -> pkt_len - outer_len;
                        uint16_t outer_udp_len = sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct int_hdr ) + inner_pkt_len;
                        
                        hdr -> net.udp.dgram_len = rte_cpu_to_be_16( outer_udp_len );
                        hdr -> net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + outer_udp_len );
                        hdr -> net.ipv4.hdr_checksum = 0;
                        hdr -> net.ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> net.ipv4 );

                        tx_bufs[ burst_idx ] = m;
                        tx_points_buf[ burst_idx ] = 0;
                        burst_idx++;

                        flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_tx_retries, &frame_first_tx_cycles, &frame_last_tx_cycles );
                    } 
                    else
                        rte_pktmbuf_free( m );
                } 
                else 
                    rte_pktmbuf_free( m );

                if ( !csv_written && logged_frames > 0 ) {
                    telemetry_to_csv();
                    csv_written = true;
                    printf( "\n[SYSTEM] End of stream detected. Changing to \"idle\" plight...\n" );
                }
                
                continue;
            }

            uint64_t t_active_process_start = rte_get_timer_cycles();

            uint32_t packet_points = ntohl( old_cam -> points_in_packet );

            frame_rx_packets++;
            frame_rx_points += packet_points;

            frame_original_points = ntohl( old_cam -> original_points );

            bool frame_complete = frame_original_points > 0 && frame_rx_points >= frame_original_points;

            if ( frame_status == 0 ) {
                rte_pktmbuf_free( m );
                
                uint64_t t_active_process_end = rte_get_timer_cycles();
                frame_active_process_cycles += t_active_process_end - t_active_process_start;

                frame_last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = frame_last_activity_cycles;

                continue;
            }

            // Propagating temporal skip applied by node
            old_cam -> temporal_skip = htons( frame_temporal_skip );

            // "INT" calculation
            uint32_t payload_len = m -> data_len - min_req;

            if ( likely( payload_len > 0 && payload_len % sizeof( struct point_tx ) == 0 ) ) {
                uint32_t num_points = payload_len / sizeof( struct point_tx );
                struct point_tx *points = ( struct point_tx * )( old_cam + 1 );
                frame_original_points = ntohl( old_cam -> original_points ); 

                for (uint32_t point = 0; point < num_points; point++) {
                    float x = points[ point ].x; // host-endian bypass to preserve meaningful cycles
                    float y = points[ point ].y;
                    float z = points[ point ].z;

                    frame_sum_x += x; frame_sum_y += y; frame_sum_z += z;
                    frame_point_count++;

                    if ( x < min_x ) min_x = x; if ( x > max_x ) max_x = x;
                    if ( y < min_y ) min_y = y; if ( y > max_y ) max_y = y;
                    if ( z < min_z ) min_z = z; if ( z > max_z ) max_z = z;
                }
            }

            // "Zero-Copy" prepending
            if ( unlikely( rte_pktmbuf_headroom( m ) < outer_len ) ) { 
                rte_pktmbuf_free( m ); 

                uint64_t t_active_process_end = rte_get_timer_cycles();
                frame_active_process_cycles += t_active_process_end - t_active_process_start;

                frame_last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = frame_last_activity_cycles;

                continue; 
            }

            size_t old_net_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );
            
            if ( unlikely( rte_pktmbuf_adj( m, old_net_len ) == NULL ) ) { 
                rte_pktmbuf_free( m ); 

                uint64_t t_active_process_end = rte_get_timer_cycles();
                frame_active_process_cycles += t_active_process_end - t_active_process_start;

                frame_last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = frame_last_activity_cycles;

                continue; 
            }
            
            char *new_hdr_start = rte_pktmbuf_prepend( m, outer_len );

            if ( unlikely( new_hdr_start == NULL ) ) { 
                rte_pktmbuf_free( m ); 

                uint64_t t_active_process_end = rte_get_timer_cycles();
                frame_active_process_cycles += t_active_process_end - t_active_process_start;

                frame_last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = frame_last_activity_cycles;

                continue; 
            }

            struct full_hdr *hdr = ( struct full_hdr * )new_hdr_start;
            
            rte_memcpy( hdr, &template_hdr, outer_len );

            uint32_t inner_pkt_len = m -> pkt_len - outer_len;
            uint16_t outer_udp_len = sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct int_hdr ) + inner_pkt_len;
            
            hdr -> net.udp.dgram_len = rte_cpu_to_be_16( outer_udp_len );
            hdr -> net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + outer_udp_len );
            hdr -> net.ipv4.hdr_checksum = 0;
            hdr -> net.ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> net.ipv4 );

            hdr -> meta.sum_x = frame_sum_x;
            hdr -> meta.sum_y = frame_sum_y;
            hdr -> meta.sum_z = frame_sum_z;
            hdr -> meta.min_x = min_x;
            hdr -> meta.min_y = min_y;
            hdr -> meta.min_z = min_z;
            hdr -> meta.max_x = max_x;
            hdr -> meta.max_y = max_y;
            hdr -> meta.max_z = max_z;

            hdr -> meta.active_point_count = htonl( frame_point_count );
            hdr -> meta.original_point_count = htonl( frame_original_points );
            
            tx_bufs[ burst_idx ] = m;
            tx_points_buf[ burst_idx ] = packet_points;

            burst_idx++;

            if ( burst_idx == BURST_SIZE || frame_complete )
                flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_tx_retries, &frame_first_tx_cycles, &frame_last_tx_cycles );

            uint64_t t_active_process_end = rte_get_timer_cycles();

            frame_active_process_cycles += t_active_process_end - t_active_process_start;
        
            frame_last_activity_cycles = t_active_process_end;

            if ( frame_complete ) {
                if ( frame_last_tx_cycles > 0 )
                    frame_completion_cycles = frame_last_tx_cycles;
                else
                    frame_completion_cycles = frame_last_activity_cycles;
            }
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It acts as an "Edge Classifier" ( with temporal control for adaptive resolution ), performs "Data-Plane" computations ( e.g., barycenter summation, bounding box ) via "INT" & "NSH" embedding ( "RFC" 8300 ) before forwarding the packets

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n");

    printf( "[SYSTEM] Booting the \"SFF1\" microservice...\n" );

    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );
    
    if ( mbuf_pool == NULL ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_RX, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Virtual \"ingress\" port configuration failed...\n" );

    if ( port_init( PORT_TX, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Virtual \"egress\" port configuration failed...\n" );

    printf( "\n" );

    header_init( &template_hdr );

    // Delegating functions to the assigned logical core
    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE ) 
        worker_loop ( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    rte_eal_cleanup();

    return 0;
}