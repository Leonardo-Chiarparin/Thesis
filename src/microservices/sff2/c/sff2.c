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
#include <rte_random.h> // for the "Random Early Discard" ( "RED" ) probability generation

// Configuration variables
#define TELEMETRY_FOLDER "/shared/log/sff2"
#define SFF1_ENCODER_PATH "/shared/log/sff2/telemetry_sff1_enc.csv"
#define ENCODER_DECODER_PATH "/shared/log/sff2/telemetry_enc_dec.csv"
#define DECODER_SFF3_PATH "/shared/log/sff2/telemetry_dec_sff3.csv"

#define K_FRAMES 300

#define TARGET_FPS 30.0

#define BURST_SIZE 32
#define MAX_RETRIES 2048

#define END_OF_STREAM 0xFFFFFFFF

// Memory pool settings for "mbufs"
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

#define PRIMARY_SPI 100
#define FEEDBACK_SPI 200

// Transmission bonds ( networking parameters )
#define PORT_SFF1 0
#define PORT_ENC  1
#define PORT_DEC  2
#define PORT_SFF3 3
#define TOTAL_PORTS 4
#define TOTAL_ROUTES 3
#define UDP_PORT 6633

// "AQM" / "RED" congestion limits ( either 50 % or 75 % about the queue depth )
#define THRESHOLD_HIGH 768
#define THRESHOLD_LOW 512

struct nsh_hdr {
    uint16_t base_flags_ttl_len; 
    uint8_t  md_type;
    uint8_t  next_protocol;
    uint32_t serv_path_hdr; 
} __attribute__((__packed__));

struct int_hdr { 
    double sum_x, sum_y, sum_z;         
    float min_x, min_y, min_z; 
    float max_x, max_y, max_z; 
    uint32_t active_point_count;  
    uint32_t original_point_count;        
} __attribute__((__packed__));

struct cam_hdr { 
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

struct enc_hdr {
    uint32_t packet_id;
    uint32_t frame_id;
    float global_scale;
    float box_center_x, box_center_y, box_center_z; 
    float final_scale;
    float yaw, pitch;
    float centroid_x, centroid_y, centroid_z; 
} __attribute__((__packed__));

struct point_tx {
    float x, y, z;
    uint8_t r, g, b;
    uint8_t padding;
} __attribute__((__packed__));

struct telemetry_csv {
    uint32_t frame_id;

    double camera_send_timestamp;
    double recv_start_timestamp;
    double node_exit_timestamp;

    uint32_t rx_points;            
    uint32_t tx_points;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t payload_bytes;
    uint32_t rx_media_bytes;
    uint32_t tx_media_bytes;
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

    uint32_t eth_errors;
    uint32_t ipv4_errors;
    uint32_t udp_errors;
    uint32_t nsh_errors;
    uint32_t aqm_drops;
    uint32_t tx_retries;
    uint32_t virtual_friction;
};

static struct rte_ether_addr src_macs[ TOTAL_PORTS ];
static struct rte_ether_addr dst_macs[ TOTAL_PORTS ];
static uint32_t src_ips[ TOTAL_PORTS ];
static uint32_t dst_ips[ TOTAL_PORTS ];

static uint32_t virtual_friction = 0;

uint32_t eth_errors = 0, ipv4_errors = 0, udp_errors = 0, nsh_errors = 0;

struct telemetry_csv telemetry_sff1_enc[ K_FRAMES ];
struct telemetry_csv telemetry_enc_dec[ K_FRAMES ];
struct telemetry_csv telemetry_dec_sff3[ K_FRAMES ];

uint32_t frames_sff1_enc = 0, frames_enc_dec = 0, frames_dec_sff3 = 0;
bool csv_written[ TOTAL_ROUTES ] = { false, false, false };

static inline int port_init( uint16_t port, struct rte_mempool *mbuf_pool ) {
    struct rte_eth_conf port_conf = { 0 };
    int retval;

    if ( ! rte_eth_dev_is_valid_port( port ) ) 
        return -1;

    retval = rte_eth_dev_configure( port, 1, 1, &port_conf );
    
    if ( retval != 0 ) 
        return retval;

    if ( port == PORT_ENC || port == PORT_DEC )
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

static void routing_tables_init() {
    // Port 0 ( SFF1 <-> SFF2 )
    rte_eth_macaddr_get( PORT_SFF1, &src_macs[ PORT_SFF1 ] );

    struct rte_ether_addr sff1_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x01 } };
    dst_macs[ PORT_SFF1 ] = sff1_mac;
    
    src_ips[ PORT_SFF1 ] = RTE_IPV4( 10, 0, 2, 2 );
    dst_ips[ PORT_SFF1 ] = RTE_IPV4( 10, 0, 2, 1 );

    // Port 1 ( SFF2 <-> Encoder )
    rte_eth_macaddr_get( PORT_ENC, &src_macs[ PORT_ENC ] );
    
    struct rte_ether_addr enc_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x02 } };
    dst_macs[ PORT_ENC ] = enc_mac;
    
    src_ips[ PORT_ENC ] = RTE_IPV4( 10, 0, 3, 1 );
    dst_ips[ PORT_ENC ] = RTE_IPV4( 10, 0, 3, 2 );

    // Port 2 ( SFF 2 <-> Decoder )
    rte_eth_macaddr_get( PORT_DEC, &src_macs[ PORT_DEC ] );
    
    struct rte_ether_addr dec_mac = { { 0x00, 0x00, 0x00, 0x00, 0x04, 0x02 } };
    dst_macs[ PORT_DEC ] = dec_mac;
    
    src_ips[ PORT_DEC ] = RTE_IPV4( 10, 0, 4, 1 );
    dst_ips[ PORT_DEC ] = RTE_IPV4( 10, 0, 4, 2 );

    // Port 3 (SFF 2 <-> SFF 3)
    rte_eth_macaddr_get( PORT_SFF3, &src_macs[ PORT_SFF3 ] );
    
    struct rte_ether_addr sff3_mac = { { 0x00, 0x00, 0x00, 0x00, 0x05, 0x02 } };
    dst_macs[ PORT_SFF3 ] = sff3_mac;
    
    src_ips[ PORT_SFF3 ] = RTE_IPV4( 10, 0, 5, 1 );
    dst_ips[ PORT_SFF3 ] = RTE_IPV4( 10, 0, 5, 2 );
}

static inline struct telemetry_csv *telemetry_slot( int route_id, uint32_t frame_id ) {
    if ( route_id < 0 || route_id >= TOTAL_ROUTES || frame_id == 0 || frame_id == END_OF_STREAM )
        return NULL;

    uint32_t idx = ( frame_id - 1 ) % K_FRAMES;

    if ( route_id == 0 )
        return &telemetry_sff1_enc[ idx ];

    if ( route_id == 1 )
        return &telemetry_enc_dec[ idx ];

    return &telemetry_dec_sff3[ idx ];
}

static inline void update_max_logged_frame( int route_id, uint32_t frame_id ) {
    if ( frame_id == 0 || frame_id == END_OF_STREAM )
        return;

    uint32_t bounded_id = ( frame_id > K_FRAMES ) ? K_FRAMES : frame_id;

    if ( route_id == 0 ) {
        if ( bounded_id > frames_sff1_enc )
            frames_sff1_enc = bounded_id;
    }
    else if ( route_id == 1 ) {
        if ( bounded_id > frames_enc_dec )
            frames_enc_dec = bounded_id;
    }
    else if ( route_id == 2 ) {
        if ( bounded_id > frames_dec_sff3 )
            frames_dec_sff3 = bounded_id;
    }
}

static void write_single_csv( const char *path, struct telemetry_csv *data_array, uint32_t max_logged_frames ) {
    FILE *f = fopen( path, "w" );

    if ( !f ) {
        printf( "[SYSTEM] Error: Could not open \".csv\" file for writing...\n" );
        return;
    }

    fprintf( f, "frame_id;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;rx_media_bytes;tx_media_bytes;internal_throughput_mbs;network_bitrate_mbps;tx_duration_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;camera_to_node_latency_ms;schedule_delay_ms;network_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;aqm_drops;tx_retries;virtual_friction\n" );

    for ( int i = 0; i < max_logged_frames; i++ ) {
        struct telemetry_csv *t = &data_array[ i ];

        if ( t -> frame_id == 0 )
            continue;

        fprintf( f, "%u;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u;%u;%u;%u;%u\n", t -> frame_id, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> rx_points, t -> tx_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> rx_media_bytes, t -> tx_media_bytes, t -> internal_throughput_mbs, t -> network_bitrate_mbps, t -> tx_duration_ms, t -> active_process_ms, t -> cycle_ms, t -> header_wait_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> camera_to_node_latency_ms, t -> schedule_delay_ms, t -> network_jitter_ms, t -> eth_errors, t -> ipv4_errors, t -> udp_errors, t -> nsh_errors, t -> aqm_drops, t -> tx_retries, t -> virtual_friction );
    }

    fclose( f );
}

static void telemetry_to_csv( int route_id ) {
    struct stat st = { 0 };

    if ( stat( TELEMETRY_FOLDER, &st ) == -1 )
        if ( mkdir( TELEMETRY_FOLDER, 0777 ) == -1 ) {
            printf( "[SYSTEM] Error: Failed to create directory \"%s\"...\n", TELEMETRY_FOLDER );
            return;
        }

    if ( route_id == 0 ) {
        write_single_csv( SFF1_ENCODER_PATH, telemetry_sff1_enc, frames_sff1_enc );
        printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", SFF1_ENCODER_PATH );
    }
    else if ( route_id == 1 ) {
        write_single_csv( ENCODER_DECODER_PATH, telemetry_enc_dec, frames_enc_dec );
        printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", ENCODER_DECODER_PATH );
    }
    else if ( route_id == 2 ) {
        write_single_csv( DECODER_SFF3_PATH, telemetry_dec_sff3, frames_dec_sff3 );
        printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", DECODER_SFF3_PATH );
    }
}

static inline bool flush_tx_burst( uint16_t tx_port, struct rte_mbuf **tx_bufs, uint32_t *tx_points_buf, uint32_t *tx_media_bytes_buf, int *burst_idx, struct telemetry_csv *frame_t, int route_id, uint64_t first_tx_cycle[ TOTAL_ROUTES ], uint64_t last_tx_cycle[ TOTAL_ROUTES ] ) {
    if ( *burst_idx == 0 ) 
        return false;
    
    uint16_t sent = 0; 
    uint16_t retries = 0;

    bool tx_exhausted = false;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint64_t t_tx_start = rte_get_timer_cycles();

        if ( frame_t != NULL && route_id >= 0 && route_id < TOTAL_ROUTES && first_tx_cycle[ route_id ] == 0 )
            first_tx_cycle[ route_id ] = t_tx_start;

        uint16_t nb_tx = rte_eth_tx_burst( tx_port, 0, &tx_bufs[ sent ], *burst_idx - sent );

        uint64_t t_tx_end = rte_get_timer_cycles();

        if ( nb_tx > 0 ) {
            if ( frame_t != NULL && route_id >= 0 && route_id < TOTAL_ROUTES ) {
                last_tx_cycle[ route_id ] = t_tx_end;
                frame_t -> tx_packets += nb_tx;

                for ( uint16_t j = 0; j < nb_tx; j++ ) {
                    frame_t -> tx_points += tx_points_buf[ sent + j ];
                    frame_t -> tx_media_bytes += tx_media_bytes_buf[ sent + j ];
                }
            }

            sent += nb_tx;
            retries = 0;

            if ( virtual_friction >= nb_tx )
                virtual_friction -= nb_tx;
            else
                virtual_friction = 0;
        }
        else {
            if ( frame_t != NULL )
                frame_t -> tx_retries++;

            if ( ++retries > MAX_RETRIES ) {
                for ( int k = sent; k < *burst_idx; k++ )
                    rte_pktmbuf_free( tx_bufs[ k ] );

                tx_exhausted = true;
                break;
            }

            if ( virtual_friction <= 1024 - BURST_SIZE )
                virtual_friction += BURST_SIZE;
            else
                virtual_friction = 1024;

            uint16_t pause_count = ( retries < pause_window ) ? retries : pause_window;

            for ( uint16_t p = 0; p < pause_count; p++ )
                rte_pause();
        }
    }

    *burst_idx = 0;
    return tx_exhausted;
}

static inline void flush_owned_port( uint16_t tx_port, struct rte_mbuf **tx_bufs, uint32_t *tx_points_buf, uint32_t *tx_media_bytes_buf, int *burst_idx, struct telemetry_csv **owner_t, int8_t *owner_route, uint64_t first_tx_cycle[ TOTAL_ROUTES ], uint64_t last_tx_cycle[ TOTAL_ROUTES ], uint64_t active_process_cycles[ TOTAL_ROUTES ], uint64_t frame_last_activity_cycles[ TOTAL_ROUTES ] ) {
    
    // Purpose: A physical Tx burst owns one telemetry frame at a time. This prevents primary packets & feedback data sharing ports from corrupting frame-level counters
    
    if ( *burst_idx == 0 ) {
        *owner_t = NULL;
        *owner_route = -1;
        return;
    }

    int route_id = *owner_route;
    uint64_t t_active_start = 0;

    if ( route_id >= 0 && route_id < TOTAL_ROUTES )
        t_active_start = rte_get_timer_cycles();

    bool tx_exhausted = flush_tx_burst( tx_port, tx_bufs, tx_points_buf, tx_media_bytes_buf, burst_idx, *owner_t, route_id, first_tx_cycle, last_tx_cycle );

    if ( route_id >= 0 && route_id < TOTAL_ROUTES ) {
        uint64_t t_active_end = rte_get_timer_cycles();
        active_process_cycles[ route_id ] += t_active_end - t_active_start;

        if ( tx_exhausted && t_active_end > frame_last_activity_cycles[ route_id ] )
            frame_last_activity_cycles[ route_id ] = t_active_end;
    }

    *owner_t = NULL;
    *owner_route = -1;
}

static inline void finalize_frame_metrics( int route_id, uint64_t timer_hz, uint64_t t_frame_arrival[ TOTAL_ROUTES ], uint64_t t_session_start[ TOTAL_ROUTES ], uint32_t first_arrival_f_id[ TOTAL_ROUTES ], uint32_t current_frame_id[ TOTAL_ROUTES ], uint16_t frame_temporal_skip[ TOTAL_ROUTES ], uint64_t first_tx_cycle[ TOTAL_ROUTES ], uint64_t last_tx_cycle[ TOTAL_ROUTES ], uint64_t frame_last_activity_cycles[ TOTAL_ROUTES ], uint64_t active_process_cycles[ TOTAL_ROUTES ], uint64_t t_cycle_start[ TOTAL_ROUTES ], double current_latency_ms[ TOTAL_ROUTES ], double current_jitter_ms[ TOTAL_ROUTES ] ) {
    uint32_t frame_id = current_frame_id[ route_id ];

    if ( frame_id == END_OF_STREAM || frame_id == 0 )
        return;

    struct telemetry_csv *t = telemetry_slot( route_id, frame_id );

    if ( t == NULL || t -> frame_id != frame_id )
        return;

    uint64_t t_end = last_tx_cycle[ route_id ];

    if ( frame_last_activity_cycles[ route_id ] > t_end )
        t_end = frame_last_activity_cycles[ route_id ];

    if ( t_end == 0 )
        t_end = t_frame_arrival[ route_id ];

    double tx_duration_sec = 0.0;

    if ( first_tx_cycle[ route_id ] > 0 && last_tx_cycle[ route_id ] >= first_tx_cycle[ route_id ] )
        tx_duration_sec = ( double )( last_tx_cycle[ route_id ] - first_tx_cycle[ route_id ] ) / timer_hz;

    double residency_sec = 0.0;

    if ( t_end >= t_frame_arrival[ route_id ] )
        residency_sec = ( double )( t_end - t_frame_arrival[ route_id ] ) / timer_hz;

    double cycle_sec = 0.0;

    if ( t_end >= t_cycle_start[ route_id ] )
        cycle_sec = ( double )( t_end - t_cycle_start[ route_id ] ) / timer_hz;

    double header_wait_sec = ( cycle_sec > residency_sec ) ? cycle_sec - residency_sec : 0.0;
    double active_process_sec = ( double )active_process_cycles[ route_id ] / timer_hz;

    uint64_t logical_rx_payload_bytes = 0;
    uint64_t logical_tx_payload_bytes = 0;
    uint64_t logical_rx_frame_bytes = 0;
    uint64_t logical_tx_frame_bytes = 0;

    if ( route_id == 0 ) {
        // SFF1 -> Encoder: point cloud + cam_hdr.
        logical_rx_payload_bytes = ( uint64_t )t -> rx_points * sizeof( struct point_tx );
        logical_tx_payload_bytes = ( uint64_t )t -> tx_points * sizeof( struct point_tx );

        logical_rx_frame_bytes = logical_rx_payload_bytes + ( t -> rx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );
        logical_tx_frame_bytes = logical_tx_payload_bytes + ( t -> tx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );
    }
    else if ( route_id == 1 ) {
        // Encoder -> Decoder: media + cam_hdr + enc_hdr.
        logical_rx_payload_bytes = t -> rx_media_bytes;
        logical_tx_payload_bytes = t -> tx_media_bytes;

        logical_rx_frame_bytes = logical_rx_payload_bytes + ( t -> rx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );
        logical_tx_frame_bytes = logical_tx_payload_bytes + ( t -> tx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );
    }
    else {
        // [ TO BE DEFINED ]
    }

    t -> payload_bytes = ( uint32_t )logical_rx_payload_bytes;
    t -> tx_duration_ms = tx_duration_sec * 1000.0;
    t -> active_process_ms = active_process_sec * 1000.0;
    t -> total_residency_ms = residency_sec * 1000.0;
    t -> cycle_ms = cycle_sec * 1000.0;
    t -> header_wait_ms = header_wait_sec * 1000.0;
    t -> node_exit_timestamp = ( double )t_end / timer_hz;
    t -> camera_to_node_latency_ms = current_latency_ms[ route_id ];

    uint32_t schedule_frame_offset = frame_id - first_arrival_f_id[ route_id ];
    double real_elapsed_sec = 0.0;

    if ( t_end >= t_session_start[ route_id ] )
        real_elapsed_sec = ( double )( t_end - t_session_start[ route_id ] ) / timer_hz;

    double ideal_elapsed_sec = ( double )schedule_frame_offset / TARGET_FPS;
    t -> schedule_delay_ms = ( real_elapsed_sec - ideal_elapsed_sec ) * 1000.0;
    t -> network_jitter_ms = current_jitter_ms[ route_id ];

    t -> internal_throughput_mbs = ( residency_sec > 0.0 ) ? ( ( double )logical_rx_frame_bytes / 1000000.0 ) / residency_sec : 0.0;

    uint16_t temporal_skip = frame_temporal_skip[ route_id ];

    if ( temporal_skip == 0 )
        temporal_skip = 1;

    double effective_fps = TARGET_FPS / temporal_skip;

    t -> network_bitrate_mbps = ( logical_tx_frame_bytes * 8.0 * effective_fps ) / 1000000.0;

    t -> node_efficiency_pct = ( cycle_sec > 0.0 ) ? ( residency_sec / cycle_sec ) * 100.0 : 0.0;

    t -> virtual_friction = virtual_friction;
    t_cycle_start[ route_id ] = t_end;
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();

    struct rte_mbuf *bufs[ BURST_SIZE ];

    struct rte_mbuf *tx_bufs[ TOTAL_PORTS ][ BURST_SIZE ];
    uint32_t tx_points_buf[ TOTAL_PORTS ][ BURST_SIZE ];
    uint32_t tx_media_bytes_buf[ TOTAL_PORTS ][ BURST_SIZE ];

    int burst_idx[ TOTAL_PORTS ] = { 0 };

    struct telemetry_csv *burst_owner_t[ TOTAL_PORTS ] = { NULL, NULL, NULL, NULL };
    int8_t burst_owner_route[ TOTAL_PORTS ] = { -1, -1, -1, -1 };

    uint32_t current_frame_id[ TOTAL_ROUTES ] = { END_OF_STREAM, END_OF_STREAM, END_OF_STREAM };
    uint32_t frame_original_points[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint16_t frame_temporal_skip[ TOTAL_ROUTES ] = { 1, 1, 1 };

    uint64_t t_frame_arrival[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t t_session_start[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t frames_received_count[ TOTAL_ROUTES ] = { 0, 0, 0 };

    double current_latency_ms[ TOTAL_ROUTES ] = { 0.0, 0.0, 0.0 };
    double current_jitter_ms[ TOTAL_ROUTES ] = { 0.0, 0.0, 0.0 };

    uint64_t prev_arrival_cyc[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t first_arrival_f_id[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t prev_arrival_f_id[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t curr_f_delta[ TOTAL_ROUTES ] = { 1, 1, 1 };

    uint64_t first_tx_cycle[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t last_tx_cycle[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t frame_last_activity_cycles[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t active_process_cycles[ TOTAL_ROUTES ] = { 0, 0, 0 };

    uint64_t worker_start_cycle = rte_get_timer_cycles();
    uint64_t t_cycle_start[ TOTAL_ROUTES ] = { worker_start_cycle, worker_start_cycle, worker_start_cycle };

    const size_t outer_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

    printf( "[SYSTEM] Listening on \"UDP\" port %u...\n\n", UDP_PORT );

    while ( 1 ) {
        uint8_t drop_probability = 0;

        if ( virtual_friction >= THRESHOLD_HIGH )
            drop_probability = 100;
        else if ( virtual_friction > THRESHOLD_LOW )
            drop_probability = ( ( virtual_friction - THRESHOLD_LOW ) * 100 ) / ( THRESHOLD_HIGH - THRESHOLD_LOW );

        for ( uint16_t rx_port = 0; rx_port < TOTAL_PORTS; rx_port++ ) {
            uint16_t nb_rx = rte_eth_rx_burst( rx_port, 0, bufs, BURST_SIZE );

            if ( unlikely( nb_rx == 0 ) ) {
                for ( uint16_t q = 0; q < TOTAL_PORTS; q++ ) {
                    if ( burst_idx[ q ] == 0 )
                        continue;

                    flush_owned_port( q, tx_bufs[ q ], tx_points_buf[ q ], tx_media_bytes_buf[ q ], &burst_idx[ q ], &burst_owner_t[ q ], &burst_owner_route[ q ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );
                }

                continue;
            }

            for ( uint16_t i = 0; i < nb_rx; i++ ) {
                struct rte_mbuf *m = bufs[ i ];

                size_t min_net_req = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr );

                if ( unlikely( m -> data_len < min_net_req ) ) {
                    rte_pktmbuf_free( m );
                    udp_errors++;
                    continue;
                }

                struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

                if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
                    rte_pktmbuf_free( m );
                    eth_errors++;
                    continue;
                }

                struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

                if ( unlikely( ipv4 -> next_proto_id != IPPROTO_UDP ) ) {
                    rte_pktmbuf_free( m );
                    ipv4_errors++;
                    continue;
                }

                struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

                if ( unlikely( udp -> dst_port != rte_cpu_to_be_16( UDP_PORT ) ) ) {
                    rte_pktmbuf_free( m );
                    udp_errors++;
                    continue;
                }

                struct nsh_hdr *nsh = ( struct nsh_hdr * )( udp + 1 );
                uint32_t sph = ntohl( nsh -> serv_path_hdr );
                uint32_t spi = sph >> 8;
                uint8_t si = sph & 0xFF;

                uint16_t target_tx_port = TOTAL_PORTS;

                if ( spi == PRIMARY_SPI ) {
                    if ( si == 255 && rx_port == PORT_SFF1 )
                        target_tx_port = PORT_ENC;
                    else if ( si == 253 && rx_port == PORT_ENC )
                        target_tx_port = PORT_DEC;
                    else if ( si == 251 && rx_port == PORT_DEC )
                        target_tx_port = PORT_SFF3;
                }
                else if ( spi == FEEDBACK_SPI ) {
                    if ( si == 255 && rx_port == PORT_SFF3 )
                        target_tx_port = PORT_ENC;
                    else if ( si == 253 && rx_port == PORT_ENC )
                        target_tx_port = PORT_SFF1;
                }

                if ( unlikely( target_tx_port == TOTAL_PORTS ) ) {
                    rte_pktmbuf_free( m );
                    nsh_errors++;
                    continue;
                }

                int route_id = -1;

                if ( spi == PRIMARY_SPI ) {
                    if ( rx_port == PORT_SFF1 && target_tx_port == PORT_ENC )
                        route_id = 0;
                    else if ( rx_port == PORT_ENC && target_tx_port == PORT_DEC )
                        route_id = 1;
                    else if ( rx_port == PORT_DEC && target_tx_port == PORT_SFF3 )
                        route_id = 2;
                }

                if ( spi == FEEDBACK_SPI ) {
                    if ( unlikely( si == 0 ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors++;
                        continue;
                    }

                    if ( burst_idx[ target_tx_port ] > 0 )
                        flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                    si--;
                    nsh -> serv_path_hdr = htonl( ( spi << 8 ) | si );

                    rte_ether_addr_copy( &src_macs[ target_tx_port ], &eth -> src_addr );
                    rte_ether_addr_copy( &dst_macs[ target_tx_port ], &eth -> dst_addr );

                    ipv4 -> src_addr = rte_cpu_to_be_32( src_ips[ target_tx_port ] );
                    ipv4 -> dst_addr = rte_cpu_to_be_32( dst_ips[ target_tx_port ] );
                    ipv4 -> hdr_checksum = 0;
                    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );

                    udp -> src_port = rte_cpu_to_be_16( UDP_PORT );
                    udp -> dst_port = rte_cpu_to_be_16( UDP_PORT );

                    udp -> dgram_cksum = 0;

                    if ( burst_idx[ target_tx_port ] == 0 ) {
                        burst_owner_t[ target_tx_port ] = NULL;
                        burst_owner_route[ target_tx_port ] = -1;
                    }

                    int queue_idx = burst_idx[ target_tx_port ];

                    tx_bufs[ target_tx_port ][ queue_idx ] = m;
                    tx_points_buf[ target_tx_port ][ queue_idx ] = 0;
                    tx_media_bytes_buf[ target_tx_port ][ queue_idx ] = 0;

                    burst_idx[ target_tx_port ]++;

                    if ( burst_idx[ target_tx_port ] == BURST_SIZE )
                        flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                    continue;
                }

                uint64_t packet_arrival_cycles = rte_get_timer_cycles();

                struct cam_hdr *cam = NULL;
                uint32_t f_id = 0;

                if ( route_id == 0 ) {
                    size_t required_len = outer_len + sizeof( struct nsh_hdr ) + sizeof( struct int_hdr ) + sizeof( struct cam_hdr );

                    if ( unlikely( m -> data_len < required_len ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors++;
                        continue;
                    }

                    cam = ( struct cam_hdr * )( ( char * )nsh + sizeof( struct nsh_hdr ) + sizeof( struct int_hdr ) );
                    f_id = ntohl( cam -> frame_id );
                }
                else if ( route_id == 1 ) {
                    size_t cam_required_len = outer_len + sizeof( struct nsh_hdr ) + sizeof( struct cam_hdr );

                    if ( unlikely( m -> data_len < cam_required_len ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors++;
                        continue;
                    }

                    cam = ( struct cam_hdr * )( ( char * )nsh + sizeof( struct nsh_hdr ) );
                    f_id = ntohl( cam -> frame_id );

                    if ( f_id != END_OF_STREAM ) {
                        size_t normal_required_len = cam_required_len + sizeof( struct enc_hdr );

                        if ( unlikely( m -> data_len < normal_required_len ) ) {
                            rte_pktmbuf_free( m );
                            nsh_errors++;
                            continue;
                        }
                    }
                }
                else if ( route_id == 2 ) {
                    // [ TO BE DEFINED ]
                }

                if ( f_id == 0 ) {
                    rte_pktmbuf_free( m );
                    nsh_errors++;
                    continue;
                }

                bool is_eos_packet = f_id == END_OF_STREAM;
                bool trigger_eos_write = false;

                if ( f_id != current_frame_id[ route_id ] ) {
                    if ( current_frame_id[ route_id ] != END_OF_STREAM ) {
                        if ( burst_idx[ target_tx_port ] > 0 )
                            flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                        finalize_frame_metrics( route_id, timer_hz, t_frame_arrival, t_session_start, first_arrival_f_id, current_frame_id, frame_temporal_skip, first_tx_cycle, last_tx_cycle, frame_last_activity_cycles, active_process_cycles, t_cycle_start, current_latency_ms, current_jitter_ms );
                    }

                    if ( is_eos_packet ) {
                        bool has_logs = ( route_id == 0 && frames_sff1_enc > 0 ) || ( route_id == 1 && frames_enc_dec > 0 ) || ( route_id == 2 && frames_dec_sff3 > 0 );

                        trigger_eos_write = has_logs && !csv_written[ route_id ];

                        current_frame_id[ route_id ] = END_OF_STREAM;
                    }
                    else {
                        uint64_t cam_tx_cycles = rte_be_to_cpu_64( cam -> timestamp );
                        uint64_t arrival_cycles = packet_arrival_cycles;

                        if ( arrival_cycles >= cam_tx_cycles )
                            current_latency_ms[ route_id ] = ( ( double )( arrival_cycles - cam_tx_cycles ) / timer_hz ) * 1000.0;
                        else
                            current_latency_ms[ route_id ] = 0.0;

                        if ( frames_received_count[ route_id ] == 0 ) {
                            t_session_start[ route_id ] = arrival_cycles;
                            first_arrival_f_id[ route_id ] = f_id;
                            t_cycle_start[ route_id ] = arrival_cycles;
                        }

                        uint32_t new_frame_delta = 1;

                        if ( prev_arrival_f_id[ route_id ] > 0 && f_id > prev_arrival_f_id[ route_id ] )
                            new_frame_delta = f_id - prev_arrival_f_id[ route_id ];

                        curr_f_delta[ route_id ] = new_frame_delta;

                        if ( frames_received_count[ route_id ] > 0 && prev_arrival_f_id[ route_id ] > 0 && f_id > prev_arrival_f_id[ route_id ] ) {
                            double real_interval_sec = ( double )( arrival_cycles - prev_arrival_cyc[ route_id ] ) / timer_hz;
                            double expected_interval_sec = ( double )curr_f_delta[ route_id ] / TARGET_FPS;
                            double diff = real_interval_sec - expected_interval_sec;

                            current_jitter_ms[ route_id ] = ( diff < 0.0 ) ? -diff * 1000.0 : diff * 1000.0;
                        }
                        else
                            current_jitter_ms[ route_id ] = 0.0;

                        prev_arrival_cyc[ route_id ] = arrival_cycles;
                        prev_arrival_f_id[ route_id ] = f_id;
                        frames_received_count[ route_id ]++;

                        current_frame_id[ route_id ] = f_id;
                        t_frame_arrival[ route_id ] = arrival_cycles;
                        first_tx_cycle[ route_id ] = 0;
                        last_tx_cycle[ route_id ] = 0;
                        frame_last_activity_cycles[ route_id ] = 0;
                        active_process_cycles[ route_id ] = 0;
                        frame_original_points[ route_id ] = 0;

                        uint16_t temporal_skip = ntohs( cam -> temporal_skip );

                        if ( temporal_skip == 0 )
                            temporal_skip = 1;

                        frame_temporal_skip[ route_id ] = temporal_skip;

                        struct telemetry_csv *new_t = telemetry_slot( route_id, f_id );

                        if ( new_t != NULL ) {
                            memset( new_t, 0, sizeof( *new_t ) );

                            new_t -> frame_id = f_id;
                            new_t -> camera_send_timestamp = ( double )cam_tx_cycles / timer_hz;
                            new_t -> recv_start_timestamp = ( double )arrival_cycles / timer_hz;

                            new_t -> eth_errors = eth_errors;
                            new_t -> ipv4_errors = ipv4_errors;
                            new_t -> udp_errors = udp_errors;
                            new_t -> nsh_errors = nsh_errors;

                            update_max_logged_frame( route_id, f_id );
                        }
                    }
                }

                struct telemetry_csv *t = NULL;
                int packet_metric_route = -1;

                if ( !is_eos_packet && f_id > 0 ) {
                    t = telemetry_slot( route_id, f_id );

                    if ( t != NULL && t -> frame_id == f_id )
                        packet_metric_route = route_id;
                }

                if ( burst_idx[ target_tx_port ] > 0 && ( burst_owner_t[ target_tx_port ] != t || burst_owner_route[ target_tx_port ] != packet_metric_route ) )
                    flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                uint64_t t_active_process_start = 0;
                bool measure_active_process = packet_metric_route >= 0;

                if ( measure_active_process )
                    t_active_process_start = rte_get_timer_cycles();

                uint32_t current_pkt_points = 0;
                uint32_t current_media_bytes = 0;
                bool frame_complete = false;

                if ( t != NULL ) {
                    t -> rx_packets++;

                    if ( route_id == 0 ) {
                        current_pkt_points = ntohl( cam -> points_in_packet );
                        t -> rx_points += current_pkt_points;

                        frame_original_points[ route_id ] = ntohl( cam -> original_points );
                        frame_complete = frame_original_points[ route_id ] > 0 && t -> rx_points >= frame_original_points[ route_id ];
                    }
                    else if ( route_id == 1 ) {
                        size_t media_offset = outer_len + sizeof( struct nsh_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr );

                        if ( m -> pkt_len > media_offset )
                            current_media_bytes = ( uint32_t )( m -> pkt_len - media_offset );

                        t -> rx_media_bytes += current_media_bytes;
                    }
                }

                if ( spi == PRIMARY_SPI && !is_eos_packet && drop_probability > 0 && ( rte_rand() % 100 ) < drop_probability ) {
                    rte_pktmbuf_free( m );

                    if ( t != NULL )
                        t -> aqm_drops++;

                    if ( measure_active_process ) {
                        uint64_t t_active_process_end = rte_get_timer_cycles();
                        active_process_cycles[ route_id ] += t_active_process_end - t_active_process_start;
                        frame_last_activity_cycles[ route_id ] = t_active_process_end;
                    }

                    if ( route_id == 0 && frame_complete && burst_idx[ target_tx_port ] > 0 )
                        flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                    continue;
                }

                if ( unlikely( si == 0 ) ) {
                    rte_pktmbuf_free( m );
                    nsh_errors++;

                    if ( measure_active_process ) {
                        uint64_t t_active_process_end = rte_get_timer_cycles();
                        active_process_cycles[ route_id ] += t_active_process_end - t_active_process_start;
                        frame_last_activity_cycles[ route_id ] = t_active_process_end;
                    }

                    if ( route_id == 0 && frame_complete && burst_idx[ target_tx_port ] > 0 )
                        flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                    continue;
                }

                si--;
                nsh -> serv_path_hdr = htonl( ( spi << 8 ) | si );

                rte_ether_addr_copy( &src_macs[ target_tx_port ], &eth -> src_addr );
                rte_ether_addr_copy( &dst_macs[ target_tx_port ], &eth -> dst_addr );

                ipv4 -> src_addr = rte_cpu_to_be_32( src_ips[ target_tx_port ] );
                ipv4 -> dst_addr = rte_cpu_to_be_32( dst_ips[ target_tx_port ] );
                ipv4 -> hdr_checksum = 0;
                ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );
                
                udp -> src_port = rte_cpu_to_be_16( UDP_PORT );
                udp -> dst_port = rte_cpu_to_be_16( UDP_PORT );
                
                udp -> dgram_cksum = 0;

                if ( burst_idx[ target_tx_port ] == 0 ) {
                    burst_owner_t[ target_tx_port ] = t;
                    burst_owner_route[ target_tx_port ] = packet_metric_route;
                }

                int queue_idx = burst_idx[ target_tx_port ];

                tx_bufs[ target_tx_port ][ queue_idx ] = m;
                tx_points_buf[ target_tx_port ][ queue_idx ] = ( t != NULL && route_id == 0 ) ? current_pkt_points : 0;
                tx_media_bytes_buf[ target_tx_port ][ queue_idx ] = ( t != NULL && route_id == 1 ) ? current_media_bytes : 0;

                burst_idx[ target_tx_port ]++;

                if ( measure_active_process ) {
                    uint64_t t_packet_process_end = rte_get_timer_cycles();
                    active_process_cycles[ route_id ] += t_packet_process_end - t_active_process_start;
                    frame_last_activity_cycles[ route_id ] = t_packet_process_end;
                }

                bool force_flush = burst_idx[ target_tx_port ] == BURST_SIZE;

                if ( route_id == 0 && frame_complete )
                    force_flush = true;

                if ( is_eos_packet )
                    force_flush = true;

                if ( force_flush )
                    flush_owned_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_process_cycles, frame_last_activity_cycles );

                if ( trigger_eos_write ) {
                    telemetry_to_csv( route_id );
                    csv_written[ route_id ] = true;

                    const char *route = ( route_id == 0 ) ? "\"SFF1\" -> \"Encoder\"" : ( route_id == 1 ) ? "\"Encoder\" -> \"Decoder\"" : "\"Decoder\" -> \"SFF3\"";

                    printf( "\n[SYSTEM] Chain: \"Primary\", Route: %s. End of stream detected. Changing to \"idle\" plight...\n\n", route );
                }

                if ( spi == FEEDBACK_SPI ) {
                    const char *route = ( rx_port == PORT_SFF3 ) ? "\"SFF3\" -> \"Encoder\"" : "\"Encoder\" -> \"SFF1\"";
                    printf( "[SYSTEM] Chain: \"Feedback\", Route: %s, Control packets flowing...\n", route );
                }
            }
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It implements an "NSH"-based routing by decrementing the "Service Index" ( "SI" ) & handles a quality adaptation mechanism ( "Packet Washing" ) to prevent heavy drop-tails downstream

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n");

    printf( "[SYSTEM] Booting the \"SFF2\" microservice...\n" );

    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );
    
    if ( mbuf_pool == NULL ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    for ( uint16_t p = 0; p < TOTAL_PORTS; p++ ) {
        if ( port_init( p, mbuf_pool ) != 0 )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Virtual port %u configuration failed...\n", p );
    }

    printf( "\n" );

    routing_tables_init();

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