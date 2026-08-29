#include <float.h>
#include <math.h>
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

// Runtime & experimental configuration variables
#define TELEMETRY_FOLDER "/shared/log/sff1"
#define TELEMETRY_PATH "/shared/log/sff1/telemetry_sff1.csv"

#define K_FRAMES 300

#define TARGET_FPS 30.0

#define BURST_SIZE 32
#define MAX_ZERO_ACCEPTS 2048

#define END_OF_STREAM 0xFFFFFFFF

// "DPDK" packet-buffer pool settings
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// "Service Function Path" ( "SFP" ) identifiers & state values
#define MAIN_SPI 100
#define MAIN_SI 255

#define TEMPORAL_SPI 200
#define TEMPORAL_SI 255

// Sending bonds & networking parameters
#define PORT_CAMERA 0
#define PORT_SFF2 1

#define CAMERA_IP RTE_IPV4( 10, 0, 1, 1 )
#define SFF1_CAMERA_IP RTE_IPV4( 10, 0, 1, 254 )
#define CAMERA_PORT 5001
#define SFF1_CAMERA_PORT 6633

#define SFF1_SFF2_IP RTE_IPV4( 10, 0, 2, 1 )
#define SFF2_SFF1_IP RTE_IPV4( 10, 0, 2, 2 )
#define SFF1_SFF2_PORT 6633
#define SFF2_SFF1_PORT 6633

// Packetization & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80
#define MAX_FRAME_POINTS 835458

#define CAMERA_DISTANCE 1200.0f
#define WIDTH 640
#define HEIGHT 480
#define PADDING 0.10f

#define MD_CLASS_EXPERIMENTAL 0xFFF6
#define MD_TYPE_GEOMETRY 0x01
#define MD_TYPE_2 0x02
#define NEXT_PROTOCOL_EXPERIMENT_1 0xFE
#define DEFAULT_TTL 63

// Wire-format structures utilized by the "DPDK" data path
struct nsh_hdr {
    uint16_t base_flags_ttl_len;
    uint8_t md_type;
    uint8_t next_protocol;
    uint32_t serv_path_hdr; // 24-bit "Service Path Identifier" ( "SPI" ) followed by the 8-bit "Service Index" ( "SI" )
} __attribute__((__packed__));

struct nsh_md2_ctx_hdr {
    uint16_t metadata_class;
    uint8_t type;
    uint8_t u_length;
} __attribute__((__packed__));

// Progressive geometry information exported via the "MD-Type-2" context, representing a "computational offloading" strategy. 
// For the active point set, "C = ( 1 / N ) * sum_i( p_i )", "E = p_max - p_min", "B = ( p_min + p_max ) / 2". 
// Middle packets carry the summary while frame-completing elements include "max_r = max_i || p_i - C ||_2" for downstream employment ( alongside remaining projection metadata )
struct geo_agg_hdr { 
    uint32_t centroid_x;
    uint32_t centroid_y;
    uint32_t centroid_z;

    uint32_t extent_x;
    uint32_t extent_y;
    uint32_t extent_z;

    uint32_t bbox_center_x;
    uint32_t bbox_center_y;
    uint32_t bbox_center_z;

    uint32_t max_r; // zero on intermediate packets / exact farthest-point radius otherwise

    uint32_t final_scale;
    uint32_t global_scale;
    uint32_t projected_bbox_x;
    uint32_t projected_bbox_y;
    uint32_t projected_bbox_z;

    uint32_t active_point_count;
} __attribute__((__packed__));

struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__, __aligned__(2)));

// Outer encapsulation header transmitted by SFF1 to SFF2
struct main_hdr {
    struct net_hdr net;
    struct nsh_hdr nsh;
    struct nsh_md2_ctx_hdr geo_ctx;
    struct geo_agg_hdr geo;
} __attribute__((__packed__, __aligned__(2)));

#define NSH_TOTAL_SIZE ( sizeof( struct nsh_hdr ) + sizeof( struct nsh_md2_ctx_hdr ) + sizeof( struct geo_agg_hdr ) )
#define NSH_LENGTH_WORDS ( NSH_TOTAL_SIZE / 4 )

struct cam_hdr { 
    uint32_t frame_id;
    uint32_t sequence_number;
    uint64_t timestamp;
    uint32_t yaw;
    uint32_t pitch;
    uint32_t zoom;
    uint16_t temporal_skip;
    uint32_t original_points;
    uint32_t points_in_packet;
    uint16_t padding;
} __attribute__((__packed__));

struct point_tx {
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t padding;
} __attribute__((__packed__));

// Reusable shot-local "XYZ" workspace facilitating the secondary pass for "max_r" computation
struct geometry_point {
    float x;
    float y;
    float z;
};

struct worker_context {
    struct geometry_point *frame_geometry_points;
    size_t frame_geometry_capacity;
};

struct temporal_payload {
    uint32_t frame_id; 
    uint64_t timestamp; 
    uint16_t skip; 
    uint16_t padding;
} __attribute__((__packed__));

// Frame & diagnostic content abstractions
struct telemetry_csv {
    uint32_t frame_id;

    uint8_t rx_complete;
    uint8_t tx_complete;

    uint16_t current_skip;

    double camera_send_timestamp;
    double recv_start_timestamp;
    double node_exit_timestamp;

    uint32_t original_points;
    uint32_t rx_points;
    uint32_t tx_points;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t payload_bytes;

    uint32_t reference_size_bytes;

    double data_integrity_pct;
    double internal_throughput_mbs;

    double reference_throughput_mbs;

    double logical_bitrate_mbps;
    double network_bitrate_mbps;

    double reference_bitrate_mbps;

    double tx_duration_ms;
    double active_tx_ms;
    double active_process_ms;
    double geometry_aggregation_ms;
    double max_r_ms;
    double cycle_ms;
    double header_wait_ms;
    double total_residency_ms;
    double node_efficiency_pct;

    double reference_efficiency_pct;

    double camera_node_ms;
    double schedule_delay_ms;
    double instant_jitter_ms;
    double desynced_jitter_ms;

    uint32_t eth_errors;
    uint32_t ipv4_errors;
    uint32_t udp_errors;
    uint32_t nsh_errors;

    uint32_t tx_zero_accepts;
    uint32_t tx_partial_accepts;
    uint32_t tx_resubmit_calls;
    uint32_t tx_resubmitted_packets;
};

// Global application state
struct main_hdr main_template_hdr;
struct net_hdr temporal_template_hdr;

static const struct rte_ether_addr camera_mac = { { 0x00, 0x00, 0x00, 0x00, 0x01, 0x01 } };
static const struct rte_ether_addr sff1_camera_mac = { { 0x00, 0x00, 0x00, 0x00, 0x01, 0x02 } };

static const struct rte_ether_addr sff1_sff2_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x01 } };
static const struct rte_ether_addr sff2_sff1_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x02 } };

struct telemetry_csv telemetry_log[ K_FRAMES ];
int logged_frames = 0;
bool csv_written = false;

static uint32_t eth_errors = 0;
static uint32_t ipv4_errors = 0;
static uint32_t udp_errors = 0;
static uint32_t nsh_errors = 0;

// Data path & support routines
static inline uint32_t float_to_be( float value ) {
    uint32_t bits;

    memcpy( &bits, &value, sizeof( bits ) );

    return rte_cpu_to_be_32( bits );
}

static inline float be_to_float( uint32_t value ) {

    // Purpose: It converts a network-order 32-bit word back to its original "IEEE-754" single-precision bit pattern before geometric aggregation

    uint32_t bits = rte_be_to_cpu_32( value );
    float result;

    memcpy( &result, &bits, sizeof( result ) );

    return result;
}

static inline uint16_t nsh_base_field( uint8_t ttl, uint8_t length_words ) {

    // Purpose: It encodes the "NSH" base "Time-to-Live" ( "TTL" ) & "Length" fields, where the extent is defined in 4-byte components as "base = ( TTL << 6 ) | Length"

    uint16_t value = ( ( uint16_t )( ttl & 0x3F ) << 6 ) | ( uint16_t )( length_words & 0x3F );

    return rte_cpu_to_be_16( value );
}

static inline uint16_t nsh_length_bytes( struct nsh_hdr *nsh ) {

    // Purpose: It decodes the 6-bit "NSH" "Length" field & returns the complete wrapping size in bytes as "4 * Length"

    uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );
    uint8_t length_words = base & 0x3F;

    return ( uint16_t )length_words * 4;
}

static inline float calculate_maximum_radius( const struct geometry_point *points, uint32_t point_count, float centroid_x, float centroid_y, float centroid_z ) {

    // Purpose: It follows the reference numerical order more closely by
    //          evaluating sqrt() per point before selecting the maximum.

    float max_r = 0.0f;

    for ( uint32_t point = 0; point < point_count; point++ ) {
        float dx = points[ point ].x - centroid_x;
        float dy = points[ point ].y - centroid_y;
        float dz = points[ point ].z - centroid_z;

        float dist = sqrtf( ( dx * dx ) + ( dy * dy ) + ( dz * dz ) );

        if ( dist > max_r )
            max_r = dist;
    }

    return max_r;
}

static inline void calculate_projection_metadata( const struct geometry_point *points, uint32_t point_count, float centroid_x, float centroid_y, float centroid_z, float max_r, float *final_scale_out, float *global_scale_out, float *center_x_out, float *center_y_out, float *center_z_out ) {

    // Purpose: It derives the frame-complete projection metadata from the offloaded geometry, reproducing the reference numerical sequence through explicit point-wise normalization, transformed spatial frontiers & final scale evaluation

    float final_scale = 1.0f;

    if ( isfinite( max_r ) && max_r > 0.0f )
        final_scale = ( CAMERA_DISTANCE * 0.2f ) / max_r;

    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float min_z = FLT_MAX;
    float max_x = -FLT_MAX;
    float max_y = -FLT_MAX;
    float max_z = -FLT_MAX;

    for ( uint32_t point = 0; point < point_count; point++ ) {
        float tx = ( points[ point ].x - centroid_x ) * final_scale;
        float ty = ( points[ point ].y - centroid_y ) * final_scale;
        float tz = ( points[ point ].z - centroid_z ) * final_scale + CAMERA_DISTANCE;

        if ( tx < min_x ) min_x = tx;
        if ( tx > max_x ) max_x = tx;
        if ( ty < min_y ) min_y = ty;
        if ( ty > max_y ) max_y = ty;
        if ( tz < min_z ) min_z = tz;
        if ( tz > max_z ) max_z = tz;
    }

    float extent_x = ( point_count > 0 ) ? max_x - min_x : 0.0f;
    float extent_y = ( point_count > 0 ) ? max_y - min_y : 0.0f;
    float extent_z = ( point_count > 0 ) ? max_z - min_z : 0.0f;

    float bbox_center_x = ( point_count > 0 ) ? ( min_x + max_x ) * 0.5f : 0.0f;
    float bbox_center_y = ( point_count > 0 ) ? ( min_y + max_y ) * 0.5f : 0.0f;
    float bbox_center_z = ( point_count > 0 ) ? ( min_z + max_z ) * 0.5f : 0.0f;

    float scale_x = extent_x / ( float )WIDTH;
    float scale_y = extent_y / ( float )HEIGHT;
    float scale_z = extent_z / ( float )WIDTH;
    float global_scale = fmaxf( fmaxf( scale_x, scale_y ), scale_z ) * ( 1.0f + PADDING );

    if ( !isfinite( global_scale ) || global_scale <= 0.0f )
        global_scale = 1.0f;

    *final_scale_out = final_scale;
    *global_scale_out = global_scale;
    *center_x_out = bbox_center_x;
    *center_y_out = bbox_center_y;
    *center_z_out = bbox_center_z;
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

static void main_header_init( struct main_hdr *hdr ) {
    memset( hdr, 0, sizeof( struct main_hdr ) );

    rte_memcpy( &hdr -> net.ethernet.src_addr, &sff1_sff2_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> net.ethernet.dst_addr, &sff2_sff1_mac, RTE_ETHER_ADDR_LEN );

    hdr -> net.ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> net.ipv4.version_ihl = 0x45;
    hdr -> net.ipv4.time_to_live = 64;
    hdr -> net.ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> net.ipv4.src_addr = rte_cpu_to_be_32( SFF1_SFF2_IP );
    hdr -> net.ipv4.dst_addr = rte_cpu_to_be_32( SFF2_SFF1_IP );

    hdr -> net.udp.src_port = rte_cpu_to_be_16( SFF1_SFF2_PORT );
    hdr -> net.udp.dst_port = rte_cpu_to_be_16( SFF2_SFF1_PORT );
    hdr -> net.udp.dgram_cksum = 0;

    hdr -> nsh.base_flags_ttl_len = nsh_base_field( DEFAULT_TTL, NSH_LENGTH_WORDS );
    hdr -> nsh.md_type = MD_TYPE_2;
    hdr -> nsh.next_protocol = NEXT_PROTOCOL_EXPERIMENT_1;
    hdr -> nsh.serv_path_hdr = htonl( ( MAIN_SPI << 8 ) | MAIN_SI );

    hdr -> geo_ctx.metadata_class = rte_cpu_to_be_16( MD_CLASS_EXPERIMENTAL );
    hdr -> geo_ctx.type = MD_TYPE_GEOMETRY;
    hdr -> geo_ctx.u_length = ( uint8_t )( sizeof( struct geo_agg_hdr ) & 0x7F );
}

static void temporal_header_init( struct net_hdr *hdr ) {
    memset( hdr, 0, sizeof( struct net_hdr ) );

    rte_memcpy( &hdr -> ethernet.src_addr, &sff1_camera_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> ethernet.dst_addr, &camera_mac, RTE_ETHER_ADDR_LEN );

    hdr -> ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> ipv4.version_ihl = 0x45;
    hdr -> ipv4.time_to_live = 64;
    hdr -> ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> ipv4.src_addr = rte_cpu_to_be_32( SFF1_CAMERA_IP );
    hdr -> ipv4.dst_addr = rte_cpu_to_be_32( CAMERA_IP );

    hdr -> udp.src_port = rte_cpu_to_be_16( SFF1_CAMERA_PORT );
    hdr -> udp.dst_port = rte_cpu_to_be_16( CAMERA_PORT );
    hdr -> udp.dgram_cksum = 0;
}

static inline void geometry_workspace_init( struct geometry_point *points, size_t point_capacity ) {

    // Purpose: It establishes node-local first-touch residency for the preallocated environment prior to timed packet processing

    volatile struct geometry_point *warm_points = ( volatile struct geometry_point * )points;

    for ( size_t point = 0; point < point_capacity; point++ ) {
        warm_points[ point ].x = 0.0f;
        warm_points[ point ].y = 0.0f;
        warm_points[ point ].z = 0.0f;
    }
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

    fprintf( f, "frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;geometry_aggregation_ms;max_r_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets\n" );

    for ( int i = 0; i < logged_frames; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        fprintf( f, "%u;%u;%u;%u;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u;%u;%u;%u;%u;%u\n", t -> frame_id, t -> rx_complete, t -> tx_complete, t -> current_skip, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> original_points, t -> rx_points, t -> tx_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> reference_size_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> reference_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> reference_bitrate_mbps, t -> tx_duration_ms, t -> active_tx_ms, t -> active_process_ms, t -> geometry_aggregation_ms, t -> max_r_ms, t -> cycle_ms, t -> header_wait_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> reference_efficiency_pct, t -> camera_node_ms, t -> schedule_delay_ms, t -> instant_jitter_ms, t -> desynced_jitter_ms, t -> eth_errors, t -> ipv4_errors, t -> udp_errors, t -> nsh_errors, t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static inline void flush_tx_burst( struct rte_mbuf **tx_bufs, uint32_t *tx_points_buf, int *burst_idx, uint32_t *frame_tx_packets, uint32_t *frame_tx_points, uint32_t *frame_zero_accepts, uint32_t *frame_partial_accepts, uint32_t *frame_resubmit_calls, uint32_t *frame_resubmitted_packets, uint64_t *first_tx_cycles, uint64_t *last_tx_cycles, uint64_t *active_tx_cycles ) {
    if ( *burst_idx == 0 )
        return;

    uint16_t sent = 0;
    uint16_t retries = 0;

    bool is_resubmission = false;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint16_t requested_packets = *burst_idx - sent;

        if ( is_resubmission ) {
            ( *frame_resubmit_calls )++;
            *frame_resubmitted_packets += requested_packets;
        }

        uint64_t t_tx_call_start = rte_get_timer_cycles();

        if ( *first_tx_cycles == 0 )
            *first_tx_cycles = t_tx_call_start;

        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF2, 0, &tx_bufs[ sent ], requested_packets );

        uint64_t t_tx_call_end = rte_get_timer_cycles();

        *active_tx_cycles += t_tx_call_end - t_tx_call_start;

        if ( nb_tx > 0 ) {
            *last_tx_cycles = t_tx_call_end;

            *frame_tx_packets += nb_tx;

            for ( uint16_t j = 0; j < nb_tx; j++ )
                *frame_tx_points += tx_points_buf[ sent + j ];

            if ( nb_tx < requested_packets )
                ( *frame_partial_accepts )++;

            sent += nb_tx;
            retries = 0;

            is_resubmission = nb_tx < requested_packets;
        }
        else {
            ( *frame_zero_accepts )++;

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

static inline void dispatch_temporal_control( struct rte_mbuf *m ) {

    // Purpose: It concludes the backward "SPI 200" path by validating its state, stripping the "NSH" encapsulation, & forwarding the native 16-byte payload to Camera

    size_t outer_net_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );
    size_t min_packet_len = outer_net_len + sizeof( struct nsh_hdr ) + sizeof( struct temporal_payload );

    if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < min_packet_len ) ) {
        ipv4_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

    if ( unlikely( !rte_is_same_ether_addr( &eth -> src_addr, &sff2_sff1_mac ) || !rte_is_same_ether_addr( &eth -> dst_addr, &sff1_sff2_mac ) ) ) {
        eth_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
        eth_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

    if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( SFF2_SFF1_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( SFF1_SFF2_IP ) ) ) {
        ipv4_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

    if ( unlikely( udp -> src_port != rte_cpu_to_be_16( SFF2_SFF1_PORT ) || udp -> dst_port != rte_cpu_to_be_16( SFF1_SFF2_PORT ) ) ) {
        udp_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
    uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );

    if ( unlikely( udp_length < sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct temporal_payload ) || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length ) ) {
        udp_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + ipv4_length ) ) {
        ipv4_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    struct nsh_hdr *nsh = ( struct nsh_hdr * )( udp + 1 );
    
    uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );
    uint8_t version = ( base >> 14 ) & 0x03;
    bool oam = ( base & 0x2000 ) != 0;
    uint8_t ttl = ( base >> 6 ) & 0x3F;
    uint8_t length_words = base & 0x3F;

    if ( unlikely( version != 0 || oam || length_words == 0 ) ) {
        nsh_errors++;
        rte_pktmbuf_free( m );
        return;
    }
    
    uint16_t nsh_length = nsh_length_bytes( nsh );

    if ( unlikely( nsh_length != sizeof( struct nsh_hdr ) || nsh -> md_type != MD_TYPE_2 || nsh -> next_protocol != NEXT_PROTOCOL_EXPERIMENT_1 ) ) {
        nsh_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( udp_length != sizeof( struct rte_udp_hdr ) + nsh_length + sizeof( struct temporal_payload ) ) ) {
        nsh_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    uint32_t sph = rte_be_to_cpu_32( nsh -> serv_path_hdr );
    uint32_t spi = sph >> 8;
    uint8_t si = sph & 0xFF;

    if ( unlikely( spi != TEMPORAL_SPI || si != TEMPORAL_SI ) ) {
        nsh_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    size_t strip_len = outer_net_len + nsh_length;

    if ( unlikely( rte_pktmbuf_adj( m, strip_len ) == NULL ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( rte_pktmbuf_pkt_len( m ) != sizeof( struct temporal_payload ) ) ) {
        nsh_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    char *new_hdr_start = rte_pktmbuf_prepend( m, sizeof( struct net_hdr ) );

    if ( unlikely( new_hdr_start == NULL ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    struct net_hdr *out_hdr = ( struct net_hdr * )new_hdr_start;

    rte_memcpy( out_hdr, &temporal_template_hdr, sizeof( struct net_hdr ) );

    uint16_t out_udp_len = sizeof( struct rte_udp_hdr ) + sizeof( struct temporal_payload );

    out_hdr -> udp.dgram_len = rte_cpu_to_be_16( out_udp_len );
    out_hdr -> ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + out_udp_len );
    out_hdr -> ipv4.hdr_checksum = 0;
    out_hdr -> ipv4.hdr_checksum = rte_ipv4_cksum( &out_hdr -> ipv4 );

    uint16_t retries = 0;

    while ( 1 ) {
        uint16_t nb_tx = rte_eth_tx_burst( PORT_CAMERA, 0, &m, 1 );

        if ( nb_tx == 1 )
            return;

        if ( ++retries > MAX_ZERO_ACCEPTS ) {
            rte_pktmbuf_free( m );
            return;
        }

        rte_pause();
    }
}

static int worker_loop( __rte_unused void *arg ) {
    struct worker_context *worker_ctx = ( struct worker_context * )arg;

    if ( worker_ctx == NULL || worker_ctx -> frame_geometry_points == NULL || worker_ctx -> frame_geometry_capacity == 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Invalid geometry context...\n" );

    struct geometry_point *frame_geometry_points = worker_ctx -> frame_geometry_points;
    const size_t frame_geometry_capacity = worker_ctx -> frame_geometry_capacity;

    geometry_workspace_init( frame_geometry_points, frame_geometry_capacity );

    uint64_t timer_hz = rte_get_timer_hz();

    struct rte_mbuf *bufs[ BURST_SIZE ];
    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    struct rte_mbuf *temporal_bufs[ BURST_SIZE ];

    uint32_t tx_points_buf[ BURST_SIZE ];

    uint32_t current_frame_id = END_OF_STREAM;
    double frame_sum_x = 0.0, frame_sum_y = 0.0, frame_sum_z = 0.0;
    float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
    float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
    uint32_t frame_point_count = 0;
    uint32_t frame_rx_points = 0;
    uint32_t frame_original_points = 0;
    uint32_t frame_expected_sequence = 0;
    bool frame_sequence_ok = true;

    uint64_t t_frame_arrival = 0;
    uint64_t last_rx_cycles = 0;
    uint64_t t_cycle_start = rte_get_timer_cycles();

    uint64_t active_process_cycles = 0;
    uint64_t active_tx_cycles = 0;
    uint64_t geometry_cycles = 0;
    uint64_t max_r_cycles = 0;

    uint32_t frame_zero_accepts = 0;
    uint32_t frame_partial_accepts = 0;
    uint32_t frame_resubmit_calls = 0;
    uint32_t frame_resubmitted_packets = 0;

    uint32_t frame_tx_packets = 0;
    uint32_t frame_rx_packets = 0;
    uint32_t frame_tx_points = 0;

    uint32_t frame_eth_errors = 0;
    uint32_t frame_ipv4_errors = 0;
    uint32_t frame_udp_errors = 0;
    uint32_t frame_nsh_errors = 0;

    uint64_t frame_completion_cycles = 0;
    uint64_t last_activity_cycles = 0;

    uint64_t first_tx_cycles = 0;
    uint64_t last_tx_cycles = 0;

    int burst_idx = 0;

    uint64_t t_session_start = 0;
    uint32_t frames_received = 0;
    double current_latency_ms = 0.0;
    double current_jitter_ms = 0.0;
    double jitter_ms = 0.0;

    uint64_t prev_arrival_cycles = 0;
    uint32_t prev_arrival_frame = 0;
    uint32_t first_arrival_frame = 0;

    uint16_t frame_temporal_skip = 1;

    const size_t outer_len = sizeof( struct main_hdr );
    const size_t camera_net_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

    printf( "[SYSTEM] Listening on every service-chain link...\n\n" );

    while ( 1 ) {
        if ( csv_written ) {
            rte_delay_us_sleep( 1000 );
            continue;
        }

        uint16_t nb_temporal = rte_eth_rx_burst( PORT_SFF2, 0, temporal_bufs, BURST_SIZE );

        for ( uint16_t i = 0; i < nb_temporal; i++ )
            dispatch_temporal_control( temporal_bufs[ i ] );

        uint16_t nb_rx = rte_eth_rx_burst( PORT_CAMERA, 0, bufs, BURST_SIZE );

        if ( unlikely( nb_rx == 0 ) )
            continue;

        for ( int i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];

            size_t min_req = camera_net_len + sizeof( struct cam_hdr );

            if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < min_req ) ) {
                frame_ipv4_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ether_hdr *old_eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

            if ( unlikely( !rte_is_same_ether_addr( &old_eth -> src_addr, &camera_mac ) || !rte_is_same_ether_addr( &old_eth -> dst_addr, &sff1_camera_mac ) ) ) {
                frame_eth_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( old_eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
                frame_eth_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ipv4_hdr *old_ipv4 = ( struct rte_ipv4_hdr * )( old_eth + 1 );

            if ( unlikely( old_ipv4 -> version_ihl != 0x45 || old_ipv4 -> next_proto_id != IPPROTO_UDP || old_ipv4 -> src_addr != rte_cpu_to_be_32( CAMERA_IP ) || old_ipv4 -> dst_addr != rte_cpu_to_be_32( SFF1_CAMERA_IP ) ) ) {
                frame_ipv4_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_udp_hdr *old_udp = ( struct rte_udp_hdr * )( old_ipv4 + 1 );

            if ( unlikely( old_udp -> src_port != rte_cpu_to_be_16( CAMERA_PORT ) || old_udp -> dst_port != rte_cpu_to_be_16( SFF1_CAMERA_PORT ) ) ) {
                frame_udp_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t old_udp_length = rte_be_to_cpu_16( old_udp -> dgram_len );
            uint16_t old_ipv4_length = rte_be_to_cpu_16( old_ipv4 -> total_length );

            if ( unlikely( old_udp_length < sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) || old_ipv4_length != sizeof( struct rte_ipv4_hdr ) + old_udp_length ) ) {
                frame_udp_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + old_ipv4_length ) ) {
                frame_ipv4_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct cam_hdr *old_cam = ( struct cam_hdr * )( old_udp + 1 );
            uint32_t f_id = rte_be_to_cpu_32( old_cam -> frame_id );
            uint32_t packet_sequence = rte_be_to_cpu_32( old_cam -> sequence_number );
            uint32_t packet_points = rte_be_to_cpu_32( old_cam -> points_in_packet );
            uint32_t packet_payload_len = old_udp_length - sizeof( struct rte_udp_hdr ) - sizeof( struct cam_hdr );

            if ( f_id != END_OF_STREAM ) {
                if ( unlikely( packet_points > POINTS_PER_PACKET || packet_payload_len != packet_points * sizeof( struct point_tx ) ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }
            }
            else if ( unlikely( packet_points != 0 || packet_payload_len != 0 ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            uint64_t packet_arrival_cycles = rte_get_timer_cycles();

            if ( unlikely( f_id != current_frame_id ) ) {
                if ( current_frame_id != END_OF_STREAM && burst_idx > 0 ) {
                    uint64_t t_final_flush_start = rte_get_timer_cycles();

                    flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_zero_accepts, &frame_partial_accepts, &frame_resubmit_calls, &frame_resubmitted_packets, &first_tx_cycles, &last_tx_cycles, &active_tx_cycles );

                    uint64_t t_final_flush_end = rte_get_timer_cycles();
                    active_process_cycles += t_final_flush_end - t_final_flush_start;

                    last_activity_cycles = t_final_flush_end;
                }

                if ( current_frame_id != END_OF_STREAM && logged_frames < K_FRAMES ) {
                    uint64_t t_now = rte_get_timer_cycles();
                    uint64_t t_frame_end;

                    if ( frame_completion_cycles > 0 )
                        t_frame_end = frame_completion_cycles;
                    else if ( last_activity_cycles > 0 )
                        t_frame_end = last_activity_cycles;
                    else
                        t_frame_end = t_now;

                    double duration_sec = 0.0;

                    if ( first_tx_cycles > 0 && last_tx_cycles >= first_tx_cycles )
                        duration_sec = ( double )( last_tx_cycles - first_tx_cycles ) / timer_hz;

                    double residency_sec = ( double )( t_frame_end - t_frame_arrival ) / timer_hz;
                    double receive_sec = ( last_rx_cycles >= t_frame_arrival ) ? ( double )( last_rx_cycles - t_frame_arrival ) / timer_hz : 0.0;
                    double cycle_sec = ( t_frame_end >= t_cycle_start ) ? ( double )( t_frame_end - t_cycle_start ) / timer_hz : 0.0;
                    double header_wait_sec = ( cycle_sec > residency_sec ) ? cycle_sec - residency_sec : 0.0;
                    double active_process_sec = ( double )active_process_cycles / timer_hz;
                    double active_tx_sec = ( double )active_tx_cycles / timer_hz;
                    double geometry_sec = ( double )geometry_cycles / timer_hz;
                    double max_r_sec = ( double )max_r_cycles / timer_hz;

                    uint64_t received_payload_bytes = ( uint64_t )frame_rx_points * sizeof( struct point_tx );
                    uint64_t logical_rx_frame_bytes = received_payload_bytes + ( frame_rx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );

                    uint64_t transmitted_payload_bytes = ( uint64_t )frame_tx_points * sizeof( struct point_tx );
                    uint64_t logical_frame_bytes = transmitted_payload_bytes + ( frame_tx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );
                    uint64_t network_frame_bytes = transmitted_payload_bytes + ( ( uint64_t )frame_tx_packets * ( sizeof( struct main_hdr ) + sizeof( struct cam_hdr ) ) );

                    uint64_t reference_frame_bytes = ( ( uint64_t )frame_original_points * sizeof( struct point_tx ) ) + ( frame_original_points > 0 ? sizeof( struct cam_hdr ) : 0 );
                    uint32_t expected_packets = ( frame_original_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET;

                    double effective_fps = TARGET_FPS / frame_temporal_skip;

                    bool rx_complete = frame_original_points > 0 && frame_rx_points == frame_original_points && frame_rx_packets == expected_packets && frame_sequence_ok;
                    bool tx_complete = rx_complete && frame_tx_points == frame_rx_points && frame_tx_packets == frame_rx_packets;

                    telemetry_log[ logged_frames ].frame_id = current_frame_id;
                    telemetry_log[ logged_frames ].rx_complete = rx_complete ? 1 : 0;
                    telemetry_log[ logged_frames ].tx_complete = tx_complete ? 1 : 0;
                    telemetry_log[ logged_frames ].current_skip = frame_temporal_skip;
                    telemetry_log[ logged_frames ].node_exit_timestamp = ( double )t_frame_end / timer_hz;

                    telemetry_log[ logged_frames ].tx_duration_ms = duration_sec * 1000.0;
                    telemetry_log[ logged_frames ].payload_bytes = transmitted_payload_bytes;

                    telemetry_log[ logged_frames ].original_points = frame_original_points;
                    telemetry_log[ logged_frames ].rx_packets = frame_rx_packets;
                    telemetry_log[ logged_frames ].rx_points = frame_rx_points;
                    telemetry_log[ logged_frames ].tx_packets = frame_tx_packets;
                    telemetry_log[ logged_frames ].tx_points = frame_tx_points;

                    telemetry_log[ logged_frames ].data_integrity_pct = ( frame_original_points > 0 ) ? ( ( double )telemetry_log[ logged_frames ].rx_points / ( double )frame_original_points ) * 100.0 : 0.0;

                    telemetry_log[ logged_frames ].tx_zero_accepts = frame_zero_accepts;
                    telemetry_log[ logged_frames ].tx_partial_accepts = frame_partial_accepts;
                    telemetry_log[ logged_frames ].tx_resubmit_calls = frame_resubmit_calls;
                    telemetry_log[ logged_frames ].tx_resubmitted_packets = frame_resubmitted_packets;

                    telemetry_log[ logged_frames ].active_tx_ms = active_tx_sec * 1000.0;
                    telemetry_log[ logged_frames ].active_process_ms = active_process_sec * 1000.0;
                    telemetry_log[ logged_frames ].geometry_aggregation_ms = geometry_sec * 1000.0;
                    telemetry_log[ logged_frames ].max_r_ms = max_r_sec * 1000.0;
                    telemetry_log[ logged_frames ].total_residency_ms = residency_sec * 1000.0;
                    telemetry_log[ logged_frames ].cycle_ms = cycle_sec * 1000.0;
                    telemetry_log[ logged_frames ].header_wait_ms = header_wait_sec * 1000.0;

                    telemetry_log[ logged_frames ].camera_node_ms = current_latency_ms;

                    uint32_t schedule_frame_offset = current_frame_id - first_arrival_frame;
                    double real_exit_sec = ( t_frame_end >= t_session_start ) ? ( double )( t_frame_end - t_session_start ) / timer_hz : 0.0;
                    double ideal_start_sec = ( double )schedule_frame_offset / TARGET_FPS;

                    telemetry_log[ logged_frames ].schedule_delay_ms = ( real_exit_sec - ideal_start_sec ) * 1000.0;
                    telemetry_log[ logged_frames ].instant_jitter_ms = current_jitter_ms;
                    telemetry_log[ logged_frames ].desynced_jitter_ms = jitter_ms;

                    telemetry_log[ logged_frames ].internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_rx_frame_bytes / 1000000.0 ) / receive_sec : 0.0;
                    telemetry_log[ logged_frames ].logical_bitrate_mbps = ( logical_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
                    telemetry_log[ logged_frames ].network_bitrate_mbps = ( network_frame_bytes * 8.0 * effective_fps ) / 1000000.0;

                    telemetry_log[ logged_frames ].reference_size_bytes = ( uint32_t )reference_frame_bytes;
                    telemetry_log[ logged_frames ].reference_throughput_mbs = ( residency_sec > 0.0 ) ? ( ( double )reference_frame_bytes / 1000000.0 ) / residency_sec : 0.0;
                    telemetry_log[ logged_frames ].reference_bitrate_mbps = ( reference_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
                    telemetry_log[ logged_frames ].reference_efficiency_pct = ( cycle_sec > 0.0 ) ? ( residency_sec / cycle_sec ) * 100.0 : 0.0;

                    telemetry_log[ logged_frames ].eth_errors = frame_eth_errors;
                    telemetry_log[ logged_frames ].ipv4_errors = frame_ipv4_errors;
                    telemetry_log[ logged_frames ].udp_errors = frame_udp_errors;
                    telemetry_log[ logged_frames ].nsh_errors = frame_nsh_errors;

                    if ( residency_sec > 0.0 )
                        telemetry_log[ logged_frames ].node_efficiency_pct = ( active_process_sec / residency_sec ) * 100.0;
                    else
                        telemetry_log[ logged_frames ].node_efficiency_pct = 0.0;

                    t_cycle_start = t_frame_end;
                    logged_frames++;
                }

                uint64_t cam_tx_cycles = rte_be_to_cpu_64( old_cam -> timestamp );
                uint64_t arrival_cycles = packet_arrival_cycles;

                if ( arrival_cycles >= cam_tx_cycles )
                    current_latency_ms = ( ( double )( arrival_cycles - cam_tx_cycles ) / timer_hz ) * 1000.0;
                else
                    current_latency_ms = 0.0;

                if ( f_id != END_OF_STREAM ) {
                    uint16_t incoming_skip = rte_be_to_cpu_16( old_cam -> temporal_skip );

                    if ( incoming_skip == 0 )
                        incoming_skip = 1;

                    frame_temporal_skip = incoming_skip;

                    if ( frames_received == 0 ) {
                        t_session_start = arrival_cycles;
                        first_arrival_frame = f_id;
                        t_cycle_start = arrival_cycles;
                    }

                    if ( prev_arrival_cycles > 0 && f_id > prev_arrival_frame ) {
                        double real_interval_sec = ( double )( arrival_cycles - prev_arrival_cycles ) / timer_hz;
                        double expected_interval_sec = ( double )( f_id - prev_arrival_frame ) / TARGET_FPS;
                        double diff_sec = real_interval_sec - expected_interval_sec;

                        current_jitter_ms = ( diff_sec < 0.0 ) ? -diff_sec * 1000.0 : diff_sec * 1000.0;
                    }
                    else {
                        current_jitter_ms = 0.0;
                    }

                    jitter_ms += ( current_jitter_ms - jitter_ms ) / 16.0;

                    prev_arrival_cycles = arrival_cycles;
                    prev_arrival_frame = f_id;
                    frames_received++;
                }

                current_frame_id = f_id;
                t_frame_arrival = arrival_cycles;
                last_rx_cycles = arrival_cycles;

                if ( logged_frames < K_FRAMES ) {
                    telemetry_log[ logged_frames ].camera_send_timestamp = ( double )cam_tx_cycles / timer_hz;
                    telemetry_log[ logged_frames ].recv_start_timestamp = ( double )arrival_cycles / timer_hz;
                }

                frame_rx_points = 0;
                frame_expected_sequence = 0;
                frame_sequence_ok = true;
                frame_tx_packets = 0;
                frame_rx_packets = 0;
                frame_zero_accepts = 0;
                frame_partial_accepts = 0;
                frame_resubmit_calls = 0;
                frame_resubmitted_packets = 0;
                frame_eth_errors = 0;
                frame_ipv4_errors = 0;
                frame_udp_errors = 0;
                frame_nsh_errors = 0;
                active_process_cycles = 0;
                active_tx_cycles = 0;
                geometry_cycles = 0;
                max_r_cycles = 0;
                frame_tx_points = 0;
                frame_completion_cycles = 0;
                last_activity_cycles = 0;
                first_tx_cycles = 0;
                last_tx_cycles = 0;
                frame_sum_x = 0.0;
                frame_sum_y = 0.0;
                frame_sum_z = 0.0;
                frame_point_count = 0;
                frame_original_points = 0;
                min_x = FLT_MAX;
                min_y = FLT_MAX;
                min_z = FLT_MAX;
                max_x = -FLT_MAX;
                max_y = -FLT_MAX;
                max_z = -FLT_MAX;
            }

            if ( unlikely( f_id == END_OF_STREAM ) ) {
                if ( unlikely( rte_pktmbuf_adj( m, camera_net_len ) == NULL ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                char *new_hdr_start = rte_pktmbuf_prepend( m, outer_len );

                if ( likely( new_hdr_start != NULL ) ) {
                    struct main_hdr *hdr = ( struct main_hdr * )new_hdr_start;

                    rte_memcpy( hdr, &main_template_hdr, outer_len );
                    memset( &hdr -> geo, 0, sizeof( struct geo_agg_hdr ) );

                    uint32_t inner_pkt_len = m -> pkt_len - outer_len;
                    uint16_t outer_udp_len = sizeof( struct rte_udp_hdr ) + NSH_TOTAL_SIZE + inner_pkt_len;

                    hdr -> net.udp.dgram_len = rte_cpu_to_be_16( outer_udp_len );
                    hdr -> net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + outer_udp_len );
                    hdr -> net.ipv4.hdr_checksum = 0;
                    hdr -> net.ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> net.ipv4 );

                    tx_bufs[ burst_idx ] = m;
                    tx_points_buf[ burst_idx ] = 0;
                    burst_idx++;

                    flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_zero_accepts, &frame_partial_accepts, &frame_resubmit_calls, &frame_resubmitted_packets, &first_tx_cycles, &last_tx_cycles, &active_tx_cycles );
                }
                else
                    rte_pktmbuf_free( m );

                if ( !csv_written && logged_frames > 0 ) {
                    telemetry_to_csv();
                    csv_written = true;
                    printf( "\n[SYSTEM] End of stream detected. Changing to \"idle\" state...\n" );
                }

                continue;
            }

            uint64_t t_active_process_start = rte_get_timer_cycles();

            last_rx_cycles = packet_arrival_cycles;

            if ( packet_sequence != frame_expected_sequence )
                frame_sequence_ok = false;

            frame_expected_sequence++;

            frame_rx_packets++;
            frame_rx_points += packet_points;
            frame_original_points = rte_be_to_cpu_32( old_cam -> original_points );

            bool frame_complete = frame_original_points > 0 && frame_rx_points == frame_original_points;

            if ( unlikely( frame_original_points > frame_geometry_capacity ) )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Frame %u declares %u points, exceeding the preallocated maximum of %zu...\n", f_id, frame_original_points, frame_geometry_capacity );

            if ( unlikely( ( size_t )frame_point_count + packet_points > frame_geometry_capacity ) )
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Frame %u geometry overflow detected...\n", f_id );

            uint64_t t_geometry_start = rte_get_timer_cycles();

            if ( likely( packet_payload_len > 0 ) ) {
                struct point_tx *points = ( struct point_tx * )( old_cam + 1 );

                for ( uint32_t point = 0; point < packet_points; point++ ) {
                    float x = be_to_float( points[ point ].x );
                    float y = be_to_float( points[ point ].y );
                    float z = be_to_float( points[ point ].z );

                    frame_geometry_points[ frame_point_count ].x = x;
                    frame_geometry_points[ frame_point_count ].y = y;
                    frame_geometry_points[ frame_point_count ].z = z;

                    frame_sum_x += x;
                    frame_sum_y += y;
                    frame_sum_z += z;

                    frame_point_count++;

                    if ( x < min_x )
                        min_x = x;
                    if ( x > max_x )
                        max_x = x;
                    if ( y < min_y )
                        min_y = y;
                    if ( y > max_y )
                        max_y = y;
                    if ( z < min_z )
                        min_z = z;
                    if ( z > max_z )
                        max_z = z;
                }
            }

            float centroid_x = 0.0f;
            float centroid_y = 0.0f;
            float centroid_z = 0.0f;

            float extent_x = 0.0f;
            float extent_y = 0.0f;
            float extent_z = 0.0f;

            float bbox_center_x = 0.0f;
            float bbox_center_y = 0.0f;
            float bbox_center_z = 0.0f;

            if ( frame_point_count > 0 ) {
                centroid_x = ( float )( frame_sum_x / frame_point_count );
                centroid_y = ( float )( frame_sum_y / frame_point_count );
                centroid_z = ( float )( frame_sum_z / frame_point_count );

                extent_x = max_x - min_x;
                extent_y = max_y - min_y;
                extent_z = max_z - min_z;

                bbox_center_x = ( min_x + max_x ) * 0.5f;
                bbox_center_y = ( min_y + max_y ) * 0.5f;
                bbox_center_z = ( min_z + max_z ) * 0.5f;
            }

            uint64_t t_geometry_end = rte_get_timer_cycles();
            geometry_cycles += t_geometry_end - t_geometry_start;

            float packet_max_r = 0.0f;
            float packet_final_scale = 0.0f;
            float packet_global_scale = 0.0f;
            float packet_bbox_x = 0.0f;
            float packet_bbox_y = 0.0f;
            float packet_bbox_z = 0.0f;

            bool final_geometry = frame_original_points > 0 && frame_point_count == frame_original_points;

            if ( final_geometry ) {
                uint64_t t_max_r_start = rte_get_timer_cycles();

                packet_max_r = calculate_maximum_radius(
                    frame_geometry_points,
                    frame_point_count,
                    centroid_x,
                    centroid_y,
                    centroid_z
                );

                uint64_t t_max_r_end = rte_get_timer_cycles();
                max_r_cycles += t_max_r_end - t_max_r_start;

                uint64_t t_projection_frontier_start = rte_get_timer_cycles();

                calculate_projection_metadata( frame_geometry_points, frame_point_count, centroid_x, centroid_y, centroid_z, packet_max_r, &packet_final_scale, &packet_global_scale, &packet_bbox_x, &packet_bbox_y, &packet_bbox_z );

                uint64_t t_projection_frontier_end = rte_get_timer_cycles();
                geometry_cycles += t_projection_frontier_end - t_projection_frontier_start;
            }

            if ( unlikely( rte_pktmbuf_adj( m, camera_net_len ) == NULL ) ) {
                rte_pktmbuf_free( m );

                uint64_t t_active_process_end = rte_get_timer_cycles();
                active_process_cycles += t_active_process_end - t_active_process_start;
                last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = last_activity_cycles;

                continue;
            }

            char *new_hdr_start = rte_pktmbuf_prepend( m, outer_len );

            if ( unlikely( new_hdr_start == NULL ) ) {
                rte_pktmbuf_free( m );

                uint64_t t_active_process_end = rte_get_timer_cycles();
                active_process_cycles += t_active_process_end - t_active_process_start;
                last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = last_activity_cycles;

                continue;
            }

            struct main_hdr *hdr = ( struct main_hdr * )new_hdr_start;

            rte_memcpy( hdr, &main_template_hdr, outer_len );

            uint32_t inner_pkt_len = m -> pkt_len - outer_len;
            uint16_t outer_udp_len = sizeof( struct rte_udp_hdr ) + NSH_TOTAL_SIZE + inner_pkt_len;

            hdr -> net.udp.dgram_len = rte_cpu_to_be_16( outer_udp_len );
            hdr -> net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + outer_udp_len );
            hdr -> net.ipv4.hdr_checksum = 0;
            hdr -> net.ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> net.ipv4 );

            hdr -> geo.centroid_x = float_to_be( centroid_x );
            hdr -> geo.centroid_y = float_to_be( centroid_y );
            hdr -> geo.centroid_z = float_to_be( centroid_z );

            hdr -> geo.extent_x = float_to_be( extent_x );
            hdr -> geo.extent_y = float_to_be( extent_y );
            hdr -> geo.extent_z = float_to_be( extent_z );

            hdr -> geo.bbox_center_x = float_to_be( bbox_center_x );
            hdr -> geo.bbox_center_y = float_to_be( bbox_center_y );
            hdr -> geo.bbox_center_z = float_to_be( bbox_center_z );

            hdr -> geo.max_r = float_to_be( packet_max_r );
            hdr -> geo.final_scale = float_to_be( packet_final_scale );
            hdr -> geo.global_scale = float_to_be( packet_global_scale );
            hdr -> geo.projected_bbox_x = float_to_be( packet_bbox_x );
            hdr -> geo.projected_bbox_y = float_to_be( packet_bbox_y );
            hdr -> geo.projected_bbox_z = float_to_be( packet_bbox_z );
            hdr -> geo.active_point_count = rte_cpu_to_be_32( frame_point_count );

            tx_bufs[ burst_idx ] = m;
            tx_points_buf[ burst_idx ] = packet_points;
            burst_idx++;

            if ( burst_idx == BURST_SIZE || frame_complete )
                flush_tx_burst( tx_bufs, tx_points_buf, &burst_idx, &frame_tx_packets, &frame_tx_points, &frame_zero_accepts, &frame_partial_accepts, &frame_resubmit_calls, &frame_resubmitted_packets, &first_tx_cycles, &last_tx_cycles, &active_tx_cycles );

            uint64_t t_active_process_end = rte_get_timer_cycles();

            active_process_cycles += t_active_process_end - t_active_process_start;
            last_activity_cycles = t_active_process_end;

            if ( frame_complete ) {
                if ( last_tx_cycles > 0 )
                    frame_completion_cycles = last_tx_cycles;
                else
                    frame_completion_cycles = last_activity_cycles;
            }
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It models the "Geometry-Aware Classifier" ( "GAC" ) for the "Main" path, classifying plain Camera traffic, deriving reusable frame geometry & imposing the "NSH" context before delivery to SFF2.
    //          On the reverse "Temporal" chain, it performs validation & decapsulation only. Thus, Camera remains solely responsible for applying the received sampling decision.
    //          "Pose" fields remain static application metadata transmitted to Encoder without interpretation
    
    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"SFF1\" microservice...\n" );

    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_CAMERA, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Camera-facing virtual port configuration failed...\n" );

    if ( port_init( PORT_SFF2, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF2-facing virtual port configuration failed...\n" );

    printf( "\n" );

    main_header_init( &main_template_hdr );
    temporal_header_init( &temporal_template_hdr );

    struct worker_context worker_ctx = { 0 };
    worker_ctx.frame_geometry_capacity = MAX_FRAME_POINTS;

    size_t geometry_workspace_bytes = worker_ctx.frame_geometry_capacity * sizeof( struct geometry_point );
    worker_ctx.frame_geometry_points = malloc( geometry_workspace_bytes );

    if ( worker_ctx.frame_geometry_points == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Failed to allocate workspace...\n" );
        
    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE )
        worker_loop( &worker_ctx );
    else {
        rte_eal_remote_launch( worker_loop, &worker_ctx, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    free( worker_ctx.frame_geometry_points );
    worker_ctx.frame_geometry_points = NULL;

    rte_eal_cleanup();

    return 0;
}