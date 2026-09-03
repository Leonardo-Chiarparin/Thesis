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
#define TELEMETRY_FOLDER "/shared/log/sff2"
#define SFF1_ENCODER_PATH "/shared/log/sff2/telemetry_sff1_enc.csv"
#define ENCODER_DECODER_PATH "/shared/log/sff2/telemetry_enc_dec.csv"
#define DECODER_SFF3_PATH "/shared/log/sff2/telemetry_dec_sff3.csv"

#define K_FRAMES 300

#define TARGET_FPS 30.0

#define BURST_SIZE 32
#define MAX_ZERO_ACCEPTS 2048

#define END_OF_STREAM 0xFFFFFFFF
#define FRAME_ID ( END_OF_STREAM - 1 )

// "DPDK" packet-buffer pool settings
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// "Service Function Path" ( "SFP" ) identifiers & state values
#define MAIN_SPI 100
#define TEMPORAL_SPI 200
#define POSE_SPI 300

#define MAIN_SI_SFF1 255
#define MAIN_SI_ENCODER 254
#define MAIN_SI_DECODER 253
#define TEMPORAL_SI 255
#define POSE_SI 255

// Sending bonds & networking parameters
#define PORT_SFF1 0
#define PORT_ENCODER 1
#define PORT_DECODER 2
#define PORT_SFF3 3
#define TOTAL_PORTS 4

#define ROUTE_SFF1_ENCODER 0
#define ROUTE_ENCODER_DECODER 1
#define ROUTE_DECODER_SFF3 2
#define TOTAL_ROUTES 3

#define SFF1_SFF2_IP RTE_IPV4( 10, 0, 2, 1 )
#define SFF2_SFF1_IP RTE_IPV4( 10, 0, 2, 2 )
#define SFF1_SFF2_PORT 6633
#define SFF2_SFF1_PORT 6633

#define SFF2_ENCODER_IP RTE_IPV4( 10, 0, 3, 254 )
#define ENCODER_IP RTE_IPV4( 10, 0, 3, 1 )
#define SFF2_ENCODER_PORT 6633
#define ENCODER_PORT 7001

#define SFF2_DECODER_IP RTE_IPV4( 10, 0, 4, 254 )
#define DECODER_IP RTE_IPV4( 10, 0, 4, 1 )
#define SFF2_DECODER_PORT 6633
#define DECODER_PORT 8001

#define SFF2_SFF3_IP RTE_IPV4( 10, 0, 5, 2 )
#define SFF3_SFF2_IP RTE_IPV4( 10, 0, 5, 1 )
#define SFF2_SFF3_PORT 6633
#define SFF3_SFF2_PORT 6633

// Packetization & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80
#define TS_PACKET_SIZE 188
#define MTU_PAYLOAD_SIZE ( 7 * TS_PACKET_SIZE )

#define MD_CLASS_EXPERIMENTAL 0xFFF6
#define MD_TYPE_GEOMETRY 0x01
#define MD_TYPE_2 0x02
#define NEXT_PROTOCOL_EXPERIMENT_1 0xFE
#define DEFAULT_TTL 63

// Wire-format structures used by "DPDK" data path
struct nsh_hdr {
    uint16_t base_flags_ttl_len;
    uint8_t md_type;
    uint8_t next_protocol;
    uint32_t serv_path_hdr;
} __attribute__((__packed__));

struct nsh_md2_ctx_hdr {
    uint16_t metadata_class;
    uint8_t type;
    uint8_t u_length;
} __attribute__((__packed__));

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

    uint32_t max_r; 

    uint32_t final_scale; 
    uint32_t global_scale;
    uint32_t projected_bbox_x;
    uint32_t projected_bbox_y;
    uint32_t projected_bbox_z;

    uint32_t active_point_count;
} __attribute__((__packed__));

#define MAIN_GEOMETRY_SIZE ( sizeof( struct nsh_hdr ) + sizeof( struct nsh_md2_ctx_hdr ) + sizeof( struct geo_agg_hdr ) )

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

struct dec_hdr {
    uint32_t frame_id;
    uint32_t sequence_number;
    uint64_t timestamp;
    uint32_t yaw;
    uint32_t pitch;
    uint32_t zoom;
    uint16_t temporal_skip;
    uint16_t padding;
    uint32_t original_points;
    uint32_t arrived_points;
    uint32_t eroded_points;
    uint32_t valid_points;
    uint32_t points_in_packet;
} __attribute__((__packed__));

struct enc_hdr {
    uint32_t frame_id;
    uint32_t packet_id;
    uint32_t global_scale;
    uint32_t box_center_x;
    uint32_t box_center_y;
    uint32_t box_center_z;
    uint32_t yaw;
    uint32_t pitch;
    uint32_t final_scale;
    uint32_t centroid_x;
    uint32_t centroid_y;
    uint32_t centroid_z; 
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

struct temporal_payload {
    uint32_t frame_id;
    uint64_t timestamp; 
    uint16_t skip;
    uint16_t padding;
} __attribute__((__packed__));

// SFF2, about the subsequent organization, validates only its size, treating internal fields ( e.g., "yaw", "pitch", "zoom" ) as semantically opaque control metadata
struct pose_payload { 
    uint64_t timestamp;
    uint32_t yaw;
    uint32_t pitch;
    uint32_t zoom;
    uint32_t padding; // results in a 24-byte network component
} __attribute__((__packed__));

struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__, __aligned__(2)));

struct temporal_hdr {
    struct net_hdr net;
    struct nsh_hdr nsh;
} __attribute__((__packed__, __aligned__(2)));

// Per-frame service-path condition maintained by the node while adjacent "NSH"-unaware functions process standard application datagrams
struct proxy_context {
    uint32_t frame_id;
    uint8_t ttl;
    uint8_t si;
    bool valid;
};

enum packet_type {
    INVALID = 0,
    MAIN_SFF1_ENCODER,
    MAIN_ENCODER_DECODER,
    MAIN_DECODER_SFF3,
    POSE_SFF3_DECODER
};

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
    uint32_t rx_media_bytes;
    uint32_t tx_media_bytes;
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
    double cycle_ms;
    double header_wait_ms;
    double total_residency_ms;
    double node_efficiency_pct;
    double cycle_occupancy_pct;

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
static struct rte_ether_addr src_macs[ TOTAL_PORTS ];
static struct rte_ether_addr dst_macs[ TOTAL_PORTS ];

static uint32_t src_ips[ TOTAL_PORTS ];
static uint32_t dst_ips[ TOTAL_PORTS ];

static uint16_t ingress_src_ports[ TOTAL_PORTS ];
static uint16_t ingress_dst_ports[ TOTAL_PORTS ];
static uint16_t egress_src_ports[ TOTAL_PORTS ];
static uint16_t egress_dst_ports[ TOTAL_PORTS ];

static struct net_hdr header_template[ TOTAL_PORTS ];

static uint32_t eth_errors[ TOTAL_PORTS ] = { 0 };
static uint32_t ipv4_errors[ TOTAL_PORTS ] = { 0 };
static uint32_t udp_errors[ TOTAL_PORTS ] = { 0 };
static uint32_t nsh_errors[ TOTAL_PORTS ] = { 0 };

static struct proxy_context proxy_main_context[ K_FRAMES ];
static struct proxy_context proxy_eos_context;

static struct telemetry_csv telemetry_sff1_enc[ K_FRAMES ];
static struct telemetry_csv telemetry_enc_dec[ K_FRAMES ];
static struct telemetry_csv telemetry_dec_sff3[ K_FRAMES ];

static uint32_t frames_sff1_enc = 0;
static uint32_t frames_enc_dec = 0;
static uint32_t frames_dec_sff3 = 0;
static bool csv_written[ TOTAL_ROUTES ] = { false, false, false };

// Data path & support routines
static inline uint16_t nsh_base_field( uint8_t ttl, uint8_t length_words ) {
    uint16_t value = ( ( uint16_t )( ttl & 0x3F ) << 6 ) | ( uint16_t )( length_words & 0x3F );

    return rte_cpu_to_be_16( value );
}

static inline uint16_t nsh_length_bytes( struct nsh_hdr *nsh ) {
    uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );
    uint8_t length_words = base & 0x3F;

    return ( uint16_t )length_words * 4;
}

static inline bool decrement_nsh_ttl( struct nsh_hdr *nsh ) {

    // Purpose: It decrements the "TTL" component prior to forwarding lookup, preserving the current "Service Index" ( "SI" ).
    //          For clueless nodes, the proxy computes the ensuing transition upon packet returns

    uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );
    uint8_t ttl = ( base >> 6 ) & 0x3F;

    if ( ttl == 0 )
        ttl = DEFAULT_TTL;
    else
        ttl--;

    if ( ttl == 0 )
        return false;

    base &= ~( ( uint16_t )0x3F << 6 );
    base |= ( ( uint16_t )ttl << 6 );

    nsh -> base_flags_ttl_len = rte_cpu_to_be_16( base );

    return true;
}

static inline uint8_t get_nsh_ttl( const struct nsh_hdr *nsh ) {
    uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );

    return ( base >> 6 ) & 0x3F;
}

static inline struct proxy_context *proxy_context_slot( uint32_t frame_id ) {
    
    // Purpose: It retrieves the surrogate condition associated with a specific frame identifier
    
    if ( frame_id == END_OF_STREAM )
        return &proxy_eos_context;

    if ( frame_id == 0 || frame_id > K_FRAMES )
        return NULL;

    return &proxy_main_context[ frame_id - 1 ];
}

static inline uint16_t port_by_route( int route_id ) {

    // Purpose: It maps each primary service-chain route to the physical port on which that frame enters the node

    if ( route_id == ROUTE_SFF1_ENCODER )
        return PORT_SFF1;

    if ( route_id == ROUTE_ENCODER_DECODER )
        return PORT_ENCODER;

    if ( route_id == ROUTE_DECODER_SFF3 )
        return PORT_DECODER;

    return TOTAL_PORTS;
}

static inline struct telemetry_csv *telemetry_slot( int route_id, uint32_t frame_id ) {
    
    // Purpose: It resolves the precise diagnostic storage position based on route & shot identifiers
    
    if ( route_id < 0 || route_id >= TOTAL_ROUTES || frame_id == 0 || frame_id == END_OF_STREAM )
        return NULL;

    uint32_t idx = ( frame_id - 1 ) % K_FRAMES;

    if ( route_id == ROUTE_SFF1_ENCODER )
        return &telemetry_sff1_enc[ idx ];

    if ( route_id == ROUTE_ENCODER_DECODER )
        return &telemetry_enc_dec[ idx ];

    if ( route_id == ROUTE_DECODER_SFF3 )
        return &telemetry_dec_sff3[ idx ];

    return NULL;
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

static void tables_init() {

    // Purpose: It initializes connection-specific "Eth" / "IPv4" / "UDP" routing elements to bind physical ingress, validate transport endpoints, & rewrite hop-local network components

    struct rte_ether_addr sff1_sff2_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x01 } };
    struct rte_ether_addr sff2_sff1_mac = { { 0x00, 0x00, 0x00, 0x00, 0x02, 0x02 } };

    src_macs[ PORT_SFF1 ] = sff2_sff1_mac;
    dst_macs[ PORT_SFF1 ] = sff1_sff2_mac;

    src_ips[ PORT_SFF1 ] = SFF2_SFF1_IP;
    dst_ips[ PORT_SFF1 ] = SFF1_SFF2_IP;

    ingress_src_ports[ PORT_SFF1 ] = SFF1_SFF2_PORT;
    ingress_dst_ports[ PORT_SFF1 ] = SFF2_SFF1_PORT;
    
    egress_src_ports[ PORT_SFF1 ] = SFF2_SFF1_PORT;
    egress_dst_ports[ PORT_SFF1 ] = SFF1_SFF2_PORT;

    struct rte_ether_addr encoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x01 } };
    struct rte_ether_addr sff2_encoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x02 } };

    src_macs[ PORT_ENCODER ] = sff2_encoder_mac;
    dst_macs[ PORT_ENCODER ] = encoder_mac;

    src_ips[ PORT_ENCODER ] = SFF2_ENCODER_IP;
    dst_ips[ PORT_ENCODER ] = ENCODER_IP;

    ingress_src_ports[ PORT_ENCODER ] = ENCODER_PORT;
    ingress_dst_ports[ PORT_ENCODER ] = SFF2_ENCODER_PORT;
    
    egress_src_ports[ PORT_ENCODER ] = SFF2_ENCODER_PORT;
    egress_dst_ports[ PORT_ENCODER ] = ENCODER_PORT;

    struct rte_ether_addr decoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x04, 0x01 } };
    struct rte_ether_addr sff2_decoder_mac = { { 0x00, 0x00, 0x00, 0x00, 0x04, 0x02 } };

    src_macs[ PORT_DECODER ] = sff2_decoder_mac;
    dst_macs[ PORT_DECODER ] = decoder_mac;

    src_ips[ PORT_DECODER ] = SFF2_DECODER_IP;
    dst_ips[ PORT_DECODER ] = DECODER_IP;

    ingress_src_ports[ PORT_DECODER ] = DECODER_PORT;
    ingress_dst_ports[ PORT_DECODER ] = SFF2_DECODER_PORT;
    
    egress_src_ports[ PORT_DECODER ] = SFF2_DECODER_PORT;
    egress_dst_ports[ PORT_DECODER ] = DECODER_PORT;

    struct rte_ether_addr sff2_sff3_mac = { { 0x00, 0x00, 0x00, 0x00, 0x05, 0x02 } };
    struct rte_ether_addr sff3_sff2_mac = { { 0x00, 0x00, 0x00, 0x00, 0x05, 0x01 } };

    src_macs[ PORT_SFF3 ] = sff2_sff3_mac;
    dst_macs[ PORT_SFF3 ] = sff3_sff2_mac;

    src_ips[ PORT_SFF3 ] = SFF2_SFF3_IP;
    dst_ips[ PORT_SFF3 ] = SFF3_SFF2_IP;

    ingress_src_ports[ PORT_SFF3 ] = SFF3_SFF2_PORT;
    ingress_dst_ports[ PORT_SFF3 ] = SFF2_SFF3_PORT;
    
    egress_src_ports[ PORT_SFF3 ] = SFF2_SFF3_PORT;
    egress_dst_ports[ PORT_SFF3 ] = SFF3_SFF2_PORT;
}

static void header_init() {
    for ( uint16_t port = 0; port < TOTAL_PORTS; port++ ) {
        struct net_hdr *hdr = &header_template[ port ];
        memset( hdr, 0, sizeof( *hdr ) );

        rte_ether_addr_copy( &src_macs[ port ], &hdr -> ethernet.src_addr );
        rte_ether_addr_copy( &dst_macs[ port ], &hdr -> ethernet.dst_addr );

        hdr -> ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

        hdr -> ipv4.version_ihl = 0x45;
        hdr -> ipv4.time_to_live = 64;
        hdr -> ipv4.next_proto_id = IPPROTO_UDP;

        hdr -> ipv4.src_addr = rte_cpu_to_be_32( src_ips[ port ] );
        hdr -> ipv4.dst_addr = rte_cpu_to_be_32( dst_ips[ port ] );

        hdr -> udp.src_port = rte_cpu_to_be_16( egress_src_ports[ port ] );
        hdr -> udp.dst_port = rte_cpu_to_be_16( egress_dst_ports[ port ] );

        hdr -> udp.dgram_cksum = 0;
    }
}

static inline void update_logged_frame( int route_id, uint32_t frame_id ) {
    
    // Purpose: It advances the highest exportable shot key independently for each telemetry-enabled course
    
    if ( frame_id == 0 || frame_id == END_OF_STREAM )
        return;

    uint32_t bounded_id = ( frame_id > K_FRAMES ) ? K_FRAMES : frame_id;

    if ( route_id == ROUTE_SFF1_ENCODER ) {
        if ( bounded_id > frames_sff1_enc )
            frames_sff1_enc = bounded_id;
    }
    else if ( route_id == ROUTE_ENCODER_DECODER ) {
        if ( bounded_id > frames_enc_dec )
            frames_enc_dec = bounded_id;
    }
    else if ( route_id == ROUTE_DECODER_SFF3 ) {
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

    fprintf( f, "frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_media_bytes;tx_media_bytes;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets\n" );

    for ( uint32_t i = 0; i < max_logged_frames; i++ ) {
        struct telemetry_csv *t = &data_array[ i ];

        if ( t -> frame_id == 0 )
            continue;

        fprintf( f, "%u;%u;%u;%u;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u;%u;%u;%u;%u;%u\n", t -> frame_id, t -> rx_complete, t -> tx_complete, t -> current_skip, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> original_points, t -> rx_points, t -> tx_points, t -> rx_media_bytes, t -> tx_media_bytes, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> reference_size_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> reference_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> reference_bitrate_mbps, t -> tx_duration_ms, t -> active_tx_ms, t -> active_process_ms, t -> cycle_ms, t -> header_wait_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> cycle_occupancy_pct, t -> camera_node_ms, t -> schedule_delay_ms, t -> instant_jitter_ms, t -> desynced_jitter_ms, t -> eth_errors, t -> ipv4_errors, t -> udp_errors, t -> nsh_errors, t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets );
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

    if ( route_id == ROUTE_SFF1_ENCODER ) {
        write_single_csv( SFF1_ENCODER_PATH, telemetry_sff1_enc, frames_sff1_enc );
        printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", SFF1_ENCODER_PATH );
    }
    else if ( route_id == ROUTE_ENCODER_DECODER ) {
        write_single_csv( ENCODER_DECODER_PATH, telemetry_enc_dec, frames_enc_dec );
        printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", ENCODER_DECODER_PATH );
    }
    else if ( route_id == ROUTE_DECODER_SFF3 ) {
        write_single_csv( DECODER_SFF3_PATH, telemetry_dec_sff3, frames_dec_sff3 );
        printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", DECODER_SFF3_PATH );
    }
}

static inline bool validate_network_header( struct rte_mbuf *m, uint16_t rx_port, struct rte_ether_hdr **eth_out, struct rte_ipv4_hdr **ipv4_out, struct rte_udp_hdr **udp_out, uint16_t *udp_length_out ) {

    // Purpose: It validates the "EtherType", "IPv4" / "UDP" structures, link-targeted pairs, & mutually consistent packet lengths prior to any path classification

    const size_t min_net_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

    if ( unlikely( rx_port >= TOTAL_PORTS ) )
        return false;

    if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < min_net_len ) ) {
        ipv4_errors[ rx_port ]++;
        return false;
    }

    struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

    if ( unlikely( !rte_is_same_ether_addr( &eth -> src_addr, &dst_macs[ rx_port ] ) || !rte_is_same_ether_addr( &eth -> dst_addr, &src_macs[ rx_port ] ) ) ) {
        eth_errors[ rx_port ]++;
        return false;
    }

    if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) {
        eth_errors[ rx_port ]++;
        return false;
    }

    struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

    if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( dst_ips[ rx_port ] ) || ipv4 -> dst_addr != rte_cpu_to_be_32( src_ips[ rx_port ] ) ) ) {
        ipv4_errors[ rx_port ]++;
        return false;
    }

    struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

    if ( unlikely( udp -> src_port != rte_cpu_to_be_16( ingress_src_ports[ rx_port ] ) || udp -> dst_port != rte_cpu_to_be_16( ingress_dst_ports[ rx_port ] ) ) ) {
        udp_errors[ rx_port ]++;
        return false;
    }

    uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
    uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );

    if ( unlikely( udp_length < sizeof( struct rte_udp_hdr ) || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length ) ) {
        udp_errors[ rx_port ]++;
        return false;
    }

    if ( unlikely( rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + ipv4_length ) ) {
        ipv4_errors[ rx_port ]++;
        return false;
    }

    *eth_out = eth;
    *ipv4_out = ipv4;
    *udp_out = udp;
    *udp_length_out = udp_length;

    return true;
}

static inline bool capture_proxy_state( uint32_t frame_id, const struct nsh_hdr *nsh ) {

    // Purpose: It preserves the post-redirecting "NSH" condition before removing encapsulation for unaware service functions

    struct proxy_context *ctx = proxy_context_slot( frame_id );

    if ( ctx == NULL )
        return false;

    uint32_t sph = rte_be_to_cpu_32( nsh -> serv_path_hdr );
    uint8_t si = sph & 0xFF;
    uint8_t ttl = get_nsh_ttl( nsh );

    if ( si != MAIN_SI_SFF1 || ttl == 0 )
        return false;

    if ( ctx -> valid && ctx -> frame_id == frame_id && ctx -> si != MAIN_SI_SFF1 )
        return false;

    ctx -> frame_id = frame_id;
    ctx -> ttl = ttl;
    ctx -> si = MAIN_SI_SFF1;
    ctx -> valid = true;

    return true;
}

static inline bool advance_proxy_state( uint32_t frame_id, uint8_t expected_si, uint8_t next_si ) {

    // Purpose: It performs the "SI" transition & applies the next SFF forwarding-hop reduction exactly once per frame condition

    struct proxy_context *ctx = proxy_context_slot( frame_id );

    if ( ctx == NULL || !ctx -> valid || ctx -> frame_id != frame_id )
        return false;

    if ( ctx -> si == next_si )
        return true;

    if ( ctx -> si != expected_si || ctx -> ttl <= 1 )
        return false;

    ctx -> si = next_si;
    ctx -> ttl--;

    return true;
}

static inline bool rebuild_packet( struct rte_mbuf *m, size_t strip_len, uint16_t tx_port ) {

    // Purpose: It removes the incoming service-chain envelope, safeguarding application metadata & payload bytes, & reconstructs the plain "Eth" / "IPv4" / "UDP" headers

    if ( unlikely( rte_pktmbuf_adj( m, strip_len ) == NULL ) )
        return false;

    uint16_t payload_len = ( uint16_t )rte_pktmbuf_pkt_len( m );
    char *new_header_start = rte_pktmbuf_prepend( m, sizeof( struct net_hdr ) );

    if ( unlikely( new_header_start == NULL ) )
        return false;

    struct net_hdr *hdr = ( struct net_hdr * )new_header_start;
    
    rte_memcpy( hdr, &header_template[ tx_port ], sizeof( *hdr ) );

    uint16_t udp_length = sizeof( struct rte_udp_hdr ) + payload_len;

    hdr -> udp.dgram_len = rte_cpu_to_be_16( udp_length );
    
    hdr -> ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + udp_length );
    hdr -> ipv4.hdr_checksum = 0;
    hdr -> ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> ipv4 );

    return true;
}

static inline bool enforce_proxy_nsh( struct rte_mbuf *m, uint16_t tx_port, const struct proxy_context *ctx ) {

    // Purpose: It reintroduces the base "NSH" using surrogate-maintained variables ( e.g., "SPI" ) when primary traffic departs the nearby clueless-SF attachment domain

    if ( ctx == NULL || !ctx -> valid || ctx -> ttl == 0 )
        return false;

    if ( unlikely( rte_pktmbuf_adj( m, sizeof( struct net_hdr ) ) == NULL ) )
        return false;

    uint16_t application_len = ( uint16_t )rte_pktmbuf_pkt_len( m );
    size_t new_outer_len = sizeof( struct net_hdr ) + sizeof( struct nsh_hdr );
    char *new_header_start = rte_pktmbuf_prepend( m, new_outer_len );

    if ( unlikely( new_header_start == NULL ) )
        return false;

    struct net_hdr *net = ( struct net_hdr * )new_header_start;
    struct nsh_hdr *nsh = ( struct nsh_hdr * )( net + 1 );

    rte_memcpy( net, &header_template[ tx_port ], sizeof( *net ) );

    memset( nsh, 0, sizeof( *nsh ) );
    
    uint16_t udp_payload_len = sizeof( struct nsh_hdr ) + application_len;
    uint16_t udp_length = sizeof( struct rte_udp_hdr ) + udp_payload_len;

    net -> udp.dgram_len = rte_cpu_to_be_16( udp_length );
    
    net -> ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + udp_length );
    net -> ipv4.hdr_checksum = 0;
    net -> ipv4.hdr_checksum = rte_ipv4_cksum( &net -> ipv4 );

    nsh -> base_flags_ttl_len = nsh_base_field( ctx -> ttl, sizeof( struct nsh_hdr ) / 4 );
    nsh -> md_type = MD_TYPE_2;
    nsh -> next_protocol = NEXT_PROTOCOL_EXPERIMENT_1;
    nsh -> serv_path_hdr = htonl( ( MAIN_SPI << 8 ) | ctx -> si );

    return true;
}

static inline void revise_network_header( uint16_t tx_port, struct rte_ether_hdr *eth, struct rte_ipv4_hdr *ipv4, struct rte_udp_hdr *udp ) {

    // Purpose: It rewrites "Eth" / "IPv4" / "UDP" endpoints for decapsulated proxy traffic while conserving application bytes

    rte_ether_addr_copy( &src_macs[ tx_port ], &eth -> src_addr );
    rte_ether_addr_copy( &dst_macs[ tx_port ], &eth -> dst_addr );

    ipv4 -> src_addr = rte_cpu_to_be_32( src_ips[ tx_port ] );
    ipv4 -> dst_addr = rte_cpu_to_be_32( dst_ips[ tx_port ] );
    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );

    udp -> src_port = rte_cpu_to_be_16( egress_src_ports[ tx_port ] );
    udp -> dst_port = rte_cpu_to_be_16( egress_dst_ports[ tx_port ] );
    udp -> dgram_cksum = 0;
}

static inline bool dispatch_temporal_control( uint16_t tx_port, struct rte_mbuf *m ) {

    // Purpose: It transmits a low-rate management datagram, applying bounded re-presentation when immediate submission is not accepted

    uint16_t retries = 0;

    while ( 1 ) {
        uint16_t nb_tx = rte_eth_tx_burst( tx_port, 0, &m, 1 );

        if ( nb_tx == 1 )
            return true;

        if ( ++retries > MAX_ZERO_ACCEPTS ) {
            rte_pktmbuf_free( m );
            return false;
        }

        rte_pause();
    }
}

static inline void classify_temporal_control( struct rte_mbuf *m ) {

    // Purpose: It groups an exact 16-byte Encoder-originated update by replacing its shell with required condition ( "SPI 200", "SI 255" ) & forwarding the payload unchanged to SFF1

    const size_t old_net_len = sizeof( struct net_hdr );
    const size_t new_outer_len = sizeof( struct temporal_hdr );

    if ( unlikely( rte_pktmbuf_adj( m, old_net_len ) == NULL ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( rte_pktmbuf_pkt_len( m ) != sizeof( struct temporal_payload ) ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    char *new_header_start = rte_pktmbuf_prepend( m, new_outer_len );

    if ( unlikely( new_header_start == NULL ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    struct temporal_hdr *hdr = ( struct temporal_hdr * )new_header_start;
    memset( hdr, 0, sizeof( *hdr ) );

    rte_ether_addr_copy( &src_macs[ PORT_SFF1 ], &hdr -> net.ethernet.src_addr );
    rte_ether_addr_copy( &dst_macs[ PORT_SFF1 ], &hdr -> net.ethernet.dst_addr );
    hdr -> net.ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> net.ipv4.version_ihl = 0x45;
    hdr -> net.ipv4.time_to_live = 64;
    hdr -> net.ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> net.ipv4.src_addr = rte_cpu_to_be_32( SFF2_SFF1_IP );
    hdr -> net.ipv4.dst_addr = rte_cpu_to_be_32( SFF1_SFF2_IP );

    uint16_t udp_length = sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct temporal_payload );

    hdr -> net.udp.src_port = rte_cpu_to_be_16( SFF2_SFF1_PORT );
    hdr -> net.udp.dst_port = rte_cpu_to_be_16( SFF1_SFF2_PORT );
    hdr -> net.udp.dgram_len = rte_cpu_to_be_16( udp_length );
    hdr -> net.udp.dgram_cksum = 0;

    hdr -> net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + udp_length );
    hdr -> net.ipv4.hdr_checksum = 0;
    hdr -> net.ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> net.ipv4 );

    hdr -> nsh.base_flags_ttl_len = nsh_base_field( DEFAULT_TTL, sizeof( struct nsh_hdr ) / 4 );
    hdr -> nsh.md_type = MD_TYPE_2;
    hdr -> nsh.next_protocol = NEXT_PROTOCOL_EXPERIMENT_1;
    hdr -> nsh.serv_path_hdr = htonl( ( TEMPORAL_SPI << 8 ) | TEMPORAL_SI );

    dispatch_temporal_control( PORT_SFF1, m );
}

static inline enum packet_type classify_nsh_packet( uint16_t rx_port, struct rte_udp_hdr *udp, uint16_t udp_length, struct nsh_hdr **nsh_out, uint16_t *nsh_length_out, uint16_t *target_tx_port_out ) {

    // Purpose: It validates "NSH" exclusively on SFC-aware attachments ( e.g., geometry from SFF1 & control from SFF3 ). 
    //          Encoder & Decoder ties remain standard "UDP" proxy domains

    uint16_t udp_payload_length = udp_length - sizeof( struct rte_udp_hdr );

    if ( unlikely( udp_payload_length < sizeof( struct nsh_hdr ) ) )
        return INVALID;

    struct nsh_hdr *nsh = ( struct nsh_hdr * )( udp + 1 );
    
    uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );
    uint8_t version = ( base >> 14 ) & 0x03;
    bool oam = ( base & 0x2000 ) != 0;
    uint8_t ttl = ( base >> 6 ) & 0x3F;
    uint8_t length_words = base & 0x3F;

    if ( unlikely( version != 0 || oam || length_words == 0 ) )
        return INVALID;
    
    uint16_t nsh_length = nsh_length_bytes( nsh );

    if ( unlikely( nsh_length < sizeof( struct nsh_hdr ) || nsh_length > udp_payload_length || nsh -> md_type != MD_TYPE_2 || nsh -> next_protocol != NEXT_PROTOCOL_EXPERIMENT_1 ) )
        return INVALID;

    uint32_t sph = rte_be_to_cpu_32( nsh -> serv_path_hdr );
    uint32_t spi = sph >> 8;
    uint8_t si = sph & 0xFF;

    enum packet_type type = INVALID;
    uint16_t target_tx_port = TOTAL_PORTS;

    if ( spi == MAIN_SPI && si == MAIN_SI_SFF1 && rx_port == PORT_SFF1 ) {
        if ( unlikely( nsh_length != MAIN_GEOMETRY_SIZE ) )
            return INVALID;

        struct nsh_md2_ctx_hdr *geo_ctx = ( struct nsh_md2_ctx_hdr * )( nsh + 1 );

        if ( unlikely( geo_ctx -> metadata_class != rte_cpu_to_be_16( MD_CLASS_EXPERIMENTAL ) || geo_ctx -> type != MD_TYPE_GEOMETRY || ( geo_ctx -> u_length & 0x7F ) != sizeof( struct geo_agg_hdr ) ) )
            return INVALID;

        type = MAIN_SFF1_ENCODER;
        target_tx_port = PORT_ENCODER;
    }
    else if ( spi == POSE_SPI && si == POSE_SI && rx_port == PORT_SFF3 ) {
        if ( unlikely( nsh_length != sizeof( struct nsh_hdr ) || udp_payload_length != nsh_length + sizeof( struct pose_payload ) ) )
            return INVALID;

        type = POSE_SFF3_DECODER;
        target_tx_port = PORT_DECODER;
    }

    if ( type == INVALID )
        return INVALID;

    if ( unlikely( !decrement_nsh_ttl( nsh ) ) )
        return INVALID;

    *nsh_out = nsh;
    *nsh_length_out = nsh_length;
    *target_tx_port_out = target_tx_port;

    return type;
}

static inline bool flush_tx_burst( uint16_t tx_port, struct rte_mbuf **tx_bufs, uint32_t *tx_points_buf, uint32_t *tx_media_bytes_buf, int *burst_idx, struct telemetry_csv *frame_t, int route_id, uint64_t first_tx_cycle[ TOTAL_ROUTES ], uint64_t last_tx_cycle[ TOTAL_ROUTES ], uint64_t active_tx_cycles[ TOTAL_ROUTES ] ) {
    if ( *burst_idx == 0 )
        return false;

    uint16_t sent = 0;
    uint16_t zero_accept_streak = 0;

    bool tx_exhausted = false;
    bool is_resubmission = false;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint16_t requested_packets = *burst_idx - sent;

        if ( is_resubmission && frame_t != NULL ) {
            frame_t -> tx_resubmit_calls++;
            frame_t -> tx_resubmitted_packets += requested_packets;
        }

        uint64_t t_tx_start = rte_get_timer_cycles();

        if ( frame_t != NULL && route_id >= 0 && route_id < TOTAL_ROUTES && first_tx_cycle[ route_id ] == 0 )
            first_tx_cycle[ route_id ] = t_tx_start;

        uint16_t nb_tx = rte_eth_tx_burst( tx_port, 0, &tx_bufs[ sent ], requested_packets );

        uint64_t t_tx_end = rte_get_timer_cycles();

        if ( route_id >= 0 && route_id < TOTAL_ROUTES )
            active_tx_cycles[ route_id ] += t_tx_end - t_tx_start;

        if ( nb_tx > 0 ) {
            if ( frame_t != NULL && route_id >= 0 && route_id < TOTAL_ROUTES ) {
                last_tx_cycle[ route_id ] = t_tx_end;
                frame_t -> tx_packets += nb_tx;

                for ( uint16_t j = 0; j < nb_tx; j++ ) {
                    frame_t -> tx_points += tx_points_buf[ sent + j ];
                    frame_t -> tx_media_bytes += tx_media_bytes_buf[ sent + j ];
                }

                if ( nb_tx < requested_packets )
                    frame_t -> tx_partial_accepts++;
            }

            sent += nb_tx;
            zero_accept_streak = 0;
            is_resubmission = nb_tx < requested_packets;
        }
        else {
            if ( frame_t != NULL )
                frame_t -> tx_zero_accepts++;

            is_resubmission = true;

            if ( ++zero_accept_streak > MAX_ZERO_ACCEPTS ) {
                for ( int k = sent; k < *burst_idx; k++ )
                    rte_pktmbuf_free( tx_bufs[ k ] );

                tx_exhausted = true;
                break;
            }

            uint16_t pause_count = ( zero_accept_streak < pause_window ) ? zero_accept_streak : pause_window;

            for ( uint16_t p = 0; p < pause_count; p++ )
                rte_pause();
        }
    }

    *burst_idx = 0;

    return tx_exhausted;
}

static inline void flush_port( uint16_t tx_port, struct rte_mbuf **tx_bufs, uint32_t *tx_points_buf, uint32_t *tx_media_bytes_buf, int *burst_idx, struct telemetry_csv **owner_t, int8_t *owner_route, uint64_t first_tx_cycle[ TOTAL_ROUTES ], uint64_t last_tx_cycle[ TOTAL_ROUTES ], uint64_t active_tx_cycles[ TOTAL_ROUTES ], uint64_t active_process_cycles[ TOTAL_ROUTES ], uint64_t last_activity_cycles[ TOTAL_ROUTES ] ) {

    // Purpose: It preserves route & element ownership across buffered transmission bursts, preserving shared egress queues from merging packets of independent telemetry contexts

    if ( *burst_idx == 0 ) {
        *owner_t = NULL;
        *owner_route = -1;
        return;
    }

    int route_id = *owner_route;
    uint64_t t_active_start = 0;

    if ( route_id >= 0 && route_id < TOTAL_ROUTES )
        t_active_start = rte_get_timer_cycles();

    bool tx_exhausted = flush_tx_burst( tx_port, tx_bufs, tx_points_buf, tx_media_bytes_buf, burst_idx, *owner_t, route_id, first_tx_cycle, last_tx_cycle, active_tx_cycles );

    if ( route_id >= 0 && route_id < TOTAL_ROUTES ) {
        uint64_t t_active_end = rte_get_timer_cycles();
        active_process_cycles[ route_id ] += t_active_end - t_active_start;

        if ( tx_exhausted && t_active_end > last_activity_cycles[ route_id ] )
            last_activity_cycles[ route_id ] = t_active_end;
    }

    *owner_t = NULL;
    *owner_route = -1;
}

static inline void finalize_metrics( int route_id, uint64_t timer_hz, uint64_t t_frame_arrival[ TOTAL_ROUTES ], uint64_t last_rx_cycle[ TOTAL_ROUTES ], uint64_t t_session_start[ TOTAL_ROUTES ], uint32_t first_arrival_frame[ TOTAL_ROUTES ], uint32_t current_frame_id[ TOTAL_ROUTES ], uint16_t frame_temporal_skip[ TOTAL_ROUTES ], uint64_t first_tx_cycle[ TOTAL_ROUTES ], uint64_t last_tx_cycle[ TOTAL_ROUTES ], uint64_t last_activity_cycles[ TOTAL_ROUTES ], uint64_t active_process_cycles[ TOTAL_ROUTES ], uint64_t active_tx_cycles[ TOTAL_ROUTES ], uint64_t t_cycle_start[ TOTAL_ROUTES ], double current_latency_ms[ TOTAL_ROUTES ], double current_jitter_ms[ TOTAL_ROUTES ], double jitter_ms[ TOTAL_ROUTES ], bool frame_sequence_ok[ TOTAL_ROUTES ], uint32_t frame_valid_points[ TOTAL_ROUTES ], uint32_t eth_errors_start[ TOTAL_ROUTES ], uint32_t ipv4_errors_start[ TOTAL_ROUTES ], uint32_t udp_errors_start[ TOTAL_ROUTES ], uint32_t nsh_errors_start[ TOTAL_ROUTES ] ) {

    // Purpose: It derives per-route timing, throughput, bitrate & data completeness indicators from accumulated counters

    uint32_t frame_id = current_frame_id[ route_id ];

    if ( frame_id == END_OF_STREAM || frame_id == 0 )
        return;

    struct telemetry_csv *t = telemetry_slot( route_id, frame_id );

    if ( t == NULL || t -> frame_id != frame_id )
        return;

    uint64_t t_end = last_tx_cycle[ route_id ];

    if ( last_activity_cycles[ route_id ] > t_end )
        t_end = last_activity_cycles[ route_id ];

    if ( t_end == 0 )
        t_end = t_frame_arrival[ route_id ];

    double tx_duration_sec = 0.0;

    if ( first_tx_cycle[ route_id ] > 0 && last_tx_cycle[ route_id ] >= first_tx_cycle[ route_id ] )
        tx_duration_sec = ( double )( last_tx_cycle[ route_id ] - first_tx_cycle[ route_id ] ) / timer_hz;

    double residency_sec = ( t_end >= t_frame_arrival[ route_id ] ) ? ( double )( t_end - t_frame_arrival[ route_id ] ) / timer_hz : 0.0;
    double receive_sec = ( last_rx_cycle[ route_id ] >= t_frame_arrival[ route_id ] ) ? ( double )( last_rx_cycle[ route_id ] - t_frame_arrival[ route_id ] ) / timer_hz : 0.0;
    double cycle_sec = ( t_end >= t_cycle_start[ route_id ] ) ? ( double )( t_end - t_cycle_start[ route_id ] ) / timer_hz : 0.0;
    double header_wait_sec = ( cycle_sec > residency_sec ) ? cycle_sec - residency_sec : 0.0;
    double active_process_sec = ( double )active_process_cycles[ route_id ] / timer_hz;
    double active_tx_sec = ( double )active_tx_cycles[ route_id ] / timer_hz;

    uint64_t logical_rx_frame_bytes = 0;
    uint64_t logical_tx_frame_bytes = 0;
    uint64_t network_frame_bytes = 0;
    uint64_t reference_frame_bytes = 0;

    if ( route_id == ROUTE_SFF1_ENCODER ) {
        uint64_t logical_rx_payload_bytes = ( uint64_t )t -> rx_points * sizeof( struct point_tx );
        uint64_t logical_tx_payload_bytes = ( uint64_t )t -> tx_points * sizeof( struct point_tx );

        logical_rx_frame_bytes = logical_rx_payload_bytes + ( t -> rx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );
        logical_tx_frame_bytes = logical_tx_payload_bytes + ( t -> tx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );

        network_frame_bytes = logical_tx_payload_bytes + ( ( uint64_t )t -> tx_packets * ( sizeof( struct net_hdr ) + sizeof( struct geo_agg_hdr ) + sizeof( struct cam_hdr ) ) );

        reference_frame_bytes = ( ( uint64_t )t -> original_points * sizeof( struct point_tx ) ) + ( t -> original_points > 0 ? sizeof( struct cam_hdr ) : 0 );

        t -> payload_bytes = ( uint32_t )logical_rx_payload_bytes;

        t -> data_integrity_pct = ( t -> original_points > 0 ) ? ( ( double )t -> rx_points / ( double )t -> original_points ) * 100.0 : 0.0;

        uint32_t expected_packets = ( t -> original_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET;
        
        bool rx_complete = t -> original_points > 0 && t -> rx_points == t -> original_points && t -> rx_packets == expected_packets && frame_sequence_ok[ route_id ];
        bool tx_complete = rx_complete && t -> tx_points == t -> rx_points && t -> tx_packets == t -> rx_packets;

        t -> rx_complete = rx_complete ? 1 : 0;
        t -> tx_complete = tx_complete ? 1 : 0;
    }
    else if ( route_id == ROUTE_ENCODER_DECODER ) {
        logical_rx_frame_bytes = ( uint64_t )t -> rx_media_bytes + ( t -> rx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );
        logical_tx_frame_bytes = ( uint64_t )t -> tx_media_bytes + ( t -> tx_packets > 0 ? sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) : 0 );

        network_frame_bytes = ( uint64_t )t -> tx_media_bytes + ( ( uint64_t )t -> tx_packets * ( sizeof( struct net_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) );

        reference_frame_bytes = logical_rx_frame_bytes;

        t -> payload_bytes = t -> rx_media_bytes;

        t -> data_integrity_pct = ( t -> rx_media_bytes > 0 ) ? ( ( double )t -> tx_media_bytes / ( double )t -> rx_media_bytes ) * 100.0 : 0.0;

        bool rx_complete = t -> rx_media_bytes > 0 && t -> rx_packets > 0;
        bool tx_complete = rx_complete && t -> tx_media_bytes == t -> rx_media_bytes && t -> tx_packets == t -> rx_packets;

        t -> rx_complete = rx_complete ? 1 : 0;
        t -> tx_complete = tx_complete ? 1 : 0;
    }
    else if ( route_id == ROUTE_DECODER_SFF3 ) {
        uint32_t expected_points = frame_valid_points[ route_id ];

        uint64_t logical_rx_payload_bytes = ( uint64_t )t -> rx_points * sizeof( struct point_tx );
        uint64_t logical_tx_payload_bytes = ( uint64_t )t -> tx_points * sizeof( struct point_tx );

        logical_rx_frame_bytes = logical_rx_payload_bytes + ( t -> rx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );
        logical_tx_frame_bytes = logical_tx_payload_bytes + ( t -> tx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );

        network_frame_bytes = logical_tx_payload_bytes + ( ( uint64_t )t -> tx_packets * ( sizeof( struct net_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct dec_hdr ) ) );

        reference_frame_bytes = ( ( uint64_t )expected_points * sizeof( struct point_tx ) ) + ( t -> rx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );

        t -> payload_bytes = ( uint32_t )logical_rx_payload_bytes;

        if ( expected_points > 0 ) {
            t -> data_integrity_pct = ( ( double )t -> rx_points / ( double )expected_points ) * 100.0;

            uint32_t expected_packets = ( expected_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET;
            
            bool rx_complete = t -> rx_points == expected_points && t -> rx_packets == expected_packets && frame_sequence_ok[ route_id ];
            bool tx_complete = rx_complete && t -> tx_points == t -> rx_points && t -> tx_packets == t -> rx_packets;
            
            t -> rx_complete = rx_complete ? 1 : 0;
            t -> tx_complete = tx_complete ? 1 : 0;
        }
        else {
            bool rx_complete = t -> rx_packets == 1 && t -> rx_points == 0 && frame_sequence_ok[ route_id ];
            bool tx_complete = rx_complete && t -> tx_points == 0 && t -> tx_packets == t -> rx_packets;

            t -> data_integrity_pct = rx_complete ? 100.0 : 0.0;
            
            t -> rx_complete = rx_complete ? 1 : 0;
            t -> tx_complete = tx_complete ? 1 : 0;
        }
    }
    else
        return;

    t -> tx_duration_ms = tx_duration_sec * 1000.0;
    t -> active_tx_ms = active_tx_sec * 1000.0;
    t -> active_process_ms = active_process_sec * 1000.0;
    t -> total_residency_ms = residency_sec * 1000.0;
    t -> cycle_ms = cycle_sec * 1000.0;
    t -> header_wait_ms = header_wait_sec * 1000.0;
    t -> node_exit_timestamp = ( double )t_end / timer_hz;
    t -> camera_node_ms = current_latency_ms[ route_id ];

    uint32_t schedule_frame_offset = frame_id - first_arrival_frame[ route_id ];
    double real_exit_sec = ( t_end >= t_session_start[ route_id ] ) ? ( double )( t_end - t_session_start[ route_id ] ) / timer_hz : 0.0;
    double ideal_start_sec = ( double )schedule_frame_offset / TARGET_FPS;

    t -> schedule_delay_ms = ( real_exit_sec - ideal_start_sec ) * 1000.0;
    t -> instant_jitter_ms = current_jitter_ms[ route_id ];
    t -> desynced_jitter_ms = jitter_ms[ route_id ];
    t -> internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_rx_frame_bytes / 1000000.0 ) / receive_sec : 0.0;

    uint16_t temporal_skip = frame_temporal_skip[ route_id ];

    if ( temporal_skip == 0 )
        temporal_skip = 1;

    double effective_fps = TARGET_FPS / temporal_skip;

    t -> logical_bitrate_mbps = ( logical_tx_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
    t -> network_bitrate_mbps = ( network_frame_bytes * 8.0 * effective_fps ) / 1000000.0;

    t -> reference_size_bytes = ( uint32_t )reference_frame_bytes;
    t -> reference_throughput_mbs = ( residency_sec > 0.0 ) ? ( ( double )reference_frame_bytes / 1000000.0 ) / residency_sec : 0.0;
    t -> reference_bitrate_mbps = ( reference_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
    t -> cycle_occupancy_pct = ( cycle_sec > 0.0 ) ? ( residency_sec / cycle_sec ) * 100.0 : 0.0;

    uint16_t ingress_port = port_by_route( route_id );

    if ( ingress_port < TOTAL_PORTS ) {
        t -> eth_errors = eth_errors[ ingress_port ] - eth_errors_start[ route_id ];
        t -> ipv4_errors = ipv4_errors[ ingress_port ] - ipv4_errors_start[ route_id ];
        t -> udp_errors = udp_errors[ ingress_port ] - udp_errors_start[ route_id ];
        t -> nsh_errors = nsh_errors[ ingress_port ] - nsh_errors_start[ route_id ];
    }

    // Efficiency = productive active work divided by the complete frame residency on this SFF2 route. active_tx_ms remains a diagnostic subset.
    t -> node_efficiency_pct = ( residency_sec > 0.0 ) ? ( active_process_sec / residency_sec ) * 100.0 : 0.0;

    t_cycle_start[ route_id ] = t_end;
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();

    struct rte_mbuf *bufs[ BURST_SIZE ];

    struct rte_mbuf *tx_bufs[ TOTAL_PORTS ][ BURST_SIZE ];
    uint32_t tx_points_buf[ TOTAL_PORTS ][ BURST_SIZE ];
    uint32_t tx_media_bytes_buf[ TOTAL_PORTS ][ BURST_SIZE ];

    int burst_idx[ TOTAL_PORTS ] = { 0, 0, 0, 0 };

    struct telemetry_csv *burst_owner_t[ TOTAL_PORTS ] = { NULL, NULL, NULL, NULL };
    int8_t burst_owner_route[ TOTAL_PORTS ] = { -1, -1, -1, -1 };

    uint32_t current_frame_id[ TOTAL_ROUTES ] = { END_OF_STREAM, END_OF_STREAM, END_OF_STREAM };
    uint32_t frame_original_points[ TOTAL_ROUTES ] = { 0, 0, 0 };
    
    uint32_t frame_arrived_points[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t frame_eroded_points[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t frame_valid_points[ TOTAL_ROUTES ] = { 0, 0, 0 };
    
    uint16_t frame_temporal_skip[ TOTAL_ROUTES ] = { 1, 1, 1 };

    uint64_t t_frame_arrival[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t last_rx_cycle[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t t_session_start[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t frames_received_count[ TOTAL_ROUTES ] = { 0, 0, 0 };

    double current_latency_ms[ TOTAL_ROUTES ] = { 0.0, 0.0, 0.0 };
    double current_jitter_ms[ TOTAL_ROUTES ] = { 0.0, 0.0, 0.0 };
    double jitter_ms[ TOTAL_ROUTES ] = { 0.0, 0.0, 0.0 };

    uint32_t frame_expected_sequence[ TOTAL_ROUTES ] = { 0, 0, 0 };
    bool frame_sequence_ok[ TOTAL_ROUTES ] = { true, true, true };

    uint64_t prev_arrival_cycles[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t first_arrival_frame[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t prev_arrival_frame[ TOTAL_ROUTES ] = { 0, 0, 0 };

    uint64_t first_tx_cycle[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t last_tx_cycle[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t last_activity_cycles[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint64_t active_process_cycles[ TOTAL_ROUTES ] = { 0, 0, 0 };

    uint64_t active_tx_cycles[ TOTAL_ROUTES ] = { 0, 0, 0 };

    // Main-chain error snapshots. Each route computes its CSV errors as current ingress-port counters minus these frame-start values.
    uint32_t eth_errors_start[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t ipv4_errors_start[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t udp_errors_start[ TOTAL_ROUTES ] = { 0, 0, 0 };
    uint32_t nsh_errors_start[ TOTAL_ROUTES ] = { 0, 0, 0 };

    uint64_t worker_start_cycle = rte_get_timer_cycles();
    uint64_t t_cycle_start[ TOTAL_ROUTES ] = { worker_start_cycle, worker_start_cycle, worker_start_cycle };

    printf( "[SYSTEM] Listening on every service-chain link...\n\n" );

    while ( 1 ) {
        if ( csv_written[ ROUTE_SFF1_ENCODER ] && csv_written[ ROUTE_ENCODER_DECODER ] && csv_written[ ROUTE_DECODER_SFF3 ] ) {
            rte_delay_us_sleep( 1000 );
            continue;
        }

        bool received_any_packet = false;

        for ( uint16_t rx_port = 0; rx_port < TOTAL_PORTS; rx_port++ ) {
            uint16_t nb_rx = rte_eth_rx_burst( rx_port, 0, bufs, BURST_SIZE );

            if ( nb_rx == 0 )
                continue;

            received_any_packet = true;

            for ( uint16_t i = 0; i < nb_rx; i++ ) {
                struct rte_mbuf *m = bufs[ i ];
                struct rte_ether_hdr *eth = NULL;
                struct rte_ipv4_hdr *ipv4 = NULL;
                struct rte_udp_hdr *udp = NULL;
                uint16_t udp_length = 0;

                if ( unlikely( !validate_network_header( m, rx_port, &eth, &ipv4, &udp, &udp_length ) ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                uint16_t udp_payload_length = udp_length - sizeof( struct rte_udp_hdr );

                if ( rx_port == PORT_ENCODER && udp_payload_length == sizeof( struct temporal_payload ) ) {
                    if ( burst_idx[ PORT_SFF1 ] > 0 )
                        flush_port( PORT_SFF1, tx_bufs[ PORT_SFF1 ], tx_points_buf[ PORT_SFF1 ], tx_media_bytes_buf[ PORT_SFF1 ], &burst_idx[ PORT_SFF1 ], &burst_owner_t[ PORT_SFF1 ], &burst_owner_route[ PORT_SFF1 ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );

                    classify_temporal_control( m );
                    continue;
                }

                if ( rx_port == PORT_ENCODER && udp_payload_length >= sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) {
                    struct cam_hdr *preroll_cam = ( struct cam_hdr * )( udp + 1 );
                    uint32_t preroll_frame_id = rte_be_to_cpu_32( preroll_cam -> frame_id );

                    if ( preroll_frame_id == FRAME_ID ) {
                        struct enc_hdr *preroll_enc = ( struct enc_hdr * )( preroll_cam + 1 );
                        uint16_t preroll_media_len = udp_payload_length - sizeof( struct cam_hdr ) - sizeof( struct enc_hdr );

                        if ( unlikely( rte_be_to_cpu_32( preroll_enc -> frame_id ) != FRAME_ID || preroll_cam -> sequence_number != 0 || rte_be_to_cpu_32( preroll_cam -> original_points ) != 0 || rte_be_to_cpu_32( preroll_cam -> points_in_packet ) != 0 || preroll_media_len == 0 || preroll_media_len > MTU_PAYLOAD_SIZE || preroll_media_len % TS_PACKET_SIZE != 0 ) ) {
                            rte_pktmbuf_free( m );
                            continue;
                        }

                        if ( burst_idx[ PORT_DECODER ] > 0 )
                            flush_port( PORT_DECODER, tx_bufs[ PORT_DECODER ], tx_points_buf[ PORT_DECODER ], tx_media_bytes_buf[ PORT_DECODER ], &burst_idx[ PORT_DECODER ], &burst_owner_t[ PORT_DECODER ], &burst_owner_route[ PORT_DECODER ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );

                        revise_network_header( PORT_DECODER, eth, ipv4, udp );
                        dispatch_temporal_control( PORT_DECODER, m );
                        continue;
                    }
                }

                struct nsh_hdr *nsh = NULL;
                uint16_t nsh_length = 0;

                uint16_t target_tx_port = TOTAL_PORTS;
                
                enum packet_type type = INVALID;

                if ( rx_port == PORT_ENCODER ) {
                    type = MAIN_ENCODER_DECODER;
                    target_tx_port = PORT_DECODER;
                }
                else if ( rx_port == PORT_DECODER ) {
                    type = MAIN_DECODER_SFF3;
                    target_tx_port = PORT_SFF3;
                }
                else
                    type = classify_nsh_packet( rx_port, udp, udp_length, &nsh, &nsh_length, &target_tx_port );

                if ( unlikely( type == INVALID ) ) {
                    rte_pktmbuf_free( m );
                    nsh_errors[ rx_port ]++;
                    continue;
                }

                if ( type == POSE_SFF3_DECODER ) {
                    if ( burst_idx[ target_tx_port ] > 0 )
                        flush_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );

                    size_t strip_len = sizeof( struct net_hdr ) + nsh_length;

                    if ( unlikely( !rebuild_packet( m, strip_len, target_tx_port ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    dispatch_temporal_control( target_tx_port, m );
                    continue;
                }

                int route_id = ( type == MAIN_SFF1_ENCODER ) ? ROUTE_SFF1_ENCODER : ( type == MAIN_ENCODER_DECODER ) ? ROUTE_ENCODER_DECODER : ROUTE_DECODER_SFF3;
                
                struct cam_hdr *cam = NULL;
                struct dec_hdr *dec = NULL;
                uint32_t actual_application_bytes = 0;

                if ( type == MAIN_SFF1_ENCODER ) {
                    size_t application_offset = sizeof( struct net_hdr ) + nsh_length;

                    if ( unlikely( rte_pktmbuf_pkt_len( m ) < application_offset + sizeof( struct cam_hdr ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    cam = ( struct cam_hdr * )( ( uint8_t * )nsh + nsh_length );
                    actual_application_bytes = udp_payload_length - nsh_length;
                }
                else if ( type == MAIN_ENCODER_DECODER ) {
                    if ( unlikely( udp_payload_length < sizeof( struct cam_hdr ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    cam = ( struct cam_hdr * )( udp + 1 );
                    actual_application_bytes = udp_payload_length;
                }
                else {
                    if ( unlikely( udp_payload_length < sizeof( struct dec_hdr ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    dec = ( struct dec_hdr * )( udp + 1 );
                    actual_application_bytes = udp_payload_length;
                }

                uint32_t f_id = rte_be_to_cpu_32( ( type == MAIN_DECODER_SFF3 ) ? dec -> frame_id : cam -> frame_id );

                if ( unlikely( f_id == 0 ) ) {
                    rte_pktmbuf_free( m );
                    nsh_errors[ rx_port ]++;
                    continue;
                }

                uint32_t current_packet_points = 0;
                uint32_t current_media_bytes = 0;
                uint32_t packet_sequence = 0;
                uint32_t packet_original_points = 0;
                uint32_t packet_arrived_points = 0;
                uint32_t packet_eroded_points = 0;
                uint32_t packet_valid_points = 0;
                uint64_t packet_camera_timestamp = 0;
                uint16_t packet_temporal_skip = 1;

                if ( type == MAIN_SFF1_ENCODER ) {
                    uint32_t points_in_packet = rte_be_to_cpu_32( cam -> points_in_packet );
                    uint32_t expected_application_bytes = sizeof( struct cam_hdr ) + points_in_packet * sizeof( struct point_tx );

                    if ( f_id != END_OF_STREAM ) {
                        if ( unlikely( points_in_packet > POINTS_PER_PACKET || actual_application_bytes != expected_application_bytes ) ) {
                            rte_pktmbuf_free( m );
                            nsh_errors[ rx_port ]++;
                            continue;
                        }
                    }
                    else if ( unlikely( points_in_packet != 0 || actual_application_bytes != sizeof( struct cam_hdr ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    current_packet_points = points_in_packet;
                    packet_sequence = rte_be_to_cpu_32( cam -> sequence_number );
                    packet_original_points = rte_be_to_cpu_32( cam -> original_points );
                    packet_valid_points = packet_original_points;
                    packet_camera_timestamp = rte_be_to_cpu_64( cam -> timestamp );
                    packet_temporal_skip = rte_be_to_cpu_16( cam -> temporal_skip );
                }
                else if ( type == MAIN_ENCODER_DECODER ) {
                    if ( f_id != END_OF_STREAM ) {
                        if ( unlikely( actual_application_bytes < sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) ) ) {
                            rte_pktmbuf_free( m );
                            nsh_errors[ rx_port ]++;
                            continue;
                        }

                        current_media_bytes = actual_application_bytes - sizeof( struct cam_hdr ) - sizeof( struct enc_hdr );
                    }
                    else if ( unlikely( actual_application_bytes != sizeof( struct cam_hdr ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    packet_original_points = rte_be_to_cpu_32( cam -> original_points );
                    packet_valid_points = packet_original_points;
                    packet_camera_timestamp = rte_be_to_cpu_64( cam -> timestamp );
                    packet_temporal_skip = rte_be_to_cpu_16( cam -> temporal_skip );
                }
                else {
                    uint32_t points_in_packet = rte_be_to_cpu_32( dec -> points_in_packet );
                    uint32_t original_points = rte_be_to_cpu_32( dec -> original_points );
                    uint32_t arrived_points = rte_be_to_cpu_32( dec -> arrived_points );
                    uint32_t eroded_points = rte_be_to_cpu_32( dec -> eroded_points );
                    uint32_t valid_points = rte_be_to_cpu_32( dec -> valid_points );
                    uint32_t expected_application_bytes = sizeof( struct dec_hdr ) + points_in_packet * sizeof( struct point_tx );
                    
                    if ( f_id != END_OF_STREAM ) {
                        bool counters_valid = valid_points <= eroded_points && eroded_points <= arrived_points;
                        bool empty_frame = valid_points == 0 && points_in_packet == 0 && rte_be_to_cpu_32( dec -> sequence_number ) == 0 && actual_application_bytes == sizeof( struct dec_hdr );
                        bool populated_frame = valid_points > 0 && points_in_packet > 0 && points_in_packet <= POINTS_PER_PACKET && actual_application_bytes == expected_application_bytes;

                        if ( unlikely( !counters_valid || ( !empty_frame && !populated_frame ) ) ) {
                            rte_pktmbuf_free( m );
                            nsh_errors[ rx_port ]++;
                            continue;
                        }
                    }
                    else if ( unlikely( original_points != 0 || arrived_points != 0 || eroded_points != 0 || valid_points != 0 || points_in_packet != 0 || actual_application_bytes != sizeof( struct dec_hdr ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    current_packet_points = points_in_packet;
                    packet_sequence = rte_be_to_cpu_32( dec -> sequence_number );
                    packet_original_points = original_points;
                    packet_arrived_points = arrived_points;
                    packet_eroded_points = eroded_points;
                    packet_valid_points = valid_points;
                    packet_camera_timestamp = rte_be_to_cpu_64( dec -> timestamp );
                    packet_temporal_skip = rte_be_to_cpu_16( dec -> temporal_skip );
                }

                uint64_t packet_arrival_cycles = rte_get_timer_cycles();
                bool is_eos_packet = f_id == END_OF_STREAM;
                bool trigger_eos_write = false;

                if ( f_id != current_frame_id[ route_id ] ) {
                    if ( current_frame_id[ route_id ] != END_OF_STREAM ) {
                        if ( burst_idx[ target_tx_port ] > 0 )
                            flush_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );

                        finalize_metrics( route_id, timer_hz, t_frame_arrival, last_rx_cycle, t_session_start, first_arrival_frame, current_frame_id, frame_temporal_skip, first_tx_cycle, last_tx_cycle, last_activity_cycles, active_process_cycles, active_tx_cycles, t_cycle_start, current_latency_ms, current_jitter_ms, jitter_ms, frame_sequence_ok, frame_valid_points, eth_errors_start, ipv4_errors_start, udp_errors_start, nsh_errors_start );
                    }

                    if ( is_eos_packet ) {
                        bool has_logs = ( route_id == ROUTE_SFF1_ENCODER && frames_sff1_enc > 0 ) || ( route_id == ROUTE_ENCODER_DECODER && frames_enc_dec > 0 ) || ( route_id == ROUTE_DECODER_SFF3 && frames_dec_sff3 > 0 );

                        trigger_eos_write = has_logs && !csv_written[ route_id ];
                        current_frame_id[ route_id ] = END_OF_STREAM;
                    }
                    else {
                         uint64_t cam_tx_cycles = packet_camera_timestamp;
                        uint64_t arrival_cycles = packet_arrival_cycles;

                        current_latency_ms[ route_id ] = ( arrival_cycles >= cam_tx_cycles ) ? ( ( double )( arrival_cycles - cam_tx_cycles ) / timer_hz ) * 1000.0 : 0.0;

                        if ( frames_received_count[ route_id ] == 0 ) {
                            t_session_start[ route_id ] = arrival_cycles;
                            first_arrival_frame[ route_id ] = f_id;
                            t_cycle_start[ route_id ] = arrival_cycles;
                        }

                        if ( frames_received_count[ route_id ] > 0 && prev_arrival_frame[ route_id ] > 0 && f_id > prev_arrival_frame[ route_id ] ) {
                            double real_interval_sec = ( double )( arrival_cycles - prev_arrival_cycles[ route_id ] ) / timer_hz;
                            double expected_interval_sec = ( double )( f_id - prev_arrival_frame[ route_id ] ) / TARGET_FPS;
                            double diff = real_interval_sec - expected_interval_sec;

                            current_jitter_ms[ route_id ] = ( diff < 0.0 ) ? -diff * 1000.0 : diff * 1000.0;
                        }
                        else {
                            current_jitter_ms[ route_id ] = 0.0;
                        }

                        jitter_ms[ route_id ] += ( current_jitter_ms[ route_id ] - jitter_ms[ route_id ] ) / 16.0;

                        prev_arrival_cycles[ route_id ] = arrival_cycles;
                        prev_arrival_frame[ route_id ] = f_id;
                        frames_received_count[ route_id ]++;

                        current_frame_id[ route_id ] = f_id;
                        t_frame_arrival[ route_id ] = arrival_cycles;
                        last_rx_cycle[ route_id ] = arrival_cycles;
                        first_tx_cycle[ route_id ] = 0;
                        last_tx_cycle[ route_id ] = 0;
                        last_activity_cycles[ route_id ] = 0;
                        active_process_cycles[ route_id ] = 0;
                        active_tx_cycles[ route_id ] = 0;
                        frame_expected_sequence[ route_id ] = 0;
                        frame_sequence_ok[ route_id ] = true;
                        frame_original_points[ route_id ] = packet_original_points;
                        frame_arrived_points[ route_id ] = packet_arrived_points;
                        frame_eroded_points[ route_id ] = packet_eroded_points;
                        frame_valid_points[ route_id ] = packet_valid_points;

                        uint16_t temporal_skip = packet_temporal_skip;

                        if ( temporal_skip == 0 )
                            temporal_skip = 1;

                        frame_temporal_skip[ route_id ] = temporal_skip;

                        uint16_t ingress_port = port_by_route( route_id );

                        if ( ingress_port < TOTAL_PORTS ) {
                            eth_errors_start[ route_id ] = eth_errors[ ingress_port ];
                            ipv4_errors_start[ route_id ] = ipv4_errors[ ingress_port ];
                            udp_errors_start[ route_id ] = udp_errors[ ingress_port ];
                            nsh_errors_start[ route_id ] = nsh_errors[ ingress_port ];
                        }

                        struct telemetry_csv *new_t = telemetry_slot( route_id, f_id );

                        if ( new_t != NULL ) {
                            memset( new_t, 0, sizeof( *new_t ) );

                            new_t -> frame_id = f_id;
                            new_t -> current_skip = temporal_skip;
                            new_t -> camera_send_timestamp = ( double )cam_tx_cycles / timer_hz;
                            new_t -> recv_start_timestamp = ( double )arrival_cycles / timer_hz;
                            new_t -> original_points = frame_original_points[ route_id ];

                            update_logged_frame( route_id, f_id );
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
                    flush_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );

                uint64_t t_active_process_start = 0;
                bool measure_active_process = packet_metric_route >= 0;

                if ( measure_active_process )
                    t_active_process_start = rte_get_timer_cycles();

                bool frame_complete = false;

                if ( t != NULL ) {
                    last_rx_cycle[ route_id ] = packet_arrival_cycles;

                    if ( route_id == ROUTE_SFF1_ENCODER || route_id == ROUTE_DECODER_SFF3 ) {
                        if ( packet_sequence != frame_expected_sequence[ route_id ] )
                            frame_sequence_ok[ route_id ] = false;

                        frame_expected_sequence[ route_id ]++;
                    }

                    t -> rx_packets++;

                    bool metadata_mismatch = packet_original_points != frame_original_points[ route_id ] || packet_valid_points != frame_valid_points[ route_id ];

                    if ( route_id == ROUTE_DECODER_SFF3 )
                        metadata_mismatch = metadata_mismatch || packet_arrived_points != frame_arrived_points[ route_id ] || packet_eroded_points != frame_eroded_points[ route_id ];

                    if ( unlikely( metadata_mismatch ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    if ( route_id == ROUTE_SFF1_ENCODER || route_id == ROUTE_DECODER_SFF3 ) {
                        t -> rx_points += current_packet_points;
                        
                        uint32_t expected_points = ( route_id == ROUTE_DECODER_SFF3 ) ? frame_valid_points[ route_id ] : frame_original_points[ route_id ];
                        frame_complete = ( expected_points > 0 && t -> rx_points == expected_points ) || ( expected_points == 0 && t -> rx_packets == 1 && t -> rx_points == 0 );
                    }
                    else if ( route_id == ROUTE_ENCODER_DECODER )
                        t -> rx_media_bytes += current_media_bytes;
                }

                if ( route_id == ROUTE_SFF1_ENCODER ) {
                    if ( unlikely( !capture_proxy_state( f_id, nsh ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    size_t strip_len = sizeof( struct net_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct nsh_md2_ctx_hdr );

                    if ( unlikely( !rebuild_packet( m, strip_len, target_tx_port ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }
                }
                else if ( route_id == ROUTE_ENCODER_DECODER ) {
                    if ( unlikely( !advance_proxy_state( f_id, MAIN_SI_SFF1, MAIN_SI_ENCODER ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    revise_network_header( target_tx_port, eth, ipv4, udp );
                }
                else {
                    if ( unlikely( !advance_proxy_state( f_id, MAIN_SI_ENCODER, MAIN_SI_DECODER ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }

                    struct proxy_context *ctx = proxy_context_slot( f_id );

                    if ( unlikely( !enforce_proxy_nsh( m, target_tx_port, ctx ) ) ) {
                        rte_pktmbuf_free( m );
                        nsh_errors[ rx_port ]++;
                        continue;
                    }
                }

                if ( burst_idx[ target_tx_port ] == 0 ) {
                    burst_owner_t[ target_tx_port ] = t;
                    burst_owner_route[ target_tx_port ] = packet_metric_route;
                }

                int queue_idx = burst_idx[ target_tx_port ];

                tx_bufs[ target_tx_port ][ queue_idx ] = m;
                tx_points_buf[ target_tx_port ][ queue_idx ] = ( route_id == ROUTE_SFF1_ENCODER || route_id == ROUTE_DECODER_SFF3 ) ? current_packet_points : 0;
                tx_media_bytes_buf[ target_tx_port ][ queue_idx ] = ( route_id == ROUTE_ENCODER_DECODER ) ? current_media_bytes : 0;

                burst_idx[ target_tx_port ]++;

                if ( measure_active_process ) {
                    uint64_t t_packet_process_end = rte_get_timer_cycles();
                    active_process_cycles[ route_id ] += t_packet_process_end - t_active_process_start;
                    last_activity_cycles[ route_id ] = t_packet_process_end;
                }

                bool force_flush = burst_idx[ target_tx_port ] == BURST_SIZE;

                if ( ( route_id == ROUTE_SFF1_ENCODER || route_id == ROUTE_DECODER_SFF3 ) && frame_complete )
                    force_flush = true;

                if ( is_eos_packet )
                    force_flush = true;

                if ( force_flush )
                    flush_port( target_tx_port, tx_bufs[ target_tx_port ], tx_points_buf[ target_tx_port ], tx_media_bytes_buf[ target_tx_port ], &burst_idx[ target_tx_port ], &burst_owner_t[ target_tx_port ], &burst_owner_route[ target_tx_port ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );

                if ( trigger_eos_write ) {
                    telemetry_to_csv( route_id );
                    csv_written[ route_id ] = true;

                    const char *route = ( route_id == ROUTE_SFF1_ENCODER ) ? "\"SFF1\" -> \"Encoder\"" : ( route_id == ROUTE_ENCODER_DECODER ) ? "\"Encoder\" -> \"Decoder\"" : "\"Decoder\" -> \"SFF3\"";

                    printf( "\n[SYSTEM] \"Main\", Route: %s. End of stream detected. Changing to \"idle\" state...\n", route );
                    
                    if ( route_id != ROUTE_DECODER_SFF3 )
                        printf( "\n" );
                }
            }
        }

        if ( !received_any_packet ) {
            for ( uint16_t tx_port = 0; tx_port < TOTAL_PORTS; tx_port++ ) {
                if ( burst_idx[ tx_port ] == 0 )
                    continue;

                flush_port( tx_port, tx_bufs[ tx_port ], tx_points_buf[ tx_port ], tx_media_bytes_buf[ tx_port ], &burst_idx[ tx_port ], &burst_owner_t[ tx_port ], &burst_owner_route[ tx_port ], first_tx_cycle, last_tx_cycle, active_tx_cycles, active_process_cycles, last_activity_cycles );
            }
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It defines the topology central element. SFF2 operates as both forwarder & proxy, enforcing pair-specific network validation & the "Main", "Temporal" & "Pose" service-path contracts while maintaining per-course diagnosis

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"SFF2\" microservice...\n" );

    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF1, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF1-facing virtual port configuration failed...\n" );

    if ( port_init( PORT_ENCODER, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Encoder-facing virtual port configuration failed...\n" );

    if ( port_init( PORT_DECODER, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Decoder-facing virtual port configuration failed...\n" );

    if ( port_init( PORT_SFF3, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF3-facing virtual port configuration failed...\n" );

    printf( "\n" );

    tables_init();
    header_init();

    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE )
        worker_loop( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    rte_eal_cleanup();

    return 0;
}