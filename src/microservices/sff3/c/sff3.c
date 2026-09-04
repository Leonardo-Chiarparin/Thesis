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
#define TELEMETRY_FOLDER "/shared/log/sff3"
#define TELEMETRY_PATH "/shared/log/sff3/telemetry_sff3.csv"

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
#define MAIN_SI_DECODER 253

#define POSE_SPI 300
#define POSE_SI 255

// Sending bonds & networking parameters
#define PORT_SFF2 0
#define PORT_USER 1

#define SFF2_SFF3_IP RTE_IPV4( 10, 0, 5, 2 )
#define SFF3_SFF2_IP RTE_IPV4( 10, 0, 5, 1 )
#define SFF2_SFF3_PORT 6633
#define SFF3_SFF2_PORT 6633

#define USER_IP RTE_IPV4( 10, 0, 6, 1 )
#define SFF3_USER_IP RTE_IPV4( 10, 0, 6, 254 )
#define USER_PORT 9001
#define SFF3_USER_PORT 6633

// Packetization & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80
#define NETWORK_MTU 1500
#define MBUF_DATA_SIZE ( RTE_PKTMBUF_HEADROOM + NETWORK_MTU + sizeof( struct rte_ether_hdr ) + 64 )


#define MD_TYPE_2 0x02
#define NEXT_PROTOCOL_EXPERIMENT_1 0xFE
#define DEFAULT_TTL 63

// Wire-format structures utilized by the "DPDK" data path
struct nsh_hdr {
    uint16_t base_flags_ttl_len;
    uint8_t md_type;
    uint8_t next_protocol;
    uint32_t serv_path_hdr; 
} __attribute__((__packed__));

struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__, __aligned__(2)));

struct pose_hdr {
    struct net_hdr net;
    struct nsh_hdr nsh;
} __attribute__((__packed__, __aligned__(2)));

#define NSH_TOTAL_SIZE ( sizeof( struct nsh_hdr ) )
#define NSH_LENGTH_WORDS ( NSH_TOTAL_SIZE / 4 )

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

struct point_tx {
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t padding;
} __attribute__((__packed__));

struct pose_payload {
    uint64_t timestamp;
    uint32_t yaw;
    uint32_t pitch;
    uint32_t zoom;
    uint32_t padding;
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
    double cycle_ms;
    double header_wait_ms;
    double total_residency_ms;
    double node_efficiency_pct;
    double cycle_occupancy_pct;

    double camera_node_ms;
    double schedule_delay_ms;
    double inter_arrival_ms;
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
struct net_hdr user_template_hdr;
struct pose_hdr pose_template_hdr;

static const struct rte_ether_addr sff2_sff3_mac = { { 0x00, 0x00, 0x00, 0x00, 0x05, 0x02 } };
static const struct rte_ether_addr sff3_sff2_mac = { { 0x00, 0x00, 0x00, 0x00, 0x05, 0x01 } };

static const struct rte_ether_addr user_mac = { { 0x00, 0x00, 0x00, 0x00, 0x06, 0x01 } };
static const struct rte_ether_addr sff3_user_mac = { { 0x00, 0x00, 0x00, 0x00, 0x06, 0x02 } };

struct telemetry_csv telemetry_log[ K_FRAMES ];
int logged_frames = 0;
bool csv_written = false;

static uint32_t eth_errors = 0;
static uint32_t ipv4_errors = 0;
static uint32_t udp_errors = 0;
static uint32_t nsh_errors = 0;

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

static void main_header_init( struct net_hdr *hdr ) {
    memset( hdr, 0, sizeof( struct net_hdr ) );

    rte_memcpy( &hdr -> ethernet.src_addr, &sff3_user_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> ethernet.dst_addr, &user_mac, RTE_ETHER_ADDR_LEN );

    hdr -> ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> ipv4.version_ihl = 0x45;
    hdr -> ipv4.time_to_live = 64;
    hdr -> ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> ipv4.src_addr = rte_cpu_to_be_32( SFF3_USER_IP );
    hdr -> ipv4.dst_addr = rte_cpu_to_be_32( USER_IP );

    hdr -> udp.src_port = rte_cpu_to_be_16( SFF3_USER_PORT );
    hdr -> udp.dst_port = rte_cpu_to_be_16( USER_PORT );
    hdr -> udp.dgram_cksum = 0;
}

static void pose_header_init( struct pose_hdr *hdr ) {
    memset( hdr, 0, sizeof( struct pose_hdr ) );

    rte_memcpy( &hdr -> net.ethernet.src_addr, &sff3_sff2_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> net.ethernet.dst_addr, &sff2_sff3_mac, RTE_ETHER_ADDR_LEN );

    hdr -> net.ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> net.ipv4.version_ihl = 0x45;
    hdr -> net.ipv4.time_to_live = 64;
    hdr -> net.ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> net.ipv4.src_addr = rte_cpu_to_be_32( SFF3_SFF2_IP );
    hdr -> net.ipv4.dst_addr = rte_cpu_to_be_32( SFF2_SFF3_IP );

    hdr -> net.udp.src_port = rte_cpu_to_be_16( SFF3_SFF2_PORT );
    hdr -> net.udp.dst_port = rte_cpu_to_be_16( SFF2_SFF3_PORT );
    hdr -> net.udp.dgram_cksum = 0;

    hdr -> nsh.base_flags_ttl_len = nsh_base_field( DEFAULT_TTL, NSH_LENGTH_WORDS );
    hdr -> nsh.md_type = MD_TYPE_2;
    hdr -> nsh.next_protocol = NEXT_PROTOCOL_EXPERIMENT_1;
    hdr -> nsh.serv_path_hdr = htonl( ( POSE_SPI << 8 ) | POSE_SI );
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

    fprintf( f, "frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;cycle_occupancy_pct;camera_node_ms;schedule_delay_ms;inter_arrival_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets\n" );

    for ( int i = 0; i < logged_frames; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        fprintf( f, "%u;%u;%u;%u;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u;%u;%u;%u;%u;%u\n", t -> frame_id, t -> rx_complete, t -> tx_complete, t -> current_skip, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> original_points, t -> rx_points, t -> tx_points, t -> rx_packets, t -> tx_packets, t -> payload_bytes, t -> reference_size_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> reference_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> reference_bitrate_mbps, t -> tx_duration_ms, t -> active_tx_ms, t -> active_process_ms, t -> cycle_ms, t -> header_wait_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> cycle_occupancy_pct, t -> camera_node_ms, t -> schedule_delay_ms, t -> inter_arrival_ms, t -> instant_jitter_ms, t -> desynced_jitter_ms, t -> eth_errors, t -> ipv4_errors, t -> udp_errors, t -> nsh_errors, t -> tx_zero_accepts, t -> tx_partial_accepts, t -> tx_resubmit_calls, t -> tx_resubmitted_packets );
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

        uint16_t nb_tx = rte_eth_tx_burst( PORT_USER, 0, &tx_bufs[ sent ], requested_packets );

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

static inline void dispatch_pose_control( struct rte_mbuf *m ) {

    // Purpose: It commences the reverse "SPI 300" path by validating the native 24-byte User ( stance ) command, imposing the base "NSH" encapsulation, & forwarding the payload unchanged to SFF2

    size_t min_packet_len = sizeof( struct net_hdr ) + sizeof( struct pose_payload );

    if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < min_packet_len ) ) {
        ipv4_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

    if ( unlikely( !rte_is_same_ether_addr( &eth -> src_addr, &user_mac ) || !rte_is_same_ether_addr( &eth -> dst_addr, &sff3_user_mac ) ) ) {
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

    if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( USER_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( SFF3_USER_IP ) ) ) {
        ipv4_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

    if ( unlikely( udp -> src_port != rte_cpu_to_be_16( USER_PORT ) || udp -> dst_port != rte_cpu_to_be_16( SFF3_USER_PORT ) ) ) {
        udp_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
    uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );

    if ( unlikely( udp_length != sizeof( struct rte_udp_hdr ) + sizeof( struct pose_payload ) || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length ) ) {
        udp_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( rte_pktmbuf_pkt_len( m ) != sizeof( struct rte_ether_hdr ) + ipv4_length ) ) {
        ipv4_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( rte_pktmbuf_adj( m, sizeof( struct net_hdr ) ) == NULL ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    if ( unlikely( rte_pktmbuf_pkt_len( m ) != sizeof( struct pose_payload ) ) ) {
        udp_errors++;
        rte_pktmbuf_free( m );
        return;
    }

    char *new_hdr_start = rte_pktmbuf_prepend( m, sizeof( struct pose_hdr ) );

    if ( unlikely( new_hdr_start == NULL ) ) {
        rte_pktmbuf_free( m );
        return;
    }

    struct pose_hdr *out_hdr = ( struct pose_hdr * )new_hdr_start;

    rte_memcpy( out_hdr, &pose_template_hdr, sizeof( struct pose_hdr ) );

    uint16_t out_udp_len = sizeof( struct rte_udp_hdr ) + NSH_TOTAL_SIZE + sizeof( struct pose_payload );

    out_hdr -> net.udp.dgram_len = rte_cpu_to_be_16( out_udp_len );
    out_hdr -> net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + out_udp_len );
    out_hdr -> net.ipv4.hdr_checksum = 0;
    out_hdr -> net.ipv4.hdr_checksum = rte_ipv4_cksum( &out_hdr -> net.ipv4 );

    uint16_t retries = 0;

    while ( 1 ) {
        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF2, 0, &m, 1 );

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
    uint64_t timer_hz = rte_get_timer_hz();

    struct rte_mbuf *bufs[ BURST_SIZE ];
    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    struct rte_mbuf *pose_bufs[ BURST_SIZE ];

    uint32_t tx_points_buf[ BURST_SIZE ];

    uint32_t current_frame_id = END_OF_STREAM;
    uint32_t frame_rx_points = 0;
    uint32_t frame_original_points = 0;
    uint32_t frame_arrived_points = 0;
    uint32_t frame_eroded_points = 0;
    uint32_t frame_valid_points = 0;
    uint32_t frame_expected_sequence = 0;
    bool frame_sequence_ok = true;

    uint64_t t_frame_arrival = 0;
    uint64_t last_rx_cycles = 0;
    uint64_t t_cycle_start = rte_get_timer_cycles();

    uint64_t active_process_cycles = 0;
    uint64_t active_tx_cycles = 0;

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
    double current_frame_latency_ms = 0.0;
    double current_jitter_ms = 0.0;
    double jitter_ms = 0.0;
    double current_inter_arrival_ms = 0.0;

    uint64_t prev_arrival_cycles = 0;
    uint32_t prev_arrival_frame = 0;
    uint32_t first_arrival_frame = 0;

    uint16_t frame_temporal_skip = 1;
    uint16_t frame_points_per_packet = POINTS_PER_PACKET;

    printf( "[SYSTEM] Listening on every service-chain link...\n\n" );

    while ( 1 ) {
        if ( csv_written ) {
            rte_delay_us_sleep( 1000 );
            continue;
        }

        uint16_t nb_pose = rte_eth_rx_burst( PORT_USER, 0, pose_bufs, BURST_SIZE );

        for ( uint16_t i = 0; i < nb_pose; i++ )
            dispatch_pose_control( pose_bufs[ i ] );

        uint16_t nb_rx = rte_eth_rx_burst( PORT_SFF2, 0, bufs, BURST_SIZE );

        if ( unlikely( nb_rx == 0 ) )
            continue;

        for ( int i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];

            size_t min_req = sizeof( struct net_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct dec_hdr );

            if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < min_req ) ) {
                frame_ipv4_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_ether_hdr *old_eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

            if ( unlikely( !rte_is_same_ether_addr( &old_eth -> src_addr, &sff2_sff3_mac ) || !rte_is_same_ether_addr( &old_eth -> dst_addr, &sff3_sff2_mac ) ) ) {
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

            if ( unlikely( old_ipv4 -> version_ihl != 0x45 || old_ipv4 -> next_proto_id != IPPROTO_UDP || old_ipv4 -> src_addr != rte_cpu_to_be_32( SFF2_SFF3_IP ) || old_ipv4 -> dst_addr != rte_cpu_to_be_32( SFF3_SFF2_IP ) ) ) {
                frame_ipv4_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct rte_udp_hdr *old_udp = ( struct rte_udp_hdr * )( old_ipv4 + 1 );

            if ( unlikely( old_udp -> src_port != rte_cpu_to_be_16( SFF2_SFF3_PORT ) || old_udp -> dst_port != rte_cpu_to_be_16( SFF3_SFF2_PORT ) ) ) {
                frame_udp_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t old_udp_length = rte_be_to_cpu_16( old_udp -> dgram_len );
            uint16_t old_ipv4_length = rte_be_to_cpu_16( old_ipv4 -> total_length );

            if ( unlikely( old_udp_length < sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct dec_hdr ) || old_ipv4_length != sizeof( struct rte_ipv4_hdr ) + old_udp_length ) ) {
                frame_udp_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( rte_pktmbuf_pkt_len( m ) < sizeof( struct rte_ether_hdr ) + old_ipv4_length ) ) {
                frame_ipv4_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct nsh_hdr *nsh = ( struct nsh_hdr * )( old_udp + 1 );

            uint16_t base = rte_be_to_cpu_16( nsh -> base_flags_ttl_len );
            uint8_t version = ( base >> 14 ) & 0x03;
            bool oam = ( base & 0x2000 ) != 0;
            uint8_t ttl = ( base >> 6 ) & 0x3F;
            uint8_t length_words = base & 0x3F;

            if ( unlikely( version != 0 || oam || ttl == 0 || length_words == 0 ) ) {
                frame_nsh_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            uint16_t nsh_length = nsh_length_bytes( nsh );

            if ( unlikely( nsh_length != sizeof( struct nsh_hdr ) || nsh -> md_type != MD_TYPE_2 || nsh -> next_protocol != NEXT_PROTOCOL_EXPERIMENT_1 ) ) {
                frame_nsh_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            uint32_t sph = rte_be_to_cpu_32( nsh -> serv_path_hdr );
            uint32_t spi = sph >> 8;
            uint8_t si = sph & 0xFF;

            if ( unlikely( spi != MAIN_SPI || si != MAIN_SI_DECODER ) ) {
                frame_nsh_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            if ( unlikely( old_udp_length < sizeof( struct rte_udp_hdr ) + nsh_length + sizeof( struct dec_hdr ) ) ) {
                frame_nsh_errors++;
                rte_pktmbuf_free( m );
                continue;
            }

            struct dec_hdr *old_dec = ( struct dec_hdr * )( ( uint8_t * )nsh + nsh_length );
            uint32_t f_id = rte_be_to_cpu_32( old_dec -> frame_id );
            uint32_t packet_sequence = rte_be_to_cpu_32( old_dec -> sequence_number );
            uint32_t packet_points = rte_be_to_cpu_32( old_dec -> points_in_packet );
            uint16_t packet_points_per_packet = rte_be_to_cpu_16( old_dec -> padding );

            if ( packet_points_per_packet == 0 )
                packet_points_per_packet = POINTS_PER_PACKET;
            uint32_t packet_original_points = rte_be_to_cpu_32( old_dec -> original_points );
            uint32_t packet_arrived_points = rte_be_to_cpu_32( old_dec -> arrived_points );
            uint32_t packet_eroded_points = rte_be_to_cpu_32( old_dec -> eroded_points );
            uint32_t packet_valid_points = rte_be_to_cpu_32( old_dec -> valid_points );
            uint32_t packet_payload_len = old_udp_length - sizeof( struct rte_udp_hdr ) - nsh_length - sizeof( struct dec_hdr );
            
            if ( f_id != END_OF_STREAM ) {
                bool counters_valid = packet_valid_points <= packet_eroded_points && packet_eroded_points <= packet_arrived_points;
                bool empty_frame = packet_valid_points == 0 && packet_points == 0 && packet_sequence == 0 && packet_payload_len == 0;
                bool populated_frame = packet_valid_points > 0 && packet_points > 0 && packet_points <= packet_points_per_packet && packet_payload_len == packet_points * sizeof( struct point_tx );

                if ( unlikely( !counters_valid || ( !empty_frame && !populated_frame ) ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }
            }
            else if ( unlikely( packet_original_points != 0 || packet_arrived_points != 0 || packet_eroded_points != 0 || packet_valid_points != 0 || packet_points != 0 || packet_payload_len != 0 ) ) {
                rte_pktmbuf_free( m );
                continue;
            }

            if ( f_id != END_OF_STREAM && f_id == current_frame_id && packet_points_per_packet != frame_points_per_packet ) {
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

                    uint64_t received_payload_bytes = ( uint64_t )frame_rx_points * sizeof( struct point_tx );
                    
                    uint64_t logical_rx_frame_bytes = received_payload_bytes + ( frame_rx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );

                    uint64_t transmitted_payload_bytes = ( uint64_t )frame_tx_points * sizeof( struct point_tx );

                    uint64_t logical_frame_bytes = transmitted_payload_bytes + ( frame_tx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );
                    uint64_t network_frame_bytes = transmitted_payload_bytes + ( ( uint64_t )frame_tx_packets * ( sizeof( struct net_hdr ) + sizeof( struct dec_hdr ) ) );

                    uint64_t reference_frame_bytes = ( ( uint64_t )frame_valid_points * sizeof( struct point_tx ) ) + ( frame_rx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );
                    uint32_t expected_packets = ( frame_valid_points > 0 ) ? ( frame_valid_points + frame_points_per_packet - 1 ) / frame_points_per_packet : 1;

                    double effective_fps = TARGET_FPS / frame_temporal_skip;

                    bool rx_complete = frame_rx_packets == expected_packets && frame_sequence_ok && ( ( frame_valid_points > 0 && frame_rx_points == frame_valid_points ) || ( frame_valid_points == 0 && frame_rx_points == 0 ) );
                    bool tx_complete = rx_complete && frame_tx_points == frame_rx_points && frame_tx_packets == frame_rx_packets;

                    telemetry_log[ logged_frames ].frame_id = current_frame_id;
                    telemetry_log[ logged_frames ].rx_complete = rx_complete ? 1 : 0;
                    telemetry_log[ logged_frames ].tx_complete = tx_complete ? 1 : 0;
                    telemetry_log[ logged_frames ].current_skip = frame_temporal_skip;
                    telemetry_log[ logged_frames ].node_exit_timestamp = ( double )t_frame_end / timer_hz;

                    telemetry_log[ logged_frames ].tx_duration_ms = duration_sec * 1000.0;
                    telemetry_log[ logged_frames ].payload_bytes = ( uint32_t )received_payload_bytes;

                    telemetry_log[ logged_frames ].original_points = frame_original_points;
                    telemetry_log[ logged_frames ].rx_packets = frame_rx_packets;
                    telemetry_log[ logged_frames ].rx_points = frame_rx_points;
                    telemetry_log[ logged_frames ].tx_packets = frame_tx_packets;
                    telemetry_log[ logged_frames ].tx_points = frame_tx_points;

                    if ( frame_valid_points > 0 )
                        telemetry_log[ logged_frames ].data_integrity_pct = ( ( double )frame_rx_points / ( double )frame_valid_points ) * 100.0;
                    else
                        telemetry_log[ logged_frames ].data_integrity_pct = rx_complete ? 100.0 : 0.0;

                    telemetry_log[ logged_frames ].tx_zero_accepts = frame_zero_accepts;
                    telemetry_log[ logged_frames ].tx_partial_accepts = frame_partial_accepts;
                    telemetry_log[ logged_frames ].tx_resubmit_calls = frame_resubmit_calls;
                    telemetry_log[ logged_frames ].tx_resubmitted_packets = frame_resubmitted_packets;

                    telemetry_log[ logged_frames ].active_tx_ms = active_tx_sec * 1000.0;
                    telemetry_log[ logged_frames ].active_process_ms = active_process_sec * 1000.0;
                    telemetry_log[ logged_frames ].total_residency_ms = residency_sec * 1000.0;
                    telemetry_log[ logged_frames ].cycle_ms = cycle_sec * 1000.0;
                    telemetry_log[ logged_frames ].header_wait_ms = header_wait_sec * 1000.0;

                    telemetry_log[ logged_frames ].camera_node_ms = current_frame_latency_ms;

                    uint32_t schedule_frame_offset = current_frame_id - first_arrival_frame;
                    double real_exit_sec = ( t_frame_end >= t_session_start ) ? ( double )( t_frame_end - t_session_start ) / timer_hz : 0.0;
                    double ideal_start_sec = ( double )schedule_frame_offset / TARGET_FPS;

                    telemetry_log[ logged_frames ].schedule_delay_ms = ( real_exit_sec - ideal_start_sec ) * 1000.0;
                    telemetry_log[ logged_frames ].inter_arrival_ms = current_inter_arrival_ms;
                    telemetry_log[ logged_frames ].instant_jitter_ms = current_jitter_ms;
                    telemetry_log[ logged_frames ].desynced_jitter_ms = jitter_ms;

                    telemetry_log[ logged_frames ].internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_rx_frame_bytes / 1000000.0 ) / receive_sec : 0.0;
                    telemetry_log[ logged_frames ].logical_bitrate_mbps = ( logical_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
                    telemetry_log[ logged_frames ].network_bitrate_mbps = ( network_frame_bytes * 8.0 * effective_fps ) / 1000000.0;

                    telemetry_log[ logged_frames ].reference_size_bytes = ( uint32_t )reference_frame_bytes;
                    telemetry_log[ logged_frames ].reference_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )reference_frame_bytes / 1000000.0 ) / receive_sec : 0.0;
                    telemetry_log[ logged_frames ].reference_bitrate_mbps = ( reference_frame_bytes * 8.0 * effective_fps ) / 1000000.0;
                    telemetry_log[ logged_frames ].cycle_occupancy_pct = ( cycle_sec > 0.0 ) ? ( residency_sec / cycle_sec ) * 100.0 : 0.0;

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

                uint64_t cam_tx_cycles = rte_be_to_cpu_64( old_dec -> timestamp );
                uint64_t arrival_cycles = packet_arrival_cycles;

                if ( arrival_cycles >= cam_tx_cycles )
                    current_frame_latency_ms = ( ( double )( arrival_cycles - cam_tx_cycles ) / timer_hz ) * 1000.0;
                else
                    current_frame_latency_ms = 0.0;

                if ( f_id != END_OF_STREAM ) {
                    uint16_t incoming_skip = rte_be_to_cpu_16( old_dec -> temporal_skip );

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

                        current_inter_arrival_ms = real_interval_sec * 1000.0;

                        current_jitter_ms = ( diff_sec < 0.0 ) ? -diff_sec * 1000.0 : diff_sec * 1000.0;
                    }
                    else {
                        current_inter_arrival_ms = 0.0;
                        current_jitter_ms = 0.0;
                    }

                    jitter_ms += ( current_jitter_ms - jitter_ms ) / 16.0;

                    prev_arrival_cycles = arrival_cycles;
                    prev_arrival_frame = f_id;
                    frames_received++;
                }

                current_frame_id = f_id;

                if ( f_id != END_OF_STREAM )
                    frame_points_per_packet = packet_points_per_packet;

                t_frame_arrival = arrival_cycles;
                last_rx_cycles = arrival_cycles;

                if ( logged_frames < K_FRAMES ) {
                    telemetry_log[ logged_frames ].camera_send_timestamp = ( double )cam_tx_cycles / timer_hz;
                    telemetry_log[ logged_frames ].recv_start_timestamp = ( double )arrival_cycles / timer_hz;
                }

                frame_rx_points = 0;
                frame_original_points = 0;
                frame_arrived_points = 0;
                frame_eroded_points = 0;
                frame_valid_points = 0;
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
                frame_tx_points = 0;
                frame_completion_cycles = 0;
                last_activity_cycles = 0;
                first_tx_cycles = 0;
                last_tx_cycles = 0;
            }

            if ( unlikely( f_id == END_OF_STREAM ) ) {
                size_t strip_len = sizeof( struct net_hdr ) + nsh_length;

                if ( unlikely( rte_pktmbuf_adj( m, strip_len ) == NULL ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                char *new_hdr_start = rte_pktmbuf_prepend( m, sizeof( struct net_hdr ) );

                if ( likely( new_hdr_start != NULL ) ) {
                    struct net_hdr *hdr = ( struct net_hdr * )new_hdr_start;

                    rte_memcpy( hdr, &user_template_hdr, sizeof( struct net_hdr ) );

                    uint16_t outer_udp_len = sizeof( struct rte_udp_hdr ) + sizeof( struct dec_hdr );

                    hdr -> udp.dgram_len = rte_cpu_to_be_16( outer_udp_len );
                    hdr -> ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + outer_udp_len );
                    hdr -> ipv4.hdr_checksum = 0;
                    hdr -> ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> ipv4 );

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

            if ( frame_rx_packets == 1 ) {
                frame_original_points = packet_original_points;
                frame_arrived_points = packet_arrived_points;
                frame_eroded_points = packet_eroded_points;
                frame_valid_points = packet_valid_points;
            }
            else if ( unlikely( packet_original_points != frame_original_points || packet_arrived_points != frame_arrived_points || packet_eroded_points != frame_eroded_points || packet_valid_points != frame_valid_points ) ) 
                frame_sequence_ok = false;

            bool frame_complete = ( frame_valid_points > 0 && frame_rx_points == frame_valid_points ) || ( frame_valid_points == 0 && frame_rx_packets == 1 && frame_rx_points == 0 );

            size_t strip_len = sizeof( struct net_hdr ) + nsh_length;

            if ( unlikely( rte_pktmbuf_adj( m, strip_len ) == NULL ) ) {
                rte_pktmbuf_free( m );

                uint64_t t_active_process_end = rte_get_timer_cycles();
                active_process_cycles += t_active_process_end - t_active_process_start;
                last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = last_activity_cycles;

                continue;
            }

            char *new_hdr_start = rte_pktmbuf_prepend( m, sizeof( struct net_hdr ) );

            if ( unlikely( new_hdr_start == NULL ) ) {
                rte_pktmbuf_free( m );

                uint64_t t_active_process_end = rte_get_timer_cycles();
                active_process_cycles += t_active_process_end - t_active_process_start;
                last_activity_cycles = t_active_process_end;

                if ( frame_complete )
                    frame_completion_cycles = last_activity_cycles;

                continue;
            }

            struct net_hdr *hdr = ( struct net_hdr * )new_hdr_start;

            rte_memcpy( hdr, &user_template_hdr, sizeof( struct net_hdr ) );

            uint16_t outer_udp_len = sizeof( struct rte_udp_hdr ) + sizeof( struct dec_hdr ) + packet_payload_len;

            hdr -> udp.dgram_len = rte_cpu_to_be_16( outer_udp_len );
            hdr -> ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + outer_udp_len );
            hdr -> ipv4.hdr_checksum = 0;
            hdr -> ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> ipv4 );

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

    // Purpose: It describes the terminal node for the "Main" service-chain route, validating & removing SFF2-provided "NSH" prior to the delivery of native point datagrams to User.
    //          Furthermore, on the backward "Pose" sequence, it elaborates plain guidelines, applies the dedicated "SPI 300" / "SI 255" context, & forwards them to SFF2 for Decoder consumption

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"SFF3\" microservice...\n" );

    uint32_t point_ipv4_len = sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct nsh_hdr ) + sizeof( struct dec_hdr ) + POINTS_PER_PACKET * sizeof( struct point_tx );

    if ( point_ipv4_len > NETWORK_MTU )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Packetization exceeded \"MTU\" size ( %u > %u )...\n", point_ipv4_len, ( unsigned int )NETWORK_MTU );

    struct rte_mempool *mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, MBUF_DATA_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF2, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF2-facing virtual port configuration failed...\n" );

    if ( port_init( PORT_USER, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: User-facing virtual port configuration failed...\n" );

    printf( "\n" );

    main_header_init( &user_template_hdr );
    pose_header_init( &pose_template_hdr );

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