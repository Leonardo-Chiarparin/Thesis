#include <errno.h>
#include <fcntl.h>
#include <math.h>
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
#include <rte_cycles.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_mbuf.h>
#include <rte_udp.h>

// Runtime & experimental configuration variables
#define TELEMETRY_FOLDER "/shared/log/user"
#define TELEMETRY_PATH "/shared/log/user/telemetry_user.csv"

#define QUALITY_FOLDER "/shared/data/loot/made"
#define QUALITY_PATH "/shared/data/loot/made/results.bin"
#define QUALITY_DONE_PATH "/tmp/sfc-user-done"
#define QUALITY_READY_PATH "/tmp/sfc-user-quality"

#define EOS_PATH "/tmp/sfc-user-eos"
#define FRAME_PATH "/dev/shm/frame.bin"
#define CTRL_PATH "/dev/shm/ctrl.bin"

#define READY_PATH "/tmp/sfc-user-ready"

#define K_FRAMES 300
#define TARGET_FPS 30.0

#define BURST_SIZE 32
#define QUALITY_BUFFER_SIZE ( 1024 * 1024 * 1024 )
#define MAX_ZERO_ACCEPTS 2048

#define END_OF_STREAM 0xFFFFFFFF

// "DPDK" packet-buffer pool settings
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// Sending bonds & networking parameters
#define PORT_SFF3 0

#define USER_IP RTE_IPV4( 10, 0, 6, 1 )
#define SFF3_USER_IP RTE_IPV4( 10, 0, 6, 254 )
#define USER_PORT 9001
#define SFF3_USER_PORT 6633

// Packetisation & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80
#define WIDTH 640
#define HEIGHT 480
#define MAX_FRAME_POINTS ( 6 * WIDTH * HEIGHT )
#define MAX_FRAME_PACKETS ( ( MAX_FRAME_POINTS + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET )

#define MAX_COMMANDS 256
#define CMD_TYPE_POSE 1
#define RETRY_FRAMES 3
#define COMMAND_TIME 5000

// Wire-format structures utilized by the "DPDK" data path
struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__, __aligned__(2)));

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

struct host_point {
    float x;
    float y;
    float z;
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

struct quality_hdr {
    uint32_t frame_id;
    uint32_t point_count;
} __attribute__((__packed__));

// Frame description consumed by the asynchronous bridge
struct web_hdr {
    uint64_t seq;
    uint32_t frame_id;
    uint32_t point_count;
    uint32_t original_points;
    uint32_t arrived_points;
    uint32_t eroded_points;
    uint32_t valid_points;
    uint32_t cmd_id;
    uint16_t temporal_skip;
    uint16_t padding;
    float yaw;
    float pitch;
    float zoom;
    uint32_t align_pad;
    double e2e_ms;
    double cmd_apply_ms;
};

// Control organization subsequently read by the involved "worker"
struct web_ctrl {
    uint64_t cmd_seq;
    uint32_t cmd_id;
    uint32_t cmd_type;
    float yaw;
    float pitch;
    float zoom;
    uint32_t padding;
    uint64_t ack_seq;
    uint32_t ack_frame;
    uint32_t ack_cmd;
    double ack_ctp_ms;
};

struct frame_state {
    bool active;
    bool sequence_ok;

    uint32_t frame_id;
    uint32_t expected_seq;
    uint32_t rx_points;
    uint32_t rx_packets;
    uint32_t original_points;
    uint32_t arrived_points;
    uint32_t eroded_points;
    uint32_t valid_points;
    uint16_t temporal_skip;

    float yaw;
    float pitch;
    float zoom;

    uint64_t camera_tx;
    uint64_t first_arrival;
    uint64_t last_arrival;
    uint64_t active_cycles;
};

struct cmd_record {
    bool used;
    bool matched;
    bool acked;

    uint32_t cmd_id;
    uint32_t frame_id;
    uint32_t last_retry_frame;
    uint64_t sent_cycles;

    float yaw;
    float pitch;
    float zoom;
    double reference_ms;
    double apply_ms;
    double photon_ms;

    bool reference_seen;
};

// Per-shot telemetry intentionally contains end-device metrics only
struct telemetry_csv {
    uint32_t frame_id;
    uint8_t rx_complete;
    uint16_t current_skip;

    float yaw;
    float pitch;
    float zoom;

    double camera_send_timestamp;
    double recv_start_timestamp;
    double node_exit_timestamp;

    uint32_t original_points;
    uint32_t arrived_points;
    uint32_t eroded_points;
    uint32_t valid_points;
    uint32_t rx_points;
    uint32_t rx_packets;
    uint32_t payload_bytes;

    double data_integrity_pct;
    double internal_throughput_mbs;
    double logical_bitrate_mbps;
    double network_bitrate_mbps;

    double arrival_pct;
    double erosion_pct;
    double valid_pct;

    double web_publish_ms;
    double web_ack_ms;
    double active_process_ms;
    double total_residency_ms;
    double node_efficiency_pct;

    double camera_node_ms;
    double e2e_latency_ms;
    double reference_e2e_ms;
    double schedule_delay_ms;
    double instant_jitter_ms;
    double desynced_jitter_ms;

    uint32_t cmd_id;
    double reference_cmd_ms;
    double cmd_apply_ms;
    double cmd_photon_ms;

    double quality_save_ms;
    double mean_error;
    double geom_rmse;
    double chamfer;
    double hausdorff;
    double mean_mm;
    double rmse_mm;
    double chamfer_mm;
    double hausdorff_mm;
};

// Global application state
static struct rte_mempool *mbuf_pool;

static const struct rte_ether_addr user_mac = { { 0x00, 0x00, 0x00, 0x00, 0x06, 0x01 } };
static const struct rte_ether_addr sff3_user_mac = { { 0x00, 0x00, 0x00, 0x00, 0x06, 0x02 } };

static struct telemetry_csv telemetry_log[ K_FRAMES ];
static struct host_point *web_points;
static uint8_t *packet_seen;

static struct frame_state frame_state;
static struct cmd_record cmd_records[ MAX_COMMANDS ];

static uint32_t active_cmd_id = 0;
static uint32_t applied_cmd_id = 0;
static bool eos_received = false;
static uint32_t last_frame_id = 0;

static uint64_t previous_arrival = 0;
static uint32_t previous_frame = 0;
static uint64_t session_arrival = 0;
static uint32_t session_frame = 0;
static double jitter_ms = 0.0;

static uint64_t web_sent_cycles[ K_FRAMES ];

static void *web_frame_map;
static size_t web_frame_size;
static struct web_ctrl *web_ctrl_map;
static uint64_t web_seq = 0;
static uint64_t last_cmd_seq = 0;
static uint64_t last_ack_seq = 0;

static struct net_hdr pose_template_hdr;

static uint8_t *quality_buffer = NULL;
static size_t quality_size = 0;
static uint32_t quality_drops = 0;
static bool quality_failed = false;
static bool quality_capture_enabled = false;

static bool csv_written = false;

// Data-path & support routines
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

static inline bool pose_matches( float yaw_a, float pitch_a, float zoom_a, float yaw_b, float pitch_b, float zoom_b ) {

    // Purpose: It correlates applied topological variables while tolerating fractional deviations originating from serializations

    const float epsilon = 1e-4f;

    return fabsf( yaw_a - yaw_b ) <= epsilon && fabsf( pitch_a - pitch_b ) <= epsilon && fabsf( zoom_a - zoom_b ) <= epsilon;
}

static inline struct cmd_record *find_cmd_record( uint32_t cmd_id ) {

    // Purpose: It reliably identifies the internal tracking entity associated with a globally synchronized command issuance

    if ( cmd_id == 0 )
        return NULL;

    struct cmd_record *record = &cmd_records[ cmd_id % MAX_COMMANDS ];

    if ( !record -> used || record -> cmd_id != cmd_id )
        return NULL;

    return record;
}

static inline struct cmd_record *store_cmd_record( uint32_t cmd_id ) {

    // Purpose: It prepares a fresh directive allocation bucket within the local memory dictionary for real-time tracking

    struct cmd_record *record = &cmd_records[ cmd_id % MAX_COMMANDS ];

    memset( record, 0, sizeof( *record ) );
    record -> used = true;
    record -> cmd_id = cmd_id;

    return record;
}

static inline int port_init( uint16_t port, struct rte_mempool *pool ) {
    struct rte_eth_conf port_conf = { 0 };
    int retval;

    if ( !rte_eth_dev_is_valid_port( port ) )
        return -1;

    retval = rte_eth_dev_configure( port, 1, 1, &port_conf );

    if ( retval != 0 )
        return retval;

    retval = rte_eth_rx_queue_setup( port, 0, 4096, rte_eth_dev_socket_id( port ), NULL, pool );

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

static void pose_header_init( struct net_hdr *hdr ) {
    memset( hdr, 0, sizeof( *hdr ) );

    rte_memcpy( &hdr -> ethernet.src_addr, &user_mac, RTE_ETHER_ADDR_LEN );
    rte_memcpy( &hdr -> ethernet.dst_addr, &sff3_user_mac, RTE_ETHER_ADDR_LEN );

    hdr -> ethernet.ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    hdr -> ipv4.version_ihl = 0x45;
    hdr -> ipv4.time_to_live = 64;
    hdr -> ipv4.next_proto_id = IPPROTO_UDP;
    hdr -> ipv4.src_addr = rte_cpu_to_be_32( USER_IP );
    hdr -> ipv4.dst_addr = rte_cpu_to_be_32( SFF3_USER_IP );

    hdr -> udp.src_port = rte_cpu_to_be_16( USER_PORT );
    hdr -> udp.dst_port = rte_cpu_to_be_16( SFF3_USER_PORT );
    hdr -> udp.dgram_cksum = 0;
}

static int shared_map_init() {

    // Purpose: It maps the inter-process shared memory regions utilized by the asynchronous web bridge to exchange frame & control structures

    web_frame_size = sizeof( struct web_hdr ) + ( ( size_t )MAX_FRAME_POINTS * sizeof( struct host_point ) );

    int frame_fd = open( FRAME_PATH, O_CREAT | O_RDWR | O_TRUNC, 0666 );

    if ( frame_fd < 0 )
        return -1;

    if ( ftruncate( frame_fd, ( off_t )web_frame_size ) != 0 ) {
        close( frame_fd );
        return -1;
    }

    web_frame_map = mmap( NULL, web_frame_size, PROT_READ | PROT_WRITE, MAP_SHARED, frame_fd, 0 );
    close( frame_fd );

    if ( web_frame_map == MAP_FAILED ) {
        web_frame_map = NULL;
        return -1;
    }

    web_points = ( struct host_point * )( ( struct web_hdr * )web_frame_map + 1 );

    int ctrl_fd = open( CTRL_PATH, O_CREAT | O_RDWR | O_TRUNC, 0666 );

    if ( ctrl_fd < 0 )
        return -1;

    if ( ftruncate( ctrl_fd, sizeof( struct web_ctrl ) ) != 0 ) {
        close( ctrl_fd );
        return -1;
    }

    void *ctrl_map = mmap( NULL, sizeof( struct web_ctrl ), PROT_READ | PROT_WRITE, MAP_SHARED, ctrl_fd, 0 );
    close( ctrl_fd );

    if ( ctrl_map == MAP_FAILED )
        return -1;

    web_ctrl_map = ( struct web_ctrl * )ctrl_map;

    memset( web_frame_map, 0, web_frame_size );
    memset( web_ctrl_map, 0, sizeof( *web_ctrl_map ) );

    return 0;
}

static void shared_map_free() {

    // Purpose: It detaches the mapped structures & cleans up the volatile file nodes upon terminal shutdown

    if ( web_frame_map != NULL )
        munmap( web_frame_map, web_frame_size );

    web_points = NULL;

    if ( web_ctrl_map != NULL )
        munmap( web_ctrl_map, sizeof( *web_ctrl_map ) );

    unlink( FRAME_PATH );
    unlink( CTRL_PATH );
}

static inline bool dispatch_pose_control( uint32_t cmd_id, float yaw, float pitch, float zoom, uint64_t command_timestamp, uint64_t *sent_cycles ) {

    // Purpose: It encapsulates & transmits a stace command over the SFF3-facing link, triggering upstream spatial transformations

    if ( cmd_id == 0 || !isfinite( yaw ) || !isfinite( pitch ) || !isfinite( zoom ) || zoom <= 0.0f )
        return false;

    struct rte_mbuf *m = rte_pktmbuf_alloc( mbuf_pool );

    if ( m == NULL )
        return false;

    uint16_t payload_len = sizeof( struct pose_payload );
    uint16_t packet_len = sizeof( struct net_hdr ) + payload_len;
    uint8_t *data = ( uint8_t * )rte_pktmbuf_append( m, packet_len );

    if ( data == NULL ) {
        rte_pktmbuf_free( m );
        return false;
    }

    struct net_hdr *hdr = ( struct net_hdr * )data;
    struct pose_payload *pose = ( struct pose_payload * )( hdr + 1 );

    rte_memcpy( hdr, &pose_template_hdr, sizeof( *hdr ) );

    hdr -> udp.dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + payload_len );
    hdr -> ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + payload_len );
    hdr -> ipv4.hdr_checksum = 0;
    hdr -> ipv4.hdr_checksum = rte_ipv4_cksum( &hdr -> ipv4 );

    uint64_t timestamp = ( command_timestamp != 0 ) ? command_timestamp : rte_get_timer_cycles();

    pose -> timestamp = rte_cpu_to_be_64( timestamp );
    pose -> yaw = float_to_be( yaw );
    pose -> pitch = float_to_be( pitch );
    pose -> zoom = float_to_be( zoom );
    pose -> padding = 0;

    uint16_t retries = 0;

    while ( 1 ) {
        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF3, 0, &m, 1 );

        if ( nb_tx == 1 ) {
            if ( sent_cycles != NULL )
                *sent_cycles = timestamp;

            return true;
        }

        if ( ++retries > MAX_ZERO_ACCEPTS ) {
            rte_pktmbuf_free( m );
            return false;
        }

        rte_pause();
    }
}

static inline void process_web_ctrl() {

    // Purpose: It periodically polls the shared memory control map for new browser-issued pose commands & translates them into "DPDK" network transmissions

    if ( web_ctrl_map == NULL || eos_received )
        return;

    uint64_t cmd_seq = __atomic_load_n( &web_ctrl_map -> cmd_seq, __ATOMIC_ACQUIRE );

    if ( cmd_seq == 0 || cmd_seq == last_cmd_seq )
        return;

    uint32_t cmd_id = web_ctrl_map -> cmd_id;
    uint32_t cmd_type = web_ctrl_map -> cmd_type;
    float yaw = web_ctrl_map -> yaw;
    float pitch = web_ctrl_map -> pitch;
    float zoom = web_ctrl_map -> zoom;

    last_cmd_seq = cmd_seq;

    if ( cmd_type != CMD_TYPE_POSE )
        return;

    uint64_t sent_cycles = 0;

    if ( !dispatch_pose_control( cmd_id, yaw, pitch, zoom, 0, &sent_cycles ) ) {
        printf( "[SYSTEM] Error: Guideline %u could not be forwarded...\n", cmd_id );
        return;
    }

    struct cmd_record *record = store_cmd_record( cmd_id );
    record -> sent_cycles = sent_cycles;
    record -> last_retry_frame = last_frame_id;
    record -> yaw = yaw;
    record -> pitch = pitch;
    record -> zoom = zoom;

    applied_cmd_id = 0;
    active_cmd_id = cmd_id;
}

static inline void retry_active_pose( uint32_t frame_id ) {

    // Purpose: It re-presents an unresolved stance request every three completed elements until the requested pose becomes observable on the returning stream

    if ( eos_received || active_cmd_id == 0 )
        return;

    struct cmd_record *record = find_cmd_record( active_cmd_id );

    if ( record == NULL || record -> matched )
        return;

    if ( frame_id < record -> last_retry_frame + RETRY_FRAMES )
        return;

    if ( dispatch_pose_control( record -> cmd_id, record -> yaw, record -> pitch, record -> zoom, record -> sent_cycles, NULL ) )
        record -> last_retry_frame = frame_id;
}

static inline void process_web_ack( uint64_t timer_hz ) {

    // Purpose: It evaluates asynchronous "Command-to-Photon" acknowledgments provided by the browser renderer, mapping them to the relative internal telemetry records

    if ( web_ctrl_map == NULL )
        return;

    uint64_t ack_seq = __atomic_load_n( &web_ctrl_map -> ack_seq, __ATOMIC_ACQUIRE );

    if ( ack_seq == 0 || ack_seq == last_ack_seq )
        return;

    uint32_t ack_frame = web_ctrl_map -> ack_frame;
    uint32_t ack_cmd = web_ctrl_map -> ack_cmd;
    double ack_ctp_ms = web_ctrl_map -> ack_ctp_ms;

    last_ack_seq = ack_seq;

    uint64_t ack_cycles = rte_get_timer_cycles();

    if ( ack_frame > 0 && ack_frame <= K_FRAMES ) {
        uint32_t idx = ack_frame - 1;

        if ( web_sent_cycles[ idx ] > 0 && ack_cycles >= web_sent_cycles[ idx ] )
            telemetry_log[ idx ].web_ack_ms = ( ( double )( ack_cycles - web_sent_cycles[ idx ] ) / timer_hz ) * 1000.0;
    }

    if ( ack_cmd > 0 && isfinite( ack_ctp_ms ) && ack_ctp_ms >= 0.0 ) {
        struct cmd_record *record = find_cmd_record( ack_cmd );

        if ( record != NULL ) {
            record -> photon_ms = ack_ctp_ms;
            record -> acked = true;

            if ( record -> frame_id > 0 && record -> frame_id <= K_FRAMES )
                telemetry_log[ record -> frame_id - 1 ].cmd_photon_ms = ack_ctp_ms;

            if ( applied_cmd_id == ack_cmd )
                applied_cmd_id = 0;
        }
    }
}

static inline double publish_web_frame( const struct frame_state *state, uint32_t cmd_id, double cmd_apply_ms, double e2e_ms, uint64_t timer_hz, uint64_t *publish_end ) {

    // Purpose: It finalizes the memory-mapped sequence frame after packet reassembly has populated the shared point payload directly

    if ( web_frame_map == NULL || state == NULL || state -> valid_points > MAX_FRAME_POINTS )
        return 0.0;

    uint64_t publish_start = rte_get_timer_cycles();
    struct web_hdr *hdr = ( struct web_hdr * )web_frame_map;
    uint64_t even_seq = web_seq + 2;

    hdr -> frame_id = state -> frame_id;
    hdr -> point_count = state -> valid_points;
    hdr -> original_points = state -> original_points;
    hdr -> arrived_points = state -> arrived_points;
    hdr -> eroded_points = state -> eroded_points;
    hdr -> valid_points = state -> valid_points;
    hdr -> cmd_id = cmd_id;
    hdr -> temporal_skip = state -> temporal_skip;
    hdr -> padding = 0;
    hdr -> yaw = state -> yaw;
    hdr -> pitch = state -> pitch;
    hdr -> zoom = state -> zoom;
    hdr -> align_pad = 0;
    hdr -> e2e_ms = e2e_ms;
    hdr -> cmd_apply_ms = cmd_apply_ms;

    __atomic_thread_fence( __ATOMIC_RELEASE );
    __atomic_store_n( &hdr -> seq, even_seq, __ATOMIC_RELEASE );
    web_seq = even_seq;

    uint64_t publish_stop = rte_get_timer_cycles();

    if ( publish_end != NULL )
        *publish_end = publish_stop;

    return ( ( double )( publish_stop - publish_start ) / timer_hz ) * 1000.0;
}

static void quality_capture_init() {
    if ( mkdir( QUALITY_FOLDER, 0777 ) != 0 && errno != EEXIST ) {
        printf( "[SYSTEM] Error: Failed to create quality directory \"%s\"...\n", QUALITY_FOLDER );
        return;
    }

    quality_buffer = ( uint8_t * )malloc( QUALITY_BUFFER_SIZE );

    if ( quality_buffer == NULL ) {
        printf( "[SYSTEM] Error: Unable to allocate quality capture memory...\n" );
        quality_failed = true;
        return;
    }

    memset( quality_buffer, 0, QUALITY_BUFFER_SIZE );

    quality_size = 0;
    quality_drops = 0;
    quality_failed = false;
}

static inline double save_quality_frame( uint32_t frame_id, uint32_t point_count, uint64_t timer_hz ) {

    // Purpose: It buffers the reconstructed geometry snapshot in memory for eventual offline quality serialization

    if ( quality_buffer == NULL || quality_failed || frame_id == 0 || frame_id > K_FRAMES || point_count > MAX_FRAME_POINTS )
        return NAN;

    size_t points_size = ( size_t )point_count * sizeof( struct host_point );
    size_t record_size = sizeof( struct quality_hdr ) + points_size;

    if ( quality_size + record_size > QUALITY_BUFFER_SIZE ) {
        quality_drops++;
        return NAN;
    }

    uint64_t save_start = rte_get_timer_cycles();

    struct quality_hdr hdr;
    hdr.frame_id = frame_id;
    hdr.point_count = point_count;

    rte_memcpy( quality_buffer + quality_size, &hdr, sizeof( hdr ) );
    quality_size += sizeof( hdr );

    if ( points_size > 0 ) {
        rte_memcpy( quality_buffer + quality_size, web_points, points_size );
        quality_size += points_size;
    }

    uint64_t save_stop = rte_get_timer_cycles();

    return ( ( double )( save_stop - save_start ) / timer_hz ) * 1000.0;
}

static void quality_capture_close() {
    if ( quality_buffer == NULL )
        return;

    FILE *quality_file = fopen( QUALITY_PATH, "wb" );

    if ( quality_file == NULL ) {
        printf( "[SYSTEM] Error: Unable to open quality capture for serialization...\n" );
        quality_failed = true;
    }
    else {
        setvbuf( quality_file, NULL, _IOFBF, 4 * 1024 * 1024 );

        if ( quality_size > 0 && fwrite( quality_buffer, 1, quality_size, quality_file ) != quality_size ) {
            printf( "[SYSTEM] Error: Unable to marshal results entirely...\n" );
            quality_failed = true;
        }

        fflush( quality_file );
        fclose( quality_file );
    }

    free( quality_buffer );
    quality_buffer = NULL;

    if ( quality_drops > 0 )
        printf( "[SYSTEM] Error: Quality capture skipped %u frames...\n", quality_drops );
}

static inline void web_eos_signal() {

    // Purpose: It publishes the terminal stream condition to the asynchronous bridge, preventing further user-originated stance propagation

    FILE *ended = fopen( EOS_PATH, "w" );

    if ( ended != NULL )
        fclose( ended );
}

static inline void user_ready_signal() {

    // Purpose: It issues the node readiness marker after network & shared-memory initialization is complete

    FILE *ready = fopen( READY_PATH, "w" );

    if ( ready != NULL )
        fclose( ready );
}

static void quality_ready_signal() {

    // Purpose: It spawns an explicit filesystem marker acting as a readiness signal for external diagnostic components

    FILE *ready = fopen( QUALITY_READY_PATH, "w" );

    if ( ready != NULL )
        fclose( ready );
}

static inline void reset_frame_state() {

    // Purpose: It clears volatile state fields & packet markers to prepare the environment for sequential reassembly

    memset( &frame_state, 0, sizeof( frame_state ) );
    memset( packet_seen, 0, MAX_FRAME_PACKETS );
}

static inline void begin_frame( const struct dec_hdr *dec, uint64_t arrival_cycles ) {

    // Purpose: It primes the continuous tracking context upon encountering an unprecedented network snapshot

    reset_frame_state();

    frame_state.active = true;
    frame_state.sequence_ok = true;
    frame_state.frame_id = rte_be_to_cpu_32( dec -> frame_id );
    frame_state.original_points = rte_be_to_cpu_32( dec -> original_points );
    frame_state.arrived_points = rte_be_to_cpu_32( dec -> arrived_points );
    frame_state.eroded_points = rte_be_to_cpu_32( dec -> eroded_points );
    frame_state.valid_points = rte_be_to_cpu_32( dec -> valid_points );
    frame_state.temporal_skip = rte_be_to_cpu_16( dec -> temporal_skip );
    frame_state.yaw = be_to_float( dec -> yaw );
    frame_state.pitch = be_to_float( dec -> pitch );
    frame_state.zoom = be_to_float( dec -> zoom );
    frame_state.camera_tx = rte_be_to_cpu_64( dec -> timestamp );
    frame_state.first_arrival = arrival_cycles;
    frame_state.last_arrival = arrival_cycles;

    if ( !quality_capture_enabled && web_frame_map != NULL ) {
        struct web_hdr *hdr = ( struct web_hdr * )web_frame_map;
        uint64_t odd_seq = web_seq + 1;
        __atomic_store_n( &hdr -> seq, odd_seq, __ATOMIC_RELEASE );
    }

    if ( frame_state.temporal_skip == 0 )
        frame_state.temporal_skip = 1;

    if ( frame_state.original_points == 0 || frame_state.arrived_points > frame_state.original_points || frame_state.eroded_points > frame_state.arrived_points || frame_state.valid_points > frame_state.eroded_points || frame_state.valid_points > MAX_FRAME_POINTS )
        frame_state.sequence_ok = false;
}

static inline bool metadata_matches( const struct dec_hdr *dec ) {

    // Purpose: It establishes identity & configuration consistency spanning distinct packets originating from the identical source frame

    if ( frame_state.frame_id != rte_be_to_cpu_32( dec -> frame_id ) )
        return false;

    return frame_state.original_points == rte_be_to_cpu_32( dec -> original_points ) && frame_state.arrived_points == rte_be_to_cpu_32( dec -> arrived_points ) && frame_state.eroded_points == rte_be_to_cpu_32( dec -> eroded_points ) && frame_state.valid_points == rte_be_to_cpu_32( dec -> valid_points ) && frame_state.temporal_skip == rte_be_to_cpu_16( dec -> temporal_skip ) && pose_matches( frame_state.yaw, frame_state.pitch, frame_state.zoom, be_to_float( dec -> yaw ), be_to_float( dec -> pitch ), be_to_float( dec -> zoom ) ) && frame_state.camera_tx == rte_be_to_cpu_64( dec -> timestamp );
}

static inline void finalize_frame( uint64_t timer_hz ) {

    // Purpose: It consolidates reception logs upon completing an entity, exposing valid points to the bridge & archiving diagnostic findings

    if ( !frame_state.active || frame_state.frame_id == 0 || frame_state.frame_id > K_FRAMES ) {
        reset_frame_state();
        return;
    }

    uint32_t frame_id = frame_state.frame_id;
    uint32_t idx = frame_id - 1;
    uint32_t expected_packets = ( frame_state.valid_points > 0 ) ? ( frame_state.valid_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET : 1;

    bool rx_complete = frame_state.sequence_ok && frame_state.rx_packets == expected_packets && ( ( frame_state.valid_points > 0 && frame_state.rx_points == frame_state.valid_points ) || ( frame_state.valid_points == 0 && frame_state.rx_points == 0 ) );

    uint64_t frame_ready = frame_state.last_arrival;
    uint64_t node_exit = frame_ready;

    uint32_t cmd_id = applied_cmd_id;
    double cmd_apply_ms = 0.0;

    if ( active_cmd_id > 0 ) {
        struct cmd_record *record = find_cmd_record( active_cmd_id );

        if ( record != NULL && !record -> reference_seen && frame_ready >= record -> sent_cycles ) {
            record -> reference_seen = true;
            record -> reference_ms = ( ( double )( frame_ready - record -> sent_cycles ) / timer_hz ) * 1000.0;
        }

        if ( record != NULL && !record -> matched && frame_ready >= record -> sent_cycles && pose_matches( frame_state.yaw, frame_state.pitch, frame_state.zoom, record -> yaw, record -> pitch, record -> zoom ) ) {
            record -> matched = true;
            record -> frame_id = frame_id;
            record -> apply_ms = ( ( double )( frame_ready - record -> sent_cycles ) / timer_hz ) * 1000.0;

            cmd_id = record -> cmd_id;
            cmd_apply_ms = record -> apply_ms;
            applied_cmd_id = record -> cmd_id;
            active_cmd_id = 0;
        }
    }

    retry_active_pose( frame_id );

    if ( applied_cmd_id > 0 )
        cmd_id = applied_cmd_id;

    double web_publish_ms = 0.0;

    if ( rx_complete && !quality_capture_enabled ) {
        double e2e_ready_ms = ( frame_ready >= frame_state.camera_tx ) ? ( ( double )( frame_ready - frame_state.camera_tx ) / timer_hz ) * 1000.0 : 0.0;
        web_publish_ms = publish_web_frame( &frame_state, cmd_id, cmd_apply_ms, e2e_ready_ms, timer_hz, &node_exit );
        web_sent_cycles[ idx ] = node_exit;
    }

    double quality_save_ms = ( rx_complete && quality_capture_enabled ) ? save_quality_frame( frame_id, frame_state.valid_points, timer_hz ) : 0.0;

    double receive_sec = ( frame_state.last_arrival >= frame_state.first_arrival ) ? ( double )( frame_state.last_arrival - frame_state.first_arrival ) / timer_hz : 0.0;
    uint64_t logical_bytes = ( uint64_t )frame_state.rx_points * sizeof( struct point_tx ) + ( frame_state.rx_packets > 0 ? sizeof( struct dec_hdr ) : 0 );
    uint64_t network_bytes = ( uint64_t )frame_state.rx_points * sizeof( struct point_tx ) + ( ( uint64_t )frame_state.rx_packets * ( sizeof( struct net_hdr ) + sizeof( struct dec_hdr ) ) );
    double effective_fps = TARGET_FPS / frame_state.temporal_skip;

    struct telemetry_csv *t = &telemetry_log[ idx ];
    memset( t, 0, sizeof( *t ) );

    t -> frame_id = frame_id;
    t -> rx_complete = rx_complete ? 1 : 0;
    t -> current_skip = frame_state.temporal_skip;

    t -> yaw = frame_state.yaw;
    t -> pitch = frame_state.pitch;
    t -> zoom = frame_state.zoom;

    t -> camera_send_timestamp = ( double )frame_state.camera_tx / timer_hz;
    t -> recv_start_timestamp = ( double )frame_state.first_arrival / timer_hz;
    t -> node_exit_timestamp = ( double )node_exit / timer_hz;

    t -> original_points = frame_state.original_points;
    t -> arrived_points = frame_state.arrived_points;
    t -> eroded_points = frame_state.eroded_points;
    t -> valid_points = frame_state.valid_points;
    t -> rx_points = frame_state.rx_points;
    t -> rx_packets = frame_state.rx_packets;
    t -> payload_bytes = ( uint32_t )( ( uint64_t )frame_state.rx_points * sizeof( struct point_tx ) );

    if ( frame_state.valid_points > 0 )
        t -> data_integrity_pct = ( ( double )frame_state.rx_points / frame_state.valid_points ) * 100.0;
    else
        t -> data_integrity_pct = rx_complete ? 100.0 : 0.0;

    t -> internal_throughput_mbs = ( receive_sec > 0.0 ) ? ( ( double )logical_bytes / 1000000.0 ) / receive_sec : 0.0;
    t -> logical_bitrate_mbps = ( logical_bytes * 8.0 * effective_fps ) / 1000000.0;
    t -> network_bitrate_mbps = ( network_bytes * 8.0 * effective_fps ) / 1000000.0;

    t -> arrival_pct = ( frame_state.original_points > 0 ) ? ( ( double )frame_state.arrived_points / frame_state.original_points ) * 100.0 : 0.0;
    t -> erosion_pct = ( frame_state.arrived_points > 0 ) ? ( ( double )frame_state.eroded_points / frame_state.arrived_points ) * 100.0 : 0.0;
    t -> valid_pct = ( frame_state.original_points > 0 ) ? ( ( double )frame_state.valid_points / frame_state.original_points ) * 100.0 : 0.0;

    t -> web_publish_ms = web_publish_ms;
    t -> active_process_ms = ( ( double )frame_state.active_cycles / timer_hz ) * 1000.0 + web_publish_ms;
    t -> total_residency_ms = ( node_exit >= frame_state.first_arrival ) ? ( ( double )( node_exit - frame_state.first_arrival ) / timer_hz ) * 1000.0 : 0.0;
    t -> node_efficiency_pct = ( t -> total_residency_ms > 0.0 ) ? ( t -> active_process_ms / t -> total_residency_ms ) * 100.0 : 0.0;

    t -> camera_node_ms = ( frame_state.first_arrival >= frame_state.camera_tx ) ? ( ( double )( frame_state.first_arrival - frame_state.camera_tx ) / timer_hz ) * 1000.0 : 0.0;
    t -> e2e_latency_ms = ( node_exit >= frame_state.camera_tx ) ? ( ( double )( node_exit - frame_state.camera_tx ) / timer_hz ) * 1000.0 : 0.0;
    t -> reference_e2e_ms = ( frame_ready >= frame_state.camera_tx ) ? ( ( double )( frame_ready - frame_state.camera_tx ) / timer_hz ) * 1000.0 : 0.0;

    if ( session_arrival == 0 ) {
        session_arrival = frame_state.first_arrival;
        session_frame = frame_id;
    }

    if ( session_arrival > 0 && frame_id >= session_frame ) {
        double real_elapsed = ( double )( node_exit - session_arrival ) / timer_hz;
        double expected_elapsed = ( double )( frame_id - session_frame ) / TARGET_FPS;
        t -> schedule_delay_ms = ( real_elapsed - expected_elapsed ) * 1000.0;
    }

    if ( previous_arrival > 0 && frame_id > previous_frame ) {
        double real_interval = ( double )( frame_state.first_arrival - previous_arrival ) / timer_hz;
        double expected_interval = ( double )( frame_id - previous_frame ) / TARGET_FPS;
        t -> instant_jitter_ms = fabs( real_interval - expected_interval ) * 1000.0;
        jitter_ms += ( t -> instant_jitter_ms - jitter_ms ) / 16.0;
    }

    t -> desynced_jitter_ms = jitter_ms;

    previous_arrival = frame_state.first_arrival;
    previous_frame = frame_id;

    if ( cmd_id > 0 ) {
        struct cmd_record *record = find_cmd_record( cmd_id );

        if ( record != NULL && record -> frame_id == frame_id ) {
            t -> cmd_id = record -> cmd_id;
            t -> reference_cmd_ms = record -> reference_ms;
            t -> cmd_apply_ms = record -> apply_ms;
            t -> cmd_photon_ms = record -> photon_ms;
        }
    }

    t -> quality_save_ms = quality_save_ms;
    t -> mean_error = NAN;
    t -> geom_rmse = NAN;
    t -> chamfer = NAN;
    t -> hausdorff = NAN;
    t -> mean_mm = NAN;
    t -> rmse_mm = NAN;
    t -> chamfer_mm = NAN;
    t -> hausdorff_mm = NAN;

    last_frame_id = frame_id;
    
    reset_frame_state();
}

static void telemetry_to_csv() {
    struct stat st = { 0 };

    if ( stat( TELEMETRY_FOLDER, &st ) == -1 )
        if ( mkdir( TELEMETRY_FOLDER, 0777 ) == -1 ) {
            printf( "[SYSTEM] Error: Failed to create directory \"%s\".\n", TELEMETRY_FOLDER );
            return;
        }

    FILE *f = fopen( TELEMETRY_PATH, "w" );

    if ( f == NULL ) {
        printf( "[SYSTEM] Error: Could not open telemetry file.\n" );
        return;
    }

    fprintf( f, "frame_id;rx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;arrived_points;eroded_points;valid_points;rx_points;rx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;arrival_pct;erosion_pct;valid_pct;web_publish_ms;web_ack_ms;active_process_ms;total_residency_ms;node_efficiency_pct;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;cmd_id;reference_cmd_ms;cmd_apply_ms;cmd_photon_ms;quality_save_ms;mean_error;geom_rmse;chamfer;hausdorff;mean_mm;rmse_mm;chamfer_mm;hausdorff_mm\n" );

    for ( uint32_t i = 0; i < K_FRAMES; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];

        if ( t -> frame_id == 0 )
            continue;

        fprintf( f, "%u;%u;%u;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%u;%u;%u;%u;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%.3f;%.3f;%.3f;%.3f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n", t -> frame_id, t -> rx_complete, t -> current_skip, t -> yaw, t -> pitch, t -> zoom, t -> camera_send_timestamp, t -> recv_start_timestamp, t -> node_exit_timestamp, t -> original_points, t -> arrived_points, t -> eroded_points, t -> valid_points, t -> rx_points, t -> rx_packets, t -> payload_bytes, t -> data_integrity_pct, t -> internal_throughput_mbs, t -> logical_bitrate_mbps, t -> network_bitrate_mbps, t -> arrival_pct, t -> erosion_pct, t -> valid_pct, t -> web_publish_ms, t -> web_ack_ms, t -> active_process_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> camera_node_ms, t -> e2e_latency_ms, t -> reference_e2e_ms, t -> schedule_delay_ms, t -> instant_jitter_ms, t -> desynced_jitter_ms, t -> cmd_id, t -> reference_cmd_ms, t -> cmd_apply_ms, t -> cmd_photon_ms, t -> quality_save_ms, t -> mean_error, t -> geom_rmse, t -> chamfer, t -> hausdorff, t -> mean_mm, t -> rmse_mm, t -> chamfer_mm, t -> hausdorff_mm );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n\n", TELEMETRY_PATH );
}

static inline bool parse_packet( struct rte_mbuf *m, uint64_t timer_hz ) {

    // Purpose: It systematically strips the "Eth" / "IPv4" / "UDP" boundaries, authenticating "DPDK" network payloads & routing valid geometry points to the reassembly buffer

    size_t min_len = sizeof( struct net_hdr ) + sizeof( struct dec_hdr );

    if ( unlikely( !rte_pktmbuf_is_contiguous( m ) || rte_pktmbuf_pkt_len( m ) < min_len ) )
        return false;

    struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );

    if ( unlikely( !rte_is_same_ether_addr( &eth -> src_addr, &sff3_user_mac ) || !rte_is_same_ether_addr( &eth -> dst_addr, &user_mac ) || eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) )
        return false;

    struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );

    if ( unlikely( ipv4 -> version_ihl != 0x45 || ipv4 -> next_proto_id != IPPROTO_UDP || ipv4 -> src_addr != rte_cpu_to_be_32( SFF3_USER_IP ) || ipv4 -> dst_addr != rte_cpu_to_be_32( USER_IP ) ) )
        return false;

    struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );

    if ( unlikely( udp -> src_port != rte_cpu_to_be_16( SFF3_USER_PORT ) || udp -> dst_port != rte_cpu_to_be_16( USER_PORT ) ) )
        return false;

    uint16_t udp_length = rte_be_to_cpu_16( udp -> dgram_len );
    uint16_t ipv4_length = rte_be_to_cpu_16( ipv4 -> total_length );

    if ( unlikely( udp_length < sizeof( struct rte_udp_hdr ) + sizeof( struct dec_hdr ) || ipv4_length != sizeof( struct rte_ipv4_hdr ) + udp_length || rte_pktmbuf_pkt_len( m ) != sizeof( struct rte_ether_hdr ) + ipv4_length ) )
        return false;

    uint16_t udp_payload_len = udp_length - sizeof( struct rte_udp_hdr );
    struct dec_hdr *dec = ( struct dec_hdr * )( udp + 1 );
    uint32_t frame_id = rte_be_to_cpu_32( dec -> frame_id );

    if ( frame_id == END_OF_STREAM ) {
        if ( udp_payload_len != sizeof( struct dec_hdr ) )
            return false;

        eos_received = true;
        web_eos_signal();

        if ( frame_state.active )
            finalize_frame( timer_hz );

        if ( !quality_capture_enabled ) {
            uint64_t wait_start = rte_get_timer_cycles();
            uint64_t minimum_wait = timer_hz / 5;
            uint64_t maximum_wait = ( timer_hz * COMMAND_TIME ) / 1000;

            while ( 1 ) {
                process_web_ack( timer_hz );

                uint64_t elapsed = rte_get_timer_cycles() - wait_start;

                if ( elapsed >= minimum_wait && applied_cmd_id == 0 )
                    break;

                if ( elapsed >= maximum_wait )
                    break;

                rte_delay_us_sleep( 1000 );
            }
        }

        if ( !csv_written ) {
            if ( quality_capture_enabled )
                quality_capture_close();

            telemetry_to_csv();

            if ( quality_capture_enabled ) {
                quality_ready_signal();
                printf( "[SYSTEM] Waiting for parameter assessments to complete...\n" );

                while ( access( QUALITY_DONE_PATH, F_OK ) != 0 )
                    rte_delay_us_sleep( 1000 );
            }

            csv_written = true;
            printf( "[SYSTEM] End of stream detected. Changing to \"idle\" state...\n" );
        }

        return true;
    }

    if ( frame_id == 0 || frame_id > K_FRAMES || frame_id <= last_frame_id )
        return false;

    uint32_t sequence_number = rte_be_to_cpu_32( dec -> sequence_number );
    uint32_t points_in_packet = rte_be_to_cpu_32( dec -> points_in_packet );
    uint32_t valid_points = rte_be_to_cpu_32( dec -> valid_points );
    uint16_t point_payload_len = udp_payload_len - sizeof( struct dec_hdr );

    bool populated_frame = valid_points > 0 && points_in_packet > 0 && points_in_packet <= POINTS_PER_PACKET && point_payload_len == points_in_packet * sizeof( struct point_tx );
    bool empty_frame = valid_points == 0 && sequence_number == 0 && points_in_packet == 0 && point_payload_len == 0;

    if ( unlikely( !populated_frame && !empty_frame ) )
        return false;

    uint64_t arrival_cycles = rte_get_timer_cycles();

    if ( !frame_state.active || frame_id != frame_state.frame_id ) {
        if ( frame_state.active )
            finalize_frame( timer_hz );

        begin_frame( dec, arrival_cycles );
    }
    else if ( unlikely( !metadata_matches( dec ) ) )
        frame_state.sequence_ok = false;

    if ( sequence_number >= MAX_FRAME_PACKETS ) {
        frame_state.sequence_ok = false;
        return false;
    }

    if ( packet_seen[ sequence_number ] ) {
        frame_state.sequence_ok = false;
        return false;
    }

    uint64_t active_start = rte_get_timer_cycles();

    packet_seen[ sequence_number ] = 1;

    if ( sequence_number != frame_state.expected_seq )
        frame_state.sequence_ok = false;

    if ( sequence_number >= frame_state.expected_seq )
        frame_state.expected_seq = sequence_number + 1;

    uint32_t point_offset = sequence_number * POINTS_PER_PACKET;

    if ( point_offset + points_in_packet > frame_state.valid_points || point_offset + points_in_packet > MAX_FRAME_POINTS ) {
        frame_state.sequence_ok = false;
        return false;
    }

    if ( points_in_packet > 0 ) {
        struct point_tx *wire_points = ( struct point_tx * )( dec + 1 );

        for ( uint32_t i = 0; i < points_in_packet; i++ ) {
            struct host_point *target = &web_points[ point_offset + i ];

            target -> x = be_to_float( wire_points[ i ].x );
            target -> y = be_to_float( wire_points[ i ].y );
            target -> z = be_to_float( wire_points[ i ].z );
            target -> r = wire_points[ i ].r;
            target -> g = wire_points[ i ].g;
            target -> b = wire_points[ i ].b;
            target -> padding = 0;
        }
    }

    frame_state.rx_points += points_in_packet;
    frame_state.rx_packets++;
    frame_state.last_arrival = arrival_cycles;

    uint64_t active_stop = rte_get_timer_cycles();
    frame_state.active_cycles += active_stop - active_start;

    uint32_t expected_packets = ( frame_state.valid_points > 0 ) ? ( frame_state.valid_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET : 1;
    bool frame_complete = frame_state.rx_packets == expected_packets && ( ( frame_state.valid_points > 0 && frame_state.rx_points == frame_state.valid_points ) || ( frame_state.valid_points == 0 && frame_state.rx_points == 0 ) );

    if ( frame_complete )
        finalize_frame( timer_hz );

    return true;
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();
    struct rte_mbuf *bufs[ BURST_SIZE ];

    printf( "[SYSTEM] Listening on the SFF3-facing end-device link...\n\n" );

    while ( 1 ) {
        process_web_ctrl();
        process_web_ack( timer_hz );

        if ( csv_written ) {
            rte_delay_us_sleep( 1000 );
            continue;
        }

        uint16_t nb_rx = rte_eth_rx_burst( PORT_SFF3, 0, bufs, BURST_SIZE );

        if ( nb_rx == 0 ) {
            rte_pause();
            continue;
        }

        for ( uint16_t i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];

            parse_packet( m, timer_hz );
            rte_pktmbuf_free( m );
        }
    }

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It boots the final destination microservice, assembling the network pool, initializing a common storage region for the "WebSocket" bridge & scheduling the asynchronous "DPDK" listener

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"User\" microservice...\n" );

    const char *quality_env = getenv( "QUALITY_CAPTURE" );
    quality_capture_enabled = ( quality_env != NULL && strcmp( quality_env, "1" ) == 0 );

    unlink( QUALITY_READY_PATH );
    unlink( READY_PATH );
    unlink( EOS_PATH );

    eos_received = false;

    if ( quality_capture_enabled )
        quality_capture_init();

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );

    if ( mbuf_pool == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF3, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: SFF3-facing virtual port configuration failed...\n" );

    packet_seen = ( uint8_t * )calloc( MAX_FRAME_PACKETS, sizeof( uint8_t ) );

    if ( packet_seen == NULL )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Frame buffers could not be allocated...\n" );

    if ( shared_map_init() != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Shared-memory initialization failed...\n" );

    printf( "\n" );

    pose_header_init( &pose_template_hdr );
    reset_frame_state();

    user_ready_signal();

    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE )
        worker_loop( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    if ( quality_capture_enabled )
        quality_capture_close();

    shared_map_free();

    free( packet_seen );
    
    rte_eal_cleanup();

    return 0;
}