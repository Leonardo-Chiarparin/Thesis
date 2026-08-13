#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_cycles.h>

// Configuration variables
#define SEQUENCE_FOLDER "/shared/data/loot/bin"
#define TELEMETRY_FOLDER "/shared/log/camera"
#define TELEMETRY_PATH "/shared/log/camera/telemetry_camera.csv"

#define K_FRAMES 300

#define TARGET_FPS 30.0 

#define BURST_SIZE 32
#define MAX_RETRIES 2048 // T_max = MAX_RETRIES * ( C_pause / F_cpu ); ( average ) C_pause = 50 clock cycles ( using Intel CPUs with Skylake architecture ), F_cpu closes to 3.40 GHz

// Loading strategies ( I/O benchmarks )
#define CACHE_MODE_BEST 0 // ( default ) "RAM" pre-cached: "malloc" + "fread" executed into the "main" function ( zero latency )
#define CACHE_MODE_MIDDLE 1 // "RAM" pre-allocated: "malloc" within the "main", "fread" ( disk to memory ) placed inside the isochronous loop
#define CACHE_MODE_WORST  2 // no a-priori computation: "malloc" + "fread" carried out in the transmission ( "worker ") cycle

#define CACHE_MODE CACHE_MODE_MIDDLE

#define PACING_MARGIN 100 // %
#define PACING_MODE 0

#define END_OF_STREAM 0xFFFFFFFF
#define WAITING_TIME 5000

// Memory pool settings for "mbufs"
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// Transmission bonds ( networking parameters )
#define PORT_TX 0
#define SRC_IP RTE_IPV4( 10, 0, 0, 2 )
#define DST_IP RTE_IPV4( 10, 0, 1, 254 ) // SFF1
#define SRC_PORT 49432
#define DST_PORT 5001

// MTU settings
// Note: A standard point is 16 bytes. 80 points = 1280 bytes of payload, fitting comfortably within the standard 1500-byte "Ethernet" "MTU"
#define POINTS_PER_PACKET 80
#define POINT_SIZE_BYTES 16

// Packet structures for "DPDK" forwarding
struct cam_hdr { // placed immediately after the UDP header, providing synchronization & sequence tracking for downstream "VNFs"
    uint32_t frame_id;
    uint32_t sequence_number;
    uint64_t timestamp;
    float yaw; // 0.0f
    float pitch; // 0.0f
    float zoom; // 1.0f
    uint16_t temporal_skip; // 1
    uint16_t padding; // maintains a 40-byte strict organization
    uint32_t original_points;
    uint32_t points_in_packet;
} __attribute__((__packed__)); // to avoid compiler padding, ensuring memory alignment strictly matches network protocol layouts

struct net_hdr {
    struct rte_ether_hdr ethernet;
    struct rte_ipv4_hdr ipv4;
    struct rte_udp_hdr udp;
} __attribute__((__packed__));

struct full_hdr {
    struct net_hdr net;
    struct cam_hdr cam;
} __attribute__((__packed__));

// Content abstractions
struct frame_data {
    uint8_t *buffer;
    size_t size;
    uint32_t point_count;
    char file_path[ 128 ];
};

struct telemetry_csv {
    uint32_t frame_id;

    double timestamp_start_tx;

    uint32_t tx_points;
    uint32_t tx_packets;
    uint32_t payload_bytes;
    double internal_throughput_mbs;
    double network_bitrate_mbps;

    double disk_io_ms;
    double tx_duration_ms;
    double active_tx_ms;
    double total_residency_ms;
    double node_efficiency_pct;
    
    uint32_t tx_retries; 
    uint32_t mbuf_starvation;
};

// Global context
struct frame_data frames[ K_FRAMES ];
struct telemetry_csv telemetry_log[ K_FRAMES ];
int loaded_frames = 0;
struct rte_mempool *mbuf_pool;
struct full_hdr template_hdr;
bool csv_written = false;

// Helper functions
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

static void header_init( struct full_hdr *hdr ) {
    memset( hdr, 0, sizeof( struct full_hdr ) );
    
    struct rte_ether_addr src_mac = { { 0x00, 0x00, 0x00, 0x00, 0x01, 0x01 } };
    struct rte_ether_addr dst_mac = { { 0x00, 0x00, 0x00, 0x00, 0x01, 0x02 } };
    
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

    // Compiling static fields
    hdr -> cam.yaw = 0.0f;
    hdr -> cam.pitch = 0.0f;
    hdr -> cam.zoom = 1.0f;
    hdr -> cam.temporal_skip = htons( 1 );
    hdr -> cam.padding = 0;
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

    fprintf( f, "frame_id;timestamp_start_tx;tx_points;tx_packets;payload_bytes;internal_throughput_mbs;network_bitrate_mbps;disk_io_ms;tx_duration_ms;active_tx_ms;total_residency_ms;node_efficiency_pct;tx_retries;mbuf_starvation\n" );

    for ( int i = 0; i < loaded_frames; i++ ) {
        struct telemetry_csv *t = &telemetry_log[ i ];
        fprintf( f, "%u;%.6f;%u;%u;%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u\n", t -> frame_id, t -> timestamp_start_tx, t -> tx_points, t -> tx_packets, t -> payload_bytes, t -> internal_throughput_mbs, t -> network_bitrate_mbps, t -> disk_io_ms, t -> tx_duration_ms, t -> active_tx_ms, t -> total_residency_ms, t -> node_efficiency_pct, t -> tx_retries, t -> mbuf_starvation );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();
    uint64_t frame_cycles = ( uint64_t )( timer_hz / TARGET_FPS );
    
    struct rte_mbuf *burst_buffer[ BURST_SIZE ];
    struct rte_mbuf *dump_bufs[ BURST_SIZE ];

    uint16_t burst_points[ BURST_SIZE ];

    // Defining common checksum for full packets
    uint16_t standard_payload_size = POINTS_PER_PACKET * POINT_SIZE_BYTES;
    template_hdr.net.ipv4.total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + standard_payload_size );
    template_hdr.net.ipv4.hdr_checksum = 0;
    template_hdr.net.ipv4.hdr_checksum = rte_ipv4_cksum( &template_hdr.net.ipv4 );
    template_hdr.net.udp.dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + sizeof( struct cam_hdr ) + standard_payload_size );

    const size_t outer_len = sizeof( struct full_hdr );

    printf( "[SYSTEM] Streaming is about to begin at %.1f FPS...\n\n", TARGET_FPS );
    
    // Primary execution loop ( "Run-to-Completion" methodology )
    while( 1 ) {   
        uint64_t start_time = rte_get_timer_cycles();

        for ( int frame = 0; frame < loaded_frames; frame++ ) {
            uint32_t frame_id = frame + 1;

            uint64_t expected_time = start_time + ( frame * frame_cycles );
            uint64_t t_start_residency = rte_get_timer_cycles();

            double current_disk_io_ms = 0.0;

            if ( CACHE_MODE != CACHE_MODE_BEST ) {
                uint64_t t_start_io = rte_get_timer_cycles();
                
                if ( CACHE_MODE == CACHE_MODE_WORST ) {
                    frames[ frame ].buffer = rte_malloc( "frame_buffer", frames[ frame ].size, RTE_CACHE_LINE_SIZE );
                    
                    if ( frames[ frame ].buffer == NULL ) 
                        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"rte_malloc\" failed in loop...\n" );
                }

                FILE *fp = fopen( frames[ frame ].file_path, "rb" );

                if ( fp ) {
                    if ( fread( frames[ frame ].buffer, 1, frames[ frame ].size, fp ) != frames[ frame ].size )
                        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: I/O anomaly while reading file \"%s\"...\n", frames[ frame ].file_path );
                    
                    fclose( fp );
                }
                
                uint64_t t_end_io = rte_get_timer_cycles();
                current_disk_io_ms = ( double )( t_end_io - t_start_io ) * 1000.0 / timer_hz;
            }

            uint32_t total_points = frames[ frame ].point_count;
            uint32_t points_sent = 0;
            uint32_t sequence_number = 0;
            int burst_idx = 0;

            uint32_t frame_tx_retries = 0;
            uint32_t frame_mbuf_drops = 0;

            uint32_t frame_tx_packets = 0;
            uint32_t frame_tx_points = 0;

            uint64_t frame_active_tx_cycles = 0;
            uint64_t frame_pacing_cycles = 0;

            uint8_t *raw_points_ptr = frames[ frame ].buffer;
            
            uint16_t nb_rx = rte_eth_rx_burst( PORT_TX, 0, dump_bufs, BURST_SIZE );
            
            for ( uint16_t i = 0; i < nb_rx; i++ ) 
                rte_pktmbuf_free( dump_bufs[ i ] );

            uint64_t t_send_start = rte_get_timer_cycles();

            uint64_t burst_interval = 0;
            uint64_t next_burst_time = 0;

            if ( PACING_MODE ) {
                uint64_t target_end_time = start_time + ( ( frame + 1 ) * frame_cycles );
                uint64_t send_window_cycles = ( target_end_time > t_send_start ) ? ( target_end_time - t_send_start ) : 0;
                
                send_window_cycles = ( send_window_cycles * PACING_MARGIN ) / 100; 
                
                uint32_t total_bursts = ( ( total_points + POINTS_PER_PACKET - 1 ) / POINTS_PER_PACKET + BURST_SIZE - 1 ) / BURST_SIZE;
                
                burst_interval = ( total_bursts > 0 ) ? ( send_window_cycles / total_bursts ) : 0;
                next_burst_time = t_send_start;
            }
            
            // Chunking & transmission
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

                    // Zero-copy delivering layout
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

                // Launching burst whenever it is full or represents the last packet
                if ( burst_idx == BURST_SIZE || points_sent == total_points ) {
                    if ( PACING_MODE )
                        if ( burst_interval > 0 ) {
                            uint64_t t_pacing_start = rte_get_timer_cycles();

                            while ( rte_get_timer_cycles() < next_burst_time )
                                rte_pause();

                            uint64_t t_pacing_end = rte_get_timer_cycles();

                            frame_pacing_cycles += t_pacing_end - t_pacing_start;
                        }
                    
                    uint16_t sent = 0;
                    uint16_t retries = 0;

                    const uint16_t pause_window = BURST_SIZE * 0.5; 

                    while ( sent < burst_idx ) {
                        uint64_t t_active_tx_start = rte_get_timer_cycles();
                        
                        uint16_t nb_tx = rte_eth_tx_burst( PORT_TX, 0, &burst_buffer[ sent ], burst_idx - sent );
                        
                        uint64_t t_active_tx_end = rte_get_timer_cycles();
                        frame_active_tx_cycles += t_active_tx_end - t_active_tx_start;

                        if ( nb_tx > 0 ) {
                            frame_tx_packets += nb_tx;

                            for ( uint16_t j = 0; j < nb_tx; j++ )
                                frame_tx_points += burst_points[ sent + j ];
                        }

                        sent += nb_tx;

                        if ( nb_tx == 0 ) {
                            frame_tx_retries++;

                            if ( ++retries > MAX_RETRIES ) {
                                for ( int k = sent; k < burst_idx; k++ ) 
                                    rte_pktmbuf_free( burst_buffer[ k ] );
                                
                                break;
                            }

                            uint16_t pause_count = ( retries < pause_window ) ? retries : pause_window;

                            for ( uint16_t p = 0; p < pause_count; p++ )
                                rte_pause(); // yielding the mechanism to prevent starvation
                        } 
                        else
                            retries = 0;
                    }

                    burst_idx = 0;
                    next_burst_time += burst_interval;
                }
            }

            uint64_t t_send_end = rte_get_timer_cycles();

            if ( CACHE_MODE == CACHE_MODE_WORST ) {
                rte_free( frames[ frame ].buffer );
                frames[ frame ].buffer = NULL;
            }

            // Telemetry aggregation ( in-memory logging )
            if ( !csv_written ) {
                uint64_t send_cycles = t_send_end - t_send_start;

                if ( send_cycles >= frame_pacing_cycles )
                    send_cycles -= frame_pacing_cycles;

                double send_duration_sec = ( double )send_cycles / timer_hz;
                double residency_sec = ( double )( t_send_end - t_start_residency ) / timer_hz;

                double active_tx_sec = ( double )frame_active_tx_cycles / timer_hz;

                uint64_t transmitted_payload_bytes = ( uint64_t )frame_tx_points * POINT_SIZE_BYTES;
                uint64_t logical_frame_bytes = transmitted_payload_bytes + ( frame_tx_packets > 0 ? sizeof( struct cam_hdr ) : 0 );

                telemetry_log[ frame ].frame_id = frame_id;
                telemetry_log[ frame ].timestamp_start_tx = ( double )t_send_start / timer_hz;
                telemetry_log[ frame ].tx_duration_ms = send_duration_sec * 1000.0;
                telemetry_log[ frame ].payload_bytes = transmitted_payload_bytes;
                telemetry_log[ frame ].disk_io_ms = current_disk_io_ms;
                telemetry_log[ frame ].internal_throughput_mbs = ( send_duration_sec > 0 ) ? ( ( double )logical_frame_bytes / 1000000.0 ) / send_duration_sec : 0.0;
                telemetry_log[ frame ].network_bitrate_mbps = ( logical_frame_bytes * 8.0 * TARGET_FPS ) / 1000000.0;
                telemetry_log[ frame ].total_residency_ms = residency_sec * 1000.0;
                telemetry_log[ frame ].tx_packets = frame_tx_packets;
                telemetry_log[ frame ].tx_points = frame_tx_points;
                telemetry_log[ frame ].tx_retries = frame_tx_retries;
                telemetry_log[ frame ].mbuf_starvation = frame_mbuf_drops;
                telemetry_log[ frame ].active_tx_ms = active_tx_sec * 1000.0;

                double disk_io_sec = current_disk_io_ms / 1000.0;
                
                if ( residency_sec > 0 )
                    telemetry_log[ frame ].node_efficiency_pct = ( ( disk_io_sec + send_duration_sec ) / residency_sec ) * 100.0;
                else
                    telemetry_log[ frame ].node_efficiency_pct = 0.0;
            }

            // Isochronous wait by sleeping until the exact deadline for the next frame
            while ( rte_get_timer_cycles() < expected_time ) 
                rte_pause();
        }

        // "EOS" signaling
        struct rte_mbuf *eos_burst[ BURST_SIZE ];
        int eos_count = 0;
        
        // Sending multiple instances to ensure "UDP" delivery
        for ( int i = 0; i < BURST_SIZE; i++ ) {
            uint64_t t0 = rte_get_timer_cycles();
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
            uint16_t retries = 0;

            while ( sent < eos_count ) {
                uint16_t nb_tx = rte_eth_tx_burst( PORT_TX, 0, &eos_burst[ sent ], eos_count - sent );
                sent += nb_tx;

                if ( nb_tx == 0 ) {
                    if ( ++retries > MAX_RETRIES ) {
                        for ( uint16_t i = sent; i < eos_count; i++ )
                            rte_pktmbuf_free( eos_burst[ i ] );

                        break;
                    }

                    rte_pause();
                } 
                else
                    retries = 0;
            }
        }

        // Exporting telemetry to ".csv" after the first sequence is completely transmitted
        if ( !csv_written ) {
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

    // Purpose: This application simulates a high-fidelity 3D volumetric sensor ( e.g., "LiDAR" ). It reads pre-converted binary point cloud data ( ".bin" ) into memory & streams it isochronously at a strict target framerate ( e.g., 30 FPS ). 
    //          Spatial & photometric processing ( e.g., "YUV" conversion ) are intentionally omitted, offloading these tasks to downstream "VNFs" ( SFF1, Encoder ) to preserve line-rate transmission capabilities on constrained core topologies.
    //          Telemetry metrics are gathered in-memory during the hot path & exported to a ".csv" file post-transmission to preserve correct timing & avoid I/O bottlenecks during the streaming phase

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );
    
    if ( ret < 0 ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n" );

    printf( "[SYSTEM] Booting the \"Camera\" microservice...\n" );

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );
    
    if ( mbuf_pool == NULL ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_TX, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Virtual port configuration failed...\n" );

    printf( "\n[SYSTEM] Preparing volumetric data for modality \"%s\"...\n", ( CACHE_MODE == CACHE_MODE_BEST ) ? "BEST" : ( CACHE_MODE == CACHE_MODE_MIDDLE ) ? "MIDDLE" : ( CACHE_MODE == CACHE_MODE_WORST ) ? "WORST" : "" );
    
    for ( int i = 0; i < K_FRAMES; i++ ) {
        snprintf( frames[ i ].file_path, sizeof( frames[ i ].file_path ), "%s/loot_vox10_%d.bin", SEQUENCE_FOLDER, i + 1000 );
        
        FILE *fp = fopen( frames[ i ].file_path, "rb" );
        
        if ( !fp ) {
            printf( "[SYSTEM] Error: File \"%s\" not found...\n", frames[ i ].file_path );
            break;
        }

        fseek( fp, 0, SEEK_END );
        size_t file_size = ftell( fp );
        rewind( fp );

        if ( file_size % POINT_SIZE_BYTES != 0 )
            rte_exit( EXIT_FAILURE, "[SYSTEM] Error: File size did not match about \"%s\"...\n", frames[ i ].file_path );

        frames[ i ].size = file_size;
        frames[ i ].point_count = file_size / POINT_SIZE_BYTES;

        if ( CACHE_MODE != CACHE_MODE_WORST ) {
            frames[ i ].buffer = rte_malloc( "frame_buffer", file_size, RTE_CACHE_LINE_SIZE );
            
            if ( frames[ i ].buffer == NULL ) 
                rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"rte_malloc\" failed...\n" );

            if ( CACHE_MODE == CACHE_MODE_BEST ) 
                if ( fread( frames[ i ].buffer, 1, file_size, fp ) != file_size )
                    rte_exit( EXIT_FAILURE, "[SYSTEM] Error: I/O anomaly while reading file \"%s\"...\n", frames[ i ].file_path );
            
        }

        fclose( fp );
        loaded_frames++;
    }

    printf( "[SYSTEM] Reference elements structurally mapped: %d.\n\n", loaded_frames );

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