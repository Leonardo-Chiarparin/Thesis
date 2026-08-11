#include "encoder.h"
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <queue>
#include <map>
#include <vector>
#include <sched.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <inttypes.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_cycles.h>

static struct rte_mempool *mbuf_pool;
static struct rte_ether_addr enc_mac  = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x02 } }; 
static struct rte_ether_addr sff2_mac = { { 0x00, 0x00, 0x00, 0x00, 0x03, 0x01 } }; 

int ffmpeg_in[ 2 ]; 
int ffmpeg_out[ 2 ]; 
pid_t ffmpeg_pid;

uint16_t current_skip = 0;
float current_yaw = 0.0f, current_pitch = 0.0f, current_zoom = 1.0f;
char current_event[ 16 ] = "Idle";

struct telemetry_csv telemetry_log[ K_FRAMES ];
bool csv_written = false;
bool eos_received = false;

struct enc_hdr encoder_metadata[ K_FRAMES ];
struct cam_hdr camera_metadata[ K_FRAMES ];

// Asynchronous synchronization bindings for "FFmpeg"
uint32_t current_out_frame_id = 0;
uint32_t last_processed_frame_id = 0;

static uint32_t current_mpeg_packet_id = 0;

static std::vector< uint8_t > ts_pending;
static std::vector< uint8_t > mpeg_chunk;

// Host construct shielding against "DPDK" starvation
struct FrameBuffer {
    std::vector< float > x, y, z;
    std::vector< uint8_t > r, g, b;

    uint32_t original_points = 0;
    uint32_t rx_packets = 0;
    uint32_t payload_bytes = 0;

    uint64_t conversion_cycles = 0;

    uint64_t camera_tx = 0;
    uint64_t first_arrival = 0;
    uint64_t last_arrival = 0;
    uint64_t frame_ready = 0;

    struct cam_hdr cam;
    struct int_hdr meta;
};

std::map< uint32_t, FrameBuffer > frame_buffers;

struct EncodeTimerEntry {
    uint32_t frame_id;
    uint64_t start_cycles;
};

std::queue< EncodeTimerEntry > encode_timer_queue;

static const size_t outer_len = sizeof( struct rte_ether_hdr ) + sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr );

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

static inline void header_init( struct rte_ether_hdr *eth, struct rte_ipv4_hdr *ipv4, struct rte_udp_hdr *udp, struct nsh_hdr *nsh, uint16_t udp_payload_len, uint8_t si ) {
    memset( eth, 0, outer_len + sizeof( struct nsh_hdr ) );

    rte_ether_addr_copy( &enc_mac, &eth -> src_addr );
    rte_ether_addr_copy( &sff2_mac, &eth -> dst_addr );

    eth -> ether_type = rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 );

    ipv4 -> version_ihl = 0x45;
    ipv4 -> time_to_live = 64;
    ipv4 -> next_proto_id = IPPROTO_UDP;
    ipv4 -> src_addr = rte_cpu_to_be_32( RTE_IPV4( 10, 0, 3, 2 ) );
    ipv4 -> dst_addr = rte_cpu_to_be_32( RTE_IPV4( 10, 0, 3, 1 ) );
    ipv4 -> total_length = rte_cpu_to_be_16( sizeof( struct rte_ipv4_hdr ) + sizeof( struct rte_udp_hdr ) + udp_payload_len );

    udp -> src_port = rte_cpu_to_be_16( UDP_PORT );
    udp -> dst_port = rte_cpu_to_be_16( UDP_PORT );
    udp -> dgram_len = rte_cpu_to_be_16( sizeof( struct rte_udp_hdr ) + udp_payload_len );
    udp -> dgram_cksum = 0;

    nsh -> base_flags_ttl_len = htons( 0x0006 );
    nsh -> md_type = 0x02;
    nsh -> next_protocol = 0x03;
    nsh -> serv_path_hdr = htonl( ( PRIMARY_SPI << 8 ) | si );

    ipv4 -> hdr_checksum = 0;
    ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );
}

static inline bool ts_starts_video_pes( const uint8_t *ts ) {
    if ( ts[ 0 ] != 0x47 )
        return false;

    if ( ( ts[ 1 ] & 0x40 ) == 0 )
        return false;

    uint8_t adaptation_control = ( ts[ 3 ] >> 4 ) & 0x03;

    if ( adaptation_control == 0 || adaptation_control == 2 )
        return false;

    size_t payload_offset = 4;

    if ( adaptation_control == 3 ) {
        uint8_t adaptation_len = ts[ 4 ];
        payload_offset += 1 + adaptation_len;
    }

    if ( payload_offset + 4 > TS_PACKET_SIZE )
        return false;

    return ts[ payload_offset ] == 0x00 &&
           ts[ payload_offset + 1 ] == 0x00 &&
           ts[ payload_offset + 2 ] == 0x01 &&
           ( ts[ payload_offset + 3 ] & 0xF0 ) == 0xE0;
}

void ffmpeg_init() {
    
    // Purpose: It establishes a unidirectional inter-process communication ( "IPC" ) channel via non-blocking pipes & spawns a dedicated "FFmpeg" subprocess. 
    //          This component is isolated to a specific logical core & is strictly configured for hardware-accelerated ( "NVENC" ) encoding of raw "YUV" frames into an "H.265" stream, adhering to ultra-low latency & constant bitare ( "CBR" ) policies
    
    if ( pipe( ffmpeg_in ) < 0 || pipe( ffmpeg_out ) < 0 ) {
        perror( "[SYSTEM] Error: Failed to create \"FFmpeg\" pipes...\n" );
        exit( 1 );
    }

    fcntl( ffmpeg_in[ 1 ], F_SETFL, O_NONBLOCK ); 
    fcntl( ffmpeg_out[ 0 ], F_SETFL, O_NONBLOCK );
    
    ffmpeg_pid = fork();

    if ( ffmpeg_pid == 0 ) {
        cpu_set_t cpuset; 
        CPU_ZERO( &cpuset );
        CPU_SET( 0, &cpuset );
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
        
        execlp( "ffmpeg", "ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo", "-s", res, "-pix_fmt", "yuv420p", "-r", "30", "-i", "-", "-c:v", "hevc_nvenc", "-preset", "p2", "-tune", "ull", "-rc", "cbr", "-b:v", TARGET_BITRATE_MBPS, "-maxrate", TARGET_BITRATE_MBPS, "-bufsize", TARGET_BUFFER_SIZE, "-g", "15", "-forced-idr", "1", "-vstats_file", FFMPEG_PATH, "-f", "mpegts", "-", NULL );
        exit( 1 );
    }

    close( ffmpeg_in[ 0 ] ); 
    close( ffmpeg_out[ 1 ] );
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
    
    fprintf( f, "frame_id;status;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;clock_offset_ms;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;network_bitrate_mbps;conversion_ms;projection_ms;tx_duration_ms;active_process_ms;total_processing_ms;total_residency_ms;node_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_to_node_latency_ms;end_to_end_latency_ms;schedule_delay_ms;network_jitter_ms;wait_raw_queue_ms;wait_render_queue_ms;encode_h265_ms;mpeg_bytes_generated;tx_retries;mbuf_starvation\n" );
    
    uint32_t limit = ( last_processed_frame_id < K_FRAMES ) ? last_processed_frame_id : K_FRAMES;
    
    for ( uint32_t i = 0; i < limit; i++ ) {
        auto &t = telemetry_log[ i ];

        if ( t.frame_id == 0 )
            continue;
    
        fprintf( f, "%u;%u;%u;%s;%.3f;%.3f;%.3f;%.6f;%.6f;%.6f;%.3f;%u;%u;%u;%u;%u;%u;%.2f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%u;%u;%u\n", t.frame_id, t.status, t.current_skip, t.event, t.yaw, t.pitch, t.zoom, t.camera_send_timestamp, t.recv_start_timestamp, t.node_exit_timestamp, t.clock_offset_ms, t.original_points, t.rx_points, t.tx_points, t.rx_packets, t.tx_packets, t.payload_bytes, t.data_integrity_pct, t.internal_throughput_mbs, t.network_bitrate_mbps, t.conversion_ms, t.projection_ms, t.tx_duration_ms, t.active_process_ms, t.total_processing_ms, t.total_residency_ms, t.node_efficiency_pct, t.gpu_transfer_ms, t.gpu_kernel_ms, t.gpu_packing_ms, t.gpu_copyback_ms, t.host_overhead_ms, t.camera_to_node_latency_ms, t.end_to_end_latency_ms, t.schedule_delay_ms, t.network_jitter_ms, t.wait_raw_queue_ms, t.wait_render_queue_ms, t.encode_h265_ms, t.mpeg_bytes_generated, t.tx_retries, t.mbuf_starvation );
    }

    fclose( f );
    printf( "[SYSTEM] Metrics successfully exported to: \"%s\".\n", TELEMETRY_PATH );
}

static inline void flush_tx_burst( struct rte_mbuf **tx_bufs, int *burst_idx, uint32_t *tx_packets, uint32_t *tx_retries ) {
    if ( *burst_idx == 0 ) 
        return;

    uint16_t sent = 0;
    uint16_t retries = 0;

    const uint16_t pause_window = BURST_SIZE * 0.5;

    while ( sent < *burst_idx ) {
        uint16_t nb_tx = rte_eth_tx_burst( PORT_SFF2, 0, &tx_bufs[ sent ], *burst_idx - sent );

        if ( nb_tx > 0 ) {
            if ( tx_packets )
                *tx_packets += nb_tx;

            sent += nb_tx;
            retries = 0;
        }
        else {
            if ( tx_retries ) 
                ( *tx_retries )++;

            if ( ++retries > MAX_RETRIES ) {
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

static inline void begin_mpeg_frame( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {
    if ( current_out_frame_id > 0 && *burst_idx > 0 ) {
        uint32_t old_idx = ( current_out_frame_id - 1 ) % K_FRAMES;

        flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ old_idx ].tx_packets, &telemetry_log[ old_idx ].tx_retries );
    }

    if ( encode_timer_queue.empty() )
        return;

    EncodeTimerEntry encode_timer_entry = encode_timer_queue.front();
    encode_timer_queue.pop();

    current_out_frame_id = encode_timer_entry.frame_id;
    current_mpeg_packet_id = 0;

    uint64_t t_comp_end = rte_get_timer_cycles();

    uint32_t idx = ( current_out_frame_id - 1 ) % K_FRAMES;

    if ( telemetry_log[ idx ].encode_h265_ms == 0.0 )
        telemetry_log[ idx ].encode_h265_ms = ( ( double )( t_comp_end - encode_timer_entry.start_cycles ) / timer_hz ) * 1000.0;
}

static inline void poll_network_rx() {

    // Purpose: It executes burst-oriented polling on the "DPDK" ingress port to capture incoming network traffic. 
    //          The function also performs line-rate packet parsing, discriminates between control-plane commands ( "Feedback" ) & data-plane volumetric payloads, dynamically aggregating fragmented point cloud packets into coherent, frame-level memory structures ready for downstream 3D projection

    struct rte_mbuf *bufs[ BURST_SIZE ];
    uint16_t nb_rx;

    while ( 1 ) {
        nb_rx = rte_eth_rx_burst( PORT_SFF2, 0, bufs, BURST_SIZE );

        if ( nb_rx == 0 )
            break;

        for ( int i = 0; i < nb_rx; i++ ) {
            struct rte_mbuf *m = bufs[ i ];
            
            if ( unlikely( m -> data_len < outer_len + sizeof( struct nsh_hdr ) ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }
            
            struct rte_ether_hdr *eth = rte_pktmbuf_mtod( m, struct rte_ether_hdr * );
            
            if ( unlikely( eth -> ether_type != rte_cpu_to_be_16( RTE_ETHER_TYPE_IPV4 ) ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }
            
            struct rte_ipv4_hdr *ipv4 = ( struct rte_ipv4_hdr * )( eth + 1 );
            
            if ( unlikely( ipv4 -> next_proto_id != IPPROTO_UDP ) ) { 
                rte_pktmbuf_free( m ); 
                continue; 
            }
            
            struct rte_udp_hdr *udp = ( struct rte_udp_hdr * )( ipv4 + 1 );
            
            if ( unlikely( udp -> dst_port != rte_cpu_to_be_16( UDP_PORT ) ) ) {
                rte_pktmbuf_free( m ); 
                continue; 
            }

            struct nsh_hdr *nsh = ( struct nsh_hdr * )( udp + 1 );
            uint32_t sph = ntohl( nsh -> serv_path_hdr );
            uint32_t spi = sph >> 8; 
            uint8_t si = sph & 0xFF;

            if ( spi == FEEDBACK_SPI && si == 254 ) {
                struct feedback_payload *fb = ( struct feedback_payload * )( nsh + 1 );

                uint16_t requested_skip = ntohs( fb -> skip );
                float requested_yaw = fb -> yaw; 
                float requested_pitch = fb -> pitch; 
                float requested_zoom = fb -> zoom;

                if ( requested_skip != current_skip || requested_yaw != current_yaw || requested_pitch != current_pitch || requested_zoom != current_zoom ) {
                    if ( requested_skip == 0 && requested_yaw == 0.0f && requested_pitch == 0.0f && requested_zoom == 1.0f ) {
                        snprintf( current_event, sizeof( current_event ), "RESET" );
                        printf( "[SYSTEM] Both pose & temporal controller reverted to baseline ( Skip: %u, Yaw: %.1f, Pitch: %.1f, Zoom: %.1f ).\n", requested_skip, requested_yaw, requested_pitch, requested_zoom );
                    } 
                    else {
                        if ( requested_yaw != current_yaw ) snprintf( current_event, sizeof( current_event ), "Yaw = %.1f", requested_yaw );
                        else if ( requested_pitch != current_pitch ) snprintf( current_event, sizeof( current_event ), "Pitch = %.1f", requested_pitch );
                        else if ( requested_zoom != current_zoom ) snprintf( current_event, sizeof( current_event ), "Zoom = %.1f", requested_zoom );
                        else if ( requested_skip != current_skip ) snprintf( current_event, sizeof( current_event ), "Skip = %u", requested_skip );

                        printf( "[SYSTEM] Both pose & temporal controller updated to ( Skip: %u, Yaw: %.1f, Pitch: %.1f, Zoom: %.1f ).\n", requested_skip, requested_yaw, requested_pitch, requested_zoom );
                    }

                    current_skip = requested_skip; 
                    current_yaw = requested_yaw; 
                    current_pitch = requested_pitch; 
                    current_zoom = requested_zoom;
                }

                nsh -> serv_path_hdr = htonl( ( FEEDBACK_SPI << 8 ) | 253 );
                
                rte_ether_addr tmp = eth -> src_addr; 
                eth -> src_addr = eth -> dst_addr; 
                eth -> dst_addr = tmp;

                ipv4 -> src_addr = htonl( RTE_IPV4( 10, 0, 3, 2 ) ); 
                ipv4 -> dst_addr = htonl( RTE_IPV4( 10, 0, 3, 1 ) );
                ipv4 -> hdr_checksum = 0;
                ipv4 -> hdr_checksum = rte_ipv4_cksum( ipv4 );

                udp -> dgram_cksum = 0;

                struct rte_mbuf *feedback_tx_bufs[ 1 ] = { m };
                int feedback_burst_idx = 1;

                flush_tx_burst( feedback_tx_bufs, &feedback_burst_idx, NULL, NULL );

                continue;
            }

            if ( spi == PRIMARY_SPI && si == 254 ) {
                size_t hdr_size = outer_len + sizeof( struct nsh_hdr ) + sizeof( struct int_hdr ) + sizeof( struct cam_hdr );

                if ( unlikely( m -> data_len < hdr_size ) ) {
                    rte_pktmbuf_free( m );
                    continue;
                }

                struct int_hdr *meta = ( struct int_hdr * )( nsh + 1 );
                struct cam_hdr *cam = ( struct cam_hdr * )( meta + 1 );
                uint32_t f_id = ntohl( cam -> frame_id );
                uint64_t packet_arrival = rte_get_timer_cycles();

                if ( f_id == END_OF_STREAM )
                    eos_received = true;
                else {
                    if ( f_id <= last_processed_frame_id ) {
                        rte_pktmbuf_free( m );
                        continue;
                    }

                    size_t point_payload_size = m -> data_len - hdr_size;

                    if ( unlikely( point_payload_size % sizeof( struct point_tx ) != 0 ) ) { 
                        rte_pktmbuf_free( m ); 
                        continue; 
                    }

                    FrameBuffer &fb = frame_buffers[ f_id ];

                    if ( fb.rx_packets == 0 ) {
                        fb.cam = *cam;

                        fb.original_points = ntohl( cam -> original_points );
                        fb.camera_tx = rte_be_to_cpu_64( cam -> timestamp );
                        fb.first_arrival = packet_arrival;
                        fb.payload_bytes = sizeof( struct cam_hdr );

                        fb.x.reserve( fb.original_points ); fb.y.reserve( fb.original_points ); fb.z.reserve( fb.original_points );
                        fb.r.reserve( fb.original_points ); fb.g.reserve( fb.original_points ); fb.b.reserve( fb.original_points );
                    }

                    fb.last_arrival = packet_arrival;

                    uint32_t incoming_active_points = ntohl( meta -> active_point_count );
                    uint32_t stored_active_points = ( fb.rx_packets == 0 ) ? 0 : ntohl( fb.meta.active_point_count );

                    if ( fb.rx_packets == 0 || incoming_active_points >= stored_active_points )
                        fb.meta = *meta;

                    struct point_tx *pts = ( struct point_tx * )( ( char * )m -> buf_addr + m -> data_off + hdr_size );
                    uint32_t p_count = point_payload_size / sizeof( struct point_tx );

                    fb.payload_bytes += point_payload_size;
                    fb.rx_packets++;

                    uint64_t t_conv_start = rte_get_timer_cycles();

                    for ( uint32_t p = 0; p < p_count; p++ ) {
                        fb.x.push_back( pts[ p ].x ); fb.y.push_back( pts[ p ].y ); fb.z.push_back( pts[ p ].z );
                        fb.r.push_back( pts[ p ].r ); fb.g.push_back( pts[ p ].g ); fb.b.push_back( pts[ p ].b );
                    }

                    uint64_t t_conv_end = rte_get_timer_cycles();

                    fb.conversion_cycles += t_conv_end - t_conv_start;
                    fb.frame_ready = t_conv_end;
                }
            }

            rte_pktmbuf_free( m );
        }
    }
}

static inline void emit_mpeg_payload( const uint8_t *mpeg_data, uint16_t mpeg_len, struct rte_mbuf **tx_bufs, int *burst_idx ) {
    if ( current_out_frame_id == 0 || mpeg_len == 0 )
        return;

    uint32_t idx = ( current_out_frame_id - 1 ) % K_FRAMES;

    telemetry_log[ idx ].mpeg_bytes_generated += mpeg_len;

    struct rte_mbuf *m_out = rte_pktmbuf_alloc( mbuf_pool );

    if ( m_out == NULL ) {
        telemetry_log[ idx ].mbuf_starvation++;
        return;
    }

    size_t headers_len = outer_len + sizeof( struct nsh_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr );

    char *data = ( char * )rte_pktmbuf_append( m_out, headers_len + mpeg_len );

    if ( data == NULL ) {
        rte_pktmbuf_free( m_out );
        telemetry_log[ idx ].mbuf_starvation++;
        return;
    }

    struct rte_ether_hdr *eth_out = ( struct rte_ether_hdr * )data;
    struct rte_ipv4_hdr *ipv4_out = ( struct rte_ipv4_hdr * )( eth_out + 1 );
    struct rte_udp_hdr *udp_out = ( struct rte_udp_hdr * )( ipv4_out + 1 );
    struct nsh_hdr *nsh_out = ( struct nsh_hdr * )( udp_out + 1 );

    uint16_t udp_payload_len = sizeof( struct nsh_hdr ) + sizeof( struct cam_hdr ) + sizeof( struct enc_hdr ) + mpeg_len;

    header_init( eth_out, ipv4_out, udp_out, nsh_out, udp_payload_len, 253 );

    uint8_t *payload_ptr = ( uint8_t * )( nsh_out + 1 );

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
        flush_tx_burst( tx_bufs, burst_idx, &telemetry_log[ idx ].tx_packets, &telemetry_log[ idx ].tx_retries );
}

static inline void process_mpeg_bytes( const uint8_t *data, size_t data_len, struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {
    ts_pending.insert( ts_pending.end(), data, data + data_len );

    size_t consumed = 0;

    while ( ts_pending.size() - consumed >= TS_PACKET_SIZE ) {
        const uint8_t *ts = ts_pending.data() + consumed;

        if ( ts[ 0 ] != 0x47 ) {
            consumed++;
            continue;
        }

        if ( ts_starts_video_pes( ts ) ) {
            if ( current_out_frame_id > 0 && !mpeg_chunk.empty() ) {
                emit_mpeg_payload( mpeg_chunk.data(), ( uint16_t )mpeg_chunk.size(), tx_bufs, burst_idx );
                mpeg_chunk.clear();
            }

            begin_mpeg_frame( tx_bufs, burst_idx, timer_hz );
        }

        mpeg_chunk.insert( mpeg_chunk.end(), ts, ts + TS_PACKET_SIZE );
        consumed += TS_PACKET_SIZE;

        while ( current_out_frame_id > 0 && mpeg_chunk.size() >= MTU_PAYLOAD_SIZE ) {
            emit_mpeg_payload( mpeg_chunk.data(), MTU_PAYLOAD_SIZE, tx_bufs, burst_idx );

            mpeg_chunk.erase( mpeg_chunk.begin(), mpeg_chunk.begin() + MTU_PAYLOAD_SIZE );
        }
    }

    if ( consumed > 0 )
        ts_pending.erase( ts_pending.begin(), ts_pending.begin() + consumed );
}

static inline void drain_ffmpeg( struct rte_mbuf **tx_bufs, int *burst_idx, uint64_t timer_hz ) {
    
    // Purpose: It asynchronously retrieves the hardware-encoded "MPEG-TS" stream from the "FFmpeg" output pipe without blocking the main execution context. 
    //          Such module encapsulates the compressed video payload into standard headers, subsequently flushing the "mbufs" to the egress interface to maintain real-time streaming constraints

    while ( 1 ) {
        uint8_t read_buffer[ MTU_PAYLOAD_SIZE ];
        int bytes_read = read( ffmpeg_out[ 0 ], read_buffer, MTU_PAYLOAD_SIZE );

        if ( bytes_read > 0 )
            process_mpeg_bytes( read_buffer, bytes_read, tx_bufs, burst_idx, timer_hz );

        else if ( bytes_read < 0 && ( errno == EAGAIN || errno == EWOULDBLOCK ) ) {
            if ( current_out_frame_id > 0 && !mpeg_chunk.empty() ) {
                emit_mpeg_payload( mpeg_chunk.data(), ( uint16_t )mpeg_chunk.size(), tx_bufs, burst_idx );

                mpeg_chunk.clear();
            }

            rte_pause();
            break;
        }
        else
            break;
    }
}

static int worker_loop( __rte_unused void *arg ) {
    uint64_t timer_hz = rte_get_timer_hz();
    struct rte_mbuf *tx_bufs[ BURST_SIZE ];
    int burst_idx = 0;

    std::vector< uint8_t > yuv_buffer( TOTAL_YUV_SIZE );

    cuda_memory_register( yuv_buffer.data(), TOTAL_YUV_SIZE );
    
    uint64_t t_session_start = 0; 
    uint32_t frames_received = 0;
    bool is_first_frame = true;
    double global_clock_offset_sec = 0.0;
    uint64_t prev_arrival_cyc = 0;

    uint32_t first_arrival_f_id = 0;
   
    printf( "\n[SYSTEM] Conversion is about to begin at %.1f FPS...\n\n", ( double )TARGET_FPS / ( current_skip > 0 ? current_skip : 1 ) );

    while ( 1 ) {
        poll_network_rx();
        drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );

        uint32_t frame_to_process = 0;
        uint32_t highest_seen_f_id = 0;

        for ( auto it = frame_buffers.begin(); it != frame_buffers.end(); ++it ) 
            if ( it -> first > highest_seen_f_id )
                highest_seen_f_id = it -> first;
        
        for ( auto it = frame_buffers.begin(); it != frame_buffers.end(); ++it ) {
            uint32_t f_id = it -> first;
            
            if ( f_id <= last_processed_frame_id ) 
                continue;

            FrameBuffer &candidate = it -> second;

            uint16_t active_skip = ntohs( candidate.cam.temporal_skip );
            
            if ( active_skip == 0 ) 
                active_skip = 1;

            uint64_t timeout_cycles = ( timer_hz / ( TARGET_FPS / active_skip ) ); 

            bool is_complete = ( candidate.x.size() == candidate.original_points );
            bool is_outdated = ( candidate.last_arrival > 0 && ( rte_get_timer_cycles() - candidate.last_arrival ) > timeout_cycles );
            bool is_superseded = ( highest_seen_f_id > f_id );

            if ( is_complete || ( is_outdated && ( is_superseded || eos_received ) ) ){
                frame_to_process = f_id;
                break; 
            }
        }

        if ( frame_to_process > 0 ) {
            FrameBuffer &fb = frame_buffers[ frame_to_process ];
            
            uint32_t idx = ( frame_to_process - 1 ) % K_FRAMES;
            uint16_t active_temporal_skip = ntohs( fb.cam.temporal_skip );
            
            if ( active_temporal_skip == 0 ) 
                active_temporal_skip = 1;

            if ( fb.x.size() > 0 ) {
                uint64_t t_proj_start = rte_get_timer_cycles();
                uint32_t active_pts = ntohl( fb.meta.active_point_count );

                bool int_geometry_valid = active_pts > 0 && std::isfinite( fb.meta.sum_x ) && std::isfinite( fb.meta.sum_y ) && std::isfinite( fb.meta.sum_z ) && std::isfinite( fb.meta.min_x ) && std::isfinite( fb.meta.min_y ) &&  std::isfinite( fb.meta.min_z ) && std::isfinite( fb.meta.max_x ) && std::isfinite( fb.meta.max_y ) && std::isfinite( fb.meta.max_z ) && fb.meta.min_x <= fb.meta.max_x && fb.meta.min_y <= fb.meta.max_y && fb.meta.min_z <= fb.meta.max_z;

                if ( !int_geometry_valid ) {
                    telemetry_log[ idx ].frame_id = frame_to_process;
                    telemetry_log[ idx ].status = 0;

                    last_processed_frame_id = frame_to_process;
                    frame_buffers.erase( frame_to_process );

                    continue;
                }
                
                float c_x = fb.meta.sum_x / active_pts; 
                float c_y = fb.meta.sum_y / active_pts; 
                float c_z = fb.meta.sum_z / active_pts;
                float ext_x = fb.meta.max_x - fb.meta.min_x; 
                float ext_y = fb.meta.max_y - fb.meta.min_y; 
                float ext_z = fb.meta.max_z - fb.meta.min_z;

                float max_r2 = 0.0f;

                for ( size_t p = 0; p < fb.x.size(); p++ ) {
                    if ( p % CHUNKING_SIZE == 0 )
                        poll_network_rx();

                    float dx = fb.x[ p ] - c_x;
                    float dy = fb.y[ p ] - c_y;
                    float dz = fb.z[ p ] - c_z;

                    float r2 = dx * dx + dy * dy + dz * dz;

                    if ( r2 > max_r2 )
                        max_r2 = r2;
                }

                float max_r = std::sqrt( max_r2 );

                float frame_yaw = current_yaw;
                float frame_pitch = current_pitch;
                float frame_zoom = current_zoom;
                char frame_event[ 16 ];

                snprintf( frame_event, sizeof( frame_event ), "%s", current_event );

                float target_radius = CAMERA_DIST * 0.2f;

                float final_scale = ( max_r > 0.0f ) ? ( target_radius / max_r ) * frame_zoom : 1.0f;

                float global_scale = 1.0f;
                float bbox_center_x = 0.0f;
                float bbox_center_y = 0.0f;
                float bbox_center_z = 0.0f;
                uint64_t projection_end_cycles = 0;
                uint64_t encode_start_cycles = 0;

                double gpu_metrics[ 4 ] = { 0.0 };

                run_projection_pipeline( fb.x.data(), fb.y.data(), fb.z.data(), fb.r.data(), fb.g.data(), fb.b.data(), ( uint32_t )fb.x.size(), c_x, c_y, c_z, ext_x, ext_y, ext_z, final_scale, frame_yaw, frame_pitch, CAMERA_DIST, yuv_buffer.data(), gpu_metrics, &global_scale, &bbox_center_x, &bbox_center_y, &bbox_center_z, &projection_end_cycles, &encode_start_cycles, poll_network_rx );
                
                uint64_t t_proj_end = projection_end_cycles;
                
                encoder_metadata[ idx ].frame_id = htonl( frame_to_process );
                encoder_metadata[ idx ].final_scale = final_scale;
                encoder_metadata[ idx ].global_scale = global_scale;
                encoder_metadata[ idx ].box_center_x = bbox_center_x;
                encoder_metadata[ idx ].box_center_y = bbox_center_y;
                encoder_metadata[ idx ].box_center_z = bbox_center_z;
                encoder_metadata[ idx ].yaw = frame_yaw;
                encoder_metadata[ idx ].pitch = frame_pitch;
                encoder_metadata[ idx ].centroid_x = c_x; 
                encoder_metadata[ idx ].centroid_y = c_y; 
                encoder_metadata[ idx ].centroid_z = c_z;
                
                camera_metadata[ idx ] = fb.cam;

                telemetry_log[ idx ].frame_id = frame_to_process; 
                telemetry_log[ idx ].status = 1; 
                telemetry_log[ idx ].current_skip = active_temporal_skip;
                telemetry_log[ idx ].camera_send_timestamp = ( double )fb.camera_tx / timer_hz;
                
                snprintf( telemetry_log[ idx ].event, sizeof( telemetry_log[ idx ].event ), "%s", frame_event );
                
                if ( strcmp( current_event, frame_event ) == 0 && strcmp( current_event, "Idle" ) != 0 ) 
                    snprintf( current_event, sizeof( current_event ), "Idle" );
                
                telemetry_log[ idx ].yaw = frame_yaw;
                telemetry_log[ idx ].pitch = frame_pitch;
                telemetry_log[ idx ].zoom = frame_zoom;

                telemetry_log[ idx ].original_points = fb.original_points;
                telemetry_log[ idx ].rx_points = fb.x.size();
                telemetry_log[ idx ].tx_points = fb.x.size(); 
                telemetry_log[ idx ].rx_packets = fb.rx_packets;
                telemetry_log[ idx ].payload_bytes = fb.payload_bytes;
                telemetry_log[ idx ].data_integrity_pct = ( fb.original_points > 0 ) ? ( ( double )telemetry_log[ idx ].rx_points / ( double )fb.original_points ) * 100.0 : 0.0;

                telemetry_log[ idx ].camera_send_timestamp = ( double )fb.camera_tx / timer_hz;
                telemetry_log[ idx ].recv_start_timestamp = ( double )fb.first_arrival / timer_hz;

                if ( is_first_frame ) {
                    global_clock_offset_sec = ( ( double )fb.first_arrival / timer_hz ) - ( ( double )fb.camera_tx / timer_hz );
                    is_first_frame = false;
                }

                telemetry_log[ idx ].clock_offset_ms = global_clock_offset_sec * 1000.0;

                double current_latency_ms = ( ( ( double )fb.first_arrival / timer_hz ) - ( ( double )fb.camera_tx / timer_hz ) - global_clock_offset_sec ) * 1000.0;

                telemetry_log[ idx ].camera_to_node_latency_ms = current_latency_ms;

                if ( prev_arrival_cyc > 0 ) {
                    double intervallo_reale_sec = ( double )( fb.first_arrival - prev_arrival_cyc ) / timer_hz;
                    double expected_interval_sec = ( 1.0 / TARGET_FPS ) * active_temporal_skip;

                    telemetry_log[ idx ].network_jitter_ms = std::abs( intervallo_reale_sec - expected_interval_sec ) * 1000.0;
                } 
                else 
                    telemetry_log[ idx ].network_jitter_ms = 0.0;

                prev_arrival_cyc = fb.first_arrival;

                double duration_recv_sec = ( fb.last_arrival >= fb.first_arrival ) ? ( double )( fb.last_arrival - fb.first_arrival ) / timer_hz : 0.0;
                telemetry_log[ idx ].internal_throughput_mbs = ( duration_recv_sec > 0 ) ? ( fb.payload_bytes / 1000000.0 ) / duration_recv_sec : 0.0;
                telemetry_log[ idx ].network_bitrate_mbps = ( fb.payload_bytes * 8.0 * ( TARGET_FPS / active_temporal_skip ) ) / 1000000.0;

                double projection_ms = ( ( double )( t_proj_end - t_proj_start ) / timer_hz ) * 1000.0;
                telemetry_log[ idx ].projection_ms = projection_ms;

                telemetry_log[ idx ].conversion_ms =  ( ( double )fb.conversion_cycles / timer_hz ) * 1000.0;
                telemetry_log[ idx ].wait_raw_queue_ms = ( fb.frame_ready > 0 && t_proj_start >= fb.frame_ready ) ? ( ( double )( t_proj_start - fb.frame_ready ) / timer_hz ) * 1000.0 : 0.0; 

                telemetry_log[ idx ].gpu_transfer_ms = gpu_metrics[ 0 ]; 
                telemetry_log[ idx ].gpu_kernel_ms = gpu_metrics[ 1 ];
                telemetry_log[ idx ].gpu_packing_ms = gpu_metrics[ 2 ]; 
                telemetry_log[ idx ].gpu_copyback_ms = gpu_metrics[ 3 ];

                double host_overhead = projection_ms - ( gpu_metrics[ 0 ] + gpu_metrics[ 1 ] + gpu_metrics[ 2 ] + gpu_metrics[ 3 ] );
                telemetry_log[ idx ].host_overhead_ms = ( host_overhead > 0 ) ? host_overhead : 0.0;

                if ( frames_received == 0 ) {
                    t_session_start = fb.first_arrival;
                    first_arrival_f_id = frame_to_process;
                }

                uint32_t frame_offset = frame_to_process - first_arrival_f_id;

                frames_received++;

                uint64_t t_send_metric_start = rte_get_timer_cycles();

                size_t yuv_written = 0;

                EncodeTimerEntry encode_timer_entry;
                encode_timer_entry.frame_id = frame_to_process;
                encode_timer_entry.start_cycles = t_send_metric_start;

                encode_timer_queue.push( encode_timer_entry ); 

                while ( yuv_written < TOTAL_YUV_SIZE ) {
                    poll_network_rx();

                    drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );

                    int ret_w = write( ffmpeg_in[ 1 ], yuv_buffer.data() + yuv_written, TOTAL_YUV_SIZE - yuv_written );

                    if ( ret_w > 0 ) yuv_written += ret_w;
                    else if ( ret_w < 0 && ( errno == EAGAIN || errno == EWOULDBLOCK ) ) 
                        rte_pause();
                    else if ( ret_w < 0 && errno == EPIPE )
                        break;
                }

                uint64_t t_send_metric_end = rte_get_timer_cycles();
                uint64_t exit_cyc = t_send_metric_end;

                double send_ms = ( ( double )( t_send_metric_end - t_send_metric_start ) / timer_hz ) * 1000.0;

                telemetry_log[ idx ].wait_render_queue_ms = ( t_send_metric_start >= t_proj_end ) ? ( ( double )( t_send_metric_start - t_proj_end ) / timer_hz ) * 1000.0 : 0.0;

                double real_elapsed_sec = ( double )( exit_cyc - t_session_start ) / timer_hz;
                double ideal_elapsed_sec = ( double )frame_offset / TARGET_FPS;

                telemetry_log[ idx ].schedule_delay_ms = ( real_elapsed_sec - ideal_elapsed_sec ) * 1000.0;

                telemetry_log[ idx ].node_exit_timestamp = ( double )exit_cyc / timer_hz;
                telemetry_log[ idx ].tx_duration_ms = send_ms;

                double residency_sec = ( double )( exit_cyc - fb.first_arrival ) / timer_hz;

                telemetry_log[ idx ].total_residency_ms = residency_sec * 1000.0;
                telemetry_log[ idx ].total_processing_ms = telemetry_log[ idx ].conversion_ms + telemetry_log[ idx ].projection_ms + telemetry_log[ idx ].tx_duration_ms;
                telemetry_log[ idx ].active_process_ms = telemetry_log[ idx ].total_processing_ms;
                telemetry_log[ idx ].node_efficiency_pct = ( residency_sec > 0 ) ? ( telemetry_log[ idx ].active_process_ms / telemetry_log[ idx ].total_residency_ms ) * 100.0 : 0.0;

                if ( telemetry_log[ idx ].camera_send_timestamp > 0 ) 
                    telemetry_log[ idx ].end_to_end_latency_ms = ( telemetry_log[ idx ].node_exit_timestamp - telemetry_log[ idx ].camera_send_timestamp - global_clock_offset_sec ) * 1000.0;

                drain_ffmpeg( tx_bufs, &burst_idx, timer_hz );
                
            }

            last_processed_frame_id = frame_to_process;

            frame_buffers.erase( frame_to_process );
        }

        if ( eos_received && frame_buffers.empty() ) {
            if ( !csv_written ) {
                close( ffmpeg_in[ 1 ] );

                int ffmpeg_flags = fcntl( ffmpeg_out[ 0 ], F_GETFL, 0 );

                if ( ffmpeg_flags >= 0 )
                    fcntl( ffmpeg_out[ 0 ], F_SETFL, ffmpeg_flags & ~O_NONBLOCK );

                while ( 1 ) {
                    uint8_t mpeg_buffer[ MTU_PAYLOAD_SIZE ];
                    int bytes_read = read( ffmpeg_out[ 0 ], mpeg_buffer, MTU_PAYLOAD_SIZE );
                    
                    if ( bytes_read > 0 ) 
                        process_mpeg_bytes( mpeg_buffer, bytes_read, tx_bufs, &burst_idx, timer_hz );
                    else if ( bytes_read == 0 )
                        break;
                    else if ( errno == EINTR )
                        continue;
                    else 
                        break;
                }

                if ( current_out_frame_id > 0 && !mpeg_chunk.empty() ) {
                    emit_mpeg_payload( mpeg_chunk.data(), ( uint16_t )mpeg_chunk.size(), tx_bufs, &burst_idx );

                    mpeg_chunk.clear();
                }

                if ( !ts_pending.empty() )
                    ts_pending.clear();

                if ( burst_idx > 0 ) {
                    if ( current_out_frame_id > 0 ) {
                        uint32_t retry_idx = ( current_out_frame_id - 1 ) % K_FRAMES;

                        flush_tx_burst( tx_bufs, &burst_idx, &telemetry_log[ retry_idx ].tx_packets, &telemetry_log[ retry_idx ].tx_retries );
                    }
                    else
                        flush_tx_burst( tx_bufs, &burst_idx, NULL, NULL );
                }
                
                close( ffmpeg_out[ 0 ] );
                waitpid( ffmpeg_pid, NULL, 0 );

                struct rte_mbuf *m_eos = rte_pktmbuf_alloc( mbuf_pool );

                if ( m_eos ) {
                    size_t headers_len = outer_len + sizeof( struct nsh_hdr ) + sizeof( struct cam_hdr );
                    
                    char *data = ( char * )rte_pktmbuf_append( m_eos, headers_len );

                    if ( data != NULL ) {
                        struct rte_ether_hdr *eth_out = ( struct rte_ether_hdr * )data; 
                        struct rte_ipv4_hdr *ipv4_out = ( struct rte_ipv4_hdr * )( eth_out + 1 );
                        struct rte_udp_hdr *udp_out = ( struct rte_udp_hdr * )( ipv4_out + 1 ); 
                        struct nsh_hdr *nsh_out = ( struct nsh_hdr * )( udp_out + 1 );

                        struct cam_hdr *cam_out = ( struct cam_hdr * )( nsh_out + 1 );

                        uint16_t udp_payload_len = sizeof( struct nsh_hdr ) + sizeof( struct cam_hdr );

                        header_init( eth_out, ipv4_out, udp_out, nsh_out, udp_payload_len, 253 );

                        memset( cam_out, 0, sizeof( struct cam_hdr ) );
                        cam_out -> frame_id = htonl( END_OF_STREAM );

                        tx_bufs[ burst_idx++ ] = m_eos;
                        flush_tx_burst( tx_bufs, &burst_idx, NULL, NULL );
                    }
                    else
                        rte_pktmbuf_free( m_eos );
                }

                telemetry_to_csv();
                csv_written = true;
                printf( "\n[SYSTEM] End of stream detected. Changing to \"idle\" plight...\n" );
            }
        }
    }

    cuda_memory_unleash( yuv_buffer.data() );

    return 0;
}

int main( int argc, char *argv[] ) {

    // Purpose: It orchestrates the core run-to-completion pipeline. 
    //          The main loop continuously evaluates the state of the aggregated point clouds, enforcing strict temporal deadlines to drop outdated geometries.
    //          Correct frames are dispatched to the "CUDA" subsystem for spatial projection, while the resulting rendering metrics & hardware telemetry are meticulously logged to estimate the node's overall efficiency

    setvbuf( stdout, NULL, _IONBF, 0 );

    int ret = rte_eal_init( argc, argv );

    if ( ret < 0 ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: \"EAL\" initialization failed...\n");

    printf( "[SYSTEM] Booting the \"Encoder\" microservice...\n" );

    mbuf_pool = rte_pktmbuf_pool_create( "MBUF_POOL", NUM_MBUFS, MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id() );
    
    if ( mbuf_pool == NULL ) 
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Memory pool allocation failed...\n" );

    if ( port_init( PORT_SFF2, mbuf_pool ) != 0 )
        rte_exit( EXIT_FAILURE, "[SYSTEM] Error: Virtual port configuration failed...\n" );

    ffmpeg_init(); 
    cuda_memory_init( MAX_POINTS );
    cuda_memory_warmup();

    // Delegating functions to the assigned logical core
    uint32_t worker_lcore = rte_get_next_lcore( -1, 1, 0 );

    if ( worker_lcore == RTE_MAX_LCORE ) 
        worker_loop ( NULL );
    else {
        rte_eal_remote_launch( worker_loop, NULL, worker_lcore );
        rte_eal_mp_wait_lcore();
    }

    cuda_memory_free();
    rte_eal_cleanup();

    return 0;
}