#ifndef ENCODER_HEADER
#define ENCODER_HEADER

#include <stddef.h>
#include <stdint.h>

// Configuration variables
#define TELEMETRY_FOLDER "/shared/log/encoder"
#define FFMPEG_PATH "/shared/log/encoder/ffmpeg.txt"
#define TELEMETRY_PATH "/shared/log/encoder/telemetry_encoder.csv"
#define STDERR_PATH "/shared/log/encoder/stderr.log"

#define K_FRAMES 300

#define TARGET_BITRATE_MBPS "10M"
#define TARGET_BUFFER_SIZE "20M"
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
#define PORT_SFF2 0
#define UDP_PORT 6633

// 1024 -> BEST, MIDDLE, WORST ( modes )
#define CHUNKING_SIZE 1024
#define MAX_POINTS 835458
#define TS_PACKET_SIZE 188
#define MTU_PAYLOAD_SIZE 7 * TS_PACKET_SIZE // exactly 7 "MPEG-TS" packets, 188 bytes each ( no tearing )

// Functional settings ( "YUV" & "Atlas", multiple of 64 for "H.265" )
#define CAMERA_DIST 1200.0f
#define WIDTH 640
#define HEIGHT 480

#define FACE_W_PADDED 640 
#define FACE_H_PADDED 512 
#define CROSS_W ( FACE_W_PADDED * 4 )
#define CROSS_H ( FACE_H_PADDED * 3 )

#define ENCODER_W CROSS_W
#define ENCODER_H ( CROSS_H * 3 )

#define SIZE_Y ( ENCODER_W * ENCODER_H )
#define SIZE_UV ( ( ENCODER_W / 2 ) * ( ENCODER_H / 2 ) )
#define TOTAL_YUV_SIZE ( SIZE_Y + ( 2 * SIZE_UV ) )

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
    float final_scale; // zoom * ( target_radius / max_r )
    float yaw, pitch; // radians
    float centroid_x, centroid_y, centroid_z; 
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
    char event[ 16 ]; // e.g., "IDLE"
    float yaw, pitch, zoom;

    // Timing references
    double camera_send_timestamp;
    double recv_start_timestamp;
    double node_exit_timestamp;
    double clock_offset_ms;

    // Load & integrity metrics
    uint32_t original_points;
    uint32_t rx_points;
    uint32_t tx_points;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t payload_bytes;
    double data_integrity_pct; // ( rx / original ) * 100
    double internal_throughput_mbs;
    double network_bitrate_mbps;

    // Processing intervals
    double conversion_ms;         
    double projection_ms;
    double tx_duration_ms; 
    double active_process_ms;
    double total_processing_ms;
    double total_residency_ms;
    double node_efficiency_pct;

    // Node latencies ( "CUDA" & pipeline )
    double gpu_transfer_ms;
    double gpu_kernel_ms;
    double gpu_packing_ms;
    double gpu_copyback_ms;
    double host_overhead_ms;

    // Network & queueing delay
    double camera_to_node_latency_ms;
    double end_to_end_latency_ms;
    double schedule_delay_ms;
    double network_jitter_ms;
    double wait_raw_queue_ms;
    double wait_render_queue_ms;
    
    // "FFmpeg" "Codec" indicators
    double encode_h265_ms;
    uint32_t mpeg_bytes_generated;

    // "DPDK" reliability
    uint32_t tx_retries;
    uint32_t mbuf_starvation;
};

// "C++" to "CUDA" bridge
typedef void ( *dpdk_poll_callback_t )();

extern "C" void cuda_memory_init( uint32_t max_pts );
extern "C" void cuda_memory_free();
extern "C" void cuda_memory_update( uint32_t required_pts );
extern "C" void cuda_memory_warmup();

extern "C" void cuda_memory_register( void* ptr, size_t size );
extern "C" void cuda_memory_unleash( void* ptr );

extern "C" void run_projection_pipeline( const float* pts_x, const float* pts_y, const float* pts_z, const uint8_t* c_r, const uint8_t* c_g, const uint8_t* c_b, uint32_t num_pts, float c_x, float c_y, float c_z, float extent_x, float extent_y, float extent_z, float final_scale, float yaw, float pitch, float cam_dist, uint8_t* out_yuv_buffer, double* gpu_metrics, float* out_global_scale, float* out_bbox_center_x, float* out_bbox_center_y, float* out_bbox_center_z, uint64_t* out_projection_end_cycles, uint64_t* out_encode_start_cycles, dpdk_poll_callback_t dpdk_poll_callback );

#endif