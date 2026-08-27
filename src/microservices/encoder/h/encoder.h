#ifndef ENCODER_HEADER
#define ENCODER_HEADER

#include <stddef.h>
#include <stdint.h>

// Runtime & experimental configuration variables
#define TELEMETRY_FOLDER "/shared/log/encoder"
#define FFMPEG_PATH "/shared/log/encoder/ffmpeg.txt"
#define TELEMETRY_PATH "/shared/log/encoder/telemetry_encoder.csv"
#define STDERR_PATH "/shared/log/encoder/stderr.log"

#define REFERENCE_PATH "/shared/log/encoder/reference_y.raw"
#define ENCODED_PATH "/shared/log/encoder/encoded.ts"
#define PSNR_PATH "/shared/log/encoder/psnr.log"
#define SSIM_PATH "/shared/log/encoder/ssim.log"

#define READY_PATH "/tmp/sfc-encoder-ready"
#define PREROLL_PATH "/tmp/sfc-decoder-ready"
#define POSTROLL_PATH "/tmp/sfc-decoder-done"

#define K_FRAMES 300

#define TARGET_BITRATE_MBPS "10M"
#define TARGET_BUFFER_SIZE "20M"
#define TARGET_FPS 30.0

#define BURST_SIZE 32
#define MAX_ZERO_ACCEPTS 2048

#define END_OF_STREAM 0xFFFFFFFF
#define FRAMES 15
#define FRAME_ID ( END_OF_STREAM - 1 )

// "DPDK" packet-buffer pool settings
#define NUM_MBUFS 16383
#define MBUF_CACHE_SIZE 256

// Sending bonds & networking parameters
#define PORT_SFF2 0

#define SFF2_ENCODER_IP RTE_IPV4( 10, 0, 3, 254 )
#define ENCODER_IP RTE_IPV4( 10, 0, 3, 1 )
#define SFF2_ENCODER_PORT 6633
#define ENCODER_PORT 7001

// Packetization & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80
#define H2D_CHUNK_POINTS 65536
#define MAX_POINTS 835458
#define TS_PACKET_SIZE 188
#define MTU_PAYLOAD_SIZE ( 7 * TS_PACKET_SIZE ) // 7 * 188 = 1316 bytes. The largest complete "MPEG-TS" group fitting the current "MTU" envelope without packet tearing
#define FFMPEG_READ_SIZE ( MTU_PAYLOAD_SIZE - TS_PACKET_SIZE )

// Data-offload selection conditions
#define OFFLOAD_MODE_DISABLED 0
#define OFFLOAD_MODE_ENABLED 1

#define OFFLOAD_MODE OFFLOAD_MODE_ENABLED

// Workload-driven temporal controller
#define TEMPORAL_ADAPTATION_DISABLED 0 
#define TEMPORAL_ADAPTATION_ENABLED 1

// Such a model is defined as "T_base = 1000 / TARGET_FPS", "T_budget( skip ) = skip * T_base", "E_n = alpha * T_n + ( 1 - alpha ) * E_{ n - 1 }" applying an "Exponentially Weighted Moving Average" ( "EWMA" )
#define TEMPORAL_ADAPTATION TEMPORAL_ADAPTATION_ENABLED

#define MAX_SKIP 9
#define MIN_FRAMES 3
#define STABLE_STREAK 3
#define MAX_FRAMES 15
#define OVERLOAD_STREAK 2
#define RECOVERY_STREAK 9
#define RETRY_FRAMES 3

#define EWMA_ALPHA 0.25
#define OVERLOAD_RATIO 0.90
#define RECOVERY_RATIO 0.75
#define OVERLOAD_FRACTION 0.25
#define RECOVERY_FRACTION 0.10

#define YUV_BUFFER_COUNT 3
#define QUALITY_STREAM_SIZE ( 64 * 1024 * 1024 )
#define FFMPEG_CPU 7
#define GOP "15"
#define FORCED_IDR "1"

// Functional settings ( projection, "YUV" & "Atlas" dimensions, restricted to multiples of 64 for the selected "H.265" input format )
#define CAMERA_DISTANCE 1200.0f
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

// Wire-format structures utilized by the "DPDK" data path
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

    uint32_t active_point_count;
} __attribute__((__packed__));

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

// Encoder metadata appended to outgoing traffic for Decoder-side reconstruction. Upstream pose fields are retained for compatibility but remain static during the operational phase
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

// Host-side 16-byte point representation shared between local "CPU" geometry fallback & "CUDA" memory shift
struct host_point {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t padding;
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
    char event[ 16 ]; // "IDLE", "WARMUP", "SKIP+1", "SKIP-1", "RETRY" or "INVALID"
    float yaw, pitch, zoom;

    // Timing references
    double camera_send_timestamp;
    double recv_start_timestamp;
    double codec_exit_time;
    double node_exit_timestamp;
    
    // Load & integrity metrics
    uint32_t original_points;
    uint32_t rx_points;
    uint32_t processed_points;
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

    // Processing intervals
    double conversion_ms;
    double geometry_aggregation_ms;
    double max_r_ms;
    double projection_ms;
    double codec_write_ms;

    double active_tx_ms;
    double active_process_ms;

    double reference_process_ms;

    double total_processing_ms;
    double total_residency_ms;

    double reference_residency_ms;

    double node_efficiency_pct;

    double reference_efficiency_pct;

    // "CUDA" stage decomposition
    double gpu_transfer_ms;
    double gpu_kernel_ms;
    double gpu_packing_ms;
    double gpu_copyback_ms;
    double host_overhead_ms;

    // Network, schedule & queuing delays
    double camera_node_ms;
    double e2e_latency_ms;

    double reference_e2e_ms;

    double schedule_delay_ms;
    double instant_jitter_ms;
    double desynced_jitter_ms;

    double reference_jitter_ms;

    double raw_queue_ms;
    double render_queue_ms;

    double workload_ewma_ms; 
    double workload_ratio; 
    uint32_t frame_backlog;
    uint32_t codec_backlog;

    // "FFmpeg" & "codec" output indicators
    double encode_service_ms;
    double encode_h265_ms;
    double mse_y;
    double psnr_y;
    double ssim_y;
    uint32_t mpeg_bytes_generated;
    uint32_t ffmpeg_write_calls;
    uint32_t ffmpeg_write_eagain; 

    uint32_t tx_zero_accepts;
    uint32_t tx_partial_accepts;
    uint32_t tx_resubmit_calls;
    uint32_t tx_resubmitted_packets;

    uint32_t mbuf_starvation;
};

// "C++" to "CUDA" integration bridge
typedef void ( *process_callback_t )();

extern "C" void cuda_memory_init( uint32_t max_pts );
extern "C" void cuda_memory_free();
extern "C" void cuda_memory_warmup();

extern "C" void cuda_memory_register( void *ptr, size_t size );
extern "C" void cuda_memory_unleash( void *ptr );

extern "C" void run_projection_pipeline( const struct host_point *points, uint32_t num_pts, float centroid_x, float centroid_y, float centroid_z, float extent_x, float extent_y, float extent_z, float raw_bbox_center_x, float raw_bbox_center_y, float raw_bbox_center_z, float final_scale, float cam_dist, uint8_t *out_yuv_buffer, double *gpu_metrics, float *out_global_scale, float *out_bbox_center_x, float *out_bbox_center_y, float *out_bbox_center_z, uint64_t *out_projection_end_cycles, process_callback_t process_callback );

#endif