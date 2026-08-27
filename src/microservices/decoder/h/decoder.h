#ifndef DECODER_HEADER
#define DECODER_HEADER

#include <stddef.h>
#include <stdint.h>

// Runtime & experimental configuration variables
#define TELEMETRY_FOLDER "/shared/log/decoder"
#define FFMPEG_PATH "/shared/log/decoder/ffmpeg.txt"
#define TELEMETRY_PATH "/shared/log/decoder/telemetry_decoder.csv"
#define STDERR_PATH "/shared/log/decoder/stderr.log"

#define READY_PATH "/tmp/sfc-decoder-ready"
#define POSTROLL_PATH "/tmp/sfc-decoder-done"

#define K_FRAMES 300
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

#define SFF2_DECODER_IP RTE_IPV4( 10, 0, 4, 254 )
#define DECODER_IP RTE_IPV4( 10, 0, 4, 1 )
#define SFF2_DECODER_PORT 6633
#define DECODER_PORT 8001

// Packetisation & "Maximum Transmission Unit" ( "MTU" ) constraints
#define POINTS_PER_PACKET 80
#define TS_PACKET_SIZE 188
#define MTU_PAYLOAD_SIZE ( 7 * TS_PACKET_SIZE )
#define MAX_SOURCE_POINTS 835458

#define QUEUE_SIZE 16384
#define WRITE_BATCH_SIZE 65536
#define I420_BUFFER_COUNT 3
#define FFMPEG_CPU 2

#define DURATION "0"
#define PROBE_SIZE "32768"

// Functional settings ( reconstruction, "YUV" & "Atlas" dimensions, restricted to multiples of 64 for the selected "H.265" input format )
#define CAMERA_DISTANCE 1200.0f
#define WIDTH 640
#define HEIGHT 480

#define FACE_W_PADDED 640
#define FACE_H_PADDED 512
#define CROSS_W ( FACE_W_PADDED * 4 )
#define CROSS_H ( FACE_H_PADDED * 3 )

#define DECODER_W CROSS_W
#define DECODER_H ( CROSS_H * 3 )

#define SIZE_Y ( DECODER_W * DECODER_H )
#define SIZE_UV ( ( DECODER_W / 2 ) * ( DECODER_H / 2 ) )
#define TOTAL_YUV_SIZE ( SIZE_Y + ( 2 * SIZE_UV ) )
#define MAX_RECONSTRUCTED_POINTS ( 6 * WIDTH * HEIGHT )

// Wire-format structures utilized by the "DPDK" data path
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

struct host_point {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t padding;
};

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
    float yaw;
    float pitch;
    float zoom;

    double camera_send_timestamp;
    double recv_start_timestamp;
    double node_exit_timestamp;

    uint32_t original_points;
    uint32_t rx_media_bytes;
    uint32_t tx_points;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t payload_bytes;

    uint64_t reference_size_bytes;

    double data_integrity_pct;
    double internal_throughput_mbs;

    double reference_throughput_mbps;

    double logical_bitrate_mbps;
    double network_bitrate_mbps;

    double reference_bitrate_mbps;

    // Volumetric counters
    uint32_t arrived_points;
    uint32_t eroded_points;
    uint32_t valid_points;

    double erosion_ms;
    double reconstruction_ms;
    double pose_ms;
    double reconstruction_pipeline_ms;
    double tx_duration_ms;
    double active_tx_ms;

    double active_process_ms;

    double reference_process_ms;

    double total_processing_ms;
    double total_residency_ms;

    double reference_residency_ms;

    double node_efficiency_pct;

    double reference_efficiency_pct;

    double gpu_transfer_ms;
    double gpu_copyback_ms;
    double host_overhead_ms;

    double camera_node_ms;
    double e2e_latency_ms;
    double schedule_delay_ms;
    double instant_jitter_ms;
    double desynced_jitter_ms;
    double pose_control_ms;
    double codec_queue_ms;
    double frame_queue_ms;
    uint32_t codec_backlog;

    double decode_service_ms;
    double decode_h265_ms;
    uint32_t ffmpeg_write_calls;
    uint32_t ffmpeg_write_failures;
    uint32_t codec_queue_drops;

    uint32_t tx_zero_accepts;
    uint32_t tx_partial_accepts;
    uint32_t tx_resubmit_calls;
    uint32_t tx_resubmitted_packets;

    uint32_t mbuf_starvation;
};

// "C++" to "CUDA" integration bridge
typedef void ( *process_callback_t )();

extern "C" void cuda_memory_init();
extern "C" void cuda_memory_free();
extern "C" void cuda_memory_warmup();

extern "C" void cuda_memory_register( void *ptr, size_t size );
extern "C" void cuda_memory_unleash( void *ptr );

extern "C" void run_reconstruction_pipeline( const uint8_t *i420_frame, const struct enc_hdr *metadata, float decoder_yaw, float decoder_pitch, float decoder_zoom, struct host_point *out_points, uint32_t *out_arrived_points, uint32_t *out_eroded_points, uint32_t *out_valid_points, double *gpu_metrics, uint64_t *out_pose_apply_end_cycles, uint64_t *out_pipeline_end_cycles, process_callback_t process_callback );

#endif