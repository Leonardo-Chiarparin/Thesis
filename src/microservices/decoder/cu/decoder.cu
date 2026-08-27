#include "decoder.h"
#include <cuda_runtime.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <rte_cycles.h>
#include <rte_byteorder.h>

#define CHECK_CUDA( call ) { \
    cudaError_t err = call; \
    if ( err != cudaSuccess ) { \
        std::cerr << "\"CUDA\" Error: " << cudaGetErrorString( err ) << std::endl; \
    } \
}

struct rotation_matrix {
    float r00, r01, r02;
    float r10, r11, r12;
    float r20, r21, r22;
};

// Persistent device-side buffers eliminating component-local allocation & memory-handling overhead from the critical streaming path
static uint8_t *d_i420 = nullptr;
static uint8_t *d_occ_eroded = nullptr;
static struct host_point *d_points = nullptr;

static uint32_t *d_arrived_count = nullptr;
static uint32_t *d_eroded_count = nullptr;
static uint32_t *d_valid_count = nullptr;

static cudaStream_t reconstruction_stream = nullptr;

static cudaEvent_t pipeline_start_event = nullptr;
static cudaEvent_t h2d_done_event = nullptr;
static cudaEvent_t erosion_done_event = nullptr;
static cudaEvent_t reconstruction_done_event = nullptr;
static cudaEvent_t pose_done_event = nullptr;
static cudaEvent_t copyback_start_event = nullptr;
static cudaEvent_t d2h_done_event = nullptr;

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

static inline struct rotation_matrix rotation_from_pose( float yaw, float pitch ) {
    
    // Purpose: It constructs a 3x3 rigid transformation matrix from the specified "Euler" angles, establishing the spatial rotational mapping required for 3D coordinate alignment
    
    struct rotation_matrix matrix;

    float cp = cosf( pitch );
    float sp = sinf( pitch );
    float cy = cosf( yaw );
    float sy = sinf( yaw );

    matrix.r00 = cy;
    matrix.r01 = sy * sp;
    matrix.r02 = sy * cp;

    matrix.r10 = 0.0f;
    matrix.r11 = cp;
    matrix.r12 = -sp;

    matrix.r20 = -sy;
    matrix.r21 = cy * sp;
    matrix.r22 = cy * cp;

    return matrix;
}

__device__ static inline uint8_t read_neutral_border( const uint8_t *raw_occ, int x, int y ) {
    
    // Purpose: It safely samples spatial occupancy from the 2D planar projection, enforcing a neutral boundary condition ( yielding 255 ) to prevent out-of-bounds memory accesses during localized "CUDA" operations like erosion
    
    if ( x < 0 || x >= CROSS_W || y < 0 || y >= CROSS_H )
        return 255;

    return raw_occ[ y * CROSS_W + x ];
}

__global__ void erosion_2x2_kernel( const uint8_t *raw_occ, uint8_t *eroded_occ, uint32_t *arrived_count, uint32_t *eroded_count ) {
    
    // Purpose: It mitigates compression artifacts by suppressing isolated edge responses within the 2D occupancy mapping prior to the volumetric reconstruction

    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if ( x >= CROSS_W || y >= CROSS_H )
        return;

    uint8_t centre = raw_occ[ y * CROSS_W + x ];

    if ( centre > 0 )
        atomicAdd( arrived_count, 1U );

    uint8_t value = centre;
    
    value = min( value, read_neutral_border( raw_occ, x - 1, y ) );
    value = min( value, read_neutral_border( raw_occ, x, y - 1 ) );
    value = min( value, read_neutral_border( raw_occ, x - 1, y - 1 ) );

    eroded_occ[ y * CROSS_W + x ] = value;

    if ( value > 0 )
        atomicAdd( eroded_count, 1U );
}

__global__ void reconstruct_3d_kernel( const uint8_t *y_plane, const uint8_t *u_plane, const uint8_t *v_plane, const uint8_t *eroded_occ, struct host_point *out_points, uint32_t *valid_count, float global_scale, float bbox_center_x, float bbox_center_y, float bbox_center_z ) {
    
    // Purpose: It reverses the geometric projection, decoding "YUV" planes back into volumetric space by correlating face mappings & overtuning the "BT.601" color transform
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int face_id = blockIdx.z * blockDim.z + threadIdx.z;

    if ( x >= WIDTH || y >= HEIGHT || face_id >= 6 )
        return;

    int grid_row = 0;
    int grid_col = 0;

    if ( face_id == 0 ) {
        grid_row = 1;
        grid_col = 0;
    }
    else if ( face_id == 1 ) {
        grid_row = 1;
        grid_col = 2;
    }
    else if ( face_id == 2 ) {
        grid_row = 1;
        grid_col = 3;
    }
    else if ( face_id == 3 ) {
        grid_row = 1;
        grid_col = 1;
    }
    else if ( face_id == 4 ) {
        grid_row = 0;
        grid_col = 1;
    }
    else {
        grid_row = 2;
        grid_col = 1;
    }

    int abs_y = grid_row * FACE_H_PADDED + y;
    int abs_x = grid_col * FACE_W_PADDED + x;
    int occ_idx = abs_y * CROSS_W + abs_x;

    if ( eroded_occ[ occ_idx ] < 140 )
        return;

    float inv_w = ( WIDTH > 1 ) ? 1.0f / ( float )( WIDTH - 1 ) : 0.0f;
    float inv_h = ( HEIGHT > 1 ) ? 1.0f / ( float )( HEIGHT - 1 ) : 0.0f;

    float u_norm = ( float )x * inv_w;
    float v_norm = ( float )y * inv_h;
    float d_norm = ( float )y_plane[ abs_y * DECODER_W + abs_x ] / 255.0f;

    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 0.0f;

    if ( face_id == 0 ) {
        nx = u_norm;
        ny = 1.0f - v_norm;
        nz = d_norm;
    }
    else if ( face_id == 1 ) {
        nx = 1.0f - u_norm;
        ny = 1.0f - v_norm;
        nz = 1.0f - d_norm;
    }
    else if ( face_id == 2 ) {
        nx = 1.0f - d_norm;
        ny = 1.0f - v_norm;
        nz = u_norm;
    }
    else if ( face_id == 3 ) {
        nx = d_norm;
        ny = 1.0f - v_norm;
        nz = 1.0f - u_norm;
    }
    else if ( face_id == 4 ) {
        nx = u_norm;
        ny = d_norm;
        nz = v_norm;
    }
    else {
        nx = u_norm;
        ny = 1.0f - d_norm;
        nz = 1.0f - v_norm;
    }

    uint32_t write_idx = atomicAdd( valid_count, 1U );

    if ( write_idx >= MAX_RECONSTRUCTED_POINTS )
        return;

    struct host_point &point = out_points[ write_idx ];

    point.x = ( nx - 0.5f ) * ( float )WIDTH * global_scale + bbox_center_x;
    point.y = ( ny - 0.5f ) * ( float )HEIGHT * global_scale + bbox_center_y;
    point.z = ( nz - 0.5f ) * ( float )WIDTH * global_scale + bbox_center_z;

    int tex_y_row = CROSS_H + abs_y;
    float Y = ( float )y_plane[ tex_y_row * DECODER_W + abs_x ];

    int uv_row = tex_y_row / 2;
    int uv_col = abs_x / 2;
    int uv_idx = uv_row * ( DECODER_W / 2 ) + uv_col;

    float U = ( float )u_plane[ uv_idx ];
    float V = ( float )v_plane[ uv_idx ];

    int r = ( int )( Y + 1.402f * ( V - 128.0f ) );
    int g = ( int )( Y - 0.344136f * ( U - 128.0f ) - 0.714136f * ( V - 128.0f ) );
    int b = ( int )( Y + 1.772f * ( U - 128.0f ) );

    point.r = ( uint8_t )min( max( r, 0 ), 255 );
    point.g = ( uint8_t )min( max( g, 0 ), 255 );
    point.b = ( uint8_t )min( max( b, 0 ), 255 );
    point.padding = 0;
}

__global__ void apply_pose_kernel( struct host_point *points, const uint32_t *valid_count, float final_scale, struct rotation_matrix encoder_rotation, struct rotation_matrix decoder_rotation, float decoder_zoom ) {
    
    // Purpose: It coordinates the final point-cloud alignment utilizing rotation matrices instantiated by upstream stance directives & down-scaling components
    
    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    uint32_t point_count = *valid_count;

    if ( i >= point_count )
        return;

    float scale = ( fabsf( final_scale ) > 1e-9f ) ? final_scale : 1.0f;

    float qx = points[ i ].x / scale;
    float qy = points[ i ].y / scale;
    float qz = ( points[ i ].z - CAMERA_DISTANCE ) / scale;

    float bx = qx * encoder_rotation.r00 + qy * encoder_rotation.r10 + qz * encoder_rotation.r20;
    float by = qx * encoder_rotation.r01 + qy * encoder_rotation.r11 + qz * encoder_rotation.r21;
    float bz = qx * encoder_rotation.r02 + qy * encoder_rotation.r12 + qz * encoder_rotation.r22;

    float px = decoder_rotation.r00 * bx + decoder_rotation.r01 * by + decoder_rotation.r02 * bz;
    float py = decoder_rotation.r10 * bx + decoder_rotation.r11 * by + decoder_rotation.r12 * bz;
    float pz = decoder_rotation.r20 * bx + decoder_rotation.r21 * by + decoder_rotation.r22 * bz;

    points[ i ].x = px * decoder_zoom;
    points[ i ].y = py * decoder_zoom;
    points[ i ].z = pz * decoder_zoom;
}

extern "C" void cuda_memory_init() {
    CHECK_CUDA( cudaFree( 0 ) );
    CHECK_CUDA( cudaStreamCreateWithFlags( &reconstruction_stream, cudaStreamNonBlocking ) );

    CHECK_CUDA( cudaEventCreate( &pipeline_start_event ) );
    CHECK_CUDA( cudaEventCreate( &h2d_done_event ) );
    CHECK_CUDA( cudaEventCreate( &erosion_done_event ) );
    CHECK_CUDA( cudaEventCreate( &reconstruction_done_event ) );
    CHECK_CUDA( cudaEventCreate( &pose_done_event ) );
    CHECK_CUDA( cudaEventCreate( &copyback_start_event ) );
    CHECK_CUDA( cudaEventCreate( &d2h_done_event ) );

    CHECK_CUDA( cudaMalloc( ( void ** )&d_i420, TOTAL_YUV_SIZE ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_occ_eroded, ( size_t )CROSS_W * CROSS_H ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_points, ( size_t )MAX_RECONSTRUCTED_POINTS * sizeof( struct host_point ) ) );

    CHECK_CUDA( cudaMalloc( ( void ** )&d_arrived_count, sizeof( uint32_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_eroded_count, sizeof( uint32_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_valid_count, sizeof( uint32_t ) ) );
}

extern "C" void cuda_memory_free() {
    if ( pipeline_start_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( pipeline_start_event ) );

    if ( h2d_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( h2d_done_event ) );

    if ( erosion_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( erosion_done_event ) );

    if ( reconstruction_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( reconstruction_done_event ) );

    if ( pose_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( pose_done_event ) );

    if ( copyback_start_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( copyback_start_event ) );

    if ( d2h_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( d2h_done_event ) );

    if ( reconstruction_stream != nullptr )
        CHECK_CUDA( cudaStreamDestroy( reconstruction_stream ) );

    pipeline_start_event = nullptr;
    h2d_done_event = nullptr;
    erosion_done_event = nullptr;
    reconstruction_done_event = nullptr;
    pose_done_event = nullptr;
    copyback_start_event = nullptr;
    d2h_done_event = nullptr;
    reconstruction_stream = nullptr;

    cudaFree( d_valid_count );
    cudaFree( d_eroded_count );
    cudaFree( d_arrived_count );
    cudaFree( d_points );
    cudaFree( d_occ_eroded );
    cudaFree( d_i420 );

    d_valid_count = nullptr;
    d_eroded_count = nullptr;
    d_arrived_count = nullptr;
    d_points = nullptr;
    d_occ_eroded = nullptr;
    d_i420 = nullptr;
}

extern "C" void cuda_memory_warmup() {
    uint8_t *dummy_i420 = nullptr;
    struct host_point *dummy_points = nullptr;

    CHECK_CUDA( cudaHostAlloc( ( void ** )&dummy_i420, TOTAL_YUV_SIZE, cudaHostAllocDefault ) );
    CHECK_CUDA( cudaHostAlloc( ( void ** )&dummy_points, ( size_t )MAX_RECONSTRUCTED_POINTS * sizeof( struct host_point ), cudaHostAllocDefault ) );

    memset( dummy_i420, 0, TOTAL_YUV_SIZE );

    memset( dummy_i420 + SIZE_Y, 128, TOTAL_YUV_SIZE - SIZE_Y );

    const int warm_x = 100;
    const int warm_y = 100;
    const int abs_x = warm_x;
    const int abs_y = FACE_H_PADDED + warm_y;
    const size_t raw_occ_offset = ( size_t )CROSS_H * 2 * DECODER_W;

    for ( int dy = -1; dy <= 0; dy++ )
        for ( int dx = -1; dx <= 0; dx++ )
            dummy_i420[ raw_occ_offset + ( size_t )( abs_y + dy ) * DECODER_W + ( abs_x + dx ) ] = 255;

    dummy_i420[ ( size_t )abs_y * DECODER_W + abs_x ] = 128;
    dummy_i420[ ( size_t )( CROSS_H + abs_y ) * DECODER_W + abs_x ] = 128;

    size_t warm_uv_idx = ( size_t )( ( CROSS_H + abs_y ) / 2 ) * ( DECODER_W / 2 ) + ( abs_x / 2 );
    dummy_i420[ SIZE_Y + warm_uv_idx ] = 128;
    dummy_i420[ SIZE_Y + SIZE_UV + warm_uv_idx ] = 128;

    struct enc_hdr dummy_metadata = { 0 };

    dummy_metadata.frame_id = rte_cpu_to_be_32( 1 );
    dummy_metadata.packet_id = 0;
    dummy_metadata.global_scale = float_to_be( 1.0f );
    dummy_metadata.box_center_x = float_to_be( 0.0f );
    dummy_metadata.box_center_y = float_to_be( 0.0f );
    dummy_metadata.box_center_z = float_to_be( CAMERA_DISTANCE );
    dummy_metadata.yaw = float_to_be( 0.0f );
    dummy_metadata.pitch = float_to_be( 0.0f );
    dummy_metadata.final_scale = float_to_be( 1.0f );
    dummy_metadata.centroid_x = float_to_be( 0.0f );
    dummy_metadata.centroid_y = float_to_be( 0.0f );
    dummy_metadata.centroid_z = float_to_be( 0.0f );

    uint32_t dummy_arrived_points = 0;
    uint32_t dummy_eroded_points = 0;
    uint32_t dummy_valid_points = 0;
    double dummy_metrics[ 5 ] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    uint64_t dummy_pose_apply_end_cycles = 0;
    uint64_t dummy_pipeline_end_cycles = 0;

    run_reconstruction_pipeline( dummy_i420, &dummy_metadata, 0.05f, -0.05f, 1.05f, dummy_points, &dummy_arrived_points, &dummy_eroded_points, &dummy_valid_points, dummy_metrics, &dummy_pose_apply_end_cycles, &dummy_pipeline_end_cycles, nullptr );

    if ( dummy_valid_points == 0 ) {
        fprintf( stderr, "[SYSTEM] Error: CUDA Decoder warm-up did not produce a valid reconstructed sample...\n" );
        cudaFreeHost( dummy_points );
        cudaFreeHost( dummy_i420 );
        exit( EXIT_FAILURE );
    }

    CHECK_CUDA( cudaFreeHost( dummy_points ) );
    CHECK_CUDA( cudaFreeHost( dummy_i420 ) );
}

extern "C" void cuda_memory_register( void *ptr, size_t size ) {
    CHECK_CUDA( cudaHostRegister( ptr, size, cudaHostRegisterDefault ) );
}

extern "C" void cuda_memory_unleash( void *ptr ) {
    CHECK_CUDA( cudaHostUnregister( ptr ) );
}

extern "C" void run_reconstruction_pipeline( const uint8_t *i420_frame, const struct enc_hdr *metadata, float decoder_yaw, float decoder_pitch, float decoder_zoom, struct host_point *out_points, uint32_t *out_arrived_points, uint32_t *out_eroded_points, uint32_t *out_valid_points, double *gpu_metrics, uint64_t *out_pose_apply_end_cycles, uint64_t *out_pipeline_end_cycles, process_callback_t process_callback ) {
    
    // Purpose: It administers asynchronous "H2D" transfer, filtering, volumetric reconstruction & "D2H" copies using pose metadata
    
    if ( i420_frame == nullptr || metadata == nullptr || out_points == nullptr )
        return;

    float global_scale = be_to_float( metadata -> global_scale );
    float bbox_center_x = be_to_float( metadata -> box_center_x );
    float bbox_center_y = be_to_float( metadata -> box_center_y );
    float bbox_center_z = be_to_float( metadata -> box_center_z );
    float encoder_yaw = be_to_float( metadata -> yaw );
    float encoder_pitch = be_to_float( metadata -> pitch );
    float final_scale = be_to_float( metadata -> final_scale );

    if ( !isfinite( global_scale ) || global_scale <= 0.0f )
        global_scale = 1.0f;

    if ( !isfinite( final_scale ) || fabsf( final_scale ) <= 1e-9f )
        final_scale = 1.0f;

    if ( !isfinite( encoder_yaw ) )
        encoder_yaw = 0.0f;
    if ( !isfinite( encoder_pitch ) )
        encoder_pitch = 0.0f;

    if ( !isfinite( decoder_yaw ) )
        decoder_yaw = 0.0f;
    if ( !isfinite( decoder_pitch ) )
        decoder_pitch = 0.0f;
    if ( !isfinite( decoder_zoom ) || decoder_zoom <= 0.0f )
        decoder_zoom = 1.0f;

    struct rotation_matrix encoder_rotation = rotation_from_pose( encoder_yaw, encoder_pitch );
    struct rotation_matrix decoder_rotation = rotation_from_pose( decoder_yaw, decoder_pitch );

    cudaStream_t stream = reconstruction_stream;

    if ( process_callback != nullptr )
        process_callback();

    CHECK_CUDA( cudaEventRecord( pipeline_start_event, stream ) );

    CHECK_CUDA( cudaMemcpyAsync( d_i420, i420_frame, TOTAL_YUV_SIZE, cudaMemcpyHostToDevice, stream ) );
    
    CHECK_CUDA( cudaEventRecord( h2d_done_event, stream ) );

    if ( process_callback != nullptr )
        process_callback();

    CHECK_CUDA( cudaMemsetAsync( d_arrived_count, 0, sizeof( uint32_t ), stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_eroded_count, 0, sizeof( uint32_t ), stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_valid_count, 0, sizeof( uint32_t ), stream ) );

    const uint8_t *raw_occ = d_i420 + ( ( size_t )CROSS_H * 2 * DECODER_W );

    dim3 erosion_threads( 16, 16 );
    dim3 erosion_blocks( ( CROSS_W + 15 ) / 16, ( CROSS_H + 15 ) / 16 );

    erosion_2x2_kernel<<< erosion_blocks, erosion_threads, 0, stream >>>( raw_occ, d_occ_eroded, d_arrived_count, d_eroded_count );
    CHECK_CUDA( cudaGetLastError() );
    CHECK_CUDA( cudaEventRecord( erosion_done_event, stream ) );

    if ( process_callback != nullptr )
        process_callback();

    const uint8_t *y_plane = d_i420;
    const uint8_t *u_plane = d_i420 + SIZE_Y;
    const uint8_t *v_plane = d_i420 + SIZE_Y + SIZE_UV;

    dim3 reconstruction_threads( 8, 8, 4 );
    dim3 reconstruction_blocks( ( WIDTH + 7 ) / 8, ( HEIGHT + 7 ) / 8, ( 6 + 3 ) / 4 );

    reconstruct_3d_kernel<<< reconstruction_blocks, reconstruction_threads, 0, stream >>>( y_plane, u_plane, v_plane, d_occ_eroded, d_points, d_valid_count, global_scale, bbox_center_x, bbox_center_y, bbox_center_z );
    CHECK_CUDA( cudaGetLastError() );
    CHECK_CUDA( cudaEventRecord( reconstruction_done_event, stream ) );

    if ( process_callback != nullptr )
        process_callback();

    apply_pose_kernel<<< ( MAX_RECONSTRUCTED_POINTS + 255 ) / 256, 256, 0, stream >>>( d_points, d_valid_count, final_scale, encoder_rotation, decoder_rotation, decoder_zoom );
    CHECK_CUDA( cudaGetLastError() );

    CHECK_CUDA( cudaEventRecord( pose_done_event, stream ) );

    if ( process_callback != nullptr ) {
        while ( cudaEventQuery( pose_done_event ) == cudaErrorNotReady )
            process_callback();

        CHECK_CUDA( cudaEventQuery( pose_done_event ) );
    }
    else
        CHECK_CUDA( cudaEventSynchronize( pose_done_event ) );

    if ( out_pose_apply_end_cycles )
        *out_pose_apply_end_cycles = rte_get_timer_cycles();

    uint32_t arrived_points = 0;
    uint32_t eroded_points = 0;
    uint32_t valid_points = 0;

    CHECK_CUDA( cudaMemcpy( &arrived_points, d_arrived_count, sizeof( uint32_t ), cudaMemcpyDeviceToHost ) );
    CHECK_CUDA( cudaMemcpy( &eroded_points, d_eroded_count, sizeof( uint32_t ), cudaMemcpyDeviceToHost ) );
    CHECK_CUDA( cudaMemcpy( &valid_points, d_valid_count, sizeof( uint32_t ), cudaMemcpyDeviceToHost ) );

    if ( valid_points > MAX_RECONSTRUCTED_POINTS )
        valid_points = MAX_RECONSTRUCTED_POINTS;

    CHECK_CUDA( cudaEventRecord( copyback_start_event, stream ) );

    if ( valid_points > 0 )
        CHECK_CUDA( cudaMemcpyAsync( out_points, d_points, ( size_t )valid_points * sizeof( struct host_point ), cudaMemcpyDeviceToHost, stream ) );

    CHECK_CUDA( cudaEventRecord( d2h_done_event, stream ) );

    if ( process_callback != nullptr ) {
        while ( cudaStreamQuery( stream ) == cudaErrorNotReady )
            process_callback();

        CHECK_CUDA( cudaStreamQuery( stream ) );
    }
    else
        CHECK_CUDA( cudaStreamSynchronize( stream ) );

    if ( out_pipeline_end_cycles )
        *out_pipeline_end_cycles = rte_get_timer_cycles();

    if ( out_arrived_points )
        *out_arrived_points = arrived_points;
    if ( out_eroded_points )
        *out_eroded_points = eroded_points;
    if ( out_valid_points )
        *out_valid_points = valid_points;

    if ( gpu_metrics ) {
        float ms = 0.0f;

        CHECK_CUDA( cudaEventElapsedTime( &ms, pipeline_start_event, h2d_done_event ) );
        gpu_metrics[ 0 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, h2d_done_event, erosion_done_event ) );
        gpu_metrics[ 1 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, erosion_done_event, reconstruction_done_event ) );
        gpu_metrics[ 2 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, reconstruction_done_event, pose_done_event ) );
        gpu_metrics[ 3 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, copyback_start_event, d2h_done_event ) );
        gpu_metrics[ 4 ] = ms;
    }
}