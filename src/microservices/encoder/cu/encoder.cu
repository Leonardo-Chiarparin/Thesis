#ifndef __CUDACC__
#define __CUDACC__
#endif

#include "encoder.h"
#include <cuda_runtime.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <rte_cycles.h>

#define CHECK_CUDA( call ) { \
    cudaError_t err = call; \
    if ( err != cudaSuccess ) { \
        std::cerr << "\"CUDA\" Error: " << cudaGetErrorString( err ) << std::endl; \
    } \
}

// Persistent device-side buffers eliminate frame-local allocation & memory-handling overhead from the critical streaming path
static struct host_point *d_points = nullptr;
static uint8_t *d_out = nullptr;
static int32_t *d_zbuf = nullptr;
static uint8_t *d_geo = nullptr;
static uint8_t *d_occ = nullptr;
static uint8_t *d_tex_y = nullptr;
static uint8_t *d_tex_u = nullptr;
static uint8_t *d_tex_v = nullptr;

static cudaStream_t projection_stream = nullptr;

static cudaEvent_t projection_start_event = nullptr;
static cudaEvent_t h2d_done_event = nullptr;
static cudaEvent_t kernel_done_event = nullptr;
static cudaEvent_t packing_done_event = nullptr;
static cudaEvent_t d2h_done_event = nullptr;

extern "C" void cuda_memory_init( uint32_t max_pts ) {

    // Purpose: It preallocates all requisite components for the fixed-resolution projection pipeline, circumventing distribution system burden

    CHECK_CUDA( cudaMalloc( ( void ** )&d_points, max_pts * sizeof( struct host_point ) ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_out, TOTAL_YUV_SIZE ) );

    size_t face_mem = 6 * FACE_H_PADDED * FACE_W_PADDED;

    CHECK_CUDA( cudaMalloc( ( void ** )&d_zbuf, face_mem * sizeof( int32_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_geo, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_occ, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_tex_y, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_tex_u, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void ** )&d_tex_v, face_mem ) );

    CHECK_CUDA( cudaFree( 0 ) );
    CHECK_CUDA( cudaStreamCreate( &projection_stream ) );

    CHECK_CUDA( cudaEventCreate( &projection_start_event ) );
    CHECK_CUDA( cudaEventCreate( &h2d_done_event ) );
    CHECK_CUDA( cudaEventCreate( &kernel_done_event ) );
    CHECK_CUDA( cudaEventCreate( &packing_done_event ) );
    CHECK_CUDA( cudaEventCreate( &d2h_done_event ) );
}

extern "C" void cuda_memory_free() {

    // Purpose: It releases "CUDA" context resources & timing events

    if ( projection_start_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( projection_start_event ) );

    if ( h2d_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( h2d_done_event ) );

    if ( kernel_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( kernel_done_event ) );

    if ( packing_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( packing_done_event ) );

    if ( d2h_done_event != nullptr )
        CHECK_CUDA( cudaEventDestroy( d2h_done_event ) );

    if ( projection_stream != nullptr )
        CHECK_CUDA( cudaStreamDestroy( projection_stream ) );

    projection_start_event = nullptr;
    h2d_done_event = nullptr;
    kernel_done_event = nullptr;
    packing_done_event = nullptr;
    d2h_done_event = nullptr;
    projection_stream = nullptr;

    cudaFree( d_points );
    cudaFree( d_out );
    cudaFree( d_zbuf );
    cudaFree( d_geo );
    cudaFree( d_occ );
    cudaFree( d_tex_y );
    cudaFree( d_tex_u );
    cudaFree( d_tex_v );

    d_points = nullptr;
    d_out = nullptr;
    d_zbuf = nullptr;
    d_geo = nullptr;
    d_occ = nullptr;
    d_tex_y = nullptr;
    d_tex_u = nullptr;
    d_tex_v = nullptr;
}

extern "C" void cuda_memory_warmup() {

    // Purpose: It executes an untimed projection pipeline iteration to initialize the forthcoming scenario, kernel paths, & buffer residency prior to measurement

    const uint32_t n_pts = 100;
    struct host_point dummy_points[ n_pts ];

    for ( uint32_t i = 0; i < n_pts; i++ ) {
        dummy_points[ i ].x = ( ( float )rand() / RAND_MAX ) * 10.0f;
        dummy_points[ i ].y = ( ( float )rand() / RAND_MAX ) * 10.0f;
        dummy_points[ i ].z = ( ( float )rand() / RAND_MAX ) * 10.0f;
        dummy_points[ i ].r = 0;
        dummy_points[ i ].g = 0;
        dummy_points[ i ].b = 0;
        dummy_points[ i ].padding = 0;
    }

    uint8_t *dummy_out = ( uint8_t * )malloc( TOTAL_YUV_SIZE );
    double dummy_metrics[ 4 ] = { 0.0, 0.0, 0.0, 0.0 };
    float dummy_global_scale = 1.0f;
    float dummy_bbox_center_x = 0.0f;
    float dummy_bbox_center_y = 0.0f;
    float dummy_bbox_center_z = 0.0f;
    uint64_t dummy_projection_end_cycles = 0;

    run_projection_pipeline( dummy_points, n_pts, 0.0f, 0.0f, 0.0f, 100.0f, 100.0f, 100.0f, 0.0f, 0.0f, 0.0f, 1.0f, CAMERA_DISTANCE, dummy_out, dummy_metrics, &dummy_global_scale, &dummy_bbox_center_x, &dummy_bbox_center_y, &dummy_bbox_center_z, &dummy_projection_end_cycles, nullptr );

    free( dummy_out );
}

extern "C" void cuda_memory_register( void *ptr, size_t size ) {
    
    // Purpose: It registers host memory with the "CUDA" runtime to enable optimized pinned transfers
    
    CHECK_CUDA( cudaHostRegister( ptr, size, cudaHostRegisterDefault ) );
}

extern "C" void cuda_memory_unleash( void *ptr ) {

    // Purpose: It removes user references from the environment

    CHECK_CUDA( cudaHostUnregister( ptr ) );
}

__global__ void generate_gbuffer_cuda_kernel( const struct host_point *points, uint32_t num_pts, float centroid_x, float centroid_y, float centroid_z, float final_scale, float cam_dist, float bbox_center_x, float bbox_center_y, float bbox_center_z, float global_scale, int32_t *z_buffer, uint8_t *geo_y, uint8_t *occ_y, uint8_t *tex_y, uint8_t *tex_u, uint8_t *tex_v ) {

    // Purpose: It fuses object-centric scaling, normalized coordinate construction, "BT.601" colour conversion, orthographic projection & z-buffer visibility mapping into a unified point-parallel kernel.
    //          For element "p", "t = ( p - C ) * final_scale + ( 0, 0, camera_distance )"

    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;

    if ( i >= num_pts )
        return;

    float tx = ( points[ i ].x - centroid_x ) * final_scale;
    float ty = ( points[ i ].y - centroid_y ) * final_scale;
    float tz = ( points[ i ].z - centroid_z ) * final_scale + cam_dist;

    float nx = ( ( tx - bbox_center_x ) / ( ( float )WIDTH * global_scale ) ) + 0.5f;
    float ny = ( ( ty - bbox_center_y ) / ( ( float )HEIGHT * global_scale ) ) + 0.5f;
    float nz = ( ( tz - bbox_center_z ) / ( ( float )WIDTH * global_scale ) ) + 0.5f;

    uint8_t Y = min( max( ( int )( 0.299f * points[ i ].r + 0.587f * points[ i ].g + 0.114f * points[ i ].b ), 0 ), 255 );
    uint8_t U = min( max( ( int )( -0.169f * points[ i ].r - 0.331f * points[ i ].g + 0.500f * points[ i ].b + 128.0f ), 0 ), 255 );
    uint8_t V = min( max( ( int )( 0.500f * points[ i ].r - 0.419f * points[ i ].g - 0.081f * points[ i ].b + 128.0f ), 0 ), 255 );

    for ( int face_id = 0; face_id < 6; face_id++ ) {
        float u = 0.0f;
        float v = 0.0f;
        float d = 0.0f;

        if ( face_id == 0 ) {
            u = nx;
            v = 1.0f - ny;
            d = nz;
        }
        else if ( face_id == 1 ) {
            u = 1.0f - nx;
            v = 1.0f - ny;
            d = 1.0f - nz;
        }
        else if ( face_id == 2 ) {
            u = nz;
            v = 1.0f - ny;
            d = 1.0f - nx;
        }
        else if ( face_id == 3 ) {
            u = 1.0f - nz;
            v = 1.0f - ny;
            d = nx;
        }
        else if ( face_id == 4 ) {
            u = nx;
            v = nz;
            d = ny;
        }
        else {
            u = nx;
            v = 1.0f - nz;
            d = 1.0f - ny;
        }

        int px = ( int )( u * ( WIDTH - 1 ) );
        int py = ( int )( v * ( HEIGHT - 1 ) );

        if ( px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT ) {
            int depth_int = ( int )( ( 1.0f - d ) * 1000000.0f );
            int idx = ( face_id * FACE_H_PADDED * FACE_W_PADDED ) + ( py * FACE_W_PADDED ) + px;
            int old_depth = atomicMax( &z_buffer[ idx ], depth_int );

            if ( depth_int >= old_depth ) {
                geo_y[ idx ] = ( uint8_t )( d * 255.0f );
                occ_y[ idx ] = 255;
                tex_y[ idx ] = Y;
                tex_u[ idx ] = U;
                tex_v[ idx ] = V;
            }
        }
    }
}

__global__ void pack_i420_stream_cuda( const uint8_t *geo_y, const uint8_t *occ_y, const uint8_t *tex_y, const uint8_t *tex_u, const uint8_t *tex_v, uint8_t *out_buffer ) {

    // Purpose: It organizes deterministic face positions into vertically stacked "Geometry" / "Texture" / "Occupancy" crosses, outputting the native "I420" format required by "FFmpeg"

    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int face_idx = blockIdx.z * blockDim.z + threadIdx.z;

    if ( x >= FACE_W_PADDED || y >= FACE_H_PADDED || face_idx >= 6 )
        return;

    int col = 0;
    int row = 0;

    if ( face_idx == 0 ) {
        col = 0;
        row = 1;
    }
    else if ( face_idx == 1 ) {
        col = 2;
        row = 1;
    }
    else if ( face_idx == 2 ) {
        col = 3;
        row = 1;
    }
    else if ( face_idx == 3 ) {
        col = 1;
        row = 1;
    }
    else if ( face_idx == 4 ) {
        col = 1;
        row = 0;
    }
    else {
        col = 1;
        row = 2;
    }

    int start_x = col * FACE_W_PADDED;
    int start_y = row * FACE_H_PADDED;
    int face_linear = ( face_idx * FACE_H_PADDED * FACE_W_PADDED ) + ( y * FACE_W_PADDED ) + x;

    int idx_geo = ( ( start_y + y ) * ENCODER_W ) + ( start_x + x );
    int idx_tex = ( ( CROSS_H + start_y + y ) * ENCODER_W ) + ( start_x + x );
    int idx_occ = ( ( ( CROSS_H * 2 ) + start_y + y ) * ENCODER_W ) + ( start_x + x );

    out_buffer[ idx_geo ] = geo_y[ face_linear ];
    out_buffer[ idx_tex ] = tex_y[ face_linear ];
    out_buffer[ idx_occ ] = occ_y[ face_linear ];

    if ( x % 2 == 0 && y % 2 == 0 ) {
        int uv_col = col * ( FACE_W_PADDED / 2 );
        int uv_row = ( CROSS_H + row * FACE_H_PADDED ) / 2;
        int idx_uv = ( ( uv_row + ( y / 2 ) ) * ( ENCODER_W / 2 ) ) + ( uv_col + ( x / 2 ) );

        out_buffer[ SIZE_Y + idx_uv ] = ( occ_y[ face_linear ] > 0 ) ? tex_u[ face_linear ] : 128;
        out_buffer[ SIZE_Y + SIZE_UV + idx_uv ] = ( occ_y[ face_linear ] > 0 ) ? tex_v[ face_linear ] : 128;
    }
}

extern "C" void run_projection_pipeline( const struct host_point *points, uint32_t num_pts, float centroid_x, float centroid_y, float centroid_z, float extent_x, float extent_y, float extent_z, float raw_bbox_center_x, float raw_bbox_center_y, float raw_bbox_center_z, float final_scale, float cam_dist, uint8_t *out_yuv_buffer, double *gpu_metrics, float *out_global_scale, float *out_bbox_center_x, float *out_bbox_center_y, float *out_bbox_center_z, uint64_t *out_projection_end_cycles, dpdk_poll_callback_t dpdk_poll_callback ) {

    // Purpose: It orchestrates asynchronous "H2D" transfer, "G-Buffer" projection, "Atlas" packing & "D2H" copies, exploiting the static pose data to optimize transformations
    //          Since "yaw" = "pitch" = 0 & "zoom" = 1, transformed bounds are exact affine images of the raw frontiers, therefore no point-wise box reduction is necessary

    if ( num_pts == 0 ) {
        if ( out_global_scale )
            *out_global_scale = 1.0f;

        if ( out_bbox_center_x )
            *out_bbox_center_x = 0.0f;

        if ( out_bbox_center_y )
            *out_bbox_center_y = 0.0f;

        if ( out_bbox_center_z )
            *out_bbox_center_z = 0.0f;

        if ( gpu_metrics ) {
            gpu_metrics[ 0 ] = 0.0;
            gpu_metrics[ 1 ] = 0.0;
            gpu_metrics[ 2 ] = 0.0;
            gpu_metrics[ 3 ] = 0.0;
        }

        if ( out_projection_end_cycles )
            *out_projection_end_cycles = rte_get_timer_cycles();

        return;
    }

    cudaStream_t stream = projection_stream;

    CHECK_CUDA( cudaEventRecord( projection_start_event, stream ) );

    for ( uint32_t offset = 0; offset < num_pts; offset += H2D_CHUNK_POINTS ) {
        uint32_t cur_pts = ( offset + H2D_CHUNK_POINTS > num_pts ) ? ( num_pts - offset ) : H2D_CHUNK_POINTS;

        if ( dpdk_poll_callback != nullptr )
            dpdk_poll_callback();

        CHECK_CUDA( cudaMemcpyAsync( d_points + offset, points + offset, cur_pts * sizeof( struct host_point ), cudaMemcpyHostToDevice, stream ) );
    }

    CHECK_CUDA( cudaEventRecord( h2d_done_event, stream ) );

    float bbox_center_x = ( raw_bbox_center_x - centroid_x ) * final_scale;
    float bbox_center_y = ( raw_bbox_center_y - centroid_y ) * final_scale;
    float bbox_center_z = ( raw_bbox_center_z - centroid_z ) * final_scale + cam_dist;

    float transformed_extent_x = extent_x * final_scale;
    float transformed_extent_y = extent_y * final_scale;
    float transformed_extent_z = extent_z * final_scale;

    float scale_x = transformed_extent_x / ( float )WIDTH;
    float scale_y = transformed_extent_y / ( float )HEIGHT;
    float scale_z = transformed_extent_z / ( float )WIDTH;
    float global_scale = fmaxf( fmaxf( scale_x, scale_y ), scale_z ) * 1.10f;

    if ( !isfinite( global_scale ) || global_scale <= 0.0f )
        global_scale = 1.0f;

    if ( out_global_scale )
        *out_global_scale = global_scale;

    if ( out_bbox_center_x )
        *out_bbox_center_x = bbox_center_x;

    if ( out_bbox_center_y )
        *out_bbox_center_y = bbox_center_y;

    if ( out_bbox_center_z )
        *out_bbox_center_z = bbox_center_z;

    size_t face_mem = 6 * FACE_H_PADDED * FACE_W_PADDED;

    CHECK_CUDA( cudaMemsetAsync( d_zbuf, 0xFF, face_mem * sizeof( int32_t ), stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_geo, 0, face_mem, stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_occ, 0, face_mem, stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_tex_y, 0, face_mem, stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_tex_u, 128, face_mem, stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_tex_v, 128, face_mem, stream ) );

    generate_gbuffer_cuda_kernel<<< ( num_pts + 255 ) / 256, 256, 0, stream >>>( d_points, num_pts, centroid_x, centroid_y, centroid_z, final_scale, cam_dist, bbox_center_x, bbox_center_y, bbox_center_z, global_scale, d_zbuf, d_geo, d_occ, d_tex_y, d_tex_u, d_tex_v );
    CHECK_CUDA( cudaGetLastError() );

    CHECK_CUDA( cudaEventRecord( kernel_done_event, stream ) );

    CHECK_CUDA( cudaMemsetAsync( d_out, 0, SIZE_Y, stream ) );
    CHECK_CUDA( cudaMemsetAsync( d_out + SIZE_Y, 128, TOTAL_YUV_SIZE - SIZE_Y, stream ) );

    dim3 threads_pack( 8, 8, 4 );
    dim3 blocks_pack( ( FACE_W_PADDED + 7 ) / 8, ( FACE_H_PADDED + 7 ) / 8, ( 6 + 3 ) / 4 );

    pack_i420_stream_cuda<<< blocks_pack, threads_pack, 0, stream >>>( d_geo, d_occ, d_tex_y, d_tex_u, d_tex_v, d_out );
    CHECK_CUDA( cudaGetLastError() );

    CHECK_CUDA( cudaEventRecord( packing_done_event, stream ) );

    CHECK_CUDA( cudaMemcpyAsync( out_yuv_buffer, d_out, TOTAL_YUV_SIZE, cudaMemcpyDeviceToHost, stream ) );
    CHECK_CUDA( cudaEventRecord( d2h_done_event, stream ) );

    if ( dpdk_poll_callback != nullptr ) {
        while ( cudaStreamQuery( stream ) == cudaErrorNotReady )
            dpdk_poll_callback();

        CHECK_CUDA( cudaStreamQuery( stream ) );
    }
    else
        CHECK_CUDA( cudaStreamSynchronize( stream ) );

    if ( out_projection_end_cycles )
        *out_projection_end_cycles = rte_get_timer_cycles();

    if ( gpu_metrics ) {
        float ms = 0.0f;

        CHECK_CUDA( cudaEventElapsedTime( &ms, projection_start_event, h2d_done_event ) );
        gpu_metrics[ 0 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, h2d_done_event, kernel_done_event ) );
        gpu_metrics[ 1 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, kernel_done_event, packing_done_event ) );
        gpu_metrics[ 2 ] = ms;

        CHECK_CUDA( cudaEventElapsedTime( &ms, packing_done_event, d2h_done_event ) );
        gpu_metrics[ 3 ] = ms;
    }
}