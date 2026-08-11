#ifndef __CUDACC__
#define __CUDACC__
#endif

#include "encoder.h"
#include <cuda_runtime.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cfloat>
#include <cstring>
#include <cstdlib>
#include <rte_cycles.h>

#define CHECK_CUDA( call ) { \
    cudaError_t err = call; \
    if ( err != cudaSuccess ) { \
        std::cerr << "\"CUDA\" Error: " << cudaGetErrorString( err ) << std::endl; \
    } \
}

// Global pointers to avoid "cold start" & memory fragmentation
static float *d_x = nullptr, *d_y = nullptr, *d_z = nullptr;
static float *d_tx = nullptr, *d_ty = nullptr, *d_tz = nullptr;
static float *d_bbox_stats = nullptr;
static uint8_t *d_r = nullptr, *d_g = nullptr, *d_b = nullptr, *d_out = nullptr;
static int32_t *d_zbuf = nullptr;
static uint8_t *d_geo = nullptr, *d_occ = nullptr, *d_tex_y = nullptr, *d_tex_u = nullptr, *d_tex_v = nullptr;

static uint32_t current_allocated_pts = 0;

extern "C" void cuda_memory_init( uint32_t max_pts ) {
   current_allocated_pts = max_pts;

    CHECK_CUDA( cudaMalloc( ( void** )&d_x, max_pts * sizeof( float ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_y, max_pts * sizeof( float ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_z, max_pts * sizeof( float ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_tx, max_pts * sizeof( float ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_ty, max_pts * sizeof( float ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_tz, max_pts * sizeof( float ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_r, max_pts * sizeof( uint8_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_g, max_pts * sizeof( uint8_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_b, max_pts * sizeof( uint8_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_out, TOTAL_YUV_SIZE ) );

    size_t face_mem = 6 * FACE_H_PADDED * FACE_W_PADDED;
    CHECK_CUDA( cudaMalloc( ( void** )&d_zbuf, face_mem * sizeof( int32_t ) ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_geo, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_occ, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_tex_y, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_tex_u, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_tex_v, face_mem ) );
    CHECK_CUDA( cudaMalloc( ( void** )&d_bbox_stats, 6 * sizeof( float ) ) );

    cudaFree( 0 );
}

extern "C" void cuda_memory_free() {
    cudaFree( d_x ); cudaFree( d_y ); cudaFree( d_z );
    cudaFree( d_tx ); cudaFree( d_ty ); cudaFree( d_tz );
    cudaFree( d_r ); cudaFree( d_g ); cudaFree( d_b );
    cudaFree( d_out );
    cudaFree( d_zbuf ); cudaFree( d_geo ); cudaFree( d_occ );
    cudaFree( d_tex_y ); cudaFree( d_tex_u ); cudaFree( d_tex_v );
    cudaFree( d_bbox_stats );

    d_x = d_y = d_z = nullptr;
    d_tx = d_ty = d_tz = nullptr;
    d_r = d_g = d_b = d_out = nullptr;
    d_zbuf = nullptr;
    d_geo = d_occ = d_tex_y = d_tex_u = d_tex_v = nullptr;
    d_bbox_stats = nullptr;
    current_allocated_pts = 0;
}

extern "C" void cuda_memory_update( uint32_t required_pts ) {
    if ( required_pts > current_allocated_pts ) {
        cudaFree( d_x ); cudaFree( d_y ); cudaFree( d_z );
        cudaFree( d_tx ); cudaFree( d_ty ); cudaFree( d_tz );
        cudaFree( d_r ); cudaFree( d_g ); cudaFree( d_b );

        current_allocated_pts = required_pts * 2;

        CHECK_CUDA( cudaMalloc( ( void** )&d_x, current_allocated_pts * sizeof( float ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_y, current_allocated_pts * sizeof( float ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_z, current_allocated_pts * sizeof( float ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_tx, current_allocated_pts * sizeof( float ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_ty, current_allocated_pts * sizeof( float ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_tz, current_allocated_pts * sizeof( float ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_r, current_allocated_pts * sizeof( uint8_t ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_g, current_allocated_pts * sizeof( uint8_t ) ) );
        CHECK_CUDA( cudaMalloc( ( void** )&d_b, current_allocated_pts * sizeof( uint8_t ) ) );
    }
}

extern "C" void cuda_memory_warmup() {
    const int n_pts = 100;
    float dummy_x[ n_pts ], dummy_y[ n_pts ], dummy_z[ n_pts ];
    uint8_t dummy_r[ n_pts ], dummy_g[ n_pts ], dummy_b[ n_pts ];

    for ( int i = 0; i < n_pts; i++ ) {
        dummy_x[ i ] = ( ( float )rand() / RAND_MAX ) * 10.0f;
        dummy_y[ i ] = ( ( float )rand() / RAND_MAX ) * 10.0f;
        dummy_z[ i ] = ( ( float )rand() / RAND_MAX ) * 10.0f;
        dummy_r[ i ] = 0;
        dummy_g[ i ] = 0;
        dummy_b[ i ] = 0;
    }

    uint8_t *dummy_out = ( uint8_t * )malloc( TOTAL_YUV_SIZE );
    double dummy_metrics[ 4 ] = { 0.0, 0.0, 0.0, 0.0 };
    float dummy_global_scale = 1.0f;
    float dummy_bbox_center_x = 0.0f;
    float dummy_bbox_center_y = 0.0f;
    float dummy_bbox_center_z = 0.0f;
    uint64_t dummy_projection_end_cycles = 0;
    uint64_t dummy_encode_start_cycles = 0;

    run_projection_pipeline( dummy_x, dummy_y, dummy_z, dummy_r, dummy_g, dummy_b, n_pts, 0.0f, 0.0f, 0.0f, 100.0f, 100.0f, 100.0f, 1.0f, 0.0f, 0.0f, CAMERA_DIST, dummy_out, dummy_metrics, &dummy_global_scale, &dummy_bbox_center_x, &dummy_bbox_center_y, &dummy_bbox_center_z, &dummy_projection_end_cycles, &dummy_encode_start_cycles, nullptr );

    free( dummy_out );
}

extern "C" void cuda_memory_register( void* ptr, size_t size ) {
    CHECK_CUDA( cudaHostRegister( ptr, size, cudaHostRegisterDefault ) );
}

extern "C" void cuda_memory_unleash( void* ptr ) {
    CHECK_CUDA( cudaHostUnregister( ptr ) );
}

__device__ static inline float atomicMinFloat( float *address, float value ) {
    int *address_as_int = reinterpret_cast<int *>( address );
    int old = *address_as_int;

    while ( value < __int_as_float( old ) ) {
        int assumed = old;
        old = atomicCAS( address_as_int, assumed, __float_as_int( value ) );
        if ( old == assumed )
            break;
    }

    return __int_as_float( old );
}

__device__ static inline float atomicMaxFloat( float *address, float value ) {
    int *address_as_int = reinterpret_cast<int *>( address );
    int old = *address_as_int;

    while ( value > __int_as_float( old ) ) {
        int assumed = old;
        old = atomicCAS( address_as_int, assumed, __float_as_int( value ) );
        if ( old == assumed )
            break;
    }

    return __int_as_float( old );
}

__global__ void reset_gbuffer_cuda_kernel( int32_t* z_buffer, uint8_t* geo_y, uint8_t* occ_y, uint8_t* tex_y, uint8_t* tex_u, uint8_t* tex_v ) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int face_id = blockIdx.z * blockDim.z + threadIdx.z;

    if ( x < FACE_W_PADDED && y < FACE_H_PADDED && face_id < 6 ) {
        int idx = ( face_id * FACE_H_PADDED * FACE_W_PADDED ) + ( y * FACE_W_PADDED ) + x;
        z_buffer[ idx ] = -1;
        geo_y[ idx ] = 0;
        occ_y[ idx ] = 0;
        tex_y[ idx ] = 0;
        tex_u[ idx ] = 128;
        tex_v[ idx ] = 128;
    }
}

__global__ void transform_points_cuda_kernel( const float *pts_x, const float *pts_y, const float *pts_z, float *tx, float *ty, float *tz, uint32_t num_pts, float c_x, float c_y, float c_z, float final_scale, float cyaw, float syaw, float cpitch, float spitch, float cam_dist ) {
    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if ( i >= num_pts )
        return;

    float x = pts_x[ i ] - c_x;
    float y = pts_y[ i ] - c_y;
    float z = pts_z[ i ] - c_z;

    tx[ i ] = ( x * cyaw + y * ( syaw * spitch ) + z * ( syaw * cpitch ) ) * final_scale;
    ty[ i ] = ( y * cpitch - z * spitch ) * final_scale;
    tz[ i ] = ( -x * syaw + y * ( cyaw * spitch ) + z * ( cyaw * cpitch ) ) * final_scale + cam_dist;
}

__global__ void reduce_transformed_bbox_kernel( const float *tx, const float *ty, const float *tz, uint32_t num_pts, float *bbox_stats ) {
    __shared__ float s_min_x[ 256 ], s_min_y[ 256 ], s_min_z[ 256 ];
    __shared__ float s_max_x[ 256 ], s_max_y[ 256 ], s_max_z[ 256 ];

    uint32_t tid = threadIdx.x;
    uint32_t i = blockIdx.x * blockDim.x + tid;

    if ( i < num_pts ) {
        float x = tx[ i ];
        float y = ty[ i ];
        float z = tz[ i ];
        s_min_x[ tid ] = x; s_min_y[ tid ] = y; s_min_z[ tid ] = z;
        s_max_x[ tid ] = x; s_max_y[ tid ] = y; s_max_z[ tid ] = z;
    }
    else {
        s_min_x[ tid ] = FLT_MAX; s_min_y[ tid ] = FLT_MAX; s_min_z[ tid ] = FLT_MAX;
        s_max_x[ tid ] = -FLT_MAX; s_max_y[ tid ] = -FLT_MAX; s_max_z[ tid ] = -FLT_MAX;
    }

    __syncthreads();

    for ( uint32_t stride = blockDim.x / 2; stride > 0; stride >>= 1 ) {
        if ( tid < stride ) {
            s_min_x[ tid ] = fminf( s_min_x[ tid ], s_min_x[ tid + stride ] );
            s_min_y[ tid ] = fminf( s_min_y[ tid ], s_min_y[ tid + stride ] );
            s_min_z[ tid ] = fminf( s_min_z[ tid ], s_min_z[ tid + stride ] );
            s_max_x[ tid ] = fmaxf( s_max_x[ tid ], s_max_x[ tid + stride ] );
            s_max_y[ tid ] = fmaxf( s_max_y[ tid ], s_max_y[ tid + stride ] );
            s_max_z[ tid ] = fmaxf( s_max_z[ tid ], s_max_z[ tid + stride ] );
        }

        __syncthreads();
    }

    if ( tid == 0 ) {
        atomicMinFloat( &bbox_stats[ 0 ], s_min_x[ 0 ] );
        atomicMinFloat( &bbox_stats[ 1 ], s_min_y[ 0 ] );
        atomicMinFloat( &bbox_stats[ 2 ], s_min_z[ 0 ] );
        atomicMaxFloat( &bbox_stats[ 3 ], s_max_x[ 0 ] );
        atomicMaxFloat( &bbox_stats[ 4 ], s_max_y[ 0 ] );
        atomicMaxFloat( &bbox_stats[ 5 ], s_max_z[ 0 ] );
    }
}

__global__ void generate_gbuffer_cuda_kernel( const float* tx, const float* ty, const float* tz, const uint8_t* r, const uint8_t* g, const uint8_t* b, uint32_t num_pts, float bbox_center_x, float bbox_center_y, float bbox_center_z, float global_scale, int32_t* z_buffer, uint8_t* geo_y, uint8_t* occ_y, uint8_t* tex_y, uint8_t* tex_u, uint8_t* tex_v ) {
    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if ( i >= num_pts )
        return;

    float nx = ( ( tx[ i ] - bbox_center_x ) / ( ( float )WIDTH * global_scale ) ) + 0.5f;
    float ny = ( ( ty[ i ] - bbox_center_y ) / ( ( float )HEIGHT * global_scale ) ) + 0.5f;
    float nz = ( ( tz[ i ] - bbox_center_z ) / ( ( float )WIDTH * global_scale ) ) + 0.5f;

    uint8_t Y = min( max( ( int )( 0.299f * r[ i ] + 0.587f * g[ i ] + 0.114f * b[ i ] ), 0 ), 255 );
    uint8_t U = min( max( ( int )( -0.169f * r[ i ] - 0.331f * g[ i ] + 0.500f * b[ i ] + 128.0f ), 0 ), 255 );
    uint8_t V = min( max( ( int )( 0.500f * r[ i ] - 0.419f * g[ i ] - 0.081f * b[ i ] + 128.0f ), 0 ), 255 );

    for ( int face_id = 0; face_id < 6; face_id++ ) {
        float u = 0.0f, v = 0.0f, d = 0.0f;

        if ( face_id == 0 ) { u = nx; v = 1.0f - ny; d = nz; }
        else if ( face_id == 1 ) { u = 1.0f - nx; v = 1.0f - ny; d = 1.0f - nz; }
        else if ( face_id == 2 ) { u = nz; v = 1.0f - ny; d = 1.0f - nx; }
        else if ( face_id == 3 ) { u = 1.0f - nz; v = 1.0f - ny; d = nx; }
        else if ( face_id == 4 ) { u = nx; v = nz; d = ny; }
        else if ( face_id == 5 ) { u = nx; v = 1.0f - nz; d = 1.0f - ny; }

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

__global__ void init_i420_buffer_cuda( uint8_t* buffer ) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if ( idx < TOTAL_YUV_SIZE ) {
        if ( idx < SIZE_Y )
            buffer[ idx ] = 0;
        else
            buffer[ idx ] = 128;
    }
}

__global__ void pack_i420_stream_cuda( uint8_t* geo_y, uint8_t* occ_y, uint8_t* tex_y, uint8_t* tex_u, uint8_t* tex_v, uint8_t* out_buffer ) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int face_idx = blockIdx.z * blockDim.z + threadIdx.z;

    if ( x >= FACE_W_PADDED || y >= FACE_H_PADDED || face_idx >= 6 )
        return;

    int col = 0, row = 0;
    if ( face_idx == 0 ) { col = 0; row = 1; }
    else if ( face_idx == 1 ) { col = 2; row = 1; }
    else if ( face_idx == 2 ) { col = 3; row = 1; }
    else if ( face_idx == 3 ) { col = 1; row = 1; }
    else if ( face_idx == 4 ) { col = 1; row = 0; }
    else if ( face_idx == 5 ) { col = 1; row = 2; }

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
        out_buffer[ SIZE_Y + ( SIZE_Y / 4 ) + idx_uv ] = ( occ_y[ face_linear ] > 0 ) ? tex_v[ face_linear ] : 128;
    }
}

extern "C" void run_projection_pipeline( const float* pts_x, const float* pts_y, const float* pts_z, const uint8_t* c_r, const uint8_t* c_g, const uint8_t* c_b, uint32_t num_pts, float c_x, float c_y, float c_z, float extent_x, float extent_y, float extent_z, float final_scale, float yaw, float pitch, float cam_dist, uint8_t* out_yuv_buffer, double* gpu_metrics, float* out_global_scale, float* out_bbox_center_x, float* out_bbox_center_y, float* out_bbox_center_z, uint64_t* out_projection_end_cycles, uint64_t* out_encode_start_cycles, dpdk_poll_callback_t dpdk_poll_callback ) {
    ( void )extent_x;
    ( void )extent_y;
    ( void )extent_z;

    if ( num_pts == 0 ) {
        if ( out_global_scale ) *out_global_scale = 1.0f;
        if ( out_bbox_center_x ) *out_bbox_center_x = 0.0f;
        if ( out_bbox_center_y ) *out_bbox_center_y = 0.0f;
        if ( out_bbox_center_z ) *out_bbox_center_z = 0.0f;
        if ( gpu_metrics ) {
            gpu_metrics[ 0 ] = 0.0;
            gpu_metrics[ 1 ] = 0.0;
            gpu_metrics[ 2 ] = 0.0;
            gpu_metrics[ 3 ] = 0.0;
        }
        return;
    }

    cuda_memory_update( num_pts );

    cudaStream_t stream;
    CHECK_CUDA( cudaStreamCreate( &stream ) );

    cudaEvent_t start, h2d_done, kernel_done, pack_done, d2h_done;
    cudaEventCreate( &start );
    cudaEventCreate( &h2d_done );
    cudaEventCreate( &kernel_done );
    cudaEventCreate( &pack_done );
    cudaEventCreate( &d2h_done );

    cudaEventRecord( start, stream );

    for ( uint32_t offset = 0; offset < num_pts; offset += CHUNKING_SIZE ) {
        uint32_t cur_pts = ( offset + CHUNKING_SIZE > num_pts ) ? ( num_pts - offset ) : CHUNKING_SIZE;

        if ( dpdk_poll_callback != nullptr )
            dpdk_poll_callback();

        CHECK_CUDA( cudaMemcpyAsync( d_x + offset, pts_x + offset, cur_pts * sizeof( float ), cudaMemcpyHostToDevice, stream ) );
        CHECK_CUDA( cudaMemcpyAsync( d_y + offset, pts_y + offset, cur_pts * sizeof( float ), cudaMemcpyHostToDevice, stream ) );
        CHECK_CUDA( cudaMemcpyAsync( d_z + offset, pts_z + offset, cur_pts * sizeof( float ), cudaMemcpyHostToDevice, stream ) );
        CHECK_CUDA( cudaMemcpyAsync( d_r + offset, c_r + offset, cur_pts * sizeof( uint8_t ), cudaMemcpyHostToDevice, stream ) );
        CHECK_CUDA( cudaMemcpyAsync( d_g + offset, c_g + offset, cur_pts * sizeof( uint8_t ), cudaMemcpyHostToDevice, stream ) );
        CHECK_CUDA( cudaMemcpyAsync( d_b + offset, c_b + offset, cur_pts * sizeof( uint8_t ), cudaMemcpyHostToDevice, stream ) );
    }

    cudaEventRecord( h2d_done, stream );

    float initial_bbox_stats[ 6 ] = { FLT_MAX, FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX };
    CHECK_CUDA( cudaMemcpyAsync( d_bbox_stats, initial_bbox_stats, 6 * sizeof( float ), cudaMemcpyHostToDevice, stream ) );

    float cyaw = cosf( yaw );
    float syaw = sinf( yaw );
    float cpitch = cosf( pitch );
    float spitch = sinf( pitch );

    transform_points_cuda_kernel<<< ( num_pts + 255 ) / 256, 256, 0, stream >>>( d_x, d_y, d_z, d_tx, d_ty, d_tz, num_pts, c_x, c_y, c_z, final_scale, cyaw, syaw, cpitch, spitch, cam_dist );
    CHECK_CUDA( cudaGetLastError() );

    reduce_transformed_bbox_kernel<<< ( num_pts + 255 ) / 256, 256, 0, stream >>>( d_tx, d_ty, d_tz, num_pts, d_bbox_stats );
    CHECK_CUDA( cudaGetLastError() );

    float bbox_stats[ 6 ];
    CHECK_CUDA( cudaMemcpyAsync( bbox_stats, d_bbox_stats, 6 * sizeof( float ), cudaMemcpyDeviceToHost, stream ) );

    if ( dpdk_poll_callback != nullptr ) {
        while ( cudaStreamQuery( stream ) == cudaErrorNotReady )
            dpdk_poll_callback();
        CHECK_CUDA( cudaStreamQuery( stream ) );
    }
    else {
        CHECK_CUDA( cudaStreamSynchronize( stream ) );
    }

    float min_tx = bbox_stats[ 0 ];
    float min_ty = bbox_stats[ 1 ];
    float min_tz = bbox_stats[ 2 ];
    float max_tx = bbox_stats[ 3 ];
    float max_ty = bbox_stats[ 4 ];
    float max_tz = bbox_stats[ 5 ];

    float bbox_center_x = ( min_tx + max_tx ) * 0.5f;
    float bbox_center_y = ( min_ty + max_ty ) * 0.5f;
    float bbox_center_z = ( min_tz + max_tz ) * 0.5f;

    float transformed_extent_x = max_tx - min_tx;
    float transformed_extent_y = max_ty - min_ty;
    float transformed_extent_z = max_tz - min_tz;

    float scale_x = transformed_extent_x / ( float )WIDTH;
    float scale_y = transformed_extent_y / ( float )HEIGHT;
    float scale_z = transformed_extent_z / ( float )WIDTH;
    float global_scale = fmaxf( fmaxf( scale_x, scale_y ), scale_z ) * 1.10f;

    if ( !std::isfinite( global_scale ) || global_scale <= 0.0f )
        global_scale = 1.0f;

    if ( out_global_scale ) *out_global_scale = global_scale;
    if ( out_bbox_center_x ) *out_bbox_center_x = bbox_center_x;
    if ( out_bbox_center_y ) *out_bbox_center_y = bbox_center_y;
    if ( out_bbox_center_z ) *out_bbox_center_z = bbox_center_z;

    dim3 threads_reset( 8, 8, 4 );
    dim3 blocks_reset( ( FACE_W_PADDED + 7 ) / 8, ( FACE_H_PADDED + 7 ) / 8, ( 6 + 3 ) / 4 );

    reset_gbuffer_cuda_kernel<<< blocks_reset, threads_reset, 0, stream >>>( d_zbuf, d_geo, d_occ, d_tex_y, d_tex_u, d_tex_v );
    CHECK_CUDA( cudaGetLastError() );

    generate_gbuffer_cuda_kernel<<< ( num_pts + 255 ) / 256, 256, 0, stream >>>( d_tx, d_ty, d_tz, d_r, d_g, d_b, num_pts, bbox_center_x, bbox_center_y, bbox_center_z, global_scale, d_zbuf, d_geo, d_occ, d_tex_y, d_tex_u, d_tex_v );
    CHECK_CUDA( cudaGetLastError() );

    cudaEventRecord( kernel_done, stream );

    if ( out_encode_start_cycles )
        *out_encode_start_cycles = rte_get_timer_cycles();

    init_i420_buffer_cuda<<< ( TOTAL_YUV_SIZE + 255 ) / 256, 256, 0, stream >>>( d_out );
    CHECK_CUDA( cudaGetLastError() );

    pack_i420_stream_cuda<<< blocks_reset, threads_reset, 0, stream >>>( d_geo, d_occ, d_tex_y, d_tex_u, d_tex_v, d_out );
    CHECK_CUDA( cudaGetLastError() );

    cudaEventRecord( pack_done, stream );

    CHECK_CUDA( cudaMemcpyAsync( out_yuv_buffer, d_out, TOTAL_YUV_SIZE, cudaMemcpyDeviceToHost, stream ) );
    cudaEventRecord( d2h_done, stream );

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
        cudaEventElapsedTime( &ms, start, h2d_done ); gpu_metrics[ 0 ] = ms;
        cudaEventElapsedTime( &ms, h2d_done, kernel_done ); gpu_metrics[ 1 ] = ms;
        cudaEventElapsedTime( &ms, kernel_done, pack_done ); gpu_metrics[ 2 ] = ms;
        cudaEventElapsedTime( &ms, pack_done, d2h_done ); gpu_metrics[ 3 ] = ms;
    }

    cudaEventDestroy( start );
    cudaEventDestroy( h2d_done );
    cudaEventDestroy( kernel_done );
    cudaEventDestroy( pack_done );
    cudaEventDestroy( d2h_done );
    cudaStreamDestroy( stream );
}