import os

os.environ[ "QT_QPA_PLATFORM" ] = "offscreen"
os.environ[ "PYOPENGL_PLATFORM" ] = "egl"

import socket
import struct
import tempfile
import time
import numpy as np
import threading
import logging
import math

# Setting logs
logging.basicConfig( level=logging.INFO, format="%(message)s" )
log = logging.getLogger( "Encoder" )

log.info( "[SYSTEM] Booting the \"Encoder\"..." )

import cv2
import open3d as o3d

# Network configuration
IP_ENCODER = "10.0.2.1"
PORT_CAMERA = 8000

IP_TRANSCODER = "10.0.3.1"
PORT_LUMINANCE = 5000
PORT_CHROMINANCE = 5001

# Application parameters
HEADER_FORMAT = "! I d I 16x" # Note: "!" forces the "Network Byte Order" for cross-platform compatibility
HEADER_SIZE = struct.calcsize( HEADER_FORMAT )
WIDTH, HEIGHT = 640, 480 # Target G-Buffer resolution
GAMMA = 1.1 # Padding factor to prevent spatial clipping

TRANSCODER_HEADER_FORMAT = "! B H H H d d d d d"
TRANSCODER_HEADER_SIZE = struct.calcsize( TRANSCODER_HEADER_FORMAT )

# Global context
PORT_TELEMETRY = 8080
network_allows_chrominance = threading.Event()
network_allows_chrominance.set()

current_network_bitrate = 50.0  # Default startup bitrate
MAX_BITRATE = 50.0 # Mbps
MIN_BITRATE = 5.0 # Mbps

MAX_QUALITY = 80
MIN_QUALITY = 32

SOCK_UDP = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
total_raw_bytes_received = 0
video_writer_color = None
video_writer_depth = None

PAYLOAD_SIZE = 1472

PACKET_PACING = 0.002

def telemetry_listener():
    
    # Purpose: It listens for explicit congestion notifications from the SDN Control Plane ( "sff1.py" ) to perform application-level bandwidth adaptation.
    
    global current_network_bitrate, network_allows_chrominance

    sock_telemetry = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
    sock_telemetry.bind( ( IP_ENCODER, PORT_TELEMETRY ) )
    
    log.info( f"[SYSTEM] Telemetry active on UDP port {PORT_TELEMETRY}." )
    
    while True:
        try:
            data, _ = sock_telemetry.recvfrom( 1024 )
            message = data.decode( "utf-8" ).strip()
            
            if message.startswith( "BITRATE:" ):
                bitrate = float( message.split( ":" )[ 1 ] )
                current_network_bitrate = bitrate
                
                if bitrate <= MIN_BITRATE:
                    if network_allows_chrominance.is_set():
                        network_allows_chrominance.clear()
                        log.info( f"[SYSTEM] Congestion ( {bitrate} Mbps ). Discarding \"Enhancement\", while scaling \"Core\" quality..." )
                        log.info( "" )

                else:
                    if not network_allows_chrominance.is_set():
                        network_allows_chrominance.set()

                    log.info( f"[SYSTEM] Network capacity at {bitrate} Mbps. Rearranging dynamic encoding quality..." )
                    log.info( "" )

        except Exception as e:
            log.error( f"[SYSTEM] Telemetry Exception: {e}" )
            log.info( "" )

def generate_projections( colors, points, s_global ):
    
    # Purpose: It applies orthographic projection mapping continuous 3D coordinates onto discrete 2D planes ( G-Buffers ).
    
    x, y, z = points[ :, 0 ], points[ :, 1 ], points[ :, 2 ]
    
    # 1) Mathematical rotations for the 6 faces ( u_proj, v_proj, depth )
    faces = {
        "FRONT":  ( x, y, z ),
        "BACK":   ( -x, y, -z ),
        "RIGHT":  ( -z, y, x ),
        "LEFT":   ( z, y, -x ),
        "TOP":    ( x, -z, y ),
        "BOTTOM": ( x, z, -y )
    }
    
    pixel_data = {}
    
    # 2) Spatial quantization from 3D space to pixels
    for face_name, ( u_proj, v_proj, depth ) in faces.items():
        # First, we provide a translation to the center of the image ( WIDTH/2, HEIGHT/2 ) after using "s_global"
        u_pixel = np.round( ( u_proj / s_global ) + ( WIDTH / 2 ) ).astype( int )
        v_pixel = np.round( ( v_proj / s_global ) + ( HEIGHT / 2 ) ).astype( int )
        
        # Clamping to guarantee that pixels don't go outside the element
        u_pixel = np.clip( u_pixel, 0, WIDTH - 1 )
        v_pixel = np.clip( v_pixel, 0, HEIGHT - 1 )

        # Then, the resulting values are saved for the next step
        pixel_data[ face_name ] = np.column_stack( ( u_pixel, v_pixel, depth, colors ) )
        
    return pixel_data

def onion_peeling( pixel_data ):

    # Purpose: It extracts visibility layers representing the object's spatial shell.
    #          The function integrates dynamic computational offloading based on network status.
    
    video_frames = {}
    
    for face_name, data in pixel_data.items():
        u = data[ :, 0 ].astype( int )
        v = data[ :, 1 ].astype( int )
        depth = data[ :, 2 ]
        colors = data[ :, 3:6 ]
        
        # 1) Converting 2D coordinates to 1D linear index for quick sorting
        linear_index = v * WIDTH + u
        
        # 2) We sort by linear index and then by depth; "lexsort" arranges first by the last key ( "linear_index" ), then by the first ( "depth" )
        sort_order = np.lexsort( ( depth, linear_index ) )
        sorted_index = linear_index[ sort_order ]
        sorted_depth = depth[ sort_order ]
        sorted_colors = colors[ sort_order ]
        
        # 3) "Layer 0" extraction by taking the first occurrence ( the point with minimum depth ) for each pixel
        _, layer_0_mask = np.unique( sorted_index, return_index=True )
        layer_0_index = sorted_index[ layer_0_mask ]
        layer_0_depth = sorted_depth[ layer_0_mask ]
        layer_0_colors = sorted_colors[ layer_0_mask ]

        # Note: A depth normalization ( 0 - 255 ) is applied by finding the minimum and maximum of the face to correctly map its shape
        min_depth, max_depth = np.min( depth ), np.max( depth )
        range_depth = max_depth - min_depth if max_depth > min_depth else 1

        # Generating 8-bit images, where "0" represents a dark background
        image_layer_0_depth = np.zeros( ( HEIGHT, WIDTH ), dtype=np.uint8 )
        image_layer_0_color = np.zeros( ( HEIGHT, WIDTH, 3 ), dtype=np.uint8 )
        
        # Rendering depth for "L0"
        image_layer_0_depth.flat[ layer_0_index ] = ( ( layer_0_depth - min_depth ) / range_depth * 255 ).astype( np.uint8 )
        
        # Defining colors for "L0" ( such elements regarding "open3d" are 0.0 - 1.0, then we normalize them into 0 - 255 )
        image_layer_0_color.reshape( -1, 3 )[ layer_0_index ] = ( layer_0_colors * 255 ).astype( np.uint8 )
        
        # Note: "Layer 1" is mathematically extracted only if the network allows it.
        # This behavior addresses the computational bottleneck during network congestion.
        image_layer_1_depth = np.zeros( ( HEIGHT, WIDTH ), dtype=np.uint8 )
        image_layer_1_color = np.zeros( ( HEIGHT, WIDTH, 3 ), dtype=np.uint8 )

        if network_allows_chrominance.is_set():
            # 4) "Layer 1" extraction removing the points from "Layer 0" 
            remaining_mask = np.ones( len( sorted_index ), dtype=bool )
            remaining_mask[ layer_0_mask ] = False
            
            layer_1_index_remaining = sorted_index[ remaining_mask ]
            layer_1_depth_remaining = sorted_depth[ remaining_mask ]
            layer_1_colors_remaining = sorted_colors[ remaining_mask ]
            
            _, layer_1_mask = np.unique( layer_1_index_remaining, return_index=True )
            layer_1_index = layer_1_index_remaining [ layer_1_mask ]
            layer_1_depth = layer_1_depth_remaining[ layer_1_mask ]
            layer_1_colors = layer_1_colors_remaining[ layer_1_mask ]

            image_layer_1_depth = np.zeros( ( HEIGHT, WIDTH ), dtype=np.uint8 )
            image_layer_1_color = np.zeros( ( HEIGHT, WIDTH, 3 ), dtype=np.uint8 )

            image_layer_1_depth.flat[ layer_1_index ] = ( ( layer_1_depth - min_depth ) / range_depth * 255 ).astype( np.uint8 )
            image_layer_1_color.reshape( -1, 3 )[ layer_1_index ] = ( layer_1_colors * 255 ).astype( np.uint8 )      
        
        video_frames[ face_name ] = {
            "L0_DEPTH": image_layer_0_depth, "L1_DEPTH": image_layer_1_depth,
            "L0_COLOR": image_layer_0_color, "L1_COLOR": image_layer_1_color
        }
        
    return video_frames

def generate_atlas( video_frames ):

    # Purpose: It aggregates multiple 2D patches into a unified mosaic ( Super-Frame ).

    atlas_width, atlas_height = WIDTH * 3, HEIGHT * 2
    layout = [ 
        [ "FRONT", "RIGHT", "TOP" ], 
        [ "BACK", "LEFT", "BOTTOM" ] 
    ]
    
    atlas_layer_0_color = np.zeros( ( atlas_height, atlas_width, 3 ), dtype=np.uint8 )
    atlas_layer_0_depth = np.zeros( ( atlas_height, atlas_width ), dtype=np.uint8 )
    
    for row in range( 2 ):
        for column in range( 3 ):
            face = layout[ row ][ column ]

            x_start, x_end = column * WIDTH, ( column + 1 ) * WIDTH
            y_start, y_end = row * HEIGHT, ( row + 1 ) * HEIGHT

            atlas_layer_0_color[ y_start:y_end, x_start:x_end ] = video_frames[ face ][ "L0_COLOR" ]
            atlas_layer_0_depth[ y_start:y_end, x_start:x_end ] = video_frames[ face ][ "L0_DEPTH" ]
            
    return atlas_layer_0_color, atlas_layer_0_depth

def stream_to_h264( atlas_color, atlas_depth, save_to_disk=0 ):
    
    # Purpose: It compresses the spatial and photometric domains using H.264 intra-frame logic.
    
    global current_network_bitrate, network_allows_chrominance, video_writer_color, video_writer_depth
    
    height, width = atlas_color.shape[ :2 ]

    bitrate = max( MIN_BITRATE, min( MAX_BITRATE, current_network_bitrate ) )
    alpha = ( bitrate - MIN_BITRATE ) / ( MAX_BITRATE - MIN_BITRATE )

    quality_luminance = int( MIN_QUALITY + ( alpha * ( MAX_QUALITY - MIN_QUALITY ) ) )
    quality_chrominance = int( alpha * MAX_QUALITY )

    # 1) Calculating metrics in RAM ( always done ). 
    # Note: we encode them in memory to know exactly how many bytes the compressed frame occupies.
    encode_parameters_core = [ int( cv2.IMWRITE_JPEG_QUALITY ), quality_luminance ]

    _, buffer_depth = cv2.imencode( ".jpg", atlas_depth, encode_parameters_core )
    bytes_depth = buffer_depth.tobytes()
    
    if quality_chrominance > 0 and network_allows_chrominance.is_set():
        encode_parameters_enhancement = [ int( cv2.IMWRITE_JPEG_QUALITY ), quality_chrominance ]
        _, buffer_color = cv2.imencode( ".jpg", atlas_color, encode_parameters_enhancement )
        bytes_color = buffer_color.tobytes()
    else:
        bytes_color = b"" # No CPU waste to compress data that will be discarded
    
    if save_to_disk == 1:
        # "Lazy" setting when obtaining the starting frame
        if video_writer_color is None:
            log.info( "[SYSTEM] Starting the H.264 ( MPEG-4 AVC ) codec..." )
            
            fourcc = cv2.VideoWriter_fourcc( *"avc1" ) # Note: "avc1" is the "FourCC" standard about "H.264"
            fps = 30.0
            
            # We physically create the video files ( e.g. in the "shared" folder so we can export them )
            video_writer_color = cv2.VideoWriter( "/shared/vpcc_color.mp4", fourcc, fps, ( width, height ), True )
            video_writer_depth = cv2.VideoWriter( "/shared/vpcc_depth.mp4", fourcc, fps, ( width, height ), True )

        # 2) Color writing ( already with 3 channesl )
        video_writer_color.write( atlas_color )
        
        # 3) Depth writing; we convert the grayscale to BGR
        atlas_depth_3_channel = cv2.cvtColor( atlas_depth, cv2.COLOR_GRAY2BGR )
        video_writer_depth.write( atlas_depth_3_channel )

    return bytes_depth, bytes_color

def process_frame( payload, frame_id ):

    # Purpose: It represents the main pipeline execution. 
    #          The function handles ingestion, projection, encoding, and custom SFC header encapsulation.

    global total_raw_bytes_received

    original_size = len( payload )
    total_raw_bytes_received += original_size
    
    # We temporarily save the payload for "open3d" to read it
    with tempfile.NamedTemporaryFile( suffix=".ply", delete=False ) as temp:
        temp.write( payload )
        temp_name = temp.name

    try:
        point_cloud = o3d.io.read_point_cloud( temp_name )
        points = np.asarray( point_cloud.points )
        num_points = len( points )
        
        if num_points == 0:
            return
        
        # Retrieving colors
        if len( point_cloud.colors ) > 0:
            colors = np.asarray( point_cloud.colors )
        else:
            colors = np.ones_like( points ) # Fallback to white if no color exists

        # 1) Computing the center
        center = np.mean( points, axis=0 )
        
        # 2) Translation invariance: p'_i = p_i - center
        point_cloud.translate( -center )
        points = np.asarray( point_cloud.points )
        
        # 3) Definition of the adaptive bounding volume ( dual-domain )
        bounding_box = point_cloud.get_axis_aligned_bounding_box()
        extents = bounding_box.get_extent() # ( Ex, Ey, Ez )
        
        padded_extents = extents * GAMMA
        
        # Rigorous unique scale ( "s_global" ) calculation
        s_global = max( padded_extents[ 0 ] / WIDTH, padded_extents[ 1 ] / HEIGHT, padded_extents[ 2 ] / WIDTH )
        
        extents_int = np.round( extents ).astype( int )

        log.info( f"[SYSTEM] Frame: {frame_id}" ) 
        log.info( f"[SYSTEM] Points: {num_points}" )
        log.info( f"[SYSTEM] Center: [ {center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f} ]" )
        log.info( f"[SYSTEM] Extents ( Ex, Ey, Ez ): [ {extents_int[ 0 ]}, {extents_int[ 1 ]}, {extents_int[ 2 ]} ]" )
        log.info( f"[SYSTEM] S_Global: {s_global:.4f}" )

        # V-PCC Pipeline
        start_process = time.time()
        pixel_data = generate_projections( colors, points, s_global )

        video_frames = onion_peeling( pixel_data )

        atlas_color, atlas_depth = generate_atlas( video_frames )

        # "H.264" Compression
        bytes_luminance, bytes_chrominance = stream_to_h264( atlas_color, atlas_depth )
        
        # SFC Classifier logic: Building the custom NSH ( vpcc_t ). 
        # Format: ! B H H H d d d d d -> 1 byte ( type ) | 2 bytes ( frame_id ) | 2 byte ( chunk_id ) | 2 byte ( total_chunks ) | 8 byte ( timestamp ) | 8 byte ( s_global ) | 8 byte ( center_x ) | 8 byte ( center_y ) | 8 byte ( center_z ), where
        # type = 1: "Core" Geometry ( must be preserved )
        # type = 2: Photometric "Enhancement" ( subject to Packet Washing )
        chunk_id = 0
        max_payload = PAYLOAD_SIZE - TRANSCODER_HEADER_SIZE
        total_chunks_luminance = math.ceil( len( bytes_luminance ) / max_payload )

        # Dispatching "Luminance" stream
        for i in range( 0, len( bytes_luminance ), max_payload ):
            header_luminance = struct.pack( TRANSCODER_HEADER_FORMAT, 1, frame_id % 65535, chunk_id, total_chunks_luminance, start_process, s_global, center[ 0 ], center[ 1 ], center[ 2 ] )

            chunk = bytes_luminance[ i : i + max_payload ]
            payload_luminance = header_luminance + chunk

            SOCK_UDP.sendto( payload_luminance, ( IP_TRANSCODER, PORT_LUMINANCE ) )

            chunk_id = chunk_id + 1

            time.sleep( PACKET_PACING )

        # Sending the "Chrominance" part ( depend on the network status )
        if network_allows_chrominance.is_set():
            chunk_id = 0
            total_chunks_chrominance = math.ceil( len( bytes_chrominance ) / max_payload )

            for i in range( 0, len( bytes_chrominance ), max_payload ):
                header_chrominance = struct.pack( TRANSCODER_HEADER_FORMAT, 2, frame_id % 65535, chunk_id, total_chunks_chrominance, start_process, s_global, center[ 0 ], center[ 1 ], center[ 2 ] )

                chunk = bytes_chrominance[ i : i + max_payload ]
                payload_chrominance = header_chrominance + chunk

                SOCK_UDP.sendto( payload_chrominance, ( IP_TRANSCODER, PORT_CHROMINANCE ) )

                chunk_id = chunk_id + 1

                time.sleep( PACKET_PACING )

        # Calculating metrics for logs
        process_time = ( time.time() - start_process ) * 1000
        compressed_size = len( bytes_luminance ) + len( bytes_chrominance )
        
        log.info( f"[SYSTEM] Traffic: [ Original ( In ): {original_size / 1024:.2f} KB, Compressed ( Out ): {compressed_size / 1024:.2f} KB ( Core: {len( bytes_luminance ) / 1024:.2f} KB, Enhancement: {len( bytes_chrominance ) / 1024:.2f} KB ) ]" )
        log.info( f"[SYSTEM] Completion Time: {process_time:.2f} ms" )

        log.info( "" )
        
    finally:
        os.remove( temp_name )

def start_encoder():

    # Purpose: It starts the TCP ingestion server to receive raw point clouds from the "Camera".

    global video_writer_color, video_writer_depth, total_raw_bytes_received

    sock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
    sock.bind( ( IP_ENCODER, PORT_CAMERA ) )
    sock.listen( 1 )
    
    log.info( f"[SYSTEM] Listening on {IP_ENCODER}:{PORT_CAMERA}..." )

    try:
        while True:
            conn, _ = sock.accept()
            buffer = bytearray()

            log.info( f"[SYSTEM] TCP connection established with the \"Camera\"." )
            log.info( "" )
            
            try:
                while True:
                    data = conn.recv( 65536 )

                    if not data:
                        break
                    
                    buffer += data
                    
                    # 32-byte Header-Based Extraction
                    while len( buffer ) >= HEADER_SIZE:
                        header = buffer[ :HEADER_SIZE ]
                        frame_id, timestamp, payload_size = struct.unpack( HEADER_FORMAT, header )
                        
                        total_packet_size = HEADER_SIZE + payload_size
                        
                        if len( buffer ) >= total_packet_size:
                            payload = buffer[ HEADER_SIZE:total_packet_size ]
                            del buffer[ :total_packet_size ] # Remove it from the buffer
                            
                            process_frame( payload, frame_id )

                        else:
                            # Waiting for more data
                            break

            except Exception as e:
                log.info( f"[SYSTEM] Connection interrupted: {e}" )

            finally:
                if video_writer_color is not None: 
                    video_writer_color.release()
                    video_writer_color = None
                
                if video_writer_depth is not None: 
                    video_writer_depth.release()
                    video_writer_depth = None

                if os.path.exists( "/shared/vpcc_color.mp4" ) and os.path.exists( "/shared/vpcc_depth.mp4" ):
                    size_color = os.path.getsize( "/shared/vpcc_color.mp4" )
                    size_depth = os.path.getsize( "/shared/vpcc_depth.mp4" )
                    total_compressed = size_color + size_depth
                    
                    if total_raw_bytes_received > 0:
                        log.info( "" )
                        log.info( f"VIDEO File: {total_compressed / 1024:.2f} KB" )
                        log.info( "" )
                    
                conn.close()
                log.info( "[SYSTEM] Client disconnected. Waiting for new interactions..." )
                    
    except KeyboardInterrupt:
        log.info( "\n[SYSTEM] Encoder shutdown initiated..." )

    finally:
        sock.close()

if __name__ == "__main__":
    threading.Thread( target=telemetry_listener, daemon=True ).start()

    start_encoder()