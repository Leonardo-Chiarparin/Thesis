import os
import logging

os.environ[ "QT_QPA_PLATFORM" ] = "offscreen"
os.environ[ "PYOPENGL_PLATFORM" ] = "egl"

# Setting logs
logging.basicConfig( level=logging.INFO, format="%(message)s" )
log = logging.getLogger( "Decoder" )

log.info( "[SYSTEM] Booting the \"Decoder\"..." )

import socket
import struct
import time
import threading
import copy
import queue
import numpy as np
import csv
import cv2
import open3d as o3d

# Network configuration
IP_DECODER = "10.0.4.1"
PORT_DECODER = 6000

# Application parameters
DECODER_HEADER_FORMAT = "! B H d d d d d I 4s"
DECODER_HEADER_SIZE = struct.calcsize( DECODER_HEADER_FORMAT )

MAGIC_WORD = b"VPCC"

WIDTH, HEIGHT = 640, 480 # Target G-Buffer resolution

# Global context
current_point_cloud = o3d.geometry.PointCloud()
rendering_lock = threading.Lock()

file_queue = queue.Queue()

def disk_writer_worker():

    # Purpose:  It represents a background process saving ".ply" files without slowing network I/O.
    
    while True:
        filename, point_cloud_data = file_queue.get()
        
        o3d.io.write_point_cloud( filename, point_cloud_data )
        
        file_queue.task_done()

def back_projection( atlas_depth, atlas_color, s_global, center ):

    # Purpose: It executes the "Inverse Orthogonal Projection" framework, mapping 
    #          2D normalized depth pixels back into the metric 3D domain.

    # 1) Fixed-Layout Mosaic slicing ( 3x4 grid mapping )
    faces = {
        "FRONT":  ( 0, 0 ),
        "RIGHT":  ( 0, 1 ),
        "TOP":    ( 0, 2 ),
        "BACK":   ( 1, 0 ),
        "LEFT":   ( 1, 1 ),
        "BOTTOM": ( 1, 2 )
    }
    
    all_points = []
    all_colors = []

    for face, ( row, col ) in faces.items():
        x_start, x_end = col * WIDTH, ( col + 1 ) * WIDTH
        y_start, y_end = row * HEIGHT, ( row + 1 ) * HEIGHT
        
        depth_patch = atlas_depth[ y_start:y_end, x_start:x_end ]
        color_patch = atlas_color[ y_start:y_end, x_start:x_end ] if atlas_color is not None else None
        
        # 2) Morphological erosion and Visibility mask ( Occupancy )
        active_pixels = depth_patch > 0
        v_coordinates, u_coordinates = np.where( active_pixels )
        depth_values = depth_patch[ active_pixels ].astype( float ) / 255.0
        
        if len( u_coordinates ) == 0:
            continue
            
        # 3) Mathematical inverse operations aligning pixels
        u_projection = ( u_coordinates - ( WIDTH / 2 ) ) * s_global
        v_projection = ( v_coordinates - ( HEIGHT / 2 ) ) * s_global
    
        depth_projection = depth_values
        
        # 4) Axis permutations derived from the orthogonal model
        if face == "FRONT":
            x, y, z = u_projection, v_projection, depth_projection
        elif face == "BACK":
            x, y, z = -u_projection, v_projection, -depth_projection
        elif face == "RIGHT":
            x, y, z = depth_projection, v_projection, -u_projection
        elif face == "LEFT":
            x, y, z = -depth_projection, v_projection, u_projection
        elif face == "TOP":
            x, y, z = u_projection, depth_projection, -v_projection
        elif face == "BOTTOM":
            x, y, z = u_projection, -depth_projection, v_projection
            
        points_3d = np.column_stack( ( x, y, z ) )
        all_points.append( points_3d )
        
        if color_patch is not None:
            colors_extracted = color_patch[ active_pixels ].astype( float ) / 255.0
            all_colors.append( colors_extracted )

    if not all_points:
        return np.empty( ( 0, 3 ) ), np.empty( ( 0, 3 ) )

    reconstructed_points = np.vstack( all_points )
    
    # 5) Object-centric restoration ( translating back to original coordinates )
    reconstructed_points += center

    reconstructed_colors = np.vstack( all_colors ) if all_colors else np.empty( ( 0, 3 ) )

    return reconstructed_points, reconstructed_colors

def start_decoder():

    # Purpose: It handles UDP ingestion, applying JPEG/HEVC decoding, generating QoE metrics,
    #          and passing data to the volumetric reconstruction engine.

    global current_point_cloud

    # Telemetry CSV initialization for offline "Super-Merge"
    os.makedirs( "/shared/csv", exist_ok=True )
    csv_file = open( "/shared/csv/decoder.csv", "w", newline="" )
    csv_writer = csv.writer( csv_file )
    csv_writer.writerow( [ "frame_id", "layer", "latency_ms", "points_count" ] )

    sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
    sock.bind( ( IP_DECODER, PORT_DECODER ) )
    
    log.info( f"[SYSTEM] Listening for \"Transcoder\" packets on UDP port {PORT_DECODER}..." )
    log.info( "" )

    buffer = bytearray()

    expected_frame_id = -1
    expected_frame_length = 0

    timestamp, s_global, center_x, center_y, center_z = 0, 0, 0, 0, 0

    try:
        while True:
            data, _ = sock.recvfrom( 65536 )

            is_new_frame = False

            if len( data ) >= DECODER_HEADER_SIZE:
                try:
                    header = data[ :DECODER_HEADER_SIZE ]
                    unpacked = struct.unpack( DECODER_HEADER_FORMAT, header )

                    magic_word = unpacked[ 8 ]

                    if magic_word == MAGIC_WORD:
                        frame_type, frame_id, timestamp, s_global, center_x, center_y, center_z, frame_length, _ = struct.unpack( DECODER_HEADER_FORMAT, header )

                        if expected_frame_length > 0 and len( buffer ) != expected_frame_length:
                            missing_bytes = expected_frame_length - len( buffer )
                            log.warning( f"[SYSTEM] Frame {expected_frame_id} dropped ( Missing: {missing_bytes} bytes )" )
                            
                        buffer = bytearray( data[ DECODER_HEADER_SIZE: ] )
                        expected_frame_id = frame_id
                        expected_frame_length = frame_length
                        is_new_frame = True
                
                except struct.error:
                    pass
            
            if not is_new_frame and expected_frame_length > 0:
                buffer += data

            # Decoding attempt ( using "FFmpeg" extraction )
            if expected_frame_length > 0 and len( buffer ) >= expected_frame_length:
                frame_array = np.frombuffer( buffer[ :expected_frame_length ], dtype=np.uint8 )
                decoded_image = cv2.imdecode( frame_array, cv2.IMREAD_UNCHANGED )
                
                if decoded_image is not None:
                    latency = ( time.time() - timestamp ) * 1000
                    is_enhancement = len( decoded_image.shape ) == 3
                    
                    if not is_enhancement:
                        atlas_depth = decoded_image
                        atlas_color = None
                        layer_name = "Core"
                    else:
                        atlas_depth = cv2.cvtColor( decoded_image, cv2.COLOR_BGR2GRAY )
                        atlas_color = decoded_image
                        layer_name = "Enhancement"
                        
                    center = np.array( [ center_x, center_y, center_z ] )
                    points, colors = back_projection( atlas_depth, atlas_color, s_global, center )
                    
                    number_points = len( points )

                    log.info( f"[SYSTEM] Frame {expected_frame_id} ( {layer_name} ) successfully restored." )
                    log.info( f"[SYSTEM] Command-to-Photon Latency: {latency:.2f} ms" )
                    log.info( f"[SYSTEM] Points: {number_points}" )
                    log.info( "" )
                    
                    csv_writer.writerow( [ expected_frame_id, layer_name, f"{latency:.2f}", number_points ] )
                    csv_file.flush()
                    
                    with rendering_lock:
                        current_point_cloud.points = o3d.utility.Vector3dVector( points )

                        if len( colors ) > 0:
                            current_point_cloud.colors = o3d.utility.Vector3dVector( colors )

                        out_dir = "/shared/data/loot/reconstructed"
                        os.makedirs( out_dir, exist_ok=True )

                        point_cloud_copy = o3d.geometry.PointCloud( current_point_cloud )
                        file_queue.put( ( f"{out_dir}/loot_vox10_{expected_frame_id + 999 }.ply", point_cloud_copy ) )
                
                buffer = buffer[ expected_frame_length: ]
                expected_frame_length = 0

    finally:
        sock.close()

if __name__ == "__main__":
    threading.Thread( target=disk_writer_worker, daemon=True ).start()

    # Starting the network ingestion loop
    network_thread = threading.Thread( target=start_decoder, daemon=True )
    network_thread.start()
    
    try:
        while network_thread.is_alive():
            time.sleep( 1.0 ) 

    except KeyboardInterrupt:
        log.info( "\n[SYSTEM] Decoder shutdown initiated by \"User\"..." )