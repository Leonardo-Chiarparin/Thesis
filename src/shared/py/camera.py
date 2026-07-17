import socket
import struct
import time
import os
import glob
import logging

# Setting logs
logging.basicConfig( level=logging.INFO, format="%(message)s" )
log = logging.getLogger( "Camera" )

# Network configuration
IP_ENCODER = "10.0.2.1" 
PORT_ENCODER = 8000

# 32-byte Header ( frame_id, timestamp, payload_size, 16-byte padding )
# Note: "!" forces the Network Byte Order to maintain cross-architecture compatibility
HEADER_FORMAT = "! I d I 16x"

# Data source & capture parameters
SEQUENCE_FOLDER = "/shared/data/loot/original" 
TARGET_FPS = 30.0
FRAME_TIME = 1.0 / TARGET_FPS
SKIP_FRAMES = 1

def camera_stream():

    # Purpose: It acts as the Volumetric Capture Node ingesting raw Point Cloud Data ( PCD ) from the file system.
    #          Moreover, the function encapsulates it with a custom 32-byte control header, and streams it to the
    #          desired micro-service via a reliable TCP connection.

    log.info( "[SYSTEM] Booting the \"Camera\"..." )

    # First, the component finds all ".ply" files in the given folder while sorting them
    ply_files = sorted( glob.glob( os.path.join( SEQUENCE_FOLDER, "*.ply" ) ) )
    
    if not ply_files:
        log.info( f"[SYSTEM] Error: No \".ply\" files discovered in {SEQUENCE_FOLDER}. Terminating..." )
        return

    log.info( f"[SYSTEM] Volumetric frames are equal to {len( ply_files )}." )

    sock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
    print( f"[SYSTEM] Establishing connection with the \"Encoder\" on TCP port {PORT_ENCODER}..." )
    
    while True:
        try:
            sock.connect( ( IP_ENCODER, PORT_ENCODER ) )
            break
        
        except ConnectionRefusedError:
            time.sleep( 1 )

    log.info( "[SYSTEM] Starting 3D continuous streaming sequence..." )
    log.info( "" )
    
    frame_id = 0

    try:
        while True:
            for file_path in ply_files:
                frame_id += 1
                start_time = time.time()
                
                # Then, it retrieves the current volumetric frame 
                with open( file_path, "rb" ) as file:
                    payload = file.read()
                
                camera_cycle_time = time.time() - start_time
                camera_fps = 1.0 / camera_cycle_time

                payload_size = len( payload )
                
                # Starting from it, a 32-byte header is created for synchronization and stream control
                header = struct.pack( HEADER_FORMAT, frame_id, start_time, payload_size )
                
                # Dispatching the encapsulated volumetric frame to the upstream processing node
                sock.sendall( header + payload )
                
                # Simulates the physical capture rate by rigorously accounting for the I/O processing time
                elapsed = time.time() - start_time
                sleep_time = FRAME_TIME - elapsed
                
                if sleep_time > 0:
                    time.sleep( sleep_time )

                total_cycle_time = time.time() - start_time
                actual_fps = 1.0 / total_cycle_time
                
                # Printing the result periodically to monitor throughput without clogging the terminal I/O
                if frame_id % SKIP_FRAMES == 0:
                    filename = os.path.basename( file_path )
                    log.info( f"[SYSTEM] Frame {frame_id} ( {filename} ), Payload: {payload_size / 1024:.2f} KB, Rate: [ {camera_fps:.2f} ( \"own\" ), {actual_fps:.2f} ( \"end-to-end\" ) ] FPS"  )
                
            log.info( "[SYSTEM] Sequence iteration completed. Restarting stream to simulate live capture all over..." )
            
    except KeyboardInterrupt:
        log.info( "\n[SYSTEM] Streaming gracefully interrupted..." )

    except BrokenPipeError:
        log.info( "\n[SYSTEM] Connection lost. \"Encoder\" disconnected unexpectedly." )

    finally:
        sock.close()

if __name__ == "__main__":
    camera_stream()