import socket
import struct
import time
import threading
import select
import logging

# Setting logs
logging.basicConfig( level=logging.INFO, format="%(message)s" )
log = logging.getLogger( "Transcoder" )

log.info( "[SYSTEM] Booting the \"Transcoder\"..." )

# Network configuration
IP_TRANSCODER = "10.0.3.1"
PORT_LUMINANCE = 5000
PORT_CHROMINANCE = 5001

IP_DECODER = "10.0.4.1"
PORT_DECODER = 6000 # Standard UDP ingestion port defined in the architecture

# Custom SFC Header ( vpcc_t ) structure matching the "Encoder" and P4 definition
HEADER_FORMAT = "! B H H H d d d d d" 
HEADER_SIZE = struct.calcsize( HEADER_FORMAT )

# Forwarding Header structure for the Decoder
# Format: ! B H d d d d d I 4s -> 1 byte ( type ) | 2 bytes ( frame_id ) | 8 byte ( timestamp ) | 8 byte ( s_global ) | 8 byte ( center_x ) | 8 byte ( center_y ) | 8 byte ( center_z ) | 4 byte ( frame_length ) | 4 byte ( magic_word )
DECODER_HEADER_FORMAT = "! B H d d d d d I 4s"
DECODER_HEADER_SIZE = struct.calcsize( DECODER_HEADER_FORMAT )

MAGIC_WORD = b"VPCC"

# Application parameters
FRAME_TIMEOUT = 0.2 # Maximum allowed time ( in seconds ) to wait for missing chunks before dropping the frame
STALE_TIMEOUT = 0.05
COMPLETED_TIMEOUT = 1.0

PAYLOAD_SIZE = 1472 # or 65507

PACKET_PACING = 0.002

# Global context ( for buffering and synchronization )
completed_frames = {}
frame_buffer = {}
buffer_lock = threading.Lock()

sock_out = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )

def garbage_collector():
    
    # Purpose: It represents an asynchronous thread designed to enforce the "Packet Washing" logic at the receiver side.
    #          The function periodically scans the buffer to identify and drop incomplete frames ( e.g., missing chunks 
    #          due to P4 In-Network congestion policies or OS tail-drops ), preventing memory leaks.
    
    while True:
        time.sleep( STALE_TIMEOUT )

        current_time = time.time()
        
        with buffer_lock:
            stale_frames = []
            
            for cache_key, frame_data in frame_buffer.items():
                if not frame_data[ "is_forwarding" ]:
                    if ( current_time - frame_data[ "timestamp" ] ) > FRAME_TIMEOUT:
                        stale_frames.append( cache_key )
            
            for cache_key in stale_frames:
                frame_type = frame_buffer[ cache_key ][ "type" ]
                frame_id = frame_buffer[ cache_key ][ "frame_id" ]
                total_chunks = frame_buffer[ cache_key ][ "total_chunks" ]
                chunks = len( frame_buffer[ cache_key ][ "chunks" ] )
                
                layer_name = "Core" if frame_type == 1 else "Enhancement"
                
                log.info( f"[SYSTEM] Corrupted frame: {frame_id} ( {layer_name} ). Missing chunks: {total_chunks - chunks}. Dropping frame..." )
                
                del frame_buffer[ cache_key ]

            expired = []

            for cache_key, timestamp in completed_frames.items():
                if current_time - timestamp > COMPLETED_TIMEOUT:
                    expired.append( cache_key )

            for cache_key in expired:
                del completed_frames[ cache_key ]

def process_chunk( data ):
    
    # Purpose: It extracts the custom header from the incoming UDP packet, registers the chunk in the structural
    #          buffer, and triggers the reassembly process if the integrity condition is met.
    
    if len( data ) < HEADER_SIZE:
        return
    
    header = data[ :HEADER_SIZE ]
    payload = data[ HEADER_SIZE: ]
    
    frame_type, frame_id, chunk_id, total_chunks, start_process, s_global, center_x, center_y, center_z = struct.unpack( HEADER_FORMAT, header )
    
    cache_key = f"{frame_id}_{frame_type}"

    with buffer_lock:
        if cache_key in completed_frames:
            return

        if cache_key in frame_buffer and frame_buffer[ cache_key ][ "is_forwarding" ]:
            return
        
        if cache_key not in frame_buffer:
            frame_buffer[ cache_key ] = {
                "type": frame_type,
                "frame_id": frame_id,
                "chunks": {},
                "total_chunks": total_chunks,
                "start_process": start_process,
                "s_global": s_global,
                "center_x": center_x,
                "center_y": center_y,
                "center_z": center_z,
                "timestamp": time.time(),
                "is_forwarding": False
            }

        frame_buffer[ cache_key ][ "chunks" ][ chunk_id ] = payload
        frame_buffer[ cache_key ][ "timestamp" ] = time.time()
        
        if len( frame_buffer[ cache_key ][ "chunks" ] ) == total_chunks:
            if not frame_buffer[ cache_key ][ "is_forwarding" ]:
                frame_buffer[ cache_key ][ "is_forwarding" ] = True
                threading.Thread( target=reassemble_and_forward, args=( cache_key,) ).start()

def reassemble_and_forward( cache_key ):
    
    # Purpose: It reconstructs the original compressed media payload by merging the chunks sequentially.
    #          Upon completion, the function applies a minimal synchronization header and forwards it to the "Decoder".
    
    frame_data = frame_buffer[ cache_key ]
    frame_id = frame_data[ "frame_id" ]
    frame_type = frame_data[ "type" ]
    frame_timestamp = frame_data[ "start_process" ]
    frame_s_global = frame_data[ "s_global" ]
    frame_center_x = frame_data[ "center_x" ]
    frame_center_y = frame_data[ "center_y" ]
    frame_center_z = frame_data[ "center_z" ]
    
    full_payload = bytearray()
    sorted_chunks_id = sorted( frame_data[ "chunks" ].keys() )
    
    # Sequential merging based on the "chunk_id" field
    for i in sorted_chunks_id:
        full_payload += frame_data[ "chunks" ][ i ]
        
    layer_name = "Core" if frame_type == 1 else "Enhancement"
    payload_size_kb = len( full_payload ) / 1024
    
    log.info( f"[SYSTEM] Frame {frame_id} ( {layer_name} ) successfully reconstructed ( Payload: {payload_size_kb:.2f} KB )." )
    
    # Dispatching frames to the "Decoder"
    try:
        header_out = struct.pack( DECODER_HEADER_FORMAT, frame_type, frame_id, frame_timestamp, frame_s_global, frame_center_x, frame_center_y, frame_center_z, len( full_payload ), MAGIC_WORD )
        max_payload = PAYLOAD_SIZE - DECODER_HEADER_SIZE

        first_packet = header_out + full_payload[ 0 : max_payload ]
        sock_out.sendto( first_packet, ( IP_DECODER, PORT_DECODER ) )

        time.sleep( PACKET_PACING )

        for i in range( max_payload, len( full_payload ), PAYLOAD_SIZE ):
            chunk_out = full_payload[ i : i + PAYLOAD_SIZE ]

            # Note: The reassembled packet is pushed to the "high-speed" UDP bridge on port 6000
            sock_out.sendto( chunk_out, ( IP_DECODER, PORT_DECODER ) )

            # Note: A minimal pacing is recommended to prevent bursts from overwhelming the network interface queues.
            time.sleep( PACKET_PACING )
        
    except Exception as e:
        log.info( f"[SYSTEM] Forwarding Exception on frame {frame_id}: {e}" )

    finally:
        with buffer_lock:
            completed_frames[ cache_key ] = time.time()
            del frame_buffer[ cache_key ]

def start_transcoder():
    
    # Purpose: It initializes the non-blocking multiplexing loop listening on the parallel "Geometry" and "Texture" channels.
    
    sock_luminance = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
    sock_luminance.bind( ( IP_TRANSCODER, PORT_LUMINANCE ) )
    
    sock_chrominance = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
    sock_chrominance.bind( ( IP_TRANSCODER, PORT_CHROMINANCE ) )
    
    log.info( f"[SYSTEM] Listening for \"Geometry\" on UDP port {PORT_LUMINANCE}..." )
    log.info( f"[SYSTEM] Listening for \"Texture\" on UDP port {PORT_CHROMINANCE}..." )
    log.info( "" )
    
    inputs = [ sock_luminance, sock_chrominance ]
    
    try:
        while True:
            # Note: "select.select" monitors multiple sockets concurrently without interrupting CPU threads
            readable, _, _ = select.select( inputs, [], [] )
            
            for sock in readable:
                data, _ = sock.recvfrom( 65536 )
                process_chunk( data )
                
    except KeyboardInterrupt:
        log.info( "\n[SYSTEM] Transcoder shutdown initiated..." )
        
    finally:
        sock_luminance.close()
        sock_chrominance.close()
        sock_out.close()

if __name__ == "__main__":
    # Booting the asynchronous garbage collector
    threading.Thread( target=garbage_collector, daemon=True ).start()
    
    # Starting the main ingestion loop
    start_transcoder()