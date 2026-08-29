import argparse
import asyncio
import json
import mmap
import os
import struct
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import websockets

FRAME_PATH = "/dev/shm/frame.bin"
CTRL_PATH = "/dev/shm/ctrl.bin"
EOS_PATH = "/tmp/sfc-user-eos"

WEB_FORMAT = "<QIIIIIIIHHfffIdd"
WEB_SIZE = struct.calcsize( WEB_FORMAT )
POINT_SIZE = 16
CTRL_SIZE = 56
CMD_TYPE_POSE = 1

frame_map = None
ctrl_map = None
ctrl_lock = asyncio.Lock()
clients = set()

class web_peer:
    def __init__( self, socket ):

        # Purpose: It establishes the client-specific queue & sender reference for active browser connections
        
        self.socket = socket
        self.queue = asyncio.Queue( maxsize = 1 )
        self.sender = None
        self.frame_ready = asyncio.Event()
        self.frame_ready.set()

async def open_maps() -> None:

    # Purpose: It defines non-blocking access to the shared-memory segments utilized for cross-process "DPDK" communication

    global frame_map, ctrl_map

    while frame_map is None or ctrl_map is None:
        if frame_map is None and os.path.exists( FRAME_PATH ):
            frame_fd = os.open( FRAME_PATH, os.O_RDWR )
            frame_map = mmap.mmap( frame_fd, 0, access = mmap.ACCESS_READ )
            os.close( frame_fd )

        if ctrl_map is None and os.path.exists( CTRL_PATH ):
            ctrl_fd = os.open( CTRL_PATH, os.O_RDWR )
            ctrl_map = mmap.mmap( ctrl_fd, CTRL_SIZE, access = mmap.ACCESS_WRITE )
            os.close( ctrl_fd )

        if frame_map is None or ctrl_map is None:
            await asyncio.sleep( 0.05 )

def read_frame_seq() -> int:

    # Purpose: It observes the element sequence marker without copying the complete point-cloud payload

    if frame_map is None:
        return 0

    seq = struct.unpack_from( "<Q", frame_map, 0 )[ 0 ]

    if seq == 0 or ( seq & 1 ) != 0:
        return 0

    return seq

def read_frame( last_seq: int = 0 ) -> tuple:

    # Purpose: It validates the sequence markers & safely extracts the active frame snapshot originating from the "C" application

    if frame_map is None:
        return None

    seq_a = struct.unpack_from( "<Q", frame_map, 0 )[ 0 ]

    if seq_a == 0 or ( seq_a & 1 ) != 0 or seq_a == last_seq:
        return None

    values = struct.unpack_from( WEB_FORMAT, frame_map, 0 )
    point_count = values[ 2 ]
    frame_size = WEB_SIZE + point_count * POINT_SIZE

    if frame_size > len( frame_map ):
        return None

    payload = bytes( frame_map[ :frame_size ] )
    seq_b = struct.unpack_from( "<Q", frame_map, 0 )[ 0 ]

    if seq_a != seq_b or ( seq_b & 1 ) != 0:
        return None

    return seq_b, payload

async def send_loop( peer: web_peer ) -> None:

    # Purpose: It defines the asynchronous consumer loop tasked with flushing queued binary shots onto the connected "WebSocket" transport.
    #          Such a module dispatches the latest shared-memory frame only after the browser has released the preceding rendered instance

    while True:
        await peer.frame_ready.wait()
        pending = await peer.queue.get()

        if pending is None:
            break

        current = read_frame()

        if current is None:
            try:
                peer.queue.put_nowait( True )
            except asyncio.QueueFull:
                pass

            await asyncio.sleep( 0.005 )
            continue

        _, payload = current

        peer.frame_ready.clear()

        try:
            await peer.socket.send( payload )
        except Exception:
            break

async def write_pose( data: dict ) -> None:

    # Purpose: It updates the shared memory control map with user-originated stance modifications, incrementing the sequence identifier for "DPDK" ingestion

    if ctrl_map is None or os.path.exists( EOS_PATH ):
        return

    try:
        cmd_id = int( data.get( "id", 0 ) )
        yaw = float( data.get( "yaw", 0.0 ) )
        pitch = float( data.get( "pitch", 0.0 ) )
        zoom = float( data.get( "zoom", 1.0 ) )
    except ( TypeError, ValueError ):
        return

    if cmd_id <= 0 or not ( 0.01 <= zoom <= 20.0 ):
        return

    async with ctrl_lock:
        old_seq = struct.unpack_from( "<Q", ctrl_map, 0 )[ 0 ]
        new_seq = old_seq + 1

        struct.pack_into( "<IIfffI", ctrl_map, 8, cmd_id, CMD_TYPE_POSE, yaw, pitch, zoom, 0 )
        struct.pack_into( "<Q", ctrl_map, 0, new_seq )

async def write_ack( data: dict ) -> None:

    # Purpose: It logs the end-to-end "Command-to-Photon" latencies resolved by the client, communicating them back to the telemetry logger

    if ctrl_map is None:
        return

    try:
        frame_id = int( data.get( "frame", 0 ) )
        cmd_id = int( data.get( "cmd", 0 ) )
        ctp_ms = float( data.get( "ctp_ms", 0.0 ) )
    except ( TypeError, ValueError ):
        return

    if frame_id <= 0 or cmd_id < 0 or ctp_ms < 0.0:
        return

    async with ctrl_lock:
        old_seq = struct.unpack_from( "<Q", ctrl_map, 32 )[ 0 ]
        new_seq = old_seq + 1

        struct.pack_into( "<IId", ctrl_map, 40, frame_id, cmd_id, ctp_ms )
        struct.pack_into( "<Q", ctrl_map, 32, new_seq )

async def handle_text( peer: web_peer, text: str ) -> None:

    # Purpose: It parses incoming "JSON" strings from the browser interface, routing them to the appropriate acknowledgment or interaction handlers

    try:
        data = json.loads( text )
    except json.JSONDecodeError:
        return

    msg_type = data.get( "type" )

    if msg_type == "pose":
        await write_pose( data )
    elif msg_type == "ack":
        await write_ack( data )
        peer.frame_ready.set()

async def ws_handler( socket ) -> None:

    # Purpose: It coordinates individual client lifecycles, streaming active geometries while consuming interactive orientation feedback

    peer = web_peer( socket )
    clients.add( peer )
    peer.sender = asyncio.create_task( send_loop( peer ) )

    if read_frame_seq() != 0:
        peer.queue.put_nowait( True )

    try:
        async for message in socket:
            if isinstance( message, str ):
                await handle_text( peer, message )
    except Exception:
        pass
    finally:
        clients.discard( peer )
        peer.frame_ready.set()

        try:
            peer.queue.put_nowait( None )
        except asyncio.QueueFull:
            try:
                peer.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass

            peer.queue.put_nowait( None )

        if peer.sender is not None:
            await peer.sender

async def frame_loop() -> None:

    # Purpose: It periodically observes the shared memory region for "DPDK" sequence updates & signals consumers without copying payloads ahead of demand

    last_seq = read_frame_seq()

    while True:
        try:
            seq = read_frame_seq()

            if seq == 0 or seq == last_seq:
                await asyncio.sleep( 0.005 )
                continue

            last_seq = seq

            for peer in list( clients ):
                if not peer.queue.full():
                    try:
                        peer.queue.put_nowait( True )
                    except asyncio.QueueFull:
                        pass

            await asyncio.sleep( 0 )
        except ( BufferError, ValueError, OSError ):
            await asyncio.sleep( 0.05 )

class quiet_handler( SimpleHTTPRequestHandler ):
    def log_message( self, fmt, *args ):
        return

def http_loop( http_port: int, web_root: str ) -> None:

    # Purpose: It exposes the visualization "HTML" application via a standard "HTTP" server residing within an independent execution thread

    os.chdir( web_root )
    server = ThreadingHTTPServer( ( "0.0.0.0", http_port ), quiet_handler )
    
    server.serve_forever()

async def main_async( ws_port: int ) -> None:

    # Purpose: It handles the "WebSocket" deployment alongside the endless reading mechanics

    await open_maps()

    async with websockets.serve( ws_handler, "0.0.0.0", ws_port, max_size = None, compression = None, ping_interval = 30, ping_timeout = 60 ):
        await frame_loop()

def main() -> None:

    # Purpose: It launches the unified visualization proxy, generating the "HTTP" server & the primary bridging architecture

    parser = argparse.ArgumentParser()
    parser.add_argument( "--http", type = int, default = 8080 )
    parser.add_argument( "--ws", type = int, default = 9999 )
    parser.add_argument( "--root", default = "/app/html" )
    args = parser.parse_args()

    http_thread = threading.Thread( target = http_loop, args = ( args.http, args.root ), daemon = True )
    http_thread.start()

    asyncio.run( main_async( args.ws ) )

if __name__ == "__main__":
    main()