import socket
import time
import subprocess
import logging
import re

# Setting logs
logging.basicConfig( level=logging.INFO, format="%(message)s" )
log = logging.getLogger( "SFF1" )

# Configuration parameters
IP_ENCODER = "10.0.2.1"
PORT_TELEMETRY = 8080
POLLING_RATE = 0.2 # Seconds between queue status checks

# Hardware constraints configured in the P4 Traffic Manager
MAX_QUEUE_DEPTH = 167 
MAX_BITRATE = 50.0 # Mbps
MIN_BITRATE = 5.0 # Mbps

def get_queue_depth():
    
    # Purpose: It queries the P4 Data Plane via the runtime CLI to retrieve the instantaneous queue depth 
    #          of the bottleneck interface ( e.g., port 3 towards the "Transcoder" ). This metric 
    #          serves as a proxy for the network's current carrying capacity.
    
    # Using list format for "subprocess" avoids shell quoting vulnerabilities ( or potential errors )
    command = [ "simple_switch_CLI" ]
    input_read = b"register_read queue_depth_register 0\n"
    input_reset = b"register_write queue_depth_register 0 0\n"

    try:
        process_read = subprocess.Popen( command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE )
        stdout, _ = process_read.communicate( input=input_read )

        result = stdout.decode()

        process_reset = subprocess.Popen( command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE )
        process_reset.communicate( input=input_reset )
        
        match = re.search( r"=\s*(\d+)", result )
        
        # Parsing the integer value from the CLI output string
        if match:
            return int( match.group( 1 ) )
        else:
            return 0
    
    except ( ValueError, IndexError, Exception ) as e:
        return 0

def calculate_available_bitrate( depth ):
    
    # Purpose: It maps the hardware queue depth into an explicit bandwidth feedback ( hard-limit of 40 ms ).
    
    if depth >= MAX_QUEUE_DEPTH:
        return MIN_BITRATE # Bufferbloat
    
    degradation_factor = depth / MAX_QUEUE_DEPTH
    bitrate = MAX_BITRATE - ( degradation_factor * ( MAX_BITRATE - MIN_BITRATE ) )
    
    return round( bitrate, 1 )

def start_monitoring():

    # Purpose: It initializes the Control Plane component responsible for In-Network Quality Adaptation. 
    #          The function acts as a micro-service generating explicit feedback for the upstream "Encoder".

    sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
    last_bitrate = -1.0

    log.info( "[SYSTEM] Telemetry initialized." )
    log.info( "[SYSTEM] Monitoring egress queue for dynamic V-PCC adaptation." )
    
    while True:
        depth = get_queue_depth()
        current_bitrate = calculate_available_bitrate( depth )

        # Updates are sent only if the bitrate changes ( thus avoiding network flooding )
        if current_bitrate != last_bitrate:
            message = f"BITRATE:{current_bitrate}"
            sock.sendto( message.encode( "utf-8" ), ( IP_ENCODER, PORT_TELEMETRY ) )
            last_bitrate = current_bitrate
            
            log.info( f"[SYSTEM] Depth: {depth}/{MAX_QUEUE_DEPTH} | Feedback: {current_bitrate} Mbps" )

        time.sleep( POLLING_RATE )

if __name__ == "__main__":
    start_monitoring()