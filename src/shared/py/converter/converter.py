import os
import glob
import time
import csv
import numpy as np
from plyfile import PlyData

# Configuration variables
BASE_DIR = os.path.dirname( os.path.abspath(__file__) )
SHARED_DIR = os.path.abspath( os.path.join( BASE_DIR, "..", ".." ) )

INPUT_DIR = os.path.join( SHARED_DIR, "data/loot/original" )
OUTPUT_DIR = os.path.join( SHARED_DIR, "data/loot/bin" )
TELEMETRY_PATH = os.path.join( SHARED_DIR, "log/converter/telemetry_converter.csv" )

# Reshaping constant applied during quantization ( e.g., 1.0 retains original dimensions, 1000.0 converts meters to mm )
SCALE_FACTOR = 1.0  

# Custom "NumPy" datatype mirroring the "C" / "DPDK" target "struct"
# Note: "<f4" = 32-bit float "Little-Endian" | "u1" = 8-bit unsigned integer
DPDK_STRUCT_DTYPE = np.dtype( [
    ( "x", "<f4" ), 
    ( "y", "<f4" ), 
    ( "z", "<f4" ),
    ( "r", "u1" ), 
    ( "g", "u1" ), 
    ( "b", "u1" ), 
    ( "padding", "u1" )
] )

def process_data( input_directory: str, output_directory: str, telemetry_file: str ) -> None:

    # Purpose: It scans the input directory for ".ply" files, extracts spatial & photometric data, applies scaling, aligns to the desired organization, & writes back ( contiguous ) binary contents
    
    if not os.path.exists( output_directory ):
        os.makedirs( output_directory )
        
    ply_files = sorted( glob.glob( os.path.join( input_directory, "*.ply" ) ) )
    
    if not ply_files:
        print( f"[SYSTEM] Error: No \".ply\" files found in {input_directory}..." )
        return

    print( f"[SYSTEM] Starting conversion of {len( ply_files )} files...\n" )
    start_time_total = time.perf_counter()

    telemetry_records = []
    
    for file_path in ply_files:
        filename = os.path.basename( file_path )
        output_filename = os.path.join( output_directory, filename.replace( ".ply", ".bin" ) )

        record = { "filename": filename, "status": "success", "num_points": 0, "read_ascii_ms": 0.0, "write_bin_ms": 0.0, "conversion_ms": 0.0, "size_ascii_bytes": 0, "size_bin_bytes": 0 }

        try: 
            record[ "size_ascii_bytes" ] = os.path.getsize( file_path ) 
        except OSError: 
            pass

        t_file_start = time.perf_counter()
        
        try:
            t_read_start = time.perf_counter() 

            ply_data = PlyData.read( file_path ) 

            t_read_end = time.perf_counter()

            vertex_data = ply_data[ "vertex" ]
            num_points = vertex_data.count
            
            x = ( vertex_data[ "x" ] * SCALE_FACTOR ).astype( np.float32 )
            y = ( vertex_data[ "y" ] * SCALE_FACTOR ).astype( np.float32 )
            z = ( vertex_data[ "z" ] * SCALE_FACTOR ).astype( np.float32 )
            
            r = vertex_data[ "red" ].astype( np.uint8 )
            g = vertex_data[ "green" ].astype( np.uint8 )
            b = vertex_data[ "blue" ].astype( np.uint8 )

            # Note: The "loot" dataset typically lacks an alpha channel. Zeroed padding byte is added to maintain strict 16-byte memory alignment, crucial for "DPDK" "SIMD" / vectorized instructions
            padding = np.zeros( num_points, dtype = np.uint8 )
            
            network_array = np.empty( num_points, dtype = DPDK_STRUCT_DTYPE )

            network_array[ "x" ] = x
            network_array[ "y" ] = y
            network_array[ "z" ] = z

            network_array[ "r" ] = r
            network_array[ "g" ] = g
            network_array[ "b" ] = b

            network_array[ "padding" ] = padding

            t_write_start = time.perf_counter()
            
            with open( output_filename, "wb" ) as file_out:
                file_out.write( network_array.tobytes() )

            t_write_end = time.perf_counter() 

            record[ "write_bin_ms" ] = ( t_write_end - t_write_start ) * 1000.0 
            record[ "num_points" ] = num_points

            try: 
                record[ "size_bin_bytes" ] = os.path.getsize( output_filename ) 
            except OSError: 
                pass
                
            print( f"[SYSTEM] Saved: \"{filename}\" ( {num_points} points ) -> \"{os.path.basename( output_filename )}\"" )
            
        except Exception as e:
            record[ "status" ] = "error"
            print( f"\n[SYSTEM] Error: Failed to convert {filename}: {e}\n" )

        finally:
            record[ "conversion_ms" ] = ( time.perf_counter() - t_file_start ) * 1000.0
            telemetry_records.append( record )

    elapsed_time = time.perf_counter() - start_time_total

    try:
        with open( telemetry_file, "w", newline = "" ) as fcsv:
            writer = csv.DictWriter( fcsv, fieldnames = [ "filename", "status", "num_points", "read_ascii_ms", "write_bin_ms", "conversion_ms", "size_ascii_bytes", "size_bin_bytes" ], delimiter = ";" ) 
            writer.writeheader() 

            writer.writerows( telemetry_records )

        print( f"\n[SYSTEM] Conversion telemetry saved to: \"{telemetry_file}\"" )
    except Exception as e:
        print( f"\n[SYSTEM] Error: Failed to save telemetry \".csv\" file: {e}" )

    print( f"\n[SYSTEM] Dataset processing successfully completed in {elapsed_time:.2f} seconds." )

if __name__ == "__main__":
    # Purpose: The script processes original "ASCII" / Binary ".ply" files & converts them into raw, header-less memory dumps ( ".bin" ).
    #          Such approach allows the "DPDK" "Camera" node to map the document directly into memory structures via pointers, eliminating costly header parsing at runtime & maximizing throughput
    process_data( INPUT_DIR, OUTPUT_DIR, TELEMETRY_PATH )