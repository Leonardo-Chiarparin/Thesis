import argparse
import csv
import math
import os
import struct
import time
import numpy as np
from scipy.spatial import cKDTree

POINT_DTYPE = np.dtype( [
    ( "x", "<f4" ), ( "y", "<f4" ), ( "z", "<f4" ),
    ( "r", "u1" ), ( "g", "u1" ), ( "b", "u1" ), ( "pad", "u1" )
] )

FRAME_FORMAT = "<II"
FRAME_SIZE = struct.calcsize( FRAME_FORMAT )
POINT_SIZE = POINT_DTYPE.itemsize
MAX_POINTS = 6 * 640 * 480

VOXEL_MM = 1.820
ICP_RADIUS = 200.0 / VOXEL_MM
ICP_REPETITIONS = 30
ICP_POINTS = 60000
OUT_K = 20
OUT_STD = 2.0

QUALITY_FIELDS = [
    "mean_error", "geom_rmse", "chamfer", "hausdorff",
    "mean_mm", "rmse_mm", "chamfer_mm", "hausdorff_mm"
]

def load_cloud( path: str, count: int = -1, offset: int = 0 ) -> np.ndarray:

    # Purpose: It extracts raw spatial coordinates from the specified binary payload & constructs the primary floating-point matrix

    data = np.fromfile( path, dtype = POINT_DTYPE, count = count, offset = offset )
    return np.column_stack( ( data[ "x" ], data[ "y" ], data[ "z" ] ) ).astype( np.float64, copy = False )

def rot_matrix( yaw: float, pitch: float ) -> np.ndarray:

    # Purpose: It computes the rigid 3x3 rotational matrix derived from the user-specified angles

    cp = math.cos( pitch )
    sp = math.sin( pitch )
    cy = math.cos( yaw )
    sy = math.sin( yaw )

    return np.array( [
        [ cy, sy * sp, sy * cp ],
        [ 0.0, cp, -sp ],
        [ -sy, cy * sp, cy * cp ]
    ], dtype = np.float64 )

def undo_pose( xyz: np.ndarray, yaw: float, pitch: float, zoom: float ) -> np.ndarray:

    # Purpose: It applies the inverse geometric transformation to revert the aligned coordinates back to their native coordinate space

    scale = zoom if abs( zoom ) > 1e-9 else 1.0
    matrix = rot_matrix( yaw, pitch )

    return ( xyz / scale ) @ matrix

def sample_cloud( xyz: np.ndarray, limit: int ) -> np.ndarray:

    # Purpose: It decimates the dense point cloud through linear selection, preserving structural geometry while reducing the computational domain

    if limit <= 0 or len( xyz ) <= limit:
        return xyz

    ids = np.linspace( 0, len( xyz ) - 1, limit, dtype = np.int64 )
    
    return xyz[ ids ]

def stat_filter( xyz: np.ndarray ) -> np.ndarray:

    # Purpose: It isolates & removes statistical outliers using spatial nearest-neighbor density evaluation

    if len( xyz ) <= OUT_K + 1:
        return xyz

    tree = cKDTree( xyz )
    dist, _ = tree.query( xyz, k = OUT_K + 1, workers = 1 )
    
    mean_dist = dist[ :, 1: ].mean( axis = 1 )
    threshold = mean_dist.mean() + OUT_STD * mean_dist.std()
    mask = mean_dist <= threshold

    if mask.sum() < 3:
        return xyz

    return xyz[ mask ]

def best_rigid( source: np.ndarray, target: np.ndarray ) -> tuple:

    # Purpose: It executes "Singular Value Decomposition" ( "SVD" ) to discover the optimal rigid translation & rotation mapping source to target

    source_ctr = source.mean( axis = 0 )
    target_ctr = target.mean( axis = 0 )
    
    source_zero = source - source_ctr
    target_zero = target - target_ctr
    
    matrix = source_zero.T @ target_zero
    u, _, vt = np.linalg.svd( matrix )
    rotation = vt.T @ u.T

    if np.linalg.det( rotation ) < 0:
        vt[ -1, : ] *= -1
        rotation = vt.T @ u.T

    shift = target_ctr - source_ctr @ rotation.T

    return rotation, shift

def robust_icp( source: np.ndarray, reference: np.ndarray ) -> tuple:

    # Purpose: It performs the "Iterative Closest Point" ( "ICP" ) algorithm over statistically filtered subsets to establish exact frame alignment

    source_fit = sample_cloud( source, ICP_POINTS )
    source_fit = stat_filter( source_fit )
    ref_fit = sample_cloud( reference, ICP_POINTS * 2 )

    initial_shift = ref_fit.mean( axis = 0 ) - source_fit.mean( axis = 0 )
    current = source_fit + initial_shift

    rotation = np.eye( 3, dtype = np.float64 )
    shift = initial_shift.copy()
    
    ref_tree = cKDTree( ref_fit )
    previous = None

    for _ in range( ICP_REPETITIONS ):
        distance, ids = ref_tree.query( current, k = 1, workers = 1 )
        mask = distance <= ICP_RADIUS

        if mask.sum() < 3:
            break

        step_rot, step_shift = best_rigid( current[ mask ], ref_fit[ ids[ mask ] ] )
        
        current = current @ step_rot.T + step_shift
        rotation = step_rot @ rotation
        shift = shift @ step_rot.T + step_shift

        score = float( distance[ mask ].mean() )

        if previous is not None and abs( previous - score ) < 1e-5:
            break

        previous = score

    return rotation, shift

def distance_metrics( source: np.ndarray, reference: np.ndarray ) -> tuple:

    # Purpose: It measures the symmetric geometric distortions between the registered source & unaltered reference employing "KD-Tree" associations

    ref_tree = cKDTree( reference )
    src_tree = cKDTree( source )

    src_dist, _ = ref_tree.query( source, k = 1, workers = 1 )
    ref_dist, _ = src_tree.query( reference, k = 1, workers = 1 )

    mean_error = float( src_dist.mean() )
    geom_rmse = float( np.sqrt( ( np.square( src_dist ).sum() + np.square( ref_dist ).sum() ) / ( len( src_dist ) + len( ref_dist ) ) ) )
    chamfer = float( src_dist.mean() + ref_dist.mean() )
    hausdorff = float( max( src_dist.max( initial = 0.0 ), ref_dist.max( initial = 0.0 ) ) )

    return mean_error, geom_rmse, chamfer, hausdorff

def capture_index( path: str ) -> dict:

    # Purpose: It parses the quality capture file sequentially, building a rapid look-up index for frame payload offsets

    index = {}

    with open( path, "rb" ) as capture:
        while True:
            header = capture.read( FRAME_SIZE )

            if not header:
                break

            if len( header ) != FRAME_SIZE:
                raise RuntimeError( "Truncated User quality frame header." )

            frame_id, point_count = struct.unpack( FRAME_FORMAT, header )

            if frame_id <= 0 or point_count > MAX_POINTS:
                raise RuntimeError( f"Invalid User quality record: frame={frame_id}, points={point_count}." )

            point_offset = capture.tell()
            data_size = point_count * POINT_SIZE
            
            capture.seek( data_size, os.SEEK_CUR )

            if capture.tell() > os.path.getsize( path ):
                raise RuntimeError( f"Truncated User quality record at frame {frame_id}." )

            index[ frame_id ] = ( point_offset, point_count )

    return index

def format_metric( value: float ) -> str:

    # Purpose: It safely transforms floating-point metric outcomes into sanitized string formats suitable for telemetry export

    if math.isnan( value ):
        return "nan"
        
    if math.isinf( value ):
        return "inf" if value > 0 else "-inf"
        
    return f"{value:.6f}"

def metric_row( frame_id: int, row: dict, capture_path: str, record: tuple, ref_dir: str ) -> dict:

    # Purpose: It orchestrates the full metric pipeline for a specific application frame, reversing applied poses & measuring objective distortions

    point_offset, point_count = record
    ref_path = os.path.join( ref_dir, f"loot_vox10_{frame_id + 999}.bin" )

    if not os.path.exists( ref_path ) or point_count == 0:
        return None

    rec_xyz = load_cloud( capture_path, count = point_count, offset = point_offset )
    ref_xyz = load_cloud( ref_path )

    if len( rec_xyz ) == 0 or len( ref_xyz ) == 0:
        return None

    yaw = float( row.get( "yaw", 0.0 ) )
    pitch = float( row.get( "pitch", 0.0 ) )
    zoom = float( row.get( "zoom", 1.0 ) )

    rec_xyz = undo_pose( rec_xyz, yaw, pitch, zoom )

    rotation, shift = robust_icp( rec_xyz, ref_xyz )
    aligned_xyz = rec_xyz @ rotation.T + shift

    mean_error, geom_rmse, chamfer, hausdorff = distance_metrics( aligned_xyz, ref_xyz )

    return {
        "mean_error": mean_error,
        "geom_rmse": geom_rmse,
        "chamfer": chamfer,
        "hausdorff": hausdorff,
        "mean_mm": mean_error * VOXEL_MM,
        "rmse_mm": geom_rmse * VOXEL_MM,
        "chamfer_mm": chamfer * VOXEL_MM,
        "hausdorff_mm": hausdorff * VOXEL_MM
    }

def metric_task( task: tuple ) -> tuple:

    # Purpose: It evaluates one frame while preserving the exact "ICP" & metric definitions

    row_index, frame_id, row, capture_path, record, ref_dir = task
    metrics = metric_row( frame_id, row, capture_path, record, ref_dir )

    return row_index, frame_id, metrics

def merge_quality( telemetry_path: str, capture_path: str, ref_dir: str ) -> tuple:

    # Purpose: It formats the retrieved quality variables with the existing "DPDK" telemetry logs, evaluating eligible frames consecutively

    with open( telemetry_path, newline = "" ) as csv_file:
        reader = csv.DictReader( csv_file, delimiter = ";" )
        fieldnames = list( reader.fieldnames or [] )
        rows = list( reader )

    for field in QUALITY_FIELDS:
        if field not in fieldnames:
            fieldnames.append( field )

    index = capture_index( capture_path )
    completed = 0
    expected = sum( 1 for row in rows if row.get( "rx_complete" ) == "1" )

    tasks = []

    for row_index, row in enumerate( rows ):
        frame_id = int( row.get( "frame_id", 0 ) )

        if row.get( "rx_complete" ) != "1" or frame_id not in index:
            continue

        tasks.append( ( row_index, frame_id, row, capture_path, index[ frame_id ], ref_dir ) )

    print( f"[SYSTEM] Quality indicators computed for {len( tasks )} elements.\n", flush = True )

    results = map( metric_task, tasks )

    for row_index, frame_id, metrics in results:
        print( f"[SYSTEM] Grading frame {frame_id}...", flush = True )

        if metrics is None:
            continue

        for field, value in metrics.items():
            rows[ row_index ][ field ] = format_metric( value )

        completed += 1

    temp_path = telemetry_path + ".tmp"

    with open( temp_path, "w", newline = "" ) as csv_file:
        writer = csv.DictWriter( csv_file, fieldnames = fieldnames, delimiter = ";" )
        writer.writeheader()
        writer.writerows( rows )

    os.replace( temp_path, telemetry_path )
    
    return completed, expected

def wait_ready( path: str ) -> None:

    # Purpose: It systematically suspends execution until the "DPDK" streaming application signals completion

    while not os.path.exists( path ):
        time.sleep( 0.5 )

def main() -> None:

    # Purpose: It drives the offline geometric evaluation sequence, computing structural similarity following "DPDK" operation & merging outcomes into the persistent ".csv" file

    parser = argparse.ArgumentParser( description = "" )
    parser.add_argument( "--telemetry", default = "/shared/log/user/telemetry_user.csv" )
    parser.add_argument( "--capture", default = "/shared/data/loot/made/results.bin" )
    parser.add_argument( "--reference", default = "/shared/data/loot/bin" )
    parser.add_argument( "--ready", default = "/tmp/sfc-user-quality" )
    parser.add_argument( "--done", default = "/tmp/sfc-user-done" )

    args = parser.parse_args()

    wait_ready( args.ready )

    if not os.path.exists( args.telemetry ):
        raise SystemExit( "User telemetry is unavailable after the quality-ready signal." )

    if not os.path.exists( args.capture ):
        raise SystemExit( "User point-cloud quality capture is unavailable after the quality-ready signal." )

    completed, expected = merge_quality( args.telemetry, args.capture, args.reference )

    if completed == expected:
        os.remove( args.capture )
    else:
        print( f"[SYSTEM] Error: Only {completed} / {expected} complete frames were evaluated...", flush = True )

    print( f"\n[SYSTEM] Metrics successfully exported to: \"{args.telemetry}\".\n", flush = True )

    with open( args.done, "w", encoding = "utf-8" ):
        pass
    
if __name__ == "__main__":
    main()