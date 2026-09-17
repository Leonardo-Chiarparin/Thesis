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
OUT_K = 20
OUT_STD = 2.0

METRIC_FIELDS = [
    "mean_error", "geom_rmse", "chamfer", "hausdorff",
    "mean_mm", "rmse_mm", "chamfer_mm", "hausdorff_mm"
]

QUALITY_FIELDS = [
    f"{stage}_{alignment}_{metric}"
    for stage in ( "pre", "post" )
    for alignment in ( "classical", "robust" )
    for metric in METRIC_FIELDS
]

def load_cloud( path: str, count: int = -1, offset: int = 0 ) -> np.ndarray:

    # Purpose: It extracts raw spatial coordinates from the specified binary payload to construct the primary floating-point matrix

    data = np.fromfile( path, dtype = POINT_DTYPE, count = count, offset = offset )
    return np.column_stack( ( data[ "x" ], data[ "y" ], data[ "z" ] ) ).astype( np.float64, copy = False )

def rot_matrix( yaw: float, pitch: float ) -> np.ndarray:

    # Purpose: It computes the rigid 3x3 rotational matrix corresponding to user-specified angles

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

    # Purpose: It applies the inverse geometric transformation to revert aligned coordinates to their native spatial reference

    scale = zoom if abs( zoom ) > 1e-9 else 1.0
    matrix = rot_matrix( yaw, pitch )

    return ( xyz / scale ) @ matrix

def stat_filter( xyz: np.ndarray ) -> np.ndarray:

    # Purpose: It isolates & removes statistical outliers via topological nearest-neighbor density evaluation

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

    # Purpose: It executes "Singular Value Decomposition" ( "SVD" ) to determine the optimal rigid translation & rotation mapping between source & target

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

def icp_fit( source: np.ndarray, reference: np.ndarray ) -> tuple:

    # Purpose: It performs the "Iterative Closest Point" ( "ICP" ) registration over complete input sets whilst retaining only correspondences inside the prescribed physical search radius

    if len( source ) < 3 or len( reference ) < 3:
        return np.eye( 3, dtype = np.float64 ), np.zeros( 3, dtype = np.float64 ), float( "nan" )

    initial_shift = reference.mean( axis = 0 ) - source.mean( axis = 0 )
    current = source + initial_shift

    rotation = np.eye( 3, dtype = np.float64 )
    shift = initial_shift.copy()
    
    ref_tree = cKDTree( reference )
    previous = None

    for _ in range( ICP_REPETITIONS ):
        distance, ids = ref_tree.query( current, k = 1, workers = 1 )
        mask = distance <= ICP_RADIUS

        if mask.sum() < 3:
            break

        step_rot, step_shift = best_rigid( current[ mask ], reference[ ids[ mask ] ] )
        
        current = current @ step_rot.T + step_shift
        rotation = step_rot @ rotation
        shift = shift @ step_rot.T + step_shift

        score = float( distance[ mask ].mean() )

        if previous is not None and abs( previous - score ) < 1e-5:
            break

        previous = score

    final_distance, _ = ref_tree.query( current, k = 1, workers = 1 )
    final_mask = final_distance <= ICP_RADIUS

    if final_mask.sum() < 3:
        inlier_rmse = float( "nan" )
    else:
        inlier_rmse = float( np.sqrt( np.square( final_distance[ final_mask ] ).mean() ) )

    return rotation, shift, inlier_rmse

def classical_icp( source: np.ndarray, reference: np.ndarray ) -> tuple:

    # Purpose: It executes the classical coupled alignment by estimating rigid registration directly upon the complete unfiltered reconstructed & reference clouds

    return icp_fit( source, reference )

def robust_icp( source: np.ndarray, reference: np.ndarray ) -> tuple:

    # Purpose: It executes the robust decoupled alignment by filtering only the reconstructed cloud employed for pose estimation whilst preserving complete geometry for subsequent measurements

    source_fit = stat_filter( source )

    return icp_fit( source_fit, reference )

def distance_metrics( source: np.ndarray, reference: np.ndarray ) -> tuple:

    # Purpose: It measures symmetric geometric distortions between the registered source & unaltered reference utilizing "KD-Tree" associations

    ref_tree = cKDTree( reference )
    src_tree = cKDTree( source )

    src_dist, _ = ref_tree.query( source, k = 1, workers = 1 )
    ref_dist, _ = src_tree.query( reference, k = 1, workers = 1 )

    mean_error = float( src_dist.mean() )
    geom_rmse = float( np.sqrt( ( np.square( src_dist ).sum() + np.square( ref_dist ).sum() ) / ( len( src_dist ) + len( ref_dist ) ) ) )
    chamfer = float( src_dist.mean() + ref_dist.mean() )
    hausdorff = float( max( src_dist.max( initial = 0.0 ), ref_dist.max( initial = 0.0 ) ) )

    return mean_error, geom_rmse, chamfer, hausdorff

def evaluate_alignment( source: np.ndarray, reference: np.ndarray, alignment: str ) -> dict:

    # Purpose: It applies the selected coupled or decoupled alignment methodology & derives the four geometric indicators according to their corresponding evaluation domains

    if alignment == "classical":
        rotation, shift, icp_rmse = classical_icp( source, reference )
    elif alignment == "robust":
        rotation, shift, icp_rmse = robust_icp( source, reference )
    else:
        raise ValueError( f"Unknown alignment mode: {alignment}" )

    aligned_xyz = source @ rotation.T + shift
    mean_error, symmetric_rmse, chamfer, hausdorff = distance_metrics( aligned_xyz, reference )

    geom_rmse = icp_rmse if alignment == "classical" else symmetric_rmse

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

def capture_index( path: str ) -> dict:

    # Purpose: It parses the quality capture file sequentially to build a rapid look-up index for frame payload offsets

    index = {}
    file_size = os.path.getsize( path )

    with open( path, "rb" ) as capture:
        while True:
            header = capture.read( FRAME_SIZE )

            if not header:
                break

            if len( header ) != FRAME_SIZE:
                raise RuntimeError( "Misshapen frame header detected..." )

            frame_id, point_count = struct.unpack( FRAME_FORMAT, header )

            if frame_id <= 0 or point_count > MAX_POINTS:
                raise RuntimeError( f"Misshapen record detected at frame {frame_id} having {point_count} points..." )

            point_offset = capture.tell()
            data_size = point_count * POINT_SIZE
            
            capture.seek( data_size, os.SEEK_CUR )

            if capture.tell() > file_size:
                raise RuntimeError( f"Misshapen record detected at frame {frame_id}..." )

            index[ frame_id ] = ( point_offset, point_count )

    return index

def format_metric( value: float ) -> str:

    # Purpose: It transforms floating-point metric outcomes into sanitized string formats suitable for telemetry export

    if math.isnan( value ):
        return "nan"
        
    if math.isinf( value ):
        return "inf" if value > 0 else "-inf"
        
    return f"{value:.6f}"

def metric_row( frame_id: int, row: dict, capture_pre_path: str, pre_record: tuple, capture_post_path: str, post_record: tuple, ref_dir: str ) -> dict:

    # Purpose: It orchestrates the comprehensive metric pipeline for a specific application frame, comparing pre/post-erosion reconstructions under classical coupled & robust decoupled registration

    pre_offset, pre_count = pre_record
    post_offset, post_count = post_record
    ref_path = os.path.join( ref_dir, f"loot_vox10_{frame_id + 999}.bin" )

    if not os.path.exists( ref_path ) or pre_count == 0 or post_count == 0:
        return None

    pre_xyz = load_cloud( capture_pre_path, count = pre_count, offset = pre_offset )
    post_xyz = load_cloud( capture_post_path, count = post_count, offset = post_offset )
    ref_xyz = load_cloud( ref_path )

    if len( pre_xyz ) == 0 or len( post_xyz ) == 0 or len( ref_xyz ) == 0:
        return None

    yaw = float( row.get( "yaw", 0.0 ) )
    pitch = float( row.get( "pitch", 0.0 ) )
    zoom = float( row.get( "zoom", 1.0 ) )

    pre_xyz = undo_pose( pre_xyz, yaw, pitch, zoom )
    post_xyz = undo_pose( post_xyz, yaw, pitch, zoom )

    stages = {
        "pre": pre_xyz,
        "post": post_xyz
    }

    metrics = {}

    for stage, source in stages.items():
        for alignment in ( "classical", "robust" ):
            values = evaluate_alignment( source, ref_xyz, alignment )

            for field, value in values.items():
                metrics[ f"{stage}_{alignment}_{field}" ] = value

    return metrics

def metric_task( task: tuple ) -> tuple:

    # Purpose: It evaluates a single frame while preserving exact "ICP" & metric definitions across both reconstruction stages

    row_index, frame_id, row, capture_pre_path, pre_record, capture_post_path, post_record, ref_dir = task
    metrics = metric_row( frame_id, row, capture_pre_path, pre_record, capture_post_path, post_record, ref_dir )

    return row_index, frame_id, metrics

def merge_quality( telemetry_path: str, capture_pre_path: str, capture_post_path: str, ref_dir: str ) -> tuple:

    # Purpose: It formats retrieved quality variables alongside existing diagnostic logs, sequentially assessing eligible pre/post-erosion frames

    with open( telemetry_path, newline = "" ) as csv_file:
        reader = csv.DictReader( csv_file, delimiter = ";" )
        fieldnames = list( reader.fieldnames or [] )
        rows = list( reader )

    legacy_fields = {
        "mean_error", "geom_rmse", "chamfer", "hausdorff",
        "mean_mm", "rmse_mm", "chamfer_mm", "hausdorff_mm"
    }

    fieldnames = [ field for field in fieldnames if field not in legacy_fields ]

    for row in rows:
        for field in legacy_fields:
            row.pop( field, None )

    for field in QUALITY_FIELDS:
        if field not in fieldnames:
            fieldnames.append( field )

    pre_index = capture_index( capture_pre_path )
    post_index = capture_index( capture_post_path )
    completed = 0
    expected = sum( 1 for row in rows if row.get( "rx_complete" ) == "1" )

    tasks = []

    for row_index, row in enumerate( rows ):
        frame_id = int( row.get( "frame_id", 0 ) )

        if row.get( "rx_complete" ) != "1" or frame_id not in pre_index or frame_id not in post_index:
            continue

        tasks.append( ( row_index, frame_id, row, capture_pre_path, pre_index[ frame_id ], capture_post_path, post_index[ frame_id ], ref_dir ) )

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

def wait_capture( path: str ) -> None:

    # Purpose: It systematically suspends execution until a reconstruction-stage capture becomes available for offline geometric inspection

    while not os.path.exists( path ):
        time.sleep( 0.5 )

def main() -> None:

    # Purpose: It drives the offline spatial evaluation sequence, computing pre/post-erosion coupled & decoupled geometric fidelity post-"DPDK" operation & merging outcomes into the persistent ".csv" file

    parser = argparse.ArgumentParser( description = "" )
    parser.add_argument( "--telemetry", default = "/shared/log/user/telemetry_user.csv" )
    parser.add_argument( "--capture-pre", default = "/shared/data/loot/made/results_pre.bin" )
    parser.add_argument( "--capture-post", default = "/shared/data/loot/made/results_post.bin" )
    parser.add_argument( "--reference", default = "/shared/data/loot/bin" )
    parser.add_argument( "--ready", default = "/tmp/sfc-user-quality" )
    parser.add_argument( "--done", default = "/tmp/sfc-user-done" )

    args = parser.parse_args()

    wait_ready( args.ready )

    if not os.path.exists( args.telemetry ):
        raise SystemExit( "Telemetry is unavailable after the quality-ready signal..." )

    wait_capture( args.capture_pre )
    wait_capture( args.capture_post )

    completed, expected = merge_quality( args.telemetry, args.capture_pre, args.capture_post, args.reference )

    if completed == expected:
        os.remove( args.capture_pre )
        os.remove( args.capture_post )
    else:
        print( f"[SYSTEM] Error: Only {completed} / {expected} complete frames were evaluated...", flush = True )

    print( f"\n[SYSTEM] Metrics successfully exported to: \"{args.telemetry}\".\n", flush = True )

    with open( args.done, "w", encoding = "utf-8" ):
        pass
    
if __name__ == "__main__":
    main()