#!/usr/bin/env python3
"""
Process raw PCD map: remove ground plane and downsample.
Complete pipeline for cleaning maps from upside-down LiDAR scans.
"""

import sys
import os
import open3d as o3d
import numpy as np

def remove_ground_plane(pcd, height_threshold=0.0):
    """
    Remove ground plane points from point cloud.
    For upside-down LiDAR: removes points ABOVE threshold (floor is positive Z).
    
    Args:
        pcd: Open3D point cloud
        height_threshold: Points above this Z height are removed (meters)
    
    Returns:
        Filtered point cloud
    """
    print(f"\n--- Step 1: Remove Ground Plane ---")
    points = np.asarray(pcd.points)
    
    # Show Z-axis range
    z_min, z_max = points[:, 2].min(), points[:, 2].max()
    print(f"Z-axis range: {z_min:.2f}m to {z_max:.2f}m")
    
    num_points_before = len(pcd.points)
    print(f"Points before: {num_points_before:,}")
    print(f"Removing points above Z = {height_threshold}m (floor/trail)...")
    
    # Filter points BELOW threshold (keep negative Z for upside-down LiDAR)
    mask = points[:, 2] < height_threshold
    pcd_filtered = pcd.select_by_index(np.where(mask)[0])
    
    num_points_after = len(pcd_filtered.points)
    removed = num_points_before - num_points_after
    percent_removed = (removed / num_points_before) * 100
    
    print(f"Points after: {num_points_after:,}")
    print(f"Removed: {removed:,} points ({percent_removed:.1f}%)")
    
    return pcd_filtered

def downsample_map(pcd, voxel_size=0.05):
    """
    Downsample point cloud using voxel grid filter.
    
    Args:
        pcd: Open3D point cloud
        voxel_size: Voxel size in meters
    
    Returns:
        Downsampled point cloud
    """
    print(f"\n--- Step 2: Downsample ---")
    num_points_before = len(pcd.points)
    print(f"Points before: {num_points_before:,}")
    print(f"Voxel size: {voxel_size}m ({voxel_size*100:.0f}cm)")
    
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    num_points_after = len(pcd_downsampled.points)
    removed = num_points_before - num_points_after
    percent_removed = (removed / num_points_before) * 100
    
    print(f"Points after: {num_points_after:,}")
    print(f"Removed: {removed:,} points ({percent_removed:.1f}%)")
    
    return pcd_downsampled

def process_map(input_path, output_path, height_threshold=1.0, voxel_size=0.1):
    """
    Complete map processing pipeline.
    
    Args:
        input_path: Path to input PCD file
        output_path: Path to save processed PCD file
        height_threshold: Z threshold for ground removal (meters, default 1.0m)
        voxel_size: Voxel size for downsampling (meters)
    """
    print("=" * 60)
    print("PCD Map Processing Pipeline")
    print("=" * 60)
    print(f"\nInput:  {input_path}")
    print(f"Output: {output_path}")
    
    # Load point cloud
    print(f"\n--- Loading Point Cloud ---")
    if not os.path.exists(input_path):
        print(f"ERROR: Input file not found: {input_path}")
        sys.exit(1)
    
    pcd = o3d.io.read_point_cloud(input_path)
    num_points_original = len(pcd.points)
    print(f"Loaded: {num_points_original:,} points")
    
    if num_points_original == 0:
        print("ERROR: Input point cloud is empty!")
        sys.exit(1)
    
    # Step 1: Remove ground plane
    pcd = remove_ground_plane(pcd, height_threshold)
    
    # Step 2: Downsample
    pcd = downsample_map(pcd, voxel_size)
    
    # Save result
    print(f"\n--- Saving Result ---")
    print(f"Output file: {output_path}")
    
    # Create output directory if needed
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    o3d.io.write_point_cloud(output_path, pcd)
    
    # Get file sizes
    input_size_mb = os.path.getsize(input_path) / (1024 * 1024)
    output_size_mb = os.path.getsize(output_path) / (1024 * 1024)
    size_reduction = ((input_size_mb - output_size_mb) / input_size_mb) * 100
    
    print(f"Input size:  {input_size_mb:.1f} MB")
    print(f"Output size: {output_size_mb:.1f} MB")
    print(f"Size reduction: {size_reduction:.1f}%")
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Original points:   {num_points_original:,}")
    print(f"Final points:      {len(pcd.points):,}")
    total_removed = num_points_original - len(pcd.points)
    total_percent = (total_removed / num_points_original) * 100
    print(f"Total removed:     {total_removed:,} ({total_percent:.1f}%)")
    print(f"File size reduced: {size_reduction:.1f}%")
    print("\nProcessing complete!")

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 process_map.py <input.pcd> <output.pcd> [height_threshold] [voxel_size]")
        print("\nProcesses PCD map through complete pipeline:")
        print("  1. Remove ground plane (for upside-down LiDAR)")
        print("  2. Downsample with voxel grid filter")
        print("\nExamples:")
        print("  # Default settings (LiDAR at 1.25m height, 10cm voxels)")
        print("  python3 process_map.py scans.pcd scans_clean.pcd")
        print()
        print("  # Custom threshold and voxel size")
        print("  python3 process_map.py scans.pcd scans_clean.pcd 1.0 0.05")
        print()
        print("  # With custom map name")
        print("  python3 process_map.py office_map.pcd office_map_clean.pcd 1.2 0.1")
        print("\nParameters:")
        print("  height_threshold: Remove points above this Z (default: 1.0 meters)")
        print("                    For LiDAR at 1.25m: removes floor 1m below LiDAR")
        print("  voxel_size:       Downsample resolution (default: 0.1m = 10cm)")
        print("                    Smaller = more detail, larger file")
        print("\nRecommended voxel sizes:")
        print("  0.05m (5cm)  - High detail, larger file")
        print("  0.1m  (10cm) - Balanced (recommended)")
        print("  0.2m  (20cm) - Fast loading, less detail")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_path = sys.argv[2]
    
    # Optional parameters
    height_threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    voxel_size = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
    
    process_map(input_path, output_path, height_threshold, voxel_size)

if __name__ == "__main__":
    main()
