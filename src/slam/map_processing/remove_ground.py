#!/usr/bin/env python3
"""
Remove ground plane from PCD map.
Useful when LiDAR is mounted upside down and scans the floor.
"""

import sys
import open3d as o3d
import numpy as np

def remove_ground_plane(input_path, output_path, height_threshold=-0.5, ransac_iterations=1000):
    """
    Remove ground plane points from PCD map.
    
    Args:
        input_path: Path to input PCD file
        output_path: Path to save cleaned PCD file
        height_threshold: Points below this Z height are candidates for removal (meters)
        ransac_iterations: RANSAC iterations for plane detection
    """
    print(f"Loading point cloud from: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    
    num_points_original = len(pcd.points)
    print(f"Original point cloud: {num_points_original:,} points")
    
    # Get point cloud as numpy array
    points = np.asarray(pcd.points)
    
    # Show Z-axis range
    z_min, z_max = points[:, 2].min(), points[:, 2].max()
    print(f"\nZ-axis range: {z_min:.2f}m to {z_max:.2f}m")
    
    # Method 1: Simple height filtering
    print(f"\nRemoving points below Z = {height_threshold}m...")
    mask = points[:, 2] > height_threshold
    
    pcd_filtered = pcd.select_by_index(np.where(mask)[0])
    
    num_points_filtered = len(pcd_filtered.points)
    removed = num_points_original - num_points_filtered
    percent_removed = (removed / num_points_original) * 100
    
    print(f"\nFiltered point cloud: {num_points_filtered:,} points")
    print(f"Removed: {removed:,} points ({percent_removed:.1f}%)")
    
    # Save cleaned map
    print(f"\nSaving ground-removed map to: {output_path}")
    o3d.io.write_point_cloud(output_path, pcd_filtered)
    print("Done!")
    
    return pcd_filtered

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 remove_ground.py <input.pcd> <output.pcd> [height_threshold]")
        print("\nExample:")
        print("  python3 remove_ground.py scans.pcd scans_no_ground.pcd")
        print("  python3 remove_ground.py scans.pcd scans_no_ground.pcd -0.3")
        print("\nParameters:")
        print("  height_threshold: Remove points below this Z height in meters (default: -0.5)")
        print("                    Adjust based on your robot height and floor level")
        print("\nTips:")
        print("  - Run first without params to see Z-axis range")
        print("  - Set threshold just below lowest valid obstacle")
        print("  - If upside down LiDAR, ground will be at negative Z")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_path = sys.argv[2]
    
    # Optional parameter
    height_threshold = float(sys.argv[3]) if len(sys.argv) > 3 else -0.5
    
    remove_ground_plane(input_path, output_path, height_threshold)

if __name__ == "__main__":
    main()
