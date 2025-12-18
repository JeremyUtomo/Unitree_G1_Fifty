#!/usr/bin/env python3
import sys
import open3d as o3d
import numpy as np

def downsample_pcd(input_file, output_file, voxel_size=0.1):
    """
    Downsample a PCD file using voxel grid filtering
    
    Args:
        input_file: Path to input PCD file
        output_file: Path to output PCD file
        voxel_size: Voxel size for downsampling (larger = fewer points)
                    0.05 = 5cm, 0.1 = 10cm, 0.2 = 20cm
    """
    print(f"Loading point cloud from {input_file}...")
    pcd = o3d.io.read_point_cloud(input_file)
    
    num_points_original = len(pcd.points)
    print(f"Original point cloud has {num_points_original:,} points")
    
    print(f"Downsampling with voxel size {voxel_size}m...")
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    num_points_downsampled = len(pcd_downsampled.points)
    print(f"Downsampled point cloud has {num_points_downsampled:,} points")
    print(f"Reduction: {100 * (1 - num_points_downsampled/num_points_original):.1f}%")
    
    print(f"Saving to {output_file}...")
    o3d.io.write_point_cloud(output_file, pcd_downsampled)
    print("Done!")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 downsample_map.py <input.pcd> <output.pcd> [voxel_size]")
        print("Example: python3 downsample_map.py map.pcd map_downsampled.pcd 0.1")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    voxel_size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    
    downsample_pcd(input_file, output_file, voxel_size)
