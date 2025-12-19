#!/usr/bin/env python3
"""
Save and process FAST-LIO map with a single command.
Collects map from running FAST-LIO node.
"""

import sys
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import math

def collect_map_from_fast_lio(output_file, duration=0, min_distance=0.2):
    """
    Collect point clouds from running FAST-LIO node.
    Works around the segfault that prevents normal shutdown saving.
    
    Args:
        output_file: Path to save the raw PCD file
        duration: How long to collect points (seconds), 0=indefinitely
        min_distance: Minimum distance from LiDAR origin (meters), filters close obstructions
    """
    print("\n" + "=" * 60)
    print("Step 1: Collecting map from FAST-LIO")
    print("=" * 60)
    print(f"Subscribing to /cloud_registered and /Odometry...")
    if duration > 0:
        print(f"Will collect for {duration} seconds while you move around.")
    else:
        print(f"Will collect until you press Ctrl+C.")
    print(f"Press Ctrl+C to save and continue.")
    print(f"Filtering points closer than {min_distance*100:.0f}cm to LiDAR (using odometry).\n")
    
    class MapCollector(Node):
        def __init__(self):
            super().__init__('map_collector')
            self.points = []
            self.filtered_count = 0
            self.current_position = None
            
            # Subscribe to odometry to know robot position
            self.odom_sub = self.create_subscription(
                Odometry,
                '/Odometry',
                self.odom_callback,
                10
            )
            
            # Subscribe to point cloud in global frame
            self.subscription = self.create_subscription(
                PointCloud2,
                '/cloud_registered',
                self.callback,
                10
            )
        
        def odom_callback(self, msg):
            # Store current robot position
            self.current_position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )
            
        def callback(self, msg):
            if self.current_position is None:
                return  # Wait for odometry
            
            robot_x, robot_y, robot_z = self.current_position
            
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity")):
                x, y, z, intensity = point
                
                # Calculate distance from point to robot position (approximates LiDAR position)
                dx = x - robot_x
                dy = y - robot_y
                dz = z - robot_z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Filter out points too close to LiDAR
                if distance >= min_distance:
                    self.points.append(point)
                else:
                    self.filtered_count += 1
            
            if len(self.points) % 50000 == 0 and len(self.points) > 0:
                print(f"  Collected {len(self.points):,} points (filtered {self.filtered_count:,} close points)...")
    
    rclpy.init()
    node = MapCollector()
    
    start_time = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if duration > 0 and (time.time() - start_time) > duration:
                print(f"\n{duration} seconds elapsed.")
                break
                
    except KeyboardInterrupt:
        print("\nCtrl+C detected, stopping collection...")
    
    total_points = len(node.points)
    
    try:
        node.destroy_node()
    except:
        pass
    
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except:
        pass
    
    if total_points == 0:
        print("ERROR: No points collected! Is FAST-LIO mapping running?")
        print("Check: ros2 topic hz /cloud_registered")
        print("Check: ros2 topic hz /Odometry")
        return False
    
    # Save to PCD
    print(f"\nSaving {total_points:,} points to {output_file}...")
    with open(output_file, 'w') as f:
        # PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {total_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {total_points}\n")
        f.write("DATA ascii\n")
        
        # Point data
        for point in node.points:
            x, y, z, intensity = point
            f.write(f"{x} {y} {z} {intensity}\n")
    
    print(f"✓ Raw map saved!")
    print(f"✓ File: {output_file}")
    print(f"✓ Points: {total_points:,}")
    print(f"✓ Filtered: {node.filtered_count:,} close points ({min_distance*100:.0f}cm threshold)")
    
    file_size_mb = os.path.getsize(output_file) / (1024 * 1024)
    print(f"✓ Size: {file_size_mb:.1f} MB")
    return True

def save_and_process_map(map_name, height_threshold=1.0, voxel_size=0.1, collect_duration=0, min_distance=0.2):
    """
    Save map from FAST-LIO and process it automatically.
    
    Args:
        map_name: Base name for the map (without extension)
        height_threshold: Z threshold for ground removal (default 1.0m)
        voxel_size: Voxel size for downsampling (default 0.1m)
        collect_duration: How long to collect (seconds), 0=until Ctrl+C
        min_distance: Minimum distance filter (meters), filters close obstructions
    """
    # Find workspace root dynamically (go up from slam/utilities to repo root)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    slam_dir = os.path.dirname(script_dir)  # slam folder
    src_dir = os.path.dirname(slam_dir)  # src folder
    workspace_dir = os.path.dirname(src_dir)  # repo root
    
    # Try multiple possible locations for PCD directory
    pcd_locations = [
        os.path.join(workspace_dir, "ros2_ws", "src", "FAST_LIO_LOCALIZATION2", "PCD"),
        os.path.join(workspace_dir, "FAST_LIO_LOCALIZATION2", "PCD"),
        os.path.join(workspace_dir, "src", "FAST_LIO_LOCALIZATION2", "PCD"),
    ]
    
    pcd_dir = None
    for location in pcd_locations:
        if os.path.exists(location):
            pcd_dir = location
            break
    
    if pcd_dir is None:
        # Default to ros2_ws location and create it
        pcd_dir = pcd_locations[0]
        os.makedirs(pcd_dir, exist_ok=True)
    
    raw_map_path = f"{pcd_dir}/{map_name}_raw.pcd"
    clean_map_path = f"{pcd_dir}/{map_name}.pcd"
    
    print("=" * 60)
    print("FAST-LIO Map Save & Process")
    print("=" * 60)
    print(f"\nMap name: {map_name}")
    print(f"Raw map: {raw_map_path}")
    print(f"Processed map: {clean_map_path}")
    print(f"Min distance filter: {min_distance*100:.0f}cm")
    print(f"Height threshold: {height_threshold}m")
    print(f"Voxel size: {voxel_size}m")
    if collect_duration > 0:
        print(f"Collection time: {collect_duration}s")
    else:
        print(f"Collection time: Until Ctrl+C")
    
    # Step 1: Collect map from FAST-LIO
    if not collect_map_from_fast_lio(raw_map_path, collect_duration, min_distance):
        print("\nERROR: Failed to collect map from FAST-LIO!")
        sys.exit(1)
    
    # Step 2: Process the map
    print("\n" + "=" * 60)
    print("Step 2: Processing map...")
    print("=" * 60)
    
    # Get the script directory (utilities folder) and find process_map.py
    script_dir = os.path.dirname(os.path.abspath(__file__))
    slam_dir = os.path.dirname(script_dir)  # Go up to slam folder
    process_script = os.path.join(slam_dir, "map_processing", "process_map.py")
    
    if not os.path.exists(process_script):
        print(f"\nERROR: process_map.py not found at {process_script}")
        sys.exit(1)
    
    try:
        # Run process_map.py directly (no venv needed)
        cmd = f"python3 {process_script} {raw_map_path} {clean_map_path} {height_threshold} {voxel_size}"
        
        result = subprocess.run(
            ["bash", "-c", cmd],
            text=True,
            timeout=300  # 5 minutes max
        )
        
        if result.returncode != 0:
            print(f"\nERROR: Map processing failed!")
            sys.exit(1)
        
    except subprocess.TimeoutExpired:
        print("\nERROR: Map processing timed out!")
        sys.exit(1)
    except Exception as e:
        print(f"\nERROR during processing: {e}")
        sys.exit(1)
    
    # Final summary
    print("\n" + "=" * 60)
    print("COMPLETE!")
    print("=" * 60)
    print(f"\nRaw map:       {raw_map_path}")
    print(f"Processed map: {clean_map_path}")
    print(f"\nUse the processed map for localization:")
    print(f"  ros2 launch fast_lio_localization localization_with_lidar.launch.py \\")
    print(f"      map:={clean_map_path}")
    print()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 save_and_process.py <map_name> [height_threshold] [voxel_size] [collect_duration] [min_distance]")
        print("\nCollects map from FAST-LIO and automatically processes it.")
        print("IMPORTANT: Run this WHILE mapping.launch.py is running!")
        print("\nExamples:")
        print("  # Default: collect until Ctrl+C, filter 20cm, 1.0m threshold, 10cm voxels")
        print("  python3 save_and_process.py office")
        print()
        print("  # Collect for 60 seconds")
        print("  python3 save_and_process.py warehouse 0.8 0.1 60")
        print()
        print("  # Custom min distance filter (30cm)")
        print("  python3 save_and_process.py lab 1.0 0.05 0 0.3")
        print("\nParameters:")
        print("  map_name:         Name for the map (without .pcd extension)")
        print("  height_threshold: Remove points above this Z (default: 1.0m)")
        print("  voxel_size:       Downsample resolution (default: 0.1m)")
        print("  collect_duration: Seconds to collect, 0=until Ctrl+C (default: 0)")
        print("  min_distance:     Filter points closer than this (default: 0.2m = 20cm)")
        print("\nWorkflow:")
        print("  1. Start: ros2 launch fast_lio_localization mapping.launch.py")
        print("  2. Move robot around to build map")
        print("  3. Run this script to collect and save the map")
        print("  4. Press Ctrl+C when done")
        print("  5. Map will be processed and saved automatically")
        sys.exit(1)
    
    map_name = sys.argv[1]
    
    # Remove .pcd extension if user added it
    if map_name.endswith('.pcd'):
        map_name = map_name[:-4]
    
    # Optional parameters
    height_threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    voxel_size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    collect_duration = int(sys.argv[4]) if len(sys.argv) > 4 else 0  # Default: until Ctrl+C
    min_distance = float(sys.argv[5]) if len(sys.argv) > 5 else 0.2  # Default: 20cm
    
    save_and_process_map(map_name, height_threshold, voxel_size, collect_duration, min_distance)

if __name__ == "__main__":
    main()
