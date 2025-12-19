#!/usr/bin/env python3
"""
Clean map for localization by removing close points.
Filters out points within 3m of LiDAR position to remove robot body and obstructions.
Does NOT remove floor or downsample - keeps geometric features for ICP alignment.
"""

import sys
import subprocess
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import math

def collect_map_from_fast_lio(output_file, min_distance=3.0):
    """
    Collect point clouds from running FAST-LIO node.
    Filters out points within min_distance meters of LiDAR position.
    
    Args:
        output_file: Path to save the cleaned PCD file
        min_distance: Minimum distance from LiDAR origin (meters), default 3.0m
    """
    print("\n" + "=" * 60)
    print("Collecting Clean Map for Localization")
    print("=" * 60)
    print(f"Subscribing to /cloud_registered and /Odometry...")
    print(f"Filtering points closer than {min_distance}m to LiDAR.")
    print(f"Press Ctrl+C to save and exit.\n")
    
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
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C detected, stopping collection...")
    finally:
        # Ensure node cleanup
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore if already shutdown
    
    # Save to PCD file
    print(f"\nSaving {len(node.points):,} points to {output_file}...")
    
    if len(node.points) == 0:
        print("Warning: No points collected! Make sure FAST-LIO mapping is running.")
        print("    Check that /cloud_registered and /Odometry topics are publishing:")
        print("    ros2 topic list | grep -E 'cloud_registered|Odometry'")
        return
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # Write PCD file
    with open(output_file, 'w') as f:
        # PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(node.points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(node.points)}\n")
        f.write("DATA ascii\n")
        
        # Write points
        for point in node.points:
            f.write(f"{point[0]} {point[1]} {point[2]} {point[3]}\n")
    
    file_size_mb = os.path.getsize(output_file) / (1024 * 1024)
    
    print(f"Clean map saved!")
    print(f"File: {output_file}")
    print(f"Points: {len(node.points):,}")
    print(f"Filtered: {node.filtered_count:,} close points ({min_distance}m threshold)")
    print(f"Size: {file_size_mb:.1f} MB")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3.10 clean_for_localization.py <map_name> [min_distance_meters]")
        print("Example: python3.10 clean_for_localization.py office 3.0")
        print("\nThis will create a clean map for localization by filtering out points")
        print("within the specified distance (default 3.0m) of the LiDAR position.")
        print("The map will be saved to: src/FAST_LIO_LOCALIZATION2/PCD/<map_name>_localization.pcd")
        sys.exit(1)
    
    map_name = sys.argv[1]
    min_distance = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    
    # Find PCD directory dynamically
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
    
    output_file = os.path.join(pcd_dir, f"{map_name}_localization.pcd")
    
    print("\n" + "=" * 60)
    print("FAST-LIO Clean Map for Localization")
    print("=" * 60)
    print(f"\nMap name: {map_name}")
    print(f"Output file: {output_file}")
    print(f"Min distance filter: {min_distance}m")
    print(f"\nThis will create a map suitable for ICP localization by:")
    print(f"  - Removing points within {min_distance}m of LiDAR (filters robot body)")
    print(f"  - Keeping all other geometric features (floor, walls, etc.)")
    print(f"  - NO downsampling (preserves detail for ICP)")
    print()
    
    # Collect map from FAST-LIO
    collect_map_from_fast_lio(output_file, min_distance)
    
    print("\n" + "=" * 60)
    print("COMPLETE!")
    print("=" * 60)
    print(f"\nClean map: {output_file}")
    print(f"\nUse this map for localization:")
    print(f"  ros2 launch fast_lio_localization localization_with_lidar.launch.py \\")
    print(f"      map:={os.path.abspath(output_file)}")


if __name__ == "__main__":
    main()
