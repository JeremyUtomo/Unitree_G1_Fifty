#!/usr/bin/env python3
"""
RViz2-based navigation with obstacle avoidance for Unitree G1.
Uses A* path planning with 30cm safety margin around obstacles from map.

Usage:
    python3 rviz_navigation_obstacle_avoidance.py <network_interface> [options]
    
Examples:
    python3 rviz_navigation_obstacle_avoidance.py enp49s0 --speed 0.3
"""

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import sys
import time
import numpy as np
from collections import deque
import argparse

# Unitree G1 SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class ObstacleMap:
    """2D occupancy grid for obstacle detection and path planning."""
    
    def __init__(self, resolution=0.1, obstacle_margin=0.3, map_size=100, min_obstacle_points=5):
        """
        Args:
            resolution: Grid cell size in meters (0.1m = 10cm cells)
            obstacle_margin: Safety distance around obstacles in meters (0.3m)
            map_size: Map dimensions in meters (centered at origin)
            min_obstacle_points: Minimum points to consider as obstacle (5 points)
        """
        self.resolution = resolution
        self.obstacle_margin = obstacle_margin
        self.map_size = map_size
        self.min_obstacle_points = min_obstacle_points
        
        # Grid dimensions
        self.grid_size = int(map_size / resolution)
        self.origin_x = -map_size / 2
        self.origin_y = -map_size / 2
        
        # Occupancy grid: 0 = free, 1 = obstacle
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        # Margin expansion
        self.margin_cells = int(obstacle_margin / resolution)
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates."""
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return x, y
    
    def is_valid(self, grid_x, grid_y):
        """Check if grid coordinates are within bounds."""
        return 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size
    
    def update_from_pointcloud(self, points, robot_x, robot_y, height_min=0.4, height_max=2.0):
        """Update occupancy grid from point cloud data.
        
        Height filtering:
        - min 0.4m: Ignore floor and robot's feet
        - max 2.0m: Ignore ceiling/overhead structures
        
        Obstacle filtering:
        - Ignore small clusters (< min_obstacle_points)
        """
        # Clear previous obstacles
        self.grid.fill(0)
        
        if len(points) == 0:
            return
        
        # Filter points by height (ignore floor and ceiling)
        valid_points = points[(points[:, 2] >= height_min) & (points[:, 2] <= height_max)]
        
        # Group points into grid cells and count
        cell_counts = {}
        for point in valid_points:
            x, y = point[0], point[1]
            grid_x, grid_y = self.world_to_grid(x, y)
            
            if self.is_valid(grid_x, grid_y):
                cell_key = (grid_y, grid_x)
                cell_counts[cell_key] = cell_counts.get(cell_key, 0) + 1
        
        # Mark obstacle cells only if they have enough points
        for (grid_y, grid_x), count in cell_counts.items():
            if count >= self.min_obstacle_points:
                self.grid[grid_y, grid_x] = 1
        
        # Expand obstacles by safety margin
        self._expand_obstacles()
        
    def _expand_obstacles(self):
        """Expand obstacles by safety margin using dilation."""
        if self.margin_cells <= 0:
            return
        
        # Create kernel for dilation
        kernel_size = 2 * self.margin_cells + 1
        kernel = np.zeros((kernel_size, kernel_size), dtype=np.uint8)
        
        # Circular kernel
        center = self.margin_cells
        for i in range(kernel_size):
            for j in range(kernel_size):
                if (i - center) ** 2 + (j - center) ** 2 <= self.margin_cells ** 2:
                    kernel[i, j] = 1
        
        # Dilate obstacles
        expanded = np.zeros_like(self.grid)
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if self.grid[y, x] == 1:
                    # Apply kernel
                    for ky in range(kernel_size):
                        for kx in range(kernel_size):
                            if kernel[ky, kx] == 1:
                                ny = y + ky - center
                                nx = x + kx - center
                                if self.is_valid(nx, ny):
                                    expanded[ny, nx] = 1
        
        self.grid = expanded
    
    def is_obstacle(self, x, y):
        """Check if world coordinate has obstacle."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if not self.is_valid(grid_x, grid_y):
            return True  # Out of bounds = obstacle
        return self.grid[grid_y, grid_x] == 1


class AStarPlanner:
    """A* path planning algorithm."""
    
    def __init__(self, obstacle_map):
        self.map = obstacle_map
        
    def plan(self, start_x, start_y, goal_x, goal_y):
        """
        Plan path from start to goal using A*.
        Returns list of (x, y) waypoints or None if no path found.
        """
        # Convert to grid coordinates
        start_gx, start_gy = self.map.world_to_grid(start_x, start_y)
        goal_gx, goal_gy = self.map.world_to_grid(goal_x, goal_y)
        
        # Check if goal is valid
        if not self.map.is_valid(goal_gx, goal_gy):
            print(f"Goal position out of map bounds")
            return None
        
        if self.map.grid[goal_gy, goal_gx] > 0:
            print(f"Goal position is inside obstacle or too close (30cm safety margin)")
            return None
        
        # A* search
        open_set = []
        open_set.append((0, start_gx, start_gy))
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        f_score = {(start_gx, start_gy): self._heuristic(start_gx, start_gy, goal_gx, goal_gy)}
        
        # 8-connected grid
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        max_iterations = 10000
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_score
            open_set.sort(key=lambda x: x[0])
            _, current_x, current_y = open_set.pop(0)
            
            # Check if goal reached
            if current_x == goal_gx and current_y == goal_gy:
                # Reconstruct path
                path = self._reconstruct_path(came_from, (current_x, current_y))
                
                # Convert to world coordinates and simplify
                world_path = []
                for gx, gy in path:
                    wx, wy = self.map.grid_to_world(gx, gy)
                    world_path.append((wx, wy))
                
                # Simplify path (remove redundant waypoints)
                simplified_path = self._simplify_path(world_path)
                
                print(f"Path found with {len(simplified_path)} waypoints (searched {iterations} cells)")
                return simplified_path
            
            # Explore neighbors
            for dx, dy in neighbors:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                
                # Check validity
                if not self.map.is_valid(neighbor_x, neighbor_y):
                    continue
                
                if self.map.grid[neighbor_y, neighbor_x] == 1:
                    continue  # Obstacle
                
                # Calculate cost
                move_cost = math.sqrt(dx**2 + dy**2)  # Diagonal = 1.414, straight = 1.0
                tentative_g = g_score[(current_x, current_y)] + move_cost
                
                if (neighbor_x, neighbor_y) not in g_score or tentative_g < g_score[(neighbor_x, neighbor_y)]:
                    # Better path found
                    came_from[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    g_score[(neighbor_x, neighbor_y)] = tentative_g
                    f = tentative_g + self._heuristic(neighbor_x, neighbor_y, goal_gx, goal_gy)
                    f_score[(neighbor_x, neighbor_y)] = f
                    
                    # Add to open set if not already there
                    if not any(x == neighbor_x and y == neighbor_y for _, x, y in open_set):
                        open_set.append((f, neighbor_x, neighbor_y))
        
        print(f"No path found after {iterations} iterations")
        return None
    
    def _heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from A* search."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def _simplify_path(self, path, tolerance=0.5):
        """Simplify path by removing redundant waypoints using Ramer-Douglas-Peucker."""
        if len(path) <= 2:
            return path
        
        # Use simple decimation - keep every Nth point plus start/end
        simplified = [path[0]]
        step = max(1, int(tolerance / self.map.resolution))
        
        for i in range(step, len(path) - 1, step):
            simplified.append(path[i])
        
        simplified.append(path[-1])  # Always keep goal
        
        return simplified


class ObstacleAvoidanceNavigator(Node):
    def __init__(self, network_interface, goal_tolerance=0.4, max_speed=0.5):
        super().__init__('obstacle_avoidance_navigator')
        
        self.network_interface = network_interface
        self.goal_tolerance = goal_tolerance
        self.max_speed = max_speed
        
        # Robot state
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        
        # Goal and path
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.waypoints = []  # Path waypoints
        self.current_waypoint_idx = 0
        self.has_goal = False
        
        # Obstacle map and planner
        self.obstacle_map = ObstacleMap(resolution=0.1, obstacle_margin=0.3, map_size=100)
        self.planner = AStarPlanner(self.obstacle_map)
        self.map_updated = False
        
        # Safety stop for nearby obstacles
        self.safety_min_distance = 0.10  # 10cm minimum
        self.safety_max_distance = 0.60  # 60cm maximum
        self.obstacle_detected = False
        self.last_obstacle_check_time = time.time()
        
        # Subscribe to odometry
        self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        
        # Subscribe to goal pose from RViz2
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Subscribe to submap for obstacle detection
        self.create_subscription(PointCloud2, '/submap', self.submap_callback, 10)
        
        # Subscribe to live LiDAR scan for safety stop (registered cloud from FAST-LIO)
        self.create_subscription(PointCloud2, '/cloud_registered', self.safety_scan_callback, 10)
        
        # Publisher for planned path visualization
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Initialize SDK
        ChannelFactoryInitialize(0, network_interface)
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        time.sleep(0.5)
        self.loco_client.Start()
        time.sleep(0.5)
        print("Unitree G1 LocoClient initialized")
        
        # Control loop
        self.create_timer(0.1, self.control_loop)
        
        print("\n" + "=" * 70)
        print("RViz2 Navigation with Obstacle Avoidance")
        print("=" * 70)
        print("Features:")
        print("  • A* path planning with 30cm safety margin around obstacles")
        print("  • Real-time obstacle detection from /submap point cloud")
        print("  • Height filtering: 0.4m-2.0m (ignores floor and robot's feet)")
        print("  • Obstacle filtering: Minimum 5 points per cell (ignores noise)")
        print("  • Automatic waypoint navigation along planned path")
        print("")
        print("How to use:")
        print("  1. Ensure FAST-LIO localization is running")
        print("  2. Click '2D Goal Pose' in RViz2 to set destination")
        print("  3. Robot plans collision-free path and navigates")
        print("")
        print(f"Settings:")
        print(f"  Safety margin: 30cm around obstacles")
        print(f"  Goal tolerance: {goal_tolerance:.2f}m")
        print(f"  Max speed: {max_speed:.2f}m/s")
        print("=" * 70 + "\n")
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        """Update robot position."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
    
    def safety_scan_callback(self, msg):
        """Check for nearby obstacles in real-time for safety stop."""
        if self.current_x is None or not self.has_goal:
            return
        
        # Throttle checks to avoid excessive processing (check every 0.2 seconds)
        current_time = time.time()
        if current_time - self.last_obstacle_check_time < 0.2:
            return
        self.last_obstacle_check_time = current_time
        
        # Extract points from live scan
        min_distance = float('inf')
        obstacle_count = 0
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            # Calculate distance from LiDAR (which is at robot position)
            dx = point[0] - self.current_x
            dy = point[1] - self.current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Count points in safety zone
            if self.safety_min_distance < distance < self.safety_max_distance:
                obstacle_count += 1
                if distance < min_distance:
                    min_distance = distance
        
        # Check if obstacle is in safety zone (require at least 10 points to avoid false positives)
        if obstacle_count > 10:
            if not self.obstacle_detected:
                print(f"\n  SAFETY STOP: Obstacle detected at {min_distance*100:.0f}cm - pausing navigation")
                self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                print(f"\n  Path clear - resuming navigation")
                self.obstacle_detected = False
    
    def submap_callback(self, msg):
        """Update obstacle map from point cloud."""
        if self.current_x is None:
            return
        
        # Extract points from PointCloud2
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        if len(points) > 0:
            points = np.array(points)
            self.obstacle_map.update_from_pointcloud(points, self.current_x, self.current_y)
            self.map_updated = True
    
    def goal_callback(self, msg):
        """Receive goal from RViz2 and plan path."""
        if self.current_x is None or not self.map_updated:
            print("Waiting for localization and map data...")
            return
        
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Extract goal orientation
        q = msg.pose.orientation
        self.goal_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        print(f"\nNew goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) heading {math.degrees(self.goal_yaw):.1f}°")
        print("Planning path with obstacle avoidance...")
        
        # Plan path
        path = self.planner.plan(self.current_x, self.current_y, self.goal_x, self.goal_y)
        
        if path is None:
            print("Cannot plan safe path to goal")
            self.has_goal = False
            return
        
        # Store waypoints
        self.waypoints = path
        self.current_waypoint_idx = 0
        self.has_goal = True
        
        # Publish path for visualization
        self.publish_path(path)
        
        print(f"Path planned! Following {len(path)} waypoints")
    
    def publish_path(self, waypoints):
        """Publish planned path for RViz visualization."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def control_loop(self):
        """Main navigation control loop."""
        if not self.has_goal or self.current_x is None:
            return
        
        # SAFETY CHECK: Stop if obstacle detected nearby
        if self.obstacle_detected:
            self.send_velocity(0.0, 0.0, 0.0)
            return
        
        # Check if reached final goal
        distance_to_goal = math.sqrt(
            (self.goal_x - self.current_x) ** 2 +
            (self.goal_y - self.current_y) ** 2
        )
        
        if distance_to_goal < self.goal_tolerance:
            # Align to goal orientation
            heading_error = self.goal_yaw - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            
            print(f"\nAt goal! Current yaw: {math.degrees(self.current_yaw):.1f}° | Target yaw: {math.degrees(self.goal_yaw):.1f}° | Error: {math.degrees(heading_error):.1f}°")
            
            if abs(heading_error) < math.radians(10):
                print(f"Goal reached! Final position: ({self.current_x:.2f}, {self.current_y:.2f}) yaw: {math.degrees(self.current_yaw):.1f}°")
                self.stop_robot()
                self.has_goal = False
                return
            else:
                # Rotate to goal heading
                angular_vel = np.clip(heading_error * 0.8, -0.8, 0.8)
                print(f"  Aligning to goal orientation: {math.degrees(heading_error):.1f}° remaining", end='\r')
                self.send_velocity(0.0, 0.0, angular_vel) 
                return
        
        # Follow waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        # Get current target waypoint
        wx, wy = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance and heading to waypoint
        dx = wx - self.current_x
        dy = wy - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Debug: print navigation state
        if self.current_waypoint_idx == 0 or distance > 1.0:  # Only print at start or when far from waypoint
            print(f"\nNavigation: Robot@({self.current_x:.2f}, {self.current_y:.2f}) yaw={math.degrees(self.current_yaw):.1f}°")
            print(f"   Target@({wx:.2f}, {wy:.2f}) | dx={dx:.2f}, dy={dy:.2f}")
            print(f"   Target heading={math.degrees(target_heading):.1f}° | Error={math.degrees(heading_error):.1f}°")
        
        # Check if waypoint reached
        if distance < 0.5:  # Waypoint tolerance
            self.current_waypoint_idx += 1
            print(f"  Waypoint {self.current_waypoint_idx}/{len(self.waypoints)} reached")
            return
        
        # Navigate to waypoint
        if abs(heading_error) > math.radians(30):
            # Rotate in place if too far off
            angular_vel = np.clip(heading_error * 0.8, -0.8, 0.8)
            print(f"  Rotating to waypoint: {math.degrees(heading_error):.1f}° off", end='\r')
            self.send_velocity(0.0, 0.0, -angular_vel) 
        else:
            # Move forward with gentle heading correction
            linear_vel = min(self.max_speed, distance * 0.5)
            angular_vel = np.clip(heading_error * 0.3, -0.4, 0.4)
            
            # Move forward ONLY - NO strafing (vx=forward, vy=0)
            print(f"  Moving forward: {linear_vel:.2f} m/s | Heading error: {math.degrees(heading_error):.1f}°", end='\r')
            self.send_velocity(linear_vel, 0.0, -angular_vel)
        
        # Status
        print(f"  Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} | "
              f"Dist: {distance:.2f}m | Goal: {distance_to_goal:.2f}m", end='\r')
    
    def send_velocity(self, vx, vy, vyaw):
        """Send velocity command to robot."""
        try:
            self.loco_client.Move(vx, vy, vyaw)
        except Exception as e:
            print(f"\nCommand failed: {e}")
    
    def stop_robot(self):
        """Stop robot movement."""
        try:
            self.loco_client.StopMove()
        except:
            pass


def main():
    parser = argparse.ArgumentParser(description='RViz2 navigation with obstacle avoidance')
    parser.add_argument('interface', help='Network interface (e.g., enp49s0)')
    parser.add_argument('--speed', type=float, default=0.5, help='Max speed in m/s (default: 0.5)')
    parser.add_argument('--goal-tol', type=float, default=0.4, help='Goal tolerance in meters (default: 0.4)')
    
    args = parser.parse_args()
    
    rclpy.init()
    navigator = ObstacleAvoidanceNavigator(
        network_interface=args.interface,
        goal_tolerance=args.goal_tol,
        max_speed=args.speed
    )
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        navigator.stop_robot()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
