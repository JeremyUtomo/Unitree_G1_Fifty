#!/usr/bin/env python3
"""
Goal-based navigation for Unitree G1 using FAST-LIO localization.
Combines SLAM position with G1 locomotion control to navigate to target points.

Usage:
    Single waypoint:
        python3.10 navigate_to_goal.py <x> <y> <network_interface> [options]
        python3.10 navigate_to_goal.py 5.0 3.0 enp2s0 --speed 0.3
    
    Multiple waypoints:
        python3.10 navigate_to_goal.py <x1> <y1> <x2> <y2> ... <network_interface> [options]
        python3.10 navigate_to_goal.py 2.0 0.0 4.0 1.0 4.0 -1.0 enp2s0 --speed 0.3
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
import time

# Unitree G1 SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class GoalNavigator(Node):
    def __init__(self, waypoints, goal_tolerance=0.3, angle_tolerance=10.0, max_speed=0.5):
        super().__init__('goal_navigator')
        
        # Waypoint parameters
        self.waypoints = waypoints  # List of (x, y) tuples
        self.current_waypoint_idx = 0
        self.goal_x = waypoints[0][0]
        self.goal_y = waypoints[0][1]
        self.goal_tolerance = goal_tolerance  # meters
        self.angle_tolerance = math.radians(angle_tolerance)  # convert to radians
        self.max_speed = max_speed  # m/s
        
        # Current robot state
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        
        # Navigation state
        self.goal_reached = False
        self.all_waypoints_reached = False
        self.navigation_active = True
        
        # Progress tracking (detect if stuck)
        self.last_distance = None
        self.stuck_start_time = None
        self.max_stuck_time = 8.0  # seconds - if no progress, try different approach
        
        # Rotation mode tracking
        self.pure_rotation_start_time = None
        self.max_pure_rotation_time = 5.0  # seconds - increased from 3s for full rotation
        
        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        # Initialize Unitree G1 LocoClient
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        time.sleep(0.5)
        
        # Ensure robot is in standing position and ready to move
        print("  Preparing robot for navigation...")
        try:
            # First check if we can get robot state
            fsm_id = 0
            self.loco_client.GetFsmId(fsm_id)
            print(f"  Robot FSM state: {fsm_id}")
        except:
            pass  # GetFsmId might not be available in Python bindings
        
        # Start movement control mode
        self.loco_client.Start()
        time.sleep(0.5)
        
        print("Unitree G1 LocoClient initialized and ready")
        
        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        print("\n" + "=" * 70)
        print("Waypoint Navigation")
        print("=" * 70)
        print(f"Total waypoints: {len(self.waypoints)}")
        for i, (wx, wy) in enumerate(self.waypoints):
            print(f"  Waypoint {i+1}: ({wx:.2f}, {wy:.2f})")
        print(f"Tolerance: {goal_tolerance:.2f}m")
        print(f"Max speed: {max_speed:.2f}m/s")
        print(f"Angle tolerance: {math.degrees(self.angle_tolerance):.1f}°")
        print("=" * 70)
        print(f"\nNavigating to Waypoint 1/{len(self.waypoints)}: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        print("Waiting for localization data...")
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Extract yaw angle from quaternion."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        """Update current robot position from localization."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
    
    def control_loop(self):
        """Main navigation control loop."""
        if not self.navigation_active or self.all_waypoints_reached:
            return
        
        if self.current_x is None:
            return  # Waiting for first odometry message
        
        # Calculate distance and angle to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        
        # Calculate heading error
        heading_error = angle_to_goal - self.current_yaw
        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Check if current waypoint reached
        if distance < self.goal_tolerance:
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= len(self.waypoints):
                # All waypoints completed!
                self.stop_robot()
                self.all_waypoints_reached = True
                print(f"\nAll waypoints reached!")
                print(f"  Final position: ({self.current_x:.3f}, {self.current_y:.3f})")
                print(f"  Total waypoints completed: {len(self.waypoints)}")
                return
            else:
                # Move to next waypoint
                self.goal_x = self.waypoints[self.current_waypoint_idx][0]
                self.goal_y = self.waypoints[self.current_waypoint_idx][1]
                self.goal_reached = False
                
                print(f"\nWaypoint {self.current_waypoint_idx}/{len(self.waypoints)} reached!")
                print(f"  Position: ({self.current_x:.3f}, {self.current_y:.3f})")
                print(f"Moving to Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: ({self.goal_x:.2f}, {self.goal_y:.2f})\n")
                
                # Brief pause at waypoint
                time.sleep(0.5)
                return
        
        # Display status
        print(f"WP {self.current_waypoint_idx + 1}/{len(self.waypoints)} | "
              f"Distance: {distance:>6.3f}m | "
              f"Heading error: {math.degrees(heading_error):>7.2f}° | "
              f"Position: ({self.current_x:>6.2f}, {self.current_y:>6.2f})")
        
        # Check if we're stuck (no progress in distance)
        if self.last_distance is not None:
            distance_change = abs(self.last_distance - distance)
            if distance_change < 0.05:  # Less than 5cm progress
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                elif time.time() - self.stuck_start_time > self.max_stuck_time:
                    print(f"  Warning: No progress for {self.max_stuck_time}s - may be stuck!")
                    self.stuck_start_time = None  # Reset
            else:
                self.stuck_start_time = None  # Making progress, reset timer
        self.last_distance = distance
        
        # Navigation strategy based on heading error
        # Rotate in place for moderate to large heading errors
        rotation_threshold = math.radians(45)  # 45 degrees - rotate in place beyond this
        
        if abs(heading_error) > rotation_threshold:
            # Moderate to large heading error - rotate in place first
            if self.pure_rotation_start_time is None:
                self.pure_rotation_start_time = time.time()
                print(f"  → Heading error {math.degrees(heading_error):.1f}° - rotating in place...")
            
            # Check if we've timed out
            if time.time() - self.pure_rotation_start_time > self.max_pure_rotation_time:
                # Timeout - force forward motion even if heading is bad
                print(f"  Rotation timeout ({self.max_pure_rotation_time}s) - forcing forward motion!")
                self.pure_rotation_start_time = None
                linear_vel = self.calculate_linear_velocity(distance) * 0.3  # Reduce speed when heading is bad
                angular_vel = self.calculate_angular_velocity(heading_error)  # Max correction
            else:
                # Pure rotation - no forward movement
                linear_vel = 0.0
                angular_vel = self.calculate_angular_velocity(heading_error)
                print(f"     [Rotating: heading_error={math.degrees(heading_error):.1f}°, angular_vel={angular_vel:.3f} rad/s]")
                angular_vel = self.calculate_angular_velocity(heading_error)
                print(f"     [Rotating: heading_error={math.degrees(heading_error):.1f}°, angular_vel={angular_vel:.3f} rad/s]")
        else:
            # Heading error small - move forward with gentle correction
            self.pure_rotation_start_time = None  # Reset rotation timer
            linear_vel = self.calculate_linear_velocity(distance)
            
            # Scale angular correction based on heading error magnitude
            if abs(heading_error) > math.radians(20):  # 20-45 degrees
                # Moderate correction while moving
                angular_vel = self.calculate_angular_velocity(heading_error) * 0.5
            else:
                # Small errors - gentle correction, focus on forward motion
                angular_vel = self.calculate_angular_velocity(heading_error) * 0.3
        
        # Send command to robot
        self.send_velocity_command(linear_vel, angular_vel)
    
    def calculate_linear_velocity(self, distance):
        """Calculate forward velocity based on distance to goal."""
        # Always maintain a minimum speed for stable walking
        min_speed = 0.25  # Increased minimum speed for reliable walking
        
        # Slow down as we approach the goal
        if distance < 1.0:
            speed = self.max_speed * distance  # Linear ramp down
        else:
            speed = self.max_speed
        
        # Keep moving at minimum speed until very close to goal
        if distance > self.goal_tolerance:  # Keep minimum speed until we reach goal tolerance
            return max(min_speed, min(speed, self.max_speed))
        else:
            # Within goal tolerance - final precision approach
            return max(0.1, min(speed, self.max_speed))
    
    def calculate_angular_velocity(self, heading_error):
        """Calculate angular velocity based on heading error."""
        # Stronger proportional control for better turning
        Kp = 0.5  # Increased from 0.3 for faster rotation
        angular_vel = Kp * heading_error
        
        # G1 uses inverted yaw convention - flip the sign
        angular_vel = -angular_vel
        
        # Limit angular velocity
        max_angular_vel = 0.6  # rad/s (increased from 0.4 for faster rotation)
        return max(-max_angular_vel, min(angular_vel, max_angular_vel))
    
    def send_velocity_command(self, linear_vel, angular_vel):
        """
        Send velocity command to Unitree G1.
        """
        try:
            # G1 LocoClient.Move(vx, vy, vyaw)
            # vx: forward velocity (m/s)
            # vy: lateral velocity (m/s) - we keep this 0 for forward motion
            # vyaw: rotational velocity (rad/s)
            self.loco_client.Move(linear_vel, 0.0, angular_vel)
        except Exception as e:
            print(f"Error sending command: {e}")
    
    def stop_robot(self):
        """Stop the robot."""
        print("\nStopping robot...")
        
        # Stop movement
        try:
            self.loco_client.StopMove()
            print("Robot stopped")
        except Exception as e:
            print(f"Error stopping robot: {e}")
        
        self.navigation_active = False
    
    def shutdown(self):
        """Clean shutdown."""
        self.stop_robot()
        
        # Give robot time to process stop command
        time.sleep(0.2)
        
        # Clean up LocoClient
        try:
            # Don't call any more methods - just clear reference
            self.loco_client = None
        except:
            pass
        
        print("Navigation shutdown complete")


def main():
    if len(sys.argv) < 4:
        print("Usage: python3.10 navigate_to_goal.py <x1> <y1> [x2 y2 ...] <network_interface> [options]")
        print("\nRequired:")
        print("  x, y              Target coordinates in map frame (meters)")
        print("                    Can specify multiple waypoints as x1 y1 x2 y2 x3 y3 ...")
        print("  network_interface Network interface name (e.g., enp2s0, eth0)")
        print("\nOptional:")
        print("  --speed <value>   Maximum speed in m/s (default: 0.5)")
        print("  --tolerance <val> Goal tolerance in meters (default: 0.3)")
        print("  --angle-tol <val> Angle tolerance in degrees (default: 10)")
        print("\nExamples:")
        print("  Single waypoint:")
        print("    python3.10 navigate_to_goal.py 5.0 3.0 enp2s0")
        print("  Multiple waypoints:")
        print("    python3.10 navigate_to_goal.py 2.0 0.0 4.0 1.0 4.0 -1.0 enp2s0 --speed 0.3")
        print("\nNote: Make sure FAST-LIO localization is running first!")
        sys.exit(1)
    
    # Parse waypoints and network interface
    # Strategy: Find network interface (last non-option argument that's not a number)
    # Everything before it should be coordinate pairs
    
    args = sys.argv[1:]
    network_interface = None
    waypoint_args = []
    
    # Find where optional arguments start
    option_start = len(args)
    for i, arg in enumerate(args):
        if arg.startswith('--'):
            option_start = i
            break
    
    # Last argument before options should be network interface
    if option_start > 0:
        network_interface = args[option_start - 1]
        waypoint_args = args[:option_start - 1]
    
    # Validate waypoints (must be pairs of numbers)
    if len(waypoint_args) < 2 or len(waypoint_args) % 2 != 0:
        print("Error: Waypoints must be specified as pairs of coordinates (x y)")
        print(f"Got {len(waypoint_args)} values, expected an even number")
        sys.exit(1)
    
    # Parse waypoints
    try:
        waypoints = []
        for i in range(0, len(waypoint_args), 2):
            x = float(waypoint_args[i])
            y = float(waypoint_args[i + 1])
            waypoints.append((x, y))
    except ValueError:
        print("Error: Invalid waypoint coordinates (must be numbers)")
        sys.exit(1)
    
    if not network_interface:
        print("Error: Network interface not specified")
        sys.exit(1)
    
    # Initialize SDK channel
    ChannelFactoryInitialize(0, network_interface)
    print(f"SDK initialized on network interface: {network_interface}")
    
    # Parse optional arguments from the remaining args
    max_speed = 0.5
    goal_tolerance = 0.3
    angle_tolerance = 10.0
    
    i = option_start
    while i < len(args):
        if args[i] == '--speed' and i + 1 < len(args):
            max_speed = float(args[i + 1])
            i += 2
        elif args[i] == '--tolerance' and i + 1 < len(args):
            goal_tolerance = float(args[i + 1])
            i += 2
        elif args[i] == '--angle-tol' and i + 1 < len(args):
            angle_tolerance = float(args[i + 1])
            i += 2
        else:
            i += 1
    
    # Initialize ROS
    rclpy.init()
    navigator = GoalNavigator(waypoints, goal_tolerance, angle_tolerance, max_speed)
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("\n\nNavigation cancelled by user.")
    except Exception as e:
        print(f"\n\nError during navigation: {e}")
    finally:
        try:
            navigator.shutdown()
        except Exception as e:
            print(f"Warning during shutdown: {e}")
        
        time.sleep(0.3)  # Give time for SDK cleanup
        
        try:
            navigator.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
