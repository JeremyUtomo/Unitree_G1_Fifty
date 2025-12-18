#!/usr/bin/env python3
"""
RViz2-based navigation for Unitree G1 using FAST-LIO localization.
Click goal points in RViz2 using "2D Goal Pose" tool to navigate.

Usage:
    python3.10 rviz_navigation.py <network_interface> [options]
    
Examples:
    python3.10 rviz_navigation.py enp49s0 --speed 0.3
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import sys
import time

# Unitree G1 SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class RVizNavigator(Node):
    def __init__(self, goal_tolerance=0.3, angle_tolerance=10.0, max_speed=0.5):
        super().__init__('rviz_navigator')
        
        # Navigation parameters
        self.goal_tolerance = goal_tolerance  # meters
        self.angle_tolerance = math.radians(angle_tolerance)
        self.max_speed = max_speed  # m/s
        
        # Current robot state
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        
        # Goal state
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None  # Target orientation
        self.has_goal = False
        self.goal_reached = False
        self.position_reached = False  # Position reached, now aligning orientation
        
        # Navigation state
        self.navigation_active = False
        self.all_waypoints_reached = False
        
        # Progress tracking
        self.last_distance = None
        self.stuck_start_time = None
        self.max_stuck_time = 8.0
        
        # Rotation mode tracking
        self.pure_rotation_start_time = None
        self.max_pure_rotation_time = 5.0
        
        # Subscribe to odometry (current position)
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        # Subscribe to RViz2 goal pose (2D Goal Pose tool)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Initialize Unitree G1 LocoClient
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        time.sleep(0.5)
        
        print("  Preparing robot for navigation...")
        self.loco_client.Start()
        time.sleep(0.5)
        
        print("Unitree G1 LocoClient initialized and ready")
        
        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        print("\n" + "=" * 70)
        print("RViz2 Interactive Navigation")
        print("=" * 70)
        print("How to use:")
        print("  1. Make sure FAST-LIO localization is running")
        print("  2. Open RViz2 with the localization visualization")
        print("  3. Click '2D Goal Pose' button in RViz2 toolbar")
        print("  4. Click and drag on the map to set goal position and orientation")
        print("  5. Robot will automatically navigate to the goal!")
        print("")
        print(f"Settings:")
        print(f"  Goal tolerance: {goal_tolerance:.2f}m")
        print(f"  Max speed: {max_speed:.2f}m/s")
        print(f"  Angle tolerance: {math.degrees(self.angle_tolerance):.1f}°")
        print("=" * 70)
        print("\nWaiting for goals from RViz2...")
        print("(Use '2D Goal Pose' tool to set navigation targets)\n")
    
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
    
    def goal_callback(self, msg):
        """Receive new goal from RViz2."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Extract goal orientation - robot will match this at the end
        goal_qx = msg.pose.orientation.x
        goal_qy = msg.pose.orientation.y
        goal_qz = msg.pose.orientation.z
        goal_qw = msg.pose.orientation.w
        self.goal_yaw = self.quaternion_to_yaw(goal_qx, goal_qy, goal_qz, goal_qw)
        
        self.has_goal = True
        self.goal_reached = False
        self.position_reached = False
        self.navigation_active = True
        
        # Reset tracking variables
        self.last_distance = None
        self.stuck_start_time = None
        self.pure_rotation_start_time = None
        
        if self.current_x is not None:
            distance = math.sqrt((self.goal_x - self.current_x)**2 + 
                               (self.goal_y - self.current_y)**2)
            print(f"\n{'='*70}")
            print(f"New goal received from RViz2!")
            print(f"{'='*70}")
            print(f"  Goal position: ({self.goal_x:.2f}, {self.goal_y:.2f})")
            print(f"  Goal heading: {math.degrees(self.goal_yaw):.1f}°")
            print(f"  Current position: ({self.current_x:.2f}, {self.current_y:.2f})")
            print(f"  Distance: {distance:.2f}m")
            print(f"{'='*70}\n")
        else:
            print(f"\nGoal set to ({self.goal_x:.2f}, {self.goal_y:.2f}, heading: {math.degrees(self.goal_yaw):.1f}°) - waiting for localization...\n")
    
    def control_loop(self):
        """Main navigation control loop."""
        if not self.navigation_active or not self.has_goal or self.goal_reached:
            return
        
        if self.current_x is None:
            return  # Waiting for first odometry message
        
        # Calculate distance to goal position
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if position reached
        if distance < self.goal_tolerance and not self.position_reached:
            self.position_reached = True
            print(f"\nPosition reached! Now aligning to goal orientation...")
            print(f"  Position: ({self.current_x:.3f}, {self.current_y:.3f})")
            # Reset rotation timer for final orientation alignment
            self.pure_rotation_start_time = None
        
        # Phase 1: Navigate to position
        if not self.position_reached:
            angle_to_goal = math.atan2(dy, dx)
            
            # Calculate heading error to reach position
            heading_error = angle_to_goal - self.current_yaw
            # Normalize to [-pi, pi]
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # Display status
            print(f"Distance: {distance:>6.3f}m | "
                  f"Heading error: {math.degrees(heading_error):>7.2f}° | "
                  f"Position: ({self.current_x:>6.2f}, {self.current_y:>6.2f})")
            
            # Navigate to position (existing logic)
            self.navigate_to_position(distance, heading_error)
        
        # Phase 2: Align to goal orientation
        else:
            # Calculate final orientation error
            final_heading_error = self.goal_yaw - self.current_yaw
            # Normalize to [-pi, pi]
            while final_heading_error > math.pi:
                final_heading_error -= 2 * math.pi
            while final_heading_error < -math.pi:
                final_heading_error += 2 * math.pi
            
            # Check if orientation reached
            if abs(final_heading_error) < self.angle_tolerance:
                self.stop_robot()
                self.goal_reached = True
                self.navigation_active = False
                print(f"\nGoal fully reached!")
                print(f"  Final position: ({self.current_x:.3f}, {self.current_y:.3f})")
                print(f"  Final heading: {math.degrees(self.current_yaw):.1f}° (target: {math.degrees(self.goal_yaw):.1f}°)")
                print(f"  Heading error: {math.degrees(final_heading_error):.2f}°")
                print(f"\nReady for next goal (use '2D Goal Pose' in RViz2)\n")
                return
            
            # Display orientation alignment status
            print(f"Aligning orientation | "
                  f"Current: {math.degrees(self.current_yaw):>6.1f}° | "
                  f"Target: {math.degrees(self.goal_yaw):>6.1f}° | "
                  f"Error: {math.degrees(final_heading_error):>6.2f}°")
            
            # Rotate in place to match goal orientation
            linear_vel = 0.0
            angular_vel = self.calculate_angular_velocity(final_heading_error)
            self.send_velocity_command(linear_vel, angular_vel)
    
    def navigate_to_position(self, distance, heading_error):
        """Navigate to goal position (Phase 1)."""
        # Check if stuck
        if self.last_distance is not None:
            distance_change = abs(self.last_distance - distance)
            if distance_change < 0.05:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                elif time.time() - self.stuck_start_time > self.max_stuck_time:
                    print(f"  Warning: No progress for {self.max_stuck_time}s - may be stuck!")
                    self.stuck_start_time = None
            else:
                self.stuck_start_time = None
        self.last_distance = distance
        
        # Navigation strategy based on heading error
        # Rotate in place for moderate to large heading errors
        rotation_threshold = math.radians(45)  # 45 degrees - rotate in place beyond this
        
        if abs(heading_error) > rotation_threshold:
            # Moderate to large heading error - rotate in place first
            if self.pure_rotation_start_time is None:
                self.pure_rotation_start_time = time.time()
                print(f"  → Heading error {math.degrees(heading_error):.1f}° - rotating in place...")
            
            if time.time() - self.pure_rotation_start_time > self.max_pure_rotation_time:
                print(f"  Rotation timeout ({self.max_pure_rotation_time}s) - forcing forward motion!")
                self.pure_rotation_start_time = None
                linear_vel = self.calculate_linear_velocity(distance) * 0.3
                angular_vel = self.calculate_angular_velocity(heading_error)
            else:
                # Pure rotation - no forward movement
                linear_vel = 0.0
                angular_vel = self.calculate_angular_velocity(heading_error)
                print(f"     [Rotating: heading_error={math.degrees(heading_error):.1f}°, angular_vel={angular_vel:.3f} rad/s]")
        else:
            # Heading error small - move forward with gentle correction
            self.pure_rotation_start_time = None
            linear_vel = self.calculate_linear_velocity(distance)
            
            if abs(heading_error) > math.radians(20):
                # 20-45 degrees - moderate correction while moving
                angular_vel = self.calculate_angular_velocity(heading_error) * 0.5
            else:
                # < 20 degrees - gentle correction
                angular_vel = self.calculate_angular_velocity(heading_error) * 0.3
        
        # Send command to robot
        self.send_velocity_command(linear_vel, angular_vel)
    
    def calculate_linear_velocity(self, distance):
        """Calculate forward velocity based on distance to goal."""
        min_speed = 0.25
        
        if distance < 1.0:
            speed = self.max_speed * distance
        else:
            speed = self.max_speed
        
        if distance > self.goal_tolerance:
            return max(min_speed, min(speed, self.max_speed))
        else:
            return max(0.1, min(speed, self.max_speed))
    
    def calculate_angular_velocity(self, heading_error):
        """Calculate angular velocity based on heading error."""
        Kp = 0.5
        angular_vel = Kp * heading_error
        
        # G1 uses inverted yaw convention
        angular_vel = -angular_vel
        
        max_angular_vel = 0.6
        return max(-max_angular_vel, min(angular_vel, max_angular_vel))
    
    def send_velocity_command(self, linear_vel, angular_vel):
        """Send velocity command to Unitree G1."""
        try:
            self.loco_client.Move(linear_vel, 0.0, angular_vel)
        except Exception as e:
            print(f"Error sending command: {e}")
    
    def stop_robot(self):
        """Stop the robot."""
        print("\nStopping robot...")
        
        try:
            self.loco_client.StopMove()
            print("Robot stopped")
        except Exception as e:
            print(f"Error stopping robot: {e}")
        
        self.navigation_active = False
    
    def shutdown(self):
        """Clean shutdown."""
        self.stop_robot()
        
        time.sleep(0.2)
        
        try:
            self.loco_client = None
        except:
            pass
        
        print("Navigation shutdown complete")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3.10 rviz_navigation.py <network_interface> [options]")
        print("\nRequired:")
        print("  network_interface Network interface name (e.g., enp49s0, eth0)")
        print("\nOptional:")
        print("  --speed <value>   Maximum speed in m/s (default: 0.5)")
        print("  --tolerance <val> Goal tolerance in meters (default: 0.3)")
        print("  --angle-tol <val> Angle tolerance in degrees (default: 10)")
        print("\nExamples:")
        print("  python3.10 rviz_navigation.py enp49s0")
        print("  python3.10 rviz_navigation.py enp49s0 --speed 0.3 --tolerance 0.25")
        print("\nNote: Make sure FAST-LIO localization is running first!")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    
    # Initialize SDK channel
    ChannelFactoryInitialize(0, network_interface)
    print(f"SDK initialized on network interface: {network_interface}")
    
    # Parse optional arguments
    max_speed = 0.5
    goal_tolerance = 0.3
    angle_tolerance = 10.0
    
    i = 2
    while i < len(sys.argv):
        if sys.argv[i] == '--speed' and i + 1 < len(sys.argv):
            max_speed = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--tolerance' and i + 1 < len(sys.argv):
            goal_tolerance = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--angle-tol' and i + 1 < len(sys.argv):
            angle_tolerance = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1
    
    # Initialize ROS
    rclpy.init()
    navigator = RVizNavigator(goal_tolerance, angle_tolerance, max_speed)
    
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
        
        time.sleep(0.3)
        
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
