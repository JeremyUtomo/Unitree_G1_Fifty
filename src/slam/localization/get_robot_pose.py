#!/usr/bin/env python3
"""
Extract and display robot location from FAST-LIO localization.
Subscribes to /Odometry and prints position and orientation.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to euler angles (roll, pitch, yaw) in degrees.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class RobotPoseMonitor(Node):
    def __init__(self, mode='continuous', output_file=None):
        super().__init__('robot_pose_monitor')
        self.mode = mode
        self.output_file = output_file
        self.file_handle = None
        
        if self.output_file:
            self.file_handle = open(self.output_file, 'w')
            self.file_handle.write("timestamp,x,y,z,roll,pitch,yaw\n")
            print(f"Logging poses to: {self.output_file}")
        
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.last_display_time = self.get_clock().now()
        self.display_interval = 0.5  # Display every 0.5 seconds in continuous mode
        
        if mode == 'continuous':
            print("\n" + "=" * 70)
            print("Robot Pose Monitor - Continuous Mode")
            print("=" * 70)
            print("Press Ctrl+C to stop\n")
        elif mode == 'once':
            print("\nWaiting for robot pose...")
    
    def odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Extract orientation (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert to euler angles
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        
        # Get timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Log to file if enabled
        if self.file_handle:
            self.file_handle.write(f"{timestamp},{x},{y},{z},{roll},{pitch},{yaw}\n")
            self.file_handle.flush()
        
        # Display based on mode
        if self.mode == 'once':
            print("\n" + "=" * 70)
            print("Current Robot Pose")
            print("=" * 70)
            print(f"Position (m):")
            print(f"  X: {x:>10.4f}")
            print(f"  Y: {y:>10.4f}")
            print(f"  Z: {z:>10.4f}")
            print(f"\nOrientation (degrees):")
            print(f"  Roll:  {roll:>8.2f}°")
            print(f"  Pitch: {pitch:>8.2f}°")
            print(f"  Yaw:   {yaw:>8.2f}°")
            print(f"\nOrientation (quaternion):")
            print(f"  X: {qx:>10.6f}")
            print(f"  Y: {qy:>10.6f}")
            print(f"  Z: {qz:>10.6f}")
            print(f"  W: {qw:>10.6f}")
            print("=" * 70)
            rclpy.shutdown()
        
        elif self.mode == 'continuous':
            # Rate limit display
            current_time = self.get_clock().now()
            if (current_time - self.last_display_time).nanoseconds * 1e-9 >= self.display_interval:
                print(f"Position: X={x:>7.3f}m  Y={y:>7.3f}m  Z={z:>7.3f}m  |  "
                      f"Yaw={yaw:>7.2f}°  Pitch={pitch:>6.2f}°  Roll={roll:>6.2f}°")
                self.last_display_time = current_time
        
        elif self.mode == 'simple':
            print(f"{x:.4f} {y:.4f} {z:.4f} {yaw:.2f}")
            rclpy.shutdown()
    
    def __del__(self):
        if self.file_handle:
            self.file_handle.close()


def main():
    # Parse arguments
    mode = 'once'  # Default mode
    output_file = None
    
    if len(sys.argv) > 1:
        if sys.argv[1] == '--continuous' or sys.argv[1] == '-c':
            mode = 'continuous'
        elif sys.argv[1] == '--simple' or sys.argv[1] == '-s':
            mode = 'simple'
        elif sys.argv[1] == '--log' or sys.argv[1] == '-l':
            mode = 'continuous'
            output_file = sys.argv[2] if len(sys.argv) > 2 else 'robot_poses.csv'
        elif sys.argv[1] == '--help' or sys.argv[1] == '-h':
            print("Usage: python3.10 get_robot_pose.py [OPTIONS]")
            print("\nOptions:")
            print("  (none)           Get current pose once and exit (default)")
            print("  -c, --continuous Continuously display pose updates")
            print("  -s, --simple     Get pose in simple format: x y z yaw")
            print("  -l, --log [FILE] Log poses to CSV file (default: robot_poses.csv)")
            print("  -h, --help       Show this help message")
            print("\nExamples:")
            print("  python3.10 get_robot_pose.py              # Get pose once")
            print("  python3.10 get_robot_pose.py -c           # Watch pose continuously")
            print("  python3.10 get_robot_pose.py -s           # Simple format")
            print("  python3.10 get_robot_pose.py -l poses.csv # Log to file")
            return
    
    rclpy.init()
    node = RobotPoseMonitor(mode=mode, output_file=output_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if mode == 'continuous':
            print("\n\nStopped monitoring.")
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
