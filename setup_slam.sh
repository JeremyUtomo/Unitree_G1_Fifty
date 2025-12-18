#!/bin/bash
# SLAM Workspace Setup Script
# Source this file to use FAST-LIO and Livox driver

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the SLAM workspace
source /home/goon/Documents/GitHub/Unitree_G1_Fifty/ros2_ws/install/setup.bash

echo "✓ ROS2 Humble sourced"
echo "✓ SLAM workspace sourced"
echo ""
echo "Available packages:"
echo "  - livox_ros_driver2"
echo "  - fast_lio_localization"
echo ""
echo "Try: ros2 launch fast_lio_localization mapping.launch.py"
echo "     ros2 launch fast_lio_localization localization_with_lidar.launch.py"
