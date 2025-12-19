#!/bin/bash
# SLAM Workspace Setup Script
# Source this file to use FAST-LIO and Livox driver
# This script automatically detects the workspace location

# Determine the workspace root directory
# This works whether you source the script from any location
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_ROOT="$SCRIPT_DIR"

# Fix RViz2 library conflicts with snap packages
# Prioritize system libraries over snap libraries to prevent symbol lookup errors
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
unset GTK_PATH

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ Error: ROS2 Humble not found at /opt/ros/humble/setup.bash"
    echo "   Please install ROS2 Humble first:"
    echo "   sudo apt install ros-humble-desktop"
    return 1
fi

# Source the SLAM workspace
if [ -f "$WORKSPACE_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"
else
    echo "❌ Error: SLAM workspace not built"
    echo "   Workspace path: $WORKSPACE_ROOT/ros2_ws/install/setup.bash"
    echo ""
    echo "   Please build the workspace first:"
    echo "   cd $WORKSPACE_ROOT/ros2_ws"
    echo "   colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble"
    return 1
fi

echo "✓ ROS2 Humble sourced"
echo "✓ SLAM workspace sourced"
echo "✓ Workspace root: $WORKSPACE_ROOT"
echo ""
echo "Available packages:"
echo "  - livox_ros_driver2"
echo "  - fast_lio_localization"
echo ""
echo "Try: ros2 launch fast_lio_localization mapping.launch.py"
echo "     ros2 launch fast_lio_localization localization_with_lidar.launch.py"
