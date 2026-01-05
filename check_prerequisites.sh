#!/bin/bash
# Quick Start Script for Integrated Controller
# This script helps verify prerequisites before running the integrated controller

echo "========================================"
echo "Integrated Controller - Prerequisites Check"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check 1: SSH Connection
echo -n "Checking SSH connection to robot... "
if ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no unitree@192.168.123.164 'exit' 2>/dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Please setup SSH keys first: see src/SSH_SETUP.md"
    echo "  Run: ssh-copy-id unitree@192.168.123.164"
    exit 1
fi

# Check 2: Robot Script Exists
echo -n "Checking robot script... "
if ssh unitree@192.168.123.164 'test -f /home/unitree/Unitree_G1_Fifty/src/center_bottle/auto_center_bottle.py' 2>/dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Script not found: /home/unitree/Unitree_G1_Fifty/src/center_bottle/auto_center_bottle.py"
    exit 1
fi

# Check 3: ROS2 Environment
echo -n "Checking ROS2 environment... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Please source ROS2 environment first:"
    echo "  source setup_slam.sh"
    exit 1
else
    echo -e "${GREEN}✓ OK (ROS_DISTRO=$ROS_DISTRO)${NC}"
fi

# Check 4: FAST-LIO Running
echo -n "Checking FAST-LIO localization... "
if ros2 topic list 2>/dev/null | grep -q "/Odometry"; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "  /Odometry topic not found"
    echo "  Make sure FAST-LIO localization is running:"
    echo "  ros2 launch fast_lio_localization localization_with_lidar.launch.py map:=/path/to/map.pcd"
fi

# Check 5: Network Interface
INTERFACE=${1:-enp49s0}
echo -n "Checking network interface ($INTERFACE)... "
if ip link show $INTERFACE &> /dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Interface $INTERFACE not found"
    echo "  Available interfaces:"
    ip -br link show | grep -v "lo" | awk '{print "    " $1}'
    exit 1
fi

# Check 6: Robot Connectivity
echo -n "Checking robot connectivity (192.168.123.164)... "
if ping -c 1 -W 2 192.168.123.164 &> /dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Cannot ping robot at 192.168.123.164"
    echo "  Check network connection and robot power"
    exit 1
fi

echo ""
echo "========================================"
echo -e "${GREEN}All prerequisites met!${NC}"
echo "========================================"
echo ""
echo "Ready to run integrated controller:"
echo "  python3 src/integrated_controller.py $INTERFACE"
echo ""
echo "Steps:"
echo "  1. Set initial pose in RViz2 (2D Pose Estimate)"
echo "  2. Set first goal in RViz2 (2D Goal Pose)"
echo "  3. Wait for auto-centering and pickup"
echo "  4. Set second goal in RViz2 (2D Goal Pose)"
echo "  5. Wait for put-down and completion"
echo ""
