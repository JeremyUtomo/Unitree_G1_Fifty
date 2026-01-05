#!/bin/bash
# ROS2 Network Diagnostic Script
# Checks ROS2 configuration for topic communication issues

echo "========================================"
echo "ROS2 Network Diagnostics"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS2 installation
echo -n "Checking ROS2 installation... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ FAILED${NC}"
    echo "  ROS2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ OK (ROS_DISTRO=$ROS_DISTRO)${NC}"
fi

# Check ROS_DOMAIN_ID
echo -n "Checking ROS_DOMAIN_ID... "
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "  ROS_DOMAIN_ID not set (using default: 0)"
    echo "  If robot uses different domain, set: export ROS_DOMAIN_ID=<value>"
else
    echo -e "${GREEN}✓ OK (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)${NC}"
fi

# Check DDS implementation
echo -n "Checking DDS implementation... "
if [ -z "$RMW_IMPLEMENTATION" ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "  Using default DDS (usually rmw_fastrtps_cpp)"
    echo "  For better compatibility: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
else
    echo -e "${GREEN}✓ OK (RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION)${NC}"
fi

# Check network connectivity to robot
echo -n "Checking robot connectivity (192.168.123.164)... "
if ping -c 1 -W 2 192.168.123.164 &> /dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Cannot ping robot at 192.168.123.164"
    exit 1
fi

# Check if ROS2 daemon is running
echo -n "Checking ROS2 daemon... "
if pgrep -x "ros2" > /dev/null; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "  ROS2 daemon not detected (this is usually OK)"
fi

# Try to list ROS2 nodes
echo -n "Checking ROS2 node discovery... "
NODES=$(ros2 node list 2>/dev/null)
if [ $? -eq 0 ]; then
    NODE_COUNT=$(echo "$NODES" | wc -l)
    echo -e "${GREEN}✓ OK (found $NODE_COUNT nodes)${NC}"
    if [ $NODE_COUNT -gt 0 ]; then
        echo "  Nodes:"
        echo "$NODES" | sed 's/^/    /'
    fi
else
    echo -e "${RED}✗ FAILED${NC}"
    echo "  Cannot discover ROS2 nodes"
fi

# Check for bottle_alignment_status topic
echo -n "Checking /bottle_alignment_status topic... "
if ros2 topic list 2>/dev/null | grep -q "/bottle_alignment_status"; then
    echo -e "${GREEN}✓ OK (topic exists)${NC}"
    
    # Check topic type
    TOPIC_TYPE=$(ros2 topic info /bottle_alignment_status 2>/dev/null | grep "Type:" | awk '{print $2}')
    echo "  Type: $TOPIC_TYPE"
    
    # Check publishers
    PUB_COUNT=$(ros2 topic info /bottle_alignment_status 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
    echo "  Publishers: $PUB_COUNT"
    
    # Check subscribers
    SUB_COUNT=$(ros2 topic info /bottle_alignment_status 2>/dev/null | grep "Subscription count:" | awk '{print $3}')
    echo "  Subscribers: $SUB_COUNT"
else
    echo -e "${YELLOW}⚠ NOT FOUND${NC}"
    echo "  Topic not published yet (auto_center_bottle.py not running)"
fi

# Check /Odometry topic (needed for navigation)
echo -n "Checking /Odometry topic... "
if ros2 topic list 2>/dev/null | grep -q "/Odometry"; then
    echo -e "${GREEN}✓ OK${NC}"
else
    echo -e "${YELLOW}⚠ NOT FOUND${NC}"
    echo "  FAST-LIO localization not running"
fi

echo ""
echo "========================================"
echo "Recommendations:"
echo "========================================"

# Provide recommendations
if [ -z "$RMW_IMPLEMENTATION" ]; then
    echo "1. Set DDS implementation:"
    echo "   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
fi

if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "2. Set ROS domain (if needed):"
    echo "   export ROS_DOMAIN_ID=0"
fi

echo ""
echo "To test topic communication:"
echo "  python3 src/test_alignment_topic.py"
echo ""
echo "To monitor topic manually:"
echo "  ros2 topic echo /bottle_alignment_status"
echo ""
