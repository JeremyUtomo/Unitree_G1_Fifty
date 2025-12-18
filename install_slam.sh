#!/bin/bash
# Automated SLAM Setup Script for Unitree G1
# This script automates the complete SLAM setup process

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_ROOT="$SCRIPT_DIR"

echo "=================================================="
echo "Unitree G1 SLAM Setup Automation"
echo "=================================================="
echo ""
echo "This script will:"
echo "  1. Install ROS2 Humble (if not present)"
echo "  2. Install system dependencies"
echo "  3. Install Livox-SDK2"
echo "  4. Clone SLAM repositories"
echo "  5. Setup ROS2 workspace"
echo "  6. Build SLAM packages"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Setup cancelled."
    exit 0
fi

echo ""
echo "=================================================="
echo "Step 1: Checking ROS2 Humble installation..."
echo "=================================================="

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS2 Humble not found. Installing..."
    
    sudo apt update
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    
    sudo apt update
    sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
    
    # Initialize rosdep if not already done
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    echo "✓ ROS2 Humble installed"
else
    echo "✓ ROS2 Humble already installed"
fi

echo ""
echo "=================================================="
echo "Step 2: Installing system dependencies..."
echo "=================================================="

sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-vision-opencv \
    ros-humble-pcl-conversions \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev

pip3 install open3d numpy

echo "✓ System dependencies installed"

echo ""
echo "=================================================="
echo "Step 3: Installing Livox-SDK2..."
echo "=================================================="

if [ ! -f "/usr/local/lib/liblivox_lidar_sdk_shared.so" ]; then
    echo "Building and installing Livox-SDK2..."
    cd /tmp
    rm -rf Livox-SDK2
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    echo "✓ Livox-SDK2 installed"
else
    echo "✓ Livox-SDK2 already installed"
fi

echo ""
echo "=================================================="
echo "Step 4: Cloning SLAM repositories..."
echo "=================================================="

cd "$WORKSPACE_ROOT"

# Clone FAST-LIO if not exists
if [ ! -d "FAST_LIO_LOCALIZATION2" ]; then
    echo "Cloning FAST_LIO_LOCALIZATION..."
    git clone https://github.com/Ericsii/FAST_LIO_LOCALIZATION.git FAST_LIO_LOCALIZATION2
    cd FAST_LIO_LOCALIZATION2
    git checkout ROS2
    git submodule update --init --recursive
    cd "$WORKSPACE_ROOT"
    echo "✓ FAST_LIO_LOCALIZATION2 cloned"
else
    echo "✓ FAST_LIO_LOCALIZATION2 already exists"
    cd FAST_LIO_LOCALIZATION2
    git checkout ROS2 2>/dev/null || true
    git submodule update --init --recursive
    cd "$WORKSPACE_ROOT"
fi

# Clone livox_ros_driver2 if not exists
if [ ! -d "livox_ros_driver2" ]; then
    echo "Cloning livox_ros_driver2..."
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    echo "✓ livox_ros_driver2 cloned"
else
    echo "✓ livox_ros_driver2 already exists"
fi

# Copy package.xml for ROS2
cd "$WORKSPACE_ROOT/livox_ros_driver2"
if [ ! -f "package.xml" ] || [ "package_ROS2.xml" -nt "package.xml" ]; then
    cp package_ROS2.xml package.xml
    echo "✓ package.xml updated for ROS2"
fi

echo ""
echo "=================================================="
echo "Step 5: Setting up ROS2 workspace..."
echo "=================================================="

cd "$WORKSPACE_ROOT"

# Create ros2_ws if not exists
if [ ! -d "ros2_ws" ]; then
    mkdir -p ros2_ws/src
    echo "✓ Created ros2_ws directory"
fi

cd ros2_ws/src

# Create symbolic links
if [ ! -L "FAST_LIO_LOCALIZATION2" ]; then
    ln -sf ../../FAST_LIO_LOCALIZATION2 FAST_LIO_LOCALIZATION2
    echo "✓ Created symlink for FAST_LIO_LOCALIZATION2"
fi

if [ ! -L "livox_ros_driver2" ]; then
    ln -sf ../../livox_ros_driver2 livox_ros_driver2
    echo "✓ Created symlink for livox_ros_driver2"
fi

echo ""
echo "=================================================="
echo "Step 6: Building SLAM workspace..."
echo "=================================================="

cd "$WORKSPACE_ROOT/ros2_ws"

source /opt/ros/humble/setup.bash

echo "Building packages (this may take a few minutes)..."
colcon build --symlink-install \
    --cmake-args \
        -DROS_EDITION=ROS2 \
        -DHUMBLE_ROS=humble \
    --allow-overriding fast_lio livox_ros_driver2

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
else
    echo "✗ Build failed"
    exit 1
fi

echo ""
echo "=================================================="
echo "Setup Complete!"
echo "=================================================="
echo ""
echo "To use SLAM, source the setup script:"
echo "  source $WORKSPACE_ROOT/setup_slam.sh"
echo ""
echo "Then run mapping:"
echo "  ros2 launch fast_lio mapping.launch.py"
echo ""
echo "Or localization:"
echo "  ros2 launch fast_lio localization_with_lidar.launch.py"
echo ""
echo "For detailed instructions, see:"
echo "  $WORKSPACE_ROOT/src/slam/docs/SETUP_INSTRUCTIONS.md"
echo "  $WORKSPACE_ROOT/src/slam/docs/RUN_INSTRUCTIONS.md"
echo ""
