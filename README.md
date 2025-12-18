# Unitree G1 Fifty - Control & Manipulation System

A comprehensive control system for the Unitree G1 humanoid robot featuring wireless controller integration, arm manipulation sequences, voice control, and state monitoring.

## Directory Structure

```
Unitree_G1_Fifty/
├── src/                       # Main control scripts
│   ├── center_bottle/         # Vision-based bottle centering
│   ├── manipulation/          # Arm and hand manipulation
│   ├── robot_state/          # State monitoring utilities
│   ├── slam/                 # SLAM and autonomous navigation
│   ├── teleoperation/        # Remote control interfaces
│   ├── vui/                  # Voice user interface
│   ├── wireless_controller/  # Wireless controller integration
│   ├── script_controller.py  # Main orchestrator
│   └── requirements.txt      # Python dependencies
├── ros2_ws/                   # ROS2 workspace for SLAM
│   └── src/
│       ├── FAST_LIO_LOCALIZATION2/   # FAST-LIO package
│       └── livox_ros_driver2/        # Livox LiDAR driver
├── mujoco/                    # MuJoCo simulation
├── unitree_sdk2_python/       # Unitree SDK
└── setup_slam.sh             # SLAM workspace setup script
```

## Module Overview

### center_bottle/
Vision-based bottle detection and centering using YOLO and RealSense camera.

**Files:**
- `auto_center_bottle.py` - Automated bottle centering controller
- `laptop_view_detections.py` - Stream viewer for detections

**Usage:**
```bash
python3 auto_center_bottle.py <network_interface> <camera_topic>
python3 laptop_view_detections.py <rtsp_url>
```

### manipulation/
Arm and hand control sequences for manipulation tasks.

**Files:**
- `arm_pick_up_bottle.py` - Left arm bottle pickup sequence (4 positions)
- `arm_pick_up_package.py` - Dual-arm package pickup sequence (7 stages)
- `67.py` - Arm sequence controller (alternating pattern)
- `arm_stop.py` - Graceful arm stop and SDK release
- `arm_damp.py` - Arm damping mode (loose arms)
- `hand_controller.py` - Dexterous hand control interface
- `hand_damp.py` - Hand damping utility

**Usage:**
```bash
python3 arm_pick_up_bottle.py <network_interface>
python3 arm_pick_up_package.py <network_interface>
python3 arm_stop.py <network_interface>
python3 arm_damp.py <network_interface>
python3 hand_controller.py <network_interface> <left|right> <open|close|damp>
python3 hand_damp.py <network_interface> <left|right|both>
```

### robot_state/
Real-time monitoring of robot state data.

**Files:**
- `hand_pressure_monitor.py` - Tactile pressure sensor monitoring
- `monitor_hand_state.py` - Hand motor state dashboard
- `monitor_low_state.py` - Low-level state monitor (all joints + IMU)

**Usage:**
```bash
python3 hand_pressure_monitor.py <network_interface> <left|right> [finger_id]
python3 monitor_hand_state.py <network_interface> <left|right|both>
python3 monitor_low_state.py <network_interface> [joint_id]
```

### teleoperation/
Remote control interfaces for locomotion and balance.

**Files:**
- `loco_client.py` - Interactive locomotion control terminal
- `hanger_boot_sequence.py` - Balance mode initialization sequence

**Usage:**
```bash
python3 loco_client.py <network_interface>
```

**Controls:**
- `w` - Move forward
- `s` - Move backward
- `a` - Side step left
- `d` - Side step right
- `q` - Rotate left
- `e` - Rotate right
- `b` - Balance mode
- `stop` - Stop/damp

### vui/
Voice user interface for audio feedback and control.

**Files:**
- `voice_input.py` - Text-to-speech and LED control

**Usage:**
```bash
python3 voice_input.py <network_interface>
```

**Features:**
- TTS (Text-to-Speech)
- LED control (RGB)
- Volume control

### slam/
SLAM (Simultaneous Localization and Mapping) and autonomous navigation using FAST-LIO and Livox Mid360 LiDAR.

**Structure:**
- `map_processing/` - Map cleaning, downsampling, and optimization
- `navigation/` - Autonomous navigation with obstacle avoidance
- `localization/` - Robot pose estimation and tracking
- `utilities/` - Helper scripts and diagnostics
- `maps/` - Storage for point cloud maps (.pcd files)
- `docs/` - Detailed documentation and guides

**Key Features:**
- Real-time LiDAR SLAM with FAST-LIO
- Interactive RViz2 navigation (click-to-navigate)
- A* path planning with obstacle avoidance
- Waypoint-based navigation
- Map processing and optimization

**Quick Start:**
```bash
# 1. Create a map (FAST-LIO mapping)
ros2 launch fast_lio_localization mapping.launch.py

# 2. Process the map
python3 src/slam/map_processing/clean_for_localization.py office 3.0

# 3. Start localization
ros2 launch fast_lio_localization localization_with_lidar.launch.py \
    map:=/path/to/office_localization.pcd

# 4. Navigate interactively
python3 src/slam/navigation/rviz_navigation_obstacle_avoidance.py enp49s0 --speed 0.3
```

**Detailed Documentation:**
See `src/slam/docs/` for:
- `RUN_INSTRUCTIONS.md` - Complete setup and usage guide
- `QUICK_REFERENCE.md` - Quick commands and configuration
- `SDK_INSTALLATION.md` - SDK integration guide

### wireless_controller/
Wireless controller integration for remote operation.

**Files:**
- `custom_wireless_controller.py` - Event-driven controller base class
- `run_wireless_controller.py` - Main wireless controller hub

**Usage:**
```bash
python3 run_wireless_controller.py <network_interface>
```

**Controls:**
- `F1 + A` - Start arm sequence (Stage 1 → Full sequence on second press)
- `F1 + X` - Emergency stop
- `F1 + Y` - Voice greeting + Start 67 sequence

### Main Scripts

#### script_controller.py
Main orchestrator for bottle pick-and-place operations combining vision, manipulation, and state monitoring.

**Features:**
- YOLO-based bottle detection
- RealSense camera integration
- Automated centering
- Arm control coordination
- Hand pressure monitoring

**Usage:**
```bash
python3 script_controller.py <network_interface>
```

## Installation

### 1. Clone the Repository

```bash
cd ~
git clone https://github.com/JeremyUtomo/Unitree_G1_Fifty.git
cd Unitree_G1_Fifty
```

### 2. Install System Dependencies

```bash
sudo apt update
sudo apt install python3-pip python3-dev git cmake build-essential
```

### 3. Install CycloneDDS (Required for Unitree SDK)

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

### 4. Install Unitree SDK2 Python

```bash
cd ~/Unitree_G1_Fifty/unitree_sdk2_python
export CYCLONEDDS_HOME="~/cyclonedds/install"
pip3 install -e .
```

To make the export permanent:
```bash
echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> ~/.bashrc
source ~/.bashrc
```

### 5. Install Project Dependencies

```bash
cd ~/Unitree_G1_Fifty/src
pip3 install -r requirements.txt
```

**Key Dependencies:**
- `unitree_sdk2py` - Unitree SDK for Python
- `opencv-python` - Computer vision
- `pyrealsense2` - RealSense camera support
- `ultralytics` - YOLO object detection
- `numpy` - Numerical operations

### 6. Install ROS2 and Build SLAM Workspace

For SLAM and autonomous navigation capabilities:

```bash
# 1. Install ROS2 Humble
sudo apt install ros-humble-desktop

# 2. Install SLAM dependencies
sudo apt install ros-humble-pcl-ros ros-humble-vision-opencv
pip3 install open3d

# 3. Build the SLAM workspace
cd ~/Unitree_G1_Fifty
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHUMBLE_ROS=humble

# 4. Source the workspace (or use setup_slam.sh script)
source ros2_ws/install/setup.bash
# OR
source setup_slam.sh
```

**Note:** The SLAM workspace is pre-configured in `ros2_ws/` with symbolic links to FAST_LIO_LOCALIZATION2 and livox_ros_driver2.

### 7. (Optional) Install MuJoCo for Simulation

```bash
pip3 install mujoco mujoco-python-viewer
```

## Safety Notes

1. **Always ensure clear space** around the robot before running manipulation sequences
2. **Robot must be in balance/walk mode** before executing arm sequences
3. **Use emergency stop** (`F1 + X` on wireless controller) if needed
4. **Monitor hand pressure** during grasping operations
5. **Test in simulation** (MuJoCo) before running on hardware

## Troubleshooting

### SDK Import Errors
```bash
# Reinstall unitree_sdk2py
cd unitree_sdk2_python
pip install -e .
```

### Camera Connection Issues
```bash
# List RealSense devices
rs-enumerate-devices

# Test camera
realsense-viewer
```

### Permission Errors
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```
