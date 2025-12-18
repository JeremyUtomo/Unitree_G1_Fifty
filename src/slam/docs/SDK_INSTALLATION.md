# Unitree SDK2 Installation Guide

## Installation Complete ✓

The Unitree SDK2 Python bindings have been successfully installed in this workspace!

### What Was Installed

1. **Unitree SDK2 Python**: `/home/goon/Documents/GitHub/Unitree_G1_Fifty/unitree_sdk2_python/`
   - Python bindings for the SDK
   - Installed as `unitree_sdk2py` module (version 1.0.1)
   - Examples in `example/g1/` directory

3. **Dependencies**:
   - `cyclonedds==0.10.2` (DDS middleware for robot communication)
   - `opencv-python` (for camera access)
   - `numpy<2.0` (kept at 1.26.4 for ROS 2 compatibility)

### Verification

```bash
python3.10 -c "import unitree_sdk2py; print('SDK installed:', unitree_sdk2py.__version__)"
```

Expected output: `SDK installed: 1.0.1`

## G1 LocoClient API

The `navigate_to_goal.py` script uses the G1 LocoClient API. Available methods:

### Movement Commands
- `Move(vx, vy, vyaw)` - Send velocity commands (m/s, m/s, rad/s)
- `SetVelocity(vx, vy, vyaw)` - Alternative velocity command
- `StopMove()` - Stop movement

### Posture Commands
- `Squat2StandUp()` - Stand up from squat
- `StandUp2Squat()` - Squat down from standing
- `Lie2StandUp()` - Stand up from lying down
- `HighStand()` - High standing posture
- `LowStand()` - Low standing posture
- `BalanceStand()` - Balanced standing

### Safety Commands
- `Damp()` - Activate damping mode (safe stop)
- `ZeroTorque()` - Zero torque on all motors

### Special Actions
- `WaveHand(turn_around=False)` - Wave hand gesture
- `ShakeHand()` - Shake hand gesture
- `Sit()` - Sit down

### Initialization
```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# Initialize communication channel
ChannelFactoryInitialize(0, "enp2s0")  # Replace with your network interface

# Create and initialize client
loco_client = LocoClient()
loco_client.SetTimeout(10.0)
loco_client.Init()

# Send movement command
loco_client.Move(0.3, 0.0, 0.0)  # Move forward at 0.3 m/s
```

## Using navigate_to_goal.py

The navigation script has been fully integrated with the Unitree SDK!

### Usage

```bash
# Source ROS environment first
source /opt/ros/humble/setup.bash
source install/setup.bash

# Navigate to a goal (with robot connected)
python3.10 src/slam/navigation/navigate_to_goal.py <x> <y> <network_interface> [options]

# Example: Navigate to (5.0, 3.0) via enp2s0 interface
python3.10 src/slam/navigation/navigate_to_goal.py 5.0 3.0 enp2s0

# Example: Navigate with custom speed and tolerance
python3.10 src/slam/navigation/navigate_to_goal.py 5.0 3.0 enp2s0 --speed 0.3 --tolerance 0.2

# Simulation mode (no robot)
python3.10 src/slam/navigation/navigate_to_goal.py 2.0 1.5 sim
```

### Parameters

**Required:**
- `x y` - Target coordinates in the map frame (meters)
- `network_interface` - Network interface connected to robot (e.g., `enp2s0`, `eth0`)
  - Use `sim` for simulation mode without robot

**Optional:**
- `--speed <value>` - Maximum speed in m/s (default: 0.5)
- `--tolerance <value>` - Goal tolerance in meters (default: 0.3)
- `--angle-tol <value>` - Angle tolerance in degrees (default: 10)

### How It Works

1. **Localization**: Subscribes to `/Odometry` topic from FAST-LIO localization
2. **Path Planning**: Calculates heading and distance to goal
3. **Control Logic**:
   - Rotates in place if heading error > 10°
   - Moves forward with gentle heading correction
   - Slows down as it approaches the goal (proportional control)
4. **Robot Control**: Sends velocity commands via `LocoClient.Move(vx, vy, vyaw)`
5. **Goal Reached**: Stops when within tolerance of goal

### Prerequisites

Before running:

1. **FAST-LIO Localization Running**:
   ```bash
   ros2 launch fast_lio_localization localization.launch.py
   ```

2. **Robot Connection**: Ensure G1 is connected to the specified network interface

3. **Robot Mode**: G1 should be in a standing position (not in squat or lie down)

### Safety Notes

- **Test in simulation first**: Use `sim` mode to verify behavior
- **Clear space**: Ensure the path to goal is obstacle-free
- **Manual override**: Keep the emergency stop ready
- **Network**: Stable connection to robot is critical
- **Start small**: Test with nearby goals first (1-2 meters)

### Troubleshooting

**"ModuleNotFoundError: No module named 'rclpy'"**
- Solution: Source ROS environment first
  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ```

**"Failed to initialize LocoClient"**
- Check network interface name: `ip addr` or `ifconfig`
- Verify robot is powered on and connected
- Try simulation mode first: `python3.10 src/slam/navigation/navigate_to_goal.py 2.0 1.5 sim`

**"No Odometry data received"**
- Ensure FAST-LIO localization is running
- Check topic: `ros2 topic echo /Odometry`

**Robot doesn't move**
- Verify robot is in standing mode (not damped or squat)
- Check network connection
- Increase `--speed` parameter if too slow

## Example Workflow

```bash
# Terminal 1: Start localization
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio_localization localization.launch.py

# Terminal 2: Test in simulation mode
source /opt/ros/humble/setup.bash
source install/setup.bash
python3.10 src/slam/navigation/navigate_to_goal.py 2.0 1.0 sim

# Terminal 3: Monitor robot pose
source /opt/ros/humble/setup.bash
source install/setup.bash
python3.10 src/slam/localization/get_robot_pose.py --mode simple

# When ready with real robot:
# Terminal 2: Run with robot (replace enp2s0 with your interface)
python3.10 src/slam/navigation/navigate_to_goal.py 2.0 1.0 enp2s0 --speed 0.3
```

## SDK Examples

The `unitree_sdk2_python/example/g1/` directory contains example scripts:

- `g1_loco_client_example.py` - Interactive locomotion control demo
- `g1_arm5_sdk_dds_example.py` - 5-DOF arm control
- `g1_arm7_sdk_dds_example.py` - 7-DOF arm control
- `g1_arm_action_example.py` - Arm action sequences

Run them to learn more about the SDK:
```bash
cd unitree_sdk2_python/example/g1/high_level
python3 g1_loco_client_example.py enp2s0
```

## Network Interface Detection

Find your network interface connected to the robot:

```bash
# List all network interfaces
ip addr

# Or
ifconfig

# Look for interface connected to robot (usually ethernet)
# Common names: enp2s0, eth0, eno1, etc.
```

## Additional Resources

- [Unitree SDK2 Documentation](https://support.unitree.com/home/en/developer)
- [G1 Developer Guide](https://support.unitree.com/home/en/developer/sports_services)
- Repository: https://github.com/unitreerobotics/unitree_sdk2_python
