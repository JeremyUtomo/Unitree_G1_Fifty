# Unitree G1 Fifty - Control & Manipulation System

A comprehensive control system for the Unitree G1 humanoid robot featuring wireless controller integration, arm manipulation sequences, voice control, and MuJoCo simulation support.

## 🎯 Features

- **Wireless Controller Integration**: Event-driven architecture for mapping gamepad buttons to robot actions
- **Arm Manipulation**: 7-DOF arm control with multiple choreographed sequences
- **Hand Control**: Dexterous hand manipulation with pressure sensing
- **Voice Interface**: Text-to-speech (TTS) integration for audio feedback
- **MuJoCo Simulation**: Test and visualize robot behaviors in simulation
- **Modular Design**: Plugin-based architecture for easy extension

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Network Setup](#network-setup)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Usage Examples](#usage-examples)
- [Documentation](#documentation)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## 🔧 Prerequisites

### Hardware Requirements
- **Unitree G1 Humanoid Robot**
- **Developer Laptop/PC** with Ethernet or WiFi capability
- **Wireless Controller** (compatible with Unitree G1)

### Software Requirements
- **Operating System**: Ubuntu 20.04 or 22.04
- **Python**: >= 3.8
- **Network**: Access to robot network (typically 192.168.123.x)

## 📦 Installation

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

CycloneDDS is the DDS implementation used for robot communication.

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

**Note**: If you encounter the error `Could not locate cyclonedds`, ensure `CYCLONEDDS_HOME` points to your CycloneDDS installation.

To make the export permanent, add it to your shell profile:
```bash
echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> ~/.bashrc  # or ~/.zshrc
source ~/.bashrc  # or source ~/.zshrc
```

### 5. Install Project Dependencies

```bash
cd ~/Unitree_G1_Fifty
pip3 install -r src/requirements.txt
```

### 6. (Optional) Install MuJoCo for Simulation

```bash
pip3 install mujoco mujoco-python-viewer
```

## 🌐 Network Setup

### Connect to the G1 Robot

The G1 robot creates a network that your laptop must connect to:

#### Option 1: Ethernet (Recommended)
1. Connect Ethernet cable from laptop to robot
2. Configure static IP on your laptop:
   - **IP Address**: `192.168.123.100` (or any `.100-.250`)
   - **Subnet Mask**: `255.255.255.0`
   - **Gateway**: `192.168.123.1`

#### Ubuntu Network Configuration:
```bash
# Find your network interface (e.g., eth0, enp0s31f6)
ip link show

# Configure static IP
#### Network Configuration:
```bash
# Find your network interface (e.g., eth0, enp0s31f6)
ip link show

# Configure static IP
sudo ip addr add 192.168.123.100/24 dev eth0
sudo ip link set eth0 up
```

#### Option 2: WiFi
# Ping the robot (default IP: 192.168.123.161)
ping 192.168.123.161

# Check DDS communication
ros2 topic list  # Should show robot topics if ROS2 installed
# OR
python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; ChannelFactoryInitialize(0, 'eth0')"
```

**Common Network Interfaces:**
- Linux Ethernet: `eth0`, `enp0s31f6`, `eno1`
- Linux WiFi: `wlan0`, `wlp2s0`
- macOS Ethernet: `en0`, `en1`
- macOS WiFi: `en0`, `en1`

Find your interface:
```bash
# Linux
ip link show

# macOS
**Common Network Interfaces:**
- Ethernet: `eth0`, `enp0s31f6`, `eno1`
- WiFi: `wlan0`, `wlp2s0`

Find your interface:
```bash
ip link show
```erminal 2: Start publisher
python3 publisher.py eth0
```

You should see messages being exchanged.

### 2. Monitor Robot State

```bash
cd ~/Unitree_G1_Fifty/src/robot_state
python3 monitor_low_state.py eth0
```

This displays real-time joint positions, IMU data, and wireless controller inputs.

### 3. Run Wireless Controller

⚠️ **SAFETY WARNING**: Ensure the robot is in a safe environment with no obstacles!

```bash
cd ~/Unitree_G1_Fifty/src
python3 run_wireless_controller.py eth0
```

**Controls:**
- **F1 + A**: Start arm sequence (press once for Stage 1, twice for full sequence)
- **F1 + X**: Emergency stop arm sequence
- **F1 + Y**: Voice greeting + start alternating arm sequence

### 4. Test Arm Control (Standalone)

```bash
cd ~/Unitree_G1_Fifty/src/manipulation

# Full 7-stage choreography
python3 pick_up_arm_sequence.py eth0

# Alternating 2-stage sequence
python3 67.py eth0

# Emergency stop / return to neutral
python3 arm_stop.py eth0
```

### 5. Run MuJoCo Simulation

```bash
cd ~/Unitree_G1_Fifty/mujoco
python3 unitree_mujoco.py
```

**Simulation Controls:**
- Use keyboard/mouse to control robot in virtual environment
- Test control algorithms safely before hardware deployment

## 📁 Project Structure

```
Unitree_G1_Fifty/
├── docs/                                    # Technical documentation
│   ├── ARM_MANIPULATION_TECHNICAL_DESIGN.md
│   ├── WIRELESS_CONTROLLER_TECHNICAL_DESIGN.md
│   └── ROBOT_STATE_TECHNICAL_DESIGN.md
│
├── src/                                     # Main source code
│   ├── custom_wireless_controller.py        # Core controller abstraction
│   ├── run_wireless_controller.py           # Main wireless control app
│   │
│   ├── manipulation/                        # Arm control modules
│   │   ├── pick_up_arm_sequence.py         # 7-stage choreography
│   │   ├── 67.py                           # 2-stage alternating
│   │   ├── arm_action.py                   # 3-position + hand control
│   │   ├── arm_stop.py                     # Emergency stop
│   │   ├── hand_controller.py              # Hand control API
│   │   └── hand_damp.py                    # Hand damping
│   │
│   ├── robot_state/                        # State monitoring
│   │   ├── monitor_low_state.py            # Real-time state display
│   │   ├── monitor_hand_state.py           # Hand state monitoring
│   │   └── hand_pressure_monitor.py        # Pressure sensor monitoring
│   │
│   ├── teleoperation/                      # Teleoperation modules
│   │   ├── loco_client.py                  # Locomotion client
│   │   └── hanger_boot_sequence.py         # Boot sequence
│   │
│   ├── vui/                                # Voice user interface
│   │   └── voice_input.py                  # TTS control
│   │
│   └── requirements.txt                     # Python dependencies
│
├── mujoco/                                  # Simulation
│   ├── unitree_mujoco.py                   # Main simulation script
│   ├── unitree_sdk2py_bridge.py            # SDK bridge
│   ├── config.py                           # Simulation config
│   └── unitree_robots/                     # Robot models (G1, H1, Go2, B2)
│
├── unitree_sdk2_python/                     # Unitree SDK2 Python binding
│   ├── setup.py
│   ├── pyproject.toml
│   └── unitree_sdk2py/                     # SDK modules
│
└── README.md                                # This file
```

## 💡 Usage Examples

### Example 1: Custom Button Mapping

Create a new controller layout by copying the runner template:

```python
# run_custom_controller.py
from custom_wireless_controller import WirelessController
from my_custom_module import MyCustomAction

class CustomControlApp:
    def __init__(self):
        self.wireless = WirelessController()
        self.custom_action = MyCustomAction()
        
    def Init(self):
        self.custom_action.Init()
        
        # Wire your custom callbacks
        self.wireless.on_start = self._my_start_action
        self.wireless.on_stop = self._my_stop_action
        
        self.wireless.Init()
        
    def _my_start_action(self):
        print("Custom start action!")
        self.custom_action.execute()
        
    def _my_stop_action(self):
        print("Custom stop action!")
        self.custom_action.stop()

if __name__ == '__main__':
    import sys
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    
    if len(sys.argv) < 2:
        print("Usage: python3 run_custom_controller.py <network_interface>")
        sys.exit(1)
    
    ChannelFactoryInitialize(0, sys.argv[1])
    
    app = CustomControlApp()
    app.Init()
    
    try:
        while True:
            import time
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")
        sys.exit(0)
```

### Example 2: Create Custom Arm Sequence

```python
# my_wave_sequence.py
import time
import threading
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.thread import RecurrentThread

LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]

# Define your wave positions
WAVE_POSITION_1 = [0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0]
WAVE_POSITION_2 = [0.0, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0]

class WaveSequence:
    def __init__(self, control_dt=0.02):
        self.control_dt = control_dt
        self.is_running = False
        # ... implement similar to ArmSequence pattern
        
    def Start(self, initial_low_state):
        # Record start positions
        # Start control thread
        pass
    
    def Stop(self):
        # Stop gracefully
        pass
```

### Example 3: Monitor Robot State

```python
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

def lowstate_handler(msg: LowState_):
    # Access joint positions
    left_shoulder_pitch = msg.motor_state[15].q
    right_shoulder_pitch = msg.motor_state[22].q
    
    # Access IMU data
    gyro_x = msg.imu_state.gyroscope[0]
    
    # Access wireless controller
    wireless_data = msg.wireless_remote
    
    print(f"Left shoulder: {left_shoulder_pitch:.3f} rad")

if __name__ == '__main__':
    import sys
    
    ChannelFactoryInitialize(0, sys.argv[1])
    
    sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    sub.Init(lowstate_handler, 10)
    
    try:
        while True:
            import time
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
```

## 📚 Documentation

Comprehensive technical documentation is available in the `docs/` directory:

- **[ARM_MANIPULATION_TECHNICAL_DESIGN.md](docs/ARM_MANIPULATION_TECHNICAL_DESIGN.md)**
  - Control protocol (PD control, trajectory generation)
  - Sequence controllers (ArmSequence, ArmSequence67, etc.)
  - Hand control integration
  - Threading & concurrency
  - Safety mechanisms

- **[WIRELESS_CONTROLLER_TECHNICAL_DESIGN.md](docs/WIRELESS_CONTROLLER_TECHNICAL_DESIGN.md)**
  - Architecture overview (Observer, Facade, Strategy patterns)
  - Button mapping and edge detection
  - Creating custom control layouts
  - Integration with subsystems

- **[ROBOT_STATE_TECHNICAL_DESIGN.md](docs/ROBOT_STATE_TECHNICAL_DESIGN.md)**
  - LowState_ message structure
  - Joint state monitoring
  - IMU data access
  - Wireless remote parsing

## 🐛 Troubleshooting

### Issue: Cannot connect to robot

**Symptoms**: `ping 192.168.123.161` fails

**Solutions**:
1. Check physical connection (Ethernet cable, WiFi)
2. Verify IP configuration: `ip addr show`
3. Ensure robot is powered on and booted (wait ~30 seconds after power-on)
4. Check firewall settings: `sudo ufw status`

### Issue: `Could not locate cyclonedds`

**Symptoms**: Error during `pip3 install -e .`

**Solutions**:
```bash
# Set CYCLONEDDS_HOME environment variable
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"

# Add to shell profile for persistence
echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> ~/.bashrc
source ~/.bashrc

# Retry installation
cd ~/Unitree_G1_Fifty/unitree_sdk2_python
pip3 install -e .
```

### Issue: Arms don't respond to commands

**Symptoms**: `run_wireless_controller.py` runs but arms don't move

**Checklist**:
- [ ] Robot is in **balancing/walking mode** (not sitting or lying down)
- [ ] SDK control is enabled (`motor_cmd[29].q = 1.0`)
- [ ] LowState subscription is receiving data (`monitor_low_state.py` shows updates)
- [ ] No other control programs are running (only one SDK client at a time)

### Issue: Python import errors

**Symptoms**: `ModuleNotFoundError: No module named 'unitree_sdk2py'`

**Solutions**:
```bash
# Reinstall SDK in editable mode
cd ~/Unitree_G1_Fifty/unitree_sdk2_python
pip3 install -e .

# Verify installation
python3 -c "import unitree_sdk2py; print('Success!')"
```

### Issue: Robot falls or loses balance during arm movement

**Symptoms**: Robot becomes unstable when arms move

**Solutions**:
1. Reduce arm movement speed (increase `stage_duration`)
2. Use smaller movements (reduce joint angle changes)
3. Enable waist stabilization (ensure `motor_cmd[12]` is set)
4. Ensure robot has good footing before starting sequences

### Issue: High latency or jittery movements

**Symptoms**: Delayed response, arm movements not smooth

**Solutions**:
1. Use Ethernet instead of WiFi for better latency
2. Close other network-heavy applications
3. Check CPU usage: `top` or `htop`
4. Reduce control frequency (increase `control_dt` from 0.02 to 0.05)

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/my-new-feature`
3. **Commit your changes**: `git commit -am 'Add some feature'`
4. **Push to the branch**: `git push origin feature/my-new-feature`
5. **Submit a pull request**

### Code Style
- Follow PEP 8 for Python code
- Use meaningful variable names
- Add docstrings to classes and functions
- Include safety warnings for hardware-interacting code

### Testing
- Test in simulation (MuJoCo) before hardware
- Document any new button mappings or sequences
- Update technical documentation for significant changes

## ⚠️ Safety Guidelines

**ALWAYS:**
- ✅ Test new sequences in simulation first
- ✅ Ensure 3+ meters clearance around robot
- ✅ Have emergency stop accessible (F1+X or power button)
- ✅ Start with conservative gains and slow movements
- ✅ Monitor robot balance during arm movements

**NEVER:**
- ❌ Run arm sequences while robot is sitting/lying down
- ❌ Use excessive joint velocities (>3 rad/s)
- ❌ Ignore waist stabilization
- ❌ Run multiple control programs simultaneously
- ❌ Leave robot unattended during operation

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- **Unitree Robotics** for the G1 platform and SDK2
- **Eclipse CycloneDDS** for DDS communication layer
- **MuJoCo** for physics simulation

## 📞 Contact

- **Author**: Jeremy Utomo
- **GitHub**: [@JeremyUtomo](https://github.com/JeremyUtomo)
- **Repository**: [Unitree_G1_Fifty](https://github.com/JeremyUtomo/Unitree_G1_Fifty)

---

**Last Updated**: December 10, 2025  
**Version**: 1.0.0
