# FAST_LIO_LOCALIZATION2 - Quick Reference

## System Configuration
- **LiDAR IP:** 192.168.123.120 ← CORRECT!
- **Main Chip IP:** 192.168.123.161
- **Dev Chip IP:** 192.168.123.164
- **Host IP:** 192.168.123.222
- **Network Interface:** enp49s0
- **ROS2:** Humble
- **Python:** 3.10.12

## Run Command
```bash
cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
source setup_slam.sh
ros2 launch fast_lio_localization localization_with_lidar.launch.py map:=/path/to/map.pcd
```

## Quick Checks

### Check LiDAR Connection
```bash
ping 192.168.123.120
```

### Check Network Interface
```bash
ip addr show enp49s0
```

### Check ROS Topics
```bash
# IMPORTANT: Source workspace first!
cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
source setup_slam.sh

# Then check topics
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once
```

### View Config Files
```bash
# LiDAR config
cat livox_ros_driver2/config/MID360_config.json

# FAST-LIO config  
cat FAST_LIO_LOCALIZATION2/config/mid360.yaml
```

## Expected Topics
- `/livox/lidar` - LiDAR point cloud (~10 Hz)
- `/livox/imu` - IMU data
- `/map` - Static map point cloud
- `/Odometry` - FAST-LIO odometry
- `/global_odom` - Global localization output

## Initial Pose Setup
1. Wait for RViz2 to show the map (3-5 seconds)
2. Click "2D Pose Estimate" in toolbar
3. Click and drag on map to set position and orientation
4. Keep robot stationary during initialization

## Common Issues

**No LiDAR data?**
- Check ping to 192.168.123.120
- Verify topics with `ros2 topic list`
- Check config file has correct IPs

**Python import errors?**
- Install dependencies: `pip3 install -r src/requirements.txt`
- Source workspace: `source setup_slam.sh`

**No map in RViz2?**
- Check map file path is correct
- Ensure PCD file exists and is valid
- Try downsampling large maps

For detailed instructions, see `src/slam/docs/RUN_INSTRUCTIONS.md`
