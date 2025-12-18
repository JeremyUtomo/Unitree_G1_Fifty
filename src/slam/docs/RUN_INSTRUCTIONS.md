## How to Run

### Available Launch Files

1. **`mapping.launch.py`** - Create a new map with FAST-LIO
2. **`localization_with_lidar.launch.py`** - Localize with an existing map

### Prerequisites Check
Before running, ensure you have:
- A pre-built PCD map file (downsampled for better RViz2 performance) - **OR** create one with mapping mode
- Livox LiDAR connected Mid360 at **192.168.123.120**
- Network interface **enp49s0**

### Network Configuration
- LiDAR IP: `192.168.123.120` 
- Host IP: `192.168.123.222`
- Interface: `enp49s0`

## Option 1: Create a Map (FAST-LIO Mapping)

### Launch FAST-LIO Mapping Mode

1. **Start mapping:**
   ```bash
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   ros2 launch fast_lio_localization mapping.launch.py
   ```

2. **What happens:**
   - LiDAR driver starts automatically
   - FAST-LIO mapping begins after IMU initialization (~10-15 seconds)
   - RViz2 opens with pre-configured visualization

3. **Create the map:**
   - Walk/drive the robot around your environment
   - Try to loop back to starting point for best accuracy

4. **Save the map:**
   
   You have TWO options for saving maps:
   
   **Option A: Clean Map for Localization (RECOMMENDED)**
   
   Best for ICP localization - removes robot body but keeps geometric features:
   ```bash
   # In a new terminal (keep mapping running)
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   
   # Collect clean map (filters points within 3m of LiDAR)
   python3.10 src/slam/map_processing/clean_for_localization.py office 3.0
   ```
   
   Press Ctrl+C when done. This creates:
   - `office_localization.pcd` - Clean map for localization
   - Removes robot body and close obstructions like the operator (within 3m)
   - Keeps floor, walls, and all other geometric features
   - No downsampling - preserves detail for ICP alignment
   
   **Option B: Processed Map for Visualization**
   
   Lighter map for viewing (removes floor + downsamples):
   ```bash
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   
   # Process and rename the map
   python3.10 src/slam/utilities/save_and_process.py office
   ```
   
   This will:
   - Wait for you to close mapping (press Enter after closing)
   - Rename `scans.pcd` to `office_raw.pcd`
   - Remove ground plane
   - Downsample to 1 point per 10cm squared
   - Create final `office.pcd`
   
   **Note:** The processed map (Option B) wont work well for localization 
   due to removed geometric features. Use Option A for localization.

5. **View your map** (optional):
   ```bash
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   
   # Publish and view the map
   ros2 run pcl_ros pcd_to_pointcloud \
       --ros-args \
       -p file_name:=FAST_LIO_LOCALIZATION2/PCD/scans_processed.pcd \
       -p tf_frame:=map \
       -p cloud_topic:=cloud_pcd &
   rviz2  # Set Fixed Frame to "map", Add /cloud_pcd topic
   ```

---

## Option 2: Localization (Use Existing Map)

### Quick Start - With Live LiDAR

1. **Source the workspace:**
   ```bash
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   ```

2. **Launch localization:**
   ```bash
   # Use the clean localization map (recommended)
   ros2 launch fast_lio_localization localization_with_lidar.launch.py \
       map:=/home/goon/Documents/GitHub/Unitree_G1_Fifty/FAST_LIO_LOCALIZATION2/PCD/office_localization.pcd
   ```

3. **Visualize map and scan (helps estimate initial pose):**
   
   In RViz2, add these displays to see both map and current scan:
   - **Add → PointCloud2 → Topic: `/map`** (the saved map)
     - Set Color: Green
     - Set Size: 0.05
   - **Add → PointCloud2 → Topic: `/cloud_registered`** (current LiDAR scan)
     - Set Color: Red  
     - Set Size: 0.05
   - Set **Fixed Frame** to `map`
   
   Now you can see:
   - Green points = Earlier saved map
   - Red points = Current LiDAR scan (Live data)
   - Look for matching features (walls, corners) to estimate where the robot is

4. **Set initial pose:**
   - Use **"2D Pose Estimate"** tool in RViz2
   - Click on the green map where you think the robot is located
   - Drag arrow to match robot's facing direction
   - Watch terminal for "ICP fitness" score (>0.5 = good alignment)
   - Keep robot still during initialization

---

## Option 3: Interactive Navigation with RViz2 (Click-to-Navigate)

### Quick Start - Click and Go!

1. **Start localization** (from Option 2 above):
   ```bash
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   ros2 launch fast_lio_localization localization_with_lidar.launch.py \
       map:=/home/goon/Documents/GitHub/Unitree_G1_Fifty/FAST_LIO_LOCALIZATION2/PCD/office_localization.pcd
   ```

2. **Set initial pose** in RViz2 using "2D Pose Estimate"

3. **Start interactive navigation:**

   **Option A: Obstacle Avoidance Navigation (RECOMMENDED)**
   ```bash
   # In a new terminal
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   
   # Intelligent path planning with 30cm safety margin
   python3 src/slam/navigation/rviz_navigation_obstacle_avoidance.py enp49s0 --speed 0.3
   ```
   
   **Option B: Direct Navigation (No Obstacle Avoidance)**
   ```bash
   # In a new terminal
   cd /home/goon/Documents/GitHub/Unitree_G1_Fifty
   source setup_slam.sh
   
   # Direct line navigation
   python3 src/slam/navigation/rviz_navigation.py enp49s0 --speed 0.3
   ```

4. **Navigate by clicking:**
   - In RViz2, click the **"2D Goal Pose"** button (toolbar)
   - Click anywhere on the map and drag to set goal position and orientation
   - Robot will automatically plan and navigate to that point!
   - Set new goals anytime - robot will navigate to each one sequentially

5. **View planned path** (Option A only):
   - In RViz2, add display: **Add → By topic → /planned_path → Path**
   - Set color to green or yellow for visibility
   - You'll see the A* planned path avoiding obstacles with 30cm safety margin

### Features:
- Real-time obstacle detection from map point cloud
- Filters out small obstacles (minimum 5 points per cell)
- Automatic waypoint generation along safe path
- Click anywhere on the map to send robot there
- Visual feedback in RViz2 showing planned path
- Intelligent rotation behavior
- Final orientation matching (robot faces goal arrow direction)
- Can send multiple goals - robot queues them automatically

### Alternative: Pre-planned Waypoint Navigation

If you want to navigate to specific coordinates programmatically:

```bash
# Single waypoint
python3 src/slam/navigation/navigate_to_goal.py 2.0 -1.0 enp49s0 --speed 0.3

# Multiple waypoints (robot visits each in sequence)
python3 src/slam/navigation/navigate_to_goal.py 2.0 0.0 4.0 1.0 4.0 -1.0 enp49s0 --speed 0.3
```

---

## Get Robot Location

4. **Get robot location:**
   
   Once localized, you can extract the robot's pose in several ways:
   
   ```bash
   # Get current pose once (detailed)
   python3 src/slam/localization/get_robot_pose.py
   
   # Watch pose continuously (updates every 0.5s)
   python3 src/slam/localization/get_robot_pose.py --continuous
   
   # Simple format: x y z yaw
   python3 src/slam/localization/get_robot_pose.py --simple
   
   # Log poses to CSV file
   python3 src/slam/localization/get_robot_pose.py --log poses.csv
   ```
   
   The pose includes:
   - **Position**: X, Y, Z coordinates in meters (map frame)
   - **Orientation**: Roll, Pitch, Yaw in degrees (or quaternion)


### Troubleshooting

**LiDAR not working:**
```bash
ping 192.168.123.120  # Check connection
ros2 topic hz /livox/lidar  # Should show ~10 Hz
```
**Multiple maps show on rviz**
   When multiple maps show up simultaneously in rviz kill them first before running: pkill -f pcd_to_pointcloud
---

## Important Notes

- **LiDAR IP:** 192.168.123.120
- **Host IP:** 192.168.123.222 (interface: enp49s0)
- Keep robot stationary during initialization
- Use downsampled maps for better performance

## Python Environment

Python packages required:
- tf_transformations, ros2-numpy, open3d, numpy<1.24
- Install via: `pip3 install -r src/requirements.txt`

---

## 🐛 Troubleshooting

**Map not appearing:**
- Check file path is correct
- Try downsampling large maps
- Select the map as a topic

**Localization not working:**
- Verify initial pose is set
- Keep robot stationary during initialization

**Obstacle avoidance not working:**
- Make sure /submap topic is publishing: `ros2 topic hz /submap`
- Check if obstacles appear in obstacle map (should see "Path found" messages)
- If goal is rejected with "too close to obstacle", try moving goal further from walls
- Adjust safety margin if needed: edit `obstacle_margin=0.3` in script (default 30cm)

**Path planning fails:**
- Goal might be inside an obstacle or within 30cm safety zone
- Try selecting a different goal location
- Check if map is loaded: `ros2 topic echo /submap --once`
- If "out of map bounds", goal is outside 100m x 100m planning area

**Robot doesn't follow planned path:**
- Verify robot is receiving commands (check SDK connection)
- Check /planned_path topic in RViz2 to see the planned route
- Speed might be too high - try lower value: `--speed 0.2`

**Python errors:**
- Install dependencies: `pip3 install -r src/requirements.txt`
- Ensure ROS2 is sourced: `source setup_slam.sh`

---

## ⚙️ System Info

- ROS2 Distribution: **Humble**
- Python Version: 3.10.12
- Built on: Ubuntu 22.04 (Jammy)
