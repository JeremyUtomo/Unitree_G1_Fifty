# Integrated Controller - Quick Reference

## Quick Start (3 Commands)

```bash
# 1. Check prerequisites
./check_prerequisites.sh enp49s0

# 2. Start FAST-LIO localization (Terminal 1)
source setup_slam.sh
ros2 launch fast_lio_localization localization_with_lidar.launch.py map:=$HOME/Documents/GitHub/Unitree_G1_Fifty/ros2_ws/src/FAST_LIO_LOCALIZATION2/PCD/office_localization.pcd

# 3. Run integrated controller (Terminal 2)
python3 src/integrated_controller.py enp49s0
```

## User Actions in RViz2

1. **Initial Pose** (once at start): 2D Pose Estimate button → click & drag on map
2. **First Goal** (Phase 1): 2D Goal Pose button → click & drag near bottle
3. **Second Goal** (Phase 4): 2D Goal Pose button → click & drag to destination

## Phase Indicators

| Phase | What's Happening | Duration | User Action |
|-------|-----------------|----------|-------------|
| 1 | Navigate to bottle | Variable | Set goal in RViz |
| 2 | Auto-center bottle | 30-60s | None (automatic SSH) |
| 3 | Pick up bottle | 10-15s | None (automatic) |
| 4 | Navigate to destination | Variable | Set goal in RViz |
| 5 | Put down bottle | 10-15s | None (automatic) |

## ROS2 Topics to Monitor

```bash
# Robot position
ros2 topic echo /Odometry

# Bottle alignment status
ros2 topic echo /bottle_alignment_status

# Navigation goal
ros2 topic echo /goal_pose

# Planned path
ros2 topic echo /planned_path
```

## Common Issues

| Problem | Quick Fix |
|---------|-----------|
| "SSH connection failed" | Run: `ssh-copy-id unitree@192.168.123.164` |
| "No Odometry data" | Start FAST-LIO localization first |
| "Navigation node crashed" | Check for obstacles, restart controller |
| "Bottle alignment timeout" | Check camera/lighting, restart Phase 2 |

## Configuration

```python
# In integrated_controller.py:
self.goal_tolerance = 0.4        # Goal reach distance (meters)
timeout = 120.0                   # Alignment timeout (seconds)
"--speed", "0.3"                  # Navigation speed (m/s)
```

## Stop/Emergency

- **Ctrl+C** in terminal → Graceful stop (no damping)
- **Emergency stop button** on robot → Hardware stop
- Robot stays in current FSM state (no automatic damping)

## Files

- **Main script**: `src/integrated_controller.py`
- **Full guide**: `src/INTEGRATED_CONTROLLER.md`
- **SSH setup**: `src/SSH_SETUP.md`
- **Prerequisites**: `./check_prerequisites.sh`

## Network

- **Laptop**: 192.168.123.222
- **Robot**: 192.168.123.164
- **LiDAR**: 192.168.123.120
- **Interface**: enp49s0 (change in commands if different)

## FSM States

- **Start**: FSM 801 (balance running)
- **Phase 2-4**: FSM 500 (balance walking)
- **End**: FSM 801 (balance running)

## Success Indicators

✅ Phase 1: "First goal reached!"
✅ Phase 2: "Bottle alignment confirmed!"
✅ Phase 3: "Bottle grasped at Position 4!"
✅ Phase 4: "Second goal reached!"
✅ Phase 5: "FSM ID set to 801"

---

**For detailed information, see**: `src/INTEGRATED_CONTROLLER.md`
