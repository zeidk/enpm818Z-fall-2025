# RWA2: Keyframe-Based LiDAR Odometry

**Course:** ENPM818Z — On-Road Automated Vehicles  
**Assignment:** 2 - SLAM Frontend Implementation  
**Due:** November 18, 2025

## Overview

This assignment implements a keyframe-based LiDAR odometry system using:
- **Scan-to-Map ICP** for pose estimation
- **Keyframe selection** based on motion thresholds
- **Sliding window local mapping** for stable registration

## Quick Start

### 1. Installation

```bash
# Navigate to workspace
cd ~/rwa2_starter

# Install Python dependencies
pip3 install -r slam_frontend/requirements.txt

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 2. Run Development Dataset

```bash
# Use this 90% of the time for development
ros2 launch slam_frontend rwa2_dev.launch.py
```

### 3. Run Full Dataset (Final Testing)

```bash
# Use before submission to test complete sequence
ros2 launch slam_frontend rwa2_full.launch.py
```

## Package Structure

```
rwa2_starter/
├── data/
│   ├── kitti_00_dev/        # Development dataset (1000 frames)
│   └── kitti_00_full/       # Full dataset (4541 frames)
│
├── slam_interfaces/         # Custom messages
│   └── msg/
│       └── Keyframe.msg     # Point cloud + pose bundle
│
└── slam_frontend/           # Main implementation package
    ├── slam_frontend/
    │   ├── icp_utils.py              # ✅ Complete - ICP alignment
    │   ├── transform_utils.py        # ✅ Complete - SE(3) operations
    │   ├── pointcloud_utils.py       # ✅ Complete - Point cloud ops
    │   ├── odometry_estimator_node.py    # ⚠️ TODO - Implement this
    │   └── local_map_manager_node.py     # ⚠️ TODO - Implement this
    │
    ├── config/
    │   └── params.yaml          # ✅ Complete - Tune parameters
    │
    ├── launch/
    │   ├── rwa2_dev.launch.py   # ✅ Complete - Dev dataset launch
    │   └── rwa2_full.launch.py  # ✅ Complete - Full dataset launch
    │
    └── rviz/
        └── rwa2.rviz            # ✅ Complete - Visualization config
```

**Legend:**
- ✅ = Complete (do not modify)
- ⚠️ = Template with TODO sections (you implement)

## What You Need to Implement

### Node 1: Odometry Estimator

**File:** `slam_frontend/odometry_estimator_node.py`

**TODO Sections:**

1. **`should_create_keyframe()`** - Keyframe selection logic
   - Compute translation distance
   - Compute rotation angle
   - Return True if either threshold exceeded

2. **`publish_keyframe()`** - Create and publish keyframe
   - Create Keyframe message
   - Set cloud and pose fields
   - Publish and update state

3. **`pointcloud_callback()`** - Main odometry loop
   - Convert ROS message to Open3D
   - Align scan to local map using ICP
   - Update pose (chain transformations)
   - Publish odometry and path
   - Check if keyframe needed

### Node 2: Local Map Manager

**File:** `slam_frontend/local_map_manager_node.py`

**TODO Sections:**

1. **`keyframe_callback()`** - Handle incoming keyframes
   - Convert ROS messages to Open3D/numpy
   - Add to deque (sliding window)
   - Trigger map update

2. **`update_local_map()`** - Rebuild local map
   - Transform each keyframe to map frame
   - Merge all clouds
   - Downsample merged cloud
   - Publish result

## Topics

### Input (from bag file)
- `/kitti/velo/pointcloud` - Raw LiDAR scans (subscribe here)
- `/ground_truth/odometry` - Ground truth poses (optional, for evaluation)
- `/tf` - Transform tree

### Output (your nodes)
- `/odom` - Estimated odometry (from odometry_estimator)
- `/path` - Accumulated trajectory (from odometry_estimator)
- `/slam/new_keyframe` - Keyframe notifications (from odometry_estimator)
- `/slam/local_map` - Aggregated local map (from local_map_manager)

## Development Workflow

```bash
# Every new terminal needs:
cd ~/rwa2_starter
source install/setup.bash

# Edit code (no rebuild needed with --symlink-install)
code slam_frontend/slam_frontend/odometry_estimator_node.py

# Test immediately
ros2 launch slam_frontend rwa2_dev.launch.py

# Monitor in another terminal
ros2 topic hz /odom
ros2 topic echo /slam/new_keyframe --once
```

## Parameter Tuning

Edit `config/params.yaml`:

```yaml
odometry_estimator_node:
  ros__parameters:
    keyframe_distance_threshold: 1.0    # meters
    keyframe_rotation_threshold: 15.0   # degrees

local_map_manager_node:
  ros__parameters:
    max_keyframes: 20
    voxel_size: 0.2                     # meters
```

**Guidelines:**
- Highway sections: Increase distance threshold (1.5-2.0m)
- Urban sections: Decrease distance threshold (0.5-1.0m)
- Slow processing: Increase voxel_size, reduce max_keyframes
- High drift: Decrease thresholds, increase max_keyframes

## Debugging Commands

```bash
# Check topics
ros2 topic list

# Monitor rates
ros2 topic hz /kitti/velo/pointcloud
ros2 topic hz /odom

# View messages
ros2 topic echo /odom --once
ros2 topic echo /slam/new_keyframe --once

# Check nodes
ros2 node list
ros2 node info /odometry_estimator_node

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

## Expected Results

**Development Dataset (1000 frames):**
- ~100 seconds runtime
- ~100-150 keyframes created
- <10m cumulative drift
- Smooth trajectory

**Full Dataset (4541 frames):**
- ~454 seconds runtime
- ~400-600 keyframes created
- <100m cumulative drift at loop closure
- Complete loop visible

## Common Issues

| Issue | Solution |
|-------|----------|
| "Package not found" | Run `source install/setup.bash` |
| "No module named 'open3d'" | Run `pip3 install -r requirements.txt` |
| Changes don't take effect | Rebuild: `colcon build --symlink-install` |
| Trajectory drifts severely | Check pose chaining order: `T_new = T_old @ delta_T` |
| Too many keyframes | Increase thresholds in params.yaml |
| No point clouds in RViz | Check Fixed Frame = "map" or "velodyne" |

## Submission

Create ZIP file: `rwa2_groupX.zip` (replace X with your group number)

**Include:**
- `slam_frontend/` - Your implementation
- `slam_interfaces/` - Message definitions
- `README.md` - This file with your analysis

**Do NOT include:**
- `data/` folder (bag files)
- `build/`, `install/`, `log/` folders
- `.pyc` files

## Grading Criteria

- **Keyframe Selection (10 pts):** Distance/rotation computation, thresholds
- **Local Map Management (10 pts):** Deque storage, transformation, merging
- **Odometry Estimation (5 pts):** Pose chaining, publishing
- **Integration (5 pts):** Nodes communicate, launch files work, parameters loaded

See assignment document for complete rubric.

## Resources

- **Assignment Document:** See Canvas
- **Open3D Documentation:** https://www.open3d.org/docs/
- **ROS 2 Documentation:** https://docs.ros.org/en/humble/
- **Office Hours:** See syllabus

## Contact

Post questions on Piazza (do not share code!).

---

**Good luck!** 🚀
