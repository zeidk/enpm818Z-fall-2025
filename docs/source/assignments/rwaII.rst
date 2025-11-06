==================================================================
Assignment 2: Building a Keyframe-Based LiDAR Odometry Pipeline
==================================================================

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L2C — SLAM for the Real World (Frontend)
:Assigned: November 6, 2025
:Due: November 18, 2025 (1.5 weeks)
:Total Points: 30 pts
:Language: Python (ROS 2, Open3D, NumPy)

---------------------------------------------------------
🎯 1. Objective
---------------------------------------------------------

Build a simplified **LiDAR odometry pipeline** that estimates vehicle trajectory from raw point cloud data. You will implement two core concepts of modern SLAM systems:

- **Keyframe Selection** – Decide which scans become keyframes to represent the map efficiently
- **Local Map Management** – Maintain a sliding window of recent keyframes as the registration reference

You will be provided with helper utilities and a complete starter package. Your task is to implement the **core SLAM logic** in two ROS 2 nodes.

---------------------------------------------------------
📚 2. Background
---------------------------------------------------------

In the lecture, we discussed why simple **scan-to-scan matching** is computationally expensive and prone to drift. A more robust approach, **scan-to-local-map matching**, aligns each incoming scan to a **local submap** built from several past keyframes.

This assignment implements that approach:

**Keyframe Selection:**
  Choose which scans to save as keyframes based on robot motion (translation and rotation). This reduces redundant data and keeps the map sparse.

**Local Map Management:**
  Maintain a sliding window of *N* keyframes. The current scan is aligned against this aggregated map using ICP. When a new keyframe is added, the oldest one is dropped to maintain a fixed-size buffer.

**Why This Matters:**
  This is the foundation of modern LiDAR SLAM systems like LOAM, LeGO-LOAM, and LIO-SAM. Understanding keyframe selection and local mapping is essential for building scalable SLAM systems.

---------------------------------------------------------
📂 3. KITTI Dataset and ROS 2 Bag Files
---------------------------------------------------------

You will use **KITTI Odometry Sequence 00** — a standard benchmark sequence combining highway and urban driving, ideal for evaluating odometry and keyframe strategies.

Dataset Overview
~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - **Property**
     - **Description**
   * - **Sequence**
     - 00 (Highway and urban driving, 4.5 km loop)
   * - **Frames**
     - 4,541 total frames (~7.5 minutes at 10 Hz)
   * - **Environment**
     - Mixed: highway (straight) → urban (turns and intersections)
   * - **Sensor**
     - Velodyne HDL-64E LiDAR (10 Hz, 64 laser rings)
   * - **Coordinate Frame**
     - +X forward, +Y left, +Z up (KITTI convention)
   * - **Characteristics**
     - Good features, moderate drift without loop closure

ROS 2 Bag Files Provided
~~~~~~~~~~~~~~~~~~~~~~~~~

Two pre-recorded bag files are provided to you:

1. **Development Dataset** (``kitti_00_dev``) — First 1,000 frames (~800 MB)
   
   - Use this for 90% of your development and testing
   - Faster iteration, easier debugging
   - Covers ~1.6 km (highway and urban transition)
   - Duration: ~100 seconds

2. **Full Dataset** (``kitti_00_full``) — Complete 4,541 frames (~3.5 GB)
   
   - Use for final testing before submission
   - Shows long-term behavior and drift accumulation
   - Covers entire ~4.5 km trajectory
   - Duration: ~454 seconds

.. tip::

   **Development Strategy:**
   
   - Days 1-9: Use ``kitti_00_dev`` exclusively (fast iterations)
   - Days 10-11: Test with ``kitti_00_full`` (final validation)
   - This saves ~75% of testing time during development

Topics in the Bag Files
~~~~~~~~~~~~~~~~~~~~~~~~

Each bag file contains the following topics:

.. list-table::
   :widths: 30 20 50
   :header-rows: 1

   * - **Topic**
     - **Type**
     - **Description**
   * - ``/kitti/velo/pointcloud``
     - sensor_msgs/PointCloud2
     - **Primary input for your SLAM nodes.** Raw Velodyne LiDAR point clouds with fields: x, y, z, intensity. ~100k-120k points per scan. Frame ID: ``velodyne``. Published at 10 Hz.
   * - ``/ground_truth/odometry``
     - nav_msgs/Odometry
     - **Optional ground truth for evaluation.** Vehicle pose in world frame from KITTI ground truth. Use this to compare your estimated trajectory against the true path. Frame IDs: ``world`` (parent) → ``base_link`` (child).
   * - ``/tf``
     - tf2_msgs/TFMessage
     - **Transform tree for coordinate frames.** Contains two transforms: (1) ``world`` → ``base_link`` (vehicle pose from ground truth), (2) ``base_link`` → ``velodyne`` (sensor calibration). Your nodes will publish ``map`` → ``odom`` transform.

**Usage Notes:**

- **Primary Topic:** Your ``odometry_estimator_node`` subscribes to ``/kitti/velo/pointcloud``
- **Ground Truth:** Useful for debugging and evaluation, but your algorithm should NOT use it for odometry estimation
- **TF Tree:** After launch, the complete tree will be: ``world`` → ``base_link`` → ``velodyne`` and ``map`` → ``odom`` → ``base_link``
- **Frame Conventions:** KITTI uses standard LiDAR convention (+X forward, +Y left, +Z up)

---------------------------------------------------------
🛠️ 4. Assignment Tasks
---------------------------------------------------------

You will implement **two ROS 2 nodes** in the provided ``slam_frontend`` package. The starter package includes templates with clear TODO sections showing exactly what you need to implement.

Node 1 — odometry_estimator_node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose:** Estimate robot pose and determine when to create new keyframes.

**Subscriptions:**

- ``/kitti/velo/pointcloud`` (sensor_msgs/PointCloud2) — Raw LiDAR scans
- ``/slam/local_map`` (sensor_msgs/PointCloud2) — Aggregated local map

**Publications:**

- ``/odom`` (nav_msgs/Odometry) — Current pose estimate
- ``/path`` (nav_msgs/Path) — Accumulated trajectory
- ``/slam/new_keyframe`` (slam_interfaces/Keyframe) — New keyframe notifications
- ``/tf`` (tf2_msgs/TFMessage) — Broadcast ``map → odom`` transform

**What You Implement:**

.. code-block:: python

   def should_create_keyframe(self, current_pose, last_keyframe_pose):
       """
       Determine if current pose warrants a new keyframe
       
       TODO: Implement keyframe selection logic
       
       Steps:
         1. Compute translation distance using compute_distance()
         2. Compute rotation angle using compute_rotation_angle()
         3. Return True if either threshold exceeded
       
       Returns:
           bool: True if new keyframe should be created
       """

.. code-block:: python

   def pointcloud_callback(self, msg):
       """
       Main processing loop - called for each new scan
       
       TODO: Implement scan processing and odometry estimation
       
       Steps:
         1. Align scan to local map using align_clouds()
         2. Update pose: self.current_pose = self.current_pose @ delta_T
         3. Publish odometry, path, and TF
         4. Check if new keyframe needed
         5. If yes, publish keyframe
       """

.. code-block:: python

   def publish_keyframe(self, cloud_msg):
       """
       Create and publish new keyframe message
       
       TODO: Implement keyframe publication
       
       Steps:
         1. Create Keyframe message
         2. Set cloud and pose fields
         3. Publish to /slam/new_keyframe
         4. Update last_keyframe_pose
       """

**Algorithm Overview:**

.. code-block:: text

   Initialize:
     current_pose ← Identity matrix
     last_keyframe_pose ← Identity matrix
     Publish first scan as keyframe
   
   For each new scan:
     ΔT ← ICP_Align(scan, local_map)
     current_pose ← current_pose × ΔT
     
     Publish odometry and path
     
     If should_create_keyframe(current_pose, last_keyframe_pose):
       Publish keyframe
       last_keyframe_pose ← current_pose

Node 2 — local_map_manager_node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose:** Maintain a sliding window of keyframes and publish the aggregated local map.

**Subscriptions:**

- ``/slam/new_keyframe`` (slam_interfaces/Keyframe) — Keyframe notifications

**Publications:**

- ``/slam/local_map`` (sensor_msgs/PointCloud2) — Merged and downsampled local map

**What You Implement:**

.. code-block:: python

   def keyframe_callback(self, msg):
       """
       Handle incoming keyframe
       
       TODO: Implement keyframe storage
       
       Steps:
         1. Convert pose to 4x4 matrix
         2. Add (cloud, pose) to deque
         3. Rebuild local map
       """

.. code-block:: python

   def update_local_map(self):
       """
       Rebuild local map from keyframe window
       
       TODO: Implement local map construction
       
       Steps:
         1. Transform each keyframe to map frame
         2. Merge all transformed clouds
         3. Downsample merged cloud
         4. Publish result
       """

**Algorithm Overview:**

.. code-block:: text

   Initialize:
     keyframes ← empty deque (max size N)
   
   On new keyframe:
     Add (cloud, pose) to keyframes
     If size > N:
       Remove oldest keyframe
     
     For each keyframe in window:
       Transform cloud to map frame
       Add to merged cloud
     
     Downsample merged cloud
     Publish as local_map

**Parameter Configuration:**

Both nodes use parameters from ``config/params.yaml``:

.. code-block:: yaml

   odometry_estimator:
     ros__parameters:
       keyframe_distance_threshold: 1.0    # meters
       keyframe_rotation_threshold: 15.0   # degrees
       
   local_map_manager:
     ros__parameters:
       max_keyframes: 20                   # sliding window size
       voxel_size: 0.2                     # meters (for downsampling)

---------------------------------------------------------
🚀 5. Provided Starter Package
---------------------------------------------------------

The instructor provides a **complete starter package** to help you focus on SLAM logic rather than infrastructure.

**Download:** ``rwa2_starter.zip`` from Canvas (under Assignment 2)

**Package Contents:**

.. code-block:: text

   rwa2_starter/
   ├── data/
   │   ├── kitti_00_dev/                 ✅ Dev dataset (1000 frames)
   │   │   ├── metadata.yaml
   │   │   └── kitti_00_dev_0.db3
   │   └── kitti_00_full/                ✅ Full dataset (4541 frames)
   │       ├── metadata.yaml
   │       └── kitti_00_full_0.db3
   │
   ├── slam_interfaces/                  ✅ Complete message package
   │   ├── msg/
   │   │   └── Keyframe.msg
   │   ├── CMakeLists.txt
   │   └── package.xml
   │
   └── slam_frontend/                    ✅ Python package
       ├── config/
       │   └── params.yaml               ✅ Default parameters
       ├── slam_frontend/
       │   ├── __init__.py
       │   ├── icp_utils.py              ✅ ICP alignment (complete)
       │   ├── transform_utils.py        ✅ Transform helpers (complete)
       │   ├── pointcloud_utils.py       ✅ Point cloud ops (complete)
       │   ├── odometry_estimator_node.py    ⚠️ Template with TODOs
       │   └── local_map_manager_node.py     ⚠️ Template with TODOs
       ├── launch/
       │   ├── rwa2_dev.launch.py        ✅ Launch with dev dataset
       │   └── rwa2_full.launch.py       ✅ Launch with full dataset
       ├── rviz/
       │   └── rwa2.rviz                 ✅ RViz config
       ├── package.xml
       ├── setup.py
       ├── requirements.txt
       └── README.md

.. note::

   **Legend:**
   
   ✅ = Fully implemented (do not modify)
   
   ⚠️ = Template with TODO sections (you implement)

Provided Utilities
~~~~~~~~~~~~~~~~~~

**1. ICP Utilities** (``icp_utils.py``)

.. code-block:: python

   def ros2_to_o3d(cloud_msg):
       """Convert ROS 2 PointCloud2 to Open3D format"""
       # Returns: open3d.geometry.PointCloud

   def align_clouds(source_msg, target_msg, voxel_size=0.2):
       """Align source to target using point-to-point ICP"""
       # Returns: 4x4 numpy array (transformation matrix)

**2. Transform Utilities** (``transform_utils.py``)

.. code-block:: python

   def pose_to_transform_matrix(pose_msg):
       """Convert geometry_msgs/Pose to 4x4 matrix"""
       # Returns: 4x4 numpy array

   def transform_to_pose(T):
       """Convert 4x4 matrix to geometry_msgs/Pose"""
       # Returns: geometry_msgs/Pose

   def compute_distance(T1, T2):
       """Compute Euclidean distance between poses"""
       # Returns: float (meters)

   def compute_rotation_angle(T1, T2):
       """Compute rotation angle between poses"""
       # Returns: float (degrees)

**3. Point Cloud Utilities** (``pointcloud_utils.py``)

.. code-block:: python

   def transform_pointcloud(cloud_msg, transform_matrix):
       """Apply 4x4 transformation to point cloud"""
       # Returns: sensor_msgs/PointCloud2

   def merge_pointclouds(cloud_list):
       """Merge multiple point clouds into one"""
       # Returns: sensor_msgs/PointCloud2

   def downsample_pointcloud(cloud_msg, voxel_size=0.2):
       """Downsample using voxel grid filter"""
       # Returns: sensor_msgs/PointCloud2

Message Definition
~~~~~~~~~~~~~~~~~~

**Keyframe.msg** (in ``slam_interfaces/msg/``)

.. code-block:: text

   sensor_msgs/PointCloud2 cloud
   geometry_msgs/Pose pose

This custom message bundles a point cloud with its pose for efficient keyframe communication.

---------------------------------------------------------
🔧 6. Setup and Command Execution Sequence
---------------------------------------------------------

This section provides step-by-step instructions for setting up your workspace and running the assignment. Follow these commands in the exact sequence shown.

Initial Setup (One-Time)
~~~~~~~~~~~~~~~~~~~~~~~~~

**Step 1: Download and Extract Starter Package**

Download ``rwa2_starter.zip`` from Canvas (under Assignment 2) and extract it:

.. code-block:: bash

   # Navigate to your workspace location
   cd ~/
   
   # Extract the starter package
   unzip rwa2_starter.zip
   
   # Navigate into the workspace
   cd rwa2_starter

The extracted directory contains:

- ``data/`` — KITTI bag files (dev and full datasets, ~4 GB total)
- ``slam_interfaces/`` — Custom message definitions (Keyframe.msg)
- ``slam_frontend/`` — Main Python package with node templates

**Step 2: Install Python Dependencies**

.. code-block:: bash

   # Install required Python packages
   pip3 install -r slam_frontend/requirements.txt

This installs: ``open3d``, ``numpy``, ``scipy`` (if not already in your ROS 2 installation)

**Step 3: Build the Workspace**

.. code-block:: bash

   # Build all packages with symlink support
   colcon build --symlink-install
   
   # Source the workspace overlay
   source install/setup.bash

The ``--symlink-install`` flag creates symbolic links to Python files, allowing you to edit them without rebuilding.

.. important::

   **Critical:** You must run ``source install/setup.bash`` in **every new terminal** before running ROS 2 commands in this workspace.

**Step 4: Verify the Setup**

.. code-block:: bash

   # Check development dataset
   ros2 bag info data/kitti_00_dev
   
   # Expected output includes:
   # Duration: ~100.0s
   # Messages: 1000 (for /kitti/velo/pointcloud)
   # Topics: /kitti/velo/pointcloud, /ground_truth/odometry, /tf
   
   # Check full dataset
   ros2 bag info data/kitti_00_full
   
   # Expected output includes:
   # Duration: ~454.0s
   # Messages: 4541 (for /kitti/velo/pointcloud)

If both commands succeed, your setup is complete!

Understanding the Command Workflow
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**The Three Core Commands You'll Use:**

1. **colcon build** — Compiles packages (C++/CMake) and indexes Python packages
2. **source install/setup.bash** — Loads packages into your environment
3. **ros2 launch** — Starts multiple nodes and plays bag file simultaneously

**When to Rebuild:**

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - **You Changed**
     - **Action Required**
   * - Python code in node files
     - No rebuild needed (with ``--symlink-install``)
   * - ``CMakeLists.txt``
     - ``colcon build --symlink-install``
   * - ``package.xml``
     - ``colcon build --symlink-install``
   * - Message definitions (.msg)
     - ``colcon build --symlink-install``
   * - Launch files (.py)
     - No rebuild needed
   * - Config files (.yaml)
     - No rebuild needed

Development Workflow (Daily Use)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is the workflow you'll repeat throughout the assignment:

**Standard Development Cycle:**

.. code-block:: bash

   # === TERMINAL 1: Main development terminal ===
   cd ~/rwa2_starter
   source install/setup.bash
   
   # 1. Edit your code
   code slam_frontend/slam_frontend/odometry_estimator_node.py
   
   # 2. Save changes
   
   # 3. Launch and test (no rebuild needed with --symlink-install)
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # 4. Observe output and RViz visualization
   
   # 5. Press Ctrl+C when done
   
   # 6. Repeat steps 1-5

**What the Launch Command Does:**

When you run ``ros2 launch slam_frontend rwa2_dev.launch.py``, it automatically:

1. ✅ Plays the bag file (``data/kitti_00_dev/``)
2. ✅ Starts ``odometry_estimator_node``
3. ✅ Starts ``local_map_manager_node``
4. ✅ Loads parameters from ``config/params.yaml``
5. ✅ Opens RViz2 with pre-configured visualization

You don't need to run these separately!

Running Your Implementation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Option 1: Use Launch Files (Recommended)**

.. code-block:: bash

   # Development dataset (1000 frames, ~100 seconds)
   # Use this 90% of the time
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # Full dataset (4541 frames, ~454 seconds)
   # Use before final submission
   ros2 launch slam_frontend rwa2_full.launch.py

**Option 2: Manual Control (Advanced)**

If you want to run components separately for debugging:

.. code-block:: bash

   # === TERMINAL 1: Play bag manually ===
   cd ~/rwa2_starter
   source install/setup.bash
   ros2 bag play data/kitti_00_dev --rate 1.0
   
   # === TERMINAL 2: Run odometry estimator ===
   cd ~/rwa2_starter
   source install/setup.bash
   ros2 run slam_frontend odometry_estimator_node
   
   # === TERMINAL 3: Run local map manager ===
   cd ~/rwa2_starter
   source install/setup.bash
   ros2 run slam_frontend local_map_manager_node
   
   # === TERMINAL 4: Open RViz ===
   cd ~/rwa2_starter
   source install/setup.bash
   rviz2 -d slam_frontend/rviz/rwa2.rviz

Monitoring and Debugging
~~~~~~~~~~~~~~~~~~~~~~~~~

While your system is running, open additional terminals to monitor:

**Check Active Topics:**

.. code-block:: bash

   # === TERMINAL 2: Topic monitoring ===
   cd ~/rwa2_starter
   source install/setup.bash
   
   ros2 topic list

Expected topics:

.. code-block:: text

   /kitti/velo/pointcloud       ← Input from bag file
   /ground_truth/odometry       ← Ground truth from bag file
   /tf                          ← Transforms from bag file + your nodes
   /odom                        ← Your odometry_estimator_node
   /path                        ← Your odometry_estimator_node
   /slam/new_keyframe           ← Your odometry_estimator_node
   /slam/local_map              ← Your local_map_manager_node

**Monitor Publishing Rates:**

.. code-block:: bash

   # Check input point cloud rate
   ros2 topic hz /kitti/velo/pointcloud
   # Expected: ~10 Hz
   
   # Check your odometry output rate
   ros2 topic hz /odom
   # Expected: ~10 Hz (should match input)
   
   # Check keyframe creation rate
   ros2 topic hz /slam/new_keyframe
   # Expected: ~1-2 Hz (depends on motion and thresholds)
   
   # Check local map updates
   ros2 topic hz /slam/local_map
   # Expected: ~1-2 Hz (matches keyframe rate)

**Inspect Message Contents:**

.. code-block:: bash

   # View one odometry message
   ros2 topic echo /odom --once
   
   # View one keyframe message
   ros2 topic echo /slam/new_keyframe --once
   
   # View point cloud info (header only)
   ros2 topic echo /kitti/velo/pointcloud --once --no-arr

**Check Transform Tree:**

.. code-block:: bash

   # Generate TF tree visualization
   ros2 run tf2_tools view_frames
   
   # Open the generated PDF
   evince frames.pdf
   
   # Expected tree:
   # world → base_link → velodyne (from bag file)
   # map → odom (from your odometry_estimator_node)

**Monitor Node Status:**

.. code-block:: bash

   # List running nodes
   ros2 node list
   # Expected: /odometry_estimator_node, /local_map_manager_node
   
   # Get detailed node information
   ros2 node info /odometry_estimator_node
   ros2 node info /local_map_manager_node
   
   # Check parameter values
   ros2 param list /odometry_estimator_node
   ros2 param get /odometry_estimator_node keyframe_distance_threshold

Complete Command Sequence by Timeline
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Day 1: Setup and Familiarization**

.. code-block:: bash

   # === One-time setup ===
   cd ~/
   unzip rwa2_starter.zip
   cd rwa2_starter
   pip3 install -r slam_frontend/requirements.txt
   colcon build --symlink-install
   source install/setup.bash
   
   # Verify datasets
   ros2 bag info data/kitti_00_dev
   ros2 bag info data/kitti_00_full
   
   # First launch (nodes are templates, won't do much yet)
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # In another terminal: check topics
   ros2 topic list
   ros2 topic hz /kitti/velo/pointcloud

**Days 2-9: Iterative Development**

.. code-block:: bash

   # === Start each session ===
   cd ~/rwa2_starter
   source install/setup.bash
   
   # Edit code
   code slam_frontend/slam_frontend/odometry_estimator_node.py
   
   # Test immediately (no rebuild needed)
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # Debug in another terminal
   ros2 topic echo /odom --once
   ros2 topic hz /slam/new_keyframe
   
   # Iterate: Edit → Save → Launch → Test → Repeat

**Days 10-11: Final Validation**

.. code-block:: bash

   # === Test with full dataset ===
   cd ~/rwa2_starter
   source install/setup.bash
   
   # Run full 4541-frame sequence
   ros2 launch slam_frontend rwa2_full.launch.py
   
   # Let it run for ~7.5 minutes
   # Verify trajectory completes the loop
   
   # Check final trajectory quality in RViz

Troubleshooting Commands
~~~~~~~~~~~~~~~~~~~~~~~~~

**Common Issues and Diagnostic Commands:**

.. code-block:: bash

   # Issue: "ros2: command not found"
   source /opt/ros/humble/setup.bash  # or your ROS 2 distro
   
   # Issue: "Package 'slam_frontend' not found"
   cd ~/rwa2_starter
   source install/setup.bash
   
   # Issue: Python import errors
   pip3 install -r slam_frontend/requirements.txt
   
   # Issue: Changes don't take effect
   colcon build --symlink-install
   source install/setup.bash
   
   # Issue: Want to see detailed logs
   ros2 launch slam_frontend rwa2_dev.launch.py --log-level debug
   
   # Issue: Node crashes on startup
   # Run node directly to see full error:
   ros2 run slam_frontend odometry_estimator_node

**Verify Your Implementation:**

.. code-block:: bash

   # After implementing keyframe selection:
   ros2 topic hz /slam/new_keyframe
   # Should see ~1-2 Hz
   
   # After implementing local map:
   ros2 topic hz /slam/local_map
   # Should see updates matching keyframes
   
   # After implementing odometry:
   ros2 topic echo /path --once
   # Should see growing list of poses

Quick Reference Card
~~~~~~~~~~~~~~~~~~~~

**Commands You'll Use Most:**

.. code-block:: bash

   # === Every new terminal needs this ===
   cd ~/rwa2_starter
   source install/setup.bash
   
   # === Development (use 90% of time) ===
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # === Final testing ===
   ros2 launch slam_frontend rwa2_full.launch.py
   
   # === Only if you modify CMakeLists.txt or package.xml ===
   colcon build --symlink-install
   
   # === Debug helpers ===
   ros2 topic list                              # See all topics
   ros2 topic hz /kitti/velo/pointcloud        # Check rate
   ros2 topic echo /odom --once                # See one message
   ros2 node list                               # See running nodes
   ros2 node info /odometry_estimator_node     # Node details

**File Locations:**

.. code-block:: text

   ~/rwa2_starter/
   ├── data/
   │   ├── kitti_00_dev/        ← Dev dataset bag file
   │   └── kitti_00_full/       ← Full dataset bag file
   ├── slam_frontend/
   │   ├── slam_frontend/
   │   │   ├── odometry_estimator_node.py    ← Edit this
   │   │   └── local_map_manager_node.py     ← Edit this
   │   ├── config/params.yaml   ← Tune parameters here
   │   └── launch/
   │       ├── rwa2_dev.launch.py
   │       └── rwa2_full.launch.py

---------------------------------------------------------
📋 7. Implementation Guide
---------------------------------------------------------

This section provides a complete, step-by-step guide with exact commands to implement and test your SLAM pipeline.

Step 1: Setup and Build (10 minutes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Extract and Navigate to Workspace:**

.. code-block:: bash

   # Extract starter package
   cd ~/
   unzip rwa2_starter.zip
   cd rwa2_starter

**Install Python Dependencies:**

.. code-block:: bash

   # Install required Python packages
   pip3 install -r slam_frontend/requirements.txt
   
   # Verify Open3D installation
   python3 -c "import open3d; print(f'Open3D version: {open3d.__version__}')"

**Build the Workspace:**

.. code-block:: bash

   # Build both packages
   colcon build --symlink-install
   
   # Source the workspace
   source install/setup.bash
   
   # Verify packages are built
   ros2 pkg list | grep slam

Expected output::

   slam_frontend
   slam_interfaces

**Verify Dataset Files:**

.. code-block:: bash

   # Check development dataset
   ros2 bag info data/kitti_00_dev

Expected output should show::

   Files:             kitti_00_dev_0.db3
   Duration:          100.0s
   Start:             <timestamp>
   End:               <timestamp>
   Messages:          1000
   Topic information: Topic: /kitti/velo/pointcloud | Type: sensor_msgs/msg/PointCloud2 | Count: 1000

.. code-block:: bash

   # Check full dataset
   ros2 bag info data/kitti_00_full

Expected output should show::

   Messages:          4541
   Topic: /kitti/velo/pointcloud | Type: sensor_msgs/msg/PointCloud2 | Count: 4541

Step 2: Understand the Templates (20 minutes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Open Node Templates:**

.. code-block:: bash

   # Open both templates in your editor
   code slam_frontend/slam_frontend/odometry_estimator_node.py
   code slam_frontend/slam_frontend/local_map_manager_node.py

Look for:

- ``TODO:`` comments marking what you implement
- ``Hints:`` providing implementation guidance  
- Existing code showing node structure and utilities

**Review Provided Utilities:**

.. code-block:: bash

   # Review utility files (complete implementations)
   code slam_frontend/slam_frontend/icp_utils.py
   code slam_frontend/slam_frontend/transform_utils.py
   code slam_frontend/slam_frontend/pointcloud_utils.py

Step 3: Test Provided Code (10 minutes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Terminal 1: Launch the System**

.. code-block:: bash

   # Source workspace
   source install/setup.bash
   
   # Launch with development dataset
   ros2 launch slam_frontend rwa2_dev.launch.py

Expected output::

   [INFO] [launch]: All log files can be found below /home/...
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [bag_player-1]: process started with pid [...]
   [INFO] [odometry_estimator_node-2]: process started with pid [...]
   [INFO] [local_map_manager_node-3]: process started with pid [...]

**Terminal 2: Monitor Topics**

.. code-block:: bash

   # List all active topics
   ros2 topic list

Expected output::

   /kitti/velo/pointcloud      ← From bag file
   /ground_truth/odometry      ← From bag file
   /tf                         ← From bag file
   /slam/new_keyframe          ← Your odometry node will publish here
   /slam/local_map             ← Your local map node will publish here
   /odom                       ← Your odometry node will publish here
   /path                       ← Your odometry node will publish here

.. code-block:: bash

   # Check point cloud publishing rate
   ros2 topic hz /kitti/velo/pointcloud

Expected output::

   average rate: 10.001
   min: 0.099s max: 0.101s std dev: 0.00050s window: 100

.. code-block:: bash

   # Echo one point cloud message
   ros2 topic echo /kitti/velo/pointcloud --once

**Terminal 3: Open RViz2**

.. code-block:: bash

   # Open RViz with provided configuration
   rviz2 -d slam_frontend/rviz/rwa2.rviz

You should see:

- Point clouds being published at 10 Hz (green/colored points)
- Ground truth path (if configured)
- No estimated path yet (nodes are templates)

.. note::

   At this stage, the bag file plays successfully and point clouds are visible, but your nodes don't do anything useful yet because they are just templates. This is expected!

Step 4: Implement Node 1 - Odometry Estimator (4-6 hours)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Implement in this order:

**4.1 First: Keyframe selection logic**

Open ``slam_frontend/slam_frontend/odometry_estimator_node.py`` and implement:

.. code-block:: python

   def should_create_keyframe(self, current_pose, last_keyframe_pose):
       """TODO: Implement keyframe selection logic"""
       # Compute distance
       distance = compute_distance(current_pose, last_keyframe_pose)
       
       # Compute rotation
       angle = compute_rotation_angle(current_pose, last_keyframe_pose)
       
       # Check thresholds
       if distance > self.keyframe_distance_threshold:
           return True
       if angle > self.keyframe_rotation_threshold:
           return True
       
       return False

**Test after implementing:**

.. code-block:: bash

   # Rebuild workspace
   colcon build --symlink-install --packages-select slam_frontend
   
   # Source
   source install/setup.bash
   
   # Run with development dataset
   ros2 launch slam_frontend rwa2_dev.launch.py

Check console output for keyframe creation messages.

**4.2 Second: Keyframe publication**

.. code-block:: python

   def publish_keyframe(self, cloud_msg):
       """TODO: Implement keyframe publication"""
       keyframe = Keyframe()
       keyframe.cloud = cloud_msg
       keyframe.pose = transform_to_pose(self.current_pose)
       
       self.keyframe_pub.publish(keyframe)
       self.last_keyframe_pose = self.current_pose.copy()
       
       self.get_logger().info(f'Created keyframe (total: {self.keyframe_count})')
       self.keyframe_count += 1

**Test after implementing:**

.. code-block:: bash

   # Rebuild and run
   colcon build --symlink-install --packages-select slam_frontend
   source install/setup.bash
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # In another terminal, check keyframe topic
   ros2 topic hz /slam/new_keyframe
   
   # Echo one keyframe
   ros2 topic echo /slam/new_keyframe --once

**4.3 Third: Main processing loop**

.. code-block:: python

   def pointcloud_callback(self, msg):
       """TODO: Implement scan processing and odometry estimation"""
       # Handle first scan
       if self.first_scan:
           self.publish_keyframe(msg)
           self.first_scan = False
           return
       
       # Wait for local map
       if self.local_map is None:
           return
       
       # Align to local map
       delta_T = align_clouds(msg, self.local_map)
       
       # Update pose
       self.current_pose = self.current_pose @ delta_T
       
       # Publish odometry and path
       self.publish_odometry()
       self.publish_path()
       
       # Check keyframe
       if self.should_create_keyframe(self.current_pose, self.last_keyframe_pose):
           self.publish_keyframe(msg)

**Test after implementing:**

.. code-block:: bash

   # Rebuild and run
   colcon build --symlink-install --packages-select slam_frontend
   source install/setup.bash
   ros2 launch slam_frontend rwa2_dev.launch.py

Check in RViz:

- Trajectory path should appear (red line)
- Path should follow the road
- Console shows periodic keyframe creation

**Verify Node 1 is Working:**

.. code-block:: bash

   # Check all output topics
   ros2 topic list | grep -E "(odom|path|keyframe)"
   
   # Check publishing rates
   ros2 topic hz /odom
   ros2 topic hz /path
   ros2 topic hz /slam/new_keyframe
   
   # Check node info
   ros2 node info /odometry_estimator

Step 5: Implement Node 2 - Local Map Manager (3-5 hours)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**5.1 First: Keyframe storage**

Open ``slam_frontend/slam_frontend/local_map_manager_node.py`` and implement:

.. code-block:: python

   def keyframe_callback(self, msg):
       """TODO: Implement keyframe storage"""
       # Convert pose to matrix
       pose_matrix = pose_to_transform_matrix(msg.pose)
       
       # Add to deque (automatically removes oldest if full)
       self.keyframes.append((msg.cloud, pose_matrix))
       
       self.get_logger().info(f'Added keyframe (window size: {len(self.keyframes)})')
       
       # Rebuild map
       self.update_local_map()

**5.2 Second: Local map construction**

.. code-block:: python

   def update_local_map(self):
       """TODO: Implement local map construction"""
       if not self.keyframes:
           return
       
       # Transform all keyframes to map frame
       transformed_clouds = []
       for cloud, pose in self.keyframes:
           transformed = transform_pointcloud(cloud, pose)
           transformed_clouds.append(transformed)
       
       # Merge
       merged = merge_pointclouds(transformed_clouds)
       
       # Downsample
       downsampled = downsample_pointcloud(merged, self.voxel_size)
       
       # Publish
       self.local_map_pub.publish(downsampled)
       
       self.get_logger().info(f'Published local map with {len(self.keyframes)} keyframes')

**Test after implementing:**

.. code-block:: bash

   # Rebuild
   colcon build --symlink-install --packages-select slam_frontend
   source install/setup.bash
   
   # Run full system
   ros2 launch slam_frontend rwa2_dev.launch.py

**Verify Node 2 is Working:**

.. code-block:: bash

   # Check local map topic
   ros2 topic hz /slam/local_map
   ros2 topic echo /slam/local_map --once
   
   # Check node info
   ros2 node info /local_map_manager

In RViz, you should now see:

- Local map visualization (aggregated point cloud)
- Smooth trajectory following the road
- Keyframes being added periodically

Step 6: Integration Testing (2-3 hours)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**6.1 Test with Development Dataset**

.. code-block:: bash

   # Terminal 1: Launch system
   source install/setup.bash
   ros2 launch slam_frontend rwa2_dev.launch.py
   
   # Terminal 2: Monitor performance
   ros2 topic hz /odom
   ros2 topic hz /slam/local_map
   
   # Terminal 3: Open RViz
   rviz2 -d slam_frontend/rviz/rwa2.rviz

Watch for:

- Smooth trajectory path
- Periodic keyframe creation
- No error messages in console
- Reasonable processing speed (~10 Hz)

**6.2 Parameter Tuning**

Edit ``slam_frontend/config/params.yaml``:

.. code-block:: yaml

   odometry_estimator:
     ros__parameters:
       keyframe_distance_threshold: 1.0    # Try 0.5-2.0
       keyframe_rotation_threshold: 15.0   # Try 10-20
       
   local_map_manager:
     ros__parameters:
       max_keyframes: 20                   # Try 15-25
       voxel_size: 0.2                     # Try 0.1-0.3

After changing parameters:

.. code-block:: bash

   # No rebuild needed (YAML file read at runtime)
   source install/setup.bash
   ros2 launch slam_frontend rwa2_dev.launch.py

**6.3 Test with Full Dataset**

.. code-block:: bash

   # Run with full dataset
   source install/setup.bash
   ros2 launch slam_frontend rwa2_full.launch.py

This takes ~7.5 minutes to complete. Monitor:

- Trajectory quality over long distance
- Memory usage (should stay reasonable)
- Processing speed (should maintain ~10 Hz)
- Drift accumulation

**6.4 Record Results**

.. code-block:: bash

   # Save trajectory for analysis
   ros2 topic echo /path > dev_trajectory.txt
   
   # Compare with ground truth
   ros2 topic echo /ground_truth/odometry > ground_truth.txt

Step 7: Final Validation (1 hour)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**7.1 Comprehensive Test Checklist**

Run these commands to verify everything works:

.. code-block:: bash

   # 1. Verify workspace builds
   colcon build --symlink-install
   
   # 2. Source and list packages
   source install/setup.bash
   ros2 pkg list | grep slam
   
   # 3. Test dev dataset
   ros2 launch slam_frontend rwa2_dev.launch.py
   # (Let it run completely, check for errors)
   
   # 4. Test full dataset
   ros2 launch slam_frontend rwa2_full.launch.py
   # (Let it run completely, check for errors)
   
   # 5. Verify all topics
   ros2 topic list | grep -E "(odom|path|keyframe|local_map)"
   
   # 6. Check TF tree
   ros2 run tf2_tools view_frames
   evince frames.pdf

**7.2 Performance Benchmarks**

Your implementation should meet these criteria:

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - **Metric**
     - **Expected Value**
   * - Odometry publishing rate
     - ~10 Hz (close to input rate)
   * - Keyframe creation rate
     - ~1-3 per second (depends on motion)
   * - Local map update rate
     - ~1-3 Hz (when keyframes created)
   * - Memory usage
     - <2 GB for dev dataset, <4 GB for full
   * - Dev dataset completion time
     - ~100 seconds (real-time playback)
   * - Full dataset completion time
     - ~454 seconds (real-time playback)

**7.3 Common Issues and Debugging**

If something doesn't work:

.. code-block:: bash

   # Check if nodes are running
   ros2 node list
   
   # Check node details
   ros2 node info /odometry_estimator
   ros2 node info /local_map_manager
   
   # Check for error messages
   ros2 topic echo /rosout | grep ERROR
   
   # Verify subscriptions/publications
   ros2 topic info /kitti/velo/pointcloud
   ros2 topic info /slam/local_map
   
   # Check parameter values
   ros2 param list /odometry_estimator
   ros2 param get /odometry_estimator keyframe_distance_threshold
          # Downsample
          downsampled = downsample_pointcloud(merged, self.voxel_size)
          
          # Publish
          self.local_map_pub.publish(downsampled)

**Test:**

.. code-block:: bash

   # Check that local map is being published
   ros2 topic hz /slam/local_map

   # In RViz, you should see local map updating

**Step 6: Integration Testing (2-3 hours)**

.. code-block:: bash

   # Run full pipeline with dev dataset
   ros2 launch slam_frontend rwa2_dev.launch.py

Check:

- ✅ Trajectory path forms smoothly
- ✅ Keyframes created at regular intervals
- ✅ Local map updates as robot moves
- ✅ No crashes or errors

**Step 7: Parameter Tuning (1-2 hours)**

Edit ``config/params.yaml`` and test different values:

.. code-block:: yaml

   # Try different thresholds
   keyframe_distance_threshold: 0.5  # More keyframes
   keyframe_distance_threshold: 2.0  # Fewer keyframes
   
   # Try different window sizes
   max_keyframes: 10   # Smaller local map
   max_keyframes: 30   # Larger local map

Observe how changes affect:

- Trajectory smoothness
- Computational cost
- Drift accumulation

**Step 8: Final Testing with Full Dataset (1 hour)**

.. code-block:: bash

   # Test with full sequence
   ros2 launch slam_frontend rwa2_full.launch.py

   # Let it run for the full 7.5 minutes
   # Check trajectory quality

---------------------------------------------------------
📊 8. Expected Behavior and Results
---------------------------------------------------------

**Trajectory Characteristics:**

On the **development dataset** (1000 frames):

- ✅ Smooth trajectory with clear highway → urban transition
- ✅ Keyframes every ~1 meter (adjust threshold if too many/few)
- ✅ Local map updates as robot moves
- ✅ Minimal drift on straight sections (<5m)
- ✅ Some drift on turns (expected without loop closure)

On the **full dataset** (4541 frames):

- ✅ Complete loop trajectory visible
- ✅ Accumulated drift at loop closure point (<100m expected)
- ✅ Demonstrates why backend optimization is needed
- ✅ Shows effectiveness of keyframe-based approach vs scan-to-scan

**Visual Verification in RViz:**

.. code-block:: text

   Expected visualization:
   
   [Point Cloud] - Current scan in white
   [Local Map] - Aggregated keyframes in color
   [Path] - Green line showing trajectory
   [TF Tree] - map → odom → base_link
   
   Timeline (dev dataset):
   0-300: Straight highway section
   300-600: Gentle curves
   600-1000: Urban turns and intersections

**Console Output:**

.. code-block:: text

   [odometry_estimator_node]: Received first scan - creating initial keyframe
   [local_map_manager_node]: Added keyframe (window size: 1)
   [odometry_estimator_node]: Created keyframe at distance: 1.02m
   [local_map_manager_node]: Added keyframe (window size: 2)
   ...
   [local_map_manager_node]: Added keyframe (window size: 20)
   [local_map_manager_node]: Removed oldest keyframe (window full)

**Performance Metrics:**

- **Processing rate:** Should maintain ~10 Hz (real-time)
- **Keyframes created:** ~1000-1500 for full sequence (depends on thresholds)
- **Memory usage:** ~2-3 GB (local map + history)
- **Trajectory error:** <100m drift on full sequence (no loop closure)

**Common Issues and Solutions:**

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - **Problem**
     - **Solution**
   * - Trajectory drifts severely (>200m)
     - Check pose chaining: ``T_new = T_old @ delta_T`` (not ``delta_T @ T_old``)
   * - Too many keyframes (>3000)
     - Increase ``keyframe_distance_threshold`` to 1.5 or 2.0
   * - Too few keyframes (<500)
     - Decrease thresholds or check ``should_create_keyframe()`` logic
   * - Local map not visible
     - Check ``/slam/local_map`` topic rate, verify ``update_local_map()`` publishes
   * - Processing too slow (<5 Hz)
     - Increase ``voxel_size`` to 0.3, reduce ``max_keyframes`` to 15
   * - First scan fails
     - Verify initialization: first scan becomes keyframe_0
   * - TF errors in RViz
     - Check frame IDs: should be ``map → odom``

---------------------------------------------------------
📦 9. Deliverables
---------------------------------------------------------

Submit on **Canvas** as a single ZIP file: ``rwa2_groupX.zip`` (replace X with your group number)

**Required Contents:**

.. code-block:: text

   rwa2_groupX.zip
   ├── slam_frontend/           Complete Python package
   │   ├── config/
   │   ├── slam_frontend/
   │   │   ├── odometry_estimator_node.py    (your implementation)
   │   │   ├── local_map_manager_node.py     (your implementation)
   │   │   └── ... (utility files)
   │   ├── launch/
   │   ├── rviz/
   │   └── ...
   └── slam_interfaces/         Message package (for completeness)

**Your submission must include:**

1. **Implemented Nodes**
   
   - ``odometry_estimator_node.py`` — All TODO sections completed
   - ``local_map_manager_node.py`` — All TODO sections completed

2. **Launch Files**
   
   - ``rwa2_dev.launch.py`` — Must work with provided dev dataset
   - ``rwa2_full.launch.py`` — Must work with provided full dataset

3. **README.md** containing:
   
   .. code-block:: markdown

      # RWA2 - Keyframe-Based LiDAR Odometry
      
      ## Group Members
      - Student 1 Name (UID) - Implemented X
      - Student 2 Name (UID) - Implemented Y
      - Student 3 Name (UID) - Implemented Z
      - Student 4 Name (UID) - Implemented W
      
      ## Installation
      ```bash
      pip3 install -r requirements.txt
      colcon build --symlink-install
      source install/setup.bash
      ```
      
      ## Usage
      ```bash
      # Development dataset
      ros2 launch slam_frontend rwa2_dev.launch.py
      
      # Full dataset
      ros2 launch slam_frontend rwa2_full.launch.py
      ```
      
      ## Parameter Tuning
      We adjusted the following parameters:
      - keyframe_distance_threshold: X.X m (reason)
      - keyframe_rotation_threshold: XX° (reason)
      - max_keyframes: XX (reason)
      
      ## Results
      - Dev dataset: ~X keyframes created, Y m drift
      - Full dataset: ~X keyframes created, Y m final drift
      
      ## Challenges and Solutions
      [Describe any issues you encountered and how you solved them]

4. **Visualization File**
   
   - ``rwa2.rviz`` (RViz config) OR
   - ``rwa2.json`` (Foxglove config)
   - **Not both** — choose one

5. **Optional: Results Analysis** (for bonus points)
   
   - Screenshots showing trajectory on dev and full datasets
   - Plot of keyframe creation over time
   - Analysis of drift accumulation
   - Comparison of different parameter settings

**Submission Checklist:**

.. code-block:: text

   Before submitting:
   
   ✅ Code compiles without errors
   ✅ Launch files work on dev dataset
   ✅ Launch files work on full dataset (test at least once)
   ✅ README.md is complete
   ✅ All group member names included
   ✅ No modified utility files (icp_utils.py, etc.)
   ✅ No dataset files included in ZIP
   ✅ Clean submission (no build/, install/, log/ folders)

**Submission Deadline:**

- **Due:** November 18, 2025, 11:59 PM
- **Late Policy:** 10% penalty per day (max 3 days)
- **No submissions accepted after:** November 21, 2025, 11:59 PM

.. warning::

   **Important:**
   
   - Do **NOT** include dataset in your ZIP (provided separately)
   - Do **NOT** modify provided utility files
   - Do **NOT** include build artifacts (``build/``, ``install/``, ``log/``)
   - Do **NOT** include the entire workspace (only two packages)

---------------------------------------------------------
📈 10. Grading Rubric (30 pts)
---------------------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 50 10 40

   * - **Criterion**
     - **Points**
     - **Evaluation Details**
   * - **Keyframe Selection Logic (Node 1)**
     - 10
     - - Distance computation correct (3 pts)
       - Rotation computation correct (3 pts)
       - Threshold checking logic correct (2 pts)
       - Parameters loaded from YAML (1 pt)
       - Keyframe messages published correctly (1 pt)
   * - **Local Map Management (Node 2)**
     - 10
     - - Deque-based keyframe storage (3 pts)
       - Correct transformation to map frame (3 pts)
       - Proper merging and downsampling (2 pts)
       - Local map published at correct rate (2 pts)
   * - **Odometry Estimation (Node 1)**
     - 5
     - - ICP transformation chained correctly (2 pts)
       - Odometry messages valid and published (2 pts)
       - TF broadcast functional (1 pt)
   * - **Integration & Quality**
     - 5
     - - Nodes communicate correctly (1 pt)
       - Launch files execute without errors (1 pt)
       - Visualization clear and accurate (1 pt)
       - README complete with all sections (1 pt)
       - Code quality (comments, structure) (1 pt)

**Detailed Point Breakdown:**

**Keyframe Selection (10 pts):**

- **Distance computation (3 pts):**
  
  - 3 pts: Correct use of ``compute_distance()``, proper comparison with threshold
  - 2 pts: Minor issues (wrong comparison operator, off-by-one)
  - 1 pt: Attempted but incorrect implementation
  - 0 pts: Not implemented or completely wrong

- **Rotation computation (3 pts):**
  
  - 3 pts: Correct use of ``compute_rotation_angle()``, proper comparison
  - 2 pts: Minor issues (degrees vs radians confusion)
  - 1 pt: Attempted but incorrect
  - 0 pts: Not implemented

- **Threshold checking (2 pts):**
  
  - 2 pts: Correct OR logic (distance OR rotation exceeds threshold)
  - 1 pt: Uses AND instead of OR (still partially functional)
  - 0 pts: Logic missing or completely wrong

- **Parameters (1 pt):**
  
  - 1 pt: Loads parameters from YAML correctly
  - 0 pts: Hardcoded values

- **Publishing (1 pt):**
  
  - 1 pt: Keyframe message published with correct fields
  - 0 pts: Not publishing or incorrect message format

**Local Map Management (10 pts):**

- **Keyframe storage (3 pts):**
  
  - 3 pts: Deque used correctly, max size enforced
  - 2 pts: Storage works but doesn't use deque or size limit
  - 1 pt: Basic storage but has issues
  - 0 pts: Not implemented

- **Transformation (3 pts):**
  
  - 3 pts: All keyframes transformed to map frame correctly
  - 2 pts: Transformation attempted but minor errors
  - 1 pt: Partial implementation
  - 0 pts: Not implemented

- **Merging/downsampling (2 pts):**
  
  - 2 pts: Uses provided utilities correctly
  - 1 pt: Attempted but incomplete
  - 0 pts: Not implemented

- **Publishing (2 pts):**
  
  - 2 pts: Local map published at reasonable rate
  - 1 pt: Published but too slow or has issues
  - 0 pts: Not published

**Odometry Estimation (5 pts):**

- **Pose chaining (2 pts):**
  
  - 2 pts: Correct matrix multiplication order
  - 1 pt: Minor errors but general approach correct
  - 0 pts: Wrong or not implemented

- **Message publishing (2 pts):**
  
  - 2 pts: Odom and path published correctly
  - 1 pt: One of two works
  - 0 pts: Neither works

- **TF broadcast (1 pt):**
  
  - 1 pt: Map→odom transform broadcast
  - 0 pts: Not broadcast or incorrect

**Integration & Quality (5 pts):**

- Each criterion worth 1 point (pass/fail)

**Deductions:**

- **-5 pts:** Modified provided utility files
- **-3 pts:** Launch file doesn't work
- **-2 pts:** Missing or incomplete README
- **-2 pts:** Hardcoded paths or parameters
- **-2 pts:** Included dataset in submission
- **-10 pts:** Trajectory completely broken (>90% frames fail)

**Bonus Points (up to +3 pts):**

- **+1 pt:** Excellent code documentation and comments
- **+1 pt:** Results analysis with plots and screenshots
- **+1 pt:** Parameter tuning analysis with justification
- **+2 pts:** Implement advanced features (e.g., adaptive thresholds, map pruning)

.. note::

   **Trajectory Quality Expectations:**
   
   This assignment does NOT have loop closure or backend optimization.
   Some drift is expected and acceptable:
   
   - Dev dataset (1000 frames): <10m cumulative drift ✅
   - Full dataset (4541 frames): <100m cumulative drift ✅
   
   Focus is on implementation correctness, not perfect accuracy.

---------------------------------------------------------
💡 11. Tips for Success
---------------------------------------------------------

**Time Management:**

- **Day 1 (Nov 6):** Setup, explore starter code, understand utilities (2 hrs)
- **Days 2-4 (Nov 7-9):** Implement Node 1 - Odometry Estimator (6 hrs)
- **Days 5-7 (Nov 10-12):** Implement Node 2 - Local Map Manager (6 hrs)
- **Days 8-9 (Nov 13-14):** Integration testing, parameter tuning (4 hrs)
- **Days 10-11 (Nov 15-16):** Full dataset testing, documentation (3 hrs)
- **Day 12 (Nov 17):** Final review, README, submission prep (2 hrs)
- **Buffer:** November 18 for last-minute issues

**Debugging Strategies:**

1. **Use ROS 2 tools:**

   .. code-block:: bash

      # Check if topics are publishing
      ros2 topic list
      ros2 topic hz /odom
      ros2 topic echo /slam/new_keyframe --once

      # Monitor TF tree
      ros2 run tf2_tools view_frames
      evince frames.pdf

      # Check node status
      ros2 node list
      ros2 node info /odometry_estimator

2. **Add logging:**

   .. code-block:: python

      # Use different log levels
      self.get_logger().info(f'Keyframe created at {distance:.2f}m')
      self.get_logger().debug(f'Pose: {self.current_pose[:3, 3]}')
      self.get_logger().warn('Local map not ready')
      self.get_logger().error('ICP alignment failed!')

3. **Test incrementally:**
   
   - Get basic odometry working first (without keyframe logic)
   - Add keyframe selection next
   - Finally integrate local map manager
   - Test each component before moving to next

4. **Use visualization:**
   
   - Keep RViz open while developing
   - Check if trajectory path appears
   - Verify point clouds are colored correctly
   - Watch TF tree for errors

**Common Pitfalls:**

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - **Pitfall**
     - **Solution**
   * - Pose chaining order wrong
     - Always: ``T_new = T_old @ delta_T`` (not reversed)
   * - Keyframes created too frequently
     - Increase thresholds in ``params.yaml``
   * - Local map never updates
     - Check that ``update_local_map()`` is called in callback
   * - First scan crashes
     - Handle ``first_scan`` flag properly in initialization
   * - Running out of memory
     - Reduce ``max_keyframes`` or increase ``voxel_size``
   * - TF warnings in RViz
     - Ensure frame IDs match: ``map``, ``odom``, ``base_link``

**Parameter Tuning Guidelines:**

.. code-block:: yaml

   # For straight sections (highway):
   keyframe_distance_threshold: 1.5-2.0  # Fewer keyframes needed
   
   # For turns and complex areas:
   keyframe_distance_threshold: 0.5-1.0  # More keyframes needed
   
   # Rotation threshold:
   keyframe_rotation_threshold: 10-20    # 15° is good default
   
   # Local map size:
   max_keyframes: 15-25                  # 20 is good default
   
   # Downsampling:
   voxel_size: 0.1-0.3                   # 0.2 is good default

**Getting Help:**

1. **Office Hours:** See syllabus for schedule
2. **Piazza:** Post questions (don't share code!)
3. **Starter Code Issues:** Check provided utilities first
4. **ROS 2 Questions:** Consult official documentation
5. **Debugging:** Use logging and RViz extensively

---------------------------------------------------------
📚 12. Learning Outcomes
---------------------------------------------------------

By completing this assignment, you will:

- ✅ **Understand keyframe-based SLAM architectures**
  
  - Why keyframes are essential for scalability
  - How to design effective selection strategies
  - Trade-offs between accuracy and efficiency

- ✅ **Implement scan-to-map registration**
  
  - Difference from scan-to-scan matching
  - Benefits of local map approach
  - ICP alignment in practice

- ✅ **Master sliding window mapping**
  
  - Fixed-size buffer management
  - Coordinate frame transformations
  - Point cloud merging and downsampling

- ✅ **Gain ROS 2 development experience**
  
  - Multi-node system design
  - Custom message creation
  - Parameter management
  - Launch file configuration

- ✅ **Analyze SLAM system behavior**
  
  - Drift accumulation characteristics
  - Parameter impact on performance
  - Failure modes and limitations

**Connection to Course Material:**

This assignment implements concepts from:

- **LOAM** (Zhang & Singh, 2014): Keyframe selection and feature-based matching
- **LeGO-LOAM** (Shan & Englot, 2018): Lightweight map management
- **LIO-SAM** (Shan et al., 2020): Sliding window optimization

You're building the **frontend** of a modern LiDAR SLAM system!

---------------------------------------------------------
📖 13. References
---------------------------------------------------------

**Primary References:**

- Zhang, J. & Singh, S. (2014). *LOAM: Lidar Odometry and Mapping in Real-time.* Robotics: Science and Systems (RSS).
- Shan, T. & Englot, B. (2018). *LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain.* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
- Geiger, A., Lenz, P., & Urtasun, R. (2012). *Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite.* Conference on Computer Vision and Pattern Recognition (CVPR).

**Documentation:**

- Open3D: https://www.open3d.org/docs/
- ROS 2 Humble: https://docs.ros.org/en/humble/
- KITTI Dataset: http://www.cvlibs.net/datasets/kitti/
- TF2 Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html

**Additional Reading:**

- Besl, P. & McKay, N. (1992). *A Method for Registration of 3-D Shapes.* IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI).
- Rusinkiewicz, S. & Levoy, M. (2001). *Efficient Variants of the ICP Algorithm.* 3DIM.

---------------------------------------------------------

**Focus on the SLAM logic — we've handled the infrastructure for you. Good luck!** 🚀