==================================================================
Assignment 2
==================================================================

:Title: Building a Keyframe-Based LiDAR Odometry Pipeline

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L2C — SLAM for the Real World (Frontend)
:Assigned: November 7, 2025
:Due: November 19, 2025 (1.5 weeks)
:Total Points: 30 pts
:Language: Python (ROS 2, Open3D)


.. note::
  Provided resource:

  - Demonstration of the output: `location <https://drive.google.com/file/d/1iegtrsupj4XY0SIrvhmpgWKO9CifDmY9/view?usp=sharing>`_.
  - Two pre-recorded bag files: `location <https://drive.google.com/drive/folders/1ni7jXjCE8NncyLBbYpta3CpqVkbWhFDU?usp=sharing>`_.
  - Starter package: `location <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/rwa2_starter>`_.


---------------------------------------------------------
1. Objective
---------------------------------------------------------

Build a simplified **LiDAR odometry pipeline** that estimates vehicle trajectory from raw point cloud data. You will implement two core concepts of modern SLAM systems:

- **Keyframe Selection** – Decide which scans become keyframes to represent the map efficiently
- **Local Map Management** – Maintain a sliding window of recent keyframes as the registration reference

You will be provided with helper utilities and a complete starter package. Your task is to implement the **core SLAM logic** in two ROS 2 nodes.

.. note::
  

---------------------------------------------------------
2. Background
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
3. KITTI Dataset and ROS 2 Bag Files
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



1. **Development Dataset** (``kitti_00_dev``) — First 1,000 frames
   
   - Use this for 90% of your development and testing
   - Faster iteration, easier debugging
   - Covers ~1.6 km (highway and urban transition)
   - Duration: ~100 seconds

2. **Full Dataset** (``kitti_00_full``) — Complete 4,541 frames
   
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
4. Assignment Tasks
---------------------------------------------------------

You will implement **two ROS 2 nodes** in the provided ``slam_frontend`` package. The starter package includes templates with clear TODO sections showing exactly what you need to implement.

**Important:** You are provided with utility functions in:

- ``icp_utils.py`` - ICP alignment (``align_clouds``, ``downsample_cloud``)
- ``transform_utils.py`` - SE(3) transformations (``pose_to_transform_matrix``, ``transform_to_pose``, ``compute_distance``, ``compute_rotation_angle``)
- ``pointcloud_utils.py`` - ROS↔Open3D conversions (``ros_to_open3d``, ``open3d_to_ros``)

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

- `should_create_keyframe(T_current, T_last_keyframe)`: Decides whether the robot has moved far enough (in translation or rotation) from the last keyframe position to warrant creating a new keyframe. This prevents redundant keyframes when the robot is stationary or moving slowly.

  .. admonition:: should_create_keyframe(T_current, T_last_keyframe)
    :class: pseudocode

    .. line-block::
        INPUT: :math:`T_{\text{current}}` (4x4 matrix), :math:`T_{\text{last_keyframe}}` (4x4 matrix)
        OUTPUT: Boolean 
        
        1) :math:`d_{trans} \leftarrow \text{compute_distance}(T_{\text{current}}, T_{\text{last_keyframe}})`
        2) :math:`d_{rot} \leftarrow \text{compute_rotation_angle}(T_{\text{current}}, T_{\text{last_keyframe}})`
        3) **IF** :math:`d_{trans} > \theta_{dist}` **OR** :math:`d_{rot} > \theta_{angle}` **THEN** **RETURN** :math:`true`
        4) **RETURN** :math:`false`

- `pointcloud_callback(cloud_msg)`: Main odometry estimation loop that processes each incoming LiDAR scan. Performs scan-to-scan matching to estimate relative motion, updates the robot's global pose, and optionally refines the estimate using the local map. This function runs at 10 Hz with each new point cloud.

  
  .. admonition:: pointcloud_callback(cloud_msg)
    :class: pseudocode

    .. line-block::
        INPUT: cloud_msg (PointCloud2 message)
        
        1) :math:`P \leftarrow \text{ros_to_open3d}(\text{cloud_msg})`  // Convert ROS point cloud to Open3D format
        2) **IF** :math:`|P| < 100` **THEN** RETURN  // Skip if too few points
        3) :math:`P_{\text{ds}} \leftarrow \text{downsample}(P, v_{\text{size}})`  // Reduce points for faster processing
        
        4) **IF** first_scan **THEN**  // Initialize system on first scan
              :math:`T_{\text{current}} \leftarrow I_{4\times 4}`  // Start at origin
              :math:`T_{\text{last_kf}} \leftarrow I_{4\times 4}`  // First keyframe at origin
              :math:`P_{\text{last}} \leftarrow P_{\text{ds}}`  // Store for next iteration
              publish_keyframe(cloud_msg)  // First scan always becomes keyframe, publish to topic /slam/new_keyframe
              publish_odometry() // publish to topic /odom
              publish_path() // publish to topic /path
              first_scan ← False
              RETURN
        
        5) :math:`\Delta T \leftarrow \text{ICP}(P_{\text{ds}}, P_{\text{last}})`  // Estimate motion between consecutive scans
        
        6) **IF** :math:`\|\Delta T_{[0:3,3]}\| > \tau_{\text{max}}` **OR** :math:`\|\Delta T_{[0:3,3]}\| < \tau_{\text{min}}` **THEN**  // Check if ICP failed
              :math:`\Delta T \leftarrow \begin{bmatrix} 1 & 0 & 0 & 0.5 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}`  // Use default forward motion
        
        7) :math:`\Delta T_{[2,3]} \leftarrow \text{clamp}(\Delta T_{[2,3]}, -z_{\text{lim}}, z_{\text{lim}})`  // Limit vertical drift
        
        8) :math:`T_{\text{current}} \leftarrow T_{\text{current}} \cdot \Delta T`  // Chain transformation to update global pose
        
        9) **IF** scan_count mod 5 = 0 **AND** local_map exists **THEN**  // Periodic map-based refinement
              :math:`T_{\text{refined}} \leftarrow \text{ICP}(P_{\text{ds}}, \text{local_map}, T_{\text{current}})`  // Align to local map
              **IF** fitness > 0.5 **AND** :math:`\|T_{\text{refined}[0:3,3]} - T_{\text{current}[0:3,3]}\| < 2.0` **THEN**  // Validate refinement
                  :math:`T_{\text{refined}[2,3]} \leftarrow \text{clamp}(T_{\text{refined}[2,3]}, -2.0, 2.0)`  // Constrain Z
                  :math:`T_{\text{current}} \leftarrow T_{\text{refined}}`  // Accept refined pose
        
        10) :math:`P_{\text{last}} \leftarrow P_{\text{ds}}`  // Update scan buffer for next iteration
        11) publish_odometry(:math:`T_{\text{current}}`)  // Broadcast current pose estimate
        12) publish_path()  // Update trajectory visualization
        
        13) **IF** should_create_keyframe(:math:`T_{\text{current}}`, :math:`T_{\text{last_kf}}`) **THEN**  // Check motion thresholds
              publish_keyframe(cloud_msg)  // Add to keyframe set
              :math:`T_{\text{last_kf}} \leftarrow T_{\text{current}}`  // Update keyframe reference// Update keyframe reference

  .. note::

   **Why Convert to Open3D Format?**
   
   ROS PointCloud2 messages contain point clouds as **raw byte arrays** designed for network transport, not processing. The `data` field is essentially a serialized byte stream that needs to be decoded before use. Open3D provides:
   
   1. **Fast ICP Implementation**: Open3D's ICP is written in C++ with Python bindings, achieving ~50-100ms per scan compared to 2-5 seconds for pure Python implementations.
   
   2. **Essential Point Cloud Operations**:
      
      - Voxel downsampling: `voxel_down_sample()`
      - KD-tree spatial indexing for nearest neighbors
      - Direct matrix transformations: `cloud.transform(T)`
      - Simple merging: `cloud1 + cloud2`
   
   3. **Performance Critical for Real-time SLAM**: Without Open3D's optimized algorithms, the system would run at <1 Hz instead of the required >5 Hz.
   
   4. **Why Open3D over alternatives?**
      
      - **PCL**: More complex C++ API, harder Python integration
      - **NumPy only**: Would require implementing ICP from scratch (too slow)
      - **scikit-learn**: Lacks ICP and 3D point cloud optimizations
   
   The conversion extracts XYZ coordinates from the byte stream and creates an Open3D PointCloud object that can be directly used with all the optimization algorithms. Think of it like converting a compressed JPEG to a NumPy array before doing computer vision — you need the data in a computational format, not a transport format.
   

- `publish_keyframe(cloud_msg)`: Creates and publishes a new keyframe message containing the current point cloud and its associated pose. This keyframe will be received by the local map manager to update the sliding window map. Also updates the last keyframe reference for future distance/rotation comparisons.

  .. admonition:: publish_keyframe(cloud_msg)
    :class: pseudocode

    .. line-block::
      INPUT: cloud_msg (PointCloud2), :math:`T_{\text{current}}` (from class member)
      
      1) keyframe.cloud ← cloud_msg  // Attach original point cloud
      2) keyframe.pose ← transform_to_pose(:math:`T_{\text{current}}`)  // Convert 4×4 matrix to ROS Pose
      3) publish(keyframe_topic, keyframe)  // Send to /slam/new_keyframe
      4) :math:`T_{\text{last\_kf}} \leftarrow T_{\text{current}}`  // Update keyframe reference
      5) keyframe_count ← keyframe_count + 1  // Increment counter


Node 2 — local_map_manager_node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose:** Maintain a sliding window of keyframes and publish the aggregated local map.

**Subscriptions:**

- ``/slam/new_keyframe`` (slam_interfaces/Keyframe) — Keyframe notifications

**Publications:**

- ``/slam/local_map`` (sensor_msgs/PointCloud2) — Merged and downsampled local map

**What You Implement:**

- `keyframe_callback(keyframe_msg)`: Processes incoming keyframes from the odometry estimator and adds them to the sliding window buffer. Transforms each keyframe's point cloud from sensor frame to map frame using the provided pose, ensuring all keyframes are in a common coordinate system before merging. This is critical to avoid the "fan pattern" artifact that occurs when clouds are merged without proper transformation.

.. admonition:: keyframe_callback(keyframe_msg)
   :class: pseudocode

   .. line-block::

      INPUT: keyframe_msg (Keyframe message with cloud and pose)
      
      1) :math:`P \leftarrow \text{ros_to_open3d}(\text{keyframe_msg.cloud})`
      2) :math:`T_{\text{pose}} \leftarrow \text{pose_to_transform_matrix}(\text{keyframe_msg.pose})`
      3) :math:`P \leftarrow \text{transform}(P, T_{\text{pose}})`  // Transform to map frame
      4) :math:`P_{\text{ds}} \leftarrow \text{downsample}(P, v_{\text{size}})`
      5) keyframes_buffer.append(:math:`P_{\text{ds}}`)  // Sliding window auto-removes oldest
      6) update_local_map()

- `update_local_map()`: Builds and publishes the aggregated local map by combining all keyframes in the sliding window buffer. Since keyframes are already transformed to the map frame, they can be directly merged. The combined map is downsampled if too large to maintain real-time performance, and grayscale colors are added based on height for better visualization in RViz.

  .. admonition:: update_local_map()
    :class: pseudocode

    .. line-block::

        INPUT: keyframes_buffer (deque of point clouds in map frame)
      
        1) **IF** keyframes_buffer is empty **THEN** RETURN
        2) :math:`P_{\text{combined}} \leftarrow \emptyset`
        3) **FOR** each :math:`P_i` **IN** keyframes_buffer **DO**
              :math:`P_{\text{combined}} \leftarrow P_{\text{combined}} \cup P_i`
        4) **IF** :math:`|P_{\text{combined}}| > N_{\text{max}}` **THEN**
              :math:`P_{\text{combined}} \leftarrow \text{downsample}(P_{\text{combined}}, v_{\text{size}} \times 1.5)`
        5) :math:`colors \leftarrow \text{height_to_grayscale}(P_{\text{combined}})`
        6) :math:`P_{\text{combined}}.\text{colors} \leftarrow colors`
        7) local_map_msg ← open3d_to_ros(:math:`P_{\text{combined}}`, frame="map")
        8) publish(local_map_topic, local_map_msg)


**Critical Note on Coordinate Frames:**

.. warning::

   Always transform clouds to map frame **before** storing in the keyframe buffer!
   
  .. admonition:: Coordinate frame handling
    :class: pseudocode

    .. line-block::

      CORRECT:
        - transform(cloud, pose_matrix)  # to map frame
        - keyframes.append(cloud)

      WRONG (creates fan pattern):
        - keyframes.append((cloud, pose))  # do NOT store separately


---------------------------------------------------------
5. Performance Optimization Tips
---------------------------------------------------------

**CRITICAL: Fast Point Cloud Conversion**

The default ROS to Open3D conversion can be extremely slow (300ms per frame). If you experience performance issues, replace the function in ``pointcloud_utils.py`` with this optimized version:

.. admonition:: Fast ROS→Open3D conversion (vectorized)
   :class: pseudocode

   .. line-block::

      INPUT: ros_cloud (PointCloud2)
      OUTPUT: cloud (Open3D PointCloud)
      
      1) Define dtype for fields: x, y, z, intensity (float32 each)
      2) points_structured ← frombuffer(ros_cloud.data, dtype)  # zero-copy
      3) points ← column_stack([points_structured['x'], 'y', 'z'])
      4) valid_mask ← rows without NaN/Inf
      5) points ← points[valid_mask]
      6) cloud ← new PointCloud
      7) cloud.points ← Vector3dVector(points)
      8) RETURN cloud

This optimization alone can provide a **7x speedup** (from 300ms to 40ms per frame)!

**ICP Parameter Tuning for Speed:**

.. admonition:: ICP parameter presets
   :class: pseudocode

   .. line-block::

      Preset A (scan-to-scan, fast):
        • max_correspondence_distance ← 1.0
        • max_iteration ← 10
      
      Preset B (scan-to-map, precise):
        • max_correspondence_distance ← 0.5
        • max_iteration ← 5
      
      Usage: choose preset based on context; provide initial guess for scan-to-map when available.

---------------------------------------------------------
6. Getting Started
---------------------------------------------------------



Step 1: Implement Node 1
~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``odometry_estimator_node.py``
2. Find TODO sections
3. Implement ``should_create_keyframe()`` first
4. Implement ``publish_keyframe()`` next
5. Implement main ``pointcloud_callback()`` last
6. Test with dev dataset

Step 2: Implement Node 2
~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``local_map_manager_node.py``
2. Find TODO sections
3. Implement ``keyframe_callback()``
4. Implement ``update_local_map()``
5. Test integration with Node 1

Step 3: Tune Parameters
~~~~~~~~~~~~~~~~~~~~~~~~

Edit ``config/params.yaml`` and experiment:

.. code-block:: yaml

   # Start with these defaults
   odometry_estimator_node:
     ros__parameters:
       keyframe_distance_threshold: 1.0   # meters
       keyframe_rotation_threshold: 15.0  # degrees
       voxel_size: 0.2                   # meters
   
   local_map_manager_node:
     ros__parameters:
       max_keyframes: 20
       voxel_size: 0.2

Step 4: Test with Dev/Full Dataset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  # Start with dev dataset
   ros2 launch slam_frontend rwa2_dev.launch.py
   # Once dev dataset works, test with full
   ros2 launch slam_frontend rwa2_full.launch.py

---------------------------------------------------------
7. Provided Package Structure
---------------------------------------------------------
.. code-block:: text

   rwa2_starter/
   ├── slam_frontend/
   │   ├── package.xml
   │   ├── setup.py
   │   ├── setup.cfg
   │   ├── config/
   │   │   └── params.yaml              # <-- Edit parameters here
   │   ├── launch/
   │   │   ├── rwa2_dev.launch.py       # Launch for dev dataset
   │   │   └── rwa2_full.launch.py      # Launch for full dataset
   │   ├── rviz/
   │   │   └── rwa2.rviz                # RViz configuration
   │   ├── slam_frontend/
   │   │   ├── __init__.py
   │   │   ├── odometry_estimator_node.py    # <-- TODO: Implement
   │   │   ├── local_map_manager_node.py     # <-- TODO: Implement
   │   │   ├── icp_utils.py             # Provided ICP utilities
   │   │   ├── transform_utils.py       # Provided SE(3) utilities
   │   │   ├── pointcloud_utils.py      # Provided conversion utilities
   │   │   ├── fast_pointcloud_utils.py # Optimized conversion (optional)
   │   │   ├── diagnostic_node.py       # Diagnostic tools (provided)
   │   │   └── verification.py          # Verification script (provided)
   │   └── resource/
   │       └── slam_frontend
   ├── slam_interfaces/
   │   ├── CMakeLists.txt
   │   ├── package.xml
   │   └── msg/
   │       └── Keyframe.msg             # Custom message definition
   └── data/
       ├── kitti_00_dev/
       │   ├── kitti_00_dev_0.db3       # Development bag (1000 frames)
       │   └── metadata.yaml
       └── kitti_00_full/
           ├── kitti_00_full_0.db3      # Full dataset (4541 frames)
           └── metadata.yaml

**Files you modify:**

- ✏️ ``src/odometry_estimator_node.py`` — Implement 3 functions
- ✏️ ``src/local_map_manager_node.py`` — Implement 2 functions
- ✏️ ``config/params.yaml`` — Tune parameters

**Files you should NOT modify:**

- ❌ ``slam_frontend/icp_utils.py`` — Provided utilities
- ❌ ``slam_frontend/transform_utils.py`` — Provided utilities
- ❌ ``slam_frontend/pointcloud_utils.py`` — Provided utilities (unless optimizing)
- ❌ Launch files (unless adding features)

---------------------------------------------------------
8. Testing and Evaluation
---------------------------------------------------------

Verify Your Implementation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the provided verification script (`verification.py`):

.. code-block:: bash

   # Run verification node
   python3 verification.py
   
   # Expected output:
   ==================================================
   VERIFICATION SUMMARY (t=60.0s)
   ==================================================
   Keyframes generated: 150
   Average processing rate: 15.2 Hz
   Total distance traveled: 357.7m
   Z-axis variation: 2.1m
   Max jump between poses: 1.2m
   ==================================================
   PERFORMANCE CHECKS:
   ==================================================
   ✅ Keyframe generation rate OK
   ✅ Processing speed OK (15.2 Hz)
   ✅ Z-drift within limits (2.1m)
   ✅ Path continuity OK
   ✅ Vehicle is moving
   
   🎉 ALL CHECKS PASSING! 🎉

Evaluation Criteria
~~~~~~~~~~~~~~~~~~~

Your implementation will be evaluated on:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - **Metric**
     - **Requirement**
   * - **Processing Speed**
     - ≥ 5 Hz (target: 10-20 Hz)
   * - **Keyframe Count**
     - 200-400 for full sequence
   * - **Z-axis Drift**
     - < 4 meters total
   * - **Path Continuity**
     - Max jump < 5 meters
   * - **Memory Usage**
     - Stable (no memory leaks)

Expected Results
~~~~~~~~~~~~~~~~

**Development Dataset (1000 frames):**

- Processing time: ~100 seconds (at 10 Hz)
- Keyframes created: 40-80
- Final position error: < 10m
- Z-drift: < 1m
- Average processing rate: > 8 Hz

**Full Dataset (4541 frames):**

- Processing time: ~450 seconds (at 10 Hz)
- Keyframes created: 200-400
- Final position error: < 100m (no loop closure)
- Z-drift: < 4m
- Average processing rate: > 5 Hz



---------------------------------------------------------
9. Submission Requirements
---------------------------------------------------------

What to Submit
~~~~~~~~~~~~~~

Submit a **single ZIP file** named ``groupX_rwa2.zip`` containing:

.. code-block:: text

   groupX_rwa2/
   ├── slam_frontend/               # Your complete package
   │   ├── src/
   │   │   ├── odometry_estimator_node.py    # Your implementation
   │   │   ├── local_map_manager_node.py     # Your implementation
   │   │   └── ...
   │   ├── config/params.yaml       # Your tuned parameters
   │   └── ...
   ├── README.md                     # Required documentation
   ├── results/
   │   ├── trajectory_dev.png        # RViz screenshot (dev dataset)
   │   ├── trajectory_full.png       # RViz screenshot (full dataset)
   │   └── verification_output.txt   # Copy of verification results
   └── report.pdf                    # Optional: Analysis report

README Requirements
~~~~~~~~~~~~~~~~~~~

Your ``README.md`` must include:

1. **Build Instructions** 
   
   Example:

   .. code-block:: markdown
   
      ## Build Instructions
      ```bash
      cd ~/ros2_ws
      colcon build --packages-select slam_frontend slam_interfaces
      source install/setup.bash
      ```

2. **Run Instructions:**
   
   .. code-block:: markdown
   
      ## How to Run
      ```bash
      # Terminal 1: Launch nodes
      ros2 launch slam_frontend rwa2_dev.launch.py
      
      # Terminal 2: Visualization
      rviz2 -d src/slam_frontend/rviz/rwa2.rviz
      ```

3. **Parameter Choices:**
   
   - Final values chosen
   - Brief justification
   - Any environment-specific tuning

4. **Results Summary:**
   
   - Processing rate achieved
   - Number of keyframes
   - Observed drift
   - Any issues encountered

5. **Implementation Notes:**
   
   - Key design decisions
   - Challenges faced
   - What you learned

Submission Checklist
~~~~~~~~~~~~~~~~~~~~

Before submitting, ensure:

- ☐ Both nodes implemented and working
- ☐ Parameters tuned in ``params.yaml``
- ☐ README.md complete with all sections
- ☐ RViz screenshots included
- ☐ Verification output saved
- ☐ Code is well-commented
- ☐ No modified utility files (unless documented)
- ☐ Package builds without errors
- ☐ Tested with both dev and full datasets
- ☐ ZIP file follows naming convention

---------------------------------------------------------
10. Grading Rubric
---------------------------------------------------------

**Total: 30 points**

**Keyframe Selection (5 pts):**

- **Logic implementation (3 pts):**
  
  - 3 pts: Both distance and rotation thresholds work correctly
  - 2 pts: One threshold works, other has issues
  - 1 pt: Attempted but buggy
  - 0 pts: Not implemented

- **Publishing (2 pts):**
  
  - 2 pts: Keyframe message published correctly with pose
  - 1 pt: Published but missing fields or wrong format
  - 0 pts: Not published

**Local Map Management (5 pts):**

- **Sliding window (2 pts):**
  
  - 2 pts: Fixed-size buffer correctly maintained
  - 1 pt: Buffer works but size management issues
  - 0 pts: No sliding window

- **Point cloud merging (2 pts):**
  
  - 2 pts: Clouds transformed and merged correctly
  - 1 pt: Merged but coordinate frame issues
  - 0 pts: Not merged or severe errors

- **Map publishing (1 pt):**
  
  - 1 pt: Local map published and downsampled
  - 0 pts: Not published

**Scan Registration (5 pts):**

- **ICP usage (3 pts):**
  
  - 3 pts: ICP called correctly with proper parameters
  - 2 pts: ICP works but suboptimal parameters
  - 1 pt: Attempted but errors
  - 0 pts: Not implemented

- **Scan-to-map matching (2 pts):**
  
  - 2 pts: Uses local map when available
  - 1 pt: Only scan-to-scan matching
  - 0 pts: No matching

**Trajectory Quality (5 pts):**

- Evaluated on **dev dataset** (1000 frames):
  
  - 5 pts: Smooth trajectory, < 10m drift
  - 4 pts: Minor jumps, < 20m drift
  - 3 pts: Some issues, < 30m drift
  - 2 pts: Major issues but recognizable path
  - 1 pt: Trajectory visible but very poor
  - 0 pts: No trajectory or completely wrong

**Processing Performance (5 pts):**

- **Speed requirement:**
  
  - 5 pts: ≥ 10 Hz
  - 4 pts: 8-10 Hz
  - 3 pts: 5-8 Hz
  - 2 pts: 3-5 Hz
  - 1 pt: 1-3 Hz
  - 0 pts: < 1 Hz

**Integration & Documentation (5 pts):**

- **System integration (2 pts):**
  
  - 2 pts: Both nodes work together seamlessly
  - 1 pt: Some integration issues
  - 0 pts: Nodes don't work together

- **README completeness (2 pts):**
  
  - 2 pts: All sections present and clear
  - 1 pt: Missing some sections
  - 0 pts: Minimal or missing

- **Code quality (1 pt):**
  
  - 1 pt: Clean, commented, follows conventions
  - 0 pts: Messy or undocumented


**Bonus Points (up to +3 pts):**

- **+1 pt:** Excellent documentation
- **+1 pt:** Results analysis with plots
- **+1 pt:** Creative improvements


---------------------------------------------------------
11. Learning Outcomes
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
13. References
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
