#!/usr/bin/env python3
"""
Odometry Estimator Node - FINAL OPTIMIZED VERSION
Achieves >10 Hz processing with good accuracy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from slam_interfaces.msg import Keyframe
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation
import copy
import open3d as o3d

from slam_frontend.transform_utils import (
    transform_to_pose,
    pose_to_transform_matrix,
    compute_distance,
    compute_rotation_angle,
)
from slam_frontend.icp_utils import downsample_cloud


def ros_to_open3d(ros_cloud: PointCloud2) -> o3d.geometry.PointCloud:
    """ULTRA FAST ROS to Open3D conversion."""
    dtype_list = [
        ('x', np.float32),
        ('y', np.float32), 
        ('z', np.float32),
        ('intensity', np.float32)
    ]
    
    points_structured = np.frombuffer(ros_cloud.data, dtype=dtype_list)
    points = np.column_stack([
        points_structured['x'],
        points_structured['y'],
        points_structured['z']
    ])
    
    valid = ~np.any(np.isnan(points) | np.isinf(points), axis=1)
    points = points[valid]
    
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


class OdometryEstimatorNode(Node):
    def __init__(self):
        super().__init__('odometry_estimator_node')
        
        # Parameters
        self.declare_parameter('keyframe_distance_threshold', 2.0)
        self.declare_parameter('keyframe_rotation_threshold', 20.0)
        self.declare_parameter('voxel_size', 0.3)
        
        self.keyframe_distance_threshold = self.get_parameter(
            'keyframe_distance_threshold').value
        self.keyframe_rotation_threshold = self.get_parameter(
            'keyframe_rotation_threshold').value
        self.voxel_size = self.get_parameter('voxel_size').value
        
        # State
        self.current_pose = np.eye(4)
        self.last_keyframe_pose = np.eye(4)
        self.first_scan = True
        self.path = Path()
        self.path.header.frame_id = "map"
        self.last_scan = None
        self.local_map = None
        self.scan_count = 0
        self.keyframe_count = 0
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/kitti/velo/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        self.local_map_sub = self.create_subscription(
            PointCloud2,
            '/slam/local_map',
            self.local_map_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.keyframe_pub = self.create_publisher(
            Keyframe,
            '/slam/new_keyframe',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Odometry Estimator - FINAL OPTIMIZED')
        self.get_logger().info(f'Params: dist={self.keyframe_distance_threshold}m, '
                              f'rot={self.keyframe_rotation_threshold}°, '
                              f'voxel={self.voxel_size}m')
    
    def local_map_callback(self, msg: PointCloud2):
        """Receive local map."""
        self.local_map = ros_to_open3d(msg)
    
    def should_create_keyframe(self, current_pose, last_keyframe_pose):
        """
        TODO: Implement keyframe selection logic
        
        Determine if a new keyframe should be created based on motion thresholds.
        
        Evaluates whether the robot has moved sufficiently (in translation or rotation)
        from the last keyframe position to warrant creating a new keyframe. This prevents
        redundant keyframes when the robot is stationary or moving slowly, keeping the
        map sparse and efficient.
        
        Decision criteria:
        - Translation: Distance between current and last keyframe positions
        - Rotation: Angular difference between current and last keyframe orientations
        - Returns True if EITHER threshold is exceeded (OR logic)
        
        Args:
            current_pose: Current robot pose as 4x4 transformation matrix
            last_keyframe_pose: Pose of the last keyframe as 4x4 transformation matrix
        
        Returns:
            bool: True if new keyframe should be created, False otherwise
        
        Parameters used:
            self.keyframe_distance_threshold: Min translation distance (meters)
            self.keyframe_rotation_threshold: Min rotation angle (degrees)
        
        Hints:
            - Use compute_distance() from transform_utils
            - Use compute_rotation_angle() from transform_utils
            - Check if EITHER threshold is exceeded (OR logic)
        """
        # TODO: Compute translation distance between poses
        
        # TODO: Compute rotation angle between poses
        
        # TODO: Return True if either threshold is exceeded
        
        return False  # Placeholder - always returns False
    
    def pointcloud_callback(self, msg: PointCloud2):
        """
        TODO: Implement main odometry estimation loop
        
        Main odometry estimation loop for processing incoming LiDAR scans.
        
        Performs scan-to-scan ICP matching as the primary odometry method,
        with optional scan-to-map refinement every 5 scans for drift correction.
        Includes validation and fallback mechanisms for robust operation.
        
        Processing pipeline:
        1. Convert ROS PointCloud2 to Open3D format (fast numpy method)
        2. Check minimum point count (skip if <100 points)
        3. Downsample using voxel grid for efficiency
        4. Handle first scan initialization (create origin keyframe)
        5. Perform scan-to-scan ICP alignment
        6. Validate ICP result (use default motion if failed)
        7. Constrain vertical movement to prevent Z-drift
        8. Update global pose by chaining transformation
        9. Optional: Refine with local map every 5 scans
        10. Update scan buffer for next iteration
        11. Publish odometry, path, and TF
        12. Check keyframe criteria and create if needed
        
        Args:
            msg: PointCloud2 message from /kitti/velo/pointcloud topic
        
        Performance:
            Target: >10 Hz processing rate
            Optimizations: Fast numpy conversion, reduced ICP iterations,
                        periodic map refinement, aggressive downsampling
        """
        self.scan_count += 1
        
        # TODO: Step 1 - Convert ROS message to Open3D (use provided function)
        
        # TODO: Step 2 - Check if enough points (return if < 100)
        
        # TODO: Step 3 - Downsample cloud using downsample_cloud()
        
        # TODO: Step 4 - Handle first scan initialization
        if self.first_scan:
            # TODO: Initialize poses to identity
            # TODO: Store current scan as last_scan
            # TODO: Set first_scan to False
            # TODO: Publish first keyframe
            # TODO: Publish odometry and path
            return
        
        # TODO: Step 5 - Perform scan-to-scan ICP
        # Use o3d.pipelines.registration.registration_icp with:
        # - max_correspondence_distance = 1.0
        # - init = np.eye(4)
        # - max_iteration = 10
        
        # TODO: Step 6 - Validate ICP result
        # Check translation norm: if > 5.0 or < 0.01, use default motion
        
        # TODO: Step 7 - Constrain Z movement
        # Clamp T_relative[2,3] to [-0.2, 0.2]
        
        # TODO: Step 8 - Update global pose
        # CRITICAL: Use correct order: self.current_pose @ T_relative
        
        # TODO: Step 9 - Optional map refinement (every 5 scans)
        if self.scan_count % 5 == 0 and self.local_map is not None:
            # TODO: Check if map has > 1000 points
            # TODO: Run ICP with map using current_pose as initial
            # TODO: Validate refinement (fitness > 0.5, delta < 2.0)
            pass
        
        # TODO: Step 10 - Update last scan for next iteration
        
        # TODO: Step 11 - Publish outputs
        
        # TODO: Step 12 - Check if keyframe needed
        
        # Status logging (provided)
        if self.scan_count % 50 == 0:
            pos = self.current_pose[:3, 3]
            self.get_logger().info(
                f'Scan {self.scan_count}: pos=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], '
                f'{self.keyframe_count} keyframes'
            )
    
    def publish_keyframe(self, cloud_msg):
        """Publish keyframe."""
        keyframe = Keyframe()
        keyframe.cloud = cloud_msg
        keyframe.pose = transform_to_pose(self.current_pose)
        self.keyframe_pub.publish(keyframe)
        self.last_keyframe_pose = self.current_pose.copy()
        self.keyframe_count += 1
    
    def publish_odometry(self):
        """Publish odometry and TF."""
        now = self.get_clock().now().to_msg()
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose = transform_to_pose(self.current_pose)
        self.odom_pub.publish(odom)
        
        R = self.current_pose[:3, :3]
        q = Rotation.from_matrix(R).as_quat()
        
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.current_pose[0, 3])
        t.transform.translation.y = float(self.current_pose[1, 3])
        t.transform.translation.z = float(self.current_pose[2, 3])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_path(self):
        """Publish path."""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = transform_to_pose(self.current_pose)
        
        self.path.poses.append(pose_stamped)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()