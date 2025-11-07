#!/usr/bin/env python3
"""
Local Map Manager Node - FINAL VERSION
With proper intensity/color handling
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from slam_interfaces.msg import Keyframe
import numpy as np
from collections import deque
import open3d as o3d

from slam_frontend.transform_utils import pose_to_transform_matrix
from slam_frontend.pointcloud_utils import ros_to_open3d, open3d_to_ros
from slam_frontend.icp_utils import downsample_cloud


class LocalMapManagerNode(Node):
    def __init__(self):
        super().__init__('local_map_manager_node')
        
        # Parameters
        self.declare_parameter('max_keyframes', 10)  # Slightly increased
        self.declare_parameter('voxel_size', 0.4)    # Balanced
        
        self.max_keyframes = self.get_parameter('max_keyframes').value
        self.voxel_size = self.get_parameter('voxel_size').value
        
        # Storage for keyframes
        self.keyframes = deque(maxlen=self.max_keyframes)
        self.update_counter = 0
        
        # Subscribers
        self.keyframe_sub = self.create_subscription(
            Keyframe,
            '/slam/new_keyframe',
            self.keyframe_callback,
            10
        )
        
        # Publishers
        self.local_map_pub = self.create_publisher(
            PointCloud2,
            '/slam/local_map',
            10
        )
        
        self.get_logger().info('Local Map Manager Node - FINAL VERSION')
        self.get_logger().info(f'Max keyframes: {self.max_keyframes}, '
                              f'Voxel size: {self.voxel_size}m')
    
    def keyframe_callback(self, msg: Keyframe):
        """
        TODO: Implement keyframe processing
        
        Handle incoming keyframe with intensity preservation.
        
        Steps:
        1. Convert cloud to Open3D format
        2. Get pose as transformation matrix
        3. Transform cloud to map frame (CRITICAL!)
        4. Downsample transformed cloud
        5. Add to sliding window buffer
        6. Update local map periodically
        
        Args:
            msg: Keyframe message containing cloud and pose
        """
        # TODO: Convert point cloud to Open3D format
        
        # TODO: Get pose as 4x4 transformation matrix
        
        # TODO: Transform cloud to map frame
        # CRITICAL: Must transform BEFORE storing to avoid "fan pattern"
        
        # TODO: Downsample the transformed cloud
        
        # TODO: Append to keyframes deque (old frames auto-removed)
        
        self.update_counter += 1
        
        # Update map every other keyframe for efficiency
        if self.update_counter % 2 == 0:
            self.update_local_map()

    def update_local_map(self):
        """
        TODO: Implement local map building
        
        Build local map with proper visualization.
        
        Steps:
        1. Check if keyframes buffer is empty
        2. Combine all keyframe clouds (already in map frame)
        3. Downsample if too large (>25000 points)
        4. Add grayscale colors based on height
        5. Convert to ROS message
        6. Publish local map
        """
        # TODO: Return if no keyframes
        
        # TODO: Combine all clouds using Open3D's + operator
        combined_cloud = o3d.geometry.PointCloud()
        
        # TODO: Downsample if too many points
        
        # Create grayscale colors based on height (provided)
        points = np.asarray(final_map.points)
        if len(points) > 0:
            z_values = points[:, 2]
            z_min, z_max = z_values.min(), z_values.max()
            if z_max - z_min > 0.1:
                z_normalized = (z_values - z_min) / (z_max - z_min)
            else:
                z_normalized = np.ones_like(z_values) * 0.7
            
            colors = np.column_stack([z_normalized, z_normalized, z_normalized])
            final_map.colors = o3d.utility.Vector3dVector(colors)
        
        # TODO: Convert to ROS message using open3d_to_ros()
        
        # TODO: Publish local map
        
        self.get_logger().debug(
            f'Published map: {len(final_map.points)} points',
            throttle_duration_sec=5.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = LocalMapManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()