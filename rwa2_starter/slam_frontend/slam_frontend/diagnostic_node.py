#!/usr/bin/env python3
"""
Diagnostic script to test transformation chain and monitor keyframe generation.

This node subscribes to keyframes published by the odometry estimator and provides
real-time feedback about the SLAM system's behavior, including keyframe frequency,
robot trajectory, and motion between keyframes. Useful for debugging parameter tuning
and verifying that transformations are being computed correctly.
"""

import rclpy
from rclpy.node import Node
from slam_interfaces.msg import Keyframe
from sensor_msgs.msg import PointCloud2
import numpy as np
from slam_frontend.transform_utils import pose_to_transform_matrix
from slam_frontend.pointcloud_utils import ros_to_open3d


class DiagnosticNode(Node):
    """
    Diagnostic node for monitoring SLAM frontend performance.
    
    Tracks keyframe generation patterns and robot motion to help identify
    issues with odometry estimation or keyframe selection parameters.
    Provides real-time logging of position, rotation, and inter-keyframe motion.
    """
    
    def __init__(self):
        """
        Initialize the diagnostic node.
        
        Sets up subscription to keyframe topic and initializes tracking variables
        for monitoring keyframe statistics and robot motion.
        """
        super().__init__('diagnostic_node')
        
        self.keyframe_count = 0
        self.last_pose = None
        
        # Subscribe to keyframes
        self.keyframe_sub = self.create_subscription(
            Keyframe,
            '/slam/new_keyframe',
            self.keyframe_callback,
            10
        )
        
        self.get_logger().info('Diagnostic node started - monitoring keyframes')
    
    def keyframe_callback(self, msg: Keyframe):
        """
        Process incoming keyframe and log diagnostic information.
        
        Extracts pose information from keyframe message, computes motion metrics,
        and logs detailed information about robot position, orientation, and
        movement since the last keyframe. Helps verify that keyframes are being
        generated at appropriate intervals.
        
        Args:
            msg: Keyframe message containing point cloud and pose
        
        Logged metrics:
            - Keyframe number
            - Position in map frame (x, y, z)
            - Yaw angle in degrees
            - Distance from previous keyframe
            - Delta position vector (if not first keyframe)
        """
        self.keyframe_count += 1
        
        # Get pose
        pose = pose_to_transform_matrix(msg.pose)
        
        # Extract position and rotation
        position = pose[:3, 3]
        
        # Calculate rotation angle (yaw)
        yaw = np.arctan2(pose[1, 0], pose[0, 0]) * 180 / np.pi
        
        # Calculate movement from last keyframe
        if self.last_pose is not None:
            delta_pos = position - self.last_pose[:3, 3]
            delta_dist = np.linalg.norm(delta_pos)
            
            self.get_logger().info(
                f'KF {self.keyframe_count}: '
                f'Pos=[{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}], '
                f'Yaw={yaw:.1f}°, '
                f'Delta={delta_dist:.2f}m, '
                f'DeltaXYZ=[{delta_pos[0]:.2f}, {delta_pos[1]:.2f}, {delta_pos[2]:.2f}]'
            )
        else:
            self.get_logger().info(
                f'KF {self.keyframe_count} (Initial): '
                f'Pos=[{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}], '
                f'Yaw={yaw:.1f}°'
            )
        
        self.last_pose = pose


def main():
    """
    Main entry point for the diagnostic node.
    
    Initializes ROS 2, creates the diagnostic node, and spins until shutdown.
    Handles graceful shutdown on keyboard interrupt.
    """
    rclpy.init()
    node = DiagnosticNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()