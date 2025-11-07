"""
Transformation Utilities

This module provides transformation matrix operations for SE(3) poses.
DO NOT MODIFY THIS FILE - It is a provided utility.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Transform
from typing import Tuple


def pose_to_transform_matrix(pose: Pose) -> np.ndarray:
    """
    Convert ROS Pose message to 4x4 homogeneous transformation matrix.
    
    Extracts position and orientation from a ROS Pose message and constructs
    the corresponding SE(3) transformation matrix used for rigid body transforms.
    
    Args:
        pose: geometry_msgs/Pose message with position and orientation (quaternion)
        
    Returns:
        4x4 transformation matrix in SE(3) with structure:
        [[R11, R12, R13, tx],
         [R21, R22, R23, ty],
         [R31, R32, R33, tz],
         [0,   0,   0,   1]]
    
    Note:
        - Quaternion order in ROS: [x, y, z, w]
        - Uses scipy for robust quaternion to rotation conversion
    """
    # Extract position
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    
    # Extract quaternion and convert to rotation matrix
    q = [pose.orientation.x, pose.orientation.y, 
         pose.orientation.z, pose.orientation.w]
    R = Rotation.from_quat(q).as_matrix()
    
    # Construct 4x4 matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T


def transform_to_pose(transform_matrix: np.ndarray) -> Pose:
    """
    Convert 4x4 transformation matrix to ROS Pose message.
    
    Extracts rotation and translation from an SE(3) matrix and creates
    a ROS Pose message suitable for publishing or storing in messages.
    
    Args:
        transform_matrix: 4x4 numpy array representing SE(3) transformation
        
    Returns:
        geometry_msgs/Pose message with position and quaternion orientation
    
    Note:
        - Output quaternion order: [x, y, z, w] (ROS convention)
        - Assumes input matrix is valid SE(3) (orthogonal rotation, bottom row [0,0,0,1])
    """
    pose = Pose()
    
    # Extract translation
    pose.position.x = float(transform_matrix[0, 3])
    pose.position.y = float(transform_matrix[1, 3])
    pose.position.z = float(transform_matrix[2, 3])
    
    # Extract rotation and convert to quaternion
    R = transform_matrix[:3, :3]
    q = Rotation.from_matrix(R).as_quat()  # Returns [x, y, z, w]
    
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    
    return pose


def compute_distance(T1: np.ndarray, T2: np.ndarray) -> float:
    """
    Compute Euclidean distance between translation components of two poses.
    
    Calculates the straight-line distance between the positions encoded in
    two transformation matrices. Used for keyframe selection based on motion.
    
    Args:
        T1: First 4x4 transformation matrix
        T2: Second 4x4 transformation matrix
        
    Returns:
        Distance in meters between the two positions
    
    Example:
        >>> dist = compute_distance(current_pose, last_keyframe_pose)
        >>> if dist > 2.0:  # Create keyframe every 2 meters
        >>>     create_keyframe()
    """
    translation_diff = T1[:3, 3] - T2[:3, 3]
    distance = np.linalg.norm(translation_diff)
    return float(distance)


def compute_rotation_angle(T1: np.ndarray, T2: np.ndarray) -> float:
    """
    Compute rotation angle difference between two poses.
    
    Calculates the minimum rotation angle needed to align the orientations
    of two poses. Used for keyframe selection based on rotation.
    
    Args:
        T1: First 4x4 transformation matrix
        T2: Second 4x4 transformation matrix
        
    Returns:
        Angle difference in degrees (0-180°)
    
    Mathematical basis:
        For relative rotation R_rel = R1^T @ R2:
        angle = arccos((trace(R_rel) - 1) / 2)
    
    Example:
        >>> angle = compute_rotation_angle(current_pose, last_keyframe_pose)
        >>> if angle > 15.0:  # Create keyframe every 15 degrees
        >>>     create_keyframe()
    """
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]
    
    # Compute relative rotation
    R_rel = R1.T @ R2
    
    # Extract angle from rotation matrix
    # angle = arccos((trace(R) - 1) / 2)
    trace = np.trace(R_rel)
    # Clamp to avoid numerical errors
    cos_angle = np.clip((trace - 1) / 2, -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)
    
    return float(angle_deg)


def inverse_transform(T: np.ndarray) -> np.ndarray:
    """
    Compute inverse of an SE(3) transformation matrix efficiently.
    
    Uses the special structure of SE(3) matrices for efficient inversion:
    T^(-1) = [[R^T, -R^T * t],
              [0,    1]]
    
    Args:
        T: 4x4 transformation matrix to invert
        
    Returns:
        Inverse transformation matrix (4x4)
    
    Note:
        - More efficient than np.linalg.inv for SE(3) matrices
        - Assumes input is valid SE(3) (proper orthogonal rotation)
    
    Example:
        >>> T_world_to_sensor = inverse_transform(T_sensor_to_world)
    """
    T_inv = np.eye(4)
    R = T[:3, :3]
    t = T[:3, 3]
    
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    
    return T_inv


def compose_transforms(T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
    """
    Compose two transformations by matrix multiplication.
    
    Applies transformation T2 followed by T1, equivalent to T1 @ T2.
    This follows the convention where transformations are applied right to left.
    
    Args:
        T1: First transformation (applied second)
        T2: Second transformation (applied first)
        
    Returns:
        Composed transformation matrix (4x4)
    
    Example:
        >>> # Transform from A to B, then B to C
        >>> T_A_to_B = get_transform_A_to_B()
        >>> T_B_to_C = get_transform_B_to_C()
        >>> T_A_to_C = compose_transforms(T_B_to_C, T_A_to_B)
    """
    return T1 @ T2


def transform_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    """
    Transform a 3D point using a transformation matrix.
    
    Applies rigid body transformation to a single point using homogeneous
    coordinates. Useful for transforming landmarks or specific points.
    
    Args:
        T: 4x4 transformation matrix
        point: 3D point as numpy array [x, y, z]
        
    Returns:
        Transformed 3D point [x', y', z']
    
    Example:
        >>> sensor_point = np.array([10, 0, 0])  # 10m ahead in sensor frame
        >>> world_point = transform_point(T_sensor_to_world, sensor_point)
    """
    point_h = np.append(point, 1.0)  # Homogeneous coordinates
    transformed = T @ point_h
    return transformed[:3]


def matrix_to_xyz_rpy(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert transformation matrix to position and Euler angles.
    
    Decomposes an SE(3) matrix into position and roll-pitch-yaw angles.
    Useful for logging, visualization, or when Euler angles are required.
    
    Args:
        T: 4x4 transformation matrix
        
    Returns:
        xyz: Position vector [x, y, z] in meters
        rpy: Roll-pitch-yaw angles [roll, pitch, yaw] in radians
    
    Note:
        - Uses 'xyz' intrinsic rotation order (roll about X, pitch about Y, yaw about Z)
        - Subject to gimbal lock at ±90° pitch
        - Range: roll, yaw ∈ [-π, π], pitch ∈ [-π/2, π/2]
    
    Example:
        >>> pos, angles = matrix_to_xyz_rpy(robot_pose)
        >>> print(f"Position: {pos}, Yaw: {np.degrees(angles[2]):.1f}°")
    """
    xyz = T[:3, 3]
    R = T[:3, :3]
    rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
    return xyz, rpy


def xyz_rpy_to_matrix(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """
    Convert position and Euler angles to transformation matrix.
    
    Constructs an SE(3) matrix from position and roll-pitch-yaw angles.
    Inverse operation of matrix_to_xyz_rpy.
    
    Args:
        xyz: Position [x, y, z] in meters
        rpy: Roll-pitch-yaw angles [roll, pitch, yaw] in radians
        
    Returns:
        4x4 transformation matrix in SE(3)
    
    Note:
        - Uses 'xyz' intrinsic rotation order
        - Suitable for representing robot poses with Euler angles
    
    Example:
        >>> pose = xyz_rpy_to_matrix([10, 5, 0], [0, 0, np.pi/2])  # 90° yaw
    """
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rpy, degrees=False).as_matrix()
    T[:3, 3] = xyz
    return T


def interpolate_transforms(T1: np.ndarray, T2: np.ndarray, alpha: float) -> np.ndarray:
    """
    Smoothly interpolate between two transformation matrices.
    
    Performs linear interpolation for translation and spherical linear
    interpolation (SLERP) for rotation, ensuring smooth motion paths.
    
    Args:
        T1: Start transformation (at alpha=0)
        T2: End transformation (at alpha=1)
        alpha: Interpolation parameter in [0, 1]
               0 returns T1, 1 returns T2, 0.5 returns midpoint
        
    Returns:
        Interpolated transformation matrix
    
    Note:
        - Uses SLERP for rotation to maintain constant angular velocity
        - Linear interpolation for translation
        - Commonly used for trajectory generation or animation
    
    Example:
        >>> # Generate 10 intermediate poses between start and goal
        >>> path = []
        >>> for i in range(11):
        >>>     alpha = i / 10.0
        >>>     T_interp = interpolate_transforms(T_start, T_goal, alpha)
        >>>     path.append(T_interp)
    """
    # Interpolate translation
    t1 = T1[:3, 3]
    t2 = T2[:3, 3]
    t_interp = (1 - alpha) * t1 + alpha * t2
    
    # Interpolate rotation using slerp
    R1 = Rotation.from_matrix(T1[:3, :3])
    R2 = Rotation.from_matrix(T2[:3, :3])
    
    key_rots = Rotation.concatenate([R1, R2])
    key_times = [0, 1]
    
    # Create interpolated slerp
    from scipy.spatial.transform import Slerp
    slerp = Slerp(key_times, key_rots)
    R_interp = slerp([alpha])[0]
    
    # Construct result
    T_interp = np.eye(4)
    T_interp[:3, :3] = R_interp.as_matrix()
    T_interp[:3, 3] = t_interp
    
    return T_interp