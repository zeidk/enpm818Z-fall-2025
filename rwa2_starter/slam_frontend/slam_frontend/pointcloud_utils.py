"""
Point Cloud Utilities

This module provides point cloud conversion and manipulation utilities.
DO NOT MODIFY THIS FILE - It is a provided utility.
"""

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
from typing import List, Tuple


def ros_to_open3d(ros_cloud: PointCloud2) -> o3d.geometry.PointCloud:
    """
    Convert ROS PointCloud2 message to Open3D point cloud.
    
    Performs fast conversion from ROS message format to Open3D format using
    numpy vectorization. This is the optimized version that achieves ~50ms
    conversion time for KITTI point clouds (~120k points).
    
    Args:
        ros_cloud: ROS PointCloud2 message with KITTI format (x,y,z,intensity)
    
    Returns:
        Open3D PointCloud object with points only (no colors or normals)
    
    Performance:
        - ~50ms for 120k points (optimized numpy)
        - ~300ms for byte-by-byte parsing (avoided)
    
    Note:
        - Assumes KITTI point structure with 4 float32 fields
        - Removes NaN and Inf points automatically
        - Intensity is ignored in output (only XYZ used)
    """
    # Define KITTI point structure
    dtype_list = [
        ('x', np.float32),
        ('y', np.float32), 
        ('z', np.float32),
        ('intensity', np.float32)
    ]
    
    # Read all points at once
    points_structured = np.frombuffer(ros_cloud.data, dtype=dtype_list)
    
    # Extract xyz as regular array
    points = np.column_stack([
        points_structured['x'],
        points_structured['y'],
        points_structured['z']
    ])
    
    # Remove invalid points
    valid = ~np.any(np.isnan(points) | np.isinf(points), axis=1)
    points = points[valid]
    
    # Create Open3D cloud
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    
    return cloud


def open3d_to_ros(cloud: o3d.geometry.PointCloud, 
                  frame_id: str = "map",
                  timestamp=None) -> PointCloud2:
    """
    Convert Open3D point cloud to ROS PointCloud2 message.
    
    Efficiently converts Open3D format back to ROS message for publishing.
    If the cloud has colors, uses the red channel as intensity (assumes grayscale).
    Otherwise, sets all intensities to 1.0.
    
    Args:
        cloud: Open3D PointCloud object to convert
        frame_id: Frame ID for the message header (default: "map")
        timestamp: ROS time for the header (optional, current time if None)
        
    Returns:
        sensor_msgs/PointCloud2 message ready for publishing
    
    Format:
        Output message has KITTI-compatible structure:
        - x, y, z: 32-bit floats for position
        - intensity: 32-bit float (from color or default 1.0)
    
    Note:
        - Returns empty message if cloud has no points
        - Uses vectorized numpy operations for efficiency
        - Point step is 16 bytes (4 fields × 4 bytes)
    """
    # Create header
    header = Header()
    header.frame_id = frame_id
    if timestamp is not None:
        header.stamp = timestamp
    
    # Get points as numpy array
    points = np.asarray(cloud.points).astype(np.float32)
    
    if len(points) == 0:
        # Return empty cloud
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = 0
        return msg
    
    # Check if we have colors/intensity
    has_intensity = cloud.has_colors()
    if has_intensity:
        colors = np.asarray(cloud.colors)
        # Use red channel as intensity (assuming grayscale)
        intensities = colors[:, 0].astype(np.float32)
    else:
        intensities = np.ones(len(points), dtype=np.float32)
    
    # Create structured array for efficient packing
    # This is MUCH faster than iterating and packing individually
    cloud_data = np.zeros(len(points), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32)
    ])
    
    cloud_data['x'] = points[:, 0]
    cloud_data['y'] = points[:, 1]
    cloud_data['z'] = points[:, 2]
    cloud_data['intensity'] = intensities
    
    # Create PointCloud2 message
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    
    # Define fields
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    
    msg.is_bigendian = False
    msg.point_step = 16  # 4 fields * 4 bytes
    msg.row_step = msg.point_step * len(points)
    msg.is_dense = True
    
    # Convert to bytes efficiently
    msg.data = cloud_data.tobytes()
    
    return msg


def merge_point_clouds(clouds: List[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    """
    Merge multiple point clouds into one combined cloud.
    
    Concatenates all points from multiple clouds into a single cloud.
    Useful for building local maps from multiple keyframes.
    
    Args:
        clouds: List of Open3D point clouds to merge
        
    Returns:
        Single merged point cloud containing all points
    
    Note:
        - Returns empty cloud if input list is empty
        - Preserves colors/normals if present in all clouds
        - No duplicate removal (use downsampling after if needed)
        - Order of points is preserved (first cloud's points come first)
    """
    if len(clouds) == 0:
        return o3d.geometry.PointCloud()
    
    if len(clouds) == 1:
        return clouds[0]
    
    # Start with first cloud
    merged = o3d.geometry.PointCloud(clouds[0])
    
    # Add remaining clouds
    for cloud in clouds[1:]:
        merged += cloud
    
    return merged


def transform_point_cloud(cloud: o3d.geometry.PointCloud, 
                         transform: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Apply rigid transformation to point cloud (returns new copy).
    
    Transforms all points and normals (if present) by the given 4x4 
    transformation matrix. Creates a deep copy to avoid modifying original.
    
    Args:
        cloud: Input point cloud to transform
        transform: 4x4 homogeneous transformation matrix
        
    Returns:
        New transformed point cloud (original unchanged)
    
    Note:
        - Creates deep copy before transformation
        - Transforms both points and normals if present
        - Use cloud.transform() for in-place transformation
    """
    import copy
    cloud_transformed = copy.deepcopy(cloud)
    cloud_transformed.transform(transform)
    return cloud_transformed


def crop_point_cloud_box(cloud: o3d.geometry.PointCloud,
                         min_bound: np.ndarray,
                         max_bound: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Crop point cloud to axis-aligned bounding box.
    
    Removes all points outside the specified bounding box. Useful for
    removing distant points or focusing on a region of interest.
    
    Args:
        cloud: Input point cloud to crop
        min_bound: Minimum bounds [x_min, y_min, z_min] in meters
        max_bound: Maximum bounds [x_max, y_max, z_max] in meters
        
    Returns:
        Cropped point cloud containing only points within bounds
    
    Example:
        >>> # Keep points within 50m cube centered at origin
        >>> cropped = crop_point_cloud_box(cloud, [-25,-25,-5], [25,25,5])
    """
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    return cloud.crop(bbox)


def compute_cloud_center(cloud: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Compute geometric center (centroid) of point cloud.
    
    Calculates the mean position of all points in the cloud.
    
    Args:
        cloud: Input point cloud
        
    Returns:
        3D center point [x, y, z] as numpy array
    
    Note:
        - Returns [0, 0, 0] for empty clouds
        - This is the arithmetic mean, not weighted by intensity
    """
    return np.asarray(cloud.get_center())


def get_cloud_bounds(cloud: o3d.geometry.PointCloud) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get axis-aligned bounding box extents of point cloud.
    
    Computes the minimum and maximum coordinates along each axis.
    
    Args:
        cloud: Input point cloud
        
    Returns:
        min_bound: Minimum coordinates [x_min, y_min, z_min]
        max_bound: Maximum coordinates [x_max, y_max, z_max]
    
    Example:
        >>> min_pt, max_pt = get_cloud_bounds(cloud)
        >>> width = max_pt[0] - min_pt[0]  # X extent
        >>> height = max_pt[2] - min_pt[2]  # Z extent
    """
    bbox = cloud.get_axis_aligned_bounding_box()
    return np.asarray(bbox.min_bound), np.asarray(bbox.max_bound)


def colorize_point_cloud(cloud: o3d.geometry.PointCloud, 
                        color: np.ndarray) -> None:
    """
    Set uniform color for all points in cloud (modifies in-place).
    
    Args:
        cloud: Point cloud to colorize (modified in-place)
        color: RGB color values [r, g, b] in range [0, 1]
    
    Example:
        >>> colorize_point_cloud(cloud, [1, 0, 0])  # Red
        >>> colorize_point_cloud(cloud, [0.5, 0.5, 0.5])  # Gray
    
    Note:
        - Modifies the cloud in-place
        - Overwrites any existing colors
        - Color values should be in [0, 1] range
    """
    cloud.paint_uniform_color(color)


def remove_ground_plane(cloud: o3d.geometry.PointCloud,
                       distance_threshold: float = 0.2,
                       ransac_n: int = 3,
                       num_iterations: int = 1000) -> o3d.geometry.PointCloud:
    """
    Remove ground plane from point cloud using RANSAC.
    
    Identifies the dominant planar surface (assumed to be ground) and
    removes all points belonging to it. Useful for focusing on obstacles
    and objects above ground.
    
    Args:
        cloud: Input point cloud potentially containing ground
        distance_threshold: Max distance to plane for inliers in meters (default: 0.2m)
        ransac_n: Minimum points to define plane (default: 3)
        num_iterations: RANSAC iteration count (default: 1000)
        
    Returns:
        Point cloud with ground plane points removed
    
    Note:
        - Returns original cloud if less than 3 points
        - May remove slanted surfaces, not just horizontal ground
        - Larger distance_threshold removes more aggressive ground region
    
    Example:
        >>> # Remove ground with 20cm tolerance
        >>> obstacles = remove_ground_plane(cloud, distance_threshold=0.2)
    """
    if len(cloud.points) < 3:
        return cloud
    
    # Segment plane
    plane_model, inliers = cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )
    
    # Return points that are not inliers (i.e., not ground)
    outlier_cloud = cloud.select_by_index(inliers, invert=True)
    
    return outlier_cloud


def estimate_cloud_resolution(cloud: o3d.geometry.PointCloud,
                              num_samples: int = 100) -> float:
    """
    Estimate average point spacing (resolution) of point cloud.
    
    Samples random points and computes their average nearest neighbor
    distance to estimate the point cloud resolution. Useful for setting
    parameters like voxel size or correspondence distance.
    
    Args:
        cloud: Input point cloud
        num_samples: Number of points to sample for estimation (default: 100)
                    More samples = more accurate but slower
        
    Returns:
        Average nearest neighbor distance in meters
        Returns 0.0 if cloud has fewer than 2 points
    
    Note:
        - Uses random sampling for efficiency on large clouds
        - Resolution varies with distance for LiDAR data
        - Typical KITTI resolution: 0.05-0.2m depending on range
    
    Example:
        >>> resolution = estimate_cloud_resolution(cloud)
        >>> voxel_size = resolution * 2  # Downsample to half density
    """
    if len(cloud.points) < 2:
        return 0.0
    
    points = np.asarray(cloud.points)
    num_points = len(points)
    
    if num_points < num_samples:
        num_samples = num_points
    
    # Randomly sample points
    indices = np.random.choice(num_points, num_samples, replace=False)
    sampled_points = points[indices]
    
    # Build KD-tree
    pcd_tree = o3d.geometry.KDTreeFlann(cloud)
    
    # Compute nearest neighbor distances
    distances = []
    for point in sampled_points:
        [k, idx, dist] = pcd_tree.search_knn_vector_3d(point, 2)  # k=2 to get nearest neighbor
        if len(dist) > 1:
            distances.append(np.sqrt(dist[1]))  # dist[0] is the point itself
    
    if len(distances) == 0:
        return 0.0
    
    return float(np.mean(distances))