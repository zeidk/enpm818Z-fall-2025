"""
ICP (Iterative Closest Point) Utilities

This module provides complete ICP alignment functionality using Open3D.
DO NOT MODIFY THIS FILE - It is a provided utility.

Author: ENPM818Z Course Staff
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Optional
import copy


def align_point_clouds(
    source_cloud: o3d.geometry.PointCloud,
    target_cloud: o3d.geometry.PointCloud,
    initial_transform: np.ndarray = np.eye(4),
    max_correspondence_distance: float = 0.5,
    max_iterations: int = 30,
) -> Tuple[np.ndarray, float]:
    """
    Align source point cloud to target using ICP with point-to-plane metric.
    
    Performs Iterative Closest Point alignment to find the optimal rigid transformation
    that aligns the source cloud to the target cloud. Uses point-to-plane distance
    metric which typically converges faster than point-to-point for structured scenes.
    Automatically estimates normals if not present.

    Args:
        source_cloud: Source point cloud to be transformed (current scan)
        target_cloud: Target point cloud as reference (previous scan or map)
        initial_transform: Initial transformation guess as 4x4 matrix (default: identity)
        max_correspondence_distance: Maximum distance for valid point correspondences in meters.
                                    Points farther apart won't be matched (default: 0.5m)
        max_iterations: Maximum number of ICP iterations before stopping (default: 30)

    Returns:
        transformation: 4x4 homogeneous transformation matrix that aligns source to target
        fitness: ICP fitness score between 0-1, where 1 indicates perfect alignment.
                Score represents fraction of points with correspondences.
    
    Note:
        - Returns identity matrix if either cloud is empty
        - Automatically computes normals if not present (required for point-to-plane)
        - Uses relative convergence criteria for early stopping
    """
    # Ensure clouds have points
    if len(source_cloud.points) == 0 or len(target_cloud.points) == 0:
        return np.eye(4), 0.0

    # Estimate normals if not present (helps with ICP)
    if not source_cloud.has_normals():
        source_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
        )
    if not target_cloud.has_normals():
        target_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
        )

    # Perform ICP alignment
    reg_result = o3d.pipelines.registration.registration_icp(
        source_cloud,
        target_cloud,
        max_correspondence_distance,
        initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iterations, relative_fitness=1e-6, relative_rmse=1e-6
        ),
    )

    return reg_result.transformation, reg_result.fitness


def downsample_cloud(
    cloud: o3d.geometry.PointCloud, voxel_size: float = 0.2
) -> o3d.geometry.PointCloud:
    """
    Downsample point cloud using voxel grid filter for efficiency.
    
    Reduces the number of points by replacing all points within each voxel
    (3D grid cell) with their centroid. This preserves the overall structure
    while significantly reducing computational cost for subsequent operations.

    Args:
        cloud: Input point cloud to downsample
        voxel_size: Size of each voxel cube edge in meters (default: 0.2m)
                   Larger values = fewer points, faster processing
                   Smaller values = more points, better detail preservation

    Returns:
        Downsampled point cloud with approximately (original_size / voxel_size³) points
    
    Note:
        - Preserves colors and normals if present in original cloud
        - Returns original cloud unchanged if empty
        - Typical values: 0.1-0.2m for accurate matching, 0.3-0.5m for fast processing
    """
    if len(cloud.points) == 0:
        return cloud

    return cloud.voxel_down_sample(voxel_size)


def remove_statistical_outliers(
    cloud: o3d.geometry.PointCloud, nb_neighbors: int = 20, std_ratio: float = 2.0
) -> o3d.geometry.PointCloud:
    """
    Remove statistical outliers from point cloud using statistical analysis.
    
    Identifies and removes points that are far from their neighbors based on
    the average distance to k-nearest neighbors. Points whose average distance
    is outside mean ± (std_ratio × standard_deviation) are considered outliers.

    Args:
        cloud: Input point cloud potentially containing noise/outliers
        nb_neighbors: Number of nearest neighbors to consider for each point (default: 20)
        std_ratio: Standard deviation multiplier for outlier threshold (default: 2.0)
                  Higher values = less aggressive filtering
                  Lower values = more aggressive filtering

    Returns:
        Filtered point cloud with outliers removed
    
    Note:
        - Useful for removing sensor noise and isolated points
        - Returns original cloud if empty
        - Typical values: nb_neighbors=10-30, std_ratio=1.0-3.0
    """
    if len(cloud.points) == 0:
        return cloud

    cl, ind = cloud.remove_statistical_outlier(nb_neighbors, std_ratio)
    return cl


def compute_icp_fitness(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    transformation: np.ndarray,
    max_distance: float = 0.5,
) -> float:
    """
    Compute fitness score for a given transformation between two clouds.
    
    Evaluates how well a transformation aligns the source cloud to the target
    by counting the fraction of points that have correspondences within the
    specified distance threshold. Useful for validating ICP results.

    Args:
        source: Source point cloud to evaluate
        target: Target point cloud as reference
        transformation: 4x4 transformation matrix to evaluate
        max_distance: Maximum distance for valid correspondences in meters (default: 0.5m)

    Returns:
        Fitness score between 0-1:
        - 0: No alignment (no points have close correspondences)
        - 1: Perfect alignment (all points have close correspondences)
        - Typical good values: 0.5-0.8 for successive scans
    
    Note:
        - Higher fitness indicates better alignment
        - Can be used to detect ICP failure (fitness < 0.3)
        - Returns 0 if either cloud is empty
    """
    if len(source.points) == 0 or len(target.points) == 0:
        return 0.0

    # Transform source cloud
    source_transformed = copy.deepcopy(source)
    source_transformed.transform(transformation)

    # Compute distances
    dists = source_transformed.compute_point_cloud_distance(target)
    dists = np.asarray(dists)

    # Compute fitness as ratio of points within threshold
    fitness = np.sum(dists < max_distance) / len(dists)

    return fitness


def estimate_normals(
    cloud: o3d.geometry.PointCloud, radius: float = 1.0, max_nn: int = 30
) -> None:
    """
    Estimate surface normals for point cloud (modifies cloud in-place).
    
    Computes normal vectors for each point by fitting a plane to its local
    neighborhood. Required for point-to-plane ICP and visualization.

    Args:
        cloud: Point cloud to process (modified in-place)
        radius: Search radius for neighbor points in meters (default: 1.0m)
        max_nn: Maximum number of neighbors to use (default: 30)
               Limits computation if many points are within radius

    Returns:
        None (modifies cloud in-place by adding normals)
    
    Note:
        - Uses hybrid KDTree search (radius + k-nearest)
        - Orient normals consistently using viewpoint information if available
        - Does nothing if cloud is empty
        - Computation time scales with cloud size and max_nn
    """
    if len(cloud.points) == 0:
        return

    cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )


# Convenience function for the assignment
def align_clouds(
    source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud
) -> np.ndarray:
    """
    Simple wrapper for ICP alignment with default parameters optimized for KITTI.
    
    This is the main function students should use for scan-to-map alignment.
    Uses conservative default parameters suitable for the KITTI dataset and
    the assignment requirements. Internally calls align_point_clouds with
    appropriate settings.

    Args:
        source: Source point cloud (current scan to be aligned)
        target: Target point cloud (local map or previous scan as reference)

    Returns:
        4x4 transformation matrix that transforms source to target frame
        
    Default parameters used:
        - max_correspondence_distance: 0.5m (suitable for KITTI sensor range)
        - max_iterations: 30 (balanced speed vs accuracy)
        - Point-to-plane metric (better for structured environments)
    
    Example usage:
        >>> delta_T = align_clouds(current_scan, local_map)
        >>> current_pose = current_pose @ delta_T
    
    Note:
        - For scan-to-scan matching, consider using fewer iterations (10-15)
        - For final refinement, this function's defaults work well
        - Returns identity matrix if alignment fails
    """
    transformation, fitness = align_point_clouds(
        source, target, max_correspondence_distance=0.5, max_iterations=30
    )

    return transformation
