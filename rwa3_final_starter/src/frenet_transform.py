"""
Frenet Coordinate Transformation

This module provides functions to convert between Cartesian (x, y) and
Frenet (s, d) coordinate systems.

YOU MUST IMPLEMENT:
- cartesian_to_frenet(): Convert global coordinates to road-relative
- frenet_to_cartesian(): Convert road-relative coordinates to global
- FrenetTransform class: Object-oriented wrapper for transformations

Refer to L6 lecture materials for the mathematical foundations.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class FrenetState:
    """State in Frenet coordinates."""
    s: float      # Arc length along reference path (meters)
    d: float      # Lateral offset from reference path (meters, positive = left)
    s_dot: float  # Longitudinal velocity (m/s)
    d_dot: float  # Lateral velocity (m/s)
    s_ddot: float = 0.0  # Longitudinal acceleration (m/s^2)
    d_ddot: float = 0.0  # Lateral acceleration (m/s^2)


@dataclass
class CartesianState:
    """State in Cartesian coordinates."""
    x: float      # Global X position (meters)
    y: float      # Global Y position (meters)
    theta: float  # Heading angle (radians)
    v: float      # Speed (m/s)
    kappa: float = 0.0  # Curvature (1/m)


def compute_reference_path_properties(reference_path: List[Tuple[float, float]]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute properties of the reference path.
    
    Args:
        reference_path: List of (x, y) waypoints
        
    Returns:
        s_values: Arc length at each waypoint
        headings: Heading angle at each waypoint
        curvatures: Curvature at each waypoint
        path_array: Nx2 array of waypoints
    """
    if len(reference_path) < 2:
        raise ValueError("Reference path must have at least 2 points")
    
    path_array = np.array(reference_path)
    n_points = len(path_array)
    
    # Compute arc length
    s_values = np.zeros(n_points)
    for i in range(1, n_points):
        dx = path_array[i, 0] - path_array[i-1, 0]
        dy = path_array[i, 1] - path_array[i-1, 1]
        s_values[i] = s_values[i-1] + np.sqrt(dx**2 + dy**2)
    
    # Compute headings using finite differences
    headings = np.zeros(n_points)
    for i in range(n_points - 1):
        dx = path_array[i+1, 0] - path_array[i, 0]
        dy = path_array[i+1, 1] - path_array[i, 1]
        headings[i] = np.arctan2(dy, dx)
    headings[-1] = headings[-2]  # Extrapolate last heading
    
    # Compute curvatures using finite differences of heading
    curvatures = np.zeros(n_points)
    for i in range(1, n_points - 1):
        ds = s_values[i+1] - s_values[i-1]
        if ds > 0.01:  # Avoid division by near-zero
            dtheta = headings[i+1] - headings[i-1]
            # Handle angle wraparound
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
            curvatures[i] = dtheta / ds
    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]
    
    return s_values, headings, curvatures, path_array


def find_closest_point_on_path(x: float, y: float, 
                                path_array: np.ndarray,
                                s_values: np.ndarray) -> Tuple[int, float, float]:
    """
    Find the closest point on the reference path to a given position.
    
    Args:
        x, y: Query position
        path_array: Nx2 array of path waypoints
        s_values: Arc length at each waypoint
        
    Returns:
        closest_idx: Index of closest waypoint
        closest_s: Interpolated arc length at closest point
        min_dist: Distance to closest point
    """
    # TODO: Implement this function
    #
    # Hints:
    # 1. Compute distance from (x, y) to each waypoint
    # 2. Find the index of the closest waypoint
    # 3. Optionally: interpolate to find exact closest point on segment
    # 4. Return index, arc length, and distance
    
    # Placeholder implementation (just finds closest waypoint)
    distances = np.sqrt((path_array[:, 0] - x)**2 + (path_array[:, 1] - y)**2)
    closest_idx = np.argmin(distances)
    min_dist = distances[closest_idx]
    closest_s = s_values[closest_idx]
    
    return closest_idx, closest_s, min_dist


def cartesian_to_frenet(x: float, y: float, theta: float, v: float,
                        reference_path: List[Tuple[float, float]]) -> FrenetState:
    """
    Convert Cartesian state to Frenet coordinates.
    
    The Frenet frame is defined relative to the reference path (road centerline):
    - s: Distance traveled along the path from the start
    - d: Perpendicular distance from the path (positive = left of path)
    - s_dot: Velocity component along the path
    - d_dot: Velocity component perpendicular to the path
    
    Args:
        x: Global X position (meters)
        y: Global Y position (meters)
        theta: Heading angle (radians, 0 = East, CCW positive)
        v: Speed (m/s)
        reference_path: List of (x, y) waypoints defining the reference line
        
    Returns:
        FrenetState with s, d, s_dot, d_dot
    """
    # TODO: Implement this function
    #
    # Steps:
    # 1. Compute reference path properties (arc length, headings)
    # 2. Find the closest point on the path to (x, y)
    # 3. Compute s as the arc length to the closest point
    # 4. Compute d as the signed perpendicular distance:
    #    - Vector from path point to vehicle: (dx, dy)
    #    - Path normal vector: (-sin(path_heading), cos(path_heading))
    #    - d = dot product of these vectors
    # 5. Compute velocities:
    #    - delta_theta = vehicle_heading - path_heading
    #    - s_dot = v * cos(delta_theta)
    #    - d_dot = v * sin(delta_theta)
    # 6. Return FrenetState
    
    # Placeholder implementation
    s_values, headings, _, path_array = compute_reference_path_properties(reference_path)
    
    closest_idx, s, _ = find_closest_point_on_path(x, y, path_array, s_values)
    
    # Get path point and heading at closest point
    path_x = path_array[closest_idx, 0]
    path_y = path_array[closest_idx, 1]
    path_heading = headings[closest_idx]
    
    # Compute d (signed perpendicular distance)
    dx = x - path_x
    dy = y - path_y
    # Normal vector (pointing left of path direction)
    normal_x = -np.sin(path_heading)
    normal_y = np.cos(path_heading)
    d = dx * normal_x + dy * normal_y
    
    # Compute velocity components
    delta_theta = theta - path_heading
    delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))  # Normalize
    s_dot = v * np.cos(delta_theta)
    d_dot = v * np.sin(delta_theta)
    
    return FrenetState(s=s, d=d, s_dot=s_dot, d_dot=d_dot)


def frenet_to_cartesian(s: float, d: float, s_dot: float, d_dot: float,
                        reference_path: List[Tuple[float, float]]) -> CartesianState:
    """
    Convert Frenet coordinates to Cartesian state.
    
    Args:
        s: Arc length along reference path (meters)
        d: Lateral offset from reference path (meters, positive = left)
        s_dot: Longitudinal velocity (m/s)
        d_dot: Lateral velocity (m/s)
        reference_path: List of (x, y) waypoints defining the reference line
        
    Returns:
        CartesianState with x, y, theta, v
    """
    # TODO: Implement this function
    #
    # Steps:
    # 1. Compute reference path properties
    # 2. Find the point on the path at arc length s (interpolate if needed)
    # 3. Get the path heading and curvature at that point
    # 4. Compute Cartesian position:
    #    - x = path_x + d * (-sin(path_heading))
    #    - y = path_y + d * cos(path_heading)
    # 5. Compute Cartesian heading:
    #    - theta = path_heading + arctan(d_dot / s_dot)
    # 6. Compute speed:
    #    - v = sqrt(s_dot^2 + d_dot^2)
    # 7. Return CartesianState
    
    # Placeholder implementation
    s_values, headings, curvatures, path_array = compute_reference_path_properties(reference_path)
    
    # Find the point at arc length s (linear interpolation)
    if s <= s_values[0]:
        idx = 0
        t = 0.0
    elif s >= s_values[-1]:
        idx = len(s_values) - 2
        t = 1.0
    else:
        idx = np.searchsorted(s_values, s) - 1
        idx = max(0, min(idx, len(s_values) - 2))
        ds = s_values[idx + 1] - s_values[idx]
        t = (s - s_values[idx]) / ds if ds > 0 else 0.0
    
    # Interpolate path point
    path_x = path_array[idx, 0] + t * (path_array[idx + 1, 0] - path_array[idx, 0])
    path_y = path_array[idx, 1] + t * (path_array[idx + 1, 1] - path_array[idx, 1])
    
    # Interpolate heading
    path_heading = headings[idx] + t * (headings[idx + 1] - headings[idx])
    
    # Compute Cartesian position
    x = path_x + d * (-np.sin(path_heading))
    y = path_y + d * np.cos(path_heading)
    
    # Compute heading
    if abs(s_dot) > 0.01:
        theta = path_heading + np.arctan2(d_dot, s_dot)
    else:
        theta = path_heading
    
    # Compute speed
    v = np.sqrt(s_dot**2 + d_dot**2)
    
    # Interpolate curvature
    kappa = curvatures[idx] + t * (curvatures[idx + 1] - curvatures[idx])
    
    return CartesianState(x=x, y=y, theta=theta, v=v, kappa=kappa)


class FrenetTransform:
    """
    Object-oriented wrapper for Frenet coordinate transformations.
    
    Precomputes reference path properties for efficient repeated transformations.
    """
    
    def __init__(self, reference_path: List[Tuple[float, float]]):
        """
        Initialize with a reference path.
        
        Args:
            reference_path: List of (x, y) waypoints
        """
        self.reference_path = reference_path
        self.s_values, self.headings, self.curvatures, self.path_array = \
            compute_reference_path_properties(reference_path)
        self.total_length = self.s_values[-1]
    
    def to_frenet(self, x: float, y: float, theta: float, v: float) -> FrenetState:
        """Convert Cartesian state to Frenet."""
        return cartesian_to_frenet(x, y, theta, v, self.reference_path)
    
    def to_cartesian(self, s: float, d: float, s_dot: float, d_dot: float) -> CartesianState:
        """Convert Frenet state to Cartesian."""
        return frenet_to_cartesian(s, d, s_dot, d_dot, self.reference_path)
    
    def get_path_heading_at_s(self, s: float) -> float:
        """Get the path heading at arc length s."""
        if s <= 0:
            return self.headings[0]
        if s >= self.total_length:
            return self.headings[-1]
        
        idx = np.searchsorted(self.s_values, s) - 1
        idx = max(0, min(idx, len(self.s_values) - 2))
        ds = self.s_values[idx + 1] - self.s_values[idx]
        t = (s - self.s_values[idx]) / ds if ds > 0 else 0.0
        
        return self.headings[idx] + t * (self.headings[idx + 1] - self.headings[idx])
    
    def get_curvature_at_s(self, s: float) -> float:
        """Get the path curvature at arc length s."""
        if s <= 0:
            return self.curvatures[0]
        if s >= self.total_length:
            return self.curvatures[-1]
        
        idx = np.searchsorted(self.s_values, s) - 1
        idx = max(0, min(idx, len(self.s_values) - 2))
        ds = self.s_values[idx + 1] - self.s_values[idx]
        t = (s - self.s_values[idx]) / ds if ds > 0 else 0.0
        
        return self.curvatures[idx] + t * (self.curvatures[idx + 1] - self.curvatures[idx])


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Testing Frenet Transform...")
    print("=" * 50)
    
    # Create a simple straight reference path
    reference_path = [(i * 2.0, 0.0) for i in range(51)]  # 100m straight path
    
    print(f"Reference path: {len(reference_path)} points, ~100m long")
    
    # Test Cartesian to Frenet
    print("\n--- Cartesian to Frenet ---")
    
    # Test point on the path
    frenet = cartesian_to_frenet(20.0, 0.0, 0.0, 10.0, reference_path)
    print(f"Point (20, 0) on path: s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    # Test point offset from the path
    frenet = cartesian_to_frenet(20.0, 2.0, 0.0, 10.0, reference_path)
    print(f"Point (20, 2) offset left: s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    frenet = cartesian_to_frenet(20.0, -1.5, 0.0, 10.0, reference_path)
    print(f"Point (20, -1.5) offset right: s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    # Test Frenet to Cartesian
    print("\n--- Frenet to Cartesian ---")
    
    cartesian = frenet_to_cartesian(50.0, 0.0, 10.0, 0.0, reference_path)
    print(f"s=50, d=0: x={cartesian.x:.2f}, y={cartesian.y:.2f}")
    
    cartesian = frenet_to_cartesian(50.0, 2.0, 10.0, 0.0, reference_path)
    print(f"s=50, d=2: x={cartesian.x:.2f}, y={cartesian.y:.2f}")
    
    # Test roundtrip
    print("\n--- Roundtrip Test ---")
    
    original_x, original_y, original_theta, original_v = 30.0, 1.5, 0.1, 15.0
    
    frenet = cartesian_to_frenet(original_x, original_y, original_theta, original_v, reference_path)
    print(f"Original: ({original_x}, {original_y}), θ={np.degrees(original_theta):.1f}°, v={original_v}")
    print(f"Frenet: s={frenet.s:.2f}, d={frenet.d:.2f}, s_dot={frenet.s_dot:.2f}, d_dot={frenet.d_dot:.2f}")
    
    cartesian = frenet_to_cartesian(frenet.s, frenet.d, frenet.s_dot, frenet.d_dot, reference_path)
    print(f"Recovered: ({cartesian.x:.2f}, {cartesian.y:.2f}), θ={np.degrees(cartesian.theta):.1f}°, v={cartesian.v:.2f}")
    
    # Check error
    x_error = abs(original_x - cartesian.x)
    y_error = abs(original_y - cartesian.y)
    print(f"Roundtrip error: x={x_error:.4f}, y={y_error:.4f}")
    
    # Test FrenetTransform class
    print("\n--- FrenetTransform Class ---")
    
    transform = FrenetTransform(reference_path)
    print(f"Total path length: {transform.total_length:.2f} m")
    
    frenet = transform.to_frenet(40.0, 1.0, 0.0, 20.0)
    print(f"to_frenet(40, 1): s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    print("\nFrenet Transform test complete!")
