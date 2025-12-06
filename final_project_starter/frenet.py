"""
frenet.py - Frenet Coordinate Transformation

This module implements conversion between Cartesian (x, y) coordinates
and Frenet (s, d) coordinates relative to a reference path.

Students must implement the TODO sections.

Functions:
    cartesian_to_frenet: Convert (x, y) to (s, d)
    frenet_to_cartesian: Convert (s, d) to (x, y)
"""

import numpy as np
from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class ReferencePath:
    """
    Reference path (road centerline) with precomputed properties.
    
    Attributes:
        points: Array of (x, y) waypoints, shape (N, 2)
        s_values: Arc length at each waypoint, shape (N,)
        tangents: Unit tangent vectors at each waypoint, shape (N, 2)
        normals: Unit normal vectors at each waypoint, shape (N, 2)
        total_length: Total path length
    """
    points: np.ndarray
    s_values: np.ndarray
    tangents: np.ndarray
    normals: np.ndarray
    total_length: float


def create_reference_path(waypoints: List[Tuple[float, float]]) -> ReferencePath:
    """
    Create a ReferencePath from a list of waypoints.
    
    Args:
        waypoints: List of (x, y) tuples defining the path
        
    Returns:
        ReferencePath object with precomputed properties
    """
    points = np.array(waypoints)
    n = len(points)
    
    # Compute arc lengths
    diffs = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    s_values = np.zeros(n)
    s_values[1:] = np.cumsum(segment_lengths)
    total_length = s_values[-1]
    
    # Compute tangent vectors (normalized direction)
    tangents = np.zeros((n, 2))
    for i in range(n - 1):
        tangents[i] = diffs[i] / segment_lengths[i]
    tangents[-1] = tangents[-2]  # Copy last tangent
    
    # Compute normal vectors (perpendicular to tangent, pointing left)
    normals = np.zeros((n, 2))
    normals[:, 0] = -tangents[:, 1]
    normals[:, 1] = tangents[:, 0]
    
    return ReferencePath(
        points=points,
        s_values=s_values,
        tangents=tangents,
        normals=normals,
        total_length=total_length
    )


def cartesian_to_frenet(x: float, y: float, ref_path: ReferencePath) -> Tuple[float, float]:
    """
    Convert Cartesian coordinates to Frenet coordinates.
    
    TODO: Implement this function.
    
    Args:
        x: Global X position (meters)
        y: Global Y position (meters)
        ref_path: Reference path object
        
    Returns:
        Tuple (s, d) where:
        - s: Arc length along path (meters)
        - d: Lateral offset from path (meters, positive = left)
    
    Algorithm:
        1. Find the closest point on the reference path:
           - Compute distance from (x, y) to each waypoint
           - Find index of minimum distance
        
        2. Get the arc length at the closest point:
           - s = ref_path.s_values[closest_idx]
        
        3. Compute lateral offset d:
           - Get normal vector at closest point
           - Compute vector from path point to (x, y)
           - d = dot product of this vector with normal
           - Positive d means left of centerline
        
        4. Return (s, d)
    
    Hint: Use np.linalg.norm for distance computation
    """
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


def frenet_to_cartesian(s: float, d: float, ref_path: ReferencePath) -> Tuple[float, float]:
    """
    Convert Frenet coordinates to Cartesian coordinates.
    
    TODO: Implement this function.
    
    Args:
        s: Arc length along path (meters)
        d: Lateral offset from path (meters, positive = left)
        ref_path: Reference path object
        
    Returns:
        Tuple (x, y) of global Cartesian coordinates
    
    Algorithm:
        1. Find position on path at arc length s:
           - Use np.searchsorted to find segment
           - Interpolate between waypoints if needed
        
        2. Get normal vector at this position
        
        3. Compute Cartesian position:
           - x = path_x + d * normal_x
           - y = path_y + d * normal_y
        
        4. Return (x, y)
    
    Hint: Handle edge cases where s is beyond path bounds
    """
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Frenet Coordinate Transformation...\n")
    
    # Create a simple straight path
    print("1. Testing with straight path (x = 0 to 100):")
    waypoints = [(i, 0) for i in range(0, 101, 10)]
    path = create_reference_path(waypoints)
    
    print(f"   Path length: {path.total_length}m")
    print(f"   Number of waypoints: {len(path.points)}")
    
    # Test point on centerline
    x, y = 50.0, 0.0
    s, d = cartesian_to_frenet(x, y, path)
    print(f"\n   Point on centerline: ({x}, {y})")
    print(f"   Frenet: s={s:.1f}m, d={d:.2f}m")
    expected_s, expected_d = 50.0, 0.0
    s_ok = abs(s - expected_s) < 0.5
    d_ok = abs(d - expected_d) < 0.1
    print(f"   Expected: s={expected_s}, d={expected_d} {'✓' if s_ok and d_ok else '✗'}")
    
    # Test point left of centerline
    x, y = 50.0, 3.5
    s, d = cartesian_to_frenet(x, y, path)
    print(f"\n   Point left of centerline: ({x}, {y})")
    print(f"   Frenet: s={s:.1f}m, d={d:.2f}m")
    expected_s, expected_d = 50.0, 3.5
    s_ok = abs(s - expected_s) < 0.5
    d_ok = abs(d - expected_d) < 0.1
    print(f"   Expected: s={expected_s}, d={expected_d} {'✓' if s_ok and d_ok else '✗'}")
    
    # Test point right of centerline
    x, y = 50.0, -3.5
    s, d = cartesian_to_frenet(x, y, path)
    print(f"\n   Point right of centerline: ({x}, {y})")
    print(f"   Frenet: s={s:.1f}m, d={d:.2f}m")
    expected_s, expected_d = 50.0, -3.5
    s_ok = abs(s - expected_s) < 0.5
    d_ok = abs(d - expected_d) < 0.1
    print(f"   Expected: s={expected_s}, d={expected_d} {'✓' if s_ok and d_ok else '✗'}")
    
    # Test frenet_to_cartesian
    print("\n2. Testing frenet_to_cartesian:")
    
    s, d = 50.0, 0.0
    x, y = frenet_to_cartesian(s, d, path)
    print(f"   Frenet: s={s}, d={d}")
    print(f"   Cartesian: ({x:.1f}, {y:.1f})")
    expected_x, expected_y = 50.0, 0.0
    x_ok = abs(x - expected_x) < 0.5
    y_ok = abs(y - expected_y) < 0.1
    print(f"   Expected: ({expected_x}, {expected_y}) {'✓' if x_ok and y_ok else '✗'}")
    
    s, d = 50.0, 3.5
    x, y = frenet_to_cartesian(s, d, path)
    print(f"\n   Frenet: s={s}, d={d}")
    print(f"   Cartesian: ({x:.1f}, {y:.1f})")
    expected_x, expected_y = 50.0, 3.5
    x_ok = abs(x - expected_x) < 0.5
    y_ok = abs(y - expected_y) < 0.1
    print(f"   Expected: ({expected_x}, {expected_y}) {'✓' if x_ok and y_ok else '✗'}")
    
    # Test roundtrip accuracy
    print("\n3. Testing roundtrip accuracy:")
    
    test_points = [(25.0, 1.5), (50.0, -2.0), (75.0, 3.0)]
    max_error = 0.0
    
    for x_orig, y_orig in test_points:
        s, d = cartesian_to_frenet(x_orig, y_orig, path)
        x_back, y_back = frenet_to_cartesian(s, d, path)
        error = np.sqrt((x_orig - x_back)**2 + (y_orig - y_back)**2)
        max_error = max(max_error, error)
        status = "✓" if error < 0.5 else "✗"
        print(f"   ({x_orig}, {y_orig}) -> (s={s:.1f}, d={d:.2f}) -> ({x_back:.1f}, {y_back:.1f}), error={error:.3f}m {status}")
    
    print(f"\n   Maximum roundtrip error: {max_error:.3f}m")
    print(f"   Requirement: < 0.5m {'✓' if max_error < 0.5 else '✗'}")
    
    print("\n" + "="*50)
    print("Run this file after implementing to verify your code.")
