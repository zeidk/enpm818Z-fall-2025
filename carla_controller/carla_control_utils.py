"""
CARLA Controller Utilities
ENPM818Z - On-Road Automated Vehicles
University of Maryland

Utility functions for vehicle control in CARLA simulator.
"""

import numpy as np
import math


def get_kinematic_params(vehicle):
    """
    Extract kinematic model parameters from CARLA vehicle.
    
    Args:
        vehicle: CARLA vehicle actor
        
    Returns:
        dict: {'L': wheelbase, 'max_steer': max steering angle in radians}
    """
    physics = vehicle.get_physics_control()
    wheels = physics.wheels
    
    # Compute wheelbase from wheel positions (CARLA uses cm)
    front_wheel = wheels[0]  # Front left
    rear_wheel = wheels[2]   # Rear left
    wheelbase = abs(front_wheel.position.x - rear_wheel.position.x) / 100.0
    
    # Steering limits (convert degrees to radians)
    max_steer = math.radians(wheels[0].max_steer_angle)
    
    return {
        'L': wheelbase,
        'max_steer': max_steer
    }


def get_dynamic_params(vehicle):
    """
    Extract dynamic model parameters from CARLA vehicle.
    
    Args:
        vehicle: CARLA vehicle actor
        
    Returns:
        dict: {'m': mass, 'Iz': inertia, 'Lf': CG to front, 
               'Lr': CG to rear, 'Cf': front cornering stiffness,
               'Cr': rear cornering stiffness, 'max_steer': max steering}
    """
    physics = vehicle.get_physics_control()
    wheels = physics.wheels
    
    # Mass
    m = physics.mass
    
    # Wheelbase
    L = abs(wheels[0].position.x - wheels[2].position.x) / 100.0
    
    # Moment of inertia about z-axis (typical approximation)
    Iz = 0.1 * m * L**2
    
    # CG position (assume 45% from front axle for front-engine cars)
    Lf = 0.45 * L
    Lr = L - Lf
    
    # Cornering stiffness (typical values, should be tuned)
    Cf = 80000.0  # N/rad (front axle)
    Cr = 80000.0  # N/rad (rear axle)
    
    # Steering limits
    max_steer = math.radians(wheels[0].max_steer_angle)
    
    return {
        'm': m,
        'Iz': Iz,
        'Lf': Lf,
        'Lr': Lr,
        'L': L,
        'Cf': Cf,
        'Cr': Cr,
        'max_steer': max_steer
    }


def get_vehicle_state(vehicle):
    """
    Extract basic vehicle state for kinematic controllers.
    
    Args:
        vehicle: CARLA vehicle actor
        
    Returns:
        dict: {'x': x position, 'y': y position, 
               'yaw': heading in radians, 'v': speed in m/s}
    """
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    
    return {
        'x': transform.location.x,
        'y': transform.location.y,
        'yaw': math.radians(transform.rotation.yaw),
        'v': math.sqrt(velocity.x**2 + velocity.y**2)
    }


def get_dynamic_state(vehicle):
    """
    Extract full vehicle state for dynamic model controllers.
    
    Args:
        vehicle: CARLA vehicle actor
        
    Returns:
        dict: {'x': x, 'y': y, 'yaw': heading,
               'vx': longitudinal velocity, 'vy': lateral velocity,
               'yaw_rate': angular velocity, 'v': total speed}
    """
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    angular_vel = vehicle.get_angular_velocity()
    
    x = transform.location.x
    y = transform.location.y
    yaw = math.radians(transform.rotation.yaw)
    
    # Transform world velocity to vehicle body frame
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    
    vx = cos_yaw * velocity.x + sin_yaw * velocity.y   # Longitudinal
    vy = -sin_yaw * velocity.x + cos_yaw * velocity.y  # Lateral
    
    # Yaw rate (CARLA angular velocity is in deg/s)
    yaw_rate = math.radians(angular_vel.z)
    
    # Total speed
    v = math.sqrt(velocity.x**2 + velocity.y**2)
    
    return {
        'x': x,
        'y': y,
        'yaw': yaw,
        'vx': vx,
        'vy': vy,
        'yaw_rate': yaw_rate,
        'v': v
    }


def generate_waypoints(world, vehicle, num_waypoints=100, spacing=2.0):
    """
    Generate waypoints along current lane from vehicle position.
    
    Args:
        world: CARLA world object
        vehicle: CARLA vehicle actor
        num_waypoints: Number of waypoints to generate
        spacing: Distance between waypoints in meters
        
    Returns:
        np.ndarray: Array of shape (N, 2) with [x, y] coordinates
    """
    carla_map = world.get_map()
    spawn_point = vehicle.get_transform()
    
    waypoint = carla_map.get_waypoint(spawn_point.location)
    waypoints_carla = [waypoint]
    
    for _ in range(num_waypoints):
        next_wps = waypoint.next(spacing)
        if next_wps:
            waypoint = next_wps[0]
            waypoints_carla.append(waypoint)
        else:
            break
    
    waypoints = np.array([
        [wp.transform.location.x, wp.transform.location.y]
        for wp in waypoints_carla
    ])
    
    return waypoints


def find_nearest_waypoint(x, y, waypoints):
    """
    Find the nearest waypoint to current position.
    
    Args:
        x, y: Current position
        waypoints: Array of waypoints (N, 2)
        
    Returns:
        tuple: (index, distance) of nearest waypoint
    """
    distances = np.sqrt((waypoints[:, 0] - x)**2 + (waypoints[:, 1] - y)**2)
    idx = np.argmin(distances)
    return idx, distances[idx]


def get_path_heading(waypoints, idx):
    """
    Compute path heading at waypoint index.
    
    Args:
        waypoints: Array of waypoints (N, 2)
        idx: Index of current waypoint
        
    Returns:
        float: Path heading in radians
    """
    if idx >= len(waypoints) - 1:
        idx = len(waypoints) - 2
    
    dx = waypoints[idx + 1, 0] - waypoints[idx, 0]
    dy = waypoints[idx + 1, 1] - waypoints[idx, 1]
    
    return np.arctan2(dy, dx)


def get_path_curvature(waypoints, idx):
    """
    Estimate path curvature at waypoint index using three-point method.
    
    Args:
        waypoints: Array of waypoints (N, 2)
        idx: Index of current waypoint
        
    Returns:
        float: Path curvature (1/radius) in 1/m
    """
    if idx < 1 or idx >= len(waypoints) - 1:
        return 0.0
    
    p1 = waypoints[idx - 1]
    p2 = waypoints[idx]
    p3 = waypoints[idx + 1]
    
    v1 = p2 - p1
    v2 = p3 - p2
    
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    
    l1 = np.linalg.norm(v1)
    l2 = np.linalg.norm(v2)
    
    if l1 < 1e-6 or l2 < 1e-6:
        return 0.0
    
    curvature = 2.0 * cross / (l1 * l2 * (l1 + l2))
    
    return curvature


def compute_cross_track_error(x, y, waypoints):
    """
    Compute signed cross-track error (distance to path).
    
    Positive error means vehicle is to the LEFT of path.
    
    Args:
        x, y: Current position
        waypoints: Array of waypoints (N, 2)
        
    Returns:
        tuple: (cross_track_error, nearest_index)
    """
    idx, _ = find_nearest_waypoint(x, y, waypoints)
    
    path_heading = get_path_heading(waypoints, idx)
    
    dx = x - waypoints[idx, 0]
    dy = y - waypoints[idx, 1]
    
    # Positive = left of path
    error = -dx * np.sin(path_heading) + dy * np.cos(path_heading)
    
    return error, idx


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].
    
    Args:
        angle: Angle in radians
        
    Returns:
        float: Normalized angle
    """
    return np.arctan2(np.sin(angle), np.cos(angle))
