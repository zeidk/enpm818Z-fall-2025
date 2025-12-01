"""
Vehicle Controller (PROVIDED)

This module implements:
- Stanley lateral controller
- PID longitudinal controller
- We will see these controllers in L7

Do NOT modify this file.
"""

import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass

from .trajectory_planner import Trajectory, TrajectoryPoint
from .carla_interface import VehicleControl


@dataclass
class ControllerState:
    """Internal state of the controller."""
    # PID state
    integral_error: float = 0.0
    prev_error: float = 0.0
    
    # Stanley state
    prev_steering: float = 0.0


class StanleyController:
    """
    Stanley lateral controller.
    
    Uses front-axle error and heading error to compute steering.
    """
    
    def __init__(self, k_crosstrack: float = 2.5, k_heading: float = 1.0,
                 k_softness: float = 1.0, max_steering: float = 0.5):
        """
        Initialize Stanley controller.
        
        Args:
            k_crosstrack: Crosstrack error gain
            k_heading: Heading error gain (usually 1.0)
            k_softness: Softening factor for low speeds
            max_steering: Maximum steering angle (radians)
        """
        self.k_crosstrack = k_crosstrack
        self.k_heading = k_heading
        self.k_softness = k_softness
        self.max_steering = max_steering
    
    def compute_steering(self, x: float, y: float, theta: float, v: float,
                         trajectory: Trajectory) -> float:
        """
        Compute steering command.
        
        Args:
            x, y: Current vehicle position
            theta: Current heading
            v: Current speed
            trajectory: Reference trajectory
            
        Returns:
            Steering angle in radians
        """
        if not trajectory.points:
            return 0.0
        
        # Find closest point on trajectory
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(trajectory.points):
            dist = np.sqrt((point.x - x)**2 + (point.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Get reference point (look ahead slightly)
        ref_idx = min(closest_idx + 1, len(trajectory.points) - 1)
        ref_point = trajectory.points[ref_idx]
        
        # Heading error
        heading_error = ref_point.theta - theta
        # Normalize to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Crosstrack error (signed distance to path)
        dx = ref_point.x - x
        dy = ref_point.y - y
        
        # Project onto perpendicular direction
        path_heading = ref_point.theta
        crosstrack_error = -dx * np.sin(path_heading) + dy * np.cos(path_heading)
        
        # Stanley formula
        # δ = θ_e + arctan(k * e / (v + k_soft))
        steering = (
            self.k_heading * heading_error +
            np.arctan2(self.k_crosstrack * crosstrack_error, 
                       v + self.k_softness)
        )
        
        # Clamp to limits
        steering = np.clip(steering, -self.max_steering, self.max_steering)
        
        return steering


class PIDController:
    """
    PID longitudinal controller for speed tracking.
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.05,
                 max_throttle: float = 1.0, max_brake: float = 1.0):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_throttle: Maximum throttle output
            max_brake: Maximum brake output
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        
        self.integral = 0.0
        self.prev_error = 0.0
    
    def compute_control(self, current_speed: float, target_speed: float,
                        dt: float) -> Tuple[float, float]:
        """
        Compute throttle and brake commands.
        
        Args:
            current_speed: Current vehicle speed (m/s)
            target_speed: Target speed (m/s)
            dt: Time step (seconds)
            
        Returns:
            throttle: [0, 1]
            brake: [0, 1]
        """
        error = target_speed - current_speed
        
        # PID terms
        p_term = self.kp * error
        
        self.integral += error * dt
        # Anti-windup
        self.integral = np.clip(self.integral, -10.0, 10.0)
        i_term = self.ki * self.integral
        
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Total control output
        control = p_term + i_term + d_term
        
        # Convert to throttle/brake
        if control >= 0:
            throttle = min(control, self.max_throttle)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-control, self.max_brake)
        
        return throttle, brake
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0


class VehicleController:
    """
    Combined lateral and longitudinal controller.
    """
    
    def __init__(self, config: Dict):
        """
        Initialize vehicle controller.
        
        Args:
            config: Configuration dictionary with 'stanley' and 'pid' keys
        """
        stanley_config = config.get('stanley', {})
        pid_config = config.get('pid', {})
        
        self.lateral = StanleyController(
            k_crosstrack=stanley_config.get('k_crosstrack', 2.5),
            k_heading=stanley_config.get('k_heading', 1.0),
            k_softness=stanley_config.get('k_softness', 1.0),
            max_steering=stanley_config.get('max_steering', 0.5)
        )
        
        self.longitudinal = PIDController(
            kp=pid_config.get('kp', 1.0),
            ki=pid_config.get('ki', 0.1),
            kd=pid_config.get('kd', 0.05),
            max_throttle=pid_config.get('max_throttle', 1.0),
            max_brake=pid_config.get('max_brake', 1.0)
        )
        
        self.prev_time = None
    
    def compute_control(self, ego_state: Dict, trajectory: Trajectory,
                        current_time: float) -> VehicleControl:
        """
        Compute vehicle control commands.
        
        Args:
            ego_state: Dict with x, y, theta, v
            trajectory: Reference trajectory
            current_time: Current simulation time
            
        Returns:
            VehicleControl with steering, throttle, brake
        """
        # Compute dt
        if self.prev_time is None:
            dt = 0.02  # Default 50Hz
        else:
            dt = current_time - self.prev_time
        self.prev_time = current_time
        
        dt = max(dt, 0.001)  # Prevent division by zero
        
        # Get current state
        x = ego_state['x']
        y = ego_state['y']
        theta = ego_state['theta']
        v = ego_state['v']
        
        # Get target speed from trajectory
        if trajectory.points:
            # Find current point on trajectory
            traj_point = trajectory.sample(0.5)  # Look ahead 0.5s
            if traj_point is None:
                traj_point = trajectory.points[0]
            target_speed = traj_point.v
        else:
            target_speed = 0.0
        
        # Compute lateral control
        steering = self.lateral.compute_steering(x, y, theta, v, trajectory)
        
        # Compute longitudinal control
        throttle, brake = self.longitudinal.compute_control(v, target_speed, dt)
        
        return VehicleControl(
            steering=steering,
            throttle=throttle,
            brake=brake
        )
    
    def reset(self):
        """Reset controller state."""
        self.longitudinal.reset()
        self.prev_time = None


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Testing Vehicle Controller...")
    print("=" * 50)
    
    # Create controller with default config
    config = {
        'stanley': {
            'k_crosstrack': 2.5,
            'k_heading': 1.0,
            'k_softness': 1.0,
            'max_steering': 0.5
        },
        'pid': {
            'kp': 1.0,
            'ki': 0.1,
            'kd': 0.05,
            'max_throttle': 1.0,
            'max_brake': 1.0
        }
    }
    
    controller = VehicleController(config)
    print("Controller created successfully")
    
    # Create a simple test trajectory
    from .trajectory_planner import TrajectoryPoint, Trajectory
    
    points = []
    for i in range(50):
        points.append(TrajectoryPoint(
            t=i * 0.1,
            x=i * 2.0,
            y=0.0,
            theta=0.0,
            v=20.0,
            kappa=0.0
        ))
    
    trajectory = Trajectory(points=points, feasible=True, collision_free=True)
    
    # Test control computation
    ego_state = {
        'x': 0.0,
        'y': 0.5,  # Slight offset
        'theta': 0.05,  # Slight heading error
        'v': 18.0  # Below target speed
    }
    
    control = controller.compute_control(ego_state, trajectory, 0.0)
    
    print("\nTest scenario:")
    print(f"  Ego: x={ego_state['x']}, y={ego_state['y']}, "
          f"theta={np.degrees(ego_state['theta']):.1f}°, v={ego_state['v']}")
    print("  Target speed: 20.0 m/s")
    print("\nControl output:")
    print(f"  Steering: {np.degrees(control.steering):.2f}°")
    print(f"  Throttle: {control.throttle:.3f}")
    print(f"  Brake: {control.brake:.3f}")
    
    print("\nController test complete!")
