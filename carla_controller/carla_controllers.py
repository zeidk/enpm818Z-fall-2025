"""
CARLA Vehicle Controllers
ENPM818Z - On-Road Automated Vehicles
University of Maryland

Controller implementations for CARLA simulator:
- Pure Pursuit (lateral, kinematic)
- Stanley (lateral, kinematic)
- PID (longitudinal speed control)
- LQR (lateral, dynamic model)
- MPC (simplified)
"""

import numpy as np
import math
from scipy.linalg import solve_continuous_are

from carla_control_utils import (
    find_nearest_waypoint,
    get_path_heading,
    get_path_curvature,
    compute_cross_track_error,
    normalize_angle
)


# =============================================================================
# PURE PURSUIT CONTROLLER
# =============================================================================

class PurePursuitController:
    """
    Pure Pursuit lateral controller.
    
    Geometric controller that steers toward a look-ahead point on the path.
    Best for low to moderate speeds (< 15 m/s).
    
    Args:
        wheelbase: Vehicle wheelbase in meters
        k: Look-ahead gain (seconds) - Ld = k * v + Ld_min
        Ld_min: Minimum look-ahead distance in meters
    """
    
    def __init__(self, wheelbase=2.9, k=0.5, Ld_min=4.0):
        self.L = wheelbase
        self.k = k
        self.Ld_min = Ld_min
    
    def find_lookahead_point(self, x, y, waypoints, Ld):
        """Find the waypoint closest to look-ahead distance."""
        nearest_idx, _ = find_nearest_waypoint(x, y, waypoints)
        
        search_range = waypoints[nearest_idx:]
        if len(search_range) < 2:
            return waypoints[-1]
        
        distances_ahead = np.sqrt(
            (search_range[:, 0] - x)**2 + (search_range[:, 1] - y)**2
        )
        
        beyond_idx = np.where(distances_ahead >= Ld)[0]
        if len(beyond_idx) > 0:
            return search_range[beyond_idx[0]]
        else:
            return search_range[-1]
    
    def get_control(self, state, waypoints):
        """
        Compute steering angle using Pure Pursuit.
        
        Args:
            state: dict with 'x', 'y', 'yaw', 'v'
            waypoints: np.ndarray of shape (N, 2)
            
        Returns:
            float: Steering angle in radians
        """
        x, y = state['x'], state['y']
        yaw = state['yaw']
        v = max(state['v'], 1.0)
        
        # Look-ahead distance
        Ld = self.k * v + self.Ld_min
        
        # Find look-ahead point
        goal = self.find_lookahead_point(x, y, waypoints, Ld)
        
        # Angle to goal point
        dx = goal[0] - x
        dy = goal[1] - y
        
        alpha = np.arctan2(dy, dx) - yaw
        alpha = normalize_angle(alpha)
        
        # Pure Pursuit steering law
        delta = np.arctan2(2.0 * self.L * np.sin(alpha), Ld)
        
        return delta


# =============================================================================
# STANLEY CONTROLLER
# =============================================================================

class StanleyController:
    """
    Stanley lateral controller (front-axle referenced).
    
    Combines heading error correction with cross-track error correction.
    Best for low to moderate speeds (< 15 m/s).
    
    Args:
        wheelbase: Vehicle wheelbase in meters
        k: Cross-track error gain
        k_soft: Softening constant to prevent division by zero
    """
    
    def __init__(self, wheelbase=2.9, k=2.5, k_soft=1.0):
        self.L = wheelbase
        self.k = k
        self.k_soft = k_soft
    
    def get_control(self, state, waypoints):
        """
        Compute steering angle using Stanley controller.
        
        Stanley control law:
            delta = heading_error + arctan(k * crosstrack_error / (v + k_soft))
        
        Args:
            state: dict with 'x', 'y', 'yaw', 'v'
            waypoints: np.ndarray of shape (N, 2)
            
        Returns:
            float: Steering angle in radians
        """
        x, y = state['x'], state['y']
        yaw = state['yaw']
        v = state['v']
        
        # Compute front axle position
        fx = x + self.L * np.cos(yaw)
        fy = y + self.L * np.sin(yaw)
        
        # Find nearest waypoint to front axle
        idx, _ = find_nearest_waypoint(fx, fy, waypoints)
        
        # Get path heading at nearest point
        path_yaw = get_path_heading(waypoints, idx)
        
        # Heading error: angle from vehicle heading to path heading
        # Positive means path is to the left, so steer left
        heading_error = normalize_angle(path_yaw - yaw)
        
        # Cross-track error: signed distance from front axle to path
        # We compute it as perpendicular distance to the path segment
        # Get the path tangent vector
        path_dx = np.cos(path_yaw)
        path_dy = np.sin(path_yaw)
        
        # Vector from waypoint to front axle
        dx = fx - waypoints[idx, 0]
        dy = fy - waypoints[idx, 1]
        
        # Cross-track error (positive = front axle is LEFT of path)
        # Using cross product: tangent × (point - waypoint)
        crosstrack_error = path_dx * dy - path_dy * dx
        
        # Stanley control law
        # If crosstrack_error > 0 (left of path), steer right (negative correction)
        # So we negate the crosstrack term
        crosstrack_term = np.arctan2(self.k * crosstrack_error, v + self.k_soft)
        
        delta = heading_error - crosstrack_term
        
        return delta


# =============================================================================
# PID SPEED CONTROLLER
# =============================================================================

class PIDSpeedController:
    """
    PID controller for longitudinal speed control.
    
    Args:
        Kp: Proportional gain
        Ki: Integral gain
        Kd: Derivative gain
        dt: Control loop period in seconds
    """
    
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05, dt=0.02):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_max = 10.0
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
    
    def get_control(self, current_speed, target_speed):
        """
        Compute throttle and brake commands.
        
        Args:
            current_speed: Current vehicle speed in m/s
            target_speed: Target speed in m/s
            
        Returns:
            tuple: (throttle, brake) both in [0.0, 1.0]
        """
        error = target_speed - current_speed
        
        # PID terms
        P = self.Kp * error
        
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        I = self.Ki * self.integral
        
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative
        self.prev_error = error
        
        output = P + I + D
        
        # Split into throttle/brake
        if output >= 0:
            throttle = np.clip(output, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-output, 0.0, 1.0)
        
        return throttle, brake


# =============================================================================
# LQR CONTROLLER
# =============================================================================

class LQRController:
    """
    Linear Quadratic Regulator for lateral control.
    
    Uses dynamic bicycle model with state feedback.
    Requires gain scheduling with speed.
    
    State: [e_y, e_y_dot, e_theta, e_theta_dot]
    
    Args:
        params: dict with 'm', 'Iz', 'Lf', 'Lr', 'Cf', 'Cr'
        Q: State penalty matrix (4x4)
        R: Control penalty (scalar or 1x1)
        vx_nominal: Nominal speed for initial gain computation
    """
    
    def __init__(self, params, Q=None, R=None, vx_nominal=10.0):
        self.params = params
        self.vx = max(vx_nominal, 1.0)
        
        if Q is None:
            self.Q = np.diag([1.0, 0.1, 1.0, 0.1])
        else:
            self.Q = Q
            
        if R is None:
            self.R = np.array([[0.1]])
        else:
            self.R = np.atleast_2d(R)
        
        self.K = self._compute_gains()
    
    def _compute_gains(self):
        """Compute LQR gains by solving continuous-time Riccati equation."""
        m = self.params['m']
        Iz = self.params['Iz']
        Lf = self.params['Lf']
        Lr = self.params['Lr']
        Cf = self.params['Cf']
        Cr = self.params['Cr']
        vx = self.vx
        
        # State-space matrices for lateral error dynamics
        A = np.array([
            [0, 1, 0, 0],
            [0, -(Cf + Cr) / (m * vx), (Cf + Cr) / m, (Lr * Cr - Lf * Cf) / (m * vx)],
            [0, 0, 0, 1],
            [0, (Lr * Cr - Lf * Cf) / (Iz * vx), (Lf * Cf - Lr * Cr) / Iz, 
             -(Lf**2 * Cf + Lr**2 * Cr) / (Iz * vx)]
        ])
        
        B = np.array([
            [0],
            [Cf / m],
            [0],
            [Lf * Cf / Iz]
        ])
        
        try:
            P = solve_continuous_are(A, B, self.Q, self.R)
            K = np.linalg.inv(self.R) @ B.T @ P
            return K.flatten()
        except Exception as e:
            print(f"Warning: Riccati solution failed: {e}")
            return np.array([0.5, 0.1, 1.0, 0.1])
    
    def update_speed(self, vx, threshold=2.0):
        """Update gains for new speed (gain scheduling)."""
        vx = max(vx, 1.0)
        
        if abs(vx - self.vx) > threshold:
            self.vx = vx
            self.K = self._compute_gains()
    
    def compute_error_state(self, state, waypoints):
        """
        Compute error state vector for LQR.
        
        Args:
            state: dict with 'x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate'
            waypoints: np.ndarray of shape (N, 2)
            
        Returns:
            np.ndarray: [e_y, e_y_dot, e_theta, e_theta_dot]
        """
        x, y = state['x'], state['y']
        yaw = state['yaw']
        vx = max(state['vx'], 1.0)
        vy = state['vy']
        yaw_rate = state['yaw_rate']
        
        # Cross-track error
        e_y, idx = compute_cross_track_error(x, y, waypoints)
        
        # Path heading
        path_yaw = get_path_heading(waypoints, idx)
        
        # Heading error
        e_theta = normalize_angle(yaw - path_yaw)
        
        # Error rates
        e_y_dot = vy + vx * np.sin(e_theta)
        
        kappa = get_path_curvature(waypoints, idx)
        e_theta_dot = yaw_rate - kappa * vx
        
        return np.array([e_y, e_y_dot, e_theta, e_theta_dot])
    
    def get_control(self, error_state):
        """
        Compute steering angle from error state.
        
        Args:
            error_state: np.ndarray [e_y, e_y_dot, e_theta, e_theta_dot]
            
        Returns:
            float: Steering angle in radians
        """
        delta = -self.K @ error_state
        return float(delta)


# =============================================================================
# SIMPLIFIED MPC CONTROLLER
# =============================================================================

class SimpleMPCController:
    """
    Simplified MPC controller for lateral control.
    
    Uses kinematic model with discrete search over steering angles.
    For full nonlinear MPC, use CasADi or ACADOS.
    
    Args:
        wheelbase: Vehicle wheelbase in meters
        horizon: Prediction horizon (number of steps)
        dt: Time step in seconds
    """
    
    def __init__(self, wheelbase=2.9, horizon=10, dt=0.1):
        self.L = wheelbase
        self.N = horizon
        self.dt = dt
        
        # Cost weights
        self.w_cte = 1.0
        self.w_epsi = 1.0
        self.w_delta = 0.1
        self.w_ddelta = 0.5
        
        self.prev_delta = 0.0
    
    def get_control(self, state, waypoints):
        """
        Compute steering using simplified MPC.
        
        Args:
            state: dict with 'x', 'y', 'yaw', 'v'
            waypoints: np.ndarray of shape (N, 2)
            
        Returns:
            float: Steering angle in radians
        """
        x, y = state['x'], state['y']
        yaw = state['yaw']
        v = max(state['v'], 1.0)
        
        # Current errors
        e_y, idx = compute_cross_track_error(x, y, waypoints)
        path_yaw = get_path_heading(waypoints, idx)
        e_psi = normalize_angle(yaw - path_yaw)
        
        best_delta = 0.0
        best_cost = float('inf')
        
        # Search over candidate steering angles
        for delta in np.linspace(-0.5, 0.5, 21):
            # Predict next state (kinematic model)
            x_next = x + v * np.cos(yaw) * self.dt
            y_next = y + v * np.sin(yaw) * self.dt
            yaw_next = yaw + (v / self.L) * np.tan(delta) * self.dt
            
            # Predicted errors
            e_y_pred, _ = compute_cross_track_error(x_next, y_next, waypoints)
            e_psi_pred = normalize_angle(yaw_next - path_yaw)
            
            # Cost
            cost = (self.w_cte * e_y_pred**2 +
                    self.w_epsi * e_psi_pred**2 +
                    self.w_delta * delta**2 +
                    self.w_ddelta * (delta - self.prev_delta)**2)
            
            if cost < best_cost:
                best_cost = cost
                best_delta = delta
        
        self.prev_delta = best_delta
        return best_delta
