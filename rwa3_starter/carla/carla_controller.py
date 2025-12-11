"""
carla_controller.py - Low-Level Vehicle Controllers for CARLA

This module provides PID controllers to convert high-level behavior commands
(target_speed, target_d) into CARLA VehicleControl messages (throttle, brake, steer).

Controllers:
    - LongitudinalPIDController: Speed control via throttle/brake
    - LateralStanleyController: Lateral control via steering (Stanley method)
    - VehicleController: Combined controller for full vehicle control

Usage:
    controller = VehicleController(vehicle)
    control = controller.run_step(target_speed=25.0, target_d=3.5)
    vehicle.apply_control(control)

DO NOT MODIFY THIS FILE.
"""

import carla
import numpy as np
from collections import deque
from typing import Tuple, Optional


class LongitudinalPIDController:
    """
    PID controller for longitudinal (speed) control.
    
    Outputs throttle (0-1) when speed is below target,
    and brake (0-1) when speed is above target.
    """
    
    def __init__(self, Kp: float = 1.0, Ki: float = 0.1, Kd: float = 0.05, dt: float = 0.1):
        """
        Initialize the PID controller.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            dt: Time step for integration/differentiation
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        
        # Error history for integral and derivative terms
        self._error_buffer = deque(maxlen=10)
        self._integral = 0.0
    
    def run_step(self, target_speed: float, current_speed: float) -> Tuple[float, float]:
        """
        Compute throttle and brake based on speed error.
        
        Args:
            target_speed: Desired speed in m/s
            current_speed: Current speed in m/s
            
        Returns:
            Tuple of (throttle, brake) each in range [0, 1]
        """
        error = target_speed - current_speed
        
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term (with anti-windup)
        self._integral += error * self.dt
        self._integral = np.clip(self._integral, -10.0, 10.0)  # Anti-windup
        i_term = self.Ki * self._integral
        
        # Derivative term
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            d_term = self.Kd * (self._error_buffer[-1] - self._error_buffer[-2]) / self.dt
        else:
            d_term = 0.0
        
        # Combined output
        output = p_term + i_term + d_term
        
        # Convert to throttle/brake
        if output >= 0:
            throttle = np.clip(output, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-output, 0.0, 1.0)
        
        return throttle, brake
    
    def reset(self):
        """Reset the controller state."""
        self._error_buffer.clear()
        self._integral = 0.0


class LateralStanleyController:
    """
    Stanley controller for lateral (steering) control.
    
    The Stanley controller combines:
    1. Heading error: Angle between vehicle heading and path tangent
    2. Cross-track error: Lateral distance from path centerline
    
    steering = heading_error + arctan(k * crosstrack_error / speed)
    
    Note: CARLA uses steer > 0 for right, steer < 0 for left.
    """
    
    def __init__(self, k: float = 1.0, k_heading: float = 1.0, Ks: float = 1.0, max_steer: float = 0.5):
        """
        Initialize the Stanley controller.
        
        Args:
            k: Cross-track error gain
            k_heading: Heading error gain
            Ks: Softening constant to prevent division by zero at low speeds
            max_steer: Maximum steering angle (normalized to [-1, 1])
        """
        self.k = k
        self.k_heading = k_heading
        self.Ks = Ks
        self.max_steer = max_steer
    
    def run_step(self, 
                 current_d: float, 
                 target_d: float, 
                 current_speed: float,
                 heading_error: float = 0.0) -> float:
        """
        Compute steering based on lateral and heading error.
        """
        # Cross-track error
        crosstrack_error = current_d - target_d
        
        # Cross-track correction term
        crosstrack_term = np.arctan2(self.k * crosstrack_error, current_speed + self.Ks)
        
        # Heading correction term
        heading_term = -self.k_heading * heading_error
        
        # Combined steering
        steer_rad = heading_term + crosstrack_term
        
        # Normalize
        steer_normalized = steer_rad / 0.52
        
        # Add small left bias to compensate for consistent rightward drift
        steer_normalized -= 0.03
        
        # Clip to valid range
        steer_normalized = np.clip(steer_normalized, -self.max_steer, self.max_steer)
        
        return steer_normalized
    
    def reset(self):
        """Reset controller state (Stanley is memoryless)."""
        pass


class VehicleController:
    """
    Combined vehicle controller for CARLA.
    
    Integrates longitudinal and lateral controllers to produce
    a complete VehicleControl message.
    """
    
    def __init__(self, vehicle: carla.Vehicle, dt: float = 0.1):
        """
        Initialize the combined controller.
        
        Args:
            vehicle: CARLA vehicle actor to control
            dt: Control loop time step
        """
        self.vehicle = vehicle
        self.dt = dt
        
        # Initialize sub-controllers tuned for highway driving
        self.longitudinal_controller = LongitudinalPIDController(
            Kp=0.5, Ki=0.05, Kd=0.01, dt=dt
        )
        self.lateral_controller = LateralStanleyController(
            k=2.0,           # Cross-track gain (strong centering)
            k_heading=2.5,   # Heading error gain (strong curve following)
            Ks=0.5,          # Speed softening (more responsive)
            max_steer=0.6    # Max steering
        )
        
        # Lane change state tracking
        self._lane_change_active = False
        self._lane_change_start_d = 0.0
        self._lane_change_target_d = 0.0
        self._lane_change_progress = 0.0
    
    def run_step(self, 
                 target_speed: float, 
                 target_d: float,
                 current_speed: float,
                 current_d: float,
                 heading_error: float = 0.0,
                 is_lane_change: bool = False) -> carla.VehicleControl:
        """
        Compute vehicle control from high-level commands.
        
        Args:
            target_speed: Desired speed in m/s
            target_d: Target lateral offset from road centerline in m
            current_speed: Current vehicle speed in m/s
            current_d: Current lateral offset in m
            heading_error: Heading error in radians (vehicle vs road)
            is_lane_change: Whether this is a lane change maneuver
            
        Returns:
            carla.VehicleControl with throttle, brake, and steer
        """
        # Longitudinal control
        throttle, brake = self.longitudinal_controller.run_step(
            target_speed, current_speed
        )
        
        # Lateral control
        # For lane changes, we may want smoother transitions
        if is_lane_change:
            # Smooth lane change trajectory
            effective_target_d = self._smooth_lane_change(current_d, target_d)
        else:
            effective_target_d = target_d
        
        steer = self.lateral_controller.run_step(
            current_d, effective_target_d, current_speed, heading_error
        )
        
        # Create CARLA control message
        control = carla.VehicleControl()
        control.throttle = float(throttle)
        control.brake = float(brake)
        control.steer = float(steer)
        control.hand_brake = False
        control.manual_gear_shift = False
        
        return control
    
    def _smooth_lane_change(self, current_d: float, target_d: float) -> float:
        """
        Generate smooth lane change trajectory.
        
        Uses a sigmoid-like profile for comfortable lane changes.
        """
        # Check if we've reached target (within tolerance)
        if abs(current_d - target_d) < 0.3:
            self._lane_change_active = False
            return target_d
        
        # For now, just return target - the Stanley controller will smooth it
        return target_d
    
    def reset(self):
        """Reset all controller states."""
        self.longitudinal_controller.reset()
        self.lateral_controller.reset()
        self._lane_change_active = False


class EmergencyController:
    """
    Emergency controller for collision avoidance.
    
    Applies maximum braking when an imminent collision is detected.
    """
    
    def __init__(self, ttc_threshold: float = 2.0):
        """
        Initialize emergency controller.
        
        Args:
            ttc_threshold: Time-to-collision threshold in seconds
        """
        self.ttc_threshold = ttc_threshold
    
    def check_emergency(self, 
                        distance_ahead: float, 
                        relative_speed: float,
                        current_speed: float) -> Optional[carla.VehicleControl]:
        """
        Check for emergency situation and return emergency control if needed.
        
        Args:
            distance_ahead: Distance to vehicle ahead in meters
            relative_speed: Relative speed (positive = closing) in m/s
            current_speed: Current ego speed in m/s
            
        Returns:
            Emergency VehicleControl if emergency detected, None otherwise
        """
        if relative_speed <= 0:
            return None  # Not closing
        
        # Calculate time to collision
        ttc = distance_ahead / relative_speed
        
        if ttc < self.ttc_threshold and distance_ahead < 15.0:
            # Emergency brake
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            control.hand_brake = False
            return control
        
        return None


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Vehicle Controllers...\n")
    print("Note: Full tests require CARLA connection.\n")
    
    # Test PID controller (no CARLA needed)
    print("1. Testing LongitudinalPIDController:")
    pid = LongitudinalPIDController(Kp=1.0, Ki=0.1, Kd=0.05)
    
    # Test acceleration (below target)
    throttle, brake = pid.run_step(target_speed=30.0, current_speed=20.0)
    print(f"   Speed 20 -> 30: throttle={throttle:.2f}, brake={brake:.2f}")
    assert throttle > 0 and brake == 0, "Should accelerate"
    
    # Test braking (above target)
    pid.reset()
    throttle, brake = pid.run_step(target_speed=20.0, current_speed=30.0)
    print(f"   Speed 30 -> 20: throttle={throttle:.2f}, brake={brake:.2f}")
    assert throttle == 0 and brake > 0, "Should brake"
    
    # Test steady state
    pid.reset()
    throttle, brake = pid.run_step(target_speed=25.0, current_speed=25.0)
    print(f"   Speed 25 -> 25: throttle={throttle:.2f}, brake={brake:.2f}")
    assert throttle < 0.1 and brake < 0.1, "Should be near zero"
    
    print("\n2. Testing LateralStanleyController:")
    stanley = LateralStanleyController(k=0.5, k_heading=1.2, Ks=0.5, max_steer=0.5)
    
    # Test steer right (we're left of target, need to steer right)
    # current_d=0.5 means we're left of center, target_d=0 means we want center
    # So we need to steer right (positive in CARLA)
    steer = stanley.run_step(current_d=0.5, target_d=0.0, current_speed=10.0, heading_error=0.0)
    print(f"   Left of center (d=0.5 -> d=0): steer={steer:.3f}")
    assert steer > 0, "Should steer right (positive) to correct leftward drift"
    
    # Test steer left (we're right of target, need to steer left)
    steer = stanley.run_step(current_d=-0.5, target_d=0.0, current_speed=10.0, heading_error=0.0)
    print(f"   Right of center (d=-0.5 -> d=0): steer={steer:.3f}")
    assert steer < 0, "Should steer left (negative) to correct rightward drift"
    
    # Test on target
    steer = stanley.run_step(current_d=0.0, target_d=0.0, current_speed=10.0, heading_error=0.0)
    print(f"   On center (d=0 -> d=0): steer={steer:.3f}")
    assert abs(steer) < 0.01, "Should be near zero"
    
    # Test heading error correction (pointing right of road)
    steer = stanley.run_step(current_d=0.0, target_d=0.0, current_speed=10.0, heading_error=0.1)
    print(f"   Heading right of road (heading_error=0.1): steer={steer:.3f}")
    assert steer < 0, "Should steer left to correct rightward heading"
    
    print("\n3. Testing EmergencyController:")
    emergency = EmergencyController(ttc_threshold=2.0)
    
    # Test no emergency
    result = emergency.check_emergency(distance_ahead=50.0, relative_speed=5.0, current_speed=25.0)
    print(f"   Distance 50m, rel_speed 5m/s: emergency={result is not None}")
    assert result is None, "Should not be emergency"
    
    # Test emergency
    result = emergency.check_emergency(distance_ahead=5.0, relative_speed=10.0, current_speed=25.0)
    print(f"   Distance 5m, rel_speed 10m/s: emergency={result is not None}")
    assert result is not None, "Should be emergency"
    
    print("\n" + "=" * 50)
    print("✅ All controller tests passed!")
