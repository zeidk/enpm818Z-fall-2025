"""
polynomial.py - Polynomial Trajectory Generation

This module generates smooth trajectories using polynomial functions:
- Quintic (5th degree) for lateral motion
- Quartic (4th degree) for longitudinal motion

Students must implement the TODO sections.
"""

import numpy as np
from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class TrajectoryState:
    """State at a single point in time."""
    t: float       # Time (seconds)
    s: float       # Longitudinal position (meters)
    d: float       # Lateral position (meters)
    s_dot: float   # Longitudinal velocity (m/s)
    d_dot: float   # Lateral velocity (m/s)
    s_ddot: float  # Longitudinal acceleration (m/s²)
    d_ddot: float  # Lateral acceleration (m/s²)


@dataclass
class Trajectory:
    """Complete trajectory with metadata."""
    states: List[TrajectoryState]  # Sequence of states
    T: float                       # Total duration (seconds)
    d_coeffs: np.ndarray          # Lateral polynomial coefficients
    s_coeffs: np.ndarray          # Longitudinal polynomial coefficients
    cost: float = float('inf')    # Total cost (set by cost function)
    feasible: bool = True         # Whether trajectory passes checks


def quintic_coefficients(
    start: Tuple[float, float, float],
    end: Tuple[float, float, float],
    T: float
) -> np.ndarray:
    """
    Compute quintic polynomial coefficients for lateral (d) trajectory.
    
    TODO: Implement this function.
    
    The quintic polynomial is:
        p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    
    Args:
        start: (p0, v0, a0) - initial position, velocity, acceleration
        end: (pf, vf, af) - final position, velocity, acceleration
        T: Duration in seconds
        
    Returns:
        Array of 6 coefficients [a0, a1, a2, a3, a4, a5]
    
    Algorithm:
        1. Set first 3 coefficients from initial conditions:
           - a0 = p0
           - a1 = v0
           - a2 = a0 / 2
        
        2. Set up 3x3 linear system for remaining coefficients:
           - Boundary conditions at t=T: p(T)=pf, p'(T)=vf, p''(T)=af
           
           Matrix A (3x3):
           | T³    T⁴     T⁵   |
           | 3T²   4T³    5T⁴  |
           | 6T    12T²   20T³ |
           
           Vector b:
           | pf - a0 - a1*T - a2*T² |
           | vf - a1 - 2*a2*T       |
           | af - 2*a2              |
        
        3. Solve: [a3, a4, a5] = np.linalg.solve(A, b)
        
        4. Return [a0, a1, a2, a3, a4, a5]
    """
    p0, v0, a0_val = start
    pf, vf, af = end
    
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


def quartic_coefficients(
    start: Tuple[float, float, float],
    end_vel: Tuple[float, float],
    T: float
) -> np.ndarray:
    """
    Compute quartic polynomial coefficients for longitudinal (s) trajectory.
    
    TODO: Implement this function.
    
    The quartic polynomial is:
        p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴
    
    Note: Final position is NOT constrained - only velocity and acceleration.
    
    Args:
        start: (p0, v0, a0) - initial position, velocity, acceleration
        end_vel: (vf, af) - final velocity, acceleration (position free)
        T: Duration in seconds
        
    Returns:
        Array of 5 coefficients [a0, a1, a2, a3, a4]
    
    Algorithm:
        1. Set first 3 coefficients from initial conditions:
           - a0 = p0
           - a1 = v0
           - a2 = a0 / 2
        
        2. Set up 2x2 linear system for remaining coefficients:
           - Boundary conditions at t=T: p'(T)=vf, p''(T)=af
           
           Matrix A (2x2):
           | 3T²   4T³  |
           | 6T    12T² |
           
           Vector b:
           | vf - a1 - 2*a2*T |
           | af - 2*a2        |
        
        3. Solve: [a3, a4] = np.linalg.solve(A, b)
        
        4. Return [a0, a1, a2, a3, a4]
    """
    p0, v0, a0_val = start
    vf, af = end_vel
    
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


def evaluate_polynomial(coeffs: np.ndarray, t: float) -> Tuple[float, float, float]:
    """
    Evaluate polynomial and its derivatives at time t.
    
    TODO: Implement this function.
    
    Args:
        coeffs: Polynomial coefficients [a0, a1, a2, ...]
        t: Time at which to evaluate
        
    Returns:
        Tuple (position, velocity, acceleration)
    
    Algorithm:
        Position: p(t) = Σ aᵢ * tⁱ
        Velocity: p'(t) = Σ i * aᵢ * tⁱ⁻¹
        Acceleration: p''(t) = Σ i*(i-1) * aᵢ * tⁱ⁻²
    
    Hint: Use a loop or vectorized operations
    """
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


def compute_jerk(coeffs: np.ndarray, t: float) -> float:
    """
    Compute jerk (third derivative) at time t.
    
    TODO: Implement this function.
    
    Args:
        coeffs: Polynomial coefficients
        t: Time at which to evaluate
        
    Returns:
        Jerk value (m/s³)
    
    Algorithm:
        Jerk: p'''(t) = Σ i*(i-1)*(i-2) * aᵢ * tⁱ⁻³ for i >= 3
    """
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


def generate_trajectory(
    d_coeffs: np.ndarray,
    s_coeffs: np.ndarray,
    T: float,
    dt: float = 0.1
) -> Trajectory:
    """
    Generate a complete trajectory by sampling the polynomials.
    
    TODO: Implement this function.
    
    Args:
        d_coeffs: Lateral polynomial coefficients
        s_coeffs: Longitudinal polynomial coefficients
        T: Total duration
        dt: Time step for sampling
        
    Returns:
        Trajectory object with list of TrajectoryState
    
    Algorithm:
        1. Create empty list for states
        2. For t = 0 to T (step dt):
           a. Evaluate d polynomial: d, d_dot, d_ddot = evaluate_polynomial(d_coeffs, t)
           b. Evaluate s polynomial: s, s_dot, s_ddot = evaluate_polynomial(s_coeffs, t)
           c. Create TrajectoryState and append to list
        3. Return Trajectory(states, T, d_coeffs, s_coeffs)
    """
    # TODO: Implement this function
    
    pass  # Remove this line when implementing


def generate_candidate_trajectories(
    current_state: dict,
    target_d: float,
    target_speed: float,
    T_base: float,
    T_range: float = 1.0,
    T_step: float = 0.5,
    d_range: float = 0.5,
    d_step: float = 0.25,
    v_range: float = 2.0,
    v_step: float = 1.0,
    dt: float = 0.1
) -> List[Trajectory]:
    """
    Generate multiple candidate trajectories by sampling end conditions.
    
    Args:
        current_state: Dict with keys 's', 'd', 's_dot', 'd_dot', 's_ddot', 'd_ddot'
        target_d: Target lateral offset
        target_speed: Target longitudinal speed
        T_base: Base time horizon
        T_range: Range around T_base to sample
        T_step: Step size for T sampling
        d_range: Range around target_d to sample
        d_step: Step size for d sampling
        v_range: Range around target_speed to sample
        v_step: Step size for v sampling
        dt: Time step for trajectory discretization
        
    Returns:
        List of Trajectory objects
    """
    candidates = []
    
    # Current state
    s0 = current_state.get('s', 0.0)
    d0 = current_state.get('d', 0.0)
    s_dot0 = current_state.get('s_dot', 0.0)
    d_dot0 = current_state.get('d_dot', 0.0)
    s_ddot0 = current_state.get('s_ddot', 0.0)
    d_ddot0 = current_state.get('d_ddot', 0.0)
    
    # Generate sample values
    T_min = max(1.0, T_base - T_range)
    T_values = np.arange(T_min, T_base + T_range + T_step / 2, T_step)
    d_values = np.arange(target_d - d_range, target_d + d_range + d_step / 2, d_step)
    v_values = np.arange(max(0, target_speed - v_range), target_speed + v_range + v_step / 2, v_step)
    
    for T in T_values:
        for d_target in d_values:
            for v_target in v_values:
                try:
                    # Quintic for lateral: start and end with zero velocity/acceleration
                    d_start = (d0, d_dot0, d_ddot0)
                    d_end = (d_target, 0.0, 0.0)
                    d_coeffs = quintic_coefficients(d_start, d_end, T)
                    
                    # Quartic for longitudinal: velocity keeping
                    s_start = (s0, s_dot0, s_ddot0)
                    s_end = (v_target, 0.0)
                    s_coeffs = quartic_coefficients(s_start, s_end, T)
                    
                    # Generate trajectory
                    traj = generate_trajectory(d_coeffs, s_coeffs, T, dt)
                    candidates.append(traj)
                    
                except (np.linalg.LinAlgError, TypeError):
                    continue
    
    return candidates


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Polynomial Trajectory Generation...\n")
    
    # Test 1: Quintic polynomial
    print("1. Quintic polynomial (lateral trajectory)")
    print("   Lane change: d=0 to d=3.5m in T=4s")
    
    d_start = (0.0, 0.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    T = 4.0
    
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    print(f"   Coefficients: {d_coeffs}")
    
    # Verify boundary conditions
    p0, v0, a0 = evaluate_polynomial(d_coeffs, 0.0)
    pT, vT, aT = evaluate_polynomial(d_coeffs, T)
    
    print(f"   At t=0: p={p0:.3f}, v={v0:.3f}, a={a0:.3f}")
    print(f"   At t=T: p={pT:.3f}, v={vT:.3f}, a={aT:.3f}")
    
    bc_ok = (abs(p0 - 0.0) < 0.01 and abs(v0 - 0.0) < 0.01 and
             abs(pT - 3.5) < 0.01 and abs(vT - 0.0) < 0.01)
    print(f"   Boundary conditions: {'✓' if bc_ok else '✗'}")
    
    # Test 2: Quartic polynomial
    print("\n2. Quartic polynomial (longitudinal trajectory)")
    print("   Speed up: v=20 to v=25 m/s in T=3s")
    
    s_start = (0.0, 20.0, 0.0)
    s_end_vel = (25.0, 0.0)
    T = 3.0
    
    s_coeffs = quartic_coefficients(s_start, s_end_vel, T)
    print(f"   Coefficients: {s_coeffs}")
    
    # Verify boundary conditions
    p0, v0, a0 = evaluate_polynomial(s_coeffs, 0.0)
    pT, vT, aT = evaluate_polynomial(s_coeffs, T)
    
    print(f"   At t=0: s={p0:.3f}, v={v0:.3f}, a={a0:.3f}")
    print(f"   At t=T: s={pT:.3f}, v={vT:.3f}, a={aT:.3f}")
    
    bc_ok = (abs(v0 - 20.0) < 0.01 and abs(vT - 25.0) < 0.01 and abs(aT - 0.0) < 0.01)
    print(f"   Boundary conditions: {'✓' if bc_ok else '✗'}")
    
    # Test 3: Full trajectory generation
    print("\n3. Full trajectory generation")
    
    traj = generate_trajectory(d_coeffs, s_coeffs, T=4.0, dt=0.5)
    print(f"   Generated {len(traj.states)} states:")
    
    for state in traj.states:
        print(f"     t={state.t:.1f}: s={state.s:.2f}m, d={state.d:.2f}m, "
              f"v={state.s_dot:.2f}m/s")
    
    # Test 4: Candidate generation
    print("\n4. Candidate trajectory generation")
    
    current = {'s': 0, 'd': 0, 's_dot': 20, 'd_dot': 0, 's_ddot': 0, 'd_ddot': 0}
    candidates = generate_candidate_trajectories(
        current, target_d=3.5, target_speed=25.0, T_base=4.0
    )
    print(f"   Generated {len(candidates)} candidate trajectories")
    
    print("\n" + "="*50)
    print("Run this file after implementing to verify your code.")
