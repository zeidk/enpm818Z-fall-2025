"""
Polynomial Trajectory Generation

This module provides functions to generate polynomial trajectories:
- Quintic polynomials for lateral (d) motion
- Quartic polynomials for longitudinal (s) motion

YOU MUST IMPLEMENT:
- quintic_coefficients(): Solve for quintic polynomial coefficients
- quartic_coefficients(): Solve for quartic polynomial coefficients
- evaluate_polynomial(): Evaluate polynomial at given time
- PolynomialTrajectory class: Trajectory representation

Refer to L6 lecture materials for the mathematical foundations.
"""

import numpy as np
from typing import Tuple, List
from dataclasses import dataclass


@dataclass
class PolynomialTrajectory:
    """
    Represents a polynomial trajectory.
    
    Attributes:
        coeffs: Polynomial coefficients [c0, c1, c2, ...]
        T: Duration of the trajectory
        order: Order of the polynomial (4 for quartic, 5 for quintic)
    """
    coeffs: np.ndarray
    T: float
    order: int
    
    def evaluate(self, t: float) -> Tuple[float, float, float]:
        """
        Evaluate trajectory at time t.
        
        Returns:
            position, velocity, acceleration at time t
        """
        return evaluate_polynomial(self.coeffs, t)
    
    def sample(self, dt: float = 0.1) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Sample the trajectory at regular intervals.
        
        Args:
            dt: Time step (seconds)
            
        Returns:
            times, positions, velocities, accelerations
        """
        times = np.arange(0, self.T + dt, dt)
        positions = np.zeros_like(times)
        velocities = np.zeros_like(times)
        accelerations = np.zeros_like(times)
        
        for i, t in enumerate(times):
            positions[i], velocities[i], accelerations[i] = self.evaluate(t)
        
        return times, positions, velocities, accelerations


def quintic_coefficients(start: Tuple[float, float, float],
                         end: Tuple[float, float, float],
                         T: float) -> np.ndarray:
    """
    Compute coefficients for a quintic polynomial trajectory.
    
    The quintic polynomial has the form:
        p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    
    The six coefficients are determined by six boundary conditions:
        - Initial position, velocity, acceleration at t=0
        - Final position, velocity, acceleration at t=T
    
    Args:
        start: Tuple of (position, velocity, acceleration) at t=0
        end: Tuple of (position, velocity, acceleration) at t=T
        T: Duration of the trajectory (seconds)
        
    Returns:
        Array of 6 coefficients [c0, c1, c2, c3, c4, c5]
    """
    # TODO: Implement this function
    #
    # The polynomial is: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    # 
    # Boundary conditions give us 6 equations:
    # p(0) = start[0]      -> c0 = start[0]
    # p'(0) = start[1]     -> c1 = start[1]
    # p''(0) = start[2]    -> 2*c2 = start[2]  -> c2 = start[2]/2
    # p(T) = end[0]
    # p'(T) = end[1]
    # p''(T) = end[2]
    #
    # The first three coefficients can be found directly.
    # The last three (c3, c4, c5) require solving a 3x3 linear system.
    #
    # Hints:
    # 1. Set c0 = start[0], c1 = start[1], c2 = start[2]/2
    # 2. Set up the matrix equation A * [c3, c4, c5]^T = b
    #    where A contains powers of T and b contains the remaining constraints
    # 3. Solve using np.linalg.solve()
    
    p0, v0, a0 = start
    pf, vf, af = end
    
    # Direct coefficients from initial conditions
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    
    # Set up linear system for c3, c4, c5
    # p(T) = c0 + c1*T + c2*T^2 + c3*T^3 + c4*T^4 + c5*T^5 = pf
    # p'(T) = c1 + 2*c2*T + 3*c3*T^2 + 4*c4*T^3 + 5*c5*T^4 = vf
    # p''(T) = 2*c2 + 6*c3*T + 12*c4*T^2 + 20*c5*T^3 = af
    
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    
    A = np.array([
        [T3,      T4,       T5],
        [3*T2,    4*T3,     5*T4],
        [6*T,     12*T2,    20*T3]
    ])
    
    b = np.array([
        pf - c0 - c1*T - c2*T2,
        vf - c1 - 2*c2*T,
        af - 2*c2
    ])
    
    x = np.linalg.solve(A, b)
    c3, c4, c5 = x
    
    return np.array([c0, c1, c2, c3, c4, c5])


def quartic_coefficients(start: Tuple[float, float, float],
                         end_vel: Tuple[float, float],
                         T: float) -> np.ndarray:
    """
    Compute coefficients for a quartic polynomial trajectory.
    
    The quartic polynomial has the form:
        p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4
    
    The five coefficients are determined by five boundary conditions:
        - Initial position, velocity, acceleration at t=0
        - Final velocity, acceleration at t=T
        (Note: Final position is NOT constrained)
    
    This is useful for longitudinal motion where we want to reach
    a target velocity but don't care about exact final position.
    
    Args:
        start: Tuple of (position, velocity, acceleration) at t=0
        end_vel: Tuple of (velocity, acceleration) at t=T
        T: Duration of the trajectory (seconds)
        
    Returns:
        Array of 5 coefficients [c0, c1, c2, c3, c4]
    """
    # TODO: Implement this function
    #
    # Similar to quintic, but with 5 equations:
    # p(0) = start[0]      -> c0 = start[0]
    # p'(0) = start[1]     -> c1 = start[1]
    # p''(0) = start[2]    -> 2*c2 = start[2]
    # p'(T) = end_vel[0]
    # p''(T) = end_vel[1]
    #
    # Hints:
    # 1. Set c0, c1, c2 from initial conditions
    # 2. Set up 2x2 linear system for c3, c4
    
    p0, v0, a0 = start
    vf, af = end_vel
    
    # Direct coefficients from initial conditions
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    
    # Set up linear system for c3, c4
    # p'(T) = c1 + 2*c2*T + 3*c3*T^2 + 4*c4*T^3 = vf
    # p''(T) = 2*c2 + 6*c3*T + 12*c4*T^2 = af
    
    T2 = T * T
    T3 = T2 * T
    
    A = np.array([
        [3*T2,    4*T3],
        [6*T,     12*T2]
    ])
    
    b = np.array([
        vf - c1 - 2*c2*T,
        af - 2*c2
    ])
    
    x = np.linalg.solve(A, b)
    c3, c4 = x
    
    return np.array([c0, c1, c2, c3, c4])


def evaluate_polynomial(coeffs: np.ndarray, t: float) -> Tuple[float, float, float]:
    """
    Evaluate a polynomial and its derivatives at time t.
    
    Args:
        coeffs: Polynomial coefficients [c0, c1, c2, ...]
        t: Time at which to evaluate
        
    Returns:
        Tuple of (position, velocity, acceleration) at time t
    """
    # TODO: Implement this function
    #
    # For polynomial p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + ...
    # p'(t) = c1 + 2*c2*t + 3*c3*t^2 + ...
    # p''(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + ...
    #
    # Hints:
    # 1. Use Horner's method or direct evaluation
    # 2. Compute position, velocity, acceleration separately
    
    n = len(coeffs)
    
    # Position: p(t) = sum(c_i * t^i)
    position = 0.0
    for i in range(n):
        position += coeffs[i] * (t ** i)
    
    # Velocity: p'(t) = sum(i * c_i * t^(i-1))
    velocity = 0.0
    for i in range(1, n):
        velocity += i * coeffs[i] * (t ** (i - 1))
    
    # Acceleration: p''(t) = sum(i * (i-1) * c_i * t^(i-2))
    acceleration = 0.0
    for i in range(2, n):
        acceleration += i * (i - 1) * coeffs[i] * (t ** (i - 2))
    
    return position, velocity, acceleration


def compute_jerk(coeffs: np.ndarray, t: float) -> float:
    """
    Compute jerk (third derivative) of polynomial at time t.
    
    Args:
        coeffs: Polynomial coefficients
        t: Time at which to evaluate
        
    Returns:
        Jerk at time t (m/s^3)
    """
    n = len(coeffs)
    jerk = 0.0
    for i in range(3, n):
        jerk += i * (i - 1) * (i - 2) * coeffs[i] * (t ** (i - 3))
    return jerk


def compute_trajectory_cost(coeffs: np.ndarray, T: float, dt: float = 0.1) -> float:
    """
    Compute integrated squared jerk cost for a trajectory.
    
    This is a measure of trajectory "smoothness" - lower is better.
    
    Args:
        coeffs: Polynomial coefficients
        T: Duration
        dt: Integration time step
        
    Returns:
        Integrated squared jerk cost
    """
    cost = 0.0
    t = 0.0
    while t <= T:
        jerk = compute_jerk(coeffs, t)
        cost += jerk ** 2 * dt
        t += dt
    return cost


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Testing Polynomial Trajectory Generation...")
    print("=" * 50)
    
    # Test quintic polynomial
    print("\n--- Quintic Polynomial Test ---")
    
    start = (0.0, 0.0, 0.0)  # position, velocity, acceleration
    end = (10.0, 0.0, 0.0)   # Move 10m, end at rest
    T = 5.0
    
    coeffs = quintic_coefficients(start, end, T)
    print(f"Coefficients: {coeffs}")
    
    # Verify boundary conditions
    p0, v0, a0 = evaluate_polynomial(coeffs, 0.0)
    pT, vT, aT = evaluate_polynomial(coeffs, T)
    
    print(f"At t=0: p={p0:.4f}, v={v0:.4f}, a={a0:.4f}")
    print(f"At t=T: p={pT:.4f}, v={vT:.4f}, a={aT:.4f}")
    print(f"Expected start: p={start[0]}, v={start[1]}, a={start[2]}")
    print(f"Expected end: p={end[0]}, v={end[1]}, a={end[2]}")
    
    # Test quartic polynomial
    print("\n--- Quartic Polynomial Test ---")
    
    start = (0.0, 10.0, 0.0)  # Starting at 10 m/s
    end_vel = (20.0, 0.0)     # Accelerate to 20 m/s
    T = 4.0
    
    coeffs = quartic_coefficients(start, end_vel, T)
    print(f"Coefficients: {coeffs}")
    
    p0, v0, a0 = evaluate_polynomial(coeffs, 0.0)
    pT, vT, aT = evaluate_polynomial(coeffs, T)
    
    print(f"At t=0: p={p0:.4f}, v={v0:.4f}, a={a0:.4f}")
    print(f"At t=T: p={pT:.4f}, v={vT:.4f}, a={aT:.4f}")
    print(f"Expected start: p={start[0]}, v={start[1]}, a={start[2]}")
    print(f"Expected end velocity: v={end_vel[0]}, a={end_vel[1]}")
    
    # Test PolynomialTrajectory class
    print("\n--- PolynomialTrajectory Class Test ---")
    
    traj = PolynomialTrajectory(
        coeffs=quintic_coefficients((0, 0, 0), (5, 0, 0), 3.0),
        T=3.0,
        order=5
    )
    
    times, positions, velocities, accelerations = traj.sample(dt=0.5)
    print(f"Sample times: {times}")
    print(f"Positions: {positions}")
    print(f"Velocities: {velocities}")
    
    # Test jerk computation
    print("\n--- Jerk Cost Test ---")
    
    coeffs = quintic_coefficients((0, 0, 0), (10, 0, 0), 5.0)
    cost = compute_trajectory_cost(coeffs, 5.0)
    print(f"Jerk cost for 10m move in 5s: {cost:.4f}")
    
    coeffs_fast = quintic_coefficients((0, 0, 0), (10, 0, 0), 2.0)
    cost_fast = compute_trajectory_cost(coeffs_fast, 2.0)
    print(f"Jerk cost for 10m move in 2s: {cost_fast:.4f}")
    print("(Faster move should have higher jerk cost)")
    
    print("\nPolynomial Trajectory test complete!")
