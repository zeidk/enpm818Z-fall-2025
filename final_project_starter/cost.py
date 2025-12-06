"""
cost.py - Cost Function and Feasibility Checking

This module implements trajectory evaluation:
- Cost computation for trajectory quality
- Feasibility checking for vehicle constraints
- Trajectory selection

Students must implement the TODO sections.
"""

import numpy as np
from typing import List, Optional
from dataclasses import dataclass

from polynomial import Trajectory, TrajectoryState, compute_jerk


@dataclass
class CostWeights:
    """Weights for cost function components."""
    w_jerk: float = 1.0       # Jerk penalty
    w_time: float = 1.0       # Time penalty
    w_d: float = 1.0          # Lateral deviation penalty
    w_v: float = 1.0          # Speed deviation penalty
    w_accel: float = 1.0      # Acceleration penalty


@dataclass
class FeasibilityLimits:
    """Vehicle constraint limits."""
    max_velocity: float = 40.0       # m/s
    max_acceleration: float = 4.0    # m/s²
    max_deceleration: float = -8.0   # m/s² (negative)
    max_lateral_accel: float = 3.0   # m/s²
    max_jerk: float = 10.0           # m/s³


def compute_total_cost(
    traj: Trajectory,
    target_d: float,
    target_speed: float,
    weights: CostWeights = None
) -> float:
    """
    Compute total cost for a trajectory.
    
    TODO: Implement this function.
    
    Args:
        traj: Trajectory to evaluate
        target_d: Target lateral position
        target_speed: Target speed
        weights: Cost weights (use defaults if None)
        
    Returns:
        Total weighted cost (lower is better)
    
    Cost Components:
        1. Jerk cost: Integral of squared jerk over trajectory
           J_jerk = Σ (j_s² + j_d²) * dt
           where j_s and j_d are longitudinal and lateral jerk
        
        2. Time cost: Trajectory duration
           J_time = T
        
        3. Lateral deviation: Final lateral position error
           J_d = (d_final - target_d)²
        
        4. Speed deviation: Final speed error
           J_v = (v_final - target_speed)²
        
        5. Acceleration cost: Integral of squared acceleration
           J_accel = Σ (s_ddot² + d_ddot²) * dt
    
    Total: cost = w_jerk*J_jerk + w_time*J_time + w_d*J_d + w_v*J_v + w_accel*J_accel
    
    Hint: Use compute_jerk() from polynomial.py
    """
    if weights is None:
        weights = CostWeights()
    
    # TODO: Implement cost computation
    
    pass  # Remove this line when implementing


def check_feasibility(
    traj: Trajectory,
    limits: FeasibilityLimits = None
) -> bool:
    """
    Check if trajectory satisfies vehicle constraints.
    
    TODO: Implement this function.
    
    Args:
        traj: Trajectory to check
        limits: Constraint limits (use defaults if None)
        
    Returns:
        True if all constraints satisfied, False otherwise
    
    Constraints to check:
        1. Velocity bounds: 0 <= s_dot <= max_velocity
        
        2. Longitudinal acceleration: max_decel <= s_ddot <= max_accel
        
        3. Lateral acceleration: |d_ddot| <= max_lateral_accel
        
        4. Jerk bounds: |jerk| <= max_jerk (for both s and d)
    
    Algorithm:
        For each state in trajectory:
            - Check velocity bounds
            - Check acceleration bounds
            - Check lateral acceleration
        
        For each time step:
            - Compute jerk from polynomial coefficients
            - Check jerk bounds
        
        Return False immediately if any constraint violated
        Return True if all constraints satisfied
    """
    if limits is None:
        limits = FeasibilityLimits()
    
    # TODO: Implement feasibility checking
    
    pass  # Remove this line when implementing


def select_best_trajectory(
    candidates: List[Trajectory],
    target_d: float,
    target_speed: float,
    weights: CostWeights = None,
    limits: FeasibilityLimits = None
) -> Optional[Trajectory]:
    """
    Select the best feasible trajectory from candidates.
    
    TODO: Implement this function.
    
    Args:
        candidates: List of candidate trajectories
        target_d: Target lateral position
        target_speed: Target speed
        weights: Cost weights
        limits: Feasibility limits
        
    Returns:
        Best feasible trajectory, or None if no feasible trajectory
    
    Algorithm:
        1. Initialize best = None, best_cost = infinity
        
        2. For each trajectory in candidates:
           a. Check feasibility - if fails, continue to next
           b. Mark trajectory as feasible
           c. Compute cost
           d. If cost < best_cost:
              - Update best and best_cost
        
        3. Return best trajectory (or None)
    """
    if weights is None:
        weights = CostWeights()
    if limits is None:
        limits = FeasibilityLimits()
    
    # TODO: Implement trajectory selection
    
    pass  # Remove this line when implementing


def get_top_trajectories(
    candidates: List[Trajectory],
    target_d: float,
    target_speed: float,
    n: int = 5,
    weights: CostWeights = None,
    limits: FeasibilityLimits = None
) -> List[Trajectory]:
    """
    Get the top N feasible trajectories sorted by cost.
    
    Useful for visualization of candidate trajectories.
    
    Args:
        candidates: List of candidate trajectories
        target_d: Target lateral position
        target_speed: Target speed
        n: Number of top trajectories to return
        weights: Cost weights
        limits: Feasibility limits
        
    Returns:
        List of top N feasible trajectories, sorted by cost (ascending)
    """
    if weights is None:
        weights = CostWeights()
    if limits is None:
        limits = FeasibilityLimits()
    
    feasible = []
    
    for traj in candidates:
        if check_feasibility(traj, limits):
            traj.feasible = True
            traj.cost = compute_total_cost(traj, target_d, target_speed, weights)
            feasible.append(traj)
    
    # Sort by cost
    feasible.sort(key=lambda t: t.cost)
    
    return feasible[:n]


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Cost Function and Feasibility...\n")
    
    from polynomial import (
        quintic_coefficients, quartic_coefficients, 
        generate_trajectory, generate_candidate_trajectories
    )
    
    # Create a test trajectory (lane change)
    print("1. Creating test trajectory (lane change):")
    
    d_start = (0.0, 0.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    s_start = (0.0, 25.0, 0.0)
    s_end_vel = (25.0, 0.0)
    T = 4.0
    
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    s_coeffs = quartic_coefficients(s_start, s_end_vel, T)
    traj = generate_trajectory(d_coeffs, s_coeffs, T)
    
    print(f"   Duration: {traj.T}s")
    print(f"   States: {len(traj.states)}")
    print(f"   Final d: {traj.states[-1].d:.2f}m")
    print(f"   Final v: {traj.states[-1].s_dot:.2f}m/s")
    
    # Test feasibility
    print("\n2. Testing feasibility checking:")
    
    limits = FeasibilityLimits()
    feasible = check_feasibility(traj, limits)
    print(f"   Default limits: {'✓ Feasible' if feasible else '✗ Infeasible'}")
    
    # Test with stricter limits
    strict_limits = FeasibilityLimits(max_velocity=20.0)
    feasible_strict = check_feasibility(traj, strict_limits)
    print(f"   Strict limits (v<20): {'✓ Feasible' if feasible_strict else '✗ Infeasible (expected)'}")
    
    # Test cost computation
    print("\n3. Testing cost computation:")
    
    weights = CostWeights()
    cost = compute_total_cost(traj, target_d=3.5, target_speed=25.0, weights=weights)
    print(f"   Cost (matching targets): {cost:.2f}")
    
    cost_off = compute_total_cost(traj, target_d=0.0, target_speed=30.0, weights=weights)
    print(f"   Cost (different targets): {cost_off:.2f}")
    
    if cost is not None and cost_off is not None:
        print(f"   Off-target cost should be higher: {'✓' if cost_off > cost else '✗'}")
    
    # Test trajectory selection
    print("\n4. Testing trajectory selection:")
    
    current = {'s': 0, 'd': 0, 's_dot': 25, 'd_dot': 0, 's_ddot': 0, 'd_ddot': 0}
    candidates = generate_candidate_trajectories(
        current, target_d=3.5, target_speed=25.0, T_base=4.0
    )
    print(f"   Generated {len(candidates)} candidates")
    
    best = select_best_trajectory(
        candidates, target_d=3.5, target_speed=25.0
    )
    
    if best is not None:
        print("   Best trajectory:")
        print(f"     Duration: {best.T:.1f}s")
        print(f"     Final d: {best.states[-1].d:.2f}m")
        print(f"     Final v: {best.states[-1].s_dot:.2f}m/s")
        print(f"     Cost: {best.cost:.2f}")
        print("   ✓ Selection working")
    else:
        print("   ✗ No trajectory selected (check implementation)")
    
    print("\n" + "="*50)
    print("Run this file after implementing to verify your code.")
