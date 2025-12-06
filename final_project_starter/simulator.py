"""
simulator.py - Integrated Highway Driving Simulator

This simulator combines:
1. Behavior Tree from RWA3 (behavioral planning)
2. Frenet Trajectory Planning (trajectory generation)

The behavior tree decides WHAT to do (lane keep, follow, change lanes)
The trajectory planner decides HOW to do it (smooth polynomial trajectories)

Usage:
    python simulator.py --scenario empty
    python simulator.py --scenario follow
    python simulator.py --scenario overtake
    python simulator.py --no-viz --scenario overtake --duration 30
"""

import numpy as np
import argparse
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Tuple

from frenet import ReferencePath, create_reference_path, cartesian_to_frenet, frenet_to_cartesian
from polynomial import (
    Trajectory, TrajectoryState, 
    quintic_coefficients, quartic_coefficients,
    generate_trajectory, generate_candidate_trajectories
)
from cost import (
    CostWeights, FeasibilityLimits,
    compute_total_cost, check_feasibility, select_best_trajectory
)
from bt_framework import EnvironmentState, BehaviorCommand, BehaviorType


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class SimConfig:
    """Simulation configuration parameters."""
    # Road geometry
    lane_width: float = 3.5        # meters
    num_lanes: int = 3             # total lanes
    road_length: float = 1000.0    # meters
    
    # Vehicle parameters
    vehicle_length: float = 4.5    # meters
    vehicle_width: float = 2.0     # meters
    
    # Speed limits
    speed_limit: float = 31.0      # m/s (~70 mph)
    min_speed: float = 0.0         # m/s
    
    # Simulation
    dt: float = 0.1                # time step (seconds)
    
    # Detection parameters
    detection_range: float = 100.0  # How far ahead to detect vehicles
    lane_change_gap: float = 25.0   # Required gap for lane change
    
    # Trajectory planning
    planning_horizon: float = 4.0  # seconds
    replan_interval: float = 0.5   # seconds


@dataclass
class Vehicle:
    """Vehicle state."""
    x: float           # Longitudinal position (meters)
    y: float           # Lateral position (meters)
    speed: float       # Longitudinal speed (m/s)
    lane: int          # Current lane (-1=right, 0=center, 1=left)
    
    # Frenet state
    s: float = 0.0     # Arc length along reference path
    d: float = 0.0     # Lateral offset from centerline
    s_dot: float = 0.0
    d_dot: float = 0.0
    s_ddot: float = 0.0
    d_ddot: float = 0.0


class Scenario(Enum):
    """Available test scenarios."""
    EMPTY = "empty"
    FOLLOW = "follow"
    OVERTAKE = "overtake"


# =============================================================================
# Simulator
# =============================================================================

class Simulator:
    """
    Integrated highway driving simulator.
    
    Combines behavior tree decision making with Frenet trajectory planning.
    """
    
    def __init__(self, config: SimConfig = None):
        self.config = config or SimConfig()
        
        # Create curved reference path (5 curves)
        self.ref_path = self._create_curved_road()
        
        # Vehicles
        self.ego = Vehicle(x=0.0, y=0.0, speed=25.0, lane=0)
        self.traffic: List[Vehicle] = []
        
        # Current trajectory
        self.current_trajectory: Optional[Trajectory] = None
        self.trajectory_start_time: float = 0.0
        
        # Current behavior command
        self.current_command: Optional[BehaviorCommand] = None
        
        # Timing
        self.time: float = 0.0
        self.last_replan_time: float = -float('inf')
        
        # Planning parameters
        self.cost_weights = CostWeights(
            w_jerk=0.1,
            w_time=1.0,
            w_d=10.0,
            w_v=5.0,
            w_accel=0.1
        )
        self.feasibility_limits = FeasibilityLimits()
    
    def _create_curved_road(self) -> ReferencePath:
        """
        Create a curved reference path with 5 S-curves.
        
        The road follows a sinusoidal pattern to demonstrate
        trajectory planning on curved roads.
        """
        waypoints = []
        
        # Parameters for 5 S-curves over 1000m
        num_points = 201
        road_length = self.config.road_length
        
        for i in range(num_points):
            s = i * road_length / (num_points - 1)
            x = s
            
            # 5 gentle S-curves: y = A * sin(2*pi*n * x / L)
            # Where n=5 for 5 full cycles, A=15m amplitude
            amplitude = 15.0
            num_curves = 5
            y = amplitude * np.sin(2 * np.pi * num_curves * s / road_length)
            
            waypoints.append((x, y))
        
        return create_reference_path(waypoints)
    
    def setup_scenario(self, scenario: Scenario):
        """Set up a test scenario."""
        self.time = 0.0
        self.last_replan_time = -float('inf')
        self.current_trajectory = None
        
        # Initialize ego at s=0, d=0 (center lane, start of road)
        self.ego = Vehicle(x=0.0, y=0.0, speed=25.0, lane=0)
        self.ego.s = 0.0
        self.ego.d = 0.0
        self.ego.s_dot = self.ego.speed
        # Convert to Cartesian
        self.ego.x, self.ego.y = frenet_to_cartesian(self.ego.s, self.ego.d, self.ref_path)
        
        self.traffic.clear()
        
        if scenario == Scenario.EMPTY:
            # No traffic - just cruise on curved road
            pass
            
        elif scenario == Scenario.FOLLOW:
            # Follow scenario: ego follows a lead vehicle while adjacent lanes are blocked
            # Lead at 15 m/s to match ego's ~14.5 m/s average speed (with trajectory dynamics)
            
            # Lead vehicle in center lane - start far enough ahead to allow ego to slow down
            # Ego starts at 25 m/s and needs time to decelerate to 15 m/s
            self._add_traffic_vehicle(s=40.0, d=0.0, speed=15.0, lane=0)
            
            # Backup lead vehicles at regular intervals
            self._add_traffic_vehicle(s=110.0, d=0.0, speed=15.0, lane=0)
            self._add_traffic_vehicle(s=180.0, d=0.0, speed=15.0, lane=0)
            
            # Left lane vehicles - convoy at 15 m/s (same as lead, so ego never catches up)
            for s_pos in range(-80, 41, 10):
                self._add_traffic_vehicle(s=float(s_pos), d=self.config.lane_width, speed=15.0, lane=1)
            
            # Right lane vehicles - same setup
            for s_pos in range(-80, 41, 10):
                self._add_traffic_vehicle(s=float(s_pos), d=-self.config.lane_width, speed=15.0, lane=-1)
            
            # Ego starts faster, will need to slow down to follow
            self.ego.speed = 25.0
            self.ego.s_dot = 25.0
            
        elif scenario == Scenario.OVERTAKE:
            # Slow vehicle in center lane ahead
            self._add_traffic_vehicle(s=40.0, d=0.0, speed=20.0, lane=0)
            
            # Block right lane initially
            self._add_traffic_vehicle(s=35.0, d=-self.config.lane_width, speed=24.0, lane=-1)
            
            # Slow vehicle in left lane further ahead
            self._add_traffic_vehicle(s=150.0, d=self.config.lane_width, speed=18.0, lane=1)
    
    def _add_traffic_vehicle(self, s: float, d: float, speed: float, lane: int):
        """Add a traffic vehicle using Frenet coordinates."""
        # Convert Frenet to Cartesian
        x, y = frenet_to_cartesian(s, d, self.ref_path)
        
        vehicle = Vehicle(
            x=x,
            y=y,
            speed=speed,
            lane=lane,
            s=s,
            d=d,
            s_dot=speed,
            d_dot=0.0
        )
        self.traffic.append(vehicle)
    
    def _update_frenet_state(self, vehicle: Vehicle):
        """Update vehicle's Frenet state from Cartesian position."""
        vehicle.s, vehicle.d = cartesian_to_frenet(vehicle.x, vehicle.y, self.ref_path)
        vehicle.s_dot = vehicle.speed
        vehicle.d_dot = 0.0  # Assume no lateral velocity for traffic
    
    def get_environment_state(self) -> EnvironmentState:
        """
        Get current environment state for behavior tree.
        
        Returns:
            EnvironmentState with all perception data
        """
        env = EnvironmentState()
        
        # Ego state
        env.ego_speed = self.ego.speed
        env.ego_d = self.ego.d
        env.speed_limit = self.config.speed_limit
        
        # Lane existence
        current_lane = round(self.ego.d / self.config.lane_width)
        env.left_lane_exists = current_lane < (self.config.num_lanes - 1) // 2
        env.right_lane_exists = current_lane > -(self.config.num_lanes - 1) // 2
        
        # Find vehicle ahead in same lane
        env.vehicle_ahead = False
        env.vehicle_ahead_distance = self.config.detection_range
        env.vehicle_ahead_speed = 0.0
        
        ego_lane = round(self.ego.d / self.config.lane_width)
        
        for v in self.traffic:
            v_lane = round(v.d / self.config.lane_width)
            
            if v_lane == ego_lane:
                dist = v.s - self.ego.s
                if 0 < dist < env.vehicle_ahead_distance:
                    env.vehicle_ahead = True
                    env.vehicle_ahead_distance = dist
                    env.vehicle_ahead_speed = v.speed
        
        # Check lane clearance for lane changes
        env.left_lane_clear = self._check_lane_clear(current_lane + 1)
        env.right_lane_clear = self._check_lane_clear(current_lane - 1)
        
        return env
    
    def _check_lane_clear(self, target_lane: int) -> bool:
        """Check if a lane is clear for lane change."""
        for v in self.traffic:
            v_lane = round(v.d / self.config.lane_width)
            if v_lane == target_lane:
                dist = abs(v.s - self.ego.s)
                if dist < self.config.lane_change_gap:
                    return False
        return True
    
    def plan_trajectory(self, command: BehaviorCommand) -> Optional[Trajectory]:
        """
        Plan a trajectory based on behavior command.
        
        Args:
            command: Behavior command from behavior tree
            
        Returns:
            Best trajectory, or None if planning fails
        """
        # Current state in Frenet
        current_state = {
            's': self.ego.s,
            'd': self.ego.d,
            's_dot': self.ego.s_dot,
            'd_dot': self.ego.d_dot,
            's_ddot': self.ego.s_ddot,
            'd_ddot': self.ego.d_ddot
        }
        
        # Generate candidate trajectories
        # Use larger T_range to allow gentler acceleration when target speed differs greatly
        # Use larger v_range to explore more speed options
        # Use larger steps to reduce computation time for smoother animation
        candidates = generate_candidate_trajectories(
            current_state=current_state,
            target_d=command.target_d,
            target_speed=command.target_speed,
            T_base=command.T,
            T_range=3.0,   # Wide range for flexibility
            T_step=1.0,    # Larger step to reduce candidates
            d_range=0.5,
            d_step=0.5,    # Larger step
            v_range=8.0,   # Wide range for speed flexibility
            v_step=2.0     # Larger step to reduce candidates
        )
        
        # Select best trajectory
        best = select_best_trajectory(
            candidates,
            target_d=command.target_d,
            target_speed=command.target_speed,
            weights=self.cost_weights,
            limits=self.feasibility_limits
        )
        
        return best
    
    def step(self, command: BehaviorCommand):
        """
        Advance simulation by one time step.
        
        Args:
            command: Behavior command from behavior tree
        """
        dt = self.config.dt
        self.current_command = command
        
        # Replan trajectory if needed
        if self.time - self.last_replan_time >= self.config.replan_interval:
            new_trajectory = self.plan_trajectory(command)
            if new_trajectory is not None:
                self.current_trajectory = new_trajectory
                self.trajectory_start_time = self.time
                self.last_replan_time = self.time
        
        # Follow current trajectory
        if self.current_trajectory is not None:
            # Get state at current time along trajectory
            t_along_traj = self.time - self.trajectory_start_time
            state = self._get_trajectory_state(self.current_trajectory, t_along_traj)
            
            if state is not None:
                # Update ego state from trajectory
                self.ego.s = state.s
                self.ego.d = state.d
                self.ego.s_dot = state.s_dot
                self.ego.d_dot = state.d_dot
                self.ego.s_ddot = state.s_ddot
                self.ego.d_ddot = state.d_ddot
                
                # Convert back to Cartesian
                self.ego.x, self.ego.y = frenet_to_cartesian(
                    self.ego.s, self.ego.d, self.ref_path
                )
                self.ego.speed = self.ego.s_dot
                self.ego.lane = round(self.ego.d / self.config.lane_width)
        else:
            # Fallback: simple kinematic update
            target_speed = command.target_speed
            speed_diff = target_speed - self.ego.speed
            max_accel = 2.0  # m/s²
            accel = np.clip(speed_diff / dt, -max_accel, max_accel)
            self.ego.speed += accel * dt
            self.ego.s += self.ego.speed * dt
            
            # Update Cartesian from Frenet
            self.ego.x, self.ego.y = frenet_to_cartesian(
                self.ego.s, self.ego.d, self.ref_path
            )
        
        # Update traffic positions (they follow the curved road too)
        for v in self.traffic:
            v.s += v.speed * dt
            # Convert to Cartesian following the road
            v.x, v.y = frenet_to_cartesian(v.s, v.d, self.ref_path)
        
        self.time += dt
    
    def _get_trajectory_state(self, traj: Trajectory, t: float) -> Optional[TrajectoryState]:
        """Get interpolated state from trajectory at time t."""
        if not traj.states:
            return None
        
        if t <= 0:
            return traj.states[0]
        
        if t >= traj.T:
            return traj.states[-1]
        
        # Find bracketing states
        for i in range(len(traj.states) - 1):
            if traj.states[i].t <= t <= traj.states[i + 1].t:
                # Linear interpolation
                s1 = traj.states[i]
                s2 = traj.states[i + 1]
                alpha = (t - s1.t) / (s2.t - s1.t) if s2.t > s1.t else 0
                
                return TrajectoryState(
                    t=t,
                    s=s1.s + alpha * (s2.s - s1.s),
                    d=s1.d + alpha * (s2.d - s1.d),
                    s_dot=s1.s_dot + alpha * (s2.s_dot - s1.s_dot),
                    d_dot=s1.d_dot + alpha * (s2.d_dot - s1.d_dot),
                    s_ddot=s1.s_ddot + alpha * (s2.s_ddot - s1.s_ddot),
                    d_ddot=s1.d_ddot + alpha * (s2.d_ddot - s1.d_ddot)
                )
        
        return traj.states[-1]
    
    def get_state_string(self) -> str:
        """Get a string representation of current state."""
        behavior = "none"
        if self.current_command:
            behavior = self.current_command.behavior.value
        
        return (f"t={self.time:5.1f}s | s={self.ego.s:6.1f}m | "
                f"d={self.ego.d:5.2f}m | v={self.ego.speed:5.1f}m/s | {behavior}")


# =============================================================================
# Main Simulation Loop
# =============================================================================

def run_simulation(scenario: Scenario, duration: float, visualize: bool = True):
    """
    Run the simulation with behavior planner and trajectory planning.
    
    Args:
        scenario: Test scenario to run
        duration: Simulation duration in seconds
        visualize: Whether to show visualization
    """
    from behavior_tree import BehaviorPlanner
    
    # Create simulator and planner
    sim = Simulator()
    sim.setup_scenario(scenario)
    planner = BehaviorPlanner()
    
    print(f"\n{'='*60}")
    print(f"Running scenario: {scenario.value}")
    print("Road type: 5 S-curves")
    print(f"Duration: {duration}s")
    print(f"Visualization: {'enabled' if visualize else 'disabled'}")
    print(f"{'='*60}\n")
    
    if visualize:
        from visualizer import SimulationVisualizer
        viz = SimulationVisualizer(sim)
        viz.run(duration, planner)
    else:
        # Text-only mode
        n_steps = int(duration / sim.config.dt)
        print_interval = int(1.0 / sim.config.dt)
        
        # Track behavior changes
        behavior_log = []
        last_behavior = None
        
        for step in range(n_steps):
            # Get environment state
            env = sim.get_environment_state()
            
            # Run behavior planner
            command = planner.get_command(env)
            
            # Log behavior changes
            current_behavior = command.behavior.value
            if current_behavior != last_behavior:
                behavior_log.append((sim.time, current_behavior, sim.ego.s))
                last_behavior = current_behavior
            
            # Step simulation with trajectory planning
            sim.step(command)
            
            # Print status every second
            if step % print_interval == 0:
                print(sim.get_state_string())
        
        print(f"\n{'='*60}")
        print("Simulation complete.")
        print(f"Final position: s={sim.ego.s:.1f}m, d={sim.ego.d:.2f}m")
        print(f"Final speed: {sim.ego.speed:.1f}m/s")
        print(f"{'='*60}")
        
        # Print behavior summary
        print(f"\n{'='*60}")
        print("BEHAVIOR SUMMARY")
        print(f"{'='*60}")
        print(f"{'Time':>8}  {'Position':>10}  {'Behavior':<20}")
        print(f"{'-'*8}  {'-'*10}  {'-'*20}")
        for time, behavior, pos in behavior_log:
            behavior_display = behavior.upper().replace('_', ' ')
            print(f"{time:>7.1f}s  {pos:>9.1f}m  {behavior_display:<20}")
        print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description='Highway Driving Simulator with Trajectory Planning')
    parser.add_argument('--scenario', type=str, default='empty',
                        choices=['empty', 'follow', 'overtake'],
                        help='Scenario to run')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='Simulation duration in seconds')
    parser.add_argument('--no-viz', action='store_true',
                        help='Run without visualization')
    
    args = parser.parse_args()
    
    scenario = Scenario(args.scenario)
    run_simulation(scenario, args.duration, visualize=not args.no_viz)


if __name__ == "__main__":
    main()
