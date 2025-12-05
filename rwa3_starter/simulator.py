"""
simulator.py - Standalone Highway Simulator for Behavioral Planning

This simulator provides a simple highway environment for testing
behavior tree implementations without requiring CARLA.

Usage:
    python simulator.py --scenario empty
    python simulator.py --scenario follow
    python simulator.py --scenario overtake
    python simulator.py --no-viz --scenario overtake --duration 30

DO NOT MODIFY THIS FILE.
"""

import argparse
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum

from bt_framework import EnvironmentState, BehaviorCommand, BehaviorType


@dataclass
class Vehicle:
    """Represents a vehicle in the simulation."""
    x: float = 0.0          # Longitudinal position (m)
    y: float = 0.0          # Lateral position (m)
    speed: float = 0.0      # Speed (m/s)
    lane: int = 0           # Lane index (0 = center)
    length: float = 4.5     # Vehicle length (m)
    width: float = 2.0      # Vehicle width (m)


@dataclass
class SimulationConfig:
    """Configuration for the simulation."""
    lane_width: float = 3.5
    speed_limit: float = 31.0  # m/s (~70 mph)
    dt: float = 0.1            # Time step (s)
    
    # Detection parameters
    detection_range: float = 100.0  # How far ahead to detect vehicles
    lane_change_gap: float = 25.0   # Required gap for lane change
    
    # Road parameters
    num_lanes: int = 3
    road_length: float = 2000.0


class Scenario(Enum):
    """Available test scenarios."""
    EMPTY = "empty"
    FOLLOW = "follow"
    OVERTAKE = "overtake"


class Simulator:
    """
    Highway driving simulator.
    
    Simulates ego vehicle and traffic on a multi-lane highway.
    Provides environment state for behavior tree decisions.
    """
    
    def __init__(self, config: SimulationConfig = None):
        self.config = config or SimulationConfig()
        self.time = 0.0
        
        # Ego vehicle
        self.ego = Vehicle(x=0.0, y=0.0, speed=25.0, lane=0)
        
        # Traffic vehicles
        self.traffic: List[Vehicle] = []
        
        # Current behavior command
        self.current_command: Optional[BehaviorCommand] = None
        
        # Lane change tracking
        self.lane_change_progress = 0.0
        self.target_lane = 0
    
    def setup_scenario(self, scenario: Scenario):
        """Set up a test scenario."""
        self.time = 0.0
        self.ego = Vehicle(x=0.0, y=0.0, speed=22.0, lane=0)
        self.traffic.clear()
        self.lane_change_progress = 0.0
        self.target_lane = 0
        
        if scenario == Scenario.EMPTY:
            # No traffic - just cruise
            pass
            
        elif scenario == Scenario.FOLLOW:
            # Simple follow: All 3 red vehicles at same speed (22 m/s)
            # Ego follows at 21 m/s (lead - 1), relative speed = 1 m/s
            # Adjacent vehicles start behind, catch up slowly, stay blocking for 30s+
            
            # Lead vehicle in center lane
            lead = Vehicle(
                x=20.0,
                y=0.0,
                speed=20.0,
                lane=0
            )
            self.traffic.append(lead)
            
            # Left lane vehicle - starts 20m behind ego
            # At 1 m/s relative speed: catches up in 20s, clears (25m ahead) at 45s
            left_vehicle = Vehicle(
                x=-10.0,
                y=self.config.lane_width,
                speed=20.0,
                lane=1
            )
            self.traffic.append(left_vehicle)
            
            # Left lane vehicle - starts 20m behind ego
            # At 1 m/s relative speed: catches up in 20s, clears (25m ahead) at 45s
            left_vehicle2 = Vehicle(
                x=0.0,
                y=self.config.lane_width,
                speed=20.0,
                lane=1
            )
            self.traffic.append(left_vehicle2)
            
            left_vehicle3 = Vehicle(
                x=10.0,
                y=self.config.lane_width,
                speed=20.0,
                lane=1
            )
            self.traffic.append(left_vehicle3)
            
            # Right lane vehicle - starts 15m behind ego
            right_vehicle = Vehicle(
                x=-10.0,
                y=-self.config.lane_width,
                speed=20.0,
                lane=-1
            )
            self.traffic.append(right_vehicle)
            
            # Right lane vehicle - starts 15m behind ego
            right_vehicle2 = Vehicle(
                x=0.0,
                y=-self.config.lane_width,
                speed=20.0,
                lane=-1
            )
            self.traffic.append(right_vehicle2)
            
            # Right lane vehicle - starts 15m behind ego
            right_vehicle3 = Vehicle(
                x=10.0,
                y=-self.config.lane_width,
                speed=20.0,
                lane=-1
            )
            self.traffic.append(right_vehicle3)
            
            # Set ego starting speed to match follow speed
            self.ego.speed = 25.0
            
        elif scenario == Scenario.OVERTAKE:
            # Phase 1: Slow vehicle in center lane, left clear -> change left
            slow_center = Vehicle(
                x=40.0,
                y=0.0,
                speed=20.0,  # Slow vehicle in center
                lane=0
            )
            self.traffic.append(slow_center)
            
            # Block right lane initially
            right_blocker = Vehicle(
                x=35.0,
                y=-self.config.lane_width,
                speed=24.0,
                lane=-1
            )
            self.traffic.append(right_blocker)
            
            # Phase 2: Slow vehicle in left lane further ahead -> change right
            slow_left = Vehicle(
                x=150.0,  # Further down the road
                y=self.config.lane_width,
                speed=18.0,  # Even slower in left lane
                lane=1
            )
            self.traffic.append(slow_left)
    
    def get_environment_state(self) -> EnvironmentState:
        """
        Get current environment state for behavior tree.
        
        Returns:
            EnvironmentState with all perception data
        """
        env = EnvironmentState()
        
        # Ego state
        env.ego_speed = self.ego.speed
        env.ego_d = self.ego.y
        env.speed_limit = self.config.speed_limit
        
        # Lane existence
        current_lane = round(self.ego.y / self.config.lane_width)
        env.left_lane_exists = current_lane < (self.config.num_lanes - 1) // 2
        env.right_lane_exists = current_lane > -(self.config.num_lanes - 1) // 2
        
        # Find vehicle ahead in same lane
        env.vehicle_ahead = False
        env.vehicle_ahead_distance = 100.0
        env.vehicle_ahead_speed = 0.0
        
        for v in self.traffic:
            # Check if in same lane
            v_lane = round(v.y / self.config.lane_width)
            ego_lane = round(self.ego.y / self.config.lane_width)
            
            if v_lane == ego_lane:
                dist = v.x - self.ego.x
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
        target_y = target_lane * self.config.lane_width
        
        for v in self.traffic:
            v_lane = round(v.y / self.config.lane_width)
            if v_lane == target_lane:
                # Check longitudinal gap
                dist = abs(v.x - self.ego.x)
                if dist < self.config.lane_change_gap:
                    return False
        
        return True
    
    def apply_command(self, command: BehaviorCommand):
        """Apply a behavior command to update vehicle state."""
        self.current_command = command
        
        # Handle lane changes
        if command.behavior == BehaviorType.LANE_CHANGE_LEFT:
            self.target_lane = round(self.ego.y / self.config.lane_width) + 1
        elif command.behavior == BehaviorType.LANE_CHANGE_RIGHT:
            self.target_lane = round(self.ego.y / self.config.lane_width) - 1
        else:
            self.target_lane = round(self.ego.y / self.config.lane_width)
    
    def step(self):
        """Advance simulation by one time step."""
        dt = self.config.dt
        
        if self.current_command is not None:
            cmd = self.current_command
            
            # Update speed towards target
            speed_diff = cmd.target_speed - self.ego.speed
            max_accel = 3.0  # m/s²
            accel = np.clip(speed_diff / 2.0, -max_accel, max_accel)
            self.ego.speed += accel * dt
            self.ego.speed = max(0, self.ego.speed)
            
            # Update lateral position towards target
            target_y = self.target_lane * self.config.lane_width
            y_diff = target_y - self.ego.y
            max_lateral_speed = 1.5  # m/s
            lateral_speed = np.clip(y_diff / 2.0, -max_lateral_speed, max_lateral_speed)
            self.ego.y += lateral_speed * dt
        
        # Update ego position
        self.ego.x += self.ego.speed * dt
        
        # Update traffic positions
        for v in self.traffic:
            v.x += v.speed * dt
        
        self.time += dt
    
    def get_state_string(self) -> str:
        """Get a string representation of current state."""
        behavior = "none"
        if self.current_command:
            behavior = self.current_command.behavior.value
        
        return (f"t={self.time:5.1f}s | x={self.ego.x:6.1f}m | "
                f"y={self.ego.y:5.2f}m | v={self.ego.speed:5.1f}m/s | {behavior}")


def run_simulation(scenario: Scenario, duration: float, visualize: bool = True):
    """
    Run the simulation with the behavior planner.
    
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
        print_interval = int(1.0 / sim.config.dt)  # Print every second
        
        for step in range(n_steps):
            # Get environment state
            env = sim.get_environment_state()
            
            # Get behavior command from planner
            command = planner.get_command(env)
            
            # Apply command and step simulation
            sim.apply_command(command)
            sim.step()
            
            # Print status periodically
            if step % print_interval == 0:
                print(sim.get_state_string())
        
        print(f"\n{'='*60}")
        print("Simulation complete!")
        print(f"Final position: x={sim.ego.x:.1f}m, y={sim.ego.y:.2f}m")
        print(f"Final speed: {sim.ego.speed:.1f}m/s")
        print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description='Highway Driving Simulator')
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
