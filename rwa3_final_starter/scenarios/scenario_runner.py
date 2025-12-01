#!/usr/bin/env python3
"""
Scenario Runner

Main script to run autonomous driving scenarios in CARLA.
This integrates the behavioral planner, trajectory planner, and controller.

Usage:
    python -m scenarios.scenario_runner --scenario highway --test lane_keep
    python -m scenarios.scenario_runner --scenario highway --test follow --visualize
    python -m scenarios.scenario_runner --scenario highway --test lane_change
    python -m scenarios.scenario_runner --help
"""

import argparse
import yaml
import time
import numpy as np
from pathlib import Path
from typing import Dict, Optional
from dataclasses import dataclass

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.carla_interface import CarlaInterface
from src.perception import PerceptionModule, PerceptionData
from src.behavioral_planner import BehavioralPlanner
from src.trajectory_planner import TrajectoryPlanner, Trajectory
from src.controller import VehicleController
from src.behavior_tree import BehavioralCommand

from scenarios.highway_scenario import (
    HighwayScenario, 
    get_scenario, 
    list_scenarios,
    TestCase
)

# Try to import visualization
try:
    from src.visualization import PygameVisualizer
    VIS_AVAILABLE = True
except ImportError:
    VIS_AVAILABLE = False


@dataclass
class RunStatistics:
    """Statistics collected during a scenario run."""
    total_time: float = 0.0
    distance_traveled: float = 0.0
    num_collisions: int = 0
    max_lateral_deviation: float = 0.0
    max_speed: float = 0.0
    min_speed: float = float('inf')
    max_lateral_accel: float = 0.0
    max_jerk: float = 0.0
    lane_changes_completed: int = 0
    planning_failures: int = 0
    
    def print_summary(self):
        """Print statistics summary."""
        print("\n" + "=" * 50)
        print("RUN STATISTICS")
        print("=" * 50)
        print(f"Total time: {self.total_time:.2f} s")
        print(f"Distance traveled: {self.distance_traveled:.1f} m")
        print(f"Collisions: {self.num_collisions}")
        print(f"Max speed: {self.max_speed:.1f} m/s")
        print(f"Min speed: {self.min_speed:.1f} m/s")
        print(f"Max lateral deviation: {self.max_lateral_deviation:.2f} m")
        print(f"Max lateral acceleration: {self.max_lateral_accel:.2f} m/s²")
        print(f"Lane changes completed: {self.lane_changes_completed}")
        print(f"Planning failures: {self.planning_failures}")
        print("=" * 50)


def load_config(config_path: Optional[str] = None) -> Dict:
    """Load configuration from YAML file."""
    if config_path is None:
        config_path = Path(__file__).parent.parent / 'config' / 'planner_config.yaml'
    
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def run_scenario(scenario_name: str, test_case: str, 
                 visualize: bool = False,
                 config_path: Optional[str] = None) -> RunStatistics:
    """
    Run a complete scenario.
    
    Args:
        scenario_name: Name of the scenario ('highway')
        test_case: Test case name ('lane_keep', 'follow', 'lane_change', 'full')
        visualize: Enable visualization
        config_path: Path to configuration file
        
    Returns:
        RunStatistics from the scenario
    """
    print("\n" + "=" * 60)
    print(f"ENPM818Z Final Project - Scenario Runner")
    print(f"Scenario: {scenario_name}, Test: {test_case}")
    print("=" * 60)
    
    # Load configuration
    config = load_config(config_path)
    
    # Get scenario configuration
    scenario_config = get_scenario(test_case)
    if scenario_config is None:
        print(f"Unknown test case: {test_case}")
        print(f"Available: {list_scenarios()}")
        return RunStatistics()
    
    # Initialize statistics
    stats = RunStatistics()
    
    # Initialize CARLA interface
    carla_config = config.get('carla', {})
    carla_interface = CarlaInterface(
        host=carla_config.get('host', 'localhost'),
        port=carla_config.get('port', 2000),
        timeout=carla_config.get('timeout', 10.0)
    )
    
    if not carla_interface.connect():
        print("Failed to connect to CARLA. Is the server running?")
        return stats
    
    # Create scenario
    scenario = HighwayScenario(carla_interface, scenario_config)
    
    try:
        # Setup scenario
        if not scenario.setup():
            print("Failed to set up scenario")
            return stats
        
        # Initialize modules
        perception = PerceptionModule(carla_interface)
        
        bp_config = config.get('behavioral_planner', {})
        behavioral_planner = BehavioralPlanner(bp_config)
        behavioral_planner.set_route(scenario.get_route())
        
        tp_config = config.get('trajectory_planner', {})
        trajectory_planner = TrajectoryPlanner(tp_config)
        
        ctrl_config = config.get('controller', {})
        controller = VehicleController(ctrl_config)
        
        # Initialize visualization
        visualizer = None
        if visualize and VIS_AVAILABLE:
            vis_config = config.get('visualization', {})
            visualizer = PygameVisualizer(
                width=vis_config.get('screen_width', 1280),
                height=vis_config.get('screen_height', 720)
            )
        
        # Main loop
        print("\nStarting scenario execution...")
        print("Press Ctrl+C to stop\n")
        
        start_time = time.time()
        prev_position = None
        current_trajectory: Optional[Trajectory] = None
        
        planning_rate = 10  # Hz
        control_rate = 50   # Hz
        
        tick_count = 0
        running = True
        
        while running:
            # Tick CARLA
            sim_time = carla_interface.tick()
            tick_count += 1
            
            # Get perception data
            perception_data = perception.update(sim_time)
            ego_state = perception_data.ego_state
            
            # Update NPC vehicles
            elapsed = time.time() - start_time
            scenario.elapsed_time = elapsed
            scenario.update_npcs(elapsed)
            
            # Update statistics
            stats.max_speed = max(stats.max_speed, ego_state.v)
            stats.min_speed = min(stats.min_speed, ego_state.v)
            
            if prev_position is not None:
                dx = ego_state.x - prev_position[0]
                dy = ego_state.y - prev_position[1]
                stats.distance_traveled += np.sqrt(dx**2 + dy**2)
            prev_position = (ego_state.x, ego_state.y)
            
            # Check for collision
            if scenario.check_collision(ego_state):
                print("COLLISION DETECTED!")
                stats.num_collisions += 1
                running = False
                continue
            
            # Behavioral planning (lower rate)
            if tick_count % (control_rate // planning_rate) == 0:
                # Convert perception to dict format
                ego_dict = {
                    'x': ego_state.x,
                    'y': ego_state.y,
                    'theta': ego_state.theta,
                    'v': ego_state.v,
                    'lane_id': ego_state.lane_id
                }
                
                obstacles = [obs.to_dict() for obs in perception_data.obstacles]
                lane_info = perception_data.lane_info.to_dict() if perception_data.lane_info else {}
                
                # Get behavioral command
                behavioral_command = behavioral_planner.plan(
                    ego_dict, obstacles, lane_info, 
                    perception_data.reference_path
                )
                
                # Generate trajectory
                trajectory, success = trajectory_planner.plan(
                    ego_dict, behavioral_command,
                    obstacles, perception_data.reference_path
                )
                
                if success:
                    current_trajectory = trajectory
                else:
                    stats.planning_failures += 1
                    if current_trajectory is None:
                        # Emergency: create simple forward trajectory
                        print("Warning: Planning failed, using emergency trajectory")
            
            # Control (every tick)
            if current_trajectory is not None:
                ego_dict = {
                    'x': ego_state.x,
                    'y': ego_state.y,
                    'theta': ego_state.theta,
                    'v': ego_state.v
                }
                
                control = controller.compute_control(
                    ego_dict, current_trajectory, sim_time
                )
                carla_interface.apply_control(control)
            
            # Update visualization
            if visualizer is not None:
                ego_dict = {
                    'x': ego_state.x,
                    'y': ego_state.y,
                    'theta': ego_state.theta,
                    'v': ego_state.v
                }
                obstacles_vis = [obs.to_dict() for obs in perception_data.obstacles]
                
                info = {
                    'Time': f"{elapsed:.1f}s",
                    'Maneuver': behavioral_command.maneuver if 'behavioral_command' in dir() else 'N/A',
                    'Target Speed': f"{behavioral_command.target_speed:.1f}" if 'behavioral_command' in dir() else 'N/A'
                }
                
                if not visualizer.update(ego_dict, obstacles_vis,
                                         current_trajectory,
                                         perception_data.reference_path,
                                         info):
                    running = False
                    continue
            
            # Update spectator camera
            carla_interface.update_spectator()
            
            # Check completion
            complete, reason = scenario.check_completion(ego_state)
            if complete:
                print(f"Scenario complete: {reason}")
                running = False
            
            # Print status periodically
            if tick_count % 100 == 0:
                print(f"  t={elapsed:.1f}s, pos=({ego_state.x:.1f}, {ego_state.y:.1f}), "
                      f"v={ego_state.v:.1f} m/s, lane={ego_state.lane_id}")
        
        # Final statistics
        stats.total_time = time.time() - start_time
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        stats.total_time = time.time() - start_time
        
    finally:
        # Cleanup
        if visualizer is not None:
            visualizer.close()
        scenario.cleanup()
    
    # Print summary
    stats.print_summary()
    
    return stats


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='ENPM818Z Final Project - Scenario Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m scenarios.scenario_runner --scenario highway --test lane_keep
  python -m scenarios.scenario_runner --scenario highway --test follow --visualize
  python -m scenarios.scenario_runner --scenario highway --test lane_change
  python -m scenarios.scenario_runner --list

Available test cases:
  lane_keep    - Basic lane keeping (no obstacles)
  follow       - Vehicle following with varying lead speed
  lane_change  - Lane change to pass slow vehicle
  full         - Combined test with multiple maneuvers
        """
    )
    
    parser.add_argument('--scenario', '-s', type=str, default='highway',
                        help='Scenario type (default: highway)')
    parser.add_argument('--test', '-t', type=str, default='lane_keep',
                        help='Test case (lane_keep, follow, lane_change, full)')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Enable pygame visualization')
    parser.add_argument('--config', '-c', type=str, default=None,
                        help='Path to configuration file')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List available scenarios and exit')
    
    args = parser.parse_args()
    
    if args.list:
        print("Available scenarios:")
        for name in list_scenarios():
            print(f"  {name}")
        return
    
    # Run scenario
    stats = run_scenario(
        args.scenario,
        args.test,
        visualize=args.visualize,
        config_path=args.config
    )
    
    # Return exit code based on success
    if stats.num_collisions > 0:
        exit(1)
    exit(0)


if __name__ == '__main__':
    main()
