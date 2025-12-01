"""
Highway Scenario

This module sets up highway driving scenarios in CARLA's Town04 map,
which features highways suitable for testing lane keeping, vehicle following,
and lane change behaviors.

Town04 Map Features:
- Multiple highway segments with 2-3 lanes
- Highway on-ramps and off-ramps
- Long straight sections ideal for testing
"""

import carla
import numpy as np
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum, auto

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.carla_interface import CarlaInterface, create_spawn_point, VehicleState


class TestCase(Enum):
    """Available test cases for the highway scenario."""
    LANE_KEEP = auto()      # Basic lane keeping
    FOLLOW = auto()         # Vehicle following
    LANE_CHANGE = auto()    # Lane change maneuver
    FULL = auto()           # Combined test


@dataclass
class ScenarioConfig:
    """Configuration for a highway scenario."""
    name: str
    test_case: TestCase
    
    # Ego vehicle spawn
    ego_spawn_x: float
    ego_spawn_y: float
    ego_spawn_z: float = 0.3
    ego_spawn_yaw: float = 90.0
    ego_initial_speed: float = 20.0
    
    # Route (target lane sequence)
    route: List[int] = None
    
    # Other vehicles
    other_vehicles: List[Dict] = None
    
    # Duration
    max_duration: float = 60.0
    
    def __post_init__(self):
        if self.route is None:
            self.route = [1]
        if self.other_vehicles is None:
            self.other_vehicles = []


# Pre-defined scenarios for Town04 highway
HIGHWAY_SCENARIOS = {
    'lane_keep': ScenarioConfig(
        name="Lane Keeping",
        test_case=TestCase.LANE_KEEP,
        ego_spawn_x=5.0,
        ego_spawn_y=-150.0,
        ego_spawn_yaw=90.0,
        ego_initial_speed=20.0,
        route=[1, 1, 1],  # Stay in lane 1
        other_vehicles=[],  # No other vehicles
        max_duration=30.0
    ),
    
    'follow': ScenarioConfig(
        name="Vehicle Following",
        test_case=TestCase.FOLLOW,
        ego_spawn_x=5.0,
        ego_spawn_y=-150.0,
        ego_spawn_yaw=90.0,
        ego_initial_speed=20.0,
        route=[1, 1, 1],
        other_vehicles=[
            {
                'x': 5.0,
                'y': -110.0,  # 40m ahead
                'z': 0.3,
                'yaw': 90.0,
                'speed': 15.0,  # Slower than ego wants
                'speed_profile': 'varying',  # Will vary speed
                'blueprint': 'vehicle.audi.a2'
            }
        ],
        max_duration=45.0
    ),
    
    'lane_change': ScenarioConfig(
        name="Lane Change",
        test_case=TestCase.LANE_CHANGE,
        ego_spawn_x=5.0,
        ego_spawn_y=-150.0,
        ego_spawn_yaw=90.0,
        ego_initial_speed=20.0,
        route=[1, 0, 0],  # Start lane 1, change to lane 0
        other_vehicles=[
            # Slow vehicle ahead in lane 1
            {
                'x': 5.0,
                'y': -100.0,  # 50m ahead
                'z': 0.3,
                'yaw': 90.0,
                'speed': 15.0,
                'speed_profile': 'constant',
                'blueprint': 'vehicle.audi.a2'
            },
            # Vehicle in lane 0 (target lane) - leaves gap
            {
                'x': 8.5,  # Lane 0 (right of lane 1)
                'y': -80.0,  # Well ahead
                'z': 0.3,
                'yaw': 90.0,
                'speed': 25.0,
                'speed_profile': 'constant',
                'blueprint': 'vehicle.tesla.model3'
            }
        ],
        max_duration=60.0
    ),
    
    'full': ScenarioConfig(
        name="Full Test",
        test_case=TestCase.FULL,
        ego_spawn_x=5.0,
        ego_spawn_y=-200.0,
        ego_spawn_yaw=90.0,
        ego_initial_speed=20.0,
        route=[1, 1, 0, 0, 1, 1],  # Multiple lane changes
        other_vehicles=[
            # Vehicle ahead in lane 1
            {
                'x': 5.0,
                'y': -150.0,
                'z': 0.3,
                'yaw': 90.0,
                'speed': 18.0,
                'speed_profile': 'varying',
                'blueprint': 'vehicle.audi.a2'
            },
            # Vehicle in lane 0
            {
                'x': 8.5,
                'y': -180.0,
                'z': 0.3,
                'yaw': 90.0,
                'speed': 22.0,
                'speed_profile': 'constant',
                'blueprint': 'vehicle.tesla.model3'
            },
            # Another vehicle in lane 0 (far ahead)
            {
                'x': 8.5,
                'y': -50.0,
                'z': 0.3,
                'yaw': 90.0,
                'speed': 20.0,
                'speed_profile': 'constant',
                'blueprint': 'vehicle.nissan.patrol'
            }
        ],
        max_duration=120.0
    )
}


class HighwayScenario:
    """
    Manages highway driving scenarios in CARLA.
    """
    
    def __init__(self, carla_interface: CarlaInterface, config: ScenarioConfig):
        """
        Initialize highway scenario.
        
        Args:
            carla_interface: Connected CarlaInterface instance
            config: Scenario configuration
        """
        self.carla = carla_interface
        self.config = config
        
        self.npc_vehicles: List[carla.Vehicle] = []
        self.npc_configs: List[Dict] = []
        
        self.start_time: float = 0.0
        self.elapsed_time: float = 0.0
        
    def setup(self) -> bool:
        """
        Set up the scenario: spawn vehicles, configure initial state.
        
        Returns:
            True if setup successful
        """
        print(f"\nSetting up scenario: {self.config.name}")
        print("=" * 50)
        
        # Load Town04 (highway map)
        if not self.carla.load_map('Town04'):
            print("Failed to load Town04 map")
            return False
        
        # Set synchronous mode for deterministic simulation
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02  # 50 Hz
        self.carla.world.apply_settings(settings)
        
        # Spawn ego vehicle
        ego_spawn = create_spawn_point(
            self.config.ego_spawn_x,
            self.config.ego_spawn_y,
            self.config.ego_spawn_z,
            self.config.ego_spawn_yaw
        )
        
        if not self.carla.spawn_ego_vehicle(ego_spawn):
            print("Failed to spawn ego vehicle")
            return False
        
        # Set initial ego speed
        self.carla.set_vehicle_velocity(
            self.carla.ego_vehicle,
            self.config.ego_initial_speed
        )
        
        # Spawn NPC vehicles
        for npc_config in self.config.other_vehicles:
            npc_spawn = create_spawn_point(
                npc_config['x'],
                npc_config['y'],
                npc_config['z'],
                npc_config['yaw']
            )
            
            vehicle = self.carla.spawn_vehicle(
                npc_spawn,
                npc_config.get('blueprint', 'vehicle.audi.a2')
            )
            
            if vehicle is not None:
                # Set initial velocity
                self.carla.set_vehicle_velocity(vehicle, npc_config['speed'])
                self.npc_vehicles.append(vehicle)
                self.npc_configs.append(npc_config)
                print(f"  Spawned NPC at ({npc_config['x']:.1f}, {npc_config['y']:.1f})")
        
        # Tick to apply changes
        self.carla.tick()
        time.sleep(0.5)
        
        print(f"Setup complete: {len(self.npc_vehicles)} NPC vehicles spawned")
        return True
    
    def update_npcs(self, elapsed_time: float):
        """
        Update NPC vehicle behaviors.
        
        Args:
            elapsed_time: Time since scenario start
        """
        for vehicle, config in zip(self.npc_vehicles, self.npc_configs):
            if vehicle is None or not vehicle.is_alive:
                continue
            
            speed_profile = config.get('speed_profile', 'constant')
            base_speed = config['speed']
            
            if speed_profile == 'constant':
                target_speed = base_speed
            elif speed_profile == 'varying':
                # Sinusoidal speed variation
                variation = 3.0 * np.sin(0.5 * elapsed_time)
                target_speed = base_speed + variation
            elif speed_profile == 'accelerating':
                target_speed = base_speed + 0.5 * elapsed_time
            elif speed_profile == 'decelerating':
                target_speed = max(5.0, base_speed - 0.5 * elapsed_time)
            else:
                target_speed = base_speed
            
            self.carla.set_vehicle_velocity(vehicle, target_speed)
    
    def get_route(self) -> List[int]:
        """Get the target lane sequence."""
        return self.config.route
    
    def check_completion(self, ego_state: VehicleState) -> Tuple[bool, str]:
        """
        Check if scenario is complete.
        
        Args:
            ego_state: Current ego vehicle state
            
        Returns:
            (is_complete, reason)
        """
        # Check time limit
        if self.elapsed_time >= self.config.max_duration:
            return True, "Time limit reached"
        
        # Check if ego has traveled sufficient distance
        # (Simple check - could be more sophisticated)
        if ego_state.y > 200.0:  # Traveled ~350m from start
            return True, "Route completed"
        
        return False, ""
    
    def check_collision(self, ego_state: VehicleState) -> bool:
        """
        Check if ego vehicle has collided with anything.
        
        Args:
            ego_state: Current ego state
            
        Returns:
            True if collision detected
        """
        # Simple distance-based collision check
        ego_radius = 2.5  # Approximate vehicle radius
        
        for vehicle in self.npc_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue
            
            npc_state = self.carla.get_vehicle_state(vehicle)
            dx = ego_state.x - npc_state.x
            dy = ego_state.y - npc_state.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < ego_radius * 2:  # Both vehicles
                return True
        
        return False
    
    def cleanup(self):
        """Clean up scenario resources."""
        print("\nCleaning up scenario...")
        
        # Reset synchronous mode
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        self.carla.world.apply_settings(settings)
        
        # Destroy NPC vehicles
        for vehicle in self.npc_vehicles:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()
        self.npc_vehicles.clear()
        self.npc_configs.clear()
        
        # Destroy ego vehicle through carla interface
        self.carla.cleanup()


def get_scenario(name: str) -> Optional[ScenarioConfig]:
    """
    Get a pre-defined scenario configuration.
    
    Args:
        name: Scenario name ('lane_keep', 'follow', 'lane_change', 'full')
        
    Returns:
        ScenarioConfig, or None if not found
    """
    return HIGHWAY_SCENARIOS.get(name.lower())


def list_scenarios() -> List[str]:
    """Get list of available scenario names."""
    return list(HIGHWAY_SCENARIOS.keys())


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Highway Scenario Module")
    print("=" * 50)
    print("\nAvailable scenarios:")
    for name, config in HIGHWAY_SCENARIOS.items():
        print(f"  {name}: {config.name}")
        print(f"    - Test case: {config.test_case.name}")
        print(f"    - Duration: {config.max_duration}s")
        print(f"    - NPC vehicles: {len(config.other_vehicles)}")
    
    print("\nTo test with CARLA, run scenario_runner.py")
