"""
carla_interface.py - CARLA Interface for Behavioral Planning

This module provides the bridge between CARLA and the behavior tree planner.
It handles:
    - Connection to CARLA server
    - Ego vehicle and traffic spawning
    - Perception: Converting CARLA world state to EnvironmentState
    - Control: Applying BehaviorCommand via low-level controllers

Key Classes:
    - CarlaInterface: Main interface for CARLA communication
    - ScenarioManager: Sets up test scenarios with traffic

Usage:
    interface = CarlaInterface()
    interface.connect()
    interface.setup_scenario(Scenario.OVERTAKE)
    
    while running:
        env_state = interface.get_environment_state()
        command = planner.get_command(env_state)
        interface.apply_command(command)
        interface.tick()

DO NOT MODIFY THIS FILE.
"""

import carla
import numpy as np
import time
import weakref
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass
from enum import Enum

# Import the behavior tree framework types
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bt_framework import EnvironmentState, BehaviorCommand, BehaviorType
from carla_controller import VehicleController, EmergencyController


# =============================================================================
# Configuration
# =============================================================================

# Known good highway spawn points for Town04
# These are on the main highway loop with 3+ lanes
TOWN04_HIGHWAY_SPAWNS = [
    # Highway section going east (good long straight)
    carla.Transform(carla.Location(x=5.0, y=-170.0, z=0.3), carla.Rotation(yaw=90)),
    carla.Transform(carla.Location(x=-5.0, y=-180.0, z=0.3), carla.Rotation(yaw=90)),
    # Highway section going west  
    carla.Transform(carla.Location(x=400.0, y=-180.0, z=0.3), carla.Rotation(yaw=-90)),
]

@dataclass
class CarlaConfig:
    """Configuration for CARLA interface."""
    host: str = 'localhost'
    port: int = 2000
    timeout: float = 10.0
    
    # Simulation settings
    synchronous_mode: bool = True
    fixed_delta_seconds: float = 0.1  # 10 Hz
    
    # Map settings - Town04 is the highway map
    # Will force load Town04 if available
    target_map: str = 'Town04'
    preferred_maps: Tuple[str, ...] = ('Town04', 'Town06', 'Town05', 'Town03', 'Town10HD')
    
    # Vehicle settings
    ego_vehicle_model: str = 'vehicle.tesla.model3'
    traffic_vehicle_model: str = 'vehicle.audi.a2'
    
    # Road parameters (should match bt_nodes.py)
    lane_width: float = 3.5
    speed_limit: float = 15.0  # m/s (~34 mph) - reduced for stable lane following
    
    # Detection parameters
    detection_range: float = 100.0
    lane_change_gap: float = 25.0


class Scenario(Enum):
    """Available test scenarios (mirrors simulator.py)."""
    EMPTY = "empty"
    FOLLOW = "follow"
    OVERTAKE = "overtake"


# =============================================================================
# CARLA Interface
# =============================================================================

class CarlaInterface:
    """
    Main interface for CARLA communication.
    
    Handles connection, vehicle spawning, perception, and control.
    """
    
    def __init__(self, config: CarlaConfig = None):
        """
        Initialize the CARLA interface.
        
        Args:
            config: Configuration settings (uses defaults if None)
        """
        self.config = config or CarlaConfig()
        
        # CARLA objects (initialized on connect)
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.map: Optional[carla.Map] = None
        self.blueprint_library: Optional[carla.BlueprintLibrary] = None
        
        # Vehicles
        self.ego_vehicle: Optional[carla.Vehicle] = None
        self.traffic_vehicles: List[carla.Vehicle] = []
        
        # Controllers
        self.controller: Optional[VehicleController] = None
        self.emergency_controller = EmergencyController(ttc_threshold=2.0)
        
        # State tracking
        self.current_command: Optional[BehaviorCommand] = None
        self.target_lane_offset: float = 0.0
        self.simulation_time: float = 0.0
        self._lane_change_initiated: bool = False
        
        # Spectator for camera following
        self.spectator: Optional[carla.Actor] = None
        
        # Original settings (for cleanup)
        self._original_settings = None
    
    def connect(self) -> bool:
        """
        Connect to CARLA server.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            print(f"Connecting to CARLA server at {self.config.host}:{self.config.port}...")
            
            self.client = carla.Client(self.config.host, self.config.port)
            self.client.set_timeout(self.config.timeout)
            
            # Check connection
            version = self.client.get_server_version()
            print(f"Connected to CARLA {version}")
            
            # IMPORTANT: Disable synchronous mode before loading a new map
            # This prevents crashes when switching maps
            try:
                world = self.client.get_world()
                settings = world.get_settings()
                if settings.synchronous_mode:
                    print("Disabling synchronous mode before map load...")
                    settings.synchronous_mode = False
                    world.apply_settings(settings)
                    # Also reset traffic manager
                    traffic_manager = self.client.get_trafficmanager()
                    traffic_manager.set_synchronous_mode(False)
            except:
                pass  # World might not exist yet
            
            # Get available maps
            available_maps = self.client.get_available_maps()
            available_map_names = [m.split('/')[-1] for m in available_maps]
            
            # Check if Town04 (highway map) is available
            town04_available = any('Town04' in m for m in available_maps)
            
            # Get current map to avoid unnecessary reload
            current_world = self.client.get_world()
            current_map_name = current_world.get_map().name.split('/')[-1]
            
            if town04_available:
                print(f"✓ Town04 (highway map) is available")
                if 'Town04' in current_map_name:
                    print(f"✓ Already on Town04, skipping reload")
                    self.world = current_world
                else:
                    print(f"Loading Town04...")
                    self.world = self.client.load_world('Town04')
                    # Wait for world to be ready
                    time.sleep(2.0)
                print(f"✓ Town04 ready!")
            else:
                print(f"⚠ Town04 not available. Available maps: {', '.join(sorted(available_map_names))}")
                # Try other preferred maps
                map_loaded = False
                for preferred_map in self.config.preferred_maps:
                    if preferred_map == 'Town04':
                        continue  # Already tried
                    if preferred_map in current_map_name:
                        print(f"Already on {preferred_map}, skipping reload")
                        self.world = current_world
                        map_loaded = True
                        break
                    target_map = f'/Game/Carla/Maps/{preferred_map}'
                    if target_map in available_maps:
                        print(f"Loading fallback map: {preferred_map}")
                        try:
                            self.world = self.client.load_world(preferred_map)
                            time.sleep(2.0)
                            map_loaded = True
                            break
                        except Exception as e:
                            print(f"  Failed to load {preferred_map}: {e}")
                            continue
                
                if not map_loaded:
                    print("Using current map")
                    self.world = self.client.get_world()
            
            self.map = self.world.get_map()
            self.blueprint_library = self.world.get_blueprint_library()
            self.spectator = self.world.get_spectator()
            
            current_map = self.map.name.split('/')[-1]
            print(f"Using map: {current_map}")
            
            if 'Town04' in current_map:
                print("✓ Highway map loaded - ideal for lane change scenarios!")
            
            # Configure synchronous mode
            self._setup_synchronous_mode()
            
            print("CARLA connection established successfully!")
            return True
            
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _setup_synchronous_mode(self):
        """Configure synchronous mode for deterministic simulation."""
        self._original_settings = self.world.get_settings()
        
        settings = self.world.get_settings()
        settings.synchronous_mode = self.config.synchronous_mode
        settings.fixed_delta_seconds = self.config.fixed_delta_seconds
        self.world.apply_settings(settings)
        
        # Also set traffic manager to synchronous mode
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(self.config.synchronous_mode)
    
    def spawn_ego_vehicle(self, spawn_transform: carla.Transform = None) -> bool:
        """
        Spawn the ego vehicle.
        
        Args:
            spawn_transform: Where to spawn (uses default if None)
            
        Returns:
            True if spawn successful
        """
        try:
            # Try multiple vehicle models in case some aren't available
            vehicle_models = [
                self.config.ego_vehicle_model,
                'vehicle.tesla.model3',
                'vehicle.audi.tt',
                'vehicle.bmw.grandtourer',
                'vehicle.toyota.prius',
                'vehicle.lincoln.mkz_2017',
            ]
            
            bp = None
            for model in vehicle_models:
                try:
                    bps = self.blueprint_library.filter(model)
                    if bps:
                        bp = bps[0]
                        print(f"Using ego vehicle: {bp.id}")
                        break
                except:
                    continue
            
            if bp is None:
                # Last resort: any vehicle
                bp = self.blueprint_library.filter('vehicle.*')[0]
                print(f"Using fallback ego vehicle: {bp.id}")
            
            bp.set_attribute('role_name', 'hero')
            
            # Use provided transform or find a good highway spawn
            if spawn_transform is None:
                spawn_transform = self._find_highway_spawn()
            
            # Spawn vehicle
            self.ego_vehicle = self.world.spawn_actor(bp, spawn_transform)
            
            if self.ego_vehicle is None:
                print("Failed to spawn ego vehicle!")
                return False
            
            print(f"Ego vehicle spawned at ({spawn_transform.location.x:.1f}, {spawn_transform.location.y:.1f})")
            
            # Reset lane tracking for new vehicle
            self.reset_lane_tracking()
            
            # Enable autopilot for stable lane keeping
            self.ego_vehicle.set_autopilot(True)
            
            # Configure traffic manager for ego vehicle
            traffic_manager = self.client.get_trafficmanager()
            traffic_manager.auto_lane_change(self.ego_vehicle, False)  # Disable auto lane change
            traffic_manager.distance_to_leading_vehicle(self.ego_vehicle, 5.0)
            
            # Initialize controller (for speed control only)
            self.controller = VehicleController(
                self.ego_vehicle, 
                dt=self.config.fixed_delta_seconds
            )
            
            # Let the vehicle settle
            self.world.tick()
            time.sleep(0.5)
            
            return True
            
        except Exception as e:
            print(f"Error spawning ego vehicle: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _find_highway_spawn(self) -> carla.Transform:
        """
        Find a suitable highway/multi-lane spawn point.
        
        For Town04, uses known highway spawn points.
        For other maps, searches for spawn points with multiple lanes.
        """
        current_map = self.map.name.split('/')[-1]
        
        # Use known highway spawn points for Town04
        if 'Town04' in current_map:
            print("Using Town04 highway spawn point...")
            # Try each known highway spawn point
            for spawn in TOWN04_HIGHWAY_SPAWNS:
                waypoint = self.map.get_waypoint(spawn.location)
                if waypoint and waypoint.lane_type == carla.LaneType.Driving:
                    # Verify it has multiple lanes
                    left = waypoint.get_left_lane()
                    right = waypoint.get_right_lane()
                    if left or right:
                        print(f"✓ Highway spawn point found at ({spawn.location.x:.0f}, {spawn.location.y:.0f})")
                        # Adjust spawn to be exactly on the lane center
                        adjusted_transform = waypoint.transform
                        adjusted_transform.location.z += 0.5
                        return adjusted_transform
            
            # If known points fail, fall back to searching
            print("Known spawn points invalid, searching for highway...")
        
        # Generic highway search for other maps
        spawn_points = self.map.get_spawn_points()
        
        if not spawn_points:
            print("Warning: No spawn points found, using origin")
            return carla.Transform(carla.Location(x=0, y=0, z=1))
        
        # Score each spawn point based on suitability
        best_spawn = None
        best_score = -1
        
        for sp in spawn_points:
            waypoint = self.map.get_waypoint(sp.location)
            
            if waypoint is None:
                continue
            
            # Only consider driving lanes
            if waypoint.lane_type != carla.LaneType.Driving:
                continue
            
            score = 0
            
            # Check for left lane (big bonus)
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                score += 10
                # Check for another lane to the left (even better - 3+ lanes)
                left_left = left_lane.get_left_lane()
                if left_left and left_left.lane_type == carla.LaneType.Driving:
                    score += 5
            
            # Check for right lane (big bonus)
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                score += 10
                # Check for another lane to the right
                right_right = right_lane.get_right_lane()
                if right_right and right_right.lane_type == carla.LaneType.Driving:
                    score += 5
            
            # Prefer straighter roads (check waypoints ahead)
            try:
                next_wps = waypoint.next(50.0)  # 50m ahead
                if next_wps:
                    # Road continues ahead
                    score += 3
                    next_next = next_wps[0].next(50.0)
                    if next_next:
                        score += 2  # Long straight road
            except:
                pass
            
            # Prefer roads with higher speed limits (typically highways)
            if waypoint.lane_width >= 3.0:
                score += 2
            
            if score > best_score:
                best_score = score
                best_spawn = sp
        
        if best_spawn is not None:
            # Get waypoint info for logging
            wp = self.map.get_waypoint(best_spawn.location)
            left = wp.get_left_lane()
            right = wp.get_right_lane()
            lanes = 1 + (1 if left and left.lane_type == carla.LaneType.Driving else 0) + \
                       (1 if right and right.lane_type == carla.LaneType.Driving else 0)
            print(f"Found spawn point with {lanes} accessible lanes (score: {best_score})")
            return best_spawn
        
        # Fallback: first spawn point
        print("Warning: No multi-lane spawn found, using first available")
        return spawn_points[0]
    
    def spawn_traffic_vehicle(self, 
                              relative_x: float, 
                              relative_d: float, 
                              speed: float) -> Optional[carla.Vehicle]:
        """
        Spawn a traffic vehicle relative to ego.
        
        Args:
            relative_x: Longitudinal offset from ego (positive = ahead)
            relative_d: Lateral offset from ego centerline (positive = left)
            speed: Initial speed in m/s
            
        Returns:
            Spawned vehicle or None if failed
        """
        if self.ego_vehicle is None:
            print("Cannot spawn traffic: ego vehicle not spawned")
            return None
        
        try:
            # Get ego transform and waypoint
            ego_transform = self.ego_vehicle.get_transform()
            ego_waypoint = self.map.get_waypoint(ego_transform.location)
            
            # Navigate to the correct lane first using waypoint system
            target_waypoint = ego_waypoint
            
            # Move to adjacent lane if needed
            if relative_d > self.config.lane_width / 2:
                # Target is in left lane
                left_lane = ego_waypoint.get_left_lane()
                if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                    target_waypoint = left_lane
                else:
                    print(f"  Warning: Left lane not available for traffic spawn")
                    return None
            elif relative_d < -self.config.lane_width / 2:
                # Target is in right lane
                right_lane = ego_waypoint.get_right_lane()
                if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                    target_waypoint = right_lane
                else:
                    print(f"  Warning: Right lane not available for traffic spawn")
                    return None
            
            # Now move forward/backward along the lane
            if relative_x > 0:
                # Move ahead
                next_wps = target_waypoint.next(relative_x)
                if next_wps:
                    target_waypoint = next_wps[0]
            elif relative_x < 0:
                # Move behind
                prev_wps = target_waypoint.previous(abs(relative_x))
                if prev_wps:
                    target_waypoint = prev_wps[0]
            
            # Get spawn transform from waypoint
            spawn_transform = target_waypoint.transform
            spawn_transform.location.z += 0.5  # Lift slightly to avoid ground collision
            
            # Get blueprint - try multiple vehicle types if first fails
            vehicle_models = [
                self.config.traffic_vehicle_model,
                'vehicle.audi.a2',
                'vehicle.toyota.prius',
                'vehicle.ford.mustang',
                'vehicle.*'  # Any vehicle as last resort
            ]
            
            vehicle = None
            for model in vehicle_models:
                try:
                    bps = self.blueprint_library.filter(model)
                    if bps:
                        bp = bps[0]
                        vehicle = self.world.try_spawn_actor(bp, spawn_transform)
                        if vehicle is not None:
                            break
                except:
                    continue
            
            if vehicle is not None:
                # Set initial velocity
                forward = spawn_transform.get_forward_vector()
                velocity = carla.Vector3D(
                    x=speed * forward.x,
                    y=speed * forward.y,
                    z=0.0
                )
                vehicle.set_target_velocity(velocity)
                
                # Enable autopilot to maintain speed
                vehicle.set_autopilot(True)
                
                # Configure traffic manager for this vehicle
                traffic_manager = self.client.get_trafficmanager()
                # Set speed as percentage difference from speed limit
                speed_diff_percent = (self.config.speed_limit - speed) / self.config.speed_limit * 100
                traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff_percent)
                traffic_manager.auto_lane_change(vehicle, False)  # Disable auto lane change
                
                self.traffic_vehicles.append(vehicle)
                print(f"  Traffic vehicle spawned at offset ({relative_x:.1f}, {relative_d:.1f})")
                
            else:
                print(f"  Warning: Could not spawn traffic at ({relative_x:.1f}, {relative_d:.1f})")
                
            return vehicle
            
        except Exception as e:
            print(f"Error spawning traffic vehicle: {e}")
            return None
    
    def setup_scenario(self, scenario: Scenario):
        """
        Set up a test scenario.
        
        Args:
            scenario: Which scenario to set up
        """
        # Clear existing traffic
        self.clear_traffic()
        
        # Reset state
        self.simulation_time = 0.0
        self.target_lane_offset = 0.0
        self.current_command = None
        self._lane_change_initiated = False
        
        # Reset lane tracking for fresh start
        self.reset_lane_tracking()
        
        # Spawn ego if not already
        if self.ego_vehicle is None:
            self.spawn_ego_vehicle()
        else:
            # Reset ego position and velocity
            spawn = self._find_highway_spawn()
            self.ego_vehicle.set_transform(spawn)
            self.ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        
        # Set initial ego speed
        if self.ego_vehicle:
            transform = self.ego_vehicle.get_transform()
            forward = transform.get_forward_vector()
            initial_speed = 10.0  # Starting speed (m/s)
            self.ego_vehicle.set_target_velocity(carla.Vector3D(
                x=initial_speed * forward.x,
                y=initial_speed * forward.y,
                z=0.0
            ))
        
        # Tick to apply changes
        self.world.tick()
        time.sleep(0.5)
        
        print(f"\nSetting up scenario: {scenario.value}")
        
        if scenario == Scenario.EMPTY:
            # No traffic - just cruise
            print("Empty road scenario - no traffic spawned")
            
        elif scenario == Scenario.FOLLOW:
            # Lead vehicle in center lane (slower than ego max)
            self.spawn_traffic_vehicle(
                relative_x=25.0,
                relative_d=0.0,
                speed=10.0  # Slow enough to follow
            )
            
            # Block left lane
            self.spawn_traffic_vehicle(
                relative_x=-10.0,
                relative_d=self.config.lane_width,
                speed=10.0
            )
            self.spawn_traffic_vehicle(
                relative_x=5.0,
                relative_d=self.config.lane_width,
                speed=10.0
            )
            self.spawn_traffic_vehicle(
                relative_x=20.0,
                relative_d=self.config.lane_width,
                speed=10.0
            )
            
            # Block right lane  
            self.spawn_traffic_vehicle(
                relative_x=-10.0,
                relative_d=-self.config.lane_width,
                speed=10.0
            )
            self.spawn_traffic_vehicle(
                relative_x=5.0,
                relative_d=-self.config.lane_width,
                speed=10.0
            )
            self.spawn_traffic_vehicle(
                relative_x=20.0,
                relative_d=-self.config.lane_width,
                speed=10.0
            )
            
            print("Follow scenario: Lead vehicle + adjacent blockers")
            
        elif scenario == Scenario.OVERTAKE:
            # Slow vehicle in center lane (triggers overtake)
            self.spawn_traffic_vehicle(
                relative_x=40.0,
                relative_d=0.0,
                speed=8.0  # Very slow to trigger lane change
            )
            
            # Block right lane initially
            self.spawn_traffic_vehicle(
                relative_x=35.0,
                relative_d=-self.config.lane_width,
                speed=12.0
            )
            
            # Slow vehicle in left lane (further ahead)
            self.spawn_traffic_vehicle(
                relative_x=150.0,
                relative_d=self.config.lane_width,
                speed=8.0  # Slow to trigger return to center
            )
            
            print("Overtake scenario: Slow center -> change left -> slow left -> change right")
        
        # Final tick to settle everything
        self.world.tick()
        print("Scenario setup complete!\n")
    
    def get_environment_state(self) -> EnvironmentState:
        """
        Get current environment state for behavior tree.
        
        Maps CARLA world state to the EnvironmentState dataclass.
        
        =====================================================================
        STUDENT TODO: Implement this method
        =====================================================================
        
        This method extracts perception data from CARLA and returns an
        EnvironmentState object that your behavior tree uses for decisions.
        
        You need to fill in the following fields:
        
        1. ego_speed (float): Current vehicle speed in m/s
           - Get velocity: self.ego_vehicle.get_velocity()
           - Compute magnitude: sqrt(vx² + vy² + vz²)
        
        2. ego_d (float): Lateral offset from lane center
           - Use the provided helper: self._calculate_lateral_offset(transform, waypoint)
        
        3. speed_limit (float): Use self.config.speed_limit
        
        4. left_lane_exists / right_lane_exists (bool):
           - Get ego waypoint: self.map.get_waypoint(ego_transform.location)
           - Check: waypoint.get_left_lane() / get_right_lane()
           - Verify: lane.lane_type == carla.LaneType.Driving
        
        5. left_lane_clear / right_lane_clear (bool):
           - For each traffic vehicle in self.traffic_vehicles
           - Calculate relative position using: self._calculate_relative_position()
           - If vehicle is in adjacent lane AND within lane_change_gap, lane is NOT clear
           - Lane boundaries:
             * Same lane: |rel_d| < lane_width/2
             * Left lane: lane_width/2 < rel_d < 1.5*lane_width
             * Right lane: -1.5*lane_width < rel_d < -lane_width/2
        
        6. vehicle_ahead / vehicle_ahead_distance / vehicle_ahead_speed:
           - Find closest traffic vehicle in same lane ahead of ego
           - Must be within detection_range
           - rel_x > 0 means vehicle is ahead
        
        Helper methods available:
        - self._calculate_lateral_offset(ego_transform, waypoint) -> float
        - self._calculate_relative_position(ego_transform, other_transform) -> (rel_x, rel_d)
          * rel_x: longitudinal offset (positive = ahead)
          * rel_d: lateral offset (positive = left)
        
        Config parameters:
        - self.config.lane_width = 3.5 meters
        - self.config.detection_range = 100 meters  
        - self.config.lane_change_gap = 25 meters
        - self.config.speed_limit = 31.0 m/s
        
        Returns:
            EnvironmentState with all perception data filled in
        """
        env = EnvironmentState()
        
        if self.ego_vehicle is None:
            return env
        
        # =====================================================================
        # TODO: Get ego vehicle state
        # =====================================================================
        # STEP 1: Get ego transform and velocity
        # ego_transform = self.ego_vehicle.get_transform()
        # ego_velocity = self.ego_vehicle.get_velocity()
        # ego_speed = np.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
        
        # STEP 2: Get waypoint for lane information
        # ego_waypoint = self.map.get_waypoint(ego_transform.location)
        
        # STEP 3: Calculate lateral offset
        # ego_d = self._calculate_lateral_offset(ego_transform, ego_waypoint)
        
        env.ego_speed = 0.0  # TODO: Replace with actual speed
        env.ego_d = 0.0      # TODO: Replace with actual lateral offset
        env.speed_limit = self.config.speed_limit
        
        # =====================================================================
        # TODO: Check lane existence
        # =====================================================================
        # left_lane = ego_waypoint.get_left_lane()
        # right_lane = ego_waypoint.get_right_lane()
        # env.left_lane_exists = (left_lane is not None and 
        #                         left_lane.lane_type == carla.LaneType.Driving)
        # env.right_lane_exists = (right_lane is not None and 
        #                          right_lane.lane_type == carla.LaneType.Driving)
        
        env.left_lane_exists = True   # TODO: Replace with actual check
        env.right_lane_exists = True  # TODO: Replace with actual check
        
        # =====================================================================
        # TODO: Detect traffic vehicles
        # =====================================================================
        # Initialize defaults
        env.vehicle_ahead = False
        env.vehicle_ahead_distance = self.config.detection_range
        env.vehicle_ahead_speed = 0.0
        env.left_lane_clear = True
        env.right_lane_clear = True
        
        # TODO: Loop through self.traffic_vehicles and check each one
        # for traffic in self.traffic_vehicles:
        #     if not traffic.is_alive:
        #         continue
        #     
        #     traffic_transform = traffic.get_transform()
        #     traffic_velocity = traffic.get_velocity()
        #     traffic_speed = np.sqrt(traffic_velocity.x**2 + traffic_velocity.y**2)
        #     
        #     # Calculate relative position
        #     rel_x, rel_d = self._calculate_relative_position(
        #         ego_transform, traffic_transform
        #     )
        #     
        #     # TODO: Check if vehicle is in same lane (use rel_d and lane_width)
        #     # TODO: If in same lane and ahead (rel_x > 0), update vehicle_ahead info
        #     # TODO: Check if vehicle is in left lane, update left_lane_clear
        #     # TODO: Check if vehicle is in right lane, update right_lane_clear
        
        return env
    
    def _calculate_lateral_offset(self, 
                                   ego_transform: carla.Transform,
                                   waypoint: carla.Waypoint) -> float:
        """
        Calculate lateral offset representing which lane ego is in.
        
        Returns lane-based offset where:
            0 = center lane (starting lane)
            +lane_width = one lane to the left
            -lane_width = one lane to the right
        
        This is different from just computing offset from current lane center,
        which would always be near 0.
        """
        # If this is our first call, record the starting lane
        if not hasattr(self, '_start_lane_id'):
            self._start_lane_id = waypoint.lane_id
            self._start_road_id = waypoint.road_id
        
        # Get current lane info
        current_lane_id = waypoint.lane_id
        current_road_id = waypoint.road_id
        
        # Calculate lane offset (how many lanes from start)
        # In CARLA, lane_id decreases going left, increases going right
        # But sign depends on road direction, so we need to be careful
        
        if current_road_id == self._start_road_id:
            # Same road - calculate lane difference
            lane_diff = self._start_lane_id - current_lane_id
            
            # Each lane difference is roughly one lane_width
            # Positive lane_diff means we moved left (positive d)
            ego_d = lane_diff * self.config.lane_width
        else:
            # Different road - try to estimate based on waypoint distance
            # This can happen on curved roads or interchanges
            ego_d = 0.0
        
        # Add small offset for position within current lane
        # This helps with smooth lane changes
        lane_center = waypoint.transform.location
        dx = ego_transform.location.x - lane_center.x
        dy = ego_transform.location.y - lane_center.y
        
        lane_forward = waypoint.transform.get_forward_vector()
        lane_right = carla.Vector3D(-lane_forward.y, lane_forward.x, 0)
        
        # Small within-lane offset
        within_lane_offset = -(dx * lane_right.x + dy * lane_right.y)
        
        # Combine: lane-based offset + small within-lane adjustment
        # Clamp within-lane offset to prevent it from triggering lane changes
        within_lane_offset = np.clip(within_lane_offset, -1.0, 1.0)
        
        return ego_d + within_lane_offset
    
    def reset_lane_tracking(self):
        """Reset lane tracking - call when resetting scenario."""
        if hasattr(self, '_start_lane_id'):
            del self._start_lane_id
        if hasattr(self, '_start_road_id'):
            del self._start_road_id
    
    def _calculate_relative_position(self,
                                      ego_transform: carla.Transform,
                                      other_transform: carla.Transform) -> Tuple[float, float]:
        """
        Calculate position of other vehicle relative to ego.
        
        Returns:
            Tuple of (longitudinal_offset, lateral_offset)
            Positive longitudinal = ahead
            Positive lateral = left
        """
        # Vector from ego to other
        dx = other_transform.location.x - ego_transform.location.x
        dy = other_transform.location.y - ego_transform.location.y
        
        # Ego's forward and right vectors
        forward = ego_transform.get_forward_vector()
        right = carla.Vector3D(-forward.y, forward.x, 0)
        
        # Project onto ego's coordinate frame
        longitudinal = dx * forward.x + dy * forward.y
        lateral = -(dx * right.x + dy * right.y)  # Positive = left
        
        return longitudinal, lateral
    
    def apply_command(self, command: BehaviorCommand):
        """
        Apply a behavior command to the ego vehicle.
        
        Args:
            command: BehaviorCommand from the behavior tree
        """
        # Check if behavior changed
        if self.current_command is None or command.behavior != self.current_command.behavior:
            self._lane_change_initiated = False
        
        self.current_command = command
        
        # Update target lane offset based on command
        if command.behavior == BehaviorType.LANE_CHANGE_LEFT:
            self.target_lane_offset += self.config.lane_width
        elif command.behavior == BehaviorType.LANE_CHANGE_RIGHT:
            self.target_lane_offset -= self.config.lane_width
        # For LANE_KEEP and FOLLOW, maintain current target
    
    def tick(self) -> bool:
        """
        Advance simulation by one time step.
        
        Uses CARLA autopilot for lane keeping, force_lane_change for lane changes.
        
        Returns:
            True if tick successful
        """
        if self.ego_vehicle is None:
            return False
        
        # Get current state
        ego_velocity = self.ego_vehicle.get_velocity()
        current_speed = np.sqrt(ego_velocity.x**2 + ego_velocity.y**2)
        
        # Get environment for emergency checking
        env = self.get_environment_state()
        
        # Check for emergency - override autopilot with full brake
        if env.vehicle_ahead and env.vehicle_ahead_distance < 10.0:
            relative_speed = current_speed - env.vehicle_ahead_speed
            if relative_speed > 0 and env.vehicle_ahead_distance / relative_speed < 2.0:
                # Emergency brake - disable autopilot temporarily
                self.ego_vehicle.set_autopilot(False)
                emergency_control = carla.VehicleControl()
                emergency_control.throttle = 0.0
                emergency_control.brake = 1.0
                emergency_control.steer = 0.0
                self.ego_vehicle.apply_control(emergency_control)
                self._update_spectator()
                self.world.tick()
                self.simulation_time += self.config.fixed_delta_seconds
                # Re-enable autopilot
                self.ego_vehicle.set_autopilot(True)
                return True
        
        # Process behavior command
        if self.current_command is not None:
            traffic_manager = self.client.get_trafficmanager()
            
            # Handle lane changes
            if self.current_command.behavior == BehaviorType.LANE_CHANGE_LEFT:
                if not hasattr(self, '_lane_change_initiated') or not self._lane_change_initiated:
                    # Check if left lane exists and is clear
                    if env.left_lane_exists and env.left_lane_clear:
                        traffic_manager.force_lane_change(self.ego_vehicle, False)  # False = left
                        self._lane_change_initiated = True
                        print("  -> Initiating lane change LEFT")
                        
            elif self.current_command.behavior == BehaviorType.LANE_CHANGE_RIGHT:
                if not hasattr(self, '_lane_change_initiated') or not self._lane_change_initiated:
                    # Check if right lane exists and is clear
                    if env.right_lane_exists and env.right_lane_clear:
                        traffic_manager.force_lane_change(self.ego_vehicle, True)  # True = right
                        self._lane_change_initiated = True
                        print("  -> Initiating lane change RIGHT")
            else:
                # LANE_KEEP or FOLLOW_VEHICLE - reset lane change flag
                self._lane_change_initiated = False
            
            # Control speed via traffic manager
            target_speed = self.current_command.target_speed
            speed_limit = self.config.speed_limit
            
            if speed_limit > 0:
                speed_diff_percent = (1.0 - target_speed / speed_limit) * 100
            else:
                speed_diff_percent = 0
            
            traffic_manager.vehicle_percentage_speed_difference(self.ego_vehicle, speed_diff_percent)
        
        # Update spectator camera
        self._update_spectator()
        
        # Tick world
        self.world.tick()
        self.simulation_time += self.config.fixed_delta_seconds
        
        return True
    
    def _get_within_lane_offset(self, ego_transform: carla.Transform, 
                                 waypoint: carla.Waypoint) -> float:
        """
        Get simple within-lane offset from lane center.
        
        Positive = left of center, Negative = right of center.
        """
        lane_center = waypoint.transform.location
        dx = ego_transform.location.x - lane_center.x
        dy = ego_transform.location.y - lane_center.y
        
        lane_forward = waypoint.transform.get_forward_vector()
        
        # Perpendicular vector (rotated 90° CCW)
        perp_x = -lane_forward.y
        perp_y = lane_forward.x
        
        # Project onto perpendicular
        offset = dx * perp_x + dy * perp_y
        
        return -offset
    
    def _calculate_heading_error(self, ego_transform: carla.Transform,
                                  waypoint: carla.Waypoint) -> float:
        """
        Calculate heading error between vehicle and road direction.
        
        Returns:
            Heading error in radians (positive = vehicle pointing LEFT of road)
            This matches the controller convention where positive heading error
            needs positive steering (steer right) to correct.
        """
        # Get vehicle heading (yaw in radians)
        vehicle_yaw = np.radians(ego_transform.rotation.yaw)
        
        # Get road heading from waypoint
        road_forward = waypoint.transform.get_forward_vector()
        road_yaw = np.arctan2(road_forward.y, road_forward.x)
        
        # Calculate heading error
        # Positive = vehicle pointing left of road (counter-clockwise)
        heading_error = vehicle_yaw - road_yaw
        
        # Normalize to [-pi, pi]
        while heading_error > np.pi:
            heading_error -= 2 * np.pi
        while heading_error < -np.pi:
            heading_error += 2 * np.pi
        
        return heading_error
    
    def _update_spectator(self):
        """Update spectator camera to follow ego vehicle."""
        if self.spectator is None or self.ego_vehicle is None:
            return
        
        ego_transform = self.ego_vehicle.get_transform()
        
        # Position camera behind and above ego
        forward = ego_transform.get_forward_vector()
        camera_transform = carla.Transform(
            carla.Location(
                x=ego_transform.location.x - 15 * forward.x,
                y=ego_transform.location.y - 15 * forward.y,
                z=ego_transform.location.z + 8
            ),
            carla.Rotation(pitch=-20, yaw=ego_transform.rotation.yaw)
        )
        
        self.spectator.set_transform(camera_transform)
    
    def get_state_string(self) -> str:
        """Get a string representation of current state."""
        if self.ego_vehicle is None:
            return "No ego vehicle"
        
        ego_velocity = self.ego_vehicle.get_velocity()
        speed = np.sqrt(ego_velocity.x**2 + ego_velocity.y**2)
        
        ego_transform = self.ego_vehicle.get_transform()
        
        behavior = "none"
        if self.current_command:
            behavior = self.current_command.behavior.value
        
        return (f"t={self.simulation_time:5.1f}s | "
                f"x={ego_transform.location.x:7.1f} | "
                f"y={ego_transform.location.y:7.1f} | "
                f"v={speed:5.1f} m/s | {behavior}")
    
    def clear_traffic(self):
        """Remove all traffic vehicles."""
        for vehicle in self.traffic_vehicles:
            try:
                if vehicle.is_alive:
                    vehicle.destroy()
            except:
                pass
        self.traffic_vehicles.clear()
    
    def cleanup(self):
        """Clean up all resources."""
        print("\nCleaning up CARLA resources...")
        
        # Destroy traffic
        self.clear_traffic()
        
        # Destroy ego
        if self.ego_vehicle is not None:
            try:
                if self.ego_vehicle.is_alive:
                    self.ego_vehicle.destroy()
            except:
                pass
            self.ego_vehicle = None
        
        # IMPORTANT: Restore original settings (disable synchronous mode)
        # This prevents crashes on the next run
        if self.world is not None:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
                
                # Also disable traffic manager sync
                if self.client is not None:
                    traffic_manager = self.client.get_trafficmanager()
                    traffic_manager.set_synchronous_mode(False)
                
                print("✓ Synchronous mode disabled")
            except Exception as e:
                print(f"Warning: Could not restore settings: {e}")
        
        print("Cleanup complete!")


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("CARLA Interface Test")
    print("=" * 60)
    print("\nNote: This test requires a running CARLA server!")
    print("Start CARLA with: ./CarlaUE4.sh or CarlaUE4.exe\n")
    
    interface = CarlaInterface()
    
    if interface.connect():
        print("\n--- Spawning Ego Vehicle ---")
        if interface.spawn_ego_vehicle():
            print("\n--- Setting Up Follow Scenario ---")
            interface.setup_scenario(Scenario.FOLLOW)
            
            print("\n--- Testing Environment State ---")
            env = interface.get_environment_state()
            print(f"Ego speed: {env.ego_speed:.1f} m/s")
            print(f"Ego lateral offset: {env.ego_d:.2f} m")
            print(f"Vehicle ahead: {env.vehicle_ahead}")
            if env.vehicle_ahead:
                print(f"  Distance: {env.vehicle_ahead_distance:.1f} m")
                print(f"  Speed: {env.vehicle_ahead_speed:.1f} m/s")
            print(f"Left lane clear: {env.left_lane_clear}")
            print(f"Right lane clear: {env.right_lane_clear}")
            
            print("\n--- Running for 5 seconds ---")
            from bt_framework import BehaviorCommand, BehaviorType
            
            # Apply lane keep command
            cmd = BehaviorCommand(
                behavior=BehaviorType.LANE_KEEP,
                target_d=0.0,
                target_speed=25.0,
                T=3.0
            )
            interface.apply_command(cmd)
            
            for i in range(50):  # 5 seconds at 10 Hz
                interface.tick()
                if i % 10 == 0:
                    print(interface.get_state_string())
            
            print("\n--- Cleanup ---")
            interface.cleanup()
            
            print("\n" + "=" * 60)
            print("✅ CARLA Interface test completed!")
        else:
            print("Failed to spawn ego vehicle")
    else:
        print("Failed to connect to CARLA server")
        print("\nMake sure CARLA is running:")
        print("  Linux: ./CarlaUE4.sh")
        print("  Windows: CarlaUE4.exe")
