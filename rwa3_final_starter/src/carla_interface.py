"""
CARLA Interface Module (PROVIDED)

This module handles all communication with the CARLA simulator including:
- Connecting to the CARLA server
- Spawning and controlling the ego vehicle
- Spawning other vehicles for scenarios
- Accessing world and map information

Students should NOT modify this file.
"""

import carla
import numpy as np
import time
import random
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass


@dataclass
class VehicleState:
    """Represents the state of a vehicle."""
    x: float          # Global X position (meters)
    y: float          # Global Y position (meters)
    z: float          # Global Z position (meters)
    theta: float      # Heading angle (radians, 0 = East, CCW positive)
    v: float          # Speed (m/s)
    vx: float         # Velocity X component (m/s)
    vy: float         # Velocity Y component (m/s)
    ax: float         # Acceleration X component (m/s^2)
    ay: float         # Acceleration Y component (m/s^2)
    lane_id: int      # Current lane ID
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array [x, y, theta, v]."""
        return np.array([self.x, self.y, self.theta, self.v])


@dataclass  
class VehicleControl:
    """Vehicle control commands."""
    steering: float   # Steering angle [-1, 1] (normalized)
    throttle: float   # Throttle [0, 1]
    brake: float      # Brake [0, 1]
    

class CarlaInterface:
    """
    Interface for communicating with CARLA simulator.
    
    Handles:
    - Server connection
    - World and map access
    - Vehicle spawning and control
    - Waypoint queries
    """
    
    def __init__(self, host: str = 'localhost', port: int = 2000, 
                 timeout: float = 10.0):
        """
        Initialize CARLA interface.
        
        Args:
            host: CARLA server hostname
            port: CARLA server port
            timeout: Connection timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.map: Optional[carla.Map] = None
        self.ego_vehicle: Optional[carla.Vehicle] = None
        self.other_vehicles: List[carla.Vehicle] = []
        
        self._spectator: Optional[carla.Actor] = None
        
    def connect(self) -> bool:
        """
        Connect to CARLA server.
        
        Returns:
            True if connection successful, False otherwise.
        """
        try:
            print(f"Connecting to CARLA server at {self.host}:{self.port}...")
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
            
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self._spectator = self.world.get_spectator()
            
            print(f"Connected to CARLA. Map: {self.map.name}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            return False
    
    def load_map(self, map_name: str) -> bool:
        """
        Load a specific map.
        
        Args:
            map_name: Name of the map (e.g., 'Town04')
            
        Returns:
            True if map loaded successfully.
        """
        try:
            if self.map.name != map_name:
                print(f"Loading map: {map_name}...")
                self.world = self.client.load_world(map_name)
                self.map = self.world.get_map()
                self._spectator = self.world.get_spectator()
                time.sleep(2.0)  # Wait for map to load
            return True
        except Exception as e:
            print(f"Failed to load map: {e}")
            return False
    
    def spawn_ego_vehicle(self, spawn_point: Optional[carla.Transform] = None,
                          blueprint_name: str = 'vehicle.tesla.model3') -> bool:
        """
        Spawn the ego vehicle.
        
        Args:
            spawn_point: Transform for spawn location. If None, uses random spawn point.
            blueprint_name: Vehicle blueprint name.
            
        Returns:
            True if vehicle spawned successfully.
        """
        try:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find(blueprint_name)
            
            if spawn_point is None:
                spawn_points = self.map.get_spawn_points()
                spawn_point = random.choice(spawn_points)
            
            self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Spawned ego vehicle at ({spawn_point.location.x:.1f}, "
                  f"{spawn_point.location.y:.1f})")
            return True
            
        except Exception as e:
            print(f"Failed to spawn ego vehicle: {e}")
            return False
    
    def spawn_vehicle(self, spawn_point: carla.Transform,
                      blueprint_name: str = 'vehicle.audi.a2') -> Optional[carla.Vehicle]:
        """
        Spawn an NPC vehicle.
        
        Args:
            spawn_point: Transform for spawn location.
            blueprint_name: Vehicle blueprint name.
            
        Returns:
            Spawned vehicle actor, or None if failed.
        """
        try:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find(blueprint_name)
            
            vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.other_vehicles.append(vehicle)
            return vehicle
            
        except Exception as e:
            print(f"Failed to spawn vehicle: {e}")
            return None
    
    def get_ego_state(self) -> Optional[VehicleState]:
        """
        Get current state of ego vehicle.
        
        Returns:
            VehicleState object, or None if ego vehicle not available.
        """
        if self.ego_vehicle is None:
            return None
        
        transform = self.ego_vehicle.get_transform()
        velocity = self.ego_vehicle.get_velocity()
        acceleration = self.ego_vehicle.get_acceleration()
        
        # Get heading angle (convert from CARLA's yaw to standard convention)
        theta = np.radians(transform.rotation.yaw)
        
        # Calculate speed
        vx = velocity.x
        vy = velocity.y
        v = np.sqrt(vx**2 + vy**2)
        
        # Get lane ID
        waypoint = self.map.get_waypoint(transform.location)
        lane_id = waypoint.lane_id if waypoint else 0
        
        return VehicleState(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z,
            theta=theta,
            v=v,
            vx=vx,
            vy=vy,
            ax=acceleration.x,
            ay=acceleration.y,
            lane_id=lane_id
        )
    
    def get_vehicle_state(self, vehicle: carla.Vehicle) -> VehicleState:
        """Get state of any vehicle."""
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        acceleration = vehicle.get_acceleration()
        
        theta = np.radians(transform.rotation.yaw)
        vx = velocity.x
        vy = velocity.y
        v = np.sqrt(vx**2 + vy**2)
        
        waypoint = self.map.get_waypoint(transform.location)
        lane_id = waypoint.lane_id if waypoint else 0
        
        return VehicleState(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z,
            theta=theta,
            v=v,
            vx=vx,
            vy=vy,
            ax=acceleration.x,
            ay=acceleration.y,
            lane_id=lane_id
        )
    
    def apply_control(self, control: VehicleControl):
        """
        Apply control to ego vehicle.
        
        Args:
            control: VehicleControl with steering, throttle, brake.
        """
        if self.ego_vehicle is None:
            return
        
        carla_control = carla.VehicleControl(
            throttle=float(np.clip(control.throttle, 0.0, 1.0)),
            steer=float(np.clip(control.steering, -1.0, 1.0)),
            brake=float(np.clip(control.brake, 0.0, 1.0))
        )
        self.ego_vehicle.apply_control(carla_control)
    
    def set_vehicle_velocity(self, vehicle: carla.Vehicle, speed: float):
        """
        Set a vehicle's velocity directly (for NPC vehicles).
        
        Args:
            vehicle: Vehicle actor
            speed: Target speed in m/s
        """
        transform = vehicle.get_transform()
        forward = transform.get_forward_vector()
        velocity = carla.Vector3D(
            x=forward.x * speed,
            y=forward.y * speed,
            z=0.0
        )
        vehicle.set_target_velocity(velocity)
    
    def get_waypoints_ahead(self, distance: float = 100.0, 
                            spacing: float = 2.0) -> List[Tuple[float, float]]:
        """
        Get waypoints ahead of the ego vehicle.
        
        Args:
            distance: How far ahead to get waypoints (meters)
            spacing: Distance between waypoints (meters)
            
        Returns:
            List of (x, y) tuples representing the road centerline.
        """
        if self.ego_vehicle is None:
            return []
        
        location = self.ego_vehicle.get_location()
        waypoint = self.map.get_waypoint(location)
        
        if waypoint is None:
            return []
        
        waypoints = []
        current_wp = waypoint
        traveled = 0.0
        
        while traveled < distance:
            waypoints.append((current_wp.transform.location.x,
                              current_wp.transform.location.y))
            next_wps = current_wp.next(spacing)
            if not next_wps:
                break
            current_wp = next_wps[0]
            traveled += spacing
        
        return waypoints
    
    def get_lane_info(self) -> Dict:
        """
        Get lane information for ego vehicle's current position.
        
        Returns:
            Dictionary with lane information.
        """
        if self.ego_vehicle is None:
            return {}
        
        location = self.ego_vehicle.get_location()
        waypoint = self.map.get_waypoint(location)
        
        if waypoint is None:
            return {}
        
        # Check for adjacent lanes
        left_lane = waypoint.get_left_lane()
        right_lane = waypoint.get_right_lane()
        
        # Lane width
        lane_width = waypoint.lane_width
        
        return {
            'current_lane': waypoint.lane_id,
            'left_lane_exists': left_lane is not None and 
                               left_lane.lane_type == carla.LaneType.Driving,
            'right_lane_exists': right_lane is not None and 
                                right_lane.lane_type == carla.LaneType.Driving,
            'lane_width': lane_width,
            'road_id': waypoint.road_id,
            'is_junction': waypoint.is_junction
        }
    
    def update_spectator(self, follow_ego: bool = True):
        """
        Update spectator camera position.
        
        Args:
            follow_ego: If True, follow the ego vehicle.
        """
        if follow_ego and self.ego_vehicle is not None:
            ego_transform = self.ego_vehicle.get_transform()
            spectator_transform = carla.Transform(
                carla.Location(
                    x=ego_transform.location.x - 10.0,
                    y=ego_transform.location.y,
                    z=ego_transform.location.z + 10.0
                ),
                carla.Rotation(pitch=-30.0, yaw=ego_transform.rotation.yaw)
            )
            self._spectator.set_transform(spectator_transform)
    
    def tick(self) -> float:
        """
        Advance the simulation by one tick.
        
        Returns:
            Simulation timestamp.
        """
        self.world.tick()
        return self.world.get_snapshot().timestamp.elapsed_seconds
    
    def cleanup(self):
        """Destroy all spawned actors and clean up."""
        print("Cleaning up CARLA actors...")
        
        # Destroy other vehicles
        for vehicle in self.other_vehicles:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()
        self.other_vehicles.clear()
        
        # Destroy ego vehicle
        if self.ego_vehicle is not None and self.ego_vehicle.is_alive:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        
        print("Cleanup complete.")


def create_spawn_point(x: float, y: float, z: float = 0.3, 
                       yaw: float = 0.0) -> carla.Transform:
    """
    Create a CARLA Transform for spawning.
    
    Args:
        x, y, z: Position coordinates
        yaw: Heading angle in degrees
        
    Returns:
        carla.Transform object
    """
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(yaw=yaw)
    )


if __name__ == '__main__':
    # Test the CARLA interface
    print("Testing CARLA Interface...")
    
    interface = CarlaInterface()
    
    if interface.connect():
        interface.load_map('Town04')
        
        # Spawn ego vehicle
        spawn = create_spawn_point(5.0, -150.0, 0.3, 90.0)
        if interface.spawn_ego_vehicle(spawn):
            print("Ego vehicle spawned successfully")
            
            # Get state
            state = interface.get_ego_state()
            print(f"Ego state: x={state.x:.1f}, y={state.y:.1f}, "
                  f"theta={np.degrees(state.theta):.1f}°, v={state.v:.1f} m/s")
            
            # Get waypoints
            waypoints = interface.get_waypoints_ahead()
            print(f"Got {len(waypoints)} waypoints ahead")
            
            # Get lane info
            lane_info = interface.get_lane_info()
            print(f"Lane info: {lane_info}")
            
            # Clean up
            interface.cleanup()
    
    print("Test complete.")
