"""
Perception Module (PROVIDED)

This module processes raw sensor data from CARLA and provides:
- Ego vehicle state
- Detected obstacles with position and velocity
- Lane information
- Reference path (waypoints)

Students should NOT modify this file.
"""

import numpy as np
from typing import List, Dict, Optional
from dataclasses import dataclass, field

from .carla_interface import CarlaInterface, VehicleState


@dataclass
class Obstacle:
    """Represents a detected obstacle."""
    id: int             # Unique identifier
    x: float            # Global X position (meters)
    y: float            # Global Y position (meters)
    vx: float           # Velocity X component (m/s)
    vy: float           # Velocity Y component (m/s)
    length: float       # Length of obstacle (meters)
    width: float        # Width of obstacle (meters)
    lane_id: int        # Lane ID where obstacle is located
    
    @property
    def speed(self) -> float:
        """Calculate speed magnitude."""
        return np.sqrt(self.vx**2 + self.vy**2)
    
    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'vx': self.vx,
            'vy': self.vy,
            'length': self.length,
            'width': self.width,
            'lane_id': self.lane_id
        }


@dataclass
class LaneInfo:
    """Information about the current lane configuration."""
    current_lane: int           # Current lane ID
    left_lane_exists: bool      # Is there a drivable lane to the left?
    right_lane_exists: bool     # Is there a drivable lane to the right?
    lane_width: float           # Width of current lane (meters)
    road_id: int = 0            # Road segment ID
    is_junction: bool = False   # Is the vehicle in a junction?
    
    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'current_lane': self.current_lane,
            'left_lane_exists': self.left_lane_exists,
            'right_lane_exists': self.right_lane_exists,
            'lane_width': self.lane_width,
            'road_id': self.road_id,
            'is_junction': self.is_junction
        }


@dataclass
class PerceptionData:
    """Complete perception data for one timestep."""
    timestamp: float                          # Simulation time
    ego_state: VehicleState                   # Ego vehicle state
    obstacles: List[Obstacle] = field(default_factory=list)  # Detected obstacles
    lane_info: Optional[LaneInfo] = None      # Lane configuration
    reference_path: List[tuple] = field(default_factory=list)  # Waypoints (x, y)
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for blackboard."""
        return {
            'timestamp': self.timestamp,
            'ego_state': {
                'x': self.ego_state.x,
                'y': self.ego_state.y,
                'theta': self.ego_state.theta,
                'v': self.ego_state.v,
                'vx': self.ego_state.vx,
                'vy': self.ego_state.vy,
                'lane_id': self.ego_state.lane_id
            },
            'obstacles': [obs.to_dict() for obs in self.obstacles],
            'lane_info': self.lane_info.to_dict() if self.lane_info else {},
            'reference_path': self.reference_path
        }


class PerceptionModule:
    """
    Perception module that processes CARLA data.
    
    Provides a clean interface for the behavioral and trajectory planners
    to access perception information.
    """
    
    def __init__(self, carla_interface: CarlaInterface, 
                 sensor_range: float = 100.0):
        """
        Initialize perception module.
        
        Args:
            carla_interface: CarlaInterface instance
            sensor_range: Maximum range for obstacle detection (meters)
        """
        self.carla = carla_interface
        self.sensor_range = sensor_range
        
        self._obstacle_id_counter = 0
        self._prev_obstacles: Dict[int, Obstacle] = {}  # For tracking
        
    def update(self, timestamp: float) -> PerceptionData:
        """
        Update perception data from CARLA.
        
        Args:
            timestamp: Current simulation timestamp
            
        Returns:
            PerceptionData object with all perception information.
        """
        # Get ego state
        ego_state = self.carla.get_ego_state()
        if ego_state is None:
            raise RuntimeError("Ego vehicle not available")
        
        # Get obstacles (other vehicles)
        obstacles = self._detect_obstacles(ego_state)
        
        # Get lane information
        lane_info = self._get_lane_info()
        
        # Get reference path
        reference_path = self.carla.get_waypoints_ahead(
            distance=self.sensor_range,
            spacing=2.0
        )
        
        return PerceptionData(
            timestamp=timestamp,
            ego_state=ego_state,
            obstacles=obstacles,
            lane_info=lane_info,
            reference_path=reference_path
        )
    
    def _detect_obstacles(self, ego_state: VehicleState) -> List[Obstacle]:
        """
        Detect obstacles (other vehicles) within sensor range.
        
        Args:
            ego_state: Current ego vehicle state
            
        Returns:
            List of detected obstacles.
        """
        obstacles = []
        
        for vehicle in self.carla.other_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue
            
            # Get vehicle state
            state = self.carla.get_vehicle_state(vehicle)
            
            # Check if within sensor range
            dx = state.x - ego_state.x
            dy = state.y - ego_state.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance > self.sensor_range:
                continue
            
            # Get vehicle dimensions
            bbox = vehicle.bounding_box
            length = bbox.extent.x * 2
            width = bbox.extent.y * 2
            
            obstacle = Obstacle(
                id=vehicle.id,
                x=state.x,
                y=state.y,
                vx=state.vx,
                vy=state.vy,
                length=length,
                width=width,
                lane_id=state.lane_id
            )
            obstacles.append(obstacle)
        
        return obstacles
    
    def _get_lane_info(self) -> Optional[LaneInfo]:
        """Get lane information from CARLA."""
        info = self.carla.get_lane_info()
        
        if not info:
            return None
        
        return LaneInfo(
            current_lane=info.get('current_lane', 0),
            left_lane_exists=info.get('left_lane_exists', False),
            right_lane_exists=info.get('right_lane_exists', False),
            lane_width=info.get('lane_width', 3.5),
            road_id=info.get('road_id', 0),
            is_junction=info.get('is_junction', False)
        )
    
    def get_obstacles_in_lane(self, lane_id: int, 
                              obstacles: List[Obstacle]) -> List[Obstacle]:
        """
        Filter obstacles by lane.
        
        Args:
            lane_id: Target lane ID
            obstacles: List of all obstacles
            
        Returns:
            Obstacles in the specified lane.
        """
        return [obs for obs in obstacles if obs.lane_id == lane_id]
    
    def get_lead_vehicle(self, ego_state: VehicleState,
                         obstacles: List[Obstacle]) -> Optional[Obstacle]:
        """
        Find the closest vehicle ahead in the same lane.
        
        Args:
            ego_state: Ego vehicle state
            obstacles: List of obstacles
            
        Returns:
            Closest obstacle ahead in same lane, or None.
        """
        same_lane = self.get_obstacles_in_lane(ego_state.lane_id, obstacles)
        
        lead_vehicle = None
        min_distance = float('inf')
        
        for obs in same_lane:
            # Calculate longitudinal distance
            dx = obs.x - ego_state.x
            dy = obs.y - ego_state.y
            
            # Project onto ego heading direction
            cos_theta = np.cos(ego_state.theta)
            sin_theta = np.sin(ego_state.theta)
            longitudinal = dx * cos_theta + dy * sin_theta
            
            # Only consider vehicles ahead
            if longitudinal > 0 and longitudinal < min_distance:
                min_distance = longitudinal
                lead_vehicle = obs
        
        return lead_vehicle
    
    def get_distance_to_obstacle(self, ego_state: VehicleState,
                                 obstacle: Obstacle) -> float:
        """Calculate distance from ego to obstacle."""
        dx = obstacle.x - ego_state.x
        dy = obstacle.y - ego_state.y
        return np.sqrt(dx**2 + dy**2)
    
    def check_lane_gap(self, ego_state: VehicleState,
                       obstacles: List[Obstacle],
                       target_lane: int,
                       min_gap: float = 25.0) -> bool:
        """
        Check if there's a sufficient gap in the target lane for lane change.
        
        Args:
            ego_state: Ego vehicle state
            obstacles: List of obstacles
            target_lane: Target lane ID
            min_gap: Minimum required gap (meters)
            
        Returns:
            True if gap is sufficient.
        """
        target_lane_obstacles = self.get_obstacles_in_lane(target_lane, obstacles)
        
        for obs in target_lane_obstacles:
            dx = obs.x - ego_state.x
            dy = obs.y - ego_state.y
            
            # Project onto ego heading direction
            cos_theta = np.cos(ego_state.theta)
            sin_theta = np.sin(ego_state.theta)
            longitudinal = dx * cos_theta + dy * sin_theta
            
            # Check if obstacle is too close (in front or behind)
            if abs(longitudinal) < min_gap:
                return False
        
        return True


if __name__ == '__main__':
    # Test perception module
    print("Perception module loaded successfully.")
    print("This module requires a running CARLA connection to test.")
