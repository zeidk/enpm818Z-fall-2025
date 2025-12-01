"""
Behavioral Planner (STUDENT IMPLEMENTATION)

This module assembles the Behavior Tree and provides the main interface
for behavioral planning.

YOU MUST IMPLEMENT:
- BehavioralPlanner._build_tree(): Construct the Behavior Tree structure
- BehavioralPlanner.plan(): Update blackboard and tick the tree

Refer to the project documentation for the required tree structure.
"""

import yaml
import numpy as np
from typing import Dict, List, Optional
from pathlib import Path

from .behavior_tree import (
    NodeStatus,
    BehavioralCommand,
    Blackboard,
    Sequence,
    Selector,
    BehaviorTree
)
from .behavior_tree.bt_nodes import (
    IsObstacleBlocking,
    ShouldChangeLane,
    IsLaneChangeSafe,
    IsVehicleAhead,
    IsTooClose,
    SetStopCommand,
    SetLaneChangeCommand,
    SetFollowCommand,
    SetLaneKeepCommand
)


class BehavioralPlanner:
    """
    Behavioral Planner using Behavior Trees.
    
    This class constructs and executes a Behavior Tree to decide
    high-level driving maneuvers based on the current perception data.
    
    The tree structure should be:
    
    Root [Selector]
    ├── Emergency Stop [Sequence]
    │   ├── [Condition] IsObstacleBlocking?
    │   └── [Action] SetStopCommand
    ├── Lane Change [Sequence]
    │   ├── [Condition] ShouldChangeLane?
    │   ├── [Condition] IsLaneChangeSafe?
    │   └── [Action] SetLaneChangeCommand
    ├── Follow Vehicle [Sequence]
    │   ├── [Condition] IsVehicleAhead?
    │   ├── [Condition] IsTooClose?
    │   └── [Action] SetFollowCommand
    └── Lane Keep [Sequence]
        └── [Action] SetLaneKeepCommand
    """
    
    def __init__(self, config: Dict):
        """
        Initialize the behavioral planner.
        
        Args:
            config: Configuration dictionary containing:
                - target_speed: Cruising speed (m/s)
                - safe_follow_distance: Min distance to lead vehicle (m)
                - lane_change_min_gap: Min gap for lane change (m)
                - stop_distance: Distance to stop before obstacle (m)
                - slow_vehicle_threshold: Speed threshold for "slow" (m/s)
        """
        self.config = config
        
        # Create blackboard and store config
        self.blackboard = Blackboard()
        self.blackboard.set('config', config)
        
        # Build the behavior tree
        self.tree = self._build_tree()
        self.tree.set_blackboard(self.blackboard)
        
        # Route information
        self._route: List[int] = []
        self._route_index: int = 0
    
    def _build_tree(self) -> BehaviorTree:
        """
        Build the Behavior Tree structure.
        
        Returns:
            BehaviorTree with the complete decision structure.
        """
        # TODO: Implement this method
        #
        # Build the tree structure shown in the class docstring:
        #
        # 1. Create the root Selector node
        # 2. Create Emergency Stop branch (Sequence):
        #    - IsObstacleBlocking condition
        #    - SetStopCommand action
        # 3. Create Lane Change branch (Sequence):
        #    - ShouldChangeLane condition
        #    - IsLaneChangeSafe condition
        #    - SetLaneChangeCommand action
        # 4. Create Follow Vehicle branch (Sequence):
        #    - IsVehicleAhead condition
        #    - IsTooClose condition
        #    - SetFollowCommand action
        # 5. Create Lane Keep branch (Sequence):
        #    - SetLaneKeepCommand action
        # 6. Add all branches to root Selector
        # 7. Return BehaviorTree(root)
        
        # Placeholder - replace with your implementation
        root = Selector("Root", [
            Sequence("LaneKeep", [
                SetLaneKeepCommand()
            ])
        ])
        
        return BehaviorTree(root)
    
    def set_route(self, route: List[int]):
        """
        Set the target lane sequence.
        
        Args:
            route: List of target lane IDs in order
        """
        self._route = route
        self._route_index = 0
        self.blackboard.set('route', route)
    
    def plan(self, ego_state: Dict, obstacles: List[Dict],
             lane_info: Dict, reference_path: List[tuple]) -> BehavioralCommand:
        """
        Generate a behavioral command for the current situation.
        
        Args:
            ego_state: Dict with keys: x, y, theta, v, lane_id
            obstacles: List of obstacle dicts with keys: x, y, vx, vy, lane_id
            lane_info: Dict with keys: current_lane, left_lane_exists,
                       right_lane_exists, lane_width
            reference_path: List of (x, y) waypoints
            
        Returns:
            BehavioralCommand with maneuver type and parameters
        """
        # TODO: Implement this method
        #
        # Steps:
        # 1. Update blackboard with current perception data
        #    - blackboard.update_perception(ego_state, obstacles, lane_info, reference_path)
        # 2. Update route progress if needed
        #    - Check if we've reached current target lane
        #    - Advance route index if so
        # 3. Tick the behavior tree
        #    - status = self.tree.tick()
        # 4. Return the behavioral command from blackboard
        #    - return self.blackboard.behavioral_command
        
        # Placeholder implementation
        self.blackboard.update_perception(ego_state, obstacles, lane_info, reference_path)
        self.tree.tick()
        return self.blackboard.behavioral_command
    
    def get_tree_status(self) -> str:
        """Get a string representation of the tree's current state."""
        return f"Root status: {self.tree.root.status}"
    
    def print_tree(self):
        """Print the tree structure for debugging."""
        self.tree.print_tree()


def load_config(config_path: str = None) -> Dict:
    """
    Load configuration from YAML file.
    
    Args:
        config_path: Path to config file. If None, uses default.
        
    Returns:
        Configuration dictionary
    """
    if config_path is None:
        config_path = Path(__file__).parent.parent / 'config' / 'planner_config.yaml'
    
    with open(config_path, 'r') as f:
        full_config = yaml.safe_load(f)
    
    return full_config.get('behavioral_planner', {})


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Testing Behavioral Planner...")
    print("=" * 50)
    
    # Load config
    try:
        config = load_config()
        print(f"Config loaded: {config}")
    except Exception as e:
        print(f"Could not load config file: {e}")
        config = {
            'target_speed': 25.0,
            'safe_follow_distance': 20.0,
            'lane_change_min_gap': 25.0,
            'stop_distance': 10.0,
            'slow_vehicle_threshold': 18.0
        }
        print(f"Using default config: {config}")
    
    # Create planner
    planner = BehavioralPlanner(config)
    
    # Print tree structure
    print("\nBehavior Tree Structure:")
    planner.print_tree()
    
    # Test with mock data
    print("\nTesting with mock perception data...")
    
    ego_state = {
        'x': 0.0,
        'y': 0.0,
        'theta': 0.0,
        'v': 20.0,
        'lane_id': 1
    }
    
    obstacles = []  # No obstacles
    
    lane_info = {
        'current_lane': 1,
        'left_lane_exists': True,
        'right_lane_exists': True,
        'lane_width': 3.5
    }
    
    reference_path = [(i * 2.0, 0.0) for i in range(50)]
    
    # Plan
    command = planner.plan(ego_state, obstacles, lane_info, reference_path)
    print(f"Behavioral command: {command}")
    
    # Test with vehicle ahead
    print("\nTesting with slow vehicle ahead...")
    obstacles = [{
        'x': 30.0,
        'y': 0.0,
        'vx': 10.0,
        'vy': 0.0,
        'lane_id': 1
    }]
    
    command = planner.plan(ego_state, obstacles, lane_info, reference_path)
    print(f"Behavioral command: {command}")
    
    print("\nBehavioral Planner test complete!")
