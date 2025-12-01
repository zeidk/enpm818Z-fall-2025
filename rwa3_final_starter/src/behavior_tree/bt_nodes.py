"""
Behavior Tree Nodes 

This module contains the custom Condition and Action nodes for the
behavioral planner's Behavior Tree.

YOU MUST IMPLEMENT:
- Condition Nodes: IsObstacleBlocking, ShouldChangeLane, IsLaneChangeSafe,
                   IsVehicleAhead, IsTooClose
- Action Nodes: SetStopCommand, SetLaneChangeCommand, SetFollowCommand,
                SetLaneKeepCommand

Refer to the project documentation for the expected behavior of each node.
"""

import numpy as np
from typing import Optional, Dict, List

from . import (
    NodeStatus,
    BehavioralCommand,
    Blackboard,
    ConditionNode,
    ActionNode
)


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def get_lead_vehicle(ego_state: Dict, obstacles: List[Dict]) -> Optional[Dict]:
    """
    Find the closest vehicle ahead in the same lane.
    
    Args:
        ego_state: Dict with x, y, theta, v, lane_id
        obstacles: List of obstacle dicts
        
    Returns:
        The closest obstacle ahead in the same lane, or None if no vehicle ahead.
    """
    # TODO: Implement this helper function
    # Hint: 
    # 1. Filter obstacles to same lane as ego
    # 2. For each obstacle, compute longitudinal distance (project onto ego heading)
    # 3. Return the closest one that is ahead (positive longitudinal distance)
    pass


def compute_distance(ego_state: Dict, obstacle: Dict) -> float:
    """
    Compute Euclidean distance between ego and obstacle.
    
    Args:
        ego_state: Dict with x, y
        obstacle: Dict with x, y
        
    Returns:
        Distance in meters
    """
    # TODO: Implement this helper function
    pass


def compute_longitudinal_distance(ego_state: Dict, obstacle: Dict) -> float:
    """
    Compute longitudinal (along-track) distance to obstacle.
    
    Positive means obstacle is ahead, negative means behind.
    
    Args:
        ego_state: Dict with x, y, theta
        obstacle: Dict with x, y
        
    Returns:
        Longitudinal distance in meters
    """
    # TODO: Implement this helper function
    # Hint: Project the vector from ego to obstacle onto ego's heading direction
    pass


def check_gap_in_lane(ego_state: Dict, obstacles: List[Dict], 
                      target_lane: int, min_gap: float) -> bool:
    """
    Check if there's a sufficient gap in the target lane.
    
    Args:
        ego_state: Dict with x, y, theta, lane_id
        obstacles: List of obstacle dicts
        target_lane: Lane ID to check
        min_gap: Minimum required gap (meters)
        
    Returns:
        True if gap is sufficient for lane change
    """
    # TODO: Implement this helper function
    # Hint:
    # 1. Filter obstacles to target lane
    # 2. For each obstacle, compute longitudinal distance
    # 3. Return False if any obstacle is within min_gap (front or rear)
    pass


# =============================================================================
# CONDITION NODES
# =============================================================================

class IsObstacleBlocking(ConditionNode):
    """
    Check if a static obstacle is blocking the current lane.
    
    Returns SUCCESS if:
    - There is an obstacle in the same lane
    - The obstacle is nearly stationary (speed < 0.5 m/s)
    - The obstacle is within stop_distance ahead
    
    Returns FAILURE otherwise.
    """
    
    def __init__(self):
        super().__init__("IsObstacleBlocking")
    
    def evaluate(self, blackboard: Blackboard) -> bool:
        """
        Evaluate if there's a blocking obstacle.
        
        Access from blackboard:
        - blackboard.ego_state: Dict with x, y, theta, v, lane_id
        - blackboard.obstacles: List of obstacle dicts with x, y, vx, vy, lane_id
        - blackboard.config['stop_distance']: Distance threshold
        """
        # TODO: Implement this condition
        # 
        # Hints:
        # 1. Get ego_state and obstacles from blackboard
        # 2. Find obstacles in same lane as ego
        # 3. Check if any obstacle is:
        #    - Nearly stationary (sqrt(vx^2 + vy^2) < 0.5)
        #    - Within stop_distance ahead of ego
        # 4. Return True if blocking obstacle found, False otherwise
        
        return False  # Placeholder


class ShouldChangeLane(ConditionNode):
    """
    Determine if a lane change is desirable.
    
    Returns SUCCESS if:
    - Route requires a different lane, OR
    - There's a slow vehicle ahead and changing lanes would be beneficial
    
    Returns FAILURE otherwise.
    """
    
    def __init__(self):
        super().__init__("ShouldChangeLane")
    
    def evaluate(self, blackboard: Blackboard) -> bool:
        """
        Evaluate if lane change is desirable.
        
        Access from blackboard:
        - blackboard.ego_state
        - blackboard.obstacles
        - blackboard.route: List of target lane IDs
        - blackboard.config['slow_vehicle_threshold']
        - blackboard.config['target_speed']
        """
        # TODO: Implement this condition
        #
        # Hints:
        # 1. Check if route requires different lane than current
        # 2. OR check if there's a slow vehicle ahead:
        #    - Get lead vehicle in current lane
        #    - If lead vehicle speed < slow_vehicle_threshold
        #    - And ego wants to go faster (target_speed > lead speed)
        # 3. Return True if lane change is desirable
        
        return False  # Placeholder


class IsLaneChangeSafe(ConditionNode):
    """
    Check if it's safe to execute a lane change.
    
    Returns SUCCESS if:
    - Target lane exists
    - Sufficient gap in target lane (front and rear)
    
    Returns FAILURE otherwise.
    """
    
    def __init__(self):
        super().__init__("IsLaneChangeSafe")
    
    def evaluate(self, blackboard: Blackboard) -> bool:
        """
        Evaluate if lane change is safe.
        
        Access from blackboard:
        - blackboard.ego_state
        - blackboard.obstacles
        - blackboard.lane_info: Dict with left_lane_exists, right_lane_exists
        - blackboard.config['lane_change_min_gap']
        """
        # TODO: Implement this condition
        #
        # Hints:
        # 1. Determine target lane (left or right based on route or passing)
        # 2. Check if target lane exists (lane_info)
        # 3. Check if gap in target lane is sufficient
        # 4. Store target_lane in blackboard for action node
        
        return False  # Placeholder


class IsVehicleAhead(ConditionNode):
    """
    Check if there's any vehicle ahead in the same lane.
    
    Returns SUCCESS if there's a vehicle ahead within sensor range.
    Returns FAILURE otherwise.
    """
    
    def __init__(self):
        super().__init__("IsVehicleAhead")
    
    def evaluate(self, blackboard: Blackboard) -> bool:
        """
        Evaluate if there's a vehicle ahead.
        
        Access from blackboard:
        - blackboard.ego_state
        - blackboard.obstacles
        """
        # TODO: Implement this condition
        #
        # Hints:
        # 1. Get lead vehicle using helper function
        # 2. Return True if lead vehicle exists
        
        return False  # Placeholder


class IsTooClose(ConditionNode):
    """
    Check if ego is too close to the lead vehicle.
    
    Returns SUCCESS if distance to lead vehicle < safe_follow_distance.
    Returns FAILURE otherwise.
    """
    
    def __init__(self):
        super().__init__("IsTooClose")
    
    def evaluate(self, blackboard: Blackboard) -> bool:
        """
        Evaluate if too close to lead vehicle.
        
        Access from blackboard:
        - blackboard.ego_state
        - blackboard.obstacles
        - blackboard.config['safe_follow_distance']
        """
        # TODO: Implement this condition
        #
        # Hints:
        # 1. Get lead vehicle
        # 2. Compute distance to lead vehicle
        # 3. Return True if distance < safe_follow_distance
        
        return False  # placeholder


# =============================================================================
# ACTION NODES
# =============================================================================

class SetStopCommand(ActionNode):
    """
    Set the behavioral command to stop.
    
    Sets maneuver='stop', target_speed=0.
    Always returns SUCCESS.
    """
    
    def __init__(self):
        super().__init__("SetStopCommand")
    
    def execute(self, blackboard: Blackboard) -> NodeStatus:
        """
        Set stop command on blackboard.
        """
        # TODO: Implement this action
        #
        # Hints:
        # 1. Create BehavioralCommand with maneuver='stop', target_speed=0
        # 2. Set blackboard.behavioral_command = command
        # 3. Return NodeStatus.SUCCESS
        
        return NodeStatus.SUCCESS  # Placeholder


class SetLaneChangeCommand(ActionNode):
    """
    Set the behavioral command for lane change.
    
    Sets maneuver='lane_change_left' or 'lane_change_right'.
    Returns RUNNING while lane change is in progress.
    Returns SUCCESS when lane change is complete.
    """
    
    def __init__(self):
        super().__init__("SetLaneChangeCommand")
    
    def execute(self, blackboard: Blackboard) -> NodeStatus:
        """
        Set lane change command on blackboard.
        
        This action should track lane change progress and return RUNNING
        until the ego vehicle reaches the target lane.
        """
        # TODO: Implement this action
        #
        # Hints:
        # 1. Check if lane change is already in progress (blackboard flag)
        # 2. If not in progress:
        #    - Determine target lane (left or right)
        #    - Set blackboard.set('lane_change_in_progress', True)
        #    - Set blackboard.set('target_lane_for_change', target_lane)
        # 3. Create BehavioralCommand with appropriate maneuver
        # 4. Check if ego has reached target lane:
        #    - If yes: clear flags, return SUCCESS
        #    - If no: return RUNNING
        
        return NodeStatus.SUCCESS  # placeholder


class SetFollowCommand(ActionNode):
    """
    Set the behavioral command to follow the lead vehicle.
    
    Sets maneuver='follow', target_speed=lead_vehicle_speed.
    Always returns SUCCESS.
    """
    
    def __init__(self):
        super().__init__("SetFollowCommand")
    
    def execute(self, blackboard: Blackboard) -> NodeStatus:
        """
        Set follow command on blackboard.
        """
        # TODO: Implement this action
        #
        # Hints:
        # 1. Get lead vehicle
        # 2. Get lead vehicle speed (sqrt(vx^2 + vy^2))
        # 3. Create BehavioralCommand with maneuver='follow', target_speed=lead_speed
        # 4. Return NodeStatus.SUCCESS
        
        return NodeStatus.SUCCESS  # placeholder


class SetLaneKeepCommand(ActionNode):
    """
    Set the behavioral command to keep lane.
    
    Sets maneuver='lane_keep', target_speed=cruising_speed.
    Always returns SUCCESS.
    """
    
    def __init__(self):
        super().__init__("SetLaneKeepCommand")
    
    def execute(self, blackboard: Blackboard) -> NodeStatus:
        """
        Set lane keep command on blackboard.
        """
        # TODO: Implement this action
        #
        # Hints:
        # 1. Get target_speed from config
        # 2. Get current lane from ego_state
        # 3. Create BehavioralCommand with maneuver='lane_keep'
        # 4. Return NodeStatus.SUCCESS
        
        return NodeStatus.SUCCESS  # Placeholder


# =============================================================================
# TESTING - feel free to modify the main fnction
# =============================================================================

if __name__ == '__main__':
    print("BT Nodes module loaded successfully.")
    print("\nCondition Nodes to implement:")
    print("  - IsObstacleBlocking")
    print("  - ShouldChangeLane")
    print("  - IsLaneChangeSafe")
    print("  - IsVehicleAhead")
    print("  - IsTooClose")
    print("\nAction Nodes to implement:")
    print("  - SetStopCommand")
    print("  - SetLaneChangeCommand")
    print("  - SetFollowCommand")
    print("  - SetLaneKeepCommand")
