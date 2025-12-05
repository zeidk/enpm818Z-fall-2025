"""
bt_nodes.py - Behavior Tree Nodes

This module contains the condition and action nodes for the behavioral planner.
Students must implement the TODO sections.

Condition Nodes (return SUCCESS or FAILURE):
    - IsVehicleAhead: Check if a blocking vehicle is ahead
    - IsVehicleSlow: Check if vehicle ahead is slow enough to overtake
    - IsLaneChangeSafe: Check if lane change is safe

Action Nodes (set behavior command and return SUCCESS):
    - SetLaneKeepCommand: Command to maintain current lane
    - SetFollowCommand: Command to follow vehicle ahead
    - SetLaneChangeCommand: Command to change lanes
"""

from bt_framework import (
    Status, BehaviorType, ConditionNode, ActionNode,
    BehaviorCommand, blackboard
)


# =============================================================================
# Configuration Parameters
# =============================================================================

SPEED_LIMIT = 31.0       # m/s (~70 mph)
LANE_WIDTH = 3.5         # meters
FOLLOW_DISTANCE = 50.0   # meters - distance to trigger following
SLOW_THRESHOLD = 5.0     # m/s - speed difference to consider vehicle "slow"


# =============================================================================
# Condition Nodes
# =============================================================================

class IsVehicleAhead(ConditionNode):
    """
    Check if there is a blocking vehicle ahead in the same lane.
    
    Returns:
        SUCCESS if a blocking vehicle is detected ahead
        FAILURE otherwise
    """
    
    def __init__(self, distance_threshold: float = FOLLOW_DISTANCE):
        super().__init__("IsVehicleAhead")
        self.distance_threshold = distance_threshold
    
    def update(self) -> Status:
        """
        TODO: Implement this method.
        
        Logic:
        1. Get environment state from blackboard
        2. Check if vehicle_ahead is True
        3. Check if vehicle_ahead_distance < distance_threshold
        4. Check if vehicle is below speed limit (blocking our cruise)
        
        A vehicle is blocking if:
        - It exists within the distance threshold AND
        - It's slower than the speed limit by more than 1 m/s
        
        Returns:
            Status.SUCCESS if blocking vehicle ahead
            Status.FAILURE otherwise
        """
        # TODO: Implement this condition
        # Hint: Access environment with blackboard.env_state
        
        pass  # Remove this line when implementing


class IsVehicleSlow(ConditionNode):
    """
    Check if the vehicle ahead is slow enough to warrant overtaking.
    
    Returns:
        SUCCESS if vehicle ahead is significantly slower than speed limit
        FAILURE otherwise
    """
    
    def __init__(self, slow_threshold: float = SLOW_THRESHOLD):
        super().__init__("IsVehicleSlow")
        self.slow_threshold = slow_threshold
    
    def update(self) -> Status:
        """
        TODO: Implement this method.
        
        Logic:
        1. Get environment state from blackboard
        2. If no vehicle ahead, return FAILURE
        3. Calculate speed difference: speed_limit - vehicle_ahead_speed
        4. If speed_diff > slow_threshold, return SUCCESS
        
        Returns:
            Status.SUCCESS if vehicle ahead is slow
            Status.FAILURE otherwise
        """
        # TODO: Implement this condition
        
        pass  # Remove this line when implementing


class IsLaneChangeSafe(ConditionNode):
    """
    Check if it's safe to change lanes.
    Prefers left lane for overtaking (standard highway convention).
    
    Returns:
        SUCCESS if a lane change is safe (stores target lane on blackboard)
        FAILURE if no safe lane change is possible
    """
    
    def __init__(self):
        super().__init__("IsLaneChangeSafe")
    
    def update(self) -> Status:
        """
        TODO: Implement this method.
        
        Logic:
        1. Get environment state from blackboard
        2. Check left lane first (preferred for passing):
           - If left_lane_exists AND left_lane_clear:
             - Store 'left' on blackboard: blackboard.set('target_lane', 'left')
             - Return SUCCESS
        3. Check right lane as fallback:
           - If right_lane_exists AND right_lane_clear:
             - Store 'right' on blackboard: blackboard.set('target_lane', 'right')
             - Return SUCCESS
        4. If neither lane is safe, return FAILURE
        
        Returns:
            Status.SUCCESS if lane change is safe
            Status.FAILURE otherwise
        """
        # TODO: Implement this condition
        
        pass  # Remove this line when implementing


# =============================================================================
# Action Nodes
# =============================================================================

class SetLaneKeepCommand(ActionNode):
    """
    Set command to maintain current lane at cruising speed.
    This is the default behavior when no other action is needed.
    """
    
    def __init__(self, speed_limit: float = SPEED_LIMIT):
        super().__init__("SetLaneKeepCommand")
        self.speed_limit = speed_limit
    
    def update(self) -> Status:
        """
        TODO: Implement this method.
        
        Logic:
        1. Create a BehaviorCommand with:
           - behavior = BehaviorType.LANE_KEEP
           - target_d = 0.0 (center of lane)
           - target_speed = speed_limit
           - T = 3.0 (planning horizon)
        2. Store on blackboard: blackboard.behavior_command = command
        3. Return SUCCESS
        
        Returns:
            Status.SUCCESS always
        """
        # TODO: Implement this action
        
        pass  # Remove this line when implementing


class SetFollowCommand(ActionNode):
    """
    Set command to follow the vehicle ahead.
    Matches the lead vehicle's speed with a small buffer.
    """
    
    def __init__(self, speed_buffer: float = 1.0):
        super().__init__("SetFollowCommand")
        self.speed_buffer = speed_buffer
    
    def update(self) -> Status:
        """
        TODO: Implement this method.
        
        Logic:
        1. Get environment state from blackboard
        2. Calculate follow speed: vehicle_ahead_speed - speed_buffer
        3. Ensure follow_speed >= 0
        4. Create a BehaviorCommand with:
           - behavior = BehaviorType.FOLLOW_VEHICLE
           - target_d = 0.0 (stay in current lane)
           - target_speed = follow_speed
           - T = 5.0 (longer horizon for following)
        5. Store on blackboard
        6. Return SUCCESS
        
        Returns:
            Status.SUCCESS always
        """
        # TODO: Implement this action
        
        pass  # Remove this line when implementing


class SetLaneChangeCommand(ActionNode):
    """
    Set command to change lanes.
    Direction is determined by the 'target_lane' stored on blackboard.
    """
    
    def __init__(self, lane_width: float = LANE_WIDTH, speed_limit: float = SPEED_LIMIT):
        super().__init__("SetLaneChangeCommand")
        self.lane_width = lane_width
        self.speed_limit = speed_limit
    
    def update(self) -> Status:
        """
        TODO: Implement this method.
        
        Logic:
        1. Get target_lane from blackboard: blackboard.get('target_lane')
        2. Determine behavior and target_d based on target_lane:
           - If 'left': behavior = LANE_CHANGE_LEFT, target_d = +lane_width
           - If 'right': behavior = LANE_CHANGE_RIGHT, target_d = -lane_width
        3. Create a BehaviorCommand with:
           - behavior = (determined above)
           - target_d = (determined above)
           - target_speed = speed_limit
           - T = 4.0 (time for lane change)
        4. Store on blackboard
        5. Return SUCCESS
        
        Returns:
            Status.SUCCESS always
        """
        # TODO: Implement this action
        
        pass  # Remove this line when implementing


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Behavior Tree Nodes...\n")
    
    # Test environment setup
    env = blackboard.env_state
    
    # Test 1: IsVehicleAhead
    print("1. Testing IsVehicleAhead:")
    
    node = IsVehicleAhead()
    
    # No vehicle ahead
    env.vehicle_ahead = False
    result = node.update()
    expected = Status.FAILURE
    status = "✓" if result == expected else "✗"
    print(f"   No vehicle ahead: {result} (expected {expected}) {status}")
    
    # Vehicle ahead but far
    env.vehicle_ahead = True
    env.vehicle_ahead_distance = 50.0
    env.vehicle_ahead_speed = 25.0
    env.ego_speed = 30.0
    result = node.update()
    expected = Status.FAILURE
    status = "✓" if result == expected else "✗"
    print(f"   Vehicle far (50m): {result} (expected {expected}) {status}")
    
    # Vehicle ahead and close, blocking
    env.vehicle_ahead_distance = 20.0
    env.vehicle_ahead_speed = 20.0
    env.ego_speed = 30.0
    result = node.update()
    expected = Status.SUCCESS
    status = "✓" if result == expected else "✗"
    print(f"   Vehicle close (20m), slow: {result} (expected {expected}) {status}")
    
    # Test 2: IsVehicleSlow
    print("\n2. Testing IsVehicleSlow:")
    
    node = IsVehicleSlow()
    
    # No vehicle ahead
    env.vehicle_ahead = False
    result = node.update()
    expected = Status.FAILURE
    status = "✓" if result == expected else "✗"
    print(f"   No vehicle ahead: {result} (expected {expected}) {status}")
    
    # Vehicle ahead, fast
    env.vehicle_ahead = True
    env.vehicle_ahead_speed = 29.0  # Close to speed limit
    env.speed_limit = 31.0
    result = node.update()
    expected = Status.FAILURE
    status = "✓" if result == expected else "✗"
    print(f"   Vehicle fast (29 m/s): {result} (expected {expected}) {status}")
    
    # Vehicle ahead, slow
    env.vehicle_ahead_speed = 22.0  # 9 m/s below limit
    result = node.update()
    expected = Status.SUCCESS
    status = "✓" if result == expected else "✗"
    print(f"   Vehicle slow (22 m/s): {result} (expected {expected}) {status}")
    
    # Test 3: IsLaneChangeSafe
    print("\n3. Testing IsLaneChangeSafe:")
    
    node = IsLaneChangeSafe()
    
    # Left lane available
    env.left_lane_exists = True
    env.left_lane_clear = True
    env.right_lane_exists = True
    env.right_lane_clear = True
    result = node.update()
    expected = Status.SUCCESS
    status = "✓" if result == expected else "✗"
    target = blackboard.get('target_lane')
    print(f"   Both lanes clear: {result}, target={target} (expected left) {status}")
    
    # Only right lane available
    env.left_lane_clear = False
    result = node.update()
    expected = Status.SUCCESS
    target = blackboard.get('target_lane')
    status = "✓" if result == expected and target == 'right' else "✗"
    print(f"   Only right clear: {result}, target={target} (expected right) {status}")
    
    # No lanes available
    env.right_lane_clear = False
    result = node.update()
    expected = Status.FAILURE
    status = "✓" if result == expected else "✗"
    print(f"   No lanes clear: {result} (expected {expected}) {status}")
    
    # Test 4: SetLaneKeepCommand
    print("\n4. Testing SetLaneKeepCommand:")
    
    node = SetLaneKeepCommand()
    result = node.update()
    cmd = blackboard.behavior_command
    expected = Status.SUCCESS
    status = "✓" if (result == expected and 
                     cmd.behavior == BehaviorType.LANE_KEEP and
                     cmd.target_d == 0.0) else "✗"
    print(f"   Lane keep: {cmd.behavior.value}, d={cmd.target_d}, v={cmd.target_speed} {status}")
    
    # Test 5: SetFollowCommand
    print("\n5. Testing SetFollowCommand:")
    
    env.vehicle_ahead_speed = 22.0
    node = SetFollowCommand()
    result = node.update()
    cmd = blackboard.behavior_command
    expected_speed = 20.0  # 22 - 2 buffer
    status = "✓" if (cmd.behavior == BehaviorType.FOLLOW_VEHICLE and
                     abs(cmd.target_speed - expected_speed) < 0.1) else "✗"
    print(f"   Follow: {cmd.behavior.value}, v={cmd.target_speed} (expected {expected_speed}) {status}")
    
    # Test 6: SetLaneChangeCommand
    print("\n6. Testing SetLaneChangeCommand:")
    
    blackboard.set('target_lane', 'left')
    node = SetLaneChangeCommand()
    result = node.update()
    cmd = blackboard.behavior_command
    status = "✓" if (cmd.behavior == BehaviorType.LANE_CHANGE_LEFT and
                     cmd.target_d == LANE_WIDTH) else "✗"
    print(f"   Lane change left: {cmd.behavior.value}, d={cmd.target_d} {status}")
    
    blackboard.set('target_lane', 'right')
    result = node.update()
    cmd = blackboard.behavior_command
    status = "✓" if (cmd.behavior == BehaviorType.LANE_CHANGE_RIGHT and
                     cmd.target_d == -LANE_WIDTH) else "✗"
    print(f"   Lane change right: {cmd.behavior.value}, d={cmd.target_d} {status}")
    
    print("\n" + "="*50)
    print("Run this file after implementing all TODOs to verify.")
    print("All tests should show ✓")
