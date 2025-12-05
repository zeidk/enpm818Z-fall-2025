"""
behavior_tree.py - Behavioral Planner using Behavior Trees

This module implements the behavioral planner that decides high-level
driving maneuvers using a Behavior Tree.

Students must implement the _build_tree() method to construct the tree.
"""

from typing import Optional
from dataclasses import dataclass

from bt_framework import (
    Status, BehaviorType, BehaviorCommand, EnvironmentState,
    Node, Sequence, Selector, blackboard, print_tree
)
from bt_nodes import (
    IsVehicleAhead, IsVehicleSlow, IsLaneChangeSafe,
    SetLaneKeepCommand, SetFollowCommand, SetLaneChangeCommand
)


class BehaviorPlanner:
    """
    Behavioral planner using a Behavior Tree for decision making.
    
    The planner evaluates the current driving situation and outputs
    a BehaviorCommand that tells the trajectory planner what maneuver
    to execute.
    """
    
    def __init__(self, speed_limit: float = 31.0, lane_width: float = 3.5):
        """
        Initialize the behavior planner.
        
        Args:
            speed_limit: Maximum cruising speed in m/s
            lane_width: Lane width in meters
        """
        self.speed_limit = speed_limit
        self.lane_width = lane_width
        
        # Build the behavior tree
        self.root = self._build_tree()
    
    def _build_tree(self) -> Node:
        """
        TODO: Implement this method to build the behavior tree.
        
        Required Tree Structure:
        
            Root [Selector]
            ├── Lane Change [Sequence]
            │   ├── IsVehicleAhead
            │   ├── IsVehicleSlow
            │   ├── IsLaneChangeSafe
            │   └── SetLaneChangeCommand
            ├── Follow Vehicle [Sequence]
            │   ├── IsVehicleAhead
            │   └── SetFollowCommand
            └── Lane Keep [Sequence]
                └── SetLaneKeepCommand
        
        The tree tries each branch in order:
        1. First, try lane change if there's a slow vehicle and it's safe
        2. If can't change lanes, follow the vehicle ahead
        3. If no vehicle ahead, just keep lane
        
        Returns:
            Root node of the behavior tree
        
        Example implementation:
            lane_change_seq = Sequence("LaneChange", [
                IsVehicleAhead(),
                IsVehicleSlow(),
                IsLaneChangeSafe(),
                SetLaneChangeCommand(self.lane_width, self.speed_limit)
            ])
            ... (create other sequences)
            root = Selector("Root", [lane_change_seq, follow_seq, keep_seq])
            return root
        """
        # TODO: Implement the behavior tree structure
        # Hint: Use Sequence for each behavior branch, Selector for the root
        
        pass  # Remove this line when implementing
    
    def get_command(self, env_state: EnvironmentState) -> BehaviorCommand:
        """
        Get the behavior command for the current environment state.
        
        Args:
            env_state: Current environment state from perception
            
        Returns:
            BehaviorCommand for the trajectory planner
        """
        # Update blackboard with current environment
        blackboard.env_state = env_state
        
        # Tick the behavior tree
        status = self.root.update()
        
        # Return the command set by the tree
        return blackboard.behavior_command
    
    def print_tree(self) -> None:
        """Print the tree structure for debugging."""
        print("\nBehavior Tree Structure:")
        print("=" * 40)
        print_tree(self.root)
        print("=" * 40)


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Behavior Planner...\n")
    
    # Create planner
    planner = BehaviorPlanner(speed_limit=31.0, lane_width=3.5)
    
    # Print tree structure
    planner.print_tree()
    
    # Test 1: Empty road - should lane keep
    print("\n1. Testing empty road (lane keep):")
    env = EnvironmentState()
    env.ego_speed = 25.0
    env.speed_limit = 31.0
    env.vehicle_ahead = False
    
    cmd = planner.get_command(env)
    expected = BehaviorType.LANE_KEEP
    status = "✓" if cmd.behavior == expected else "✗"
    print(f"   Behavior: {cmd.behavior.value} (expected {expected.value}) {status}")
    print(f"   Target: d={cmd.target_d}m, v={cmd.target_speed}m/s, T={cmd.T}s")
    
    # Test 2: Vehicle ahead, can't pass - should follow
    print("\n2. Testing vehicle ahead, no passing (follow):")
    env.vehicle_ahead = True
    env.vehicle_ahead_distance = 20.0
    env.vehicle_ahead_speed = 22.0
    env.left_lane_exists = True
    env.left_lane_clear = False  # Can't pass on left
    env.right_lane_exists = True
    env.right_lane_clear = False  # Can't pass on right
    
    cmd = planner.get_command(env)
    expected = BehaviorType.FOLLOW_VEHICLE
    status = "✓" if cmd.behavior == expected else "✗"
    print(f"   Behavior: {cmd.behavior.value} (expected {expected.value}) {status}")
    print(f"   Target: d={cmd.target_d}m, v={cmd.target_speed}m/s, T={cmd.T}s")
    
    # Test 3: Slow vehicle ahead, left lane clear - should lane change left
    print("\n3. Testing slow vehicle, left clear (lane change left):")
    env.vehicle_ahead = True
    env.vehicle_ahead_distance = 20.0
    env.vehicle_ahead_speed = 22.0  # Slow
    env.left_lane_clear = True  # Can pass on left
    
    cmd = planner.get_command(env)
    expected = BehaviorType.LANE_CHANGE_LEFT
    status = "✓" if cmd.behavior == expected else "✗"
    print(f"   Behavior: {cmd.behavior.value} (expected {expected.value}) {status}")
    print(f"   Target: d={cmd.target_d}m, v={cmd.target_speed}m/s, T={cmd.T}s")
    
    # Test 4: Slow vehicle ahead, only right lane clear - should lane change right
    print("\n4. Testing slow vehicle, only right clear (lane change right):")
    env.left_lane_clear = False
    env.right_lane_clear = True
    
    cmd = planner.get_command(env)
    expected = BehaviorType.LANE_CHANGE_RIGHT
    status = "✓" if cmd.behavior == expected else "✗"
    print(f"   Behavior: {cmd.behavior.value} (expected {expected.value}) {status}")
    print(f"   Target: d={cmd.target_d}m, v={cmd.target_speed}m/s, T={cmd.T}s")
    
    # Test 5: Fast vehicle ahead - should follow (not overtake)
    print("\n5. Testing fast vehicle ahead (follow, not overtake):")
    env.vehicle_ahead_speed = 29.0  # Fast, close to speed limit
    env.left_lane_clear = True
    env.right_lane_clear = True
    
    cmd = planner.get_command(env)
    expected = BehaviorType.FOLLOW_VEHICLE
    status = "✓" if cmd.behavior == expected else "✗"
    print(f"   Behavior: {cmd.behavior.value} (expected {expected.value}) {status}")
    print(f"   Target: d={cmd.target_d}m, v={cmd.target_speed}m/s, T={cmd.T}s")
    
    print("\n" + "="*50)
    print("All 5 tests should show ✓ when implementation is complete.")
