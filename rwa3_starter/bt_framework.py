"""
bt_framework.py - Behavior Tree Framework

This module provides the base classes for building behavior trees.
DO NOT MODIFY THIS FILE.

Classes:
    Status: Enumeration of node return statuses
    Blackboard: Shared data structure for nodes
    Node: Abstract base class for all nodes
    Composite: Base class for nodes with children
    Sequence: Executes children until one fails
    Selector: Executes children until one succeeds
    ConditionNode: Checks a condition
    ActionNode: Performs an action
"""

from enum import Enum
from typing import List, Optional, Any, Dict
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


class Status(Enum):
    """Return status for behavior tree nodes."""
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class BehaviorType(Enum):
    """Types of driving behaviors."""
    LANE_KEEP = "lane_keep"
    FOLLOW_VEHICLE = "follow_vehicle"
    LANE_CHANGE_LEFT = "lane_change_left"
    LANE_CHANGE_RIGHT = "lane_change_right"
    STOP = "stop"


@dataclass
class EnvironmentState:
    """
    Environment state from perception.
    This is what the behavior tree uses to make decisions.
    """
    # Ego vehicle state
    ego_speed: float = 0.0           # Current speed (m/s)
    ego_d: float = 0.0               # Lateral offset from centerline (m)
    
    # Road information
    speed_limit: float = 31.0        # Speed limit (m/s)
    left_lane_exists: bool = True    # Is there a lane to the left?
    right_lane_exists: bool = True   # Is there a lane to the right?
    left_lane_clear: bool = True     # Is left lane safe to enter?
    right_lane_clear: bool = True    # Is right lane safe to enter?
    
    # Vehicle ahead detection
    vehicle_ahead: bool = False           # Is there a vehicle ahead?
    vehicle_ahead_distance: float = 100.0 # Distance to vehicle ahead (m)
    vehicle_ahead_speed: float = 0.0      # Speed of vehicle ahead (m/s)


@dataclass
class BehaviorCommand:
    """
    Command output from behavioral planner to trajectory planner.
    """
    behavior: BehaviorType = BehaviorType.LANE_KEEP
    target_d: float = 0.0          # Target lateral offset (m)
    target_speed: float = 31.0     # Target speed (m/s)
    T: float = 3.0                 # Planning horizon (s)


class Blackboard:
    """
    Shared data structure for behavior tree nodes.
    Nodes read perception data and write commands to the blackboard.
    """
    
    def __init__(self):
        self._data: Dict[str, Any] = {}
        self.env_state: EnvironmentState = EnvironmentState()
        self.behavior_command: BehaviorCommand = BehaviorCommand()
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get a value from the blackboard."""
        return self._data.get(key, default)
    
    def set(self, key: str, value: Any) -> None:
        """Set a value on the blackboard."""
        self._data[key] = value
    
    def clear(self) -> None:
        """Clear all custom data (keeps env_state and behavior_command)."""
        self._data.clear()


# Global blackboard instance
blackboard = Blackboard()


class Node(ABC):
    """Abstract base class for all behavior tree nodes."""
    
    def __init__(self, name: str = ""):
        self.name = name or self.__class__.__name__
        self.status: Status = Status.FAILURE
    
    @abstractmethod
    def update(self) -> Status:
        """Execute the node and return its status."""
        pass
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.name})"


class Composite(Node):
    """Base class for nodes that have children."""
    
    def __init__(self, name: str = "", children: List[Node] = None):
        super().__init__(name)
        self.children: List[Node] = children or []
    
    def add_child(self, child: Node) -> None:
        """Add a child node."""
        self.children.append(child)


class Sequence(Composite):
    """
    Sequence node: Executes children left-to-right.
    
    - Returns SUCCESS if ALL children succeed
    - Returns FAILURE immediately if any child fails
    - Returns RUNNING if a child returns RUNNING
    """
    
    def __init__(self, name: str = "", children: List[Node] = None):
        super().__init__(name, children)
        self.current_child_idx = 0
    
    def update(self) -> Status:
        """Execute children in sequence."""
        while self.current_child_idx < len(self.children):
            child = self.children[self.current_child_idx]
            status = child.update()
            
            if status == Status.FAILURE:
                self.current_child_idx = 0  # Reset for next tick
                self.status = Status.FAILURE
                return Status.FAILURE
            
            if status == Status.RUNNING:
                self.status = Status.RUNNING
                return Status.RUNNING
            
            # Child succeeded, move to next
            self.current_child_idx += 1
        
        # All children succeeded
        self.current_child_idx = 0  # Reset for next tick
        self.status = Status.SUCCESS
        return Status.SUCCESS


class Selector(Composite):
    """
    Selector node: Executes children left-to-right.
    
    - Returns SUCCESS immediately if any child succeeds
    - Returns FAILURE if ALL children fail
    - Returns RUNNING if a child returns RUNNING
    """
    
    def __init__(self, name: str = "", children: List[Node] = None):
        super().__init__(name, children)
        self.current_child_idx = 0
    
    def update(self) -> Status:
        """Execute children until one succeeds."""
        while self.current_child_idx < len(self.children):
            child = self.children[self.current_child_idx]
            status = child.update()
            
            if status == Status.SUCCESS:
                self.current_child_idx = 0  # Reset for next tick
                self.status = Status.SUCCESS
                return Status.SUCCESS
            
            if status == Status.RUNNING:
                self.status = Status.RUNNING
                return Status.RUNNING
            
            # Child failed, try next
            self.current_child_idx += 1
        
        # All children failed
        self.current_child_idx = 0  # Reset for next tick
        self.status = Status.FAILURE
        return Status.FAILURE


class ConditionNode(Node):
    """
    Base class for condition nodes.
    Conditions check something and return SUCCESS or FAILURE.
    Conditions should NEVER return RUNNING.
    """
    pass


class ActionNode(Node):
    """
    Base class for action nodes.
    Actions perform something and typically return SUCCESS.
    Actions MAY return RUNNING for multi-tick actions.
    """
    pass


def print_tree(node: Node, indent: int = 0) -> None:
    """Print the tree structure for debugging."""
    prefix = "  " * indent
    print(f"{prefix}{node}")
    if isinstance(node, Composite):
        for child in node.children:
            print_tree(child, indent + 1)


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing Behavior Tree Framework...\n")
    
    # Test 1: Sequence behavior
    print("1. Testing Sequence node:")
    
    class AlwaysSucceed(ActionNode):
        def update(self) -> Status:
            return Status.SUCCESS
    
    class AlwaysFail(ActionNode):
        def update(self) -> Status:
            return Status.FAILURE
    
    seq = Sequence("TestSequence", [AlwaysSucceed(), AlwaysSucceed()])
    result = seq.update()
    assert result == Status.SUCCESS, f"Expected SUCCESS, got {result}"
    print(f"   Sequence of [SUCCESS, SUCCESS] = {result.value} ✓")
    
    seq2 = Sequence("TestSequence2", [AlwaysSucceed(), AlwaysFail()])
    result2 = seq2.update()
    assert result2 == Status.FAILURE, f"Expected FAILURE, got {result2}"
    print(f"   Sequence of [SUCCESS, FAILURE] = {result2.value} ✓")
    
    # Test 2: Selector behavior
    print("\n2. Testing Selector node:")
    
    sel = Selector("TestSelector", [AlwaysFail(), AlwaysSucceed()])
    result = sel.update()
    assert result == Status.SUCCESS, f"Expected SUCCESS, got {result}"
    print(f"   Selector of [FAILURE, SUCCESS] = {result.value} ✓")
    
    sel2 = Selector("TestSelector2", [AlwaysFail(), AlwaysFail()])
    result2 = sel2.update()
    assert result2 == Status.FAILURE, f"Expected FAILURE, got {result2}"
    print(f"   Selector of [FAILURE, FAILURE] = {result2.value} ✓")
    
    # Test 3: Blackboard
    print("\n3. Testing Blackboard:")
    
    blackboard.set("test_key", 42)
    value = blackboard.get("test_key")
    assert value == 42, f"Expected 42, got {value}"
    print(f"   Set and get value: {value} ✓")
    
    blackboard.env_state.ego_speed = 25.0
    assert blackboard.env_state.ego_speed == 25.0
    print(f"   Environment state: ego_speed = {blackboard.env_state.ego_speed} ✓")
    
    # Test 4: Tree printing
    print("\n4. Tree structure example:")
    
    root = Selector("Root", [
        Sequence("Branch1", [AlwaysSucceed(), AlwaysSucceed()]),
        Sequence("Branch2", [AlwaysSucceed()])
    ])
    print_tree(root)
    
    print("\n✅ All framework tests passed!")
