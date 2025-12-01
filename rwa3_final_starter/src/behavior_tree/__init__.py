"""
Behavior Tree Framework (PROVIDED)

This module provides the base classes for implementing a Behavior Tree:
- NodeStatus: Enumeration of possible node return values
- Blackboard: Shared data structure for the tree
- BTNode: Base class for all nodes
- ConditionNode: Base class for condition checks
- ActionNode: Base class for actions
- Sequence: Composite node that runs children in sequence
- Selector: Composite node that tries children until one succeeds

Students should NOT modify this file.
Use these base classes in bt_nodes.py to implement your custom nodes.
"""

from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field


class NodeStatus(Enum):
    """
    Possible return values for Behavior Tree nodes.
    
    SUCCESS: The node completed successfully
    FAILURE: The node failed to complete
    RUNNING: The node is still executing (will continue next tick)
    """
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


@dataclass
class BehavioralCommand:
    """
    Command output from behavioral planner.
    
    This is the interface between behavioral planning and trajectory planning.
    """
    maneuver: str = 'lane_keep'  # 'lane_keep', 'follow', 'lane_change_left', 'lane_change_right', 'stop'
    target_lane: int = 0
    target_speed: float = 25.0  # m/s
    
    def __repr__(self):
        return f"BehavioralCommand(maneuver={self.maneuver}, target_lane={self.target_lane}, target_speed={self.target_speed:.1f})"


class Blackboard:
    """
    Shared data structure for Behavior Tree nodes.
    
    The blackboard allows nodes to read and write data that persists
    across ticks and is accessible to all nodes in the tree.
    
    Standard keys:
    - ego_state: Dict with x, y, theta, v, lane_id
    - obstacles: List of obstacle dicts
    - lane_info: Dict with lane configuration
    - reference_path: List of (x, y) waypoints
    - route: List of target lane IDs
    - behavioral_command: BehavioralCommand object (output)
    - config: Configuration parameters
    """
    
    def __init__(self):
        self._data: Dict[str, Any] = {}
        
        # Initialize with defaults
        self._data['behavioral_command'] = BehavioralCommand()
        self._data['lane_change_in_progress'] = False
        self._data['lane_change_start_time'] = None
        
    def get(self, key: str, default: Any = None) -> Any:
        """Get a value from the blackboard."""
        return self._data.get(key, default)
    
    def set(self, key: str, value: Any):
        """Set a value on the blackboard."""
        self._data[key] = value
    
    def has(self, key: str) -> bool:
        """Check if a key exists on the blackboard."""
        return key in self._data
    
    def clear(self):
        """Clear all data from the blackboard."""
        self._data.clear()
        self._data['behavioral_command'] = BehavioralCommand()
        self._data['lane_change_in_progress'] = False
        self._data['lane_change_start_time'] = None
    
    def update_perception(self, ego_state: Dict, obstacles: List[Dict],
                          lane_info: Dict, reference_path: List[tuple]):
        """
        Update blackboard with perception data.
        
        Args:
            ego_state: Dict with x, y, theta, v, lane_id
            obstacles: List of obstacle dicts
            lane_info: Dict with lane configuration
            reference_path: List of (x, y) waypoints
        """
        self._data['ego_state'] = ego_state
        self._data['obstacles'] = obstacles
        self._data['lane_info'] = lane_info
        self._data['reference_path'] = reference_path
    
    @property
    def ego_state(self) -> Dict:
        return self._data.get('ego_state', {})
    
    @property
    def obstacles(self) -> List[Dict]:
        return self._data.get('obstacles', [])
    
    @property
    def lane_info(self) -> Dict:
        return self._data.get('lane_info', {})
    
    @property
    def reference_path(self) -> List[tuple]:
        return self._data.get('reference_path', [])
    
    @property
    def route(self) -> List[int]:
        return self._data.get('route', [])
    
    @property
    def behavioral_command(self) -> BehavioralCommand:
        return self._data.get('behavioral_command', BehavioralCommand())
    
    @behavioral_command.setter
    def behavioral_command(self, cmd: BehavioralCommand):
        self._data['behavioral_command'] = cmd
    
    @property
    def config(self) -> Dict:
        return self._data.get('config', {})


class BTNode(ABC):
    """
    Base class for all Behavior Tree nodes.
    
    All nodes must implement the tick() method which is called
    each time the tree is executed.
    """
    
    def __init__(self, name: str = ""):
        """
        Initialize a BT node.
        
        Args:
            name: Human-readable name for debugging
        """
        self.name = name or self.__class__.__name__
        self._status = NodeStatus.FAILURE
    
    @abstractmethod
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        """
        Execute the node.
        
        Args:
            blackboard: Shared data structure
            
        Returns:
            NodeStatus indicating the result
        """
        pass
    
    @property
    def status(self) -> NodeStatus:
        """Get the last status returned by tick()."""
        return self._status
    
    def __repr__(self):
        return f"{self.__class__.__name__}({self.name})"


class ConditionNode(BTNode):
    """
    Base class for condition nodes.
    
    Condition nodes check some condition and return SUCCESS or FAILURE.
    They should NEVER return RUNNING.
    
    Inherit from this class and implement evaluate() to create custom conditions.
    """
    
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        """
        Execute the condition check.
        
        Calls evaluate() and converts the boolean result to NodeStatus.
        """
        result = self.evaluate(blackboard)
        self._status = NodeStatus.SUCCESS if result else NodeStatus.FAILURE
        return self._status
    
    @abstractmethod
    def evaluate(self, blackboard: Blackboard) -> bool:
        """
        Evaluate the condition.
        
        Args:
            blackboard: Shared data structure
            
        Returns:
            True if condition is met, False otherwise
        """
        pass


class ActionNode(BTNode):
    """
    Base class for action nodes.
    
    Action nodes perform some action and can return SUCCESS, FAILURE, or RUNNING.
    
    Inherit from this class and implement execute() to create custom actions.
    """
    
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        """Execute the action."""
        self._status = self.execute(blackboard)
        return self._status
    
    @abstractmethod
    def execute(self, blackboard: Blackboard) -> NodeStatus:
        """
        Execute the action.
        
        Args:
            blackboard: Shared data structure
            
        Returns:
            NodeStatus indicating the result
        """
        pass


class Sequence(BTNode):
    """
    Sequence composite node.
    
    Executes children from left to right.
    - Returns SUCCESS if ALL children return SUCCESS
    - Returns FAILURE immediately if any child returns FAILURE
    - Returns RUNNING if a child returns RUNNING (will resume from there next tick)
    
    Use case: AND logic - "do A, then B, then C" (all must succeed)
    """
    
    def __init__(self, name: str = "", children: List[BTNode] = None):
        """
        Initialize a Sequence node.
        
        Args:
            name: Human-readable name
            children: List of child nodes
        """
        super().__init__(name)
        self.children = children or []
        self._current_index = 0
    
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        """
        Execute children in sequence.
        """
        while self._current_index < len(self.children):
            child = self.children[self._current_index]
            status = child.tick(blackboard)
            
            if status == NodeStatus.FAILURE:
                # Reset for next tick
                self._current_index = 0
                self._status = NodeStatus.FAILURE
                return self._status
            
            elif status == NodeStatus.RUNNING:
                # Will continue from this child next tick
                self._status = NodeStatus.RUNNING
                return self._status
            
            # SUCCESS - move to next child
            self._current_index += 1
        
        # All children succeeded
        self._current_index = 0
        self._status = NodeStatus.SUCCESS
        return self._status
    
    def add_child(self, child: BTNode):
        """Add a child node."""
        self.children.append(child)


class Selector(BTNode):
    """
    Selector composite node (also called Fallback).
    
    Executes children from left to right.
    - Returns SUCCESS immediately if any child returns SUCCESS
    - Returns FAILURE if ALL children return FAILURE
    - Returns RUNNING if a child returns RUNNING (will resume from there next tick)
    
    Use case: OR logic - "try A, if that fails try B, if that fails try C"
    """
    
    def __init__(self, name: str = "", children: List[BTNode] = None):
        """
        Initialize a Selector node.
        
        Args:
            name: Human-readable name
            children: List of child nodes
        """
        super().__init__(name)
        self.children = children or []
        self._current_index = 0
    
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        """
        Execute children until one succeeds.
        """
        while self._current_index < len(self.children):
            child = self.children[self._current_index]
            status = child.tick(blackboard)
            
            if status == NodeStatus.SUCCESS:
                # Reset for next tick
                self._current_index = 0
                self._status = NodeStatus.SUCCESS
                return self._status
            
            elif status == NodeStatus.RUNNING:
                # Will continue from this child next tick
                self._status = NodeStatus.RUNNING
                return self._status
            
            # FAILURE - try next child
            self._current_index += 1
        
        # All children failed
        self._current_index = 0
        self._status = NodeStatus.FAILURE
        return self._status
    
    def add_child(self, child: BTNode):
        """Add a child node."""
        self.children.append(child)


class BehaviorTree:
    """
    Behavior Tree executor.
    
    Manages the tree structure and executes ticks.
    """
    
    def __init__(self, root: BTNode):
        """
        Initialize the Behavior Tree.
        
        Args:
            root: Root node of the tree
        """
        self.root = root
        self.blackboard = Blackboard()
    
    def tick(self) -> NodeStatus:
        """
        Execute one tick of the tree.
        
        Returns:
            Status of the root node
        """
        return self.root.tick(self.blackboard)
    
    def set_blackboard(self, blackboard: Blackboard):
        """Set a custom blackboard."""
        self.blackboard = blackboard
    
    def print_tree(self, node: BTNode = None, indent: int = 0):
        """Print the tree structure for debugging."""
        if node is None:
            node = self.root
        
        prefix = "  " * indent
        print(f"{prefix}{node.name} [{node.__class__.__name__}]")
        
        if hasattr(node, 'children'):
            for child in node.children:
                self.print_tree(child, indent + 1)


# Utility decorators (optional - for advanced use)

class Inverter(BTNode):
    """
    Decorator that inverts the result of its child.
    
    SUCCESS -> FAILURE
    FAILURE -> SUCCESS
    RUNNING -> RUNNING
    """
    
    def __init__(self, child: BTNode, name: str = ""):
        super().__init__(name or f"Inverter({child.name})")
        self.child = child
    
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        status = self.child.tick(blackboard)
        
        if status == NodeStatus.SUCCESS:
            self._status = NodeStatus.FAILURE
        elif status == NodeStatus.FAILURE:
            self._status = NodeStatus.SUCCESS
        else:
            self._status = NodeStatus.RUNNING
        
        return self._status


class Repeater(BTNode):
    """
    Decorator that repeats its child a specified number of times.
    """
    
    def __init__(self, child: BTNode, count: int = -1, name: str = ""):
        """
        Args:
            child: Child node to repeat
            count: Number of times to repeat (-1 for infinite)
        """
        super().__init__(name or f"Repeater({child.name})")
        self.child = child
        self.count = count
        self._iteration = 0
    
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.count != -1 and self._iteration >= self.count:
            self._iteration = 0
            self._status = NodeStatus.SUCCESS
            return self._status
        
        status = self.child.tick(blackboard)
        
        if status == NodeStatus.SUCCESS or status == NodeStatus.FAILURE:
            self._iteration += 1
            self._status = NodeStatus.RUNNING
        else:
            self._status = status
        
        return self._status


if __name__ == '__main__':
    # Simple test of the framework
    print("Behavior Tree Framework - Test")
    print("=" * 40)
    
    # Create a simple test tree
    class TestCondition(ConditionNode):
        def __init__(self, result: bool):
            super().__init__(f"TestCondition({result})")
            self.result = result
        
        def evaluate(self, blackboard: Blackboard) -> bool:
            return self.result
    
    class TestAction(ActionNode):
        def __init__(self, name: str):
            super().__init__(name)
        
        def execute(self, blackboard: Blackboard) -> NodeStatus:
            print(f"  Executing: {self.name}")
            return NodeStatus.SUCCESS
    
    # Build tree:
    # Selector
    #   ├── Sequence (will fail)
    #   │   ├── TestCondition(False)
    #   │   └── TestAction("A")
    #   └── Sequence (will succeed)
    #       ├── TestCondition(True)
    #       └── TestAction("B")
    
    tree = BehaviorTree(
        Selector("Root", [
            Sequence("Branch1", [
                TestCondition(False),
                TestAction("A")
            ]),
            Sequence("Branch2", [
                TestCondition(True),
                TestAction("B")
            ])
        ])
    )
    
    print("\nTree structure:")
    tree.print_tree()
    
    print("\nExecuting tick:")
    status = tree.tick()
    print(f"Result: {status}")
    
    print("\nFramework test complete!")
