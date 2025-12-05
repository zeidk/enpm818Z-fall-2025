# RWA3: Behavioral Planning with Behavior Trees

## Overview

This assignment implements a behavioral planner for autonomous driving using Behavior Trees.
The planner decides high-level driving maneuvers (lane keep, follow, lane change) based on
the current driving situation.

## Files to Implement

You need to implement the following files:

1. **`bt_nodes.py`** - Condition and action nodes for the behavior tree
   - `IsVehicleAhead.update()` - Check if blocking vehicle ahead
   - `IsVehicleSlow.update()` - Check if vehicle ahead is slow
   - `IsLaneChangeSafe.update()` - Check if lane change is safe
   - `SetLaneKeepCommand.update()` - Set lane keeping command
   - `SetFollowCommand.update()` - Set vehicle following command
   - `SetLaneChangeCommand.update()` - Set lane change command

2. **`behavior_tree.py`** - Assemble the behavior tree
   - `BehaviorPlanner._build_tree()` - Build the tree structure

## Provided Files (Do Not Modify)

- `bt_framework.py` - Behavior tree base classes
- `test_behavior_tree.py` - Unit tests
- `simulator.py` - Highway driving simulator
- `visualizer.py` - Real-time matplotlib visualization

## Setup

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
```

## Testing

```bash
# Run unit tests (required for grading)
python test_behavior_tree.py

# Test individual modules
python bt_nodes.py
python behavior_tree.py
```

## Visualization (Optional)

Once your implementation passes unit tests, visualize it in action:

```bash
# Run with visualization
python simulator.py --scenario empty
python simulator.py --scenario follow
python simulator.py --scenario overtake

# Run in text-only mode
python simulator.py --no-viz --scenario overtake --duration 30
```

## Expected Behavior Tree Structure

```
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
```

## Key Concepts

- **Selector**: Tries children until one succeeds (OR logic)
- **Sequence**: Executes children until one fails (AND logic)
- **Status.SUCCESS**: Node completed successfully
- **Status.FAILURE**: Node failed to complete
- **blackboard**: Shared data structure for communication between nodes

## Submission

Submit a ZIP file containing:
- `bt_nodes.py`
- `behavior_tree.py`
- `results/` folder with test output
- `README.md` with your implementation notes a ZIP file containing:
- `bt_nodes.py`
- `behavior_tree.py`
- `results/` folder with screenshots and test output
- `README.md` with your implementation notes
