# ENPM818Z Final Project: Behavioral and Trajectory Planning in CARLA

## Overview

This project implements a complete autonomous driving pipeline consisting of:
- **Behavioral Planner**: Uses a Behavior Tree to decide driving maneuvers
- **Trajectory Planner**: Generates smooth trajectories using the Frenet optimal trajectory method
- **Controller** (Provided): Stanley lateral control + PID longitudinal control

## Project Structure

```
enpm818z_final_project/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── config/
│   └── planner_config.yaml      # Configuration parameters
├── src/
│   ├── __init__.py
│   ├── carla_interface.py       # CARLA connection and vehicle control (PROVIDED)
│   ├── perception.py            # Perception module (PROVIDED)
│   ├── behavior_tree/
│   │   ├── __init__.py
│   │   ├── bt_framework.py      # Behavior Tree framework (PROVIDED)
│   │   └── bt_nodes.py          # YOUR IMPLEMENTATION: Custom BT nodes
│   ├── behavioral_planner.py    # YOUR IMPLEMENTATION: Assemble BT and run tick
│   ├── frenet_transform.py      # YOUR IMPLEMENTATION: Coordinate transforms
│   ├── polynomial_trajectory.py # YOUR IMPLEMENTATION: Polynomial generation
│   ├── trajectory_planner.py    # YOUR IMPLEMENTATION: Frenet trajectory planner
│   ├── controller.py            # Vehicle controller (PROVIDED)
│   └── visualization.py         # Visualization tools (PROVIDED)
├── scenarios/
│   ├── highway_scenario.py      # Highway test scenario loader
│   └── scenario_runner.py       # Main scenario execution script
├── tests/
│   ├── test_frenet.py           # Unit tests for Frenet transform
│   ├── test_polynomial.py       # Unit tests for polynomial generation
│   └── test_bt_nodes.py         # Unit tests for BT nodes
└── docs/
    └── architecture.md          # System architecture documentation
```

## Prerequisites

### 1. CARLA Simulator

Download and install CARLA 0.9.13 or later:
- https://carla.readthedocs.io/en/latest/start_quickstart/

### 2. Python Environment

Python 3.8 or later is required.

```bash
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Running the Project

### Step 1: Start CARLA Server

Open a terminal and start the CARLA server:

```bash
# Navigate to your CARLA installation directory
cd /path/to/carla

# Start CARLA server (Linux)
./CarlaUE4.sh -quality-level=Low

# Or on Windows:
# CarlaUE4.exe -quality-level=Low
```

Wait until the CARLA window appears and the world is loaded.

### Step 2: Run Unit Tests (Recommended First)

Before running the full scenario, test your individual modules:

```bash
# Test Frenet transformation
python -m tests.test_frenet

# Test polynomial generation
python -m tests.test_polynomial

# Test Behavior Tree nodes
python -m tests.test_bt_nodes
```

### Step 3: Run Highway Scenario

Once CARLA is running, execute the scenario:

```bash
# Run the highway scenario
python -m scenarios.scenario_runner --scenario highway

# With visualization enabled
python -m scenarios.scenario_runner --scenario highway --visualize

# Specify a specific test case
python -m scenarios.scenario_runner --scenario highway --test lane_keep
python -m scenarios.scenario_runner --scenario highway --test follow
python -m scenarios.scenario_runner --scenario highway --test lane_change
```

### Step 4: Run Individual Components for Debugging

You can test individual components separately:

```bash
# Test just the controller with a simple trajectory
python -m src.controller --test

# Test the Behavior Tree with mock data
python -m src.behavioral_planner --test

# Test the trajectory planner with mock behavioral commands
python -m src.trajectory_planner --test
```

## Configuration

Edit `config/planner_config.yaml` to adjust parameters:

```yaml
behavioral_planner:
  target_speed: 25.0          # m/s - cruising speed
  safe_follow_distance: 20.0  # m - min distance to lead vehicle
  lane_change_min_gap: 25.0   # m - min gap for lane change
  stop_distance: 10.0         # m - distance to stop before obstacle

trajectory_planner:
  max_speed: 30.0             # m/s
  max_accel: 3.0              # m/s^2
  max_decel: -6.0             # m/s^2
  max_curvature: 0.2          # 1/m
  planning_horizon: 5.0       # seconds
  dt: 0.1                     # trajectory timestep
```

## What You Need to Implement

### 1. `src/behavior_tree/bt_nodes.py`
- Condition nodes: `IsObstacleBlocking`, `ShouldChangeLane`, `IsLaneChangeSafe`, `IsVehicleAhead`, `IsTooClose`
- Action nodes: `SetStopCommand`, `SetLaneChangeCommand`, `SetFollowCommand`, `SetLaneKeepCommand`

### 2. `src/behavioral_planner.py`
- `BehavioralPlanner` class: Build the Behavior Tree and run the tick loop

### 3. `src/frenet_transform.py`
- `cartesian_to_frenet()`: Convert (x, y, θ, v) to (s, d, ṡ, ḋ)
- `frenet_to_cartesian()`: Convert (s, d, ṡ, ḋ) to (x, y, θ, v)

### 4. `src/polynomial_trajectory.py`
- `quintic_coefficients()`: Compute quintic polynomial coefficients
- `quartic_coefficients()`: Compute quartic polynomial coefficients
- `evaluate()`: Evaluate polynomial at time t

### 5. `src/trajectory_planner.py`
- `TrajectoryPlanner` class: Implement the Frenet optimal trajectory method

## Test Scenarios

### Scenario 1: Lane Keeping
- Highway driving with no obstacles
- Tests basic trajectory generation and tracking

### Scenario 2: Vehicle Following
- Lead vehicle at varying speeds
- Tests FOLLOW behavior and speed adaptation

### Scenario 3: Lane Change
- Slow vehicle ahead, gap available in adjacent lane
- Tests full BT decision-making and lane change trajectory

## Troubleshooting

### CARLA Connection Issues
```
Error: Cannot connect to CARLA server
```
- Ensure CARLA server is running before starting the scenario
- Check that port 2000 is not blocked by firewall
- Try: `python -c "import carla; carla.Client('localhost', 2000)"`

### Import Errors
```
ModuleNotFoundError: No module named 'carla'
```
- Add CARLA Python API to your PYTHONPATH:
```bash
export PYTHONPATH=$PYTHONPATH:/path/to/carla/PythonAPI/carla/dist/carla-0.9.13-py3.8-linux-x86_64.egg
```

### Visualization Issues
- Ensure pygame is installed: `pip install pygame`
- Try running with `--no-render` flag for headless mode

## Submission

Submit the following files:
1. `src/behavior_tree/bt_nodes.py`
2. `src/behavioral_planner.py`
3. `src/frenet_transform.py`
4. `src/polynomial_trajectory.py`
5. `src/trajectory_planner.py`
6. Report (PDF, 3-5 pages)

## Contact

For questions, post on Canvas or attend office hours.
