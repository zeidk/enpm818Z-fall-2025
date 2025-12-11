# CARLA Integration for RWA3 - Behavioral Planning

This folder contains the CARLA integration for testing your behavior tree implementation in a 3D simulation environment.

## Prerequisites

### 1. Install CARLA Simulator

Download and install CARLA 0.9.13 or later:

- **Linux**: https://github.com/carla-simulator/carla/releases
- **Windows**: https://github.com/carla-simulator/carla/releases

Extract to a location like `/opt/carla-simulator/` (Linux) or `C:\CARLA\` (Windows).

### 2. Download Town04 (Recommended Highway Map)

Town04 has a dedicated multi-lane highway loop - ideal for this assignment.

**Download additional maps:**
1. Go to: https://github.com/carla-simulator/carla/releases
2. Find your CARLA version (e.g., 0.9.13, 0.9.14, 0.9.15)
3. Download: `AdditionalMaps_X.X.X.tar.gz`
4. Extract into your CARLA folder:

```bash
# Linux
cd /opt/carla-simulator/
tar -xzf AdditionalMaps_0.9.XX.tar.gz

# Windows: Use 7-Zip to extract into CARLA folder
```

5. Restart CARLA server - Town04 should now appear

### 3. Install Python Packages

```bash
# Match the carla version to your CARLA server!
pip install carla==0.9.13
pip install pygame numpy

# Or for newer versions:
pip install carla==0.9.14
pip install carla==0.9.15
```

### 4. Verify Installation

```bash
# Start CARLA server first (see below), then:
python carla_simulator.py --scenario empty
```

## Running CARLA Server

### Linux
```bash
cd /opt/carla-simulator/
./CarlaUE4.sh

# Low quality mode (faster):
./CarlaUE4.sh -quality-level=Low

# Offscreen rendering (no display):
./CarlaUE4.sh -RenderOffScreen
```

### Windows
```cmd
cd C:\CARLA\
CarlaUE4.exe

# Low quality mode:
CarlaUE4.exe -quality-level=Low
```

### Docker
```bash
# Basic CARLA (may not have Town04)
docker run --privileged --gpus all --net=host -it carlasim/carla:0.9.16

# If you've created an image with additional maps:
docker run --privileged --gpus all --net=host -it yourusername/carla-with-maps:0.9.16
```

### Docker: Installing Additional Maps for Town04

```bash
# 1. Start container with a name
docker run --privileged --gpus all --net=host \
  --name carla-server \
  -it carlasim/carla:0.9.16

# 2. In another terminal, copy maps to container
docker cp ~/Downloads/AdditionalMaps_0.9.16.tar.gz carla-server:/workspace/

# 3. Enter container as root
docker exec -it --user root carla-server bash

# 4. Inside container: extract and import
cd /workspace
tar -xzf AdditionalMaps_0.9.16.tar.gz
./ImportAssets.sh
exit

# 5. Restart container
docker stop carla-server
docker start -ai carla-server
```

## Map Selection

The simulator **automatically loads Town04** (highway map) if available:

```
Connecting to CARLA server at localhost:2000...
Connected to CARLA 0.9.16
✓ Town04 (highway map) is available
Loading Town04...
✓ Town04 loaded successfully!
Using map: Town04
✓ Highway map loaded - ideal for lane change scenarios!
Using Town04 highway spawn point...
✓ Highway spawn point found at (5, -170)
```

**Town04 features:**
- Dedicated multi-lane highway loop
- Long straight sections for overtaking
- 3+ lanes in each direction
- Known spawn points for consistent testing

If Town04 is not available, the simulator falls back to other maps with highway sections.

## File Structure

```
carla/
├── carla_simulator.py    # Main entry point (like simulator.py)
├── carla_interface.py    # CARLA perception & vehicle management
├── carla_controller.py   # PID controllers for vehicle control
├── README.md             # This file
└── verify_setup.py       # Setup verification script
```

## Usage

Make sure your behavior tree implementation files are in the parent directory:
- `bt_framework.py`
- `bt_nodes.py`
- `behavior_tree.py`

### Run Scenarios

```bash
# Empty road - test lane keeping
python carla_simulator.py --scenario empty

# Follow vehicle - test vehicle following
python carla_simulator.py --scenario follow

# Overtake - test lane changes
python carla_simulator.py --scenario overtake

# With custom duration (seconds)
python carla_simulator.py --scenario overtake --duration 60
```

### Pygame Window Controls

| Key | Action |
|-----|--------|
| `ESC` | Quit simulation |
| `P` | Pause/Resume |
| `R` | Reset scenario |

### Command Line Options

| Option | Description |
|--------|-------------|
| `--scenario`, `-s` | Scenario: `empty`, `follow`, or `overtake` |
| `--duration`, `-d` | Simulation duration in seconds (default: 60) |

## Scenarios

### Empty Road
- No traffic vehicles
- Expected behavior: `LANE_KEEP` throughout
- Tests: Basic lane keeping at speed limit

### Follow Vehicle  
- Lead vehicle at 20 m/s in center lane
- Adjacent lanes blocked by traffic
- Expected behavior: `FOLLOW_VEHICLE` throughout
- Tests: Speed matching, safe following distance

### Overtake
- Slow vehicle (20 m/s) in center lane, 40m ahead
- Slow vehicle (18 m/s) in left lane, 150m ahead
- Right lane initially blocked
- Expected sequence:
  1. `LANE_CHANGE_LEFT` - pass first slow vehicle
  2. `LANE_KEEP` - cruise in left lane
  3. `LANE_CHANGE_RIGHT` - avoid second slow vehicle
- Tests: Lane change decisions, return to center

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    carla_simulator.py                       │
│                    (Main Entry Point)                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    carla_interface.py                       │
│  ┌─────────────────┐    ┌─────────────────┐                │
│  │ get_environment │    │  apply_command  │                │
│  │     _state()    │    │       ()        │                │
│  └────────┬────────┘    └────────┬────────┘                │
│           │                      │                          │
│           ▼                      ▼                          │
│  ┌─────────────────┐    ┌─────────────────┐                │
│  │ EnvironmentState│    │ BehaviorCommand │                │
│  └─────────────────┘    └─────────────────┘                │
└─────────────────────────────────────────────────────────────┘
           │                       ▲
           │                       │
           ▼                       │
┌─────────────────────────────────────────────────────────────┐
│                    behavior_tree.py                         │
│                  (YOUR IMPLEMENTATION)                      │
│                                                             │
│   BehaviorPlanner.get_command(env_state) → BehaviorCommand │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   carla_controller.py                       │
│  ┌─────────────────────┐  ┌─────────────────────┐          │
│  │ LongitudinalPID     │  │ LateralStanley      │          │
│  │ Controller          │  │ Controller          │          │
│  └─────────────────────┘  └─────────────────────┘          │
│                              │                              │
│                              ▼                              │
│                    carla.VehicleControl                     │
│                 (throttle, brake, steer)                    │
└─────────────────────────────────────────────────────────────┘
```

## Perception Mapping

The `carla_interface.py` maps CARLA world state to `EnvironmentState`:

| EnvironmentState Field | CARLA Source |
|------------------------|--------------|
| `ego_speed` | `vehicle.get_velocity()` magnitude |
| `ego_d` | Lateral offset from lane centerline |
| `speed_limit` | Configuration (31 m/s) |
| `left_lane_exists` | `waypoint.get_left_lane()` |
| `right_lane_exists` | `waypoint.get_right_lane()` |
| `left_lane_clear` | No vehicles within 25m in left lane |
| `right_lane_clear` | No vehicles within 25m in right lane |
| `vehicle_ahead` | Vehicle in same lane ahead |
| `vehicle_ahead_distance` | Longitudinal distance to vehicle |
| `vehicle_ahead_speed` | Speed of vehicle ahead |

## Troubleshooting

### "Cannot connect to CARLA server"
1. Make sure CARLA server is running
2. Check if port 2000 is available: `netstat -an | grep 2000`
3. Try restarting CARLA server

### "No preferred highway map found"
This is normal! The simulator will:
- Automatically use your current map
- Find a spawn point with multiple lanes for lane change testing
- This works fine for testing behavior trees

If you want Town04 (dedicated highway map):
- CARLA 0.9.13+: Download additional maps from CARLA releases
- Extract to your CARLA installation folder
- Restart CARLA server

### "Vehicle spawn failed"
- Too many actors in the scene
- Restart CARLA server
- The script will try multiple vehicle models automatically

### "Left/Right lane not available"
- The spawn point may only have 2 lanes
- Some scenarios need 3 lanes for full testing
- Try a different map or restart to get a different spawn point

### "Vehicle spawn failed"
- Too many actors in the scene
- Restart CARLA server
- Try a different spawn point

### Performance Issues
- Use low quality mode: `./CarlaUE4.sh -quality-level=Low`
- Reduce traffic vehicles
- Use offscreen rendering for testing

### Python Version Mismatch
- CARLA Python package must match server version
- Check: `python -c "import carla; print(carla.__version__)"`

## Recording & Playback

### Record Simulation
```bash
python carla_simulator.py --scenario overtake --record
```

### Playback in CARLA
```python
client = carla.Client('localhost', 2000)
client.replay_file("carla_recording_overtake_xxx.log", 0, 0, 0)
```

## Bonus Points Submission

For the +10 bonus points, submit:

1. `carla/` folder with all integration code
2. Video recordings of each scenario (MP4, max 30s each):
   - `carla_empty.mp4`
   - `carla_follow.mp4`
   - `carla_overtake.mp4`

### Recording Videos

Option 1: Screen recording (OBS, etc.)

Option 2: CARLA built-in recorder + convert to video

Option 3: Use CARLA's sensor-based recording:
```python
# Add camera sensor and save frames
camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))
```

## Support

If you encounter issues:
1. Check CARLA documentation: https://carla.readthedocs.io/
2. Verify your behavior tree passes all unit tests first
3. Use `--test-connection` to verify CARLA setup
4. Check the CARLA Discord or GitHub issues
