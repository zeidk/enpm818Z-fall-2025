# CARLA Vehicle Control Scripts
## ENPM818Z - On-Road Automated Vehicles
### University of Maryland

Vehicle control demonstration in CARLA simulator with pygame visualization and runtime controller switching.

---

## Files

| File | Description |
|------|-------------|
| `carla_control_utils.py` | Utility functions (state extraction, waypoints, errors) |
| `carla_controllers.py` | Controller classes: Pure Pursuit, Stanley, PID, LQR, MPC |
| `carla_pygame_display.py` | Pygame visualization with camera and HUD |
| `carla_control_main_pygame.py` | Main script with runtime controller switching |

---

## Requirements

- CARLA Simulator 0.9.13+
- Python 3.7+
- NumPy
- SciPy
- Pygame

### Setup

```bash
# Install Python dependencies
pip install numpy scipy pygame

# Set CARLA Python path
export PYTHONPATH=$PYTHONPATH:/path/to/carla/PythonAPI/carla
```

---

## Quick Start

```bash
# 1. Start CARLA server
cd /path/to/carla
./CarlaUE4.sh

# 2. Run control demo (in another terminal)
python carla_control_main_pygame.py
```

---

## Keyboard Controls (Runtime)

| Key | Action |
|-----|--------|
| `1` | Switch to Pure Pursuit |
| `2` | Switch to Stanley |
| `3` | Switch to LQR |
| `4` | Switch to MPC |
| `Q` | Toggle HUD |
| `ESC` | Quit |

**No restart needed to switch controllers!**

---

## Command Line Options

```bash
python carla_control_main_pygame.py [OPTIONS]

Options:
  --controller   Initial controller: pure_pursuit, stanley, lqr, mpc
  --speed        Target speed in m/s (default: 10.0)
  --duration     Max duration in seconds (default: 300)
  --map          CARLA map name (e.g., Town04)
  --host         CARLA server host (default: localhost)
  --port         CARLA server port (default: 2000)
  --no-display   Run without pygame visualization
```

### Examples

```bash
# Start with Stanley at 12 m/s
python carla_control_main_pygame.py --controller stanley --speed 12

# Use Town04 map
python carla_control_main_pygame.py --map Town04

# Headless mode (no visualization)
python carla_control_main_pygame.py --no-display
```

---

## Controllers

### Pure Pursuit (Key: 1)
- Geometric path tracking
- Steers toward a look-ahead point
- Best for: < 15 m/s

### Stanley (Key: 2)
- Front-axle referenced
- Combines heading + cross-track error
- Best for: < 15 m/s

### LQR (Key: 3)
- Dynamic bicycle model
- Optimal state feedback
- Gain scheduling with speed
- Best for: 10-25 m/s

### MPC (Key: 4)
- Simplified predictive control
- Kinematic model
- Discrete steering search
- Educational implementation

---

## HUD Display

```
┌─────────────────────────────────┐
│  Controller: Pure Pursuit       │
│  Speed: 36.0 / 36.0 km/h        │
│  Steering: 2.5°                 │
│  Throttle: 0.45                 │
│  Brake: 0.00                    │
│  Cross-track: 0.12 m            │
│  Heading err: 1.2°              │
└─────────────────────────────────┘
```

---

## Tuning Guide

### Pure Pursuit
| Parameter | Effect |
|-----------|--------|
| `k` ↑ | Larger look-ahead, smoother but cuts corners |
| `k` ↓ | Tighter tracking, may oscillate |
| `Ld_min` ↑ | More stable at low speed |

### Stanley
| Parameter | Effect |
|-----------|--------|
| `k` ↑ | Faster cross-track correction |
| `k` ↓ | Smoother but slower correction |
| `k_soft` ↑ | More stable at low speed |

### LQR
| Matrix | Effect |
|--------|--------|
| Q[0,0] ↑ | Penalize cross-track error more |
| Q[2,2] ↑ | Penalize heading error more |
| R ↑ | Smoother steering, slower response |

---

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                  carla_control_main_pygame.py            │
│                                                          │
│  ┌─────────────┐    ┌──────────────┐    ┌────────────┐   │
│  │ CARLA World │───▶│ Vehicle State│───▶│ Controller │   │
│  └─────────────┘    └──────────────┘    └─────┬──────┘   │
│         │                                     │          │
│         ▼           ┌──────────────┐          │          │
│  ┌─────────────┐    │ Pygame + HUD │◀─────────┘          │
│  │   Vehicle   │◀───┴──────────────┘                     │
│  └─────────────┘                                         │
└──────────────────────────────────────────────────────────┘

Control Loop @ 50 Hz:
1. world.tick()
2. get_vehicle_state()
3. controller.get_control()
4. speed_ctrl.get_control()
5. vehicle.apply_control()
6. display.render()
```

---

## Troubleshooting

### "CARLA Python API not found"
```bash
export PYTHONPATH=$PYTHONPATH:/path/to/carla/PythonAPI/carla
```

### "No module named pygame"
```bash
pip install pygame
```

### Vehicle doesn't move
- Ensure CARLA server is running
- Check `world.tick()` is being called (synchronous mode)

### Black pygame window
- Wait 1-2 seconds for camera to initialize
- Check CARLA server is responsive

### Oscillating behavior
- Reduce controller gains
- Try a lower target speed

---

## References

1. Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm
2. Thrun, S. et al. (2006). Stanley: The Robot that Won the DARPA Grand Challenge
3. Rajamani, R. (2012). Vehicle Dynamics and Control

---

## License

Educational use for ENPM818Z - University of Maryland
