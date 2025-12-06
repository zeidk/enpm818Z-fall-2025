# Final Project: Trajectory Planning in Frenet Coordinates

## Overview

This project implements a trajectory planner for autonomous driving using the Frenet
optimal trajectory method. The planner generates smooth, collision-free trajectories
in road-relative coordinates.

## Files to Implement

You need to implement the following files:

1. **`frenet.py`** - Frenet coordinate transformation
   - `cartesian_to_frenet()` - Convert (x, y) to (s, d)
   - `frenet_to_cartesian()` - Convert (s, d) to (x, y)

2. **`polynomial.py`** - Polynomial trajectory generation
   - `quintic_coefficients()` - Compute 5th-degree polynomial for lateral motion
   - `quartic_coefficients()` - Compute 4th-degree polynomial for longitudinal motion
   - `evaluate_polynomial()` - Evaluate polynomial and derivatives
   - `compute_jerk()` - Compute third derivative
   - `generate_trajectory()` - Generate full trajectory from polynomials

3. **`cost.py`** - Cost function and feasibility checking
   - `compute_total_cost()` - Calculate trajectory quality score
   - `check_feasibility()` - Verify vehicle constraints
   - `select_best_trajectory()` - Choose optimal trajectory from candidates

## Provided Files (Do Not Modify)

- `test_frenet.py` - Unit tests for Frenet transforms
- `test_polynomial.py` - Unit tests for polynomial generation
- `test_cost.py` - Unit tests for cost function

## Prerequisites

Copy your `behavior_tree.py` from RWA3 into this directory.

## Setup

```bash
# Use same environment from RWA3
source venv/bin/activate

# Install additional dependencies
pip install -r requirements.txt
```

## Testing

```bash
# Test each module
python test_frenet.py
python test_polynomial.py
python test_cost.py

# Test individual modules
python frenet.py
python polynomial.py
python cost.py
```

## Key Concepts

### Frenet Coordinates
- **s**: Arc length along the reference path (longitudinal position)
- **d**: Lateral offset from the path (positive = left)

### Polynomial Trajectories
- **Quintic (5th degree)**: For lateral motion - 6 boundary conditions
  - Position, velocity, acceleration at start and end
- **Quartic (4th degree)**: For longitudinal motion - 5 boundary conditions
  - Position, velocity, acceleration at start; velocity, acceleration at end

### Cost Function Components
- Jerk cost: Penalizes jerky motion
- Time cost: Penalizes longer trajectories
- Lateral deviation: Penalizes missing target lane
- Speed deviation: Penalizes missing target speed

## Submission

Submit a ZIP file containing:
- `frenet.py`
- `polynomial.py`
- `cost.py`
- `behavior_tree.py` (from RWA3)
- `results/` folder with screenshots and test output
- `report.pdf` (3-5 pages)
