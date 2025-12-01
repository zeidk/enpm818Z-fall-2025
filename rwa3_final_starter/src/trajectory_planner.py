"""
Trajectory Planner

This module implements the Frenet Optimal Trajectory planner.

YOU MUST IMPLEMENT:
- TrajectoryPlanner._generate_candidates(): Sample and generate trajectory candidates
- TrajectoryPlanner._check_feasibility(): Verify trajectory constraints
- TrajectoryPlanner._check_collision(): Check for collisions with obstacles
- TrajectoryPlanner._compute_cost(): Evaluate trajectory cost
- TrajectoryPlanner.plan(): Main planning function

Refer to L6 lecture materials for the algorithm details.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field

from .frenet_transform import (
    FrenetState, 
    CartesianState,
    FrenetTransform,
    cartesian_to_frenet,
    frenet_to_cartesian
)
from .polynomial_trajectory import (
    quintic_coefficients,
    quartic_coefficients,
    evaluate_polynomial,
    compute_jerk,
    PolynomialTrajectory
)
from .behavior_tree import BehavioralCommand


@dataclass
class TrajectoryPoint:
    """A single point on a trajectory."""
    t: float      # Time (seconds)
    x: float      # Global X position (meters)
    y: float      # Global Y position (meters)
    theta: float  # Heading (radians)
    v: float      # Speed (m/s)
    kappa: float  # Curvature (1/m)
    a: float = 0.0  # Acceleration (m/s^2)
    

@dataclass
class Trajectory:
    """
    Complete trajectory with metadata.
    """
    points: List[TrajectoryPoint] = field(default_factory=list)
    cost: float = float('inf')
    feasible: bool = False
    collision_free: bool = False
    
    # Frenet trajectory components (for debugging/visualization)
    s_traj: Optional[PolynomialTrajectory] = None
    d_traj: Optional[PolynomialTrajectory] = None
    
    @property
    def duration(self) -> float:
        if not self.points:
            return 0.0
        return self.points[-1].t - self.points[0].t
    
    def sample(self, t: float) -> Optional[TrajectoryPoint]:
        """Get trajectory point at time t (linear interpolation)."""
        if not self.points or t < self.points[0].t or t > self.points[-1].t:
            return None
        
        # Find surrounding points
        for i in range(len(self.points) - 1):
            if self.points[i].t <= t <= self.points[i + 1].t:
                p0, p1 = self.points[i], self.points[i + 1]
                dt = p1.t - p0.t
                alpha = (t - p0.t) / dt if dt > 0 else 0.0
                
                return TrajectoryPoint(
                    t=t,
                    x=p0.x + alpha * (p1.x - p0.x),
                    y=p0.y + alpha * (p1.y - p0.y),
                    theta=p0.theta + alpha * (p1.theta - p0.theta),
                    v=p0.v + alpha * (p1.v - p0.v),
                    kappa=p0.kappa + alpha * (p1.kappa - p0.kappa),
                    a=p0.a + alpha * (p1.a - p0.a)
                )
        
        return None


class TrajectoryPlanner:
    """
    Frenet Optimal Trajectory Planner.
    
    Generates smooth, collision-free trajectories by:
    1. Sampling candidate end states based on behavioral command
    2. Generating polynomial trajectories in Frenet coordinates
    3. Converting to Cartesian coordinates
    4. Checking feasibility and collision constraints
    5. Selecting the minimum-cost trajectory
    """
    
    def __init__(self, config: Dict):
        """
        Initialize the trajectory planner.
        
        Args:
            config: Configuration dictionary containing:
                - max_speed, max_accel, max_decel, max_curvature
                - planning_horizon, dt
                - num_d_samples, num_v_samples, num_t_samples
                - d_sample_range, v_sample_range
                - t_sample_min, t_sample_max
                - cost_weights: dict with jerk, lateral_deviation, etc.
                - vehicle_length, vehicle_width, safety_margin
        """
        self.config = config
        
        # Extract commonly used parameters
        self.max_speed = config.get('max_speed', 30.0)
        self.max_accel = config.get('max_accel', 3.0)
        self.max_decel = config.get('max_decel', -6.0)
        self.max_curvature = config.get('max_curvature', 0.2)
        self.max_lateral_accel = config.get('max_lateral_accel', 3.0)
        
        self.planning_horizon = config.get('planning_horizon', 5.0)
        self.dt = config.get('dt', 0.1)
        
        # Sampling parameters
        self.num_d_samples = config.get('num_d_samples', 5)
        self.num_v_samples = config.get('num_v_samples', 5)
        self.num_t_samples = config.get('num_t_samples', 5)
        
        self.d_sample_range = config.get('d_sample_range', 0.5)
        self.v_sample_range = config.get('v_sample_range', 2.0)
        self.t_sample_min = config.get('t_sample_min', 3.0)
        self.t_sample_max = config.get('t_sample_max', 6.0)
        
        # Cost weights
        self.cost_weights = config.get('cost_weights', {
            'jerk': 0.1,
            'lateral_deviation': 1.0,
            'speed_deviation': 1.0,
            'time': 0.5,
            'obstacle_proximity': 10.0
        })
        
        # Vehicle dimensions
        self.vehicle_length = config.get('vehicle_length', 4.5)
        self.vehicle_width = config.get('vehicle_width', 2.0)
        self.safety_margin = config.get('safety_margin', 1.0)
        
        # Lane width (for computing target d values)
        self.lane_width = config.get('lane_width', 3.5)
    
    def plan(self, ego_state: Dict, behavioral_command: BehavioralCommand,
             obstacles: List[Dict], reference_path: List[Tuple[float, float]]) -> Tuple[Trajectory, bool]:
        """
        Generate optimal trajectory.
        
        Args:
            ego_state: Dict with x, y, theta, v, lane_id
            behavioral_command: Output from behavioral planner
            obstacles: List of obstacle dicts with x, y, vx, vy, length, width
            reference_path: List of (x, y) waypoints for road centerline
            
        Returns:
            trajectory: Best trajectory found
            success: True if valid trajectory found
        """
        # TODO: Implement this method
        #
        # Steps:
        # 1. Create FrenetTransform from reference_path
        # 2. Convert ego_state to Frenet coordinates
        # 3. Determine target d based on behavioral_command:
        #    - lane_keep: current lane center
        #    - lane_change_left: left lane center
        #    - lane_change_right: right lane center
        #    - follow: current lane center
        #    - stop: current lane center
        # 4. Determine target velocity based on behavioral_command
        # 5. Generate candidate trajectories
        # 6. For each candidate:
        #    - Check feasibility
        #    - Check collision
        #    - Compute cost
        # 7. Select minimum-cost feasible trajectory
        # 8. Return trajectory and success flag
        
        if len(reference_path) < 2:
            return Trajectory(), False
        
        # Create Frenet transform
        frenet_transform = FrenetTransform(reference_path)
        
        # Convert ego state to Frenet
        ego_frenet = frenet_transform.to_frenet(
            ego_state['x'], ego_state['y'],
            ego_state['theta'], ego_state['v']
        )
        
        # Determine target states based on behavioral command
        target_d = self._compute_target_d(behavioral_command, ego_state)
        target_v = behavioral_command.target_speed
        
        # Generate candidate trajectories
        candidates = self._generate_candidates(
            ego_frenet, target_d, target_v, 
            behavioral_command, frenet_transform
        )
        
        # Evaluate candidates
        best_trajectory = Trajectory()
        best_cost = float('inf')
        
        for traj in candidates:
            # Check feasibility
            if not self._check_feasibility(traj):
                continue
            
            traj.feasible = True
            
            # Check collision
            if not self._check_collision(traj, obstacles):
                continue
            
            traj.collision_free = True
            
            # Compute cost
            cost = self._compute_cost(traj, target_d, target_v, obstacles)
            traj.cost = cost
            
            if cost < best_cost:
                best_cost = cost
                best_trajectory = traj
        
        success = best_trajectory.feasible and best_trajectory.collision_free
        return best_trajectory, success
    
    def _compute_target_d(self, command: BehavioralCommand, 
                          ego_state: Dict) -> float:
        """
        Compute target lateral position based on behavioral command.
        
        Args:
            command: Behavioral command with target_lane
            ego_state: Current ego state with lane_id
            
        Returns:
            Target d value (positive = left of centerline)
        """
        # TODO: Implement based on lane geometry
        # For now, assume d=0 is current lane center
        # Positive d is left, negative d is right
        
        current_lane = ego_state.get('lane_id', 0)
        target_lane = command.target_lane
        
        # Each lane is lane_width apart
        lane_diff = target_lane - current_lane
        target_d = lane_diff * self.lane_width
        
        return target_d
    
    def _generate_candidates(self, ego_frenet: FrenetState,
                             target_d: float, target_v: float,
                             command: BehavioralCommand,
                             frenet_transform: FrenetTransform) -> List[Trajectory]:
        """
        Generate candidate trajectories by sampling end states.
        
        Args:
            ego_frenet: Current state in Frenet coordinates
            target_d: Target lateral offset
            target_v: Target velocity
            command: Behavioral command
            frenet_transform: Coordinate transformer
            
        Returns:
            List of candidate trajectories
        """
        # TODO: Implement this method
        #
        # Steps:
        # 1. Sample target d values around target_d
        # 2. Sample target velocities around target_v
        # 3. Sample trajectory durations
        # 4. For each combination:
        #    a. Generate quintic polynomial for d(t)
        #    b. Generate quartic polynomial for s(t)
        #    c. Sample trajectory at dt intervals
        #    d. Convert each point to Cartesian
        #    e. Create Trajectory object
        
        candidates = []
        
        # Sample d values
        d_samples = np.linspace(
            target_d - self.d_sample_range,
            target_d + self.d_sample_range,
            self.num_d_samples
        )
        
        # Sample v values
        v_samples = np.linspace(
            max(0, target_v - self.v_sample_range),
            min(self.max_speed, target_v + self.v_sample_range),
            self.num_v_samples
        )
        
        # Sample durations
        t_samples = np.linspace(
            self.t_sample_min,
            self.t_sample_max,
            self.num_t_samples
        )
        
        # Handle stop command specially
        if command.maneuver == 'stop':
            v_samples = [0.0]
        
        for d_f in d_samples:
            for v_f in v_samples:
                for T in t_samples:
                    traj = self._generate_single_trajectory(
                        ego_frenet, d_f, v_f, T, frenet_transform
                    )
                    if traj is not None:
                        candidates.append(traj)
        
        return candidates
    
    def _generate_single_trajectory(self, ego_frenet: FrenetState,
                                    d_f: float, v_f: float, T: float,
                                    frenet_transform: FrenetTransform) -> Optional[Trajectory]:
        """
        Generate a single trajectory from current state to target.
        
        Args:
            ego_frenet: Current Frenet state
            d_f: Target lateral offset
            v_f: Target velocity
            T: Duration
            frenet_transform: Coordinate transformer
            
        Returns:
            Trajectory object, or None if generation fails
        """
        try:
            # Lateral trajectory (quintic)
            d_start = (ego_frenet.d, ego_frenet.d_dot, 0.0)  # Assume zero lateral acceleration
            d_end = (d_f, 0.0, 0.0)  # End with zero lateral velocity and acceleration
            d_coeffs = quintic_coefficients(d_start, d_end, T)
            
            # Longitudinal trajectory (quartic)
            s_start = (ego_frenet.s, ego_frenet.s_dot, 0.0)  # Assume zero longitudinal acceleration
            s_end_vel = (v_f, 0.0)  # Target velocity with zero acceleration
            s_coeffs = quartic_coefficients(s_start, s_end_vel, T)
            
            # Sample trajectory
            points = []
            t = 0.0
            while t <= T:
                # Get Frenet state
                s, s_dot, s_ddot = evaluate_polynomial(s_coeffs, t)
                d, d_dot, d_ddot = evaluate_polynomial(d_coeffs, t)
                
                # Convert to Cartesian
                cart = frenet_transform.to_cartesian(s, d, s_dot, d_dot)
                
                # Compute curvature (approximate)
                kappa = cart.kappa
                
                # Compute acceleration (approximate from s_ddot and d_ddot)
                a = s_ddot  # Simplified
                
                points.append(TrajectoryPoint(
                    t=t,
                    x=cart.x,
                    y=cart.y,
                    theta=cart.theta,
                    v=cart.v,
                    kappa=kappa,
                    a=a
                ))
                
                t += self.dt
            
            return Trajectory(
                points=points,
                s_traj=PolynomialTrajectory(s_coeffs, T, 4),
                d_traj=PolynomialTrajectory(d_coeffs, T, 5)
            )
            
        except Exception as e:
            return None
    
    def _check_feasibility(self, traj: Trajectory) -> bool:
        """
        Check if trajectory satisfies feasibility constraints.
        
        Constraints:
        - Velocity bounds: 0 <= v <= max_speed
        - Acceleration bounds: max_decel <= a <= max_accel
        - Curvature bounds: |kappa| <= max_curvature
        - Lateral acceleration: v^2 * kappa <= max_lateral_accel
        
        Args:
            traj: Trajectory to check
            
        Returns:
            True if trajectory is feasible
        """
        # TODO: Implement this method
        #
        # For each point in the trajectory:
        # 1. Check velocity: 0 <= v <= max_speed
        # 2. Check longitudinal acceleration: max_decel <= a <= max_accel
        # 3. Check curvature: |kappa| <= max_curvature
        # 4. Check lateral acceleration: v^2 * |kappa| <= max_lateral_accel
        
        for point in traj.points:
            # Velocity check
            if point.v < 0 or point.v > self.max_speed:
                return False
            
            # Acceleration check
            if point.a < self.max_decel or point.a > self.max_accel:
                return False
            
            # Curvature check
            if abs(point.kappa) > self.max_curvature:
                return False
            
            # Lateral acceleration check
            lateral_accel = point.v ** 2 * abs(point.kappa)
            if lateral_accel > self.max_lateral_accel:
                return False
        
        return True
    
    def _check_collision(self, traj: Trajectory, 
                         obstacles: List[Dict]) -> bool:
        """
        Check if trajectory collides with any obstacle.
        
        Uses simple circle-based collision checking with predicted
        obstacle positions.
        
        Args:
            traj: Trajectory to check
            obstacles: List of obstacles with x, y, vx, vy, length, width
            
        Returns:
            True if trajectory is collision-free
        """
        # TODO: Implement this method
        #
        # For each point in the trajectory:
        # 1. Predict obstacle position at that time
        # 2. Check if vehicle footprint overlaps with obstacle
        # 3. Include safety margin
        
        vehicle_radius = np.sqrt((self.vehicle_length/2)**2 + 
                                 (self.vehicle_width/2)**2) + self.safety_margin
        
        for point in traj.points:
            for obs in obstacles:
                # Predict obstacle position at time t
                obs_x = obs['x'] + obs['vx'] * point.t
                obs_y = obs['y'] + obs['vy'] * point.t
                
                obs_length = obs.get('length', 4.5)
                obs_width = obs.get('width', 2.0)
                obs_radius = np.sqrt((obs_length/2)**2 + (obs_width/2)**2)
                
                # Check distance
                dx = point.x - obs_x
                dy = point.y - obs_y
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance < vehicle_radius + obs_radius:
                    return False
        
        return True
    
    def _compute_cost(self, traj: Trajectory, target_d: float,
                      target_v: float, obstacles: List[Dict]) -> float:
        """
        Compute cost of a trajectory.
        
        Cost components:
        - Jerk (integral of squared jerk)
        - Lateral deviation from target
        - Speed deviation from target
        - Time to complete
        - Proximity to obstacles
        
        Args:
            traj: Trajectory to evaluate
            target_d: Target lateral offset
            target_v: Target velocity
            obstacles: List of obstacles
            
        Returns:
            Total cost (lower is better)
        """
        # TODO: Implement this method
        #
        # Compute each cost component and combine with weights
        
        weights = self.cost_weights
        
        # Jerk cost (from polynomial if available)
        jerk_cost = 0.0
        if traj.s_traj is not None:
            for t in np.arange(0, traj.duration, self.dt):
                s_jerk = compute_jerk(traj.s_traj.coeffs, t)
                jerk_cost += s_jerk ** 2 * self.dt
        if traj.d_traj is not None:
            for t in np.arange(0, traj.duration, self.dt):
                d_jerk = compute_jerk(traj.d_traj.coeffs, t)
                jerk_cost += d_jerk ** 2 * self.dt
        
        # Lateral deviation cost (final d vs target d)
        if traj.d_traj is not None:
            final_d, _, _ = evaluate_polynomial(traj.d_traj.coeffs, traj.duration)
            lateral_cost = (final_d - target_d) ** 2
        else:
            lateral_cost = 0.0
        
        # Speed deviation cost (final v vs target v)
        if traj.points:
            final_v = traj.points[-1].v
            speed_cost = (final_v - target_v) ** 2
        else:
            speed_cost = 0.0
        
        # Time cost
        time_cost = traj.duration
        
        # Obstacle proximity cost
        obstacle_cost = 0.0
        for point in traj.points:
            for obs in obstacles:
                obs_x = obs['x'] + obs['vx'] * point.t
                obs_y = obs['y'] + obs['vy'] * point.t
                distance = np.sqrt((point.x - obs_x)**2 + (point.y - obs_y)**2)
                
                # Penalize being close to obstacles
                if distance < 20.0:
                    obstacle_cost += (20.0 - distance) ** 2
        
        # Combine costs
        total_cost = (
            weights.get('jerk', 0.1) * jerk_cost +
            weights.get('lateral_deviation', 1.0) * lateral_cost +
            weights.get('speed_deviation', 1.0) * speed_cost +
            weights.get('time', 0.5) * time_cost +
            weights.get('obstacle_proximity', 10.0) * obstacle_cost
        )
        
        return total_cost


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Testing Trajectory Planner...")
    print("=" * 50)
    
    import yaml
    from pathlib import Path
    
    # Load config
    config_path = Path(__file__).parent.parent / 'config' / 'planner_config.yaml'
    try:
        with open(config_path, 'r') as f:
            full_config = yaml.safe_load(f)
        config = full_config.get('trajectory_planner', {})
    except:
        config = {
            'max_speed': 30.0,
            'max_accel': 3.0,
            'max_decel': -6.0,
            'max_curvature': 0.2,
            'planning_horizon': 5.0,
            'dt': 0.1
        }
    
    # Create planner
    planner = TrajectoryPlanner(config)
    print(f"Planner created with config: max_speed={planner.max_speed}")
    
    # Create test scenario
    ego_state = {
        'x': 0.0,
        'y': 0.0,
        'theta': 0.0,
        'v': 20.0,
        'lane_id': 0
    }
    
    behavioral_command = BehavioralCommand(
        maneuver='lane_keep',
        target_lane=0,
        target_speed=25.0
    )
    
    obstacles = []
    
    reference_path = [(i * 2.0, 0.0) for i in range(100)]
    
    # Plan
    print("\nPlanning trajectory...")
    trajectory, success = planner.plan(ego_state, behavioral_command, 
                                       obstacles, reference_path)
    
    print(f"Success: {success}")
    print(f"Trajectory points: {len(trajectory.points)}")
    print(f"Duration: {trajectory.duration:.2f}s")
    print(f"Cost: {trajectory.cost:.4f}")
    
    if trajectory.points:
        start = trajectory.points[0]
        end = trajectory.points[-1]
        print(f"Start: ({start.x:.2f}, {start.y:.2f}), v={start.v:.2f}")
        print(f"End: ({end.x:.2f}, {end.y:.2f}), v={end.v:.2f}")
    
    # Test with lane change
    print("\n--- Lane Change Test ---")
    behavioral_command = BehavioralCommand(
        maneuver='lane_change_left',
        target_lane=1,
        target_speed=25.0
    )
    
    trajectory, success = planner.plan(ego_state, behavioral_command,
                                       obstacles, reference_path)
    
    print(f"Lane change success: {success}")
    if trajectory.points:
        end = trajectory.points[-1]
        print(f"End position: ({end.x:.2f}, {end.y:.2f})")
    
    print("\nTrajectory Planner test complete!")
