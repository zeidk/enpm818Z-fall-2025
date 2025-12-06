"""
visualizer.py - Real-time Simulation Visualization

Real-time visualization of:
- Curved highway with lane markings
- Ego and traffic vehicles positioned on curved roads
- Planned trajectories in both Frenet and Cartesian coordinates
- Multi-panel display with trajectory profiles

Layout:
- Top: Main road view (follows ego vehicle)
- Bottom Left: Trajectory profile (s, d vs time)
- Bottom Right: Speed/acceleration profile
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.transforms import Affine2D
from typing import Optional, List, Tuple

from frenet import frenet_to_cartesian, ReferencePath


class SimulationVisualizer:
    """
    Real-time visualization with curved road support.
    """
    
    def __init__(self, simulator):
        """
        Initialize visualizer.
        
        Args:
            simulator: Simulator instance
        """
        self.sim = simulator
        self.config = simulator.config
        self.ref_path = simulator.ref_path
        
        # Create figure with custom gridspec for better layout
        # Wider figure to accommodate the full road view
        self.fig = plt.figure(figsize=(20, 11))
        
        # Use gridspec for precise control over subplot sizes
        # Top row takes ~70% of height, bottom row takes ~30%
        gs = self.fig.add_gridspec(3, 2, height_ratios=[2.5, 1, 1], 
                                   hspace=0.3, wspace=0.25,
                                   left=0.05, right=0.95, top=0.92, bottom=0.08)
        
        # Main road view (top, spanning both columns)
        self.ax_main = self.fig.add_subplot(gs[0, :])
        
        # Behavior summary (bottom left)
        self.ax_behavior = self.fig.add_subplot(gs[1:3, 0])
        
        # Polynomial coefficients (bottom right)
        self.ax_coeffs = self.fig.add_subplot(gs[1:3, 1])
        
        # Setup views
        self._setup_main_view()
        self._setup_behavior_view()
        self._setup_coeffs_view()
        
        # Create vehicle patches
        self.ego_patch = None
        self.traffic_patches = []
        
        # Trajectory lines
        self.traj_line = None
        self.traj_points = None
        
        # Ego trail (path history)
        self.ego_trail_x = []
        self.ego_trail_y = []
        self.ego_trail_line = None
        
        # Behavior tracking
        self.behavior_log = []
        self.last_behavior = None
        
        # Text annotations
        self.time_text = None
        self.behavior_text = None
        self.traj_info_text = None
        self.behavior_summary_text = None
        self.coeffs_text = None
        
        # Initialize all graphics
        self._init_graphics()
        
        # Add main title to figure (not subplot)
        self.fig.suptitle('Highway Driving with Frenet Trajectory Planning (5 S-Curves)', 
                          fontsize=16, fontweight='bold', y=0.98)
    
    def _setup_main_view(self):
        """Setup the main road view."""
        self.ax_main.set_aspect('equal')
        self.ax_main.set_xlabel('X (meters)', fontsize=12)
        self.ax_main.set_ylabel('Y (meters)', fontsize=12)
        self.ax_main.grid(True, alpha=0.3, linestyle='--')
        
        # Set fixed view to show entire road (no moving camera)
        # Road is 1000m long with amplitude ±15m
        self.ax_main.set_xlim(-20, 520)  # Show first half of road
        self.ax_main.set_ylim(-40, 40)
    
    def _setup_behavior_view(self):
        """Setup the behavior summary view."""
        self.ax_behavior.set_xlim(0, 1)
        self.ax_behavior.set_ylim(0, 1)
        self.ax_behavior.axis('off')
        self.ax_behavior.set_title('Behavior Summary', fontsize=12, fontweight='bold')
    
    def _setup_coeffs_view(self):
        """Setup the polynomial coefficients view."""
        self.ax_coeffs.set_xlim(0, 1)
        self.ax_coeffs.set_ylim(0, 1)
        self.ax_coeffs.axis('off')
        self.ax_coeffs.set_title('Trajectory Polynomial Coefficients', fontsize=12, fontweight='bold')
    
    def _init_graphics(self):
        """Initialize all graphic elements."""
        # Draw initial road
        self._draw_road()
        
        # Create ego vehicle patch
        self.ego_patch = self._create_vehicle_patch('limegreen', 'darkgreen')
        self.ax_main.add_patch(self.ego_patch)
        
        # Trajectory visualization
        self.traj_line, = self.ax_main.plot([], [], 'b-', linewidth=3, alpha=0.8, 
                                            label='Planned trajectory')
        self.traj_points, = self.ax_main.plot([], [], 'bo', markersize=5, alpha=0.6)
        
        # Ego trail (shows path history)
        self.ego_trail_line, = self.ax_main.plot([], [], 'g-', linewidth=2, alpha=0.5,
                                                  label='Ego path')
        
        # Text annotations on main view
        self.time_text = self.ax_main.text(
            0.02, 0.98, '', transform=self.ax_main.transAxes,
            fontsize=11, verticalalignment='top',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9)
        )
        
        # Behavior text - positioned lower left to avoid overlap
        self.behavior_text = self.ax_main.text(
            0.02, 0.35, '', transform=self.ax_main.transAxes,
            fontsize=14, verticalalignment='top',
            fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray')
        )
        
        self.traj_info_text = self.ax_main.text(
            0.98, 0.98, '', transform=self.ax_main.transAxes,
            fontsize=10, verticalalignment='top',
            horizontalalignment='right',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9)
        )
        
        # Behavior summary text (bottom left panel)
        self.behavior_summary_text = self.ax_behavior.text(
            0.05, 0.95, '', transform=self.ax_behavior.transAxes,
            fontsize=10, verticalalignment='top',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9)
        )
        
        # Coefficients text (bottom right panel)
        self.coeffs_text = self.ax_coeffs.text(
            0.05, 0.95, '', transform=self.ax_coeffs.transAxes,
            fontsize=10, verticalalignment='top',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9)
        )
    
    def _draw_road(self):
        """Draw the curved road with lane markings."""
        # Get reference path points
        path_points = self.ref_path.points
        path_normals = self.ref_path.normals
        
        lane_width = self.config.lane_width
        num_lanes = self.config.num_lanes
        
        # Road edges
        half_road = (num_lanes / 2) * lane_width
        
        # Draw road surface (gray polygon)
        left_edge = path_points + half_road * path_normals
        right_edge = path_points - half_road * path_normals
        
        road_x = np.concatenate([left_edge[:, 0], right_edge[::-1, 0]])
        road_y = np.concatenate([left_edge[:, 1], right_edge[::-1, 1]])
        self.ax_main.fill(road_x, road_y, color='gray', alpha=0.3, zorder=0)
        
        # Draw centerline (dashed yellow)
        self.ax_main.plot(path_points[:, 0], path_points[:, 1], 
                         'y--', linewidth=2, alpha=0.8, dashes=(10, 5), zorder=1)
        
        # Draw lane boundaries
        for i in range(-num_lanes//2 - 1, num_lanes//2 + 2):
            offset = i * lane_width
            if offset == 0:
                continue  # Skip centerline (already drawn)
            
            lane_line = path_points + offset * path_normals
            
            if abs(i) == num_lanes // 2 + 1:
                # Solid edge lines (white)
                self.ax_main.plot(lane_line[:, 0], lane_line[:, 1], 
                                 'w-', linewidth=3, alpha=0.9, zorder=1)
            elif abs(offset) < half_road:
                # Dashed lane dividers (white)
                self.ax_main.plot(lane_line[:, 0], lane_line[:, 1], 
                                 'w--', linewidth=1.5, alpha=0.7, dashes=(5, 5), zorder=1)
    
    def _create_vehicle_patch(self, facecolor: str, edgecolor: str) -> patches.Rectangle:
        """Create a vehicle rectangle patch."""
        return patches.Rectangle(
            (0, 0),
            self.config.vehicle_length,
            self.config.vehicle_width,
            facecolor=facecolor,
            edgecolor=edgecolor,
            linewidth=2,
            zorder=10
        )
    
    def _update_vehicle_patch(self, patch: patches.Rectangle, x: float, y: float, angle: float = 0):
        """Update vehicle patch position and rotation."""
        half_length = self.config.vehicle_length / 2
        half_width = self.config.vehicle_width / 2
        
        # Compute corner position (bottom-left of unrotated rectangle)
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        # Corner offset from center
        corner_x = x - half_length * cos_a + half_width * sin_a
        corner_y = y - half_length * sin_a - half_width * cos_a
        
        # Update position
        patch.set_xy((corner_x, corner_y))
        
        # Update rotation
        transform = Affine2D().rotate_around(x, y, angle) + self.ax_main.transData
        patch.set_transform(transform)
    
    def _get_road_angle(self, s: float) -> float:
        """Get road heading angle at position s."""
        # Find closest waypoint index
        idx = np.searchsorted(self.ref_path.s_values, s)
        idx = np.clip(idx, 0, len(self.ref_path.tangents) - 1)
        
        tangent = self.ref_path.tangents[idx]
        return np.arctan2(tangent[1], tangent[0])
    
    def update(self, frame, planner, duration):
        """Update function for animation."""
        if self.sim.time >= duration:
            return []
        
        # Get environment state
        env = self.sim.get_environment_state()
        
        # Run behavior planner
        command = planner.get_command(env)
        
        # Step simulation
        self.sim.step(command)
        
        # Get ego position
        ego_x, ego_y = frenet_to_cartesian(self.sim.ego.s, self.sim.ego.d, self.ref_path)
        ego_angle = self._get_road_angle(self.sim.ego.s)
        
        # Update ego vehicle
        self._update_vehicle_patch(self.ego_patch, ego_x, ego_y, ego_angle)
        
        # Update ego trail (path history)
        self.ego_trail_x.append(ego_x)
        self.ego_trail_y.append(ego_y)
        self.ego_trail_line.set_data(self.ego_trail_x, self.ego_trail_y)
        
        # Update traffic vehicles
        while len(self.traffic_patches) < len(self.sim.traffic):
            patch = self._create_vehicle_patch('salmon', 'darkred')
            self.ax_main.add_patch(patch)
            self.traffic_patches.append(patch)
        
        for i, v in enumerate(self.sim.traffic):
            v_x, v_y = frenet_to_cartesian(v.s, v.d, self.ref_path)
            v_angle = self._get_road_angle(v.s)
            self._update_vehicle_patch(self.traffic_patches[i], v_x, v_y, v_angle)
        
        # View is fixed (set in _setup_main_view) - no camera following
        # This shows the full road path without jerky motion
        
        # Track behavior changes
        current_behavior = command.behavior.value
        if current_behavior != self.last_behavior:
            self.behavior_log.append((self.sim.time, current_behavior, self.sim.ego.s))
            self.last_behavior = current_behavior
        
        # Update trajectory visualization
        if self.sim.current_trajectory is not None:
            traj = self.sim.current_trajectory
            
            # Convert trajectory to Cartesian for main view
            traj_x = []
            traj_y = []
            for state in traj.states:
                x, y = frenet_to_cartesian(state.s, state.d, self.ref_path)
                traj_x.append(x)
                traj_y.append(y)
            
            self.traj_line.set_data(traj_x, traj_y)
            self.traj_points.set_data(traj_x[::3], traj_y[::3])
            
            # Update coefficients display
            s_coeffs = traj.s_coeffs
            d_coeffs = traj.d_coeffs
            
            # Get start and end values
            s_start = traj.states[0].s if traj.states else 0
            s_end = traj.states[-1].s if traj.states else 0
            d_start = traj.states[0].d if traj.states else 0
            d_end = traj.states[-1].d if traj.states else 0
            
            s_dot_start = traj.states[0].s_dot if traj.states else 0
            s_dot_end = traj.states[-1].s_dot if traj.states else 0
            d_dot_start = traj.states[0].d_dot if traj.states else 0
            d_dot_end = traj.states[-1].d_dot if traj.states else 0
            
            coeffs_str = (
                f"Duration: T = {traj.T:.2f}s\n"
                f"\n"
                f"Longitudinal (s):\n"
                f"  Start: s={s_start:.1f}m, ṡ={s_dot_start:.1f}m/s\n"
                f"  End:   s={s_end:.1f}m, ṡ={s_dot_end:.1f}m/s\n"
                f"  Coeffs: [{', '.join(f'{c:.2f}' for c in s_coeffs[:4])}...]\n"
                f"\n"
                f"Lateral (d):\n"
                f"  Start: d={d_start:.2f}m, ḋ={d_dot_start:.2f}m/s\n"
                f"  End:   d={d_end:.2f}m, ḋ={d_dot_end:.2f}m/s\n"
                f"  Coeffs: [{', '.join(f'{c:.2f}' for c in d_coeffs[:4])}...]\n"
                f"\n"
                f"Cost: {traj.cost:.1f}"
            )
            self.coeffs_text.set_text(coeffs_str)
            
            # Trajectory info text
            traj_info = f"T={traj.T:.1f}s\nCost={traj.cost:.1f}"
            self.traj_info_text.set_text(traj_info)
        else:
            self.traj_line.set_data([], [])
            self.traj_points.set_data([], [])
            self.coeffs_text.set_text("No trajectory")
            self.traj_info_text.set_text("No trajectory")
        
        # Update behavior summary display
        behavior_lines = ["Time      Pos       Behavior", "-" * 35]
        # Show last 8 behavior changes
        for time, behavior, pos in self.behavior_log[-8:]:
            behavior_display = behavior.upper().replace('_', ' ')
            behavior_lines.append(f"{time:>5.1f}s  {pos:>6.1f}m  {behavior_display}")
        self.behavior_summary_text.set_text('\n'.join(behavior_lines))
        
        # Update text annotations
        self.time_text.set_text(
            f"Time: {self.sim.time:.1f}s\n"
            f"Speed: {self.sim.ego.speed:.1f} m/s\n"
            f"s: {self.sim.ego.s:.1f} m\n"
            f"d: {self.sim.ego.d:.2f} m"
        )
        
        behavior_name = command.behavior.value.upper().replace('_', ' ')
        self.behavior_text.set_text(f"{behavior_name}")
        
        # Set color based on behavior type
        behavior_colors = {
            'lane_keep': 'darkgreen',
            'follow_vehicle': 'darkorange', 
            'lane_change_left': 'darkblue',
            'lane_change_right': 'purple'
        }
        color = behavior_colors.get(command.behavior.value, 'black')
        self.behavior_text.set_color(color)
        
        return [self.ego_patch, self.ego_trail_line, self.traj_line, self.traj_points,
                self.time_text, self.behavior_text, self.traj_info_text,
                self.behavior_summary_text, self.coeffs_text] + self.traffic_patches
    
    def run(self, duration: float, planner):
        """
        Run the visualization.
        
        Args:
            duration: Simulation duration in seconds
            planner: Behavior planner instance
        """
        # Clear ego trail and behavior log for new run
        self.ego_trail_x = []
        self.ego_trail_y = []
        self.behavior_log = []
        self.last_behavior = None
        
        n_frames = int(duration / self.sim.config.dt) + 1
        
        ani = FuncAnimation(
            self.fig,
            lambda frame: self.update(frame, planner, duration),
            frames=n_frames,
            interval=int(self.sim.config.dt * 1000),
            blit=False,
            repeat=False
        )
        
        plt.show()


# =============================================================================
# Matplotlib Visualization (Static, for testing)
# =============================================================================

def visualize_trajectory_static(
    ref_path,
    selected_trajectory=None,
    candidate_trajectories=None,
    title="Trajectory Visualization"
):
    """
    Static visualization of trajectories (for testing).
    
    Args:
        ref_path: Reference path
        selected_trajectory: Best trajectory (optional)
        candidate_trajectories: List of candidate trajectories (optional)
        title: Plot title
    """
    from frenet import frenet_to_cartesian
    
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Plot reference path
    ax.plot(
        ref_path.points[:, 0],
        ref_path.points[:, 1],
        'k-',
        linewidth=2,
        label='Reference Path',
        alpha=0.7
    )
    
    # Plot lane boundaries
    lane_width = 3.5
    for offset in [-lane_width * 1.5, -lane_width * 0.5, lane_width * 0.5, lane_width * 1.5]:
        boundary = ref_path.points + offset * ref_path.normals
        linestyle = '-' if abs(offset) > lane_width else '--'
        ax.plot(boundary[:, 0], boundary[:, 1], 'gray', linewidth=1, 
                linestyle=linestyle, alpha=0.5)
    
    # Plot candidate trajectories
    if candidate_trajectories:
        for i, traj in enumerate(candidate_trajectories):
            points_xy = []
            for state in traj.states:
                x, y = frenet_to_cartesian(state.s, state.d, ref_path)
                points_xy.append([x, y])
            points_xy = np.array(points_xy)
            
            label = 'Candidates' if i == 0 else None
            ax.plot(points_xy[:, 0], points_xy[:, 1], 'c-', linewidth=1, 
                   alpha=0.3, label=label)
    
    # Plot selected trajectory
    if selected_trajectory:
        points_xy = []
        for state in selected_trajectory.states:
            x, y = frenet_to_cartesian(state.s, state.d, ref_path)
            points_xy.append([x, y])
        points_xy = np.array(points_xy)
        
        ax.plot(points_xy[:, 0], points_xy[:, 1], 'g-', linewidth=3, 
               label='Selected Trajectory')
        ax.scatter(points_xy[::3, 0], points_xy[::3, 1], c='green', s=30, zorder=5)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


# =============================================================================
# Unit Tests
# =============================================================================

if __name__ == "__main__":
    print("Testing visualizer...")
    
    # Test with actual simulator
    from simulator import Simulator, Scenario
    from behavior_tree import BehaviorPlanner
    
    sim = Simulator()
    sim.setup_scenario(Scenario.OVERTAKE)
    planner = BehaviorPlanner()
    
    print(f"Reference path: {len(sim.ref_path.points)} points")
    print(f"Total length: {sim.ref_path.total_length:.1f}m")
    
    viz = SimulationVisualizer(sim)
    viz.run(30.0, planner)
