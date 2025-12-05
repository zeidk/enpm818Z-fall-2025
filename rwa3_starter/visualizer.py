"""
visualizer.py - Matplotlib Visualization for Highway Simulator

Provides real-time visualization of the highway simulation
with ego vehicle, traffic, and lane markings.

DO NOT MODIFY THIS FILE.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.animation import FuncAnimation
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simulator import Simulator
    from behavior_tree import BehaviorPlanner


class SimulationVisualizer:
    """
    Real-time visualization of highway simulation.
    
    Shows:
    - Ego vehicle (green)
    - Traffic vehicles (red)
    - Lane markings
    - Current behavior and speed
    """
    
    def __init__(self, simulator: 'Simulator'):
        self.sim = simulator
        self.planner = None
        
        # Visualization parameters
        self.view_ahead = 80.0    # Meters ahead to show
        self.view_behind = 20.0   # Meters behind to show
        self.view_width = 15.0    # Total lateral width to show
        
        # Set up figure
        self.fig, self.ax = plt.subplots(figsize=(14, 6))
        self.fig.canvas.manager.set_window_title('Highway Simulation - RWA3')
        
        # Initialize plot elements
        self._init_plot()
    
    def _init_plot(self):
        """Initialize plot elements."""
        self.ax.set_aspect('equal')
        self.ax.set_facecolor('#404040')  # Dark gray road
        
        # Create ego vehicle patch
        self.ego_patch = FancyBboxPatch(
            (0, 0), 4.5, 2.0,
            boxstyle="round,pad=0.1",
            facecolor='#00AA00',
            edgecolor='white',
            linewidth=2,
            zorder=10
        )
        self.ax.add_patch(self.ego_patch)
        
        # Create traffic vehicle patches (will add dynamically)
        self.traffic_patches = []
        
        # Text annotations
        self.info_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            fontsize=12, fontfamily='monospace',
            verticalalignment='top',
            color='white',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7)
        )
        
        self.behavior_text = self.ax.text(
            0.98, 0.98, '', transform=self.ax.transAxes,
            fontsize=14, fontweight='bold',
            horizontalalignment='right',
            verticalalignment='top',
            color='yellow',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7)
        )
    
    def _draw_road(self):
        """Draw road surface and lane markings."""
        ego_x = self.sim.ego.x
        
        # Road boundaries
        x_min = ego_x - self.view_behind
        x_max = ego_x + self.view_ahead
        
        lane_width = self.sim.config.lane_width
        num_lanes = self.sim.config.num_lanes
        
        # Draw lane markings
        for i in range(-(num_lanes // 2), (num_lanes // 2) + 2):
            y = (i - 0.5) * lane_width
            
            if i == -(num_lanes // 2) or i == (num_lanes // 2) + 1:
                # Solid edge lines
                self.ax.plot([x_min, x_max], [y, y], 'w-', linewidth=2)
            else:
                # Dashed lane dividers
                self.ax.plot([x_min, x_max], [y, y], 'w--', linewidth=1, alpha=0.7)
        
        # Draw center reference line
        self.ax.plot([x_min, x_max], [0, 0], 'y--', linewidth=1, alpha=0.5)
    
    def _update_vehicles(self):
        """Update vehicle positions."""
        # Update ego vehicle
        ego = self.sim.ego
        self.ego_patch.set_x(ego.x - ego.length/2)
        self.ego_patch.set_y(ego.y - ego.width/2)
        
        # Remove old traffic patches
        for patch in self.traffic_patches:
            patch.remove()
        self.traffic_patches.clear()
        
        # Add traffic vehicles
        for v in self.sim.traffic:
            patch = FancyBboxPatch(
                (v.x - v.length/2, v.y - v.width/2),
                v.length, v.width,
                boxstyle="round,pad=0.1",
                facecolor='#CC0000',
                edgecolor='white',
                linewidth=1,
                zorder=5
            )
            self.ax.add_patch(patch)
            self.traffic_patches.append(patch)
    
    def _update_text(self):
        """Update text annotations."""
        # Info text
        env = self.sim.get_environment_state()
        info = (
            f"Time: {self.sim.time:.1f}s\n"
            f"Speed: {self.sim.ego.speed:.1f} m/s ({self.sim.ego.speed * 2.237:.0f} mph)\n"
            f"Position: x={self.sim.ego.x:.1f}m, y={self.sim.ego.y:.2f}m\n"
            f"Vehicle ahead: {'Yes' if env.vehicle_ahead else 'No'}"
        )
        if env.vehicle_ahead:
            info += f" ({env.vehicle_ahead_distance:.0f}m, {env.vehicle_ahead_speed:.0f}m/s)"
        
        self.info_text.set_text(info)
        
        # Behavior text
        if self.sim.current_command:
            behavior = self.sim.current_command.behavior.value.upper().replace('_', ' ')
            self.behavior_text.set_text(behavior)
            
            # Color based on behavior
            colors = {
                'LANE KEEP': '#00FF00',
                'FOLLOW VEHICLE': '#FFFF00',
                'LANE CHANGE LEFT': '#00FFFF',
                'LANE CHANGE RIGHT': '#FF00FF',
                'STOP': '#FF0000'
            }
            self.behavior_text.set_color(colors.get(behavior, 'white'))
    
    def _update_view(self):
        """Update view limits to follow ego vehicle."""
        ego_x = self.sim.ego.x
        
        self.ax.set_xlim(ego_x - self.view_behind, ego_x + self.view_ahead)
        self.ax.set_ylim(-self.view_width/2, self.view_width/2)
        
        self.ax.set_xlabel('Longitudinal Position (m)', fontsize=10)
        self.ax.set_ylabel('Lateral Position (m)', fontsize=10)
    
    def update(self, frame: int) -> list:
        """Update function for animation."""
        # Clear previous road markings
        # Keep only patches and text, remove lines
        lines_to_remove = [line for line in self.ax.lines]
        for line in lines_to_remove:
            line.remove()
        
        # Get environment and command from planner
        env = self.sim.get_environment_state()
        command = self.planner.get_command(env)
        
        # Apply command and step simulation
        self.sim.apply_command(command)
        self.sim.step()
        
        # Update visualization
        self._draw_road()
        self._update_vehicles()
        self._update_text()
        self._update_view()
        
        return [self.ego_patch, self.info_text, self.behavior_text] + self.traffic_patches
    
    def run(self, duration: float, planner: 'BehaviorPlanner'):
        """
        Run the visualization.
        
        Args:
            duration: Simulation duration in seconds
            planner: BehaviorPlanner to use for decisions
        """
        self.planner = planner
        
        # Calculate frames
        n_frames = int(duration / self.sim.config.dt)
        interval = int(self.sim.config.dt * 1000)  # Convert to milliseconds
        
        print(f"Starting visualization with {n_frames} frames...")
        print("Close the window to stop the simulation.\n")
        
        # Create animation
        anim = FuncAnimation(
            self.fig,
            self.update,
            frames=n_frames,
            interval=interval,
            blit=False,
            repeat=False
        )
        
        plt.tight_layout()
        plt.show()


def test_visualizer():
    """Test the visualizer with a simple scenario."""
    from simulator import Simulator, Scenario
    from bt_framework import BehaviorCommand, BehaviorType
    
    # Create a mock planner that just does lane keeping
    class MockPlanner:
        def get_command(self, env):
            return BehaviorCommand(
                behavior=BehaviorType.LANE_KEEP,
                target_d=0.0,
                target_speed=31.0,
                T=3.0
            )
    
    sim = Simulator()
    sim.setup_scenario(Scenario.OVERTAKE)
    
    viz = SimulationVisualizer(sim)
    viz.run(duration=10.0, planner=MockPlanner())


if __name__ == "__main__":
    print("Testing visualizer with mock planner...")
    print("This should show lane keeping behavior only.\n")
    test_visualizer()
