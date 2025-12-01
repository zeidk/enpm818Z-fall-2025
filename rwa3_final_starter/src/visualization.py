"""
Visualization Module (PROVIDED)

This module provides real-time visualization for debugging and analysis.

Students should NOT modify this file.
"""

import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("Warning: pygame not available. Visualization disabled.")

try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

from .trajectory_planner import Trajectory


class PygameVisualizer:
    """
    Real-time visualization using pygame.
    """
    
    def __init__(self, width: int = 1280, height: int = 720,
                 scale: float = 5.0, title: str = "ENPM818Z - AV Planning"):
        """
        Initialize pygame visualizer.
        
        Args:
            width: Window width in pixels
            height: Window height in pixels
            scale: Pixels per meter
            title: Window title
        """
        if not PYGAME_AVAILABLE:
            self.enabled = False
            return
        
        self.enabled = True
        self.width = width
        self.height = height
        self.scale = scale
        
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption(title)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 16)
        
        # Camera position (follows ego vehicle)
        self.camera_x = 0.0
        self.camera_y = 0.0
        
        # Colors
        self.colors = {
            'background': (40, 40, 40),
            'road': (60, 60, 60),
            'lane_marking': (255, 255, 255),
            'ego': (0, 150, 255),
            'obstacle': (255, 80, 80),
            'trajectory': (0, 255, 100),
            'reference_path': (255, 255, 0),
            'text': (255, 255, 255)
        }
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        sx = int((x - self.camera_x) * self.scale + self.width / 2)
        sy = int(self.height / 2 - (y - self.camera_y) * self.scale)
        return sx, sy
    
    def update(self, ego_state: Dict, obstacles: List[Dict],
               trajectory: Optional[Trajectory] = None,
               reference_path: Optional[List[Tuple[float, float]]] = None,
               info: Optional[Dict] = None):
        """
        Update visualization.
        
        Args:
            ego_state: Dict with x, y, theta, v
            obstacles: List of obstacle dicts
            trajectory: Planned trajectory
            reference_path: Reference path waypoints
            info: Additional info to display
        """
        if not self.enabled:
            return True
        
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.scale *= 1.2
                elif event.key == pygame.K_MINUS:
                    self.scale /= 1.2
        
        # Update camera to follow ego
        self.camera_x = ego_state['x']
        self.camera_y = ego_state['y']
        
        # Clear screen
        self.screen.fill(self.colors['background'])
        
        # Draw reference path
        if reference_path and len(reference_path) > 1:
            points = [self.world_to_screen(p[0], p[1]) for p in reference_path]
            pygame.draw.lines(self.screen, self.colors['reference_path'], 
                              False, points, 2)
        
        # Draw trajectory
        if trajectory and trajectory.points:
            points = [self.world_to_screen(p.x, p.y) for p in trajectory.points]
            if len(points) > 1:
                pygame.draw.lines(self.screen, self.colors['trajectory'], 
                                  False, points, 3)
        
        # Draw obstacles
        for obs in obstacles:
            sx, sy = self.world_to_screen(obs['x'], obs['y'])
            length = obs.get('length', 4.5) * self.scale
            width = obs.get('width', 2.0) * self.scale
            
            # Draw as rectangle
            rect = pygame.Rect(sx - length/2, sy - width/2, length, width)
            pygame.draw.rect(self.screen, self.colors['obstacle'], rect)
            pygame.draw.rect(self.screen, (255, 255, 255), rect, 1)
        
        # Draw ego vehicle
        ex, ey = self.world_to_screen(ego_state['x'], ego_state['y'])
        ego_length = 4.5 * self.scale
        ego_width = 2.0 * self.scale
        
        # Create rotated rectangle for ego
        theta = ego_state['theta']
        corners = [
            (ego_length/2, ego_width/2),
            (ego_length/2, -ego_width/2),
            (-ego_length/2, -ego_width/2),
            (-ego_length/2, ego_width/2)
        ]
        
        rotated = []
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        for cx, cy in corners:
            rx = cx * cos_t - cy * sin_t + ex
            ry = -(cx * sin_t + cy * cos_t) + ey  # Flip y for screen coords
            rotated.append((rx, ry))
        
        pygame.draw.polygon(self.screen, self.colors['ego'], rotated)
        pygame.draw.polygon(self.screen, (255, 255, 255), rotated, 2)
        
        # Draw heading indicator
        head_x = ex + np.cos(theta) * ego_length * 0.7
        head_y = ey - np.sin(theta) * ego_length * 0.7
        pygame.draw.line(self.screen, (255, 255, 255), (ex, ey), 
                         (head_x, head_y), 2)
        
        # Draw info text
        y_offset = 10
        if info:
            for key, value in info.items():
                text = self.font.render(f"{key}: {value}", True, 
                                        self.colors['text'])
                self.screen.blit(text, (10, y_offset))
                y_offset += 20
        
        # Draw ego state
        state_texts = [
            f"Speed: {ego_state['v']:.1f} m/s",
            f"Position: ({ego_state['x']:.1f}, {ego_state['y']:.1f})",
            f"Heading: {np.degrees(ego_state['theta']):.1f}°"
        ]
        
        for text_str in state_texts:
            text = self.font.render(text_str, True, self.colors['text'])
            self.screen.blit(text, (10, y_offset))
            y_offset += 20
        
        # Update display
        pygame.display.flip()
        self.clock.tick(60)
        
        return True
    
    def close(self):
        """Close the visualization window."""
        if self.enabled:
            pygame.quit()


class MatplotlibVisualizer:
    """
    Static visualization using matplotlib for analysis.
    """
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        """Initialize matplotlib visualizer."""
        if not MATPLOTLIB_AVAILABLE:
            self.enabled = False
            return
        
        self.enabled = True
        self.figsize = figsize
    
    def plot_trajectory(self, trajectory: Trajectory,
                        reference_path: Optional[List[Tuple[float, float]]] = None,
                        obstacles: Optional[List[Dict]] = None,
                        title: str = "Trajectory"):
        """
        Plot a trajectory with optional reference path and obstacles.
        """
        if not self.enabled:
            return
        
        fig, axes = plt.subplots(2, 2, figsize=self.figsize)
        
        # XY plot
        ax = axes[0, 0]
        if reference_path:
            rx = [p[0] for p in reference_path]
            ry = [p[1] for p in reference_path]
            ax.plot(rx, ry, 'y--', label='Reference', linewidth=2)
        
        if trajectory.points:
            tx = [p.x for p in trajectory.points]
            ty = [p.y for p in trajectory.points]
            ax.plot(tx, ty, 'g-', label='Trajectory', linewidth=2)
            ax.plot(tx[0], ty[0], 'go', markersize=10, label='Start')
            ax.plot(tx[-1], ty[-1], 'r^', markersize=10, label='End')
        
        if obstacles:
            for obs in obstacles:
                circle = Circle((obs['x'], obs['y']), 
                                 obs.get('length', 4.5)/2,
                                 color='red', alpha=0.5)
                ax.add_patch(circle)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Path')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)
        
        # Velocity profile
        ax = axes[0, 1]
        if trajectory.points:
            times = [p.t for p in trajectory.points]
            velocities = [p.v for p in trajectory.points]
            ax.plot(times, velocities, 'b-', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.grid(True)
        
        # Steering/Curvature
        ax = axes[1, 0]
        if trajectory.points:
            times = [p.t for p in trajectory.points]
            curvatures = [p.kappa for p in trajectory.points]
            ax.plot(times, curvatures, 'r-', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Curvature (1/m)')
        ax.set_title('Curvature Profile')
        ax.grid(True)
        
        # Acceleration
        ax = axes[1, 1]
        if trajectory.points:
            times = [p.t for p in trajectory.points]
            accels = [p.a for p in trajectory.points]
            ax.plot(times, accels, 'm-', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        ax.set_title('Acceleration Profile')
        ax.grid(True)
        
        plt.suptitle(title)
        plt.tight_layout()
        plt.show()
    
    def plot_comparison(self, trajectories: List[Trajectory],
                        labels: List[str],
                        title: str = "Trajectory Comparison"):
        """
        Compare multiple trajectories.
        """
        if not self.enabled:
            return
        
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        
        colors = ['b', 'r', 'g', 'c', 'm', 'y']
        
        # XY comparison
        ax = axes[0]
        for i, (traj, label) in enumerate(zip(trajectories, labels)):
            if traj.points:
                tx = [p.x for p in traj.points]
                ty = [p.y for p in traj.points]
                ax.plot(tx, ty, f'{colors[i % len(colors)]}-', 
                        label=label, linewidth=2)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Paths')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)
        
        # Velocity comparison
        ax = axes[1]
        for i, (traj, label) in enumerate(zip(trajectories, labels)):
            if traj.points:
                times = [p.t for p in traj.points]
                velocities = [p.v for p in traj.points]
                ax.plot(times, velocities, f'{colors[i % len(colors)]}-',
                        label=label, linewidth=2)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profiles')
        ax.legend()
        ax.grid(True)
        
        plt.suptitle(title)
        plt.tight_layout()
        plt.show()


# =============================================================================
# TESTING
# =============================================================================

if __name__ == '__main__':
    print("Testing Visualization Module...")
    print("=" * 50)
    
    if PYGAME_AVAILABLE:
        print("Pygame is available")
        
        # Quick pygame test
        vis = PygameVisualizer()
        
        ego_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 20.0}
        obstacles = [
            {'x': 30.0, 'y': 0.0, 'length': 4.5, 'width': 2.0},
            {'x': 50.0, 'y': 3.5, 'length': 4.5, 'width': 2.0}
        ]
        reference_path = [(i * 2.0, 0.0) for i in range(50)]
        
        print("Opening pygame window... (press ESC to close)")
        
        running = True
        frame = 0
        while running and frame < 200:  # Run for ~3 seconds
            ego_state['x'] = frame * 0.5
            running = vis.update(
                ego_state, obstacles,
                reference_path=reference_path,
                info={'Frame': frame, 'Status': 'Testing'}
            )
            frame += 1
        
        vis.close()
        print("Pygame test complete")
    else:
        print("Pygame not available, skipping pygame test")
    
    if MATPLOTLIB_AVAILABLE:
        print("\nMatplotlib is available")
        print("(Matplotlib visualization can be used for static analysis)")
    else:
        print("\nMatplotlib not available")
    
    print("\nVisualization module test complete!")
