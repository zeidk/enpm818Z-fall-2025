#!/usr/bin/env python3
"""
carla_simulator.py - CARLA Highway Simulator with Pygame Visualization

This script runs the behavior tree planner in CARLA with a real-time
pygame window showing the vehicle camera feed and behavior status.

Usage:
    # Make sure CARLA server is running first!
    
    # Run scenarios:
    python carla_simulator.py --scenario empty
    python carla_simulator.py --scenario follow
    python carla_simulator.py --scenario overtake --duration 60

Controls:
    ESC - Quit simulation
    P   - Pause/Resume
    R   - Reset scenario

Requirements:
    - CARLA 0.9.13 or later running on localhost:2000
    - Python packages: carla, pygame, numpy
    - Recommended: Town04 map (has dedicated highway)

DO NOT MODIFY THIS FILE.
"""

import argparse
import sys
import os
import time
import signal
import weakref
from typing import Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import carla
except ImportError:
    print("ERROR: CARLA Python package not found!")
    print("Install with: pip install carla")
    sys.exit(1)

try:
    import pygame
    from pygame.locals import K_ESCAPE, K_p, K_r
except ImportError:
    print("ERROR: Pygame not found!")
    print("Install with: pip install pygame")
    sys.exit(1)

import numpy as np

from carla_interface import CarlaInterface, CarlaConfig, Scenario
from bt_framework import BehaviorType


# =============================================================================
# Pygame Display Configuration
# =============================================================================

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
FPS = 30  # Display refresh rate


# =============================================================================
# Camera Manager
# =============================================================================

class CameraManager:
    """Manages camera sensor attached to ego vehicle."""
    
    def __init__(self, vehicle: carla.Vehicle, world: carla.World):
        self.vehicle = vehicle
        self.world = world
        self.sensor = None
        self.surface = None
        self._image_data = None
        
        # Camera settings
        self.width = WINDOW_WIDTH
        self.height = WINDOW_HEIGHT
        
        # Spawn camera
        self._spawn_camera()
    
    def _spawn_camera(self):
        """Spawn camera sensor attached to vehicle."""
        bp_library = self.world.get_blueprint_library()
        
        # Get camera blueprint
        camera_bp = bp_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')
        
        # Camera position: behind and above the vehicle
        camera_transform = carla.Transform(
            carla.Location(x=-8.0, y=0.0, z=5.0),
            carla.Rotation(pitch=-15.0)
        )
        
        # Spawn and attach to vehicle
        self.sensor = self.world.spawn_actor(
            camera_bp, 
            camera_transform, 
            attach_to=self.vehicle
        )
        
        # Set up callback
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: CameraManager._on_image(weak_self, image))
    
    @staticmethod
    def _on_image(weak_self, image):
        """Callback for camera images."""
        self = weak_self()
        if self is None:
            return
        
        # Convert CARLA image to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA
        array = array[:, :, :3]  # Remove alpha
        array = array[:, :, ::-1]  # BGR to RGB
        
        self._image_data = array
    
    def get_surface(self) -> Optional[pygame.Surface]:
        """Get pygame surface from latest camera image."""
        if self._image_data is None:
            return None
        
        # Convert numpy array to pygame surface
        self.surface = pygame.surfarray.make_surface(
            self._image_data.swapaxes(0, 1)
        )
        return self.surface
    
    def destroy(self):
        """Clean up camera sensor."""
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
            self.sensor = None


# =============================================================================
# HUD (Heads-Up Display)
# =============================================================================

class HUD:
    """Displays information overlay on pygame window."""
    
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        
        # Initialize fonts
        pygame.font.init()
        self.font_large = pygame.font.Font(None, 48)
        self.font_medium = pygame.font.Font(None, 36)
        self.font_small = pygame.font.Font(None, 28)
        
        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GREEN = (0, 255, 0)
        self.YELLOW = (255, 255, 0)
        self.CYAN = (0, 255, 255)
        self.MAGENTA = (255, 0, 255)
        self.RED = (255, 0, 0)
        
        # Behavior colors
        self.behavior_colors = {
            BehaviorType.LANE_KEEP: self.GREEN,
            BehaviorType.FOLLOW_VEHICLE: self.YELLOW,
            BehaviorType.LANE_CHANGE_LEFT: self.CYAN,
            BehaviorType.LANE_CHANGE_RIGHT: self.MAGENTA,
            BehaviorType.STOP: self.RED,
        }
    
    def render(self, display: pygame.Surface, info: dict):
        """Render HUD overlay."""
        # Semi-transparent background for info panel
        info_surface = pygame.Surface((350, 230))
        info_surface.set_alpha(180)
        info_surface.fill(self.BLACK)
        display.blit(info_surface, (10, 10))
        
        # Current behavior (large, colored)
        behavior = info.get('behavior', BehaviorType.LANE_KEEP)
        behavior_text = behavior.value.upper().replace('_', ' ')
        behavior_color = self.behavior_colors.get(behavior, self.WHITE)
        
        text = self.font_large.render(behavior_text, True, behavior_color)
        display.blit(text, (20, 20))
        
        # Speed
        speed = info.get('speed', 0)
        speed_mph = speed * 2.237  # m/s to mph
        speed_text = f"Speed: {speed:.1f} m/s ({speed_mph:.0f} mph)"
        text = self.font_medium.render(speed_text, True, self.WHITE)
        display.blit(text, (20, 70))
        
        # Time
        sim_time = info.get('time', 0)
        time_text = f"Time: {sim_time:.1f}s"
        text = self.font_medium.render(time_text, True, self.WHITE)
        display.blit(text, (20, 105))
        
        # Ego lateral position (d value)
        ego_d = info.get('ego_d', 0)
        lane_name = "CENTER"
        if ego_d > 2.0:
            lane_name = "LEFT"
        elif ego_d < -2.0:
            lane_name = "RIGHT"
        d_text = f"Lane: {lane_name} (d={ego_d:.1f}m)"
        text = self.font_small.render(d_text, True, self.CYAN)
        display.blit(text, (20, 140))
        
        # Vehicle ahead info
        if info.get('vehicle_ahead', False):
            ahead_dist = info.get('vehicle_ahead_distance', 0)
            ahead_speed = info.get('vehicle_ahead_speed', 0)
            ahead_text = f"Vehicle ahead: {ahead_dist:.0f}m @ {ahead_speed:.0f} m/s"
            text = self.font_small.render(ahead_text, True, self.YELLOW)
            display.blit(text, (20, 170))
        else:
            text = self.font_small.render("No vehicle ahead", True, self.GREEN)
            display.blit(text, (20, 170))
        
        # Lane info
        left_clear = "OK" if info.get('left_lane_clear', False) else "X"
        right_clear = "OK" if info.get('right_lane_clear', False) else "X"
        left_color = self.GREEN if info.get('left_lane_clear', False) else self.RED
        right_color = self.GREEN if info.get('right_lane_clear', False) else self.RED
        
        # Render lane status with colors
        lane_label = self.font_small.render("Lanes: L=", True, self.WHITE)
        display.blit(lane_label, (20, 200))
        left_status = self.font_small.render(left_clear, True, left_color)
        display.blit(left_status, (110, 200))
        right_label = self.font_small.render(" R=", True, self.WHITE)
        display.blit(right_label, (145, 200))
        right_status = self.font_small.render(right_clear, True, right_color)
        display.blit(right_status, (175, 200))
        
        # Controls help (bottom of screen)
        help_surface = pygame.Surface((300, 30))
        help_surface.set_alpha(150)
        help_surface.fill(self.BLACK)
        display.blit(help_surface, (10, self.height - 40))
        
        help_text = "ESC: Quit | P: Pause | R: Reset"
        text = self.font_small.render(help_text, True, self.WHITE)
        display.blit(text, (20, self.height - 35))
        
        # Pause indicator
        if info.get('paused', False):
            pause_text = self.font_large.render("PAUSED", True, self.YELLOW)
            text_rect = pause_text.get_rect(center=(self.width // 2, self.height // 2))
            display.blit(pause_text, text_rect)


# =============================================================================
# Main Simulation Loop
# =============================================================================

class CarlaSimulation:
    """Main simulation class with pygame visualization."""
    
    def __init__(self, scenario: Scenario, duration: float):
        self.scenario = scenario
        self.duration = duration
        
        # CARLA interface
        self.interface: Optional[CarlaInterface] = None
        self.planner = None
        
        # Pygame
        self.display = None
        self.clock = None
        self.camera = None
        self.hud = None
        
        # State
        self.running = False
        self.paused = False
        self.sim_time = 0.0
    
    def setup(self) -> bool:
        """Initialize CARLA and pygame."""
        # Import behavior planner
        try:
            from behavior_tree import BehaviorPlanner
        except ImportError as e:
            print(f"ERROR: Could not import BehaviorPlanner: {e}")
            return False
        
        # Initialize pygame
        pygame.init()
        pygame.display.set_caption(f'CARLA Simulation - {self.scenario.value.upper()}')
        
        self.display = pygame.display.set_mode(
            (WINDOW_WIDTH, WINDOW_HEIGHT),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self.clock = pygame.time.Clock()
        self.hud = HUD(WINDOW_WIDTH, WINDOW_HEIGHT)
        
        # Connect to CARLA
        config = CarlaConfig()
        self.interface = CarlaInterface(config)
        
        if not self.interface.connect():
            print("\nFailed to connect to CARLA!")
            print("Make sure CARLA server is running.")
            return False
        
        # Spawn ego vehicle
        if not self.interface.spawn_ego_vehicle():
            print("Failed to spawn ego vehicle!")
            return False
        
        # Attach camera
        self.camera = CameraManager(
            self.interface.ego_vehicle,
            self.interface.world
        )
        
        # Setup scenario
        self.interface.setup_scenario(self.scenario)
        
        # Create planner
        self.planner = BehaviorPlanner(
            speed_limit=config.speed_limit,
            lane_width=config.lane_width
        )
        
        print("\nSimulation ready!")
        print(f"Scenario: {self.scenario.value}")
        print(f"Duration: {self.duration}s")
        print("\nControls: ESC=Quit, P=Pause, R=Reset\n")
        
        return True
    
    def run(self):
        """Main simulation loop."""
        self.running = True
        self.sim_time = 0.0
        
        # For real-time pacing
        dt = self.interface.config.fixed_delta_seconds
        last_tick_time = time.time()
        
        behavior_history = []
        
        try:
            while self.running and self.sim_time < self.duration:
                # Handle pygame events
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == K_ESCAPE:
                            self.running = False
                        elif event.key == K_p:
                            self.paused = not self.paused
                            print("PAUSED" if self.paused else "RESUMED")
                        elif event.key == K_r:
                            print("Resetting scenario...")
                            self.interface.setup_scenario(self.scenario)
                            self.sim_time = 0.0
                            behavior_history.clear()
                
                if not self.paused:
                    # Real-time pacing: wait until enough time has passed
                    current_time = time.time()
                    elapsed = current_time - last_tick_time
                    
                    if elapsed < dt:
                        # Sleep to maintain real-time
                        time.sleep(dt - elapsed)
                    
                    last_tick_time = time.time()
                    
                    # Get environment state
                    env = self.interface.get_environment_state()
                    
                    # Get behavior command
                    command = self.planner.get_command(env)
                    
                    # Track behavior changes
                    if not behavior_history or behavior_history[-1] != command.behavior:
                        behavior_history.append(command.behavior)
                        print(f"[{self.sim_time:6.1f}s] Behavior: {command.behavior.value.upper()}")
                    
                    # Apply command
                    self.interface.apply_command(command)
                    
                    # Tick CARLA (this advances simulation)
                    self.interface.tick()
                    self.sim_time += dt
                    
                    # Prepare HUD info
                    hud_info = {
                        'behavior': command.behavior,
                        'speed': env.ego_speed,
                        'ego_d': env.ego_d,
                        'time': self.sim_time,
                        'vehicle_ahead': env.vehicle_ahead,
                        'vehicle_ahead_distance': env.vehicle_ahead_distance,
                        'vehicle_ahead_speed': env.vehicle_ahead_speed,
                        'left_lane_clear': env.left_lane_clear,
                        'right_lane_clear': env.right_lane_clear,
                        'paused': False,
                    }
                else:
                    hud_info = {'paused': True, 'time': self.sim_time}
                
                # Render
                self._render(hud_info)
                
                # Limit display refresh rate
                self.clock.tick(FPS)
            
            # Simulation complete
            print(f"\n{'=' * 60}")
            print("SIMULATION COMPLETE")
            print(f"{'=' * 60}")
            print(f"Duration: {self.sim_time:.1f}s")
            print(f"\nBehavior sequence:")
            for i, b in enumerate(behavior_history):
                print(f"  {i+1}. {b.value.upper()}")
            
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        finally:
            self.cleanup()
    
    def _render(self, hud_info: dict):
        """Render frame to pygame display."""
        # Get camera image
        surface = self.camera.get_surface()
        
        if surface is not None:
            self.display.blit(surface, (0, 0))
        else:
            # No image yet, fill with dark gray
            self.display.fill((50, 50, 50))
        
        # Render HUD overlay
        self.hud.render(self.display, hud_info)
        
        # Update display
        pygame.display.flip()
    
    def cleanup(self):
        """Clean up resources."""
        print("\nCleaning up...")
        
        if self.camera is not None:
            self.camera.destroy()
        
        if self.interface is not None:
            self.interface.cleanup()
        
        pygame.quit()
        print("Done!")


# =============================================================================
# Entry Point
# =============================================================================

def print_banner():
    """Print startup banner."""
    print("=" * 70)
    print("  CARLA Highway Simulator - RWA3 Behavioral Planning")
    print("  ENPM818Z: On-Road Automated Vehicles")
    print("=" * 70)


def print_scenario_info(scenario: Scenario):
    """Print scenario information."""
    info = {
        Scenario.EMPTY: (
            "EMPTY ROAD",
            "No traffic. Cruise at speed limit.",
            "Expected: LANE_KEEP"
        ),
        Scenario.FOLLOW: (
            "FOLLOW VEHICLE",
            "Lead vehicle at 20 m/s, adjacent lanes blocked.",
            "Expected: FOLLOW_VEHICLE"
        ),
        Scenario.OVERTAKE: (
            "OVERTAKE",
            "Slow vehicles ahead. Pass left, then right.",
            "Expected: LANE_CHANGE_LEFT → LANE_KEEP → LANE_CHANGE_RIGHT"
        )
    }
    
    title, desc, expected = info[scenario]
    print(f"\n  Scenario: {title}")
    print(f"  {desc}")
    print(f"  {expected}\n")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='CARLA Highway Simulator with Pygame Visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python carla_simulator.py --scenario empty
  python carla_simulator.py --scenario follow --duration 45
  python carla_simulator.py --scenario overtake --duration 60

Scenarios:
  empty     - No traffic, test lane keeping
  follow    - Lead vehicle with blocked lanes
  overtake  - Multiple slow vehicles, test lane changes

Controls:
  ESC - Quit
  P   - Pause/Resume
  R   - Reset scenario

Note: CARLA server must be running. For best results, use Town04 map.
Download additional maps from CARLA releases if needed.
        """
    )
    
    parser.add_argument(
        '--scenario', '-s',
        type=str,
        default='empty',
        choices=['empty', 'follow', 'overtake'],
        help='Scenario to run (default: empty)'
    )
    
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=60.0,
        help='Simulation duration in seconds (default: 60)'
    )
    
    args = parser.parse_args()
    
    # Print info
    print_banner()
    scenario = Scenario(args.scenario)
    print_scenario_info(scenario)
    
    # Create simulation
    sim = CarlaSimulation(scenario, args.duration)
    
    # Setup signal handlers for clean shutdown
    def signal_handler(sig, frame):
        print("\n\nReceived interrupt signal, cleaning up...")
        sim.running = False
        sim.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if sim.setup():
        sim.run()
    else:
        print("\nFailed to initialize simulation.")
        print("\nTroubleshooting:")
        print("  1. Make sure CARLA server is running")
        print("  2. Try running: python reset_carla.py")
        print("  3. Or restart Docker: docker restart carla-server")
        sys.exit(1)


if __name__ == "__main__":
    main()
