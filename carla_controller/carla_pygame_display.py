"""
CARLA Pygame Visualization
ENPM818Z - On-Road Automated Vehicles
University of Maryland

Pygame-based visualization for CARLA vehicle control.
Provides a third-person camera view that follows the vehicle.
"""

import pygame
import numpy as np
import weakref

pygame.init()


class CameraManager:
    """Manages a camera sensor attached to the vehicle."""
    
    def __init__(self, world, vehicle, display_width=1280, display_height=720):
        self.world = world
        self.vehicle = vehicle
        self.display_width = display_width
        self.display_height = display_height
        
        self.surface = None
        self.camera = None
        
        self._spawn_camera()
    
    def _spawn_camera(self):
        """Spawn RGB camera attached to vehicle."""
        import carla
        
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        
        camera_bp.set_attribute('image_size_x', str(self.display_width))
        camera_bp.set_attribute('image_size_y', str(self.display_height))
        camera_bp.set_attribute('fov', '90')
        
        # Third-person view: 8m behind, 4m above, looking slightly down
        camera_transform = carla.Transform(
            carla.Location(x=-8.0, z=4.0),
            carla.Rotation(pitch=-15.0)
        )
        
        self.camera = self.world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=self.vehicle
        )
        
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: CameraManager._on_image(weak_self, image))
    
    @staticmethod
    def _on_image(weak_self, image):
        """Callback when camera captures an image."""
        self = weak_self()
        if self is None:
            return
        
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA
        array = array[:, :, :3]  # Remove alpha
        array = array[:, :, ::-1]  # BGR to RGB
        
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def render(self, display):
        """Render camera view to pygame display."""
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
    
    def destroy(self):
        """Clean up camera sensor."""
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None


class HUD:
    """Heads-up display showing vehicle telemetry."""
    
    def __init__(self, width, height):
        self.width = width
        self.height = height
        
        self.font = pygame.font.Font(pygame.font.get_default_font(), 20)
        self.font_large = pygame.font.Font(pygame.font.get_default_font(), 28)
        
        self.speed = 0.0
        self.target_speed = 0.0
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self.controller_name = "None"
        
    def update(self, speed, target_speed, steering, throttle, brake, 
               cross_track_error=0.0, heading_error=0.0, controller_name=""):
        self.speed = speed
        self.target_speed = target_speed
        self.steering = steering
        self.throttle = throttle
        self.brake = brake
        self.cross_track_error = cross_track_error
        self.heading_error = heading_error
        self.controller_name = controller_name
    
    def render(self, display):
        # Semi-transparent background
        hud_surface = pygame.Surface((300, 220))
        hud_surface.set_alpha(180)
        hud_surface.fill((0, 0, 0))
        display.blit(hud_surface, (10, 10))
        
        # Title
        title = self.font_large.render(f"Controller: {self.controller_name}", True, (255, 255, 0))
        display.blit(title, (20, 15))
        
        # Speed
        speed_kmh = self.speed * 3.6
        target_kmh = self.target_speed * 3.6
        speed_text = self.font.render(f"Speed: {speed_kmh:.1f} / {target_kmh:.1f} km/h", True, (255, 255, 255))
        display.blit(speed_text, (20, 55))
        
        # Controls
        steer_deg = np.degrees(self.steering)
        steer_text = self.font.render(f"Steering: {steer_deg:.1f}°", True, (255, 255, 255))
        display.blit(steer_text, (20, 85))
        
        throttle_text = self.font.render(f"Throttle: {self.throttle:.2f}", True, (100, 255, 100))
        display.blit(throttle_text, (20, 115))
        
        brake_text = self.font.render(f"Brake: {self.brake:.2f}", True, (255, 100, 100))
        display.blit(brake_text, (20, 145))
        
        # Errors
        cte_text = self.font.render(f"Cross-track: {self.cross_track_error:.2f} m", True, (200, 200, 255))
        display.blit(cte_text, (20, 175))
        
        heading_deg = np.degrees(self.heading_error)
        heading_text = self.font.render(f"Heading err: {heading_deg:.1f}°", True, (200, 200, 255))
        display.blit(heading_text, (20, 205))
        
        # Instructions (bottom of screen)
        instructions = self.font.render("ESC: Quit | Q: HUD | 1: PP | 2: Stanley | 3: LQR | 4: MPC", True, (200, 200, 200))
        display.blit(instructions, (self.width // 2 - 260, self.height - 30))


class PygameDisplay:
    """Main pygame display manager for CARLA visualization."""
    
    def __init__(self, width=1280, height=720, title="CARLA Vehicle Control"):
        self.width = width
        self.height = height
        
        self.display = pygame.display.set_mode((width, height))
        pygame.display.set_caption(title)
        
        self.clock = pygame.time.Clock()
        self.hud = HUD(width, height)
        self.show_hud = True
        self.camera_manager = None
        self.running = True
        self.controller_switch = None
    
    def setup_camera(self, world, vehicle):
        """Set up camera to follow vehicle."""
        self.camera_manager = CameraManager(world, vehicle, self.width, self.height)
    
    def process_events(self):
        """Process pygame events."""
        self.controller_switch = None
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_q:
                    self.show_hud = not self.show_hud
                # Controller switching
                elif event.key == pygame.K_1:
                    self.controller_switch = 'pure_pursuit'
                elif event.key == pygame.K_2:
                    self.controller_switch = 'stanley'
                elif event.key == pygame.K_3:
                    self.controller_switch = 'lqr'
                elif event.key == pygame.K_4:
                    self.controller_switch = 'mpc'
        
        return self.running
    
    def get_controller_switch(self):
        """Check if user requested a controller switch."""
        return self.controller_switch
    
    def update_hud(self, speed, target_speed, steering, throttle, brake,
                   cross_track_error=0.0, heading_error=0.0, controller_name=""):
        self.hud.update(speed, target_speed, steering, throttle, brake,
                       cross_track_error, heading_error, controller_name)
    
    def render(self):
        """Render the display."""
        self.display.fill((0, 0, 0))
        
        if self.camera_manager is not None:
            self.camera_manager.render(self.display)
        
        if self.show_hud:
            self.hud.render(self.display)
        
        pygame.display.flip()
        self.clock.tick(60)
    
    def destroy(self):
        """Clean up resources."""
        if self.camera_manager is not None:
            self.camera_manager.destroy()
        pygame.quit()


def create_display(width=1280, height=720, title="CARLA Vehicle Control"):
    """Create and return a pygame display manager."""
    return PygameDisplay(width, height, title)
