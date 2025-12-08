#!/usr/bin/env python3
"""
CARLA Vehicle Control Demo with Pygame Visualization
ENPM818Z - On-Road Automated Vehicles
University of Maryland

Features:
    - Runtime controller switching (press 1-4)
    - Third-person camera view
    - Real-time telemetry HUD

Usage:
    1. Start CARLA server: ./CarlaUE4.sh
    2. Run: python carla_control_main_pygame.py
    
Keyboard Controls:
    1: Pure Pursuit
    2: Stanley
    3: LQR
    4: MPC
    Q: Toggle HUD
    ESC: Quit
"""

import argparse
import sys
import time
import math
import numpy as np

try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found.")
    print("Set PYTHONPATH: export PYTHONPATH=$PYTHONPATH:/path/to/carla/PythonAPI/carla")
    sys.exit(1)

from carla_control_utils import (
    get_kinematic_params,
    get_dynamic_params,
    get_vehicle_state,
    get_dynamic_state,
    generate_waypoints,
    compute_cross_track_error,
    get_path_heading,
    normalize_angle
)

from carla_controllers import (
    PurePursuitController,
    StanleyController,
    PIDSpeedController,
    LQRController,
    SimpleMPCController
)


def setup_carla(host='localhost', port=2000, timeout=10.0):
    """Connect to CARLA and set up synchronous mode."""
    print(f"Connecting to CARLA at {host}:{port}...")
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    
    world = client.get_world()
    original_settings = world.get_settings()
    
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.02  # 50 Hz
    world.apply_settings(settings)
    
    print(f"Connected. Map: {world.get_map().name}")
    return client, world, original_settings


def spawn_vehicle(world, vehicle_filter='model3'):
    """Spawn a vehicle at a spawn point."""
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(vehicle_filter)[0]
    spawn_points = world.get_map().get_spawn_points()
    
    if not spawn_points:
        raise RuntimeError("No spawn points found")
    
    for spawn_point in spawn_points:
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Spawned: {vehicle.type_id}")
            return vehicle
        except RuntimeError:
            continue
    
    raise RuntimeError("Could not spawn vehicle")


class ControllerManager:
    """Manages multiple controllers with runtime switching."""
    
    def __init__(self, vehicle, target_speed=10.0):
        self.vehicle = vehicle
        self.target_speed = target_speed
        
        # Get parameters
        self.kin_params = get_kinematic_params(vehicle)
        self.dyn_params = get_dynamic_params(vehicle)
        self.max_steer = self.kin_params['max_steer']
        
        print(f"Vehicle: wheelbase={self.kin_params['L']:.2f}m, "
              f"max_steer={math.degrees(self.max_steer):.1f}°")
        
        # Initialize all controllers
        self.controllers = {
            'pure_pursuit': PurePursuitController(
                wheelbase=self.kin_params['L'], k=0.5, Ld_min=4.0
            ),
            'stanley': StanleyController(
                wheelbase=self.kin_params['L'], k=2.5, k_soft=1.0
            ),
            'lqr': LQRController(
                params=self.dyn_params,
                Q=np.diag([2.0, 0.5, 2.0, 0.5]),
                R=np.array([[0.5]]),
                vx_nominal=target_speed
            ),
            'mpc': SimpleMPCController(
                wheelbase=self.kin_params['L'], horizon=10, dt=0.1
            )
        }
        
        self.names = {
            'pure_pursuit': 'Pure Pursuit',
            'stanley': 'Stanley',
            'lqr': 'LQR',
            'mpc': 'MPC'
        }
        
        self.speed_ctrl = PIDSpeedController(Kp=1.0, Ki=0.1, Kd=0.05, dt=0.02)
        self.active = 'pure_pursuit'
    
    def switch(self, controller_type):
        """Switch controller at runtime."""
        if controller_type in self.controllers and controller_type != self.active:
            self.active = controller_type
            self.speed_ctrl.reset()
            print(f"\n>>> Switched to {self.names[controller_type]} <<<\n")
    
    def get_name(self):
        return self.names[self.active]
    
    def compute_control(self, waypoints):
        """Compute control using active controller."""
        if self.active == 'lqr':
            state = get_dynamic_state(self.vehicle)
            self.controllers['lqr'].update_speed(state['vx'], threshold=2.0)
            error_state = self.controllers['lqr'].compute_error_state(state, waypoints)
            delta = self.controllers['lqr'].get_control(error_state)
            cte, he = error_state[0], error_state[2]
        else:
            state = get_vehicle_state(self.vehicle)
            cte, idx = compute_cross_track_error(state['x'], state['y'], waypoints)
            path_heading = get_path_heading(waypoints, idx)
            he = normalize_angle(state['yaw'] - path_heading)
            delta = self.controllers[self.active].get_control(state, waypoints)
        
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        v = state.get('v', state.get('vx', 0))
        throttle, brake = self.speed_ctrl.get_control(v, self.target_speed)
        
        return delta, throttle, brake, cte, he, state


def run_control_loop(vehicle, world, waypoints, initial_controller='pure_pursuit',
                     target_speed=10.0, duration=300.0, use_display=True):
    """Main control loop with runtime switching."""
    
    ctrl_mgr = ControllerManager(vehicle, target_speed)
    ctrl_mgr.switch(initial_controller)
    
    pygame_display = None
    if use_display:
        from carla_pygame_display import create_display
        pygame_display = create_display(
            width=1280, height=720,
            title="CARLA Control - Press 1-4 to switch controllers"
        )
        pygame_display.setup_camera(world, vehicle)
    
    print(f"\nTarget speed: {target_speed:.1f} m/s ({target_speed*3.6:.1f} km/h)")
    print("=" * 55)
    print("RUNTIME CONTROLLER SWITCHING:")
    print("  1 = Pure Pursuit    2 = Stanley")
    print("  3 = LQR             4 = MPC")
    print("  Q = Toggle HUD      ESC = Quit")
    print("=" * 55 + "\n")
    
    start_time = time.time()
    frame = 0
    
    try:
        while time.time() - start_time < duration:
            if pygame_display is not None:
                if not pygame_display.process_events():
                    break
                
                switch = pygame_display.get_controller_switch()
                if switch:
                    ctrl_mgr.switch(switch)
            
            world.tick()
            
            delta, throttle, brake, cte, he, state = ctrl_mgr.compute_control(waypoints)
            
            control = carla.VehicleControl()
            control.steer = delta / ctrl_mgr.max_steer
            control.throttle = throttle
            control.brake = brake
            vehicle.apply_control(control)
            
            if pygame_display is not None:
                v = state.get('v', state.get('vx', 0))
                pygame_display.update_hud(
                    speed=v, target_speed=target_speed,
                    steering=delta, throttle=throttle, brake=brake,
                    cross_track_error=cte, heading_error=he,
                    controller_name=ctrl_mgr.get_name()
                )
                pygame_display.render()
            
            if frame % 100 == 0:
                v = state.get('v', state.get('vx', 0))
                print(f"[{ctrl_mgr.get_name():12s}] "
                      f"Speed: {v:5.1f} m/s | "
                      f"Steer: {math.degrees(delta):6.1f}° | "
                      f"CTE: {cte:6.2f} m")
            
            frame += 1
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if pygame_display is not None:
            pygame_display.destroy()


def main():
    parser = argparse.ArgumentParser(
        description='CARLA Vehicle Control with Runtime Switching'
    )
    parser.add_argument('--controller', type=str, default='pure_pursuit',
                        choices=['pure_pursuit', 'stanley', 'lqr', 'mpc'],
                        help='Initial controller (default: pure_pursuit)')
    parser.add_argument('--speed', type=float, default=10.0,
                        help='Target speed in m/s (default: 10.0)')
    parser.add_argument('--duration', type=float, default=300.0,
                        help='Max duration in seconds (default: 300)')
    parser.add_argument('--host', type=str, default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--map', type=str, default=None,
                        help='CARLA map (e.g., Town04)')
    parser.add_argument('--no-display', action='store_true',
                        help='Run without pygame')
    
    args = parser.parse_args()
    
    client = None
    vehicle = None
    original_settings = None
    
    try:
        client, world, original_settings = setup_carla(args.host, args.port)
        
        if args.map and args.map != world.get_map().name:
            print(f"Loading map: {args.map}")
            world = client.load_world(args.map)
            time.sleep(2.0)
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.02
            world.apply_settings(settings)
        
        vehicle = spawn_vehicle(world)
        
        # Let vehicle settle
        for _ in range(20):
            world.tick()
        
        print("Generating waypoints...")
        waypoints = generate_waypoints(world, vehicle, num_waypoints=200, spacing=2.0)
        print(f"Generated {len(waypoints)} waypoints")
        
        run_control_loop(
            vehicle, world, waypoints,
            initial_controller=args.controller,
            target_speed=args.speed,
            duration=args.duration,
            use_display=not args.no_display
        )
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\nCleaning up...")
        if vehicle is not None:
            vehicle.destroy()
        if original_settings is not None and client is not None:
            world = client.get_world()
            world.apply_settings(original_settings)
        print("Done")


if __name__ == '__main__':
    main()
