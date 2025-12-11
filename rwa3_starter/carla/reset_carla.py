#!/usr/bin/env python3
"""
reset_carla.py - Reset CARLA to a clean state

Run this script if CARLA becomes unresponsive or crashes after a previous run.
It disables synchronous mode and destroys all spawned actors.

Usage:
    python reset_carla.py
"""

import sys

try:
    import carla
except ImportError:
    print("ERROR: CARLA Python package not found!")
    print("Install with: pip install carla")
    sys.exit(1)


def reset_carla(host='localhost', port=2000):
    """Reset CARLA to a clean state."""
    print("=" * 50)
    print("  CARLA Reset Utility")
    print("=" * 50)
    
    try:
        print(f"\nConnecting to CARLA at {host}:{port}...")
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        
        version = client.get_server_version()
        print(f"✓ Connected to CARLA {version}")
        
        world = client.get_world()
        
        # 1. Disable synchronous mode
        print("\nDisabling synchronous mode...")
        settings = world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            print("✓ Synchronous mode disabled")
        else:
            print("✓ Synchronous mode was already disabled")
        
        # 2. Reset traffic manager
        print("\nResetting traffic manager...")
        try:
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(False)
            print("✓ Traffic manager reset")
        except Exception as e:
            print(f"⚠ Could not reset traffic manager: {e}")
        
        # 3. Destroy all vehicles
        print("\nDestroying all vehicles...")
        actors = world.get_actors()
        vehicles = actors.filter('vehicle.*')
        
        destroyed = 0
        for vehicle in vehicles:
            try:
                vehicle.destroy()
                destroyed += 1
            except:
                pass
        
        print(f"✓ Destroyed {destroyed} vehicles")
        
        # 4. Destroy all sensors
        print("\nDestroying all sensors...")
        sensors = actors.filter('sensor.*')
        
        destroyed = 0
        for sensor in sensors:
            try:
                sensor.stop()
                sensor.destroy()
                destroyed += 1
            except:
                pass
        
        print(f"✓ Destroyed {destroyed} sensors")
        
        # 5. Tick world to apply changes
        print("\nApplying changes...")
        world.tick()
        
        print("\n" + "=" * 50)
        print("  ✓ CARLA reset complete!")
        print("  You can now run your simulation again.")
        print("=" * 50)
        
        return True
        
    except Exception as e:
        print(f"\n✗ Failed to reset CARLA: {e}")
        print("\nTroubleshooting:")
        print("  1. Make sure CARLA server is running")
        print("  2. Try restarting the CARLA Docker container:")
        print("     docker stop carla-server")
        print("     docker start -ai carla-server")
        return False


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Reset CARLA to a clean state')
    parser.add_argument('--host', default='localhost', help='CARLA host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port')
    
    args = parser.parse_args()
    
    success = reset_carla(args.host, args.port)
    sys.exit(0 if success else 1)
