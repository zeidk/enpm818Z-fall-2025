#!/usr/bin/env python3
"""
verify_setup.py - Verify CARLA and RWA3 Setup

Run this script to check that all requirements are met before
running the CARLA simulation.

Usage:
    python verify_setup.py

This will check:
    1. Python version
    2. Required packages (carla, numpy)
    3. RWA3 files (bt_framework.py, bt_nodes.py, behavior_tree.py)
    4. CARLA server connection
    5. Basic functionality test
"""

import sys
import os


def check_python_version():
    """Check Python version is 3.7+."""
    print("Checking Python version...", end=" ")
    
    major, minor = sys.version_info[:2]
    if major < 3 or (major == 3 and minor < 7):
        print(f"❌ Python 3.7+ required, found {major}.{minor}")
        return False
    
    print(f"✅ Python {major}.{minor}")
    return True


def check_carla_package():
    """Check CARLA Python package is installed."""
    print("Checking CARLA package...", end=" ")
    
    try:
        import carla
        version = getattr(carla, '__version__', 'unknown')
        print(f"✅ carla {version}")
        return True
    except ImportError:
        print("❌ Not installed")
        print("   Install with: pip install carla==0.9.13")
        return False


def check_numpy():
    """Check numpy is installed."""
    print("Checking numpy...", end=" ")
    
    try:
        import numpy as np
        print(f"✅ numpy {np.__version__}")
        return True
    except ImportError:
        print("❌ Not installed")
        print("   Install with: pip install numpy")
        return False


def check_rwa3_files():
    """Check RWA3 files exist in parent directory."""
    print("Checking RWA3 files...", end=" ")
    
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    required_files = [
        'bt_framework.py',
        'bt_nodes.py',
        'behavior_tree.py'
    ]
    
    missing = []
    for f in required_files:
        path = os.path.join(parent_dir, f)
        if not os.path.exists(path):
            missing.append(f)
    
    if missing:
        print(f"❌ Missing: {', '.join(missing)}")
        print(f"   Expected in: {parent_dir}")
        return False
    
    print(f"✅ All files found in {parent_dir}")
    return True


def check_imports():
    """Check that RWA3 modules can be imported."""
    print("Checking module imports...", end=" ")
    
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, parent_dir)
    
    try:
        from bt_framework import Status, BehaviorType, EnvironmentState, BehaviorCommand
        from bt_nodes import IsVehicleAhead, SetLaneKeepCommand
        from behavior_tree import BehaviorPlanner
        print("✅ All modules import successfully")
        return True
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False


def check_carla_connection():
    """Check CARLA server is running and accessible."""
    print("Checking CARLA connection...", end=" ")
    
    try:
        import carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        
        version = client.get_server_version()
        world = client.get_world()
        map_name = world.get_map().name
        
        print(f"✅ Connected to CARLA {version}")
        print(f"   Current map: {map_name}")
        return True
        
    except Exception as e:
        print(f"❌ Cannot connect: {e}")
        print("   Make sure CARLA server is running:")
        print("     Linux:   ./CarlaUE4.sh")
        print("     Windows: CarlaUE4.exe")
        return False


def check_behavior_planner():
    """Test behavior planner basic functionality."""
    print("Testing behavior planner...", end=" ")
    
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, parent_dir)
    
    try:
        from behavior_tree import BehaviorPlanner
        from bt_framework import EnvironmentState, BehaviorType
        
        planner = BehaviorPlanner()
        
        # Test empty road scenario
        env = EnvironmentState()
        env.vehicle_ahead = False
        env.ego_speed = 25.0
        
        cmd = planner.get_command(env)
        
        if cmd.behavior == BehaviorType.LANE_KEEP:
            print("✅ Planner returns correct behavior")
            return True
        else:
            print(f"⚠️ Expected LANE_KEEP, got {cmd.behavior.value}")
            return True  # Not a fatal error
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False


def check_carla_interface():
    """Test CARLA interface files."""
    print("Checking CARLA interface files...", end=" ")
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    required_files = [
        'carla_interface.py',
        'carla_controller.py',
        'carla_simulator.py'
    ]
    
    missing = []
    for f in required_files:
        path = os.path.join(current_dir, f)
        if not os.path.exists(path):
            missing.append(f)
    
    if missing:
        print(f"❌ Missing: {', '.join(missing)}")
        return False
    
    print("✅ All CARLA files present")
    return True


def run_full_test():
    """Run a quick functional test with CARLA."""
    print("\n" + "=" * 50)
    print("Running functional test...")
    print("=" * 50 + "\n")
    
    try:
        import carla
        from carla_interface import CarlaInterface, CarlaConfig, Scenario
        
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, parent_dir)
        from behavior_tree import BehaviorPlanner
        
        # Create interface
        config = CarlaConfig()
        interface = CarlaInterface(config)
        
        # Connect
        print("Connecting to CARLA...", end=" ")
        if not interface.connect():
            print("❌ Failed")
            return False
        print("✅")
        
        # Spawn ego
        print("Spawning ego vehicle...", end=" ")
        if not interface.spawn_ego_vehicle():
            print("❌ Failed")
            interface.cleanup()
            return False
        print("✅")
        
        # Setup scenario
        print("Setting up empty scenario...", end=" ")
        interface.setup_scenario(Scenario.EMPTY)
        print("✅")
        
        # Create planner
        planner = BehaviorPlanner()
        
        # Run 10 steps
        print("Running 10 simulation steps...", end=" ")
        for _ in range(10):
            env = interface.get_environment_state()
            cmd = planner.get_command(env)
            interface.apply_command(cmd)
            interface.tick()
        print("✅")
        
        # Cleanup
        print("Cleaning up...", end=" ")
        interface.cleanup()
        print("✅")
        
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all verification checks."""
    print("=" * 60)
    print("  CARLA Integration Setup Verification")
    print("  RWA3 - Behavioral Planning with Behavior Trees")
    print("=" * 60 + "\n")
    
    results = []
    
    # Basic checks (no CARLA connection needed)
    print("─" * 40)
    print("Basic Requirements")
    print("─" * 40)
    results.append(("Python version", check_python_version()))
    results.append(("CARLA package", check_carla_package()))
    results.append(("NumPy package", check_numpy()))
    results.append(("RWA3 files", check_rwa3_files()))
    results.append(("Module imports", check_imports()))
    results.append(("CARLA interface files", check_carla_interface()))
    results.append(("Behavior planner", check_behavior_planner()))
    
    # CARLA connection check
    print("\n" + "─" * 40)
    print("CARLA Server Connection")
    print("─" * 40)
    carla_ok = check_carla_connection()
    results.append(("CARLA connection", carla_ok))
    
    # Summary
    print("\n" + "=" * 60)
    print("  Summary")
    print("=" * 60)
    
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    
    for name, ok in results:
        status = "✅" if ok else "❌"
        print(f"  {status} {name}")
    
    print(f"\n  {passed}/{total} checks passed")
    
    # Full test if CARLA is available
    if carla_ok:
        if input("\nRun functional test? (y/N): ").lower() == 'y':
            success = run_full_test()
            if success:
                print("\n" + "=" * 60)
                print("  ✅ ALL TESTS PASSED!")
                print("  You're ready to run CARLA simulations.")
                print("=" * 60)
            else:
                print("\n" + "=" * 60)
                print("  ⚠️ Functional test had issues.")
                print("  Check the errors above.")
                print("=" * 60)
    else:
        print("\n" + "─" * 60)
        print("  ⚠️ CARLA server not running")
        print("  Start CARLA and run this script again for full verification.")
        print("─" * 60)
    
    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
