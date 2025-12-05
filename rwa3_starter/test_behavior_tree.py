"""
test_behavior_tree.py - Unit Tests for Behavior Tree Implementation

Run this file to test your implementation:
    python test_behavior_tree.py

All tests should pass when implementation is complete.
"""

import sys
from bt_framework import Status, BehaviorType, EnvironmentState, blackboard
from bt_nodes import (
    IsVehicleAhead, IsVehicleSlow, IsLaneChangeSafe,
    SetLaneKeepCommand, SetFollowCommand, SetLaneChangeCommand,
    SPEED_LIMIT, LANE_WIDTH
)
from behavior_tree import BehaviorPlanner


def run_tests():
    """Run all tests and report results."""
    
    passed = 0
    failed = 0
    
    print("=" * 60)
    print("BEHAVIOR TREE UNIT TESTS")
    print("=" * 60)
    
    # Reset environment
    env = blackboard.env_state
    
    # =========================================================================
    # Test Condition Nodes
    # =========================================================================
    
    print("\n--- Condition Node Tests ---\n")
    
    # Test IsVehicleAhead
    print("IsVehicleAhead:")
    node = IsVehicleAhead()
    
    # Test 1: No vehicle
    env.vehicle_ahead = False
    result = node.update()
    if result == Status.FAILURE:
        print("  [PASS] No vehicle ahead -> FAILURE")
        passed += 1
    else:
        print(f"  [FAIL] No vehicle ahead -> {result} (expected FAILURE)")
        failed += 1
    
    # Test 2: Vehicle far away
    env.vehicle_ahead = True
    env.vehicle_ahead_distance = 70.0  # Beyond FOLLOW_DISTANCE (50m)
    env.vehicle_ahead_speed = 20.0
    env.ego_speed = 25.0
    result = node.update()
    if result == Status.FAILURE:
        print("  [PASS] Vehicle far (70m) -> FAILURE")
        passed += 1
    else:
        print(f"  [FAIL] Vehicle far (70m) -> {result} (expected FAILURE)")
        failed += 1
    
    # Test 3: Vehicle close and blocking
    env.vehicle_ahead_distance = 20.0
    env.vehicle_ahead_speed = 20.0
    env.ego_speed = 30.0
    result = node.update()
    if result == Status.SUCCESS:
        print("  [PASS] Vehicle close and slow -> SUCCESS")
        passed += 1
    else:
        print(f"  [FAIL] Vehicle close and slow -> {result} (expected SUCCESS)")
        failed += 1
    
    # Test IsVehicleSlow
    print("\nIsVehicleSlow:")
    node = IsVehicleSlow()
    
    # Test 4: No vehicle
    env.vehicle_ahead = False
    result = node.update()
    if result == Status.FAILURE:
        print("  [PASS] No vehicle -> FAILURE")
        passed += 1
    else:
        print(f"  [FAIL] No vehicle -> {result} (expected FAILURE)")
        failed += 1
    
    # Test 5: Fast vehicle
    env.vehicle_ahead = True
    env.vehicle_ahead_speed = 29.0
    env.speed_limit = 31.0
    result = node.update()
    if result == Status.FAILURE:
        print("  [PASS] Fast vehicle (29 m/s) -> FAILURE")
        passed += 1
    else:
        print(f"  [FAIL] Fast vehicle (29 m/s) -> {result} (expected FAILURE)")
        failed += 1
    
    # Test 6: Slow vehicle
    env.vehicle_ahead_speed = 22.0
    result = node.update()
    if result == Status.SUCCESS:
        print("  [PASS] Slow vehicle (22 m/s) -> SUCCESS")
        passed += 1
    else:
        print(f"  [FAIL] Slow vehicle (22 m/s) -> {result} (expected SUCCESS)")
        failed += 1
    
    # Test IsLaneChangeSafe
    print("\nIsLaneChangeSafe:")
    node = IsLaneChangeSafe()
    
    # Test 7: Left lane clear (preferred)
    env.left_lane_exists = True
    env.left_lane_clear = True
    env.right_lane_exists = True
    env.right_lane_clear = True
    result = node.update()
    target = blackboard.get('target_lane')
    if result == Status.SUCCESS and target == 'left':
        print("  [PASS] Both clear -> SUCCESS, target=left")
        passed += 1
    else:
        print(f"  [FAIL] Both clear -> {result}, target={target} (expected SUCCESS, left)")
        failed += 1
    
    # Test 8: Only right lane clear
    env.left_lane_clear = False
    result = node.update()
    target = blackboard.get('target_lane')
    if result == Status.SUCCESS and target == 'right':
        print("  [PASS] Only right clear -> SUCCESS, target=right")
        passed += 1
    else:
        print(f"  [FAIL] Only right clear -> {result}, target={target} (expected SUCCESS, right)")
        failed += 1
    
    # Test 9: No lanes clear
    env.right_lane_clear = False
    result = node.update()
    if result == Status.FAILURE:
        print("  [PASS] No lanes clear -> FAILURE")
        passed += 1
    else:
        print(f"  [FAIL] No lanes clear -> {result} (expected FAILURE)")
        failed += 1
    
    # =========================================================================
    # Test Action Nodes
    # =========================================================================
    
    print("\n--- Action Node Tests ---\n")
    
    # Test SetLaneKeepCommand
    print("SetLaneKeepCommand:")
    node = SetLaneKeepCommand()
    result = node.update()
    cmd = blackboard.behavior_command
    if (result == Status.SUCCESS and 
        cmd.behavior == BehaviorType.LANE_KEEP and
        cmd.target_d == 0.0 and
        cmd.target_speed == SPEED_LIMIT):
        print(f"  [PASS] behavior={cmd.behavior.value}, d={cmd.target_d}, v={cmd.target_speed}")
        passed += 1
    else:
        print(f"  [FAIL] behavior={cmd.behavior.value}, d={cmd.target_d}, v={cmd.target_speed}")
        failed += 1
    
    # Test SetFollowCommand
    print("\nSetFollowCommand:")
    env.vehicle_ahead_speed = 22.0
    node = SetFollowCommand()
    result = node.update()
    cmd = blackboard.behavior_command
    expected_speed = 21.0  # 22 - 1 buffer
    if (result == Status.SUCCESS and 
        cmd.behavior == BehaviorType.FOLLOW_VEHICLE and
        abs(cmd.target_speed - expected_speed) < 0.5):
        print(f"  [PASS] behavior={cmd.behavior.value}, v={cmd.target_speed} (expected ~{expected_speed})")
        passed += 1
    else:
        print(f"  [FAIL] behavior={cmd.behavior.value}, v={cmd.target_speed} (expected ~{expected_speed})")
        failed += 1
    
    # Test SetLaneChangeCommand - Left
    print("\nSetLaneChangeCommand (left):")
    blackboard.set('target_lane', 'left')
    node = SetLaneChangeCommand()
    result = node.update()
    cmd = blackboard.behavior_command
    if (result == Status.SUCCESS and 
        cmd.behavior == BehaviorType.LANE_CHANGE_LEFT and
        cmd.target_d == LANE_WIDTH):
        print(f"  [PASS] behavior={cmd.behavior.value}, d={cmd.target_d}")
        passed += 1
    else:
        print(f"  [FAIL] behavior={cmd.behavior.value}, d={cmd.target_d}")
        failed += 1
    
    # Test SetLaneChangeCommand - Right
    print("\nSetLaneChangeCommand (right):")
    blackboard.set('target_lane', 'right')
    result = node.update()
    cmd = blackboard.behavior_command
    if (result == Status.SUCCESS and 
        cmd.behavior == BehaviorType.LANE_CHANGE_RIGHT and
        cmd.target_d == -LANE_WIDTH):
        print(f"  [PASS] behavior={cmd.behavior.value}, d={cmd.target_d}")
        passed += 1
    else:
        print(f"  [FAIL] behavior={cmd.behavior.value}, d={cmd.target_d}")
        failed += 1
    
    # =========================================================================
    # Test Behavior Planner Integration
    # =========================================================================
    
    print("\n--- Behavior Planner Integration Tests ---\n")
    
    planner = BehaviorPlanner()
    
    # Test: Empty road
    print("Empty road scenario:")
    env = EnvironmentState()
    env.vehicle_ahead = False
    cmd = planner.get_command(env)
    if cmd.behavior == BehaviorType.LANE_KEEP:
        print(f"  [PASS] Empty road -> {cmd.behavior.value}")
        passed += 1
    else:
        print(f"  [FAIL] Empty road -> {cmd.behavior.value} (expected lane_keep)")
        failed += 1
    
    # Test: Slow vehicle, can overtake
    print("\nSlow vehicle, left clear:")
    env.vehicle_ahead = True
    env.vehicle_ahead_distance = 20.0
    env.vehicle_ahead_speed = 22.0
    env.ego_speed = 30.0
    env.left_lane_clear = True
    env.right_lane_clear = True
    cmd = planner.get_command(env)
    if cmd.behavior == BehaviorType.LANE_CHANGE_LEFT:
        print(f"  [PASS] Slow vehicle, can pass -> {cmd.behavior.value}")
        passed += 1
    else:
        print(f"  [FAIL] Slow vehicle, can pass -> {cmd.behavior.value} (expected lane_change_left)")
        failed += 1
    
    # Test: Vehicle ahead, can't overtake
    print("\nVehicle ahead, lanes blocked:")
    env.left_lane_clear = False
    env.right_lane_clear = False
    cmd = planner.get_command(env)
    if cmd.behavior == BehaviorType.FOLLOW_VEHICLE:
        print(f"  [PASS] Can't pass -> {cmd.behavior.value}")
        passed += 1
    else:
        print(f"  [FAIL] Can't pass -> {cmd.behavior.value} (expected follow_vehicle)")
        failed += 1
    
    # =========================================================================
    # Summary
    # =========================================================================
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✅ All tests passed! Your implementation is correct.")
    else:
        print(f"\n❌ {failed} test(s) failed. Check your implementation.")
    
    return failed == 0


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
