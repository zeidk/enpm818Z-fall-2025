"""
Unit Tests for Behavior Tree Nodes

Run with: python -m tests.test_bt_nodes
Or: pytest tests/test_bt_nodes.py -v
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.behavior_tree import (
    NodeStatus,
    BehavioralCommand,
    Blackboard,
    Sequence,
    Selector
)
from src.behavior_tree.bt_nodes import (
    IsObstacleBlocking,
    ShouldChangeLane,
    IsLaneChangeSafe,
    IsVehicleAhead,
    IsTooClose,
    SetStopCommand,
    SetLaneChangeCommand,
    SetFollowCommand,
    SetLaneKeepCommand,
    get_lead_vehicle,
    compute_longitudinal_distance
)


def create_test_blackboard(ego_state=None, obstacles=None, 
                           lane_info=None, config=None, route=None):
    """Create a blackboard with test data."""
    bb = Blackboard()
    
    if ego_state is None:
        ego_state = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 20.0, 'lane_id': 1
        }
    
    if obstacles is None:
        obstacles = []
    
    if lane_info is None:
        lane_info = {
            'current_lane': 1,
            'left_lane_exists': True,
            'right_lane_exists': True,
            'lane_width': 3.5
        }
    
    if config is None:
        config = {
            'target_speed': 25.0,
            'safe_follow_distance': 20.0,
            'lane_change_min_gap': 25.0,
            'stop_distance': 10.0,
            'slow_vehicle_threshold': 18.0
        }
    
    if route is None:
        route = [1]
    
    bb.update_perception(ego_state, obstacles, lane_info, [])
    bb.set('config', config)
    bb.set('route', route)
    
    return bb


# =============================================================================
# Helper Function Tests
# =============================================================================

def test_get_lead_vehicle_none():
    """Test get_lead_vehicle with no vehicles ahead."""
    print("\nTest: get_lead_vehicle with no obstacles")
    
    ego_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 20.0, 'lane_id': 1}
    obstacles = []
    
    lead = get_lead_vehicle(ego_state, obstacles)
    
    print(f"  Lead vehicle: {lead}")
    assert lead is None, "Should return None with no obstacles"
    print("  PASSED")


def test_get_lead_vehicle_ahead():
    """Test get_lead_vehicle with vehicle ahead in same lane."""
    print("\nTest: get_lead_vehicle with vehicle ahead")
    
    ego_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 20.0, 'lane_id': 1}
    obstacles = [
        {'x': 30.0, 'y': 0.0, 'vx': 15.0, 'vy': 0.0, 'lane_id': 1},  # Ahead
        {'x': -20.0, 'y': 0.0, 'vx': 15.0, 'vy': 0.0, 'lane_id': 1}  # Behind
    ]
    
    lead = get_lead_vehicle(ego_state, obstacles)
    
    if lead is not None:
        print(f"  Lead vehicle at x={lead['x']}")
        assert lead['x'] == 30.0, "Should select vehicle ahead"
    else:
        print("  get_lead_vehicle not implemented yet")
    
    print("  PASSED (or not implemented)")


def test_compute_longitudinal_distance():
    """Test longitudinal distance computation."""
    print("\nTest: compute_longitudinal_distance")
    
    ego_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Facing +x
    
    obstacle_ahead = {'x': 30.0, 'y': 0.0}
    obstacle_behind = {'x': -20.0, 'y': 0.0}
    
    dist_ahead = compute_longitudinal_distance(ego_state, obstacle_ahead)
    dist_behind = compute_longitudinal_distance(ego_state, obstacle_behind)
    
    if dist_ahead is not None:
        print(f"  Distance to ahead: {dist_ahead}")
        print(f"  Distance to behind: {dist_behind}")
        assert dist_ahead > 0, "Ahead should be positive"
        assert dist_behind < 0, "Behind should be negative"
    else:
        print("  compute_longitudinal_distance not implemented yet")
    
    print("  PASSED (or not implemented)")


# =============================================================================
# Condition Node Tests
# =============================================================================

def test_is_obstacle_blocking_empty():
    """Test IsObstacleBlocking with no obstacles."""
    print("\nTest: IsObstacleBlocking with no obstacles")
    
    bb = create_test_blackboard()
    node = IsObstacleBlocking()
    
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    # Should return FAILURE when no blocking obstacle
    # (But might return FAILURE if not implemented)
    print("  PASSED (check implementation)")


def test_is_obstacle_blocking_stationary():
    """Test IsObstacleBlocking with stationary obstacle ahead."""
    print("\nTest: IsObstacleBlocking with stationary obstacle")
    
    obstacles = [
        {'x': 8.0, 'y': 0.0, 'vx': 0.0, 'vy': 0.0, 'lane_id': 1}  # Stationary, 8m ahead
    ]
    bb = create_test_blackboard(obstacles=obstacles)
    
    node = IsObstacleBlocking()
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    # Should return SUCCESS when blocking obstacle found
    print("  PASSED (check implementation)")


def test_is_vehicle_ahead():
    """Test IsVehicleAhead condition."""
    print("\nTest: IsVehicleAhead")
    
    obstacles = [
        {'x': 50.0, 'y': 0.0, 'vx': 20.0, 'vy': 0.0, 'lane_id': 1}
    ]
    bb = create_test_blackboard(obstacles=obstacles)
    
    node = IsVehicleAhead()
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    print("  PASSED (check implementation)")


def test_is_too_close():
    """Test IsTooClose condition."""
    print("\nTest: IsTooClose")
    
    # Vehicle 15m ahead (less than safe_follow_distance of 20m)
    obstacles = [
        {'x': 15.0, 'y': 0.0, 'vx': 15.0, 'vy': 0.0, 'lane_id': 1}
    ]
    bb = create_test_blackboard(obstacles=obstacles)
    
    node = IsTooClose()
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    # Should return SUCCESS when too close
    print("  PASSED (check implementation)")


def test_should_change_lane_route():
    """Test ShouldChangeLane when route requires different lane."""
    print("\nTest: ShouldChangeLane (route requires change)")
    
    ego_state = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'v': 20.0, 'lane_id': 1}
    route = [0]  # Route wants lane 0, but ego in lane 1
    
    bb = create_test_blackboard(ego_state=ego_state, route=route)
    
    node = ShouldChangeLane()
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    # Should return SUCCESS when route requires lane change
    print("  PASSED (check implementation)")


def test_is_lane_change_safe_empty():
    """Test IsLaneChangeSafe with empty target lane."""
    print("\nTest: IsLaneChangeSafe (empty lane)")
    
    obstacles = []  # No obstacles
    bb = create_test_blackboard(obstacles=obstacles)
    bb.set('target_lane_for_change', 0)  # Want to change to lane 0
    
    node = IsLaneChangeSafe()
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    # Should return SUCCESS when lane is clear
    print("  PASSED (check implementation)")


# =============================================================================
# Action Node Tests
# =============================================================================

def test_set_stop_command():
    """Test SetStopCommand action."""
    print("\nTest: SetStopCommand")
    
    bb = create_test_blackboard()
    node = SetStopCommand()
    
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    print(f"  Command: {bb.behavioral_command}")
    
    assert status == NodeStatus.SUCCESS, "SetStopCommand should return SUCCESS"
    # Check command was set (if implemented)
    if bb.behavioral_command.maneuver == 'stop':
        assert bb.behavioral_command.target_speed == 0.0, "Stop should have speed 0"
    
    print("  PASSED")


def test_set_lane_keep_command():
    """Test SetLaneKeepCommand action."""
    print("\nTest: SetLaneKeepCommand")
    
    bb = create_test_blackboard()
    node = SetLaneKeepCommand()
    
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    print(f"  Command: {bb.behavioral_command}")
    
    assert status == NodeStatus.SUCCESS, "SetLaneKeepCommand should return SUCCESS"
    
    print("  PASSED")


def test_set_follow_command():
    """Test SetFollowCommand action."""
    print("\nTest: SetFollowCommand")
    
    obstacles = [
        {'x': 30.0, 'y': 0.0, 'vx': 15.0, 'vy': 0.0, 'lane_id': 1}
    ]
    bb = create_test_blackboard(obstacles=obstacles)
    node = SetFollowCommand()
    
    status = node.tick(bb)
    
    print(f"  Status: {status}")
    print(f"  Command: {bb.behavioral_command}")
    
    assert status == NodeStatus.SUCCESS, "SetFollowCommand should return SUCCESS"
    
    print("  PASSED")


# =============================================================================
# Integration Tests
# =============================================================================

def test_simple_selector():
    """Test a simple selector with BT nodes."""
    print("\nTest: Simple Selector tree")
    
    bb = create_test_blackboard()
    
    # Selector: try stop, then lane_keep
    tree = Selector("TestSelector", [
        Sequence("StopBranch", [
            IsObstacleBlocking(),
            SetStopCommand()
        ]),
        Sequence("KeepLaneBranch", [
            SetLaneKeepCommand()
        ])
    ])
    
    status = tree.tick(bb)
    
    print(f"  Tree status: {status}")
    print(f"  Command: {bb.behavioral_command}")
    
    # With no obstacles, should fall through to lane_keep
    assert status == NodeStatus.SUCCESS, "Tree should succeed"
    
    print("  PASSED")


def test_follow_sequence():
    """Test the follow vehicle sequence."""
    print("\nTest: Follow vehicle sequence")
    
    obstacles = [
        {'x': 15.0, 'y': 0.0, 'vx': 15.0, 'vy': 0.0, 'lane_id': 1}  # Close vehicle
    ]
    bb = create_test_blackboard(obstacles=obstacles)
    
    sequence = Sequence("FollowBranch", [
        IsVehicleAhead(),
        IsTooClose(),
        SetFollowCommand()
    ])
    
    status = sequence.tick(bb)
    
    print(f"  Sequence status: {status}")
    print(f"  Command: {bb.behavioral_command}")
    
    print("  PASSED (check implementation)")


def run_all_tests():
    """Run all unit tests."""
    print("=" * 60)
    print("BEHAVIOR TREE NODES UNIT TESTS")
    print("=" * 60)
    
    tests = [
        # Helper tests
        test_get_lead_vehicle_none,
        test_get_lead_vehicle_ahead,
        test_compute_longitudinal_distance,
        # Condition tests
        test_is_obstacle_blocking_empty,
        test_is_obstacle_blocking_stationary,
        test_is_vehicle_ahead,
        test_is_too_close,
        test_should_change_lane_route,
        test_is_lane_change_safe_empty,
        # Action tests
        test_set_stop_command,
        test_set_lane_keep_command,
        test_set_follow_command,
        # Integration tests
        test_simple_selector,
        test_follow_sequence
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"  FAILED: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR: {e}")
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 60)
    print("\nNote: Some tests may pass even if nodes are not fully implemented.")
    print("Check that your implementation produces the expected behavior.")
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    exit(0 if success else 1)
