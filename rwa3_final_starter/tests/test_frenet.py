"""
Unit Tests for Frenet Coordinate Transformation

Run with: python -m tests.test_frenet
Or: pytest tests/test_frenet.py -v
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.frenet_transform import (
    cartesian_to_frenet,
    frenet_to_cartesian,
    FrenetTransform,
    compute_reference_path_properties
)


def test_straight_path_on_centerline():
    """Test point exactly on a straight reference path."""
    print("\nTest: Point on straight centerline")
    
    # Straight path along x-axis
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    # Point at x=20, y=0 (on the path)
    frenet = cartesian_to_frenet(20.0, 0.0, 0.0, 10.0, reference_path)
    
    print(f"  Input: (20, 0), θ=0, v=10")
    print(f"  Output: s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    assert abs(frenet.s - 20.0) < 1.0, f"Expected s≈20, got {frenet.s}"
    assert abs(frenet.d) < 0.1, f"Expected d≈0, got {frenet.d}"
    assert abs(frenet.s_dot - 10.0) < 0.1, f"Expected s_dot≈10, got {frenet.s_dot}"
    
    print("  PASSED")


def test_straight_path_offset_left():
    """Test point offset to the left of a straight path."""
    print("\nTest: Point offset left of straight path")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    # Point at x=20, y=2 (2m left of path)
    frenet = cartesian_to_frenet(20.0, 2.0, 0.0, 10.0, reference_path)
    
    print(f"  Input: (20, 2), θ=0, v=10")
    print(f"  Output: s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    assert abs(frenet.s - 20.0) < 1.0, f"Expected s≈20, got {frenet.s}"
    assert abs(frenet.d - 2.0) < 0.2, f"Expected d≈2, got {frenet.d}"
    
    print("  PASSED")


def test_straight_path_offset_right():
    """Test point offset to the right of a straight path."""
    print("\nTest: Point offset right of straight path")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    # Point at x=20, y=-1.5 (1.5m right of path)
    frenet = cartesian_to_frenet(20.0, -1.5, 0.0, 10.0, reference_path)
    
    print(f"  Input: (20, -1.5), θ=0, v=10")
    print(f"  Output: s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    assert abs(frenet.s - 20.0) < 1.0, f"Expected s≈20, got {frenet.s}"
    assert abs(frenet.d - (-1.5)) < 0.2, f"Expected d≈-1.5, got {frenet.d}"
    
    print("  PASSED")


def test_frenet_to_cartesian_straight():
    """Test conversion from Frenet back to Cartesian on straight path."""
    print("\nTest: Frenet to Cartesian (straight path)")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    # Frenet state: s=50, d=0 (on centerline)
    cart = frenet_to_cartesian(50.0, 0.0, 10.0, 0.0, reference_path)
    
    print(f"  Input: s=50, d=0, s_dot=10, d_dot=0")
    print(f"  Output: x={cart.x:.2f}, y={cart.y:.2f}, θ={np.degrees(cart.theta):.1f}°")
    
    assert abs(cart.x - 50.0) < 1.0, f"Expected x≈50, got {cart.x}"
    assert abs(cart.y) < 0.1, f"Expected y≈0, got {cart.y}"
    
    print("  PASSED")


def test_frenet_to_cartesian_offset():
    """Test Frenet to Cartesian with lateral offset."""
    print("\nTest: Frenet to Cartesian with offset")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    # Frenet state: s=50, d=3.5 (offset left by one lane)
    cart = frenet_to_cartesian(50.0, 3.5, 10.0, 0.0, reference_path)
    
    print(f"  Input: s=50, d=3.5, s_dot=10, d_dot=0")
    print(f"  Output: x={cart.x:.2f}, y={cart.y:.2f}")
    
    assert abs(cart.x - 50.0) < 1.0, f"Expected x≈50, got {cart.x}"
    assert abs(cart.y - 3.5) < 0.5, f"Expected y≈3.5, got {cart.y}"
    
    print("  PASSED")


def test_roundtrip_straight():
    """Test roundtrip: Cartesian -> Frenet -> Cartesian on straight path."""
    print("\nTest: Roundtrip conversion (straight path)")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    original_x, original_y = 30.0, 1.5
    original_theta, original_v = 0.05, 15.0
    
    # Convert to Frenet
    frenet = cartesian_to_frenet(original_x, original_y, original_theta, 
                                  original_v, reference_path)
    
    # Convert back to Cartesian
    cart = frenet_to_cartesian(frenet.s, frenet.d, frenet.s_dot, 
                               frenet.d_dot, reference_path)
    
    print(f"  Original: ({original_x}, {original_y})")
    print(f"  Frenet: s={frenet.s:.2f}, d={frenet.d:.2f}")
    print(f"  Recovered: ({cart.x:.2f}, {cart.y:.2f})")
    
    x_error = abs(original_x - cart.x)
    y_error = abs(original_y - cart.y)
    
    print(f"  Error: x={x_error:.4f}, y={y_error:.4f}")
    
    assert x_error < 1.0, f"X error too large: {x_error}"
    assert y_error < 0.5, f"Y error too large: {y_error}"
    
    print("  PASSED")


def test_velocity_decomposition():
    """Test velocity decomposition into s_dot and d_dot."""
    print("\nTest: Velocity decomposition")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    # Vehicle moving at 45° angle
    v = 10.0
    theta = np.pi / 4  # 45 degrees
    
    frenet = cartesian_to_frenet(20.0, 0.0, theta, v, reference_path)
    
    print(f"  Input: θ=45°, v=10 m/s")
    print(f"  Output: s_dot={frenet.s_dot:.2f}, d_dot={frenet.d_dot:.2f}")
    
    # At 45°, s_dot and d_dot should be approximately equal
    expected = v / np.sqrt(2)
    
    assert abs(frenet.s_dot - expected) < 1.0, f"s_dot error: expected {expected:.2f}, got {frenet.s_dot:.2f}"
    assert abs(frenet.d_dot - expected) < 1.0, f"d_dot error: expected {expected:.2f}, got {frenet.d_dot:.2f}"
    
    print("  PASSED")


def test_frenet_transform_class():
    """Test the FrenetTransform class."""
    print("\nTest: FrenetTransform class")
    
    reference_path = [(i * 2.0, 0.0) for i in range(51)]
    
    transform = FrenetTransform(reference_path)
    
    print(f"  Total path length: {transform.total_length:.2f} m")
    
    # Test conversion methods
    frenet = transform.to_frenet(40.0, 1.0, 0.0, 20.0)
    print(f"  to_frenet(40, 1): s={frenet.s:.2f}, d={frenet.d:.2f}")
    
    cart = transform.to_cartesian(40.0, 1.0, 20.0, 0.0)
    print(f"  to_cartesian(40, 1): x={cart.x:.2f}, y={cart.y:.2f}")
    
    # Test path properties
    heading = transform.get_path_heading_at_s(50.0)
    curvature = transform.get_curvature_at_s(50.0)
    
    print(f"  Path heading at s=50: {np.degrees(heading):.1f}°")
    print(f"  Path curvature at s=50: {curvature:.4f}")
    
    assert abs(transform.total_length - 100.0) < 1.0, "Path length error"
    assert abs(heading) < 0.1, "Heading should be ~0 for straight path"
    assert abs(curvature) < 0.01, "Curvature should be ~0 for straight path"
    
    print("  PASSED")


def run_all_tests():
    """Run all unit tests."""
    print("=" * 60)
    print("FRENET TRANSFORM UNIT TESTS")
    print("=" * 60)
    
    tests = [
        test_straight_path_on_centerline,
        test_straight_path_offset_left,
        test_straight_path_offset_right,
        test_frenet_to_cartesian_straight,
        test_frenet_to_cartesian_offset,
        test_roundtrip_straight,
        test_velocity_decomposition,
        test_frenet_transform_class
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
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    exit(0 if success else 1)
