"""
test_frenet.py - Unit Tests for Frenet Coordinate Transformation

Run: python test_frenet.py
"""

import sys
import numpy as np
from frenet import create_reference_path, cartesian_to_frenet, frenet_to_cartesian


def run_tests():
    passed = 0
    failed = 0
    
    print("=" * 60)
    print("FRENET COORDINATE TRANSFORMATION TESTS")
    print("=" * 60)
    
    # Create straight path
    waypoints = [(i, 0) for i in range(0, 101, 10)]
    path = create_reference_path(waypoints)
    
    print(f"\nTest path: straight line, length={path.total_length}m\n")
    
    # Test 1: Point on centerline
    print("Test 1: Point on centerline (50, 0)")
    x, y = 50.0, 0.0
    s, d = cartesian_to_frenet(x, y, path)
    if s is not None and abs(s - 50.0) < 1.0 and abs(d) < 0.1:
        print(f"  [PASS] s={s:.1f}, d={d:.2f}")
        passed += 1
    else:
        print(f"  [FAIL] s={s}, d={d} (expected s≈50, d≈0)")
        failed += 1
    
    # Test 2: Point left of centerline
    print("\nTest 2: Point left of centerline (50, 3.5)")
    x, y = 50.0, 3.5
    s, d = cartesian_to_frenet(x, y, path)
    if s is not None and abs(s - 50.0) < 1.0 and abs(d - 3.5) < 0.1:
        print(f"  [PASS] s={s:.1f}, d={d:.2f}")
        passed += 1
    else:
        print(f"  [FAIL] s={s}, d={d} (expected s≈50, d≈3.5)")
        failed += 1
    
    # Test 3: Point right of centerline
    print("\nTest 3: Point right of centerline (50, -3.5)")
    x, y = 50.0, -3.5
    s, d = cartesian_to_frenet(x, y, path)
    if s is not None and abs(s - 50.0) < 1.0 and abs(d - (-3.5)) < 0.1:
        print(f"  [PASS] s={s:.1f}, d={d:.2f}")
        passed += 1
    else:
        print(f"  [FAIL] s={s}, d={d} (expected s≈50, d≈-3.5)")
        failed += 1
    
    # Test 4: Frenet to Cartesian
    print("\nTest 4: Frenet to Cartesian (s=50, d=0)")
    s, d = 50.0, 0.0
    x, y = frenet_to_cartesian(s, d, path)
    if x is not None and abs(x - 50.0) < 1.0 and abs(y) < 0.1:
        print(f"  [PASS] x={x:.1f}, y={y:.2f}")
        passed += 1
    else:
        print(f"  [FAIL] x={x}, y={y} (expected x≈50, y≈0)")
        failed += 1
    
    # Test 5: Frenet to Cartesian with offset
    print("\nTest 5: Frenet to Cartesian (s=50, d=3.5)")
    s, d = 50.0, 3.5
    x, y = frenet_to_cartesian(s, d, path)
    if x is not None and abs(x - 50.0) < 1.0 and abs(y - 3.5) < 0.1:
        print(f"  [PASS] x={x:.1f}, y={y:.2f}")
        passed += 1
    else:
        print(f"  [FAIL] x={x}, y={y} (expected x≈50, y≈3.5)")
        failed += 1
    
    # Test 6: Roundtrip accuracy
    print("\nTest 6: Roundtrip accuracy")
    test_points = [(25.0, 1.5), (50.0, -2.0), (75.0, 3.0)]
    max_error = 0.0
    
    for x_orig, y_orig in test_points:
        s, d = cartesian_to_frenet(x_orig, y_orig, path)
        if s is None:
            print("  [FAIL] cartesian_to_frenet returned None")
            failed += 1
            continue
        x_back, y_back = frenet_to_cartesian(s, d, path)
        if x_back is None:
            print("  [FAIL] frenet_to_cartesian returned None")
            failed += 1
            continue
        error = np.sqrt((x_orig - x_back)**2 + (y_orig - y_back)**2)
        max_error = max(max_error, error)
        print(f"  ({x_orig}, {y_orig}) -> (s={s:.1f}, d={d:.2f}) -> ({x_back:.1f}, {y_back:.1f}), error={error:.3f}m")
    
    if max_error < 0.5:
        print(f"  [PASS] Max error: {max_error:.3f}m < 0.5m")
        passed += 1
    else:
        print(f"  [FAIL] Max error: {max_error:.3f}m >= 0.5m")
        failed += 1
    
    # Summary
    print("\n" + "=" * 60)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✅ All Frenet tests passed!")
    else:
        print(f"\n❌ {failed} test(s) failed.")
    
    return failed == 0


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
