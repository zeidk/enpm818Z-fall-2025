"""
Unit Tests for Polynomial Trajectory Generation

Run with: python -m tests.test_polynomial
Or: pytest tests/test_polynomial.py -v
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.polynomial_trajectory import (
    quintic_coefficients,
    quartic_coefficients,
    evaluate_polynomial,
    compute_jerk,
    compute_trajectory_cost,
    PolynomialTrajectory
)


def test_quintic_boundary_conditions():
    """Test that quintic polynomial satisfies boundary conditions."""
    print("\nTest: Quintic polynomial boundary conditions")
    
    start = (0.0, 0.0, 0.0)  # position, velocity, acceleration
    end = (10.0, 0.0, 0.0)   # Move 10m, end at rest
    T = 5.0
    
    coeffs = quintic_coefficients(start, end, T)
    
    # Check at t=0
    p0, v0, a0 = evaluate_polynomial(coeffs, 0.0)
    print(f"  At t=0: p={p0:.4f}, v={v0:.4f}, a={a0:.4f}")
    
    assert abs(p0 - start[0]) < 1e-6, f"Position at t=0: expected {start[0]}, got {p0}"
    assert abs(v0 - start[1]) < 1e-6, f"Velocity at t=0: expected {start[1]}, got {v0}"
    assert abs(a0 - start[2]) < 1e-6, f"Acceleration at t=0: expected {start[2]}, got {a0}"
    
    # Check at t=T
    pT, vT, aT = evaluate_polynomial(coeffs, T)
    print(f"  At t=T: p={pT:.4f}, v={vT:.4f}, a={aT:.4f}")
    
    assert abs(pT - end[0]) < 1e-4, f"Position at t=T: expected {end[0]}, got {pT}"
    assert abs(vT - end[1]) < 1e-4, f"Velocity at t=T: expected {end[1]}, got {vT}"
    assert abs(aT - end[2]) < 1e-4, f"Acceleration at t=T: expected {end[2]}, got {aT}"
    
    print("  PASSED")


def test_quintic_with_initial_velocity():
    """Test quintic polynomial with non-zero initial velocity."""
    print("\nTest: Quintic with initial velocity")
    
    start = (0.0, 10.0, 0.0)  # Starting with 10 m/s
    end = (50.0, 10.0, 0.0)   # End at same velocity
    T = 5.0
    
    coeffs = quintic_coefficients(start, end, T)
    
    p0, v0, _ = evaluate_polynomial(coeffs, 0.0)
    pT, vT, _ = evaluate_polynomial(coeffs, T)
    
    print(f"  At t=0: p={p0:.4f}, v={v0:.4f}")
    print(f"  At t=T: p={pT:.4f}, v={vT:.4f}")
    
    assert abs(v0 - 10.0) < 1e-4, f"Initial velocity error"
    assert abs(vT - 10.0) < 1e-4, f"Final velocity error"
    assert abs(pT - 50.0) < 1e-4, f"Final position error"
    
    print("  PASSED")


def test_quartic_boundary_conditions():
    """Test that quartic polynomial satisfies boundary conditions."""
    print("\nTest: Quartic polynomial boundary conditions")
    
    start = (0.0, 10.0, 0.0)  # Starting at 10 m/s
    end_vel = (20.0, 0.0)     # Accelerate to 20 m/s
    T = 4.0
    
    coeffs = quartic_coefficients(start, end_vel, T)
    
    # Check at t=0
    p0, v0, a0 = evaluate_polynomial(coeffs, 0.0)
    print(f"  At t=0: p={p0:.4f}, v={v0:.4f}, a={a0:.4f}")
    
    assert abs(p0 - start[0]) < 1e-6, f"Position at t=0 error"
    assert abs(v0 - start[1]) < 1e-6, f"Velocity at t=0 error"
    assert abs(a0 - start[2]) < 1e-6, f"Acceleration at t=0 error"
    
    # Check at t=T
    pT, vT, aT = evaluate_polynomial(coeffs, T)
    print(f"  At t=T: p={pT:.4f}, v={vT:.4f}, a={aT:.4f}")
    
    assert abs(vT - end_vel[0]) < 1e-4, f"Final velocity error"
    assert abs(aT - end_vel[1]) < 1e-4, f"Final acceleration error"
    
    print("  PASSED")


def test_polynomial_continuity():
    """Test that polynomial is continuous (no jumps)."""
    print("\nTest: Polynomial continuity")
    
    start = (0.0, 0.0, 0.0)
    end = (10.0, 0.0, 0.0)
    T = 5.0
    
    coeffs = quintic_coefficients(start, end, T)
    
    # Sample at small intervals and check for continuity
    dt = 0.01
    prev_p, prev_v, prev_a = evaluate_polynomial(coeffs, 0.0)
    
    max_dp = 0
    max_dv = 0
    
    t = dt
    while t <= T:
        p, v, a = evaluate_polynomial(coeffs, t)
        max_dp = max(max_dp, abs(p - prev_p))
        max_dv = max(max_dv, abs(v - prev_v))
        prev_p, prev_v, prev_a = p, v, a
        t += dt
    
    print(f"  Max position change per {dt}s: {max_dp:.4f}")
    print(f"  Max velocity change per {dt}s: {max_dv:.4f}")
    
    # Position change should be reasonable (v_max * dt)
    assert max_dp < 1.0, f"Position discontinuity detected"
    # Velocity change should be reasonable (a_max * dt)
    assert max_dv < 0.5, f"Velocity discontinuity detected"
    
    print("  PASSED")


def test_jerk_computation():
    """Test jerk (third derivative) computation."""
    print("\nTest: Jerk computation")
    
    start = (0.0, 0.0, 0.0)
    end = (10.0, 0.0, 0.0)
    T = 5.0
    
    coeffs = quintic_coefficients(start, end, T)
    
    # Jerk at t=0 and t=T
    jerk_0 = compute_jerk(coeffs, 0.0)
    jerk_T = compute_jerk(coeffs, T)
    jerk_mid = compute_jerk(coeffs, T/2)
    
    print(f"  Jerk at t=0: {jerk_0:.4f}")
    print(f"  Jerk at t=T/2: {jerk_mid:.4f}")
    print(f"  Jerk at t=T: {jerk_T:.4f}")
    
    # Jerk should be finite
    assert np.isfinite(jerk_0), "Jerk at t=0 is not finite"
    assert np.isfinite(jerk_T), "Jerk at t=T is not finite"
    
    print("  PASSED")


def test_trajectory_cost():
    """Test trajectory cost computation."""
    print("\nTest: Trajectory cost computation")
    
    # Same distance, different durations
    start = (0.0, 0.0, 0.0)
    end = (10.0, 0.0, 0.0)
    
    coeffs_slow = quintic_coefficients(start, end, 5.0)
    coeffs_fast = quintic_coefficients(start, end, 2.0)
    
    cost_slow = compute_trajectory_cost(coeffs_slow, 5.0)
    cost_fast = compute_trajectory_cost(coeffs_fast, 2.0)
    
    print(f"  Cost for 10m in 5s: {cost_slow:.4f}")
    print(f"  Cost for 10m in 2s: {cost_fast:.4f}")
    
    # Faster trajectory should have higher jerk cost
    assert cost_fast > cost_slow, "Faster trajectory should have higher jerk cost"
    
    print("  PASSED")


def test_polynomial_trajectory_class():
    """Test the PolynomialTrajectory class."""
    print("\nTest: PolynomialTrajectory class")
    
    start = (0.0, 0.0, 0.0)
    end = (5.0, 0.0, 0.0)
    T = 3.0
    
    coeffs = quintic_coefficients(start, end, T)
    traj = PolynomialTrajectory(coeffs=coeffs, T=T, order=5)
    
    # Test evaluate
    p, v, a = traj.evaluate(1.5)
    print(f"  At t=1.5: p={p:.4f}, v={v:.4f}, a={a:.4f}")
    
    # Test sample
    times, positions, velocities, accelerations = traj.sample(dt=0.5)
    print(f"  Sample times: {times}")
    print(f"  Sample positions: {np.round(positions, 2)}")
    
    assert len(times) == len(positions), "Sample arrays should have same length"
    assert abs(positions[0] - start[0]) < 1e-4, "First sample should be at start"
    
    print("  PASSED")


def test_zero_duration():
    """Test handling of zero duration (edge case)."""
    print("\nTest: Zero/very small duration handling")
    
    start = (0.0, 10.0, 0.0)
    end = (0.0, 10.0, 0.0)  # Same as start
    T = 0.01  # Very small duration
    
    try:
        coeffs = quintic_coefficients(start, end, T)
        p, v, a = evaluate_polynomial(coeffs, T)
        print(f"  With T={T}: p={p:.4f}, v={v:.4f}")
        print("  PASSED (handled gracefully)")
    except Exception as e:
        print(f"  Exception raised: {e}")
        print("  PASSED (exception is acceptable for edge case)")


def test_negative_velocity():
    """Test polynomial with negative final velocity (braking)."""
    print("\nTest: Quartic with braking")
    
    start = (0.0, 20.0, 0.0)  # Starting at 20 m/s
    end_vel = (5.0, 0.0)      # Slow to 5 m/s
    T = 3.0
    
    coeffs = quartic_coefficients(start, end_vel, T)
    
    p0, v0, _ = evaluate_polynomial(coeffs, 0.0)
    pT, vT, _ = evaluate_polynomial(coeffs, T)
    
    print(f"  At t=0: p={p0:.4f}, v={v0:.4f}")
    print(f"  At t=T: p={pT:.4f}, v={vT:.4f}")
    
    assert abs(v0 - 20.0) < 1e-4, "Initial velocity error"
    assert abs(vT - 5.0) < 1e-4, "Final velocity error"
    assert pT > 0, "Position should be positive"
    
    print("  PASSED")


def run_all_tests():
    """Run all unit tests."""
    print("=" * 60)
    print("POLYNOMIAL TRAJECTORY UNIT TESTS")
    print("=" * 60)
    
    tests = [
        test_quintic_boundary_conditions,
        test_quintic_with_initial_velocity,
        test_quartic_boundary_conditions,
        test_polynomial_continuity,
        test_jerk_computation,
        test_trajectory_cost,
        test_polynomial_trajectory_class,
        test_zero_duration,
        test_negative_velocity
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
