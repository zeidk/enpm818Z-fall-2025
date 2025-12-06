"""
test_polynomial.py - Unit Tests for Polynomial Trajectory Generation

Run this file to test your implementation:
    python test_polynomial.py
"""

import sys
import numpy as np
from polynomial import (
    quintic_coefficients, quartic_coefficients,
    evaluate_polynomial, compute_jerk, generate_trajectory
)


def run_tests():
    """Run all polynomial trajectory tests."""
    
    passed = 0
    failed = 0
    
    print("=" * 60)
    print("POLYNOMIAL TRAJECTORY UNIT TESTS")
    print("=" * 60)
    
    # =========================================================================
    # Test quintic_coefficients
    # =========================================================================
    
    print("\n--- Quintic Polynomial Tests ---\n")
    
    # Test 1: Lane change (d: 0 -> 3.5)
    print("Test: Lane change d=0 to d=3.5m in T=4s")
    d_start = (0.0, 0.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    T = 4.0
    
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    
    if d_coeffs is not None and len(d_coeffs) == 6:
        # Check boundary conditions at t=0
        p0, v0, a0 = evaluate_polynomial(d_coeffs, 0.0)
        if p0 is not None:
            bc0_ok = abs(p0 - 0.0) < 0.01 and abs(v0 - 0.0) < 0.01 and abs(a0 - 0.0) < 0.01
            if bc0_ok:
                print(f"[PASS] t=0: p={p0:.3f}, v={v0:.3f}, a={a0:.3f}")
                passed += 1
            else:
                print(f"[FAIL] t=0: p={p0:.3f}, v={v0:.3f}, a={a0:.3f} (expected 0, 0, 0)")
                failed += 1
        else:
            print("[FAIL] evaluate_polynomial returned None")
            failed += 1
        
        # Check boundary conditions at t=T
        pT, vT, aT = evaluate_polynomial(d_coeffs, T)
        if pT is not None:
            bcT_ok = abs(pT - 3.5) < 0.01 and abs(vT - 0.0) < 0.01 and abs(aT - 0.0) < 0.01
            if bcT_ok:
                print(f"[PASS] t=T: p={pT:.3f}, v={vT:.3f}, a={aT:.3f}")
                passed += 1
            else:
                print(f"[FAIL] t=T: p={pT:.3f}, v={vT:.3f}, a={aT:.3f} (expected 3.5, 0, 0)")
                failed += 1
        else:
            print("[FAIL] evaluate_polynomial returned None")
            failed += 1
    else:
        print("[FAIL] quintic_coefficients returned invalid result")
        failed += 2
    
    # Test 2: Non-zero initial velocity
    print("\nTest: Quintic with initial velocity v0=2 m/s")
    d_start = (0.0, 2.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    T = 3.0
    
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    
    if d_coeffs is not None:
        p0, v0, a0 = evaluate_polynomial(d_coeffs, 0.0)
        if v0 is not None and abs(v0 - 2.0) < 0.01:
            print(f"[PASS] Initial velocity v0={v0:.3f}")
            passed += 1
        else:
            print(f"[FAIL] Initial velocity v0={v0} (expected 2.0)")
            failed += 1
    else:
        print("[FAIL] quintic_coefficients returned None")
        failed += 1
    
    # =========================================================================
    # Test quartic_coefficients
    # =========================================================================
    
    print("\n--- Quartic Polynomial Tests ---\n")
    
    # Test 3: Speed up (v: 20 -> 25 m/s)
    print("Test: Speed change v=20 to v=25 m/s in T=3s")
    s_start = (0.0, 20.0, 0.0)
    s_end_vel = (25.0, 0.0)
    T = 3.0
    
    s_coeffs = quartic_coefficients(s_start, s_end_vel, T)
    
    if s_coeffs is not None and len(s_coeffs) == 5:
        # Check initial conditions
        p0, v0, a0 = evaluate_polynomial(s_coeffs, 0.0)
        if v0 is not None:
            bc0_ok = abs(v0 - 20.0) < 0.01 and abs(a0 - 0.0) < 0.01
            if bc0_ok:
                print(f"[PASS] t=0: v={v0:.3f}, a={a0:.3f}")
                passed += 1
            else:
                print(f"[FAIL] t=0: v={v0:.3f}, a={a0:.3f} (expected v=20, a=0)")
                failed += 1
        else:
            print("[FAIL] evaluate_polynomial returned None")
            failed += 1
        
        # Check final velocity/acceleration
        pT, vT, aT = evaluate_polynomial(s_coeffs, T)
        if vT is not None:
            bcT_ok = abs(vT - 25.0) < 0.01 and abs(aT - 0.0) < 0.01
            if bcT_ok:
                print(f"[PASS] t=T: v={vT:.3f}, a={aT:.3f}")
                passed += 1
            else:
                print(f"[FAIL] t=T: v={vT:.3f}, a={aT:.3f} (expected v=25, a=0)")
                failed += 1
        else:
            print("[FAIL] evaluate_polynomial returned None")
            failed += 1
    else:
        print("[FAIL] quartic_coefficients returned invalid result")
        failed += 2
    
    # =========================================================================
    # Test compute_jerk
    # =========================================================================
    
    print("\n--- Jerk Computation Tests ---\n")
    
    # Test 4: Jerk at t=0
    d_start = (0.0, 0.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    T = 4.0
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    
    if d_coeffs is not None:
        jerk_0 = compute_jerk(d_coeffs, 0.0)
        jerk_mid = compute_jerk(d_coeffs, T/2)
        jerk_T = compute_jerk(d_coeffs, T)
        
        if jerk_0 is not None and jerk_mid is not None:
            print(f"[PASS] Jerk at t=0: {jerk_0:.3f} m/s³")
            print(f"[PASS] Jerk at t=T/2: {jerk_mid:.3f} m/s³")
            print(f"[PASS] Jerk at t=T: {jerk_T:.3f} m/s³")
            passed += 1
        else:
            print("[FAIL] compute_jerk returned None")
            failed += 1
    else:
        print("[FAIL] Could not test jerk - quintic_coefficients failed")
        failed += 1
    
    # =========================================================================
    # Test generate_trajectory
    # =========================================================================
    
    print("\n--- Trajectory Generation Tests ---\n")
    
    # Test 5: Generate complete trajectory
    d_start = (0.0, 0.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    s_start = (0.0, 25.0, 0.0)
    s_end_vel = (25.0, 0.0)
    T = 4.0
    
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    s_coeffs = quartic_coefficients(s_start, s_end_vel, T)
    
    if d_coeffs is not None and s_coeffs is not None:
        traj = generate_trajectory(d_coeffs, s_coeffs, T, dt=0.1)
        
        if traj is not None and hasattr(traj, 'states') and len(traj.states) > 0:
            expected_states = int(T / 0.1) + 1
            n_states = len(traj.states)
            
            if abs(n_states - expected_states) <= 1:
                print(f"[PASS] Generated {n_states} states (expected ~{expected_states})")
                passed += 1
            else:
                print(f"[FAIL] Generated {n_states} states (expected ~{expected_states})")
                failed += 1
            
            # Check first state
            first = traj.states[0]
            if abs(first.t - 0.0) < 0.01 and abs(first.d - 0.0) < 0.1:
                print(f"[PASS] First state: t={first.t:.2f}, d={first.d:.2f}")
                passed += 1
            else:
                print(f"[FAIL] First state: t={first.t:.2f}, d={first.d:.2f}")
                failed += 1
            
            # Check last state
            last = traj.states[-1]
            if abs(last.d - 3.5) < 0.2:
                print(f"[PASS] Last state: t={last.t:.2f}, d={last.d:.2f}")
                passed += 1
            else:
                print(f"[FAIL] Last state: t={last.t:.2f}, d={last.d:.2f} (expected d≈3.5)")
                failed += 1
        else:
            print("[FAIL] generate_trajectory returned invalid result")
            failed += 3
    else:
        print("[FAIL] Could not test trajectory generation - coefficient functions failed")
        failed += 3
    
    # =========================================================================
    # Summary
    # =========================================================================
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✅ All polynomial trajectory tests passed!")
    else:
        print(f"\n❌ {failed} test(s) failed. Check your implementation.")
    
    return failed == 0


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
