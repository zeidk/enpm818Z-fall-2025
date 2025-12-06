"""
test_cost.py - Unit Tests for Cost Function and Feasibility

Run this file to test your implementation:
    python test_cost.py
"""

import sys
import numpy as np
from polynomial import (
    quintic_coefficients, quartic_coefficients,
    generate_trajectory, generate_candidate_trajectories
)
from cost import (
    CostWeights, FeasibilityLimits,
    compute_total_cost, check_feasibility, select_best_trajectory
)


def run_tests():
    """Run all cost function tests."""
    
    passed = 0
    failed = 0
    
    print("=" * 60)
    print("COST FUNCTION AND FEASIBILITY UNIT TESTS")
    print("=" * 60)
    
    # Create test trajectory
    print("\n--- Setup ---\n")
    
    d_start = (0.0, 0.0, 0.0)
    d_end = (3.5, 0.0, 0.0)
    s_start = (0.0, 25.0, 0.0)
    s_end_vel = (25.0, 0.0)
    T = 4.0
    
    d_coeffs = quintic_coefficients(d_start, d_end, T)
    s_coeffs = quartic_coefficients(s_start, s_end_vel, T)
    traj = generate_trajectory(d_coeffs, s_coeffs, T)
    
    if traj is None or len(traj.states) == 0:
        print("[FAIL] Could not create test trajectory - check polynomial.py")
        return False
    
    print(f"Test trajectory: {len(traj.states)} states, T={T}s")
    print(f"Final state: d={traj.states[-1].d:.2f}m, v={traj.states[-1].s_dot:.2f}m/s")
    
    # =========================================================================
    # Test check_feasibility
    # =========================================================================
    
    print("\n--- Feasibility Checking Tests ---\n")
    
    # Test 1: Normal limits (should pass)
    limits = FeasibilityLimits()
    feasible = check_feasibility(traj, limits)
    
    if feasible is not None:
        if feasible:
            print("[PASS] Normal limits: trajectory is feasible")
            passed += 1
        else:
            print("[FAIL] Normal limits: trajectory should be feasible")
            failed += 1
    else:
        print("[FAIL] check_feasibility returned None")
        failed += 1
    
    # Test 2: Very strict velocity limit (should fail)
    strict_limits = FeasibilityLimits(max_velocity=20.0)
    feasible_strict = check_feasibility(traj, strict_limits)
    
    if feasible_strict is not None:
        if not feasible_strict:
            print("[PASS] Strict velocity limit (v<20): trajectory is infeasible")
            passed += 1
        else:
            print("[FAIL] Strict velocity limit (v<20): trajectory should be infeasible")
            failed += 1
    else:
        print("[FAIL] check_feasibility returned None")
        failed += 1
    
    # Test 3: Create high-acceleration trajectory
    print("\nTest: High acceleration trajectory")
    s_start_fast = (0.0, 10.0, 0.0)  # Start slow
    s_end_fast = (35.0, 0.0)  # End fast
    T_short = 2.0
    
    s_coeffs_fast = quartic_coefficients(s_start_fast, s_end_fast, T_short)
    traj_fast = generate_trajectory(d_coeffs, s_coeffs_fast, T_short)
    
    if traj_fast is not None:
        strict_accel = FeasibilityLimits(max_acceleration=2.0)
        feasible_accel = check_feasibility(traj_fast, strict_accel)
        
        if feasible_accel is not None and not feasible_accel:
            print("[PASS] High acceleration: correctly rejected")
            passed += 1
        else:
            print("[FAIL] High acceleration: should be rejected")
            failed += 1
    else:
        print("[SKIP] Could not create high-acceleration trajectory")
    
    # =========================================================================
    # Test compute_total_cost
    # =========================================================================
    
    print("\n--- Cost Computation Tests ---\n")
    
    weights = CostWeights()
    
    # Test 4: Cost with matching targets
    cost_match = compute_total_cost(traj, target_d=3.5, target_speed=25.0, weights=weights)
    
    if cost_match is not None and cost_match >= 0:
        print(f"[PASS] Cost (matching targets): {cost_match:.2f}")
        passed += 1
    else:
        print(f"[FAIL] Cost computation returned invalid result: {cost_match}")
        failed += 1
    
    # Test 5: Cost with different targets (should be higher)
    cost_off = compute_total_cost(traj, target_d=0.0, target_speed=30.0, weights=weights)
    
    if cost_off is not None and cost_match is not None:
        if cost_off > cost_match:
            print(f"[PASS] Cost (different targets): {cost_off:.2f} > {cost_match:.2f}")
            passed += 1
        else:
            print(f"[FAIL] Cost should be higher with different targets: {cost_off:.2f} vs {cost_match:.2f}")
            failed += 1
    else:
        print("[FAIL] Cost computation returned None")
        failed += 1
    
    # Test 6: Zero lateral deviation has lower d-cost
    print("\nTest: Lateral deviation cost component")
    d_coeffs_center = quintic_coefficients((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), T)
    traj_center = generate_trajectory(d_coeffs_center, s_coeffs, T)
    
    if traj_center is not None:
        # Use high d-weight to emphasize lateral cost
        weights_d = CostWeights(w_d=10.0, w_jerk=0.1, w_v=0.1, w_time=0.1)
        cost_center = compute_total_cost(traj_center, target_d=0.0, target_speed=25.0, weights=weights_d)
        cost_offset = compute_total_cost(traj, target_d=0.0, target_speed=25.0, weights=weights_d)
        
        if cost_center is not None and cost_offset is not None:
            if cost_center < cost_offset:
                print(f"[PASS] Center trajectory has lower cost: {cost_center:.2f} < {cost_offset:.2f}")
                passed += 1
            else:
                print("[FAIL] Center trajectory should have lower d-cost")
                failed += 1
        else:
            print("[FAIL] Cost computation returned None")
            failed += 1
    else:
        print("[SKIP] Could not create center trajectory")
    
    # =========================================================================
    # Test select_best_trajectory
    # =========================================================================
    
    print("\n--- Trajectory Selection Tests ---\n")
    
    # Test 7: Generate candidates and select best
    current = {'s': 0, 'd': 0, 's_dot': 25, 'd_dot': 0, 's_ddot': 0, 'd_ddot': 0}
    candidates = generate_candidate_trajectories(
        current, target_d=3.5, target_speed=25.0, T_base=4.0,
        T_range=1.0, T_step=0.5, d_range=0.5, d_step=0.5, v_range=2.0, v_step=2.0
    )
    
    print(f"Generated {len(candidates)} candidate trajectories")
    
    best = select_best_trajectory(candidates, target_d=3.5, target_speed=25.0)
    
    if best is not None:
        print("[PASS] Selected best trajectory:")
        print(f"       Duration: {best.T:.1f}s")
        print(f"       Final d: {best.states[-1].d:.2f}m")
        print(f"       Final v: {best.states[-1].s_dot:.2f}m/s")
        print(f"       Cost: {best.cost:.2f}")
        passed += 1
        
        # Verify it's actually the best
        if best.feasible:
            print("[PASS] Best trajectory is marked feasible")
            passed += 1
        else:
            print("[FAIL] Best trajectory should be marked feasible")
            failed += 1
    else:
        print("[FAIL] select_best_trajectory returned None")
        failed += 2
    
    # Test 8: All infeasible candidates
    print("\nTest: All candidates infeasible")
    very_strict = FeasibilityLimits(max_velocity=5.0)  # No trajectory will pass
    best_none = select_best_trajectory(candidates, target_d=3.5, target_speed=25.0, limits=very_strict)
    
    if best_none is None:
        print("[PASS] Correctly returns None when no feasible trajectory")
        passed += 1
    else:
        print("[FAIL] Should return None when all trajectories infeasible")
        failed += 1
    
    # =========================================================================
    # Summary
    # =========================================================================
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {passed} passed, {failed} failed")
    print("=" * 60)
    
    if failed == 0:
        print("\n✅ All cost function tests passed!")
    else:
        print(f"\n❌ {failed} test(s) failed. Check your implementation.")
    
    return failed == 0


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
