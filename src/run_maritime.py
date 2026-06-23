#!/usr/bin/env python3
"""
Maritime Vessel Encounter Scenario for Temporal-HOPA.
Models two vessels (Voyager, Horizon) in a head-on situation with
COLREGs-inspired safety-separation constraints.
Uses the v3.1 solver with pre-computed candidates and sparse nogoods.
"""
import sys, os, time, math, statistics

# Add solver to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from solver_v3 import SolverConfig, THOPASolver, build_candidates, make_asp_program
import clingo

def run_maritime_scenario():
    print("=" * 60)
    print("Maritime Vessel Encounter — COLREGs Head-On Scenario")
    print("=" * 60)

    # Scenario: 2 vessels on near-collision course
    # Grid: 12x12x4 km at 1km resolution
    # Time: tau=4 steps (dt=5min, total 20min)
    cfg = SolverConfig(
        grid_x=12, grid_y=12, grid_z=4,
        max_time=4,
        max_speed=3.0,   # conservative: 3 km/step ≈ 10.3 m/s ≈ 20 knots
        max_accel=3.0,
        seed=42,
        use_accel=False,
        timeout=60
    )

    solver = THOPASolver(cfg)
    objects = [1, 2]  # 1=Voyager, 2=Horizon

    # Initial positions: Voyager SW, Horizon NE
    init_pos = {
        1: (1, 6, 0),   # Voyager starts at west side, surface
        2: (11, 6, 1),  # Horizon starts at east side, slightly elevated
    }

    # ================================================================
    # CONSISTENT SCENARIO: safety separation d ∈ [2, 12] at all t
    # ================================================================
    safe_constraints = []
    for t in range(cfg.max_time + 1):
        safe_constraints.append({
            'type': 'dist_range',
            'k': 1, 'l': 2,
            'd1': 2,   # min 2 km separation
            'd2': 12,  # max 12 km (within detection range)
            'time': t
        })

    print(f"\n{'='*40}")
    print("SCENARIO A: Safe head-on passage (d ∈ [2,12] km)")
    print(f"Grid: {cfg.grid_x}x{cfg.grid_y}x{cfg.grid_z}, "
          f"T={cfg.max_time}, vmax={cfg.max_speed}")
    print(f"Voyager init: {init_pos[1]}, Horizon init: {init_pos[2]}")
    print(f"Constraints: {len(safe_constraints)} distance-range")

    t0 = time.time()
    r_safe, s_safe = solver.solve_one(
        objects, safe_constraints, init_pos,
        "maritime_safe", "maritime_consistent"
    )
    t1 = time.time()

    print(f"\nResults:")
    print(f"  SAT: {r_safe.sat}")
    print(f"  Models: {r_safe.models}")
    print(f"  Ground time: {r_safe.ground_time:.3f}s")
    print(f"  Solve time: {r_safe.solve_time:.3f}s")
    print(f"  Total time: {r_safe.total_time:.3f}s")
    print(f"  Atoms: {r_safe.atoms}, Rules: {r_safe.rules}")
    print(f"  Nogoods: vel={s_safe['vel_nogoods']}, "
          f"dist={s_safe['dist_nogoods']}")

    if r_safe.error:
        print(f"  ERROR: {r_safe.error}")

    # ================================================================
    # INCONSISTENT SCENARIO: safety separation tightened to d ∈ [4, 12]
    # while initial positions are only ~11.2 km apart
    # ================================================================
    unsafe_constraints = []
    for t in range(cfg.max_time + 1):
        if t <= 2:
            # Tighter constraint: d ∈ [4, 12] — possible at t=0 (d≈10.2)
            # but forces separation ≥4 at all steps
            unsafe_constraints.append({
                'type': 'dist_range',
                'k': 1, 'l': 2,
                'd1': 4,   # stricter minimum
                'd2': 12,
                'time': t
            })
        else:
            # At t=3: contradictory — require d ∈ [0, 1] AND d ∈ [4, 12]
            unsafe_constraints.append({
                'type': 'dist_range',
                'k': 1, 'l': 2,
                'd1': 0, 'd2': 1,   # must be very close
                'time': 3
            })
            unsafe_constraints.append({
                'type': 'dist_range',
                'k': 1, 'l': 2,
                'd1': 8, 'd2': 12,  # must be far apart
                'time': 3
            })

    print(f"\n{'='*40}")
    print("SCENARIO B: Unsafe — contradictory constraints at t=3")
    print(f"Constraints: {len(unsafe_constraints)} distance-range "
          f"(including contradictory pair)")

    t0 = time.time()
    r_unsafe, s_unsafe = solver.solve_one(
        objects, unsafe_constraints, init_pos,
        "maritime_unsafe", "maritime_inconsistent"
    )
    t1 = time.time()

    print(f"\nResults:")
    print(f"  SAT: {r_unsafe.sat}")
    print(f"  Models: {r_unsafe.models}")
    print(f"  Ground time: {r_unsafe.ground_time:.3f}s")
    print(f"  Solve time: {r_unsafe.solve_time:.3f}s")
    print(f"  Total time: {r_unsafe.total_time:.3f}s")
    print(f"  Atoms: {r_unsafe.atoms}, Rules: {r_unsafe.rules}")
    print(f"  Nogoods: vel={s_unsafe['vel_nogoods']}, "
          f"dist={s_unsafe['dist_nogoods']}")

    if r_unsafe.error:
        print(f"  ERROR: {r_unsafe.error}")

    # ================================================================
    # SUMMARY
    # ================================================================
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    print(f"Scenario              |V|  tau  SAT?   Time")
    print(f"{'-'*50}")
    print(f"Safe head-on           {len(objects):2d}    {cfg.max_time}   "
          f"{'SAT' if r_safe.sat else 'UNSAT':5s}  "
          f"{r_safe.total_time:.2f}s")
    print(f"Unsafe (contradictory) {len(objects):2d}    {cfg.max_time}   "
          f"{'SAT' if r_unsafe.sat else 'UNSAT':5s}  "
          f"{r_unsafe.total_time:.2f}s")

    # Compute candidate counts
    cands, lookup = build_candidates(init_pos, objects, cfg)
    for p in objects:
        sizes = [len(cands[(p, t)]) for t in range(1, cfg.max_time + 1)]
        print(f"  Object {p} candidates/step: {sizes} "
              f"(mean={statistics.mean(sizes):.0f})")

    return {
        'safe': {'sat': r_safe.sat, 'time': r_safe.total_time,
                 'ground': r_safe.ground_time, 'solve': r_safe.solve_time,
                 'vel_nogoods': s_safe['vel_nogoods'],
                 'dist_nogoods': s_safe['dist_nogoods']},
        'unsafe': {'sat': r_unsafe.sat, 'time': r_unsafe.total_time,
                   'ground': r_unsafe.ground_time, 'solve': r_unsafe.solve_time,
                   'vel_nogoods': s_unsafe['vel_nogoods'],
                   'dist_nogoods': s_unsafe['dist_nogoods']},
    }

if __name__ == '__main__':
    results = run_maritime_scenario()
