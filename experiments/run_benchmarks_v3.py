#!/usr/bin/env python3
"""
Temporal-HOPA Benchmark Suite
==============================
Runs multiple configurations of T-HOPA consistency checking
and collects comprehensive experimental results.
"""
import sys, json, time, statistics, random
sys.path.insert(0, 'src')
from solver_v3 import *

# ============================================================
# Benchmark configurations
# ============================================================

CONFIGS = [
    # (name, grid_x, grid_y, grid_z, max_time, max_speed, n_obj, n_cons, n_inst, timeout)
    ('S_2x4_T2',    8,  8,  4,  2,  3.0,  2,  4,  30,  '2obj,4cons,2steps,8x8x4'),
    ('M1_4x8_T2',  10, 10,  5,  2,  4.0,  4,  8,  20,  '4obj,8cons,2steps,10x10x5'),
    ('M2_6x12_T2', 12, 12,  5,  2,  5.0,  6, 12,  15,  '6obj,12cons,2steps,12x12x5'),
    ('T_4x8_T3',   10, 10,  5,  3,  4.0,  4,  8,  15,  '4obj,8cons,3steps,10x10x5'),
    ('G_4x8_G12',  12, 12,  6,  2,  5.0,  4,  8,  15,  '4obj,8cons,2steps,12x12x6'),
]

# ============================================================
# Main
# ============================================================

def run_all_benchmarks():
    print("=" * 70)
    print("Temporal-HOPA Benchmark Suite")
    print("=" * 70)
    
    all_results = {}
    
    for cfg_spec in CONFIGS:
        name, gx, gy, gz, mt, ms, nobj, ncons, n_inst, desc = cfg_spec
        
        print(f"\n{'='*60}")
        print(f"Benchmark: {name} — {desc}")
        print(f"  Grid={gx}x{gy}x{gz}, max_time={mt}, max_speed={ms}")
        print(f"  n_objects={nobj}, n_constraints={ncons}, n_instances={n_inst}")
        print(f"{'='*60}")
        
        cfg = SolverConfig(grid_x=gx, grid_y=gy, grid_z=gz,
                          max_time=mt, max_speed=ms, use_accel=False)
        solver = THOPASolver(cfg)
        
        cres, ires = [], []
        t_start = time.time()
        
        for i in range(n_inst):
            solver.gen.rng = random.Random(i * 100 + 42)
            
            # Consistent instance
            objs, cons, init, _ = solver.gen.gen_consistent(nobj, ncons)
            r, s = solver.solve_one(objs, cons, init, f"{name}_C_{i}", "consistent")
            cres.append(r)
            
            # Inconsistent instance
            objs, cons, init, _ = solver.gen.gen_inconsistent(nobj, ncons)
            r, s = solver.solve_one(objs, cons, init, f"{name}_I_{i}", "inconsistent")
            ires.append(r)
            
            if (i + 1) % 5 == 0 or i == n_inst - 1:
                c_sat = sum(1 for r in cres if r.sat)
                i_sat = sum(1 for r in ires if r.sat)
                c_t = statistics.mean([r.total_time for r in cres[-5:]])
                i_t = statistics.mean([r.total_time for r in ires[-5:]])
                elapsed = time.time() - t_start
                print(f"  [{i+1:3d}/{n_inst}] "
                      f"C: {c_sat:2d}/{len(cres)} (t={c_t:.2f}s) | "
                      f"I: {i_sat:2d}/{len(ires)} (t={i_t:.2f}s) | "
                      f"elapsed={elapsed:.1f}s")
        
        def summarize(rs, typ):
            if not rs: return {}
            ts = [r.total_time for r in rs]
            gs = [r.ground_time for r in rs]
            ss = [r.solve_time for r in rs]
            sc = sum(1 for r in rs if r.sat)
            n = len(rs)
            errs = sum(1 for r in rs if r.error)
            return {
                'n': n, 'sat_n': sc,
                'sat_pct': f"{100.0*sc/n:.1f}%",
                'errors': errs,
                't_mean': round(statistics.mean(ts), 3),
                't_median': round(statistics.median(ts), 3),
                't_max': round(max(ts), 3),
                't_std': round(statistics.stdev(ts), 3) if n > 1 else 0,
                'gnd_mean': round(statistics.mean(gs), 3),
                'sol_mean': round(statistics.mean(ss), 3),
                'atoms_mean': round(statistics.mean([r.atoms for r in rs])),
                'rules_mean': round(statistics.mean([r.rules for r in rs])),
            }
        
        all_results[name] = {
            'desc': desc,
            'params': {
                'grid': f'{gx}x{gy}x{gz}',
                'max_time': mt,
                'max_speed': ms,
                'n_objects': nobj,
                'n_constraints': ncons,
                'n_instances': n_inst,
                'acceleration': False,
            },
            'consistent': summarize(cres, 'consistent'),
            'inconsistent': summarize(ires, 'inconsistent'),
        }
        
        elapsed = time.time() - t_start
        print(f"  Done in {elapsed:.1f}s")
    
    return all_results


if __name__ == '__main__':
    results = run_all_benchmarks()
    
    # Save results
    out_path = '/mnt/agent-workspace/d68154738cd34be5999b72983a6487f3/universal_run-dade0502-63ac-4bca-95bd-5f8118be7e9c/artifacts/temporal-hopa/experiments/results_v3.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n\n{'='*70}")
    print("BENCHMARK RESULTS SUMMARY")
    print(f"{'='*70}")
    
    for name, data in results.items():
        print(f"\n--- {name}: {data['desc']} ---")
        for typ in ['consistent', 'inconsistent']:
            r = data[typ]
            print(f"  {typ:12s}: SAT={r['sat_n']:2d}/{r['n']:2d} ({r['sat_pct']}) | "
                  f"t_mean={r['t_mean']:.3f}s | t_max={r['t_max']:.3f}s | "
                  f"gnd={r['gnd_mean']:.3f}s | "
                  f"atoms={r['atoms_mean']:.0f} | rules={r['rules_mean']:.0f}")
    
    print(f"\nResults saved to {out_path}")
