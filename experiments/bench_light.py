#!/usr/bin/env python3
"""Temporal-HOPA: Lightweight Benchmark (quick-completion version)"""
import sys, json, time, statistics, random
sys.path.insert(0, 'src')
from solver_v3 import *

CONFIGS = [
    ('S_2x4_T2',    8,  8,  4,  2,  3.0,  2,  4,  20),
    ('M_4x8_T2',   10, 10,  5,  2,  4.0,  4,  8,  10),
    ('T_4x8_T3',   10, 10,  5,  3,  4.0,  4,  8,   8),
    ('G_4x8_LG',   12, 12,  6,  2,  5.0,  4,  8,   8),
]

def run():
    print("="*60)
    print("Temporal-HOPA Lightweight Benchmark")
    print("="*60)
    all_results = {}
    for name, gx, gy, gz, mt, ms, nobj, ncons, ninst in CONFIGS:
        print(f"\n{'='*50}")
        print(f"Config: {name} | {nobj}obj, {ncons}cons, {gx}x{gy}x{gz}, T={mt}")
        print(f"{'='*50}")
        cfg = SolverConfig(grid_x=gx, grid_y=gy, grid_z=gz, max_time=mt, max_speed=ms, use_accel=False)
        solver = THOPASolver(cfg)
        cres, ires = [], []
        t0 = time.time()
        for i in range(ninst):
            solver.gen.rng = random.Random(i * 100 + 42)
            objs, cons, init, _ = solver.gen.gen_consistent(nobj, ncons)
            r, s = solver.solve_one(objs, cons, init, f"{name}_C_{i}", "consistent")
            cres.append(r)
            objs, cons, init, _ = solver.gen.gen_inconsistent(nobj, ncons)
            r, s = solver.solve_one(objs, cons, init, f"{name}_I_{i}", "inconsistent")
            ires.append(r)
            if (i+1) % 5 == 0 or i == ninst-1:
                c_sat = sum(1 for r in cres if r.sat)
                i_sat = sum(1 for r in ires if r.sat)
                c_t = statistics.mean([r.total_time for r in cres[-5:]])
                i_t = statistics.mean([r.total_time for r in ires[-5:]])
                print(f"  [{i+1:2d}/{ninst}] C: SAT={c_sat}/{len(cres)} t={c_t:.2f}s | I: SAT={i_sat}/{len(ires)} t={i_t:.2f}s | {time.time()-t0:.0f}s")
        def summarize(rs):
            if not rs: return {}
            ts = [r.total_time for r in rs]; gs = [r.ground_time for r in rs]
            ss = [r.solve_time for r in rs]; sc = sum(1 for r in rs if r.sat); n = len(rs)
            return {'n':n,'sat_n':sc,'sat_pct':f"{100*sc/n:.1f}%",
                't_mean':round(statistics.mean(ts),3),'t_max':round(max(ts),3),
                'gnd_mean':round(statistics.mean(gs),3),'sol_mean':round(statistics.mean(ss),3),
                'atoms_mean':round(statistics.mean([r.atoms for r in rs])),
                'rules_mean':round(statistics.mean([r.rules for r in rs]))}
        all_results[name] = {
            'params': {'grid':f'{gx}x{gy}x{gz}','max_time':mt,'max_speed':ms,'n_obj':nobj,'n_cons':ncons,'n_inst':ninst},
            'consistent':summarize(cres), 'inconsistent':summarize(ires)}
        print(f"  Done in {time.time()-t0:.0f}s")
    # Save
    out = '/mnt/agent-workspace/d68154738cd34be5999b72983a6487f3/universal_run-dade0502-63ac-4bca-95bd-5f8118be7e9c/artifacts/temporal-hopa/experiments/results_v3_final.json'
    with open(out,'w') as f: json.dump(all_results, f, indent=2)
    print(f"\nResults saved to {out}")
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for name, data in all_results.items():
        print(f"\n--- {name} ---")
        for typ in ['consistent','inconsistent']:
            r = data[typ]
            print(f"  {typ:12s}: SAT={r['sat_n']:2d}/{r['n']:2d} ({r['sat_pct']}) | t_mean={r['t_mean']:.3f}s | t_max={r['t_max']:.3f}s | gnd={r['gnd_mean']:.3f}s | atoms={r['atoms_mean']} | rules={r['rules_mean']}")
    return all_results

if __name__ == '__main__':
    run()
