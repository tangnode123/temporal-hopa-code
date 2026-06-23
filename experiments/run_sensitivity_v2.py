#!/usr/bin/env python3
"""Sensitivity to Contradiction Placement v2 — fixed placement logic.
   Three positions = contradiction at different TIME STEPS (t=0, 1, 2).
   This reflects a genuine spatial-temporal property: early detection
   is cheaper because candidate sets are smaller at earlier time steps.
   Config: 4obj, 8cons, T=2, 10×10×5, vmax=4.  5 instances each.
"""
import sys, time, json, statistics, math
sys.path.insert(0, '/mnt/agent-workspace/d68154738cd34be5999b72983a6487f3/universal_run-dade0502-63ac-4bca-95bd-5f8118be7e9c/artifacts/temporal-hopa/src')
from solver_v3 import SolverConfig, THOPASolver
from itertools import combinations

CFG = SolverConfig(grid_x=10, grid_y=10, grid_z=5, max_time=2,
                   max_speed=4.0, seed=42, use_accel=False, timeout=120)
NOBJ, NCONS = 4, 8
N_INST = 5
T_CONTRAS = [0, 1, 2]  # time steps to place contradiction

def run():
    solver = THOPASolver(CFG)
    all_results = {f't={t}': [] for t in T_CONTRAS}

    for seed_base in range(N_INST):
        solver.gen.rng = __import__('random').Random(seed_base * 100 + 42)
        traj = solver.gen.gen_trajectory(NOBJ)
        init = {k: traj[k][0] for k in traj}
        objs = list(range(1, NOBJ+1))
        pairs = list(combinations(range(1, NOBJ+1), 2))

        # Generate NCONS consistent constraints
        cons_base = []
        for t in range(CFG.max_time + 1):
            for k, l in pairs:
                if len(cons_base) >= NCONS: break
                x1,y1,z1 = traj[k][t]; x2,y2,z2 = traj[l][t]
                d = int(math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2))
                cons_base.append({'type':'dist_range','k':k,'l':l,
                    'd1':max(0,d-2),'d2':d+3,'time':t})
        cons_base = cons_base[:NCONS]

        for t_contra in T_CONTRAS:
            # Remove (1,2) at t_contra, insert contradictory pair in its place
            cons = [c for c in cons_base
                    if not (c['k']==1 and c['l']==2 and c['time']==t_contra)]
            # Take NCONS-2 base constraints + 2 contradictory = NCONS
            cons = cons[:NCONS-2]
            cons = cons + [
                {'type':'dist_range','k':1,'l':2,'d1':0,'d2':1,'time':t_contra},
                {'type':'dist_range','k':1,'l':2,'d1':20,'d2':50,'time':t_contra},
            ]

            r, stats = solver.solve_one(objs, cons, init,
                f"sens_t{t_contra}_{seed_base}", "inconsistent")
            all_results[f't={t_contra}'].append({
                'seed': seed_base,
                'sat': r.sat, 'total_time': r.total_time,
                'ground_time': r.ground_time, 'solve_time': r.solve_time,
                'atoms': r.atoms, 'rules': r.rules,
                'vel_nogoods': stats['vel_nogoods'],
                'dist_nogoods': stats['dist_nogoods'],
                'error': r.error,
            })
            print(f"  [t={t_contra} seed={seed_base}] SAT={r.sat} "
                  f"gnd={r.ground_time:.2f}s sol={r.solve_time:.2f}s "
                  f"tot={r.total_time:.2f}s rules={r.rules} "
                  f"d_ng={stats['dist_nogoods']}"
                  f"{' ERR='+r.error if r.error else ''}")

    return all_results

if __name__ == '__main__':
    print("=" * 70)
    print("Sensitivity to Contradiction Time-Step — Real Data")
    print("Config: 4obj, 8cons, T=2, 10×10×5, vmax=4")
    print(f"Contradiction at t={T_CONTRAS}, {N_INST} instances each")
    print("=" * 70)

    t0 = time.time()
    results = run()
    elapsed = time.time() - t0

    print(f"\n{'='*70}")
    print("RESULTS SUMMARY")
    print(f"{'='*70}")

    summary = {}
    for label, recs in results.items():
        ts = [r['total_time'] for r in recs]
        gs = [r['ground_time'] for r in recs]
        ss = [r['solve_time'] for r in recs]
        rs = [r['rules'] for r in recs]
        dn = [r['dist_nogoods'] for r in recs]
        sats = sum(1 for r in recs if r['sat'])
        errs = sum(1 for r in recs if r['error'])

        s = {
            'n': len(recs), 'sat_count': sats, 'errors': errs,
            't_mean': round(statistics.mean(ts), 3),
            't_median': round(statistics.median(ts), 3),
            't_min': round(min(ts), 3), 't_max': round(max(ts), 3),
            'gnd_mean': round(statistics.mean(gs), 3),
            'sol_mean': round(statistics.mean(ss), 3),
            'rules_mean': round(statistics.mean(rs)),
            'atoms_mean': round(statistics.mean([r['atoms'] for r in recs])),
            'd_ng_mean': round(statistics.mean(dn)),
        }
        if len(ts) > 1:
            s['t_std'] = round(statistics.stdev(ts), 3)
        summary[label] = s

        print(f"\n--- Contradiction at {label} (n={s['n']}) ---")
        print(f"  SAT: {s['sat_count']}/{s['n']}, Errors: {s['errors']}")
        print(f"  Total time:  mean={s['t_mean']:.3f}s median={s['t_median']:.3f}s")
        print(f"  Ground: {s['gnd_mean']:.3f}s  Solve: {s['sol_mean']:.3f}s")
        print(f"  Rules: {s['rules_mean']}  Dist nogoods: {s['d_ng_mean']}")

    out_path = '/mnt/agent-workspace/d68154738cd34be5999b72983a6487f3/universal_run-dade0502-63ac-4bca-95bd-5f8118be7e9c/artifacts/temporal-hopa/experiments/sensitivity_results_v2.json'
    with open(out_path, 'w') as f:
        json.dump({'config': 'M_4x8_T2', 'summary': summary,
                   'raw': results, 'elapsed': round(elapsed, 1)}, f, indent=2)

    print(f"\nSaved to {out_path}")
    print(f"Total elapsed: {elapsed:.1f}s")
