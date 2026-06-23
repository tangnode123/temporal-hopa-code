#!/usr/bin/env python3
"""
Temporal-HOPA Solver v3.1 — Sparse Nogoods + Optional Acceleration
==================================================================
Key improvements:
1. Acceleration constraints are OFF by default (optional feature)
2. Distance constraints at t=0 are checked in Python (no ASP nogoods needed)
3. Sparse nogood generation: only emit violations, not all combinations
4. Velocity nogoods use efficient incremental validation
"""
import clingo, random, time, math, statistics, json, sys
from dataclasses import dataclass
from typing import List, Dict, Tuple
from itertools import combinations

@dataclass
class SolverConfig:
    grid_x: int = 10; grid_y: int = 10; grid_z: int = 5
    max_time: int = 3; max_speed: float = 4.0
    max_accel: float = 3.0; timeout: int = 60; seed: int = 42
    use_accel: bool = False  # acceleration constraints OFF by default

@dataclass
class InstanceResult:
    iid: str = ""; n_obj: int = 0; n_cons: int = 0
    grid: str = ""; max_time: int = 0; config_type: str = ""
    sat: bool = False; models: int = 0
    ground_time: float = 0.0; solve_time: float = 0.0
    total_time: float = 0.0; atoms: int = 0; rules: int = 0
    error: str = ""

def reachable_from(x0, y0, z0, max_v, gx, gy, gz):
    mv = int(max_v); mv2 = max_v * max_v
    result = []
    for dx in range(-mv, mv+1):
        dx2 = dx*dx
        for dy in range(-mv, mv+1):
            dz2_max = int(mv2 - dx2 - dy*dy)
            if dz2_max < 0: continue
            max_dz = int(math.sqrt(dz2_max))
            for dz in range(-max_dz, max_dz+1):
                if dx2 + dy*dy + dz*dz > mv2: continue
                nx, ny, nz = x0+dx, y0+dy, z0+dz
                if 1 <= nx <= gx and 1 <= ny <= gy and 1 <= nz <= gz:
                    result.append((nx, ny, nz))
    return result

def build_candidates(init_pos, objects, cfg):
    cands = {}; lookup = {}
    for p in objects:
        x0, y0, z0 = init_pos[p]
        cands[(p,0)] = [(0, x0, y0, z0)]
        lookup[(p,0,0)] = (x0, y0, z0)
        frontier = [(x0, y0, z0)]
        for t in range(1, cfg.max_time+1):
            layer = {}
            for cx, cy, cz in frontier:
                for nx, ny, nz in reachable_from(cx, cy, cz, cfg.max_speed, cfg.grid_x, cfg.grid_y, cfg.grid_z):
                    key = (nx, ny, nz)
                    if key not in layer:
                        cid = len(layer)
                        layer[key] = cid
                        lookup[(p, t, cid)] = (nx, ny, nz)
            cands[(p,t)] = [(cid, x, y, z) for (x,y,z), cid in layer.items()]
            frontier = [(x,y,z) for _,x,y,z in cands[(p,t)]]
    return cands, lookup

def make_asp_program(cands, lookup, objects, constraints, cfg):
    lines = []
    mv = int(cfg.max_speed); ma = int(cfg.max_accel)
    mv2 = cfg.max_speed * cfg.max_speed + 1.0
    
    lines.append(f"#const max_t = {cfg.max_time}.")
    lines.append("time(0..max_t).")
    for p in objects:
        lines.append(f"obj({p}).")
    
    # Initial positions
    for p in objects:
        x,y,z = lookup[(p,0,0)]
        lines.append(f"init_x({p},{x}). init_y({p},{y}). init_z({p},{z}).")
    
    # Candidate facts
    for (p,t), layer in cands.items():
        if t == 0: continue
        for cid, x, y, z in layer:
            lines.append(f"cand({p},{t},{cid},{x},{y},{z}).")
        lines.append(f"ncands({p},{t},{len(layer)}).")
    
    # Choice rule
    lines.append("1 { selected(P,T,ID) : cand(P,T,ID,_,_,_) } 1 :- obj(P), time(T), T>0.")
    
    # Position derivation
    lines.append("xpos(P,T,X) :- selected(P,T,ID), cand(P,T,ID,X,_,_).")
    lines.append("ypos(P,T,Y) :- selected(P,T,ID), cand(P,T,ID,_,Y,_).")
    lines.append("zpos(P,T,Z) :- selected(P,T,ID), cand(P,T,ID,_,_,Z).")
    lines.append("xpos(P,0,X) :- init_x(P,X). ypos(P,0,Y) :- init_y(P,Y). zpos(P,0,Z) :- init_z(P,Z).")
    
    vel_count = 0
    accel_count = 0
    dist_count = 0
    
    # Velocity nogoods: only emit for VIOLATING pairs (sparse)
    for p in objects:
        for t in range(1, cfg.max_time+1):
            prev = cands[(p, t-1)]
            curr = cands[(p, t)]
            for id_p, px, py, pz in prev:
                for id_c, cx, cy, cz in curr:
                    sd = (px-cx)**2 + (py-cy)**2 + (pz-cz)**2
                    if sd > mv2:
                        if t == 1:
                            # t=0 uses fixed position (id=0)
                            lines.append(f":- selected({p},{t},{id_c}), init_x({p},{px}), "
                                        f"init_y({p},{py}), init_z({p},{pz}).")
                        else:
                            lines.append(f":- selected({p},{t-1},{id_p}), selected({p},{t},{id_c}).")
                        vel_count += 1
    
    # Acceleration nogoods (optional)
    if cfg.use_accel:
        ma2 = cfg.max_accel * cfg.max_accel + 1.0
        for p in objects:
            for t in range(1, cfg.max_time):
                prev = cands[(p, t-1)]
                curr = cands[(p, t)]
                next_ = cands[(p, t+1)]
                for id_p, px, py, pz in prev:
                    for id_c, cx, cy, cz in curr:
                        d1 = (px-cx)**2 + (py-cy)**2 + (pz-cz)**2
                        for id_n, nx, ny, nz in next_:
                            d2 = (cx-nx)**2 + (cy-ny)**2 + (cz-nz)**2
                            if abs(d1 - d2) > ma2:
                                if t == 1:
                                    lines.append(f":- selected({p},{t},{id_c}), "
                                                f"selected({p},{t+1},{id_n}), "
                                                f"init_x({p},{px}), init_y({p},{py}), init_z({p},{pz}).")
                                else:
                                    lines.append(f":- selected({p},{t-1},{id_p}), "
                                                f"selected({p},{t},{id_c}), "
                                                f"selected({p},{t+1},{id_n}).")
                                accel_count += 1
    
    # Distance constraint nogoods
    for i, con in enumerate(constraints):
        t = con.get('time', 0)
        if con['type'] == 'dist_range':
            k, l, d1, d2 = con['k'], con['l'], con['d1'], con['d2']
            d1_2, d2_2 = d1*d1, d2*d2
            
            if t == 0:
                # Positions are fixed at t=0 — check directly in Python
                kx, ky, kz = lookup[(k, 0, 0)]
                lx, ly, lz = lookup[(l, 0, 0)]
                sd = (kx-lx)**2 + (ky-ly)**2 + (kz-lz)**2
                if sd < d1_2 or sd > d2_2:
                    # Constraint violated by fixed positions → make program UNSAT
                    # Using direct contradiction: fail is true, and :- fail eliminates all answer sets
                    lines.append("fail.")
                    lines.append(":- fail.")
                    dist_count += 1
            else:
                k_layer = cands[(k, t)]
                l_layer = cands[(l, t)]
                for id_k, kx, ky, kz in k_layer:
                    for id_l, lx, ly, lz in l_layer:
                        sd = (kx-lx)**2 + (ky-ly)**2 + (kz-lz)**2
                        if sd < d1_2 or sd > d2_2:
                            lines.append(f":- selected({k},{t},{id_k}), selected({l},{t},{id_l}).")
                            dist_count += 1
    
    lines.append("#show xpos/3. #show ypos/3. #show zpos/3.")
    
    prog = "\n".join(lines)
    stats = {'vel_nogoods': vel_count, 'accel_nogoods': accel_count, 'dist_nogoods': dist_count}
    return prog, stats

class BenchmarkGenerator:
    def __init__(self, cfg: SolverConfig):
        self.cfg = cfg; self.rng = random.Random(cfg.seed)
    
    def gen_trajectory(self, n_obj):
        cfg = self.cfg; mv = int(cfg.max_speed)
        traj = {}
        for k in range(1, n_obj+1):
            x0 = self.rng.randint(1, cfg.grid_x)
            y0 = self.rng.randint(1, cfg.grid_y)
            z0 = self.rng.randint(1, cfg.grid_z)
            pts = [(x0, y0, z0)]
            for t in range(1, cfg.max_time+1):
                cands = reachable_from(pts[-1][0], pts[-1][1], pts[-1][2],
                                       cfg.max_speed, cfg.grid_x, cfg.grid_y, cfg.grid_z)
                if cands:
                    valid = []
                    if len(pts) >= 2 and cfg.use_accel:
                        px, py, pz = pts[-2]; cx, cy, cz = pts[-1]
                        prev_sq = (px-cx)**2 + (py-cy)**2 + (pz-cz)**2
                        ma2 = cfg.max_accel**2
                        for nx, ny, nz in cands:
                            cur_sq = (cx-nx)**2 + (cy-ny)**2 + (cz-nz)**2
                            if abs(cur_sq - prev_sq) <= ma2+1:
                                valid.append((nx, ny, nz))
                    else:
                        valid = cands
                    pts.append(self.rng.choice(valid if valid else cands))
                else:
                    pts.append(pts[-1])
            traj[k] = pts
        return traj
    
    def gen_consistent(self, n_obj, n_cons):
        traj = self.gen_trajectory(n_obj)
        init = {k: traj[k][0] for k in traj}
        pairs = list(combinations(range(1, n_obj+1), 2))
        cons = []; cid = 0
        for t in range(self.cfg.max_time+1):
            n_per_t = max(1, n_cons // (self.cfg.max_time+1))
            for k, l in self.rng.sample(pairs, min(n_per_t, len(pairs))):
                x1,y1,z1 = traj[k][t]; x2,y2,z2 = traj[l][t]
                d = int(math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2))
                cons.append({'type':'dist_range','k':k,'l':l,'d1':max(0,d-2),'d2':d+3,'time':t})
                cid += 1
                if cid >= n_cons: return list(range(1,n_obj+1)), cons, init, traj
        return list(range(1,n_obj+1)), cons[:n_cons], init, traj
    
    def gen_inconsistent(self, n_obj, n_cons):
        objs, cons, init, traj = self.gen_consistent(n_obj, n_cons)
        if n_obj >= 2:
            k, l = 1, 2
            t = self.rng.randint(0, self.cfg.max_time)
            cons = [c for c in cons if not (c.get('k')==k and c.get('l')==l and c.get('time')==t)]
            # Insert contradictory pair at FRONT so truncation doesn't drop them
            cons = [
                {'type':'dist_range','k':k,'l':l,'d1':0,'d2':1,'time':t},
                {'type':'dist_range','k':k,'l':l,'d1':20,'d2':50,'time':t}
            ] + cons
        return objs, cons[:n_cons], init, traj

class THOPASolver:
    def __init__(self, cfg: SolverConfig):
        self.cfg = cfg; self.gen = BenchmarkGenerator(cfg)
    
    def solve_one(self, objects, constraints, init_pos, iid, ctype):
        result = InstanceResult(iid=iid, n_obj=len(objects),
            n_cons=len(constraints), grid=f"{self.cfg.grid_x}x{self.cfg.grid_y}x{self.cfg.grid_z}",
            max_time=self.cfg.max_time, config_type=ctype)
        
        cands, lookup = build_candidates(init_pos, objects, self.cfg)
        prog, stats = make_asp_program(cands, lookup, objects, constraints, self.cfg)
        
        try:
            ctl = clingo.Control(['0', '--stats=2', '--opt-mode=optN'])
            ctl.configuration.solve.models = 1
            ctl.add('base', [], prog)
            t0 = time.time()
            ctl.ground([('base', [])])
            t1 = time.time()
            result.ground_time = t1 - t0
            
            models = []
            def on_model(m):
                models.append(m)
                return False
            sr = ctl.solve(on_model=on_model)
            t2 = time.time()
            result.solve_time = t2 - t1
            result.total_time = t2 - t0
            result.models = len(models)
            result.sat = sr.satisfiable if sr else False
            
            st = ctl.statistics
            result.atoms = int(st.get('problem',{}).get('lp',{}).get('atoms',0))
            result.rules = int(st.get('problem',{}).get('lp',{}).get('rules',0))
        except Exception as e:
            result.error = str(e)[:200]; result.total_time = 999
        
        return result, stats
    
    def run_benchmark(self, n_instances, n_obj, n_cons, label=""):
        cres, ires = [], []
        for i in range(n_instances):
            self.gen.rng = random.Random(self.cfg.seed + i)
            objs, cons, init, _ = self.gen.gen_consistent(n_obj, n_cons)
            r, s = self.solve_one(objs, cons, init, f"{label}_C_{i}", "consistent")
            cres.append(r)
            objs, cons, init, _ = self.gen.gen_inconsistent(n_obj, n_cons)
            r, s = self.solve_one(objs, cons, init, f"{label}_I_{i}", "inconsistent")
            ires.append(r)
            if (i+1) % 10 == 0:
                c_sat = sum(1 for r in cres if r.sat)
                i_sat = sum(1 for r in ires if r.sat)
                c_avg = statistics.mean([r.total_time for r in cres[-10:]])
                i_avg = statistics.mean([r.total_time for r in ires[-10:]])
                print(f"  [{i+1:3d}/{n_instances}] C: SAT={c_sat}/{len(cres)} "
                      f"t={c_avg:.2f}s | I: SAT={i_sat}/{len(ires)} t={i_avg:.2f}s")
        
        def summarize(rs):
            if not rs: return {}
            ts = [r.total_time for r in rs]; gs = [r.ground_time for r in rs]
            sc = sum(1 for r in rs if r.sat); n = len(rs)
            return {'n':n,'sat_n':sc,'sat_pct':f"{100*sc/n:.1f}%",
                't_mean':f"{statistics.mean(ts):.2f}",'t_max':f"{max(ts):.2f}",
                'gnd_mean':f"{statistics.mean(gs):.2f}",
                'atoms_mean':f"{statistics.mean([r.atoms for r in rs]):.0f}",
                'rules_mean':f"{statistics.mean([r.rules for r in rs]):.0f}"}
        
        return {'config':label,'params':{'grid':f"{self.cfg.grid_x}x{self.cfg.grid_y}x{self.cfg.grid_z}",
            'max_time':self.cfg.max_time,'n_obj':n_obj,'n_cons':n_cons,
            'max_speed':self.cfg.max_speed},
            'consistent':summarize(cres),'inconsistent':summarize(ires)}

if __name__ == '__main__':
    print("="*60)
    print("Temporal-HOPA Solver v3.1 — Sparse Nogoods")
    print("="*60)
    
    # Quick test
    cfg = SolverConfig(grid_x=8, grid_y=8, grid_z=4, max_time=2,
                       max_speed=3.0, max_accel=2.0, seed=42, use_accel=False)
    solver = THOPASolver(cfg)
    
    print(f"\nGrid={cfg.grid_x}x{cfg.grid_y}x{cfg.grid_z}, T={cfg.max_time}, "
          f"vmax={cfg.max_speed}, accel={'ON' if cfg.use_accel else 'OFF'}")
    
    print("\n--- Test 1: 2obj consistent ---")
    objs, cons, init, _ = solver.gen.gen_consistent(2, 4)
    r, s = solver.solve_one(objs, cons, init, "t1", "consistent")
    print(f"  SAT={r.sat} gnd={r.ground_time:.3f}s sol={r.solve_time:.3f}s "
          f"tot={r.total_time:.3f}s atoms={r.atoms} rules={r.rules}")
    print(f"  Nogoods: vel={s['vel_nogoods']} accel={s['accel_nogoods']} dist={s['dist_nogoods']}")
    
    print("\n--- Test 2: 2obj inconsistent ---")
    objs, cons, init, _ = solver.gen.gen_inconsistent(2, 4)
    r, s = solver.solve_one(objs, cons, init, "t2", "inconsistent")
    print(f"  SAT={r.sat} gnd={r.ground_time:.3f}s sol={r.solve_time:.3f}s "
          f"tot={r.total_time:.3f}s atoms={r.atoms} rules={r.rules}")
    print(f"  Nogoods: vel={s['vel_nogoods']} accel={s['accel_nogoods']} dist={s['dist_nogoods']}")
    
    print("\n--- Test 3: 4obj, 8cons, 3timesteps ---")
    cfg2 = SolverConfig(grid_x=10, grid_y=10, grid_z=5, max_time=3,
                        max_speed=4.0, max_accel=3.0, seed=123, use_accel=False)
    solver2 = THOPASolver(cfg2)
    objs, cons, init, _ = solver2.gen.gen_consistent(4, 8)
    r, s = solver2.solve_one(objs, cons, init, "t3", "consistent")
    print(f"  SAT={r.sat} gnd={r.ground_time:.3f}s sol={r.solve_time:.3f}s "
          f"tot={r.total_time:.3f}s atoms={r.atoms} rules={r.rules}")
    print(f"  Nogoods: vel={s['vel_nogoods']} accel={s['accel_nogoods']} dist={s['dist_nogoods']}")
    
    print("\nSolver v3.1 ready.")
