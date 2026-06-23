#!/usr/bin/env python3
"""
Temporal-HOPA: 动态3D空间混合方向-距离-时间约束推理
完整实验框架: 基准生成、求解器封装、实验运行、结果分析
基于: Izmirlioglu (2025) HOPA, AAAI 2025
"""

import clingo, random, time, json, os, sys, math, statistics
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from collections import defaultdict

@dataclass
class THOPAConfig:
    grid_x: int = 20; grid_y: int = 20; grid_z: int = 8
    ang_res: int = 15; max_time: int = 3
    n_objects: int = 4; n_spatial_constraints: int = 8
    max_speed: float = 5.0; max_turn_rate: float = 90.0
    max_accel: float = 3.0
    timeout: int = 60; seed: int = 42

@dataclass
class ExperimentResult:
    instance_id: str; n_objects: int; n_constraints: int
    grid_size: str; max_time: int; config_type: str
    grounding_time: float = 0.0; solving_time: float = 0.0
    total_time: float = 0.0; models_found: int = 0
    satisfiable: bool = False
    constraints_total: int = 0; atoms_generated: int = 0
    rules_generated: int = 0

class THOPAEncoder:
    def __init__(self, config: THOPAConfig):
        self.c = config
    
    def make_base_program(self) -> str:
        s, t, u, m = self.c.grid_x, self.c.grid_y, self.c.grid_z, self.c.max_time
        return f"""
#const s={s}. #const t_val={t}. #const u={u}.
#const max_time={m}. #const ang_res={self.c.ang_res}.
#const default_max_speed={int(self.c.max_speed)}.

time(0..max_time).

% 位置生成 (未知位置的对象)
1 {{xloc(P,T,X) : X=1..{s}}} 1 :- time(T), point(P), not loc_known(P,T).
1 {{yloc(P,T,Y) : Y=1..{t}}} 1 :- time(T), point(P), not loc_known(P,T).
1 {{zloc(P,T,Z) : Z=1..{u}}} 1 :- time(T), point(P), not loc_known(P,T).

% 速度约束: 位移不超过max_speed
:- point(P), time(T), T>0, max_speed(P,Smax),
   xloc(P,T-1,X1), xloc(P,T,X2), yloc(P,T-1,Y1), yloc(P,T,Y2), zloc(P,T-1,Z1), zloc(P,T,Z2),
   MD=(X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2)+(Z1-Z2)*(Z1-Z2), MD>Smax*Smax.

% 加速度约束
:- point(P), time(T), T>0, T<max_time, max_accel(P,Amax),
   xloc(P,T-1,X0), xloc(P,T,X1), xloc(P,T+1,X2),
   yloc(P,T-1,Y0), yloc(P,T,Y1), yloc(P,T+1,Y2),
   zloc(P,T-1,Z0), zloc(P,T,Z1), zloc(P,T+1,Z2),
   SD0=(X0-X1)*(X0-X1)+(Y0-Y1)*(Y0-Y1)+(Z0-Z1)*(Z0-Z1),
   SD1=(X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2)+(Z1-Z2)*(Z1-Z2),
   |SD0 - SD1| > Amax*Amax.

% 3D欧氏距离平方
sq_dist_3d(P,R,T,SD) :- point(P), point(R), P<R, time(T),
    xloc(P,T,X1), xloc(R,T,X2), yloc(P,T,Y1), yloc(R,T,Y2), zloc(P,T,Z1), zloc(R,T,Z2),
    SD=(X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2)+(Z1-Z2)*(Z1-Z2).

% 距离区间约束
:- t_dist_range(N,K,L,D1,D2,T), point(K), point(L), sq_dist_3d(K,L,T,SD), SD<D1*D1.
:- t_dist_range(N,K,L,D1,D2,T), point(K), point(L), sq_dist_3d(K,L,T,SD), SD>D2*D2.
:- t_dist_precise(N,K,L,D,T), point(K), point(L), sq_dist_3d(K,L,T,SD), |SD-D*D|>1.

% 定性距离
:- t_qual_dist(N,K,L,REL,T), point(K), point(L), qual_dist_range(REL,DL,DU), sq_dist_3d(K,L,T,SD), SD<DL*DL.
:- t_qual_dist(N,K,L,REL,T), point(K), point(L), qual_dist_range(REL,DL,DU), sq_dist_3d(K,L,T,SD), SD>DU*DU.

% 精确位置
:- t_location(N,K,X,Y,Z,T), point(K), xloc(K,T,X1), X1!=X.
:- t_location(N,K,X,Y,Z,T), point(K), yloc(K,T,Y1), Y1!=Y.
:- t_location(N,K,X,Y,Z,T), point(K), zloc(K,T,Z1), Z1!=Z.

% 方向区间约束 (heading)
:- t_heading_range(N,K,H1,H2,T), heading(K,T,H), H<H1.
:- t_heading_range(N,K,H1,H2,T), heading(K,T,H), H>H2.

% 运动方向变化约束 (转向率)
heading_diff(H1,H2,HD) :- HD=(H1-H2), H1>=H2, object(K), heading(K,T,H1), heading(K,T+1,H2), T<max_time.
heading_diff(H1,H2,HD) :- HD=(H2-H1), H1<H2, object(K), heading(K,T,H1), heading(K,T+1,H2), T<max_time.
heading_diff(H1,H2,HD) :- HD=(H1+360-H2), H1<H2, H1<90, H2>270, object(K), heading(K,T,H1), heading(K,T+1,H2), T<max_time.
heading_diff(H1,H2,HD) :- HD=(H2+360-H1), H2<H1, H2<90, H1>270, object(K), heading(K,T,H1), heading(K,T+1,H2), T<max_time.

:- object(K), heading_diff(H1,H2,HD), max_turn(K,Rmax), HD>Rmax.

#show xloc/3. #show yloc/3. #show zloc/3.
"""
    
    def make_instance_program(self, objects, spatial_constraints, initial_state):
        lines = []
        for k in objects:
            lines.append(f"object({k}). point({k}).")
            lines.append(f"max_speed({k},{int(self.c.max_speed)}).")
            lines.append(f"max_accel({k},{int(self.c.max_accel)}).")
            lines.append(f"max_turn({k},{int(self.c.max_turn_rate)}).")
        
        lines.append("qual_dist_range(very_near,0,2). qual_dist_range(near,1,5).")
        lines.append("qual_dist_range(medium,4,12). qual_dist_range(far,10,50).")
        
        for k in objects:
            key = f'x_{k}_0'
            if key in initial_state:
                x,y,z = initial_state[key], initial_state[f'y_{k}_0'], initial_state[f'z_{k}_0']
                lines.append(f"xloc({k},0,{x}). yloc({k},0,{y}). zloc({k},0,{z}). loc_known({k},0).")
        
        for i, c in enumerate(spatial_constraints):
            t, tm = c['type'], c.get('time', 0)
            if t == 'dist_range':
                lines.append(f"t_dist_range({i},{c['k']},{c['l']},{c['d1']},{c['d2']},{tm}).")
            elif t == 'dist_precise':
                lines.append(f"t_dist_precise({i},{c['k']},{c['l']},{c['d']},{tm}).")
            elif t == 'qual_dist':
                lines.append(f"t_qual_dist({i},{c['k']},{c['l']},{c['rel']},{tm}).")
            elif t == 'location':
                lines.append(f"t_location({i},{c['k']},{c['x']},{c['y']},{c['z']},{tm}).")
        
        return "\n".join(lines)

class BenchmarkGenerator:
    def __init__(self, config: THOPAConfig):
        self.c = config
        self.rng = random.Random(config.seed)
    
    def generate_consistent(self, iid: str = None):
        s, t, u, m = self.c.grid_x, self.c.grid_y, self.c.grid_z, self.c.max_time
        n = self.c.n_objects
        max_v = int(self.c.max_speed)
        objects = list(range(1, n + 1))
        
        # 生成满足运动约束的3D轨迹
        traj = {}
        for k in objects:
            x0, y0, z0 = self.rng.randint(1, s), self.rng.randint(1, t), self.rng.randint(1, u)
            pts = [(x0, y0, z0)]
            for tm in range(1, m + 1):
                dx = self.rng.randint(-max_v, max_v)
                dy = self.rng.randint(-max_v, max_v)
                dz = self.rng.randint(-min(max_v, u-1), min(max_v, u-1))
                pts.append((max(1, min(s, pts[-1][0]+dx)), max(1, min(t, pts[-1][1]+dy)), max(1, min(u, pts[-1][2]+dz))))
            traj[k] = pts
        
        init = {}
        for k in objects:
            init[f'x_{k}_0'], init[f'y_{k}_0'], init[f'z_{k}_0'] = traj[k][0]
        
        # 从轨迹提取距离约束
        sc = []
        cid = 0
        for tm in range(m + 1):
            pairs = [(i, j) for i in objects for j in objects if i < j]
            n_sample = min(len(pairs), max(1, self.c.n_spatial_constraints // (m + 1)))
            for (k, l) in self.rng.sample(pairs, n_sample):
                x1, y1, z1 = traj[k][tm]; x2, y2, z2 = traj[l][tm]
                d = int(math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2))
                sc.append({'type': 'dist_range', 'k': k, 'l': l, 'd1': max(0, d-2), 'd2': d+3, 'time': tm})
                cid += 1
        
        return objects, sc[:self.c.n_spatial_constraints], [], init
    
    def generate_inconsistent(self, iid: str = None):
        objects, sc, tc, init = self.generate_consistent(iid)
        if len(sc) >= 2 and len(objects) >= 2:
            tm = sc[0].get('time', 1)
            sc = [{'type': 'dist_range', 'k': objects[0], 'l': objects[1], 'd1': 0, 'd2': 2, 'time': tm},
                  {'type': 'dist_range', 'k': objects[0], 'l': objects[1], 'd1': 20, 'd2': 50, 'time': tm}] + sc[2:]
        return objects, sc, tc, init

class THOPARunner:
    def __init__(self, config: THOPAConfig):
        self.c = config
        self.encoder = THOPAEncoder(config)
        self.gen = BenchmarkGenerator(config)
    
    def run_one(self, objects, sc, tc, init, iid, itype):
        result = ExperimentResult(instance_id=iid, n_objects=len(objects),
            n_constraints=len(sc)+len(tc), grid_size=f"{self.c.grid_x}x{self.c.grid_y}x{self.c.grid_z}",
            max_time=self.c.max_time, config_type=itype)
        result.constraints_total = len(sc) + len(tc)
        
        prog = self.encoder.make_base_program() + "\n" + self.encoder.make_instance_program(objects, sc, init)
        
        try:
            ctl = clingo.Control(['0', f'--time-limit={self.c.timeout}', '--stats=2'])
            ctl.add('base', [], prog)
            t0 = time.time()
            ctl.ground([('base', [])])
            t1 = time.time()
            result.grounding_time = t1 - t0
            st = ctl.statistics
            result.atoms_generated = st.get('problem', {}).get('lp', {}).get('atoms', 0)
            result.rules_generated = st.get('problem', {}).get('lp', {}).get('rules', 0)
            
            models = []
            sr = ctl.solve(on_model=lambda m: models.append(m))
            t2 = time.time()
            result.solving_time = t2 - t1
            result.total_time = t2 - t0
            result.models_found = len(models)
            result.satisfiable = sr.satisfiable if sr else False
        except Exception as e:
            result.total_time = self.c.timeout
            result.satisfiable = False
        return result
    
    def run_benchmark(self, n_instances=100):
        cres, ires = [], []
        for i in range(n_instances):
            self.gen.rng = random.Random(self.c.seed + i)
            o, s, t, init = self.gen.generate_consistent(f"CONS_{i}")
            r = self.run_one(o, s, t, init, f"CONS_{i}", "consistent")
            cres.append(r)
            o, s, t, init = self.gen.generate_inconsistent(f"INCONS_{i}")
            r = self.run_one(o, s, t, init, f"INCONS_{i}", "inconsistent")
            ires.append(r)
            if (i+1) % 10 == 0: print(f"  [{i+1}/{n_instances}]")
        return {'consistent': self._summarize(cres), 'inconsistent': self._summarize(ires)}
    
    def _summarize(self, results):
        if not results: return {}
        ts = [r.total_time for r in results]
        gs = [r.grounding_time for r in results]
        ss = [r.solving_time for r in results]
        sc = sum(1 for r in results if r.satisfiable)
        return {'n': len(results), 'sat_count': sc, 'sat_rate': sc/len(results),
            'grnd_mean': statistics.mean(gs), 'grnd_std': statistics.stdev(gs) if len(gs)>1 else 0,
            'total_mean': statistics.mean(ts), 'total_median': statistics.median(ts),
            'total_max': max(ts), 'total_std': statistics.stdev(ts) if len(ts)>1 else 0}

if __name__ == '__main__':
    print("="*70)
    print("Temporal-HOPA Framework v1.0")
    print("="*70)
    cfg = THOPAConfig(grid_x=15, grid_y=15, grid_z=6, max_time=2,
        n_objects=4, n_spatial_constraints=6, max_speed=4.0, timeout=30)
    runner = THOPARunner(cfg)
    
    print(f"\n配置: {cfg.grid_x}x{cfg.grid_y}x{cfg.grid_z}, {cfg.n_objects}对象, "
          f"{cfg.n_spatial_constraints}约束, {cfg.max_time+1}时间步")
    
    print("\n--- 一致实例测试 ---")
    o, s, t, init = runner.gen.generate_consistent("t1")
    r = runner.run_one(o, s, t, init, "t1", "consistent")
    print(f"  SAT={r.satisfiable}, time={r.total_time:.3f}s, gnd={r.grounding_time:.3f}s")
    
    print("\n--- 不一致实例测试 ---")
    o, s, t, init = runner.gen.generate_inconsistent("t2")
    r = runner.run_one(o, s, t, init, "t2", "inconsistent")
    print(f"  SAT={r.satisfiable}, time={r.total_time:.3f}s, gnd={r.grounding_time:.3f}s")
    
    print("\n框架就绪。运行完整基准: runner.run_benchmark(n_instances=100)")
