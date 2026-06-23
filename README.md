# Temporal-HOPA: Dynamic 3D Spatial-Temporal Constraint Reasoning with Answer Set Programming

This repository accompanies the paper **"Temporal-HOPA: Dynamic 3D Spatial-Temporal Constraint Reasoning with Answer Set Programming"** (submitted to *Journal of Spatial Science*, Taylor & Francis).

Temporal-HOPA (T-HOPA) extends ASP-based qualitative spatial reasoning to dynamic 3D domains. The framework combines direction, distance, and temporal motion dynamics (speed, acceleration, turn rate) within a single grounded constraint network, solved via Answer Set Programming (Clingo 5.8) with a **pre-computed candidate strategy** that avoids the grounding explosion typical of arithmetic-rich spatial ASP encodings.

## Key Idea

Rather than encoding spatial arithmetic directly in ASP (which produces prohibitive grounding), T-HOPA:

1. **Python pre-computes** all positions reachable within the maximum-speed sphere by frontier expansion across time steps.
2. **Sparse nogoods** are emitted only for *constraint-violating* pairs — not all combinatorial combinations.
3. **ASP selects** one position per object per time step via choice rules, while integrity constraints reject any invalid assignment.

This cuts [candidate pairs × constraints] nogoods down to ~28–48% of the full cross-product, and yields 100% classification accuracy with solving times of 0.5–14 seconds on standard hardware.

## Repository Structure

```
temporal-hopa/
├── README.md
├── requirements.txt
├── src/
│   ├── solver_v3.py              # Core solver: candidate enumeration + ASP encoding + Clingo interface
│   ├── thopa_framework.py        # Extended framework: heading/OPRA/qualitative distance support
│   ├── run_maritime.py           # Maritime vessel encounter scenario (COLREGs-inspired)
│   └── temporal_hopa.lp          # Pure-ASP template (supplementary, not used for v3 benchmarks)
└── experiments/
    ├── run_benchmarks_v3.py      # Full benchmark suite (3 configs × 15–30 instances each)
    ├── run_sensitivity_v2.py     # Sensitivity to contradiction time-step position
    ├── bench_light.py            # Lightweight benchmark runner (subset of configs)
    └── sensitivity_results_v2.json  # Measured sensitivity experiment data (15 instances)

```

## Requirements

- Python 3.9+
- [Clingo 5.8+](https://potassco.org/clingo/) with Python API (`clingo` module)
- NumPy
- Matplotlib 3.5+ (for figure generation)
- LaTeX distribution with `pdflatex` (for paper compilation)

Install Python dependencies:

```bash
pip install numpy matplotlib
```

Clingo must be installed separately and its Python module must be importable. On Ubuntu/Debian:

```bash
sudo apt install gringo
pip install clingo
```

On macOS:

```bash
brew install clingo
```

## Quick Start

### 1. Run the core benchmark

```bash
cd experiments
python run_benchmarks_v3.py
```

This runs three benchmark configurations (Simple: 2 objects, Medium: 4 objects, Temporal: 4 objects with T=3), each with 15–30 consistent and inconsistent random instances. Output goes to `experiments/results_v3.json`.

### 2. Run the sensitivity experiment

```bash
cd experiments
python run_sensitivity_v2.py
```

Tests how solver performance degrades as a contradictory constraint pair is placed at earlier vs. later time steps. 5 seeds × 3 positions = 15 instances. Output: `experiments/sensitivity_results_v2.json` (pre-computed data included in the repo).

### 3. Run the maritime scenario

```bash
cd src
python run_maritime.py
```

Two vessels (Voyager, Horizon) on a head-on encounter course in a 12×12×4 km grid with T=4 time steps. Demonstrates safe-separation and contradictory-constraint detection.



## Benchmark Configurations

| Config | Objects | Constraints | Grid | T | v_max | Instances | Time (mean) |
|--------|---------|-------------|------|---|-------|-----------|-------------|
| Simple (S_2×4_T2) | 2 | 4 | 8×8×4 | 2 | 3.0 | 30 | 0.46 s |
| Medium (M_4×8_T2) | 4 | 8 | 10×10×5 | 2 | 4.0 | 20 | 4.42 s |
| Temporal (T_4×8_T3) | 4 | 8 | 10×10×5 | 3 | 4.0 | 15 | 11.5 s |

All configurations achieve **100% classification accuracy** (consistent vs. inconsistent instances correctly distinguished). Solving is **grounding-dominated** (85–96% of total time spent in Clingo's grounder).

## Sensitivity Results

Contradiction placement effect (Medium config, 4 objects, T=2):

| Contradiction at | Total Time | Ground Rules | Distance Nogoods |
|-----------------|------------|--------------|-------------------|
| t=0 (fixed position) | 1.26 s | 20,490 | 5,270 |
| t=1 (one step) | 1.79 s | 170,710 | 33,299 |
| t=2 (final step) | 4.96 s | 434,295 | 296,884 |

Nogood and rule counts grow superlinearly (~O(v_max³ · t³)) with the contradiction's time step, reflecting the quadratic expansion of candidate frontiers.

## Maritime Case Study

COLREGs-inspired head-on vessel encounter:
- **Grid**: 12×12×4 km, **T**: 4 time steps (20 min), **v_max**: 3 km/step (~20 knots)
- **Safe scenario**: d ∈ [2, 12] km — correctly classified SAT
- **Unsafe scenario**: contradictory d ∈ [0, 1] AND d ∈ [8, 12] at t=3 — correctly classified UNSAT

## Architecture

```
T-HOPA Instance          Python Pre-processing       ASP Solver (Clingo 5.8)
(Objects V, Time T,      (Candidate Generation       (Choice Rules + Integrity
 3D Domain D,             + Sparse Nogood             Constraints → Answer
 Constraints C)           Encoding)                   Sets)
      │                          │                          │
      └──── instance ──────────►├──── nogoods ────────────►│
                                 │                          │
                                 │         ◄─── solve ──────┤
                                 │                          │
                                 ▼
                              Output
                         (SAT/UNSAT + 3D Trajectories)
```

## Citation

If you use this code or data in your research, please cite:

```
Temporal-HOPA: Dynamic 3D Spatial-Temporal Constraint Reasoning
with Answer Set Programming. Journal of Spatial Science, 202X.
```

## License

This code is made available for research and reproducibility purposes. Please contact the authors for usage terms.

