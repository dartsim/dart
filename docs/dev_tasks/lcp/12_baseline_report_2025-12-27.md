# LCP Solver Comparison Report (Baseline)

## Summary

- Solver name: Dantzig (baseline run)
- Commit / branch: ad1cdeb675e / feature/lcp_solver_plan
- Build type and flags: Release (pixi run config defaults)
- Machine (CPU, cores, OS): AMD Ryzen Threadripper 3970X 32-Core Processor (64
  logical), Linux 6.8.0-90-generic
- Notes on determinism (seed, randomized ordering): Fixed seeds in
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`

## Contract Settings

- `maxIterations` (per solver): 100 (Dantzig)
- `absoluteTolerance`: 1e-6
- `relativeTolerance`: 1e-4
- `complementarityTolerance`: 1e-6
- `relaxation`: 1.0
- Warm start enabled: false

## Correctness (Tests)

| Suite                  | Result  | Notes      |
| ---------------------- | ------- | ---------- |
| LCP comparison harness | NOT RUN | Benchmarks |
| LCP edge cases         | NOT RUN | Benchmarks |
| LCP stress tests       | NOT RUN | Benchmarks |

## Benchmark Results (Release Build)

Command used:

```bash
pixi run bm lcp_compare -- --benchmark_out=build/default/cpp/Release/benchmarks/lcp_compare_2025-12-27.json --benchmark_out_format=json
```

Artifact:

- `build/default/cpp/Release/benchmarks/lcp_compare_2025-12-27.json`

### Standard SPD

| Size | Solver  | Time      | Iterations | Residual | Complementarity | Bound violation | Contract OK |
| ---- | ------- | --------- | ---------- | -------- | --------------- | --------------- | ----------- |
| 12   | Dantzig | 2040 ns   | 1          | 3.55271f | 3.55271f        | 0               | 1           |
| 24   | Dantzig | 5004 ns   | 1          | 14.2109f | 14.2109f        | 0               | 1           |
| 48   | Dantzig | 18190 ns  | 1          | 42.6326f | 42.6326f        | 0               | 1           |
| 96   | Dantzig | 108473 ns | 1          | 142.109f | 142.109f        | 0               | 1           |

### Boxed (Active Bounds)

| Size | Solver  | Time     | Iterations | Residual | Complementarity | Bound violation | Contract OK |
| ---- | ------- | -------- | ---------- | -------- | --------------- | --------------- | ----------- |
| 12   | Dantzig | 2313 ns  | 1          | 666.134a | 666.134a        | 0               | 1           |
| 24   | Dantzig | 10234 ns | 1          | 3.55271f | 3.55271f        | 0               | 1           |
| 48   | Dantzig | 62561 ns | 1          | 21.3163f | 21.3163f        | 0               | 1           |

### Friction Index

| Contacts | Solver  | Time      | Iterations | Residual | Complementarity | Bound violation | Contract OK |
| -------- | ------- | --------- | ---------- | -------- | --------------- | --------------- | ----------- |
| 4        | Dantzig | 2299 ns   | 1          | 3.55271f | 3.55271f        | 0               | 1           |
| 16       | Dantzig | 19684 ns  | 1          | 42.6326f | 42.6326f        | 0               | 1           |
| 64       | Dantzig | 832958 ns | 1          | 568.434f | 568.434f        | 0               | 1           |

### Scaling

| Size | Scale | Solver  | Time    | Iterations | Residual | Complementarity | Bound violation | Contract OK |
| ---- | ----- | ------- | ------- | ---------- | -------- | --------------- | --------------- | ----------- |
| 12   | 1e-6  | Dantzig | 2051 ns | 1          | 0        | 3.38813z        | 0               | 1           |
| 12   | 1e6   | Dantzig | 2088 ns | 1          | 1.86265n | 1.86265n        | 0               | 1           |

## Notes and Follow-ups

- Regressions or anomalies: PenalizedFischerBurmeisterNewton Standard/24 reports
  `contract_ok=0` with residual/complementarity around 1.59e-2.
- Any solver-specific tuning applied: None (benchmark defaults).
- Next action items: None.
