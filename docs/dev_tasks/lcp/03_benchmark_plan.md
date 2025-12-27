# LCP Solver Benchmark Plan

This plan defines the benchmark metrics, fairness constraints, and harness
structure for apples-to-apples LCP solver comparisons.

## Metrics

- Wall time per solve (primary).
- Iterations reported by solver (if applicable).
- Residual, complementarity, and bound violation at exit (contract checks).
- Contract pass rate (`contract_ok` counter).

## Fairness Controls

- Same problem instances and seeds for all solvers (see generator functions in
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`).
- `MakeBenchmarkOptions()` provides the baseline profile:
  - `absoluteTolerance = 1e-6`, `relativeTolerance = 1e-4`,
    `complementarityTolerance = 1e-6`
  - `relaxation = 1.0`, `warmStart = false`, `validateSolution = false`,
    `earlyTermination = false`
  - `maxIterations` is set per solver entry in the benchmark file.
- Solver-specific custom options are recorded in the benchmark harness:
  - NNCG/SubspaceMinimization PGS iterations, ShockPropagation block params,
    Penalized FB lambda.
- Repeat count and iteration count reported by Google Benchmark.
- Pin threads when using threaded math backends (e.g., set `OMP_NUM_THREADS=1`,
  `MKL_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`) to reduce variance.
- Run on Release builds (`-DCMAKE_BUILD_TYPE=Release`) with consistent flags.
- Record machine info (CPU model, cores, OS, build flags) in the benchmark log.

## Benchmark Categories (Aligned With Test Matrix)

1. Standard SPD: sizes 12, 24, 48, 96 (Direct uses 2 and 3 only).
2. Boxed (active bounds): sizes 12, 24, 48.
3. Friction-index: contact groups 4, 16, 64 (3 vars per group).
4. Scaling/conditioning: size 12 scaled by 1e-6 and 1e6 (Dantzig, Pgs).
5. Smoke: size 12 standard SPD (Dantzig).

## Output Format

Use Google Benchmark's built-in JSON/CSV output:

```
--benchmark_out=benchmarks/lcp.json
--benchmark_out_format=json
```

Required columns/fields:

- `name` (solver + problem + size)
- `cpu_time` / `real_time`
- custom counters for `iterations`, `residual`, `complementarity`,
  `bound_violation`, `contract_ok`
- ShockPropagation counters: `layer_count`, `block_count`, `max_block_size`,
  `max_blocks_per_layer`

Also produce a short human summary (markdown table) using
`11_result_reporting_template.md` for comparisons shared in reviews.

## CI-Friendly Smoke Benchmark

- A single small-size run (e.g., standard SPD size 12) with short repetition.
- Execute locally with:
  - `--benchmark_filter=BM_LCP_COMPARE_SMOKE`
  - `--benchmark_min_time=0.01`
  - `--benchmark_repetitions=3`
- Suggested (Unverified):
  - `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE --benchmark_min_time=0.01 --benchmark_repetitions=3`

## Notes

- PenalizedFischerBurmeisterNewton occasionally fails the contract on standard
  SPD size 24 (contract_ok=0 with residual/complementarity around 1.6e-2).
  Call this out as a known outlier in reports.
- Benchmark generators are local to `bm_lcp_compare.cpp`; keep seeds and
  problem construction stable so comparisons remain apples-to-apples.
