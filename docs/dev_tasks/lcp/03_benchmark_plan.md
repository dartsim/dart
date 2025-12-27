# LCP Solver Benchmark Plan

This plan defines the benchmark metrics, fairness constraints, and harness
structure for apples-to-apples LCP solver comparisons.

## Metrics

- Wall time per solve (primary).
- Iterations reported by solver (if applicable).
- Residual and complementarity at exit (for validation and comparison).
- Success rate (fraction of runs meeting the contract tolerances).

## Fairness Controls

- Same problem instances and seeds for all solvers.
- Same `LcpOptions` contract profile across solvers:
  - `maxIterations` fixed per category.
  - `absoluteTolerance`, `relativeTolerance`, `complementarityTolerance`
    consistent with the comparison contract.
  - `warmStart = false`, `validateSolution = false`.
- No solver-specific early termination unless specified by the contract.
- Fixed number of warm-up iterations for each benchmark.
- Repeat count and iteration count reported by Google Benchmark.
- Run on Release builds (`-DCMAKE_BUILD_TYPE=Release`) with consistent flags.
- Record machine info (CPU model, cores, OS, build flags) in the benchmark log.

## Benchmark Categories (Aligned With Test Matrix)

1) Standard SPD: sizes 12, 24, 48, 96.
2) Boxed (active bounds): sizes 12, 24, 48.
3) Friction-index: contact groups 4, 16, 64 (3 vars per group).
4) Scaling/conditioning: scaled versions of standard SPD (1e-6, 1e6).
5) Large dense baseline: size 250 (local only; not for CI).

## Output Format

Use Google Benchmark's built-in JSON/CSV output:

```
--benchmark_out=benchmarks/lcp.json
--benchmark_out_format=json
```

Required columns/fields:
- `name` (solver + problem + size)
- `cpu_time` / `real_time`
- custom counters for `iterations`, `residual`, `complementarity`

## CI-Friendly Smoke Benchmark

- A single small-size run (e.g., standard SPD size 12) with short repetition.
- Execute locally with:
  - `--benchmark_filter=BM_LCP_COMPARE_SMOKE`
  - `--benchmark_min_time=0.01`
  - `--benchmark_repetitions=3`

## Notes

- For ill-conditioned or degenerate categories, report both strict and relaxed
  tolerance pass rates. Use the contract’s tolerance scaling.
- Benchmark harness should reuse the same fixture generators as tests to avoid
  divergence between correctness and performance comparisons.
