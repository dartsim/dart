# Experimental Collision Benchmark Results

> **Last Updated**: 2026-01-19

## Purpose

Maintain a living record of benchmark results and gate status for the
experimental collision module. Update this file after each structured suite
run, and keep the most recent results at the top.

## Run Configuration Template

- **Date**: YYYY-MM-DD
- **Branch / Commit**: `<branch>` / `<sha>`
- **Build**: `Release` / `RelWithDebInfo`
- **CPU**: `<model>`
- **OS**: `<version>`
- **Compiler**: `<compiler>`
- **Notes**: `<flags, power settings, thermal mode>`
- **Command**:
  - `pixi run build`
  - `<benchmark command>`
- **Raw Output**: `<path to benchmark JSON/CSV>`

## Gates (Must Pass)

| Gate                      | Target                       | Status | Notes |
| ------------------------- | ---------------------------- | ------ | ----- |
| Narrow-phase speedup      | >= best backend              | TBD    |       |
| Distance speedup          | >= best backend              | TBD    |       |
| Scenario throughput       | >= best backend              | TBD    |       |
| Cross-backend correctness | No blocking mismatches       | TBD    |       |
| Scale sweep stability     | No regressions across scales | TBD    |       |

## Latest Results (Summary)

| Suite / Case                          | Experimental | FCL | Bullet | ODE | Speedup | Notes |
| ------------------------------------- | ------------ | --- | ------ | --- | ------- | ----- |
| Narrow-phase edge cases (scale sweep) | TBD          | TBD | TBD    | TBD | TBD     |       |
| Distance edge cases (scale sweep)     | TBD          | TBD | TBD    | TBD | TBD     |       |
| Mixed primitives (dense/sparse)       | TBD          | TBD | TBD    | TBD | TBD     |       |
| Mesh-heavy scenario                   | TBD          | TBD | TBD    | TBD | TBD     |       |
| Batched raycasts                      | TBD          | TBD | TBD    | TBD | TBD     |       |

## Result History (Keep Brief)

| Date       | Commit | Summary                          | Notes |
| ---------- | ------ | -------------------------------- | ----- |
| 2026-01-19 | TBD    | Baseline results (pre-structure) |       |

## Run 2026-01-19 — Pipeline breakdown

Raw Output: docs/dev_tasks/experimental_collision/results/bm_pipeline_breakdown_2026-01-19.json
Build dir: build
Summary:

```
BM_Scenario_PipelineBreakdown_Dense_Experimental/1000: aabb_ns=52482.5 broadphase_ns=8879334.5 narrowphase_ns=102131804.33333333 merge_ns=7131.333333333333 pairs=70988.0 contacts=10000.0
BM_Scenario_PipelineBreakdown_Dense_Experimental/10000: aabb_ns=526647.0 broadphase_ns=916786881.0 narrowphase_ns=13924391607.0 merge_ns=96570.0 pairs=6935040.0 contacts=100000.0
BM_Scenario_PipelineBreakdown_Sparse_Experimental/1000: aabb_ns=57942.34951456311 broadphase_ns=6258597.95145631 narrowphase_ns=393491.23300970875 merge_ns=426.9611650485437 pairs=841.0 contacts=427.0
BM_Scenario_PipelineBreakdown_Sparse_Experimental/10000: aabb_ns=509226.0 broadphase_ns=648132646.0 narrowphase_ns=2234726435.0 merge_ns=32116.0 pairs=82932.0 contacts=41125.0
```
