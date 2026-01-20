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

| Gate                         | Target                         | Status | Notes |
| ---------------------------- | ------------------------------ | ------ | ----- |
| Narrow-phase speedup         | >= best backend                | TBD    |       |
| Distance speedup             | >= best backend                | TBD    |       |
| Scenario throughput          | >= best backend                | TBD    |       |
| Cross-backend correctness    | No blocking mismatches         | TBD    |       |
| Scale sweep stability        | No regressions across scales   | TBD    |       |

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
