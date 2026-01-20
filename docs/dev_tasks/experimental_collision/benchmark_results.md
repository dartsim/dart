# Experimental Collision Benchmark Results

> **Last Updated**: 2026-01-20

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

| Gate                      | Target                       | Status  | Notes                                                            |
| ------------------------- | ---------------------------- | ------- | ---------------------------------------------------------------- |
| Narrow-phase speedup      | >= best backend              | PASS    | 77 cases; min 2.7x, p50 7.0x, p90 11.0x vs best backend.         |
| Distance speedup          | >= best backend              | FAIL    | Experimental slower (0.02x-0.69x); ODE distance unsupported.     |
| Scenario throughput       | >= best backend              | PARTIAL | Mixed 0.06x-1.4x; mesh 3.6x-4.7x; raycast batch now runs.        |
| Cross-backend correctness | No blocking mismatches       | PARTIAL | Capsule support gaps in FCL checks; ODE distance unsupported.    |
| Scale sweep stability     | No regressions across scales | PARTIAL | Narrow-phase/distance sweeps run; pipeline ok; raycast batch ok. |

## Latest Results (Summary)

| Suite / Case                          | Experimental | FCL      | Bullet   | ODE      | Speedup                  | Notes                                                       |
| ------------------------------------- | ------------ | -------- | -------- | -------- | ------------------------ | ----------------------------------------------------------- |
| Narrow-phase edge cases (scale sweep) | see JSON     | see JSON | see JSON | see JSON | 2.7x-27x (p50 7.0x)      | 77 cases across scales/edge cases.                          |
| Distance edge cases (scale sweep)     | see JSON     | see JSON | see JSON | see JSON | 0.02x-0.69x (p50 0.086x) | Experimental slower; ODE distance unsupported (noisy logs). |
| Mixed primitives (dense/sparse)       | see JSON     | see JSON | see JSON | see JSON | 0.06x-1.4x (p50 0.35x)   | 6 cases (100/1k/10k); BigO/RMS excluded.                    |
| Mesh-heavy scenario                   | see JSON     | see JSON | see JSON | see JSON | 3.6x-4.7x                | 2 cases; mesh loader warns about empty path.                |
| Batched raycasts                      | see JSON     | N/A      | see JSON | N/A      | TBD                      | 500 rays, 1k/2k objects; SweepAndPrune for experimental.    |
| libccd microbench (GJK/EPA/MPR)       | see JSON     | N/A      | N/A      | N/A      | mixed                    | DART faster on EPA (~12.8k×), slower on GJK/MPR (1.1-2.2×). |

## Result History (Keep Brief)

| Date       | Commit      | Summary                             | Notes                                              |
| ---------- | ----------- | ----------------------------------- | -------------------------------------------------- |
| 2026-01-20 | 1fac7c64227 | libccd microbench                   | EPA faster; GJK/MPR slower vs libccd.              |
| 2026-01-20 | 3e0bd1bb41b | RP3D-aligned pipeline breakdown     | Dense/sparse spheres 1k/10k recorded.              |
| 2026-01-20 | f315999cdfe | Raycast batch + comparative raycast | SweepAndPrune; 500 rays; 1k/2k objects             |
| 2026-01-19 | TBD         | Baseline results (pre-structure)    |                                                    |
| 2026-01-19 | b1f6e5e     | Comparative + scenarios runs        | Distance/mixed underperform; raycast blocked then. |

## Run 2026-01-20 — libccd microbench

- **Branch / Commit**: `feature/new_coll` / `1fac7c64227`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - libccd from pixi env (`.pixi/envs/default`), since the local libccd CMakeLists target name collides with the already-imported `ccd` target.
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `pixi run bm bm_experimental_libccd -- --benchmark_min_time=0.05s --benchmark_format=json --benchmark_out=docs/dev_tasks/experimental_collision/results/bm_experimental_libccd_2026-01-20_012403.json`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_experimental_libccd_2026-01-20_012403.json`

Summary (speedup = libccd time / experimental time):

- GJK sphere-sphere: 0.46x (experimental 2.16x slower).
- GJK+EPA sphere-sphere: 12,763x (experimental faster).
- MPR sphere-sphere: 0.88x (experimental 1.14x slower).
- GJK box-box: 0.53x (experimental 1.88x slower).

## Run 2026-01-20 — Raycast comparative + batch scenarios

- **Branch / Commit**: `feature/new_coll` / `f315999cdfe`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - Raycast batch uses SweepAndPrune broadphase to avoid AabbTree crash.
  - Raycast batch uses 500 rays with 1k/2k objects for bounded runtime.
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `cmake --build build/default/cpp/Release --target bm_comparative_raycast bm_scenarios_raycast_batch`
  - `build/default/cpp/Release/bin/bm_comparative_raycast --benchmark_min_time=0.05s --benchmark_format=json`
  - `build/default/cpp/Release/bin/bm_scenarios_raycast_batch --benchmark_min_time=0.05s --benchmark_format=json`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_comparative_raycast_2026-01-20_000232.json`
  - `docs/dev_tasks/experimental_collision/results/bm_scenarios_raycast_batch_2026-01-20_001850.json`

## Run 2026-01-19 — Comparative narrow-phase/distance + scenarios

- **Branch / Commit**: `feature/new_coll` / `b1f6e5ebf057ff13232a0c9f1c4cf84dcdf35ed8`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Linux 6.17.0-8-generic
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - Dirty working tree; other agent changes present.
  - `bm_comparative_narrow_phase` used `--benchmark_min_time=0.05` (warning about missing suffix).
  - `bm_comparative_distance` logs include "ODE doesn't support distance".
  - `bm_scenarios_mesh_heavy` warns about `file://` mesh path.
  - Raycast batch handled in 2026-01-20 run.
- **Command**:
  - `cmake --build build/default/cpp/Release --target bm_comparative_narrow_phase bm_comparative_distance bm_scenarios_mixed_primitives bm_scenarios_mesh_heavy`
  - `build/default/cpp/Release/bm_comparative_narrow_phase --benchmark_min_time=0.05`
  - `build/default/cpp/Release/bm_comparative_distance --benchmark_min_time=0.05s`
  - `build/default/cpp/Release/bm_scenarios_mixed_primitives --benchmark_min_time=0.05s`
  - `build/default/cpp/Release/bm_scenarios_mesh_heavy --benchmark_min_time=0.05s`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_comparative_narrow_phase_2026-01-19_233015_rerun1.json`
  - `docs/dev_tasks/experimental_collision/results/bm_comparative_distance_2026-01-19_233015.json`
  - `docs/dev_tasks/experimental_collision/results/bm_scenarios_mixed_primitives_2026-01-19_233015_rerun1.json`
  - `docs/dev_tasks/experimental_collision/results/bm_scenarios_mesh_heavy_2026-01-19_233015.json`

Summary (speedup = best backend time / Experimental time):

- Narrow-phase: min 2.7x, p50 7.0x, p90 11.0x (77 cases).
- Distance: min 0.02x, p50 0.086x, p90 0.48x (66 cases; Experimental slower).
- Mixed primitives: min 0.06x, p50 0.35x, p90 0.79x (6 cases; BigO/RMS excluded).
- Mesh-heavy: 3.6x-4.7x (2 cases).
- Blockers: none in this subset; raycast handled in 2026-01-20 run.

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

## Run 2026-01-20 — Pipeline breakdown (RP3D-aligned)

- **Branch / Commit**: `feature/new_coll` / `3e0bd1bb41b`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - RP3D-aligned sphere-only dense/sparse scenes (1k/10k).
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `build/default/cpp/Release/bin/bm_scenarios_pipeline_breakdown --benchmark_filter="PipelineBreakdown_RP3D_.*" --benchmark_format=json --benchmark_min_time=0.05s`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_pipeline_breakdown_rp3d_2026-01-20_015424.json`

Summary:

```
BM_Scenario_PipelineBreakdown_RP3D_Dense_Spheres_Experimental/1000: aabb_update_ns=44155.97003745318 broadphase_ns=149967.65917602996 narrowphase_ns=76474.0 merge_ns=144.69662921348313 pairs=458.0 contacts=252.0
BM_Scenario_PipelineBreakdown_RP3D_Dense_Spheres_Experimental/10000: aabb_update_ns=421720.0 broadphase_ns=10237884.0 narrowphase_ns=656721732.0 merge_ns=32321.0 pairs=46345.0 contacts=24686.0
BM_Scenario_PipelineBreakdown_RP3D_Sparse_Spheres_Experimental/1000: aabb_update_ns=46493.759825327514 broadphase_ns=50766.985443959245 narrowphase_ns=329.29694323144105 merge_ns=16.973799126637555 pairs=4.0 contacts=4.0
BM_Scenario_PipelineBreakdown_RP3D_Sparse_Spheres_Experimental/10000: aabb_update_ns=428326.14814814815 broadphase_ns=2075412.5925925926 narrowphase_ns=75518.92592592593 merge_ns=138.1851851851852 pairs=403.0 contacts=209.0
```
