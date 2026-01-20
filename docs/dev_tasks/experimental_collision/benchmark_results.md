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

| Gate                      | Target                       | Status  | Notes                                                                              |
| ------------------------- | ---------------------------- | ------- | ---------------------------------------------------------------------------------- |
| Narrow-phase speedup      | >= best backend              | PASS    | 77 cases; min 2.75x, p50 7.01x, p90 11.04x vs best backend.                        |
| Distance speedup          | >= best backend              | FAIL    | Experimental slower (min 0.02x, p50 0.09x, p90 0.49x); Bullet/ODE distance warn.   |
| Scenario throughput       | >= best backend              | PARTIAL | Mixed 0.06x-1.40x (p50 0.42x, p90 1.10x); mesh 3.58x-4.67x; ray batch 2.17x-5.72x. |
| Cross-backend correctness | No blocking mismatches       | PARTIAL | Collision checks pass; Bullet/ODE distance unsupported.                            |
| Scale sweep stability     | No regressions across scales | PARTIAL | Narrow-phase/distance sweeps run; pipeline ok; raycast batch ok.                   |

Summary status legend: WIN = min speedup > 1.0; MIXED = min < 1.0 and p90 > 1.0; LOSS = p50 < 1.0.

## Latest Results (Summary)

Values are min/p50/p90 for each suite (iteration runs only; BigO/RMS excluded).
Latest full-suite summary is from 2026-01-19; the 2026-01-20 rerun refreshed
comparative suites and the baseline `bm_comparative` with accuracy checks
passing (cylinder-box fallback + box-box depth aligned; see run notes). The
2026-01-20 distance rerun captures the plane/cylinder distance support update.

Legend: WIN = min speedup > 1.0; MIXED = min < 1.0 and p90 > 1.0; LOSS = p50 < 1.0.

| Suite / Case                          | Unit | Experimental (min/p50/p90) | FCL (min/p50/p90)      | Bullet (min/p50/p90) | ODE (min/p50/p90)    | Speedup (min/p50/p90 vs best) | Status | Notes                                                    |
| ------------------------------------- | ---- | -------------------------- | ---------------------- | -------------------- | -------------------- | ----------------------------- | ------ | -------------------------------------------------------- |
| Narrow-phase edge cases (scale sweep) | ns   | 26.5/42.3/177.0            | 179.8/512.0/1,609.5    | 247.6/429.6/1,009.9  | 159.0/778.8/1,541.5  | 2.75x/7.01x/11.04x            | WIN    | 77 cases across scales/edge cases.                       |
| Distance edge cases (scale sweep)     | ns   | 4.7/38.4/124.7             | 224.3/409.3/28,778.3   | 2.6/3.2/3.8          | 380.1/511.7/643.9    | 0.02x/0.09x/0.49x             | LOSS   | 66 cases; ODE distance unsupported (noisy logs).         |
| Mixed primitives (dense/sparse)       | ms   | 0.03/55.42/7,854.78        | 1.52/54.84/836.05      | 0.01/52.99/11,839.86 | 0.03/43.30/14,055.96 | 0.06x/0.42x/1.10x             | MIXED  | 6 cases (100/1k/10k); BigO/RMS excluded.                 |
| Mesh-heavy scenario                   | ms   | 8.65/96.38/166.56          | 144.12/709.23/1,161.32 | 30.98/445.37/776.89  | N/A                  | 3.58x/4.13x/4.56x             | WIN    | 2 cases; mesh loader warns about empty path.             |
| Batched raycasts                      | ms   | 21.63/37.73/52.93          | N/A                    | 47.00/125.00/270.99  | N/A                  | 2.17x/3.31x/5.00x             | WIN    | 500 rays, 1k/2k objects; SweepAndPrune for experimental. |
| Comparative raycast (single)          | ns   | 23.3/52.1/66.5             | N/A                    | 186.1/476.8/731.1    | N/A                  | 6.85x/9.14x/10.99x            | WIN    | 5 cases; experimental vs Bullet only.                    |
| libccd microbench (GJK/EPA/MPR)       | ns   | 59.7/71.4/108.8            | N/A                    | N/A                  | N/A                  | 0.43x/0.53x/8,699.10x         | MIXED  | libccd 29.1/49.5/651,682.0 ns; speedup uses libccd/DART. |

### Performance Gaps (Experimental < best backend)

- Distance edge cases: slower across all percentiles (p50 0.09x).
- Mixed primitives: slower on median; only reaches parity/win at p90.
- libccd microbench: slower on GJK/MPR; EPA dominates the p90 speedup.

### Wins (Experimental >= best backend)

- Narrow-phase edge cases: strong wins across all percentiles (p50 7.01x).
- Mesh-heavy scenario: consistent wins (p50 4.13x).
- Batched raycasts: consistent wins vs Bullet (p50 3.31x).
- Comparative raycast (single): strong wins vs Bullet (p50 9.14x).

## Result History (Keep Brief)

| Date       | Commit      | Summary                              | Notes                                              |
| ---------- | ----------- | ------------------------------------ | -------------------------------------------------- |
| 2026-01-20 | c2d07524b37 | Distance rerun (plane/cylinder)      | Accuracy pass; Bullet/ODE distance unsupported.    |
| 2026-01-20 | 788a69e7bb2 | SDF/ESDF resolution sweep (extended) | Voxblox lib not found; added 80/96 dims.           |
| 2026-01-20 | be3923bf398 | SDF/ESDF resolution sweep            | Voxblox lib not found; resolution scaling logged.  |
| 2026-01-20 | db9b3f36dc2 | Cylinder-box + box-box benchmark fix | Accuracy checks pass; depth warning resolved.      |
| 2026-01-20 | e1b474aeb80 | Comparative + baseline rerun         | Capsule fallback fixed; cylinder-box mismatch.     |
| 2026-01-20 | 154e5440be3 | SDF/ESDF benchmark                   | Voxblox lib not found; ESDF build/query recorded.  |
| 2026-01-20 | d3d2174bff0 | Partial comparative + libccd rerun   | Capsule fallback; `bm_comparative` build break.    |
| 2026-01-20 | 1fac7c64227 | libccd microbench                    | EPA faster; GJK/MPR slower vs libccd.              |
| 2026-01-20 | 7bc79f0dd6b | RP3D-aligned pipeline breakdown try  | Segfault persists after rebuild; partial JSON.     |
| 2026-01-20 | f315999cdfe | Raycast batch + comparative raycast  | SweepAndPrune; 500 rays; 1k/2k objects             |
| 2026-01-19 | TBD         | Baseline results (pre-structure)     |                                                    |
| 2026-01-19 | b1f6e5e     | Comparative + scenarios runs         | Distance/mixed underperform; raycast blocked then. |

## Run 2026-01-20 — Comparative distance rerun (plane/cylinder support)

- **Branch / Commit**: `feature/new_coll` / `c2d07524b37`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - Plane/cylinder distance support added; accuracy check passed.
  - Bullet/ODE distance still unsupported (distance returns 0 / -1).
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `build/default/cpp/Release/bin/bm_comparative_distance --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_distance_20260120_091559.json --benchmark_out_format=json`
- **Raw Output**:
  - `build/default/cpp/Release/benchmarks/bm_comparative_distance_20260120_091559.json`

Summary (key cases):

- Sphere-sphere: experimental 5.18 ns vs FCL 211 ns.

## Run 2026-01-20 — SDF/ESDF resolution sweep (extended)

- **Branch / Commit**: `feature/new_coll` / `788a69e7bb2`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - CPU scaling enabled; expect some variance.
  - Voxblox comparisons skipped (no `libvoxblox` found).
  - Resolution sweep keeps world extent fixed at 3.2 m with dims 32/40/64/80/96 and voxel sizes 0.10/0.08/0.05/0.04/0.033333.
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `build/default/cpp/Release/dart/collision/experimental/bm_collision_experimental_sdf_compare --benchmark_min_time=0.05s --benchmark_format=json --benchmark_out=docs/dev_tasks/experimental_collision/results/bm_sdf_compare_resolution_2026-01-20_090950.json`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_sdf_compare_resolution_2026-01-20_090950.json`

Summary (items per second, 4096 queries unless noted):

- Dense SDF distance sweep: 28.60 M (32^3, 0.10 m), 28.90 M (40^3, 0.08 m), 25.59 M (64^3, 0.05 m), 23.98 M (80^3, 0.04 m), 22.06 M (96^3, 0.033333 m).
- Dense ESDF distance sweep: 27.75 M (32^3), 29.81 M (40^3), 24.86 M (64^3), 26.03 M (80^3), 23.31 M (96^3).
- Dense ESDF build sweep: 4.90 M (32^3), 4.52 M (40^3), 4.67 M (64^3), 4.11 M (80^3), 4.05 M (96^3) voxels/s.

## Run 2026-01-20 — SDF/ESDF resolution sweep

- **Branch / Commit**: `feature/new_coll` / `be3923bf398`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - CPU scaling enabled; expect some variance.
  - Voxblox comparisons skipped (no `libvoxblox` found).
  - Resolution sweep keeps world extent fixed at 3.2 m with dims 32/40/64 and voxel sizes 0.10/0.08/0.05.
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `build/default/cpp/Release/dart/collision/experimental/bm_collision_experimental_sdf_compare --benchmark_min_time=0.05s --benchmark_format=json --benchmark_out=docs/dev_tasks/experimental_collision/results/bm_sdf_compare_resolution_2026-01-20_085738.json`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_sdf_compare_resolution_2026-01-20_085738.json`

Summary (items per second, 4096 queries unless noted):

- Dense SDF distance sweep: 33.24 M (32^3, 0.10 m), 33.18 M (40^3, 0.08 m), 30.89 M (64^3, 0.05 m).
- Dense ESDF distance sweep: 33.74 M (32^3), 32.01 M (40^3), 32.19 M (64^3).
- Dense ESDF build sweep: 5.92 M (32^3), 5.97 M (40^3), 5.67 M (64^3) voxels/s.

## Run 2026-01-20 — SDF/ESDF compare

- **Branch / Commit**: `feature/new_coll` / `154e5440be3`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - CPU scaling enabled; expect some variance.
  - Voxblox comparisons skipped (no `libvoxblox` found).
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `build/default/cpp/Release/dart/collision/experimental/bm_collision_experimental_sdf_compare --benchmark_min_time=0.05s --benchmark_format=json --benchmark_out=docs/dev_tasks/experimental_collision/results/bm_sdf_compare_2026-01-20_033202.json`
- **Raw Output**:
  - `docs/dev_tasks/experimental_collision/results/bm_sdf_compare_2026-01-20_033202.json`

Summary (items per second):

- Dense SDF distance: 20.53/20.28/21.54 M queries/s (1024/4096/16384 points).
- Sphere vs SDF distance: 2.50/2.34/2.70 M queries/s.
- Dense ESDF distance: 23.03/19.82/13.78 M queries/s.
- Dense ESDF build: 4.41 M voxels/s (32^3 grid).

## Run 2026-01-20 — Comparative rerun (cylinder-box fix)

- **Branch / Commit**: `feature/new_coll` / `db9b3f36dc2`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - Accuracy checks pass; cylinder-box fallback + box-box depth alignment.
  - Bullet/ODE distance still unsupported (distance returns 0 / -1).
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `cmake --build build/default/cpp/Release --target bm_comparative`
  - `build/default/cpp/Release/bin/bm_comparative_narrow_phase --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_narrow_phase_20260120_035117.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_comparative_distance --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_distance_20260120_035157.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_comparative_raycast --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_raycast_20260120_035243.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_experimental_libccd --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_035253.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_comparative --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_20260120_035302.json --benchmark_out_format=json`
- **Raw Output**:
  - `build/default/cpp/Release/benchmarks/bm_comparative_narrow_phase_20260120_035117.json`
  - `build/default/cpp/Release/benchmarks/bm_comparative_distance_20260120_035157.json`
  - `build/default/cpp/Release/benchmarks/bm_comparative_raycast_20260120_035243.json`
  - `build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_035253.json`
  - `build/default/cpp/Release/benchmarks/bm_comparative_20260120_035302.json`

Summary (valid cases only):

- Narrow-phase: accuracy checks pass; performance consistent with prior run.
- Distance: experimental vs FCL consistent; Bullet/ODE distance unsupported.
- Raycast: experimental remains 5.7x-11.6x faster vs Bullet.
- libccd: GJK sphere-sphere 67.9 ns vs 29.1 ns; GJK+EPA 74.9 ns vs 930,944 ns;
  MPR 59.7 ns vs 29.5 ns; GJK box-box 123.3 ns vs 69.5 ns.
- `bm_comparative`: accuracy checks pass; box-box depth ~0.2.

## Run 2026-01-20 — Comparative + baseline rerun (dirty)

- **Branch / Commit**: `feature/new_coll` / `e1b474aeb80`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - Dirty tree; other agent changes present (SDF fields, batch API tests).
  - `bm_comparative_narrow_phase` accuracy mismatch for Cylinder-Box
    (Experimental reports no collision, FCL reports collision).
  - Capsule-related checks now pass after enabling FCL CapsuleShape support.
  - `bm_comparative` warns on box-box depth (expected ~0.2, got ~1.2).
  - `bm_comparative_distance` warns that Bullet distance returns 0 and ODE
    distance returns -1 (unsupported).
  - `bm_comparative_narrow_phase` ran with `--benchmark_min_time=0.05` (missing
    suffix), which triggers gbenchmark warnings.
- **Command**:
  - `cmake --build build/default/cpp/Release --target bm_comparative`
  - `build/default/cpp/Release/bin/bm_comparative_narrow_phase --benchmark_min_time=0.05 --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_narrow_phase_20260120_025641.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_comparative_distance --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_distance_20260120_025724.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_comparative_raycast --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_raycast_20260120_025811.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_experimental_libccd --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_025817.json --benchmark_out_format=json`
  - `build/default/cpp/Release/bin/bm_comparative --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_comparative_20260120_025823.json --benchmark_out_format=json`
- **Raw Output**:
  - `build/default/cpp/Release/benchmarks/bm_comparative_narrow_phase_20260120_025641.json`
  - `build/default/cpp/Release/benchmarks/bm_comparative_distance_20260120_025724.json`
  - `build/default/cpp/Release/benchmarks/bm_comparative_raycast_20260120_025811.json`
  - `build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_025817.json`
  - `build/default/cpp/Release/benchmarks/bm_comparative_20260120_025823.json`

Summary (valid cases only):

- Narrow-phase (sphere/box/plane/cylinder pairs): experimental 4.6x-119x faster
  than FCL/Bullet/ODE; Cylinder-Box mismatch remains.
- Distance (sphere-sphere): experimental ~41x faster vs FCL; Bullet/ODE numbers
  not comparable.
- Raycast (single): experimental 5.7x-11.6x faster than Bullet across sphere,
  box, capsule, cylinder, plane.
- libccd: GJK sphere-sphere ~1.4x slower vs libccd; MPR ~1.03x faster; GJK+EPA
  ~14,186x faster; GJK box-box ~2.0x slower.
- Baseline `bm_comparative` recorded; see box-box depth warning in notes.

## Baseline (pre-structured suite)

Baseline numbers from `bm_comparative.cpp` before the structured suite. Keep
these for historical context; current results are in the summary table above.
A rerun of `bm_comparative` is captured in the 2026-01-20 entry above.

**Narrow-phase performance comparison (lower is better):**

| Shape Pair      | Experimental | FCL          | Bullet       | ODE          | Speedup |
| --------------- | ------------ | ------------ | ------------ | ------------ | ------- |
| Sphere-Sphere   | **41 ns**    | 774-1107 ns  | 411-586 ns   | 991-1002 ns  | **10x** |
| Box-Box         | **210 ns**   | 2429-2486 ns | 1094-1147 ns | 2210-2226 ns | **5x**  |
| Capsule-Capsule | **41 ns**    | 242 ns       | 440 ns       | 1381-1382 ns | **6x**  |
| Distance        | **7 ns**     | 286-288 ns   | N/A          | N/A          | **40x** |

**N-body scaling (brute-force broad-phase):**

| N Objects  | Experimental | FCL        | Bullet     | ODE        |
| ---------- | ------------ | ---------- | ---------- | ---------- |
| 10         | 693 ns       | 1475 ns    | 876 ns     | 2397 ns    |
| 50         | 11,365 ns    | 4,836 ns   | 3,910 ns   | 12,458 ns  |
| 100        | 44,792 ns    | 12,733 ns  | 10,827 ns  | 29,006 ns  |
| Complexity | O(N²)        | O(N log N) | O(N log N) | O(N log N) |

**Accuracy verification:** ✓ PASSED

- Sphere-sphere: Penetration depth matches FCL within 1cm tolerance
- Box-box: Collision detection agrees with FCL
- Distance queries: Exact results (within 1e-6)

## Run 2026-01-20 — libccd microbench

- **Branch / Commit**: `feature/new_coll` / `db9b3f36dc2`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - libccd from pixi env (`.pixi/envs/default`), since the local libccd CMakeLists target name collides with the already-imported `ccd` target.
  - `--benchmark_min_time=0.05s`.
- **Command**:
  - `build/default/cpp/Release/bin/bm_experimental_libccd --benchmark_min_time=0.05s --benchmark_out=build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_035253.json --benchmark_out_format=json`
- **Raw Output**:
  - `build/default/cpp/Release/benchmarks/bm_experimental_libccd_20260120_035253.json`

Summary (speedup = libccd time / experimental time):

- GJK sphere-sphere: 0.43x (experimental ~2.3x slower).
- GJK+EPA sphere-sphere: 12,427x (experimental faster).
- MPR sphere-sphere: 0.49x (experimental ~2.0x slower).
- GJK box-box: 0.56x (experimental ~1.8x slower).

## Run 2026-01-20 — Raycast comparative + batch scenarios

- **Branch / Commit**: `feature/new_coll` / `f315999cdfe`
- **Build**: `Release` (build/default/cpp/Release)
- **CPU**: 13th Gen Intel(R) Core(TM) i9-13950HX
- **OS**: Ubuntu 25.10
- **Compiler**: c++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
- **Notes**:
  - Raycast batch uses SweepAndPrune broadphase (was to avoid AabbTree crash, now fixed).
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

- Narrow-phase: min 2.75x, p50 7.01x, p90 11.04x (77 cases).
- Distance: min 0.02x, p50 0.09x, p90 0.49x (66 cases; Experimental slower).
- Mixed primitives: min 0.06x, p50 0.42x, p90 1.10x (6 cases; BigO/RMS excluded).
- Mesh-heavy: 3.58x-4.67x (2 cases).
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

## Run 2026-01-20 — Pipeline breakdown (RP3D-aligned) [FIXED]

- **Status**: ~~segfault in `AabbTreeBroadPhase::combine()` during object creation.~~ **FIXED** (2026-01-20)
- **Root cause**: Dangling reference after vector reallocation in `insertLeaf()`.
- **Command**:
  - `build/default/cpp/Release/bin/bm_scenarios_pipeline_breakdown --benchmark_filter="PipelineBreakdown_RP3D_.*" --benchmark_format=json --benchmark_min_time=0.1s`
- **Results** (after fix):
  - Dense 1k: broadphase 163µs, narrowphase 83µs, 458 pairs, 252 contacts
  - Dense 10k: broadphase 11.3ms, narrowphase 665ms, 46.3k pairs, 24.7k contacts
  - Sparse 1k: broadphase 57µs, narrowphase 0.4µs, 4 pairs, 4 contacts
  - Sparse 10k: broadphase 2.3ms, narrowphase 87µs, 403 pairs, 209 contacts
- **Notes**: Fix applied to `aabb_tree.cpp` - copy AABB values before `allocateNode()` to avoid dangling references.
