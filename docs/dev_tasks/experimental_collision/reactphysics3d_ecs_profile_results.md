# ReactPhysics3D ECS Profiling Results

Status: initial baseline
Last updated: 2026-01-19

## Setup

- Repo: `/home/js/dev/physics/reactphysics3d`
- Build: `build_profile` with `IS_RP3D_PROFILING_ENABLED`
- Driver: `docs/dev_tasks/experimental_collision/rp3d_profile_driver.cpp`
- Shapes: spheres only (radius 0.5)
- Velocities: zero (static positions, dynamic bodies)
- Steps: warmup 1, measure 2 (10k) or 3 (1k)
- Profiling output: `Profiler::printReport()` text

## Scenarios

- Dense: range 10 (positions in [-10, 10])
- Sparse: range 50 (positions in [-50, 50])

## Collision detection timings (ms/frame)

These are from `CollisionDetectionSystem::computeCollisionDetection()` and its
children in the profiler report.

| Scenario | Count | Range | Calls | CollisionDetect | Broadphase | Middle | Narrow  |
| -------- | ----- | ----- | ----- | --------------- | ---------- | ------ | ------- |
| Dense    | 1k    | 10    | 3     | 0.2426          | 0.0655     | 0.0741 | 0.1029  |
| Sparse   | 1k    | 50    | 3     | 0.0039          | 0.0026     | 0.0003 | 0.0010  |
| Dense    | 10k   | 10    | 2     | 36.6275         | 22.9269    | 3.1029 | 10.5967 |
| Sparse   | 10k   | 50    | 2     | 0.2735          | 0.1393     | 0.0541 | 0.0799  |

## Raw outputs

- `docs/dev_tasks/experimental_collision/results/rp3d_profile_2026-01-19_1k_dense.txt`
- `docs/dev_tasks/experimental_collision/results/rp3d_profile_2026-01-19_1k_sparse.txt`
- `docs/dev_tasks/experimental_collision/results/rp3d_profile_2026-01-19_10k_dense.txt`
- `docs/dev_tasks/experimental_collision/results/rp3d_profile_2026-01-19_10k_sparse.txt`

## DART comparison (planned)

Run DART pipeline breakdown with RP3D-aligned scenarios:

```bash
./tests/benchmark/bm_scenarios_pipeline_breakdown \
  --benchmark_filter="PipelineBreakdown_RP3D_.*"
```

Record results in `benchmark_results.md` alongside the RP3D baselines.

## Notes

- Broadphase dominates the dense 10k case (dynamic AABB tree update + overlap).
- Narrowphase becomes a larger share as density increases.
- These numbers are single-thread and include the full collision detection
  pipeline (broadphase + middle + narrow), not solver or integration costs.
