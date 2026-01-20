# ReactPhysics3D ECS Profiling Results

Status: baseline + DART comparison
Last updated: 2026-01-20

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

## DART comparison (completed)

Run DART pipeline breakdown with RP3D-aligned scenarios:

```bash
build/default/cpp/Release/bin/bm_scenarios_pipeline_breakdown \
  --benchmark_filter="PipelineBreakdown_RP3D_.*" \
  --benchmark_format=json \
  --benchmark_min_time=0.05s
```

Results (ns, single-thread):

- Dense 1k: aabb_update_ns=44155.97003745318 broadphase_ns=149967.65917602996 narrowphase_ns=76474.0 merge_ns=144.69662921348313 pairs=458.0 contacts=252.0
- Dense 10k: aabb_update_ns=421720.0 broadphase_ns=10237884.0 narrowphase_ns=656721732.0 merge_ns=32321.0 pairs=46345.0 contacts=24686.0
- Sparse 1k: aabb_update_ns=46493.759825327514 broadphase_ns=50766.985443959245 narrowphase_ns=329.29694323144105 merge_ns=16.973799126637555 pairs=4.0 contacts=4.0
- Sparse 10k: aabb_update_ns=428326.14814814815 broadphase_ns=2075412.5925925926 narrowphase_ns=75518.92592592593 merge_ns=138.1851851851852 pairs=403.0 contacts=209.0

Raw output:

- `docs/dev_tasks/experimental_collision/results/bm_pipeline_breakdown_rp3d_2026-01-20_015424.json`

See `benchmark_results.md` for the full run log.

## Notes

- RP3D dense 10k shows broadphase as the largest share; DART dense 10k shows
  narrowphase dominating the frame time.
- The DART narrowphase spike, combined with fast single-pair microbenchmarks,
  points to per-pair overhead (data access, handle construction, result merge).
- These numbers are single-thread and include the full collision detection
  pipeline (broadphase + middle + narrow), not solver or integration costs.
