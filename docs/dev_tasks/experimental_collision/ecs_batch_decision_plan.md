# ECS Batch Decision Plan (Experimental Collision)

Status: in progress
Last updated: 2026-01-20

## Decision goal

Determine whether ECS-friendly data layout + algorithm changes can materially
improve batch collision throughput (10k+ objects), enough to justify moving the
experimental collision detector toward an ECS-first design.

## Evidence so far

- RP3D profiling (spheres, dense/sparse, 1k/10k) shows broadphase-heavy costs in
  dense scenes with a well-optimized ECS layout.
- DART RP3D-aligned pipeline breakdown shows narrowphase dominating dense 10k
  cases, despite fast single-pair microbenchmarks.
- The disparity suggests per-pair overhead (data access, object lookup,
  result accumulation) is a likely bottleneck, not the narrowphase algorithms
  themselves.

Conclusion: proceed with ECS-friendly data layout changes before drawing final
performance conclusions.

## Phased plan (with checkpoints)

### Phase 0: Evidence baseline (done)

- Capture RP3D profiler outputs (dense/sparse, 1k/10k).
- Capture DART pipeline breakdown on RP3D-aligned scenarios.
- Log results in `reactphysics3d_ecs_profile_results.md` and
  `benchmark_results.md`.

Exit criteria:

- Both RP3D and DART baselines captured and reviewed.

### Phase 1: Data layout and batch APIs (implemented, pending validation)

Goal: remove per-pair ECS lookup overhead and align hot data into SoA storage.

Planned changes:

- Stable `ObjectId` assigned at creation (never reused).
- `BatchStorage` SoA cache (ids, shapes, transforms, AABBs, flags).
- `BatchView` to access hot arrays without registry lookups.
- Broadphase operates on `ObjectId` for deterministic ordering.
- Narrowphase overload that accepts shape + transform directly.

Exit criteria:

- Batch path avoids per-pair `CollisionObject` construction.
- Deterministic pair ordering based on `ObjectId`.

Checkpoint: commit after Phase 1 wiring.

### Phase 2: Batch algorithm upgrades

Goal: reduce narrowphase work via reuse and caching.

Planned changes:

- Shape-pair dispatch table to reduce branching.
- Pair cache keyed by `(id1, id2)` for temporal coherence.
- Manifold cache with invalidation rules for moved objects.
- `BatchScratch` arenas to avoid heap churn.

Exit criteria:

- Reuse rate measurable for stable scenes.
- No per-frame heap allocations in hot loop.

Checkpoint: commit after cache + scratch path.

### Phase 3: Parallel narrowphase

Goal: multi-core scaling without sacrificing determinism.

Planned changes:

- Pair chunks sized by `BatchSettings::grainSize`.
- Thread-local `BatchScratch` and per-thread `CollisionResult`.
- Deterministic merge by `(id1, id2)` ordering.

Exit criteria:

- Near-linear speedup on 4-8 cores.
- Deterministic outputs across thread counts.

Checkpoint: commit after threaded merge.

### Phase 4: Validation and decision

Goal: decide ECS migration based on evidence.

Planned steps (no benchmark edits while owned elsewhere):

- Run RP3D-aligned pipeline breakdown on new ECS batch path.
- Compare against RP3D profiler baselines for stage ratios.
- Record results and decide whether to proceed with full ECS migration.

Exit criteria:

- Evidence-backed decision captured in `progress.md`.

## Non-goals (current phase)

- Benchmark code edits.
- DART backend integration changes.

## Immediate next steps

1. Implement Phase 1 data layout changes in `dart/collision/experimental/`.
2. Update `ecs_data_layout.md` and `parallelization_plan.md` to reflect Phase 1.
3. Once benchmark ownership is clear, re-run pipeline breakdown.
