# Batch Collision Parallelization Plan (Experimental Collision)

## Goal

Enable fast batch collision queries for scenes with 10k+ objects, scaling across
multi-core when available while keeping single-thread performance competitive
and deterministic.

## Scope

- Standalone experimental collision world (`dart/collision/experimental`)
- Batch collision, distance, raycast, and CCD queries
- Broadphase + narrowphase pipeline for large scenes

## Non-Goals (Early Phases)

- GPU acceleration
- DART backend integration
- Networked or distributed simulation

## Constraints

- Determinism: stable ordering and reproducible results across thread counts
- No per-frame heap churn in hot paths (use thread-local arenas)
- DART contact conventions (normal direction, depth sign)

## Phase 0: Baselines and Instrumentation

Deliverables:

- Stage timing for AABB update, broadphase, narrowphase, merge
- Memory and allocation counters (pair count, contact count, temp bytes)
- Scenario benchmarks for 1k and 10k objects (dense and sparse)

Exit criteria:

- Baseline numbers captured for single-thread mode
- JSON outputs captured for regression tracking

Status update (2026-01-20):

- Added batch API scaffolding in `dart/collision/experimental/batch.hpp`
  (BatchSettings, BatchStats, BroadPhaseSnapshot).
- Added `CollisionWorld::updateAll`, `buildBroadPhaseSnapshot`,
  and `collideAll` for explicit pipeline staging.
- AABB tree crash fixed; RP3D-aligned pipeline breakdown captured and logged
  in `benchmark_results.md` and `reactphysics3d_ecs_profile_results.md`.

## Phase 1: Data Layout and Batch API

Deliverables:

- Define a batch query interface that separates input data, broadphase output,
  and narrowphase results.
- Stable object IDs and deterministic pair ordering strategy.
- Thread-local scratch allocation plan.

API sketch (draft, subject to change):

```
struct BatchSettings {
  int maxThreads = 1;
  bool deterministic = true;
  bool collectStats = false;
  std::size_t grainSize = 256;
};

struct BatchOutput {
  CollisionResult result;
  std::vector<BroadPhasePair> pairs;
};

struct BroadPhaseSnapshot {
  std::vector<BroadPhasePair> pairs;
  std::size_t numObjects = 0;
};

struct BatchStats {
  std::size_t numObjects = 0;
  std::size_t numPairs = 0;
  std::size_t numPairsTested = 0;
  std::size_t numContacts = 0;
};

class CollisionWorld {
public:
  std::size_t updateAll(const BatchSettings& settings,
                        BatchStats* stats = nullptr);
  BroadPhaseSnapshot buildBroadPhaseSnapshot(const BatchSettings& settings) const;
  void collideAll(const BroadPhaseSnapshot& snapshot,
                  const CollisionOption& option,
                  BatchOutput& out,
                  const BatchSettings& settings,
                  BatchStats* stats = nullptr);
};
```

Exit criteria:

- Batch API agreed and documented
- Data layout changes scoped (SoA vs AoS, entt usage, alignment)

## Phase 2: Broadphase Acceleration

Deliverables:

- Replace O(n^2) brute force with dynamic AABB tree and/or sweep-and-prune.
- Incremental updates for moved objects only.
- Deterministic pair ordering (stable sort by object IDs).

Exit criteria:

- Broadphase beats brute-force for 10k objects
- Pair output is stable across runs

## Phase 3: Parallel Narrowphase

Deliverables:

- Partition pairs into fixed-size chunks for worker threads.
- Thread-local scratch buffers and output vectors.
- Deterministic merge of results (stable sort or prefix-sum ordering).

Exit criteria:

- Linear or near-linear speedup up to core count
- Single-thread performance within 5% of baseline

## Phase 4: Parallel Broadphase (Optional)

Deliverables:

- Parallel AABB recompute and fat-AABB update.
- Optional bulk rebuild path for large scene edits.

Exit criteria:

- Broadphase update scales for large object counts

## Phase 5: Validation and Tuning

Deliverables:

- Cross-backend correctness checks vs FCL/Bullet/ODE
- Thread scaling tests (1/2/4/8/16) across scenarios
- Regression guardrails in benchmarks

Exit criteria:

- Experimental is >= best backend for supported scenarios
- Determinism verified across thread counts

## Notes on Determinism

- Stable object IDs assigned at creation time
- Pair ordering based on `(id1, id2)` with `id1 < id2`
- Merge results with stable sort on `(id1, id2, contact_index)`
- Avoid unordered containers in hot paths

## Immediate Next Steps

1. Implement ECS-friendly data layout changes described in
   `ecs_data_layout.md` (ObjectId + BatchStorage/BatchView + batch narrowphase).
2. Wire the batch path in `CollisionWorld` to avoid per-pair registry lookups.
3. Once benchmark ownership is clear, re-run the RP3D-aligned pipeline
   breakdown to measure impact.
