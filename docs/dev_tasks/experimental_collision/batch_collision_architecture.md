# Batch Collision Architecture and API (Experimental Collision)

Status: draft
Last updated: 2026-01-19

## Goals

- Handle 10k+ objects with scalable multi-core throughput.
- Preserve or improve single-thread performance and determinism.
- Avoid per-frame heap churn in hot paths (preallocate + thread-local scratch).
- Keep the API minimal and consistent with the existing experimental collision world.

## Non-goals (early phases)

- GPU acceleration.
- Integration into the production DART backend.
- Distributed or networked collision queries.

## Core concepts

### Stable object identity

Use a stable object id (monotonic, never reused during the world lifetime) to
ensure deterministic pair ordering and reproducible results across thread
counts. Map entt::entity to this id and store in a compact array.

### SoA-friendly layout

Keep SoA arrays for the hot data (transform, AABB, shape pointer, flags) with
separate arrays for cold metadata. This reduces cache misses for broadphase and
narrowphase loops.

### Pipeline staging

1. **Update stage**: apply transforms and recompute AABBs for dirty objects.
2. **Broadphase**: update broadphase structure, generate candidate pairs.
3. **Narrowphase**: evaluate pairs, build contact manifolds.
4. **Merge**: deterministic merge into output buffers.

Each stage has explicit entry points for profiling and benchmarking.

## Proposed batch API (draft)

The existing `BatchSettings`, `BatchStats`, `BatchTimings`, and
`BroadPhaseSnapshot` in `dart/collision/experimental/batch.hpp` are the base.
This sketch refines how they are used end-to-end.

```cpp
struct BatchSettings {
  int maxThreads = 1;           // 1 = single-thread
  bool deterministic = true;    // stable ordering and merges
  bool collectStats = false;    // timings + counters
  std::size_t grainSize = 256;  // pair chunk size for parallel loops
};

struct BatchScratch {
  // Thread-local storage for GJK/EPA, contact buffers, temp arrays.
  void reset();
};

struct BatchOutput {
  CollisionResult result;
  std::vector<CollisionPair> pairs;  // (id1,id2) aligned with result manifolds
};

class CollisionWorld {
public:
  std::size_t updateAll(const BatchSettings& settings,
                        BatchStats* stats = nullptr);

  BroadPhaseSnapshot buildBroadPhaseSnapshot(const BatchSettings& settings,
                                             BatchStats* stats = nullptr) const;

  void collideAll(const BroadPhaseSnapshot& snapshot,
                  const CollisionOption& option,
                  BatchOutput& out,
                  BatchStats* stats = nullptr,
                  const BatchSettings& settings = {});
};
```

Notes:

- `BatchScratch` lives per worker thread (TLS or an explicit pool).
- `BatchOutput` separates pair ordering from manifolds and allows stable merges.
- `grainSize` can be tuned or auto-selected based on pair count.

## Parallelization strategy

### Single-thread path

Use the same code paths with `maxThreads=1` and avoid task creation. Keep
branching minimal so the single-thread loop stays tight.

### Multi-core path

- Update stage: parallel for over dirty objects (transform + AABB).
- Broadphase: optional parallel rebuild or partial update for large edits.
- Narrowphase: split candidate pair list into fixed-size chunks, each processed
  by a worker with its own `BatchScratch`.
- Merge: deterministic order via pair index or stable sort by `(id1, id2)`.

Taskflow is the preferred executor because it is already a dependency and is
used elsewhere in the experimental modules.

## Determinism

To keep results stable across thread counts:

- Assign stable object ids at creation.
- Order pairs by `(id1, id2)` with `id1 < id2`.
- Use a deterministic merge: prefix-sum per thread chunk or stable sort on
  `(id1, id2, manifoldIndex)`.
- Avoid unordered containers in the hot path.

## Memory and scratch management

- Preallocate arrays for transforms, AABBs, shape pointers, and flags.
- Keep a dedicated `BatchScratch` pool sized by `maxThreads`.
- Store contact manifolds in a stable vector and reuse capacity across frames.

## Instrumentation

Extend `BatchStats` and `BatchTimings` to capture:

- AABB update count and time.
- Broadphase time and pair counts.
- Narrowphase time and contact counts.
- Merge time and output size.
- Optional allocation counters (bytes and count).

## Open design decisions

- Broadphase choice: dynamic AABB tree vs sweep-and-prune, or both via policy.
- Pair cache strategy: keep last-frame pairs for temporal coherence or always
  rebuild.
- Contact manifold caching: reuse last-frame data to reduce narrowphase cost.

## Next steps

1. Review API sketch with implementation team and lock signatures.
2. Decide broadphase strategy and data layout changes.
3. Implement staged pipeline with timings in the experimental collision world.
4. Add scaling benchmarks for 1k/10k/50k and thread counts (1/2/4/8/16).
