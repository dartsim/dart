# ECS Architecture and Batch Collision API

Status: in progress
Last updated: 2026-01-20

## Goals

- Handle 10k+ objects with scalable multi-core throughput.
- Preserve or improve single-thread performance and determinism.
- Avoid per-frame heap churn in hot paths (preallocate + thread-local scratch).
- Keep the API minimal and consistent with the existing experimental collision world.

## Non-goals (early phases)

- GPU acceleration.
- Integration into the production DART backend.
- Distributed or networked collision queries.

---

## Core Concepts

### Stable Object Identity

Use a stable object id (monotonic, never reused during the world lifetime) to
ensure deterministic pair ordering and reproducible results across thread
counts. Map entt::entity to this id and store in a compact array.

- Assign a monotonic `ObjectId` on creation (never reused in world lifetime).
- Store `ObjectId` in a component and keep `entity <-> ObjectId` maps.
- Use `(id1, id2)` ordering for determinism and stable merges.
- Initial implementation: `BroadPhaseComponent::broadPhaseId` assigned in
  `CollisionWorld`, exposed via `CollisionObject::getId()`.

### SoA-Friendly Layout (BatchStorage)

Keep SoA arrays for the hot data (transform, AABB, shape pointer, flags) with
separate arrays for cold metadata. This reduces cache misses for broadphase and
narrowphase loops.

Maintain a compact SoA cache for hot data:

- `std::vector<ObjectId> ids`
- `std::vector<const Shape*> shapes`
- `std::vector<Eigen::Isometry3d> transforms`
- `std::vector<Aabb> aabbs`
- `std::vector<uint8_t> flags` (dirty, enabled, static)

The registry remains authoritative, but the batch cache is the fast path for
bulk queries. Implementation lives in `dart/collision/experimental/batch.hpp`.

### BatchView

A lightweight adapter over SoA arrays:

- `BatchView` exposes `shape(i)`, `transform(i)`, `aabb(i)`, and flags.
- Lookup by `ObjectId` uses a compact `id -> index` map.

### Dirty/Moved Tracking

- Maintain a `std::vector<ObjectId> dirtyIds` for AABB updates.
- Optionally keep a bitset to avoid duplicates when multiple updates occur.

### Broadphase Snapshot Caching

- Cache the most recent broadphase snapshot and reuse it when no objects moved.
- Invalidate the cache on create/destroy and on any AABB update.
- Keep a separate cache key for deterministic vs non-deterministic ordering.

### Pair and Contact Cache (Temporal Coherence)

- Store a `PairId` per broadphase overlap and reuse last-frame narrowphase
  data (GJK/SAT state) when possible.
- Keep a per-pair manifold cache keyed by `(id1, id2)` or `PairId`.

---

## Pipeline Staging

1. **Update stage**: apply transforms and recompute AABBs for dirty objects.
2. **Broadphase**: update broadphase structure, generate candidate pairs.
3. **Narrowphase**: evaluate pairs, build contact manifolds.
4. **Merge**: deterministic merge into output buffers.

Each stage has explicit entry points for profiling and benchmarking.

---

## Batch API

The existing `BatchSettings`, `BatchStats`, `BatchTimings`, and
`BroadPhaseSnapshot` in `dart/collision/experimental/batch.hpp` are the base.

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
  std::vector<BroadPhasePair> pairs;  // (id1,id2) aligned with result manifolds
};

class CollisionWorld {
public:
  std::size_t updateAll(const BatchSettings& settings,
                        BatchStats* stats = nullptr);

  // Explicit dirty update (implemented)
  std::size_t updateDirty(std::span<const ObjectId> ids);

  BroadPhaseSnapshot buildBroadPhaseSnapshot(const BatchSettings& settings) const;

  void collideAll(const BroadPhaseSnapshot& snapshot,
                  const CollisionOption& option,
                  BatchOutput& out,
                  const BatchSettings& settings = {},
                  BatchStats* stats = nullptr);

  // Reserve capacity for SoA buffers
  void reserveObjects(std::size_t count);

  // Read-only SoA data for batch queries
  BatchView getBatchView() const;

  // Map snapshot ids back to handles
  CollisionObject* getObjectById(ObjectId id);
};
```

Notes:

- `BatchScratch` lives per worker thread (TLS or an explicit pool).
- `BatchOutput` separates pair ordering from manifolds and allows stable merges.
- `grainSize` can be tuned or auto-selected based on pair count.

### BroadPhase Bulk Interfaces

Add bulk interfaces to reduce per-object overhead:

- `build(const std::span<const Aabb>& aabbs)`
- `updateRange(const std::span<const ObjectId>& ids, const std::span<const Aabb>& aabbs)`
- `queryPairs(std::vector<BroadPhasePair>& out)` (caller-owned storage)

### NarrowPhase Batch-Friendly Entry Points

- `NarrowPhase::collide()` overload for raw shape + transform inputs.
- Dispatch path reuses existing algorithms (sphere, box, capsule, etc.).
- `collidePairs(const std::span<BroadPhasePair>& pairs, const BatchView& view, BatchOutput& out, BatchScratch& scratch)`

---

## Parallelization Strategy

### Single-Thread Path

Use the same code paths with `maxThreads=1` and avoid task creation. Keep
branching minimal so the single-thread loop stays tight.

### Multi-Core Path

- **Update stage**: parallel for over dirty objects (transform + AABB).
- **Broadphase**: optional parallel rebuild or partial update for large edits.
- **Narrowphase**: split candidate pair list into fixed-size chunks, each processed
  by a worker with its own `BatchScratch`.
- **Merge**: deterministic order via pair index or stable sort by `(id1, id2)`.

Taskflow is the preferred executor because it is already a dependency and is
used elsewhere in the experimental modules.

---

## Determinism

To keep results stable across thread counts:

- Assign stable object ids at creation.
- Order pairs by `(id1, id2)` with `id1 < id2`.
- Use a deterministic merge: prefix-sum per thread chunk or stable sort on
  `(id1, id2, manifoldIndex)`.
- Avoid unordered containers in the hot path.

---

## Memory and Scratch Management

- Preallocate arrays for transforms, AABBs, shape pointers, and flags.
- Keep a dedicated `BatchScratch` pool sized by `maxThreads`.
- Store contact manifolds in a stable vector and reuse capacity across frames.

---

## ECS-Friendly Algorithm Improvements

- Precompute a shape-pair dispatch table (function pointers) to avoid
  branching in tight loops.
- Use `BatchScratch` per thread to avoid heap churn in GJK/EPA.
- Deterministic merge via stable sort or prefix-sum per chunk.

---

## Instrumentation (BatchStats)

Extend `BatchStats` and `BatchTimings` to capture:

- AABB update count and time.
- Broadphase time and pair counts.
- Narrowphase time and contact counts.
- Merge time and output size.
- Optional allocation counters (bytes and count).

---

## Implementation Status

### Phase 1: Stable Ids + Snapshot Cache (done)

- `CollisionObject::getId()` available.
- `CollisionWorld::buildBroadPhaseSnapshot()` reuses cached pairs when valid.

### Phase 2: BatchStorage + BatchView (done)

- SoA cache for hot data (ids, shape pointers, transforms, AABBs).
- `BatchView` adapter for read-only access without ECS lookups.
- `CollisionWorld::getBatchView()` API and `reserveObjects()` preallocation.
- Create/destroy/update path keeps batch storage in sync.

### Phase 3: ECS-Friendly Narrowphase Interface (done)

- `NarrowPhase::collide()` overload for raw shape + transform inputs.
- Dispatch path reuses existing algorithms (sphere, box, capsule, etc.).
- `CollisionWorld::collideAll` uses shape/transform dispatch directly.

### Phase 4: Parallel Batch Pipeline (future)

- Chunked pair processing with `BatchSettings::maxThreads`.
- Thread-local `BatchScratch` buffers for GJK/EPA and contact data.
- Deterministic merge strategy across worker outputs.
- Exit criteria: Near-linear scaling on multi-core for large pair sets;
  single-thread mode within 5% of baseline.

---

## Open Design Decisions

- Broadphase choice: dynamic AABB tree vs sweep-and-prune, or both via policy.
- Pair cache strategy: keep last-frame pairs for temporal coherence or always rebuild.
- Contact manifold caching: reuse last-frame data to reduce narrowphase cost.

---

## Next Steps

1. Extend the batch view to expose optional flags (static/disabled) once
   filtering rules are defined.
2. Add a filtered broadphase query path using the flags (mask/group support).
3. Validate single-thread regressions after batch-view plumbing (timing-only,
   no benchmark code changes).

---

## Risks / Open Questions

- Cost of keeping SoA cache in sync with `entt` (update frequency + memory).
- Deletion compaction strategy for SoA without invalidating ids.
- Best keying strategy for pair cache (hash vs direct index).
