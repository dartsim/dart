# ECS Data Layout and Batch-Friendly Interfaces

Status: draft
Last updated: 2026-01-19

## Purpose

Define data structures and API changes that make batch collision queries
ECS-friendly and scalable for 10k+ objects, while keeping single-thread
performance competitive.

## Current baseline (experimental collision)

- `entt::registry` owns components
- Hot path components:
  - `CollisionObjectTag`
  - `ShapeComponent` (shape pointer)
  - `TransformComponent`
  - `AabbComponent` (AABB + dirty flag)
  - `UserDataComponent`
- Broadphase works on AABBs queried from the registry each frame

## Pain points for batch performance

- Indirection through `entt` lookups on every pair
- Cache-unfriendly layout for hot arrays (transform, AABB, shape pointer)
- No stable object id for deterministic pair ordering across threads
- Per-call allocations for pair and contact output

## Proposed data layout

### Stable object id

- Assign a monotonic `ObjectId` on creation (never reused in world lifetime).
- Store `ObjectId` in a component and keep `entity <-> ObjectId` maps.
- Use `(id1, id2)` ordering for determinism and stable merges.

### Batch storage (SoA)

Maintain a compact SoA cache for hot data:

- `std::vector<ObjectId> ids`
- `std::vector<const Shape*> shapes`
- `std::vector<Eigen::Isometry3d> transforms`
- `std::vector<Aabb> aabbs`
- `std::vector<uint8_t> flags` (dirty, enabled, static)

The registry remains authoritative, but the batch cache is the fast path for
bulk queries.

### Dirty/moved tracking

- Maintain a `std::vector<ObjectId> dirtyIds` for AABB updates.
- Optionally keep a bitset to avoid duplicates when multiple updates occur.

### Pair and contact cache (temporal coherence)

- Store a `PairId` per broadphase overlap and reuse last-frame narrowphase
  data (GJK/SAT state) when possible.
- Keep a per-pair manifold cache keyed by `(id1, id2)` or `PairId`.

## Phased adoption plan

1. Phase 1: Stable ids + deterministic ordering
   - Add `ObjectId` and id <-> entity mapping.
   - Sort pairs by `(id1, id2)` consistently across batch outputs.
2. Phase 2: SoA cache and dirty updates
   - Introduce `BatchStorage` vectors for ids, shapes, transforms, AABBs.
   - Track dirty ids and update AABBs incrementally.
3. Phase 3: Batch view + bulk APIs
   - Add `BatchView` adapter and bulk broadphase/narrowphase entry points.
   - Use `BatchScratch` per thread to avoid allocations.
4. Phase 4: Pair and manifold cache
   - Persist overlap pairs and reuse contact manifolds for temporal coherence.
   - Add invalidation rules on shape/transform changes.
5. Phase 5: Evidence-based tuning
   - Run RP3D-aligned pipeline breakdown after AABB tree crash is fixed.
   - Compare stage ratios and adjust data layout as needed (no benchmark edits here).

## API/interface changes (draft)

### CollisionWorld

- `reserveObjects(std::size_t count)` to preallocate SoA buffers.
- `updateAll(const BatchSettings&, BatchStats*)` already exists; add
  `updateDirty(const std::span<ObjectId>& ids)` for explicit updates.
- `buildBroadPhaseSnapshot(const BatchSettings&)` should avoid allocations when
  provided with a reusable output buffer.

### BroadPhase

Add bulk interfaces to reduce per-object overhead:

- `build(const std::span<const Aabb>& aabbs)`
- `updateRange(const std::span<const ObjectId>& ids,
           const std::span<const Aabb>& aabbs)`
- `queryPairs(std::vector<BroadPhasePair>& out)` (caller-owned storage)

### NarrowPhase

Expose batch-friendly entry points:

- `collidePairs(const std::span<BroadPhasePair>& pairs,
            const BatchView& view,
            BatchOutput& out,
            BatchScratch& scratch)`

### Batch view

A lightweight adapter over SoA arrays:

- `BatchView` exposes `shape(i)`, `transform(i)`, `aabb(i)`, and flags.
- Lookup by `ObjectId` uses a compact `id -> index` map.

## ECS-friendly algorithm improvements

- Precompute a shape-pair dispatch table (function pointers) to avoid
  branching in tight loops.
- Use `BatchScratch` per thread to avoid heap churn in GJK/EPA.
- Deterministic merge via stable sort or prefix-sum per chunk.

## Measurement plan

- Use RP3D-aligned pipeline breakdown benchmarks for apples-to-apples timing.
- Track update cost, broadphase, narrowphase, and merge separately.
- Compare single-thread and multi-thread with deterministic on/off.

## Risks / open questions

- Cost of keeping SoA cache in sync with `entt` (update frequency + memory).
- Deletion compaction strategy for SoA without invalidating ids.
- Best keying strategy for pair cache (hash vs direct index).
