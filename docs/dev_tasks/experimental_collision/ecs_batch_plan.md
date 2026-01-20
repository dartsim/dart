# ECS Batch Collision Plan (Experimental Collision)

Status: in progress
Last updated: 2026-01-20

## Goal

Deliver ECS-friendly data structures and collision APIs that scale to 10k+
objects with strong single-thread performance and a clear path to multi-core.

## Constraints

- No benchmark code edits in this phase (owned by another agent).
- Determinism preserved via stable object ids and ordered pairs.
- Avoid per-frame heap churn in hot paths.

## Phase 1: Stable Ids + Snapshot Cache (done)

Deliverables:

- Stable `ObjectId` assigned on creation.
- Broadphase pairs are emitted in stable `(id1, id2)` order.
- Snapshot caching avoids repeated broadphase queries when no updates occur.

Exit criteria:

- `CollisionObject::getId()` available.
- `CollisionWorld::buildBroadPhaseSnapshot()` reuses cached pairs when valid.

## Phase 2: BatchStorage + BatchView (in progress)

Deliverables:

- SoA cache for hot data (ids, shape pointers, transforms, AABBs).
- `BatchView` adapter for read-only access without ECS lookups.
- `CollisionWorld::getBatchView()` API and `reserveObjects()` preallocation.

Exit criteria:

- Create/destroy/update path keeps batch storage in sync.
- `collideAll` can consume `BatchView` (no per-pair entt lookups).

## Phase 3: ECS-Friendly Narrowphase Interface (next)

Deliverables:

- `NarrowPhase::collide()` overload for raw shape + transform inputs.
- Dispatch path reuses existing algorithms (sphere, box, capsule, etc.).
- Batch collision path uses the new overload.

Exit criteria:

- `CollisionWorld::collideAll` uses shape/transform dispatch directly.
- No behavior change vs existing `CollisionObject` API.

## Phase 4: Parallel Batch Pipeline (future)

Deliverables:

- Chunked pair processing with `BatchSettings::maxThreads`.
- Thread-local `BatchScratch` buffers for GJK/EPA and contact data.
- Deterministic merge strategy across worker outputs.

Exit criteria:

- Near-linear scaling on multi-core for large pair sets.
- Single-thread mode within 5% of baseline.

## Dependencies

- RP3D profiling results to validate ECS data layout benefits.
- AABB tree stability fixes (already reported resolved by user).

## Immediate Next Steps

1. Implement BatchStorage + BatchView and wire into `CollisionWorld`.
2. Add narrowphase shape/transform overload and use it in batch collide.
3. Update `ecs_data_layout.md` and `RESUME.md` with implementation status.
