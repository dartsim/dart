# Collision Detector Architecture Proposal

## Intent and Constraints
- Provide a collision core in namespace `dart::collision` while keeping DART public APIs stable.
- Preserve current contact conventions (normal direction, penetration depth semantics, filtering).
- Keep the existing world and constraint solver integration behavior intact.
- Maintain the existing integration workflow for gazebo without extra patches.
- Delay default switching until parity and performance are acceptable.

## Namespace and Layout
- Core API namespace: `dart::collision`.
- Implementation lives under `dart/collision/dart` during this effort; DART8 can consolidate to `dart/collision/` when legacy backends are removed.
- Adapter responsibilities remain within `dart::collision` to avoid public API churn.

## Layered Design
- Core engine: geometry representations, broadphase, narrowphase, and query results with minimal dependencies.
- DART adapter: maps ShapeFrame and Shape to core objects, owns CollisionObject handles, and translates options/results for the existing API.
- Compatibility layer: keeps factory keys, configuration options, and public types stable during migration.

## Adapter Responsibilities
- Maintain object lifetimes and stable pointers expected by the constraint solver.
- Translate DART filters (collision, distance, raycast) into core query filters.
- Honor ShapeFrame versioning so geometry caches refresh only when needed.

## Core Data Model
- Geometry: immutable shape data with type-specific parameters and bounds.
- Geometry cache: shared heavy data (mesh acceleration structures, convex data) keyed by shape version.
- Object instance: geometry + transform + user data + flags (collidable, category, filter tags).
- Collection: a set of object instances used for group and group-pair queries.

## Query Pipeline
- Update phase: refresh transforms each step; rebuild cached geometry only when shape versions change.
- Broadphase: dynamic AABB tree with incremental updates; optional static vs dynamic partitioning. MVP uses sweep-and-prune while core data types stabilize.
- Narrowphase: pair-specific algorithms with analytic paths for primitives, convex-convex via iterative methods, and mesh/heightfield paths via BVH traversal.
- Result reduction: contact merging and deterministic ordering; Biased to keep results stable for tests.

## Contact Semantics
- Contact normal points from object2 to object1.
- Penetration depth is positive for overlap; negative values may be emitted only when explicitly allowed.
- Contact points are in world coordinates and include optional triangle identifiers when mesh data is available.
- Contact forces are not computed by the detector; solver owns force and impulse calculations.

## Distance and Raycast
- Distance queries return the minimum signed distance, with optional nearest points.
- Distance lower bound supports early exit to match current option behavior.
- Raycast supports closest hit or all hits with optional sorting and a filter predicate.

## Filtering
- Pair filtering integrates with existing DART collision and distance filters.
- Self-collision, adjacency, and collidable flags are enforced consistently across all queries.
- Filtering is applied before narrowphase, with a final safety check before result emission.

## Determinism and Threading
- Stable ordering for contacts and ray hits to prevent test flakiness.
- Default single-threaded implementation; parallel stages are optional and isolated.

## Standalone Packaging
- Core is built into the `dart` target for now; no separate CMake target in this phase.
- The adapter remains part of `dart` and preserves the current public API.
- Dependencies remain minimal and align with existing DART build requirements.

## Migration Notes
- Existing detector keys and configuration options remain valid during transition.
- Default detector changes only after parity and performance acceptance.
- Legacy backends remain available until the deprecation milestone is reached.
