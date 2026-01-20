# Experimental Collision Module - Progress Tracker

> **Last Updated**: 2026-01-20
> **Current Focus**: Standalone Library Development

## Status Summary

| Phase                      | Status       | Progress |
| -------------------------- | ------------ | -------- |
| Core Types & Primitives    | **Complete** | 100%     |
| Standalone CollisionWorld  | **Complete** | 100%     |
| Additional Shapes          | **Complete** | 100%     |
| Distance Queries           | **Complete** | 100%     |
| Comparative Benchmarks     | **Partial**  | 90%      |
| Raycast Support            | **Complete** | 100%     |
| ECS Refactoring            | **Complete** | 100%     |
| GJK/EPA Algorithm          | **Complete** | 100%     |
| Convex/Mesh Shapes         | **Complete** | 100%     |
| Continuous Collision (CCD) | **Complete** | 100%     |
| Visual Verification        | **Complete** | 100%     |
| DART Integration           | **Deferred** | -        |

---

## Completed Components (403 tests)

| Component                        | Files                                    | Tests | Notes                                               |
| -------------------------------- | ---------------------------------------- | ----- | --------------------------------------------------- |
| ContactPoint, ContactManifold    | types.hpp/.cpp                           | 21    | Core contact representation                         |
| CollisionResult, CollisionOption | types.hpp/.cpp                           | -     | (included above)                                    |
| Aabb                             | aabb.hpp/.cpp                            | 26    | Axis-aligned bounding box                           |
| All Shapes                       | shapes/shape.hpp/.cpp                    | 24    | Sphere, Box, Capsule, Cylinder, Plane, Convex, Mesh |
| GJK/EPA Algorithm                | narrow_phase/gjk.hpp/.cpp                | 15    | Generic convex collision detection                  |
| Sphere-sphere                    | narrow_phase/sphere_sphere.hpp/.cpp      | 17    | Narrow-phase                                        |
| Box-box (SAT)                    | narrow_phase/box_box.hpp/.cpp            | 18    | Separating Axis Theorem                             |
| Sphere-box                       | narrow_phase/sphere_box.hpp/.cpp         | 19    | Mixed pair                                          |
| Capsule collision                | narrow*phase/capsule*\*.hpp/.cpp         | 14    | All capsule pairs                                   |
| Cylinder collision               | narrow_phase/cylinder_collision.hpp/.cpp | 18    | All cylinder pairs                                  |
| Plane collision                  | narrow_phase/plane_sphere.hpp/.cpp       | 11    | All plane pairs                                     |
| Distance queries                 | narrow_phase/distance.hpp/.cpp           | 18    | 6 primitive pairs + Convex/Mesh                     |
| Raycast                          | narrow_phase/raycast.hpp/.cpp            | 39    | 7 shape types incl. Mesh/Convex                     |
| CCD (sphere/capsule cast)        | narrow_phase/ccd.hpp/.cpp                | 62    | Swept volume + conservative advancement             |
| BruteForceBroadPhase             | broad_phase/brute_force.hpp/.cpp         | 15    | O(n²) broad-phase                                   |
| AabbTreeBroadPhase               | broad_phase/aabb_tree.hpp/.cpp           | 21    | O(n log n) dynamic AABB tree                        |
| SpatialHashBroadPhase            | broad_phase/spatial_hash.hpp/.cpp        | 24    | O(1) avg for uniform distributions                  |
| SweepAndPruneBroadPhase          | broad_phase/sweep_and_prune.hpp/.cpp     | 19    | O(n + k) sorted endpoint lists                      |
| CollisionObject                  | collision_object.hpp/.cpp                | 12    | Lightweight ECS handle                              |
| NarrowPhase                      | narrow_phase/narrow_phase.hpp/.cpp       | 7     | Shape-type dispatch                                 |
| CollisionWorld                   | collision_world.hpp/.cpp                 | 8     | Standalone collision detection                      |

---

## Next Steps

### Priority 1: More Shapes

| Shape         | Implementation | Tests | Notes                                                |
| ------------- | -------------- | ----- | ---------------------------------------------------- |
| CapsuleShape  | **Complete**   | 14    | Sphere-swept line segment                            |
| CylinderShape | **Complete**   | 3     | Axis-aligned cylinder (shape only, no collision yet) |
| PlaneShape    | **Complete**   | 11    | Infinite half-space                                  |
| ConvexShape   | **Complete**   | 5     | Vertex-based convex hull with support function       |
| MeshShape     | **Complete**   | 4     | Triangle mesh with support function                  |

### Priority 2: Shape Pairs ✅ COMPLETE (all primitive pairs)

| Pair              | Status       | Tests | Notes                      |
| ----------------- | ------------ | ----- | -------------------------- |
| Capsule-capsule   | **Complete** | 14    | Common in robotics         |
| Capsule-sphere    | **Complete** | -     | Included in capsule tests  |
| Capsule-box       | **Complete** | -     | Included in capsule tests  |
| Plane-sphere      | **Complete** | 11    | Ground contact             |
| Plane-box         | **Complete** | -     | Included in plane tests    |
| Plane-capsule     | **Complete** | -     | Included in plane tests    |
| Cylinder-cylinder | **Complete** | 18    | Full cylinder support      |
| Cylinder-sphere   | **Complete** | -     | Included in cylinder tests |
| Cylinder-box      | **Complete** | -     | Included in cylinder tests |
| Cylinder-capsule  | **Complete** | -     | Included in cylinder tests |
| Cylinder-plane    | **Complete** | -     | Included in cylinder tests |
| Mesh-primitive    | Pending      |       | GJK/EPA based              |

### Priority 3: Distance Queries ✅ COMPLETE (6 primitive pairs + Convex/Mesh)

| Query Type           | Status       | Notes                |
| -------------------- | ------------ | -------------------- |
| Sphere-sphere dist   | **Complete** | 18 tests total       |
| Sphere-box dist      | **Complete** | Included in distance |
| Box-box dist         | **Complete** | Included in distance |
| Capsule-capsule dist | **Complete** | Included in distance |
| Capsule-sphere dist  | **Complete** | Included in distance |
| Capsule-box dist     | **Complete** | Included in distance |
| Convex-Convex dist   | **Complete** | GJK-based, 4 tests   |
| Convex-Mesh dist     | **Complete** | GJK-based            |
| Mesh-Mesh dist       | **Complete** | GJK-based            |

### Priority 4: Raycast ✅ COMPLETE (7 shapes)

| Feature      | Status       | Tests | Notes                        |
| ------------ | ------------ | ----- | ---------------------------- |
| Ray-sphere   | **Complete** | 5     | Hit/miss, inside, max dist   |
| Ray-box      | **Complete** | 5     | All faces, rotated           |
| Ray-capsule  | **Complete** | 3     | Cylindrical + spherical caps |
| Ray-cylinder | **Complete** | 4     | Curved surface + caps        |
| Ray-plane    | **Complete** | 6     | Backface culling, offset     |
| Ray-mesh     | **Complete** | 8     | Moller-Trumbore algorithm    |
| Ray-convex   | **Complete** | 7     | GJK-based binary search      |

**CollisionWorld raycast API:**

- `raycast(ray, option, result)` - Returns closest hit
- `raycastAll(ray, option, results)` - Returns all hits sorted by distance

### Priority 5: Benchmarks (structured suite) — Complete

Baseline benchmarks exist:

- `tests/benchmark/collision/bm_experimental.cpp` (experimental-only microbenchmarks)
- `tests/benchmark/collision/bm_comparative.cpp` (comparative subset vs FCL/Bullet/ODE)

Structured suite updates:

- Scaffolded `tests/benchmark/collision/{experimental,comparative,scenarios,fixtures,data}`
- Added shared fixtures in `tests/benchmark/collision/fixtures/`
- Added comparative narrow-phase benchmark in `tests/benchmark/collision/comparative/bm_narrow_phase.cpp`
- Added comparative distance benchmark in `tests/benchmark/collision/comparative/bm_distance.cpp`
- Added comparative raycast benchmark in `tests/benchmark/collision/comparative/bm_raycast.cpp` (experimental vs Bullet; FCL/ODE not available)
- Added mixed primitives scenario benchmark in `tests/benchmark/collision/scenarios/bm_mixed_primitives.cpp` (dense/sparse; experimental vs FCL/Bullet/ODE)
- Added batched raycast scenario in `tests/benchmark/collision/scenarios/bm_raycast_batch.cpp` (experimental vs Bullet)
- Raycast batch uses SweepAndPrune broadphase and 500 rays with 1k/2k objects for bounded runtime
- Added mesh-heavy scenario in `tests/benchmark/collision/scenarios/bm_mesh_heavy.cpp` (experimental vs FCL/Bullet)
- Added pipeline breakdown scenario in `tests/benchmark/collision/scenarios/bm_pipeline_breakdown.cpp` (AABB update + broadphase + narrowphase timing for 1k/10k)
- Added experimental CCD microbench in `tests/benchmark/collision/experimental/bm_ccd.cpp`
- Added edge-case + scale sweep coverage for comparative narrow-phase and distance (touching, deep penetration, grazing, thin features; 1e-3, 1, 1e3)
- Added fairness-aligned distance options (nearest points + no early exit) across backends
- Added cross-backend correctness integration test (experimental vs FCL/Bullet/ODE)
- Added benchmark results log in `docs/dev_tasks/experimental_collision/benchmark_results.md`

Structured suite coverage (complete):

1. Directory layout in `tests/benchmark/collision/`:
   - `experimental/` microbenchmarks (narrow phase, distance, raycast, CCD)
   - `comparative/` side-by-side vs FCL/Bullet/ODE
   - `scenarios/` world and broad-phase scaling/mixed scenes
   - `fixtures/` shared shape and scene builders with fixed RNG seeds
   - `data/` meshes/convex fixtures used by multiple benchmarks
2. Coverage matrix:
   - Narrow phase: primitive pairs with touching/deep/grazing/thin-feature regimes and 1e-3/1/1e3 sweeps
   - Distance: same matrix with edge-case regimes and 1e-3/1/1e3 sweeps
   - Raycast: per shape baseline + batch scenarios (hit/miss mix)
   - CCD: sphere-cast, capsule-cast, conservative advancement
   - World/broad-phase: dense/sparse mixed scenes and mesh-heavy variants
3. Fairness and determinism:
   - Pre-create shapes/groups outside timed loops
   - Match options (contacts, distance settings) across backends
   - Fixed seeds and shared fixtures for identical inputs
4. Metrics and gates:
   - Benchmark results recorded in `benchmark_results.md`
   - Cross-backend correctness checks via integration tests
   - Experimental must meet or beat best backend per supported case

Baseline and structured results are consolidated in
`docs/dev_tasks/experimental_collision/benchmark_results.md`.

### Priority 6: ECS Refactoring (Next)

**Goal:** Adopt ECS pattern from `dart/simulation/experimental` for better data locality.

Current architecture uses `shared_ptr<CollisionObject>`. Target architecture:

| Current                       | Target (ECS)                               |
| ----------------------------- | ------------------------------------------ |
| `shared_ptr<CollisionObject>` | Lightweight handle (entity ID + world ptr) |
| Shape\* owned by object       | ShapeComponent in registry                 |
| Transform in object           | TransformComponent in registry             |
| AABB in object                | AabbComponent in registry                  |
| `std::vector<shared_ptr>`     | `entt::registry`                           |

**Benefits:**

- Better cache locality for iteration
- Copyable handles (no pointer ownership issues)
- Consistent with simulation/experimental pattern

### Priority 7: Continuous Collision Detection (CCD) ✅ COMPLETE

CCD prevents fast-moving objects from tunneling through thin obstacles.

| Algorithm                    | Status       | Tests | Notes                                               |
| ---------------------------- | ------------ | ----- | --------------------------------------------------- |
| Sphere-cast (all primitives) | **Complete** | 37    | Sphere, Box, Capsule, Cylinder, Plane, Convex, Mesh |
| Capsule-cast (all shapes)    | **Complete** | 16    | Sphere, Box, Capsule, Plane, Cylinder, Convex, Mesh |
| Conservative advancement     | **Complete** | 4     | General convex-convex via GJK                       |
| NarrowPhase dispatcher       | **Complete** | 3     | `sphereCast()`, `isSphereCastSupported()`           |

**Implemented API:**

```cpp
// Sphere-cast functions (sweep sphere against static target)
bool sphereCastSphere/Box/Capsule/Cylinder/Plane/Convex/Mesh(...);

// Capsule-cast functions (sweep capsule against static target)
bool capsuleCastSphere/Box/Capsule/Plane/Cylinder/Convex/Mesh(...);

// Conservative advancement for general convex pairs
bool conservativeAdvancement(shapeA, tfAStart, tfAEnd, shapeB, tfB, option, result);

// NarrowPhase dispatcher
bool NarrowPhase::sphereCast(start, end, radius, target, option, result);
bool NarrowPhase::isSphereCastSupported(ShapeType type);
```

### Priority 8: Visual Verification — ✅ COMPLETE (Using VSG)

**Implementation**: `dart/gui/vsg/` module with `examples/collision_viz/`

| Feature                         | Status       | Notes                                     |
| ------------------------------- | ------------ | ----------------------------------------- |
| Primitive shapes                | **Complete** | Sphere, Box, Capsule, Cylinder, Plane     |
| Convex/Mesh shapes              | **Complete** | Via GeometryBuilders                      |
| Contact points                  | **Complete** | Red spheres with configurable size        |
| Contact normals                 | **Complete** | Yellow arrows with configurable length    |
| AABB wireframes                 | **Complete** | `createWireframeBox()` + `addAabb()`      |
| Distance query visualization    | **Complete** | `addDistanceResult()` - points + line     |
| Raycast visualization           | **Complete** | `addRaycast()` - ray + hit point + normal |
| Sphere-cast / CCD visualization | **Complete** | `addSphereCast()` - trajectory + hit      |
| Grid and coordinate axes        | **Complete** | `addGrid()` + `addAxes()`                 |
| Headless rendering              | **Complete** | `SimpleViewer::headless()` + PPM export   |
| Interactive windowed mode       | **Complete** | Mouse rotation/zoom, close to exit        |

**Key APIs:**

```cpp
// CollisionSceneBuilder
builder.addObject(obj, color);           // Render collision object
builder.addContacts(result, normalLen);  // Contact points + normals
builder.addAabb(aabb, color);            // AABB wireframe
builder.addDistanceResult(result);       // Distance query visualization
builder.addRaycast(ray, hit);            // Raycast visualization
builder.addSphereCast(start, end, r);    // CCD visualization

// DebugDraw helpers
createWireframeBox(min, max, color);     // AABB edges
createLine(start, end, color);           // Single line
createArrow(start, dir, len, color);     // Arrow with cone head
createPoint(pos, size, color);           // Sphere marker
createGrid(size, spacing, color);        // Ground grid
createAxes(length);                      // RGB XYZ axes
```

**Example usage:**

```bash
# Interactive mode
./build/default/cpp/Release/bin/collision_viz

# Headless mode (CI verification)
./build/default/cpp/Release/bin/collision_viz --headless
# Outputs: collision_viz_headless.ppm
```

---

## Module Structure

```
dart/collision/experimental/
├── CMakeLists.txt              # Build configuration
├── export.hpp                  # DLL export macros
├── fwd.hpp                     # Forward declarations
├── types.hpp/.cpp              # ContactPoint, Ray, RaycastResult, etc.
├── aabb.hpp/.cpp               # Axis-aligned bounding box
├── collision_object.hpp/.cpp   # Shape + Transform wrapper
├── collision_world.hpp/.cpp    # Standalone collision detection
├── shapes/
│   └── shape.hpp/.cpp          # All primitive shapes
├── narrow_phase/
│   ├── narrow_phase.hpp/.cpp   # Shape-type dispatch
│   ├── sphere_sphere.hpp/.cpp  # Sphere-sphere detection
│   ├── box_box.hpp/.cpp        # Box-box SAT detection
│   ├── sphere_box.hpp/.cpp     # Sphere-box detection
│   ├── capsule_*.hpp/.cpp      # Capsule collision pairs
│   ├── cylinder_collision.hpp/.cpp  # Cylinder collision pairs
│   ├── plane_sphere.hpp/.cpp   # Plane collision pairs
│   ├── distance.hpp/.cpp       # Distance queries
│   └── raycast.hpp/.cpp        # Ray-shape intersections
└── broad_phase/
    ├── broad_phase.hpp         # BroadPhase interface
    ├── brute_force.hpp/.cpp    # O(n^2) broad-phase
    ├── aabb_tree.hpp/.cpp      # O(n log n) dynamic AABB tree
    └── sweep_and_prune.hpp/.cpp # O(n + k) sorted endpoints
```

---

## Build Commands

```bash
# Configure with experimental collision
pixi run cmake -B build -DDART_BUILD_COLLISION_EXPERIMENTAL=ON -DDART_BUILD_TESTS=ON

# Build all experimental tests
cmake --build build --target dart_collision_experimental_tests

# Run all tests
for t in test_types test_aabb test_shapes test_sphere_sphere test_brute_force \
         test_box_box test_sphere_box test_collision_object test_narrow_phase \
         test_collision_world; do
  ./build/bin/$t
done
```

---

## Decisions Log

| Date       | Decision                                      | Rationale                                        |
| ---------- | --------------------------------------------- | ------------------------------------------------ |
| 2026-01-19 | Defer DART integration                        | Build standalone library to feature parity first |
| 2026-01-19 | Use `dart::collision::experimental` namespace | Matches simulation/experimental pattern          |
| 2026-01-19 | Keep DART's normal convention (obj2 -> obj1)  | Backward compatibility                           |
| 2026-01-19 | Design manifold-based contacts                | Future extensibility for deformable/cloth        |
| 2026-01-19 | Start with brute-force broad-phase            | Simple first, optimize later                     |
| 2026-01-19 | Standalone shapes (no dynamics dep)           | Enables pure collision testing                   |
| 2026-01-19 | Use PascalCase for abbreviations (Aabb)       | Codebase convention                              |
| 2026-01-19 | BoxShape stores full extents internally       | API takes half-extents for compatibility         |
| 2026-01-19 | AABB Tree with SAH insertion                  | O(n log n) vs O(n²), 95-188x speedup at scale    |
| 2026-01-19 | Fat AABBs in AABB Tree                        | Reduce update frequency for small movements      |
| 2026-01-19 | CollisionWorld as standalone API              | Simpler than full CollisionDetector integration  |
| 2026-01-19 | Runtime broadphase selection via enum         | AabbTree (default), SweepAndPrune, BruteForce    |
| 2026-01-19 | Sweep-and-Prune using sorted endpoint lists   | O(n + k) queries, good for mostly-static scenes  |

---

## Broad-Phase Next Steps

### Priority 1: Fix AabbTree Crash (Blocking) ✅ FIXED

**Issue**: Segfault in `AabbTreeBroadPhase::combine()` during object creation.
Blocks RP3D-aligned pipeline breakdown benchmarks with AabbTree.

**Root cause**: Dangling reference after vector reallocation. In `insertLeaf()`,
a reference to `nodes_[leafIndex].fatAabb` was stored, then `allocateNode()` was
called which could trigger `nodes_.emplace_back()` and reallocate the vector,
invalidating the reference.

**Fix**: Copy AABB values before calling `allocateNode()` instead of storing
references.

**Status**: Fixed (2026-01-20)

### Priority 2: Bulk API Interfaces (ECS Phase 1) ✅ DONE

Added batch-friendly methods to `BroadPhase` interface:

```cpp
void queryPairs(std::vector<BroadPhasePair>& out) const;  // caller-owned storage
void build(std::span<const std::size_t> ids, std::span<const Aabb> aabbs);
void updateRange(std::span<const std::size_t> ids, std::span<const Aabb> aabbs);
```

**Implementation**: Default implementations in base class with `using` declarations
in derived classes (AabbTree, BruteForce, SpatialHash, SweepAndPrune).

**Status**: Complete (2026-01-20)

### Priority 3: Parallel Broadphase (Phase 4 - Optional)

- Parallel AABB recompute and fat-AABB update
- Optional bulk rebuild path for large scene edits
- Exit criteria: Broadphase update scales for large object counts

**Status**: Deferred - Phase 3 (Parallel Narrowphase) should come first per parallelization plan.
See `parallelization_plan.md` for the full roadmap.

---

## Blockers

- ~~**AabbTree segfault**: Crash in `combine()` during object creation.~~ **FIXED** (2026-01-20)
- `bm_comparative_narrow_phase` accuracy mismatch for Cylinder-Box (Experimental
  reports no collision, FCL reports collision).
- `bm_comparative` accuracy warns on box-box depth (expected ~0.2, got ~1.2);
  needs consistency check vs FCL contact depth interpretation.
