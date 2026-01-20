# Experimental Collision Module - Implementation Tasks

> **Last Updated**: 2026-01-19

## First 10 Implementation Tasks

These are small, reviewable increments ordered by dependency.

---

### Task 1: Create Module Skeleton

**Goal**: Minimal buildable module structure

**Files**:

```
dart/collision/experimental/
├── CMakeLists.txt
├── export.hpp
└── fwd.hpp
```

**Acceptance Criteria**:

- [ ] `pixi run build` succeeds
- [ ] Module is compiled (even if empty)
- [ ] Export macros defined correctly

**Estimated Time**: 2 hours

---

### Task 2: Implement ContactPoint and CollisionResult

**Goal**: Core contact data structures with tests

**Files**:

```
dart/collision/experimental/
├── types.hpp
└── types.cpp

tests/unit/collision/experimental/
├── CMakeLists.txt
├── helpers/
│   └── collision_test_utils.hpp
└── test_types.cpp
```

**Acceptance Criteria**:

- [ ] ContactPoint struct with all fields
- [ ] CollisionResult with add/clear/query methods
- [ ] Unit tests for all methods
- [ ] Tests pass

**Estimated Time**: 3 hours

---

### Task 3: Implement Aabb

**Goal**: Axis-aligned bounding box with tests and benchmark

**Files**:

```
dart/collision/experimental/
├── aabb.hpp
└── aabb.cpp

tests/unit/collision/experimental/
└── test_aabb.cpp

tests/benchmark/collision/experimental/
├── CMakeLists.txt
└── bm_aabb.cpp
```

**Acceptance Criteria**:

- [ ] Aabb class with overlaps(), contains(), merge(), expand()
- [ ] Factory methods: forSphere(), forBox(), transformed()
- [ ] Unit tests for all methods including edge cases
- [ ] Benchmark for overlaps() performance
- [ ] Tests pass

**Estimated Time**: 3 hours

---

### Task 4: Implement Standalone Shape Classes

**Goal**: Sphere and Box shapes without dynamics dependency

**Files**:

```
dart/collision/experimental/
└── shapes/
    ├── shape.hpp
    ├── sphere.hpp
    ├── sphere.cpp
    ├── box.hpp
    └── box.cpp

tests/unit/collision/experimental/
└── test_shapes.cpp
```

**Acceptance Criteria**:

- [ ] Shape base class with getType(), computeLocalAabb()
- [ ] SphereShape with radius
- [ ] BoxShape with halfExtents
- [ ] Unit tests for AABB computation
- [ ] Tests pass

**Estimated Time**: 3 hours

---

### Task 5: Implement Sphere-Sphere Collision

**Goal**: First narrow-phase algorithm with comprehensive tests

**Files**:

```
dart/collision/experimental/
└── narrow_phase/
    ├── narrow_phase.hpp
    ├── sphere_sphere.hpp
    └── sphere_sphere.cpp

tests/unit/collision/experimental/
└── test_sphere_sphere.cpp

tests/benchmark/collision/experimental/
└── bm_sphere_sphere.cpp
```

**Acceptance Criteria**:

- [ ] collideSphereSphere() function
- [ ] Correct contact point, normal, depth
- [ ] Edge cases: touching, concentric, separated
- [ ] Determinism test (100 runs identical)
- [ ] Performance benchmark
- [ ] Tests pass

**Test Cases**:

```
1. Separated spheres → no collision
2. Touching spheres → depth = 0
3. Overlapping spheres → correct depth
4. Concentric spheres → valid normal (not NaN)
5. Same sphere → self-collision handling
```

**Estimated Time**: 4 hours

---

### Task 6: Implement Brute-Force Broad-Phase

**Goal**: Simple O(n^2) broad-phase for small object counts

**Files**:

```
dart/collision/experimental/
└── broad_phase/
    ├── broad_phase.hpp
    ├── brute_force.hpp
    └── brute_force.cpp

tests/unit/collision/experimental/
└── test_brute_force_broad_phase.cpp

tests/benchmark/collision/experimental/
└── bm_broad_phase.cpp
```

**Acceptance Criteria**:

- [ ] BroadPhase interface
- [ ] BruteForceBroadPhase implementation
- [ ] Deterministic pair ordering
- [ ] add/remove/update/clear operations
- [ ] Scaling benchmark (n objects)
- [ ] Tests pass

**Estimated Time**: 4 hours

---

### Task 7: Implement Box-Box Collision (SAT)

**Goal**: Most complex primitive narrow-phase

**Files**:

```
dart/collision/experimental/
└── narrow_phase/
    ├── box_box.hpp
    └── box_box.cpp

tests/unit/collision/experimental/
└── test_box_box.cpp

tests/benchmark/collision/experimental/
└── bm_box_box.cpp
```

**Acceptance Criteria**:

- [ ] collideBoxBox() using Separating Axis Theorem
- [ ] Correct contact points for face/edge/vertex contacts
- [ ] Edge cases: touching faces, edge-edge, aligned boxes
- [ ] Determinism test
- [ ] Performance benchmark
- [ ] Tests pass

**Test Cases**:

```
1. Separated boxes → no collision
2. Face-face contact → multiple contacts, shared normal
3. Edge-edge contact → line contact
4. Vertex-face contact → single point
5. Identical overlapping boxes
6. Rotated boxes at various angles
```

**Estimated Time**: 8 hours

---

### Task 8: Implement Sphere-Box Collision

**Goal**: Mixed-shape narrow-phase

**Files**:

```
dart/collision/experimental/
└── narrow_phase/
    ├── sphere_box.hpp
    └── sphere_box.cpp

tests/unit/collision/experimental/
└── test_sphere_box.cpp

tests/benchmark/collision/experimental/
└── bm_sphere_box.cpp
```

**Acceptance Criteria**:

- [ ] collideSphereBox() function
- [ ] Handles sphere vs face, edge, vertex
- [ ] Correct contact geometry
- [ ] Determinism test
- [ ] Performance benchmark
- [ ] Tests pass

**Test Cases**:

```
1. Sphere outside box → no collision
2. Sphere touching box face → single contact
3. Sphere touching box edge → single contact
4. Sphere touching box corner → single contact
5. Sphere inside box → correct depth
```

**Estimated Time**: 4 hours

---

### Task 9: Implement CollisionObject Wrapper

**Goal**: Object that combines shape + transform

**Files**:

```
dart/collision/experimental/
├── collision_object.hpp
└── collision_object.cpp

tests/unit/collision/experimental/
└── test_collision_object.cpp
```

**Acceptance Criteria**:

- [ ] CollisionObject holds Shape + Transform
- [ ] Caches world-space AABB
- [ ] updateAabb() method
- [ ] Integration with BroadPhase
- [ ] Tests pass

**Estimated Time**: 3 hours

---

### Task 10: Integration and Baseline Documentation

**Goal**: Wire everything together, document baselines

**Files**:

```
dart/collision/experimental/
├── collision_group.hpp    # Simple group for testing
└── collision_group.cpp

tests/unit/collision/experimental/
└── test_collision_group.cpp

docs/dev_tasks/experimental_collision/
└── baselines.md
```

**Acceptance Criteria**:

- [ ] Simple CollisionGroup that orchestrates broad+narrow phase
- [ ] End-to-end test with multiple objects
- [ ] Performance baselines documented
- [ ] All Phase 1 tests pass
- [ ] All Phase 1 benchmarks run

**Estimated Time**: 4 hours

---

## Summary

| Task      | Description                   | Hours  | Dependencies |
| --------- | ----------------------------- | ------ | ------------ |
| 1         | Module skeleton               | 2      | None         |
| 2         | ContactPoint, CollisionResult | 3      | Task 1       |
| 3         | Aabb                          | 3      | Task 1       |
| 4         | Shape classes                 | 3      | Task 3       |
| 5         | Sphere-sphere collision       | 4      | Tasks 2, 4   |
| 6         | Brute-force broad-phase       | 4      | Tasks 3, 9   |
| 7         | Box-box collision (SAT)       | 8      | Tasks 2, 4   |
| 8         | Sphere-box collision          | 4      | Tasks 5, 7   |
| 9         | CollisionObject wrapper       | 3      | Tasks 3, 4   |
| 10        | Integration + baselines       | 4      | All above    |
| **Total** |                               | **38** |              |

## Dependency Graph

```
Task 1 (skeleton)
    │
    ├──► Task 2 (types) ──────────────────┐
    │                                     │
    └──► Task 3 (Aabb) ──► Task 4 (shapes)┼──► Task 5 (sphere-sphere)
              │                           │            │
              │                           │            ▼
              └──► Task 9 (object) ◄──────┘    Task 8 (sphere-box)
                         │                            ▲
                         ▼                            │
                   Task 6 (broad) ──► Task 7 (box-box)┘
                         │
                         ▼
Task 10 (integration)
```

## Benchmark Suite Plan (Phase 5)

**Goal**: Structured, comparable benchmarks vs FCL/Bullet/ODE with complete
coverage and shared fixtures.

**Proposed layout** (under `tests/benchmark/collision/`):

```
experimental/  # microbenchmarks (narrow phase, distance, raycast, CCD)
comparative/   # side-by-side vs FCL/Bullet/ODE
scenarios/     # world and broad-phase scaling, mixed scenes
fixtures/      # shared shape + scene builders, fixed RNG seeds
data/          # mesh/convex fixtures used by multiple benchmarks
```

**Benchmark coverage**:

- Narrow-phase: all primitive pairs + convex/mesh where supported
- Distance: same matrix with separated/touching/penetrating regimes
- Raycast: per shape with hit/miss/grazing and backface culling toggles
- CCD: sphere-cast, capsule-cast, conservative advancement
- World/broad-phase: N-body scaling with mixed scenes
- Edge-case matrix: grazing contacts, near-parallel faces, thin features, deep penetration
- Scale sweeps: tiny/nominal/large (1e-3, 1, 1e3) for both shape size and transforms

**Current status (2026-01-19):**

- Scaffolded directory layout and shared fixtures
- Comparative narrow-phase benchmark added (`comparative/bm_narrow_phase.cpp`)
- Comparative distance benchmark added (`comparative/bm_distance.cpp`)
- Comparative raycast benchmark added (`comparative/bm_raycast.cpp`, experimental vs Bullet)
- Mixed primitives scenario benchmark added (`scenarios/bm_mixed_primitives.cpp`, dense/sparse)
- Batched raycast scenario benchmark added (`scenarios/bm_raycast_batch.cpp`, experimental vs Bullet)
- Mesh-heavy scenario benchmark added (`scenarios/bm_mesh_heavy.cpp`, experimental vs FCL/Bullet)
- Experimental CCD microbench added (`experimental/bm_ccd.cpp`)
- Edge-case + scale sweep coverage added for comparative narrow-phase and distance
- Cross-backend consistency integration test added (experimental vs FCL/Bullet/ODE)
- Benchmark results log added (`docs/dev_tasks/experimental_collision/benchmark_results.md`)

**Acceptance criteria**:

- Baseline runs are reproducible and use shared fixtures/options
- Comparative benchmarks include FCL, Bullet, and ODE where supported
- Edge-case and scale-sweep benchmarks are present for each query type
- Google Benchmark JSON output is captured for regression tracking

---

## Cross-Backend Validation Tests

These are correctness checks (not performance) that run the same cases through
experimental + reference backends and compare results within tolerances.

### Task V1: Pairwise Reference Cross-Checks

**Goal**: Verify collision, distance, and raycast consistency across backends.

**Acceptance Criteria**:

- [x] Shared fixtures drive the same inputs for experimental, FCL, Bullet, ODE
- [x] Collision: consistent hit/no-hit and depth within tolerance
- [x] Distance: consistent distance within tolerance (signed or near-zero)
- [ ] Raycast: consistent hit/no-hit and distance ordering (when supported)
- [x] Normals compared with DART convention alignment checks
- [ ] Known discrepancies documented with rationale (e.g., Bullet margins)
- [ ] If results disagree, isolate with analytic references for primitives and
      record a follow-up task for the suspected backend

---

## Batch Benchmarks and Ordering (Cross-cutting)

These tasks apply across later phases; keep them tracked separately from the
first 10 tasks to avoid mixing early scaffolding with batch-scale work.

### Task B1: Batch Benchmark Scenarios

**Goal**: Implement batch-scale benchmarks defined in `phases.md`.

**Acceptance Criteria**:

- [ ] Mixed primitives scene at 1k and 10k objects (dense/sparse added, more variants needed)
- [ ] Dense vs sparse distributions (broadphase stress)
- [ ] Mesh-heavy scene (static meshes + moving primitives) (experimental + FCL/Bullet added)
- [ ] Batched raycasts (10k rays) with hit/miss mix (experimental + Bullet added)
- [ ] Metrics include pairs/sec, queries/sec, per-frame time, and memory stats
- [ ] Comparative results against FCL/Bullet/ODE

### Task B2: Deterministic Batch Ordering

**Goal**: Enforce deterministic ordering for batched query results.

**Acceptance Criteria**:

- [ ] Stable object IDs assigned on creation
- [ ] collideAll/distanceAll results sorted by `(id1, id2, query_index)`
- [ ] raycastAll results sorted by `(distance, object_id)`
- [ ] Tests cover order independence and multi-threaded merge determinism
