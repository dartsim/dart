# Resume: Experimental Collision Module

## Last Session Summary (2026-02-09)

Completed all remaining "Immediate Next Steps" from the previous session: distance query wiring, raycast wiring, expanded shape adapters (Cone, Ellipsoid, Heightmap, MultiSphere), integration test expansion, and documentation updates. Merged latest `origin/main` into `feature/new_coll` (3 conflicts resolved). All backend unit tests pass (14+ cases). Integration tests pass for experimental backend.

## Current Branch

`feature/new_coll` — **ahead of origin by 33 commits** (uncommitted changes pending commit)

```bash
git checkout feature/new_coll
git status && git log -3 --oneline
```

## What Was Completed This Session

### Merge: origin/main → feature/new_coll ✅

- Resolved conflicts in `.gitignore`, `CHANGELOG.md`, `tests/unit/CMakeLists.txt`

### Distance Query Wiring ✅

- Replaced both stubbed `distance()` methods with real implementations
- Constructs temporary `experimental::CollisionObject` wrappers per pair
- Supports DistanceFilter, distanceLowerBound clamping, nearest points, ShapeFrame assignment
- Both single-group and cross-group overloads
- File: `dart/collision/experimental_backend/experimental_collision_detector.cpp`

### Raycast Wiring ✅

- Constructs `experimental::Ray` from `from`/`to` vectors
- Per-object filter via `RaycastOption::passesFilter`
- All-hits vs closest-only, sort-by-closest behavior
- Converts `experimental::RaycastResult` → DART `RayHit`
- Files: `experimental_collision_detector.hpp` (declaration), `.cpp` (implementation)

### Shape Adapter Expansion ✅

- ConeShape → ConvexShape (32 base points + apex)
- Non-sphere EllipsoidShape → ConvexShape (latitude/longitude surface sampling)
- HeightmapShape<float/double> → triangulated MeshShape
- MultiSphereConvexHullShape → ConvexShape (surface point sampling)
- File: `dart/collision/experimental_backend/shape_adapter.cpp`

### Unit Tests ✅

- Added 14+ test cases: DistanceSphereSphere, DistanceOverlapping, DistanceCrossGroup, DistanceWithFilter, RaycastHit, RaycastMiss, RaycastMultipleObjects, RaycastFilter, ConeShapeAdapter, EllipsoidShapeAdapter, HeightmapShapeAdapter
- File: `tests/unit/collision/test_experimental_backend.cpp`

### Integration Test Expansion ✅

- Added `"experimental"` to `INSTANTIATE_TEST_SUITE_P` Values in `test_collision_groups.cpp`
- CollisionGroups and consistency tests pass with experimental backend

### Documentation Updates ✅

- Updated `gap_analysis.md` — shapes at 100%, all critical gaps resolved
- Updated `progress.md` — added backend distance/raycast/shape adapter rows, test count 575+

## Previous Session Work (2026-02-03)

### F1: Compound Shapes ✅

### F2: Parallel Narrowphase ✅

### F3: DART Backend Integration ✅

### F4: VSG CI Polish ✅

## Test Status

```bash
# Collision-experimental: 28/29 pass (1 pre-existing raycast edge case)
ctest --test-dir build/default/cpp/Release -L collision-experimental -j$(nproc)

# DART backend: 1/1 pass (14+ individual test cases inside)
ctest --test-dir build/default/cpp/Release -R UNIT_collision_ExperimentalBackend

# Integration: CollisionGroups passes with "experimental" backend
ctest --test-dir build/default/cpp/Release -R INTEGRATION_collision_CollisionGroups
```

## Remaining Work

The core implementation is **feature-complete**. Remaining items are optimization/polish:

1. **Performance benchmarking** — Run comparative benchmarks vs FCL/Bullet/ODE
2. **Performance parity with FCL** — Optimize until experimental is faster
3. **Persistent manifolds** (P1 from gap analysis)
4. **Contact warm-starting** (P2)
5. **BVH traversal for mesh-mesh** (P1)
6. **Full integration test suite** — Some existing integration tests have pre-existing SEGFAULTs unrelated to experimental backend

## Context That Would Be Lost

- **BoxShape convention difference**: `dynamics::BoxShape` uses full size, `experimental::BoxShape` uses half-extents. Shape adapter does `size * 0.5`.
- **Normal convention**: Both use obj2→obj1, so no conversion needed.
- **Pre-existing test failures**: `test_raycast` has `RayStartsInside` failures (Box, Capsule) — known edge cases, not from our changes.
- **Distance implementation**: Creates temporary `experimental::CollisionObject` wrappers per pair, calls `NarrowPhase::distance()`, converts results.
- **Raycast implementation**: Iterates group objects, constructs `experimental::Ray` from from/to vectors, filters per-object.

## How to Resume

```bash
cd /home/js/dev/dartsim/dart/task_2
git checkout feature/new_coll
git status && git log -3 --oneline

pixi run build
ctest --test-dir build/default/cpp/Release -L collision-experimental -j$(nproc)
ctest --test-dir build/default/cpp/Release -R UNIT_collision_ExperimentalBackend
```

Then choose from "Remaining Work" above.

## Key Files Reference

| Component            | Location                                                                  |
| -------------------- | ------------------------------------------------------------------------- |
| Backend detector     | `dart/collision/experimental_backend/experimental_collision_detector.cpp` |
| Backend header       | `dart/collision/experimental_backend/experimental_collision_detector.hpp` |
| Shape adapter        | `dart/collision/experimental_backend/shape_adapter.cpp`                   |
| Backend test         | `tests/unit/collision/test_experimental_backend.cpp`                      |
| Integration test     | `tests/integration/collision/test_collision_groups.cpp`                   |
| Compound shapes      | `dart/collision/experimental/shapes/shape.hpp`                            |
| Parallel narrowphase | `dart/collision/experimental/collision_world.cpp`                         |
| Progress docs        | `docs/dev_tasks/experimental_collision/progress.md`                       |
| Gap analysis         | `docs/dev_tasks/experimental_collision/gap_analysis.md`                   |
