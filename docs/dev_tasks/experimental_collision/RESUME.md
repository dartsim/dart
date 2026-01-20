# Resume: Experimental Collision Module

## Current State (2026-01-20)

**Branch**: `feature/new_coll` — mixed primitives + batched raycast + mesh-heavy scenarios + CCD microbench added; local uncommitted unrelated changes present

**Tests**: 292 passing across 17 test files

**Performance**: Baseline validated — 5-40x faster on subset; structured suite complete with edge-case/scale sweeps and cross-backend checks added; results log created

**New in this session**: Added edge-case + scale sweep benchmarks, cross-backend integration test, and `benchmark_results.md`; added batch API scaffolding + pipeline breakdown benchmark; captured ReactPhysics3D 10k profiling summary.

## What's Complete

| Component                                                    | Status | Notes                                                                                                                          |
| ------------------------------------------------------------ | ------ | ------------------------------------------------------------------------------------------------------------------------------ |
| Core types (ContactPoint, Manifold, Result, Ray)             | ✅     | 21 tests                                                                                                                       |
| Aabb                                                         | ✅     | 26 tests                                                                                                                       |
| Shapes (Sphere, Box, Capsule, Cylinder, Plane, Convex, Mesh) | ✅     | 24 tests                                                                                                                       |
| Sphere-sphere collision                                      | ✅     | 17 tests                                                                                                                       |
| Box-box collision (SAT)                                      | ✅     | 18 tests                                                                                                                       |
| Sphere-box collision                                         | ✅     | 19 tests                                                                                                                       |
| Capsule collision (all pairs)                                | ✅     | 14 tests                                                                                                                       |
| Cylinder collision (all pairs)                               | ✅     | 18 tests                                                                                                                       |
| Plane collision (all pairs)                                  | ✅     | 11 tests                                                                                                                       |
| Distance queries (6 pairs + Convex/Mesh)                     | ✅     | 18 tests                                                                                                                       |
| Raycast (6 shapes incl. Mesh)                                | ✅     | 32 tests                                                                                                                       |
| Brute-force broad-phase                                      | ✅     | 15 tests                                                                                                                       |
| CollisionObject (ECS handle)                                 | ✅     | 12 tests                                                                                                                       |
| NarrowPhase dispatcher                                       | ✅     | 7 tests                                                                                                                        |
| CollisionWorld (ECS-based)                                   | ✅     | 8 tests                                                                                                                        |
| ECS Refactoring                                              | ✅     | Handle-based API                                                                                                               |
| **GJK/EPA Algorithm**                                        | ✅     | 15 tests                                                                                                                       |
| **ConvexShape**                                              | ✅     | 5 tests                                                                                                                        |
| **MeshShape**                                                | ✅     | 4 tests                                                                                                                        |
| **GJK-based Convex/Mesh Collision**                          | ✅     | 10 tests                                                                                                                       |
| **Ray-Mesh Intersection**                                    | ✅     | 8 tests (Moller-Trumbore)                                                                                                      |
| **GJK-based Distance (Convex/Mesh)**                         | ✅     | 4 tests                                                                                                                        |
| **Ray-Convex Intersection**                                  | ✅     | 7 tests (GJK-based binary search)                                                                                              |
| Comparative benchmarks vs FCL/Bullet/ODE                     | ✅     | Baseline subset in `bm_comparative.cpp` + structured narrow-phase/distance/raycast in `tests/benchmark/collision/comparative/` |
| Accuracy verification                                        | ✅     | Matches FCL within tolerance                                                                                                   |

## ECS Architecture

```
CollisionWorld
  └── entt::registry
        ├── CollisionObjectTag
        ├── ShapeComponent (unique_ptr<Shape>)
        ├── TransformComponent (Eigen::Isometry3d)
        ├── AabbComponent (Aabb + dirty flag)
        └── UserDataComponent (void*)

CollisionObject (handle) = entt::entity + CollisionWorld*
```

## GJK/EPA Implementation

- `Gjk::query()` - Full GJK with intersection status and separation axis
- `Gjk::intersect()` - Simple boolean intersection test
- `Epa::penetration()` - Penetration depth using Expanding Polytope Algorithm
- Support function interface for any convex shape

## Benchmark Results

| Shape Pair      | Experimental | Best Alternative | Speedup |
| --------------- | ------------ | ---------------- | ------- |
| Sphere-Sphere   | **41 ns**    | 411 ns (Bullet)  | **10x** |
| Box-Box         | **210 ns**   | 1094 ns (Bullet) | **5x**  |
| Capsule-Capsule | **41 ns**    | 242 ns (FCL)     | **6x**  |
| Distance        | **7 ns**     | 288 ns (FCL)     | **40x** |

Baseline only; updated runs should go into `benchmark_results.md`.

## What's NOT Done

| Component                              | Priority   | Notes                                                                                                                 |
| -------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------- |
| ~~GJK-based mesh-primitive collision~~ | ~~Medium~~ | ✅ DONE — `collideConvexConvex()` via GJK                                                                             |
| ~~Ray-mesh intersection~~              | ~~Medium~~ | ✅ DONE — Moller-Trumbore algorithm                                                                                   |
| ~~Distance for Convex/Mesh~~           | ~~Medium~~ | ✅ DONE — `distanceConvexConvex()` via GJK                                                                            |
| ~~Ray-convex intersection~~            | ~~Medium~~ | ✅ DONE — GJK-based point-in-convex + binary search                                                                   |
| Benchmark results log updates          | High       | Run structured suite, capture JSON, update `benchmark_results.md` gates                                               |
| Raycast edge-case expansion (optional) | Medium     | Add grazing/origin-inside cases if needed for coverage                                                               |
| Visual verification tool               | Low        | Raylib available in DART, needs visualizer                                                                            |
| Optimized broad-phase                  | Low        | BVH or spatial hash (current is O(N²))                                                                                |
| DART integration                       | Deferred   | Wait for feature parity                                                                                               |

## How to Resume

```bash
# 1. Checkout and verify
git checkout feature/new_coll
git status && git log -3 --oneline

# 2. Verify build with experimental collision enabled
pixi run cmake -B build/default/cpp/Release -DDART_BUILD_COLLISION_EXPERIMENTAL=ON
pixi run cmake --build build/default/cpp/Release

# 3. Run tests to verify state
cd build/default/cpp/Release
for t in bin/test_*; do $t 2>&1 | tail -1; done
ctest -R INTEGRATION_collision_ExperimentalBackendConsistency --output-on-failure

# 4. Run comparative benchmarks
./tests/benchmark/bm_comparative --benchmark_filter="BM_SphereSphere|BM_BoxBox"
./tests/benchmark/bm_comparative_narrow_phase --benchmark_filter="BM_Collision_.*"
./tests/benchmark/bm_comparative_distance --benchmark_filter="BM_Distance_.*"
./tests/benchmark/bm_comparative_raycast --benchmark_filter="BM_Raycast_.*"
./tests/benchmark/bm_experimental_ccd --benchmark_filter="BM_CCD_.*"
./tests/benchmark/bm_scenarios_mixed_primitives --benchmark_filter="BM_Scenario_.*"
./tests/benchmark/bm_scenarios_raycast_batch --benchmark_filter="BM_Scenario_.*"
./tests/benchmark/bm_scenarios_mesh_heavy --benchmark_filter="BM_Scenario_.*"
```

## Key Files

| File                                                         | Purpose                                               |
| ------------------------------------------------------------ | ----------------------------------------------------- |
| `dart/collision/experimental/shapes/shape.hpp`               | All shape classes including ConvexShape and MeshShape |
| `dart/collision/experimental/narrow_phase/gjk.hpp`           | GJK/EPA algorithm                                     |
| `dart/collision/experimental/narrow_phase/convex_convex.hpp` | Support functions + GJK collision dispatch            |
| `dart/collision/experimental/narrow_phase/raycast.hpp`       | Raycast for all shapes including Mesh                 |
| `dart/collision/experimental/collision_world.hpp`            | ECS-based world with registry                         |
| `dart/collision/experimental/collision_object.hpp`           | Lightweight handle class                              |
| `tests/unit/collision/experimental/test_gjk.cpp`             | GJK tests                                             |
| `tests/unit/collision/experimental/test_convex.cpp`          | Convex/Mesh collision tests                           |
| `tests/unit/collision/experimental/test_raycast.cpp`         | Raycast tests including mesh (32 tests)               |

## Key Constraints

1. **Use `Aabb` not `AABB`** — PascalCase for abbreviations
2. **BSD copyright headers only** — no other boilerplate comments
3. **snake_case for file names** — experimental module convention
4. **Determinism required** — bit-exact results, use `std::vector`, ordered iteration
5. **Normal convention** — from object2 to object1 (DART's convention)
6. **BoxShape** — API takes half-extents, stores half-extents internally
7. **Handle-based API** — CollisionObject is copyable, cheap to pass by value

## Reference Implementations

| Library     | Path                           | Algorithms                             |
| ----------- | ------------------------------ | -------------------------------------- |
| **libccd**  | `/home/js/dev/physics/libccd`  | GJK, EPA, MPR (C style, port to C++20) |
| **FCL**     | `/home/js/dev/physics/fcl`     | BVH mesh-mesh, distance, broad-phase   |
| **Bullet3** | `/home/js/dev/physics/bullet3` | Battle-tested collision (legacy)       |
| **ODE**     | `/home/js/dev/physics/ODE`     | Battle-tested collision (legacy)       |

## Suggested Next Steps

1. **Run structured suite + record results** — update `benchmark_results.md` gates
2. **Cross-backend mismatch triage** — review integration test output, log discrepancies
3. **Parallelization Phase 0** — add instrumentation + 10k object scenarios
4. **BVH broad-phase** — replace brute-force O(N²) with BVH

## Commit History (Recent)

```
0c6fb8e64a1 bench(collision): add experimental CCD benchmarks
dbb161de9dc bench(collision): add mesh-heavy scenario benchmark
22dd147d34f bench(collision): add batched raycast scenario
c03e038fdc0 bench(collision): add mixed primitives scenario benchmark
9853dbb9472 bench(collision): add comparative distance and raycast benchmarks
e029676a1d3 docs(dev_tasks): track distance and raycast benchmarks
81809bee033 docs(collision): expand ecosystem comparison
3edd8e5c1ef docs(dev_tasks): update benchmark plan status
```
