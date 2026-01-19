# Resume: Experimental Collision Module

## Current State (2026-01-19)

**Branch**: `feature/new_coll` — all changes committed

**Tests**: 239 passing across 15 test files

**Performance**: VALIDATED — 5-40x faster than FCL/Bullet/ODE in narrow-phase

## What's Complete

| Component | Status | Notes |
|-----------|--------|-------|
| Core types (ContactPoint, Manifold, Result, Ray) | ✅ | 21 tests |
| Aabb | ✅ | 26 tests |
| Shapes (Sphere, Box, Capsule, Cylinder, Plane) | ✅ | 15 tests |
| Sphere-sphere collision | ✅ | 17 tests |
| Box-box collision (SAT) | ✅ | 18 tests |
| Sphere-box collision | ✅ | 19 tests |
| Capsule collision (all pairs) | ✅ | 14 tests |
| Cylinder collision (all pairs) | ✅ | 18 tests |
| Plane collision (all pairs) | ✅ | 11 tests |
| Distance queries (6 pairs) | ✅ | 14 tests |
| Raycast (5 shapes) | ✅ | 24 tests |
| Brute-force broad-phase | ✅ | 15 tests |
| CollisionObject (ECS handle) | ✅ | 12 tests |
| NarrowPhase dispatcher | ✅ | 7 tests |
| CollisionWorld (ECS-based) | ✅ | 8 tests |
| **ECS Refactoring** | ✅ | Handle-based API |
| Comparative benchmarks vs FCL/Bullet/ODE | ✅ | `bm_comparative.cpp` |
| Accuracy verification | ✅ | Matches FCL within tolerance |

## ECS Architecture (Completed)

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

API changed from `addObject(shared_ptr)` to `createObject(unique_ptr<Shape>)`.

## Benchmark Results

| Shape Pair | Experimental | Best Alternative | Speedup |
|------------|-------------|------------------|---------|
| Sphere-Sphere | **41 ns** | 411 ns (Bullet) | **10x** |
| Box-Box | **210 ns** | 1094 ns (Bullet) | **5x** |
| Capsule-Capsule | **41 ns** | 242 ns (FCL) | **6x** |
| Distance | **7 ns** | 288 ns (FCL) | **40x** |

## What's NOT Done

| Component | Priority | Notes |
|-----------|----------|-------|
| MeshShape | Medium | Triangle mesh collision |
| Ray-mesh | Medium | After MeshShape |
| Visual verification tool | Low | Rerun or raylib for debugging |
| Optimized broad-phase | Low | BVH or spatial hash (current is O(N²)) |
| DART integration | Deferred | Wait for feature parity |

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

# 4. Run comparative benchmarks
./bin/bm_comparative --benchmark_filter="BM_SphereSphere|BM_BoxBox"
```

## Key Files to Read

| File | Purpose |
|------|---------|
| `dart/collision/experimental/` | All source code |
| `dart/collision/experimental/collision_world.hpp` | ECS-based world with registry |
| `dart/collision/experimental/collision_object.hpp` | Lightweight handle class |
| `dart/collision/experimental/comps/collision_object.hpp` | ECS components |
| `dart/collision/experimental/narrow_phase/raycast.hpp/.cpp` | Raycast implementation |
| `tests/unit/collision/experimental/` | All unit tests (15 files, 239 tests) |
| `tests/benchmark/collision/bm_comparative.cpp` | Performance benchmarks |
| `docs/dev_tasks/experimental_collision/progress.md` | Detailed progress |

## Key Constraints

1. **Use `Aabb` not `AABB`** — PascalCase for abbreviations
2. **BSD copyright headers only** — no other boilerplate comments
3. **snake_case for file names** — experimental module convention
4. **Determinism required** — bit-exact results, use `std::vector`, ordered iteration
5. **Normal convention** — from object2 to object1 (DART's convention)
6. **BoxShape** — API takes half-extents, stores full extents internally
7. **Handle-based API** — CollisionObject is copyable, cheap to pass by value

## Suggested Next Steps

1. **MeshShape** — For realistic simulation
   - Triangle mesh representation
   - GJK/EPA for mesh-primitive collision (reference: `/home/js/dev/physics/libccd`)

2. **Visual verification** — Debug tool
   - Consider rerun (Python) or raylib (C)
   - Visualize contacts, normals, AABBs

## Commit History (Recent)

```
86891abfadd feat(collision): refactor to ECS architecture with raycast support
0d6273a1832 docs(collision): update RESUME.md for fresh agent session resumption
f9b0b00bb56 perf(collision): add comparative benchmarks against FCL/Bullet/ODE
42109796b7e docs(collision): update RESUME.md with comparative benchmark requirement
5fd50afdbc5 docs(collision): add CRITICAL performance requirement
```
