# Resume: Experimental Collision Module

## Current State (2026-01-19)

**Branch**: `feature/new_coll` — all changes committed

**Tests**: 263 passing across 16 test files

**Performance**: VALIDATED — 5-40x faster than FCL/Bullet/ODE in narrow-phase

## What's Complete

| Component | Status | Notes |
|-----------|--------|-------|
| Core types (ContactPoint, Manifold, Result, Ray) | ✅ | 21 tests |
| Aabb | ✅ | 26 tests |
| Shapes (Sphere, Box, Capsule, Cylinder, Plane, Convex, Mesh) | ✅ | 24 tests |
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
| ECS Refactoring | ✅ | Handle-based API |
| **GJK/EPA Algorithm** | ✅ | 15 tests |
| **ConvexShape** | ✅ | 5 tests |
| **MeshShape** | ✅ | 4 tests |
| Comparative benchmarks vs FCL/Bullet/ODE | ✅ | `bm_comparative.cpp` |
| Accuracy verification | ✅ | Matches FCL within tolerance |

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

| Shape Pair | Experimental | Best Alternative | Speedup |
|------------|-------------|------------------|---------|
| Sphere-Sphere | **41 ns** | 411 ns (Bullet) | **10x** |
| Box-Box | **210 ns** | 1094 ns (Bullet) | **5x** |
| Capsule-Capsule | **41 ns** | 242 ns (FCL) | **6x** |
| Distance | **7 ns** | 288 ns (FCL) | **40x** |

## What's NOT Done

| Component | Priority | Notes |
|-----------|----------|-------|
| GJK-based mesh-primitive collision | Medium | Wire GJK into NarrowPhase for Convex/Mesh shapes |
| Ray-mesh intersection | Medium | After mesh collision |
| Visual verification tool | Low | Raylib available in DART, needs visualizer |
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

## Key Files

| File | Purpose |
|------|---------|
| `dart/collision/experimental/shapes/shape.hpp` | All shape classes including ConvexShape and MeshShape |
| `dart/collision/experimental/narrow_phase/gjk.hpp` | GJK/EPA algorithm |
| `dart/collision/experimental/collision_world.hpp` | ECS-based world with registry |
| `dart/collision/experimental/collision_object.hpp` | Lightweight handle class |
| `tests/unit/collision/experimental/test_gjk.cpp` | GJK tests |
| `tests/unit/collision/experimental/test_shapes.cpp` | Shape tests including Convex/Mesh |

## Key Constraints

1. **Use `Aabb` not `AABB`** — PascalCase for abbreviations
2. **BSD copyright headers only** — no other boilerplate comments
3. **snake_case for file names** — experimental module convention
4. **Determinism required** — bit-exact results, use `std::vector`, ordered iteration
5. **Normal convention** — from object2 to object1 (DART's convention)
6. **BoxShape** — API takes half-extents, stores half-extents internally
7. **Handle-based API** — CollisionObject is copyable, cheap to pass by value

## Suggested Next Steps

1. **Wire GJK into NarrowPhase** — Add Convex/Mesh collision using GJK
   - Add cases to `NarrowPhase::collide()` for ConvexShape and MeshShape
   - Use EPA for penetration depth on intersection

2. **Ray-mesh intersection** — Moller-Trumbore algorithm
   - Iterate triangles, find first hit
   - Add to raycast dispatcher

3. **Visual verification** (optional)
   - Use `DART_BUILD_GUI_RAYLIB=ON`
   - Create simple visualizer for debugging

## Commit History (Recent)

```
c31920a83bc feat(collision): add ConvexShape, MeshShape, and GJK/EPA algorithm
8fbc3fee5e0 docs(collision): update RESUME.md with ECS completion status
86891abfadd feat(collision): refactor to ECS architecture with raycast support
0d6273a1832 docs(collision): update RESUME.md for fresh agent session resumption
f9b0b00bb56 perf(collision): add comparative benchmarks against FCL/Bullet/ODE
```
