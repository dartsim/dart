# Resume: Experimental Collision Module

## Current State (2026-01-19)

**Branch**: `feature/new_coll` — clean, all changes committed and pushed

**Tests**: 213 passing across 14 test files

**Performance**: VALIDATED — 5-40x faster than FCL/Bullet/ODE in narrow-phase

## What's Complete

| Component | Status | Notes |
|-----------|--------|-------|
| Core types (ContactPoint, Manifold, Result) | ✅ | 21 tests |
| Aabb | ✅ | 26 tests |
| Shapes (Sphere, Box, Capsule, Cylinder, Plane) | ✅ | 15 tests |
| Sphere-sphere collision | ✅ | 17 tests |
| Box-box collision (SAT) | ✅ | 18 tests |
| Sphere-box collision | ✅ | 19 tests |
| Capsule collision (all pairs) | ✅ | 14 tests |
| Cylinder collision (all pairs) | ✅ | 18 tests |
| Plane collision (all pairs) | ✅ | 11 tests |
| Distance queries (6 pairs) | ✅ | 14 tests |
| Brute-force broad-phase | ✅ | 15 tests |
| CollisionObject wrapper | ✅ | 10 tests |
| NarrowPhase dispatcher | ✅ | 7 tests |
| CollisionWorld | ✅ | 8 tests |
| Comparative benchmarks vs FCL/Bullet/ODE | ✅ | `bm_comparative.cpp` |
| Accuracy verification | ✅ | Matches FCL within tolerance |

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
| Raycast support | Medium | Ray-sphere, ray-box, ray-capsule, ray-plane, ray-mesh |
| MeshShape | Medium | Triangle mesh collision |
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
for t in bin/test_types bin/test_aabb bin/test_shapes bin/test_sphere_sphere \
         bin/test_brute_force bin/test_box_box bin/test_sphere_box \
         bin/test_collision_object bin/test_narrow_phase bin/test_collision_world \
         bin/test_capsule_capsule bin/test_cylinder bin/test_plane bin/test_distance; do
  $t 2>&1 | tail -1
done

# 4. Run comparative benchmarks
./bin/bm_comparative --benchmark_filter="BM_SphereSphere|BM_BoxBox"
```

## Key Files to Read

| File | Purpose |
|------|---------|
| `dart/collision/experimental/` | All source code |
| `tests/unit/collision/experimental/` | All unit tests |
| `tests/benchmark/collision/bm_comparative.cpp` | Performance benchmarks |
| `tests/benchmark/collision/bm_experimental.cpp` | Internal benchmarks |
| `docs/dev_tasks/experimental_collision/progress.md` | Detailed progress |
| `docs/dev_tasks/experimental_collision/phases.md` | Phase plan |

## Key Constraints

1. **Use `Aabb` not `AABB`** — PascalCase for abbreviations
2. **BSD copyright headers only** — no other boilerplate comments
3. **snake_case for file names** — experimental module convention
4. **Determinism required** — bit-exact results, use `std::vector`, ordered iteration
5. **Normal convention** — from object2 to object1 (DART's convention)
6. **BoxShape** — API takes half-extents, stores full extents internally

## Suggested Next Steps

1. **Add raycast support** — Most impactful missing feature
   - Start with `Ray` struct in `types.hpp`
   - Implement `raycastSphere()`, `raycastBox()`, etc.
   - Add tests in `tests/unit/collision/experimental/test_raycast.cpp`

2. **Add MeshShape** — For realistic simulation
   - Triangle mesh representation
   - GJK/EPA for mesh-primitive collision

3. **Visual verification** — Debug tool
   - Consider rerun (Python) or raylib (C)
   - Visualize contacts, normals, AABBs

## Commit History (Recent)

```
f9b0b00bb56 perf(collision): add comparative benchmarks against FCL/Bullet/ODE
42109796b7e docs(collision): update RESUME.md with comparative benchmark requirement
5fd50afdbc5 docs(collision): add CRITICAL performance requirement
703f833b24b perf(collision): add comprehensive benchmarks for experimental module
b0cfeeef41e feat(collision): add capsule, cylinder, plane shapes with collision and distance
```
