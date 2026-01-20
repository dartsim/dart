# Resume: Experimental Collision Module

## Current State (2026-01-20)

**Branch**: `feature/new_coll`  
**Build Status**: ✅ Passing  
**Tests**: 403+ tests passing (all broadphase, narrow-phase, CCD, distance, raycast)

## Last Session Summary

Completed runtime broadphase algorithm selection for CollisionWorld:

1. ✅ AABB Tree broadphase — O(n log n) with SAH insertion, fat AABBs, 21 tests
2. ✅ Sweep-and-Prune broadphase — O(n + k) sorted endpoints, 19 tests  
3. ✅ Runtime selection via `BroadPhaseType` enum in CollisionWorld constructor
4. ✅ Benchmarks for all 3 algorithms (QueryPairs, Add, Update, QueryOverlapping)
5. ✅ Documentation updated with broadphase selection guide

## Recent Commits (Committed)

```
c607ab7ff46 docs(experimental_collision): log benchmark run results
83b795c3226 feat(collision): add runtime broadphase selection with Sweep-and-Prune
02c0e35ef54 docs: record rp3d ecs profiling baseline
ab0a1c10581 docs: add batch collision architecture and rp3d plan
b1f6e5ebf05 docs: update progress tracker with AABB tree broadphase
```

## Uncommitted Changes

### Code Changes (from earlier sessions, not yet committed)
| File | Description |
|------|-------------|
| `dart/collision/experimental/narrow_phase/gjk.cpp` | GJK algorithm improvements |
| `dart/collision/experimental/narrow_phase/gjk.hpp` | GJK header updates |
| `dart/collision/experimental/narrow_phase/ccd.cpp` | CCD fixes |

### Documentation Updates (uncommitted)
| File | Description |
|------|-------------|
| `docs/dev_tasks/experimental_collision/design.md` | Added broadphase selection guide |
| `docs/dev_tasks/experimental_collision/progress.md` | Updated test count to 403 |
| Various other docs in `docs/dev_tasks/experimental_collision/` | Minor updates |

### Untracked Files (work in progress from earlier sessions)
| File | Description |
|------|-------------|
| `dart/collision/experimental/narrow_phase/mpr.hpp/cpp` | MPR algorithm (incomplete) |
| `tests/unit/collision/experimental/test_libccd_algorithms.cpp` | libccd comparison tests |
| `tests/benchmark/collision/experimental/bm_libccd.cpp` | libccd benchmarks |
| `cmake/dart_test_libccd.cmake` | CMake for libccd tests |
| `rp3d_profiling_*.txt` | ReactPhysics3D profiling data |

## What's Complete

| Component | Tests | Status |
|-----------|-------|--------|
| **Broadphase: AABB Tree** | 21 | ✅ 95-188x faster than brute-force |
| **Broadphase: Sweep-and-Prune** | 19 | ✅ O(n + k) for mostly-static |
| **Broadphase: Brute Force** | 15 | ✅ O(n²) reference |
| **CollisionWorld** | 17 | ✅ Runtime broadphase selection |
| Narrow-phase (all primitives) | ~100 | ✅ Sphere, Box, Capsule, Cylinder, Plane |
| GJK/EPA | 15 | ✅ Generic convex collision |
| Distance queries | 18 | ✅ 6 primitive pairs + Convex/Mesh |
| Raycast | 39 | ✅ 7 shape types |
| CCD (sphere/capsule cast) | 62 | ✅ Conservative advancement |

## CollisionWorld API

```cpp
// Default: AABB Tree (best general-purpose)
CollisionWorld world;
CollisionWorld world(BroadPhaseType::AabbTree);

// For mostly-static scenes
CollisionWorld world(BroadPhaseType::SweepAndPrune);

// For debugging or very small N
CollisionWorld world(BroadPhaseType::BruteForce);

// Query
BroadPhaseType type = world.getBroadPhaseType();
BroadPhase& bp = world.getBroadPhase();
```

## Broadphase Performance

| Objects | Brute-Force | AABB Tree | Speedup |
|---------|-------------|-----------|---------|
| 10 | 252 ns | 70 ns | 3.6x |
| 50 | 6,944 ns | 568 ns | 12x |
| 100 | 29,543 ns | 1,405 ns | 21x |
| 500 | 1,137,928 ns | 12,024 ns | 95x |
| 1000 | 4,700,044 ns | ~25,000 ns | ~188x |

## How to Resume

```bash
# 1. Checkout and verify state
cd /home/js/dev/dartsim/dart/task_2
git checkout feature/new_coll
git status && git log -3 --oneline

# 2. Build (if needed)
cmake --build build/default/cpp/Release --parallel

# 3. Run tests to verify
./build/default/cpp/Release/bin/test_aabb_tree
./build/default/cpp/Release/bin/test_sweep_and_prune
./build/default/cpp/Release/bin/test_collision_world

# 4. Run benchmarks
./build/default/cpp/Release/bin/bm_experimental_broadphase --benchmark_filter="QueryPairs"
```

## Possible Next Steps

### Option A: Commit Documentation Updates
The design.md and progress.md have useful updates. Could commit them:
```bash
git add docs/dev_tasks/experimental_collision/design.md \
        docs/dev_tasks/experimental_collision/progress.md
git commit -m "docs(collision): update broadphase selection guide and test counts"
```

### Option B: Commit GJK/CCD Improvements
Earlier sessions improved GJK and CCD. Review and commit if stable:
```bash
git diff dart/collision/experimental/narrow_phase/gjk.cpp
git diff dart/collision/experimental/narrow_phase/ccd.cpp
```

### Option C: Continue Feature Development
- **Spatial Hash broadphase** — O(1) for uniform distributions
- **Incremental SAP** — Exploit temporal coherence
- **DART integration** — Wire as CollisionDetector backend
- **Parallel batch collision** — Multi-threaded narrow-phase

### Option D: Clean Up WIP Files
Remove or complete the untracked MPR/libccd work:
- `dart/collision/experimental/narrow_phase/mpr.hpp/cpp`
- `tests/unit/collision/experimental/test_libccd_algorithms.cpp`

## Key Technical Notes

1. **`Aabb.min` and `Aabb.max` are public members** — Access with `[index]` not `(index)`
2. **Use `Aabb` not `AABB`** — PascalCase for abbreviations in this codebase
3. **Determinism required** — All broadphases return sorted pairs for reproducibility
4. **Default broadphase is AabbTree** — Best general-purpose performance

## Key Files Reference

| File | Purpose |
|------|---------|
| `dart/collision/experimental/broad_phase/aabb_tree.hpp` | AABB Tree (SAH, fat AABBs) |
| `dart/collision/experimental/broad_phase/sweep_and_prune.hpp` | SAP (sorted endpoints) |
| `dart/collision/experimental/broad_phase/brute_force.hpp` | O(n²) reference |
| `dart/collision/experimental/collision_world.hpp` | Runtime broadphase selection |
| `dart/collision/experimental/fwd.hpp` | BroadPhaseType enum |
| `tests/unit/collision/experimental/test_aabb_tree.cpp` | 21 AABB Tree tests |
| `tests/unit/collision/experimental/test_sweep_and_prune.cpp` | 19 SAP tests |
| `tests/benchmark/collision/experimental/bm_broadphase.cpp` | Broadphase benchmarks |
| `docs/dev_tasks/experimental_collision/progress.md` | Full component status |
| `docs/dev_tasks/experimental_collision/design.md` | API design and architecture |
