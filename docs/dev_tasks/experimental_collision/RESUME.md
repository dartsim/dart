# Resume: Experimental Collision Module

## Last Session Summary

Completed all primitive shapes and collision pairs. Added comprehensive benchmarks. The module now has **213 tests passing** across 14 test files. All commits pushed to origin.

**Implemented this session:**
- Capsule collision: capsule-capsule, capsule-sphere, capsule-box
- Plane collision: plane-sphere, plane-box, plane-capsule  
- Cylinder collision: cylinder-cylinder, cylinder-sphere, cylinder-box, cylinder-capsule, cylinder-plane
- Distance queries: 6 pairs (sphere-sphere, sphere-box, box-box, capsule-capsule, capsule-sphere, capsule-box)
- Comprehensive benchmarks with scale tests

## ⚠️ CRITICAL: Performance Requirement

**The module MUST outperform existing backends (FCL, Bullet, ODE) to justify its existence.**

Current benchmarks only measure our implementation in isolation. **Next priority: Create comparative benchmarks** (`bm_comparative.cpp`) that test against FCL, Bullet, and ODE for the same scenarios.

If we can't demonstrate performance wins with equal accuracy, this module should not ship.

## Current Branch

`feature/new_coll` — clean, pushed to origin

## Immediate Next Step

**Create comparative benchmarks against FCL, Bullet, ODE:**
1. Create `tests/benchmark/collision/bm_comparative.cpp`
2. For each shape pair, benchmark our implementation vs FCL/Bullet/ODE
3. Verify accuracy parity (contact position, normal, depth)
4. Document results in `docs/dev_tasks/experimental_collision/benchmarks.md`

## Context That Would Be Lost

- **CRITICAL**: Must beat FCL/Bullet/ODE in performance with equal accuracy
- **213 tests passing**: All primitive shapes complete
- **Standalone library focus**: No dynamics dependencies, pure collision testing
- **Normal convention**: From object2 to object1 (DART's convention)
- **Shape pair matrix**: 25 collision combinations complete, 6 distance pairs

## How to Resume

```bash
git checkout feature/new_coll
# Verify state:
git status && git log -5 --oneline
```

Then: Create `bm_comparative.cpp` to benchmark against existing backends.

## Key Constraints

1. **Use `Aabb` not `AABB`** (PascalCase for abbreviations)
2. **BSD copyright headers only** - no other comments unless necessary
3. **snake_case for file names** in experimental modules
4. **Determinism required** - bit-exact results, use `std::vector`, ordered iteration
5. **Performance goal**: Must beat FCL, Bullet, ODE

## Commit History (This Session)

```
5fd50afdbc5 docs(collision): add CRITICAL performance requirement - must beat FCL/Bullet/ODE
44ba53feb56 docs(collision): update progress tracking for Phase 2 completion
703f833b24b perf(collision): add comprehensive benchmarks for experimental module
b0cfeeef41e feat(collision): add capsule, cylinder, plane shapes with collision and distance
```
