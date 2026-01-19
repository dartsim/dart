# Resume: Experimental Collision Module

## Last Session Summary

Added CapsuleShape, CylinderShape, and PlaneShape with full narrow-phase collision detection. The module now has **181 tests passing** across 12 test files. Implemented:

- Capsule collision: capsule-capsule, capsule-sphere, capsule-box
- Plane collision: plane-sphere, plane-box, plane-capsule

## Current Branch

`feature/new_coll` — uncommitted changes (new shapes and collision code)

## Immediate Next Step

Continue standalone library development. Options:

1. **Add cylinder collision pairs** - CylinderShape exists but no collision yet
2. **Add distance queries** - Signed distance, closest points
3. **Add benchmarks** - Establish performance baselines
4. **Add visual verification GUI** - Debug tool

Recommend: Add cylinder collision pairs to complete primitive shapes.

## Context That Would Be Lost

- **Direction change**: DART API integration is DEFERRED until feature parity
- **149 tests passing**: All current components well-tested
- **Standalone library focus**: No dynamics dependencies, pure collision testing
- **Normal convention**: From object2 to object1 (DART's convention)
- **BoxShape**: API takes half-extents, stores full extents internally

## How to Resume

```bash
git checkout feature/new_coll
# Verify state:
git status && git log -3 --oneline
```

Then: Read `docs/dev_tasks/experimental_collision/progress.md` for full status. Next implementation task is adding CapsuleShape to `dart/collision/experimental/shapes/`.

## Key Constraints

1. **NEVER mention FCL, Bullet, ODE** by name in code or docs
2. **Use `Aabb` not `AABB`** (PascalCase for abbreviations)
3. **BSD copyright headers only** - no other comments unless necessary
4. **snake_case for file names** in experimental modules
5. **Determinism required** - bit-exact results, use `std::vector`, ordered iteration
