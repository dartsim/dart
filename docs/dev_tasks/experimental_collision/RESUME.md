# Resume: Experimental Collision Module

## Last Session Summary

Removed batch rebuilds on transform-only changes by syncing the SoA cache during
`updateObject()`/`updateAll()` and leaving rebuilds only for structural changes
(create/destroy). Updated ECS data layout/progress docs to reflect the cache
sync path.

## Current Branch

`feature/new_coll` — dirty (many uncommitted changes from parallel agents).

## Immediate Next Step

Validate the batch path with the existing collision-experimental unit tests,
then start Phase 2 (pair/manifold cache + scratch buffers) once the workspace
is settled.

## Context That Would Be Lost

- `CollisionWorld` now updates `BatchStorage` entries on AABB recompute without
  forcing a full rebuild when only transforms move.
- Rebuild is still triggered for create/destroy via `m_batchDirty`.

## How to Resume

```bash
git checkout feature/new_coll
# Verify state:
git status && git log -3 --oneline
```

Then: keep `convex_convex.cpp` untouched (owned by another agent), and continue
with Phase 2 ECS batch improvements once the workspace is settled.
