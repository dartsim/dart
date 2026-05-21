# Resume: Native Collision Visual Verification

## Last Session Summary

PLAN-037 and this dev-task folder were created to track the requested visual
verification follow-up after native collision feature and performance
completion. No implementation has started yet; the current evidence points to
`examples/collision_sandbox` and `dart/gui/vsg/collision_scene_builder.*` as
the starting point.

## Current Branch

`docs/native-collision-visual-verification-plan` - docs-only planning
checkpoint branch. The worktree should be clean after the checkpoint commit.

## Immediate Next Step

Implement the Phase 1 pair registry for `examples/collision_sandbox`, then add
a focused test or checker proving the registry covers the expected native
narrow-phase matrix and explicit unsupported placeholders.

## Context That Would Be Lost

- The completed native-collision performance work remains the baseline; do not
  reopen FCL, Bullet, or ODE as runtime GUI dependencies.
- Existing `collision_sandbox` already supports VSG/ImGui, picking, headless
  screenshot output, simple contacts, AABBs, distance, raycast, and CCD, but it
  is not yet pair-complete and has no manifold/depth or BVH/Morton overlays.
- `BroadPhaseSnapshot` currently records only candidate pairs and object count;
  AABB tree/BVH visualization requires a copied debug snapshot API because tree
  nodes are private implementation details.
- The branch was created from current `origin/main`; a separate local
  `docs/retrospect-ai-review-bot-examples` branch still contains the unrelated
  bot-review wording checkpoint.

## How To Resume

```bash
git checkout docs/native-collision-visual-verification-plan
git status --short --branch
git log -3 --oneline
sed -n '1,220p' docs/plans/037-native-collision-visual-verification.md
sed -n '1,220p' docs/dev_tasks/native_collision_visual_verification/README.md
```

Then start Phase 1 in `examples/collision_sandbox`, keeping any new collision
or broad-phase API tests focused under `tests/unit/collision/`.
