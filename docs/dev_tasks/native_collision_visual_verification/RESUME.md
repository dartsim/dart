# Resume: Native Collision Visual Verification

## Last Session Summary

PLAN-037 and this dev-task folder were created, then Phase 1 started in
`examples/collision_sandbox`. The sandbox now has a `PairCase` registry covering
every unordered native shape pair, a pair dropdown, uniform scale and pose
controls for both objects, raw first-contact/result values in the panel, a
`--pair` headless selector, and a focused registry coverage test.

## Current Branch

`task/native-collision-visual-verification` - implementation branch. The
worktree should be clean after the checkpoint commit.

## Immediate Next Step

Add richer shape-parameter inspectors and manifold-aware/depth rendering in the
VSG collision scene builder, then update the headless smoke to cover
representative pair and overlay presets.

## Context That Would Be Lost

- The completed native-collision performance work remains the baseline; do not
  reopen FCL, Bullet, or ODE as runtime GUI dependencies.
- Existing `collision_sandbox` already supports VSG/ImGui, picking, headless
  screenshot output, pair selection, simple contacts, AABBs, distance, raycast,
  and CCD, but it does not yet have manifold/depth rendering or BVH/Morton
  overlays.
- The pair registry lives in `examples/collision_sandbox/pair_registry.*` and
  is tested by `test_collision_sandbox_pair_registry`.
- `BroadPhaseSnapshot` currently records only candidate pairs and object count;
  AABB tree/BVH visualization requires a copied debug snapshot API because tree
  nodes are private implementation details.
- The implementation branch was renamed from
  `docs/native-collision-visual-verification-plan`; a separate local
  `docs/retrospect-ai-review-bot-examples` branch still contains the unrelated
  bot-review wording checkpoint.

## How To Resume

```bash
git checkout task/native-collision-visual-verification
git status --short --branch
git log -3 --oneline
sed -n '1,220p' docs/plans/037-native-collision-visual-verification.md
sed -n '1,220p' docs/dev_tasks/native_collision_visual_verification/README.md
```

Then continue Phase 2 in `examples/collision_sandbox` and
`dart/gui/vsg/collision_scene_builder.*`, keeping any new collision or
broad-phase API tests focused under `tests/unit/collision/`.
