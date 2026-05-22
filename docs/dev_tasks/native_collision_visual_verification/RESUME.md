# Resume: Native Collision Visual Verification

## Last Session Summary

PLAN-037 and this dev-task folder were created, then Phase 1 started in
`examples/collision_sandbox`. The sandbox now has a `PairCase` registry covering
every unordered native shape pair and has been ported onto the
Filament-backed `dart::gui` application path from
`feature/filament-gui-full-execution`. It has a pair selector, type-aware shape
parameter controls and pose controls for both objects, raw
contact/manifold/result values in the panel, contact point/normal/depth
rendering, AABB-tree/candidate-pair overlays, a `--pair` headless selector, a
CMake-driven headless screenshot sweep, and focused tests for the registry and
broad-phase debug snapshot API.

## Current Branch

`task/native-collision-visual-verification` - implementation branch. The
worktree should be clean after the checkpoint commit.

## Immediate Next Step

Add spatial-order overlays for broad-phase implementations that expose ordering
data, then decide whether collision filtering or broad-phase selection controls
belong in the first completion pass.

## Context That Would Be Lost

- The completed native-collision performance work remains the baseline; do not
  reopen FCL, Bullet, or ODE as runtime GUI dependencies.
- Existing `collision_sandbox` is now Filament-backed and supports headless
  screenshot output, pair selection, type-aware shape parameter controls,
  object posing, contact point/normal/depth rendering, and
  AABB-tree/candidate-pair overlays.
- `examples/collision_sandbox/run_headless_sweep.cmake` drives the
  deterministic smoke sweep for representative pair screenshots plus a wide UI
  screenshot that keeps the Native Collision panel visible.
- The pair registry lives in `examples/collision_sandbox/pair_registry.*` and
  is tested by `test_collision_sandbox_pair_registry`.
- `BroadPhaseSnapshot` records only candidate pairs and object count;
  `BroadPhaseDebugSnapshot` records copied debug data for AABB-tree topology.
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

Then continue the remaining PLAN-037 work in `examples/collision_sandbox` and
keep any new collision or broad-phase API tests focused under
`tests/unit/collision/`.

To rerun the current visual smoke directly:

```bash
cmake \
  -DDART_COLLISION_SANDBOX_EXECUTABLE=$PWD/build/default/cpp/Release/bin/collision_sandbox \
  -DDART_COLLISION_SANDBOX_OUTPUT_DIR=/tmp/collision_sandbox_sweep \
  -P examples/collision_sandbox/run_headless_sweep.cmake
```
