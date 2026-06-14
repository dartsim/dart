# PR #2705 — Convention Re-alignment to main's #2698

Historical handoff note for the PR #2705 merge/reconciliation. It records why
that branch conflicted with main at the time; it is not current solver status.
For current status use [`README.md`](README.md), [`RESUME.md`](RESUME.md), and
the durable solver architecture design.

Reference detail for the Codex hand-off. The short goal prompt lives in
[`codex_handoff_prompt.md`](codex_handoff_prompt.md); broader roadmap in
[`RESUME.md`](RESUME.md); the durable design intent that should steer this
reconciliation is in
[`architecture_principles.md`](architecture_principles.md) (SoA / cache-friendly
/ batch / backend-portable pure kernels, with DART-6 dynamics parity). This file
is the source of truth for _why the PR conflicts with main and exactly what
"reconciled" means_.

## Goal

Merge current `origin/main` into `feature/experimental-rigid-body-dynamics`
(PR #2705) and reconcile the rigid-body integration **conventions** that main's
#2698 changed underneath this branch, so the whole suite is green and the PR is
`MERGEABLE`. The MVP (all joint types, two-sided contacts, floating base) must
keep working; only the rigid-body integration layer needs realignment.

## State (verified 2026-05-24; main moves fast — re-fetch before pushing)

- Branch `feature/experimental-rigid-body-dynamics` locally merged
  `origin/main` `b7f5380679c`; PR #2705 still needs the merge commit pushed
  before GitHub can report `MERGEABLE`.
- `origin/main` `b7f5380679c`: includes #2698 (`41942cac6bb`, scalable-compute
  foundation), #2700 (CCD), #2707. Merge-base `0c0d63c3d5c`.

## The divergence (verified by reading both trees)

#2698 changed two rigid-body conventions this branch was built on:

1. **Gravity removed from integration.** main's
   `compute/world_step_stage.cpp` `integrateRigidBody(registry, entity,
timeStep)` does `velocity.linear += (force.force / mass.mass) * timeStep;`
   — no gravity term. main's default `world.step()` (`world.cpp` ~794-810) is
   just `RigidBodyIntegrationStage` + `KinematicsStage`, so **bodies do not
   fall by default on main**.
2. **Forces persist (no per-step clear).** main's `integrateRigidBody` never
   calls `force.setZero()`; forces are persistent inputs.

This branch does the opposite: `integrateRigidBodyVelocity(...)` applies
`(force/mass + gravity)*dt` then `force.force.setZero(); force.torque.setZero();`
and the default `world.step()` is the split pipeline
`RigidBodyVelocityStage → RigidBodyContactStage → MultibodyForwardDynamicsStage
→ RigidBodyPositionStage → KinematicsStage` (gravity on, forces cleared).

## Merge surface

- New-file-only on main (no conflict, just land): `rigid_body_state_batch.*`,
  `rigid_body_integration_kernel.hpp`, `world_batch.*`,
  `compute_stage_metadata.*`, CCD files.
- **Conflicts** (both sides edited): `dart/simulation/`
  `CMakeLists.txt`, `compute/world_step_stage.hpp`, `compute/world_step_stage.cpp`.
  Resolve by **union**: keep main's `RigidBodyIntegrationStage`,
  `BatchedRigidBodyIntegrationStage`, `integrateRigidBody` (untouched) **and**
  this branch's `RigidBodyVelocityStage`/`RigidBodyPositionStage`/
  `RigidBodyContactStage`/`MultibodyForwardDynamicsStage`.
- The API-boundary policy lives in `docs/onboarding/api-boundaries.md`; the
  enforced gate is `pixi run check-api-boundaries` (an on-demand signal report
  is available via `pixi run report-api-boundary-inventory`). Resolve
  `CHANGELOG.md` by keeping both.

## Two gating tests (main's `tests/.../world/test_world.cpp`)

- `BatchedRigidBodyIntegrationStageMatchesPerEntityForFreeBodies` (~L1500):
  `RigidBodyIntegrationStage` vs `BatchedRigidBodyIntegrationStage`, both
  gravity-free. **Keep both untouched** → stays green.
- `RolloutWorldsBatchedMatchesReference` (~L2037): 2 free bodies,
  `setForce(0.15,-0.05,0.1)` once, dt 0.01; asserts
  `rolloutWorldsBatched(worlds, initial, 5, executor)` == `reference.step(5)`.
  This passes when `rolloutWorldsBatched` and the default `world.step()` share
  the same force assembly and persistence policy. The branch's old gravity+clear
  step broke it because rollout re-applied an initial state but not a cleared
  force accumulator.

## Reconciliation decision

Adopt main's #2698 **persistent applied force/torque** convention and keep all
integrators gravity-free. A physics step assembles a transient SoA force buffer
from the persistent applied `comps::Force` plus `mass * world.gravity` for
dynamic rigid bodies, then integrates from that buffer without mutating
`comps::Force`. Callers clear or update applied forces explicitly.

This preserves #2698's pure integration kernels and rollout behavior while
restoring the DART-6 "bodies fall under default gravity" user-facing behavior.
The same default `World::step()` path is used by `rolloutWorldsBatched`, so the
rollout/reference gate compares the same force-assembly policy on both sides.

## Reconciliation options considered

### RECOMMENDED — pure kernel + batch-friendly gravity stage (DART-6 parity)

Per [`architecture_principles.md`](architecture_principles.md), reconcile in the
batch-friendly direction while **keeping** DART-6 "bodies fall" behavior:

- Keep main's integrator/rollout **pure and gravity-free** (don't bake gravity
  into `integrateRigidBody` / `integrateRigidBodyStateBatch*` /
  `RigidBodyIntegrationStage` / `BatchedRigidBodyIntegrationStage`) — required
  for backend portability, and keeps `BatchedMatchesPerEntity` green untouched.
- Restore gravity via a **batch-friendly force-assembly stage** that fills the
  SoA force buffer with `mass*gravity` (+ applied forces) before integration, on
  every backend. Make `world.step()` and `rolloutWorldsBatched` use the _same_
  force-assembly so `RolloutWorldsBatchedMatchesReference` holds.
- Force persistence policy: adopt #2698's persist convention. The force-
  assembly stage rebuilds only the transient buffer each step; it does not clear
  persistent applied loads.

### Minimal fallback — adopt #2698 verbatim (gravity opt-in)

If the gravity stage can't land in this PR, make `RigidBodyVelocityStage` drop
the `+ gravity` term and the `setZero()` clear (matches main exactly; both gating
tests pass without touching main's files), and defer the gravity force-assembly
stage to a follow-up tracked in RESUME.md. Note: this temporarily drops the
"bodies fall by default" semantic, so it's a fallback, not the target.

Decision recorded here and in RESUME.md.

## Commands

- Build: `pixi run ninja -C build/default/cpp/Release tests dartpy`
- C++ tests: `cd build/default/cpp/Release && ctest` (esp. `test_world`); single
  case `build/default/cpp/Release/bin/test_world --gtest_filter='World.Rollout*'`
- Python: `PYTHONPATH=build/default/cpp/Release/python pixi run pytest
python/tests/unit/simulation/test_experimental_world.py -q`
- Lint (auto-formats): `pixi run lint` — re-read generated files afterward.
- Verify the real exit code, not just the task-notification code (compound
  commands mask failures).

## Acceptance criteria

- `origin/main` fully merged; conflicts resolved by union as above.
- `ninja` exit 0; `ctest` 0 failures incl. both gating tests; pytest all pass;
  `pixi run lint` clean.
- PR #2705 `mergeable == MERGEABLE`.
- This file + RESUME.md record the chosen convention.

## Hard constraints

- **No Claude/AI attribution** anywhere in commits or the PR body (no
  `Co-Authored-By: Claude`, no "Generated with Claude Code").
- Track remaining work in `docs/` (RESUME.md), **not** GitHub issues.
- dartpy must stay Pythonic (snake_case, properties).
- Do not name "Genesis" in core code/docs. Do not modify
  `/home/js/multiphysics-api-design.md`.
- Re-fetch `origin/main` before merging; expect possible re-conflicts since main
  actively refactors this subsystem.
