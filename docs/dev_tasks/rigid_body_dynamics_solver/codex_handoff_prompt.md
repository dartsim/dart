# Codex hand-off prompt

Historical prompt for the PR #2705 conflict-resolution session. Do not use this
as live task direction; use [`README.md`](README.md), [`RESUME.md`](RESUME.md),
and the durable solver architecture design for current status.

Paste the block below into Codex.

---

DART repo (`dartsim/dart`, C++20 + nanobind dartpy, pixi/ninja). Branch
`feature/experimental-rigid-body-dynamics` = PR #2705: a rigid-body dynamics MVP
(all joint types, two-sided contacts, floating base) on the experimental ECS
`World` (`dart::simulation`). Green on its own, but PR #2705 is
`CONFLICTING` — while it was in review `origin/main` landed #2698, which
refactored the rigid-body integration foundation underneath it.

**Two deliverables:**

1. Unblock PR #2705: merge current `origin/main` and reconcile the integration
   _conventions_ #2698 changed, so the suite is green and the PR is `MERGEABLE`,
   without regressing the MVP.
2. Add at least one MVP **GUI** example that steps the DART 7 World and
   renders it live with `dart-gui`.

Required reading (verified analysis + design intent — don't skip), under
`docs/dev_tasks/rigid_body_dynamics_solver/`:

- `convention_realignment.md` — exact divergence (file:line), merge surface, the
  two gating tests, how to reconcile (+ a fallback).
- `architecture_principles.md` — durable intent: SoA / cache-friendly / batch /
  backend-portable (multi-thread CPU, CUDA, hybrid) **pure kernels**, with
  **DART-6 dynamics parity**; reuse DART-6 code where it fits the new API, else
  rewrite (DART 8 ships only the DART 7 API).
- `RESUME.md` — full roadmap; the GUI example is "Subsystem C".

Re-alignment: #2698 made integration gravity-free +
force-persisting (no per-step `setZero`); this branch's `world.step()` applies
gravity and clears forces, and `RolloutWorldsBatchedMatchesReference` asserts
`world.step()` == `rolloutWorldsBatched`, so they must agree. **Reconcile in the
batch-friendly direction:** keep main's integrator/rollout pure & gravity-free,
restore DART-6 "bodies fall" via a batch-friendly gravity/force-assembly stage
that fills the SoA force buffer, used identically by step and rollout. Confirm
the force-persistence policy with the maintainer; record it in the docs.

GUI example: renderer is `dart-gui` (Filament+GLFW+ImGui, `DART_BUILD_GUI=ON`;
headers `dart/gui/{scene,viewer,renderable}.hpp`); dartpy gui bindings exist.
Template `examples/collision_sandbox/main.cpp` (follow its viewer wiring, not its
legacy physics). Add a thin experimental-`World`→`gui::Renderable` extraction
(none exists yet): each frame read body `comps::Transform` + shape, update a
`gui::Scene`, `world.step()`. Show something obvious (bodies falling onto a
floor, or a swinging chain). Details in RESUME.md Subsystem C.

Merge: `git fetch origin main` first (expect fresh conflicts — main moves fast).
Conflicts are confined to experimental `CMakeLists.txt`,
`compute/world_step_stage.{hpp,cpp}`; resolve by _union_ (keep main's
`RigidBodyIntegrationStage`/`BatchedRigidBodyIntegrationStage`/`integrateRigidBody`
**and** this branch's `RigidBodyVelocityStage`/`RigidBodyPositionStage`/
`RigidBodyContactStage`/`MultibodyForwardDynamicsStage`).
The API-boundary policy lives in `docs/onboarding/api-boundaries.md`; the
enforced gate is `pixi run check-api-boundaries` (on-demand signal report via
`pixi run report-api-boundary-inventory`).

Commands:

- Build: `pixi run ninja -C build/default/cpp/Release tests dartpy`
- C++: `cd build/default/cpp/Release && ctest` (incl. `test_world`); focus
  `bin/test_world --gtest_filter='World.*Batched*:World.Rollout*'`
- Python: `PYTHONPATH=build/default/cpp/Release/python pixi run pytest
python/tests/unit/simulation/test_experimental_world.py -q`
- Lint: `pixi run lint` (re-read generated files after). Check real exit codes.

Done when: main merged; `ninja` exit 0; `ctest` 0 failures incl. the two gating
tests; pytest passes; lint clean; PR #2705 `MERGEABLE`; ≥1 GUI example builds
under `DART_BUILD_GUI=ON` and renders the stepping World.

Constraints: **no Claude/AI attribution** in commits or PR. Track work in
`docs/`, not GitHub issues. dartpy Pythonic (snake_case, properties). Don't name
"Genesis" in code/docs; don't modify `/home/js/multiphysics-api-design.md`.
