# Resume: Variational Integrator Solver (PLAN-084)

> **Canonical roadmap is the [README North Star + Gaps](README.md#north-star).**
> This file is the fast-resume snapshot; the README is the source of truth for
> _what_ is done and _what_ remains. Graduation status lives in
> [`graduation-criteria.md`](graduation-criteria.md); the contact sequencing in
> [`../../plans/084-variational-integrator-solver/contact-roadmap.md`](../../plans/084-variational-integrator-solver/contact-roadmap.md).

Branch/PR details below are historical handoff evidence, not current checkout
instructions. Live status should be read from the README, the graduation
criteria, the plan dashboard, and the current code.

## Last Session Summary (2026-05-30)

Phase A/B were already done. This session executed **Phase C contact & friction**
end-to-end and the **graduation prep**, then proved out the two genuinely-remaining
items with investigation evidence:

- **C1 lagged Coulomb friction, C2 compliant ground contact, C3 augmented-
  Lagrangian** drift-free contact — all landed, **all reachable from
  `World::step()`** via `Multibody::setGroundContact(...)`. C3 is **opt-in** via
  the `dualUpdateCadence` argument (`0` = robust C2 default; `N>0` advances the
  per-point duals every `N` steps).
- **Link-vs-link** sphere-sphere self-contact slice
  (`makeVariationalLinkSphereContactHook`).
- **Serialization**: the contact config (`comps::VariationalContact`, now a
  serialized `Property`) and the AL dual state (`comps::VariationalContactDualState`,
  a serialized `State`) round-trip across binary save/load (binary format **v8**);
  added a generic POD-vector path to `io/auto_serialization.hpp` to carry
  `std::vector<std::size_t>` / `std::vector<double>` fields.
- **Demo + bindings**: Python `sx_variational_contact` scene (registered, in the
  demos-cycle); dartpy `set_ground_contact` / `add_ground_contact_point`.
- **Graduation**: every [criterion](graduation-criteria.md) met/declared; the
  API-freeze surface declared; a readiness assessment + a "graduation mechanics"
  section explaining why the flip is structural (see below).
- **Dedup**: shared spatial-algebra primitives hoisted into a `detail/` header
  (the old SI/VI duplication follow-up is closed).

## Current Branch / PR

- Branch: **`feature/vi-roadmap-followups`**, HEAD `53a6bcef4e2` (31 commits ahead
  of `origin/main`, 0 behind — kept current).
- PR: **#2779** — `OPEN`, `MERGEABLE`, `BLOCKED` (awaiting required CI + maintainer
  review; **do not request merge** — maintainer-owned, and per process the Codex
  bot must re-review after any silent fix; "no new findings" ≠ approval).

## Verified (all green)

- Full simulation ctest: **43/43** (`pixi run test-simulation`),
  re-run green after each merge of `origin/main`.
- `test_variational_integration` contact/serialization gtests (12) incl.:
  - `VariationalGroundContact.WorldSurfaceCompliantContactRestsOnGround` (C2 rests at `mg/k`)
  - `…WorldSurfaceAugmentedLagrangianCentersContact` (C3 drift-free on `world.step()`)
  - `…ConfigRoundTripsThroughBinarySaveLoad` (contact config persists)
  - `…AugmentedLagrangianDualStateRoundTrips` (AL scene resumes **bit-identically**)
  - `…LaggedFrictionDeceleratesSlidingBlock`, `…AugmentedLagrangianCentersContactAtZeroPenetration`
  - `VariationalLinkContact.SphereContactStopsSlidingLink`

## What Remains (evidence-backed — NOT in-session-executable)

All four are the task's **own explicit deferrals**, now backed by concrete facts
(not judgment calls). Ordered by tractability:

1. **Arbitrary-geometry link-vs-link contact — PLAN-scale (~2–3 weeks).** A
   2026-05-30 stack audit: `detail/deformable_contact/primitive_distance.hpp`
   already gives reusable analytic `(d, ∂d/∂q)` kernels (separable from the IPC
   log-barrier), and `detail/rigid_ipc/rigid_ipc_ccd.*` gives CCD for a line-search bound.
   The **dominant missing piece is rigid/articulated candidate generation** —
   `candidate_set.hpp` is mesh-vertex specific; there is no warm-started "refit
   link AABBs at the trial config, re-check the active set" query object. That
   broad-phase adapter + per-step query is the net-new work. Coordinated with
   PLAN-081 / `082-rigid-implicit-barrier-contact`. The sphere-sphere hook is the
   gate-1 minimum already landed.
2. **experimental → supported graduation — structural, maintainer-scoped.**
   `experimental` is encoded in the namespace (`dart::simulation`),
   directory, and `DART_EXPERIMENTAL_API` macro. There is **no per-family stability
   flag** — `MultibodyIntegrationMethod` (in `world.hpp`) is just
   `SemiImplicit`/`Variational`. The VI **shares the module with `World` and every
   other solver**, so it cannot be graduated in isolation: the flip is a
   whole-module promotion or a ~50–100-file VI-extraction refactor. All criteria
   are met → the VI is **ready to propose**; the flip itself is the maintainer's
   call. See "Graduation mechanics" in [`graduation-criteria.md`](graduation-criteria.md).
3. **C4 hard IPC barrier — out of scope by the task.** The contact-roadmap says
   verbatim "barrier (C4) last" / "stop at C3"; C4 is the _optional_ rung (stiff
   barrier curvature mis-scales the `Δt·M⁻¹` quasi-Newton). Do **not** implement
   it as part of this task.
4. **Manifold preconditioner for very long _floating_ chains — north-star
   stretch.** The exact recursive-Jacobian preconditioner is Euclidean-only;
   spherical/floating use the manifold Anderson (verified to ~20 links). A
   Lie-group extension is open numerical-methods work, explicitly a stretch item.

## Key Files (hot paths)

- `dart/simulation/compute/variational_integration.{hpp,cpp}` — the
  integrator, the contact hooks (`makeVariationalGroundContactHook`,
  `VariationalGroundContactSolver` with `setDuals`/`updateDuals`,
  `makeVariationalLinkSphereContactHook`), and
  `MultibodyVariationalIntegrationStage::execute` (the World-step wiring incl. the
  C3 seed→integrate→update→persist outer loop).
- `dart/simulation/comps/variational_contact.hpp` — contact config
  (`Property`, incl. `dualUpdateCadence`).
- `dart/simulation/comps/variational_contact_dual_state.hpp` — AL
  duals + cadence counter (`State`, serialized).
- `dart/simulation/io/{auto_serialization.hpp,serializer.cpp,binary_io.hpp}`
  — serialization (generic POD-vector path; `kBinaryFormatVersion = 8`).
- `dart/simulation/multibody/multibody.{hpp,cpp}` — `setGroundContact`
  / `addGroundContactPoint`.
- `python/dartpy/simulation/module.cpp` — dartpy bindings.
- `tests/unit/simulation/experimental/compute/test_variational_integration.cpp`
  — all VI/contact tests.
- `python/examples/demos/scenes/sx_variational_contact.py` + `registry.py`.

## Context That Would Be Lost (gotchas)

- **C3 cadence stability.** The undamped symplectic step makes per-step dual
  updates unstable; the dual ascent **must** be an outer loop slower than the
  primal (`dualUpdateCadence > 1`) and paired with `dampingCoefficient > 0`. The
  unit tests use cadence 20.
- **Lagged friction.** Friction direction + normal are taken at `q^k` (constant
  across the step's RIQN iterates), so contact converges like frictionless —
  avoids the tangential-regularization stiffness blow-up near `v_t = 0`. Don't
  "fix" it to use the trial config.
- **Binary format v8 is unreleased** (exists only on this branch/PR), so adding
  fields/components to it needs **no migration** — there are no old v8 files. Keep
  folding VI-contact serialization into v8 until the PR merges.
- **Component categories drive serialization**: `Property`/`State` are serialized
  (PFR field-walk), `Cache` is not. The contact config is `Property`; the duals
  are `State`. `pointLinkIndices` are positions in `structure.links` (NOT
  `entt::entity`), so no entity-remap pass entry is needed.
- **Contact-as-forcing only.** Contact enters strictly as a generalized force in
  the forced-DEL residual (`residual -= dt·Q_c`); never as a velocity projection
  (that breaks symplecticity). Equal-and-opposite for link-vs-link; root link
  (index 0) has a zero Jacobian.
- **Python tests/demos**: the built `_dartpy.so` lives in
  `build/default/cpp/Release/python/dartpy/`, **not** the source tree — run pytest
  from `python/tests` with `PYTHONPATH=$PWD/../../build/default/cpp/Release/python`.
  Vector3d args need `np.array([...])` (not tuples); `Link.transform` is a numpy
  4×4.
- **Lint gotcha**: a leftover `.claude/worktrees/agent-*` worktree breaks
  `pixi run lint` (the collision-isolation check rglobs duplicate sources) — remove
  it with `git worktree remove --force` + `git worktree prune`.
- **No AI attribution** in commits/PRs.

## How to Resume

```bash
git checkout feature/vi-roadmap-followups
git fetch origin main && git merge origin/main --no-edit   # keep current; clean so far
git log --oneline -8

# Build + run the full simulation suite (the gate):
pixi run build-simulation-tests
pixi run test-simulation                      # expect 43/43

# Run just the VI / contact tests:
build/default/cpp/Release/bin/test_variational_integration \
  --gtest_filter='VariationalGroundContact.*:VariationalLinkContact.*:VariationalIntegration.*'

# Format before committing:
pixi run lint-simulation   # clang-format C++
pixi run lint-md                        # prettier docs

# (Optional) dartpy in-place build for the Python demo / smoke:
pixi run build-py-dev ON Release
```

**If picking up item 1 (arbitrary geometry):** start from the contact-roadmap
"Still future" section — wrap `primitive_distance.hpp`'s squared-distance kernels
to emit signed `d` + `∂d/∂q` (`∂d = ∂(d²)/(2d)`), chain through the link point
Jacobians already in `VariationalContactContext`, and build the rigid candidate-
generation adapter (the real gap). Reuse the existing compliant/AL force laws.
