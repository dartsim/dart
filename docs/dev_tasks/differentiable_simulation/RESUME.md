# Resume: Differentiable Simulation (PLAN-110)

## Current Reality (2026-06-06)

The WS1-W5 first-slice surface described in this resume landed on `main` via PR
#2761. Treat the older branch-local "Current Branch" and "Nothing committed or
pushed" notes below as historical context, not current resume instructions.
Current PLAN-110 work resumes from this folder's `README.md`,
`docs/design/differentiable_simulation.md`, and `docs/plans/dashboard.md`:
harden the landed differentiable surface, add examples and promotion evidence,
and run the Dojo-style follow-up spike without exposing solver, cache, ECS,
backend, or tensor-framework types through the public facade. Older sections in
this resume that describe WS2 as blocked, frictionless-only, or rotational
contact as out of scope are historical snapshots; the current boxed-LCP
derivative slice covers rigid-body normal/friction contact, rotational/off-COM
contacts, and multi-contact cases, while articulated-link contact gradients
remain unsupported.
World-level differentiability/contact policy flags now round-trip through binary
save/load, and replay restores differentiable parameter registrations. Binary
serialization of parameter registrations remains unimplemented and must be
settled before stable promotion.

## Last Session Summary

Planned and designed opt-in analytic differentiable simulation for the
DART 7 `World` (the Nimble method, arXiv:2103.16021). Produced the durable
design doc, the PLAN-110 plan card + dashboard entry, a feature-by-feature Nimble
gap audit + cross-engine API survey (Brax/Newton/Genesis/MJX), a `werling-2021`
papers-catalog entry, and this dev-task scaffold. Ran three specialized reviews
(architecture/oracle, plan/momus, API/north-star) and **incorporated all
CRITICAL/HIGH findings** into the docs. Then implemented and verified the WS1
first slice (see "WS1 First Slice — Implemented And Verified" below).

Key review corrections now baked into the docs:

- Position Jacobian is **joint-type-keyed** (SO(3)/SE(3) `dexp`/`dlog`), not
  identity — DART integrates on the manifold (`multibody_dynamics.cpp:1037-1067`).
- At planning time the analytic **contact** gradient was blocked until PLAN-080
  WS4 made the experimental contact path a boxed-LCP solve. That prerequisite is
  now implemented: Seam-2 is the experimental boxed-LCP path, **not** legacy
  `dart/constraint/constraint_solver.cpp`.
- `∂M/∂q`, `∂c/∂q` are substantive smooth-term derivatives (the larger part of
  the work), produced by a WS1-chosen mechanism — not "cheap bookkeeping."
- Gates are concrete now (FD `rel<1e-4`, `h∈{1e-5,1e-6,1e-7}`, named scenes;
  `test_diff_zero_cost_parity`; `DART_BUILD_DIFF` CI on/off; serialization).
- API renamed for house style (`get_step_derivatives`, `add_differentiable_parameter`,
  `world.state_vector`, `applyStepVjp`, `traj.gradients`, one `sx` import,
  `sx.state.rollout(..., differentiable=True)`, `PRE_CONTACT_SURROGATE`).

## WS1 First Slice — Implemented And Verified

WS1.0 mechanism chosen: **FD-of-terms + analytic assembly** (central-difference
`computeMultibodyDynamicsTerms` for `∂M/∂q`, `∂c/∂q`, `∂c/∂q̇`; analytic
semi-implicit-Euler assembly of the step Jacobian). This is FD-checkable without
circularity because the checker is FD-of-**step**, distinct from the
FD-of-**terms** used to build the derivative inputs; the assembly formula is what
gets validated, and a closed-form gravity-torque oracle gives a third check.

Implemented (revolute/prismatic, contact-free):

- Always-compiled core: `world_options.hpp` (`WorldOptions{timeStep, gravity,
solver-family/contact/differentiability policies, memory options}`),
  `World(const WorldOptions&)`, `isDifferentiable()`,
  `getStepDerivatives()`, serialized flag (binary format v2→v3),
  `diff/step_derivatives.hpp` (`StepDerivatives`, header-only public).
- Behind `DART_BUILD_DIFF` (`DART_HAS_DIFF`): `detail/smooth_jacobians.{hpp,cpp}`
  (`detail::contactFreeStepDerivatives`, **exported with `DART_SIMULATION_API`** —
  required so tests link it across the `.so`, per the deformable-contact detail
  precedent).
- CMake: `DART_BUILD_DIFF` option (default OFF, CUDA-pattern guard); wired into
  the pixi `config`/`config-debug` tasks via `DART_BUILD_DIFF_OVERRIDE`.
- Tests: `test_diff_zero_cost_parity` (both configs), `test_diff_smooth_jacobian`
  (ON only).

Verified: OFF 22/22, ON 23/23; FD-vs-analytic ~1e-12; `check-api-boundaries` +
`lint` green; cache rests at OFF.

**Gotcha for the next session:** `pixi run build` re-runs `config` and resets
`DART_BUILD_DIFF` to OFF. To work the ON config, set
`DART_BUILD_DIFF_OVERRIDE=ON` for BOTH `pixi run config` AND any subsequent
build, or build the target directly with `cmake --build` against an ON cache.

## Current Branch

`main` — uncommitted local changes: the docs (design/plan/dashboard/papers/
dev-task) AND the WS1 code (`world_options.hpp`, `world.{hpp,cpp}`,
`diff/step_derivatives.hpp`, `detail/smooth_jacobians.{hpp,cpp}`, `io/binary_io.hpp`,
two CMakeLists, `pixi.toml`, two new tests, `world/test_serialization.cpp`).
Nothing committed or pushed.

## PLAN-080 WS4 First Slice — Implemented And Verified (unblocks WS2)

Added an **opt-in boxed-LCP rigid-body contact path** to the DART 7 World
(the WS2 prerequisite):

- `WorldOptions::contactSolverMethod` = `SequentialImpulse` (default, unchanged)
  | `BoxedLcp`; `World::getContactSolverMethod()`.
- `detail/rigid_contact/boxed_lcp_contact.{hpp,cpp}` — `solveBoxedLcpContacts(...)`
  (`DART_SIMULATION_API`, exported, `nm`-verified) assembles `A = J M⁻¹ Jᵀ`,
  `b = −J v_free + bias`, one normal row plus two Coulomb tangent rows per
  contact (`findex`-coupled box bounds), solves with `dart::math::DantzigSolver`,
  applies `Δv = M⁻¹ Jᵀ f`, and returns
  **`BoxedLcpContactSnapshot { A, b, lo, hi, findex, f, J }`** — the seam WS2
  differentiates.
- Routed inside `RigidBodyContactStage::execute` (default path untouched).
- `test_boxed_lcp_contact`: 4/4 — resting-height parity vs SequentialImpulse
  (1e-6), single-step normal velocity (1e-9), equal-mass head-on momentum (1e-12).

Verified by me: build green; full experimental suite 23/23 (default config);
`check-api-boundaries` + `lint` green.

## WS2 First Slice — Implemented And Verified (the Nimble analytic contact gradient)

`detail/contact_jacobians.{hpp,cpp}` — `contactStepDerivatives(...)`
(`DART_SIMULATION_API`, exported, `nm`-verified). Classifies clamping rows
(`f_i>1e-9`) of the `BoxedLcpContactSnapshot`, forms `A_CC`, and computes the
analytic LCP gradient `∂f_C/∂z = A_CC⁻¹(∂b_C/∂z − (∂A_CC/∂z)·f_C)` via rank-
revealing `completeOrthogonalDecomposition`; geometric inputs (`∂A/∂z`, `∂b/∂z`,
free-velocity/position partials) via FD-of-terms with `f` frozen, then composes
`∂x'/∂z = (smooth FD, f frozen) + [Δt M⁻¹Jᵀ; M⁻¹Jᵀ]·∂f/∂z`.

**Sign-convention correction (important):** the forward solver uses `w = A f − b`
with `b = −(J v_free)+bias` (`boxed_lcp_contact.cpp`), so at a clamping row
`A_CC f_C = b_C` and `∂f_C/∂z = A_CC⁻¹(∂b_C/∂z − (∂A_CC/∂z) f_C)` — NOT the
plan's literal `A_CC f_C + b_C = 0`. Match the forward code's convention.

Historical initial scope: frictionless (`findex=-1`), translational DOFs, and a
single active clamping contact. The current slice has since grown to Coulomb
friction, rotational/off-COM contacts, and multi-contact cases while still
assembling translational `[pos; linvel]` output. Test `test_diff_contact_jacobian`
(ON only) started from the clamping/separating FD checks and now includes the
later friction/rotational coverage described below.

Verified by me (ON config): full experimental suite **25/25**; symbol exported;
`check-api-boundaries` + `lint` green; cache reset to OFF.

## Public-Facade Consolidation — Implemented And Verified

The public `World::getStepDerivatives()` now returns the **contact-aware**
analytic Jacobian (not just the contact-free one): when `differentiable &&
contactSolverMethod==BoxedLcp`, `captureStepDerivatives()` captures the pre-step
active contacts (via `collide()`) and routes through
`detail::contactStepDerivatives`. No active contacts → reduces exactly to the
contact-free Jacobian. Out-of-scope active contacts touching a multibody link
**throw `NotImplementedException`** with a specific message (NOT a silent
contact-omitting fallback). All contact-routing is inside `#ifdef DART_HAS_DIFF`,
so the default build is unaffected. Test `test_diff_public_contact_jacobian`
(ON only) covers in-scope contact-aware vs FD-of-step, no-contact closed-form +
FD, and later rotational/off-COM support.

Verified by me (ON config): full experimental suite **26/26**; default build
unaffected; `check-api-boundaries` + `lint` green; cache reset to OFF.

## Immediate Next Step

The full vertical slice (opt-in API → boxed-LCP contact → analytic contact
gradient → public `getStepDerivatives()`) **plus Coulomb friction** is proven,
FD-validated, and exposed through the public facade.

**WS2 friction — DONE & verified:** `solveBoxedLcpContacts` has tangential rows

- `findex` cone coupling (friction-row CFM; parity vs SequentialImpulse: sliding
  within 0.15 m/s, static hold |Δx|<5e-3); `contactStepDerivatives` differentiates
  it via the Nimble upper-bound mapping (`Ahat = A_CC + A_CU E`, sliding rows
  `f_U = E f_C`) over a full 6-DOF Delassus (tangential impulse induces spin, so
  angular DOFs are needed even with translational output). Test
  `SlidingFrictionMatchesFiniteDifference` asserts FD-of-step match AND the
  non-trivial sliding-friction coupling `|∂vx'/∂vz| > 1e-3`. Suite 26/26 (ON),
  boundaries/lint green, cache OFF.
  **Follow-up (robustness):** a benign `LCP internal error, s<=0` `DART_WARN_ONCE`
  can fire in the static-friction Dantzig degenerate-pivot path; it recovers
  correctly (assertions pass). Worth hardening the pivot/CFM later.

**WS3 (dartpy bridge) first slice — DONE & verified.** C++
`World::applyStepVjp(g) -> StepGradient{state,control}` (`diff/step_gradient.hpp`,
explicit `Jᵀ·g`; test `test_diff_apply_step_vjp` 3/3). dartpy:
`World(time_step, *, gravity, rigid_body_solver, multibody_options,
differentiable, contact_solver_method)` kwargs (no separate `WorldOptions`
Python class — matches the design's common-path example); props
`is_differentiable`/`num_dofs`/`num_efforts`/`state_vector`/`control_vector`/
`contact_solver_method`; methods `get_step_derivatives()`/
`apply_step_vjp()`; bound `ContactSolverMethod`/`StepDerivatives`/`StepGradient`;
stubs regenerated. `sx.diff.timestep(world, state, action)` is a
`torch.autograd.Function` with LAZY torch import (base import works without
torch; `sx.diff` exists; calling without torch → `ImportError`). Verified:
C++ VJP 3/3; `test-py` 399 passed/2 skipped; diff Python tests 6 passed/1 skipped;
`check-api-boundaries`+`lint`+stub-sync green; surface hand-checked.
**Env caveat:** torch is NOT in the pixi env, so the torch autograd gradient test
is skipped (import-safety + ImportError verified; numerics verified via non-torch
C++/Python VJP+Jacobian tests). `pip install torch` to run it.

**Rotational / multi-contact — DONE & verified.** No algorithmic change was
needed: the existing full-6-DOF Delassus + multi-row `A_CC` already handle
off-COM (rotational) and multi-point rigid-body contacts; only the
`captureStepDerivatives()` guards + a stale doc comment were updated. The output
stays translational (exact for a single step: position advances only by linear
velocity; angular coupling is captured in the gradient via the screw-axis rows of
`J`). New tests at FD ~1.2e-11: `MultiContactBox` (4 corners) and
`RotationalContact` (tilted ground, off-COM, `∂vx'/∂vz ≈ -0.148`); public box
test matches FD; multibody-link contact still throws. Suite **27/27** (ON),
boundaries/lint green, cache OFF.
**Known limitation (out of scope):** a world-pinned contact point whose lever arm
changes with body position (`∂arm/∂q ≠ 0`, e.g. a box on a narrow off-center
rail) shows ~9% error — the frozen-active-set analytic doesn't model contact-point
migration. Natural resting contacts (sphere/box on ground/slope, where the
contact point tracks the body) are exact; document this when promoting.

**WS4 params — DONE & verified.** Public `diff/physical_parameter.hpp`
(`PhysicalParameter{MASS,CENTER_OF_MASS,INERTIA,FRICTION}` +
`PhysicalParameterSelector`); `StepDerivatives.parameterJacobian` (always
present, empty when none registered, so the OFF build compiles);
`World::addDifferentiableParameter(body, param)` +
`getNumDifferentiableParameters()`; `detail::contactStepDerivativesWithParameters`
(FD-of-step over registered mass/inertia/friction quantities, restored exactly).
dartpy: `PhysicalParameter` enum, `world.add_differentiable_parameter(...)`,
`parameter_jacobian` on the step-derivatives object; stubs synced. Current first
slice supports MASS, INERTIA diagonal entries, and FRICTION. CENTER_OF_MASS is
excluded because the rigid-body step fixes the COM at the body origin, so its
gradient is identically zero.

Continue from the current status in `README.md`, not from this historical
mid-implementation list. Remaining follow-ups are promotion hardening: runnable
trajectory-optimization / system-identification examples, the torch autograd
end-to-end test in an environment with torch installed, binary serialization or
explicit restart documentation for differentiable parameter registrations, the
Dojo-style follow-up spike, and robustness for the static-friction Dantzig
degenerate-pivot `DART_WARN_ONCE` (`s<=0`).

## Context That Would Be Lost

- **The hard prerequisite is satisfied**: the experimental boxed-LCP contact
  path now emits the `{A,b,lo,hi,findex,f,J}` snapshot that the analytic contact
  gradient differentiates. Keep the gradient tied to that pivoting LCP seam; do
  not use the legacy `dart/constraint/constraint_solver.cpp` path.
- **The five attachment seams** (confirmed in code): (1) `dart/math/lcp/lcp_solver.hpp`
  `LcpSolver::solve(LcpProblem{A,b,lo,hi,findex}, x)` — capture snapshot + classify
  - `A_CC` solve; (2) constraint/contact assembly → `∂A/∂q, ∂b/∂q`; (3)
    `compute/multibody_dynamics.*` `computeMultibodyDynamicsTerms` (M,C,g);
    (4) `computeMultibodyLinkJacobian` → `∂(Jᵀf)/∂q`; (5) integration/impulse
    stages assemble the public Jacobian blocks.
- **The deliberate divergences from upstream Nimble**: opt-in (not always-on);
  framework-neutral core (torch only in the optional `sx.diff` dartpy submodule,
  imported lazily); DART-owned names; explicit Jacobians made first-class.
- **The correctness stance**: finite-difference checker is the gate; mode-switch
  subgradients and elastic approximation are documented; complementarity-aware
  and contacts-from-distance are opt-in non-true-gradient aids.
- The diff module lives in `dart/simulation/diff/` behind a build
  option (mirrors `compute/` and CUDA isolation); reverse-pass cache / LCP
  snapshot / clamping classification all live in `detail/` (never public).

## How to Resume

```bash
git status && git log -3 --oneline
# Read the design doc, plan, and gap audit:
#   docs/design/differentiable_simulation.md
#   docs/plans/110-differentiable-simulation.md
#   docs/plans/110-differentiable-simulation/nimble-gap-audit.md
```

Then: incorporate any pending review feedback, then implement WS1 starting with
the `differentiable` option in `WorldOptions` and the `StepSnapshot` skeleton in
`dart/simulation/diff/`, plus the finite-difference checker test
utility. Keep `pixi run lint`, `pixi run build`, focused experimental tests, and
`check-api-boundaries` green per slice.
