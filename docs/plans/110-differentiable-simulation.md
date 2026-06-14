# PLAN-110: Differentiable Simulation (Opt-In Gradient Capabilities)

- Operating state: `PLAN-110` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` can, **opt-in**, produce analytic derivatives
  of a physics step — state, control, and physical parameters — via DART-owned
  differentiable solver capabilities. The first committed path is implicit
  differentiation of the contact LCP and articulated-body dynamics, matching the
  Nimble method (arXiv:2103.16021). Derivatives are **exact within a contact
  mode** (subgradients at mode/limit switches; elastic contact approximate) and
  verified by finite differencing. The additional Dojo track evaluates a separate
  maximal-coordinate variational hard-contact NCP solver with a primal-dual
  interior-point method and implicit gradients (arXiv:2203.00806); it is a
  planned evaluation track, not a replacement for the active Nimble-style
  implementation. Differentiability is off by default with bitwise-identical
  results when off; the public surface exposes DART-owned derivative value types
  and optional framework bridges, never solver internals, reverse-pass caches, or
  tensor backends.
- Current evidence:
  - The solver architecture reserves the seam: a _coupler_ reverse pass for
    cross-domain interaction and (this plan) a _rigid-body-solver-internal_
    reverse pass for contact/dynamics gradients; Model/State/Control/Contacts
    separation is "the precondition for … differentiability"
    (`docs/design/simulation_solver_architecture.md`).
  - The capability matrix's `Differentiability` axis is defined in
    `docs/design/simulation_cpp_api.md` (do not restate it here;
    that doc is the single source of truth for the axis values).
  - `compute_backend_research.md` recorded differentiability as _deferred_ with an
    explicit promotion question; this plan answers it (analytic method committed
    for contact; autodiff-scalar templating reserved for smooth terms only).
    Public scalar precision selection remains out of scope for this plan and is
    governed by the simulation API design docs plus PLAN-041.
  - Forward-pass ingredients DART already owns: boxed-LCP library `dart/math/lcp/`
    (`LcpResult LcpSolver::solve(const LcpProblem&, VectorXd& x, const LcpOptions&)`),
    articulated-body dynamics `compute/multibody_dynamics.*` (`M, C, g`, link
    Jacobians), and the `tan-lcp` formulation.
  - As built: an opt-in boxed-LCP rigid-body contact path landed on the
    DART 7 `World` (`detail/boxed_lcp_contact.{hpp,cpp}` assembles the
    Delassus system `A = J M⁻¹ Jᵀ` and solves it via the `dart/math/lcp` Dantzig
    pivoting solver, behind `WorldOptions::contactSolverMethod == BoxedLcp`; the
    default `SequentialImpulse` path is unchanged). The analytic contact gradient
    is implemented on that seam (`detail/contact_jacobians.{hpp,cpp}`).
  - DART's manifold position integration (SO(3)/SE(3) exp/log) makes the position
    Jacobian joint-type-keyed, not identity; implemented in
    `detail/smooth_jacobians.cpp`.
  - Dojo adds a distinct differentiable-rigid-body reference: maximal-coordinate
    state, variational integration, hard contact/friction as a nonlinear
    complementarity problem with second-order cone constraints, a custom
    primal-dual interior-point solver, and gradients from implicit
    differentiation of a relaxed contact solve. The upstream Dojo.jl README says
    the project is no longer actively developed, so DART treats Dojo as method
    evidence and a comparison baseline, not a dependency.

## Owner Docs

- Architecture, API, decisions, algorithm mapping (durable source of truth):
  [`../design/differentiable_simulation.md`](../design/differentiable_simulation.md)
- Feature-by-feature Nimble parity audit + cross-engine API survey:
  [`110-differentiable-simulation/nimble-gap-audit.md`](110-differentiable-simulation/nimble-gap-audit.md)
- Dojo solver gap audit + integration notes:
  [`110-differentiable-simulation/dojo-gap-audit.md`](110-differentiable-simulation/dojo-gap-audit.md)
- Public facade rules:
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md),
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md)
- Research references: `werling-2021` and `howell-2022-dojo` in
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- Active implementation tracking (transient; durable owners are the design + gap
  audit): `docs/dev_tasks/differentiable_simulation/`.

## Dependencies

- **Boxed-LCP contact prerequisite — landed in this work** (the PLAN-080
  Workstream 4 seam, scoped opt-in here). The DART 7 `World` gained a
  `BoxedLcp` contact method whose solve emits the `{A,b,lo,hi,findex,f}` snapshot
  the analytic contact gradient differentiates, via a pivoting (Dantzig) solver
  with a clean active set — exactly the differentiation-ready seam this plan
  required. The default `SequentialImpulse` path is unchanged.
- Coordinates with PLAN-030 (deferred GPU/batched differentiable track) and
  PLAN-081 (deferred differentiable-deformable track); neither blocks the
  rigid-body analytic CPU method.
- The Dojo track coordinates with PLAN-084 (variational integration) and the
  rigid-body solver roadmap because it would require a separate maximal-coordinate
  variational/NCP/IPM path rather than an extension of the existing boxed-LCP
  reverse pass.

## Workstreams

First slices of workstreams 1-5 are implemented and verified (see the dashboard
entry and `docs/dev_tasks/differentiable_simulation/`); each keeps
`check-api-boundaries` green and is finite-difference-of-step validated. The
Dojo workstream is unstarted and must begin with the gap audit/spike below, not
with public API or runtime dependencies. Slice detail for implemented work lives
in the dev-task roadmap.

1. **Opt-in seam + contact-free Jacobians** (implemented) — `WorldOptions::differentiable`
   (default false) + `DART_BUILD_DIFF`, nullable-sink snapshot plumbing, the
   smooth-term derivative mechanism (FD-of-terms over `computeMultibodyDynamicsTerms`
   - analytic semi-implicit-Euler assembly), **joint-type-keyed position
     Jacobians** (Euclidean `I`/`Δt·I`; SO(3)/SE(3) `dexp`/`dlog` + free-joint
     coupling — all joint types), the FD checker, and the zero-cost parity test.
2. **Analytic contact gradient** (implemented) — clamping/separating classification,
   `A_CC` rank-revealing solve with CFM, `∂f_C/∂{A,b}`, friction `findex` cone
   (upper-bound mapping), `∂(Jᵀf)/∂q`, rotational/off-COM and multi-contact; over
   the `BoxedLcp` pivoting solve.
3. **Reverse product + PyTorch bridge** (implemented) — framework-neutral
   `applyStepVjp` and `diff::rollout`, the optional `sim.diff` submodule
   (`timestep`/`rollout` `autograd.Function`, lazy torch), `state_vector`/
   `num_efforts` helpers, dartpy stubs, a torch-absent import test, Python tests,
   and a throw-to-target trajectory-optimization test.
4. **Parameter derivatives** (implemented) — `PhysicalParameterSelector` +
   `parameter_jacobian` for MASS, INERTIA, and FRICTION (COM excluded: no effect
   on the rigid-body step → identically-zero gradient), with a system-identification
   (mass-recovery) test.
5. **Refinement opt-in modes** (implemented; split, separately gated) — `ContactGradientMode`:
   - 5a elastic/restitution (FD-checked in a stable clamping regime; approximate
     at the make/break instant);
   - 5b complementarity-aware saddle escape (heuristic; **no FD gate by
     construction** — asserted to produce a non-zero direction where ANALYTIC stalls);
   - 5c pre-contact surrogate (backward-only; **no FD gate** — asserted non-zero
     toward-contact where ANALYTIC is zero).
6. **Dojo-style differentiable solver evaluation** (planned) — audit and
   de-risk a separate maximal-coordinate variational hard-contact NCP/IPM solver:
   model/state representation, nonlinear friction cone, primal-dual
   interior-point residuals and central-path smoothness knob, implicit-gradient
   KKT solve, examples/baselines, and coexistence with the current
   boxed-LCP/Nimble-style path. This stays inside the DART 7 `World`
   multi-solver architecture as an alternate rigid-domain method family with
   DART-owned capability names, not as a `DojoWorld`, public solver registry, or
   Dojo.jl dependency. The first slice is evidence-only: keep
   [`dojo-gap-audit.md`](110-differentiable-simulation/dojo-gap-audit.md)
   current, then run a minimal solver spike before any public API promise.

Remaining follow-ups (not workstreams): worked trajectory-optimization /
system-identification example programs, the torch-autograd end-to-end test (needs
torch in-env), a `DART_BUILD_DIFF` CI job, and minor robustness/coverage items —
tracked in `docs/dev_tasks/differentiable_simulation/`.

This plan must not add a public `World` precision selector as a differentiability
shortcut. Autodiff-scalar templating may remain an internal smooth-term or spike
mechanism, but public `sim.World(dtype=...)`, `sim.World[...]`, or scalar-specific
`World` aliases belong to a separate scalar-instantiation plan with its own API,
binding, serialization, collision, determinism, and package gates. That separate
plan should wait until the DART 7 rigid-body and multibody stack is already in
good shape for humanoid locomotion and manipulation; PLAN-110 may keep internal
scalar-generic seams open, but should not turn them into a user-facing precision
family.

## Acceptance Criteria

- Each workstream lands as small verifiable slices with focused tests; gap-audit
  rows flip to PRESENT with FD evidence.
- **Zero-cost-when-off**: a named test (`test_diff_zero_cost_parity`) asserts
  bitwise-identical final `[q; q̇]` and zero reverse-snapshot allocations with
  `differentiable=false`; a named benchmark reports the on/off overhead against a
  stated budget. (The design's overhead is _measured and reported_, not asserted
  as "zero.")
- **Correctness gate**: analytic `state_jacobian`/`control_jacobian`/
  `parameter_jacobian` agree with central finite differences at relative error
  `< 1e-4` over a perturbation sweep `h ∈ {1e-5, 1e-6, 1e-7}` on a named scene set
  (single/double pendulum, free body for WS1; sphere-on-ground, box-stack for
  WS2), with Tied-set and non-smooth-boundary configurations excluded and logged.
- **Determinism**: warm-start state and pivot/iteration policy are fixed on the
  differentiable path so the clamping classification is deterministic; gradient
  outputs are reproducible to a documented tolerance.
- **Build option / CI**: a named option (`DART_BUILD_DIFF`); CI builds with it on
  _and_ off; the zero-cost parity test runs in the off configuration; a workflow
  guard prevents the option being silently dropped (mirroring the CUDA workflow
  guard).
- **Serialization**: `WorldOptions::differentiable`, `contact_gradient_mode`, and
  registered differentiable parameters round-trip through serialization, or are
  explicitly documented as non-serialized.
- **Examples / docs / changelog**: WS3 ships a runnable trajectory-optimization
  example and WS4 a system-identification example; changelog and migration notes
  accompany any promoted surface.
- The public API never exposes the reverse-pass cache, the LCP snapshot,
  `Solver`/`Coupler`/`PhysicsDomain`/backend types, ECS storage, component types,
  or a tensor framework in the C++ core; `pixi run check-api-boundaries` stays
  green. Owner commands per slice: `pixi run lint`, `pixi run build` (diff on/off),
  the focused C++ FD/parity tests, `check-api-boundaries`, `pixi run test-py`.
- The differentiability surface does not imply a public scalar-specialized
  `World` family. Any future scalar precision API is planned and gated outside
  PLAN-110 after the rigid-body and multibody locomotion/manipulation baseline
  is ready, before it reaches user-facing C++ or Python names.
- Gradient-correctness limits (mode-switch/limit subgradients, elastic
  approximation) are documented; saddle-escape and pre-contact surrogates are
  opt-in only.
- Architecture invariants held: it is the rigid-body-solver-internal reverse pass,
  the forward single-domain fast path is unchanged, and the public object model is
  not reshaped.
- Dojo-style work remains an evaluation track until the gap audit is complete and
  a minimal internal spike proves the maximal-coordinate variational/NCP/IPM
  step, implicit-gradient solve, finite-difference agreement, and comparison
  scenes can coexist with the current DART 7 `World` without leaking solver
  internals or making Dojo.jl a runtime dependency.

## Revision Triggers

- A workstream completes, splits, or is reprioritized.
- The PLAN-080 boxed-LCP contact contract changes shape, or the boxed-LCP
  problem representation / `findex` convention changes (changes seams 1–2 and the
  friction-cone gradient).
- The boxed-LCP solver interface (`dart/math/lcp/`) changes.
- Dojo gap-audit evidence changes, or a maintainer decides whether the
  maximal-coordinate variational/NCP/IPM track should be promoted, parked, or
  split into a separate initiative.
- Finite-difference evidence reveals a needed change in the analytic derivation,
  the position-Jacobian treatment, or the architecture doc.
- A maintainer reprioritizes against other DART 7 clean-break work, or promotes
  the deferred GPU/batched/deformable differentiable tracks.
