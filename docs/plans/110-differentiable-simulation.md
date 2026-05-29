# PLAN-110: Differentiable Simulation (First Opt-In Gradient Capability)

- Operating state: `PLAN-110` in [`dashboard.md`](dashboard.md)
- Outcome: the experimental `World` can, **opt-in**, produce analytic derivatives
  of a physics step — state, control, and physical parameters — via implicit
  differentiation of the contact LCP and articulated-body dynamics, matching the
  Nimble method (arXiv:2103.16021). Derivatives are **exact within a contact mode**
  (subgradients at mode/limit switches; elastic contact approximate) and verified
  by finite differencing. Differentiability is off by default with bitwise-identical
  results when off; the public surface exposes DART-owned derivative value types
  and an optional PyTorch bridge, never solver internals, the reverse-pass cache,
  or a tensor backend.
- Current evidence:
  - The solver architecture reserves the seam: a _coupler_ reverse pass for
    cross-domain interaction and (this plan) a _rigid-body-solver-internal_
    reverse pass for contact/dynamics gradients; Model/State/Control/Contacts
    separation is "the precondition for … differentiability"
    (`docs/design/simulation_solver_architecture.md`).
  - The capability matrix's `Differentiability` axis is defined in
    `docs/design/simulation_experimental_cpp_api.md` (do not restate it here;
    that doc is the single source of truth for the axis values).
  - `compute_backend_research.md` recorded differentiability as _deferred_ with an
    explicit promotion question; this plan answers it (analytic method committed
    for contact; autodiff-scalar templating reserved for smooth terms only).
  - Forward-pass ingredients DART already owns: boxed-LCP library `dart/math/lcp/`
    (`LcpResult LcpSolver::solve(const LcpProblem&, VectorXd& x, const LcpOptions&)`),
    articulated-body dynamics `compute/multibody_dynamics.*` (`M, C, g`, link
    Jacobians), and the `tan-lcp` formulation.
  - As built: an opt-in boxed-LCP rigid-body contact path landed on the
    experimental `World` (`detail/boxed_lcp_contact.{hpp,cpp}` assembles the
    Delassus system `A = J M⁻¹ Jᵀ` and solves it via the `dart/math/lcp` Dantzig
    pivoting solver, behind `WorldOptions::contactSolverMethod == BoxedLcp`; the
    default `SequentialImpulse` path is unchanged). The analytic contact gradient
    is implemented on that seam (`detail/contact_jacobians.{hpp,cpp}`).
  - DART's manifold position integration (SO(3)/SE(3) exp/log) makes the position
    Jacobian joint-type-keyed, not identity; implemented in
    `detail/smooth_jacobians.cpp`.

## Owner Docs

- Architecture, API, decisions, algorithm mapping (durable source of truth):
  [`../design/differentiable_simulation.md`](../design/differentiable_simulation.md)
- Feature-by-feature Nimble parity audit + cross-engine API survey:
  [`110-differentiable-simulation/nimble-gap-audit.md`](110-differentiable-simulation/nimble-gap-audit.md)
- Public facade rules:
  [`../design/simulation_experimental_cpp_api.md`](../design/simulation_experimental_cpp_api.md),
  [`../design/simulation_experimental_python_api.md`](../design/simulation_experimental_python_api.md)
- Research reference: `werling-2021` in
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- Active implementation tracking (transient; durable owners are the design + gap
  audit): `docs/dev_tasks/differentiable_simulation/`.

## Dependencies

- **Boxed-LCP contact prerequisite — landed in this work** (the PLAN-080
  Workstream 4 seam, scoped opt-in here). The experimental `World` gained a
  `BoxedLcp` contact method whose solve emits the `{A,b,lo,hi,findex,f}` snapshot
  the analytic contact gradient differentiates, via a pivoting (Dantzig) solver
  with a clean active set — exactly the differentiation-ready seam this plan
  required. The default `SequentialImpulse` path is unchanged.
- Coordinates with PLAN-030 (deferred GPU/batched differentiable track) and
  PLAN-081 (deferred differentiable-deformable track); neither blocks the
  rigid-body analytic CPU method.

## Workstreams

First slices of all five workstreams are implemented and verified (see the
dashboard entry and `docs/dev_tasks/differentiable_simulation/`); each keeps
`check-api-boundaries` green and is finite-difference-of-step validated. Slice
detail lives in the dev-task roadmap.

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
   `applyStepVjp` and `diff::rollout`, the optional `sx.diff` submodule
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

Remaining follow-ups (not workstreams): worked trajectory-optimization /
system-identification example programs, the torch-autograd end-to-end test (needs
torch in-env), a `DART_BUILD_DIFF` CI job, and minor robustness/coverage items —
tracked in `docs/dev_tasks/differentiable_simulation/`.

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
- Gradient-correctness limits (mode-switch/limit subgradients, elastic
  approximation) are documented; saddle-escape and pre-contact surrogates are
  opt-in only.
- Architecture invariants held: it is the rigid-body-solver-internal reverse pass,
  the forward single-domain fast path is unchanged, and the public object model is
  not reshaped.

## Revision Triggers

- A workstream completes, splits, or is reprioritized.
- The PLAN-080 boxed-LCP contact contract changes shape, or the experimental LCP
  problem representation / `findex` convention changes (changes seams 1–2 and the
  friction-cone gradient).
- The boxed-LCP solver interface (`dart/math/lcp/`) changes.
- Finite-difference evidence reveals a needed change in the analytic derivation,
  the position-Jacobian treatment, or the architecture doc.
- A maintainer reprioritizes against other DART 7 → DART 8 work, or promotes the
  deferred GPU/batched/deformable differentiable tracks.
