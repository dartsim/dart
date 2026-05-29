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
  - **The experimental contact path is currently sequential-impulse PGS, not a
    boxed LCP** (verified: zero `dart/math/lcp/` usage under
    `dart/simulation/experimental/`). The analytic contact gradient is undefined
    until the LCP solve exists on the experimental path — see Dependencies.
  - DART's manifold position integration (SO(3)/SE(3) exp/log) makes the position
    Jacobian joint-type-keyed, not identity; the design captures this.

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

- **Hard prerequisite — PLAN-080 Workstream 4** (boxed-LCP contact + joint-limit
  solving on the experimental `World`). This is **currently unstarted**: the
  experimental contact path is sequential-impulse PGS, and PLAN-080's own next
  step is still landing the rigid-body MVP. WS2–WS5 of this plan are therefore
  **Blocked** until WS4 lands an LCP solve. To avoid a stall and avoid
  re-touching WS4 later, PLAN-110 contributes a requirement _into_ PLAN-080 WS4:
  the experimental LCP contact solve must emit `{A,b,lo,hi,findex,f}` as
  first-class outputs (a snapshot-friendly, differentiation-ready seam), and
  should target a pivoting (Dantzig-style) solver that yields a clean active set.
- **WS1 is unblocked** — the contact-free articulated Jacobians (seams 3–5) do
  not depend on WS4.
- Coordinates with PLAN-030 (deferred GPU/batched differentiable track) and
  PLAN-081 (deferred differentiable-deformable track); neither blocks the
  rigid-body analytic CPU method.

## Workstreams

Sequenced; slice detail in the dev-task roadmap. Each slice keeps
`check-api-boundaries` green and flips gap-audit rows to PRESENT with
finite-difference evidence.

1. **Opt-in seam + contact-free Jacobians** (unblocked) — `WorldOptions::differentiable`
   (default false; new options/constructor surface), nullable-sink snapshot
   plumbing, the **chosen smooth-term derivative mechanism** (analytic
   spatial-algebra recursion, the reserved autodiff-scalar path, or FD-of-dynamics
   as an interim — decided and recorded at WS1 start), **joint-type-keyed position
   Jacobians** (Euclidean `I`/`Δt·I`; SO(3)/SE(3) `dexp`/`dlog` + free-joint
   coupling), `state_jacobian`/`control_jacobian` for the contact-free path, the
   FD checker, and the zero-cost parity test. Enumerate which joint types ship.
2. **Analytic contact gradient** (blocked on WS4) — clamping/separating/tied
   classification, `A_CC` rank-revealing solve with CFM, `∂f_C/∂{A,b}`, friction
   `findex` cone mapping (convention pinned to WS4's LCP), `∂(Jᵀf)/∂q`; target a
   pivoting solver (iterative solvers out of scope for the analytic gradient).
3. **Reverse product + PyTorch bridge** (blocked on WS2) — framework-neutral
   `applyStepVjp`, the optional `sx.diff` submodule (`timestep`/`rollout`
   `autograd.Function`, lazy torch), `state_vector`/`num_efforts` helpers, the
   checkpointing knob (recompute vs. store) with a peak-memory test, dartpy stubs,
   a torch-absent import test, Python tests, and a trajectory-optimization example.
4. **Parameter derivatives** (blocked on WS2) — `PhysicalParameterSelector`
   (mass/COM/inertia/friction with bounds), `parameter_jacobian`, a
   system-identification example.
5. **Refinement opt-in modes (split, separately gated)** —
   - 5a elastic/restitution (approximate continuous-collision model; FD-checked
     where defined; labeled approximate);
   - 5b complementarity-aware saddle escape (heuristic; **no FD gate by
     construction**; gated by a documented optimization-convergence benchmark);
   - 5c pre-contact surrogate (backward-only; **no FD gate**; gated by a
     documented trajectory-optimization task).

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
