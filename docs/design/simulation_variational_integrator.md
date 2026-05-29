# Linear-Time Variational Integrator (Discrete-Mechanics Solver)

## Status

Proposal. This document owns the durable design rationale for adding a
**linear-time variational integrator** to the experimental simulation `World`
as a new integration family. It owns architecture and math rationale, not
timeline or status. Sequencing, phase gates, and acceptance criteria live in
PLAN-082 (`docs/plans/082-variational-integrator-solver.md` and its
`contact-roadmap.md` sidecar); a `docs/dev_tasks/variational_integrator_solver/`
folder is created when implementation begins (per `docs/dev_tasks/README.md`).

Companion docs:

- [`simulation_solver_architecture.md`](simulation_solver_architecture.md) — the
  solver/coupler/domain architecture this integrator plugs into.
- [`simulation_experimental_cpp_api.md`](simulation_experimental_cpp_api.md) —
  the public-facade capability matrix, which already lists `variational
integrator` as an integration-family value (`Solver And Execution Policy`).

This design was drafted, then revised against an adversarial architecture,
plan, risk, and numerical-correctness review; the review re-baselined every
"reuse / already exists" claim against the code as it is today. The honest
new-vs-reused split below is the result.

## Why This Integrator

DART's north star is that "the easiest place to reproduce and evaluate a new
algorithm should be inside DART." The experimental `World` already hosts a
semi-implicit articulated forward-dynamics path (PLAN-080) and an IPC-class
deformable implicit-barrier path (PLAN-081). This adds a third, structurally
distinct _time-integration_ family:

> Lee, Liu, Park, Srinivasa. **"A Linear-Time Variational Integrator for
> Multibody Systems."** WAFR 2016 / arXiv:1609.02898 (catalog id
> `lee-vi-2016`).

Motivation:

1. **Structure preservation (scoped claim).** Variational integrators derive
   the update from a discrete least-action principle, so they are symplectic
   and **near-energy-conserving for smooth, conservative forcing at fixed
   `Δt`** — the regime of a passive articulated chain, where semi-implicit Euler
   visibly dissipates (paper Fig. 2: 10-link chain, 1e6 steps). This is the
   capability gap the current semi-implicit `MultibodyForwardDynamicsStage`
   cannot fill. The property is **not** advertised for systems with friction or
   contact, which are dissipative by design (see Risks).
2. **Linear-time scalability.** The paper's contribution is an O(n) solver
   versus the O(n³) of prior generalized-coordinate variational integrators.
   This is _relevant in principle_ to DART's scalable-compute direction
   (PLAN-030), but note the O(n) result depends on machinery DART must still
   build (see "the central new component"); the synergy is not mechanical today.
3. **First-party algorithm.** This is DART's own research lineage (implemented
   on classic DART; reference repo `jslee02/wafr2016`). Bringing it into the
   experimental world makes DART the reference reproduction.
4. **Differentiability runway.** The method admits an analytic recursive
   Jacobian (paper Appendix, Alg. 4), aligning with the capability matrix's
   `analytic` differentiability target.

## Method Summary

Notation follows the paper. Configuration `q` in generalized coordinates;
per-body pose `T ∈ SE(3)`; spatial velocity (twist) `V ∈ se(3)`, ordered
`(angular; linear)` — **matching DART's existing convention** (verified against
`adjoint`/`motionCross`/`spatialInertia`); spatial inertia `G`; joint motion
subspace `Sᵢ`; parent `λ(i)`; children `σ(i)`.

- **Per-step problem.** Advance by _root-finding_, not forward integration:
  given `qᵏ⁻¹, qᵏ`, solve for `qᵏ⁺¹` so the **forced discrete Euler–Lagrange
  (DEL)** residual vanishes (Eq. 3, 17):
  `f(qᵏ⁺¹) = D₂L_d(qᵏ⁻¹,qᵏ) + D₁L_d(qᵏ,qᵏ⁺¹) + Fᵏ = 0`,
  where `Fᵏ ∈ se*(3)` is the discrete (impulse) forcing.
- **Discrete Lagrangian (SE(3), trapezoidal):** `L = ½VᵀGV − P(T)` (Eq. 6),
  trapezoidal `L_d` (Eq. 9) with **average velocity** `Vᵏ = log(ΔTᵏ)/Δt`
  (Eq. 10). The `log` map makes the residual implicit in `qᵏ⁺¹`.
- **Gravity is forcing-side, not potential-side (decision).** The paper's
  Eq. 16b/16c carry a `(Δt/2)·P` potential term, but the reference
  implementation instead injects gravity as a per-body spatial impulse in `Fᵏ`
  and uses only the kinetic `½VᵀGV` in the momentum. We adopt the
  **forcing-side** convention. The two are equivalent at the continuous level;
  stating it explicitly prevents the double-counting bug of doing both. Joint
  actuation/friction always enter as `Qᵢ` (Eq. 22).
- **DRNEA — O(n) residual (Alg. 2):** a _discrete_ inverse-dynamics recursion.
  Forward sweep computes per-body average velocity `Vᵢᵏ = log(ΔTᵢᵏ)/Δt`
  (Eq. 19); backward sweep accumulates discrete momentum
  `μᵢᵏ = [dlog_{ΔtVᵢᵏ}]ᵀ Gᵢ Vᵢᵏ` (Eq. 21b) and transmitted impulse
  `Fᵢᵏ = μᵢᵏ − [Ad_{exp(ΔtVᵢᵏ⁻¹)}]ᵀ μᵢᵏ⁻¹ + Σ_{c}[Ad]ᵀF_cᵏ − Fᵢ^{ext}`
  (Eq. 21a); residual `fᵢ = Sᵢᵀ Fᵢᵏ − Qᵢᵏ` (Eq. 22).
- **RIQN — O(n) quasi-Newton root update (Alg. 3):** approximate
  `J_f⁻¹ ≈ Δt·M(qᵏ)⁻¹`; update `qᵏ⁺¹_{(l+1)} = qᵏ⁺¹_{(l)} ⊖ Δt·M⁻¹·e_{(l)}`
  with `e_{(l)} = f(qᵏ⁺¹_{(l)})`. The paper computes `Δt·M⁻¹·e` in **O(n) via an
  impulse-based articulated-body-inertia (ABI) algorithm** (set `q̇≡0`,
  `Q≡Δt·e`), _not_ a dense solve. Converges in ~5–6 iterations on the paper's
  benchmarks.
- **Initial guess (Sec. 3.3):** IG1 `qᵏ`, IG2 explicit Euler, IG3 semi-implicit
  Euler (default — reuses the existing semi-implicit manifold-integration path).
- **Manifold updates.** The `⊖`/`⊕` above are **manifold operations**, not
  vector arithmetic: for spherical/floating joints, velocity is `log(ΔT)/Δt` and
  the RIQN increment is applied by retraction (exp), not addition. The reference
  impl only ever handled Euclidean joints correctly here (open TODOs for
  ball/free), so this is genuinely new, correctness-sensitive work for Phase B.

## How It Maps Onto The Experimental World

### Insertion seam: a peer integration-family stage + a minimal selector

The `World` runs an ordered pipeline of `WorldStepStage` objects
(`compute/world_step_stage.hpp`); the variational integrator is introduced as a
new stage

`MultibodyVariationalIntegrationStage : WorldStepStage`

declared beside `MultibodyForwardDynamicsStage`
(`compute/multibody_dynamics.hpp`), implemented in `multibody_dynamics.cpp`, and
registered in `dart/simulation/experimental/CMakeLists.txt` (explicit source
list).

A **minimal selection mechanism is required** and must be designed (it does not
exist today). The default `World::step()` hard-codes its pipeline in four
overloads (`world.cpp`), and the single-stage `step(executor, WorldStepStage&)`
overload **appends** the supplied stage to that default pipeline — passing a VI
stage there would run semi-implicit _and_ variational integration on the same
multibody (double integration). Therefore the VI must enter by **pipeline
substitution** (replacing `MultibodyForwardDynamicsStage`), driven by a
facade-safe integration-family selector — a value object carrying the
`"variational integrator"` method name (modeled on the existing
`LoopClosureFamily`/spec value-object pattern), consulted by default-pipeline
construction. This is _not_ a plugin/factory registry (those remain deferred per
the facade rules); it is the smallest selector that lets the default `step()`
choose the dynamics stage. Selection scope (per-`World` vs per-`step`) is a
PLAN-082 decision.

### Honest reuse map (re-baselined against current code)

**Reusable as-is** (verified; conventions match the paper):

| Need                                        | Existing primitive (`multibody_dynamics.cpp`)                         |
| ------------------------------------------- | --------------------------------------------------------------------- |
| `Ad_T`, `ad_V`, `ad*_V`                     | `adjoint`, `motionCross`, `forceCross`                                |
| `Gᵢ` spatial inertia                        | `spatialInertia`                                                      |
| SO(3) `exp`/`log`                           | `rotationExp`, `rotationLog`                                          |
| `Sᵢ`, joint bias, all joint types           | `jointSubspaceInJointFrame`, `jointBiasTerms`, `jointMotionTransform` |
| Tree recursion `λ(i)/σ(i)`                  | `DynamicsTree`/`LinkDynamics`, `buildDynamicsTree`                    |
| Body Jacobians                              | `linkBodyJacobians`                                                   |
| IG3 bootstrap step                          | the existing semi-implicit manifold-integration path                  |
| Dense `M(q)`,`C`,`g` (oracle/baseline only) | `computeMultibodyDynamicsTerms` / `computeMassAndBias`                |

**The central new component — an O(n) impulse-based ABI.** RIQN's `Δt·M⁻¹·e`
must be O(n). The experimental World has **no ABA/ABI** — `computeMassAndBias`
builds a dense `M` column-by-column and `simulateMultibody` solves with
`ldlt()` (O(n²) assembly + O(n³) factor; its bias also includes gravity, so it
cannot be fed `q̇≡0,Q≡Δt·e` to yield a clean `Δt·M⁻¹·e`). The reference impl
used classic DART's `computeImpulseForwardDynamics` (a two-sweep bias-impulse /
velocity-change recursion); the experimental rewrite must implement that ABI
from scratch. This is the largest single piece of new work and the primary
risk; it is sequenced as an explicit early spike (PLAN-082 Phase A2). Until it
lands, a dense `M.ldlt().solve(Δt·e)` is an acceptable **O(n³) placeholder** for
correctness validation — but then no "linear-time" claim may be made and the
scaling benchmark is not yet a gate.

**Other genuinely new pieces:**

1. **SE(3) discrete-mechanics calculus** — this is **in common with
   `dart/math/lie_group/`**, so the VI reuses and validates against the shared
   library rather than maintaining a private copy. The general operations
   already exist there in the _matching_ `[angular; linear]` convention: SE(3)
   `exp`/`log` (`SE3Tangent::exp`, `SE3::log`), the Lie bracket
   (`SE3Tangent::ad`, identical to the VI's `ad`), and — crucially — the
   inverse right-trivialized tangent **`dexp⁻¹` is exactly
   `SE3::LeftJacobianInverse`** (the paper's `dlog_V`), as the exact
   Barfoot–Furgale closed form rather than the paper's 4th-order Bernoulli
   truncation (so it is strictly more accurate; the truncation error the paper
   carries is ~machine-ε at `Δt = 1 ms`). A small VI-specific kernel
   (`detail/variational/discrete_mechanics_math.hpp`) provides the
   discrete-mechanics-shaped wrappers (`dexpInvTranspose` for the discrete
   momentum `μ`, the dual adjoints `dAdT`/`dAdInvT` for momentum transport, and
   the `adInvRLinear` gravity helper); a parity unit test cross-checks every
   kernel against `dart::math::lie_group` so the equivalence is guarded, and the
   `dexp⁻¹` kernel delegates to `SE3::LeftJacobianInverse`. The existing
   experimental `rotationExp`/`rotationLog` are SO(3)-only and insufficient on
   their own. The Cayley map (Kobilarov–Crane–Desbrun) is the documented escape
   hatch if the `log`-map / `dexp⁻¹` rotation-by-π singularity ever becomes
   binding.
2. **The DRNEA residual recursion** — the continuous `recursiveNewtonEuler`
   (which computes `V=Sq̇`, accelerations) is _not_ reusable; DRNEA needs the
   average-velocity log map, discrete momentum, and the `Ad_{exp(ΔtV)}`
   previous-step momentum-transport term, with two-step history.
3. **The RIQN outer loop** (IG3 seed → DRNEA residual → ABI solve → manifold
   update), a `max_iter` cap, a **defined non-convergence contract** (a
   documented unsupported/diagnostic error — never a silent NaN or silent
   fallback), and convergence diagnostics (residual norm, iteration count).
4. **An articulated total-mechanical-energy helper** (kinetic + gravity
   potential) for the conservation test — none exists today (only per-rigid-body
   energy).

### State model: a two-step integrator needs history

The VI needs `qᵏ⁻¹`. This is **State**, stored as a per-multibody
previous-configuration buffer, owned by the stage and hidden from the facade.
The pattern is proven in-tree: the deformable solver stores
`DeformableNodeState.previousPositions` and reconstructs velocity as
`(positions − previous)/Δt` each step. Decisions:

- **Bootstrap (one strategy):** on the first step, synthesize `qᵏ⁻¹` from one
  IG3 semi-implicit step (equivalently seed `q̇ᵏ` and derive `qᵏ⁻¹` by manifold
  retraction), then run RIQN from step 1 onward.
- **Lifecycle on hooks that exist:** history is cleared by `clear()` and reset
  on `enterSimulationMode()`. There is **no** `World::reset()`/per-solver
  `finalize()` today; the doc does not depend on them. If the solver-architecture
  lifecycle (`finalize(model)`, substep windowing) later lands, the VI adopts it.
- **Serialization & determinism:** the previous-configuration State serializes
  with the world (joint state is already a serializable component), requiring a
  binary-format **version bump**. A world loaded mid-trajectory must carry
  `qᵏ⁻¹` and **not** re-bootstrap (a bootstrap-done flag), so save/load is
  bit-reproducible. An iterative root-finder with a tolerance is a determinism
  risk; PLAN-082 gates repeated-run determinism explicitly.

### Coexistence with other solvers/backends

- **Deformable solver:** both are stages; for now the VI is single-domain and
  opt-in. Rigid-VI ↔ deformable cross-domain interaction is a future `Coupler`
  (solver-architecture doc), out of scope here.
- **Scalable-compute / CUDA:** the CUDA path is a rigid-body SoA _batch_; it
  shares no representation with articulated generalized coordinates. The VI is
  **CPU-only** for now; no GPU/batch synergy is claimed.
- **Python (`dartpy.simulation_experimental`):** the integration-family selector
  is the only new public surface and must bind cleanly; PLAN-082 includes a
  binding + import-coverage slice.

## Contact, Friction, And Constraints (durable rationale)

The paper handles only joint (equality) constraints via reduced coordinates;
closed loops, contact, and friction are out of scope (Sec. 5). The **durable
design constraint** (the sequencing lives in
`082-variational-integrator-solver/contact-roadmap.md`):

- **Contact/friction can enter the VI only as a generalized force / spatial
  impulse** in the DEL residual (`Fᵏ`, `Qᵢ`, `Fᵢ^{ext}`). RIQN's `Δt·M⁻¹`
  preconditioner represents articulated-inertia curvature only — not stiff
  barrier Hessians, active sets, or inequality multipliers.
- **A smooth potential** added to the discrete Lagrangian (compliant penalty, or
  a barrier) keeps the integrator a _forced variational integrator_, so the
  symplectic structure survives (nonsmooth-/forced-VI theory: Fetecau–Marsden–
  Ortiz–West 2003; Kane–Marsden–Ortiz). Projection / velocity-level
  complementarity bolt-ons break it and add numerical damping.
- **Bounded-stiffness** contact (penalty / compliant / augmented-Lagrangian)
  keeps RIQN O(n); a **stiff log-barrier** injects large local curvature that
  `Δt·M⁻¹` mis-scales, risking iteration blow-up — so a hard barrier is the
  _last_, highest-risk option, not the first.
- **Prerequisite gap:** a root-finder needs contact/distance evaluated at the
  _trial_ `qᵏ⁺¹` each inner iteration. Today `World::collide()` rebuilds the
  whole collision world once per step (no distance-at-configuration query); any
  contact phase is blocked on a contact-query redesign. The barrier/distance/
  tangent/CCD kernels in `detail/deformable_contact/` are reusable _geometry_
  (12-DOF mesh-vertex stencils), but the (world contact point) → (link,
  body-frame point) → `Sᵢᵀ` generalized-force projection is net-new.

The recommended ordering (equality constraints → friction kernels →
compliant/penalty contact → augmented-Lagrangian → optional hard barrier), with
the IPC/VBD/AVBD/XPBD evidence and per-rung tradeoffs, is in the contact-roadmap
sidecar. Contact is **out of scope for the initial plan** and gated behind a
separate go/no-go.

## Public Facade

- Selection by capability/method name only, via the integration-family selector
  value object. Capability-matrix row: Integration = `variational integrator`;
  Dynamics = `articulated-body method`; Coordinate = `generalized coordinates`;
  Supported features grow per phase (`joints`, then `closed chains`, later
  `contacts`); Differentiability = `finite-difference checked` first, `analytic`
  once Alg. 4 lands.
- No leak of `WorldStepStage`, solver types, ECS components, or backend names.
  Previous-configuration State and solver diagnostics surface only through
  documented state/diagnostic views.
- Unsupported requests (e.g., contact under the equality-only phases, or
  floating base before Phase B) return documented unsupported-capability errors,
  not silent fallback.

## Verification Approach

Concrete numeric gates live in PLAN-082; the _approach_ mirrors the paper's own
experiments (reference repo `experiments/`: `energy_conservation`, `convergence`,
`iteration_numbers`, `performance_comparisons`), realized as DART tests/benchmarks:

- **Analytic single-DOF** (hard gate, like `test_world.cpp`): one VI step of a
  revolute pendulum / prismatic free-fall matches closed form.
- **Energy & momentum conservation** (the headline property, smooth/passive
  regime only): long-horizon passive chain shows bounded energy oscillation with
  no secular drift where semi-implicit Euler drifts; momentum conserved for
  force-free systems. Needs the new articulated-energy helper.
- **Convergence & iteration count:** residual decreases; mean/max RIQN
  iterations bounded and reported via diagnostics.
- **Parity & cross-check:** short-horizon trajectories track the semi-implicit
  path within tolerance; residual cross-checked against the analytic recursive
  Jacobian (Alg. 4) and/or finite differences.
- **Scaling (Phase A2 only):** per-step cost grows ~linearly in DOF once ABI
  lands; recorded as benchmark JSON.
- **Determinism & serialization:** repeated runs from identical state are
  bit-identical; save/load round-trips the history without re-bootstrapping.

## Risks & Open Questions

1. **No ABA today (the crux).** The O(n) headline depends on an impulse-ABI that
   does not exist; the current path is dense O(n³). Highest-leverage early spike:
   implement+verify O(n) ABI and benchmark `Δt·M⁻¹·e` vs DOF before building the
   rest. If ABI cannot be committed, drop the linear-time claim and ship the
   dense placeholder for correctness only.
2. **Energy claim is regime-specific.** Symplectic/near-energy-conserving only
   for smooth conservative forcing at fixed `Δt`; friction is deliberately
   dissipative and augmented-Lagrangian contact has no proven long-time energy
   behavior. Never advertise "energy-conserving" for frictional/contact scenes.
3. **RIQN preconditioner vs. chain length / barrier stiffness:** `Δt·M⁻¹` is an
   approximate inverse of the DEL Jacobian; its effectiveness degrades as the
   system gets longer or stiffer. _Measured_ (`bm_variational_integration`): the
   per-iteration cost is O(n) (3% RMS linear fit on the inverse-mass kernel) and
   the integrator converges in a few iterations and scales linearly for
   realistic DOF counts (≤~32), but the RIQN iteration count rises sharply for
   long chains (≥~64 links) at `Δt = 1 ms`. The IG3 (semi-implicit) initial
   guess is used and helps continuous rollouts but does not resolve the cliff —
   it is a convergence-_rate_ issue (the fixed `Δt·M⁻¹` preconditioner), not a
   starting-point one. Deeper mitigations (relative/scaled convergence
   tolerance, line-search or Anderson acceleration, or the exact
   recursive-Jacobian preconditioner from the paper's Appendix) are tracked
   follow-ups; stiff contact barriers would compound this, so favor compliant/AL
   contact and prototype any barrier on sparse contact first.
4. **Manifold RIQN increment for ball/free joints** — _resolved_ (Phase B1):
   `jointRetract`/`jointLogDifference` apply the increment on SO(3)/SE(3)
   (`R_new = R·exp(δω)`, `p_new = p + R·δp`), where the reference impl had open
   TODOs. Verified by floating-base free-fall (exact) and a tumbling
   asymmetric-inertia body conserving energy over 2e4 steps.
5. **Contact query at trial configuration** is unavailable (`collide()` is a
   once-per-step full rebuild); any contact phase needs a query redesign first.
6. **RIQN non-convergence / joint limits:** define the failure contract; hard
   joint-limit clamps are non-smooth and conflict with the symplectic claim —
   decide whether limits are hard stops or barrier potentials.
7. **`dexp⁻¹` truncation & log-map singularity** at large `Δt·V` / rotation ≈ π:
   pick/document the order; test near the singularity.
8. **Floating base is net-new** to the dynamics path (current stage is
   fixed-base, open-chain only) — Phase B cost regardless of the integrator.
9. **DynamicsTree rebuild cost:** cache the allocation across inner iterations
   and refresh only configuration-dependent fields, or constant factors regress.

## Alternatives Considered

- **Exact Newton with the recursive Jacobian (Alg. 4):** fewer iterations but
  O(n²) Jacobian + O(n³) solve — rejected as default; retained as a verification
  oracle and a differentiability building block.
- **Backward-Euler incremental potential (like `DeformableDynamicsStage`):**
  simpler and in-tree, but **not symplectic** (dissipates) — defeats the reason
  to add this family; useful only as a baseline.
- **Maximal-coordinate variational integrator:** simpler contact coupling but
  loses the generalized-coordinate O(n) structure and DART's articulated
  toolkit — rejected.

## References

- `lee-vi-2016` — Lee, Liu, Park, Srinivasa, "A Linear-Time Variational
  Integrator for Multibody Systems," WAFR 2016 / arXiv:1609.02898. Reference
  impl: `github.com/jslee02/wafr2016`.
- `marsden-west-2001` — Marsden & West, "Discrete mechanics and variational
  integrators," Acta Numerica 10.
- Forced/nonsmooth VI: Fetecau, Marsden, Ortiz, West (2003); Kane, Marsden,
  Ortiz, West.
- Contact-extension inspiration (catalog): `ipc-2020`, `macklin-xpbd-2016`,
  `vbd-2024`, `avbd-2025`.
- Architecture: `simulation_solver_architecture.md`,
  `simulation_experimental_cpp_api.md`.

## Verification Expectations

Docs-only edits use the docs-only gate set in `docs/ai/verification.md`.
Implementation PRs realizing this design include `pixi run lint`,
`pixi run build`, focused C++ tests under `tests/unit/simulation/experimental/`,
`check-api-boundaries` when public headers/bindings change, benchmark evidence
for the O(n) claim (Phase A2), `test-py` when bindings change, a serialization
version-bump note, and a changelog entry. Reviewers reject any public API that
leaks solver, stage, component, or backend types.
