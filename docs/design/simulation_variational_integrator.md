# Linear-Time Variational Integrator (Discrete-Mechanics Solver)

## Status

Implemented for Phases A1, A2, B1, and B2, with Phase C C1-C3 implemented for
the scoped contact/friction envelope and arbitrary-geometry contact plus the C4
hard-barrier rung deferred. This document owns the durable design rationale for
the **linear-time variational integrator** in the DART 7 simulation
`World` integration family. It owns architecture and math rationale, not
timeline or status. Sequencing, phase gates, and acceptance criteria live in
PLAN-084
(`docs/plans/084-variational-integrator-solver.md` and its `contact-roadmap.md`
sidecar); the implementation tracker, measured paper-experiment replication, and
the per-gate evidence live in `docs/dev_tasks/variational_integrator_solver/`
(`README.md`, `RESUME.md`, `paper-experiment-replication.md`).

Companion docs:

- [`simulation_solver_architecture.md`](simulation_solver_architecture.md) — the
  solver/coupler/domain architecture this integrator plugs into.
- [`simulation_cpp_api.md`](simulation_cpp_api.md) —
  the public-facade capability matrix, which already lists `variational
integrator` as an integration-family value (`Solver And Execution Policy`).

This design was drafted, then revised against an adversarial architecture,
plan, risk, and numerical-correctness review; the review re-baselined every
"reuse / already exists" claim against the code as it is today. The honest
new-vs-reused split below is the result.

## Why This Integrator

DART's north star is that "the easiest place to reproduce and evaluate a new
algorithm should be inside DART." The DART 7 `World` already hosts a
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

## Improvements, Corrections, and New Features (relative to the paper)

This is a fresh implementation in DART's experimental ECS `World`, not a port of
the author's classic-DART reference (`github.com/jslee02/wafr2016`). It
reproduces the paper's results (see
[`../dev_tasks/variational_integrator_solver/paper-experiment-replication.md`](../dev_tasks/variational_integrator_solver/paper-experiment-replication.md))
and differs from the paper and its reference code in the following ways. Each
item names the artifact that verifies it.

### Corrections (bug fixes vs a naive port)

- **O(n) kinematic-tree build.** A direct port computed each link's parent by a
  linear scan (`std::find`), making tree construction O(n²) and silently
  defeating the linear-time claim. The scaling benchmark surfaced this; it is
  fixed with an `unordered_map` parent lookup, so `buildVarTree` is O(n)
  (`bm_variational_integration`, `BM_ArticulatedInverseMass` = `O(n)`).
- **Non-convergence is a hard error, not a silent best-effort step.** The
  reference returned the last iterate regardless of convergence (silent NaN /
  garbage risk). Here a step that fails to reach tolerance within `maxIterations`
  raises a documented `InvalidOperationException`
  (`NonConvergenceRaisesDocumentedError`).

### Improvements (more accurate / more robust than the paper)

- **Exact `dexp⁻¹` via `SE3::LeftJacobianInverse`** instead of the paper's
  truncated Bernoulli series `I − ad/2 + ad²/12 − ad⁴/720`. The closed-form
  left-Jacobian inverse is exact (no truncation error); cross-checked against
  `dart::math::lie_group` (`test_discrete_mechanics_lie_group_parity`,
  `DexpInvIsLeftJacobianInverse`).
- **Anderson-accelerated RIQN.** The paper's fixed `Δt·M⁻¹` quasi-Newton
  preconditioner converges in a few iterations for small systems but its rate
  degrades for long chains (measured: a 64-link chain peaked near **456**
  iterations; 100 links failed to converge within 500). Depth-`m` Anderson
  mixing of the fixed-point iteration — applied only where the generalized
  coordinates form a vector space (all joints revolute/prismatic), since
  spherical/floating coordinates cannot be linearly mixed — restores fast
  convergence (64 links: max **19**; 100 links: max **205**)
  (`LongChainConvergesWithinDefaultBudget`, `DISABLED_RiqnIterationsVsChainLength`).
- **Dimension-consistent (`√n`-scaled) convergence tolerance.** `tolerance` is a
  per-coordinate accuracy; the residual L2 norm is tested against
  `tolerance·√(dofCount)`, so the accuracy is uniform across chain lengths rather
  than `√n` times stricter for an `n`-DOF system (which left the hardest
  long-chain steps stalled just above an unscaled floor).

### New features (beyond the paper's scope)

- **Manifold-correct floating and spherical joints.** The paper's reference
  implementation only handled Euclidean joints in the RIQN retraction (open
  TODOs for ball/free). Here the increment is applied by SE(3)/SO(3) retraction
  (`R_new = R·exp(δω)`, `p_new = p + R·δp`), verified by floating free-fall,
  torque-free tumbling energy conservation, and a finite-difference manifold
  check (`FloatingBaseFreeFall`, `FloatingBaseTorqueFreeConservesEnergy`,
  `ExpRetractionTangentMatchesFiniteDifference`).
- **Holonomic loop closures through the public API.** The paper handles only
  joint equality constraints. Here Point, Distance, and Rigid closures (the
  paper's Sec. 5 extension) are enforced by an impulse-based projection onto
  `g(q)=0` reusing the O(n) ABI, wired through the World's `LoopClosure`
  components and solved on the `world.step()` path under the variational method
  (the semi-implicit path still rejects `Solve`). The constraint Jacobians
  (including the 6-row Rigid position+orientation Jacobian) are finite-difference
  verified (`ConstraintJacobianMatchesFiniteDifference`,
  `RigidConstraintJacobianMatchesFiniteDifference`,
  `LoopClosure{,Distance,Rigid}SolvedThroughWorldStep`).
- **State serialization.** The two-step discrete-mechanics history is a
  registered, serialized component, so a binary save/load resumes a trajectory
  bit-identically without re-bootstrapping (`StateSerializationRoundTripsTrajectory`).
- **Facade-safe solver configuration.** The integrator is selected by the
  documented method-family name through a `MultibodyOptions` value object
  (`WorldOptions::multibodyOptions` at construction, `World::setMultibodyOptions`,
  and `multibody_options` in dartpy) -- a categorized config that future
  capabilities extend as fields rather than as new World methods -- with no
  public exposure of solver/stage/component types;
  `check-api-boundaries` stays green.

## How It Maps Onto The Experimental World

### Insertion seam: a peer integration-family stage + a minimal selector

The `World` runs an ordered pipeline of `WorldStepStage` objects
(`compute/world_step_stage.hpp`); the variational integrator is introduced as a
new stage

`MultibodyVariationalIntegrationStage : WorldStepStage`

declared beside `MultibodyForwardDynamicsStage`
(`compute/multibody_dynamics.hpp`), implemented in `multibody_dynamics.cpp`, and
registered in `dart/simulation/CMakeLists.txt` (explicit source
list).

The selection mechanism is now implemented as a facade-safe integration-family
selector: `WorldOptions::multibodyOptions` at construction,
`World::setMultibodyOptions(...)` after construction, and the dartpy
`MultibodyOptions` binding carry the `"variational integrator"` method-family
name. The same value object carries the variational solve budget
(`variationalMaxIterations` / `variationalTolerance`) so paper-scale stability
smokes can raise the default `100` / `1e-10` budget without exposing solver,
stage, or component types. The built-in `World::step()` schedule is
centralized in `detail/world_step_schedule.hpp`; when the variational family is
selected and a multibody domain is active, the schedule substitutes
`MultibodyVariationalIntegrationStage` for the semi-implicit multibody stage
instead of appending both. The custom-final-stage `step(executor, stage)` path
reuses the same built-in schedule before the caller's final stage, so it does not
double-integrate a multibody. This is _not_ a plugin/factory registry (those
remain deferred per the facade rules); it is the smallest selector that lets the
default `step()` choose the dynamics stage while keeping selection scoped to the
`World` configuration.

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

**The central implemented component — an O(n) impulse-based ABI.** RIQN's
`Δt·M⁻¹·e` must be O(n). At Phase A1 time the DART 7 World only had dense
mass-matrix assembly/solve (`computeMassAndBias`, `ldlt()`), which was useful as
an oracle but could not support the linear-time claim. Phase A2 added the
two-sweep impulse / inverse-mass product (`computeMultibodyInverseMassProduct`)
so the selected VI path no longer depends on a dense placeholder. The dense
terms remain baseline/oracle tools; the O(n) inverse-mass path and its scaling
benchmark are now regression gates for the linear-time claim.

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
   potential) for the conservation test — implemented as
   `computeMultibodyMechanicalEnergy` and used by the VI energy-drift tests.

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
  risk; PLAN-084 gates repeated-run determinism explicitly.

### Coexistence with other solvers/backends

- **Deformable solver:** both are stages; for now the VI is single-domain and
  opt-in. Rigid-VI ↔ deformable cross-domain interaction is a future `Coupler`
  (solver-architecture doc), out of scope here.
- **Scalable-compute / CUDA:** the CUDA path is a rigid-body SoA _batch_; it
  shares no representation with articulated generalized coordinates. The VI is
  **CPU-only** for now; no GPU/batch synergy is claimed.
- **Python (`dartpy.simulation`):** the integration-family selector
  is the only new public surface and must bind cleanly; PLAN-084 includes a
  binding + import-coverage slice.

## Contact, Friction, And Constraints (durable rationale)

The paper handles only joint (equality) constraints via reduced coordinates;
closed loops, contact, and friction are out of scope (Sec. 5). The **durable
design constraint** (the sequencing lives in
`084-variational-integrator-solver/contact-roadmap.md`):

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
- **Contact-query boundary:** a root-finder needs contact/distance evaluated at
  the _trial_ `qᵏ⁺¹` each inner iteration. The implemented Phase C slice does
  this through VI contact hooks for link-point-vs-analytic-ground contact and a
  link-vs-link sphere-sphere hook. Arbitrary link geometry is still deferred
  because `World::collide()` remains a once-per-step collision-world rebuild,
  and DART still needs a warm-started "evaluate link geometry at this trial
  configuration" query object plus generalized-force projection from contact
  point to `Sᵢᵀ`.

The recommended ordering (equality constraints → friction kernels →
compliant/penalty contact → augmented-Lagrangian → optional hard barrier), with
the IPC/VBD/AVBD/XPBD evidence and per-rung tradeoffs, is in the contact-roadmap
sidecar. C1-C3 have landed for the supported envelope; arbitrary link geometry
and the optional C4 hard barrier remain separate follow-up work.

## Public Facade

- Selection by capability/method name only, via the integration-family selector
  value object. Capability-matrix row: Integration = `variational integrator`;
  Dynamics = `articulated-body method`; Coordinate = `generalized coordinates`;
  Supported features include joints, floating/spherical coordinates, loop
  closures, link-point ground contact, lagged friction, augmented-Lagrangian
  centering, and the link-sphere pair slice; arbitrary link geometry remains a
  declared future extension. Differentiability = `finite-difference checked`
  first, `analytic` once Alg. 4 lands.
- No leak of `WorldStepStage`, solver types, ECS components, or backend names.
  Previous-configuration State and solver diagnostics surface only through
  documented state/diagnostic views.
- Unsupported requests (for example unknown integration-family names,
  unsupported closure endpoint combinations, or contact outside the declared
  envelope) return documented errors, not silent fallback.

## Verification Approach

Concrete numeric gates live in PLAN-084; the _approach_ mirrors the paper's own
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

1. **O(n) inverse-mass path must stay load-bearing.** The O(n) headline depends
   on the impulse-ABI / inverse-mass product and recursive preconditioner staying
   on the selected path. Keep `computeMultibodyInverseMassProduct` and the
   scaling benchmarks as regression gates for any solver-stage refactor.
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
5. **Arbitrary-geometry contact query at trial configuration** remains
   unavailable (`collide()` is a once-per-step full rebuild). The implemented
   hook-based ground/sphere slices prove the solver seam; a production geometry
   adapter still needs a query redesign.
6. **RIQN non-convergence / joint limits:** define the failure contract; hard
   joint-limit clamps are non-smooth and conflict with the symplectic claim —
   decide whether limits are hard stops or barrier potentials.
7. **Log-map singularity** at large `Δt·V` / rotation ≈ π remains a numerical
   boundary even though `dexp⁻¹` now delegates to the exact SE(3)
   left-Jacobian inverse.
8. **Very long floating/spherical chains** may need a manifold extension of the
   exact recursive preconditioner beyond the currently documented supported
   envelope.
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
  `simulation_cpp_api.md`.

## Verification Expectations

Docs-only edits use the docs-only gate set in `docs/ai/verification.md`.
Implementation PRs realizing this design include `pixi run lint`,
`pixi run build`, focused C++ tests under `tests/unit/simulation/`,
`check-api-boundaries` when public headers/bindings change, benchmark evidence
for the O(n) claim (Phase A2), `test-py` when bindings change, a serialization
version-bump note, and a changelog entry. Reviewers reject any public API that
leaks solver, stage, component, or backend types.
