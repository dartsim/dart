# Nimble Paper/Repository Gap Audit + Cross-Engine API Survey

Feature-by-feature parity tracking for PLAN-110 against the Nimble method and a
decision-relevant survey of other differentiable engines. This is the durable
evidence sidecar for [`110-differentiable-simulation.md`](../110-differentiable-simulation.md);
the durable architecture/API rationale lives in
[`../../design/differentiable_simulation.md`](../../design/differentiable_simulation.md).

Status legend: **PRESENT** (implemented + finite-difference checked),
**PARTIAL** (partly implemented or unchecked), **GAP** (not started),
**PREREQ** (blocked on a non-differentiability prerequisite).

> **As-built note.** The per-row `GAP`/`PREREQ` labels below were the
> **planning-time** gap analysis. As implemented (see `dashboard.md` PLAN-110 and
> the dev-task), the first slices of WS1–WS5 **and** the boxed-LCP contact
> prerequisite have all landed and are finite-difference-verified: opt-in seam,
> contact-free Jacobians for all joint types, the analytic contact gradient
> (frictionless + Coulomb friction + rotational/multi-contact), the
> `applyStepVjp`/`diff::rollout` reverse pass, the dartpy `sx.diff` bridge,
> mass/inertia/friction parameter derivatives, and the `ContactGradientMode`
> refinement modes. Read the rows below as the original baseline, not the current
> status; the dashboard and plan card own the current state.

## 1. Method Overview

Reference: Werling, Omens, Lee, Exarchos, Liu, "Fast and Feature-Complete
Differentiable Physics for Articulated Rigid Bodies," RSS 2021, arXiv:2103.16021;
reference implementation `keenon/nimblephysics` (itself a DART fork, so its
forward pass is DART's). Catalog id `werling-2021`.

The forward step is semi-implicit Euler with an LCP contact solve. With
`x=[q; q̇]`, control `u=τ`, parameters `θ`:

```
q̇_free = q̇ + Δt·M(q)⁻¹(τ − c(q,q̇))
solve LCP:  v = A f + b,  0 ≤ f ⊥ v ≥ 0,   A = J M⁻¹ Jᵀ,  b = J q̇_free
q̇'     = q̇_free + M⁻¹ Jᵀ f
q'      = q + Δt·q̇'
```

The contribution is **analytic** timestep Jacobians via implicit differentiation
of the LCP, exploiting active-set sparsity: classify each constraint as Clamping
(`f>0,v=0`), Separating (`f=0,v>0`), or Tied (`f=0,v=0`); restrict `A` to the
clamping rows `A_CC`; then `∂f_C/∂z = −A_CC⁻¹(∂A_CC/∂z·f_C + ∂b_C/∂z)`.
`A_CC` is `|C|×|C|` with `|C|` typically far below the total constraint count,
which is the source of the reported ~87x speedup over finite differencing.

## 2. Named Jacobians → DART Mapping

State `x=[q; q̇]`. Position integration is on the joint manifold, so the position
Jacobian is **joint-type-keyed, not identity** (verified in `simulateMultibody`,
`multibody_dynamics.cpp:1037-1067`): Euclidean DOFs give `∂q'/∂q=I`,
`∂q'/∂q̇=Δt·I`; SO(3)/ball give the right/left Jacobians (`dexp`/`dlog`) at `q`
and `q̇·Δt`; SE(3)/free add a translation-vs-orientation coupling block. The
blanket `∂q'/∂q=I` from the paper is false for DART's ball/free integration and
would fail the WS1 finite-difference gate.

| Quantity    | Expression (paper Eq. 7–10)                         | Nimble cache                       | DART public name              | DART seam (file)                                                     | Status                |
| ----------- | --------------------------------------------------- | ---------------------------------- | ----------------------------- | -------------------------------------------------------------------- | --------------------- |
| ∂q̇'/∂q      | `∂(M⁻¹z)/∂q + M⁻¹(−Δt·∂c/∂q + ∂Jᵀ/∂q·f + Jᵀ·∂f/∂q)` | `posVel`\*                         | block of `state_jacobian`     | `multibody_dynamics.*`, contact-Jacobian deriv                       | GAP                   |
| ∂q̇'/∂q̇      | `I + M⁻¹(−Δt·∂c/∂q̇ + Jᵀ·∂f/∂q̇)`                     | `velVel`                           | block of `state_jacobian`     | `multibody_dynamics.*`, LCP gradient                                 | GAP                   |
| ∂q̇'/∂τ      | `M⁻¹(Δt·I + Jᵀ·∂f/∂τ)`                              | `forceVel`                         | block of `control_jacobian`   | `multibody_dynamics.*`, LCP gradient                                 | GAP                   |
| ∂q̇'/∂θ      | `∂(M⁻¹z)/∂θ + M⁻¹(−Δt·∂c/∂θ + Jᵀ·∂f/∂θ)`            | `WithRespectTo`                    | block of `parameter_jacobian` | parameter selector + `multibody_dynamics.*`                          | GAP                   |
| ∂f_C/∂{A,b} | `−A_CC⁻¹(∂A_CC/∂z·f_C + ∂b_C/∂z)`                   | `ConstrainedGroupGradientMatrices` | (detail)                      | LCP snapshot + clamping solve at the _future_ experimental LCP stage | PREREQ (PLAN-080 WS4) |
| ∂(Jᵀf)/∂q   | screw-axis derivative of contact wrench             | `DifferentiableContactConstraint`  | (detail)                      | `computeMultibodyLinkJacobian` + contact                             | GAP                   |

\*Position Jacobians compose into the public `state_jacobian = ∂x'/∂x` and are
joint-type-keyed (Euclidean `I`/`Δt·I`; SO(3)/SE(3) `dexp`/`dlog` + free-joint
coupling), per the note above; `control_jacobian = ∂x'/∂u`. The smooth velocity
blocks (`∂q̇'/∂q`, `velVel`, `forceVel`) require `∂M/∂q`, `∂c/∂q`, `∂c/∂q̇` —
substantive spatial-algebra derivatives (the larger part of the work), produced
by the WS1-chosen mechanism (analytic recursion, the reserved autodiff-scalar
path, or FD-of-dynamics interim). The LCP-gradient `∂f_C/∂{A,b}` block is the
cheap part.

## 3. Forward-Pass Ingredient Inventory (what DART already owns)

| Ingredient                                             | DART location                                                                                                                                                                                                                                                                                                                    | Status for diff                                              |
| ------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| LCP library `{A,b,lo,hi,findex}` + solver              | `dart/math/lcp/` `LcpResult LcpSolver::solve(const LcpProblem&, VectorXd& x, const LcpOptions&)` (30+ solvers: Dantzig/Lemke/PGS/APGD/Newton). The analytic gradient needs a **pivoting (Dantzig-style)** solve for a clean active set; iterative solvers give a noisy, tolerance-dependent classification and are out of scope. | PRESENT as a library; not used by the experimental World yet |
| Mass matrix / Coriolis / gravity `M,C,g` (values only) | `compute/multibody_dynamics.*` `computeMultibodyDynamicsTerms` (returns `M,C·q̇,g`; **no** `∂M/∂q`, `∂c/∂q`)                                                                                                                                                                                                                      | PARTIAL (values PRESENT; derivatives GAP)                    |
| Link Jacobians `J`                                     | `compute/multibody_dynamics.hpp` `computeMultibodyLinkJacobian`                                                                                                                                                                                                                                                                  | PRESENT                                                      |
| Inverse dynamics                                       | `computeMultibodyInverseDynamics`                                                                                                                                                                                                                                                                                                | PRESENT                                                      |
| Joint types (forward)                                  | revolute/prismatic/screw/universal/planar/ball/**free** all implemented (`multibody_dynamics.cpp:192,277,1045-1067`; error strings list "free" as supported). The `multibody_dynamics.hpp:139` header comment claiming free is unimplemented is **stale** — trust the `.cpp`.                                                    | PRESENT                                                      |
| Boxed-LCP **contact** on experimental World            | none — contact is sequential-impulse PGS (`RigidBodyContactStage`, `simulateMultibody`); zero `dart/math/lcp/` usage in the experimental tree                                                                                                                                                                                    | PREREQ (PLAN-080 WS4)                                        |
| Model/State/Control/Contacts separation                | `dart/simulation/experimental/` (ECS + `StateSpace`)                                                                                                                                                                                                                                                                             | PRESENT                                                      |
| Reverse pass / snapshot / gradient                     | none in experimental tree (grep-confirmed)                                                                                                                                                                                                                                                                                       | GAP                                                          |

## 4. Feature-Complete Scope (Nimble "feature-complete" → DART rows)

"Feature-complete" in the paper = every forward-sim feature is also
differentiable. Target rows:

### 4.1 Differentiable joint types

| Joint                        | Forward in DART                                                                         | Differentiable                                                                                   | Status |
| ---------------------------- | --------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ | ------ |
| Revolute, Prismatic, Screw   | PRESENT                                                                                 | target WS1–2                                                                                     | GAP    |
| Universal, Planar            | PRESENT                                                                                 | target WS1–2                                                                                     | GAP    |
| Ball (exp. coords, SO(3))    | PRESENT                                                                                 | target WS1–2 (needs `dexp`/`dlog` position Jacobian)                                             | GAP    |
| Free / floating base (SE(3)) | PRESENT (free forward dynamics implemented, `multibody_dynamics.cpp:192,277,1045-1067`) | target WS1; needs SE(3) `dexp`/`dlog` position Jacobian + translation-orientation coupling block | GAP    |
| Weld / fixed                 | PRESENT                                                                                 | target WS1–2                                                                                     | GAP    |

The smooth articulated path differentiates in WS1 before any contact gradient,
but only for joint types whose forward dynamics exist and whose joint-type-keyed
position Jacobian is implemented. WS1 enumerates which joint types ship;
free/floating differentiability waits on its forward implementation.

### 4.2 Differentiable contact geometry

| Geometry                                 | Status             | Notes                                                        |
| ---------------------------------------- | ------------------ | ------------------------------------------------------------ |
| Sphere / box / capsule contact gradients | GAP (WS2)          | via shared native collision producing contact points/normals |
| Convex mesh (vertex-face, edge-edge)     | GAP (later in WS2) | depends on native collision contact features                 |

### 4.3 Differentiable contact models

| Model                              | Status    | Notes                                                            |
| ---------------------------------- | --------- | ---------------------------------------------------------------- |
| Inelastic (static) contact         | GAP (WS2) | the first contact-gradient slice                                 |
| Coulomb friction (linearized cone) | GAP (WS2) | friction rows linked via LCP `findex` upper-bound mapping        |
| Elastic / restitution              | GAP (WS5) | approximate continuous-collision-time model; labeled approximate |

### 4.4 Differentiable physical parameters (`WithRespectTo` → selector)

| Parameter                   | Status        | Notes                                   |
| --------------------------- | ------------- | --------------------------------------- |
| Position / velocity / force | GAP (WS1/WS3) | state & control derivatives             |
| Mass                        | GAP (WS4)     | registration-style selector with bounds |
| Center of mass              | GAP (WS4)     |                                         |
| Inertia                     | GAP (WS4)     |                                         |
| Friction coefficient        | GAP (WS4)     | enters the `findex` mapping             |
| Geometry scale              | Deferred      | lower priority than mass/COM/inertia    |

## 5. Internal Structure: Nimble `dart/neural/` → DART `diff/` module

| Nimble (upstream fork)                  | Role                                           | DART-owned analogue (`dart/simulation/experimental/diff/`) | Public?                  |
| --------------------------------------- | ---------------------------------------------- | ---------------------------------------------------------- | ------------------------ |
| `BackpropSnapshot`                      | per-step cache for backward                    | `StepSnapshot` (detail)                                    | no                       |
| `ConstrainedGroupGradientMatrices`      | clamping partition + `A_CC⁻¹` + `∂f_C/∂{A,b}`  | `ContactGradient` (detail)                                 | no                       |
| `DifferentiableContactConstraint`       | `∂(Jᵀf)/∂q` screw-axis algebra                 | `ContactJacobianDerivative` (detail)                       | no                       |
| `WithRespectTo` / `WithRespectToMass`   | parameter-selection abstraction                | `PhysicalParameterSelector` (value type)                   | yes                      |
| `forwardPass` + `backpropState`         | high-level forward+Jacobian / VJP entry points | `World::getStepDerivatives()` + `World::applyStepVjp()`    | yes                      |
| `IKMapping` / `MappedBackpropSnapshot`  | world-space coordinate mapping                 | deferred (map utility, later slice)                        | later                    |
| `timestep.py` `torch.autograd.Function` | PyTorch bridge                                 | dartpy `sx.diff.timestep` / `sx.diff.rollout`              | yes (optional submodule) |

Key divergences from upstream (all deliberate, see design doc):

- **Opt-in, not always-on.** Upstream always caches a snapshot in `forwardPass`;
  DART allocates the snapshot only when `differentiable=true`.
- **Framework-neutral core.** Upstream's gradient types are plain; the PyTorch
  coupling is in `python/`. DART keeps the same split: Eigen-typed core,
  torch only in the optional dartpy submodule (imported lazily).
- **DART-owned names.** No upstream project name appears as a solver identity.
- **Both explicit Jacobians and VJP** are public (upstream exposes both too;
  DART makes the explicit-Jacobian path first-class for iLQR/DDP/sensitivity).

## 6. Cross-Engine API Survey (decision evidence)

Surveyed May 2026 for the "best user-friendly API" comparison the goal requires.

| Engine                             | Diff mechanism                              | Opt-in?                                    | State model                                          | Contact gradient                                                | Lesson for DART                                                                       |
| ---------------------------------- | ------------------------------------------- | ------------------------------------------ | ---------------------------------------------------- | --------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| **Nimble** (DART fork)             | analytic LCP + PyTorch `autograd.Function`  | no (always on)                             | `[q; q̇]` flat; mutable world context                 | exact hard-contact within mode; complementarity-aware heuristic | adopt the method + `autograd.Function` UX; reject always-on                           |
| **Brax** (JAX)                     | JAX autodiff through pure functions         | no (functional)                            | immutable pytree `State{q,qd,x,xd,contact}`          | soft; NaN/zero-gradient bugs (#387, #540)                       | functional rollout + `vmap` batch; soft-contact autodiff is the failure mode to avoid |
| **Newton 1.2** (Warp)              | Warp reverse-mode tape, kernel adjoints     | **yes** (`requires_grad=True` + `wp.Tape`) | immutable `Model`+`State`+`Control`+`Contacts` (GPU) | per-solver; not all kernels have adjoints                       | validates DART's exact structure + opt-in; reverse-only is a limitation DART beats    |
| **Genesis 0.4** (Taichi/Quadrants) | source-transform AD                         | **yes** (`SimOptions(requires_grad=True)`) | substep-indexed fields; `gs.tensor`                  | rigid-body diff still unstable                                  | scene-level opt-in _flag_ (not a separate solver class) is the right ergonomic        |
| **MJX** (JAX)                      | JAX autodiff; classic FD `mjd_transitionFD` | inherent / FD-in-`autograd.Function`       | immutable `mjx.Data`                                 | soft; stiff-contact gradient oscillation                        | FD-in-`autograd.Function` is the correctness _checker_ pattern; A/B Jacobian naming   |
| **Dojo** (Julia)                   | implicit gradients through relaxed NCP/IPM  | central-path smoothness/accuracy knob      | maximal-coordinate rigid bodies                      | hard NCP with nonlinear friction cones                          | evaluate as a possible second solver family; do not replace the active boxed-LCP path |

Decisions taken (rationale in the design doc §Design Decisions):

- **Opt-in via a world-level flag** (Newton/Genesis), default off, zero cost off.
- **Analytic hard-contact gradient** (Nimble), DART's genuine advantage over the
  soft-contact-autodiff engines whose contact gradients are documented as
  unreliable.
- **DART-owned typed state** matching the Model/State/Control/Contacts structure
  Newton independently converged on.
- **Framework-neutral core + thin PyTorch `autograd.Function` bridge** for the
  lowest-friction Python UX without a core tensor dependency; JAX/dlpack later.
- **Explicit Jacobians + efficient VJP** to serve both optimal-control and
  learning users and to beat reverse-only tape engines.
- **Dojo-style solver separated from the active path**: the maximal-coordinate
  variational hard-contact NCP/IPM method is promising enough to audit, but it
  needs separate model/state, solver-residual, and central-path contracts before
  DART can expose it as a second opt-in solver family.

## 7. Gradient Correctness: Limits And Mitigations

Contact-gradient correctness is an open research problem (arXiv:2207.05060 shows
even analytic engines' gradients "are not always correct" across contact mode
switches). DART's stance: be exact where it is exact, document the rest, gate on
finite differencing, and make non-true-gradient aids opt-in.

| Issue                                                                                                                                                                                   | Where it bites                                                      | DART handling                                                                                                                                                 |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Mode-switch / Tied set non-differentiability                                                                                                                                            | all analytic methods                                                | documented subgradient; opt-in `COMPLEMENTARITY_AWARE` heuristic (labeled non-true; no FD gate)                                                               |
| **Non-smooth articulated features** (joint-limit clamping, velocity-limit clamping, Coulomb _joint_ friction stiction, velocity actuators) — all silently active in `simulateMultibody` | ordinary articulated scenes (e.g. a pendulum hitting a joint limit) | documented as subgradient boundaries; WS1 gradients exact only when these features are inactive; FD checker scoped to scenes that avoid the active boundaries |
| Zero gradient before contact                                                                                                                                                            | trajectory optimization                                             | opt-in `PRE_CONTACT_SURROGATE` backward-only surrogate (arXiv:2506.14186; no FD gate)                                                                         |
| Stiff-contact gradient oscillation                                                                                                                                                      | penalty models                                                      | N/A — DART uses hard LCP, not penalty contact                                                                                                                 |
| Rank-deficient `A_CC` (redundant contacts)                                                                                                                                              | stacking/parallel                                                   | complete-orthogonal-decomposition solve + CFM diagonal                                                                                                        |
| Warm-start changes active set across steps                                                                                                                                              | iterative re-solve                                                  | differentiable path fixes warm-start + pivot policy so the clamping classification is deterministic                                                           |
| Elastic/restitution                                                                                                                                                                     | bouncing                                                            | approximate continuous-collision model (WS5a), labeled approximate                                                                                            |

Mitigations surveyed but not adopted as defaults: TOI-position (DiffTaichi 2020),
TOI-velocity (arXiv:2305.00092), adaptive-timestep DiffMJX (arXiv:2506.14186).
These target penalty/soft-contact engines; DART's hard-LCP analytic path avoids
their root cause. Revisit only if evidence shows DART needs them.

## 7b. Planned Test And Benchmark Layout

Named owners so the FD gate and overhead claims are reproducible (code is the
source of truth once these land):

- Tests (`tests/unit/simulation/experimental/diff/`):
  - `test_diff_zero_cost_parity` — bitwise-identical final `[q;q̇]` + zero
    snapshot allocation with `differentiable=false` (WS1).
  - `test_diff_fd_smooth` — FD check of `state_jacobian`/`control_jacobian` on the
    contact-free scenes (single/double pendulum, free body) across all shipped
    joint types and the joint-type-keyed position Jacobian (WS1).
  - `test_diff_fd_contact` — FD check on contact scenes (sphere-on-ground,
    box-stack), Tied-set configs excluded/logged (WS2).
  - `test_diff_parameter_jacobian` — FD check of `parameter_jacobian` for
    mass/COM/inertia/friction (WS4).
  - `test_diff_determinism` — repeated differentiable steps reproduce Jacobians to
    tolerance (WS2/WS3).
  - dartpy: `import dartpy.simulation_experimental` succeeds with torch absent;
    `gradcheck`-style vs FD on `sx.diff.timestep`/`rollout` (WS3).
- Benchmarks (`tests/benchmark/simulation/experimental/`):
  - `bm_diff_step_overhead` — on/off step cost vs the stated budget (WS1/WS2).
  - `bm_diff_jacobian` — explicit-Jacobian vs VJP cost (WS3).
  - `bm_diff_rollout_memory` — peak memory vs the checkpointing knob (WS3).
- Examples: a trajectory-optimization example (WS3) and a system-identification
  example (WS4), each with a runnable command, mirroring PLAN-081's example
  evidence discipline.

## 8. Sources

- Nimble: arXiv:2103.16021; nimblephysics.org/docs (backprop, quick-start,
  understanding-worlds, system-identification); github.com/keenon/nimblephysics
  (`dart/neural/BackpropSnapshot.hpp`, `python/nimblephysics/timestep.py`); RSS
  2021 p034.
- Brax: github.com/google/brax (`brax/base.py`, `brax/envs/base.py`); arXiv:2106.13281;
  issues #387 (NaN generalized), #540 (inconsistent gradients).
- Newton: github.com/newton-physics/newton (diffsim ball/drone examples);
  newton-physics.github.io; PyPI `newton-physics` 1.2.0 (May 2026); NVIDIA Warp
  differentiability docs; Warp issue #200 (Featherstone adjoint).
- Genesis: genesis-world.readthedocs.io (differentiable_simulation,
  naming_and_variables, parallel_simulation); issue #1180.
- MuJoCo/MJX: mujoco.readthedocs.io (mjx, mjwarp, `mjd_transitionFD`);
  github.com/EladSharony/DiffMjStep; MJX NaN issues #1349, #2237.
- Dojo: arXiv:2203.00806; sites.google.com/view/dojo-sim/home;
  github.com/dojo-sim/Dojo.jl; PLAN-110 Dojo gap audit.
- Contact-gradient correctness: arXiv:2207.05060; arXiv:2305.00092 (TOI-velocity);
  arXiv:2506.14186 (hard-contacts-soft-gradients / CFD); arXiv:2409.07107 (explicit
  contact-mode differentiation); DiffTaichi arXiv:1910.00935.
