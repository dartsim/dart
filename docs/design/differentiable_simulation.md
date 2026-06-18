# Differentiable Simulation Architecture And API

## Status

Living design and implementation contract. This document owns the durable
architecture and public-API rationale for **opt-in differentiable simulation**
in the DART 7 `World`: how DART computes derivatives of a physics step,
what the public surface looks like, how differentiability stays opt-in and
backend-neutral, and how the Nimble-style analytic LCP-gradient method maps onto
DART's existing boxed-LCP and articulated-body dynamics.

It is the differentiability companion to the existing design docs and does not
restate their rules:

- [`simulation_solver_architecture.md`](simulation_solver_architecture.md) owns
  the multi-solver/multi-physics engine architecture. It reserves two reverse
  passes: a _coupler_ reverse pass for cross-domain interaction, and (added for
  this plan) a _rigid-body-solver-internal_ reverse pass for contact/dynamics
  gradients. It names Model/State/Control/Contacts separation "the precondition
  for batching and differentiability."
- [`simulation_cpp_api.md`](simulation_cpp_api.md)
  owns the public C++ object model, the solver capability matrix (whose
  `Differentiability` axis is `unsupported, finite-difference checked, analytic,
autodiff`), and the DART 7 promotion contract.
- [`simulation_python_api.md`](simulation_python_api.md)
  owns the dartpy facade, the `StateSpace` metadata object, and the functional
  `sx.state.rollout` shape this design extends rather than forks.
- [`compute_backend_research.md`](compute_backend_research.md) recorded
  differentiability as a _deferred_ capability and the open question of whether
  to promote it. This document is the affirmative answer and supersedes the
  deferred status for the analytic-gradient method.

Active operating state lives in `docs/plans/dashboard.md` (PLAN-110) and
`docs/dev_tasks/differentiable_simulation/`. Feature-by-feature parity tracking
against the Nimble paper and repository lives in
`docs/plans/110-differentiable-simulation/nimble-gap-audit.md`; the planned
Dojo-style solver evaluation lives in
`docs/plans/110-differentiable-simulation/dojo-gap-audit.md`.

This document was revised after architecture, plan, and API reviews. The
"prose ran ahead of the code" in two places that those reviews corrected and
that this revision now states accurately: (1) DART's manifold integrator makes
the position Jacobian joint-type-dependent, not identity; and (2) the
experimental contact path is currently sequential-impulse, not a boxed LCP, so
the analytic _contact_ gradient is undefined until that path becomes an LCP
solve (PLAN-080 Workstream 4).

## Purpose

DART should let a researcher compute exact, fast gradients of a simulation step
with respect to state, control, and physical parameters — for trajectory
optimization, system identification, optimal control, and gradient-based policy
learning — **without paying any cost when they do not ask for gradients**, and
without learning solver, backend, or ECS vocabulary.

The target capability is the one demonstrated by Nimble Physics (Werling,
Omens, Lee, Exarchos, Liu, RSS 2021; arXiv 2103.16021), itself a DART fork:
analytical gradients through the contact LCP in generalized coordinates,
reported at up to 87x over finite differencing, covering all joint types and
mesh contact. DART already owns much of the forward-pass machinery — the
boxed-LCP library (`dart/math/lcp/`, the `tan-lcp` formulation), articulated-body
dynamics (`computeMultibodyDynamicsTerms`, `computeMultibodyLinkJacobian`), and a
typed Model/State/Control/Contacts separation — which makes DART the natural home
to reproduce and improve on that method rather than maintain a fork.

PLAN-110 also tracks Dojo (Howell et al., arXiv 2203.00806) as an additional
differentiable-rigid-body solver reference. Dojo is not the current
implementation target: it uses a maximal-coordinate variational hard-contact
NCP with nonlinear friction cones, a custom primal-dual interior-point solver,
and implicit gradients through a relaxed contact solve. That is a separate
evaluation track for a possible second opt-in solver family after the
Nimble-style path lands.

Architecturally, that means Dojo-style work fits behind the same experimental
`World` multi-solver facade as the existing rigid solver. It is not a new public
engine, and it is not a `ContactGradientMode` refinement of the boxed-LCP path.
It would be an alternate rigid-body method family with its own model/state
mapping, forward NCP/IPM solve, and solver-internal reverse pass. The public
surface would continue to expose DART-owned state/control/parameter derivative
objects; selection would use DART-owned solver-capability options, and unsupported
builds would report unsupported-capability errors.

The core design sentence is:

> Differentiable simulation is an **opt-in mode** of the same experimental
> `World`. When enabled, a step also records a reverse-pass snapshot and can
> produce DART-owned derivative value types via analytic differentiation of the
> LCP contact solve and the dynamics; when disabled, the step and its cost are
> bitwise-identical to the non-differentiable path. The public surface exposes
> typed state/control/parameter derivatives and an optional framework bridge,
> never solver internals, the reverse-pass cache, or a tensor backend.

## Scope

In scope: the opt-in model and overhead contract; the public C++ and dartpy
surface; the analytic-gradient method and its mapping onto DART's existing
seams; the internal reverse-pass architecture; the verification strategy (finite
differencing is the correctness gate); honest documentation of correctness
limits; and phasing, including the hard prerequisite on the LCP contact path.

Out of scope for the current committed implementation: GPU/accelerator
differentiable kernels; batched differentiable rollouts beyond a documented
leading-dimension contract; autodiff-scalar templating of the _contact_ path;
differentiable deformable/IPC contact (PLAN-081); and a public third-party
custom-gradient plugin API. The Dojo-style maximal-coordinate variational/NCP/IPM
solver is in scope only as a planned audit/spike track until PLAN-110 records a
promotion or split decision.

## Background: What The Method Computes

DART's experimental step is semi-implicit Euler with a contact/constraint solve.
With state `x = [q; q̇]`, control `u = τ`, parameters `θ`, the **smooth core** of
one step is:

```
q̇_free  = q̇ + Δt · M(q)⁻¹ (τ − c(q, q̇))          # unconstrained velocity
solve LCP: v = A f + b,  0 ≤ f ⊥ v ≥ 0,            # A = J M⁻¹ Jᵀ,  b = J q̇_free
q̇'       = q̇_free + M⁻¹ Jᵀ f                       # constrained velocity
q'        = integrate(q, q̇', Δt)                    # position update (see below)
```

This clean model is the differentiable core. DART's actual forward path also
applies, between the unconstrained velocity and the position update, several
**non-smooth operations** that the design must treat honestly (verified in
`compute/multibody_dynamics.cpp` `simulateMultibody`): Coulomb _joint_ friction
(velocity-level stiction), velocity-actuator equality constraints, joint-limit
and velocity-limit clamping. Each has a switching point at which the step is
non-differentiable. The analytic gradient is exact only away from those
switching points and away from the contact-mode boundaries discussed under
[Gradient Correctness](#gradient-correctness-and-verification); the
finite-difference gate is scoped to scenes that avoid active boundaries (or
flags configurations that hit them).

The method (Nimble) computes the timestep Jacobians **analytically**. The two
parts have very different costs, which the design keeps distinct (see D1):

- the **contact block** is implicit differentiation of the LCP — a small dense
  solve plus bookkeeping (cheap);
- the **smooth articulated terms** (`∂M/∂q`, `∂c/∂q`, `∂c/∂q̇`, and the
  manifold position Jacobians below) are substantive spatial-algebra derivatives
  (the harder, larger part of the work, comparable to a differentiated
  Featherstone pass).

### Named Jacobians → DART mapping

| DART-owned name      | Derivative                                    | Note                                                              |
| -------------------- | --------------------------------------------- | ----------------------------------------------------------------- |
| `state_jacobian`     | ∂x'/∂x = [[∂q'/∂q, ∂q'/∂q̇], [∂q̇'/∂q, ∂q̇'/∂q̇]] | position blocks are joint-type-keyed (below)                      |
| `control_jacobian`   | ∂x'/∂u                                        | velocity block `M⁻¹(Δt·I + Jᵀ ∂f/∂τ)`, position block `Δt·∂q̇'/∂τ` |
| `parameter_jacobian` | ∂x'/∂θ (optional)                             | mass/COM/inertia/friction; opt-in selection                       |

The velocity blocks are the Nimble caches (`velVel = I + M⁻¹(−Δt ∂c/∂q̇ + Jᵀ
∂f/∂q̇)`, `velPos`-type `∂q̇'/∂q`, `forceVel = M⁻¹(Δt·I + Jᵀ ∂f/∂τ)`), and
`∂f/∂·` comes from the LCP gradient below.

### Position Jacobian is joint-type-keyed (not identity)

DART integrates position on the joint's manifold, so the position Jacobian is
**not** the textbook `∂q'/∂q = I`, `∂q'/∂q̇ = Δt·I` for rotational/free joints
(verified in `simulateMultibody`, `multibody_dynamics.cpp:1037-1067`):

- **Euclidean coordinates** (fixed scalar part of revolute/prismatic/screw,
  translational DOFs): `∂q'/∂q = I`, `∂q'/∂q̇ = Δt·I`.
- **Spherical / ball** (SO(3), integrated as `Log(Exp(q)·Exp(q̇·Δt))`):
  `∂q'/∂q` and `∂q'/∂q̇` are the right/left Jacobians of SO(3)
  (`dexp`/`dlog`) at `q` and `q̇·Δt`; they reduce to `I`/`Δt·I` only at zero
  rotation.
- **Floating / free** (SE(3)): the SO(3) blocks above, plus a nonzero
  translation-vs-orientation coupling block because the translation update is
  rotated by `R(orientation)`.

WS1 must implement these joint-type-keyed position Jacobians, because WS1 targets
all joint types and the finite-difference checker is the WS1 gate; a blanket
`I`/`Δt·I` would fail FD on any ball/free scene.

### Gradient through the LCP (the analytic core)

After the LCP solve, classify each constraint row: **Clamping** (`f_i>0, v_i=0`),
**Separating** (`f_i=0, v_i>0`), **Tied** (`f_i=0, v_i=0`, the degenerate
non-differentiable boundary). Restricting `A` to the clamping rows `C` gives a
small `|C|×|C|` block `A_CC`; at the active set `f_C = −A_CC⁻¹ b_C`, so

```
∂f_C/∂z = −A_CC⁻¹ ( ∂A_CC/∂z · f_C + ∂b_C/∂z )
```

(standard implicit-function-theorem result on the equality-constrained KKT
block; the sign is correct). `A_CC⁻¹` is realized by a rank-revealing solve
(complete orthogonal decomposition) with a constraint-force-mixing (CFM) diagonal
for redundant contacts. Friction rows link to their normal row through the LCP
friction index (`findex`); the gradient's friction-cone derivative depends on the
exact `findex`/linearized-cone convention the forward LCP uses, which must be
pinned once PLAN-080 WS4 defines it.

This analytic LCP gradient is the differentiator versus Brax/Newton/MJX, which
differentiate a _soft/penalty_ contact model through framework autodiff and
consequently suffer documented gradient pathologies (NaNs, stiffness-driven
oscillation, zero-before-contact). It is _exact within a contact mode_. Its
honest limit — shared by all analytic methods — is that it does not capture
sensitivity _across_ a mode switch.

## Cross-Engine Evidence

Surveyed for API ergonomics and the opt-in/overhead model (May 2026). Full notes
and sources live in the PLAN-110 gap audit; the decision-relevant lessons:

| Engine          | Diff mechanism                                      | Opt-in?                                                        | State model                                    | Contact gradient                                                            | Lesson taken into DART                                                                            |
| --------------- | --------------------------------------------------- | -------------------------------------------------------------- | ---------------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| **Nimble**      | Analytic LCP gradient + PyTorch `autograd.Function` | No — always on                                                 | `[q; q̇]` flat, mutable world context           | Exact hard-contact within mode; complementarity-aware heuristic for saddles | Adopt the method and the `autograd.Function` ergonomics; **reject always-on**                     |
| **Brax**        | JAX autodiff through pure functions                 | No — always on (functional)                                    | immutable pytree `State`                       | Soft; documented NaN/zero-gradient bugs                                     | Functional rollout composes with batch; soft-contact autodiff is the failure mode                 |
| **Newton 1.2**  | Warp reverse-mode tape, kernel adjoints             | **Yes** — `requires_grad` + `wp.Tape`; zero overhead otherwise | immutable `Model`+`State`+`Control`+`Contacts` | Per-solver; not all kernels have adjoints                                   | **Validates DART's structure and opt-in model**; reverse-only is a limit DART beats               |
| **Genesis 0.4** | Taichi source-transform AD                          | **Yes** — `SimOptions(requires_grad=True)` flag                | substep-indexed fields                         | Rigid-body diff still unstable                                              | Scene-level opt-in _flag_ (not a separate solver class) is the right ergonomic                    |
| **MJX**         | JAX autodiff; classic FD `mjd_transitionFD`         | inherent / FD in `autograd.Function`                           | immutable `mjx.Data`                           | Soft; stiff-contact oscillation                                             | FD-in-`autograd.Function` is the correctness _checker_ pattern                                    |
| **Dojo**        | Implicit gradients through a relaxed NCP/IPM solve  | Solver tolerance / central-path smoothness knob                | maximal-coordinate rigid bodies                | hard NCP with nonlinear friction cones                                      | Worth auditing as a second solver family; not a dependency or replacement for the active LCP path |

Synthesis lessons that shaped the decisions: the Model/State/Control/Contacts
structure DART already designed is the structure Newton converged on; opt-in via
a scene/world-level flag is the right ergonomic; the PyTorch `autograd.Function`
bridge is the lowest-friction Python UX and decouples a C++/analytic backward
from the framework; and contact-gradient correctness is an open problem (arXiv
2207.05060) so DART's credibility comes from analytic exactness _plus_ honest
documentation _plus_ a finite-difference gate. Dojo adds a different research
lesson: a maximal-coordinate variational hard-contact NCP/IPM solver can expose
a smoothness/accuracy tradeoff through the interior-point central path, but that
requires a separate solver contract and cannot be folded into the boxed-LCP
reverse pass without a gap audit and spike.

## Design Decisions

### D1 — Analytic LCP contact gradient; smooth-term derivatives by a chosen mechanism

DART commits to the **analytic** method (the capability-matrix `analytic`
value). The two parts are treated distinctly:

- **Contact block** — implicit differentiation of the LCP (the
  clamping/separating/tied partition and the small `A_CC` solve). This is the
  cheap "bookkeeping plus one small dense solve" and is Nimble's contribution and
  DART's hard-contact advantage. Autodiff _through the iterative LCP solver_ is
  rejected: it is slow and wrong at the solver's non-smooth points.
- **Smooth articulated terms** — `∂M/∂q`, `∂c/∂q`, `∂c/∂q̇`, and the manifold
  position Jacobians are substantive derivatives, **not** cheap bookkeeping. WS1
  chooses one mechanism for them: (a) analytic spatial-algebra recursion (a
  differentiated Featherstone pass, most accurate/fastest, most work); (b) the
  scalar-generic autodiff path the dynamics already keeps open (instantiate the
  scalar-generic dynamics on an AD scalar — less work, some overhead); or (c)
  finite-difference-of-dynamics as an interim to unblock WS1 while (a) is built.
  The contact block stays analytic regardless. This decision is recorded in the
  dev-task roadmap when WS1 starts.

### D2 — Opt-in, default off, bitwise-identical when off

Differentiability is **off by default** and enabled per `World` (imperative
path) or per rollout/`sx.diff` call (functional path). When off, the step
executes the identical non-differentiable code path: the contract is _bitwise-
identical final state and zero reverse-snapshot allocation_, enforced by a named
executor-parity test (`test_diff_zero_cost_parity`). Snapshot capture is plumbed
as a **nullable sink** passed through the forward functions (`if (sink)
sink->record(...)`), populated only when on, so the off-path is a single
predictable branch (see [Internal Architecture](#internal-architecture)). Overhead
when on is **measured and reported by a benchmark against a stated budget**, not
asserted as "zero." This deliberately diverges from Nimble (always-on) and
matches Newton/Genesis, because the reverse cache (per-step LCP snapshot,
clamping classification, `A_CC` factorization) carries real cost that non-gradient
workloads should never pay — "opt-in because it comes with overhead by design" is
a stated project requirement.

### D3 — DART-owned typed state, building on the existing separation

The differentiable surface reuses the Model/State/Control/Contacts separation and
the existing `StateSpace` metadata. State is the existing `[q; q̇]` content;
control is generalized effort `τ`; parameters are an explicit, opt-in selection
(mass, COM, inertia, friction) via a DART-owned selector. No raw ECS storage,
registry, or component types are exposed. Derivatives are typed value objects
with documented shape, dtype, ownership, and freshness (valid until the next
`step()`).

### D4 — Framework-neutral C++ core, optional PyTorch bridge in dartpy

The C++ core produces plain Eigen-typed derivative value objects and knows
nothing about any tensor framework. The lowest-friction Python experience — a
`torch.autograd.Function` so users write a normal loop and call `.backward()` —
lives only in the optional dartpy `sx.diff` submodule and is a thin adapter over
the framework-neutral core. `import dartpy.simulation` must succeed with torch
absent; `sx.diff` always exists as an attribute, and calling into it
without torch raises a clear, guiding `ImportError`. This keeps the engine
dependency-free and leaves room for a future JAX/`dlpack` bridge added the same
way.

### D5 — Expose both explicit Jacobians and an efficient reverse product

The analytic method yields explicit Jacobian blocks, so DART offers both: an
explicit-Jacobian accessor (for iLQR/DDP, sensitivity, inspection) — an advantage
over reverse-only tape engines (Warp/Newton) — and an efficient vector-Jacobian
product (for long-rollout learning, the path the PyTorch bridge uses) that avoids
forming full matrices. The explicit dense Jacobian (O(ndof²)) is opt-in, not the
implied default, and exploits per-island block structure rather than always
materializing a dense matrix.

### D6 — Contact-gradient honesty and a finite-difference correctness gate

DART documents where gradients are exact (within a fixed contact mode, away from
the non-smooth feature switches listed in Background), where they are
subgradients (mode switches / Tied set / joint-limit / velocity-limit / Coulomb
joint-friction boundaries), and where they are approximate (elastic/restitution).
The committed correctness mechanism is a finite-difference checker with **concrete
defaults** (see Verification). Saddle-escape (complementarity-aware) and
zero-before-contact (pre-contact surrogate) aids are **separate opt-in modes**
that intentionally return a non-true gradient and have no FD gate by construction.

### D7 — A rigid-body-solver-internal reverse pass, not a new engine

Differentiability is an additive capability of the rigid-body solver: a
_solver-internal_ reverse pass over contact and dynamics. This is distinct from
the _coupler_ reverse pass the architecture reserves for cross-domain interaction
(rigid-internal contact is the rigid-body solver's responsibility, not a
coupler). There is no new user-facing engine and no parallel `DiffWorld` type.
The reverse-pass snapshot is owned by the `World`/solver (not by the stack-local
step stages), which requires the snapshot-ownership plumbing in
[Internal Architecture](#internal-architecture).

## Public API Design

Progressive disclosure: the common path is unchanged (default off); the
gradient headline is one import and one call; advanced surfaces escalate
sensibly. Names describe the method/quantity, never an engine. One import alias
(`sx`) throughout.

### Python — headline (lowest friction, lead with this)

Using `sx.diff` _is_ the opt-in for the functional/learning path; a user who
just wants `.backward()` writes a normal loop, exactly as in Nimble:

```python
from dartpy import simulation as sx
import torch

world = sx.World(time_step=0.001)
arm = world.add_multibody("arm")
# ... build topology ...

state  = torch.tensor(world.state_vector, requires_grad=True)   # [q; q̇] (property)
action = torch.zeros(world.num_efforts, requires_grad=True)
for _ in range(100):
    state = sx.diff.timestep(world, state, action)              # differentiable step
loss = state[:world.num_dofs].norm()
loss.backward()                                                  # state/action .grad set
```

### Python — explicit Jacobians (for optimal control / inspection)

For users who need the matrices themselves, opt in at the `World` and read the
typed derivatives of the most recent step:

```python
world = sx.World(time_step=0.001, differentiable=True)   # opt-in; default False
# ... build, step ...
world.step()
d = world.get_step_derivatives()        # method: allocates a snapshot-backed value object
d.state_jacobian                         # ∂x'/∂x   (2·ndof × 2·ndof)
d.control_jacobian                       # ∂x'/∂u   (2·ndof × nu)
# d.parameter_jacobian present only if differentiable parameters were registered
```

### Python — functional rollout (one rollout API, not a fork)

Differentiability is an opt-in flag on the _existing_ `sx.state.rollout` shape,
preserving its `out=` buffer contract; `sx.diff.rollout` is documented as the
torch-tensor bridge over it, not a second rollout:

```python
traj = sx.state.rollout(
    world.model(),
    initial_state=state0,        # StateSpace-typed
    control=control_sequence,    # [steps, nu] (optional leading batch dim)
    steps=200,
    differentiable=True,         # opt-in capture; default False keeps house behavior
    out=optional_output_buffer,  # house out= contract preserved
)
grads = traj.gradients(d_loss_d_states)   # → grads.state, grads.control, grads.parameter
```

(`gradients(...)`, not `backward(...)`, to avoid colliding with torch
`Tensor.backward()` semantics.)

### Python — opt-in parameter derivatives and gradient modes

```python
world.add_differentiable_parameter(arm.links["forearm"], sx.PhysicalParameter.MASS)
# subsequent get_step_derivatives()/gradients() now include the parameter block

world.contact_gradient_mode = sx.ContactGradientMode.ANALYTIC            # default
# sx.ContactGradientMode.COMPLEMENTARITY_AWARE   # saddle-escape heuristic (not the true gradient)
# sx.ContactGradientMode.PRE_CONTACT_SURROGATE   # nonzero pre-contact gradient (backward-only surrogate)
```

### C++ (`dart::simulation`)

Mirrors the Python facade with value objects, no framework types:

```cpp
WorldOptions opts;            // construction-time value object
opts.timeStep = 0.001;
opts.differentiable = true;   // opt-in; default false
World world(opts);

world.step();
const StepDerivatives& d = world.getStepDerivatives();  // throws if not differentiable
d.stateJacobian();      // Eigen::MatrixXd  ∂x'/∂x
d.controlJacobian();    // Eigen::MatrixXd  ∂x'/∂u
d.parameterJacobian();  // optional; present iff parameters registered

// Efficient reverse product (no framework dependency), named for the quantity:
StepGradient g = world.applyStepVjp(/*dLoss_dNextState=*/gradOut);
g.state; g.control; g.parameter;
```

The differentiable C++ types live in an optional module
`dart/simulation/diff/` compiled behind the `DART_BUILD_DIFF` option,
mirroring how `compute/` and CUDA are isolated. The reverse-pass cache, clamping
classification, `A_CC` factorization, and LCP snapshot live in `detail/` and never
appear in a public signature.

### Facade rules this API obeys

- No `Solver`, `Coupler`, `PhysicsDomain`, schedule-phase, ECS, component,
  backend, or tensor-framework type is public; the reverse cache stays in
  `detail/`.
- `differentiable=False` is the default with bitwise-identical, zero-allocation
  behavior.
- Method names are method/quantity names (`get_step_derivatives`, `applyStepVjp`,
  `timestep`, `rollout`, `add_differentiable_parameter`, `contact_gradient_mode`),
  never engine or framework verbs.
- New public symbols (`state_vector`, `num_efforts` on `World`;
  `PhysicalParameter`, `PhysicalParameterSelector`, `ContactGradientMode`; the
  derivative value objects) must be added to the dartpy committed-stub symbol
  tables in the Python-API doc as they land.

## Algorithm → DART Seam Mapping

The five attachment seams and what each contributes to the reverse pass. Seams
1–2 attach to the experimental boxed-LCP rigid-contact path selected by
`WorldOptions::contactSolverMethod = ContactSolverMethod::BoxedLcp`, not to the
default sequential-impulse PGS path. The implemented slice covers the
rigid-body normal/friction contact snapshot and contact-aware derivative path;
the legacy `dart/constraint/constraint_solver.cpp` is _not_ the seam.

| #   | Seam (file)                                                                                                                                                                                                                               | Forward output      | Reverse contribution                                                                |
| --- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------- | ----------------------------------------------------------------------------------- |
| 1   | LCP solve — `dart/math/lcp/lcp_solver.hpp` `LcpResult LcpSolver::solve(const LcpProblem&, VectorXd& x, const LcpOptions&)`; the current boxed-LCP contact slice uses a **pivoting (Dantzig-style)** solver that yields a clean active set | impulses `f`        | snapshot `{A,b,lo,hi,findex,f}`; classify C/S/T; build/factor `A_CC`; `∂f_C/∂{A,b}` |
| 2   | Experimental boxed-LCP contact stage — `detail/rigid_contact/boxed_lcp_contact.*` plus `detail/contact_jacobians.*`; opt-in through `WorldOptions::contactSolverMethod` while default `RigidBodyContactStage` remains sequential impulse  | `A,b` from `J, M⁻¹` | `∂A/∂q, ∂b/∂q` via contact Jacobian derivatives                                     |
| 3   | Articulated dynamics — `compute/multibody_dynamics.*` `computeMultibodyDynamicsTerms`                                                                                                                                                     | `M, C, g`           | `∂(M⁻¹(τ−c))/∂{q,q̇,τ,θ}` (substantive; see D1)                                      |
| 4   | Link Jacobian — `compute/multibody_dynamics.hpp` `computeMultibodyLinkJacobian`                                                                                                                                                           | `J`                 | `∂(Jᵀf)/∂q` (screw-axis derivative)                                                 |
| 5   | Manifold integration — `simulateMultibody` position/velocity update                                                                                                                                                                       | `x'`                | joint-type-keyed position Jacobians; assemble `state_jacobian`, `control_jacobian`  |

Iterative/projection LCP solvers (PGS, APGD) do not return a clean active set, so
the C/S/T classification threshold would be solver- and tolerance-dependent; the
boxed-LCP derivative path therefore stays tied to the pivoting contact solve.
Iterative-solver gradients remain out of scope unless they gain a documented
classification tolerance.

PLAN-080 WS4 satisfied the original PLAN-110 prerequisite by emitting
`{A,b,lo,hi,findex,f}` as first-class boxed-LCP contact outputs (a
snapshot-friendly, differentiation-ready seam). Remaining promotion work is to
extend that seam beyond the current rigid-body contact slice to articulated-link
and coupled contact cases, preserve the same snapshot contract, and reject
unsupported cases rather than silently returning contact-free gradients.

## Internal Architecture

`dart/simulation/diff/` (optional, `DART_BUILD_DIFF`) realizes the
solver-internal reverse pass:

- **Snapshot plumbing.** The data the reverse pass needs (`M`, `J`, the per-island
  LCP `{A,b,findex,f}`, the clamping classification, joint-type position-Jacobian
  inputs) is computed by the forward path and captured only for differentiable
  worlds. WS1 threaded a **nullable snapshot sink** through those forward
  functions, populated only when non-null. The `World` owns the snapshot buffer
  behind `WorldOptions::differentiable`; the step stages are stack-local and
  cannot hold per-step state, so ownership lives on the World/solver, not the
  stage.
- **`StepSnapshot`** (detail): per-step record for backward (lazily allocated only
  when differentiable).
- **`ContactGradient`** (detail): C/S/T partition and `∂f_C/∂{A,b}` (Nimble's
  `ConstrainedGroupGradientMatrices`), including the friction `findex` mapping.
- **`ContactJacobianDerivative`** (detail): `∂(Jᵀf)/∂q` screw-axis algebra.
- **`PhysicalParameterSelector`** (public value type): the with-respect-to
  selection (mass/COM/inertia/friction), with bounds.
- **`StepDerivatives` / `StepGradient`** (public value types): explicit Jacobian
  blocks and the reverse-product result, Eigen-typed.
- **`diff::rollout` / `diff::applyStepVjp`** (public functions).

The dartpy `sx.diff` submodule adds the `timestep`/`rollout` `autograd.Function`
wrappers and the dlpack/NumPy plumbing; it imports torch lazily.

Concurrency note: a differentiable `World` is more stateful (snapshot mutated in
forward, read in backward) and is not safe for concurrent step/backprop. The
multibody dynamics stage advertises task-parallel execution across multibodies,
so snapshot recording is **per-multibody/per-island** and lock-free, never a
shared sink. Batched/replicated differentiable worlds (deferred) need separate
snapshot ownership per world.

## Opt-In And Overhead Model

- `differentiable=false` (default): nullable sink is null; no `StepSnapshot`
  allocation, no extra compute; bitwise-identical results. `test_diff_zero_cost_parity`
  asserts bitwise-equal final `[q;q̇]` and zero snapshot allocations.
- `differentiable=true`: each step records the snapshot. Explicit Jacobians cost
  one `A_CC` factorization plus block assembly; the VJP avoids forming full
  matrices and exploits island block structure.
- **Memory / checkpointing.** Trajectory memory scales with stored snapshots
  (O(steps · ndof²) in the worst case). WS3 delivers a checkpointing knob
  (recompute vs. store) with a peak-memory test; `sx.state.rollout(...,
differentiable=True)` and the `traj.states` buffer interact with it. This is a
  first-class WS3 interface item, not a deferred knob.
- Overhead (on vs. off) is reported by a benchmark with a stated budget.

## Gradient Correctness And Verification

- **Exact** within a fixed contact mode and away from the non-smooth feature
  boundaries (joint limits, velocity limits, Coulomb joint friction, velocity
  actuators) for inelastic contact, linearized-cone friction, all joint types,
  and registered parameters.
- **Subgradient** at contact-mode switches, the Tied set, and the non-smooth
  feature boundaries above; documented, not hidden.
- **Approximate** for elastic/restitution contact (continuous-collision-time
  approximation); a later slice, labeled approximate.
- **Determinism.** The differentiable path fixes warm-start state and pivot/
  iteration policy so the clamping classification is a deterministic function of
  inputs; gradient outputs are reproducible to a documented tolerance (the
  `A_CC` COD+CFM solve is numerically sensitive).
- **Finite-difference gate (concrete defaults).** Central differences validate
  `state_jacobian`, `control_jacobian`, and `parameter_jacobian` with relative
  error `< 1e-4` over a perturbation sweep `h ∈ {1e-5, 1e-6, 1e-7}`, on a named
  scene set — single pendulum, double pendulum, free-floating body (WS1);
  sphere-on-ground, box-stack (WS2) — with Tied-set and active-boundary
  configurations explicitly excluded and logged. A feature's gap-audit row flips
  to PRESENT only when its FD check passes. Owner commands: `pixi run lint`,
  `pixi run build` (with `DART_BUILD_DIFF` on and off), the focused C++ FD/parity
  tests, `pixi run check-api-boundaries`, and `pixi run test-py` for the dartpy
  bridge.

## Phasing

Historical sequence; slice detail lives in the dev-task roadmap. WS1–WS5 and
the PLAN-080 boxed-LCP contact prerequisite now have implemented first slices in
the experimental stack. Keep this list as the dependency map that explains how
the work landed and what stable promotion must still harden.

0. **Prerequisite (PLAN-080 WS4)**: boxed-LCP contact + joint-limit solve on the
   DART 7 `World`, emitting `{A,b,lo,hi,findex,f}` as first-class outputs.
   Not owned here; gates seams 1–2.
1. **Opt-in seam + contact-free Jacobians** — `WorldOptions::differentiable`
   (default false), nullable-sink plumbing, the chosen smooth-term derivative
   mechanism (D1), joint-type-keyed position Jacobians, `state_jacobian`/
   `control_jacobian` for the contact-free path, the FD checker, and the
   zero-cost parity test. Enumerate which joint types ship in WS1.
2. **Analytic contact gradient** — C/S/T classification, `A_CC` solve, `∂f_C/∂{A,b}`,
   friction `findex` mapping, `∂(Jᵀf)/∂q`; pin the `findex`/cone convention.
3. **Reverse product + PyTorch bridge** — `applyStepVjp`, the `sx.diff` submodule,
   `state_vector`/`num_efforts` helpers, the checkpointing knob, dartpy stubs +
   torch-absent import test + Python tests, and a trajectory-optimization example.
4. **Parameter derivatives** — `PhysicalParameterSelector`, `parameter_jacobian`,
   a system-identification example.
5. **Refinement opt-in modes (split, separately gated)** — (5a) elastic/restitution
   (approximate; FD-checked where defined); (5b) complementarity-aware saddle
   escape (no FD gate; gated by a documented optimization-convergence benchmark);
   (5c) pre-contact surrogate (no FD gate; gated by a documented trajectory-opt
   task). Each labeled honestly.
6. **Dojo-style solver evaluation (planned)** — audit and spike a separate
   maximal-coordinate variational NCP/IPM solver with implicit gradients,
   central-path smoothness control, and comparison examples. This starts as
   internal-only evidence; public API waits for the PLAN-110 split/promotion
   decision.

## Stable Promotion Contract

Promotes to stable API only after the promoted subset has: public wrapper value
types for state/control/parameter derivatives with no `detail`/ECS/backend
includes; finite-difference-checked correctness on the named scenes; focused C++
and Python tests; a named build option (`DART_BUILD_DIFF`) with CI covering on
_and_ off configurations and a workflow guard so the option cannot be silently
dropped; serialization round-trip (or documented non-serialization) of
`WorldOptions::differentiable`, `contact_gradient_mode`, and parameter
registrations; ownership/freshness/unsupported-case docs; dartpy bindings with
committed stubs; a trajectory-optimization and a system-identification example
with runnable commands; changelog and migration notes; and a tracked overhead
benchmark. The zero-cost `differentiable=false` default is part of the contract.

Current experimental status: binary save/load preserves the World-level
differentiability/contact policy flags, and replay restores registered
differentiable parameters. Binary serialization of parameter registrations is
not implemented yet; stable promotion must either add that round-trip or
document runtime re-registration as the supported restart contract.

## Deferred Capabilities

- GPU/accelerator differentiable kernels (revisit at the PLAN-030 Phase 6 gate).
- Batched differentiable rollouts beyond a documented homogeneous
  leading-dimension contract.
- Autodiff-scalar templating as the _contact_-gradient mechanism (kept available
  for smooth terms per D1; not the contact path).
- Differentiable deformable/IPC contact (coordinate with PLAN-081).
- World-space coordinate mapping (`IKMapping`-style) for differentiable task-space
  objectives (a later utility slice).
- A public third-party custom-gradient plugin API (needs the deferred ABI/GIL
  design).
- Public Dojo-style maximal-coordinate/NCP/IPM solver API until the PLAN-110
  Dojo gap audit and internal spike prove the path.

## Design Rationale

- Analytic LCP gradients are DART's genuine research advantage: exact hard-contact
  gradients where the JAX/Warp/Taichi engines rely on soft contact and inherit its
  gradient pathologies.
- Opt-in respects the stated overhead requirement and matches the engines (Newton,
  Genesis) that converged on the same Model/State/Control/Contacts structure.
- A framework-neutral core with a thin lazy PyTorch `autograd.Function` bridge
  gives the lowest-friction Python UX without a core tensor dependency.
- Exposing explicit Jacobians _and_ an efficient reverse product serves both
  optimal-control and learning users and beats reverse-only tape engines.
- A finite-difference gate plus honest correctness documentation is how DART earns
  research trust where every surveyed engine has undocumented failure modes.
- Treating it as the rigid-body solver's reverse pass keeps the forward fast path
  and the public object model unchanged.
- Stating the manifold position Jacobian and the not-yet-LCP contact path
  accurately keeps the design from encoding two known-false simplifications into
  WS1/WS2 implementations and FD-gate failures.

## Verification Expectations

Docs-only edits use the docs-only gate set from `docs/ai/verification.md`.

Implementation PRs should include: `pixi run lint`; `pixi run build` with
`DART_BUILD_DIFF` on and off; focused C++ tests under
`tests/unit/simulation/` (derivatives, FD checks, the zero-cost
parity test, error handling, lifetime); `pixi run check-api-boundaries` when
public headers or dartpy bindings change; `pixi run test-py` plus committed stub
updates and a torch-absent import test when the bridge changes; benchmark evidence
for the overhead claim; and changelog/migration notes when a promoted surface
changes.

Reviewers should reject any user-facing API that leaks the reverse-pass cache, the
LCP snapshot, solver/coupler/backend types, ECS storage, or a tensor framework
into the C++ core.

## References

- Werling, Omens, Lee, Exarchos, Liu. "Fast and Feature-Complete Differentiable
  Physics for Articulated Rigid Bodies." RSS 2021. arXiv:2103.16021. Catalog id
  `werling-2021`.
- Tan, Siu & Liu. "Contact Handling for Articulated Rigid Bodies Using LCP."
  Catalog id `tan-lcp` — the LCP formulation this method differentiates.
- Cross-engine survey (Brax arXiv:2106.13281; Newton/Warp; Genesis; MuJoCo MJX;
  Dojo arXiv:2203.00806; contact-gradient correctness arXiv:2207.05060;
  TOI-velocity arXiv:2305.00092; hard-contacts-soft-gradients arXiv:2506.14186)
  — notes and sources in the PLAN-110 gap audits.
