# Differentiable Simulation — Slice Roadmap

Slice-level breakdown of the PLAN-110 workstreams. Each slice is one bounded,
independently verifiable change with a finite-difference (FD) or parity gate.
Code is the source of truth; this file tracks intent and sequencing only.

## WS0 — Prerequisite (owned by PLAN-080)

- **0.1** Boxed-LCP contact + joint-limit solve on the experimental `World`
  (replaces/augments `RigidBodyContactStage` with an `LcpProblem`-based solve).
  Gates WS2. Track under `docs/dev_tasks/rigid_body_dynamics_solver/`.

## WS1 — Opt-in seam + contact-free Jacobians (unblocked)

- **1.0 (decision, do first)** Choose and record the smooth-term derivative
  mechanism for `∂M/∂q`, `∂c/∂q`, `∂c/∂q̇`: (a) analytic spatial-algebra recursion
  (differentiated Featherstone — most work, most accurate/fast), (b) the reserved
  scalar-generic autodiff path, or (c) FD-of-dynamics as an interim. Record the
  choice and the joint-type scope of WS1 here. This is real work, not bookkeeping.
- **1.1** New `WorldOptions` value object with `differentiable` (default false) +
  `World(WorldOptions)` constructor (none exists today; passes `check-api-boundaries`);
  new optional module `dart/simulation/experimental/diff/` behind `DART_BUILD_DIFF`;
  CI builds with the option on and off.
- **1.2** Nullable-sink snapshot plumbing threaded through `simulateMultibody`/
  `computeMassAndBias` (populated only when non-null); World-owned `StepSnapshot`
  buffer; per-multibody/island recording (no shared sink under the task-parallel
  executor).
- **1.3** Zero-cost-when-off: `test_diff_zero_cost_parity` — bitwise-identical
  final `[q;q̇]` and zero snapshot allocations when off; `bm_diff_step_overhead`
  on/off benchmark vs a stated budget.
- **1.4** Joint-type-keyed **position Jacobians**: Euclidean `I`/`Δt·I`; SO(3)/ball
  `dexp`/`dlog`; SE(3)/free `dexp`/`dlog` + translation-orientation coupling
  (free forward dynamics is already implemented in `multibody_dynamics.cpp`, so
  free is in WS1 scope; only the SE(3) position Jacobian is new).
- **1.5** `state_jacobian` / `control_jacobian` for the contact-free path: the
  smooth velocity blocks (via the 1.0 mechanism) composed with the 1.4 position
  blocks.
- **1.6** FD checker utility (central differences, rel err `< 1e-4` over
  `h ∈ {1e-5,1e-6,1e-7}`); `test_diff_fd_smooth` on single/double pendulum + free
  body across shipped joint types, with non-smooth-feature boundaries excluded.
  → rows flip PRESENT.

## WS2 — Analytic contact gradient (BLOCKED on WS0)

- **2.1** Capture LCP snapshot `{A,b,lo,hi,findex,f}` post-solve at the **future
  experimental** boxed-LCP contact stage created by WS0 (target a pivoting /
  Dantzig-style solver for a clean active set; not the legacy
  `dart/constraint/constraint_solver.cpp`). Detail-only; no public leak.
- **2.2** Clamping/Separating/Tied classification (threshold) → `A_CC`.
- **2.3** `A_CC⁻¹` via complete-orthogonal-decomposition + CFM diagonal;
  `∂f_C/∂{A,b}`.
- **2.4** Friction rows via `findex` upper-bound mapping; friction-coefficient
  entry point reserved for WS4.
- **2.5** `∂(Jᵀf)/∂q` screw-axis derivative (`ContactJacobianDerivative`).
- **2.6** Compose full `state_jacobian`/`control_jacobian` with contact; FD-check
  on sphere/box/capsule contact scenes. → rows flip PRESENT.

## WS3 — Reverse product + PyTorch bridge (BLOCKED on WS2)

- **3.1** `World::applyStepVjp(dL/dx')` → `{state, control, parameter}` VJP
  (framework-neutral, Eigen; quantity name, not a framework verb); exploits
  per-island block structure rather than forming the dense Jacobian.
- **3.2** `world.state_vector` (property, `[q;q̇]`) + `world.num_efforts` helpers
  over `StateSpace`; matching C++ accessors; add all new symbols to the dartpy
  committed-stub symbol tables.
- **3.3** Checkpointing knob (recompute vs. store) with `bm_diff_rollout_memory`
  peak-memory test; interacts with `sx.state.rollout(..., differentiable=True)`
  and `traj.states`.
- **3.4** dartpy `sx.diff` submodule: `timestep`/`rollout` `torch.autograd.Function`
  wrappers (lazy torch import; `sx.diff` exists with torch absent and raises a
  guiding `ImportError` when called); extend `sx.state.rollout` with
  `differentiable=True` + `traj.gradients(...)`; committed stubs.
- **3.5** `test_diff_zero_torch_import` (base import works without torch) +
  `gradcheck`-style Python tests vs FD + a runnable trajectory-optimization
  example (e.g. throw-to-target / hopper control).

## WS4 — Parameter derivatives (BLOCKED on WS2)

- **4.1** `PhysicalParameterSelector` value type (mass/COM/inertia/friction,
  with bounds); `world.add_differentiable_parameter(...)` registration API.
- **4.2** `parameter_jacobian` (∂x'/∂θ) through `M`/`A`/`findex` terms; FD-check.
- **4.3** System-identification example (recover mass/friction from a trajectory).

## WS5 — Refinement opt-in modes (split, each separately gated)

These are three independent efforts with different verification stories; do not
treat "labeled approximate" as a delivery gate.

- **5a** Elastic/restitution gradient (approximate continuous-collision model);
  labeled approximate; **gate**: FD-check where the gradient is defined.
- **5b** `COMPLEMENTARITY_AWARE` saddle-escape mode (heuristic; returns a non-true
  gradient by construction → **no FD gate**); **gate**: a documented
  optimization-convergence benchmark on a named task that stalls under the plain
  analytic gradient.
- **5c** `PRE_CONTACT_SURROGATE` backward-only pre-contact gradient; forward sim
  unchanged; non-true gradient → **no FD gate**; **gate**: a documented
  trajectory-optimization task where the plain gradient is zero before contact.

## Promotion checkpoint (DART 8)

After WS1–WS4 are PRESENT and FD-checked, evaluate the
[DART 8 promotion contract](../../design/differentiable_simulation.md#dart-8-promotion-contract):
public wrapper types, no detail/ECS/backend includes, tests + stubs, two
worked examples, and a tracked overhead benchmark.
