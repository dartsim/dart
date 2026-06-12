# Design: Contact-Aware Inverse Dynamics (DART 6.19)

## Problem

`Skeleton::computeInverseDynamics()` returns generalized forces for a desired
acceleration, but for floating-base skeletons in contact the first 6 (root)
generalized forces are not actuatable: they must be realized by contact forces
inside friction cones. DART offers no API to split the result into feasible
joint torques + contact forces, so users bolt on external QP solvers.

## Do we need a QP solver? (explicit gate)

No. Parameterizing each contact force as a nonnegative combination of
linearized friction-cone edge generators `d_j = normalize(n + mu*t_j)` turns
the wrench-distribution problem into

```
min_beta ||A*beta - b||^2 + lambda*||beta||^2,   beta >= 0
```

where columns of `A` are the unactuated-row entries of `J_k^T d_kj` and `b` is
the unactuated rows of the full inverse-dynamics force vector. This is a
nonnegative least squares (NNLS) problem solved exactly by the Lawson–Hanson
active-set method (small: 6 x (numBasis * numContacts)). Cone feasibility holds
by construction; no general inequality-constrained QP is required.

Alternatives rejected:

- General QP (new dependency or large in-house solver): unnecessary surface for
  an LTS branch; NNLS is the minimal exact solver for this objective.
- Boxed LCP (Dantzig/PGS, already in-tree): solves complementarity systems with
  PD-ish matrices, wrong shape for a (possibly rank-deficient) least-squares
  objective; would need Gram-matrix formation + regularization anyway.
- Optional IPOPT/NLOPT: cannot be required by a core 6.x feature.

## New API (additive only)

### `dart::math` NNLS utility (new header, PascalCase per 6.x)

```cpp
/// Lawson–Hanson active-set NNLS: min ||A x - b||^2 s.t. x >= 0.
/// Returns true on convergence. tolerance <= 0 picks an automatic value.
bool solveNonNegativeLeastSquares(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    double tolerance = -1.0,
    std::size_t maxIterations = 0);  // 0 => 3*cols heuristic
```

### `dart::dynamics::ContactInverseDynamics` (new class)

- `struct Contact { BodyNode* bodyNode; Eigen::Vector3d localOffset; Eigen::Vector3d normal; double frictionCoeff; std::size_t numBasis = 4; }`
  (normal in world frame, pointing from environment into the body)
- `struct Result { Eigen::VectorXd jointForces; std::vector<Eigen::Vector3d> contactForces; Eigen::VectorXd unactuatedResidual; bool feasible; }`
  `jointForces = tau_full - sum_k J_k^T f_k`; its unactuated rows equal the
  residual. Caller applies via `skel->setForces(result.jointForces)`.
- Construction over `SkeletonPtr`; setters: contacts, regularization
  (default 1e-8), residual tolerance (default 1e-6, scaled by max(1, |b|_inf)),
  optional explicit unactuated DOF list (default: DOFs of every tree root joint
  with 6 DOFs, i.e. FreeJoint floating bases).
- `Result compute(bool withExternalForces = false, bool withDampingForces = false, bool withSpringForces = false)`
  — uses the skeleton's current `q`, `dq`, `ddq` (same semantics as
  `computeInverseDynamics`); side-effect free (joint forces saved/restored).
- Degenerate cases: no contacts and/or no unactuated DOFs reduce to plain
  inverse dynamics with residual reporting; `frictionCoeff == 0` uses the
  normal direction as the single generator.
- Not thread-safe (reusable internal workspace to avoid per-call allocation).

### Existing APIs reused, not duplicated

`Joint::getBodyConstraintWrench()` / `BodyNode::getBodyForce()` already expose
the transmitted/structural wrench that explains "nonzero torques on
non-actuated axes" for 1-DOF joints; gaps there are documentation/dartpy
bindings, not C++ API.

## Validation plan (TDD)

Unit tests `tests/unit/math/test_NonNegativeLeastSquares.cpp`:
identity/diagonal cases, interior solution matches unconstrained least squares,
active-bound cases vs brute-force active-set enumeration (small dims),
rank-deficient + regularized, zero/empty inputs, determinism.

Unit tests `tests/unit/dynamics/test_ContactInverseDynamics.cpp`:

1. Fixed-base arm, no contacts == `computeInverseDynamics` output.
2. Static floating box on 4 contacts: contact forces sum to weight, residual
   ~0, symmetric distribution, forces inside cones.
3. Friction feasibility: requested horizontal acceleration below mu*g feasible;
   above mu*g infeasible with nonzero residual and `feasible == false`.
4. Forward-dynamics round trip (key test): set computed joint torques
   (actuated rows), apply contact forces as external body forces at the contact
   points, `computeForwardDynamics`, recovered accelerations match the desired
   ones (1e-5 tolerance class), on random poses of a multi-DOF floating-base
   model.
5. Input validation: foreign BodyNode, zero normal, negative friction handled
   with `dterr` + `feasible == false` (no crash).

Benchmarks `tests/benchmark/integration/bm_inverse_dynamics.cpp`:
plain `computeInverseDynamics` on serial chains (10/20/40 links) and
`ContactInverseDynamics::compute` vs number of contacts (2/4/8) on a
floating-base model; microsecond units; used to keep the solver
allocation-free in steady state.

GUI example `examples/contact_inverse_dynamics/`: biped (`fullbody1.skel`)
squat cycle generated with DART IK (feet pinned, pelvis target, balance
within support polygon), kinematically tracked; per-frame contact-aware ID
computes leg torques + per-contact ground reaction forces; OSG renders force
arrows (ArrowShape) + SupportPolygonVisual; ImGui shows torque/force plots,
friction-coefficient slider, residual/feasibility readouts.

dartpy: bind `ContactInverseDynamics` (+`Joint::getBodyConstraintWrench` if
unbound); pytest `python/tests/unit/dynamics/test_contact_inverse_dynamics.py`
mirroring tests 1–4 in reduced form.

## Compatibility

Additive-only; no changes to existing classes/signatures/behavior; no new
dependencies; gz-physics unaffected. CHANGELOG entry under 6.19.0 "Dynamics".
