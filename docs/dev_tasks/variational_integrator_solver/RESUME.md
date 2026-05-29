# Resume: Variational Integrator Solver (PLAN-082)

## Last Session Summary

PLAN-082 (plan + design doc + contact-roadmap sidecar + dashboard + catalog
entries) landed after a 4-agent adversarial review. **Phase A1 core is
implemented and verified**: SE(3) discrete-mechanics kernels (delegating to
`dart::math::lie_group`), the DRNEA residual recursion, the RIQN root-find with
a dense `M⁻¹` placeholder, two-step history with bootstrap, an articulated
energy helper, and the `MultibodyVariationalIntegrationStage`. The reference
review of the paper's bibliography is folded into the catalog and design doc.

## Current Branch

`feature/variational-integrator-mvp` — Phase A1 core committed (kernels +
integrator + 3 test suites) plus the PLAN-082 docs.

## Verified (all green)

- `test_discrete_mechanics_math`, `test_discrete_mechanics_lie_group_parity`
  (kernels == `dart::math::lie_group`; `dexp⁻¹ ≡ SE3::LeftJacobianInverse`),
  `test_variational_integration` (prismatic free-fall accel = −9.81 to 1e-9;
  pendulum first-step accel matches Newtonian; RIQN converges; **energy
  conserved over 1e5 steps**; ABI inverse-mass == dense `M⁻¹` to 1e-9; floating
  free-fall + tumbling energy conservation; **`MaintainsDistanceLoopClosure`** —
  a Distance loop closure holds to 1e-6 while the 1-DOF arm swings). Full
  experimental suite green.

## Immediate Next Step

The selector and the **O(n) ABI** (Phase A2 core) have landed:
`computeMultibodyInverseMassProduct` (Featherstone ABA: backward
articulated-inertia sweep + forward acceleration sweep) drives RIQN and matches
the dense `M⁻¹` to 1e-9 (`ArticulatedInverseMassMatchesDenseSolve`); the
integrator is linear-time, symplectic (energy-conserving over 1e5 steps), and
selectable via `World::setMultibodyOptions({.integrationFamily = "variational integrator"})`.
All committed PLAN-082 phases (A1, A2, B1, B2), every acceptance gate, the
paper-experiment replication, and the post-plan follow-ups are complete and
verified: A1's symplectic headline / momentum / non-convergence-error /
determinism / **serialization**; A2's O(n) ABI + scaling benchmark + the
**Anderson + `√n`-tolerance long-chain convergence fix**; B1's manifold
finite-difference gate; B2's **Point/Distance/Rigid loop closures through the
public API** (constraint Jacobians incl. the Rigid orientation Jacobian
FD-verified; semi-implicit still rejects `Solve`); the **dartpy** integration-
family selector; and the **improvements/corrections/new-features** write-up in
the design doc. Phase C is a recorded NO-GO. What remains:

1. **Phase C**: contact/friction — recorded **NO-GO** (deferred, go/no-go; see
   the plan sidecar).
2. **A2 extreme chains (≥100 links)**: the exact recursive-Jacobian
   preconditioner (paper Appendix) for chains needing a budget above the default.
3. **Dedup**: hoist the local spatial-algebra/kinematic-tree helpers shared with
   `multibody_dynamics.cpp` into an internal header.

## Context That Would Be Lost

- The dense `M⁻¹` (via `computeMultibodyDynamicsTerms`) is a deliberate Phase A1
  **placeholder** — O(n³), NOT linear-time. Do NOT claim O(n) until Phase A2's
  ABI lands. The energy/analytic correctness is independent of this.
- `dexp⁻¹` delegates to the exact `SE3::LeftJacobianInverse` (more accurate than
  the paper's Bernoulli truncation); `discrete_mechanics_math.hpp` keeps the
  VI-specific wrappers (`dexpInvTranspose`, `dAdT`/`dAdInvT`, `adInvRLinear`),
  all cross-checked against `dart::math::lie_group` by the parity test.
- Gravity is forcing-side (per-body spatial impulse), not a potential — never
  add it both ways.
- Phase A1 is fixed-base, revolute/prismatic only (Euclidean). Ball/free joints
  need manifold-correct RIQN retraction (Phase B1; the reference impl has open
  TODOs there). Floating base + loop closures are Phase B; contact is Phase C
  (deferred, go/no-go).
- The `MultibodyVariationalState` is currently a plain (non-serialized) ECS
  component; the local Phase-A1 spatial helpers in `variational_integration.cpp`
  duplicate a subset of `multibody_dynamics.cpp` (documented dedup follow-up).

## How to Resume

```bash
git checkout feature/variational-integrator-mvp
git status && git log -5 --oneline
# build + run just the VI tests:
pixi run bash -lc 'cmake --build build/default/cpp/Release --target test_variational_integration -j && ctest --test-dir build/default/cpp/Release -R variational_integration --output-on-failure'
```
