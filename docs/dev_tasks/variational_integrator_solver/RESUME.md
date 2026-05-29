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
  conserved over 1e5 steps**). Full experimental suite 24/24.

## Immediate Next Step

The selector and the **O(n) ABI** (Phase A2 core) have landed:
`computeMultibodyInverseMassProduct` (Featherstone ABA: backward
articulated-inertia sweep + forward acceleration sweep) drives RIQN and matches
the dense `M⁻¹` to 1e-9 (`ArticulatedInverseMassMatchesDenseSolve`); the
integrator is linear-time, symplectic (energy-conserving over 1e5 steps), and
selectable via `World::setMultibodyIntegrationMethod("variational integrator")`.
Remaining, in priority order:

1. **A2 large-chain convergence** (follow-up): `bm_variational_integration`
   confirms the inverse-mass kernel is O(n) (3% RMS) and the integrator scales
   linearly for ≤~32 DOF, but the RIQN iteration count rises for ≥~64-link
   chains. The IG3 (semi-implicit) initial guess is implemented and did not fix
   the cliff (it is a convergence-_rate_ issue, not a starting-point one), so the
   open mitigations are a relative/scaled convergence tolerance, line-search or
   Anderson acceleration, or the exact recursive-Jacobian preconditioner.
2. **A1 finish**: serialize `MultibodyVariationalState` (binary-format version
   bump + bootstrap-done flag) + a save/load determinism round-trip test;
   optionally promote RIQN non-convergence from a `converged=false` diagnostic to
   a documented error.
3. **Phase B1**: floating base — DONE. Floating + Spherical joints with
   manifold-correct SE(3)/SO(3) RIQN retraction (`jointRetract`/
   `jointLogDifference`), verified by free-fall + tumbling energy conservation.
4. **Phase B2**: holonomic constraints (loop closures) — a constraint Jacobian
   `∂g/∂q` + impulse-based constraint solve folded into RIQN; flip the
   loop-closure `Solve` validation. (`johnson-murphey-2009`/`leyendecker-2008`.)
5. **Phase C**: contact/friction (deferred, go/no-go; see the plan sidecar).

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
