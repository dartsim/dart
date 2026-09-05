# Resume: Rigid-Body Dynamics Solver

## Current Routing (2026-09-04)

Read [PLAN-040](../../plans/040-dart7-release-hardening.md),
[PLAN-080](../../plans/080-rigid-body-dynamics-solver.md) and the
[dashboard](../../plans/dashboard.md) before selecting work. Start with
WP-040.1; WP-080.1 requires accepted WP-040.1 and WP-030.3. The historical
contact-polish sequence below cannot bypass those prerequisites.

## Implementation Snapshot (2026-09-01, Historical)

The rigid-body MVP (PR #2705), the model-loading bridge and unified contact
line (PR #2838), and the `Locked` (PR #3251), `Servo` (PR #3258), and
`Acceleration` (PR #3276) actuator modes are all on `main`; no feature branch
is open. The B2 rigid open-chain parity harness was removed with the World
promotion in PR #2932 and has no replacement yet; because `main` may not carry
classic-World references, earlier work used `release-6.*` comparisons. Current
physical acceptance comes from PLAN-040's independent oracles; existing checker
requirements remain enforced until WP-040.2 migrates them. The contact-solver
plan below records historical implementation detail, not current routing.

- **Open baseline to beat (measured 2026-09-02 on `main`):** a 0.1 m
  half-extent box (mass 0.5, restitution 0, `dt` 0.005) dropped 0.3 m onto a
  static box settles to a 0.8-2 mm support gap within 0.5 s but from 1 s to
  6 s keeps |v| ≈ 5e-3-1.3e-2 m/s and |ω| ≈ 4e-2-1e-1 rad/s with three active
  contacts out of the four-point face manifold, and never reaches the 1e-3
  deactivation thresholds even with `DeactivationOptions.enabled`. Cause: the
  rank-deficient near-coplanar fallback in the coupled boxed-LCP increment.
  The warm-starting/manifold polish must beat this; until then grade rest
  claims with an explicit gap/drift criterion
  (`docs/onboarding/agent-sim-verification.md`).

## Historical Contact-Solver Plan

This snapshot explains earlier contact slices. Re-audit any retained detail
against current source before using it in a packet admitted by PLAN-080.

**Status: steps 1-4 below are DONE** (on `main`) — `CollisionBody`
handle, link collision shapes, links in `World::collide()`, and the rigid-body
stage skipping link pairs. The remaining work is **step 5 (articulated contact
response)** and beyond.

1. **(DONE) Generalize the contact body reference.** `Contact`
   (`body/contact.hpp`) now holds a `CollisionBody` handle
   (`body/collision_body.{hpp,cpp}`) with
   `getEntity`/`getWorld`/`getName`/`isRigidBody`/`isLink`/`asRigidBody`/
   `asLink`. dartpy `Contact.body_a`/`body_b` return `CollisionBody`.
2. **(DONE) Link collision shapes.** `Link::setCollisionShape`/
   `getCollisionShape`/`hasCollisionShape` store `comps::CollisionGeometry` on
   the link entity (dartpy `link.set_collision_shape`/`collision_shape`/
   `has_collision_shape`).
3. **(DONE) `collide()` includes links.** `World::collide()` now has a second
   pass over `CollisionGeometry + comps::Link + comps::FrameCache`, posing each
   link by `FrameCache.worldTransform`.
4. **(DONE) Rigid-body contact stage skips link pairs.** `RigidBodyContactStage`
   guards each contact with `registry.all_of<comps::RigidBodyTag>` for both
   bodies; link contacts are left for the articulated contact solve.
5. **(DONE for link-vs-static) Articulated contact response.** Implemented for
   link-vs-static-rigid-body contacts (one-sided) inside `simulateMultibody`'s
   velocity phase (`LinkContact`, contact-point normal Jacobian, unilateral
   normal impulse + Baumgarte), routed by `MultibodyForwardDynamicsStage::execute`
   via `world.collide()`. **Two-tangent Coulomb friction** (accumulated-impulse,
   friction-cone bounded) and **per-contact restitution** are also implemented.
   Verified by a prismatic-leg drop-and-rest, a sliding-link friction brake, and
   a dropped-link bounce (C++ + dartpy). **Still pending:** link-vs-dynamic-body
   and link-vs-link (two-sided) contacts, and a boxed-LCP for coupled
   simultaneous contacts. The original plan, for reference:
   - In `MultibodyForwardDynamicsStage::execute`, call `world.collide()` once and
     route to each multibody the contacts whose link belongs to it and whose
     other body `isRigidBody()` and is static (`comps::StaticBodyTag`). Pass them
     into `simulateMultibody`.
   - Map the contact's link entity to its index in the `DynamicsTree`; get the
     body Jacobian from `linkBodyJacobians(tree)` and the world Jacobian via the
     link world rotation; shift the linear block to the contact point `p`:
     `J_point_linear = R*J_body_linear - skew(p - o)*R*J_body_angular`, `o` =
     link world origin (`tree.links[i].worldTransform.translation()`).
   - Normal row `Jn = n^T J_point_linear` (1 x dof). **Sign care:** orient `n`
     to point from the obstacle into the link (the contact normal points
     bodyA->bodyB; flip if the link is bodyA). Normal velocity `vn = Jn*nextVel`.
   - If `vn < 0` (approaching) apply `lambda_n = max(0, -m_eff*vn)` with
     `m_eff = 1/(Jn M^-1 Jn^T)` (reuse `mb.massMatrix.inverse()` /
     `J M^-1 J^T`); `nextVelocity += M^-1 Jn^T lambda_n`. Add a small Baumgarte
     bias `+ beta*depth/dt` to stop sinking. Iterate a few times for multiple
     contacts.
   - Later: restitution/friction (tangent rows + cone), then the boxed-LCP for
     simultaneous coupled contacts (`dart/math/lcp`, PLAN-020) and islands, and
     link-vs-dynamic-body / link-vs-link (two-sided) contacts.
6. **Verify by emergent behavior** (the only non-closed-form tests in the task,
   so budget time to tune tolerances): a fixed-base 1-DOF prismatic "leg" with a
   sphere shape descends under gravity and rests on a static rigid ground
   (penetration stops, normal velocity -> 0).

The old branch-publication guidance is obsolete: the model-loading and unified
contact/constraint line is already on `main` via PR #2838. Future follow-ups
should start from current `main`, keep PRs scoped to one polish slice, and use
the normal `DART 7.0` PR milestone and validation gates.

## Context That Would Be Lost

- Current domain/representation, solver/coupling and state ownership lives in
  `docs/design/simulation_solver_architecture.md`. Keep that the source of
  truth; do not duplicate it.
- Use method/approach names for DART solvers, presets and examples. The current
  [research scope decision](../../design/simulation_solver_architecture.md#verified-engine-lessons-and-dart-inferences)
  permits named, cited engine comparisons and supersedes the earlier blanket
  documentation restriction. Do not modify the maintainer's external
  multiphysics API design notes.
- The legacy DART 6 dynamics live in `dart/dynamics/` (Featherstone ABA in
  `detail/articulated_dynamics_algorithms.hpp`, `Skeleton::computeForwardDynamics`)
  and `dart/constraint/` (`BoxedLcpConstraintSolver`, `ContactConstraint`). The
  boxed-LCP library is `dart/math/lcp/` (Dantzig/PGS), already contracted under
  PLAN-020.
- The maintained native collision engine is `dart/collision/native/`
  (PLAN-035/036/037); Phase 2 bridges it into the experimental world.
- Existing experimental tests assume zero gravity — changing the default is a
  deliberate, documented decision (matches DART 6 and the user vision).
- PLAN-041/042 own facade shape; the former `simulation_api_design` task is
  retired. This task retains dynamics implementation detail.

## How To Resume

```bash
cd <your DART checkout>
git status -sb && git log -3 --oneline
```

Then read, in order:

```bash
docs/dev_tasks/rigid_body_dynamics_solver/README.md
docs/plans/040-dart7-release-hardening.md
docs/plans/080-rigid-body-dynamics-solver.md
docs/plans/030-compute-scalability-roadmap.md
docs/plans/dashboard.md
docs/design/simulation_solver_architecture.md
docs/design/simulation_cpp_api.md
```

Then claim a ready packet through the current prerequisite graph. Consult
`01-gap-analysis.md`, `02-roadmap.md` and the contact plan above only as
historical evidence after the owning plans.
