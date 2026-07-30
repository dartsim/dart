# M2.0 — ABI-safe FEM integration seam (de-risking findings)

Purpose: answer the make-or-break question behind the branch-strategy decision —
*can a volumetric-FEM deformable subsystem be integrated into DART 6's step loop
as ABI-safe additive API on `release-6.20`, with per-step internal-DOF
integration, coupling to rigid/skeleton bodies, zero pure-rigid overhead, and
preserved rigid determinism, without changing any existing public class layout
or vtable?* Plan reference: `10-full-parity-execution-plan.md` §7, §11 step 2.

## Verdict

**Feasible.** DART 6 exposes an ABI-safe per-step hook and additive extension
points sufficient to host a new deformable subsystem without touching existing
layouts. There is one real design constraint on two-way coupling (below) that is
solvable within the same ABI-safe surface. The additive-on-`release-6.20`
direction is therefore viable; no evidence forces a clean-break line for the
*integration mechanism* (model-authoring scale is a separate question tracked in
the plan).

## The seam (code-verified)

DART has **no FEM code today** (greenfield). The extension surface:

- **Per-step hook = `constraint::ConstraintBase::update()`.** A custom constraint
  registered via `ConstraintSolver::addConstraint()` is stored in
  `mManualConstraints` and its `update()` is called **unconditionally every
  `solve()`** — `ConstraintSolver.cpp:1077`, *before* the `isActive()` LCP gate
  at `:1079`. Returning `isActive()==false` keeps it entirely out of the LCP
  (its `getInformation`/`applyImpulse`/… are never called), so `update()` is a
  clean per-step integration callback. `solve()` is called once per
  `World::step()` (`World.cpp:1327`).
- **Additive geometry/state**: a new `dynamics::Shape` subclass (runtime-string
  `getType()`, additive per-detector support) for the tet volume + embedded
  surface, and a custom `Node` / `EmbeddedStateAndPropertiesAspect` on the owning
  `BodyNode` for FEM node state / rest pose / material / reduced basis. All
  additive; no existing layout changes.
- **Custom solver install** is available (`World::setConstraintSolver(
  UniqueConstraintSolverPtr)`, `World.hpp:340`) if a subclassed solver is ever
  preferred — but note `ConstraintSolver::solve()` is **non-virtual**
  (`ConstraintSolver.hpp:255`), so a subclass cannot inject a system pass into
  `solve()`; the `ConstraintBase` hook is the clean route.

## The one real constraint: two-way FEM→rigid coupling

`update()` runs **mid-step**, during `solve()` — i.e. *after*
`computeForwardDynamics`+`integrateVelocities` and *before* `integratePositions`
(`World.cpp:1313-1373`). External forces are cleared at **end of step**
(`clearExternalForces`, `World.cpp:1378`). So a reaction applied via
`BodyNode::addExtForce()` from inside `update()` is cleared before the next
step's `computeForwardDynamics` consumes it → **mid-solve `addExtForce` does not
couple** (it is a no-op for the rigid body).

Two-way coupling must therefore use one of:

1. **LCP participation** — the FEM coupling is a real `ConstraintBase` with
   `isActive()==true` and implemented velocity-level virtuals
   (`getInformation`/`applyUnitImpulse`/`getVelocityChange`/`applyImpulse`), so
   the FEM↔rigid exchange happens inside the constraint solve. Tighter, more
   work; fully ABI-safe (same extension point).
2. **Caller-/scene-driven co-simulation** — advance the FEM and apply coupling
   forces at the `World::step()` boundary (before `step()`), so
   `computeForwardDynamics` consumes them. Simplest; explicit 1-step lag; still
   ABI-safe (public API only), but it is caller responsibility rather than a
   DART-integrated pass.

**One-way coupling** (Kim/Pollard one-way mode: skeleton drives the embedded
FEM) is unaffected — `update()` reads body transforms and advances FEM DOFs with
no force applied back to the rigid body. Clean and immediate.

## Zero rigid overhead & determinism (structural)

For a pure-rigid world no FEM constraint is registered, so `mManualConstraints`
is empty and the `:1076` loop iterates zero times — **zero added per-step cost**.
An inactive FEM constraint (`isActive()==false`) never enters `mActiveConstraints`
(`:1079-1082`), so it never perturbs the LCP or the rigid solve → **rigid
determinism is preserved**. This mirrors how the existing soft-body loops cost
nothing when empty.

## ABI-safety

All extension points are subclassing / additive registration of existing public
interfaces (`ConstraintBase`, `Shape`, `Node`/`Aspect`, `setConstraintSolver`).
No change to `BodyNode`/`Skeleton`/`ConstraintSolver` layouts or vtables is
required — and specifically the design must **avoid** SoftBodyNode's two ABI
intrusions (the `Skeleton::mSoftBodyNodes` registry member and the
`BodyNode::asSoftBodyNode()` virtual).

## Prototype (empirical) — PASSED

A throwaway `DeformableSeamProbe` (`ConstraintBase` subclass) + two
`FemSeamProbe.*` tests were added to `tests/integration/test_ConstraintSolver.cpp`:
a single explicit-Euler particle one-way spring-coupled to a free rigid body,
advanced only from the `update()` hook, `isActive()==false`. Built clean into the
`test_ConstraintSolver` target (`DART_DISABLE_COMPILER_CACHE=ON`); **the prototype
changed only the test file — `dart/constraint/ConstraintSolver.cpp` and every
production header were untouched, empirically confirming the seam is ABI-safe
additive.**

Results (both tests PASSED, 2 ms):

- `UpdateHookRunsEverySolveAndIntegratesDeterministically`: `update()` was called
  exactly **200/200** times (once per `World::step()`→`solve()`); the particle
  state stayed finite and produced an **identical checksum across two independent
  runs** (deterministic explicit integration through the hook).
- `OneWaySeamLeavesRigidTrajectoryBitIdentical`: the rigid body's 200-step final
  translation was **bit-identical** with vs without the probe (‖Δ‖ == 0.0) —
  the one-way seam adds **zero perturbation** to the rigid solve, so determinism
  and zero-rigid-overhead behavior are preserved.

The probe was reverted after capture (throwaway de-risking; the tree is clean
and matches the pushed head). Two-way LCP coupling and the caller-driven path
were established analytically above (mid-solve `addExtForce` is cleared at
`World.cpp:1378`, so they are the coupling routes); an empirical two-way LCP
demo is deferred to the first real PR-2 milestone.

## Recommendation → branch strategy

Proceed with the additive-on-`release-6.20` FEM backend using the
`ConstraintBase`-hook seam: `update()` for FEM internal integration + one-way
skeleton drive; LCP-participating constraint for two-way coupling. Re-open the
branch decision only if a later milestone shows the ABI-safe surface blocks
correctness/performance parity. Model-authoring scale (tet meshes, characters)
remains a separate plan risk, not an integration-mechanism blocker.
