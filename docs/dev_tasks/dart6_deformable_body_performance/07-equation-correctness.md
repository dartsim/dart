# WP-DB.04 equation correctness

Initially captured on 2026-07-07 from branch
`wp-db-soft-skel-allocation-gates`. Updated on 2026-07-12 for the PR #3382
mass-matrix review correction.

## Command

```bash
pixi run cmake --build build/default/cpp/Release --target test_SoftDynamics --parallel 8
ctest --test-dir build/default/cpp/Release -R 'test_SoftDynamics$' --output-on-failure
```

Result: `test_SoftDynamics` passed locally in 0.09 seconds after the
point-mass gravity, combined-vector, mass-matrix, augmented-mass,
inverse-mass, inverse-augmented-mass, external-force, and representative
equation matrix/vector gate was added.

## Current slice

The active WP-DB.04 gate covers point-mass contribution to the
generalized mass, augmented-mass, inverse-mass, and inverse-augmented-mass
matrices and to gravity, combined Coriolis/gravity force, and external-force
vectors:

- `PointMass` equation-cache vectors are initialized to zero instead of
  relying on later aggregation paths to write every cache before read.
- `PointMass::aggregateGravityForceVector()` now computes the point mass's
  local gravity force, respects the parent soft body's gravity mode, and stores
  the cache used by `SoftBodyNode::aggregateGravityForceVector()`.
- `PointMass::aggregateCombinedVector()` now computes the point-mass combined
  vector contribution, and `SoftBodyNode::aggregateCombinedVector()` folds
  point-mass contributions into the parent soft body's generalized vector.
- `PointMass::aggregateMassMatrix()` now computes the point-mass inertial force
  for the current mass-matrix column, and
  `SoftBodyNode::{updateMassMatrix,aggregateMassMatrix}()` fold those point
  contributions into the parent soft body's generalized mass matrix.
- `PointMass::aggregateAugMassMatrix()` now computes the same point-mass
  inertial contribution for the augmented-mass path. The parent soft body's
  joint damping and stiffness terms remain handled by
  `SoftBodyNode::aggregateAugMassMatrix()`.
- `Skeleton::{updateInvMassMatrix,updateInvAugMassMatrix}()` keep rigid trees
  on the existing recursive inverse path, but compute soft-body tree inverse
  matrices from the public mass and augmented-mass matrices. This keeps the
  `Skeleton` API internally consistent without changing the forward-dynamics
  articulated-inertia Schur complement used for free point-mass coordinates.
- `SoftBodyNode::aggregateExternalForces()` projects point-mass local external
  forces through the parent joint into `Skeleton::getExternalForces()` without
  the old point-coordinate segment assignment. `PointMass` now documents that
  the parent soft body owns this projection because point masses are not
  exposed as DART 6 `Skeleton` DOFs.
- `PointMass::setMass()` now dirties the parent tree's articulated-inertia and
  dependent matrix/vector caches so gravity-force queries are recomputed after
  point-mass mass changes.
- `SoftDynamicsTest.pointMassGravityContributesToGeneralizedForceVectors`
  loads `test_drop_box.skel`, computes the analytical point-mass gravity
  wrench, halves every point mass, and verifies that the resulting
  `Skeleton::getGravityForces()` and `Skeleton::getCoriolisAndGravityForces()`
  deltas equal the projected point-mass gravity contribution at rest. The same
  test verifies the `Skeleton::getMassMatrix()` and
  `Skeleton::getAugMassMatrix()` deltas against
  `sum(m * J_point^T * J_point)` over the soft body's dependent DOFs, and
  verifies both left and right identity products for
  `Skeleton::getInvMassMatrix()` and `Skeleton::getInvAugMassMatrix()` before
  and after the point-mass mass change. It also applies deterministic local
  point-mass external forces, verifies `Skeleton::getExternalForces()` against
  the analytical soft-body Jacobian projection, and verifies
  `Skeleton::clearExternalForces()` clears the point-mass forces.
- `SoftDynamicsTest.representativeEquationMatrixAndVectorChecks` replaces the
  disabled random soft-body equation loop with a deterministic representative
  gate. It checks a rigid-only world (`double_pendulum.skel`), a soft-body
  world (`test_drop_box.skel`), and a mixed rigid/soft world
  (`softBodies.skel`). For every mobile skeleton in those worlds it verifies
  mass and augmented-mass matrices against Jacobian projections, verifies left
  and right inverse identities, and verifies Coriolis and combined
  Coriolis/gravity vectors against inverse dynamics at the loaded state.

This completes the current public DART 6 `Skeleton` matrix/vector equation
gate for WP-DB.04. It is not a claim of paper-level deformable solver parity.

## 2026-07-12 review correction

The current PR review found that the first implementation still composed each
point-mass mass-matrix column with the retained
`PointMass::State::mAccelerations`. `Skeleton::updateMassMatrix()` sets one
Skeleton DOF acceleration at a time, but does not overwrite point-mass state;
therefore previous soft simulation acceleration leaked as a constant term into
every public mass and augmented-mass column.

Local follow-up commit `2ad156e7b82` removes that state term from
`SoftBodyNode::updateMassMatrix()`. The point-mass column now contains only the
parent body response to the generalized-coordinate basis acceleration. Physical
inverse dynamics continues to include real point-mass accelerations through its
separate path.

`SoftDynamicsTest.pointMassAccelerationsDoNotAffectMassMatrices` is the
regression:

1. compute baseline public mass and augmented-mass matrices and compare them to
   the analytical Jacobian projection;
2. inject deterministic nonzero retained accelerations into every point mass;
3. explicitly dirty the articulated-inertia/matrix caches;
4. prove both freshly assembled public matrices are unchanged.

The test failed before the production correction with large state-dependent
terms and passed afterward. Final local evidence on `2ad156e7b82`:

```text
pixi run lint                                      PASS
DART_DISABLE_COMPILER_CACHE=ON pixi run build     PASS
DART_DISABLE_COMPILER_CACHE=ON pixi run test      PASS (152/152)
test_SoftDynamics                                 PASS (16/16)
INTEGRATION_StepAllocation                        PASS
independent semantic and regression reviews      CLEAN x2
```

This exact correction is release-only: DART 7 `main` still has point-mass mass
aggregation disabled. The separate zero-DoF soft bias-impulse assertion fix on
this PR does apply to `main` and remains subject to the dual-PR policy.

## Remaining gaps

- `Skeleton::updateTotalMass()` sums the non-virtual `BodyNode::getMass()`,
  so flesh mass appears in the shadowing `SoftBodyNode::getMass()` but never
  in Skeleton-level totals (`Skeleton::getMass()` and COM-style aggregates).
  This is a pre-existing DART 6 accounting split, left unchanged for
  compatibility: point-mass gravity and inertia reach the skeleton through
  the aggregation paths gated above. `PointMass::setMass()` now follows the
  `BodyNode::setInertia()` dirty protocol (articulated-inertia dirtying,
  total-mass recompute, deactivation-state wake, version bump) via
  `SoftBodyNode::handlePointMassMassChange()`, and the gravity gate asserts
  the rigid-only skeleton total stays fixed while the soft body's
  flesh-inclusive mass tracks point-mass changes.

- Point-mass inverse-mass and inverse-augmented-mass aggregation methods are
  still legacy stubs for the old point-coordinate formulation; the active
  public `Skeleton` inverse-matrix API is covered by the soft-tree solve above.
- The remaining legacy comments that assign point-mass vector segments to
  independent generalized coordinates still need a DART 6-compatible
  replacement or an explicit design decision, because current soft point masses
  are not exposed as ordinary `Skeleton` DOFs.
