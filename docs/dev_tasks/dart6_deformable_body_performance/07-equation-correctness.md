# WP-DB.04 equation correctness

Captured on 2026-07-07 from branch `wp-db-soft-skel-allocation-gates`.

## Command

```bash
pixi run cmake --build build/default/cpp/Release --target test_SoftDynamics --parallel 8
ctest --test-dir build/default/cpp/Release -R 'test_SoftDynamics$' --output-on-failure
```

Result: `test_SoftDynamics` passed locally in 0.09 seconds after the
point-mass gravity, combined-vector, and mass-matrix aggregation gate was
added.

## Current slice

The first active WP-DB.04 sub-gate covers point-mass contribution to the
generalized mass matrix and to gravity and combined Coriolis/gravity force
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
- `PointMass::setMass()` now dirties the parent tree's articulated-inertia and
  dependent matrix/vector caches so gravity-force queries are recomputed after
  point-mass mass changes.
- `SoftDynamicsTest.pointMassGravityContributesToGeneralizedForceVectors`
  loads `test_drop_box.skel`, computes the analytical point-mass gravity
  wrench, halves every point mass, and verifies that the resulting
  `Skeleton::getGravityForces()` and `Skeleton::getCoriolisAndGravityForces()`
  deltas equal the projected point-mass gravity contribution at rest. The same
  test verifies the `Skeleton::getMassMatrix()` delta against
  `sum(m * J_point^T * J_point)` over the soft body's dependent DOFs.

This is a correctness slice, not full equation parity.

## Remaining gaps

- `SoftDynamicsTest.compareEquationsOfMotion` remains disabled.
- Point-mass augmented-mass, inverse-mass, and inverse-augmented-mass
  aggregation paths are still stubbed or only partially wired.
- The legacy comments that assign point-mass vector segments to independent
  generalized coordinates still need a DART 6-compatible replacement or an
  explicit design decision, because current soft point masses are not exposed
  as ordinary `Skeleton` DOFs.
- Rigid-only, soft-only, and mixed rigid-soft matrix/vector checks still need
  active tests before WP-DB.04 can be considered complete.
