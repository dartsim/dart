# FCL Primitive Shapes (Issue #19)

## Status

- Implemented a primitive collision-pair matrix test and a contact conversion
  fix; PR #2339 open.
- Added base collision-pair matrix tests for both pure FCL and DART primitive
  mode; plane contacts accept mid-penetration points per FCL behavior.
- Switched cylinder/cone primitive handling to analytic FCL shapes to align
  normals/penetration depth with FCL results.
- Added an X-axis shallow overlap case for non-cone pairs, cone side contact,
  edge/vertex diagonal cases, and rotated plane-normal coverage.
- Added containment cases for box/sphere/cylinder/ellipsoid containers in both
  the pure FCL and DART collision matrices.
- Completed the comprehensive primitive collision test plan for pure FCL and
  DART coverage; edge cases are now covered.

## Context

- Issue #19 asks DART to pass primitive shapes directly to FCL instead of
  meshing them; historical fallback exists because contact normals, penetration
  depths, and contact points have not been consistent across shape pairs.
- DART’s Contact convention is the source of truth: the contact normal points
  from collision object 2 to collision object 1, and penetration depth is
  positive for overlap (see `dart/collision/Contact.hpp`).

## Current Notes

- FCL primitive handling is selectable via `FCLCollisionDetector` and currently
  defaults to mesh mode.
- DART expects contact normals from collision object 2 → 1, while FCL normals
  are defined from o1 → o2; FCL may also swap contact geometry order vs the
  input collision objects.

## Decisions (So Far)

- Built a collision-pair matrix test that compares contact normals, penetration
  depth sign, and contact point placement against the DART convention before
  changing primitive handling.
- Normalize primitive contact conversion by checking whether FCL’s contact
  geometry order matches or swaps the input collision objects, and flip the
  normal/IDs accordingly.
- Use FCL primitives for Cylinder/Cone in primitive mode (remove mesh fallback)
  so DART primitive contacts match FCL conventions.

## Repro

- Run the primitive contact matrix test:
  `ctest -R INTEGRATION_collision_FclPrimitiveContactMatrix`
- Run the pure FCL matrix test:
  `ctest -R INTEGRATION_collision_FclPrimitiveContactMatrixFcl`

## Local Checks

- `pixi run test`
- `pixi run test-all`
- `pixi run lint`
- `DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run -e gazebo test-gz`

## Next Steps

- Reassess whether to switch the default FCL primitive mode now that the
  expanded collision matrix and edge-case coverage are complete.
