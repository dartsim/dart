# FCL Primitive Collision Test Plan

## Goals

- Verify collision normals and penetration depths across all primitive shape
  pairs using both pure FCL and DART's FCLCollisionDetector (primitive mode).
- Cover multiple collision cases (separated, touching, shallow/deep overlap,
  containment, rotated/edge cases) with stable, deterministic fixtures.
- Provide evidence to switch DART to FCL primitive shapes by default (issue #19).

## Scope

### Primitive shapes (FCL + DART)

- Box
- Sphere
- Ellipsoid
- Cylinder
- Cone
- Plane (FCL Halfspace)

Notes:

- DART's FCL primitive path does not currently map CapsuleShape.
- Plane is represented as FCL Halfspace; it is unbounded and only supports
  one-sided contact.
- Cylinder and Cone now use analytic FCL primitives in PRIMITIVE mode; mesh
  BVH remains the fallback for MESH mode.

### Pair matrix

- Cover all ordered pairs of the shapes above (including shape vs itself).
- Validate symmetry by testing both orderings (A,B) and (B,A) for each case.

## Collision cases

Each pair is tested against a small, reusable set of canonical cases to avoid
combinatorial explosion. Applicable cases vary per pair.

Base cases (for all bounded pairs unless noted):

- Separated: no contact.
- Touching: contact with ~0 penetration (epsilon tolerance).
- Shallow overlap: small penetration depth.
- Deep overlap: larger penetration depth.
- Contained: one object fully inside another (not for Plane).

Orientation cases (non-spherical shapes):

- Rotated: rotate one object 90 degrees around a non-symmetric axis.
- Edge/vertex: align to create edge or vertex contact for boxes/cones/cylinders.

Axis-variation cases (non-cone pairs):

- X-axis shallow overlap: translate along +X to verify normal alignment on a
  non-Z axis without changing orientation.

Plane cases:

- Above plane: separated.
- On plane: touching (epsilon).
- Below plane: overlapping penetration.
- Rotated normal: plane normal not aligned with global axes.

## Expectations

### Pure FCL

- `contact.normal` points from `o1` toward `o2`.
- `contact.penetration_depth` is positive for overlap (>= 0 for touching).
- Contact point lies within the overlap region; for Halfspace contacts, the
  signed distance to the plane should be within the penetration depth (it may
  be a mid-penetration point, not necessarily on the plane).

### DART (FCLCollisionDetector)

- DART contact normal follows the DART convention: from object 2 to object 1.
- Penetration depth is positive for overlap (>= 0 for touching).
- Contact point lies within the overlap region; for PlaneShape contacts, the
  signed distance to the plane should be within the penetration depth (it may
  be a mid-penetration point, not necessarily on the plane).

## Implementation plan

1. Pure FCL test harness

- New test file under `tests/integration/collision/` that:
  - Builds `fcl::CollisionObject` pairs directly.
  - Runs `fcl::collide` with `enable_contact`.
  - Verifies normal direction, penetration depth, and contact placement.
  - Executes the case matrix for all ordered pairs.

2. DART API test harness

- Expand `test_FclPrimitiveContactMatrix.cpp` to:
  - Add Cone and additional case coverage.
  - Mirror the same case matrix as pure FCL.
  - Check DART's normal orientation and depth conventions.

3. Edge-case validation

- Include cases for near-zero penetration and containment.
- Validate behavior around plane contacts and non-axis-aligned normals.
- Defer cone side-contact offsets until a dedicated placement strategy is
  defined (cone taper makes naive X-axis offsets miss contact).

## Parameter choices

Base sizes (tunable for stability):

- Sphere radius: 0.5
- Box extents: (1.0, 0.8, 0.6)
- Ellipsoid radii: (0.6, 0.4, 0.3)
- Cylinder radius/height: 0.4 / 0.8
- Cone radius/height: 0.5 / 1.0
- Plane: normal (0, 0, 1), offset 0

Offsets for cases:

- Separated: +0.2
- Touching: -1e-4
- Shallow overlap: -0.02
- Deep overlap: -0.2

## Progress tracking

- [x] Confirm primitive mapping support and finalize shape list
- [x] Implement pure FCL harness + base cases
- [x] Expand DART harness to mirror cases
- [x] Add containment cases (box/sphere/cylinder/ellipsoid containers)
- [x] Add cone side-contact placement strategy and cases
- [x] Run targeted tests and record outcomes
- [x] Update README with summary and remaining issues

Notes:

- Added a Z-axis rotation case (`rotated_shallow`) to both FCL and DART matrix
  tests; cone side-contact placement is now covered.
