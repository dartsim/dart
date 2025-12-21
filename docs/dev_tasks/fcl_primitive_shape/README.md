# FCL Primitive Shapes (Issue #19)

## Status

- Implemented a primitive collision-pair matrix test and a contact conversion
  fix; awaiting PR/CI validation.

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

## Repro

- Run the primitive contact matrix test:
  `ctest -R INTEGRATION_collision_FclPrimitiveContactMatrix`

## Next Steps

- Sync with `origin/main`, push branch, and open PR for review/CI.
