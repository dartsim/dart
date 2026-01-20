# Collision Conventions and Terminology (DART Experimental)

## Purpose

Standardize normals, signed distances, and contact semantics across backends so
comparisons and adapters stay consistent and testable.

## DART Target Conventions

**Contacts**

- Contact normal points from object2 to object1.
- Penetration depth is positive for overlap.
- Contact position is the midpoint between witness points (pointOnA/pointOnB)
  unless a single surface point is explicitly required.

**Distance queries**

- Signed distance is positive when separated and negative when penetrating.
- Distance normal should point from object1 to object2, i.e.
  `normal = (pointOnObject2 - pointOnObject1).normalized()`.

## Definitions

- **Signed distance**: minimum distance between surfaces. Negative when
  penetrating.
- **Penetration depth**: magnitude of overlap (positive), often computed by EPA
  or MPR.
- **Normal direction**: depends on convention; must be normalized before
  comparisons.

## Backend Mapping (Observed)

| Backend        | Normal convention                | Penetration sign  | Signed distance | Notes                                             |
| -------------- | -------------------------------- | ----------------- | --------------- | ------------------------------------------------- |
| DART           | object2 -> object1               | positive          | yes             | Target convention for experimental module         |
| FCL            | o1 -> o2                         | positive          | yes             | Flip normals for DART contact convention          |
| Coal (HPP-FCL) | o1 -> o2                         | positive          | yes             | Similar to FCL                                    |
| Bullet         | normalWorldOnB (B -> A)          | distance negative | yes             | If B == object2, normal already matches DART      |
| ODE            | normal points into body1         | positive          | no              | Equivalent to object2 -> object1 if g1 is object1 |
| Parry          | shape1 -> shape2                 | negative dist     | yes             | Signed distance built in; flip for DART contact   |
| ReactPhysics3D | body1 -> body2                   | positive          | no              | Discrete contacts only                            |
| libccd         | direction moves obj2 out of obj1 | positive          | no              | Flip direction for DART contact normal            |
| OpenGJK        | distance-only                    | N/A               | no              | Witness points only, no manifold                  |

## Conversion Rules

**Normals**

- If backend normal is object1 -> object2, flip to match DART contact normal.
- If backend normal is object2 -> object1, keep as-is for contacts.

**Penetration depth**

- Keep positive depth values.
- If backend reports negative penetration distance, negate before storing in
  ContactPoint.depth.

**Signed distance**

- Keep negative for penetration, positive for separation.
- If backend reports penetration depth separately, set
  `signed_distance = -penetration_depth` and provide witness points.

## DART Experimental TODOs

- Standardize DistanceResult.normal across all helpers to `point2 - point1`.
- Add regression tests to lock down sign and normal conversions.
- Provide adapter helpers that normalize backend results before comparison.
