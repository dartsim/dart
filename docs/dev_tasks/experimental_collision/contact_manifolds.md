# Contact Manifold Notes (Experimental Collision)

## Purpose

Capture how contact results are represented across libraries and define the
targets for DART's experimental collision module.

## Contact Result Types

| Type   | Representation                   | Typical use                          |
| ------ | -------------------------------- | ------------------------------------ |
| Point  | Single position + normal + depth | Sphere-sphere, vertex-face           |
| Edge   | Line segment or two points       | Edge-edge contacts (boxes, capsules) |
| Face   | Polygon/patch with shared normal | Box-face, mesh-face                  |
| Patch  | Curved surface region            | Capsule-cylinder, sphere-plane       |
| Volume | Overlap volume + gradient        | SDF/voxel collision, deformation     |

## Manifold Generation Techniques

- **SAT clipping**: Use separating axes and clip incident/reference faces to
  produce up to 4 points (common in box-box and polyhedral contacts).
- **GJK/EPA**: Provides one penetration direction and witness points; needs
  additional steps to generate a manifold or patch.
- **MPR/XenoCollide**: Similar to EPA for penetration; typically yields a
  single contact direction and position estimate.
- **Persistent manifolds**: Cache contact points across frames and refresh
  them using relative motion (Bullet, ReactPhysics3D, BEPU).
- **Contact patches**: Compute polygonal contact regions (Coal) for improved
  torque/friction stability.

## Reduction and Stability

- Limit manifolds to a fixed max count (usually 4) for solver stability.
- Prefer reduction heuristics that preserve area and torque (e.g., keep points
  farthest from centroid in tangent plane).
- Refresh cached manifolds each frame with projection + distance thresholds.

## DART Experimental Targets

- Keep `ContactManifold` as the primary container for multi-point contacts.
- Emit a single-point manifold for convex-convex (GJK/EPA/MPR) initially.
- Add manifold generators for box-box and polyhedral contacts (SAT clipping).
- Extend to contact patches for face-face and near-coplanar cases.
- Provide contact type tags (Point/Edge/Face/Patch) to enable solver tuning.
- Current experimental `ContactManifold` keeps up to 4 contacts and applies a deterministic spread-based reduction.
- ContactType is a semantic tag; `isTypeCompatible()` is a lightweight sanity check for counts and shared normals.
- The 4-point cap is a solver limit; it still represents a surface manifold/patch, not a volumetric overlap.

## Notes for Backends

- Bullet and ReactPhysics3D limit to 4 points with persistent caching.
- Coal provides contact patches; Parry returns contact manifolds.
- libccd and OpenGJK return a single direction/point; manifold generation is
  caller responsibility.
