# Shape and query capability draft

Source revision: `4ff47d438c3695f2ce63ef702949890e48258b52` (`main`,
2026-10-13). This is an implementation and test-contract inventory, with no new
runtime qualification. It prepares the published matrix required by
[M1f/D13](README.md#default-detector-switch); publication and M1b's strict
rejection of unsupported shapes remain gates. The C++ default stays FCL in 7.0;
its later switch also needs representative FCL comparisons and PLAN-622's
coverage gate. No detector default, API, binding or rename is changed here.

## Reading the cells

Conversion describes admission and representation, not collision or query
support. Collision and distance are pair operations: a shape admitted for one
peer can be unsupported for another. Raycast is an object query. All cells
remain **unqualified for the release** until exact-build evidence covers their
claimed options, operands and geometry.

- `C`, `D`, `R`: native collision, distance/nearest-point or raycast route
  exists in the independent dispatch below. These are source facts.
- `FC`, `FD`: the FCL adapter delegates collision or distance for this
  representation. Coverage depends on the pair, geometry mode and linked FCL;
  conversion alone supplies no pair-level guarantee.
- `U`: intended geometry or operation is explicitly unsupported. A placeholder,
  skipped object, false hit or unfound distance result is not support.
- `Q`: a conditional or incomplete route whose intended capability needs
  qualification. Existing test names below are contracts present in source,
  not test runs performed for this draft.

Names are current `dart::dynamics` names. Bound shapes retain their current
`dartpy.dynamics` names; `ConvexMeshShape`, `PyramidShape` and
`HeightmapShaped` have no registered class in the inspected dartpy sources.
`HeightmapShape<float/double>` is public C++; `HeightmapShapef` and
`HeightmapShaped` are aliases, and only `HeightmapShapef` is bound.
`MultiSphereShape` is the deprecated C++ alias of
`MultiSphereConvexHullShape`, not another geometry. This inventory does not
adopt the planned Python vocabulary: rename table v0 still needs maintainer
approval. See [shape bindings](../../../python/dartpy/dynamics/Shape.cpp) and
[public shape headers](../../../dart/dynamics/Shape.hpp).

## Native detector: public shapes

The conversion source is
[`NativeShapeConversion::create`](../../../dart/collision/dart/detail/NativeShapeConversion.cpp)
(lines 341–434 at the pin). Unsupported types warn once and return null; an
AABB retained by the collision object does not create a narrow-phase shape.

| Public shape | Conversion and conditions | Collision | Distance / nearest points | Raycast |
| --- | --- | --- | --- | --- |
| `Shape` | Abstract base; custom derived type needs a converter | U for an unrecognized type | U | U |
| `SphereShape` | Analytic sphere | C | D, excluding mesh peers | R |
| `BoxShape` | Analytic box | C | D, excluding mesh peers | R |
| `CapsuleShape` | Analytic capsule | C | D, excluding mesh peers | R |
| `CylinderShape` | Analytic cylinder | C | D, excluding mesh peers | R |
| `EllipsoidShape` | Sphere if `isSphere()`; otherwise a convex approximation with 162 vertices / 320 triangular faces | C on that representation; separate soft route | D on that representation, excluding mesh peers | R on that representation |
| `ConeShape` | Convex approximation: apex plus a 64-segment rim | C | D, excluding mesh peers | R on the approximation |
| `PyramidShape` | Five convex vertices; convex faces computed from them | C | D, excluding mesh peers | R with valid faces |
| `ConvexMeshShape` | Supplied convex vertices/faces; empty vertices rejected | C | D, excluding mesh peers | R with nonempty valid convex faces; degeneracy Q |
| `MeshShape` | Scaled triangle mesh; empty data or invalid/unrepresentable indices rejected | C with primitives, convex, mesh and plane | D only with mesh or plane; primitive/convex peers U | R on triangles |
| `ArrowShape` | Inherits `MeshShape` type and uses its generated mesh | Same as mesh | Same as mesh | Same as mesh |
| `PlaneShape` | Analytic plane normal/offset | C except plane/plane | D except plane/plane | R |
| `VoxelGridShape` | `HAVE_OCTOMAP`: occupied leaves become a compound of boxes; empty tree produces empty compound | C through children | Q through child pairs; a box/mesh child pair remains U | R through children; closest child only |
| `SoftMeshShape` | Separate live `SoftBodyNode` cache; no generic native shape | C only with sphere, ellipsoid, box, plane or soft mesh, in both orders | U: null generic shape is skipped | U: null generic shape is skipped |
| `HeightmapShape<float/double>` | U: warning and skipped object | U | U | U |
| `MultiSphereConvexHullShape` / `MultiSphereShape` | U: warning and skipped object | U | U | U |
| `PointCloudShape` | U: warning and skipped object | U | U | U |
| `LineSegmentShape` | U: warning and skipped object | U | U | U |

Mesh data and scaling must be recorded as inputs. `ConvexMeshShape` assumes
convex geometry; admission is not proof that arbitrary supplied triangles form
a valid convex surface. Cone radial under-approximation is bounded by the
conversion's 64-segment rim; ellipsoid approximation has no accuracy gate in
this draft. Voxel occupancy resolution and threshold affect the representation.
The shape/mesh facilities retain their existing Assimp dependency, and OctoMap
remains conditional; native selection does not remove required FCL.

## Native operations: independent dispatch

[`NarrowPhase`](../../../dart/collision/dart/narrow_phase/NarrowPhase.cpp)
defines collision admission in `isSupported` (1119), distance admission in
`isDistanceSupported` (1196), and raycast admission in `isRaycastSupported`
(1251). The corresponding execution dispatches, rather than conversion alone,
back these cells:

| Native representations | Collision | Distance / nearest points |
| --- | --- | --- |
| Any pair among sphere, box, capsule, cylinder and convex | C | D |
| Plane with those five representations | C | D |
| Mesh with those five representations | C via triangle tests | U |
| Mesh/mesh, mesh/plane | C | D |
| Plane/plane | U | U |
| Compound | Recurse over admitted children | Recurse over supported child pairs; unsupported children supply no distance |

Internal native `Sdf` dispatch is not admission for a public dynamics shape;
there is no public-shape conversion to it in this inventory. Unsupported soft
pairs include soft mesh with capsule, cylinder, cone, pyramid, convex mesh,
rigid mesh or voxel grid. The
[soft dispatcher](../../../dart/collision/dart/SoftCollision.cpp) returns no
contacts and logs an error for unsupported pairs (1602). Default supported soft
contacts use point/face sampling; sphere/box face-interior contacts are opt-in
through `setSoftFaceInteriorContactsEnabled` or the existing environment switch.
Conversion, Boolean overlap and contact accuracy therefore need separate
evidence, including finite normals, both operand orders and deformation.

The public
[`DARTCollisionDetector`](../../../dart/collision/dart/DARTCollisionDetector.cpp)
adapter targets four solver-facing contacts per rigid manifold and retains a
persistent manifold cache. `CollisionOption` controls contacts, global/per-pair
caps, filtering and negative-depth admission; record all of them. Contact count
and warm-cache behavior are not interchangeable with FCL output.

For `distance`, record `DistanceOption.enableNearestPoints`, `distanceLowerBound`
and `distanceFilter`. Both group overloads exist. The reported minimum is clamped
to the lower bound; `unclampedMinDistance` retains the computed value. Early
termination can occur at the lower bound; use `-inf` to disable it. An unsupported
or filtered-out pair can produce a scalar zero with `DistanceResult.found()`
false, so scalar zero cannot establish overlap or support. Nearest points are
populated only when requested (adapter lines 335–365, 551–608, 1799–1865).

For `raycast`, the adapter uses the finite segment from `from` to `to`, rejects
zero-length segments, and disables native backface culling. Sphere, box, capsule,
cylinder, plane, mesh, convex and compound have independent raycast dispatch.
Convex raycasts need valid faces; empty faces return false. `mEnableAllHits`
collects one nearest hit per collision object, not every entry/exit or mesh-face
crossing. A compound contributes its closest child hit. `mSortByClosest` and
`mFilter` apply to those object hits (adapter 482–545, 1869–1910;
[`raycastConvex`](../../../dart/collision/dart/narrow_phase/Raycast.cpp):672).

## FCL detector: public shapes

[`createFCLCollisionGeometry`](../../../dart/collision/fcl/FCLCollisionDetector.cpp)
(1116–1275) selects the representations below. Current defaults are
`FCLCollisionDetector::PRIMITIVE` and contact processing
`FCLCollisionDetector::DART`. Geometry mode should be selected before objects
are constructed; changing the mode field does not invalidate cached geometry.

| Public shape | Conversion and conditions | Collision | Distance / nearest points | Raycast |
| --- | --- | --- | --- | --- |
| `Shape` | Abstract base; unknown derived types substitute geometry | U for intended geometry | U | U |
| `SphereShape` | PRIMITIVE analytic sphere; MESH tessellated ellipsoid | FC | FD | U |
| `BoxShape` | PRIMITIVE analytic box; MESH triangle cube | FC | FD | U |
| `EllipsoidShape` | PRIMITIVE analytic ellipsoid; MESH fixed tessellation | FC | FD | U |
| `CylinderShape` | PRIMITIVE analytic cylinder; MESH 16-slice/16-stack mesh | FC | FD | U |
| `ConeShape` | PRIMITIVE analytic cone; MESH generated BVH | FC | FD | U |
| `PyramidShape` | Triangle BVH in both modes | FC | FD | U |
| `ConvexMeshShape` | FCL convex with vertices and triangles; empty input substitutes geometry | FC for valid input | FD; plane peer Q | U |
| `MeshShape` | Scaled triangle mesh in OBBRSS BVH | FC | FD | U |
| `ArrowShape` | Inherits mesh conversion | FC | FD | U |
| `PlaneShape` | Analytic halfspace in both modes; MESH warns | FC | FD for listed analytic peers; other peers Q | U |
| `SoftMeshShape` | Refreshed Assimp triangle BVH, with moving vertices | FC; deformation accuracy Q | FD; soft-query accuracy Q | U |
| `VoxelGridShape` | OcTree only with both `HAVE_OCTOMAP` and `FCL_HAVE_OCTOMAP`; otherwise substituted geometry | FC conditionally | FD conditionally; plane peer Q | U |
| `CapsuleShape` | U: substituted ellipsoid mesh | U for capsule geometry | U for capsule geometry | U |
| `HeightmapShape<float/double>` | U: substituted ellipsoid mesh | U | U | U |
| `MultiSphereConvexHullShape` / `MultiSphereShape` | U: substituted ellipsoid mesh | U | U | U |
| `PointCloudShape` | U: substituted ellipsoid mesh | U | U | U |
| `LineSegmentShape` | U: substituted ellipsoid mesh | U | U | U |

The unsupported fallback logs a sphere substitution but constructs
`createEllipsoid(0.1, 0.1, 0.1)`; it does not implement the requested shape.
Capsule/FCL comparison fixtures must name any replacement shape explicitly;
such a comparison cannot qualify FCL capsule support.

FCL is required (minimum 0.7.0); linked FCL, libccd precision and optional OctoMap
versions belong in runtime evidence. Its compile guard rejects single-precision
libccd unless `DART_ALLOW_SINGLE_PRECISION_LIBCCD` is explicitly enabled. FCL
halfspaces are excluded from its AABB tree and processed separately. Analytic
halfspace distance handles OBBRSS BVHs,
box, sphere, ellipsoid, cylinder and cone, plus a separate halfspace/halfspace
case. Convex and OcTree fall back to upstream FCL distance and remain Q for those
pairs; no blanket plane-distance claim follows from conversion. FCL nearest-point
requests are forwarded without an explicit signed-distance request. The lower
bound and unfound-result caveats also apply to its distance adapter. BVH/MESH
contact processing can remove duplicate/collinear contacts and zero normals;
published contact counts need direct checks. FCL contact conversion copies the
reported penetration depth (2356), including negative values; comparisons must
preserve those values rather than clamp them to manufacture parity. Native
negative-depth admission checks are separate (adapter 882, 924). FCL has no
raycast override: the
[base method](../../../dart/collision/CollisionDetector.cpp) warns and returns
false (80–90), regardless of shape.

## Evidence record and remaining gates

Use one record per `(shape, backend, operation, peer, options)`; `operation` is
`conversion`, `collision`, `distance`, `nearest_points` or `raycast`. Preserve
current C++/Python names, representation/approximation, dependency conditions,
source revision/symbol, test case, expected behavior, and qualification status.
A qualified record additionally needs exact source/build/binary attribution,
command, date, geometry/assets, operands/order, options, tolerances, actual result
and limitations. This is a documentation schema; no new generator is introduced.

Source-present test seeds, unrun for this draft:

| Surface | Existing test contracts |
| --- | --- |
| Native conversion and public queries | [test_DARTCollisionEngine.cpp](../../../tests/unit/collision/test_DARTCollisionEngine.cpp): `LeavesUnsupportedShapesNull`, `ConeConvertsToConvex`, `ConvertsVoxelGridToCompoundShape`, `DistanceMatchesFclForSupportedPrimitivePairs`, `DistanceLeavesMeshPrimitivePairsUnsupported`, `RaycastCoversPrimitiveRows`, `RaycastCoversConvexRows`, `RaycastRespectsAllHitsSortingAndFilters` |
| Native pair dispatch and distance | [test_narrow_phase_dispatch.cpp](../../../tests/unit/collision/dart/test_narrow_phase_dispatch.cpp), [test_distance.cpp](../../../tests/unit/collision/dart/test_distance.cpp): pair order, unsupported plane rows, capsule containment, nearest-point options and rotated boxes; internal SDF tests do not qualify public-shape conversion |
| Native soft collision | [test_DARTCollisionDetector.cpp](../../../tests/unit/collision/test_DARTCollisionDetector.cpp): order symmetry, deformation/topology, finite contacts and opt-in face interiors |
| FCL collision | [test_FclPrimitiveContactMatrix.cpp](../../../tests/regression/test_FclPrimitiveContactMatrix.cpp): six PRIMITIVE kinds, with explicit skipped/relaxed cases; pyramid BVH parity, rotated planes, containment and cone-side contacts |
| FCL distance | Same regression file: cone/plane nearest points in both orders; [test_Distance.cpp](../../../tests/integration/test_Distance.cpp): options/minimum and opposed halfspaces |
| Python query bindings | [CollisionGroup.cpp](../../../python/dartpy/collision/CollisionGroup.cpp) exposes current `distance`/`raycast`; the [Python raycast test](../../../python/tests/unit/collision/test_collision.py) constructs Bullet and does not qualify native or FCL raycast |

The legacy C++ raycast integration helpers also return early for non-Bullet
detectors; invoking them with DART/FCL is not query coverage. Native detector
tests above supply distinct source contracts, still requiring exact-build runs.
The earlier 624 collision rows and 312 separated controls are fixed-binary
diagnostics without exact commit/build attribution. Their six strict soft-pair
misses remain diagnostic; they qualify neither this revision nor distance,
nearest points, raycast, dynamics or a default switch. They were not rerun here.

Open work: exact-build pair/query qualification across rotations, shallow contact,
containment, degeneracy, scale, mesh assets, voxel dependencies and soft
deformation; current dartpy native query tests; numerical/behavior comparisons;
and the published matrix plus M1b strict rejection. Unsupported query pairs on
otherwise admitted shapes need an explicit M1b/M1f policy; this draft introduces
no new rejection behavior. Images do not establish conversion or dispatch
coverage; source contracts and query-specific text oracles own this evidence.

Changelog decision: `decide`, base `4ff47d438`, documentation/schema draft only;
no entry required because released APIs, defaults, packages and user workflows
are unchanged. Target section and entry: N/A. Follow-up: none for this draft;
later default/API changes need their own decision and acceptance gates.
