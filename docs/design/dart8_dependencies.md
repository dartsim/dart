# DART 8 Gazebo-Bound Dependencies

This document owns the DART 8.0 design for what DART 7 keeps only because the
C++ API that gz-physics and gz-sim use binds it: assimp, FCL, Bullet, ODE,
tinyxml2 in installed headers, octomap, the C++ APIs that DART 7 deprecates
for removal, and the C++17 interface. For each item it records the
motivation, the Gazebo compatibility constraints, the prerequisites and the
open questions.

None of this changes DART 7. No deprecated C++ API is removed during 7.x (see
the [DART 7 API boundary](dart7_python_api.md#goals-and-boundary)), and the
[compatibility rules](../../AGENTS.md#compatibility-rules) keep the
Gazebo-facing API source- and behavior-compatible. Roadmap state and
sequencing belong to PLAN-625 in the [plan dashboard](../plans/dashboard.md).
The collision facades below follow the
[later backend lifecycle](dart6_collision_backends.md#later-backend-lifecycle).

DART facts were read on `main` at `38b1484eb` (2026-10-24). Gazebo facts come
from the gz-physics releases that DART's Gazebo lanes build, 7.8.0 (Harmonic),
8.4.0 (Ionic) and 9.5.2 (Jetty), and from the gz-physics revision that the
[gz API inventory](../../tools/gazebo/README.md#dart-c-api-inventory) records.
Sizes marked "reported" come from DART 7's dependency inventory (2026-10-18,
`main` `17ce7eefa`) and have not been reproduced by a repository script.

## Summary

| Item | What binds it in DART 7 | DART 8 direction |
| --- | --- | --- |
| assimp | `MeshShape.hpp` exposes `aiScene`, which gz-physics' `CustomMeshShape` builds | A private, optional importer; the `aiScene` API removed |
| FCL and libccd | The C++ default detector, and the `"fcl"` key that gz-physics offers | Optional behind an FCL-free `FCLCollisionDetector` facade |
| Bullet and ODE | gz-physics subclasses both detectors and, from 9.5.2, casts rays through raw Bullet and ODE objects | Facades over the native detector, on a DART-owned ray query |
| tinyxml2 | `XmlHelpers.hpp` names tinyxml2 types | Private to `dart-io` in shared builds |
| 7.0 deprecations | No deprecated C++ API leaves during 7.x | Removed where no supported Gazebo release uses them |
| C++17 interface | gz-physics builds as C++17 and probes `ContactSurface.hpp` as C++14 (Harmonic) or C++17 | Export C++20 |
| octomap | `VoxelGridShape` and `PointCloudShape` APIs; FCL's packages require it | A DART-native voxel grid |

## Shared constraints

These apply to every item below.

- **The package request.** gz-physics asks gz-cmake for DART 6.10 or newer
  with the `collision-bullet`, `collision-ode`, `utils` and `utils-urdf`
  components. When that request fails, gz-cmake only warns and builds
  gz-physics without its dartsim plugin. Every DART 8 build that Gazebo can
  use keeps those components and a version file that accepts the request,
  including builds in which a component no longer brings an external library.
- **Subclasses.** All three gz-physics releases subclass
  `OdeCollisionDetector` and `BulletCollisionDetector`, register the
  subclasses under the `"ode"` and `"bullet"` keys, override both `collide`
  overloads to cap contacts per pair, and create every world with the ODE
  subclass, so ODE's contacts are Gazebo's default. gz-physics 9.5.2 also
  overrides `createCollisionGroup` and `raycast` and subclasses
  `OdeCollisionGroup` and `BulletCollisionGroup`. `CustomMeshShape`
  subclasses `MeshShape`. These classes keep their protected constructors,
  virtual interfaces and `Registrar` support, so they stay subclassable.
  `tests/integration/test_GazeboShim.cpp` mirrors these extension points.
- **The contact-surface probe.** gz-physics compiles
  `dart/constraint/ContactSurface.hpp` with `CHECK_INCLUDE_FILE_CXX` and
  `-std=c++14` (`/std:c++14` on MSVC). Harmonic (`cmake_minimum_required`
  3.5.1) compiles the probe as C++14. Ionic and Jetty (3.22.1) get policy
  CMP0067, so the check also receives their `CMAKE_CXX_STANDARD` 17, and
  `-std=c++14 -std=c++17` compiles as C++17. DART's `ContactSurfaceProbe`
  keeps the strictest case, `-std=c++14`. When the probe fails, gz-physics
  compiles out contact-surface customization with only a configure status
  message, and gz-sim's tracked vehicles and conveyors stop working without
  an error. DART's Gazebo lanes fail on a lost probe since #3849.
- **gz-sim.** gz-sim finds DART's package only for its physics-system
  integration test, which includes `dart/config.hpp`, checks
  `DART_VERSION_AT_LEAST` and skips itself when DART is not found. Its other
  dependence on DART is the behavior of the dartsim plugin.
- **Evidence.** The gz API inventory records the DART C++ surface that
  gz-physics and gz-sim use, but it is lexical. For example, it does not
  record gz-physics' use of `Eigen::Vector6d`, an alias that DART declares in
  namespace `Eigen` (`dartsim/src/JointFeatures.cc` in all three releases). A
  removal is decided by building and testing unmodified gz-physics and gz-sim
  against it; the inventory shows where to look.
- **Release pairing.** Several items need gz-physics changes first. An item
  can ship in DART 8 only when no gz-physics or gz-sim release that DART 8
  supports still depends on what it removes.

**Open question.** Which gz-physics and gz-sim releases DART 8 must build
unmodified. A release that calls a removed API, as gz-physics 9.5.2 does with
raw Bullet and ODE objects, either stays on DART 7 or needs DART 8 to keep the
real backend as an option.

## assimp

**Motivation.**

- `MeshShape.hpp` includes `<assimp/scene.h>` and exposes `aiScene` through
  two constructors, four `setMesh` overloads, `getMesh()`, three `loadMesh`
  overloads and the protected `MeshHandle`. `SoftMeshShape.hpp` exposes
  `aiMesh`, `AssimpInputResourceAdaptor.hpp` derives from assimp's IO
  classes, and `dart/collision/fcl/CollisionShapes.hpp` reads an `aiScene` in
  `createMesh`. The core library therefore links assimp publicly, with a
  public `DART_ASSIMP_VERSION_MAJOR` definition, and every core consumer needs
  assimp's development files.
- assimp is the largest library in the dartpy wheel (reported 4.8 MB
  compressed), and assimp with zlib is about half of the core library's
  dependency closure (reported 51% of 27.9 MB).
- assimp parses untrusted model files, and 70 assimp CVEs were published from
  2023-10 to 2026-10 (reported from NVD on 2026-10-19).
- DART 6.20 deprecated the `aiScene` API of `MeshShape` for removal in DART 8,
  with `TriMesh` replacements (#3145).

**Design.** assimp stays DART's mesh importer, private and optional; DART
writes no format readers of its own. DART 8 removes the `aiScene` and `aiMesh`
API above, `AssimpInputResourceAdaptor.hpp` from the installed set, the
`aiScene` form of `createMesh`, and the assimp configuration macros
(`ASSIMP_AISCENE_HAS_SKELETONS`, `DART_ASSIMP_VERSION_MAJOR`). assimp then
links privately in shared builds and is exported only for static builds, as
urdfdom is since #3884. A build without assimp still accepts meshes built from
a `TriMesh` and reports a clear error for mesh files.

**Compatibility.**

- gz-physics' `CustomMeshShape` passes a null `aiScene*` to the deprecated
  `MeshShape` constructor, assembles an `aiScene` from a gz-common mesh and
  assigns it to the protected `mMesh` (all three releases). It reaches
  assimp's headers and library only through DART's core target. DART 7 owns a
  scene assigned that way, as DART 6 did (#3865).
- gz-sim uses no assimp API through DART.

**Prerequisites.**

- gz-physics builds `CustomMeshShape` with the `TriMesh` constructor in every
  release that DART 8 supports.
- DART's own code stops using `aiScene` and `aiMesh`. Today the SKEL, MJCF,
  SDF and URDF parsers load meshes through `MeshShape::loadMesh`; the FCL and
  Bullet detectors read soft meshes through `SoftMeshShape::getAssimpMesh`;
  and the Filament GUI reads `getMesh()` for the instance transforms, texture
  coordinates, vertex colors and materials that the `TriMesh` cache lacks
  (`dart/gui/detail/shape_descriptions.cpp`). DART's own mesh data carries
  those first.
- A 7.x release deprecates `SoftMeshShape::getAssimpMesh`,
  `AssimpInputResourceAdaptor`, `createFileIO` and the `aiScene` form of
  `createMesh`, and adds a `TriMesh` accessor in place of `getAssimpMesh`.

**Open questions.**

- Whether a build without assimp is a supported packaging configuration or
  only a minimal one, since every model format DART parses can reference mesh
  files, and whether parsers then fail the load or skip that geometry.
- Whether the mesh data that replaces `aiScene` for rendering extends
  `TriMesh` or is a separate DART type.

## FCL

**Motivation.**

- FCL has had no release since 0.7.0 (2021), and libccd, which it requires,
  none since 2018.
- Distributions differ in ways DART cannot fix. Ubuntu's FCL 0.7.0 keeps
  assertions and aborts in GJK on touching ellipsoids, and DART carries its
  fix only where it builds FCL itself (#3870). Fedora 44 and EPEL 10 retired
  FCL, so DART's core cannot build there.
- FCL's packages require octomap on conda-forge and Ubuntu, which makes
  octomap mandatory in practice.
- FCL reaches every core consumer. The core library links it publicly and
  lists it as a package dependency, the generated
  `dart/collision/collision.hpp` includes `fcl/fcl.hpp`, and
  `BackwardCompatibility.hpp`, `FCLTypes.hpp`, `CollisionShapes.hpp`,
  `FCLCollisionObject.hpp`, `FCLCollisionGroup.hpp` and a protected member of
  `FCLCollisionDetector` name FCL types. FCL is about a third of the core
  library's dependency closure (reported 37% of 27.9 MB).
- DART's native detector (`"dart"`) already provides collision, distance and
  raycasts. The C++ default stays FCL in 7.0 for Gazebo, and switching it is
  a labeled 7.x change.

**Design.** The header `dart/collision/fcl/FCLCollisionDetector.hpp`, the
class `dart::collision::FCLCollisionDetector`, `create()`, the `"fcl"` factory
key and type string, and the `PrimitiveShape` and
`ContactPointComputationMethod` settings stay. Without FCL, the class is a
facade over the native detector. Headers and members that name FCL types
leave the installed set after a 7.x deprecation. Observed semantics are what
callers see through the `CollisionDetector` interface and what DART's and
Gazebo's tests assert: the type string, the
[downstream contract](dart6_collision_backends.md#downstream-contract) for
contacts, contact caps, filters, distance results and ray support. FCL's
numbers are not reproduced bit for bit; each difference is measured against
real FCL on representative scenes and labeled.

**Compatibility.**

- gz-physics creates the detector with `FCLCollisionDetector::create()` when
  a world selects `"fcl"`, reports the detector's type to its users and tests
  that the type is `"fcl"`. It uses none of the FCL-typed headers.
- All three gz-physics releases test that `"fcl"` finds no ray hit. The same
  test broke when the `"dart"` detector gained raycasts (#3355), so the facade
  keeps `"fcl"` raycasts unsupported unless gz-physics changes that test.
- A facade changes shape support. FCL replaces capsules, heightmaps,
  multi-sphere shapes, point clouds and line segments with a small placeholder
  sphere, while the native detector collides capsules and skips heightmaps.
  FCL's default `PRIMITIVE` mode is analytic for ellipsoids and cones, which
  the native detector approximates with 162 vertices and a 64-segment rim
  (`dart/collision/dart/detail/NativeShapeConversion.cpp`).

**Prerequisites.**

- The native detector is the C++ default, after representative comparisons
  against FCL and the deformable-body coverage gate (PLAN-620).
- The native detector covers the shapes and queries that FCL serves today,
  qualified in the shape and query capability matrix that the DART 7 program
  publishes: ellipsoid and cone accuracy, voxel grids, soft meshes and
  mesh-primitive distance.
- A 7.x release deprecates the FCL-typed headers and members.

**Open questions.**

- Whether a build may still link real FCL behind the same class, for users
  who depend on FCL's numbers, or FCL leaves entirely.
- What `PrimitiveShape::MESH` and `ContactPointComputationMethod::FCL` mean
  in a facade: honored approximately, accepted and ignored, or deprecated.
- Whether `"fcl"` gains raycasts together with a gz-physics test update.

## Bullet and ODE

**Motivation.**

- Bullet has had no release since 3.25 (2022). Its scalar type is an ABI
  choice: DART probes it at configure and prefers the double-precision
  libraries since #3868, and one process must not load both precisions, which
  constrains every application that also loads Bullet.
- ODE needs double precision as well (#3900), and its global initialization
  is not thread-safe, which gz-physics 9.5.2 works around with a mutex.
- Both expose raw types through installed headers: `BulletInclude.hpp`,
  `BulletCollisionGroup.hpp` (`btCollisionWorld`), `OdeCollisionDetector.hpp`
  (`dWorldID`), `OdeCollisionGroup.hpp` (`dSpaceID`) and `OdeTypes.hpp`.
- Their fetch fallbacks needed repair to build under CMake 4 (#3900).

**Design.** DART adds the ray query that gz-physics needs to its collision
API: the closest hit for each ray in a batch, written into a caller-owned
buffer so repeated batches do not allocate, with hit or no hit reported as
`RaycastResult` does. gz-physics maps a miss to its own value (NaN in 9.5.2).
While Bullet and ODE are still real, DART serves the query through Bullet's
collision world, ODE's ray geometry and the native narrow phase. In 7.x the
query is additive: it adds no virtual function to the classes whose vtables
gz-physics relies on (see
[performance compatibility](../onboarding/architecture.md#performance-compatibility)),
and it leaves what `raycast` reports for each existing key unchanged. In
DART 8, `BulletCollisionDetector` and `OdeCollisionDetector` become facades
over the native detector. Their headers, classes, `create()`, `"bullet"` and
`"ode"` keys, `collision-bullet` and `collision-ode` components,
collision-group classes and virtual `collide`, `raycast` and
`createCollisionGroup` stay. Accessors that return raw Bullet or ODE objects
leave after a 7.x deprecation, and the opt-in Bullet and ODE fetch paths
retire with the libraries.

**Compatibility.**

- gz-physics 9.5.2 casts single and batched rays itself: through
  `btCollisionWorld::rayTest` on the world from the protected
  `getBulletCollisionWorld()`, and through `dCreateRay`, `dSpaceCollide2` and
  `dCollide` on the space from the protected `getOdeSpaceId()`. A facade has
  neither object, so that code moves to DART's ray query first.
- Ray support differs by release. gz-physics 7.8.0 and 8.4.0 test `"bullet"`
  as supported and `"ode"` as unsupported; 9.5.2 tests both as supported, for
  single and batched rays. A `"bullet"` facade matches Bullet's hit point,
  normal and fraction within gz-physics' tolerances. The native raycast
  already passes gz-physics' analytic-sphere assertions once the test lists
  `"dart"` as supported (`tools/gazebo/patches/`).
- ODE is Gazebo's default detector, and gz-physics' tests already relax some
  dartsim expectations for ODE's box-cylinder contacts on macOS
  (gazebosim/gz-physics#620). An ODE facade changes Gazebo's default
  contacts, so it is compared on Gazebo worlds, not only on DART's tests.
- gz-physics creates heightmaps, which ODE and Bullet collide and the native
  detector does not support yet.

**Prerequisites.**

- The ray query in a 7.x release, and gz-physics releases that use it instead
  of raw Bullet and ODE objects.
- Native heightmap collision, and native contacts compared with Bullet's and
  ODE's on representative Gazebo worlds, each difference labeled.
- A 7.x release deprecates the raw-typed headers and accessors.

**Open questions.**

- Whether ODE becomes a facade in DART 8 or stays a real, optional backend
  until Gazebo changes its default detector.
- Whether DART's own `"ode"` gains raycasts, which gz-physics 7.8.0 and 8.4.0
  test as unsupported.

## tinyxml2

**Motivation.**

- `dart/utils/XmlHelpers.hpp` includes `<tinyxml2.h>`, and its element,
  attribute and file helpers take or return tinyxml2 types. It and
  `detail/XmlHelpers-impl.hpp` are installed, and the generated
  `dart/utils/utils.hpp` includes it. `dart-io` therefore links tinyxml2
  publicly and lists it as a package dependency of the `io` component, so
  every gz-physics configure needs tinyxml2's CMake package through `utils`
  and `utils-urdf`.
- tinyxml2 changes its major version, and with it its shared-library version,
  about once per Ubuntu LTS: 9.0.0, 10.0.0 and 11.0.0 on 22.04, 24.04 and
  26.04.
- The library stays either way. urdfdom and sdformat link a shared tinyxml2
  themselves, so a private copy in DART would be the same library or a second
  tinyxml2 ABI in one process. The gain is the interface only, as for
  urdfdom's types in #3884 and sdformat in #3889.

**Design.** The tinyxml2-typed helpers move to a non-installed header, and
`dart-io` links tinyxml2 privately in shared builds and exports it only for
static builds.

**Compatibility.**

- gz-physics and gz-sim call no `XmlHelpers` function. gz-physics' DART test
  plugin includes `dart/utils/utils.hpp` and uses only `DartLoader`, and
  gz-sim finds tinyxml2 itself.
- Code that relied on `utils.hpp` to include `<tinyxml2.h>` must include it
  itself.

**Prerequisites.** A 7.x release deprecates the tinyxml2-typed helpers.
PLAN-625 does not include this deprecation, so it needs maintainer approval.

**Open questions.** Whether the string-conversion helpers in `XmlHelpers.hpp`
(`toString`, `toVector3d` and similar), which name no tinyxml2 type, stay
installed.

## The 7.0 deprecations

DART 8 removes the C++ APIs that DART 7 deprecates, or carries deprecated,
for removal:

- the `aiScene` and `aiMesh` API (see [assimp](#assimp));
- the deprecated names that DART declares in namespace `Eigen`:
  `EIGEN_V_VEC3D`, `EIGEN_VV_VEC3D`, `aligned_vector`, `aligned_map` and
  `make_aligned_shared`;
- `dart/utils/urdf/BackwardCompatibility.hpp`, its `.in` template and
  `URDFTypes.hpp`, which gz-physics and gz-sim do not use;
- the build options kept as no-ops through 7.x, `DART_SKIP_spdlog` and
  `DART_USE_SYSTEM_FMT`;
- the FCL-, Bullet-, ODE-, tinyxml2- and octomap-typed APIs in this document,
  once their 7.x deprecations have shipped.

**Rules.**

- An API leaves only when no gz-physics or gz-sim release that DART 8
  supports uses it, checked by building those releases. Deprecated APIs that
  Gazebo still uses stay until it migrates: `DART_VERSION_AT_LEAST`,
  deprecated since DART 6.14, in gz-physics and gz-sim, and the `aiScene`
  constructor in gz-physics.
- `Eigen::Vector6d` and `Eigen::Matrix6d` are not deprecated and stay:
  gz-physics' `JointFeatures.cc` uses `Eigen::Vector6d`.
- Each removal gets a breaking-change changelog entry that names its
  replacement.

**Open question.** Whether DART 8 also removes the deprecations inherited
from DART 6 under the same rules; this record covers only those listed above.

## C++20 interface

**Motivation.** DART 7 exports `cxx_std_17`, even where its own targets build
as C++20, keeps installed headers C++17-clean and keeps the
`ContactSurface.hpp` include tree parseable as C++14. DART's API therefore
cannot use C++20, and installed headers carry about 300 lines of C++20
polyfills (reported), such as `dart/collision/dart/detail/Span.hpp`.

**Design.** DART 8 exports `cxx_std_20`, installed headers may use C++20, and
the polyfills go.

**Compatibility.**

- gz-physics sets `CMAKE_CXX_STANDARD 17` and probes `ContactSurface.hpp` as
  C++14 or C++17 (see [Shared constraints](#shared-constraints)), so a
  C++20-only construct in the probe's include tree silently disables Gazebo's
  contact-surface features. On MSVC, DART's own copy of the probe compiles as
  C++17, because MSVC rejects in C++14 the C++17 syntax that Unix compilers
  accept as an extension. Whether gz-physics' probe passes on Windows depends
  on which of `/std:c++14` and the CMP0067 standard flag MSVC applies; it is
  untested, and Windows has no Gazebo lane.
- Exporting `cxx_std_20` raised 23 gz-physics compile commands per lane to
  C++20, with the probe still passing and no new Harmonic or Jetty failures
  once a load-related Jetty timeout passed on rerun (2026-10-18, `main`
  `17ce7eefa`). Consumers that cannot follow the exported standard break,
  among them gz-physics' Bazel build and C++17 pkg-config users.
- GCC releases before 16 ship parts of the C++20 standard library with an ABI
  that can still change, so exporting C++20 earlier puts that ABI into DART's
  interface. DART code avoids the affected components until its GCC floor
  reaches 16. Ubuntu 26.04 LTS ships GCC 15.2.

**Prerequisites.**

- gz-physics probes at the standard that DART exports, or checks DART's
  version instead of compiling the header, in every release that DART 8
  supports, and its Bazel and pkg-config builds accept C++20.
- A GCC 16 floor, with matching Clang, AppleClang and MSVC floors.
- DART's installed-header checks move from C++17 to C++20, and its C++14
  probe check retires with gz-physics' change.

**Open questions.**

- The compiler and platform floors that pair with GCC 16.
- Whether DART 8 keeps the C++17 interface if gz-physics cannot change its
  probe in time.

## Native voxel grid

**Motivation.**

- octomap is optional in DART's CMake but required in practice, because
  FCL's packages require it.
- Its types are in DART's API: `VoxelGridShape`'s octree constructor,
  `setOctree()`, `getOctree()` and point-cloud `updateOccupancy()` overloads,
  and `PointCloudShape`'s `setPoints()` and `addPoints()` overloads.
- octomap 1.10.0 and older warn under C++20 with libstdc++ 15, and octomap
  bounds its 100,000-key ray buffer only with an assertion; DART has checked
  ray length itself since #3859.
- Both detectors that collide voxel grids read the octree: FCL wraps it, and
  the native detector converts its leaves into a compound shape.

**Design.** `VoxelGridShape` stores a DART-native sparse voxel grid that keeps
its model of resolution, occupancy probabilities and ray insertion, and the
native detector and the FCL facade read it directly. The octree-typed APIs
leave after a 7.x deprecation with DART-typed replacements, which dartpy
binds as their Python surface.

**Compatibility.** gz-physics and gz-sim use neither `VoxelGridShape` nor
`PointCloudShape`; the constraint is DART's own C++ and Python users.

**Prerequisites.**

- FCL is optional (see [FCL](#fcl)). While FCL is required, a native grid
  removes no package and adds an estimated 600 to 1,000 owned lines.
- A benchmark and accuracy comparison against OctoMap for ray insertion,
  occupancy and memory, with tolerances, since results are not expected to be
  bit-identical.
- A 7.x release deprecates the octree-typed APIs. PLAN-625 does not include
  this deprecation, so it needs maintainer approval.

**Open questions.**

- Whether octomap remains as an optional import and export adapter.
- The acceptance tolerances for occupancy and contacts against OctoMap.

## Non-goals

- In-house mesh format readers; assimp stays the importer.
- Replacing urdfdom or sdformat. SDF stays an optional part of `dart-io`
  unless new evidence favors a separate library, which the `io-sdf` component
  name already allows.
- Changing gz-physics or gz-sim from DART's side. Each upstream proposal is
  drafted separately and needs approval.
- C++23 and C++ modules.

## Evidence owners

- Roadmap state and sequencing: PLAN-625 and PLAN-620 in the
  [plan dashboard](../plans/dashboard.md).
- The Gazebo surface: the gz API inventory, the Gazebo lanes in
  [`tools/gazebo/README.md`](../../tools/gazebo/README.md) and the CTest
  gates in [testing](../onboarding/testing.md#gazebo-extension-gates-in-ctest).
- The collision-backend lifecycle and downstream contract:
  [DART 6 collision backends](dart6_collision_backends.md).
