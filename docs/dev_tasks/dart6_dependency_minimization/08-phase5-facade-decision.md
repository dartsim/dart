# Phase 5 — backend consolidation and later-release facade proposal

> Decision doc (v2, 2026-07-10). Evidence: source-verified gz usage surface
> (`.deps/gz-physics/dartsim/src/`), the phase-0 acceptance envelope
> ([05-phase0-baseline-packet.md](05-phase0-baseline-packet.md)), and the
> 2026-07-10 audit snapshot. Its `/tmp/audit_head_20260710T011207Z` directory
> was session-local and is not durable evidence; reverify gz source and all
> acceptance gates on the eventual target parent. Maintainer ratification points
> are listed at the end.
>
> **Executed (2026-07-11):** Decision 1 below was implemented in the
> detector-consolidation branch — `dart/collision/native/` folded into
> `dart/collision/dart/`, `NativeCollisionDetector`/`Group`/`Object` renamed
> to `DARTCollisionDetector`/`Group`/`Object`, canonical key `"dart"`. The
> unreleased `"native"` key is removed rather than retained as an alias. The
> legacy narrowphase-only implementation is deleted, while the released
> `DARTCollide.{hpp,cpp}` API remains as thin wrappers over the consolidated
> detector.
>
> **Reverted (2026-07-23):** the default flip referenced above was
> reverted per maintainer direction; PR #3381 ultimately shipped the
> consolidation alone, and the built-in default remains **`fcl`**. The
> flip is deferred beyond DART 6.20.
>
> **Merged (2026-07-29):** PR #3381 final head
> `64d476b68ad5ae0dcca4e98abb9bba15b6962b87` merged to `release-6.20` as
> `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`. Decision 1 is implemented.
> The remaining actions are maintainer ratification points 1 and 4: the ODE/gz
> compatibility approach and the target later-release sequence. This document
> does not authorize a 6.20 default flip or dependency removal.

## Goal restated

The consolidation is complete. The unratified follow-up proposal is to
deprecate the FCL, Bullet, and ODE collision backends after an accepted default
flip, then remove their external dependencies in a later release while
preserving the downstream gz-physics/gz-sim contract. The 6.21/6.22 labels
below are planning examples, not an approved schedule.

## Decision 1 — canonical backend and naming (maintainer-directed, 2026-07-10)

**The native engine merges INTO the `dart` detector: `dart/collision/native/`
folds into `dart/collision/dart/`, and `NativeCollisionDetector` merges into
`DARTCollisionDetector`, replacing its legacy collision implementation. The
single built-in backend is the `dart` detector, canonical factory key `"dart"`.
The keys `"fcl"`, `"bullet"`, `"ode"` remain resolvable and, in the proposed
eventual dependency-removal release, create dart-backed facade
implementations.**

Maintainer direction (2026-07-10): "NativeDetector must merged into
DartDetector (so native/ into dart/ as well)."

Mechanics, and why this is clean now:

- 6.20.0 is unreleased, so the interim `"native"` factory key, the
  `NativeCollisionDetector` class name, and its dartpy binding have never
  shipped; the consolidation PR renames/folds them with no deprecation cycle.
  All in-tree users are updated to `"dart"` in the same PR; no `"native"`
  selector or factory alias remains.
- Before #3381, the incumbent `DARTCollisionDetector` had six primitive
  narrowphase pairs, an in-detector AABB sweep broadphase, a `distance()` stub,
  and no raycast. The merged consolidated engine deliberately replaced and
  re-baselined that behavior while extending the `"dart"` key's shape and query
  capabilities; the PR's recorded old/new guard rows are historical evidence.
- gz-physics keeps working: `SetWorldCollisionDetector("dart")` returns the
  consolidated engine; the other names keep resolving. Under the facade
  proposal, gz's own default remains its `GzOdeCollisionDetector` subclass
  until the approved transition (`EntityManagementFeatures.cc:728` in the
  audited snapshot).
- No new enum value is needed: `CollisionDetectorType::Dart` and the dartpy
  `DARTCollisionDetector` binding now denote the consolidated engine, and the
  phase-6 flip simply changes the default from Fcl to Dart across the flip
  surface (`ConstraintSolver.cpp:416`/`:433`, `WorldConfig`, `SkelParser`).

> **Correction recorded during execution (2026-07-11):** the "strict subset"
> premise above does not hold for `SoftMeshShape` (deformable body) contacts.
> The legacy narrowphase-only `DARTCollisionDetector` supported `SoftMeshShape`
> point contacts (against Box/Plane/Sphere/Ellipsoid/SoftMesh, matching
> FCL/Bullet/ODE); the native engine's shape conversion has no `SoftMeshShape`
> case. Deleting the legacy engine therefore drops `SoftMeshShape` support
> from the `"dart"` key until a follow-up ports it into the native
> shape/narrowphase layer. See the consolidation commit message for the full
> enumeration of affected call sites.
>
> **Resolution (2026-07-11, same branch):** both capability gaps closed
> before the flip PR:
> - `SoftMeshShape` ported into the consolidated detector (`SoftCollision.*`,
>   soft caches on `DARTCollisionObject`, soft AABBs in both broadphases;
>   soft pairs bypass the persistent manifold cache). All eight soft
>   StepAllocation gates report contacts with zero steady-state allocations.
>   Parallel soft-soft batching is deferred (serial path only, TODO in
>   source).
> - `EllipsoidShape` support added to the native shape conversion (exact
>   sphere for equal radii, deterministic icosphere convex hull otherwise).
>
> Rigid determinism guards stayed bit-identical throughout (S1
> `0xd6736cd716faf01d`, S3 `0x6088ea0177efa6a`, S4 `0x55bf77ebc1c491b2`,
> S5 `0x4f265a803b596035`).

## Proposal 2 — facades over the dart detector, not component removal

**The proposed dependency-removal release drops the external fcl/bullet/ode
dependencies while detector classes and CMake components survive as
compatibility facades over the consolidated `dart` detector.** This approach
and its target release are not yet ratified. Dependency removal is gated on
migrating installed public detector headers and API types so they no longer
include or expose FCL, Bullet, or ODE headers. Header-only downstream compile
checks and component smoke tests must pass without those packages installed.
The source-verified gz obligations and how facades would satisfy them:

| gz obligation (evidence) | facade answer |
| --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode ...)` (`gz-physics/CMakeLists.txt:74-84`) | components remain, containing facade libs with no external dep |
| `GzOdeCollisionDetector : public OdeCollisionDetector`, overrides `collide()x2`, adds per-pair capping (`GzOdeCollisionDetector.hh:26-69`) | `OdeCollisionDetector` stays a real subclassable class whose `collide()` delegates to the dart engine and honors `CollisionOption.maxNumContactsPerPair` |
| keys "fcl"/"bullet"/"ode"/"dart" resolve via `create()` (`WorldFeatures.cc:47-70`) | facade factories keep all four registrations |
| `getLastCollisionResult()` Contact fields {point, normal, penetrationDepth, force, collisionObject1/2} (`SimulationFeatures.cc:213-256`) | native populates the same DART 6 `Contact` (proven by adapter parity tests) |
| raycast (Bullet incumbent, `SimulationFeatures.cc:176-185`) | native raycast merged (#3355) |
| distance (FCL incumbent) | native distance merged (#3343/#3352) |
| VoxelGrid/octree (FCL incumbent) | native compound voxel support merged (#3358) |

Notes: gz subclasses **only** `OdeCollisionDetector` (there is no
`GzBulletCollisionDetector`; the doc-03 matrix row overstated this). Bullet
and FCL only need `create()` + name resolution + `getType()`.

Behavior disclosure: facades would emit the dart engine's contact profiles
(counts/normals may differ from real FCL/Bullet/ODE). The eventual migration
release must document that semantic, provide a ratified deprecation window, and
pass the gz gate against facades before dependency removal ships.

## Mechanics facts that shape the implementation

- The two FCL-hardcoded ConstraintSolver ctors are at
  `dart/constraint/ConstraintSolver.cpp:416` and `:433` (doc 03's 322/342 was
  stale). The phase-6 flip surface also includes `WorldConfig` (`World.hpp:103`
  defaults to Fcl) and `SkelParser`'s FCL helpers
  (`dart/utils/SkelParser.cpp:718-731`). 25 code files reference
  `FCLCollisionDetector` outside `dart/collision/fcl/` (5 core, 2 dartpy,
  18 tests/examples/scripts).
- Component asymmetry: the consolidated `collision/dart` implementation and FCL
  compile **into core libdart**; `collision/native` no longer exists, while
  `collision-bullet`/`collision-ode` are separate exported components. Thus a
  future bullet/ode drop is component-level, while the FCL drop is core surgery:
  `target_link_libraries(dart PUBLIC ... fcl ...)` (`dart/CMakeLists.txt:117`),
  `DART_PKG_EXTERNAL_DEPS` (`CMakeLists.txt:103` -> `dart.pc` Requires), and
  `dart_check_required_package(fcl)` (`cmake/DARTFindDependencies.cmake:21-22`).
- `DART_DEPRECATED(version)` ignores its argument and carries no message
  (`dart/common/Deprecated.hpp:44-53`). A future deprecation release should use
  `[[deprecated("...use the dart collision detector...")]]` (or a new
  `DART_DEPRECATED_MESSAGE`) on `create()`/constructors — NOT on the classes
  gz subclasses; warning-cleanliness of `GzOdeCollisionDetector` under
  -Werror must be prototyped before that release ships.
- SOVERSION is `MAJOR.MINOR` (`cmake/DARTMacros.cmake:94`): every minor gets
  a new SONAME, so converting classes to facades in a later minor is
  ABI-permissible; the constraint is source/API compatibility plus the gz gate.

## Proposed timeline (unratified beyond 6.20)

- **6.20:** ship detector consolidation only. Keep FCL as the built-in default,
  keep FCL/Bullet/ODE implementations and package components real, and prove
  their paths are structurally unchanged and free of runtime regression. No
  collision-backend deprecations.
- **Later default-flip release:** only after the full acceptance packet and
  maintainer approval, change both `ConstraintSolver` constructors plus the
  WorldConfig/SkelParser surface.
- **Proposed deprecation release, after an accepted default flip:** attributes
  with migration messages on FCL/Bullet/ODE `create()`/ctors; CHANGELOG +
  migration guide; CMake configure-time notices. Everything remains
  functional.
- **Proposed following release:** drop external fcl/bullet/ode from the required
  surface; classes become facades over the dart detector; FCL is decoupled from
  core (`phase 7`), bullet/ode components are rebuilt as facade components, and
  package/export smoke tests pass.

## Maintainer ratification points

1. Facade-over-dart for `OdeCollisionDetector` vs coordinating a gz-physics
   change that drops the `GzOdeCollisionDetector` subclass first (facade is
   the recommended default; either path must keep the gz gate green at the
   approved transition).
2. ~~Canonical name~~ RESOLVED by maintainer direction 2026-07-10: canonical
   `"dart"`, with `dart/collision/native/` merged into `dart/collision/dart/`
   and `NativeCollisionDetector` merged into `DARTCollisionDetector`.
3. (retired) The legacy `DARTCollisionDetector` engine question is subsumed
   by the consolidation: its narrowphase is replaced in 6.20, not deprecated.
4. Target release sequence: ratify which later release owns the default flip,
   how long deprecation lasts, and which following release may remove external
   dependencies. The historical 6.21/6.22 labels are proposal placeholders.
