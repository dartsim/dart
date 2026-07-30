# Phase 5 — backend consolidation and later facade lifecycle

> Decision doc (v3, refreshed 2026-07-30). Evidence: source-verified gz usage
> surface
> (`.deps/gz-physics/dartsim/src/`), the phase-0 acceptance envelope
> ([05-phase0-baseline-packet.md](05-phase0-baseline-packet.md)), and the
> 2026-07-10 current-head audit (`/tmp/audit_head_20260710T011207Z`).
> Maintainer ratification points are listed at the end.
>
> **Merged (2026-07-30):** Decision 1 below was implemented by
> [PR #3381](https://github.com/dartsim/dart/pull/3381), exact reviewed head
> `64d476b68a`, squash merge `46719bfbd75`. `dart/collision/native/` folded into
> `dart/collision/dart/`, `NativeCollisionDetector`/`Group`/`Object` renamed
> to `DARTCollisionDetector`/`Group`/`Object`, canonical key `"dart"`. The
> unreleased `"native"` key is removed rather than retained as an alias. The
> legacy narrowphase-only implementation is deleted, while the released
> `DARTCollide.{hpp,cpp}` API remains as thin wrappers over the consolidated
> detector.
>
> **DART 6.20 boundary:** the earlier default-flip candidate was reverted per
> maintainer direction. PR #3381 shipped consolidation alone; the built-in
> default remains **`fcl`**, and the FCL/Bullet/ODE implementations,
> components, dependencies, and selection paths remain real and unchanged.
> The default flip and facade implementation are deferred beyond DART 6.20.
>
> Durable current-state rationale:
> [DART 6 Collision Backend Consolidation](../../design/dart6_collision_backend_consolidation.md).

## Goal restated

Consolidate DART's built-in collision detection into one backend on DART 6.20,
then, only after an accepted later-release default flip, deprecate and
eventually remove the external FCL, Bullet, and ODE dependencies while
preserving the downstream gz-physics/gz-sim contract. The former 6.21/6.22
labels describe the intended sequence, not branches or milestones that
currently exist.

## Decision 1 — canonical backend and naming (maintainer-directed, 2026-07-10)

**The native engine merges INTO the `dart` detector: `dart/collision/native/`
folds into `dart/collision/dart/`, and `NativeCollisionDetector` merges into
`DARTCollisionDetector`, replacing its legacy collision implementation. The
single built-in backend is the `dart` detector, canonical factory key `"dart"`.
The keys `"fcl"`, `"bullet"`, `"ode"` remain resolvable
and may become dart-backed facade implementations only in a later removal
release.**

Maintainer direction (2026-07-10): "NativeDetector must merged into
DartDetector (so native/ into dart/ as well)."

Mechanics, and why this is clean now:

- 6.20.0 is unreleased, so the interim `"native"` factory key, the
  `NativeCollisionDetector` class name, and its dartpy binding have never
  shipped; the consolidation PR renames/folds them with no deprecation cycle.
  All in-tree users are updated to `"dart"` in the same PR; no `"native"`
  selector or factory alias remains.
- Before consolidation, the incumbent `DARTCollisionDetector` had six
  primitive narrowphase pairs, an in-detector AABB sweep broadphase (including
  plane pruning and parallel scratch paths), a `distance()` stub, and no
  raycast. The consolidated engine preserved or deliberately replaced and
  re-baselined that behavior while extending the `"dart"` key's shape and
  query capabilities. The consolidation PR's A/B evidence records the allowed
  pre-release `dart`-detector re-baseline.
- gz-physics keeps working: `SetWorldCollisionDetector("dart")` returns the
  consolidated engine; the other names keep resolving; gz's own default
  remains its `GzOdeCollisionDetector` subclass through DART 6.20 and any
  real-backend deprecation period
  (`EntityManagementFeatures.cc:728`).
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
> **Resolution (2026-07-11, same branch):** both capability gaps closed before
> the consolidation merged:
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

## Decision 2 — facades over the dart detector, not component removal

**A later removal release may remove the external fcl/bullet/ode dependencies;
the detector classes and CMake components survive as compatibility facades
over the consolidated `dart` detector.** This is a ratified design direction,
not DART 6.20 implementation state. Dependency removal is gated on migrating
the installed public detector headers and API types so they no longer include
or expose FCL, Bullet, or ODE headers. Header-only downstream compile checks
and component smoke tests must pass without those packages installed. The
source-verified gz obligations and how facades satisfy them:

| gz obligation (evidence) | facade answer |
| --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode ...)` (`gz-physics/CMakeLists.txt:74-84`) | components remain, containing facade libs with no external dep |
| `GzOdeCollisionDetector : public OdeCollisionDetector`, overrides `collide()x2`, adds per-pair capping (`GzOdeCollisionDetector.hh:26-69`) | `OdeCollisionDetector` stays a real subclassable class whose `collide()` delegates to the dart engine and honors `CollisionOption.maxNumContactsPerPair` |
| keys "fcl"/"bullet"/"ode"/"dart" resolve via `create()` (`WorldFeatures.cc:47-70`) | facade factories keep all four registrations |
| `getLastCollisionResult()` Contact fields {point, normal, penetrationDepth, force, collisionObject1/2} (`SimulationFeatures.cc:213-256`) | the consolidated detector populates the same DART 6 `Contact` (proven by adapter parity tests) |
| raycast (Bullet incumbent, `SimulationFeatures.cc:176-185`) | native raycast merged (#3355) |
| distance (FCL incumbent) | native distance merged (#3343/#3352) |
| VoxelGrid/octree (FCL incumbent) | native compound voxel support merged (#3358) |

Notes: gz subclasses **only** `OdeCollisionDetector` (there is no
`GzBulletCollisionDetector`; the doc-03 matrix row overstated this). Bullet
and FCL only need `create()` + name resolution + `getType()`.

Behavior disclosure: facades emit the dart engine's contact profiles
(counts/normals may differ from real FCL/Bullet/ODE). That is the documented
semantic of the eventual removal release; the preceding deprecation release is
the migration window, and the gz gate must pass against facades before removal
ships.

## Mechanics facts that shape the implementation

- The two FCL-hardcoded ConstraintSolver ctors are at
  `dart/constraint/ConstraintSolver.cpp:416` and `:433` (doc 03's 322/342 was
  stale). The phase-6 flip surface also includes `WorldConfig` (`World.hpp:103`
  defaults to Fcl) and `SkelParser`'s FCL helpers
  (`dart/utils/SkelParser.cpp:718-731`). At the 2026-07-10 audit, 23 files
  referenced `FCLCollisionDetector` outside `dart/collision/fcl/` (5 core, 2
  dartpy, ~16 tests/examples); refresh that inventory on the future flip
  parent.
- Component asymmetry: before consolidation, `collision/fcl`,
  `collision/dart`, and `collision/native` all compiled **into core libdart**
  (`dart_add_core_headers/sources`); #3381 folded the last of those into
  `collision/dart`.
  `collision-bullet`/`collision-ode` are separate exported components. So the
  later Bullet/ODE drop is component-level, while the FCL drop is core surgery:
  `target_link_libraries(dart PUBLIC ... fcl ...)` (`dart/CMakeLists.txt:117`),
  `DART_PKG_EXTERNAL_DEPS` (`CMakeLists.txt:103` -> `dart.pc` Requires), and
  `dart_check_required_package(fcl)` (`cmake/DARTFindDependencies.cmake:21-22`).
- `DART_DEPRECATED(version)` ignores its argument and carries no message
  (`dart/common/Deprecated.hpp:44-53`). Future deprecations should use
  `[[deprecated("...use the dart collision detector...")]]` (or a new
  `DART_DEPRECATED_MESSAGE`) on `create()`/constructors — NOT on the classes
  gz subclasses; warning-cleanliness of `GzOdeCollisionDetector` under
  -Werror must be prototyped before the deprecation release ships.
- SOVERSION is `MAJOR.MINOR` (`cmake/DARTMacros.cmake:94`): every minor gets
  a new SONAME, so converting classes to facades in later minor releases is
  ABI-permissible; the constraint is source/API compatibility plus the gz
  gate.

## Timeline

- **6.20 (complete):** ship detector consolidation only. Keep FCL as the
  built-in default,
  keep FCL/Bullet/ODE implementations and package components real, and prove
  their paths are structurally unchanged and free of runtime regression. PR
  #3381 provides that result. No collision-backend deprecations.
- **Later default-flip release:** only after the full acceptance packet and
  maintainer approval, change both `ConstraintSolver` constructors plus the
  WorldConfig/SkelParser surface.
- **Deprecation release, after an accepted default flip:** deprecation
  attributes
  with migration messages on FCL/Bullet/ODE `create()`/ctors; CHANGELOG +
  migration guide; CMake configure-time notices. Everything remains
  functional.
- **Removal release:** drop external fcl/bullet/ode from the required surface;
  classes
  become facades over the dart detector; FCL decoupled from core
  (`phase 7`), bullet/ode components rebuilt as facade components; package/
  export smoke tests.

## Maintainer ratification points

1. Facade-over-dart for `OdeCollisionDetector` vs coordinating a gz-physics
   change that drops the `GzOdeCollisionDetector` subclass first (facade is
   the recommended default; either route must keep the gz gate green in the
   removal release).
2. ~~Canonical name~~ RESOLVED by maintainer direction 2026-07-10: canonical
   `"dart"`, with `dart/collision/native/` merged into `dart/collision/dart/`
   and `NativeCollisionDetector` merged into `DARTCollisionDetector`.
3. (retired) The legacy `DARTCollisionDetector` engine question is subsumed
   by the consolidation: its narrowphase is replaced in 6.20, not deprecated.

As of 2026-07-30, only `release-6.19` and `release-6.20` exist remotely, and
the only open DART 6 milestone is `DART 6.20.0`. No future facade or
dependency-removal implementation should begin until a proposing release and
milestone exist and a maintainer explicitly authorizes the packet.
