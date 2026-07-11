# Phase 5 — backend consolidation, facades, and the 6.21/6.22 deprecation plan

> Decision doc (v2, 2026-07-10). Evidence: source-verified gz usage surface
> (`.deps/gz-physics/dartsim/src/`), the phase-0 acceptance envelope
> ([05-phase0-baseline-packet.md](05-phase0-baseline-packet.md)), and the
> 2026-07-10 current-head audit (`/tmp/audit_head_20260710T011207Z`).
> Maintainer ratification points are listed at the end.

## Goal restated

Consolidate DART's built-in collision detection into one backend; deprecate
the FCL, Bullet, and ODE collision backends in DART 6.21.0 and remove them in
DART 6.22.0, while preserving the downstream gz-physics/gz-sim contract.

## Decision 1 — canonical backend and naming (maintainer-directed, 2026-07-10)

**The native engine merges INTO the `dart` detector: `dart/collision/native/`
folds into `dart/collision/dart/`, and `NativeCollisionDetector` merges into
`DARTCollisionDetector`, replacing its legacy narrowphase-only
implementation. The single built-in backend is the `dart` detector, canonical
factory key `"dart"`. The keys `"fcl"`, `"bullet"`, `"ode"` remain resolvable
and, from 6.22, create dart-backed facade implementations.**

Maintainer direction (2026-07-10): "NativeDetector must merged into
DartDetector (so native/ into dart/ as well)."

Mechanics, and why this is clean now:

- 6.20.0 is unreleased, so the interim `"native"` factory key, the
  `NativeCollisionDetector` class name, and its dartpy binding have never
  shipped; the consolidation PR renames/folds them with no deprecation cycle.
  All in-tree users are updated in the same PR (`contact_benchmark
  --collision native`, dashboard detector index 4, native unit tests, the
  comparison harness's `--detector native`, the dartpy binding).
- The legacy `DARTCollisionDetector` narrowphase (six primitive pairs, no
  broadphase, `distance()` stub, no raycast) is a strict subset of the engine
  replacing it, so the `"dart"` key only gains capability. `dart`-detector
  guard rows re-baseline with the consolidation PR's A/B evidence
  (pre-release change, allowed with recorded old/new rows).
- gz-physics keeps working: `SetWorldCollisionDetector("dart")` returns the
  consolidated engine; the other names keep resolving; gz's own default
  remains its `GzOdeCollisionDetector` subclass until 6.22
  (`EntityManagementFeatures.cc:728`).
- No new enum value is needed: `CollisionDetectorType::Dart` and the dartpy
  `DARTCollisionDetector` binding now denote the consolidated engine, and the
  phase-6 flip simply changes the default from Fcl to Dart across the flip
  surface (`ConstraintSolver.cpp:416`/`:433`, `WorldConfig`, `SkelParser`).

## Decision 2 — facades over the dart detector, not component removal

**6.22 removes the external fcl/bullet/ode dependencies; the detector classes
and CMake components survive as compatibility facades over the consolidated
`dart` detector.** The source-verified gz obligations and how facades satisfy them:

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

Behavior disclosure: facades emit the dart engine's contact profiles (counts/normals may
differ from real FCL/Bullet/ODE). That is the documented semantic of the 6.22
removal; 6.21's deprecation cycle is the migration window, and the gz gate
must pass against facades before 6.22 ships.

## Mechanics facts that shape the implementation

- The two FCL-hardcoded ConstraintSolver ctors are at
  `dart/constraint/ConstraintSolver.cpp:416` and `:433` (doc 03's 322/342 was
  stale). The phase-6 flip surface also includes `WorldConfig` (`World.hpp:103`
  defaults to Fcl) and `SkelParser`'s FCL helpers
  (`dart/utils/SkelParser.cpp:718-731`). 23 files reference
  `FCLCollisionDetector` outside `dart/collision/fcl/` (5 core, 2 dartpy,
  ~16 tests/examples).
- Component asymmetry: `collision/fcl`, `collision/dart`, `collision/native`
  compile **into core libdart** (`dart_add_core_headers/sources`);
  `collision-bullet`/`collision-ode` are separate exported components. So the
  6.22 bullet/ode drop is component-level, while the FCL drop is core surgery:
  `target_link_libraries(dart PUBLIC ... fcl ...)` (`dart/CMakeLists.txt:117`),
  `DART_PKG_EXTERNAL_DEPS` (`CMakeLists.txt:103` -> `dart.pc` Requires), and
  `dart_check_required_package(fcl)` (`cmake/DARTFindDependencies.cmake:21-22`).
- `DART_DEPRECATED(version)` ignores its argument and carries no message
  (`dart/common/Deprecated.hpp:44-53`). 6.21 deprecations should use
  `[[deprecated("...use the dart collision detector...")]]` (or a new
  `DART_DEPRECATED_MESSAGE`) on `create()`/constructors — NOT on the classes
  gz subclasses; warning-cleanliness of `GzOdeCollisionDetector` under
  -Werror must be prototyped before 6.21 ships.
- SOVERSION is `MAJOR.MINOR` (`cmake/DARTMacros.cmake:94`): every minor gets
  a new SONAME, so converting classes to facades in 6.21/6.22 is
  ABI-permissible; the constraint is source/API compatibility plus the gz
  gate.

## Timeline

- **6.20 (now, phases 4-6 of the port plan):** close the remaining native
  performance gaps (broadphase landed; S3 narrowphase delta and small-scene
  overhead remain; the S6 dense-pile row was resolved by the documented
  acceptance re-scope), then flip only the DEFAULT detector (both ctors +
  WorldConfig/SkelParser surface). All legacy detectors stay real and
  selectable; no deprecations on the LTS branch.
- **6.21:** deprecation attributes with migration messages on
  FCL/Bullet/ODE `create()`/ctors; CHANGELOG + migration guide;
  CMake configure-time notices. Everything still functional.
- **6.22:** drop external fcl/bullet/ode from the required surface; classes
  become facades over the dart detector; FCL decoupled from core
  (`phase 7`), bullet/ode components rebuilt as facade components; package/
  export smoke tests.

## Maintainer ratification points

1. Facade-over-dart for `OdeCollisionDetector` vs coordinating a gz-physics
   change that drops the `GzOdeCollisionDetector` subclass first (facade is
   the recommended default; both keep the gz gate green in 6.22).
2. ~~Canonical name~~ RESOLVED by maintainer direction 2026-07-10: canonical
   `"dart"`, with `dart/collision/native/` merged into `dart/collision/dart/`
   and `NativeCollisionDetector` merged into `DARTCollisionDetector`.
3. (retired) The legacy `DARTCollisionDetector` engine question is subsumed
   by the consolidation: its narrowphase is replaced in 6.20, not deprecated.
