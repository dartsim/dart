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

## Decision 1 — canonical backend and naming

**The native engine (`dart::collision::native`, class
`NativeCollisionDetector`) is the single built-in backend; canonical factory
key `"native"`. The legacy keys `"dart"`, `"fcl"`, `"bullet"`, `"ode"` remain
permanently resolvable and, from 6.22, create native-backed implementations.**

Rationale:

- During the 6.20/6.21 transition `"dart"` still names the legacy limited
  `DARTCollisionDetector`; documenting `"dart"` as canonical would be
  ambiguous exactly when users migrate. `"native"` is unambiguous across the
  whole window and already shipped as the opt-in key in 6.20.
- The legacy `dart` detector is a strict subset of native (six primitive
  pairs, narrowphase-only, `distance()` stub, no raycast, no broadphase), so
  aliasing `"dart"` to native at 6.22 loses nothing.
- gz-physics keeps working: all four names keep resolving
  (`WorldFeatures.cc:47-70`), and gz's actual default is its own
  `GzOdeCollisionDetector` subclass (`EntityManagementFeatures.cc:728`), so
  DART's default flip does not change gz behavior by itself.
- Phase 6 adds `CollisionDetectorType::Native` (World enum) and the dartpy
  `NATIVE` enum value; the dartpy `NativeCollisionDetector` class binding
  already landed with the comparison harness PR.

## Decision 2 — facades over native, not component removal

**6.22 removes the external fcl/bullet/ode dependencies; the detector classes
and CMake components survive as compatibility facades over the native
engine.** The source-verified gz obligations and how facades satisfy them:

| gz obligation (evidence) | facade answer |
| --- | --- |
| `find_package(DART COMPONENTS collision-bullet collision-ode ...)` (`gz-physics/CMakeLists.txt:74-84`) | components remain, containing facade libs with no external dep |
| `GzOdeCollisionDetector : public OdeCollisionDetector`, overrides `collide()x2`, adds per-pair capping (`GzOdeCollisionDetector.hh:26-69`) | `OdeCollisionDetector` stays a real subclassable class whose `collide()` delegates to native and honors `CollisionOption.maxNumContactsPerPair` |
| keys "fcl"/"bullet"/"ode"/"dart" resolve via `create()` (`WorldFeatures.cc:47-70`) | facade factories keep all four registrations |
| `getLastCollisionResult()` Contact fields {point, normal, penetrationDepth, force, collisionObject1/2} (`SimulationFeatures.cc:213-256`) | native populates the same DART 6 `Contact` (proven by adapter parity tests) |
| raycast (Bullet incumbent, `SimulationFeatures.cc:176-185`) | native raycast merged (#3355) |
| distance (FCL incumbent) | native distance merged (#3343/#3352) |
| VoxelGrid/octree (FCL incumbent) | native compound voxel support merged (#3358) |

Notes: gz subclasses **only** `OdeCollisionDetector` (there is no
`GzBulletCollisionDetector`; the doc-03 matrix row overstated this). Bullet
and FCL only need `create()` + name resolution + `getType()`.

Behavior disclosure: facades emit native contact profiles (counts/normals may
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
  `[[deprecated("...use the native collision detector...")]]` (or a new
  `DART_DEPRECATED_MESSAGE`) on `create()`/constructors — NOT on the classes
  gz subclasses; warning-cleanliness of `GzOdeCollisionDetector` under
  -Werror must be prototyped before 6.21 ships.
- SOVERSION is `MAJOR.MINOR` (`cmake/DARTMacros.cmake:94`): every minor gets
  a new SONAME, so converting classes to facades in 6.21/6.22 is
  ABI-permissible; the constraint is source/API compatibility plus the gz
  gate.

## Timeline

- **6.20 (now, phases 4-6 of the port plan):** close the remaining native
  performance gaps (broadphase landed; S3 narrowphase delta, small-scene
  overhead, and the S6 dense-pile resting profile remain), then flip only the
  DEFAULT detector (both ctors + WorldConfig/SkelParser surface). All legacy
  detectors stay real and selectable; no deprecations on the LTS branch.
- **6.21:** deprecation attributes with migration messages on
  FCL/Bullet/ODE/legacy-dart `create()`/ctors; CHANGELOG + migration guide;
  CMake configure-time notices. Everything still functional.
- **6.22:** drop external fcl/bullet/ode from the required surface; classes
  become facades-over-native; `"dart"` key resolves to the native engine;
  legacy `DARTCollisionDetector` narrowphase retired; FCL decoupled from core
  (`phase 7`), bullet/ode components rebuilt as facade components; package/
  export smoke tests.

## Maintainer ratification points

1. Facade-over-native for `OdeCollisionDetector` vs coordinating a gz-physics
   change that drops the `GzOdeCollisionDetector` subclass first (facade is
   the recommended default; both keep the gz gate green in 6.22).
2. Canonical name `"native"` (recommended) vs `"dart"` — both keys resolve to
   the same engine from 6.22 either way; this only decides documentation and
   the eventual class naming.
3. Whether 6.21 should also deprecate the legacy `DARTCollisionDetector`
   engine explicitly (recommended: yes, same cycle).
