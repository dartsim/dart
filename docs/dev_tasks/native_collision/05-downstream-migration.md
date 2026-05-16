# Downstream Migration Plan

This file defines the migration path for downstream users while DART converges
on one built-in collision detector.

## Compatibility Contract

During the DART 7 migration window, legacy collision names remain source
compatible but do not select external runtime engines:

| Surface                                                                                        | Migration behavior                                                                                                        |
| ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| Factory keys `fcl`, `fcl_mesh`, `bullet`, `ode`                                                | Accepted as aliases for the built-in `dart` detector.                                                                     |
| C++ detector classes `FCLCollisionDetector`, `BulletCollisionDetector`, `OdeCollisionDetector` | Native-backed facades over `DartCollisionDetector`; legacy display strings may be preserved for gz-physics compatibility. |
| dartpy detector API                                                                            | Clean DART 7 Python API: use `DartCollisionDetector`; legacy backend detector aliases are not retained.                   |
| Package components `collision-fcl`, `collision-bullet`, `collision-ode`                        | Native-backed interface facades that link the built-in `dart` stack.                                                      |
| Explicit reference targets `dart-test-reference-*`                                             | Test/benchmark-only opt-in build targets for FCL, Bullet, and ODE comparisons.                                            |

The downstream-compatible runtime spelling is `dart`, `DartCollisionDetector`,
`CollisionDetectorType::Dart`, and the default `dart` package component. Direct
C++ legacy facade objects may still report `"fcl"`, `"bullet"`, or `"ode"` as
compatibility display strings, but those strings do not select external
runtime engines. Per-engine FCL/Bullet/ODE collision build switches are gone
from the current build surface; core DART, dartpy, gz-physics runtime
integration, and native-backed compatibility component facades do not need
backend-shaped build flags. Explicit `dart-test-reference-*` comparison targets
are built only through the reference test/benchmark gates, are not installed
package components, and are not compatibility switches for normal core
libraries, dartpy wheels, package facades, or downstream runtime integration.

Current project steering for future changes: choose the clean long-term DART
collision API over preserving old backend-selection behavior. The compatibility
surface should stay as narrow as possible: enough for gz-physics and source
compatible downstream C++/package builds, native-backed everywhere, and never a
path back to FCL/Bullet/ODE as normal runtime engines.

Invalid convex or soft mesh data is non-collidable on the native runtime path
and emits a warning once instead of manufacturing placeholder geometry. The FCL
reference adapter still contains a legacy 0.1-radius sphere fallback for empty
`ConvexMeshShape` data; that divergence is documented in code as DART 8
technical debt and should be removed when reference behavior is aligned with
native.

Current status: the DART-side compatibility facade tests cover those display
strings, FCL/ODE facades preserve gz-required unsupported raycast behavior, a
fresh local `pixi run -e gazebo test-gz` run passes 65/65 tests through the
built-in detector, and a current native compatibility package smoke links the
retained `collision-fcl`, `collision-bullet`, and `collision-ode` components to
the built-in stack without installed old-engine runtime libraries. The current
local refreshes check those built artifacts directly: `readelf` shows the gz
DART plugin on `6742a21ab0f` and the native compatibility package smoke
executable on `dcfc994542f` depend on `libdart-collision-native.so` without any
`libdart-collision-reference-*`, `libdart-test-reference-*`, FCL, Bullet, ODE,
or libccd runtime dependency. The local `audit-collision-compat-facades` guard
now verifies that retained factory keys, C++ facades, and package component
names route to native DART collision while dartpy exposes only the clean
`DartCollisionDetector` API. Manual GitHub workflow runs are reference evidence
only. Downstream migration/deprecation policy evidence is still required before
retained C++ and package compatibility facades can be removed or hard-deprecated.

## DART 7 Deprecation Policy

DART 7 is the migration window for C++ and package source compatibility, not a
requirement to preserve legacy dartpy collision detector spellings. For dartpy,
prefer a clean API over compatibility shims: Python users should construct
`DartCollisionDetector` or use the default detector, and DART 7 does not expose
`DARTCollisionDetector`, `FCLCollisionDetector`, `BulletCollisionDetector`, or
`OdeCollisionDetector` aliases.

| Surface                                                                               | DART 7 behavior                                                                                                  | Planned DART 8 cleanup                                  |
| ------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| C++ factory keys `fcl`, `fcl_mesh`, `bullet`, `ode`                                   | Still accepted; emit deprecation warnings when `DART_COLLISION_DEPRECATE_LEGACY_NAMES=ON` and route to `dart`.   | Delete the alias registrars after downstream migration. |
| C++ classes `FCLCollisionDetector`, `BulletCollisionDetector`, `OdeCollisionDetector` | Retained as native-backed facades; marked deprecated by default through `DART_COLLISION_DEPRECATE_LEGACY_NAMES`. | Delete or move behind a final compatibility opt-in.     |
| CMake package components `collision-fcl`, `collision-bullet`, `collision-ode`         | Retained as native-backed interface targets for source/package compatibility.                                    | Remove once DART 6.x compatibility is no longer needed. |
| dartpy legacy detector names                                                          | Not retained; no Python `DeprecationWarning` shim. The clean API is `DartCollisionDetector`.                     | No additional cleanup expected.                         |
| Explicit reference APIs and targets                                                   | Not deprecated. `createReference()` and `dart-test-reference-*` remain the intentional comparison surface.       | Preserve while reference comparisons remain useful.     |

Downstreams that temporarily need to compile C++ legacy detector classes without
deprecation warnings can configure with
`-DDART_COLLISION_DEPRECATE_LEGACY_NAMES=OFF`. That switch is only a migration
aid; it does not restore external runtime backend selection.

## Deprecation Evidence Acceptance Criteria

The downstream migration/deprecation gate is closed only when each artifact
below has concrete evidence. Local documentation and verifier evidence is
already present for the policy shape; final PR/downstream CI evidence is still
required before retained C++ and package facades can be removed or
hard-deprecated further.

| Artifact                               | Required evidence                                                                                                                                                                        | Current status                                                                                                                                                                        |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Retained compatibility surface         | A named list of every DART 7 compatibility facade and the planned DART 8 cleanup path.                                                                                                   | Defined above for factory keys, C++ detector classes, package components, dartpy aliases, and reference APIs.                                                                         |
| C++ deprecation behavior               | `DART_COLLISION_DEPRECATE_LEGACY_NAMES` defaults `ON`, legacy C++ detector classes carry deprecation attributes through that gate, and legacy factory keys warn while routing to `dart`. | Implemented locally; `audit-collision-compat-facades` verifies retained names route to native DART collision.                                                                         |
| dartpy clean API behavior              | dartpy exposes `DartCollisionDetector` and does not expose `DARTCollisionDetector`, `FCLCollisionDetector`, `BulletCollisionDetector`, or `OdeCollisionDetector` aliases or shims.       | Implemented locally; Python tests and `audit-collision-compat-facades` verify the aliases are absent.                                                                                 |
| Runtime/package compatibility evidence | gz-physics, native package smokes, installed headers, and link inspections prove retained names are migration facades rather than external runtime backend selectors.                    | Complete locally on current local heads `6742a21ab0f` and `dcfc994542f`; final PR/downstream CI evidence and PR evidence transfer remain open.                                        |
| Removal plan                           | A future cleanup plan states what to delete and what to preserve after downstream migration.                                                                                             | Delete C++ factory aliases, C++ facade classes/headers, and `collision-fcl/bullet/ode` package facades only after downstream evidence; preserve explicit reference APIs while useful. |

Until the final PR or downstream CI surface carries that evidence, the retained
C++ and package compatibility facades stay in place. The open gate is evidence
transfer and downstream confirmation, not reintroducing dartpy aliases or
per-engine runtime build switches.

## Downstream Work

Downstream runtime code should migrate in this order:

1. Replace explicit factory keys `fcl`, `fcl_mesh`, `bullet`, and `ode` with
   `dart`, or remove explicit selection and use the default detector.
2. Replace public C++ construction of `FCLCollisionDetector`,
   `BulletCollisionDetector`, and `OdeCollisionDetector` with
   `DartCollisionDetector` unless the code is an intentional reference
   benchmark.
3. Replace dartpy use of `DARTCollisionDetector`, `FCLCollisionDetector`,
   `BulletCollisionDetector`, or `OdeCollisionDetector` with
   `DartCollisionDetector`; DART 7 does not keep Python aliases for these
   names.
4. Replace package requests for `collision-fcl`, `collision-bullet`, and
   `collision-ode` with `dart` when downstream no longer needs source
   compatibility with DART 6.x.
5. Keep reference-engine comparisons in tests or benchmarks only, using
   `dart-test-reference-*` targets and the explicit `createReference()` APIs.

## Gates Before Removing Compatibility Facades

The north-star PR cannot delete retained compatibility facades until these
checks pass:

1. `pixi run -e gazebo test-gz` passes without local gz-physics behavioral
   source patches beyond the intended DART CMake version requirement bump. This
   is complete locally from a fresh gz-physics clone on local head
   `6742a21ab0f`; manual GitHub gz-physics CI on `1e1faf6feb1` is reference
   evidence only.
2. A downstream package smoke that requests `collision-fcl`,
   `collision-bullet`, and `collision-ode` links only the built-in `dart` stack.
   This is complete locally with the dev-task native compatibility package
   smoke on `dcfc994542f`, and direct `readelf` inspection shows only
   `libdart-collision-native` from the collision stack.
3. dartpy exposes `DartCollisionDetector` and does not expose legacy detector
   aliases.
4. Source-tree and installed legacy detector headers compile without FCL,
   Bullet, or ODE headers in native-only builds.
5. `check-collision-runtime-isolation` and
   `audit-collision-compat-facades` pass in lint, proving non-reference DART
   source paths do not include old-engine or reference-backend headers, non-test
   code does not include `dart/test/reference_collision/...`, old reference
   implementation sources stay under `tests/dart/test/reference_collision/`,
   top-level legacy headers are pure native-backed forwarding facades, and
   retained compatibility names route to native DART collision.
6. Wheel artifacts pass `wheel-verify`, including the collision-isolation
   verifier. Local wheel verifier evidence exists, and the current wheel-matrix
   workflow pass is reference evidence only until final PR packaging.
7. Reference tests and benchmarks still have explicit opt-in access through
   `dart-test-reference-*` targets.

## Final Removal Sequence

1. Land downstream changes that stop treating legacy names as runtime backend
   selectors.
2. Keep one release window where retained DART compatibility facades route to
   built-in `dart`.
3. Remove or hard-deprecate compatibility facades only after downstream CI and
   migration/deprecation evidence prove they are no longer needed.
4. Keep optional FCL/Bullet/ODE reference engines available only for tests and
   benchmarks while they remain useful for correctness and performance
   comparisons.
