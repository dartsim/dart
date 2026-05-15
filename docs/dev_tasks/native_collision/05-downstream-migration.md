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
| Explicit reference targets `collision-reference-*`                                             | Test/benchmark-only opt-in surfaces for FCL, Bullet, and ODE comparisons.                                                 |

The downstream-compatible runtime spelling is `dart`, `DartCollisionDetector`,
`CollisionDetectorType::Dart`, and the default `dart` package component. Direct
C++ legacy facade objects may still report `"fcl"`, `"bullet"`, or `"ode"` as
compatibility display strings, but those strings do not select external
runtime engines. `DART_BUILD_COLLISION_FCL`,
`DART_BUILD_COLLISION_BULLET`, and `DART_BUILD_COLLISION_ODE` are no longer
needed by core DART, dartpy, gz-physics runtime integration, or the
native-backed compatibility component facades; they only opt into explicit
`collision-reference-*` comparison components. They are not compatibility
switches for normal core libraries, dartpy wheels, package facades, or
downstream runtime integration.

Current status: the DART-side compatibility facade tests cover those display
strings, FCL/ODE facades preserve gz-required unsupported raycast behavior, a
fresh local `pixi run -e gazebo test-gz` run passes 65/65 tests through the
built-in detector, and a current native compatibility package smoke links the
retained `collision-fcl`, `collision-bullet`, and `collision-ode` components to
the built-in stack without installed old-engine runtime libraries. The local
`4b155655890` refresh also checks those built artifacts directly: `readelf`
shows the gz DART plugin and the native compatibility package smoke executable
depend on `libdart-collision-native.so` without any
`libdart-collision-reference-*`, FCL, Bullet, ODE, or libccd runtime
dependency. The local `audit-collision-compat-facades` guard now verifies
that retained factory keys, C++ facades, and package component names route to
native DART collision while dartpy exposes only the clean
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
| Explicit reference APIs and components                                                | Not deprecated. `createReference()` and `collision-reference-*` remain the intentional comparison surface.       | Preserve while reference comparisons remain useful.     |

Downstreams that temporarily need to compile C++ legacy detector classes without
deprecation warnings can configure with
`-DDART_COLLISION_DEPRECATE_LEGACY_NAMES=OFF`. That switch is only a migration
aid; it does not restore external runtime backend selection.

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
   `collision-reference-*` components and the explicit `createReference()` APIs.

## Gates Before Removing Compatibility Facades

The north-star PR cannot delete retained compatibility facades until these
checks pass:

1. `pixi run -e gazebo test-gz` passes without downstream patches. This is
   complete locally from a fresh gz-physics clone on `4b155655890`; manual
   GitHub gz-physics CI on `1e1faf6feb1` is reference evidence only.
2. A downstream package smoke that requests `collision-fcl`,
   `collision-bullet`, and `collision-ode` links only the built-in `dart` stack.
   This is complete locally with the dev-task native compatibility package
   smoke, and direct `readelf` inspection shows only `libdart-collision-native`
   from the collision stack.
3. dartpy exposes `DartCollisionDetector` and does not expose legacy detector
   aliases.
4. Source-tree and installed legacy detector headers compile without FCL,
   Bullet, or ODE headers in native-only builds.
5. `check-collision-runtime-isolation` and
   `audit-collision-compat-facades` pass in lint, proving non-reference DART
   source paths do not include old-engine or reference-backend headers, legacy
   implementation sources remain under explicit `reference/` paths, top-level
   legacy headers are pure native-backed forwarding facades, and retained
   compatibility names route to native DART collision.
6. Wheel artifacts pass `wheel-verify`, including the collision-isolation
   verifier. Local wheel verifier evidence exists, and the current wheel-matrix
   workflow pass is reference evidence only until final PR packaging.
7. Reference tests and benchmarks still have explicit opt-in access through
   `collision-reference-*` targets.

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
