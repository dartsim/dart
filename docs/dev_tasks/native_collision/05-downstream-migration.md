# Downstream Migration Plan

This file defines the migration path for downstream users while DART converges
on one built-in collision detector.

## Compatibility Contract

During the DART 7 migration window, legacy collision names remain source
compatible but do not select external runtime engines:

| Surface                                                                                         | Migration behavior                                                        |
| ----------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| Factory keys `fcl`, `fcl_mesh`, `bullet`, `ode`                                                 | Accepted as aliases for the built-in `dart` detector.                     |
| C++ detector classes `FCLCollisionDetector`, `BulletCollisionDetector`, `OdeCollisionDetector`  | Native-backed facades over `DartCollisionDetector`.                       |
| Python detector names `FCLCollisionDetector`, `BulletCollisionDetector`, `OdeCollisionDetector` | Native-backed aliases of `DartCollisionDetector`.                         |
| Package components `collision-fcl`, `collision-bullet`, `collision-ode`                         | Native-backed interface facades that link the built-in `dart` stack.      |
| Explicit reference targets `collision-reference-*`                                              | Test/benchmark-only opt-in surfaces for FCL, Bullet, and ODE comparisons. |

The downstream-compatible runtime spelling is `dart`, `DartCollisionDetector`,
`CollisionDetectorType::Dart`, and the default `dart` package component.

## Downstream Work

Downstream runtime code should migrate in this order:

1. Replace explicit factory keys `fcl`, `fcl_mesh`, `bullet`, and `ode` with
   `dart`, or remove explicit selection and use the default detector.
2. Replace public C++ construction of `FCLCollisionDetector`,
   `BulletCollisionDetector`, and `OdeCollisionDetector` with
   `DartCollisionDetector` unless the code is an intentional reference
   benchmark.
3. Replace package requests for `collision-fcl`, `collision-bullet`, and
   `collision-ode` with `dart` when downstream no longer needs source
   compatibility with DART 6.x.
4. Keep reference-engine comparisons in tests or benchmarks only, using
   `collision-reference-*` components and the explicit `createReference()` APIs.

## Gates Before Removing Compatibility Facades

The north-star PR cannot delete retained compatibility facades until these
checks pass:

1. `pixi run -e gazebo test-gz` passes without downstream patches.
2. A downstream package smoke that requests `collision-fcl`,
   `collision-bullet`, and `collision-ode` links only the built-in `dart` stack.
3. Python compatibility names construct and report detector type `dart`.
4. Source-tree and installed legacy detector headers compile without FCL,
   Bullet, or ODE headers in native-only builds.
5. CI wheel artifacts pass `wheel-verify`, including the collision-isolation
   verifier.
6. Reference tests and benchmarks still have explicit opt-in access through
   `collision-reference-*` targets.

## Final Removal Sequence

1. Land downstream changes that stop treating legacy names as runtime backend
   selectors.
2. Keep one release window where retained DART compatibility facades route to
   built-in `dart`.
3. Remove or hard-deprecate compatibility facades only after downstream CI and
   package smoke evidence prove they are no longer needed.
4. Keep optional FCL/Bullet/ODE reference engines available only for tests and
   benchmarks while they remain useful for correctness and performance
   comparisons.
