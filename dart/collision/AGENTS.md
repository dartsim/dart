# dart/collision/

Agent guidelines for the collision module.

## Overview

DART collision is converging on one built-in runtime collision stack. The
canonical detector is `dart`, implemented by `dart/collision/dart/` over the
native core in `dart/collision/native/`.

Legacy backend names are compatibility surfaces or optional reference harnesses
for tests and benchmarks. Do not add new runtime backend selection paths.

## Backends

| Surface    | Directory | Status               | Notes                                    |
| ---------- | --------- | -------------------- | ---------------------------------------- |
| **DART**   | `dart/`   | Default runtime      | Built-in native-backed detector          |
| **Native** | `native/` | Internal engine      | Geometry, broadphase, narrowphase, query |
| **FCL**    | `fcl/`    | Compatibility facade | Reference code lives under `tests/`      |
| **Bullet** | `bullet/` | Compatibility facade | Reference code lives under `tests/`      |
| **ODE**    | `ode/`    | Compatibility facade | Reference code lives under `tests/`      |

## Key Concepts

- **CollisionDetector**: Public DART collision interface
- **CollisionGroup**: Broad-phase optimization via grouping
- **CollisionObject**: Wrapper for shapes in collision system
- **Contact**: Point, normal, penetration depth

## Code Patterns

- Use `DartCollisionDetector::create()` or factory key `"dart"` for runtime
  code.
- Legacy factory keys/classes must route to the built-in detector unless they
  are explicitly part of an opt-in reference test or benchmark target.
- Reference tests and benchmarks that intentionally compare against FCL,
  Bullet, or ODE should use the tracked
  `tests/dart/test/reference_collision/` harness and the explicit
  `createReference()` APIs, not ordinary runtime detector creation.
- Keep the runtime layer split clean: public API and compatibility facades call
  into `dart/collision/dart/`, while `dart/collision/native/` owns geometry,
  broadphase, narrowphase, distance, raycast, contact persistence, cache
  invalidation, and profiling.
- Do not expose FCL, Bullet, ODE, or native-core implementation knobs through
  public collision options or results. Add DART-semantic options instead.
- Design new native hot paths around persistent scene state, stable handles,
  dirty shape/transform updates, deterministic query snapshots, low allocation,
  and benchmark/profiler visibility.
- `CollisionGroup::collide()` for collision queries
- Filter pairs via `CollisionFilter` for performance

## Compatibility Checklist

When changing the default collision runtime or native public headers, verify all
compatibility surfaces, not just factory registration:

- C++ legacy facades stay native-backed and preserve their documented legacy
  display names; factory aliases still construct the built-in detector.
- Generated umbrella headers still include retained compatibility facades.
- CMake component exports, installed headers, pkg-config `Requires`, and
  `package.xml` list every public header dependency.
- dartpy keeps the clean `DartCollisionDetector` API and does not reintroduce
  legacy backend aliases unless policy changes.
- gz-physics compatibility uses the normal installed package path without
  optional FCL/Bullet/ODE reference engines.

## Gotchas

- **Object destruction lifecycle**: In `notifyCollisionObjectDestroying()`, do NOT call `object->getShape()` — the underlying `ShapeFrame` may already be freed. Shape cleanup is handled by the object's deleter (e.g., `BulletCollisionShapeDeleter`), so manual reclamation in the notification callback is both redundant and unsafe (causes heap-use-after-free).
- **Reference engines are not runtime fallbacks**: FCL, Bullet, and ODE belong
  behind explicit reference test/benchmark APIs. Do not use reference-engine
  availability to decide normal runtime behavior or package compatibility.
- **Signed query contracts matter**: Overlap depth must remain available through
  the signed/unclamped distance path even when public `minDistance` is clamped by
  the query lower bound. Contact normals/signs must match DART semantics, and
  query result objects must not retain stale entries from previous calls.
- **Unbounded shapes need special broadphase handling**: Shapes such as planes
  can produce infinite AABBs; do not hash, floor, or cast those bounds as if
  they were finite grid coordinates.

## Testing

Unit tests: `tests/unit/collision/`
Integration tests: `tests/integration/test_Collision*.cpp`
ASAN testing: `pixi run test-asan` (catches memory errors like use-after-free)

For native-runtime changes, include the focused collision unit tests plus
package/downstream evidence when relevant:

- `pixi run lint` runs collision runtime isolation and compatibility facade
  audits.
- Native-only/default-package builds should succeed without FCL, Bullet, or ODE
  reference components.
- Run gz-physics compatibility when headers, factory aliases, component exports,
  or installed package behavior can affect downstream consumers.

## See Also

- @docs/onboarding/architecture.md - Collision in physics pipeline
- @dart/constraint/AGENTS.md - How contacts become constraints
