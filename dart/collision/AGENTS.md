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
  Bullet, or ODE should include `dart/test/reference_collision/...` and use the
  explicit `createReference()` APIs, not ordinary runtime detector creation.
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

## Gotchas

- **Object destruction lifecycle**: In `notifyCollisionObjectDestroying()`, do NOT call `object->getShape()` — the underlying `ShapeFrame` may already be freed. Shape cleanup is handled by the object's deleter (e.g., `BulletCollisionShapeDeleter`), so manual reclamation in the notification callback is both redundant and unsafe (causes heap-use-after-free).

## Testing

Unit tests: `tests/unit/collision/`
Integration tests: `tests/integration/test_Collision*.cpp`
ASAN testing: `pixi run test-asan` (catches memory errors like use-after-free)

## See Also

- @docs/onboarding/architecture.md - Collision in physics pipeline
- @dart/constraint/AGENTS.md - How contacts become constraints
