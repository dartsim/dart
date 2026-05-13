# dart/collision/

Agent guidelines for the collision module.

## Overview

Pluggable collision detection with multiple backends.

## Backends

| Backend    | Directory | Status    | Notes                        |
| ---------- | --------- | --------- | ---------------------------- |
| **DART**   | `dart/`   | Default   | Built-in native backend      |
| **FCL**    | `fcl/`    | Reference | Flexible Collision Library   |
| **Bullet** | `bullet/` | Reference | Bullet Physics backend       |
| **ODE**    | `ode/`    | Reference | Open Dynamics Engine backend |

## Key Concepts

- **CollisionDetector**: Abstract interface for backends
- **CollisionGroup**: Broad-phase optimization via grouping
- **CollisionObject**: Wrapper for shapes in collision system
- **Contact**: Point, normal, penetration depth

## Code Patterns

- Use `DartCollisionDetector::create()` or factory key `"dart"` for default
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
