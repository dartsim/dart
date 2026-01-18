# dart/collision/

Agent guidelines for the collision module.

## Overview

Pluggable collision detection with multiple backends.

## Backends

| Backend    | Directory | Status   | Notes                        |
| ---------- | --------- | -------- | ---------------------------- |
| **FCL**    | `fcl/`    | Default  | Flexible Collision Library   |
| **Bullet** | `bullet/` | Optional | Bullet Physics backend       |
| **ODE**    | `ode/`    | Optional | Open Dynamics Engine backend |
| **DART**   | `dart/`   | Native   | Built-in primitive support   |

## Key Concepts

- **CollisionDetector**: Abstract interface for backends
- **CollisionGroup**: Broad-phase optimization via grouping
- **CollisionObject**: Wrapper for shapes in collision system
- **Contact**: Point, normal, penetration depth

## Code Patterns

- Use `FCLCollisionDetector::create()` for default
- `CollisionGroup::collide()` for collision queries
- Filter pairs via `CollisionFilter` for performance

## Testing

Unit tests: `tests/unit/collision/`
Integration tests: `tests/integration/test_Collision*.cpp`

## See Also

- @docs/onboarding/architecture.md - Collision in physics pipeline
- @dart/constraint/AGENTS.md - How contacts become constraints
