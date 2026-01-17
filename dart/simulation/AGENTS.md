# dart/simulation/

Agent guidelines for the simulation module.

## Overview

World container and time-stepping: manages skeletons, collision, constraints, and integration.

## Key Concepts

- **World**: Top-level container; coordinates all simulation components
- **Recording**: Playback support for simulation state
- **Time-stepping**: `World::step()` orchestrates collision → constraints → integration

## Code Patterns

- Use `World::create()` factory
- Default time step: 0.001s (1ms)
- Default gravity: [0, 0, -9.81]
- Semi-implicit Euler integration (built-in)

## Physics Pipeline (per step)

1. `computeForwardKinematics()` - Update transforms
2. `computeForwardDynamics()` - ABA: q̈ = M⁻¹(τ - C - g)
3. `detectCollision()` - Generate contacts
4. `ConstraintSolver::solve()` - Resolve constraints via LCP
5. `integrateState()` - Advance q, q̇

## Testing

Unit tests: `tests/unit/simulation/`
Integration tests: `tests/integration/test_World*.cpp`

## See Also

- @docs/onboarding/architecture.md - Physics pipeline details
- @dart/constraint/AGENTS.md - Constraint solving
- @dart/collision/AGENTS.md - Collision detection
