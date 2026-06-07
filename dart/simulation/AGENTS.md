# dart/simulation/

Agent guidelines for the simulation module.

## Overview

World container and time-stepping for the DART 7 simulation API.

## Key Concepts

- **World**: Value-owned top-level container; coordinates DART 7 rigid,
  deformable, multibody, collision, and solver components
- **Recording**: Playback support for simulation state
- **Time-stepping**: `World::step()` orchestrates collision → constraints → integration

## Code Patterns

- Construct `World` directly (`dart::simulation::World world;`)
- Default time step: 0.001s (1ms)
- Default gravity: [0, 0, -9.81]
- Configure solver families through public value options; do not expose solver
  registries or backend implementation types

## Physics Pipeline (per step)

1. Sync public handles and frame state
2. Advance configured rigid, deformable, and multibody solver stages
3. Resolve contacts and couplings through the selected public method families
4. Update state, replay, diagnostics, and profiling snapshots

## Testing

Unit tests: `tests/unit/simulation/`
Integration tests: `tests/integration/test_World*.cpp`

## See Also

- @docs/onboarding/architecture.md - Physics pipeline details
- @dart/constraint/AGENTS.md - Constraint solving
- @dart/collision/AGENTS.md - Collision detection
