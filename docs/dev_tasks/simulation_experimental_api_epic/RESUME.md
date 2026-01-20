# Resume: Simulation Experimental API Epic

## Quick Status

**Phase 5.2 (Dynamics) COMPLETE. Phase 5.5 (step()) COMPLETE (without collision/constraints).**

| Phase | Status         | Description                                              |
| ----- | -------------- | -------------------------------------------------------- |
| 0     | ✅ Complete    | Ground-truth: tests, docs, example                       |
| 1     | ✅ Complete    | API cleanup: all joint types, state accessors, RigidBody |
| 2     | ✅ Complete    | Python bindings: all classes + StateSpace                |
| 3     | 🔄 In Progress | Testing strategy: golden tests done, coverage pending    |
| 4     | 🔄 In Progress | Performance: benchmarks done, profiling pending          |
| 5.1   | ✅ Complete    | Forward Kinematics: joint transforms + Link integration  |
| CG    | ✅ Complete    | Compute Graph: Taskflow integration for parallel exec    |
| 5.2   | ✅ Complete    | Forward Dynamics: ABA algorithm with CoM + external forces |
| 5.5   | ✅ Complete    | World::step(): Physics simulation with semi-implicit Euler |
| 5.3-4 | Deferred       | Collision + Constraints: Complex integration deferred    |
| 6     | Future         | Migration story                                          |

## Current Branch

```
Branch: feature/sim_exp
Status: Clean (pending commit)
```

## Last Session Summary

Completed Phase 5.2 (Dynamics) and Phase 5.5 (step()):

1. **Added CoM offset to MassProperties**
   - `localCOM` field in `comps/dynamics.hpp`
   - ABA now correctly uses CoM for spatial inertia and gravity torques

2. **Enabled exact validation against classic DART**
   - `ValidateAgainstClassicDART` test now uses 1e-10 tolerance
   - Experimental API produces identical accelerations to classic DART

3. **Added external force support**
   - `externalForce` and `externalTorque` fields in Link component
   - `Link::addExternalForce()`, `setExternalForce()`, `clearExternalForces()`
   - ABA includes external forces in bias force computation

4. **Implemented World::step()**
   - Semi-implicit Euler integration (velocity then position)
   - Configurable gravity, timeStep
   - Time and frame tracking
   - Automatic force clearing

## Test Status

| Test File                   | Tests | Status |
| --------------------------- | ----- | ------ |
| `test_spatial_math.cpp`     | 15    | ✅     |
| `test_motion_subspace.cpp`  | 24    | ✅     |
| `test_articulated_body.cpp` | 9     | ✅     |
| `test_forward_dynamics.cpp` | 9     | ✅     |
| `test_world.cpp`            | 12    | ✅     |
| **Total**                   | 69    | ✅     |

New tests added:
- `ExternalForces` - Validates external force integration
- `StepRequiresSimulationMode` - Mode check
- `StepAdvancesTime` - Time/frame tracking
- `FreeFallDynamics` - Physics correctness
- `ExternalForceIntegration` - Force integration correctness

## Immediate Next Steps

1. **Commit current changes** - CoM, external forces, step()
2. **Run full test suite** - Ensure no regressions
3. **Phase 5.3-5.4 (Collision/Constraints)** - Complex, deferred for now

## How to Resume

```bash
git checkout feature/sim_exp
git status && git log -5 --oneline

# Build and test
pixi run cmake --build build/default/cpp/Release --target dart-simulation-experimental
./build/default/cpp/Release/bin/test_forward_dynamics
./build/default/cpp/Release/bin/test_world
```

## Phase 5 Deliverables Status

| Component               | Status | Location                            |
| ----------------------- | ------ | ----------------------------------- |
| Spatial math utilities  | ✅     | `dynamics/spatial_math.hpp/cpp`     |
| Motion subspace (S)     | ✅     | `dynamics/motion_subspace.hpp/cpp`  |
| ABA data structures     | ✅     | `dynamics/articulated_body.hpp/cpp` |
| ForwardDynamicsSystem   | ✅     | `dynamics/forward_dynamics.hpp/cpp` |
| CoM offset support      | ✅     | `comps/dynamics.hpp` (MassProperties)|
| External forces         | ✅     | `comps/link.hpp`, `multi_body/link.cpp` |
| World::step()           | ✅     | `world.hpp/cpp`                     |
| Collision integration   | Deferred | Requires adapter to classic collision |
| Constraint integration  | Deferred | Requires adapter to classic solver |

## Known Limitations

1. **No collision detection**: Requires adapting classic `dart::collision` to ECS
2. **No constraint solving**: Requires adapting classic `dart::constraint` to ECS
3. **Fixed base only**: Root link cannot have velocity/acceleration

## Related Files

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **Phase 5 design**: `docs/dev_tasks/simulation_experimental_api_epic/phase5_physics_design.md`
- **Dynamics module**: `dart/simulation/experimental/dynamics/`
- **Tests**: `tests/unit/simulation/experimental/`
