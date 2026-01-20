# Resume: Simulation Experimental API Epic

## Quick Status

**Phases 0-2 COMPLETE. Phases 3-4 IN PROGRESS. Phase 5.1 (FK) COMPLETE. Compute Graph COMPLETE. Phase 5.2 (Dynamics) IN PROGRESS.**

| Phase | Status         | Description                                              |
| ----- | -------------- | -------------------------------------------------------- |
| 0     | ✅ Complete    | Ground-truth: tests, docs, example                       |
| 1     | ✅ Complete    | API cleanup: all joint types, state accessors, RigidBody |
| 2     | ✅ Complete    | Python bindings: all classes + StateSpace                |
| 3     | 🔄 In Progress | Testing strategy: golden tests done, coverage pending    |
| 4     | 🔄 In Progress | Performance: benchmarks done, profiling pending          |
| 5.1   | ✅ Complete    | Forward Kinematics: joint transforms + Link integration  |
| CG    | ✅ Complete    | Compute Graph: Taskflow integration for parallel exec    |
| 5.2   | 🔄 In Progress | Forward Dynamics: ABA algorithm implementation           |
| 5.3-5 | Pending        | Collision, Constraints, step()                           |
| 6     | Future         | Migration story                                          |

## Current Branch

```
Branch: feature/sim_exp
Status: Clean (all changes committed)
Last commit: 218939daf45 feat(simulation-experimental): Add ABA velocity pass and integration tests
```

## Last Session Summary

Completed ABA velocity pass and added integration tests:

1. **Added `computeVelocities()` method** implementing ABA Pass 1:
   - Computes link spatial velocities: `v[i] = Xup[i] * v[parent[i]] + S * qd`
   - Computes Coriolis/partial accelerations: `c[i] = v × (S * qd)`

2. **Added 4 new integration tests** (7 total tests now):
   - `SimplePendulumAcceleration` - Single link with revolute joint
   - `TwoLinkChain` - 2-DOF serial chain
   - `PrismaticJoint` - Vertical slider under gravity
   - `ComputeAllMultiBodies` - Multiple robots processed

3. **All tests passing**: 7 tests in `test_forward_dynamics`

## Commits This Session

```
218939daf45 feat(simulation-experimental): Add ABA velocity pass and integration tests
```

## Dynamics Tests Status

| Test File                   | Tests | Status |
| --------------------------- | ----- | ------ |
| `test_spatial_math.cpp`     | 15    | ✅     |
| `test_motion_subspace.cpp`  | 24    | ✅     |
| `test_articulated_body.cpp` | 9     | ✅     |
| `test_forward_dynamics.cpp` | 7     | ✅     |
| **Total**                   | 55    | ✅     |

## Immediate Next Steps

1. **Validate ABA against classic DART** - Create comparison test
2. **Add CoM offset support** - Current implementation assumes CoM at joint origin
3. **End-to-end physics test** - Simulate pendulum, verify energy conservation

## How to Resume

```bash
git checkout feature/sim_exp
git status  # Should be clean
git log -5 --oneline

# Verify tests
pixi run cmake --build build/default/cpp/Release --target dart-simulation-experimental
./build/default/cpp/Release/bin/test_forward_dynamics
./build/default/cpp/Release/bin/test_spatial_math
./build/default/cpp/Release/bin/test_motion_subspace
./build/default/cpp/Release/bin/test_articulated_body
```

## Phase 5.2 Deliverables Status

| Component              | Status | Location                            |
| ---------------------- | ------ | ----------------------------------- |
| Spatial math utilities | ✅     | `dynamics/spatial_math.hpp/cpp`     |
| Motion subspace (S)    | ✅     | `dynamics/motion_subspace.hpp/cpp`  |
| ABA data structures    | ✅     | `dynamics/articulated_body.hpp/cpp` |
| ForwardDynamicsSystem  | ✅     | `dynamics/forward_dynamics.hpp/cpp` |
| Unit tests (48)        | ✅     | `tests/.../dynamics/test_*.cpp`     |
| Velocity pass          | ✅     | `computeVelocities()` implemented   |
| Integration tests      | ✅     | 4 tests for various configurations  |
| Classic DART validation| ❌     | Needs comparison test               |

## Known Limitations

1. **No CoM offset**: Mass is assumed at joint origin, limiting gravity torque accuracy
2. **No external forces**: Only gravity is applied, no contact/applied forces
3. **Fixed base only**: Root link cannot have velocity/acceleration

## Related Files

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **Phase 5 design**: `docs/dev_tasks/simulation_experimental_api_epic/phase5_physics_design.md`
- **Compute graph design**: `docs/dev_tasks/simulation_experimental_api_epic/compute_graph_design.md`
- **C++ source**: `dart/simulation/experimental/`
- **Dynamics module**: `dart/simulation/experimental/dynamics/`
- **Tests**: `tests/unit/simulation/experimental/dynamics/`
- **Classic DART ABA reference**: `dart/dynamics/Skeleton.cpp:3675`
