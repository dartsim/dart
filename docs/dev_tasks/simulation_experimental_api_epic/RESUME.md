# Resume: Simulation Experimental API Epic

## Quick Status

**Phase 5.2 (Dynamics) COMPLETE. Phase 5.5 (step()) COMPLETE. Collision/Constraints DEFERRED.**

| Phase | Status      | Description                                                |
| ----- | ----------- | ---------------------------------------------------------- |
| 0-2   | ✅ Complete | Ground-truth, API cleanup, Python bindings                 |
| 3-4   | 🔄 Partial  | Testing/perf: golden tests done, coverage/optimization TBD |
| 5.1   | ✅ Complete | Forward Kinematics: joint transforms + Link integration    |
| 5.2   | ✅ Complete | Forward Dynamics: ABA with CoM + external forces           |
| 5.5   | ✅ Complete | World::step(): Semi-implicit Euler integration             |
| 5.3-4 | ⏸️ Deferred | Collision + Constraints: Complex adapter work needed       |
| 6     | Future      | Migration story                                            |

## Current Branch

```
Branch: feature/sim_exp
Status: Clean (all changes committed)
Commits ahead of origin: 10
```

## What Works Now

The experimental API has a **fully functional physics simulation**:

```cpp
namespace dse = dart::simulation::experimental;

dse::World world;
world.setGravity({0, 0, -9.81});
world.setTimeStep(0.001);

auto robot = world.addMultiBody("robot");
auto base = robot.addLink("base");
auto link = robot.addLink("link", {
    .parentLink = base,
    .jointType = dse::comps::JointType::Revolute
});

link.setMass(1.0);
link.addExternalForce({10, 0, 0});

world.enterSimulationMode();
for (int i = 0; i < 1000; ++i) {
    world.step();  // Physics simulation works!
}
```

## Last Session Summary

Completed Phase 5.2 (Dynamics) and Phase 5.5 (step()):

1. **CoM offset support** - `localCOM` in MassProperties, used in spatial inertia/gravity
2. **External forces** - `Link::addExternalForce()`, `setExternalTorque()`, `clearExternalForces()`
3. **World::step()** - Semi-implicit Euler, configurable gravity/timeStep, time tracking
4. **Exact validation** - Experimental ABA matches classic DART to 1e-10 tolerance

## Test Status

| Test File                   | Tests | Status |
| --------------------------- | ----- | ------ |
| `test_spatial_math.cpp`     | 15    | ✅     |
| `test_motion_subspace.cpp`  | 24    | ✅     |
| `test_articulated_body.cpp` | 9     | ✅     |
| `test_forward_dynamics.cpp` | 9     | ✅     |
| `test_world.cpp`            | 12    | ✅     |
| `test_joint_transform.cpp`  | 74    | ✅     |
| **Dynamics Total**          | 143+  | ✅     |

## How to Resume

### 1. Verify Environment

```bash
git checkout feature/sim_exp
git status  # Should be clean
git log -5 --oneline

# Build
pixi run cmake --build build/default/cpp/Release --target dart-simulation-experimental

# Run tests
./build/default/cpp/Release/bin/test_forward_dynamics
./build/default/cpp/Release/bin/test_world
./build/default/cpp/Release/bin/test_spatial_math
```

### 2. Choose Next Task

**Option A: Complete Phase 3-4 (Testing/Performance)**
- Run coverage analysis: `pixi run test-coverage` (if available)
- Profile and optimize hot paths
- Document performance characteristics

**Option B: Phase 5.3-5.4 (Collision/Constraints) - COMPLEX**
- Requires creating adapter classes to bridge ECS with classic DART's collision/constraint modules
- Key challenge: `CollisionObject` expects `dynamics::ShapeFrame*`, not ECS entities
- Possible approaches:
  1. Create `ExperimentalShapeFrame` adapter wrapping Link
  2. Modify collision module to accept transform providers
  3. Create standalone collision system for experimental API

**Option C: Phase 6 (Migration)**
- Write migration guide: classic to experimental
- Create conversion utilities
- Add deprecation warnings

**Option D: Polish and PR**
- Run `pixi run lint` and fix any issues
- Run full test suite: `pixi run test-all`
- Create PR for the feature branch

## Key Files Reference

### Dynamics Module (Phase 5.2)
```
dart/simulation/experimental/dynamics/
├── spatial_math.hpp/cpp      # Spatial vector types, transforms
├── motion_subspace.hpp/cpp   # Joint S matrix computation
├── articulated_body.hpp/cpp  # ABA workspace data structures
└── forward_dynamics.hpp/cpp  # ABA algorithm implementation
```

### World::step() (Phase 5.5)
```
dart/simulation/experimental/
├── world.hpp    # step(), setGravity(), setTimeStep(), getTime()
└── world.cpp    # step() implementation with semi-implicit Euler
```

### Link External Forces
```
dart/simulation/experimental/
├── comps/link.hpp           # externalForce, externalTorque fields
├── multi_body/link.hpp/cpp  # addExternalForce(), clearExternalForces()
```

## Architecture Notes

### ABA Algorithm Flow
```
World::step()
  └─> ForwardDynamicsSystem::compute()
        ├─> computeVelocities()        # Pass 1: v[i], c[i]
        ├─> computeArticulatedInertias() # Pass 2a: IA[i]
        ├─> computeBiasForces()        # Pass 2b: pA[i] with gravity + external
        └─> computeAccelerations()     # Pass 3: ddq[i]
  └─> Integrate velocities: dq += ddq * dt
  └─> Integrate positions: q += dq * dt
  └─> Clear external forces (optional)
```

### Why Collision/Constraints Are Deferred
Classic DART's collision module uses `dynamics::ShapeFrame` extensively:
- `CollisionObject` constructor requires `const dynamics::ShapeFrame*`
- Shape transforms come from `ShapeFrame::getWorldTransform()`
- Experimental API uses ECS entities, not inheritance

Options considered:
1. **Adapter pattern**: Create wrapper classes (adds complexity)
2. **Parallel system**: New collision for ECS (duplicates code)
3. **Modify classic**: Add transform provider interface (invasive)

## Related Documents

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **Phase 5 design**: `docs/dev_tasks/simulation_experimental_api_epic/phase5_physics_design.md`
- **Classic DART ABA**: `dart/dynamics/Skeleton.cpp:3675`
- **Classic World::step()**: `dart/simulation/World.cpp:293`
