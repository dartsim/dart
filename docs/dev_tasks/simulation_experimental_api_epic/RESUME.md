# Resume: Simulation Experimental API Epic

## Quick Status

**Phase 5.2 (Dynamics), 5.3/5.4 (Collision + Constraints), and 5.5 (step()) COMPLETE.**

| Phase | Status      | Description                                                |
| ----- | ----------- | ---------------------------------------------------------- |
| 0-2   | ✅ Complete | Ground-truth, API cleanup, Python bindings                 |
| 3-4   | 🔄 Partial  | Testing/perf: golden tests done, coverage/optimization TBD |
| 5.1   | ✅ Complete | Forward Kinematics: joint transforms + Link integration    |
| 5.2   | ✅ Complete | Forward Dynamics: ABA with CoM + external forces           |
| 5.3-4 | ✅ Complete | Collision + Constraints: classic adapter + ShapeNode       |
| 5.5   | ✅ Complete | World::step(): Semi-implicit Euler integration             |
| 6     | Future      | Migration story                                            |

## Current Branch

```
Branch: feature/sim_exp
Status: Clean (no uncommitted changes)
Commits ahead of origin: 3
```

## What Works Now

The experimental API has a **fully functional physics simulation** with collision/constraint resolution via the classic adapter:

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

ShapeNodes attached to Links or RigidBodies participate in collision detection
and contact constraints during `step()`.

## Last Session Summary

Fixed classic-adapter build issues, corrected support polygon end-effector
mapping, and updated the epic docs with Phase 5 completion plus a new
simulation-experimental unit test README.

## Test Status

| Test File                        | Tests | Status |
| -------------------------------- | ----- | ------ |
| `test_spatial_math.cpp`          | 15    | ✅     |
| `test_motion_subspace.cpp`       | 24    | ✅     |
| `test_articulated_body.cpp`      | 9     | ✅     |
| `test_forward_dynamics.cpp`      | 9     | ✅     |
| `test_world.cpp`                 | 12    | ✅     |
| `test_shape_node.cpp`            | 2     | ✅     |
| `test_collision_integration.cpp` | 1     | ✅     |
| `test_joint_transform.cpp`       | 74    | ✅     |
| **Dynamics Total**               | 143+  | ✅     |

**Python Tests**: 18 tests in `python/tests/unit/simulation_experimental/`

## How to Resume

### 1. Verify Environment

```bash
git checkout feature/sim_exp
git status  # Expect clean tree with local commits
git log -5 --oneline

# Build
pixi run cmake --build build/default/cpp/Release --target dart-simulation-experimental

# Run tests
./build/default/cpp/Release/bin/test_forward_dynamics
./build/default/cpp/Release/bin/test_world
./build/default/cpp/Release/bin/test_shape_node
./build/default/cpp/Release/bin/test_collision_integration
./build/default/cpp/Release/bin/test_spatial_math
```

### 2. Choose Next Task

**Option A: Complete Phase 3-4 (Testing/Performance)**

- Run coverage analysis: `pixi run coverage-report`
- Profile and optimize hot paths
- Document performance characteristics

**Option B: Phase 6 (Migration)**

- Write migration guide: classic to experimental
- Create conversion utilities
- Add deprecation warnings

**Option C: Polish and PR**

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

### Collision/Constraint Adapter

Classic DART's `ConstraintSolver` is used as an internal backend:

- Experimental entities are mirrored into classic Skeletons
- `ShapeNode` components attach to classic ShapeNodes for collision/contact
- State is synchronized before/after constraint solving

Options considered:

1. **Adapter pattern**: Create wrapper classes (adds complexity)
2. **Parallel system**: New collision for ECS (duplicates code)
3. **Modify classic**: Add transform provider interface (invasive)

## Related Documents

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **Phase 5 design**: `docs/dev_tasks/simulation_experimental_api_epic/phase5_physics_design.md`
- **Classic DART ABA**: `dart/dynamics/Skeleton.cpp:3675`
- **Classic World::step()**: `dart/simulation/World.cpp:293`
