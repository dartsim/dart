# Resume: Simulation Experimental API Epic

## Quick Status

**Phases 0, 1, 2 COMPLETE. Ready for Phase 3 (Testing) or Phase 4 (Performance).**

| Phase | Status | Description |
|-------|--------|-------------|
| 0 | ✅ Complete | Ground-truth: tests, docs, example |
| 1 | ✅ Complete | API cleanup: all joint types, state accessors, RigidBody |
| 2 | ✅ Complete | Python bindings: all classes + StateSpace |
| 3 | ⏳ Next | Testing strategy: coverage audit, golden tests |
| 4 | ⏳ Next | Performance: benchmarks (can run parallel with 3) |
| 5 | Future | Physics integration: `World::step()` |
| 6 | Future | Migration story |

## Current Branch

```
Branch: feature/sim_exp
Status: 2 commits ahead of origin (unpushed)
Working tree: clean
```

**Unpushed commits:**
```
cd33d970ef9 feat(simulation-experimental): Add StateSpace Python bindings
6c1e872348c feat(simulation-experimental): Add Python bindings for core classes
```

## Immediate Next Step

**Choose one:**

1. **Push and create PR** for Phases 0-2 (ship what's done)
2. **Continue to Phase 3** (testing coverage audit)
3. **Continue to Phase 4** (benchmarks - can run parallel with Phase 3)

## How to Resume

```bash
# 1. Checkout and verify branch state
git checkout feature/sim_exp
git status
git log --oneline -5

# 2. Verify tests pass
pixi run build
ctest -L simulation-experimental --test-dir build/default/cpp/Release
pixi run pytest python/tests/unit/simulation_experimental/ -v

# 3. Choose next action:
# Option A: Push and create PR
git push origin feature/sim_exp
gh pr create --title "feat(simulation-experimental): Python bindings for core classes" --body "..."

# Option B: Start Phase 3 (Testing)
# See README.md Phase 3 tasks

# Option C: Start Phase 4 (Performance)
# See README.md Phase 4 tasks
```

## What Was Built

### C++ (Phases 0-1)

| Component | Tests | Location |
|-----------|-------|----------|
| Joint (all 8 types) | 41 | `tests/unit/simulation/experimental/test_joint.cpp` |
| Link | 14 | `tests/unit/simulation/experimental/test_link.cpp` |
| RigidBody | 14 | `tests/unit/simulation/experimental/test_rigid_body.cpp` |
| Hello World Example | - | `examples/simulation_experimental_hello_world/` |

### Python Bindings (Phase 2)

| Class | File |
|-------|------|
| World | `python/dartpy/simulation_experimental/world.cpp` |
| MultiBody, Link, Joint | `python/dartpy/simulation_experimental/multi_body.cpp` |
| Frame, FreeFrame, FixedFrame | `python/dartpy/simulation_experimental/frame.cpp` |
| RigidBody | `python/dartpy/simulation_experimental/rigid_body.cpp` |
| StateSpace, StateSpaceVariable | `python/dartpy/simulation_experimental/state_space.cpp` |
| JointType enum | `python/dartpy/simulation_experimental/joint_type.cpp` |

**Python tests:** 16 tests in `python/tests/unit/simulation_experimental/test_experimental_world.py`

## Test Verification Commands

```bash
# C++ tests (13 test binaries, ~70 tests)
ctest -L simulation-experimental --test-dir build/default/cpp/Release

# Python tests (16 tests)
pixi run pytest python/tests/unit/simulation_experimental/ -v

# Full validation
pixi run test-all
```

## Key Design Decisions

1. **Binding function naming**: Used `Exp` prefix (e.g., `defExpFrame`, `defExpJoint`) to avoid conflicts with existing `dynamics/frame.cpp` bindings
2. **LinkOptions workaround**: Exposed `add_link_with_joint()` Python method instead of `LinkOptions` struct (couldn't be default-constructed)
3. **Python docstrings**: Deferred - follows existing dartpy pattern where C++ Doxygen is source of truth
4. **StateSpace**: Full binding including Variable struct with all properties

## Python API Example

```python
import dartpy as dart
import numpy as np

exp = dart.simulation_experimental

# Create world and robot
world = exp.World()
robot = world.add_multi_body("robot")

# Add links with joints
root = robot.add_link("base")
arm = robot.add_link_with_joint("arm", root, "shoulder",
    exp.JointType.Revolute, np.array([0, 0, 1]))

# Access joint state
joint = arm.get_parent_joint()
joint.set_position(np.array([1.0]))
print(f"Position: {joint.get_position()}")

# StateSpace for optimization
space = exp.StateSpace()
space.add_variable("joint_pos", 6, -3.14, 3.14)
space.finalize()
print(f"Dimension: {space.get_dimension()}")  # 6
```

## Phase 3 Tasks (Testing Strategy)

From README.md:
- [ ] Audit test coverage with `gcov`/`lcov`
- [ ] Write missing unit tests to reach 80% coverage
- [ ] Create serialization golden tests (known-good binary files)
- [ ] Add integration test: load URDF then convert to experimental
- [ ] Benchmark test: create 1000-body world, serialize, deserialize

## Phase 4 Tasks (Performance)

From README.md:
- [ ] Create benchmark: MultiBody creation (100, 1000, 10000 bodies)
- [ ] Create benchmark: Frame transform computation
- [ ] Create benchmark: StateSpace extraction/injection
- [ ] Profile ECS component access patterns
- [ ] Compare kinematics performance vs classic `Skeleton`

## Related Files

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **C++ source**: `dart/simulation/experimental/`
- **Python bindings**: `python/dartpy/simulation_experimental/`
- **Python tests**: `python/tests/unit/simulation_experimental/`
- **C++ tests**: `tests/unit/simulation/experimental/`
