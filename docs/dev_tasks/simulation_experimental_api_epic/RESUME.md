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
Status: Modified files (uncommitted dynamics code + tests)
Working tree: Modified (needs commit)
```

## Last Session Summary

Continued Phase 5.2 Forward Dynamics implementation:

1. **Fixed build errors** in `forward_dynamics.cpp`:
   - Replaced `comps::MultiBody` with `comps::MultiBodyStructure`
   - Added missing `MultiBody::getEntity()` implementation

2. **Created dynamics unit tests** (48 tests total):
   - `test_spatial_math.cpp` (15 tests) - Spatial inertia, vectors, transforms
   - `test_motion_subspace.cpp` (24 tests) - Joint S matrix for all 8 types
   - `test_articulated_body.cpp` (9 tests) - ABA workspace data structures

3. **All tests passing**: Build + lint + dynamics tests all pass

## Files Modified This Session

```
dart/simulation/experimental/
├── dynamics/forward_dynamics.cpp     # Fixed component name (MultiBodyStructure)
└── multi_body/multi_body.cpp         # Added getEntity() implementation

tests/unit/simulation/experimental/
├── CMakeLists.txt                    # Added dynamics test directory
└── dynamics/
    ├── test_spatial_math.cpp         # NEW - 15 tests
    ├── test_motion_subspace.cpp      # NEW - 24 tests
    └── test_articulated_body.cpp     # NEW - 9 tests
```

## Files Created Previously (Uncommitted)

```
dart/simulation/experimental/dynamics/
├── spatial_math.hpp        # Spatial vector types and operations
├── spatial_math.cpp        # makeSpatialInertia implementation
├── motion_subspace.hpp     # Joint motion subspace matrices (S)
├── motion_subspace.cpp     # Implementations for all 8 joint types
├── articulated_body.hpp    # ABA data structures (LinkABAData, JointABAData)
├── articulated_body.cpp    # ABAWorkspace implementation
├── forward_dynamics.hpp    # ForwardDynamicsSystem class
└── forward_dynamics.cpp    # ABA algorithm implementation

dart/simulation/experimental/CMakeLists.txt  # Modified - added dynamics sources
```

## Immediate Next Steps

1. **Commit the dynamics module** with tests
2. **Push to origin**
3. **Review ABA implementation** for correctness (velocity pass incomplete)
4. **Add integration tests** - End-to-end ABA test with known results

## How to Resume

```bash
# 1. Checkout and verify branch state
git checkout feature/sim_exp
git status
git log -5 --oneline

# 2. Build and verify tests
pixi run build
./build/default/cpp/Release/bin/test_spatial_math
./build/default/cpp/Release/bin/test_motion_subspace
./build/default/cpp/Release/bin/test_articulated_body

# 3. Commit dynamics work
git add dart/simulation/experimental/dynamics/
git add dart/simulation/experimental/multi_body/multi_body.cpp
git add dart/simulation/experimental/CMakeLists.txt
git add tests/unit/simulation/experimental/dynamics/
git add tests/unit/simulation/experimental/CMakeLists.txt
git commit -m "feat(simulation-experimental): Add forward dynamics (ABA) infrastructure

- Add dynamics/ module with spatial math, motion subspace, ABA algorithm
- Implement all 8 joint motion subspaces (Fixed, Revolute, Prismatic, etc.)
- Add ForwardDynamicsSystem with workspace management
- Add 48 unit tests for dynamics components
- Fix MultiBody::getEntity() missing implementation"

# 4. Push to origin
git push origin feature/sim_exp
```

## Test Verification Commands

```bash
# Dynamics tests (48 tests across 3 binaries)
./build/default/cpp/Release/bin/test_spatial_math
./build/default/cpp/Release/bin/test_motion_subspace
./build/default/cpp/Release/bin/test_articulated_body

# Compute graph tests (32 tests)
./build/default/cpp/Release/bin/test_compute_graph

# Full simulation-experimental unit tests
ctest -L simulation-experimental --test-dir build/default/cpp/Release

# Python tests (16 tests)
pixi run pytest python/tests/unit/simulation_experimental/ -v
```

## Phase 5.2 Deliverables Status

| Component              | Status | Location                             |
| ---------------------- | ------ | ------------------------------------ |
| Spatial math utilities | ✅     | `dynamics/spatial_math.hpp/cpp`      |
| Motion subspace (S)    | ✅     | `dynamics/motion_subspace.hpp/cpp`   |
| ABA data structures    | ✅     | `dynamics/articulated_body.hpp/cpp`  |
| ForwardDynamicsSystem  | ✅     | `dynamics/forward_dynamics.hpp/cpp`  |
| Unit tests (48)        | ✅     | `tests/.../dynamics/test_*.cpp`      |
| Velocity pass          | ⚠️     | Workspace initialized but incomplete |
| Integration test       | ❌     | Needs end-to-end validation          |

## Known Issues / TODO

1. **Velocity pass incomplete**: The ABA implementation initializes workspace but doesn't properly compute link velocities from joint velocities (Pass 1 of ABA)
2. **Gravity handling**: Bias force gravity term may need adjustment for coordinate system
3. **No integration with FK**: Should call FK first to compute link transforms
4. **No end-to-end test**: Need a simple pendulum or chain test to validate full ABA

## Related Files

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **Phase 5 design**: `docs/dev_tasks/simulation_experimental_api_epic/phase5_physics_design.md`
- **Compute graph design**: `docs/dev_tasks/simulation_experimental_api_epic/compute_graph_design.md`
- **C++ source**: `dart/simulation/experimental/`
- **Dynamics module**: `dart/simulation/experimental/dynamics/`
- **Kinematics**: `dart/simulation/experimental/kinematics/`
- **Tests**: `tests/unit/simulation/experimental/`
- **Classic DART ABA reference**: `dart/dynamics/Skeleton.cpp:3675`
