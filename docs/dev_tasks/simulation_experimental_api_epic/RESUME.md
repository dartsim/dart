# Resume: Simulation Experimental API Epic

## Quick Status

**Phases 0, 1, 2 COMPLETE. Phase 3, 4 IN PROGRESS. Phase 5.1 (FK) COMPLETE. Compute Graph COMPLETE.**

| Phase | Status         | Description                                              |
| ----- | -------------- | -------------------------------------------------------- |
| 0     | ✅ Complete    | Ground-truth: tests, docs, example                       |
| 1     | ✅ Complete    | API cleanup: all joint types, state accessors, RigidBody |
| 2     | ✅ Complete    | Python bindings: all classes + StateSpace                |
| 3     | 🔄 In Progress | Testing strategy: golden tests done, coverage pending    |
| 4     | 🔄 In Progress | Performance: benchmarks done, profiling pending          |
| 5.1   | ✅ Complete    | Forward Kinematics: joint transforms + Link integration  |
| NEW   | ✅ Complete    | Compute Graph: Taskflow integration for parallel exec    |
| 5.2-5 | Pending        | Forward Dynamics, Collision, Constraints, step()         |
| 6     | Future         | Migration story                                          |

## Current Branch

```
Branch: feature/sim_exp
Status: Ahead of origin by 4 commits (unpushed) + uncommitted compute graph work
Working tree: Modified files pending commit
```

## Last Session Summary

Implemented multi-core compute graph infrastructure:

1. **ComputeNode** (`compute/compute_node.hpp/cpp`)
   - Unit of work with name and callable function
   - Non-copyable, movable

2. **ComputeGraph** (`compute/compute_graph.hpp/cpp`)
   - DAG of ComputeNodes with dependency edges
   - Kahn's algorithm for topological sort
   - Cycle detection on edge addition
   - Deterministic execution order

3. **SequentialExecutor** (`compute/sequential_executor.hpp/cpp`)
   - Single-threaded baseline executor

4. **TaskflowExecutor** (`compute/taskflow_executor.hpp/cpp`)
   - Parallel executor backed by Taskflow
   - Configurable thread count

5. **WorldConfig** (`world_config.hpp`)
   - ThreadPolicy enum (Auto, Static)
   - Gravity, timeStep, numThreads configuration

6. **Unit Tests** (32 tests in `test_compute_graph.cpp`)
   - ComputeNode: construction, execution, move semantics
   - ComputeGraph: add/dependency, topology, cycle detection
   - SequentialExecutor: determinism, order
   - TaskflowExecutor: parallelism, correctness

## Immediate Next Step

1. **Commit compute graph work**
2. **Push all commits** to origin
3. **Continue Phase 5.2**: Forward Dynamics (ABA algorithm)

## Files Created This Session

```
dart/simulation/experimental/compute/
├── compute_node.hpp
├── compute_node.cpp
├── compute_graph.hpp
├── compute_graph.cpp
├── compute_executor.hpp
├── sequential_executor.hpp
├── sequential_executor.cpp
├── taskflow_executor.hpp
└── taskflow_executor.cpp

dart/simulation/experimental/world_config.hpp

tests/unit/simulation/experimental/compute/
└── test_compute_graph.cpp

docs/dev_tasks/simulation_experimental_api_epic/
└── compute_graph_design.md
```

## How to Resume

```bash
# 1. Checkout and verify branch state
git checkout feature/sim_exp
git status
git log -5 --oneline

# 2. Build and verify
pixi run build
./build/default/cpp/Release/bin/test_compute_graph

# 3. Commit compute graph work
git add dart/simulation/experimental/compute/
git add dart/simulation/experimental/world_config.hpp
git add tests/unit/simulation/experimental/compute/
git add docs/dev_tasks/simulation_experimental_api_epic/compute_graph_design.md
git commit -m "feat(simulation-experimental): Add compute graph infrastructure with Taskflow"

# 4. Push to origin
git push origin feature/sim_exp

# 5. Continue to Phase 5.2 (Forward Dynamics)
```

## Test Verification Commands

```bash
# Compute graph tests (32 tests)
./build/default/cpp/Release/bin/test_compute_graph

# All C++ simulation-experimental tests (15 test binaries)
ctest -L simulation-experimental --test-dir build/default/cpp/Release

# Python tests (16 tests)
pixi run pytest python/tests/unit/simulation_experimental/ -v
```

## Compute Graph Deliverables (COMPLETE)

| Component                 | Status | Location                                        |
| ------------------------- | ------ | ----------------------------------------------- |
| ComputeNode class         | ✅     | `compute/compute_node.hpp/cpp`                  |
| ComputeGraph class        | ✅     | `compute/compute_graph.hpp/cpp`                 |
| ComputeExecutor interface | ✅     | `compute/compute_executor.hpp`                  |
| SequentialExecutor        | ✅     | `compute/sequential_executor.hpp/cpp`           |
| TaskflowExecutor          | ✅     | `compute/taskflow_executor.hpp/cpp`             |
| WorldConfig               | ✅     | `world_config.hpp`                              |
| Unit tests (32)           | ✅     | `tests/unit/.../compute/test_compute_graph.cpp` |
| Design doc                | ✅     | `docs/.../compute_graph_design.md`              |

## Phase 5.2 Next Steps (Forward Dynamics)

Based on `phase5_physics_design.md`:

1. Create `dart/simulation/experimental/dynamics/` directory
2. Implement spatial math utilities (SE3, spatial inertia)
3. Implement ABA algorithm following Featherstone
4. Add `World::computeForwardDynamics()` method
5. Wire compute graph into `World::step()`

Key reference: `dart/dynamics/Skeleton.cpp` (classic DART ABA implementation)

## Related Files

- **Epic tracker**: `docs/dev_tasks/simulation_experimental_api_epic/README.md`
- **Phase 5 design**: `docs/dev_tasks/simulation_experimental_api_epic/phase5_physics_design.md`
- **Compute graph design**: `docs/dev_tasks/simulation_experimental_api_epic/compute_graph_design.md`
- **C++ source**: `dart/simulation/experimental/`
- **Compute module**: `dart/simulation/experimental/compute/`
- **Kinematics**: `dart/simulation/experimental/kinematics/`
- **Tests**: `tests/unit/simulation/experimental/`
