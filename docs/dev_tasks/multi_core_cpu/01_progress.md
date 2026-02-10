# Progress Log: Multi-Core CPU Execution

## Current Status

| Field            | Value                                          |
| ---------------- | ---------------------------------------------- |
| **Phase**        | 2 (World Integration) + Test Hardening         |
| **Status**       | Complete — PR-ready                             |
| **Branch**       | `feature/multi_core` (8 commits ahead of main) |
| **Last Updated** | 2026-02-10                                     |

**TL;DR**: Compute graph module complete and integrated. Comprehensive tests prove bit-exact correctness vs sequential path (200 multi-step, multi-DOF). Real-world benchmarks show 2–5x speedup for 32+ skeleton scenarios with batching.

## Completed Tasks

### Session 1: Initial Implementation (in experimental/)

- [x] Created compute_graph directory structure
- [x] Implemented `fwd.hpp` with forward declarations, NodeId, ExecutionContext
- [x] Implemented `ComputeNode` base class and `LambdaNode`
- [x] Implemented `ComputeGraph` with node/edge management and topological sort
- [x] Implemented `GraphExecutor` interface and `SequentialExecutor`
- [x] Implemented `TaskflowExecutor` backend
- [x] Implemented physics nodes: ForwardDynamicsNode, ConstraintSolveNode, IntegratePositionsNode
- [x] Implemented `WorldStepGraph` wrapper for World integration
- [x] Extended WorldConfig with `GraphExecutionConfig`
- [x] Created unit tests
- [x] Created benchmarks

### Session 2: Relocation to Main Library

- [x] Moved compute_graph from `dart/simulation/experimental/` to `dart/simulation/`
- [x] Updated namespace from `dart::simulation::experimental` to `dart::simulation`
- [x] Updated export macro from `DART_EXPERIMENTAL_API` to `DART_API`
- [x] Added Taskflow dependency to main dart library (cmake/DARTFindDependencies.cmake)
- [x] Added Taskflow linkage to dart/CMakeLists.txt
- [x] Updated dart/simulation/CMakeLists.txt to include compute_graph sources
- [x] Moved tests to `tests/unit/simulation/compute_graph/`
- [x] Moved benchmarks to `tests/benchmark/simulation/`
- [x] Verified build passes: `pixi run build`
- [x] Verified tests pass: `ctest -R compute_graph` (8/8 passed)

### Session 3: Phase 2 Integration and Profiling

- [x] Integrated compute graph into `World::step()` via configuration flag
- [x] Added batching support for skeleton nodes in `WorldStepGraph`
- [x] Added profiling hooks (Tracy/text + callbacks) to graph executors
- [x] Added graph rebuild triggers for skeleton/time step/solver changes
- [x] Added unit tests for WorldStepGraph integration and batching

### Session 4: Test Hardening and Real-World Benchmarks

- [x] Added multi-step correctness test (200 steps, 3 multi-DOF skeletons, bit-exact)
- [x] Added edge case tests (empty world, single skeleton, immobile-only, WorldConfig constructor)
- [x] Added TaskflowExecutor direct parallel execution test
- [x] Added realistic World::step() benchmarks with actual physics (8-DOF skeletons)
- [x] Verified all 18 tests pass, no regressions in broader test suite
- [x] Collected benchmark evidence proving benefit with batching

## Performance: Abstract Graph (2026-01-19)

Command:

```bash
pixi run bm bm_compute_graph -- --benchmark_min_time=0.1s --benchmark_format=json
```

Context: host `mark26`, 32 CPUs @ 5300 MHz, build `release`, benchmark lib
`v1.9.4`.

Selected results (real_time):

- Linear graph 256 nodes: Sequential 30.8 us, Taskflow 238.9 us (0.13x).
- Parallel graph 64 nodes: Sequential 37.9 us, Taskflow 249.8 us (0.15x).
- Thread scaling (32-parallel graph): 1 thread 41.7 us, 2 threads 43.7 us
  (0.95x), 4 threads 48.6 us (0.86x), 8 threads 71.7 us (0.58x).

Notes: Abstract graph microbenchmarks emphasize scheduling overhead; Taskflow is
slower for small per-node work.

## Performance: Real World::step() (2026-02-10)

Command:

```bash
./build/default/cpp/Release/bin/bm_compute_graph --benchmark_filter="BM_WorldStep" --benchmark_min_time=0.5s
```

Context: same host, 32 CPUs @ 5300 MHz, 8-DOF skeletons (FreeJoint→RevoluteJoint×2).

### Sequential vs Graph (auto-batch)

| Skeletons | Sequential (CPU) | Graph auto (CPU) | Speedup |
| --------- | ---------------- | ---------------- | ------- |
| 1         | 3,153 ns         | 5,438 ns         | 0.58x   |
| 8         | 14,770 ns        | 25,249 ns        | 0.58x   |
| 16        | 28,763 ns        | 39,764 ns        | 0.72x   |
| 32        | 61,684 ns        | 64,440 ns        | 0.96x   |
| 64        | 112,683 ns       | 105,029 ns       | 1.07x   |

### Batching effect (graph vs sequential)

| Skeletons | Batch | Graph (CPU)  | vs Sequential |
| --------- | ----- | ------------ | ------------- |
| 32        | 1     | 65,612 ns    | 0.94x         |
| 32        | 4     | 25,440 ns    | **2.42x**     |
| 32        | 8     | 11,069 ns    | **5.57x**     |
| 64        | 1     | 105,181 ns   | 1.07x         |
| 64        | 4     | 48,286 ns    | **2.33x**     |
| 64        | 8     | 20,379 ns    | **5.53x**     |

Key insight: Batching is the key to benefit. With batch=4+ and 32+ skeletons,
the graph path is 2–5x faster than sequential. Without batching, overhead
dominates for small skeleton counts.

## Next Steps (Phase 3)

1. **GPU Acceleration**
   - Implement CUDA/OpenCL compute nodes
   - Add device selection/configuration API

2. **Distributed Execution**
   - Prototype multi-process graph executor
   - Define serialization format for graph state

3. **Dynamic Graph Restructuring**
   - Add workload-aware graph rebuilding heuristics
   - Support incremental updates when skeletons are added/removed

## Known Issues

None currently.

## Verification Commands

```bash
# Build
pixi run build

# Run compute_graph tests
pixi run build-tests
cd build/default/cpp/Release && ctest -R compute_graph -V

# Run benchmarks
./build/default/cpp/Release/tests/benchmark/bm_compute_graph
```

## Files Changed (This Session)

```
dart/simulation/compute_graph/          # NEW - moved from experimental
├── fwd.hpp
├── compute_node.hpp
├── compute_graph.hpp
├── compute_graph.cpp
├── graph_executor.hpp
├── graph_executor.cpp
├── taskflow_executor.hpp
├── taskflow_executor.cpp
├── physics_nodes.hpp
├── physics_nodes.cpp
├── world_step_graph.hpp
└── world_step_graph.cpp

cmake/DARTFindDependencies.cmake        # Added Taskflow find_package
dart/CMakeLists.txt                     # Added Taskflow link
dart/simulation/CMakeLists.txt          # Added compute_graph sources
dart/simulation/World.hpp               # Added GraphExecutionConfig

tests/unit/simulation/compute_graph/    # NEW - moved from experimental
└── test_compute_graph.cpp
tests/unit/CMakeLists.txt               # Added compute_graph test

tests/benchmark/simulation/             # NEW location
└── bm_compute_graph.cpp
tests/benchmark/CMakeLists.txt          # Added bm_compute_graph
```

## Files Changed (Session 3)

```
dart/simulation/compute_graph/fwd.hpp
dart/simulation/compute_graph/graph_executor.hpp
dart/simulation/compute_graph/graph_executor.cpp
dart/simulation/compute_graph/taskflow_executor.cpp
dart/simulation/compute_graph/world_step_graph.hpp
dart/simulation/compute_graph/world_step_graph.cpp
dart/simulation/World.hpp
dart/simulation/World.cpp

tests/unit/simulation/compute_graph/test_world_step_graph.cpp
tests/unit/CMakeLists.txt
```
