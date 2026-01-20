# Multi-Core CPU Execution for DART Simulation

## Overview

This task implements parallel execution support for DART's simulation loop using a compute-graph abstraction backed by [Taskflow](https://taskflow.github.io/).

**Status**: Phase 1 Complete (Core Infrastructure)

## Goal

Enable multi-threaded simulation stepping in DART by:

1. Creating a compute-graph abstraction that models the physics pipeline as a DAG
2. Using Taskflow for parallel task scheduling
3. Providing a `WorldStepGraph` wrapper that integrates with the existing `World` class

## Architecture

### Location

The compute_graph module is in `dart/simulation/compute_graph/` (NOT in experimental).

```
dart/simulation/compute_graph/
├── fwd.hpp                 # Forward declarations, NodeId, ExecutionContext
├── compute_node.hpp        # ComputeNode base class, LambdaNode
├── compute_graph.hpp/cpp   # ComputeGraph with node/edge management, cycle detection
├── graph_executor.hpp/cpp  # GraphExecutor interface, SequentialExecutor
├── taskflow_executor.hpp/cpp # TaskflowExecutor - Taskflow backend
├── physics_nodes.hpp/cpp   # ForwardDynamicsNode, ConstraintSolveNode, etc.
└── world_step_graph.hpp/cpp # WorldStepGraph wrapper for World integration
```

### Key Design Decisions

1. **External wrapper approach**: `WorldStepGraph` wraps `dart::simulation::World` rather than modifying `World::step()` directly. This allows A/B testing and keeps the code isolated.

2. **Graph structure** (per step):
   - Phase 1: ForwardDynamics nodes (parallel per skeleton)
   - Phase 2: ConstraintSolve node (synchronization barrier)
   - Phase 3: IntegratePositions nodes (parallel per skeleton)
   - Phase 4: TimeAdvance node

3. **Determinism**: Topological sort via Kahn's algorithm ensures deterministic execution order.

4. **Backend modularity**: `GraphExecutor` interface allows swapping Taskflow for other backends (TBB, OpenMP).

### Dependencies

- **Taskflow 4.0.0+**: Already a project dependency in `pixi.toml`
- Taskflow is conditionally linked; `DART_HAVE_TASKFLOW` compile definition indicates availability

## Files Modified

| File                               | Change                                     |
| ---------------------------------- | ------------------------------------------ |
| `cmake/DARTFindDependencies.cmake` | Added Taskflow find_package                |
| `dart/CMakeLists.txt`              | Added Taskflow link and DART_HAVE_TASKFLOW |
| `dart/simulation/CMakeLists.txt`   | Added compute_graph sources                |
| `dart/simulation/World.hpp`        | Added GraphExecutionConfig struct          |
| `tests/unit/CMakeLists.txt`        | Added compute_graph tests                  |
| `tests/benchmark/CMakeLists.txt`   | Added bm_compute_graph                     |

## Usage

### Basic Usage

```cpp
#include <dart/simulation/compute_graph/world_step_graph.hpp>

auto world = dart::simulation::World::create();
// ... add skeletons ...

dart::simulation::WorldStepGraph stepGraph(*world);

// Step using compute graph (parallel if Taskflow available)
stepGraph.step();
```

### Configuration

```cpp
dart::simulation::ExecutorConfig config;
config.numWorkers = 4;           // 0 = auto, 1 = sequential
config.forceSequential = false;  // Force sequential for debugging
config.enableProfiling = true;   // Enable node timing

dart::simulation::WorldStepGraph stepGraph(*world, config);
```

## Testing

```bash
# Build and run compute_graph tests
pixi run build-tests
cd build/default/cpp/Release
ctest -R compute_graph -V
```

## Benchmarks

```bash
pixi run build
./build/default/cpp/Release/tests/benchmark/bm_compute_graph
```

## Phase Roadmap

### Phase 1 (Complete)

- [x] Core compute graph infrastructure
- [x] Sequential and Taskflow executors
- [x] Physics nodes for forward dynamics pipeline
- [x] WorldStepGraph wrapper
- [x] Unit tests
- [x] Benchmarks

### Phase 2 (Planned)

- [ ] Integration with World::step() via configuration flag
- [ ] Batch skeleton operations for better parallelism
- [ ] SIMD-optimized node implementations
- [ ] Profiling integration with Tracy

### Phase 3 (Planned)

- [ ] GPU acceleration via CUDA/OpenCL nodes
- [ ] Distributed execution support
- [ ] Dynamic graph restructuring based on workload

## Resuming This Task

If continuing from a fresh agent session:

1. Load this file: `@docs/dev_tasks/multi_core_cpu/README.md`
2. Review current status in `01_progress.md`
3. Check `02_design.md` for architectural decisions
4. Run `pixi run build && ctest -R compute_graph` to verify current state

## See Also

- `docs/onboarding/architecture.md` - DART physics pipeline
- `docs/onboarding/dynamics.md` - Forward dynamics algorithms
