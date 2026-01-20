# Progress Log: Multi-Core CPU Execution

## Current Status

**Phase**: 1 (Core Infrastructure)
**Status**: Complete
**Last Updated**: 2025-01-19

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

## Next Steps (Phase 2)

1. **Integration with World::step()**
   - Add configuration flag to World to use graph-based stepping
   - Wire WorldStepGraph into World::step() when enabled

2. **Performance Optimization**
   - Profile with Tracy to identify bottlenecks
   - Implement batch skeleton operations
   - Consider SIMD optimizations

3. **Testing**
   - Add integration tests with actual physics scenarios
   - Compare numerical results between sequential and parallel execution
   - Add stress tests with many skeletons

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
