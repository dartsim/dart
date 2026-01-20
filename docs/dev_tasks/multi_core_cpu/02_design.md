# Design: Multi-Core CPU Execution

## Overview

This document describes the design of the compute-graph based parallel execution system for DART.

## Requirements

1. **Parallel skeleton processing**: Multiple skeletons should be processed in parallel when possible
2. **Determinism**: Same inputs must produce same outputs regardless of thread count
3. **Backward compatibility**: Existing World::step() API must continue to work
4. **Modularity**: Execution backend should be swappable (Taskflow, TBB, etc.)
5. **Minimal overhead**: Sequential execution should have negligible overhead vs current implementation

## Architecture

### Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         World                                    │
│  ┌──────────────┐                                               │
│  │ WorldConfig  │                                               │
│  │ ┌──────────────────────┐                                     │
│  │ │GraphExecutionConfig  │                                     │
│  │ │ - numWorkers         │                                     │
│  │ │ - forceSequential    │                                     │
│  │ └──────────────────────┘                                     │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
                                │
                                │ wraps
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      WorldStepGraph                              │
│  - Builds compute graph from World state                        │
│  - Creates physics nodes for each skeleton                      │
│  - Manages graph execution                                      │
└─────────────────────────────────────────────────────────────────┘
                                │
                                │ uses
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ComputeGraph                               │
│  - DAG of ComputeNode instances                                 │
│  - Topological sort (Kahn's algorithm)                          │
│  - Cycle detection                                              │
│  - Edge management (dependencies)                               │
└─────────────────────────────────────────────────────────────────┘
                                │
                                │ executed by
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      GraphExecutor                               │
│              (interface)                                         │
├─────────────────────────────────────────────────────────────────┤
│  SequentialExecutor    │    TaskflowExecutor                    │
│  - Single-threaded     │    - Taskflow backend                  │
│  - Deterministic       │    - Parallel execution                │
│  - Debug/reference     │    - Work stealing                     │
└─────────────────────────────────────────────────────────────────┘
```

### Physics Pipeline as Compute Graph

```
        ┌─────────────────────┐
        │   ForwardDynamics   │──┐
        │    (Skeleton 0)     │  │
        └─────────────────────┘  │
        ┌─────────────────────┐  │
        │   ForwardDynamics   │──┼──────┐
        │    (Skeleton 1)     │  │      │
        └─────────────────────┘  │      ▼
        ┌─────────────────────┐  │  ┌─────────────────────┐
        │   ForwardDynamics   │──┘  │  ConstraintSolve    │
        │    (Skeleton N)     │─────│  (synchronization)  │
        └─────────────────────┘     └─────────────────────┘
                                            │
        ┌───────────────────────────────────┼───────────────────────┐
        │                                   │                       │
        ▼                                   ▼                       ▼
┌─────────────────────┐       ┌─────────────────────┐       ┌─────────────────────┐
│ IntegratePositions  │       │ IntegratePositions  │       │ IntegratePositions  │
│   (Skeleton 0)      │       │   (Skeleton 1)      │       │   (Skeleton N)      │
└─────────────────────┘       └─────────────────────┘       └─────────────────────┘
        │                                   │                       │
        └───────────────────────────────────┼───────────────────────┘
                                            │
                                            ▼
                                 ┌─────────────────────┐
                                 │    TimeAdvance      │
                                 │  (update world t)   │
                                 └─────────────────────┘
```

## Key Classes

### ExecutionContext

Passed to all nodes during execution. Contains simulation state.

```cpp
struct ExecutionContext {
  double timeStep{0.001};
  double currentTime{0.0};
  std::uint64_t frameNumber{0};
  bool resetCommand{true};
  void* worldPtr{nullptr};  // Opaque World pointer
};
```

### ComputeNode

Base class for all compute nodes.

```cpp
class ComputeNode {
public:
  virtual ~ComputeNode() = default;
  virtual void execute(const ExecutionContext& ctx) = 0;
  virtual std::string name() const = 0;

  NodeId id() const { return mId; }

private:
  NodeId mId{kInvalidNodeId};
};
```

### ComputeGraph

Container for nodes and edges with topological sort.

```cpp
class ComputeGraph {
public:
  NodeId addNode(ComputeNodePtr node);
  bool addEdge(NodeId from, NodeId to);  // Returns false if cycle detected
  void clear();

  const std::vector<NodeId>& topologicalOrder() const;
  ComputeNode* getNode(NodeId id) const;
  const std::vector<NodeId>& getDependencies(NodeId id) const;

private:
  void recomputeTopologicalOrder();  // Kahn's algorithm
  bool detectCycle() const;
};
```

### GraphExecutor

Interface for execution backends.

```cpp
class GraphExecutor {
public:
  virtual ~GraphExecutor() = default;
  virtual void execute(ComputeGraph& graph, const ExecutionContext& ctx) = 0;
  virtual std::string name() const = 0;
};
```

### TaskflowExecutor

Parallel executor using Taskflow.

```cpp
class TaskflowExecutor : public GraphExecutor {
public:
  explicit TaskflowExecutor(std::size_t numWorkers = 0);
  void execute(ComputeGraph& graph, const ExecutionContext& ctx) override;

private:
  tf::Executor mExecutor;
};
```

## Determinism

Determinism is guaranteed by:

1. **Topological sort**: Kahn's algorithm produces a unique ordering for nodes at the same level
2. **Node IDs**: Stable node identifiers ensure consistent ordering across runs
3. **Synchronization barriers**: ConstraintSolve node ensures all forward dynamics complete before integration

## Thread Safety

- `ComputeGraph` is NOT thread-safe; build graph before execution
- `GraphExecutor::execute()` is thread-safe; can be called from any thread
- Individual nodes must not modify shared state without synchronization
- `ExecutionContext` is passed by const reference

## Configuration

```cpp
struct GraphExecutionConfig {
  std::size_t numWorkers{0};  // 0 = auto, 1 = sequential
  bool forceSequential{false};
};
```

## Future Considerations

### GPU Nodes

```cpp
class CudaForwardDynamicsNode : public ComputeNode {
  // Batch process all skeletons on GPU
};
```

### Profiling

```cpp
struct ExecutorConfig {
  bool enableProfiling{false};
  std::function<void(NodeId, double)> onNodeComplete;
};
```

### Dynamic Graph Restructuring

For scenarios where skeleton count changes, support incremental graph updates:

```cpp
class WorldStepGraph {
  void onSkeletonAdded(SkeletonPtr skel);
  void onSkeletonRemoved(SkeletonPtr skel);
};
```
