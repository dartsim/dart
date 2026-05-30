# Multi-Core CPU Compute Graph Design

> **Status**: Archived reference — extracted from `archive/obsolete-multi-core-2026-05-20`
> (formerly `feature/multi_core`). The branch was marked obsolete because
> Taskflow-based CPU parallelism only breaks even at ~32 skeletons, and
> underperforms for DART's typical 1-20 skeleton use case. The architecture
> patterns and pipeline decomposition are backend-agnostic and worth preserving.

## Requirements

1. **Parallel skeleton processing**: Multiple skeletons should be processed in
   parallel when possible
2. **Determinism**: Same inputs must produce same outputs regardless of thread
   count
3. **Backward compatibility**: Existing `World::step()` API must continue to work
4. **Modularity**: Execution backend should be swappable (Taskflow, TBB, etc.)
5. **Minimal overhead**: Sequential execution should have negligible overhead
   vs current implementation

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

### Batched Form (WorldStepGraph)

```
[BatchForwardDynamics 0..B-1] --\
[BatchForwardDynamics B..2B-1] ---+--> [ConstraintSolve] --> [BatchIntegratePositions 0..B-1] --\
[BatchForwardDynamics ...] ------/                             [BatchIntegratePositions B..2B-1] ---+--> [TimeAdvance]
                                                                  [BatchIntegratePositions ...] -----/
```

When batching is disabled (batchSize = 1), each batch node is replaced with
per-skeleton ForwardDynamics/IntegratePositions nodes. If there are no mobile
skeletons, the graph reduces to `ConstraintSolve -> TimeAdvance`.

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

Container for nodes and edges with topological sort (Kahn's algorithm) and
cycle detection.

```cpp
class ComputeGraph {
public:
  NodeId addNode(ComputeNodePtr node);
  bool addEdge(NodeId from, NodeId to);  // false if cycle detected
  void clear();
  const std::vector<NodeId>& topologicalOrder() const;
  ComputeNode* getNode(NodeId id) const;
  const std::vector<NodeId>& getDependencies(NodeId id) const;
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

Parallel executor using the Taskflow library.

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

1. **Topological sort**: Kahn's algorithm produces a unique ordering for nodes
   at the same level
2. **Node IDs**: Stable identifiers ensure consistent ordering across runs
3. **Synchronization barriers**: ConstraintSolve ensures all forward dynamics
   complete before integration

## Thread Safety

- `ComputeGraph` is NOT thread-safe; build graph before execution
- `GraphExecutor::execute()` is thread-safe; callable from any thread
- Individual nodes must not modify shared state without synchronization
- `ExecutionContext` is passed by const reference

## Configuration

```cpp
struct GraphExecutionConfig {
  bool enableComputeGraph{false};
  std::size_t numWorkers{0};  // 0 = auto, 1 = sequential
  bool forceSequential{false};
  std::size_t batchSize{0};   // 0 = auto, 1 = per-skeleton
  bool enableProfiling{false};
};
```

## Performance Profile

| Skeletons | Sequential | Graph (auto-batch) | Speedup |
| --------- | ---------- | ------------------ | ------- |
| 1         | 3,153 ns   | 5,438 ns           | 0.58x   |
| 8         | 14,770 ns  | 25,249 ns          | 0.58x   |
| 16        | 28,763 ns  | 39,764 ns          | 0.72x   |
| 32        | 61,684 ns  | 64,440 ns          | 0.96x   |
| 64        | 112,683 ns | 105,029 ns         | 1.07x   |

Cross-over at ~32 skeletons; 2-5x speedup for 256+ skeletons with batching.

## See Also

- `docs/onboarding/architecture.md` — DART physics pipeline
- `docs/onboarding/dynamics.md` — Forward dynamics algorithms
