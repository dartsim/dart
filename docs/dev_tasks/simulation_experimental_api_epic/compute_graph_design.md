# Compute Graph Foundation Design

## Overview

This document describes the design for adding multi-core CPU execution to DART's simulation loop via a compute-graph abstraction backed by Taskflow.

**Goal**: Introduce an explicit compute-graph API for the World/Solver simulation loop that makes dependencies and ordering explicit, enabling future parallel execution.

**Non-Goal**: This is NOT "parallelize everything." Phase 1 focuses on the foundation - even if many nodes still execute serially.

---

## Current State

### Classic DART Simulation Loop (`World::step()`)

```cpp
void World::step(bool _resetCommand) {
    // Stage 1: Forward Dynamics + Velocity Integration (per skeleton, independent)
    for (auto& skel : mSkeletons) {
        skel->computeForwardDynamics();  // ABA algorithm
        skel->integrateVelocities(mTimeStep);
    }

    // Stage 2: Constraint Solving (global, depends on Stage 1)
    mConstraintSolver->solve();  // Collision detection + constraint resolution

    // Stage 3: Impulse Dynamics + Position Integration (per skeleton, depends on Stage 2)
    for (auto& skel : mSkeletons) {
        if (skel->isImpulseApplied()) {
            skel->computeImpulseForwardDynamics();
        }
        skel->integratePositions(mTimeStep);
    }

    // Stage 4: Cleanup (depends on Stage 3)
    mTime += mTimeStep;
    mSensorManager.updateSensors(*this);
}
```

### Parallelization Opportunities

| Stage | Operation            | Parallelizable?        | Notes                        |
| ----- | -------------------- | ---------------------- | ---------------------------- |
| 1a    | Forward Dynamics     | **Yes (per skeleton)** | Skeletons are independent    |
| 1b    | Velocity Integration | **Yes (per skeleton)** | After own FD complete        |
| 2a    | Collision Detection  | **Partially**          | Broad-phase can be parallel  |
| 2b    | Constraint Solving   | **Limited**            | LCP is inherently sequential |
| 3a    | Impulse Dynamics     | **Yes (per skeleton)** | After constraint solve       |
| 3b    | Position Integration | **Yes (per skeleton)** | After own impulse dynamics   |
| 4     | Sensor Update        | **Yes (per sensor)**   | Independent operations       |

---

## Compute Graph API Design

### Core Abstractions

```cpp
namespace dart::simulation::experimental::compute {

/// A node in the compute graph representing a unit of work
class ComputeNode {
public:
    using ExecuteFn = std::function<void()>;

    explicit ComputeNode(std::string name, ExecuteFn fn);

    const std::string& getName() const;
    void execute();

private:
    std::string m_name;
    ExecuteFn m_fn;
};

/// Edge representing a dependency between nodes
struct ComputeEdge {
    ComputeNode* from;  // Must complete before
    ComputeNode* to;    // This node can start
};

/// The compute graph representing the simulation pipeline
class ComputeGraph {
public:
    /// Add a node to the graph
    ComputeNode& addNode(std::string name, ComputeNode::ExecuteFn fn);

    /// Add dependency: 'from' must complete before 'to' can start
    void addDependency(ComputeNode& from, ComputeNode& to);

    /// Get all nodes in topological order
    std::vector<ComputeNode*> getTopologicalOrder() const;

    /// Validate graph (no cycles, all dependencies satisfied)
    bool validate() const;

private:
    std::vector<std::unique_ptr<ComputeNode>> m_nodes;
    std::vector<ComputeEdge> m_edges;
};

/// Abstract executor interface for running compute graphs
class ComputeExecutor {
public:
    virtual ~ComputeExecutor() = default;

    /// Execute the graph
    virtual void execute(const ComputeGraph& graph) = 0;

    /// Get number of worker threads
    virtual std::size_t getWorkerCount() const = 0;
};

/// Sequential executor (baseline, single-threaded)
class SequentialExecutor : public ComputeExecutor {
public:
    void execute(const ComputeGraph& graph) override;
    std::size_t getWorkerCount() const override { return 1; }
};

/// Taskflow-backed parallel executor
class TaskflowExecutor : public ComputeExecutor {
public:
    explicit TaskflowExecutor(std::size_t numThreads = 0);  // 0 = auto

    void execute(const ComputeGraph& graph) override;
    std::size_t getWorkerCount() const override;

private:
    std::unique_ptr<tf::Executor> m_executor;
};

} // namespace dart::simulation::experimental::compute
```

### WorldConfig Extension

```cpp
/// Thread count policy for parallel execution
enum class ThreadPolicy {
    Auto,     // Use system default (Taskflow decides)
    Static,   // Use fixed number specified
    // Future: Dynamic, Affinity, etc.
};

struct WorldConfig {
    // Existing fields...
    Eigen::Vector3d gravity = {0, 0, -9.81};
    double timeStep = 0.001;

    // NEW: Parallel execution configuration
    ThreadPolicy threadPolicy = ThreadPolicy::Auto;
    std::size_t numThreads = 0;  // Only used if threadPolicy == Static

    // Future options (documented for roadmap)
    // bool enableWorkStealing = true;
    // std::size_t taskGranularity = 1;  // Min skeletons per task
};
```

### Integration with World::step()

```cpp
class World {
public:
    /// Build the compute graph for a single simulation step
    /// This maps the sequential loop to explicit graph nodes
    ComputeGraph buildStepGraph();

    /// Execute one simulation step using the configured executor
    void step();

    /// Set the executor (default: auto-configured based on WorldConfig)
    void setExecutor(std::unique_ptr<ComputeExecutor> executor);

private:
    std::unique_ptr<ComputeExecutor> m_executor;
    WorldConfig m_config;
};
```

---

## Graph Structure for step()

```
                    ┌─────────────────────────────────────────┐
                    │              Simulation Step            │
                    └─────────────────────────────────────────┘
                                        │
          ┌─────────────────────────────┼─────────────────────────────┐
          ▼                             ▼                             ▼
    ┌───────────┐                 ┌───────────┐                 ┌───────────┐
    │ Skel_0_FD │                 │ Skel_1_FD │       ...       │ Skel_N_FD │
    │ (dynamics)│                 │ (dynamics)│                 │ (dynamics)│
    └─────┬─────┘                 └─────┬─────┘                 └─────┬─────┘
          │                             │                             │
          ▼                             ▼                             ▼
    ┌───────────┐                 ┌───────────┐                 ┌───────────┐
    │ Skel_0_VI │                 │ Skel_1_VI │       ...       │ Skel_N_VI │
    │(vel integ)│                 │(vel integ)│                 │(vel integ)│
    └─────┬─────┘                 └─────┬─────┘                 └─────┬─────┘
          │                             │                             │
          └─────────────────────────────┼─────────────────────────────┘
                                        │
                                        ▼
                              ┌─────────────────┐
                              │ Constraint Solve│
                              │   (barrier)     │
                              └────────┬────────┘
                                       │
          ┌────────────────────────────┼────────────────────────────┐
          ▼                            ▼                            ▼
    ┌───────────┐                ┌───────────┐                ┌───────────┐
    │ Skel_0_ID │                │ Skel_1_ID │      ...       │ Skel_N_ID │
    │(imp dyn)  │                │(imp dyn)  │                │(imp dyn)  │
    └─────┬─────┘                └─────┬─────┘                └─────┬─────┘
          │                            │                            │
          ▼                            ▼                            ▼
    ┌───────────┐                ┌───────────┐                ┌───────────┐
    │ Skel_0_PI │                │ Skel_1_PI │      ...       │ Skel_N_PI │
    │(pos integ)│                │(pos integ)│                │(pos integ)│
    └─────┬─────┘                └─────┬─────┘                └─────┬─────┘
          │                            │                            │
          └────────────────────────────┼────────────────────────────┘
                                       │
                                       ▼
                              ┌─────────────────┐
                              │  Update Time    │
                              │  + Sensors      │
                              └─────────────────┘
```

---

## Taskflow Integration

### Executor Construction

```cpp
TaskflowExecutor::TaskflowExecutor(std::size_t numThreads)
{
    if (numThreads == 0) {
        // Auto: Let Taskflow use hardware concurrency
        m_executor = std::make_unique<tf::Executor>();
    } else {
        // Static: Use specified thread count
        m_executor = std::make_unique<tf::Executor>(numThreads);
    }
}
```

### Graph-to-Taskflow Translation

```cpp
void TaskflowExecutor::execute(const ComputeGraph& graph)
{
    tf::Taskflow taskflow;

    // Map our nodes to Taskflow tasks
    std::unordered_map<ComputeNode*, tf::Task> taskMap;

    for (auto* node : graph.getNodes()) {
        taskMap[node] = taskflow.emplace([node]() {
            node->execute();
        }).name(node->getName());
    }

    // Add dependencies
    for (const auto& edge : graph.getEdges()) {
        taskMap[edge.from].precede(taskMap[edge.to]);
    }

    // Execute and wait
    m_executor->run(taskflow).wait();
}
```

---

## File Structure

```
dart/simulation/experimental/
├── compute/
│   ├── compute_node.hpp
│   ├── compute_node.cpp
│   ├── compute_graph.hpp
│   ├── compute_graph.cpp
│   ├── compute_executor.hpp
│   ├── sequential_executor.hpp
│   ├── sequential_executor.cpp
│   ├── taskflow_executor.hpp
│   └── taskflow_executor.cpp
├── world.hpp          # Add buildStepGraph(), setExecutor()
├── world.cpp          # Implement step() with graph
└── world_config.hpp   # Add ThreadPolicy, numThreads
```

---

## Validation & Testing

### Unit Tests

```cpp
// Test graph construction
TEST(ComputeGraph, TopologicalOrder) {
    ComputeGraph graph;
    auto& a = graph.addNode("A", [](){});
    auto& b = graph.addNode("B", [](){});
    auto& c = graph.addNode("C", [](){});

    graph.addDependency(a, b);  // A -> B
    graph.addDependency(a, c);  // A -> C
    graph.addDependency(b, c);  // B -> C

    auto order = graph.getTopologicalOrder();
    // A must come before B and C; B must come before C
}

// Test determinism
TEST(ComputeGraph, DeterministicExecution) {
    std::vector<int> order;
    ComputeGraph graph;
    // ... build graph with nodes that record execution order

    // Execute multiple times, verify same order
    for (int i = 0; i < 10; ++i) {
        order.clear();
        SequentialExecutor().execute(graph);
        EXPECT_EQ(order, expectedOrder);
    }
}

// Test equivalence between sequential and parallel
TEST(ComputeExecutor, ResultsEquivalent) {
    World worldSeq(WorldConfig{.threadPolicy = ThreadPolicy::Static, .numThreads = 1});
    World worldPar(WorldConfig{.threadPolicy = ThreadPolicy::Static, .numThreads = 4});

    // Add same skeletons to both
    // Run same number of steps
    // Compare final states (within tolerance)
}

// Test thread count configuration
TEST(TaskflowExecutor, RespectsThreadCount) {
    TaskflowExecutor exec(4);
    EXPECT_EQ(exec.getWorkerCount(), 4);
}
```

### Benchmarks

```cpp
// Baseline vs graph execution
BENCHMARK(BM_StepSequential)->Arg(10)->Arg(100)->Arg(1000);  // N skeletons
BENCHMARK(BM_StepTaskflow)->Arg(10)->Arg(100)->Arg(1000);

// Scaling across CPU counts
BENCHMARK(BM_StepScaling)
    ->Args({100, 1})   // 100 skeletons, 1 thread
    ->Args({100, 2})
    ->Args({100, 4})
    ->Args({100, 8});

// Overhead for small workloads (ensure no regression)
BENCHMARK(BM_StepOverhead)->Arg(1)->Arg(2)->Arg(5);
```

---

## Phased Roadmap

### Phase 1: Foundation (This Document) ✅

- [ ] ComputeNode, ComputeGraph, ComputeExecutor interfaces
- [ ] SequentialExecutor (baseline)
- [ ] TaskflowExecutor with static thread count
- [ ] WorldConfig.numThreads (Static policy only)
- [ ] Unit tests for graph construction + determinism
- [ ] Benchmarks comparing sequential vs Taskflow

### Phase 2: Parallelize Hotspots

- [ ] Parallel forward dynamics (per-skeleton)
- [ ] Parallel position/velocity integration
- [ ] Benchmark improvements

### Phase 3: Advanced Policies

- [ ] ThreadPolicy::Auto (runtime detection)
- [ ] ThreadPolicy::Dynamic (adjust based on load)
- [ ] Work stealing configuration
- [ ] Task granularity tuning

### Phase 4: Multi-Solver Pipelines

- [ ] Multiple constraint solvers
- [ ] Custom pipeline stages
- [ ] Pipeline composition API

---

## References

- [Taskflow Documentation](https://taskflow.github.io/)
- Classic DART: `dart/simulation/World.cpp`
- Experimental API: `dart/simulation/experimental/`
