---
description: DART dynamics algorithm and performance optimization specialist
mcp:
  performance-profiler:
    command: python3
    args: ["-m", "http.server", "8002"]
    working_dir: "/tmp/perf-profiler"
    description: "Local performance analysis server"
---

# Performance Optimization Skill

## Purpose

Specialized skill for optimizing DART's performance-critical algorithms, particularly the Featherstone Articulated Body Algorithm and collision detection. Focus on maintaining O(n) complexity while improving real-world performance.

## When to Use This Skill

### Dynamics Algorithm Optimization

- Optimize forward kinematics computation
- Improve constraint solver performance
- Accelerate Jacobian calculations
- Enhance numerical integration schemes
- Memory access pattern optimization

### Collision Detection Optimization

- Broad-phase filtering improvements
- Narrow-phase algorithm tuning
- Spatial data structure optimization
- Multi-threading collision queries
- Cache optimization for static scenes

### Memory and Cache Optimization

- Improve data locality in critical loops
- Optimize memory layout for SIMD
- Reduce memory allocations in hot paths
- Cache-friendly algorithm design

### System-Level Performance

- Multi-threading strategy for large simulations
- SIMD vectorization opportunities
- GPU acceleration potential
- Profiling and bottleneck identification

## Key Tools and Commands

```bash
# Performance profiling with different tools
pixi run benchmark --profile dynamics
pixi run benchmark --profile collision
pixi run -e debug build --config DART_BUILD_PROFILE_TRACY=ON

# Specific algorithm benchmarking
pixi run benchmark --filter forward-kinematics
pixi run benchmark --filter jacobian-computation
pixi run benchmark --filter constraint-solver

# Memory profiling
pixi run -e debug build --config DART_ENABLE_ASAN=ON
pixi run -e debug build --config DART_BUILD_TSAN=ON

# Cache performance analysis
pixi run benchmark --cache-analysis
perf record --call-graph pixi run benchmark dynamics
perf report --stdio perf.data
```

## Performance Optimization Patterns

### 1. Data Layout Optimization

```cpp
// Wrong: Poor cache locality
class Skeleton {
    std::vector<BodyNode*> bodyNodes;
    std::vector<Joint*> joints;
    // Random memory access patterns
};

// Correct: Cache-friendly layout
class Skeleton {
    struct alignas(64) BodyNodeBlock {
        BodyNode nodes[4];  // Sequential access
        Joint joints[3];     // Related data together
    };
    std::vector<BodyNodeBlock> optimizedBlocks;
};
```

### 2. SIMD Vectorization

```cpp
// Wrong: Scalar operations
for (size_t i = 0; i < n; ++i) {
    result[i] = a[i] * b[i] + c[i];
}

// Correct: Vectorized operations
for (size_t i = 0; i < n; i += 4) {
    auto av = Eigen::Vector4d::Map(&a[i]);
    auto bv = Eigen::Vector4d::Map(&b[i]);
    auto cv = Eigen::Vector4d::Map(&c[i]);
    Eigen::Vector4d rv = av.cwiseProduct(bv) + cv;
    rv.writeToSegment(&result[i], 4);
}
```

### 3. Memory Access Patterns

```cpp
// Wrong: Cache-thrashing random access
for (size_t i = 0; i < n; ++i) {
    auto& body = bodies[indices[i]];  // Random memory access
    body->updateInertia();
}

// Correct: Sequential access pattern
for (auto& body : bodies) {  // Sequential memory access
    body->updateInertia();
}
// Or use permutation to make sequential:
for (size_t i = 0; i < n; ++i) {
    auto& body = bodies[optimizedIndices[i]];
    body->updateInertia();
}
```

## Critical DART Hotspots

### 1. Forward Kinematics Chain

**Location**: `Skeleton::computeForwardKinematics()`
**Impact**: Called every timestep, O(n) complexity
**Optimization Targets**:

- Transform propagation efficiency
- Matrix multiplication ordering
- Trigonometric function caching

### 2. Constraint Solver

**Location**: Constraint resolution in dynamics integration
**Impact**: Determines simulation stability and speed
**Optimization Targets**:

- Sparse matrix operations
- Iterative solver convergence
- Preconditioning strategies

### 3. Collision Detection Pipeline

**Location**: Broad-phase → narrow-phase filtering
**Impact**: Dominates performance for complex scenes
**Optimization Targets**:

- Spatial partitioning efficiency
- Early-out implementations
- Multi-threading scalability

## Optimization Strategies

### 1. Algorithm-Level Optimizations

```cpp
// Cache transform computations
class OptimizedSkeleton : public Skeleton {
private:
    mutable std::vector<bool> transformDirty;
    mutable std::vector<Eigen::Isometry3d> cachedTransforms;

public:
    void computeForwardKinematics() override {
        for (size_t i = 0; i < mBodyNodes.size(); ++i) {
            if (transformDirty[i]) {
                cachedTransforms[i] = computeTransform(i);
                transformDirty[i] = false;
            }
        }
    }

    void markTransformDirty(size_t index) {
        transformDirty[index] = true;
        // Mark all descendants as dirty
        for (size_t child : getDescendants(index)) {
            transformDirty[child] = true;
        }
    }
};
```

### 2. Memory Management Optimization

```cpp
// Use object pools for frequent allocations
class BodyNodePool {
private:
    std::vector<std::unique_ptr<BodyNode>> pool;
    std::queue<size_t> availableIndices;

public:
    BodyNode* acquire() {
        if (availableIndices.empty()) {
            pool.emplace_back(std::make_unique<BodyNode>());
            return pool.back().get();
        }
        auto index = availableIndices.front();
        availableIndices.pop();
        return pool[index].get();
    }

    void release(BodyNode* node) {
        // Find pool index and mark as available
    }
};
```

### 3. Multi-threading Strategy

```cpp
// Parallel collision detection
class ParallelCollisionDetector : public CollisionDetector {
public:
    void detectCollisions() override {
        const size_t numThreads = std::thread::hardware_concurrency();
        std::vector<std::future<CollisionResult>> futures;

        for (size_t i = 0; i < numThreads; ++i) {
            futures.emplace_back(std::async(std::launch::async, [this, i]() {
                return detectCollisionsSubset(i, numThreads);
            }));
        }

        // Collect and merge results
        std::vector<CollisionResult> results;
        for (auto& future : futures) {
            results.push_back(future.get());
        }

        return mergeCollisionResults(results);
    }
};
```

## Benchmarking and Profiling

### 1. Micro-benchmarking Critical Operations

```cpp
// Benchmark single operations
BENCHMARK(ForwardKinematicsBenchmark, SingleTransform) {
    auto skeleton = createTestSkeleton(100);
    skeleton->setRandomPositions();

    auto start = std::chrono::high_resolution_clock::now();
    skeleton->computeForwardKinematics();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    state.SetComplexityN(100);  // O(n) with n=100
    state.AddBytesProcessed(100 * sizeof(double) * 3);  // Position data
}
```

### 2. Memory Profiling

```cpp
// Track memory usage patterns
class MemoryProfiler {
private:
    std::unordered_map<std::string, size_t> allocations;

public:
    void trackAllocation(const std::string& type, size_t size) {
        allocations[type] += size;
    }

    void printReport() {
        for (const auto& [type, size] : allocations) {
            std::cout << type << ": " << size << " bytes\n";
        }
    }
};
```

## Common Optimization Pitfalls

### ❌ Premature Optimization

```cpp
// Wrong: Optimize without profiling
void computeSomething() {
    // Complex optimization based on assumption
    if (someCondition) {
        // Fast path based on guess
    } else {
        // Slow path
    }
}

// Correct: Profile first, then optimize
void computeSomething() {
    // Simple, correct implementation
    // Profile to find actual hotspots
    // Optimize based on measured data
}
```

### ❌ Breaking Algorithmic Complexity

```cpp
// Wrong: Change O(n) to O(n²) for "optimization"
for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {  // O(n²)!
        if (needInteraction(i, j)) {
            processInteraction(i, j);
        }
    }
}

// Correct: Maintain O(n) with smarter algorithm
for (size_t i = 0; i < n; ++i) {
    // Only check relevant interactions
    for (size_t j : getRelevantIndices(i)) {  // Still O(n)!
        processInteraction(i, j);
    }
}
```

## Performance Validation

### 1. Regression Testing

```cpp
// Ensure optimizations don't break correctness
TEST(OptimizationRegressionTest, ForwardKinematicsCorrectness) {
    auto skeleton = createTestSkeleton();

    // Test many random configurations
    for (int i = 0; i < 1000; ++i) {
        skeleton->setRandomPositions();

        auto original = computeReferenceForwardKinematics(skeleton);
        skeleton->computeForwardKinematics();
        auto optimized = skeleton->getPositions();

        EXPECT_NEAR((original - optimized).norm(), 0.0, 1e-12);
    }
}
```

### 2. Performance Baselines

```cpp
// Establish and track performance baselines
class PerformanceBaseline {
private:
    std::map<std::string, double> baselineTimes;

public:
    void updateBaseline(const std::string& operation, double time) {
        baselineTimes[operation] = time;
    }

    bool checkRegression(const std::string& operation, double currentTime) {
        auto baseline = baselineTimes[operation];
        return (currentTime / baseline) > 1.2;  // 20% regression threshold
    }
};
```

## Success Criteria

### Algorithm Performance

- [ ] Forward kinematics: < 1μs per DOF
- [ ] Constraint solving: < 10μs per constraint
- [ ] Collision detection: Linear scaling with scene complexity
- [ ] Memory usage: < 3x theoretical minimum

### System Performance

- [ ] Multi-threading: > 80% CPU utilization on 8+ cores
- [ ] SIMD utilization: Vectorized code paths active
- [ ] Cache efficiency: > 90% cache hit rate in hot loops
- [ ] No regressions: All optimizations maintain correctness

### Maintainability

- [ ] Code clarity: Optimized code remains readable
- [ ] Test coverage: Performance paths well tested
- [ ] Documentation: Optimization strategies documented
- [ ] Portability: Optimizations work across platforms

## Getting Help

- **Algorithm design**: Use Oracle agent for optimization strategies
- **Performance research**: Use Librarian agent for state-of-the-art techniques
- **Profiling analysis**: Use Explore agent to identify bottlenecks
- **Complex debugging**: Use multi-model orchestration for tough performance issues

## Related Documentation

- [Dynamics Module Guidelines](../../../dart/dynamics/AGENTS.md)
- [Collision Module Guidelines](../../../dart/collision/AGENTS.md)
- [DART Physics Skill](../dart-physics/SKILL.md)
- [Benchmarking Guide](../../../docs/onboarding/building.md#performance)

---

_Performance optimization requires measurement-driven development. Always profile before optimizing and validate correctness after each change._
