# Agent Guidelines for DART Testing Module

This module contains comprehensive test suites covering unit tests, integration tests, benchmarks, and CI/CD workflows. Agents working here must understand DART's testing philosophy, performance requirements, and continuous integration patterns.

## Read First

- **Base Guidelines**: Read `/AGENTS.md` for overall DART patterns
- **Test Organization**: Unit tests for components, integration for workflows
- **Performance**: Benchmarks must maintain O(n) characteristics
- **CI/CD**: Tests run on Ubuntu, macOS, Windows with Gazebo integration

## Module-Specific Commands

```bash
# Run different test categories
pixi run test                           # All tests
pixi run test --filter unit              # Unit tests only
pixi run test --filter integration         # Integration tests only
pixi run test --filter python            # Python binding tests
pixi run test --filter collision         # Collision module tests
pixi run test --filter dynamics          # Dynamics module tests
pixi run test --filter io               # IO module tests

# Performance benchmarking
pixi run benchmark                      # All benchmarks
pixi run benchmark --filter dynamics       # Dynamics benchmarks
pixi run benchmark --filter collision      # Collision benchmarks
pixi run benchmark --profile dynamics     # With profiling

# Specialized test workflows
pixi run -e gazebo test-gz            # Gazebo integration tests
pixi run -e asan test                 # With AddressSanitizer
pixi run -e debug test                 # Debug builds with tests

# Coverage reporting
pixi run test --coverage
pixi run coverage --report
```

## Test Architecture

### 1. Test Hierarchy

```
tests/
├── unit/                    # Isolated component tests
│   ├── common/             # Core utilities (Aspect, math, etc.)
│   ├── collision/          # Collision detection tests
│   ├── dynamics/           # Physics engine tests
│   ├── io/               # File loading tests
│   └── simulation/        # Simulation logic tests
├── integration/            # End-to-end workflow tests
│   ├── physics/           # Full simulation scenarios
│   ├── python/           # Python binding tests
│   ├── io/               # File format compatibility
│   └── gui/              # Visual/interactive tests
├── benchmark/             # Performance regression tests
│   ├── dynamics/          # Algorithm performance
│   ├── collision/         # Collision performance
│   └── io/               # Loading performance
└── regression/            # Bug regression tests
```

### 2. Test Naming Conventions

```cpp
// Unit test naming: [ModuleName]Test[FeatureName]
TEST(CollisionTest, BroadPhaseFiltering) {
    // Test collision broad-phase filtering
}

// Integration test naming: [Scenario]IntegrationTest
TEST(BipedWalkingIntegrationTest, BasicGait) {
    // Test complete biped walking scenario
}

// Benchmark naming: [ModuleName]Benchmark[OperationName]
BENCHMARK(DynamicsBenchmark, ForwardKinematics) {
    // Benchmark forward kinematics performance
}
```

## Critical Testing Patterns

### 1. Fixtures and Setup

```cpp
// Correct: Use test fixtures for common setup
class SkeletonTest : public ::testing::Test {
protected:
    void SetUp() override {
        skeleton = Skeleton::create();
        // Add standard test configuration
    }

    SkeletonPtr skeleton;
};

TEST_F(SkeletonTest, JointLimitEnforcement) {
    // Test with skeleton fixture already set up
}
```

### 2. Performance Testing

```cpp
// Correct: Benchmark with realistic complexity
BENCHMARK(DynamicsBenchmark, LargeSkeletonForwardDynamics) {
    auto skeleton = createLargeSkeleton(1000);  // Realistic size
    skeleton->setRandomPositions();

    auto start = std::chrono::high_resolution_clock::now();
    skeleton->computeForwardDynamics();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    ASSERT_LT(duration.count(), 1000);  // Should complete in < 1ms
}
```

### 3. Integration Test Patterns

```cpp
// Correct: Full workflow testing
TEST(WorldSimulationIntegrationTest, CompleteSimulationCycle) {
    // Load a complex robot
    auto world = dart::io::readWorld("atlas_puppet/robot.urdf");
    ASSERT_NE(world, nullptr);

    // Run simulation for multiple timesteps
    for (int i = 0; i < 1000; ++i) {
        world->step();
        ASSERT_NO_FATAL_FAILURE(world->validateState());
    }

    // Verify expected behavior
    EXPECT_GT(world->getTime(), 0.0);
    EXPECT_FALSE(world->hasErrors());
}
```

## When to Modify This Module

### Add New Unit Tests

1. **Create test file** in appropriate `tests/unit/` subdirectory
2. **Follow naming convention**: `[Module]Test[Feature]`
3. **Use test fixtures** for common setup
4. **Cover edge cases**: Boundary conditions, error cases
5. **Add to CMake**: Update `tests/CMakeLists.txt`

### Add Integration Tests

1. **Identify user workflow**: Complete end-to-end scenario
2. **Use real files**: Test with actual URDF/SDF files
3. **Multiple steps**: Verify complete workflow
4. **Performance validation**: Ensure acceptable runtime
5. **Cross-platform**: Test on different platforms

### Add Benchmarks

1. **Focus on bottlenecks**: Critical performance paths
2. **Use realistic data**: Large skeletons, complex scenes
3. **Establish baseline**: Current performance levels
4. **Regression detection**: Alert on performance degradation
5. **Statistical validity**: Multiple runs, median values

## Test Data Management

### 1. Test Files Organization

```
tests/
├── data/                   # Test input files
│   ├── urdf/             # URDF test files
│   ├── sdf/              # SDF test files
│   ├── mjcf/             # MJCF test files
│   └── meshes/           # Test geometry files
├── expected/              # Expected results
│   ├── outputs/           # Expected simulation outputs
│   └── errors/           # Expected error cases
└── generated/             # Generated during tests (gitignored)
```

### 2. Test Data Best Practices

- **Small files**: Fast loading for unit tests
- **Realistic files**: Complex scenarios for integration tests
- **Known results**: Analytical solutions when possible
- **Error cases**: Malformed files, edge cases

## Performance Requirements

### 1. Test Execution Time

- **Unit tests**: Should complete in < 5 minutes total
- **Integration tests**: Should complete in < 30 minutes total
- **Benchmarks**: Each benchmark < 1 minute, all < 15 minutes
- **CI/CD pipeline**: Full test suite < 2 hours

### 2. Memory Usage

- **Unit tests**: Minimal memory footprint
- **Integration tests**: Reasonable for scenario complexity
- **Benchmarks**: Track memory usage alongside performance
- **No leaks**: AddressSanitizer should pass

### 3. Performance Regression Detection

```cpp
// Correct: Performance baseline testing
BENCHMARK(DynamicsBenchmark, ForwardKinematicsPerformance) {
    static const double BASELINE_MICROSECONDS = 100.0;
    static const double REGRESSION_THRESHOLD = 1.2; // 20% slowdown

    auto duration = measureForwardKinematics(skeleton);
    EXPECT_LT(duration, BASELINE_MICROSECONDS * REGRESSION_THRESHOLD);
}
```

## Common Pitfalls to Avoid

### ❌ Brittle Tests

```cpp
// Wrong: Hardcoded values that may change
TEST(SkeletonTest, DefaultPosition) {
    EXPECT_EQ(skeleton->getPosition(0), 0.0);  // Brittle
}

// Correct: Test behavior, not exact values
TEST(SkeletonTest, ZeroPositionOnInitialization) {
    auto positions = skeleton->getPositions();
    EXPECT_TRUE(positions.isZero(1e-15));  // Numerical tolerance
}
```

### ❌ Slow Tests

```cpp
// Wrong: Large computation in unit test
TEST(SkeletonTest, LargeScaleDynamics) {
    auto skeleton = createSkeletonWith10000Joints();  // Too slow for unit test
    // ... long computation
}

// Correct: Move to benchmark or integration test
BENCHMARK(DynamicsBenchmark, LargeScaleDynamics) {
    auto skeleton = createSkeletonWith10000Joints();  // Appropriate for benchmark
    // ... measure performance
}
```

### ❌ Platform-Specific Assumptions

```cpp
// Wrong: Platform-specific behavior
TEST(FileIOTest, FilePathHandling) {
    std::string path = "/tmp/test.urdf";  // Unix only
    // ...
}

// Correct: Cross-platform paths
TEST(FileIOTest, FilePathHandling) {
    std::string path = (fs::temp_directory_path() / "test.urdf").string();
    // ...
}
```

## CI/CD Integration

### 1. GitHub Actions Workflows

- **Continuous testing**: Ubuntu, macOS, Windows
- **Gazebo integration**: Physics compatibility validation
- **Performance regression**: Automated benchmark comparisons
- **Code coverage**: Track test coverage over time

### 2. Test Matrix Configuration

```yaml
# Example test matrix
strategy:
  matrix:
    os: [ubuntu-latest, macos-latest, windows-latest]
    build_type: [Release, Debug]
    compiler: [gcc-13, clang-15, msvc-19]
    include:
      # Special configuration for Gazebo tests
      - os: ubuntu-latest
        build_type: Release
        gazebo: true
```

## Quality Assurance

### 1. Test Coverage Requirements

- **Unit tests**: > 80% line coverage for core modules
- **Integration tests**: Cover all major user workflows
- **Python bindings**: 100% API coverage
- **Error paths**: Test all error handling code

### 2. Test Reliability

- **Flaky test detection**: Automatic identification and reporting
- **Test isolation**: No dependencies between tests
- **Deterministic results**: Same results across runs
- **Resource cleanup**: Proper teardown after each test

## Getting Help

- **Test design**: Use Oracle agent for complex test scenarios
- **Performance testing**: Use Librarian agent for benchmarking techniques
- **CI/CD issues**: Use Explore agent to find workflow patterns
- **Flaky test debugging**: Use multi-model orchestration

## Files to Understand First

1. `tests/CMakeLists.txt` - Test build configuration
2. `tests/unit/` - Component-level test implementations
3. `tests/integration/` - End-to-end test scenarios
4. `tests/benchmark/` - Performance regression tests
5. `.github/workflows/` - CI/CD configuration

---

_Remember: Tests are DART's quality foundation. Maintain fast, reliable, comprehensive test coverage._
