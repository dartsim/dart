---
description: Gazebo physics integration and compatibility specialist
mcp:
  gazebo-worlds:
    command: gz
    args: ["world", "--verbose"]
    description: "Gazebo world simulation"
  sdformat-validator:
    command: sdformat
    args: ["--check", "--print"]
    description: "SDF format validation"
---

# Gazebo Integration Skill

## Purpose

Specialized skill for DART and Gazebo physics integration, focusing on compatibility validation, physics breaking changes, and maintaining consistency between the two simulation ecosystems.

## When to Use This Skill

### Physics Compatibility Testing

- Validate DART physics changes don't break Gazebo integration
- Test contact and collision behavior consistency
- Verify constraint solver compatibility
- Check joint limit enforcement

### Model Integration

- Convert URDF models for Gazebo-specific features
- Validate SDF models work with both DART and Gazebo
- Debug physics discrepancies between engines
- Optimize models for both simulation environments

### Continuous Integration

- Run automated `test-gz` workflows
- Validate physics breaking changes automatically
- Test across multiple Gazebo versions
- Ensure simulation consistency

## Key Tools and Commands

```bash
# Gazebo integration testing
pixi run -e gazebo test-gz              # Full integration test suite
pixi run -e gazebo test-gz --world my_world.sdf  # Specific world
pixi run -e gazebo test-gz --physics-only         # Physics tests only

# Model validation and conversion
sdformat --check model.urdf                  # URDF validation
sdformat --print model.sdf                    # Processed SDF output
gz model --model-file model.urdf              # Visual model check

# Physics debugging
gz --verbose --physics-engine dart physics.dae     # Verbose physics logging
gz gui --physics-engine dart world.sdf            # Interactive debugging
gz server --physics-engine dart -s 0.01         # Headless simulation

# Performance comparison
pixi run benchmark --filter gazebo            # DART vs Gazebo perf
pixi run -e gazebo test-gz --benchmark    # Gazebo benchmarks
```

## DART-Gazebo Interface

### 1. Physics Engine Bridge

DART provides physics engine implementation for Gazebo through:

- **GazeboPhysicsPlugin**: DART physics in Gazebo ecosystem
- **Compatibility layer**: Maps Gazebo API calls to DART implementations
- **SDF/URDF handling**: Shared model loading between systems

### 2. Data Structure Mapping

| Gazebo Concept | DART Equivalent | Notes                     |
| -------------- | --------------- | ------------------------- |
| Model          | Skeleton        | Robot representation      |
| Link           | BodyNode        | Rigid body                |
| Joint          | Joint           | Kinematic constraints     |
| Collision      | CollisionObject | Collision geometry        |
| World          | World           | Complete simulation scene |

## Physics Integration Patterns

### 1. Joint Type Compatibility

```xml
<!-- URDF compatible with both DART and Gazebo -->
<joint name="arm_joint" type="revolute">
  <parent link="base"/>
  <child link="arm"/>
  <axis xyz="0 0 1"/>

  <!-- DART-specific limits -->
  <dynamics damping="0.1"/>

  <!-- Gazebo-specific physics -->
  <gazebo reference="arm_joint">
    <physics>
      <ode>
        <implicit_spring_damper>true</implicit_spring_damper>
        <limit>
          <cfm>0.0</cfm>
          <erp>0.9</erp>
        </limit>
      </ode>
    </physics>
  </gazebo>
</joint>
```

### 2. Contact and Collision Consistency

```xml
<!-- Ensure collision parameters work in both engines -->
<gazebo reference="base_link">
  <collision name="base_collision">
    <geometry>
      <box size="1 1 1"/>
    </geometry>

    <!-- DART collision properties -->
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>  <!-- Works in both engines -->
          <mu2>0.6</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

### 3. Material Property Mapping

```xml
<!-- Visual material compatibility -->
<material name="blue_material">
  <!-- Standard URDF visual properties -->
  <color rgba="0 0 1 1"/>

  <!-- Gazebo-specific rendering -->
  <gazebo reference="blue_material">
    <script>
      <uri>file://materials/scripts/blue</uri>
    </script>
  </gazebo>
</material>
```

## Testing and Validation

### 1. Physics Breaking Changes Test

```bash
# Automated test workflow
pixi run -e gazebo test-gz --physics-breaking

# Test categories included:
- Forward dynamics consistency
- Contact force computation
- Constraint satisfaction
- Energy conservation
- Stability over long simulations
```

### 2. Model Compatibility Validation

```cpp
// Test model loads in both systems
class ModelCompatibilityTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Load model in DART
        dartWorld = dart::io::readWorld("test_robot.urdf");
        ASSERT_NE(dartWorld, nullptr);

        // Load model in Gazebo
        gazeboWorld = loadGazeboWorld("test_robot.urdf");
        ASSERT_NE(gazeboWorld, nullptr);
    }

    void comparePhysicsProperties() {
        // Compare joint limits, masses, inertia
        compareJointProperties();
        compareInertiaProperties();
        compareCollisionProperties();
    }
};
```

### 3. Performance Comparison Tests

```cpp
// Benchmark DART vs Gazebo physics
BENCHMARK(PhysicsComparison, ForwardDynamics) {
    auto dartSkeleton = createBenchmarkSkeleton();
    auto gazeboWorld = createGazeboEquivalent();

    // Benchmark DART
    auto dartTime = benchmarkDARTForwardDynamics(dartSkeleton);

    // Benchmark Gazebo with DART physics
    auto gazeboTime = benchmarkGazeboDynamics(gazeboWorld);

    // Results should be comparable (within 10%)
    EXPECT_LT(std::abs(dartTime - gazeboTime),
              std::max(dartTime, gazeboTime) * 0.1);
}
```

## Common Integration Issues

### ❌ Physics Parameter Mismatches

```xml
<!-- Wrong: Incompatible physics parameters -->
<gazebo reference="joint">
  <physics>
    <ode>
      <max_step_size>0.001</max_step_size>  <!-- Too small for DART -->
      <cfm>0.0</cfm>                    <!-- No constraint force mixing -->
    </ode>
  </physics>
</gazebo>

<!-- Correct: Compatible parameters -->
<gazebo reference="joint">
  <physics>
    <ode>
      <max_step_size>0.01</max_step_size>   <!-- DART-friendly timestep -->
      <cfm>0.00001</cfm>                   <!-- Small but non-zero -->
      <erp>0.2</erp>                        <!-- Reasonable error reduction -->
    </ode>
  </physics>
</gazebo>
```

### ❌ Joint Limit Inconsistencies

```xml
<!-- Wrong: Different limits in different places -->
<joint limit="0 1.57" type="revolute"/>  <!-- URDF limits -->
<gazebo reference="joint">
  <physics>
    <ode>
      <limit><cfm>0.0</cfm><erp>0.9</erp></limit>  <!-- No limits! -->
    </ode>
  </physics>
</gazebo>

<!-- Correct: Consistent limits -->
<joint limit="0 1.57" type="revolute"/>  <!-- URDF limits -->
<gazebo reference="joint">
  <physics>
    <ode>
      <limit>
        <lower>0</lower>
        <upper>1.57</upper>
        <cfm>0.0</cfm>
        <erp>0.9</erp>
      </limit>
    </ode>
  </physics>
</gazebo>
```

## Validation Workflows

### 1. Continuous Integration Testing

```yaml
# GitHub Actions workflow for Gazebo integration
name: DART-Gazebo Integration Tests
on: [push, pull_request]

jobs:
  gazebo-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2

      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y gazebo11 libgazebo11-dev

      - name: Build DART
        run: pixi run build --config DART_BUILD_GAZEBO=ON

      - name: Run integration tests
        run: pixi run -e gazebo test-gz

      - name: Upload results
        uses: actions/upload-artifact@v2
        with:
          name: gazebo-test-results
          path: test-results/
```

### 2. Local Development Testing

```bash
# Quick development cycle
# 1. Make changes to DART physics
# 2. Build with Gazebo support
pixi run build --config DART_BUILD_GAZEBO=ON

# 3. Run integration tests
pixi run -e gazebo test-gz

# 4. If tests fail, debug
gazebo --verbose --physics-engine dart test_world.sdf
```

## Performance Monitoring

### 1. Physics Benchmark Comparison

```bash
# Run comparative benchmarks
pixi run benchmark --filter physics-comparison

# Output format:
# Test, DART_Time(ms), Gazebo_Time(ms), Diff(%), Status
ForwardDynamics, 45.2, 47.1, 4.2, PASS
ConstraintSolve, 12.8, 13.1, 2.3, PASS
CollisionDetect, 8.4, 8.2, -2.4, PASS
```

### 2. Memory Usage Tracking

```cpp
// Monitor memory in long simulations
class MemoryMonitor {
private:
    size_t baselineMemory;
    size_t peakMemory;

public:
    void startMonitoring() {
        baselineMemory = getCurrentMemoryUsage();
        peakMemory = baselineMemory;
    }

    void checkMemory() {
        size_t current = getCurrentMemoryUsage();
        peakMemory = std::max(peakMemory, current);

        if (current > baselineMemory * 1.5) {  // 50% increase threshold
            std::cerr << "Memory usage spike: " << current << " bytes\n";
        }
    }
};
```

## Success Criteria

### Integration Compatibility

- [ ] All DART physics features work in Gazebo
- [ ] No physics breaking changes in CI/CD
- [ ] Performance within 10% of baseline
- [ ] Consistent behavior across versions

### Model Compatibility

- [ ] URDF models load identically in both systems
- [ ] SDF models work across both ecosystems
- [ ] Material properties preserved correctly
- [ ] Joint limits enforced consistently

### Validation and Testing

- [ ] Automated `test-gz` passes consistently
- [ ] Physics breaking changes detected automatically
- [ ] Performance regressions caught in CI
- [ ] Clear debugging workflows for failures

## Getting Help

- **Physics engine design**: Use Oracle agent for DART-Gazebo integration patterns
- **Gazebo ecosystem**: Use Librarian agent for Gazebo-specific features
- **Physics debugging**: Use Explore agent to find existing integration issues
- **Performance analysis**: Use multi-model orchestration for complex debugging

## Files to Understand First

1. `gazebo/` - Gazebo physics plugin implementation
2. `tests/integration/gazebo/` - Integration test suite
3. `.github/workflows/ci_gz_physics.yml` - CI/CD configuration
4. `examples/` - Models demonstrating Gazebo integration

---

_Gazebo integration ensures DART physics work correctly in the broader robotics simulation ecosystem. Maintain compatibility and consistent behavior._
