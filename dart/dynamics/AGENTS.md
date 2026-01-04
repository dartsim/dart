# Agent Guidelines for DART Dynamics Module

This module is the core of DART's physics simulation, implementing Featherstone's Articulated Body Algorithm for O(n) dynamics computation. Agents working here must understand complex kinematic chains, constraint solving, and performance-critical numerical algorithms.

## Read First

- **Base Guidelines**: Read `/AGENTS.md` for overall DART patterns
- **Featherstone Algorithm**: Core implementation in `Skeleton.cpp`
- **Performance Critical**: O(n) complexity must be maintained
- **Aspect System**: Uses `JointAspect`, `BodyNodeAspect` for extensibility
- **Testing**: Dynamics tests in `tests/unit/dynamics/` and `tests/integration/dynamics/`

## Module-Specific Commands

```bash
# Build dynamics with profiling
pixi run -e debug build --config DART_BUILD_PROFILE=ON

# Run dynamics-specific tests
pixi run test --filter dynamics
pixi run test --filter integration/dynamics

# Performance benchmarking (critical!)
pixi run benchmark --filter dynamics
pixi run benchmark --filter skeleton

# Debug memory issues
pixi run -e asan build --config DART_ENABLE_ASAN=ON

# Profile dynamics algorithms
pixi run benchmark --profile dynamics
```

## Critical Architecture Patterns

### 1. Featherstone Articulated Body Algorithm

The core algorithm is implemented in `Skeleton::computeDynamics()` - **do not modify without deep understanding**.

```cpp
// Core algorithm structure (simplified)
void Skeleton::computeDynamics(double timeStep) {
    computeForwardKinematics();           // O(n) forward pass
    computeVelocities();                   // O(n) velocity propagation
    computeArticulatedInertia();           // O(n) inertia computation
    computeArticulatedCoriolis();          // O(n) Coriolis forces
    computeExternalForces();               // O(n) external forces
    solveConstraints();                    // O(n) constraint resolution
    integrateVelocities();                  // O(n) time integration
}
```

### 2. Joint Types and DOF Management

```cpp
// Correct: Use factory patterns for joint creation
auto joint = Joint::create<RevoluteJoint>(properties);

// Wrong: Direct instantiation bypassing DOF management
auto joint = new RevoluteJoint(); // Avoid
```

### 3. Frame Hierarchy and Transformations

```cpp
// Correct: Use relative transforms efficiently
Eigen::Isometry3d worldTransform = frame->getWorldTransform();

// Wrong: Compute transforms manually
Eigen::Isometry3d worldTransform = accumulateTransforms(frame); // Inefficient
```

## Performance Requirements (CRITICAL)

### 1. O(n) Complexity Must Be Maintained

- **Forward kinematics**: Single pass from root to leaves
- **Dynamics computation**: No nested loops over joints
- **Jacobian computation**: Use analytical methods, not numerical
- **Constraint resolution**: Linear time with respect to DOFs

### 2. Memory Access Patterns

```cpp
// Correct: Cache-friendly access patterns
for (auto* bodyNode : mBodyNodes) {
    bodyNode->updateInertia();  // Sequential memory access
}

// Wrong: Random access patterns
for (size_t i = 0; i < n; ++i) {
    mBodyNodes[indices[i]]->updateInertia();  // Cache misses
}
```

### 3. Numerical Stability

- **Use appropriate precision**: `double` for dynamics, `float` for visualization
- **Avoid singular Jacobians**: Check for near-singular configurations
- **Stable integration**: Use semi-implicit Euler or Runge-Kutta

## When to Modify This Module

### Add New Joint Type

1. **Inherit from appropriate base**: `Joint`, `ZeroDofJoint`, etc.
2. **Implement required virtual methods**: `updateLocalTransform()`, `getJacobian()`
3. **Update DOF counting**: Ensure `mNumDofs` is correct
4. **Add factory support**: `Joint::create<NewJointType>()`
5. **Comprehensive testing**: Unit tests + integration tests
6. **Documentation**: Update joint type documentation

### Optimize Performance

1. **Profile first**: Use `pixi run benchmark --profile dynamics`
2. **Identify hotspots**: Focus on per-frame computations
3. **Maintain O(n)**: No algorithm that scales worse than linear
4. **Cache results**: Avoid recomputing unchanged values
5. **Vectorize when possible**: Use Eigen's SIMD optimizations

### Fix Dynamics Accuracy Issues

1. **Create minimal test case**: Reproduce with simplest skeleton
2. **Compare with reference**: Use analytical solutions for simple cases
3. **Check numerical precision**: Look for accumulated errors
4. **Verify constraint formulation**: Ensure forces are correctly computed
5. **Test edge cases**: Singular configurations, high velocities

## Common Pitfalls to Avoid

### ❌ Breaking O(n) Complexity

```cpp
// Wrong: Nested loops over joints (O(n²))
for (size_t i = 0; i < mJoints.size(); ++i) {
    for (size_t j = 0; j < mJoints.size(); ++j) {
        // Compute interaction between all joints
    }
}

// Correct: Single pass operations (O(n))
for (auto* joint : mJoints) {
    joint->updateContribution();  // Each joint processes its contribution
}
```

### ❌ Incorrect DOF Management

```cpp
// Wrong: Manual DOF counting
class MyJoint : public Joint {
public:
    size_t getNumDofs() const override { return 6; }  // Hardcoded
};

// Correct: Use DOF properties
class MyJoint : public Joint {
public:
    MyJoint() : Joint(Properties().addDof(1).addDof(1).addDof(1)) {}
    size_t getNumDofs() const override { return mProperties.getNumDofs(); }
};
```

### ❌ Frame Transform Errors

```cpp
// Wrong: Mixing coordinate frames incorrectly
Eigen::Vector3d localForce = worldForce;  // Frame mismatch

// Correct: Explicit coordinate transformations
Eigen::Vector3d localForce = frame->worldToLocal(worldForce);
```

## Testing Requirements

### Before Submitting Changes:

1. **Unit Tests**: `pixi run test --filter unit/dynamics`
2. **Integration Tests**: `pixi run test --filter integration/dynamics`
3. **Performance Tests**: `pixi run benchmark --filter dynamics`
4. **Memory Tests**: Check with AddressSanitizer for memory issues
5. **Accuracy Tests**: Compare with analytical solutions

### Critical Test Categories:

- **Forward kinematics**: Known pose configurations
- **Dynamics**: Compare with analytical solutions for simple mechanisms
- **Jacobian**: Verify analytical vs numerical Jacobians
- **Performance**: Ensure O(n) scaling with skeleton size
- **Numerical stability**: Large timesteps, stiff configurations

## Integration Points

### With Collision Module:

- **Contact forces**: `Contact` objects create constraints for dynamics
- **Collision filtering**: Avoid self-collision within kinematic chains
- **Body relationships**: `BodyNode` maps to `CollisionObject`

### With Constraint Module:

- **Joint limits**: Upper/lower bounds create inequality constraints
- **Contact constraints**: Friction and normal forces from collisions
- **Solver integration**: Feed constraint matrices to dynamics solver

### With GUI Module:

- **Real-time visualization**: Update skeleton poses each frame
- **Debug rendering**: Show forces, velocities, coordinate frames
- **Interactive manipulation**: Direct joint control in GUI

## Numerical Algorithms Used

### 1. Featherstone Algorithm Variants

- **Hybrid Dynamics**: Combines articulated body and constraint forces
- **Forward Dynamics**: Compute accelerations given forces
- **Inverse Dynamics**: Compute forces given desired accelerations

### 2. Jacobian Computation

- **Analytical Jacobians**: Closed-form for common joint types
- **Numerical Jacobians**: Finite differences for complex joints
- **Time-derivative of Jacobian**: For velocity-level IK

### 3. Integration Schemes

- **Semi-implicit Euler**: Stable for typical timesteps
- **Runge-Kutta 4**: Higher accuracy for sensitive systems
- **Variable timestep**: Adaptive integration for stability

## Getting Help

- **Architecture decisions**: Use Oracle agent for dynamics algorithm design
- **Performance optimization**: Use Librarian agent to research physics techniques
- **Numerical methods**: Use Explore agent to find existing implementations
- **Complex debugging**: Use multi-model orchestration (Oracle + Librarian + Explore)

## Files to Understand First

1. `Skeleton.hpp/.cpp` - Core dynamics implementation
2. `Joint.hpp/.cpp` - Joint base class and factory
3. `BodyNode.hpp/.cpp` - Rigid body dynamics
4. `DegreeOfFreedom.hpp/.cpp` - DOF management
5. `MetaSkeleton.hpp/.cpp` - Skeleton management utilities

---

_Remember: DART dynamics is performance-critical and numerically sensitive. Always benchmark optimizations and verify accuracy with analytical solutions._
