# Phase 5: Physics Integration Design

## Overview

This document outlines the design for implementing physics simulation in the `dart::simulation::experimental` module. The goal is to implement `World::step()` with full forward kinematics, forward dynamics, collision detection, and constraint solving.

## Current State

### What Exists

1. **Frame System** (`frame/`)
   - `Frame::getTransform()` computes world transforms with lazy evaluation
   - `FrameCache` stores cached transforms with dirty flags
   - Parent-child relationships via `FrameState::parentFrame`
   - Transform computation: `worldTransform = parent.getTransform() * getLocalTransform()`

2. **Joint Component** (`comps/joint.hpp`)
   - All 8 joint types defined (Fixed, Revolute, Prismatic, Screw, Universal, Ball, Planar, Free)
   - State storage: position, velocity, acceleration, torque
   - Limits: position, velocity, effort
   - Parent/child link references

3. **Link Component** (`comps/link.hpp`)
   - Parent joint reference
   - MultiBody reference

4. **Dynamics Components** (`comps/dynamics.hpp`)
   - Transform (position + orientation)
   - Velocity (linear + angular)
   - MassProperties (mass + inertia)
   - Force (force + torque accumulators)

### What's Missing

1. **Joint Transform Computation**: No function to compute transform from joint type + position
2. **Link Transform Computation**: Links don't compute transforms from joint state
3. **Forward Kinematics System**: No batch transform update
4. **Forward Dynamics**: No ABA/RNEA implementation
5. **Collision Integration**: No connection to `dart/collision/`
6. **Constraint Solver Integration**: No connection to `dart/constraint/`
7. **Time Stepping**: No `World::step()` implementation

---

## Phase 5.1: Forward Kinematics

### Design

The forward kinematics system computes world transforms for all Links based on joint positions.

#### Joint Transform Functions

Create `dart/simulation/experimental/kinematics/joint_transform.hpp`:

```cpp
namespace dart::simulation::experimental::kinematics {

/// Compute the relative transform for a joint given its type, axis, and position
Eigen::Isometry3d computeJointTransform(
    comps::JointType type,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& axis2,
    double pitch,
    const Eigen::VectorXd& position);

/// Specializations for each joint type
Eigen::Isometry3d computeRevoluteTransform(
    const Eigen::Vector3d& axis, double angle);

Eigen::Isometry3d computePrismaticTransform(
    const Eigen::Vector3d& axis, double displacement);

Eigen::Isometry3d computeScrewTransform(
    const Eigen::Vector3d& axis, double pitch, double angle);

// ... etc for all joint types

} // namespace
```

#### Link Transform Computation

Modify `Link::getLocalTransform()` to compute from joint state:

```cpp
const Eigen::Isometry3d& Link::getLocalTransform() const {
    auto* joint = getParentJoint();
    if (!joint) {
        // Root link: identity or base offset
        return m_baseOffset;
    }

    // Compute joint transform from current position
    return kinematics::computeJointTransform(
        joint->getType(),
        joint->getAxis(),
        joint->getAxis2(),
        joint->getPitch(),
        joint->getPosition()
    );
}
```

#### Batch Update System

For simulation efficiency, provide a batch update:

```cpp
void World::updateKinematics() {
    // Already exists but needs enhancement

    // 1. Mark all link caches dirty
    // 2. Optionally: pre-compute all transforms in topological order
    //    (more cache-friendly than lazy evaluation during traversal)
}
```

### Implementation Plan

1. Create `dart/simulation/experimental/kinematics/` directory
2. Implement `joint_transform.hpp/cpp` with all joint type transforms
3. Modify `Link::getLocalTransform()` to use joint transforms
4. Add unit tests for each joint type transform
5. Add integration test: set joint positions, verify link transforms

### Test Cases

```cpp
TEST(Kinematics, RevoluteJointTransform) {
    // Rotate 90° around Z axis
    auto T = computeRevoluteTransform({0,0,1}, M_PI/2);
    EXPECT_TRUE(T.translation().isApprox(Vector3d::Zero()));
    EXPECT_TRUE(T.linear().isApprox(AngleAxisd(M_PI/2, Vector3d::UnitZ()).matrix()));
}

TEST(Kinematics, TwoLinkArm) {
    World world;
    auto robot = world.addMultiBody("robot");
    auto base = robot.addLink("base");
    auto link1 = robot.addLink("link1", {.parentLink=base, .jointType=Revolute, .axis={0,0,1}});

    // Set joint to 90°
    link1.getParentJoint()->setPosition({M_PI/2});

    // Verify world transform
    auto T = link1.getTransform();
    // ... verify rotation matches expectation
}
```

---

## Phase 5.2: Forward Dynamics (ABA)

### Design

Implement the Articulated Body Algorithm (ABA) for computing joint accelerations from applied forces/torques.

The classic DART uses the Featherstone algorithm:

```
Forward pass: Compute velocities and bias forces
Backward pass: Compute articulated body inertias
Forward pass: Compute accelerations
```

#### Components Needed

1. **ArticulatedBodyInertia** - 6x6 spatial inertia for ABA
2. **BiasForce** - Velocity-dependent forces (Coriolis, centrifugal)
3. **SpatialVelocity** - Twist (angular, linear)
4. **SpatialAcceleration** - Spatial acceleration

#### ECS Approach

Rather than storing intermediate ABA quantities in the ECS (wasteful memory), use a **system** that computes on-demand:

```cpp
class ForwardDynamicsSystem {
public:
    /// Compute joint accelerations for a MultiBody
    void compute(World& world, MultiBody& mb);

private:
    // Temporary storage (reused across calls)
    std::vector<Eigen::Matrix6d> articulatedInertias;
    std::vector<Eigen::Vector6d> biasForces;
    std::vector<Eigen::Vector6d> spatialAccelerations;
};
```

### Implementation Plan

1. Create `dart/simulation/experimental/dynamics/` directory
2. Implement spatial math utilities (SE3, spatial inertia)
3. Implement ABA algorithm following Featherstone
4. Add `World::computeForwardDynamics()` method
5. Add unit tests against known analytical solutions

---

## Phase 5.3: Collision Detection Integration

### Design

Integrate with existing `dart::collision::CollisionDetector`:

```cpp
class World {
public:
    void setCollisionDetector(collision::CollisionDetectorPtr detector);
    CollisionResult detectCollisions();
};
```

### Challenges

1. **Shape attachment**: Experimental Links need to reference collision shapes
2. **Transform sync**: Keep collision object transforms in sync with Link transforms
3. **Broad-phase**: Use existing BVH/spatial hash from collision module

### Implementation Plan

1. Add `ShapeNode` handle class to experimental API
2. Add `Link::addShapeNode()` method
3. Implement transform sync in `updateKinematics()`
4. Add collision detection tests

---

## Phase 5.4: Constraint Solver Integration

### Design

Integrate with existing `dart::constraint::ConstraintSolver`:

```cpp
class World {
public:
    void setConstraintSolver(constraint::ConstraintSolverPtr solver);
    void solveConstraints(double dt);
};
```

### Constraint Types

1. **Contact constraints** (from collision detection)
2. **Joint limit constraints** (position/velocity limits)
3. **User constraints** (ball-and-socket, fixed, etc.)

---

## Phase 5.5: World::step()

### Design

The main simulation loop:

```cpp
void World::step(double dt) {
    // 1. Update kinematics (FK)
    updateKinematics();

    // 2. Apply external forces (gravity, user forces)
    applyGravity();

    // 3. Compute forward dynamics (ABA)
    computeForwardDynamics();

    // 4. Detect collisions
    auto collisions = detectCollisions();

    // 5. Solve constraints (contacts, limits)
    solveConstraints(collisions, dt);

    // 6. Integrate state (semi-implicit Euler)
    integrateState(dt);

    // 7. Clear force accumulators
    clearForces();
}
```

### Configurable Components

```cpp
struct StepOptions {
    double timeStep = 0.001;
    Eigen::Vector3d gravity = {0, 0, -9.81};
    IntegratorType integrator = IntegratorType::SemiImplicitEuler;
    int maxConstraintIterations = 10;
};
```

---

## File Structure

```
dart/simulation/experimental/
├── kinematics/
│   ├── joint_transform.hpp
│   ├── joint_transform.cpp
│   └── forward_kinematics.hpp
├── dynamics/
│   ├── spatial_math.hpp
│   ├── articulated_body.hpp
│   ├── articulated_body.cpp
│   ├── forward_dynamics.hpp
│   └── forward_dynamics.cpp
├── collision/
│   └── collision_adapter.hpp
├── constraint/
│   └── constraint_adapter.hpp
└── simulation/
    ├── step.hpp
    └── step.cpp
```

---

## Testing Strategy

1. **Unit tests**: Each joint transform type
2. **Integration tests**: Multi-link chains, compare with analytical solutions
3. **Regression tests**: Compare with classic API (same inputs → same outputs)
4. **Performance tests**: Benchmark against classic API

---

## Timeline

| Task                       | Estimated Duration |
| -------------------------- | ------------------ |
| 5.1 Forward Kinematics     | 2 weeks            |
| 5.2 Forward Dynamics       | 3 weeks            |
| 5.3 Collision Integration  | 1 week             |
| 5.4 Constraint Integration | 1 week             |
| 5.5 World::step()          | 1 week             |
| Testing & Validation       | 2 weeks            |
| **Total**                  | **10 weeks**       |

---

## Next Steps

1. [ ] Review this design with team
2. [ ] Start Phase 5.1: Forward Kinematics
3. [ ] Create kinematics/ directory structure
4. [ ] Implement joint transform functions
