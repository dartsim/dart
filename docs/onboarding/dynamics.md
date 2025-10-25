# DART Dynamics Module Exploration

## Overview
This document provides an exploration of the core dynamics classes in DART (Dynamic Animation and Robotics Toolkit), located in the `dart/dynamics` directory.

## Core Dynamics Classes

### 1. Skeleton (`Skeleton.hpp`)
**File Path:** `dart/dynamics/Skeleton.hpp`

**Purpose:** The Skeleton class represents a complete articulated rigid body system. It is the top-level container that manages the entire kinematic tree.

**Key Features:**
- **Hierarchical Structure:** Contains multiple trees of interconnected BodyNodes and Joints
- **State Management:** Manages positions, velocities, accelerations, forces, and commands for all degrees of freedom
- **Configuration:** Provides `Configuration` struct for joint configurations (positions, velocities, accelerations, forces, commands)
- **Properties:**
  - Name management
  - Self-collision checking (with adjacent body checking option)
  - Mobile/immobile status (for forward dynamics)
  - Time step and gravity settings

**Key Methods:**
- **Structure:** `createJointAndBodyNodePair()` - Creates Joint and BodyNode pairs
- **Kinematics:** `computeForwardKinematics()` - Computes transforms, velocities, and accelerations
- **Dynamics:**
  - `computeForwardDynamics()` - Solves equations of motion forward in time
  - `computeInverseDynamics()` - Computes required forces/torques for desired motion
- **Equations of Motion:**
  - `getMassMatrix()`, `getInvMassMatrix()` - Mass matrix operations
  - `getCoriolisForces()`, `getGravityForces()` - Force vector computations
  - `getExternalForces()`, `getConstraintForces()` - External and constraint forces
- **Jacobians:** Various Jacobian computation methods for kinematics analysis
- **Center of Mass:** `getCOM()`, `getCOMLinearVelocity()`, `getCOMJacobian()` - COM-related functions
- **Integration:** `integratePositions()`, `integrateVelocities()` - Euler integration
- **Impulse-based Dynamics:** Support for constraint impulses and impulse-based forward dynamics

**Internal Structure:**
- Manages multiple trees (allows for disconnected kinematic structures)
- Uses `DataCache` for computational efficiency
- Implements dirty flags for lazy evaluation
- Support for IK (Inverse Kinematics) module

---

### 2. BodyNode (`BodyNode.hpp`)
**File Path:** `dart/dynamics/BodyNode.hpp`

**Purpose:** Represents a single rigid body (link) in the articulated system. BodyNodes are hierarchically connected through Joints.

**Key Features:**
- **Physical Properties:**
  - Mass
  - Moment of inertia (spatial inertia)
  - Center of mass (local COM)
  - Friction and restitution coefficients (deprecated, now per-ShapeNode)

- **Kinematic Properties:**
  - Transform relative to parent
  - Spatial velocity and acceleration
  - Partial acceleration
  - Body Jacobians (body frame and world frame)

- **Dynamic Properties:**
  - External forces and torques
  - Body forces (transmitted from parent)
  - Articulated body inertia
  - Bias forces and impulses

- **Structural Properties:**
  - Parent and child BodyNodes
  - Parent Joint
  - Index within Skeleton and tree
  - Gravity mode flag
  - Collidable flag

**Key Methods:**
- **Property Management:**
  - `setMass()`, `getMass()`
  - `setMomentOfInertia()`, `getMomentOfInertia()`
  - `setLocalCOM()`, `getLocalCOM()`
  - `setInertia()`, `getInertia()`

- **Kinematics:**
  - `getRelativeTransform()` - Transform relative to parent
  - `getJacobian()`, `getWorldJacobian()` - Various Jacobian forms
  - `getCOM()`, `getCOMLinearVelocity()` - Center of mass kinematics

- **Dynamics:**
  - `addExtForce()`, `addExtTorque()` - Apply external forces
  - `getBodyForce()` - Get transmitted body force
  - `computeKineticEnergy()`, `computePotentialEnergy()` - Energy calculations

- **Structure Manipulation:**
  - `moveTo()`, `split()`, `copyTo()`, `copyAs()` - Tree restructuring operations
  - `createChildJointAndBodyNodePair()` - Create child nodes
  - `remove()` - Remove subtree

- **ShapeNode Management:**
  - `createShapeNode()` - Attach collision/visual shapes
  - `getShapeNodesWith<Aspect>()` - Query shapes with specific aspects
  - `eachShapeNodeWith<Aspect>()` - Iterate over shapes

- **Specialized Nodes:**
  - EndEffector support
  - Marker support

**Internal Dynamics Routines:**
- `updateTransform()`, `updateVelocity()` - Kinematic updates
- `updateArtInertia()` - Articulated body inertia computation
- `updateBiasForce()`, `updateBiasImpulse()` - Bias force updates
- `updateAccelerationFD()`, `updateAccelerationID()` - Forward/Inverse dynamics
- `updateMassMatrix()`, `aggregateMassMatrix()` - Mass matrix assembly

---

### 3. Joint (`Joint.hpp`)
**File Path:** `dart/dynamics/Joint.hpp`

**Purpose:** Abstract base class representing a kinematic constraint between two BodyNodes. Defines the degrees of freedom and their constraints.

**Key Features:**
- **Actuator Types:**
  - `FORCE` - Force-controlled (dynamic)
  - `PASSIVE` - No actuation (dynamic)
  - `SERVO` - Servo-controlled
  - `MIMIC` - Mimics another joint
  - `ACCELERATION` - Acceleration-controlled (kinematic)
  - `VELOCITY` - Velocity-controlled (kinematic)
  - `LOCKED` - Fixed (kinematic)

- **Degree of Freedom Management:**
  - Multiple DOFs per joint
  - Position, velocity, acceleration, force for each DOF
  - Position/velocity/acceleration/force limits
  - Initial values for reset

- **Transformations:**
  - Transform from parent BodyNode to Joint
  - Transform from child BodyNode to Joint
  - Relative transform between parent and child

**Key Methods (All Pure Virtual):**
- **DOF Access:**
  - `getNumDofs()` - Number of degrees of freedom
  - `getDof(index)` - Access specific DOF
  - `setDofName()`, `getDofName()` - DOF naming

- **Position Control:**
  - `setPosition()`, `getPosition()`
  - `setPositionLowerLimit()`, `setPositionUpperLimit()`
  - `isCyclic()` - Check if DOF wraps around
  - `resetPosition()`, `setInitialPosition()`

- **Velocity Control:**
  - `setVelocity()`, `getVelocity()`
  - `setVelocityLowerLimit()`, `setVelocityUpperLimit()`
  - `resetVelocity()`, `setInitialVelocity()`

- **Acceleration Control:**
  - `setAcceleration()`, `getAcceleration()`
  - `setAccelerationLowerLimit()`, `setAccelerationUpperLimit()`
  - `resetAccelerations()`

- **Force Control:**
  - `setForce()`, `getForce()`
  - `setForceLowerLimit()`, `setForceUpperLimit()`
  - `resetForces()`

- **Commands:**
  - `setCommand()`, `getCommand()` - Set desired commands
  - `resetCommands()`

- **Passive Forces:**
  - `setSpringStiffness()`, `getSpringStiffness()`
  - `setRestPosition()`, `getRestPosition()`
  - `setDampingCoefficient()`, `getDampingCoefficient()`
  - `setCoulombFriction()`, `getCoulombFriction()`

- **Kinematics:**
  - `getRelativeTransform()` - Transform between bodies
  - `getRelativeSpatialVelocity()`, `getRelativeSpatialAcceleration()`
  - `getRelativeJacobian()`, `getRelativeJacobianTimeDeriv()`

- **Integration:**
  - `integratePositions()`, `integrateVelocities()`
  - `getPositionDifferences()` - Configuration space differences

- **Energy:**
  - `computePotentialEnergy()` - Spring potential energy

- **Constraint Impulses:**
  - `setConstraintImpulse()`, `getConstraintImpulse()`
  - `setVelocityChange()`, `getVelocityChange()`

**Mimic Joint Feature:**
- Allows a joint to mimic the motion of another joint
- Supports different multipliers and offsets per DOF
- Useful for coupled mechanisms

**Internal Update Methods:**
- `updateRelativeTransform()`, `updateRelativeSpatialVelocity()`
- `updateRelativeSpatialAcceleration()`, `updateRelativePrimaryAcceleration()`
- `updateRelativeJacobian()`, `updateRelativeJacobianTimeDeriv()`
- Various forward/inverse dynamics update methods

---

### 4. DegreeOfFreedom (`DegreeOfFreedom.hpp`)
**File Path:** `dart/dynamics/DegreeOfFreedom.hpp`

**Purpose:** Proxy class for accessing individual degrees of freedom (generalized coordinates). Provides a unified interface to control single DOFs.

**Key Features:**
- **Naming:** Automatic naming based on Joint, with option to preserve custom names
- **Indexing:**
  - Index within Skeleton
  - Index within tree
  - Index within Joint
  - Tree index

- **Properties:** Access to all properties of a single DOF
  - Commands
  - Position (with limits)
  - Velocity (with limits)
  - Acceleration (with limits)
  - Force (with limits)
  - Initial values
  - Passive force parameters (spring, damping, friction)

**Key Methods:**
- **Naming:**
  - `setName()`, `getName()`
  - `preserveName()` - Prevent automatic renaming

- **Indexing:**
  - `getIndexInSkeleton()`, `getIndexInTree()`, `getIndexInJoint()`
  - `getTreeIndex()`

- **State Access:** Similar to Joint methods but for single DOF
  - Position: `setPosition()`, `getPosition()`, etc.
  - Velocity: `setVelocity()`, `getVelocity()`, etc.
  - Acceleration, Force, Command methods
  - Limits and initial values

- **Relationships:**
  - `getJoint()` - Parent Joint
  - `getSkeleton()` - Parent Skeleton
  - `getChildBodyNode()`, `getParentBodyNode()` - Connected bodies

**Usage Pattern:**
```cpp
// Access DOF through Skeleton or Joint
DegreeOfFreedom* dof = skeleton->getDof(index);
dof->setPosition(value);
double pos = dof->getPosition();
```

---

### 5. Inertia (`Inertia.hpp`)
**File Path:** `dart/dynamics/Inertia.hpp`

**Purpose:** Represents the inertial properties of a rigid body (mass, center of mass, moment of inertia).

**Key Features:**
- **Minimal Parameters:**
  - Mass
  - Center of mass (3D vector)
  - Moment of inertia (6 parameters: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)

- **Spatial Inertia Tensor:** 6x6 matrix representation for spatial dynamics
- **Validation:** Methods to verify physical validity of inertia parameters

**Key Methods:**
- **Parameter Access:**
  - `setParameter()`, `getParameter()` - Generic parameter access
  - `setMass()`, `getMass()`
  - `setLocalCOM()`, `getLocalCOM()`
  - `setMoment()`, `getMoment()` - Moment of inertia matrix

- **Spatial Tensor:**
  - `setSpatialTensor()`, `getSpatialTensor()` - 6x6 spatial inertia

- **Validation:**
  - `verify()` - Check if inertia is physically valid
  - `verifyMoment()` - Static method to check moment of inertia
  - `verifySpatialTensor()` - Static method to check spatial tensor

**Physical Constraints:**
- Mass must be positive
- Moment of inertia must be positive semi-definite
- Triangle inequality for principal moments

---

### 6. Shape (`Shape.hpp`)
**File Path:** `dart/dynamics/Shape.hpp`

**Purpose:** Abstract base class for geometric shapes used for collision detection and visualization.

**Key Features:**
- **Shape Types:** (enumeration, deprecated - use `getType()` instead)
  - Primitive shapes: Sphere, Box, Ellipsoid, Cylinder, Capsule, Cone, Pyramid
  - Complex shapes: Mesh, SoftMesh, MultiSphere, HeightMap, LineSegment, Plane

- **Data Variance:** Hints for renderers about what might change
  - `STATIC` - Nothing changes
  - `DYNAMIC_TRANSFORM` - Transform might change
  - `DYNAMIC_PRIMITIVE` - Primitive properties might change
  - `DYNAMIC_COLOR` - Color/texture might change
  - `DYNAMIC_VERTICES` - Mesh vertices might change
  - `DYNAMIC_ELEMENTS` - Mesh topology might change
  - `DYNAMIC` - Everything might change

- **Bounding Box:** Automatic computation for efficient collision detection
- **Volume:** Automatic computation for mass properties
- **Inertia:** Can compute inertia tensor from mass or density

**Key Methods:**
- **Type:**
  - `getType()` - String representing shape type
  - `is<Type>()` - Template method for type checking (from Castable)

- **Geometry:**
  - `getBoundingBox()` - Axis-aligned bounding box
  - `getVolume()` - Enclosed volume
  - `computeInertia()` - Inertia tensor computation
  - `computeInertiaFromDensity()`, `computeInertiaFromMass()`

- **Data Variance:**
  - `setDataVariance()`, `getDataVariance()`
  - `addDataVariance()`, `removeDataVariance()`
  - `checkDataVariance()`

- **Updates:**
  - `refreshData()` - Update shape data
  - `incrementVersion()` - Notify subscribers of changes

- **Cloning:**
  - `clone()` - Deep copy the shape

**Usage:** Shapes are typically attached to BodyNodes via ShapeNodes, which add additional aspects like visual appearance, collision properties, etc.

---

## Key Relationships

```
Skeleton
 ├── Contains multiple BodyNode trees
 ├── Manages all Joint objects
 └── Provides global DOF indexing

BodyNode (hierarchical tree structure)
 ├── Connected to parent BodyNode via parent Joint
 ├── Has multiple child BodyNodes (via child Joints)
 ├── Contains Inertia object (mass properties)
 ├── Has attached ShapeNodes (for collision/visualization)
 │   └── Each ShapeNode references a Shape object
 ├── Has attached EndEffectors (for manipulation)
 └── Has attached Markers (for tracking)

Joint (connection between BodyNodes)
 ├── Connects parent BodyNode to child BodyNode
 ├── Defines relative transformation
 ├── Contains multiple DegreeOfFreedom objects
 └── Specifies motion constraints

DegreeOfFreedom
 ├── Belongs to a specific Joint
 ├── Represents a single generalized coordinate
 └── Provides unified interface to DOF properties
```

## Dynamics Computation Flow

### Forward Kinematics
1. Start from root BodyNode
2. For each BodyNode in tree order:
   - Update Joint transform (`updateRelativeTransform()`)
   - Update BodyNode world transform
   - Update spatial velocities
   - Update spatial accelerations

### Inverse Dynamics
1. Forward kinematics (compute transforms, velocities, accelerations)
2. Backward recursion from leaves to root:
   - Compute spatial forces at each BodyNode
   - Project forces to joint space to get required torques

### Forward Dynamics
1. Forward kinematics (transforms and velocities)
2. Forward pass (root to leaves):
   - Compute articulated body inertias
   - Compute bias forces
3. Backward pass (leaves to root):
   - Compute joint accelerations
4. Forward pass (root to leaves):
   - Update body accelerations

### Impulse-based Dynamics
Similar to forward dynamics but works with impulses and velocity changes instead of forces and accelerations. Used for contact resolution.

## Important Design Patterns

### 1. Lazy Evaluation
- Uses dirty flags to avoid redundant computations
- Jacobians, mass matrices, force vectors are computed only when needed
- Example: `mNeedTransformUpdate`, `mIsBodyJacobianDirty`

### 2. Aspect-Oriented Programming
- Uses Aspects to add properties/behaviors to classes
- Example: ShapeNode can have VisualAspect, CollisionAspect, DynamicsAspect

### 3. Smart Pointers
- Uses std::shared_ptr and std::weak_ptr for memory management
- Defines type aliases like SkeletonPtr, BodyNodePtr, etc.

### 4. Composite Pattern
- Skeleton contains trees of BodyNodes
- Each BodyNode can have multiple children
- Properties can be aggregated from components

### 5. Template Specialization
- GenericJoint<SpaceT> provides concrete joint types
- TemplatedJacobianNode<NodeT> provides Jacobian functionality

## Common Joint Types (Derived from Joint)

While not in the files we examined, DART provides several concrete Joint types:
- `RevoluteJoint` - 1-DOF rotational joint
- `PrismaticJoint` - 1-DOF translational joint
- `UniversalJoint` - 2-DOF joint (2 perpendicular revolute axes)
- `BallJoint` - 3-DOF rotational joint
- `FreeJoint` - 6-DOF joint (full spatial freedom)
- `WeldJoint` - 0-DOF joint (fixed connection)
- `TranslationalJoint` - 3-DOF translational joint
- `PlanarJoint` - 3-DOF planar joint (2 translations + 1 rotation)
- `EulerJoint`, `QuaternionJoint` - Various 3-DOF rotational representations

## Usage Example (Conceptual)

```cpp
// Create a skeleton
auto skeleton = Skeleton::create("MyRobot");

// Create root body and joint
auto [rootJoint, rootBody] = skeleton->createJointAndBodyNodePair<FreeJoint>();
rootBody->setMass(1.0);
rootBody->setMomentOfInertia(Ixx, Iyy, Izz);

// Add a child body
auto [joint1, body1] = rootBody->createChildJointAndBodyNodePair<RevoluteJoint>();
body1->setMass(0.5);

// Set joint properties
joint1->setPosition(0, 0.5);  // Set position of first DOF
joint1->setPositionLimits(0, -M_PI, M_PI);  // Set limits

// Add a shape for visualization/collision
auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.5));
auto shapeNode = body1->createShapeNode(shape);

// Compute dynamics
skeleton->setGravity(Eigen::Vector3d(0, 0, -9.81));
skeleton->computeForwardDynamics();

// Access computed results
const Eigen::MatrixXd& M = skeleton->getMassMatrix();
const Eigen::VectorXd& C = skeleton->getCoriolisForces();
const Eigen::VectorXd& g = skeleton->getGravityForces();
```

## MeshShape and TriMesh

**Design Decision:** MeshShape internally uses `dart::math::TriMesh<double>` for mesh representation instead of Assimp's `aiScene*`. This decouples mesh data from the loading library (Assimp), enabling format-agnostic mesh handling. The deprecated `aiScene*` API is maintained for backward compatibility via lazy on-demand conversion. For OSG rendering, materials/textures are accessed through `getMaterials()` (Assimp-free), while scene graph hierarchy uses the deprecated `getMesh()` (acceptable since TriMesh doesn't include scene graph structure). See `dart/dynamics/MeshShape.hpp` and `dart/utils/MeshLoader.hpp` for implementation.

## Summary

The DART dynamics module provides a comprehensive framework for:
1. **Modeling** articulated rigid body systems with arbitrary topology
2. **Kinematics** - Computing positions, velocities, accelerations, and Jacobians
3. **Dynamics** - Computing forces, torques, mass matrices, and solving equations of motion
4. **Collision** - Representing shapes and handling collision detection
5. **Constraints** - Handling joint limits, contacts, and other constraints
6. **Control** - Supporting various actuation models and control strategies

The modular design with clear separation between kinematics, dynamics, and geometry makes it suitable for robotics simulation, animation, and control applications.
