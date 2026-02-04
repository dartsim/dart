# DART Core Architecture Analysis

## Start here next time

- Modifying dynamics/kinematics? Start with [Dynamics Layer](#3-dynamics-layer-dartdynamics)
- Working on collision? See [Collision Detection Layer](#2-collision-detection-layer-dartcollision)
- Adding constraints? Check [Constraint Layer](#4-constraint-layer-dartconstraint)
- Understanding simulation loop? Read [Complete Simulation Pipeline](#complete-simulation-pipeline)
- Key command: `pixi run test-all`

## Overview

DART (Dynamic Animation and Robotics Toolkit) is a comprehensive physics simulation library for robotics and computer animation. It provides accurate and stable simulation of articulated rigid body systems using **generalized coordinates** and **Featherstone's Articulated Body Algorithm**.

**Project Location:** ``

---

## Module Architecture

DART follows a **modular layered architecture** with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                       │
│                (GUI, Examples, Python API)                  │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                   Simulation Layer                          │
│         (World, Recording) - simulation/                    │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                  Constraint Layer                           │
│   (ConstraintSolver, Constraints) - constraint/             │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                   Dynamics Layer                            │
│    (Skeleton, BodyNode, Joint) - dynamics/                  │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│              Collision Detection Layer                      │
│  (CollisionDetector, CollisionGroup) - collision/           │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                   Foundation Layer                          │
│ (Math, Common utilities, LCP) - math/, common/, lcpsolver/  │
└─────────────────────────────────────────────────────────────┘
```

---

## Core Modules

### 1. **Foundation Layer** (common/, math/, lcpsolver/)

> Legacy `dart/integration` has been removed; time stepping now lives alongside the world/simulator code. Advanced optimizers moved into `dart/math/optimization/` with deprecated shims in `dart/optimizer/`.

#### 1.1 Common Module (`dart/common/`)

**Purpose:** Provides foundational utilities and design patterns used throughout DART.

**Key Components:**

- **Aspect System** (`Aspect.hpp`)
  - Plugin architecture for extending objects with arbitrary properties and state
  - Separates **State** (frequently changing) from **Properties** (rarely changing)
  - Enables runtime composition of behaviors
  - Used extensively in dynamics (BodyNode, Skeleton, etc.)
  - See [`aspect-system.md`](aspect-system.md) for implementation details

- **Memory Management**
  - `MemoryManager.hpp` - Custom memory allocation
  - `PoolAllocator.hpp` - Pool-based allocation for performance
  - `FreeListAllocator.hpp` - Free list allocation
  - `SmartPointer.hpp` - Smart pointer utilities

- **Observer Pattern**
  - `Subject.hpp`, `Observer.hpp` - Event notification system
  - `Signal.hpp` - Signal/slot mechanism for callbacks

- **Factory Pattern**
  - `Factory.hpp` - Generic factory for creating objects by type string
  - `Singleton.hpp` - Singleton implementation
  - Used for collision detector selection, constraint types, etc.

- **Composite Pattern**
  - `Composite.hpp` - Base for composable objects
  - Integrates with Aspect system

- **Utilities**
  - `NameManager.hpp` - Manages unique names
  - `VersionCounter.hpp` - Version tracking for change detection
  - `Stopwatch.hpp` - Timing utilities
  - `Logging.hpp` - Logging utilities
  - `ResourceRetriever.hpp` - Resource loading abstraction

**Key Design Patterns:**

- Aspect-Oriented Design for extensibility
- Observer pattern for event handling
- Factory pattern for polymorphic creation
- RAII for resource management

---

#### 1.2 Math Module (`dart/math/`)

**Purpose:** Mathematical utilities for robotics and physics simulation.

**Key Components:**

- **Geometry.hpp** - Transformation utilities:
  - Euler angle conversions (all 12 conventions: XYZ, ZYX, etc.)
  - Quaternion operations (expToQuat, quatToExp)
  - Skew-symmetric matrix operations
  - Rotation matrix derivatives
  - Axis-angle conversions
  - SE(3) transformations

- **Constants.hpp** - Mathematical constants (PI, tolerances, etc.)

- **MathTypes.hpp** - Type definitions for matrices and vectors

- **Random.hpp** - Random number generation utilities

- **TriMesh.hpp** - Triangle mesh operations

- **ConfigurationSpace.hpp** - Configuration space utilities

- **Icosphere.hpp** - Icosphere generation for spherical structures

**Dependencies:**

- Heavy use of **Eigen** library for linear algebra
- All math operations designed for real-time performance

---

#### 1.3 SIMD Module (`dart/simd/`)

**Purpose:** Explicit SIMD acceleration for batch operations on geometric data.

**Design Decisions:**

- **Explicit vectorization** over auto-vectorization for predictable performance
- **Multi-ISA support**: SSE4.2, AVX, AVX2+FMA, AVX-512, ARM NEON (SVE detection ready)
- **Eigen interop**: Seamless conversion between `dart::simd` types and Eigen types
- **camelCase naming**: Follows DART C++ conventions (`fromAxisAngle`, `transformPoint`)

**Key Components:**

- `Vec<T,W>` - Fixed-width SIMD register wrapper (e.g., `Vec<float,8>` for AVX)
- `Vector3<T>`, `Matrix4x4<T>` - SIMD-backed geometric types in `dart/simd/geometry/`
- `Quaternion<T>`, `Isometry3<T>` - Rigid body transforms with SIMD operations
- `DynamicVector<T>`, `DynamicMatrix<T>` - Runtime-sized SIMD containers in `dart/simd/dynamic/`
- `simdChunks<N>()` - Streaming iterator for processing Eigen vectors in batches

**When to Use:**

- Batch collision detection (AABB tests, point transforms)
- Bulk force/velocity updates in dynamics
- Mesh/point cloud transformations
- Any operation on arrays of 3D/4D vectors

**Performance:** See `tests/benchmark/simd/` for benchmarks. Run with `pixi run bm-simd`.

---

#### 1.4 LCP Solver Module (`dart/math/lcp/`)

**Purpose:** Solve Linear Complementarity Problems (LCP) for constraint-based dynamics.

**Key Components:**

- **pivoting/LemkeSolver.hpp** - Lemke's algorithm for solving LCPs
  - `int Lemke(M, q, z)` - Main solver
  - `bool validate(M, z, q)` - Solution validation

**Mathematical Foundation:**
LCP formulation: Find `z ≥ 0` such that `w = Mz + q ≥ 0` and `z^T w = 0`

Used by constraint solvers to compute contact forces, joint limits, and other constraints.

---

#### 1.4 Time Integration (Simulation Internals)

**Purpose:** Advance the world state each tick after dynamics and constraints are resolved.

**Implementation Notes:**

- Integration happens inside `dart::simulation::World::step()`.
- The solver integrates generalized velocities using computed accelerations, then integrates positions using the updated velocities (semi-implicit Euler).
- The integrator is no longer a standalone module; downstream code does not plug in custom schemes.

**Default Scheme:** Semi-implicit (symplectic) Euler chosen for its stability on rigid-body problems without incurring RK4 cost.

---

### 2. **Collision Detection Layer** (`dart/collision/`)

**Purpose:** Detect collisions and compute contact information between rigid bodies.

**Core Architecture:**

```
CollisionDetector (Factory)
    ├── CollisionGroup (manages objects)
    ├── CollisionObject (wraps shapes)
    └── Contact (collision result)
```

#### Key Classes:

**1. CollisionDetector** (`dart/collision/CollisionDetector.hpp`)

- Abstract base class for all collision engines
- Factory pattern for runtime backend selection
- Methods:
  - `createCollisionGroup()` - Create collision group
  - `collide(group1, group2, option, result)` - Collision detection
  - `distance(group1, group2, option, result)` - Distance queries
  - `raycast(group, from, to, option, result)` - Ray casting

**2. CollisionGroup** (`dart/collision/CollisionGroup.hpp`)

- Manages a collection of collision objects
- Automatic tracking of BodyNodes/Skeletons
- Observer pattern to track ShapeFrame destruction
- Backend-specific implementations (FCL, Bullet, etc.)

**3. Contact** (`dart/collision/Contact.hpp`)

- Represents a single collision contact:
  - `point` (Vector3d) - Contact point in world frame
  - `normal` (Vector3d) - Normal from object2 to object1
  - `penetrationDepth` (double) - Penetration depth
  - `force` (Vector3d) - Contact force on object1
  - `collisionObject1/2` - The colliding objects

**4. Supporting Classes:**

- `CollisionObject` - Wraps backend collision geometry
- `CollisionOption` - Query configuration (max contacts, filters)
- `CollisionResult` - Stores all contacts from query
- `DistanceOption/Result` - For distance queries
- `RaycastOption/Result` - For ray casting
- `CollisionFilter` - Filters collision pairs

---

#### Collision Detection Backends:

DART supports **4 pluggable backends** via Strategy Pattern:

| Backend    | Directory | Purpose                    | Pros                                           | Cons                            |
| ---------- | --------- | -------------------------- | ---------------------------------------------- | ------------------------------- |
| **FCL**    | `fcl/`    | Flexible Collision Library | Fast, good broadphase (AABB tree), **default** | FCL bugs in contact computation |
| **Bullet** | `bullet/` | Bullet Physics             | Robust, well-tested                            | Heavier dependency              |
| **DART**   | `dart/`   | Native implementation      | No dependencies, lightweight                   | Limited features                |
| **ODE**    | `ode/`    | Open Dynamics Engine       | Good for ODE integration                       | No distance queries             |

**Recommendation:** Use **FCL** with MESH mode and DART contact computation (default).

**Backend-specific implementations:**

- Each backend implements `CollisionDetector` and `CollisionGroup`
- Factory registration allows runtime selection
- Example: `FCLCollisionDetector`, `BulletCollisionDetector`

**Files:**

- FCL: `dart/collision/fcl/FCLCollisionDetector.hpp`
- Bullet: `dart/collision/bullet/BulletCollisionDetector.hpp`
- DART: `dart/collision/dart/DARTCollisionDetector.hpp`
- ODE: `dart/collision/ode/OdeCollisionDetector.hpp`

---

### 3. **Dynamics Layer** (`dart/dynamics/`)

**Purpose:** Represent and compute dynamics of articulated rigid body systems.

**Core Hierarchy:**

```
Skeleton (articulated system)
    └── BodyNode (rigid body/link)
          ├── Joint (kinematic constraint)
          │     └── DegreeOfFreedom (single DOF)
          ├── Shape (geometry)
          ├── Inertia (mass properties)
          ├── EndEffector (task-space point)
          └── Marker (tracking point)
```

#### Key Classes:

**1. Skeleton** (`dart/dynamics/Skeleton.hpp`)

- Top-level container for articulated body systems
- Manages tree structures (supports multiple trees)
- **Key Features:**
  - Forward/inverse kinematics
  - Forward/inverse dynamics (Featherstone's ABA)
  - Mass matrix computation
  - Jacobian computation (spatial, linear, angular)
  - Constraint and impulse handling
  - State management (positions, velocities, accelerations, forces)
- **Methods:**
  - `computeForwardKinematics()` - Update positions/velocities
  - `computeForwardDynamics()` - Compute accelerations from forces
  - `computeInverseDynamics()` - Compute forces from accelerations
  - `getMassMatrix()` - Get mass matrix M
  - `getCoriolisAndGravityForces()` - Get C + G
  - `getConstraintForces()` - Get constraint forces

**2. BodyNode** (`dart/dynamics/BodyNode.hpp`)

- Represents a single rigid body (link) in the system
- **Physical Properties:**
  - Mass, center of mass, moment of inertia
  - Collision shapes
  - Visualization shapes
- **Kinematic State:**
  - Transform relative to parent and world
  - Spatial velocity and acceleration
  - Jacobians (position, velocity)
- **Dynamic Quantities:**
  - External forces
  - Internal forces (from joint actuators)
  - Constraint forces
  - Articulated body inertia
- **Relationships:**
  - Parent/child tree structure
  - Attached shapes, markers, end effectors

**3. Joint** (`dart/dynamics/Joint.hpp`)

- Defines kinematic constraints between BodyNodes
- **Actuator Types:**
  - `FORCE` - Force/torque control
  - `PASSIVE` - No actuation
  - `SERVO` - Position/velocity servo
  - `ACCELERATION` - Acceleration control
  - `VELOCITY` - Velocity control
  - `LOCKED` - Fixed joint
  - `MIMIC` - Mimics another joint
- **DOF Management:**
  - Position, velocity, acceleration limits
  - Force/torque limits
  - Passive forces (spring, damping, friction)
- **Joint Types:**
  - Revolute (1 DOF rotation)
  - Prismatic (1 DOF translation)
  - Screw (coupled rotation + translation)
  - Universal (2 DOF rotation)
  - Ball (3 DOF rotation)
  - Euler (3 DOF rotation with Euler angles)
  - Translational (3 DOF translation)
  - Planar (2D motion: 2 translation + 1 rotation)
  - Free (6 DOF)
  - Weld (0 DOF - fixed)

**4. DegreeOfFreedom** (`dart/dynamics/DegreeOfFreedom.hpp`)

- Proxy for accessing a single generalized coordinate
- Unified interface regardless of joint type
- Access position, velocity, acceleration, force

**5. Shape** (`dart/dynamics/Shape.hpp`)

- Geometric shapes for collision and visualization
- Types: Box, Sphere, Cylinder, Capsule, Cone, Mesh, SoftMesh, etc.
- Provides bounding boxes and volume computation

**6. Inertia** (`dart/dynamics/Inertia.hpp`)

- Encapsulates inertial properties:
  - Mass
  - Center of mass offset
  - Moment of inertia tensor
- Validation to ensure physical validity

**7. Supporting Classes:**

- **EndEffector** - Task-space control points
- **Marker** - Tracking points (e.g., for motion capture)
- **Frame** - Coordinate frame abstraction
- **SimpleFrame** - Standalone frame (not part of skeleton)
- **ShapeFrame** - Frame with attached collision/visual shape

---

#### Dynamics Computation Flow:

**Forward Kinematics:**

```
Skeleton::computeForwardKinematics()
  └─> For each BodyNode (root to leaf):
        ├─> Joint::updateRelativeTransform()
        ├─> Joint::updateRelativeSpatialVelocity()
        ├─> Joint::updateRelativeSpatialAcceleration()
        ├─> BodyNode::updateWorldTransform()
        ├─> BodyNode::updateWorldVelocity()
        └─> BodyNode::updateWorldAcceleration()
```

**Forward Dynamics (Featherstone's ABA):**

```
Skeleton::computeForwardDynamics()
  ├─> Pass 1 (leaf to root): Compute articulated body inertia
  ├─> Pass 2 (root to leaf): Compute accelerations
  └─> updateImplicitJointVelDampingForces()
```

**Mass Matrix:**

```
Skeleton::getMassMatrix()
  └─> For each DOF:
        ├─> Set unit acceleration
        ├─> computeInverseDynamics()
        └─> Extract generalized forces → column of M
```

---

#### Design Patterns in Dynamics:

1. **Composite Pattern** - Skeleton contains BodyNodes in tree structure
2. **Aspect Pattern** - Extensibility through runtime-attached aspects
3. **Template Method** - Joint base class with customizable implementations
4. **Lazy Evaluation** - Jacobians, mass matrix computed on demand
5. **Caching** - Version tracking to avoid redundant computations
6. **Observer Pattern** - Notifications for state changes

---

**Key Files:**

- Skeleton: `dart/dynamics/Skeleton.hpp`
- BodyNode: `dart/dynamics/BodyNode.hpp`
- Joint: `dart/dynamics/Joint.hpp`
- DegreeOfFreedom: `dart/dynamics/DegreeOfFreedom.hpp`

---

### 4. **Constraint Layer** (`dart/constraint/`)

**Purpose:** Handle constraints in the simulation (contacts, joint limits, motors, etc.).

**Core Architecture:**

```
ConstraintSolver
    ├── ConstrainedGroup (groups skeletons)
    │     └── ConstraintBase (individual constraints)
    └── LcpSolver (Dantzig, Pgs, etc.)
```

#### Key Classes:

**1. ConstraintSolver** (`dart/constraint/ConstraintSolver.hpp`)

- Main orchestrator for constraint-based simulation
- **Responsibilities:**
  - Collision detection
  - Constraint grouping (independent groups for parallel solving)
  - LCP construction and solving
  - Impulse application
- **Process:**
  1.  Detect collisions → create ContactConstraints
  2.  Check joint limits → create JointLimitConstraints
  3.  Group interacting skeletons → ConstrainedGroups
  4.  Solve LCP for each group → compute impulses
  5.  Apply impulses to update velocities

**2. ConstraintBase** (`dart/constraint/ConstraintBase.hpp`)

- Abstract base for all constraint types
- **LCP Interface:**
  - `getDimension()` - Number of constraint equations
  - `update()` - Update constraint data
  - `getInformation()` - Fill LCP matrices (Jacobian, bounds, etc.)
  - `applyUnitImpulse()` - Apply unit impulse for Jacobian computation
  - `excite()` - Mark related skeletons as active
- **Lifecycle:**
  - Created → Updated → LCP solved → Applied → Destroyed

**4. ConstrainedGroup** (`dart/constraint/ConstrainedGroup.hpp`)

- Groups skeletons that interact through constraints
- Enables parallel solving of independent groups
- Efficient for multi-body systems with disconnected components

---

#### Constraint Types:

**Contact Constraints:**

- **ContactConstraint** - Rigid body contacts with:
  - Non-penetration constraint
  - Coulomb friction (pyramid approximation)
  - Restitution (coefficient of restitution)
  - ERP (Error Reduction Parameter) for penetration correction
  - CFM (Constraint Force Mixing) for soft contacts

**Joint Constraints:**

- **JointLimitConstraint** - Position/velocity limit enforcement
- **ServoMotorConstraint** - PD control as constraint
- **MimicMotorConstraint** - Joint coupling (unilateral servo)
- **CouplerConstraint** - Bilateral mimic coupling (equal/opposite impulses)

**Specialized Constraints:**

- **WeldJointConstraint** - Rigid connection between bodies
- **BallJointConstraint** - Ball joint constraint
- **BalanceConstraint** - Balance/support polygon constraint
- **DynamicJointConstraint** - Dynamic constraint for soft joints
- **SoftContactConstraint** - Soft/compliant contacts

---

#### LCP Solvers:

| Solver            | Algorithm                    | Accuracy | Speed  | Robustness/Notes                         |
| ----------------- | ---------------------------- | -------- | ------ | ---------------------------------------- |
| **DantzigSolver** | Principal pivoting (boxed)   | High     | Medium | Supports bounds + friction index mapping |
| **LemkeSolver**   | Complementary pivoting (LCP) | High     | Medium | Standard LCP (no bounds)                 |
| **PgsSolver**     | Projected Gauss-Seidel       | Medium   | Fast   | Iterative boxed LCP fallback with findex |

**Default Strategy:** ConstraintSolver uses `math::DantzigSolver` as primary
and optionally falls back to `math::PgsSolver` when configured.

---

#### Simulation Pipeline (World::step):

```
World::step()
  ├─> 1. Integrate unconstrained velocity (F = ma)
  ├─> 2. ConstraintSolver::solve()
  │     ├─> Collision detection
  │     ├─> Create constraints (contacts, limits, motors)
  │     ├─> Group skeletons into ConstrainedGroups
  │     ├─> For each group:
  │     │     ├─> Build LCP (A, x, b, lo, hi)
  │     │     ├─> Solve LCP → constraint impulses
  │     │     └─> Apply impulses → update velocities
  │     └─> Update positions using corrected velocities
  ├─> 3. Integrate state (semi-implicit Euler)
  │     └─> Update velocities then positions
  └─> 4. Record state (if recording enabled)
```

---

**Key Files:**

- ConstraintSolver: `dart/constraint/ConstraintSolver.hpp`
- ConstraintBase: `dart/constraint/ConstraintBase.hpp`
- ContactConstraint: `dart/constraint/ContactConstraint.hpp`
- LcpSolver implementations: `dart/math/lcp/*`

---

### 5. **Simulation Layer** (`dart/simulation/`)

**Purpose:** High-level simulation management and execution.

**Core Architecture:**

```
World (simulation environment)
    ├── Skeleton (multiple articulated systems)
    ├── SimpleFrame (independent frames)
    ├── ConstraintSolver (constraint handling)
    ├── Internal integrator (semi-implicit Euler)
    └── Recording (state history)
```

#### Key Classes:

**1. World** (`dart/simulation/World.hpp`)

- Top-level simulation container
- **Manages:**
  - Multiple Skeletons (articulated systems)
  - SimpleFrames (independent coordinate frames)
  - Collision detection
  - Constraint solving
  - Time stepping
- **Key Properties:**
  - Gravity (default: -9.81 m/s² in Y)
  - Time step (default: 0.001s = 1ms)
  - Current time and frame number
- **Key Methods:**
  - `step(resetCommand)` - Advance simulation by one time step
  - `checkCollision(option, result)` - Perform collision detection
  - `getSkeleton(index/name)` - Access skeletons
  - `addSkeleton(skeleton)` - Add skeleton to world
  - `setGravity(g)` - Set gravitational acceleration
  - `setTimeStep(dt)` - Set simulation time step
- **Workflow:**
  ```
  while (running):
      world.step()
        ├─> Compute dynamics
        ├─> Detect collisions
        ├─> Solve constraints
        ├─> Integrate positions
        └─> Update frame counter
  ```

**2. Recording** (`dart/simulation/Recording.hpp`)

- Records simulation state history
- Methods:
  - `bake()` - Store current state
  - `getNumFrames()` - Get number of recorded frames
  - `getSkeletonPositions(frame)` - Get skeleton state at frame
- Use cases:
  - Playback of simulations
  - Analysis of trajectory data
  - State checkpointing

#### Sensors

DART includes lightweight sensor scaffolding that is updated by the World to keep time and step context consistent. Sensors are managed centrally to keep naming and lifecycle coherent across world objects, while leaving concrete sensor types to downstream code.

---

**Key Files:**

- World: `dart/simulation/World.hpp`
- Recording: `dart/simulation/Recording.hpp`

---

## Main Entry Point

The main DART header that includes all modules:

**File:** `dart/All.hpp`

**Includes (in order):**

```cpp
#include <dart/config.hpp>         // Build configuration
#include <dart/common/All.hpp>     // Common utilities
#include <dart/math/All.hpp>       // Math utilities (includes math/optimization)
#include <dart/optimizer/All.hpp>  // Deprecated aliases forwarding to math/optimization
#include <dart/collision/All.hpp>  // Collision detection
#include <dart/constraint/All.hpp> // Constraints
#include <dart/dynamics/All.hpp>   // Dynamics
#include <dart/simulation/All.hpp> // Simulation
```

Users typically include just `<dart/All.hpp>` to access all functionality.

---

## Complete Simulation Pipeline

Here's the complete flow of a single simulation step:

```
User calls: world->step()
    │
    ├─> [1] Skeleton::computeForwardKinematics()
    │     └─> Update all transforms, velocities from joint states
    │
    ├─> [2] Skeleton::computeForwardDynamics()
    │     └─> Compute unconstrained accelerations using ABA
    │           q̈ = M^(-1) * (τ + τ_external - C - G)
    │
    ├─> [3] ConstraintSolver::solve()
    │     ├─> CollisionDetector::collide()
    │     │     └─> Detect collisions, generate contacts
    │     │
    │     ├─> Create ContactConstraints from contacts
    │     ├─> Create JointLimitConstraints from limits
    │     ├─> Create ServoMotorConstraints from motors
    │     │
    │     ├─> Group skeletons into ConstrainedGroups
    │     │
    │     ├─> For each ConstrainedGroup:
    │     │     ├─> Build boxed LCP (A, b, lo, hi, findex)
    │     │     │
    │     │     ├─> math::LcpSolver::solve()
    │     │     │     ├─> Primary: DantzigSolver (pivoting)
    │     │     │     └─> Optional fallback: PgsSolver (iterative)
    │     │     │
    │     │     └─> Apply constraint impulses λ to velocities
    │     │           v_new = v_old + M^(-1) * J^T * λ
    │     │
    │     └─> Update positions from corrected velocities
    │
    ├─> [4] Integrate state (semi-implicit Euler)
    │     ├─> v_{n+1} = v_n + a_n * dt
    │     └─> q_{n+1} = q_n + v_{n+1} * dt
    │
    ├─> [5] Update world time and frame counter
    │     time += dt
    │     frame += 1
    │
    └─> [6] Recording::bake() (if recording enabled)
          └─> Store current state for playback
```

---

## Key Design Patterns Used

DART employs numerous software design patterns throughout:

### 1. **Factory Pattern**

- `CollisionDetector::Factory` - Create detectors by type string
- `constraint::ConstraintFactory` - Create constraints dynamically
- Enables runtime polymorphism and plugin architectures

### 2. **Strategy Pattern**

- `CollisionDetector` - Different collision engines (FCL, Bullet, etc.)
- `LcpSolver` (`dart/math/lcp`) - Pivoting vs projection solvers (Dantzig,
  Lemke, Pgs)
- Allows swapping algorithms at runtime

### 3. **Observer Pattern**

- `Subject`/`Observer` - General event notification
- `Signal` - Typed signal/slot connections
- Used for name changes, state updates, etc.

### 4. **Composite Pattern**

- `Skeleton` contains `BodyNode` tree structure
- `Composite` base class for extensible objects
- Enables hierarchical structures

### 5. **Aspect Pattern**

- `Aspect` - Plugin properties and state to objects
- Separates concerns (visual, collision, physics, etc.)
- Enables runtime composition

### 6. **Template Method Pattern**

- `Joint` base class with customizable joint types
- `ConstraintBase` with standard solving flow
- Defines algorithm skeleton with customizable steps

### 7. **Singleton Pattern**

- `Factory::SingletonFactory` - Global factory instance
- Used for registration/lookup of collision detectors

### 8. **Lazy Evaluation**

- Mass matrices computed only when requested
- Jacobians cached until state changes
- Version tracking avoids redundant computation

### 9. **Visitor Pattern**

- Iteration over skeletons, body nodes, shapes
- `eachSkeleton()`, `eachBodyNode()` with callbacks

### 10. **RAII (Resource Acquisition Is Initialization)**

- Smart pointers throughout (`shared_ptr`, `unique_ptr`)
- Automatic resource cleanup
- Memory allocators with RAII wrappers

---

## Relationships Between Components

```
World
  ├─ Contains ──> Skeleton(s)
  ├─ Contains ──> SimpleFrame(s)
  ├─ Uses ────┬─> ConstraintSolver
  │           └─> CollisionDetector (via ConstraintSolver)
  └─ Integrates state internally (semi-implicit Euler)

Skeleton
  ├─ Contains ──> BodyNode tree
  ├─ Contains ──> Joint(s)
  └─ Contains ──> DegreeOfFreedom(s) (via Joints)

BodyNode
  ├─ Contains ──> Inertia
  ├─ Contains ──> Shape(s) via ShapeFrame(s)
  ├─ Contains ──> EndEffector(s)
  ├─ Contains ──> Marker(s)
  ├─ Has ──────> Parent Joint
  └─ Has ──────> Child Joint(s)

Joint
  ├─ Connects ──> Parent BodyNode
  ├─ Connects ──> Child BodyNode
  └─ Contains ──> DegreeOfFreedom(s)

ConstraintSolver
  ├─ Uses ──────> CollisionDetector
  ├─ Creates ───> ConstraintBase instances
  ├─ Creates ───> ConstrainedGroup(s)
  └─ Uses ──────> LcpSolver (math::DantzigSolver primary, PgsSolver fallback)

CollisionDetector
  ├─ Creates ───> CollisionGroup
  ├─ Creates ───> CollisionObject
  └─ Returns ───> Contact(s) in CollisionResult
```

---

## Key Algorithms

### 1. **Featherstone's Articulated Body Algorithm (ABA)**

- **Purpose:** Compute forward dynamics (accelerations from forces)
- **Complexity:** O(n) where n = number of bodies
- **Method:** Two passes through kinematic tree
  - Pass 1 (leaf→root): Compute articulated body inertia
  - Pass 2 (root→leaf): Compute accelerations
- **Implementation:** `Skeleton::computeForwardDynamics()`

### 2. **Composite Rigid Body Algorithm (CRBA)**

- **Purpose:** Compute mass matrix M
- **Complexity:** O(n²)
- **Method:** For each DOF, compute inverse dynamics with unit acceleration
- **Implementation:** `Skeleton::getMassMatrix()`

### 3. **Recursive Newton-Euler Algorithm (RNEA)**

- **Purpose:** Compute inverse dynamics (forces from accelerations)
- **Complexity:** O(n)
- **Method:** Two passes:
  - Pass 1 (root→leaf): Forward kinematics
  - Pass 2 (leaf→root): Backward dynamics
- **Implementation:** `Skeleton::computeInverseDynamics()`

### 4. **Linear Complementarity Problem (LCP) Solving**

- **Purpose:** Solve constraints (contacts, limits, etc.)
- **Formulation:** A·λ + b ≥ 0, λ ≥ 0, λ^T(A·λ + b) = 0
- **Solvers:**
  - Lemke/Dantzig (pivoting method)
  - PGS (iterative method)
- **Implementation:** `math::LemkeSolver`, `math::DantzigSolver`,
  `math::PgsSolver`

### 5. **Semi-Implicit Euler Integration**

- **Purpose:** Numerical time integration
- **Method:**
  ```
  v_{n+1} = v_n + a_n * dt
  q_{n+1} = q_n + v_{n+1} * dt  (uses new velocity!)
  ```
- **Benefits:** Energy conservation, symplectic
- **Implementation:** `World::step()` (built-in)

---

## Performance Considerations

### Computational Costs (per time step):

| Operation               | Complexity | Cost                          |
| ----------------------- | ---------- | ----------------------------- |
| Forward Kinematics      | O(n)       | Very fast                     |
| Forward Dynamics (ABA)  | O(n)       | Fast                          |
| Inverse Dynamics (RNEA) | O(n)       | Fast                          |
| Mass Matrix             | O(n²)      | Expensive                     |
| Jacobians               | O(n)       | Medium                        |
| Collision Detection     | O(m log m) | Variable (m = # objects)      |
| LCP Solving             | O(c³)      | Expensive (c = # constraints) |
| Integration             | O(n)       | Fast                          |

### Optimization Strategies:

1. **Lazy Evaluation** - Compute only when needed
2. **Caching** - Store results until state changes (version tracking)
3. **LCP Caching** - Reuse LCP structure across frames
4. **Broadphase Collision** - Reduce collision pairs (AABB trees)
5. **Independent Groups** - Solve disconnected systems separately
6. **Time Step Tuning** - Semi-implicit scheme stays stable if dt is kept modest

---

## Dependencies

### External Libraries:

| Library                   | Purpose             | Usage                      |
| ------------------------- | ------------------- | -------------------------- |
| **Eigen**                 | Linear algebra      | Math operations throughout |
| **FCL** (optional)        | Collision detection | Default collision backend  |
| **Bullet** (optional)     | Collision detection | Alternative backend        |
| **ODE** (optional)        | Collision/physics   | Alternative backend        |
| **urdfdom** (optional)    | URDF parsing        | Robot model loading        |
| **tinyxml2** (optional)   | XML parsing         | File I/O                   |
| **OpenGL/OSG** (optional) | Visualization       | GUI applications           |
| **assimp** (optional)     | 3D model loading    | Mesh import                |

### Build System:

- **CMake** - Cross-platform build configuration
- Located at: `CMakeLists.txt`

---

## Usage Example (Pseudocode)

```cpp
#include <dart/all.hpp>

int main() {
  // 1. Create world
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(0.001); // 1ms timestep

  // 2. Load or create skeleton
  auto skeleton = createRobotSkeleton(); // User-defined
  world->addSkeleton(skeleton);

  // 3. Set initial state
  skeleton->setPosition(0, 1.0);  // Set first DOF position
  skeleton->setVelocity(1, 0.5);  // Set second DOF velocity

  // 4. Simulation loop
  for (int i = 0; i < 10000; i++) {
    // Apply forces
    skeleton->setForce(0, 10.0);  // Apply torque to first DOF

    // Step simulation
    world->step();

    // Read state
    Eigen::VectorXd positions = skeleton->getPositions();
    Eigen::VectorXd velocities = skeleton->getVelocities();

    // Check collisions
    auto collisionResult = world->getLastCollisionResult();
    if (collisionResult.isCollision()) {
      // Handle collision
    }
  }

  return 0;
}
```

---

## Summary

DART is a well-architected physics simulation library with:

### **Strengths:**

1. ✅ **Modular design** - Clear separation of concerns
2. ✅ **Extensible** - Aspect system, factory pattern, plugin architecture
3. ✅ **Efficient** - O(n) algorithms (ABA, RNEA), lazy evaluation
4. ✅ **Accurate** - Featherstone's algorithms, constraint-based dynamics
5. ✅ **Flexible** - Multiple collision backends, integrators, solvers
6. ✅ **Well-tested** - Mature codebase with extensive testing
7. ✅ **Standards-compliant** - Supports URDF, SDF robot models

### **Core Components:**

- **Foundation:** Math, utilities, memory management, patterns
- **Collision:** Pluggable detection (FCL, Bullet, ODE, native)
- **Dynamics:** Skeleton, BodyNode, Joint, efficient algorithms
- **Constraints:** LCP-based contact resolution, joint limits, motors
- **Simulation:** World container, time stepping, integration
- **Integration:** Euler, Semi-Implicit Euler, RK4

### **Key Algorithms:**

- Featherstone's ABA (forward dynamics)
- RNEA (inverse dynamics)
- LCP solving (constraints)
- Semi-implicit Euler (integration)

### **Design Philosophy:**

- Performance through algorithmic efficiency (O(n) dynamics)
- Flexibility through design patterns (Strategy, Factory, Aspect)
- Maintainability through modularity and clean interfaces
- Extensibility through plugin architectures

This architecture makes DART suitable for:

- **Robotics** - Manipulation, locomotion, control
- **Animation** - Character animation, physics-based motion
- **Simulation** - Training environments, virtual prototyping
- **Research** - Algorithm development, benchmarking

---

## File Structure Summary

```
├── dart/
│   ├── dart.hpp              # Main entry point
│   ├── dart.cpp              # Empty implementation file
│   ├── common/               # Utilities, patterns, memory
│   ├── math/                 # Mathematical utilities (includes math/optimization/)
│   ├── lcpsolver/            # LCP solvers
│   ├── collision/            # Collision detection
│   │   ├── fcl/             # FCL backend
│   │   ├── bullet/          # Bullet backend
│   │   ├── dart/            # Native backend
│   │   └── ode/             # ODE backend
│   ├── dynamics/             # Articulated body dynamics
│   ├── constraint/           # Constraint solving
│   ├── simulation/           # World and simulation loop / time stepping
│   ├── math/optimization/    # Optimization helpers
│   ├── optimizer/            # Deprecated headers forwarding to math/optimization
│   └── gui/                  # Visualization (OSG, ImGui)
├── CMakeLists.txt            # Build configuration
└── README.md                 # Project overview
```
