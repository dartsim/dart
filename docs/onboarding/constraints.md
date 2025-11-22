# DART Constraint Module Analysis

## Overview

This document provides a comprehensive analysis of the constraint subsystem in the DART (Dynamic Animation and Robotics Toolkit) simulation library. The legacy `dart/integration` module was removed in DART 7; time integration now runs inside `dart::simulation::World::step()` using a built-in semi-implicit Euler scheme.

---

## Directory Structure

### Constraint Module (`dart/constraint`)

**Core Components:**

- ConstraintBase.hpp/cpp - Base class for all constraints
- ConstraintSolver.hpp/cpp - Main solver managing constraints
- ConstrainedGroup.hpp/cpp - Groups of interacting skeletons
- BoxedLcpConstraintSolver.hpp/cpp - LCP-based constraint solver

**Constraint Types:**

- ContactConstraint.hpp/cpp - Contact between rigid bodies
- SoftContactConstraint.hpp/cpp - Soft body contacts
- JointConstraint.hpp/cpp - Joint-space constraints
- JointLimitConstraint.hpp/cpp - Joint position/velocity limits
- JointCoulombFrictionConstraint.hpp/cpp - Joint friction
- ServoMotorConstraint.hpp/cpp - Servo motor control
- MimicMotorConstraint.hpp/cpp - Mimic motor behavior
- CouplerConstraint.hpp/cpp - Bilateral mimic coupling
- BalanceConstraint.hpp/cpp - Balance constraints
- BallJointConstraint.hpp/cpp - Ball joint constraints
- WeldJointConstraint.hpp/cpp - Weld joint constraints
- DynamicJointConstraint.hpp/cpp - Dynamic joint constraints

**LCP Solvers:**

- LCPSolver.hpp/cpp - Base LCP solver
- BoxedLcpSolver.hpp - Boxed LCP solver interface
- DantzigLCPSolver.hpp/cpp - Dantzig LCP solver
- DantzigBoxedLcpSolver.hpp/cpp - Dantzig boxed LCP solver
- PGSLCPSolver.hpp/cpp - Projected Gauss-Seidel LCP solver
- PgsBoxedLcpSolver.hpp/cpp - PGS boxed LCP solver

**Dantzig Solver Implementation:**
Modern C++20 template-based implementation in `dart/math/lcp/Dantzig/` with two key design decisions:

- **PivotMatrix:** Hybrid architecture combining Eigen storage with pointer-based O(1) row swapping
- **Hybrid SIMD:** Threshold-based strategy switching between raw pointers and Eigen SIMD based on problem size
  See code for implementation details.

**Utilities:**

- ContactSurface.hpp/cpp - Contact surface parameters
- SmartPointer.hpp - Smart pointer definitions

### Time Integration

Time integration is no longer a standalone module. `dart::simulation::World::step()` advances states directly using a semi-implicit Euler update (compute accelerations, update velocities, then update positions). The legacy `dart/integration` headers/sources have been removed.

---

## Constraint Module Architecture

### 1. ConstraintBase (Base Class)

**File:** `dart/constraint/ConstraintBase.hpp`

**Role:** Abstract base class defining the interface for all constraint types.

**Key Responsibilities:**

- Define constraint dimension
- Update constraint state based on skeleton states
- Provide constraint information for LCP formulation
- Apply impulses to constrained bodies
- Track constraint activation state

**Key Data Structures:**

```cpp
struct ConstraintInfo {
    double* x;        // Impulse
    double* lo;       // Lower bound of x
    double* hi;       // Upper bound of x
    double* b;        // Bias term
    double* w;        // Slack variable
    int* findex;      // Friction index
    double invTimeStep;
};
```

**Core Interface:**

- `update()` - Update constraint using skeleton states
- `getInformation(ConstraintInfo*)` - Fill LCP variables
- `applyUnitImpulse(index)` - Apply unit impulse to constraint space
- `getVelocityChange(vel, withCfm)` - Get velocity change from unit impulse
- `applyImpulse(lambda)` - Apply computed constraint impulse
- `isActive()` - Check if constraint is active
- `excite()/unexcite()` - Control constraint activation

### 2. ConstraintSolver (Main Solver)

**File:** `dart/constraint/ConstraintSolver.hpp`

**Role:** Central manager for all constraints in a simulation, orchestrating constraint solving.

**Key Responsibilities:**

- Manage multiple skeletons
- Handle constraint registration (manual and automatic)
- Perform collision detection
- Build and solve constrained groups
- Coordinate with LCP solvers

**Constraint Categories:**

1. **Automatic Constraints** (created by solver):
   - Contact constraints (from collision detection)
   - Soft contact constraints
   - Joint limit constraints
   - Joint Coulomb friction constraints
   - Mimic joint constraints (CouplerConstraint or MimicMotorConstraint)

2. **Manual Constraints** (user-added):
   - Custom constraints added via `addConstraint()`

**Workflow:**

```
solve()
  → updateConstraints()
  → buildConstrainedGroups()
  → solveConstrainedGroups()
    → solveConstrainedGroup() [per group]
```

**Collision Detection Integration:**

- Uses `CollisionDetector` to find contacts
- Creates `ContactConstraint` objects for each contact
- Manages `ContactSurfaceHandler` for surface parameters
- Supports multiple collision detection backends

**Key Features:**

- Separates constraints into independent groups for efficiency
- Supports skeleton unification for articulated systems
- Configurable time stepping
- Extensible constraint surface handling

### 3. ConstrainedGroup

**File:** `dart/constraint/ConstrainedGroup.hpp`

**Role:** Represents a group of skeletons that interact through constraints.

**Key Responsibilities:**

- Group constraints affecting the same set of bodies
- Provide unified interface for group-based solving
- Track total constraint dimension

**Purpose:** Enables efficient parallel solving of independent constraint groups.

### 4. BoxedLcpConstraintSolver

**File:** `dart/constraint/BoxedLcpConstraintSolver.hpp`

**Role:** Concrete implementation of ConstraintSolver using Boxed LCP formulation.

**Key Responsibilities:**

- Formulate constraints as boxed LCP: `A*x = b + w`
- Solve using primary and optional secondary LCP solvers
- Handle solver fallback on failure

**LCP Formulation:**

```
A*x = b + w
where each x[i], w[i] satisfies:
  (1) x = lo, w >= 0
  (2) x = hi, w <= 0
  (3) lo < x < hi, w = 0
```

**Solver Strategy:**

- Primary solver: DantzigBoxedLcpSolver (default)
- Secondary solver: PgsBoxedLcpSolver (fallback, default)
- Caches matrix data (A, x, b, lo, hi, w, findex) for efficiency

### 5. Constraint Types

#### ContactConstraint

**File:** `dart/constraint/ContactConstraint.hpp`

**Role:** Handles rigid body contact constraints with friction.

**Key Features:**

- Normal contact impulse (non-penetration)
- Tangential friction (Coulomb friction model)
- Restitution (bounce) modeling
- Contact surface motion velocity
- Error reduction parameter (ERP) for constraint stabilization
- Constraint force mixing (CFM) for soft constraints

**Physics Parameters:**

- Primary and secondary friction coefficients
- Slip compliance (for soft friction)
- Restitution coefficient
- Error allowance and reduction velocity

**Formulation:**

- 3D constraint (1 normal + 2 tangential directions)
- Uses spatial jacobians for constraint application
- Supports self-collision detection

#### JointConstraint

**File:** `dart/constraint/JointConstraint.hpp`

**Role:** Handles multiple joint-space constraints (limits, servo motors).

**Key Features:**

- Joint position limits
- Joint velocity limits
- Servo motor control
- Up to 6 DOF per joint
- Per-DOF activation tracking

**Constraint Parameters:**

- Impulse bounds (upper/lower)
- Desired velocity change
- Lifetime tracking (for iterative solvers)
- Error reduction parameters

#### JointLimitConstraint

**File:** `dart/constraint/JointLimitConstraint.hpp`

**Role:** Specifically handles joint position and velocity limits.

**Key Features:**

- Position limit violation detection and correction
- Velocity limit enforcement
- Per-DOF activation (up to 6 DOF)
- Violation magnitude tracking

**Implementation:**

- Computes required negative velocity to satisfy limits
- Uses impulse bounds to enforce constraints
- Tracks lifetime for stability

#### ServoMotorConstraint

**File:** `dart/constraint/ServoMotorConstraint.hpp`

**Role:** Implements servo motor control as a constraint.

**Key Features:**

- Position/velocity tracking
- Force limits (impulse bounds)
- Per-DOF activation
- CFM-based compliance

**Use Case:** Enables PD-style control through constraint framework.

#### CouplerConstraint

**File:** `dart/constraint/CouplerConstraint.hpp`

**Role:** Bilaterally enforces a mimic relationship between two joints by
applying equal-and-opposite impulses. Unlike `MimicMotorConstraint`, both the
reference joint and the dependent joint participate in the constraint solve, so
reaction torques propagate through the articulated system.

**Key Features:**

- Shares the same `MimicDofProperties` multipliers/offsets as mimic motors
- Activated by `Joint::setUseCouplerConstraint(true)` (per mimic joint)
- Couples multiple skeletons when reference and dependent joints belong to
  different rigs
- Uses the same LCP infrastructure (lifetime, impulse bounds, CFM/Coulomb
  handling) as the other joint constraints

**Recommended Use:** Gearbox-style mimic joints that require reaction forces
on both shafts (e.g., Gazebo gearbox joints). Leave the flag disabled to keep
the legacy servo-style behavior when bilateral coupling is not required.

### 6. LCP Solvers

#### BoxedLcpSolver Interface

**File:** `dart/constraint/BoxedLcpSolver.hpp`

**Role:** Abstract interface for boxed LCP solvers.

**Key Method:**

```cpp
bool solve(int n, double* A, double* x, double* b,
           int nub, double* lo, double* hi,
           int* findex, bool earlyTermination)
```

**Available Implementations:**

1. **DantzigBoxedLcpSolver** - Dantzig pivoting method (accurate but slower)
2. **PgsBoxedLcpSolver** - Projected Gauss-Seidel (fast iterative method)

**Solver Selection Strategy:**

- Dantzig: Better for small, dense systems; guaranteed convergence
- PGS: Better for large systems; faster but may not converge
- Default: Dantzig with PGS fallback

---

## Time Integration Overview

- `dart::simulation::World::step()` performs the semi-implicit Euler update internally once forces/constraints are resolved.
- Users no longer configure an external `Integrator`; the time step is controlled by `World::setTimeStep()`.
- The same integration logic applies across core C++ and dartpy.

---

## Simulation Pipeline Integration

### Overall Simulation Flow

```
World::step(dt)
  ↓
[1] Skeleton::computeForwardKinematics()
  ↓
[2] Skeleton::computeForwardDynamics()
    → Compute M (mass matrix)
    → Compute C (Coriolis/centrifugal forces)
    → Compute external forces
    → Solve: M*a = tau - C + F_ext
  ↓
[3] ConstraintSolver::solve()
    → Detect collisions
    → Create contact constraints
    → Update constraints
    → Build constrained groups
    → Solve each group (LCP)
    → Apply constraint impulses
  ↓
[4] Integrate state (semi-implicit Euler)
    → Get accelerations
    → Integrate velocities: v += a*dt
    → Integrate positions: q += v*dt
  ↓
[5] Update state (configurations, velocities)
```

### Constraint Solving Pipeline (Detailed)

```
ConstraintSolver::solve()
  |
  ├─> [1] updateConstraints()
  |     ├─> Detect collisions → ContactConstraints
  |     ├─> Check joint limits → JointLimitConstraints
  |     ├─> Check joint friction → JointCoulombFrictionConstraints
  |     ├─> Check mimic joints → CouplerConstraints or MimicMotorConstraints
  |     └─> Update manual constraints
  |
  ├─> [2] buildConstrainedGroups()
  |     ├─> Unite skeletons connected by constraints
  |     ├─> Group constraints by root skeleton
  |     └─> Create ConstrainedGroup objects
  |
  └─> [3] solveConstrainedGroups()
        └─> For each ConstrainedGroup:
              |
              └─> BoxedLcpConstraintSolver::solveConstrainedGroup()
                    |
                    ├─> Build LCP matrices:
                    |     ├─> A matrix (constraint jacobians * inv(M) * jacobians^T)
                    |     ├─> b vector (constraint velocity errors)
                    |     ├─> lo/hi bounds (friction cones, limits)
                    |     └─> findex (friction coupling)
                    |
                    ├─> Solve: A*x = b + w (primary solver)
                    |     └─> If failed: try secondary solver
                    |
                    └─> Apply impulses: lambda = x
                          └─> Each constraint: applyImpulse(lambda)
```

### Integration Notes

- `World::step()` uses semi-implicit Euler for all bodies.
- Users tune accuracy/stability via the time step rather than swapping integrators.

---

## Key Design Patterns

### 1. Strategy Pattern

- **Where:** BoxedLcpSolver hierarchy
- **Purpose:** Allow runtime selection of constraint solving method
- **Benefit:** Easy to add new solvers without modifying existing code

### 2. Template Method Pattern

- **Where:** ConstraintSolver with abstract `solveConstrainedGroup()`
- **Purpose:** Define skeleton of constraint solving algorithm
- **Benefit:** Concrete solvers only implement specific solving logic

### 3. Factory Pattern

- **Where:** Automatic constraint creation in ConstraintSolver
- **Purpose:** Centralized constraint creation based on simulation state
- **Benefit:** Encapsulates constraint creation logic

### 4. Visitor Pattern (Implicit)

- **Where:** Constraint iteration with callbacks
- **Purpose:** `eachConstraint()` allows operations on all constraints
- **Benefit:** Extensible without modifying ConstraintSolver

### 5. Object Pool Pattern (Implicit)

- **Where:** Constraint caching in ConstraintSolver
- **Purpose:** Reuse constraint objects across timesteps
- **Benefit:** Reduces allocation overhead

---

## Performance Considerations

### Constraint Solving

1. **Group Independence:** Independent groups can be solved in parallel
2. **LCP Caching:** Matrix data cached and reused when possible
3. **Solver Selection:** Dantzig for accuracy, PGS for speed
4. **Collision Detection:** Can use spatial acceleration structures
5. **Constraint Activation:** Inactive constraints skipped

### Integration

1. **Method Selection:**
   - Semi-implicit Euler: Best balance for real-time
   - RK4: Better accuracy, 4x cost
   - Euler: Fastest but least stable
2. **Timestep Selection:** Smaller dt = more stable but slower
3. **Separate Position/Velocity Integration:** Allows partial updates

---

## Configuration and Tuning

### Constraint Parameters

**Contact Constraints:**

- `ErrorAllowance` - How much penetration is allowed (default: small value)
- `ErrorReductionParameter` (ERP) - Constraint stabilization strength (0-1, default: 0.01)
- `MaxErrorReductionVelocity` - Maximum correction velocity
- `ConstraintForceMixing` (CFM) - Soft constraint parameter (1e-9 to 1, default: 1e-5)

**Joint Constraints:**

- Similar parameters as contact constraints
- Configured per constraint type

### Solver Configuration

**Primary Solver Selection:**

```cpp
auto dantzig = std::make_shared<DantzigBoxedLcpSolver>();
auto solver = std::make_shared<BoxedLcpConstraintSolver>(dantzig);
```

**With Fallback:**

```cpp
auto primary = std::make_shared<DantzigBoxedLcpSolver>();
auto secondary = std::make_shared<PgsBoxedLcpSolver>();
auto solver = std::make_shared<BoxedLcpConstraintSolver>(primary, secondary);
```

## Extension Points

### Adding New Constraint Types

1. Inherit from `ConstraintBase`
2. Implement all virtual methods
3. Register with `ConstraintSolver` (manually or automatically)
4. Define constraint dimension in constructor
5. Implement LCP formulation in `getInformation()`

### Adding New LCP Solvers

1. Inherit from `BoxedLcpSolver`
2. Implement `solve()` method
3. Handle the boxed LCP formulation
4. Register with `BoxedLcpConstraintSolver`

## Common Issues and Solutions

### Issue 1: Constraint Jitter

**Symptom:** Bodies vibrate at contacts
**Solution:**

- Reduce ERP (slower correction)
- Increase CFM (softer constraints)
- Use smaller timestep

### Issue 2: Penetration

**Symptom:** Bodies pass through each other
**Solution:**

- Reduce timestep
- Increase ERP (faster correction)
- Check collision detection accuracy
- Verify constraint activation

### Issue 3: Instability

**Symptom:** Simulation explodes
**Solution:**

- Keep timestep modest (semi-implicit Euler remains stable with small dt)
- Reduce timestep
- Check mass/inertia values
- Verify constraint bounds

### Issue 4: Performance

**Symptom:** Simulation too slow
**Solution:**

- Use PGS solver for large systems
- Optimize collision detection
- Reduce timestep resolution
- Profile constraint count

---

## Best Practices

1. **Timestep Selection:**
   - Start with dt = 0.001 (1ms) for semi-implicit Euler
   - Adjust based on stability requirements
   - Use fixed timestep for deterministic results

2. **Constraint Tuning:**
   - Keep CFM small but non-zero (1e-5 to 1e-4)
   - Use ERP around 0.01-0.1 for stability
   - Tune friction coefficients based on physical materials

3. **Solver Selection:**
   - Default: Dantzig + PGS fallback
   - For large systems: PGS only
   - For small, critical systems: Dantzig only

4. **Integration Stability:**
   - Semi-implicit Euler is fixed; tune dt for stability/accuracy
   - Use substepping for highly stiff scenes

5. **Collision Detection:**
   - Use appropriate collision detector for scene complexity
   - Balance accuracy vs. performance
   - Consider swept collision detection for fast objects

---

## References

### Key Files Analyzed

**Constraint Module:**

- `dart/constraint/ConstraintBase.hpp`
- `dart/constraint/ConstraintSolver.hpp`
- `dart/constraint/ConstrainedGroup.hpp`
- `dart/constraint/BoxedLcpConstraintSolver.hpp`
- `dart/constraint/ContactConstraint.hpp`
- `dart/constraint/JointConstraint.hpp`
- `dart/constraint/JointLimitConstraint.hpp`
- `dart/constraint/ServoMotorConstraint.hpp`
- `dart/constraint/BoxedLcpSolver.hpp`

### Related Concepts

- Linear Complementarity Problem (LCP)
- Dantzig Algorithm
- Projected Gauss-Seidel (PGS)
- Constraint Force Mixing (CFM)
- Error Reduction Parameter (ERP)
- Symplectic Integration
- Runge-Kutta Methods

---

## Summary

The DART constraint and integration modules provide a sophisticated framework for physics-based simulation:

**Constraint Module** handles:

- Multiple constraint types (contact, joint limits, motors)
- Flexible LCP-based solving with multiple solver options
- Automatic constraint generation from collisions and joint states
- Efficient grouping of independent constraints

**Integration Module** provides:

- Multiple numerical integration schemes
- Configurable accuracy vs. performance tradeoffs
- Support for special configuration spaces

Together, these modules enable robust, stable, and efficient physics simulation suitable for robotics, animation, and general multibody dynamics applications.
