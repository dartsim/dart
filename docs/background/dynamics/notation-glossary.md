# Dynamics Notation Glossary

> **Attribution**: This glossary maps notation from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain to DART's C++ API.
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Recursive Inverse Dynamics](07_recursive-inverse-dynamics.md) | [Index](../README.md)

---

## Core Quantities

### State Variables

| PDF Symbol    | Description               | DART API                       | Return Type       |
| ------------- | ------------------------- | ------------------------------ | ----------------- |
| $q$           | Generalized positions     | `Skeleton::getPositions()`     | `Eigen::VectorXd` |
| $\dot{q}$     | Generalized velocities    | `Skeleton::getVelocities()`    | `Eigen::VectorXd` |
| $\ddot{q}$    | Generalized accelerations | `Skeleton::getAccelerations()` | `Eigen::VectorXd` |
| $\tau$ or $Q$ | Generalized forces        | `Skeleton::getForces()`        | `Eigen::VectorXd` |
| $n$           | Number of DOFs            | `Skeleton::getNumDofs()`       | `std::size_t`     |

### Dynamics Matrices

| PDF Symbol       | Description                 | DART API                                  | Return Type       |
| ---------------- | --------------------------- | ----------------------------------------- | ----------------- |
| $M(q)$           | Mass matrix                 | `Skeleton::getMassMatrix()`               | `Eigen::MatrixXd` |
| $M^{-1}(q)$      | Inverse mass matrix         | `Skeleton::getInvMassMatrix()`            | `Eigen::MatrixXd` |
| $M_{aug}$        | Augmented mass matrix       | `Skeleton::getAugMassMatrix()`            | `Eigen::MatrixXd` |
| $C(q,\dot{q})$   | Coriolis/centrifugal forces | `Skeleton::getCoriolisForces()`           | `Eigen::VectorXd` |
| $g(q)$ or $N(q)$ | Gravity forces              | `Skeleton::getGravityForces()`            | `Eigen::VectorXd` |
| $C + g$          | Combined bias forces        | `Skeleton::getCoriolisAndGravityForces()` | `Eigen::VectorXd` |
| $F_k$            | Constraint forces           | `Skeleton::getConstraintForces()`         | `Eigen::VectorXd` |

### Equation of Motion

The standard form in the PDF:

$$M(q)\ddot{q} + C(q,\dot{q}) = Q$$

In DART (with gravity and external forces):

$$M(q)\ddot{q} + C(q,\dot{q}) + g(q) = \tau + \tau_{ext} + J^T \lambda$$

## Per-Body Quantities

### Rigid Body Properties

| PDF Symbol | Description              | DART API                              | Notes             |
| ---------- | ------------------------ | ------------------------------------- | ----------------- |
| $m$        | Mass                     | `BodyNode::getMass()`                 | Scalar            |
| $I_c$      | Inertia tensor (COM)     | `BodyNode::getInertia()`              | 3×3 matrix        |
| $I_0$      | Inertia at zero rotation | Internal                              | $I_c = R I_0 R^T$ |
| $x$        | COM position (world)     | `BodyNode::getCOM()`                  | `Eigen::Vector3d` |
| $R$        | Rotation matrix          | `BodyNode::getTransform().rotation()` | `Eigen::Matrix3d` |
| $v$        | Linear velocity (world)  | `BodyNode::getCOMLinearVelocity()`    | `Eigen::Vector3d` |
| $\omega$   | Angular velocity (world) | `BodyNode::getAngularVelocity()`      | `Eigen::Vector3d` |
| $M_c$      | Spatial inertia          | `BodyNode::getSpatialInertia()`       | 6×6 matrix        |
| $V$        | Spatial velocity         | `BodyNode::getSpatialVelocity()`      | `Eigen::Vector6d` |

### Jacobians

| PDF Symbol           | Description            | DART API                             | Size   |
| -------------------- | ---------------------- | ------------------------------------ | ------ |
| $J_k$                | Body Jacobian          | `BodyNode::getJacobian()`            | 6×n    |
| $J_{vk}$             | Linear Jacobian        | `BodyNode::getLinearJacobian()`      | 3×n    |
| $J_{\omega k}$       | Angular Jacobian       | `BodyNode::getAngularJacobian()`     | 3×n    |
| $J'$                 | Jacobian at point      | `BodyNode::getWorldJacobian(offset)` | 6×n    |
| $\hat{J}_{\omega k}$ | Local angular Jacobian | Internal                             | 3×n(k) |

## Matrix Notation

### Skew-Symmetric Matrix

The PDF uses $[a]$ to denote the skew-symmetric matrix of vector $a$:

$$[a] = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}$$

In DART: `dart::math::makeSkewSymmetric(a)` or cross product operations.

### Key Identities

| Identity                         | Meaning                        |
| -------------------------------- | ------------------------------ |
| $[a]b = a \times b$              | Cross product                  |
| $[a]b = -[b]a$                   | Anti-commutativity             |
| $[a]^T = -[a]$                   | Skew-symmetric property        |
| $-[a][a] = (a^T a)I_3 - aa^T$    | Useful for inertia             |
| $[a \times b] = [a][b] - [b][a]$ | Jacobi identity                |
| $[R\omega] = R[\omega]R^T$       | Rotation transformation        |
| $[\omega] = \dot{R}R^T$          | Angular velocity from rotation |

## Coordinate Frames

| PDF Notation        | Description                        |
| ------------------- | ---------------------------------- |
| World frame         | Global inertial frame              |
| $R_k^0$             | Rotation from world to link k      |
| $W_k^0$             | Homogeneous transform to link k    |
| $^\ell$ superscript | Local frame of link k              |
| $\hat{\omega}_k$    | Angular velocity in parent's frame |

## Tree Structure Notation

| Symbol   | Description           | Example                    |
| -------- | --------------------- | -------------------------- |
| $p(k)$   | Parent link index     | $p(4) = 2$                 |
| $p(1,k)$ | Path from root to k   | $p(1,4) = \{1,2,4\}$       |
| $c(k)$   | Child link indices    | Used in RNEA               |
| $n(k)$   | DOFs in joint k       | 1 for revolute, 3 for ball |
| $d_k$    | Joint-to-joint vector | Position offset            |
| $c_k$    | COM in local frame    | Constant                   |

## Algorithm Names

| PDF Name         | DART Implementation        | Complexity      |
| ---------------- | -------------------------- | --------------- |
| Forward Dynamics | `computeForwardDynamics()` | $O(n)$ via ABA  |
| Inverse Dynamics | `computeInverseDynamics()` | $O(n)$ via RNEA |
| Mass Matrix      | `getMassMatrix()`          | $O(n^2)$        |
| RNEA             | Recursive Newton-Euler     | $O(n)$          |
| ABA              | Articulated Body Algorithm | $O(n)$          |

## Python (dartpy) Equivalents

```python
import dartpy as dart

# State
q = skeleton.getPositions()      # numpy array
qdot = skeleton.getVelocities()
qddot = skeleton.getAccelerations()
tau = skeleton.getForces()

# Dynamics matrices
M = skeleton.getMassMatrix()     # numpy 2D array
C = skeleton.getCoriolisForces()
g = skeleton.getGravityForces()

# Per-body
for body in skeleton.getBodyNodes():
    mass = body.getMass()
    com = body.getCOM()
    J = body.getJacobian()
```

## Units Convention

DART uses SI units:

| Quantity | Unit                |
| -------- | ------------------- |
| Length   | meters (m)          |
| Mass     | kilograms (kg)      |
| Time     | seconds (s)         |
| Force    | Newtons (N)         |
| Torque   | Newton-meters (N·m) |
| Angle    | radians (rad)       |

## See Also

- [`docs/onboarding/dynamics.md`](../../onboarding/dynamics.md) — Code exploration
- [`docs/readthedocs/topics/control-theory.md`](../../readthedocs/topics/control-theory.md) — Control notation mapping
- [`dart/dynamics/Skeleton.hpp`](../../../dart/dynamics/Skeleton.hpp) — Source code

---

**Navigation**: [← Recursive Inverse Dynamics](07_recursive-inverse-dynamics.md) | [Index](../README.md)
