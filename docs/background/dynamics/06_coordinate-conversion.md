# Conversion between Cartesian and Generalized Coordinates

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Articulated Dynamics](05_articulated-dynamics.md) | [Index](../README.md) | [Recursive Inverse Dynamics →](07_recursive-inverse-dynamics.md)

---

## Overview

In practice, we often want to use third-party rigid body simulators rather than develop
our own. ODE, PhysX, and Bullet are popular physics engines that use the **maximal
representation** (6 DOFs per link) rather than **generalized coordinates**.

A common practice is to:

1. Develop control algorithms in generalized coordinates
2. Do forward simulation using a commercial physics engine (e.g., ODE)

This requires conversion between Cartesian and generalized coordinates.

## Velocity Conversion

### Stacked Jacobian

We concatenate all 2m Jacobians into a single matrix relating generalized to Cartesian velocity:

$$V \equiv \begin{pmatrix} v_1 \\ \vdots \\ v_m \\ \omega_1 \\ \vdots \\ \omega_m \end{pmatrix} = \begin{bmatrix} J_{v1} \\ \vdots \\ J_{vm} \\ J_{\omega 1} \\ \vdots \\ J_{\omega m} \end{bmatrix} \begin{pmatrix} \dot{q}_1 \\ \vdots \\ \dot{q}_m \end{pmatrix} \equiv J \dot{q} \tag{60}$$

### Pseudo-Inverse Solution

The Jacobian $J$ is typically **full column rank** because the maximal representation has
more DOFs than generalized: $6m > n$.

To compute $\dot{q}$ from $V$, we solve an over-constrained system using the pseudo-inverse:

$$\dot{q} = J^+ V \tag{61}$$

where $J^+ = (J^T J)^{-1} J^T$.

If the least-squares solution doesn't exactly solve $J\dot{q} = V$, it indicates that $V$ cannot
be achieved in generalized coordinates without violating constraints.

### Efficient Computation via Relative Velocities

Computing $J^+$ may be expensive for systems with many links. We can simplify by using
**relative velocities** between child and parent links in the parent's local frame.

For angular velocity of link k:

$$\omega_k - \omega_{p(k)} = R_{p(k)}^0 \hat{J}_{\omega k} \dot{q}_k$$

$$\Rightarrow \begin{pmatrix} -I_3 & I_3 \end{pmatrix} \begin{pmatrix} \omega_{p(k)} \\ \omega_k \end{pmatrix} = R_{p(k)}^0 \hat{J}_{\omega k} \dot{q}_k \tag{62}$$

Combining for all links:

$$D \omega = D J_\omega \dot{q} = \text{blockdiag}(\hat{J}_{\omega 1}, \ldots, R_{p(m)}^0 \hat{J}_{\omega m}) \dot{q} = R \hat{J}_\omega \dot{q} \tag{63}$$

where $D$ is a connectivity matrix encoding parent-child relationships:

$$D = \begin{bmatrix} I_3 & 0 & 0 & 0 \\ -I_3 & I_3 & 0 & 0 \\ 0 & -I_3 & I_3 & 0 \\ 0 & -I_3 & 0 & I_3 \end{bmatrix} \tag{64}$$

The matrix $\hat{J}_\omega$ being **block diagonal** is much sparser than $J_\omega$.

### Solution for Rotational DOFs

For systems with only rotational DOFs, we can invert $J_\omega$ (which is full column rank):

$$\dot{q} = J_\omega^+ \omega = \hat{J}_\omega^+ R^T D \omega = \hat{J}_\omega^+ \hat{\omega} = \text{blockdiag}(\hat{J}_{\omega 1}^+, \ldots, \hat{J}_{\omega m}^+) \hat{\omega}$$

or

$$\dot{q}_k = \hat{J}_{\omega k}^+ \hat{\omega}_k, \quad k \in 1 \ldots m \tag{66}$$

This reduces the problem to computing $m$ pseudo-inverses of **small constant-sized matrices**.

For systems with translational DOFs, solve separately:

- Rotational DOFs: use equation (66)
- Translational DOFs: $\dot{q}_k = J_{vk}^+ v_k$ (often $J_{v1} = I_3$ for root)

## Force Conversion

### Cartesian to Generalized

The relation between Cartesian force and generalized force (from virtual work):

$$Q = \sum_k (J'^T_{vk} f_k + J'^T_{\omega k} \tau_k) = \begin{pmatrix} J'^T_v & J'^T_\omega \end{pmatrix} \begin{pmatrix} f \\ \tau \end{pmatrix} \tag{67}$$

where $J'_{vk} = \frac{\partial r'_k}{\partial q}$ with $r'_k$ being the **point of application** of force $f_k$.

> **Note**: Body torque $\tau'_k$ is different from $\tau_k$ in earlier equations:
>
> - $\tau'_k$: torque in world frame, **excluding** torque from linear forces
> - $\tau_k$: includes the torque $[r_k - x_k]f_k$ due to each force

### Joint Torques vs Body Torques

Often controllers compute **joint torques** in the parent's local frame. Joint torque $\hat{\tau}_k$
is defined such that:

- Positive torque $R_{p(k)}^0 \hat{\tau}_k$ is applied to link k
- Negative torque $-R_{p(k)}^0 \hat{\tau}_k$ is applied to parent link p(k)

The relation between body torques and joint torques:

$$\tau' = D^T R \hat{\tau} = (R^T D)^T \hat{\tau} \tag{68}$$

Substituting into the force conversion:

$$Q = \begin{pmatrix} J'^T_v & \hat{J}^T_\omega \end{pmatrix} \begin{pmatrix} f \\ \hat{\tau} \end{pmatrix} \tag{69}$$

### Generalized to Cartesian

To convert generalized forces $Q$ to Cartesian forces:

**General case** (unknown points of application):

$$\begin{pmatrix} f \\ \tau \end{pmatrix} = \begin{pmatrix} J_v^T & J_\omega^T \end{pmatrix}^+ Q \tag{70}$$

**No linear forces** (joint torques only):

$$\hat{\tau} = (\hat{J}^T_\omega)^+ Q$$

or

$$\hat{\tau}_k = (\hat{J}^T_{\omega k})^+ Q_k, \quad \forall k \in 1 \ldots m \tag{71}$$

Note: $\hat{J}^T_{\omega k}$ is $n(k) \times 3$ with $n(k) \leq 3$, giving a least-squares solution for $\hat{\tau}_k$.

## DART Implementation

DART provides utilities for coordinate conversions:

```cpp
// Get full Jacobian for entire skeleton
// This is the stacked Jacobian J relating q̇ to all body velocities

// Per-body Jacobians
Eigen::MatrixXd J_world = bodyNode->getWorldJacobian();
Eigen::MatrixXd J_linear = bodyNode->getLinearJacobian();
Eigen::MatrixXd J_angular = bodyNode->getAngularJacobian();

// Jacobian at a specific point
Eigen::Vector3d offset(0.1, 0, 0);  // offset from body origin
Eigen::MatrixXd J_point = bodyNode->getWorldJacobian(offset);

// Convert Cartesian wrench to generalized forces
Eigen::Vector6d wrench;  // [torque; force]
wrench << tau_x, tau_y, tau_z, f_x, f_y, f_z;
Eigen::VectorXd Q = bodyNode->getWorldJacobian().transpose() * wrench;
```

For joint-space conversions:

```cpp
// Get joint forces from generalized forces
for (auto* joint : skeleton->getJoints()) {
    Eigen::VectorXd joint_forces = joint->getForces();
}
```

---

**Navigation**: [← Articulated Dynamics](05_articulated-dynamics.md) | [Index](../README.md) | [Recursive Inverse Dynamics →](07_recursive-inverse-dynamics.md)
