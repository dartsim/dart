# Articulated Rigid Body Dynamics

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Rigid Body Lagrange](04_rigid-body-lagrange.md) | [Index](../README.md) | [Coordinate Conversion →](06_coordinate-conversion.md)

---

## Overview

We now derive the equations of motion for an **articulated rigid body system**, following
the same recipe as rigid body dynamics in generalized coordinates.

An articulated rigid body system is represented as a set of rigid bodies connected through
joints in a **tree structure**:

- Every rigid link has exactly one parent joint
- The root joint is special; the root link does not link to any other rigid link
- Generalized coordinates are the DOFs of the root link plus all joint angles

## Definitions

The state of an articulated rigid body system can be expressed as $(x_k, R_k, v_k, \omega_k)$ for
$k = 1, \ldots, m$ where $m$ is the number of rigid links.

| Symbol     | Description                              |
| ---------- | ---------------------------------------- |
| $x_k$      | Position of COM of link k                |
| $R_k$      | Orientation of link k                    |
| $v_k$      | Linear velocity of link k (world frame)  |
| $\omega_k$ | Angular velocity of link k (world frame) |
| $f_k$      | Cartesian force on link k (world frame)  |
| $\tau_k$   | Cartesian torque on link k (world frame) |

In generalized coordinates, the state is $(q, \dot{q})$ where $q = (q_1, \ldots, q_k, \ldots, q_m)$ and
each $q_k$ is the set of DOFs of the joint connecting link k to its parent.

### Notation

| Symbol   | Description                             | Example                  |
| -------- | --------------------------------------- | ------------------------ |
| $p(k)$   | Index of parent link of link k          | p(4) = 2                 |
| $p(1,k)$ | Indices of all links from root to k     | p(1,4) = {1, 2, 4}       |
| $n(k)$   | Number of DOFs in joint k               | n(2) = 3 for ball joint  |
| $n$      | Total DOFs in system                    | $n = \sum_k n(k)$        |
| $R_k$    | Local rotation matrix for link k        | Depends only on $q_k$    |
| $R_k^0$  | Chain of rotations from world to link k | $R_k^0 = R_{p(k)}^0 R_k$ |

## Cartesian and Generalized Velocities

### Angular Velocity

The angular velocity (in skew-symmetric form) of link k in world frame:

$$[\omega_k] = \dot{R}_k^0 (R_k^0)^T = (\dot{R}_{p(k)}^0 R_k + R_{p(k)}^0 \dot{R}_k) R_k^T (R_{p(k)}^0)^T \equiv [\omega_{p(k)}] + R_{p(k)}^0 [\hat{\omega}_k] (R_{p(k)}^0)^T \tag{49}$$

where $[\hat{\omega}_k] = \dot{R}_k R_k^T$ is the angular velocity of link k in its parent's frame.

We can write $\hat{\omega}_k = \hat{J}_{\omega k} \dot{q}_k$ where $\hat{J}_{\omega k}$ is the **local Jacobian** relating joint velocity
to angular velocity in parent frame. Dimension: $3 \times n(k)$.

In vector form:

$$\omega_k = \omega_{p(k)} + R_{p(k)}^0 \hat{J}_{\omega k} \dot{q}_k = \sum_{l \in p(1,k)} R_{p(l)}^0 \hat{J}_{\omega l} \dot{q}_l \equiv J_{\omega k} \dot{q} \tag{50}$$

where the Jacobian $J_{\omega k}$ is:

$$J_{\omega k} = [\hat{J}_{\omega 1} \cdots R_{p(l)}^0 \hat{J}_{\omega l} \cdots 0 \cdots] \tag{51}$$

Note: Zero matrices correspond to joint DOFs not in the chain from root to link k.

### Example: Jacobian Structure

For an articulated system with 4 links:

$$\omega_1 = (\hat{J}_{\omega 1} \quad 0 \quad 0 \quad 0) \dot{q}$$
$$\omega_4 = (\hat{J}_{\omega 1} \quad R_1^0\hat{J}_{\omega 2} \quad 0 \quad R_2^0\hat{J}_{\omega 4}) \dot{q}$$

### Joint Representations

For Euler angles $R_2(q_2) = R^{(x)}(q_{21}) R^{(y)}(q_{22}) R^{(z)}(q_{23})$:

$$\hat{J}_{\omega 2} = \begin{bmatrix} \begin{pmatrix}1\\0\\0\end{pmatrix} & R^{(x)}\begin{pmatrix}0\\1\\0\end{pmatrix} & R^{(x)}R^{(y)}\begin{pmatrix}0\\0\\1\end{pmatrix} \end{bmatrix} \tag{52}$$

For exponential map $q_k = (q_{k1}, q_{k2}, q_{k3})$ with $\theta = \|q_k\|$:

$$R_k(q_k) = e^{[q_k]} = I_3 + \frac{\sin\theta}{\theta} [q_k] + \frac{1 - \cos\theta}{\theta^2} [q_k]^2 \tag{53}$$

$$\hat{J}_{\omega k} = I_3 + \frac{1 - \cos\theta}{\theta^2} [q_k] + \frac{\theta - \sin\theta}{\theta^3} [q_k]^2 \tag{54}$$

For small $\theta \to 0$:

$$R_k \approx I_3 + [q_k] + \frac{1}{2} [q_k]^2 \tag{55}$$
$$\hat{J}_{\omega k} \approx I_3 + \frac{1}{2} [q_k] + \frac{1}{6} [q_k]^2 \tag{56}$$

### Linear Velocity

The linear velocity of the COM of link k:

$$v_k = J_{vk} \dot{q}, \quad \text{where } J_{vk} = \frac{\partial x_k}{\partial q} = \frac{\partial (W_k^0 c_k)}{\partial q} \tag{57}$$

where $W_k^0$ is the chain of homogeneous transformations and $c_k$ is the COM in local frame.

### Combined Velocity Relation

$$V_k = J_k \dot{q} \quad \text{where } V_k = \begin{pmatrix} v_k \\ \omega_k \end{pmatrix} \text{ and } J_k = \begin{bmatrix} J_{vk} \\ J_{\omega k} \end{bmatrix} \tag{58}$$

## Equations of Motion

The kinetic energy of the entire system is $T = \sum_k T_k$. Therefore:

$$
\begin{aligned}
\frac{d}{dt}\left(\frac{\partial T}{\partial \dot{q}}\right) - \frac{\partial T}{\partial q} &= \sum_k \left[\frac{d}{dt}\left(\frac{\partial T_k}{\partial \dot{q}}\right) - \frac{\partial T_k}{\partial q}\right] \\
&= \sum_k \left[J_k^T M_{ck} J_k \ddot{q} + (J_k^T M_{ck} \dot{J}_k + J_k^T [\tilde{\omega}_k] M_{ck} J_k) \dot{q}\right] \\
&= \left(\sum_k J_k^T M_{ck} J_k\right) \ddot{q} + \sum_k (J_k^T M_{ck} \dot{J}_k + J_k^T [\tilde{\omega}_k] M_{ck} J_k) \dot{q}
\end{aligned} \tag{59}
$$

This gives us:

$$M(q) = \sum_k J_k^T M_{ck} J_k$$

$$C(q,\dot{q}) = \sum_k (J_k^T M_{ck} \dot{J}_k + J_k^T [\tilde{\omega}_k] M_{ck} J_k) \dot{q}$$

## DART Implementation

DART uses the **Articulated Body Algorithm (ABA)** for efficient O(n) computation:

```cpp
// Full articulated body dynamics
Eigen::MatrixXd M = skeleton->getMassMatrix();        // O(n²) storage
Eigen::VectorXd C = skeleton->getCoriolisForces();    // O(n)
Eigen::VectorXd g = skeleton->getGravityForces();     // O(n)

// Forward dynamics via ABA: O(n) complexity
skeleton->computeForwardDynamics();

// Inverse dynamics via RNEA: O(n) complexity
skeleton->computeInverseDynamics();

// Access per-link Jacobians
for (auto* bodyNode : skeleton->getBodyNodes()) {
    Eigen::MatrixXd J = bodyNode->getJacobian();      // 6 × n
    Eigen::MatrixXd Jv = bodyNode->getLinearJacobian();
    Eigen::MatrixXd Jw = bodyNode->getAngularJacobian();
}
```

See `dart/dynamics/Skeleton.hpp` and `dart/dynamics/BodyNode.hpp`.

---

**Navigation**: [← Rigid Body Lagrange](04_rigid-body-lagrange.md) | [Index](../README.md) | [Coordinate Conversion →](06_coordinate-conversion.md)
