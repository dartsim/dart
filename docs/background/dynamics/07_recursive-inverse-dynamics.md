# Recursive Inverse Dynamics

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Coordinate Conversion](06_coordinate-conversion.md) | [Index](../README.md) | [Notation Glossary](notation-glossary.md)

---

## Overview

As Featherstone pointed out, inverse dynamics can be computed efficiently by exploiting
the **recursive structure** of an articulated rigid body system.

A recursive algorithm allows computation of inverse dynamics in **linear time O(n)**
proportional to the number of links.

## Local Frame Notation

For each body link k:

| Symbol                              | Description                                        |
| ----------------------------------- | -------------------------------------------------- |
| $c_k$                               | Center of mass in local frame                      |
| $d_{\tilde{i}}$                     | Vector from joint to parent to joint to i-th child |
| $f_k$                               | Force received from parent link                    |
| $\tau_k$                            | Torque received from parent link                   |
| $-f_{\tilde{i}}, -\tau_{\tilde{i}}$ | Force/torque from i-th child                       |
| $c(k)$                              | Returns indices of child links of link k           |

Superscript $\ell$ denotes a vector expressed in the local frame of link k: $a_k^\ell$

## Newton-Euler Equations in Local Frame

For link k:

**Linear force equation:**

$$m_k (\dot{v}_k)^\ell = f_k^\ell - \sum_{\tilde{i} \in c(k)} R_{\tilde{i}} f_{\tilde{i}}^\ell \tag{72}$$

**Torque equation:**

$$I_{ck} (\dot{\omega}_k)^\ell + \omega_k^\ell \times I_{ck} \omega_k^\ell = \tau_k^\ell - c_k \times f_k^\ell - \sum_{\tilde{i} \in c(k)} (R_{\tilde{i}} \tau_{\tilde{i}}^\ell + (d_{\tilde{i}} - c_k) \times (R_{\tilde{i}} f_{\tilde{i}}^\ell)) \tag{73}$$

## Two-Pass Algorithm

The algorithm visits each link twice:

1. **Pass 1** (root → leaves): Compute velocity and acceleration
2. **Pass 2** (leaves → root): Compute forces and torques

### Pass 1: Compute Velocity and Acceleration

The algorithm is recursive because computation at each link depends on its parent.

**Angular velocity:**

$$\omega_k^\ell = R_k^T (\omega_{p(k)}^\ell + \hat{\omega}_k) \tag{78}$$

**Linear velocity:**

$$v_k^\ell = R_k^T (v_{p(k)}^\ell + \omega_{p(k)}^\ell \times (d_k - c_{p(k)})) + \omega_k^\ell \times c_k \tag{76}$$

**Angular acceleration:**

$$(\dot{\omega}_k)^\ell = R_k^T ((\dot{\omega}_{p(k)})^\ell + \dot{\hat{\omega}}_k + \omega_{p(k)}^\ell \times \hat{\omega}_k) \tag{80}$$

**Linear acceleration:**

The linear acceleration must be computed in the world frame first, then transformed:

$$(\dot{v}_k)^\ell = R_k^T ((\dot{v}_{p(k)})^\ell + (\dot{\omega}_{p(k)})^\ell \times (d_k - c_{p(k)}) + \omega_{p(k)}^\ell \times (\omega_{p(k)}^\ell \times (d_k - c_{p(k)}))) + \omega_k^\ell \times (\omega_k^\ell \times c_k) + (\dot{\omega}_k)^\ell \times c_k \tag{79}$$

> **Note**: $(\dot{v}_k)^\ell$ (true linear acceleration in local frame) is different from $\dot{v}_k^\ell$
> (time derivative of local velocity) due to Coriolis effects from the moving frame.

### Base Case (Root Link)

For the root link (k=0):

$$v_0^\ell = \omega_0^\ell \times c_0$$
$$\omega_0^\ell = R_0^T \hat{\omega}_0$$
$$(\dot{v}_0)^\ell = \omega_0^\ell \times (\omega_0^\ell \times c_0) + (\dot{\omega}_0)^\ell \times c_0$$
$$(\dot{\omega}_0)^\ell = R_0^T \dot{\hat{\omega}}_0 \tag{81}$$

### Translational DOFs

For joints with translational DOFs (where $R_k = I_3$ and $d_k = q_k$):

$$v_k^\ell = v_{p(k)}^\ell + \omega_{p(k)}^\ell \times (c_k + q_k - c_{p(k)}) + \dot{q}_k \tag{83}$$
$$\omega_k^\ell = \omega_{p(k)}^\ell$$
$$\dot{v}_k^\ell = \dot{v}_{p(k)}^\ell + \dot{\omega}_{p(k)}^\ell \times (c_k + q_k - c_{p(k)}) + \omega_{p(k)}^\ell \times \dot{q}_k + \ddot{q}_k$$
$$\dot{\omega}_k^\ell = \dot{\omega}_{p(k)}^\ell \tag{84}$$

### Pass 2: Compute Force and Torque

Visit each link from leaves to root, computing $f_k^\ell$ and $\tau_k^\ell$ using the Newton-Euler
equations (72) and (73), given:

- Velocities and accelerations from Pass 1
- Forces and torques from child links

**Base case**: Leaf nodes have no children, so $f_{\tilde{i}} = \tau_{\tilde{i}} = 0$.

**Root link**: $f_0 = \tau_0 = 0$ (no parent).

## Handling Gravity

Instead of treating gravity as an external force, we can offset the linear acceleration
of the root link by $-g$:

$$(\dot{v}_0)^\ell = \omega_0^\ell \times (\omega_0^\ell \times c_0) + (\dot{\omega}_0)^\ell \times c_0 - R_0^T g \tag{85}$$

This is equivalent to adding a fictitious force $-m_k g$ to each link. The rest of the
algorithm remains unchanged.

## Algorithm Summary

**RECURSIVE INVERSE DYNAMICS (RNEA)**

**Input**: $q, \dot{q}, \ddot{q}$ (positions, velocities, accelerations)

**Output**: $\tau$ (joint torques)

**Pass 1**: Forward recursion (root → leaves)

For $k = 1$ to $m$:

- Compute $v_k^\ell, \omega_k^\ell$ from parent values
- Compute $(\dot{v}_k)^\ell, (\dot{\omega}_k)^\ell$ from parent values and $\ddot{q}_k$

**Pass 2**: Backward recursion (leaves → root)

For $k = m$ down to $1$:

- Compute $f_k^\ell, \tau_k^\ell$ from Newton-Euler equations
- Extract $\tau_k$ = projection of $\tau_k^\ell$ onto joint axes

**Complexity**: $O(n)$ where $n$ is the number of links

## DART Implementation

DART implements RNEA (Recursive Newton-Euler Algorithm):

```cpp
// Set desired accelerations
skeleton->setAccelerations(qddot);

// Compute inverse dynamics
// This runs RNEA internally
skeleton->computeInverseDynamics(
    true,   // withExternalForces
    true,   // withDampingForces
    true    // withSpringForces
);

// Get computed joint forces
Eigen::VectorXd tau = skeleton->getForces();
```

For forward dynamics, DART uses the **Articulated Body Algorithm (ABA)**, also O(n):

```cpp
// Set joint forces
skeleton->setForces(tau);

// Compute forward dynamics (ABA)
skeleton->computeForwardDynamics();

// Get computed accelerations
Eigen::VectorXd qddot = skeleton->getAccelerations();
```

See `dart/dynamics/Skeleton.cpp` for implementation details.

## Acknowledgements

The original authors thank Jeff Bingham, Stelian Coros, Marco da Silva, and Yuting Ye
for proofreading the original document and providing valuable comments.

---

**Navigation**: [← Coordinate Conversion](06_coordinate-conversion.md) | [Index](../README.md) | [Notation Glossary](notation-glossary.md)
