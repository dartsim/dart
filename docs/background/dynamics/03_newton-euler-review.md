# Review: Newton-Euler Equations

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Lagrangian Dynamics](02_lagrangian-dynamics.md) | [Index](../README.md) | [Rigid Body Lagrange →](04_rigid-body-lagrange.md)

---

## Overview

This section reviews Newton-Euler equations for rigid body dynamics. The derivation of
mass matrix $M(q)$ and Coriolis term $C(q,\dot{q})$ for a rigid body will be presented in the
next section.

If you are familiar with Newton-Euler equations and the math notations used in Witkin and
Baraff's course notes (linear momentum, angular momentum, skew-symmetric matrices), you
can safely skip this section.

## Rigid Body State

A rigid body has the following properties:

| Symbol | Description                      | DART API                           |
| ------ | -------------------------------- | ---------------------------------- |
| `m`    | Mass                             | `BodyNode::getMass()`              |
| `x`    | Position of center of mass (COM) | `BodyNode::getCOM()`               |
| `R`    | Orientation (rotation matrix)    | `BodyNode::getTransform()`         |
| `v`    | Linear velocity of COM           | `BodyNode::getCOMLinearVelocity()` |
| `ω`    | Angular velocity                 | `BodyNode::getAngularVelocity()`   |

## Linear Momentum

The linear momentum $P$ is computed as:

$$P = \sum_i P_i = \sum_i \mu_i \dot{r}_i = \sum_i \mu_i(v + \omega \times r'_i) = m v \tag{18}$$

where $r'_i = r_i - x$. Because $\sum_i \mu_i r'_i = 0$ (property of the COM), the second term vanishes.

## Angular Momentum

The angular momentum $L$ about the COM is computed as:

$$
\begin{aligned}
L &= \sum_i L_i = \sum_i r'_i \times P_i \\
  &= \sum_i \mu_i r'_i \times (v + \omega \times r'_i) \\
  &= 0 + \sum_i \mu_i [r'_i][\omega]r'_i \\
  &= \left(\sum_i -\mu_i [r'_i][r'_i]\right) \omega
\end{aligned} \tag{19}
$$

## Skew-Symmetric Matrix Notation

The notation $[a]$ denotes the cross product $a \times b$ with $[a]$ being the skew-symmetric
matrix corresponding to the vector $a$:

$$[a] = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix} \tag{20}$$

**Key identities:**

- $[a]b = -[b]a$
- $[a]^T = -[a]$
- $-[a][a] = (a^T a)I_3 - aa^T$
- $[a \times b] = [a][b] - [b][a]$
- $[R\omega] = R[\omega]R^T$

## Inertia Tensor

Recall the inertia tensor about the COM defined in Witkin and Baraff's course notes:

$$I_c = \sum_i \mu_i((r'^T_i r'_i)I_3 - r'_i r'^T_i)$$

Using the identity $-[a][a] = (a^T a)I_3 - aa^T$, we can show that:

$$I_c = \sum_i -\mu_i [r'_i][r'_i]$$

Therefore, the angular momentum of a rigid body is:

$$L = I_c \omega \tag{21}$$

where the inertia tensor can be written as $I_c = R I_0 R^T$. Here $R$ is the rotation matrix
and $I_0$ is the constant inertia tensor defined at zero rotation.

The angular velocity in skew-symmetric form is related to the rotation matrix as:

$$[\omega] = \dot{R} R^T$$

## Equations of Motion

### Linear Force Equation

The equation corresponding to linear force:

$$f = \dot{P} = m \dot{v} \tag{22}$$

### Torque Equation

The equation corresponding to torque:

$$
\begin{aligned}
\tau &= \dot{L} = (I_c \omega)\dot{} \\
  &= I_c \dot{\omega} + (R I_0 R^T)\dot{} \omega \\
  &= I_c \dot{\omega} + \dot{R} I_0 R^T \omega + R I_0 \dot{R}^T \omega \\
  &= I_c \dot{\omega} + \dot{R} R^T I_c \omega + I_c (\dot{R} R^T)^T \omega \\
  &= I_c \dot{\omega} + [\omega] I_c \omega - I_c [\omega] \omega \quad \text{(Using } [\omega]^T = -[\omega]\text{)} \\
  &= I_c \dot{\omega} + \omega \times I_c \omega
\end{aligned} \tag{23}
$$

## Newton-Euler Equations (Combined Form)

Combining the linear and angular equations:

$$\begin{bmatrix} m I_3 & 0 \\ 0 & I_c \end{bmatrix} \begin{bmatrix} \dot{v} \\ \dot{\omega} \end{bmatrix} + \begin{bmatrix} 0 \\ \omega \times I_c \omega \end{bmatrix} = \begin{bmatrix} f \\ \tau \end{bmatrix} \tag{24}$$

Or in compact form:

$$M_c \dot{V} + C = F$$

where:

- $M_c = \text{blockdiag}(m I_3, I_c)$ is the **spatial inertia matrix**
- $V = [v; \omega]$ is the **spatial velocity**
- $C = [0; \omega \times I_c \omega]$ is the **bias force** (Coriolis/centrifugal)
- $F = [f; \tau]$ is the **spatial force**

## DART Implementation

In DART, the `BodyNode` class computes these quantities:

```cpp
// Get spatial inertia
Eigen::Matrix6d I = bodyNode->getSpatialInertia();

// Get body velocity (twist)
Eigen::Vector6d V = bodyNode->getSpatialVelocity();

// Get body acceleration
Eigen::Vector6d Vdot = bodyNode->getSpatialAcceleration();
```

---

**Navigation**: [← Lagrangian Dynamics](02_lagrangian-dynamics.md) | [Index](../README.md) | [Rigid Body Lagrange →](04_rigid-body-lagrange.md)
