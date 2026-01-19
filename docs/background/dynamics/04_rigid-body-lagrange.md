# Rigid Body Dynamics: Lagrange's Equations

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Newton-Euler Review](03_newton-euler-review.md) | [Index](../README.md) | [Articulated Dynamics →](05_articulated-dynamics.md)

---

## Overview

The Newton-Euler equations are defined in terms of velocities instead of position and
orientation. We now derive the equations in generalized coordinates $q$ that define the
position and orientation:

- First 3 coordinates: position of COM
- Next 3 (or 4) coordinates: rotation (exponential map, Euler angles, or quaternion)

In particular, we will show how the **mass matrix** and **Coriolis term** are computed.

## Kinetic Energy of a Rigid Body

Starting from the kinetic energy using the particle notation:

$$
\begin{aligned}
T &= \sum_i T_i = \sum_i \frac{1}{2} \mu_i \dot{r}_i^T \dot{r}_i = \sum_i \frac{1}{2} \mu_i (v + \omega \times r'_i)^T (v + \omega \times r'_i) \\
  &= \sum_i \frac{1}{2} \mu_i (v^T v + v^T[\omega]r'_i + r'^T_i[\omega]^T v + r'^T_i[\omega]^T[\omega]r'_i)
\end{aligned} \tag{25}
$$

Because $\sum_i \mu_i r'_i = 0$, the cross terms vanish. Using $[\omega]r'_i = -[r'_i]\omega$:

$$T = \frac{1}{2} m v^T v + \frac{1}{2} \omega^T \left(\sum_i -\mu_i[r'_i][r'_i]\right) \omega = \frac{1}{2} m v^T v + \frac{1}{2} \omega^T I_c \omega \tag{26}$$

In vector form:

$$T = \frac{1}{2} \begin{pmatrix} v^T & \omega^T \end{pmatrix} \begin{bmatrix} m I_3 & 0 \\ 0 & I_c \end{bmatrix} \begin{pmatrix} v \\ \omega \end{pmatrix} \equiv \frac{1}{2} V^T M_c V \tag{27}$$

where $V = (v^T, \omega^T)^T$ and $M_c = \text{blockdiag}(m I_3, I_c)$.

## Relating Cartesian to Generalized Velocities

Let $x(q)$ and $R(q)$ represent the position of COM and rotation matrix. The linear
velocity is:

$$v = \dot{x}(q) = \frac{\partial x}{\partial q} \dot{q} \equiv J_v \dot{q} \tag{28}$$

The angular velocity is computed from the rotation matrix:

$$[\omega] = \dot{R}(q) R^T(q) = \sum_j \frac{\partial R}{\partial q_j} R^T \dot{q}_j \equiv \sum_j [j_j] \dot{q}_j \tag{29}$$

where $\frac{\partial R}{\partial q_j} R^T$ is always a skew-symmetric matrix represented as $[j_j]$.

In vector form:

$$\omega = J_\omega \dot{q} \tag{30}$$

where $j_j$ is the jth column of matrix $J_\omega$.

Combining these:

$$V = \begin{bmatrix} J_v \\ J_\omega \end{bmatrix} \dot{q} \equiv J(q) \dot{q} \tag{31}$$

## Kinetic Energy in Generalized Coordinates

Substituting into the kinetic energy:

$$T = \frac{1}{2} \dot{q}^T J^T M_c J \dot{q} \tag{32}$$

## Deriving Lagrange's Equations

Computing the partial derivatives:

$$\frac{\partial T}{\partial \dot{q}_j} = (J)_j^T M_c J \dot{q} \tag{33}$$

$$\frac{d}{dt}\left(\frac{\partial T}{\partial \dot{q}_j}\right) = (J)_j^T M_c J \ddot{q} + (J)_j^T M_c \dot{J} \dot{q} + (J)_j^T \dot{M}_c J \dot{q} + (\dot{J})_j^T M_c J \dot{q} \tag{34}$$

$$\frac{\partial T}{\partial q_j} = \dot{q}^T \left(\frac{\partial J}{\partial q_j}\right)^T M_c J \dot{q} + \frac{1}{2} \dot{q}^T J^T \frac{\partial M_c}{\partial q_j} J \dot{q} \tag{35}$$

The Lagrangian becomes:

$$\frac{d}{dt}\left(\frac{\partial T}{\partial \dot{q}_j}\right) - \frac{\partial T}{\partial q_j} = (J)_j^T M_c J \ddot{q} + [\text{remaining terms}] \tag{36}$$

After simplification (see original PDF for detailed derivation), terms 3, 4, and 5 reduce significantly:

- **Term 3**: $j_j^T [\omega] I_c \omega$
- **Term 4**: $-j_j^T [\omega] I_c \omega$
- **Term 5 (linear)**: $0$
- **Term 5 (angular)**: $-j_j^T [\omega] I_c \omega$

## Final Result

The equations of motion in vector form:

$$\frac{d}{dt}\left(\frac{\partial T}{\partial \dot{q}}\right) - \frac{\partial T}{\partial q} = J^T M_c J \ddot{q} + (J^T M_c \dot{J} + J^T [\tilde{\omega}] M_c J) \dot{q} \tag{45}$$

where:

$$[\tilde{\omega}] = \begin{bmatrix} 0 & 0 \\ 0 & [J_\omega \dot{q}] \end{bmatrix}$$

## Mass Matrix and Coriolis Term

Comparing to $M(q)\ddot{q} + C(q,\dot{q}) = Q$:

$$M(q) = J^T M_c J \tag{48}$$

$$C(q,\dot{q}) = (J^T M_c \dot{J} + J^T [\tilde{\omega}] M_c J) \dot{q}$$

$$Q = J_v^T f + J_\omega^T \tau$$

## Alternative Derivation via Newton-Euler

We can alternatively derive the same result from the Newton-Euler equations. Substituting
the Cartesian velocities in terms of generalized velocities into the Newton-Euler equations
and pre-multiplying by $J^T$ (principle of virtual work):

$$J^T M_c J \ddot{q} + (J^T M_c \dot{J} + J^T [\tilde{\omega}] M_c J) \dot{q} = J_v^T f + J_\omega^T \tau \tag{47}$$

This confirms the Lagrangian derivation.

## Computing J and $\dot{J}$

The second term involves $\dot{J}$ which can be computed as:

$$\dot{J} = \sum_k \frac{\partial J}{\partial q_k} \dot{q}_k$$

This requires computing first and second derivatives of the rotation matrix ($\frac{\partial R}{\partial q_j}$ and
$\frac{\partial^2 R}{\partial q_i \partial q_j}$).

## DART Implementation

DART computes these quantities efficiently using the Articulated Body Algorithm:

```cpp
// Mass matrix M(q)
Eigen::MatrixXd M = skeleton->getMassMatrix();

// Coriolis + gravity: C(q,qdot) + g(q)
Eigen::VectorXd Cg = skeleton->getCoriolisAndGravityForces();

// Separate Coriolis only
Eigen::VectorXd C = skeleton->getCoriolisForces();
```

---

**Navigation**: [← Newton-Euler Review](03_newton-euler-review.md) | [Index](../README.md) | [Articulated Dynamics →](05_articulated-dynamics.md)
