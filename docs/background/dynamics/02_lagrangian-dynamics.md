# Lagrangian Dynamics

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [← Introduction](01_introduction.md) | [Index](../README.md) | [Newton-Euler Review →](03_newton-euler-review.md)

---

## Overview

Articulated human motions can be described by a set of dynamic equations of motion of
multibody systems. Since the direct application of Newton's second law becomes difficult
when a complex articulated rigid body system is considered, we use **Lagrange's equations**
derived from **D'Alembert's principle** to describe the dynamics of motion.

## Particle System Representation

To simplify the math, let's temporarily imagine that the entire human skeleton consists
of a collection of particles $\{r_1, r_2, \ldots, r_{n_p}\}$. Each particle $r_i$ is defined by Cartesian
coordinates that describe the translation with respect to world coordinates.

We can represent $r_i$ by a set of generalized coordinates that indicate the joint
configuration of the human skeleton:

$$r_i = r_i(q_1, q_2, \ldots, q_{n_j}, t) \tag{1}$$

where $t$ is the time and $q_j$ is a joint degree of freedom (DOF) in the skeleton.
Each $q_j$ is a function of time but we assume that $r_i$ is not an explicit function of time.

## Virtual Displacement

The **virtual displacement** $\delta r_i$ refers to an infinitesimal change in the system coordinates
such that the constraint remains satisfied. In the context of human skeleton:

- The system coordinates are the generalized coordinates $q_j$
- The constraint manifold lies in the Cartesian space

The virtual displacement $\delta r_i$ is a tangent vector to the constraint manifold at a fixed time:

$$\delta r_i = \sum_j \frac{\partial r_i}{\partial q_j} \delta q_j \tag{2}$$

## Virtual Work and Generalized Forces

We can write the virtual work done by a force $f_i$ acting on particle $r_i$ as:

$$f_i \cdot \delta r_i = f_i \cdot \sum_j \frac{\partial r_i}{\partial q_j} \delta q_j \equiv \sum_j Q_{ij} \delta q_j = Q_i \cdot \delta q \tag{3}$$

where $Q_{ij} = \left(\frac{\partial r_i}{\partial q_j}\right)^T f_i$ is the component of the **generalized force** associated
with coordinate $q_j$.

In vector form, $Q_i$ is the generalized force corresponding to the Cartesian force $f_i$
with the relation:

$$Q_i = J_i^T f_i$$

where $J_i$ is the **Jacobian matrix** with the jth column defined as $\frac{\partial r_i}{\partial q_j}$.

## D'Alembert's Principle

From D'Alembert's principle, we know that the sum of the differences between the forces
acting on a system and the inertial force of the system along any virtual displacement
consistent with the constraints of the system, is zero.

The virtual work at $r_i$ can be written as:

$$\delta W_i = f_i \cdot \delta r_i = \mu_i \ddot{r}_i \cdot \delta r_i = \sum_j \mu_i \ddot{r}_i \cdot \frac{\partial r_i}{\partial q_j} \delta q_j \tag{4}$$

where $\mu_i$ is the infinitesimal mass associated with $r_i$.

## Deriving the Lagrangian Equation

The component of inertial force associated with $q_j$ can be written as:

$$\mu_i \ddot{r}_i \cdot \frac{\partial r_i}{\partial q_j} = \frac{d}{dt}\left(\mu_i \dot{r}_i \cdot \frac{\partial r_i}{\partial q_j}\right) - \mu_i \dot{r}_i \cdot \frac{d}{dt}\left(\frac{\partial r_i}{\partial q_j}\right) \tag{5}$$

Now let us consider the velocity of $r_i$ in terms of joint velocity $\dot{q}_j$:

$$\dot{r}_i = \sum_j \frac{\partial r_i}{\partial q_j} \dot{q}_j \tag{6}$$

from which we derive two identities:

$$\frac{\partial \dot{r}_i}{\partial \dot{q}_j} = \frac{\partial r_i}{\partial q_j} \tag{7}$$

$$\frac{\partial \dot{r}_i}{\partial q_j} = \sum_k \frac{\partial^2 r_i}{\partial q_j \partial q_k} \dot{q}_k = \frac{d}{dt}\left(\frac{\partial r_i}{\partial q_j}\right) \tag{8}$$

Using these identities, we rewrite the inertial force in terms of **kinetic energy**:

$$T_i = \frac{1}{2} \mu_i \dot{r}_i^T \dot{r}_i \tag{10}$$

This gives us:

$$\mu_i \ddot{r}_i \cdot \frac{\partial r_i}{\partial q_j} = \frac{d}{dt}\left(\frac{\partial T_i}{\partial \dot{q}_j}\right) - \frac{\partial T_i}{\partial q_j} \tag{11}$$

## The Lagrangian Equation

Combining the definition of generalized force, D'Alembert's principle, and the generalized
inertial force, we arrive at:

$$\left[\frac{d}{dt}\left(\frac{\partial T_i}{\partial \dot{q}_j}\right) - \frac{\partial T_i}{\partial q_j}\right] \delta q_j = Q_{ij} \delta q_j \tag{12}$$

If the set of generalized coordinates $q_j$ is **linearly independent**, this leads to
the **Lagrangian equation**:

$$\frac{d}{dt}\left(\frac{\partial T_i}{\partial \dot{q}_j}\right) - \frac{\partial T_i}{\partial q_j} - Q_{ij} = 0 \tag{13}$$

## Equations of Motion in Vector Form

Equation (13) is the equation of motion for one generalized coordinate in a multibody system.
We can combine $n_j$ scalar equations into the familiar vector form:

$$M(q)\ddot{q} + C(q,\dot{q}) = Q \tag{14}$$

where:

- $M(q)$ is the **mass matrix**
- $C(q,\dot{q})$ is the **Coriolis and centrifugal term**
- $Q$ is the vector of **generalized forces** for all DOFs

**Key properties:**

- $M$ only depends on $q$
- $C$ depends quadratically on $\dot{q}$

## Deriving M and C from Kinetic Energy

Using the velocity relation $\dot{r}_i = J_i(q)\dot{q}$, the kinetic energy of the system is:

$$T = \sum_i T_i = \sum_i \frac{1}{2} \mu_i \dot{r}_i^T \dot{r}_i = \frac{1}{2} \dot{q}^T \left(\sum_i \mu_i J_i^T J_i\right) \dot{q} = \frac{1}{2} \dot{q}^T M(q) \dot{q} \tag{16}$$

where we define the **mass matrix**: $M(q) = \sum_i \mu_i J_i^T J_i$

From the kinetic energy, we can derive the Lagrangian terms:

$$\frac{d}{dt}\left(\frac{\partial T}{\partial \dot{q}}\right) - \frac{\partial T}{\partial q} = M \ddot{q} + \dot{M} \dot{q} - \frac{1}{2} \dot{q}^T \left(\frac{\partial M}{\partial q}\right)^T \dot{q} \equiv M \ddot{q} + C(q,\dot{q}) \tag{17}$$

This confirms that the mass matrix is identical in both equations, and the Coriolis term is:

$$C = \dot{M} \dot{q} - \frac{1}{2} \left(\frac{\partial M}{\partial q}\right)^T \dot{q} \dot{q}$$

## Forward and Inverse Dynamics

Once we know how to compute the mass matrix, Coriolis term, and generalized forces:

- **Forward dynamics**: Compute acceleration $\ddot{q}$ from forces (simulation)
- **Inverse dynamics**: Compute forces from acceleration $\ddot{q}$ (control)

## DART Implementation

```cpp
// Get dynamics quantities
Eigen::MatrixXd M = skeleton->getMassMatrix();
Eigen::VectorXd C = skeleton->getCoriolisForces();
Eigen::VectorXd g = skeleton->getGravityForces();

// Forward dynamics: solve M*qddot = tau - C - g
skeleton->setForces(tau);
skeleton->computeForwardDynamics();
Eigen::VectorXd qddot = skeleton->getAccelerations();

// Inverse dynamics: compute tau from qddot
skeleton->setAccelerations(qddot);
skeleton->computeInverseDynamics();
Eigen::VectorXd tau = skeleton->getForces();
```

---

**Navigation**: [← Introduction](01_introduction.md) | [Index](../README.md) | [Newton-Euler Review →](03_newton-euler-review.md)
