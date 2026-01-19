# Introduction

> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at [`docs/dynamics.pdf`](../../dynamics.pdf).

**Navigation**: [Index](../README.md) | [Lagrangian Dynamics →](02_lagrangian-dynamics.md)

---

## Prerequisites

If you have not read the excellent SIGGRAPH course notes on physics-based animation
by Witkin and Baraff, you can stop reading further right now. Go look for those notes at
http://www.cs.cmu.edu/~baraff/sigcourse/ and come back when you fully understand
everything in those notes.

## Target Audience

If you are still reading this document, you probably fit the following profile:

- You are a computer scientist with no mechanical engineering background
- You have minimal training in physics in high school
- You are seriously interested in physics-based character animation
- You have read Witkin and Baraff's SIGGRAPH course notes a few times
- You don't know where to go from simulating rigid bodies to human figures
- You have played with some commercial physics engines like ODE, PhysX, Havok, or Bullet
- You wish to simulate human behaviors more interesting than ragdoll effects

## Scope

Physics-based character animation consists of two parts: **simulation** and **control**.
This document focuses on the **simulation** part.

It's quite likely that you do not need to understand how the underlying simulation works
if your control algorithm is simple enough. However, complex human behaviors often require
sophisticated controllers that exploit the dynamics of a multibody system. A good
understanding of multibody dynamics is paramount for designing effective controllers.

## Questions This Document Answers

1. **Equations of Motion**: "I know how to derive the equations of motion for one rigid
   body and I have seen people use the following equations for articulated rigid bodies,
   but I don't know how they are derived."

   $$M(q)\ddot{q} + C(q,\dot{q}) = Q$$

2. **Euler-Lagrange Equation**: "I have seen the Euler-Lagrange equation in the following
   form before, but I don't know how it is related to the equations of motion above."

   $$\frac{d}{dt}\left(\frac{\partial T}{\partial \dot{q}}\right) - \frac{\partial T}{\partial q} - Q = 0$$

3. **Coordinate Conversion**: "I use generalized coordinates to compute the control forces,
   how do I convert them to Cartesian forces such that I can use simulators like ODE, PhysX,
   or Bullet which represent rigid bodies in the maximal coordinates?"

4. **Recursive Inverse Dynamics**: "I heard that inverse dynamics can be computed efficiently
   using a recursive formulation. How does that work?"

## DART Implementation

In DART, the equations of motion are computed via the `Skeleton` class:

| Concept                 | DART API                             | Notes                           |
| ----------------------- | ------------------------------------ | ------------------------------- |
| Mass matrix $M(q)$      | `Skeleton::getMassMatrix()`          | Returns `Eigen::MatrixXd`       |
| Coriolis $C(q,\dot{q})$ | `Skeleton::getCoriolisForces()`      | Returns `Eigen::VectorXd`       |
| Gravity $g(q)$          | `Skeleton::getGravityForces()`       | Returns `Eigen::VectorXd`       |
| Forward dynamics        | `Skeleton::computeForwardDynamics()` | Computes $\ddot{q}$ from $\tau$ |
| Inverse dynamics        | `Skeleton::computeInverseDynamics()` | Computes $\tau$ from $\ddot{q}$ |

See `dart/dynamics/Skeleton.hpp` for the implementation.

---

**Navigation**: [Index](../README.md) | [Lagrangian Dynamics →](02_lagrangian-dynamics.md)
