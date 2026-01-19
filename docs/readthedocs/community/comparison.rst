How DART Compares
=================

This page helps you understand where DART fits in the robotics simulation
landscape and what differentiates it from other options.

.. note::

   The simulation ecosystem evolves rapidly. This page focuses on DART's
   design philosophy rather than feature-by-feature comparisons that may
   become outdated. For current capabilities of other simulators, please
   consult their official documentation.

DART's Design Philosophy
------------------------

DART was designed with specific goals that distinguish it from other simulators:

**Accuracy over speed**
   DART prioritizes numerical accuracy and constraint satisfaction. It uses
   Featherstone's O(n) Articulated Body Algorithm with careful attention to
   numerical stability.

**Open internals (white-box)**
   Unlike black-box simulators, DART exposes internal quantities: Jacobians,
   mass matrices, Coriolis forces, constraint forces, and their derivatives.
   This is essential for research in control, optimization, and learning.

**Gazebo integration**
   DART is a native physics backend for `Gazebo <https://gazebosim.org>`_,
   enabling use in ROS-based robotics workflows.

**Multi-format support**
   Load models from URDF, SDF, MJCF, and SKEL formats through a unified API.

**Python + C++ parity**
   The ``dartpy`` bindings provide equivalent functionality to the C++ API,
   allowing rapid prototyping in Python with production deployment in C++.

When to Explore Other Options
-----------------------------

Every simulator makes tradeoffs. You may want to explore alternatives if:

* **Maximum simulation speed** is your primary concern
* **GPU-accelerated parallel simulation** is needed for large-scale RL training
* **Simplest possible API** is preferred over control and transparency
* **Soft-body or fluid simulation** is a core requirement

The right choice depends on your specific use case. Many researchers use
multiple simulators for different purposes.

Other Simulators
----------------

Here are some other physics simulators commonly used in robotics research:

* `MuJoCo <https://mujoco.org/>`_
* `Drake <https://drake.mit.edu/>`_
* `PyBullet <https://pybullet.org/>`_
* `Isaac Sim <https://developer.nvidia.com/isaac-sim>`_
* `ODE <https://www.ode.org/>`_

We encourage you to evaluate multiple options for your specific needs.

Performance Notes
-----------------

Simulation speed depends on scene complexity, required accuracy, and use case.
DART uses O(n) Featherstone algorithms.

For a historical benchmark comparing physics engines in Gazebo, see
`this 2014 comparison video <https://vimeo.com/105584932>`_. Note that all
simulators have evolved significantly since then.
