.. DART documentation master file, created by
   sphinx-quickstart on Sun Feb 19 22:01:28 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Overview
========

DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open-source, research-focused physics engine developed by the `Graphics Lab <http://www.cc.gatech.edu/~karenliu/Home.html>`_ and `Humanoid Robotics Lab <http://www.golems.org/>`_ at the `Georgia Institute of Technology <http://www.gatech.edu/>`_, with ongoing contributions from the `Personal Robotics Lab <http://personalrobotics.cs.washington.edu/>`_ at the `University of Washington <http://www.washington.edu/>`_ and the `Open Source Robotics Foundation <https://www.osrfoundation.org/>`_. It provides data structures and algorithms for kinematic and dynamic applications in robotics, animation, and machine learning. DART stands out because it exposes accurate, stable dynamics foundations rather than hiding the simulator behind a black-box API. It uses generalized coordinates for articulated rigid body systems and Featherstone's Articulated Body Algorithm to compute motion dynamics.

For curated how-to material, visit :doc:`topics/index`, which aggregates the tutorial series and other deep dives such as the inverse kinematics guide.

For developers and researchers, DART offers full access to internal kinematic and dynamic quantities, such as the mass matrix, Coriolis and centrifugal forces, transformation matrices, and their derivatives, unlike many popular physics engines that treat the simulator as a black box. It also provides efficient computation of Jacobian matrices for arbitrary body points and coordinate frames. The frame semantics of DART allow users to define and use arbitrary reference frames (both inertial and non-inertial) to specify or request data.

DART is intended to be easy to start with and practical to extend. Python and C++ packages cover common installation paths, while pixi-based source builds support reproducible development. Its math, collision, constraint, model-loading, benchmark, and test foundations are organized so new algorithms can be implemented and compared against existing DART baselines. Current platform support is centered on cross-platform CPU execution, with roadmap work tracking multi-core, SIMD, and accelerator backends.

For the DART 7 direction — how the simulation pipeline supports **multiple physics domains, multiple solver methods, and multiple compute backends** in one step, with the available options at each abstraction seam — see :doc:`architecture`.

For visible citation guidance and credits for research methods implemented or
being reproduced in DART, see :doc:`papers`.

DART is suitable for real-time controllers due to its lazy evaluation, which automatically updates forward kinematics and dynamics values to ensure code safety. It also allows for the extension of the API to embed user-provided classes into DART data structures. Contacts and collisions are handled using an implicit time-stepping, velocity-based linear complementarity problem (LCP) to guarantee non-penetration, directional friction, and approximated Coulomb friction cone conditions.

In summary, DART supports robotics, animation, machine-learning research, and best-effort production use with a transparent multibody dynamics engine, kinematic tools for control and motion planning, and extension points for new algorithms.

Features
--------

.. note::

   Some capabilities listed below are not yet available in the DART 7 facade and
   remain **DART 6** features — notably the modular inverse-kinematics framework,
   analytic IkFast integration, and soft body nodes. See the `documentation
   migration plan
   <https://github.com/dartsim/dart/blob/main/docs/onboarding/dart7-docs-migration.md>`_
   for porting status.

General
~~~~~~~

* Open-source C++ library licensed under the BSD license.
* Supports multiple platforms including Ubuntu, FreeBSD, macOS, and Windows.
* Fully integrated with Gazebo.
* Supports models in URDF and SDF formats.
* Provides a default semi-implicit Euler timestepper plus low-level dynamics access for implementing custom numerical integration methods.
* Supports lazy evaluation and automatic updates of kinematic and dynamic quantities.
* Allows for the extension of the API to embed user-provided classes into its data structures.
* Provides comprehensive event recording in the simulation history.
* 3D visualization API using Filament with GLFW3 and Dear ImGui support.
* Extensible API to interface with various optimization problems, such as nonlinear programming and multi-objective optimization.

Collision Detection
~~~~~~~~~~~~~~~~~~~

* Built-in native collision detector for normal runtime use, with optional FCL,
  Bullet, and ODE reference comparisons for tests and benchmarks.
* Support for various collision shapes including primitive shapes, concave mesh, and probabilistic voxel grid.
* Support for minimum distance computation.

Kinematics
~~~~~~~~~~

* Support for numerous types of Joints.
* Support for numerous primitive and arbitrary body shapes with customizable inertial and material properties.
* Support for flexible skeleton modeling, including cloning and reconfiguring skeletons or subsections of a skeleton.
* Comprehensive access to kinematic states (e.g. transformation, position, velocity, or acceleration) of arbitrary entities and coordinate frames.
* Comprehensive access to various Jacobian matrices and their derivatives.
* Flexible conversion of coordinate frames.
* Fully modular inverse kinematics framework.
* Plug-and-play hierarchical whole-body inverse kinematics solver.
* Analytic inverse kinematics interface with IkFast support
  (:doc:`shared/inverse_kinematics/ikfast`).

Dynamics
~~~~~~~~

* High performance for articulated dynamic systems using Lie Group representation and Featherstone hybrid algorithms.
* Exact enforcement of joints between body nodes using generalized coordinates.
* Comprehensive API for dynamic quantities and their derivatives, such as the mass matrix, Coriolis force, gravitational force, and other external and internal forces.
* Support for both rigid and soft body nodes.
* Modeling of viscoelastic joint dynamics with joint friction and hard joint limits.
* Support for various types of actuators.
* Handling of contacts and collisions using an implicit LCP to guarantee non-penetration, directional friction, and approximated Coulomb friction cone conditions.
* Use of the "Island" technique to subdivide constraint handling for efficient performance.
* Support for various Cartesian constraints and extensible API for user-defined constraints.
* Multiple constraint solvers: Lemke method, Dantzig method, and PSG method.
* Support for dynamic systems with closed-loop structures.
