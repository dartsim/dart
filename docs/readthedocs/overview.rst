.. DART documentation master file, created by
   sphinx-quickstart on Sun Feb 19 22:01:28 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Overview
========

DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open-source library developed by the `Graphics Lab <http://www.cc.gatech.edu/~karenliu/Home.html>`_ and `Humanoid Robotics Lab <http://www.golems.org/>`_ at the `Georgia Institute of Technology <http://www.gatech.edu/>`_, with ongoing contributions from the `Personal Robotics Lab <http://personalrobotics.cs.washington.edu/>`_ at the `University of Washington <http://www.washington.edu/>`_ and the `Open Source Robotics Foundation <https://www.osrfoundation.org/>`_. It provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation. DART stands out due to its accuracy and stability, which are achieved through the use of generalized coordinates to represent articulated rigid body systems and the application of Featherstone's Articulated Body Algorithm to compute motion dynamics.

For developers, DART offers full access to internal kinematic and dynamic quantities, such as the mass matrix, Coriolis and centrifugal forces, transformation matrices, and their derivatives, unlike many popular physics engines that treat the simulator as a black box. It also provides efficient computation of Jacobian matrices for arbitrary body points and coordinate frames. The frame semantics of DART allow users to define and use arbitrary reference frames (both inertial and non-inertial) to specify or request data.

DART is suitable for real-time controllers due to its lazy evaluation, which automatically updates forward kinematics and dynamics values to ensure code safety. It also allows for the extension of the API to embed user-provided classes into DART data structures. Contacts and collisions are handled using an implicit time-stepping, velocity-based linear complementarity problem (LCP) to guarantee non-penetration, directional friction, and approximated Coulomb friction cone conditions.

In summary, DART has applications in robotics and computer animation as it features a multibody dynamic simulator and various kinematic tools for control and motion planning.

Features
--------

General
~~~~~~~

* Open-source C++ library licensed under the BSD license.
* Supports multiple platforms including Ubuntu, Archlinux, FreeBSD, macOS, and Windows.
* Fully integrated with Gazebo.
* Supports models in URDF and SDF formats.
* Provides default integration methods (semi-implicit Euler and RK4) and an extensible API for other numerical integration methods.
* Supports lazy evaluation and automatic updates of kinematic and dynamic quantities.
* Allows for the extension of the API to embed user-provided classes into its data structures.
* Provides comprehensive event recording in the simulation history.
* 3D visualization API using OpenGL and OpenSceneGraph with ImGui support.
* Extensible API to interface with various optimization problems, such as nonlinear programming and multi-objective optimization.

Collision Detection
~~~~~~~~~~~~~~~~~~~

* Support for multiple collision detectors: FCL, Bullet, and ODE.
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
* Analytic inverse kinematics interface with ikfast support.

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
