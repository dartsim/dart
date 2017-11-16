---
title: 'DART: Dynamic Animation and Robotics Toolkit'
tags:
  - robotics
  - computer animation
  - forward kinematics
  - inverse kinematics
  - multibody dynamics
authors:
 - name: Jeongseok Lee
   orcid: 0000-0002-2146-7502
   affiliation: 1
 - name: Michael X. Grey
   orcid: 0000-0002-8159-2428
   affiliation: 2
 - name: Tobias Kunz
   orcid: 0000-0000-0000-0000
   affiliation: 3
 - name: Sehoon Ha
   orcid: 0000-0002-1972-328X
   affiliation: 4
 - name: Sumit Jain
   orcid: 0000-0000-0000-0000
   affiliation: 5
 - name: Yuting Ye
   orcid: 0000-0000-0000-0000
   affiliation: 6
 - name: Siddhartha S. Srinivasa
   orcid: 0000-0002-5091-106X
   affiliation: 1
 - name: Mike Stilman
   orcid: 0000-0000-0000-0000
   affiliation: 3
 - name: C. Karen Liu
   orcid: 0000-0000-0000-0000
   affiliation: 3
affiliations:
 - name: University of Washington
   index: 1
 - name: Open Source Robotics Foundation
   index: 2
 - name: Georgia Institute of Technology
   index: 3
 - name: Disney Research
   index: 4
 - name: Google
   index: 5
 - name: Oculus Research
   index: 6
date: 12 September 2017
bibliography: paper.bib
---

# Summary

DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open source library created by the [Graphics Lab](http://www.cc.gatech.edu/~karenliu/Home.html) and [Humanoid Robotics Lab](http://www.golems.org/) at [Georgia Institute of Technology](http://www.gatech.edu/) with ongoing contributions from the [Personal Robotics Lab](http://personalrobotics.cs.washington.edu/) at [University of Washington](http://www.washington.edu/) and [Open Source Robotics Foundation](https://www.osrfoundation.org/). The library provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation. DART is distinguished by its accuracy and stability due to its use of generalized coordinates to represent articulated rigid body systems and Featherstone’s Articulated Body Algorithm [@featherstone2014rigid] to compute the dynamics of motion. For developers, in contrast to many popular physics engines which view the simulator as a black box, DART gives full access to internal kinematic and dynamic quantities, such as the mass matrix, Coriolis and centrifugal forces, transformation matrices and their derivatives. DART also provides an efficient computation of Jacobian matrices for arbitrary body points and coordinate frames. The frame semantics of DART allows users to define arbitrary reference frames (both inertial and non-inertial) and use those frames to specify or request data. For air-tight code safety, forward kinematics and dynamics values are updated automatically through lazy evaluation, making DART suitable for real-time controllers. In addition, DART gives provides flexibility to extend the API for embedding user-provided classes into DART data structures. Contacts and collisions are handled using an implicit time-stepping, velocity-based LCP (linear complementarity problem) to guarantee non-penetration, directional friction, and approximated Coulomb friction cone conditions [@stewart1996implicit]. DART has applications in robotics and computer animation because it features a multibody dynamic simulator and various kinematic tools for control and motion planning.

# References
