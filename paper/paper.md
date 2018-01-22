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
 - name: Sehoon Ha
   orcid: 0000-0002-1972-328X
   affiliation: 3
 - name: Tobias Kunz
   orcid: 0000-0002-5614-2265
   affiliation: 4
 - name: Sumit Jain
   orcid: 0000-0003-2244-0077
   affiliation: 5
 - name: Yuting Ye
   orcid: 0000-0003-2643-7457
   affiliation: 6
 - name: Siddhartha S. Srinivasa
   orcid: 0000-0002-5091-106X
   affiliation: 1
 - name: Mike Stilman
   affiliation: 4
 - name: C. Karen Liu
   orcid: 0000-0001-5926-0905
   affiliation: 4
affiliations:
 - name: University of Washington
   index: 1
 - name: Open Source Robotics Foundation
   index: 2
 - name: Disney Research
   index: 3
 - name: Georgia Institute of Technology
   index: 4
 - name: Google Inc.
   index: 5
 - name: Oculus Research
   index: 6
date: 15 November 2017
bibliography: paper.bib
---

# Summary

DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open source library created by the [Graphics Lab](http://www.cc.gatech.edu/~karenliu/Home.html) and [Humanoid Robotics Lab](http://www.golems.org/) at [Georgia Institute of Technology](http://www.gatech.edu/) with ongoing contributions from the [Personal Robotics Lab](http://personalrobotics.cs.washington.edu/) at [University of Washington](http://www.washington.edu/) and [Open Source Robotics Foundation](https://www.osrfoundation.org/). The library provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation. DART is distinguished by its accuracy and stability due to its use of generalized coordinates to represent articulated rigid body systems in the geometric notations [@park1995lie] and Featherstone’s Articulated Body Algorithm [@featherstone2014rigid] using a Lie group formulation to compute forward dynamics [@ploen1999coordinate] and hybrid dynamics [@sohl2001recursive]. For developers, in contrast to many popular physics engines which view the simulator as a black box, DART gives full access to internal kinematic and dynamic quantities, such as the mass matrix, Coriolis and centrifugal forces, transformation matrices and their derivatives. DART also provides an efficient computation of Jacobian matrices for arbitrary body points and coordinate frames. The frame semantics of DART allows users to define arbitrary reference frames (both inertial and non-inertial) and use those frames to specify or request data. For air-tight code safety, forward kinematics and dynamics values are updated automatically through lazy evaluation, making DART suitable for real-time controllers. In addition, DART provides flexibility to extend the API for embedding user-provided classes into DART data structures. Contacts and collisions are handled using an implicit time-stepping, velocity-based LCP (linear complementarity problem) to guarantee non-penetration, directional friction, and approximated Coulomb friction cone conditions [@stewart1996implicit]. DART has applications in robotics and computer animation because it features a multibody dynamic simulator and various kinematic tools for control and motion planning.

# Research publications utilizing DART

1. Bayesian Optimization with Automatic Prior Selection for Data-Efficient Direct Policy Search, Rémi Pautrat, Konstantinos Chatzilygeroudis and Jean-Baptiste MouretRémi Pautrat, Konstantinos Chatzilygeroudis and Jean-Baptiste Mouret, in Proceedings of the International Conference on Robotics and Automation (ICRA), 2018 [[PDF](https://arxiv.org/pdf/1709.06919.pdf)]
1. Data-Driven Approach to Simulating Realistic Human Joint Constraints, Yifeng Jiang, and C. Karen Liu, (preprint), 2017 [[arXiv](https://arxiv.org/abs/1709.08685)]
1. Multi-task Learning with Gradient Guided Policy Specialization, Wenhao Yu, Greg Turk, and C. Karen Liu, (preprint), 2017 [[arXiv](https://arxiv.org/abs/1709.07979)]
1. Learning Human Behaviors for Robot-Assisted Dressing, Alex Clegg, Wenhao Yu, Jie Tan, Charlie Kemp, Greg Turk, and C. Karen Liu, (preprint), 2017 [[arXiv](https://arxiv.org/abs/1709.07033)]
1. Expanding Motor Skills through Relay Neural Networks, Visak C.V. Kumar, Sehoon Ha, and C. Karen Liu, (preprint), 2017 [[arXiv](https://arxiv.org/abs/1709.07932)]
1. Reset-free Trial-and-Error Learning for Robot Damage Recovery, Konstantinos Chatzilygeroudis, Vassilis Vassiliades, Jean-Baptiste Mouret, Robotics and Autonomous Systems, 2017 [[PDF](https://arxiv.org/pdf/1610.04213.pdf)]
1. Stair Negotiation Made Easier Using Novel Interactive Energy-Recycling Assistive Stairs, Yun Seong Song, Sehoon Ha, Hsiang Hsu, Lena H. Ting, and C. Karen Liu, in PLOS ONE, 2017 [[PDF](http://journals.plos.org/plosone/article?id=10.1371/journal.pone.0179637)]
1. Learning a Unified Control Policy for Safe Falling, Visak C.V. Kumar, Sehoon Ha, and C. Karen Liu, in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017 [[PDF](https://arxiv.org/abs/1703.02905)]
1. Learning to Navigate Cloth using Haptics, Alex Clegg, Wenhao Yu, Zackory Erickson, C. Karen Liu, and Greg Turk, in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017 [[PDF](https://arxiv.org/abs/1703.06905)]
1. Preparing for the Unknown: Learning a Universal Policy with Online System Identification, Wenhao Yu, Jie Tan, C. Karen Liu, and Greg Turk, Robotics Science and Systems (RSS), 2017 [[PDF](https://arxiv.org/abs/1702.02453)]
1. Using Centroidal Voronoi Tessellations to Scale Up the Multi-dimensional Archive of Phenotypic Elites Algorithm, Vassilis Vassiliades, Konstantinos Chatzilygeroudis, and Jean-Baptiste Mouret, in IEEE Transactions on Evolutionary Computation, 2016 [[PDF](https://arxiv.org/pdf/1610.05729.pdf)]
1. A Linear-Time Variational Integrator for Multibody Systems, Jeongseok Lee, C. Karen Liu, Frank C. Park, and Siddhartha S. Srinivasa, in Workshop on the Algorithmic Foundations of Robotics (WAFR), 2016 [[PDF](https://arxiv.org/abs/1609.02898)]
1. Simulation-Based Design of Dynamic Controllers for Humanoid Balancing, Jie Tan, Zhaoming Xie, Byron Boots, and C. Karen Liu, in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2016 [[PDF](http://www.jie-tan.net/project/simulationBasedDesign.pdf)]
1. Humanoid Manipulation Planning using Backward-Forward Search, Michael X. Grey, Caelan R. Garrett, C. Karen Liu, Aaron D. Ames, and Andrea L. Thomaz, in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2016 [[PDF](https://www.cc.gatech.edu/graphics/projects/Grey/humanoid_hbf_iros2016.pdf)]
1. Evolutionary Optimization for Parameterized Whole-body Dynamic Motor Skills, Sehoon Ha, and C. Karen Liu, in IEEE International Conference on Robotics and Automation (ICRA), 2016 [[PDF](http://sehoonha.com/projects/ha2016eop/2016_optskill.pdf)]
1. Dexterous Manipulation of Cloth, Yunfei Bai, Wenhao Yu (co-first author), and C. Karen Liu, in Computer Graphics Forum (Eurographics), 2016 [[PDF](https://www.cc.gatech.edu/~ybai30/cloth_manipulation/paper_cloth_manipulation.pdf)]
1. Multiple Contact Planning for Minimizing Damage of Humanoid Falls, Sehoon Ha, and C. Karen Liu, in IEEE/RSJ International Conference on Intelligent Robots and Systems, 2015 [[PDF](http://sehoonha.com/projects/ha2015mcs/2015_falling.pdf)]
1. Animating Human Dressing, Alex Clegg, Jia Tan, Greg Turk, and C. Karen Liu, in Transactions on Graphics (SIGGRAPH), 2015 [[PDF](https://www.cc.gatech.edu/~aclegg3/projects/2015_AnimatingHumanDressing.pdf)]
1. Coupling Cloth and Rigid Bodies for Dexterous Manipulation, Yunfei Bai, and C. Karen Liu, in Motion in Games, 2014 (Best Student Paper Award) [[PDF](https://www.cc.gatech.edu/~ybai30/cloth/paper_cloth.pdf)]
1. Orienting in Mid-air through Configuration Changes to Achieve a Rolling Landing for Reducing Impact after a Fall, Jeffrey T. Bingham, Jeongseok Lee, Ravi N. Haksar, Jun Ueda, and C. Karen Liu, in IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014 [[PDF](https://www.cc.gatech.edu/~karenliu/bingham_inertiabot_IROS2014.pdf)]
1. Dexterous Manipulation Using Both Palm and Fingers, Yunfei Bai, and C. Karen Liu, in IEEE International Conference on Robotics and Automation, 2014 [[PDF](https://www.cc.gatech.edu/~ybai30/hand/paper_hand.pdf)]
1. Synthesis of Concurrent Object Manipulation Tasks, Yunfei Bai, Kristin Siu, and C. Karen Liu, in ACM Transactions on Graphics (presented at SIGGRAPH Asia), 2012 [[PDF](https://www.cc.gatech.edu/~ybai30/multitask/multitask.pdf)]

# References
