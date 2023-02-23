.. DART documentation master file, created by
   sphinx-quickstart on Sun Feb 19 22:01:28 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to DART 7's documentation!
==================================

Introduction
------------

DART (Dynamic Animation and Robotics Toolkit) is a collaborative,
cross-platform, open-source library developed by the
`Graphics Lab <http://www.cc.gatech.edu/~karenliu/Home.html>`_ and
`Humanoid Robotics Lab <http://www.golems.org/>`_ at the
`Georgia Institute of Technology <http://www.gatech.edu/>`_, with ongoing
contributions from the
`Personal Robotics Lab <http://personalrobotics.cs.washington.edu/>`_ at the
`University of Washington <http://www.washington.edu/>`_ and the
`Open Source Robotics Foundation <https://www.osrfoundation.org/>`_. It provides
data structures and algorithms for kinematic and dynamic applications in
robotics and computer animation. DART stands out due to its accuracy and
stability, which are achieved through the use of generalized coordinates to
represent articulated rigid body systems and the application of Featherstone's
Articulated Body Algorithm to compute motion dynamics.

.. note::
   This document pertains to the new major version, DART 7, which is currently
   under heavy development. Please note that this early stage version may
   contain bugs or other issues. If you prefer to use a more stable version,
   please use DART 6.13 instead.

Vision for DART 7
-----------------

* Elevate the Python binding to a first-class component, ensuring full support and equivalent functionality to the C++ APIs, rather than remaining in an experimental stage.
* Modularize the library so that users can select specific components to use with minimal required dependencies, rather than having to use the entire library, including unnecessary parts.
* Utilize hardware accelerations, such as SIMD, multi-core CPUs, and GPUs, whenever available and enabled by the user, to maximize overall performance.
* Support both single and double precision, with options to compile the library for required scalar types or leave the template code uncompiled.
* Minimize dependencies to make the library usable without bringing in all transitive dependencies.
* Modernize implementation and public APIs to enable users to work with more intuitive and user-friendly APIs.
* Provide various resources, such as a quick start guide, examples, and tutorials, to lower the initial learning curve for users.

Citation
--------

If you use DART in an academic publication, please consider citing this
`JOSS Paper <https://doi.org/10.21105/joss.00500>`_
[`BibTeX <https://gist.github.com/jslee02/998b8809e3ae1b7aef6ef04dd2ad5e27>`_]

.. code-block:: bib

   @article{Lee2018,
     doi = {10.21105/joss.00500},
     url = {https://doi.org/10.21105/joss.00500},
     year  = {2018},
     month = {Feb},
     publisher = {The Open Journal},
     volume = {3},
     number = {22},
     pages = {500},
     author = {Jeongseok Lee and Michael X. Grey and Sehoon Ha and Tobias Kunz and Sumit Jain and Yuting Ye and Siddhartha S. Srinivasa and Mike Stilman and C. Karen Liu},
     title = {{DART}: Dynamic Animation and Robotics Toolkit},
     journal = {The Journal of Open Source Software}
   }


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Home
   
   overview
   gallery

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: User Guide
   
   installation
   tutorials

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Developer Guide

   build
   python_binding
