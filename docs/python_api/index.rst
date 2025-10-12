dartpy Python API Reference
============================

Welcome to the dartpy Python API documentation. This documentation is automatically generated from the dartpy Python bindings source code.

dartpy provides Python bindings for DART (Dynamic Animation and Robotics Toolkit), allowing you to use DART's powerful physics simulation and robotics capabilities from Python.

.. toctree::
   :maxdepth: 2
   :caption: API Modules

   modules/common
   modules/math
   modules/dynamics
   modules/simulation
   modules/collision
   modules/constraint
   modules/optimizer
   modules/utils
   modules/gui

Getting Started
---------------

To use dartpy in your Python code:

.. code-block:: python

   import dartpy as dart

   # Create a world
   world = dart.simulation.World()

   # Add a skeleton
   skeleton = dart.dynamics.Skeleton()
   world.addSkeleton(skeleton)

   # Simulate
   world.step()

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
