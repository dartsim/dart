Python API Reference
====================

Welcome to the dartpy Python API documentation. This section provides comprehensive reference documentation for all dartpy modules.

dartpy provides Python bindings for DART (Dynamic Animation and Robotics Toolkit), allowing you to use DART's powerful physics simulation and robotics capabilities from Python.

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

API Modules
-----------

.. toctree::
   :maxdepth: 2

   modules/common
   modules/math
   modules/dynamics
   modules/simulation
   modules/collision
   modules/constraint
   modules/optimizer
   modules/utils
   modules/gui
