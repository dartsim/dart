dartpy Python API Reference
============================

Welcome to the dartpy Python API documentation. This documentation is automatically generated from the dartpy Python bindings source code.

dartpy provides Python bindings for DART (Dynamic Animation and Robotics Toolkit), a research-focused physics engine for robotics, animation, and machine learning. It gives Python users access to DART's dynamics, collision, model-loading, and simulation foundations without requiring them to leave the Python workflow.

.. toctree::
   :maxdepth: 2
   :caption: API Modules

   modules/dartpy
   modules/simulation
   modules/io
   modules/gui

Getting Started
---------------

To use dartpy in your Python code:

.. code-block:: python

   import dartpy as dart

   # Create a world
   world = dart.World()

   # Add a skeleton
   skeleton = dart.Skeleton()
   world.addSkeleton(skeleton)

   # Simulate
   world.step()

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
