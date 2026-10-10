Tutorials
=========

These four hands-on tutorials introduce DART's simulation and control APIs
using Python and NumPy. Each directory in
`python/tutorials <https://github.com/dartsim/dart/tree/main/python/tutorials>`_
contains ``main.py`` with numbered exercises and ``main_finished.py`` with
the answers. The directory names are ``multi_pendulum``, ``biped``,
``collisions``, and ``dominoes``.

Setup and execution
-------------------

From a source checkout, build dartpy with the OSG viewer:

.. code-block:: console

   pixi run build-py-dev
   pixi run tu-multi-pendulum-fi

The ``tu-*`` tasks build dartpy and launch the tutorials. Use the task without
``-fi`` to work through the exercise, or with ``-fi`` to run the solution:

.. list-table::
   :header-rows: 1

   * - Tutorial
     - Exercise
     - Solution
   * - Multi-pendulum
     - ``pixi run tu-multi-pendulum``
     - ``pixi run tu-multi-pendulum-fi``
   * - Biped
     - ``pixi run tu-biped``
     - ``pixi run tu-biped-fi``
   * - Collisions
     - ``pixi run tu-collisions``
     - ``pixi run tu-collisions-fi``
   * - Dominoes
     - ``pixi run tu-dominoes``
     - ``pixi run tu-dominoes-fi``

You can also run a script directly in an environment with dartpy installed:

.. code-block:: console

   python python/tutorials/biped/main.py
   python python/tutorials/biped/main_finished.py

A desktop session with OpenGL is required for the interactive viewer.
The scripts print the viewer and tutorial controls. Space starts or pauses
simulation. Exercise code that needs an unfinished model or controller
raises ``NotImplementedError`` identifying the lesson to complete.

Each script exposes ``build_scene()`` to construct its world node without
opening a window. The node retains its controller and event handler;
``main()`` attaches it to an OSG ``Viewer``. This also lets you inspect or step
a completed scene from Python.

.. toctree::
   :maxdepth: 1
   :caption: Lessons

   tutorials/multi-pendulum
   tutorials/biped
   tutorials/collisions
   tutorials/dominoes
