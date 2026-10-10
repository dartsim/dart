Code Style Guide
================

This section describes the code style used in DART project.

C++ Style Guide
---------------

Macro Definitions
~~~~~~~~~~~~~~~~~

DART 6 uses all-caps macro names. The generated ``dart/config.hpp`` defines
optional dependency flags such as ``HAVE_BULLET``, ``HAVE_ODE``, and
``HAVE_OCTOMAP``, and feature flags such as ``DART_ENABLE_SIMD``. These flags
have values of 0 or 1. Follow the existing names when checking build features.

Naming Conventions
~~~~~~~~~~~~~~~~~~

C++ functions use camelCase and member variables generally use an ``m`` prefix
(for example, ``mTimeStep``). Follow the naming conventions in nearby code for
local variables.

Python Style Guide
------------------

Naming Conventions
~~~~~~~~~~~~~~~~~~

DART 6's dartpy bindings generally retain the C++ method names, including
camelCase. C++ namespaces are exposed as Python modules, such as
``dartpy.simulation`` and ``dartpy.utils``.

For example, these C++ methods keep the same names in Python:

.. code-block:: cpp

   dart::simulation::World world;
   world.setTimeStep(0.002);
   double timeStep = world.getTimeStep();

The equivalent dartpy code is:

.. code-block:: python

   import dartpy as dart

   world = dart.simulation.World()
   world.setTimeStep(0.002)
   time_step = world.getTimeStep()

Some Eigen adapters use snake_case methods, such as
``dartpy.math.Isometry3.set_rotation`` and ``set_translation``. Use the method
names exposed by each binding; there is no general camelCase-to-snake_case
conversion.

CMake Style Guide
-----------------

Follow the CMake style in ``CONTRIBUTING.md`` and nearby ``CMakeLists.txt`` or
``*.cmake`` files:

* Use two-space indentation.
* Quote singleton variable expansions, especially paths.
* Split complex commands across semantic groups instead of making one long
  line.
* Keep configure templates (``*.cmake.in``) conservative because they contain
  substitution tokens that are expanded by CMake.

The branch formats CMake sources with gersemi through ``scripts/lint_cmake.py``.
Run ``pixi run lint-cmake`` for CMake-only edits, or ``pixi run lint`` before
committing.
