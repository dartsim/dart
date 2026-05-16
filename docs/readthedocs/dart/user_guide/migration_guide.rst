Migration Guide
===============

This page captures noteworthy source-level changes between stable releases so you can migrate projects with minimal surprises. Only migrations for **published** versions are listed here; when a new major release approaches we will expand the guide accordingly.

DART 6 → DART 7
----------------

Component Headers Renamed
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Change**: ``<component>.hpp`` → ``All.hpp``

.. code-block:: cpp

   // Before
   #include <dart/dynamics/dynamics.hpp>
   #include <dart/collision/collision.hpp>

   // After
   #include <dart/dynamics/All.hpp>
   #include <dart/collision/All.hpp>

**Note**: Old headers still work but show deprecation warnings (removed in a future major release).

**Suppress warnings** (if needed during migration):

.. code-block:: cpp

   #define DART_SUPPRESS_DEPRECATED_HEADER_WARNING
   #include <dart/dynamics/dynamics.hpp>

Header Names: PascalCase → snake_case
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Change**: PascalCase headers → snake_case headers

.. code-block:: cpp

   // Before
   #include <dart/dynamics/BodyNode.hpp>
   #include <dart/collision/CollisionDetector.hpp>

   // After
   #include <dart/dynamics/body_node.hpp>
   #include <dart/collision/collision_detector.hpp>

**Note**: PascalCase headers continue to compile in DART 7.x but emit deprecation warnings. They are slated for removal in the next major release.

**Suppress warnings** (if needed during migration):

.. code-block:: cpp

   #define DART_SUPPRESS_DEPRECATED_HEADER_WARNING
   #include <dart/dynamics/BodyNode.hpp>  // No warning

Collision Detector Runtime
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Change**: the built-in DART collision detector is the normal runtime
collision stack. Use the ``dart`` factory key, ``DartCollisionDetector``, or
the default detector instead of selecting FCL, Bullet, or ODE as runtime
backends.

The C++ factory keys ``fcl``, ``fcl_mesh``, ``bullet``, and ``ode`` and the C++
classes ``FCLCollisionDetector``, ``BulletCollisionDetector``, and
``OdeCollisionDetector`` remain as DART 7 migration facades for downstream
source compatibility. These names route to the built-in DART detector; they do
not select external runtime engines.

The dartpy API intentionally keeps the cleaner DART 7 surface. Python code
should use ``dartpy.DartCollisionDetector`` or the default detector. The legacy
dartpy detector aliases ``DARTCollisionDetector``, ``FCLCollisionDetector``,
``BulletCollisionDetector``, and ``OdeCollisionDetector`` are not retained as
compatibility shims.

FCL, Bullet, and ODE remain available only as explicit reference-comparison
dependencies for tests and benchmarks. Enable those through the reference
test/benchmark gates, normally via the ``collision-reference`` Pixi
environment; normal core DART, dartpy, package, and downstream runtime builds
do not use per-engine collision build switches.

Need another migration?
-----------------------

Looking for guidance on a different upgrade path? Open a discussion or issue on GitHub so we can expand this page with the details you need.
