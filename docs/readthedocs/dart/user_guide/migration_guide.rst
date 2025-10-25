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

Need another migration?
-----------------------

Looking for guidance on a different upgrade path? Open a discussion or issue on GitHub so we can expand this page with the details you need.

.. note::
   For Python migrations, see :doc:`/dartpy/user_guide/migration_guide`.
