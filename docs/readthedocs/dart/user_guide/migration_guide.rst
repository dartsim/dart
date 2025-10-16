Migration Guide
===============

DART 7 → DART 8
----------------

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

**Note**: Old PascalCase headers still work in DART 7.x with deprecation warnings (removed in DART 8.0)

**Suppress warnings** (if needed during migration):

.. code-block:: cpp

   #define DART_SUPPRESS_DEPRECATED_HEADER_WARNING
   #include <dart/dynamics/BodyNode.hpp>  // No warning

DART 6 → DART 7
----------------

Component Headers Renamed
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Change**: ``<component>.hpp`` → ``all.hpp``

.. code-block:: cpp

   // Before
   #include <dart/dynamics/dynamics.hpp>
   #include <dart/collision/collision.hpp>

   // After
   #include <dart/dynamics/all.hpp>
   #include <dart/collision/all.hpp>

**Note**: Old headers still work but show deprecation warnings (removed in DART 8.0)

**Suppress warnings** (if needed during migration):

.. code-block:: cpp

   #define DART_SUPPRESS_DEPRECATED_HEADER_WARNING
   #include <dart/dynamics/dynamics.hpp>
