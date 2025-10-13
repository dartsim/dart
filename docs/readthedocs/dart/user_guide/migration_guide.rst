Migration Guide
===============

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
