Migration Guide
===============

.. note::
   This is a quick reference cheatsheet. For detailed documentation, see the User Guide.

.. warning::
   **Deprecation Timeline**: Deprecated APIs marked in this guide will be **removed in DART 7.1**
   (the next feature release). Please migrate to the new APIs as soon as possible to avoid
   breaking changes in future releases.

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

Component Headers
~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   // OLD                              // NEW
   #include <dart/dynamics/dynamics.hpp>  →  #include <dart/dynamics/all.hpp>
   #include <dart/collision/collision.hpp> →  #include <dart/collision/all.hpp>

MeshShape API
~~~~~~~~~~~~~

.. code-block:: cpp

   // OLD (deprecated) - load from file
   const aiScene* scene = MeshShape::loadMesh("mesh.obj", retriever);
   auto shape = make_shared<MeshShape>(scale, scene, "mesh.obj", retriever);

   // NEW (preferred) - load from file
   auto loader = make_unique<AssimpMeshLoaderd>();
   auto triMesh = loader->load("mesh.obj", retriever);
   auto shape = make_shared<MeshShape>(scale, triMesh, "mesh.obj");

   // NEW (preferred) - procedural mesh
   auto triMesh = make_shared<TriMesh<double>>();
   triMesh->addVertex(v1); triMesh->addVertex(v2); triMesh->addVertex(v3);
   triMesh->addTriangle(Triangle(0, 1, 2));
   auto shape = make_shared<MeshShape>(scale, triMesh);

.. note::
   For Python migration guide, see :doc:`/dartpy/user_guide/migration_guide`
