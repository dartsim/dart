Migration Guide (Python)
========================

.. note::
   This is a quick reference for Python users migrating to DART 7.

.. warning::
   **Deprecation Timeline**: Deprecated APIs marked in this guide will be **removed in DART 7.1**
   (the next feature release). Please migrate to the new APIs as soon as possible to avoid
   breaking changes in future releases.

DART 6 → DART 7
----------------

MeshShape API
~~~~~~~~~~~~~

.. code-block:: python

   # OLD (deprecated)
   ai_scene = dart.MeshShape.loadMesh("mesh.obj", retriever)
   mesh_shape = dart.MeshShape(scale, ai_scene, "mesh.obj")

   # NEW (preferred) - load from file
   loader = dart.utils.AssimpMeshLoaderd()
   tri_mesh = loader.load("mesh.obj", retriever)
   mesh_shape = dart.MeshShape(scale, tri_mesh, "mesh.obj")

   # NEW (preferred) - procedural mesh
   tri_mesh = dart.math.TriMesh()
   tri_mesh.addVertex(v1)
   tri_mesh.addVertex(v2)
   tri_mesh.addVertex(v3)
   tri_mesh.addTriangle(dart.math.Triangle(0, 1, 2))
   mesh_shape = dart.MeshShape(scale, tri_mesh)

**Benefits:**

- Better performance (no aiScene conversion)
- Easier to create procedural meshes
- Consistent with C++ API
