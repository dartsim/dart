Section: API

  * Deprecate more Assimp-typed APIs for removal in DART 8:
    `SoftMeshShape::getAssimpMesh()` (use the new
    `SoftMeshShape::getTriMesh()`, which returns a copy of the soft mesh's
    current vertices and faces), the `aiScene` overload of
    `dart::collision::createMesh` in `dart/collision/fcl/CollisionShapes.hpp`
    (use `MeshShape::getTriMesh()`), and `AssimpInputResourceAdaptor` and
    `createFileIO` (use `MeshShape`'s TriMesh-based APIs). Code that uses
    them now gets deprecation warnings.
    [#3938](https://github.com/dartsim/dart/pull/3938)
