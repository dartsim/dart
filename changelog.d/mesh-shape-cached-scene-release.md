Section: Dynamics

  * `MeshShape` no longer crashes when `setMesh()` replaces the mesh of a shape
    built from a `TriMesh` after `getMesh()` was called on it. The shape
    released the `aiScene` it created for `getMesh()` twice.
