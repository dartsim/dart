Section: Python

  * **Behavior change:** Linux pip wheels load only Collada (`.dae`, `.zae`),
    STL, OBJ and glTF 2.0 (`.gltf`, `.glb`) meshes. They no longer load the
    other formats assimp reads, such as FBX, PLY, 3DS, Blender, DirectX `.x`,
    X3D, 3MF, IFC and glTF 1.0; macOS and Windows wheels still do. Convert such
    meshes to a supported format, or build dartpy from source.
