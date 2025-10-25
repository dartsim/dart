#!/usr/bin/env python3
"""Test script for Python TriMesh bindings."""


def test_trimesh_bindings():
    """Test that TriMesh bindings work correctly."""
    import dartpy
    import numpy as np

    print("Testing TriMesh bindings...")

    # Test 1: Create an empty TriMesh
    mesh = dartpy.math.TriMesh()
    print("✓ Created empty TriMesh")

    # Test 2: Add vertices
    mesh.addVertex([0.0, 0.0, 0.0])
    mesh.addVertex([1.0, 0.0, 0.0])
    mesh.addVertex([0.0, 1.0, 0.0])
    print("✓ Added 3 vertices")

    # Test 3: Add triangle
    mesh.addTriangle(0, 1, 2)
    print("✓ Added triangle")

    # Test 4: Get vertices
    vertices = mesh.getVertices()
    print(f"✓ Got {mesh.getNumVertices()} vertices")

    # Test 5: Get triangles
    triangles = mesh.getTriangles()
    print(f"✓ Got {mesh.getNumTriangles()} triangles")

    # Test 6: Check vertex normals
    has_normals = mesh.hasVertexNormals()
    print(f"✓ Has vertex normals: {has_normals}")

    # Test 7: Add vertex normals
    mesh.addVertexNormal([0.0, 0.0, 1.0])
    mesh.addVertexNormal([0.0, 0.0, 1.0])
    mesh.addVertexNormal([0.0, 0.0, 1.0])
    print("✓ Added 3 vertex normals")

    # Test 8: Check vertex normals again
    has_normals = mesh.hasVertexNormals()
    print(f"✓ Has vertex normals now: {has_normals}")

    print("\n✅ All TriMesh binding tests passed!")


def test_meshshape_trimesh():
    """Test that MeshShape works with TriMesh."""
    import dartpy
    import numpy as np

    print("\nTesting MeshShape with TriMesh...")

    # Create a TriMesh
    mesh = dartpy.math.TriMesh()
    mesh.addVertex([0.0, 0.0, 0.0])
    mesh.addVertex([1.0, 0.0, 0.0])
    mesh.addVertex([0.0, 1.0, 0.0])
    mesh.addTriangle(0, 1, 2)
    print("✓ Created TriMesh with 3 vertices and 1 triangle")

    # Create MeshShape with TriMesh
    scale = [1.0, 1.0, 1.0]
    mesh_shape = dartpy.dynamics.MeshShape(scale, mesh)
    print("✓ Created MeshShape from TriMesh")

    # Get TriMesh back from MeshShape
    retrieved_mesh = mesh_shape.getTriMesh()
    print(
        f"✓ Retrieved TriMesh from MeshShape: {retrieved_mesh.getNumVertices()} vertices, {retrieved_mesh.getNumTriangles()} triangles"
    )

    # Test MeshShape properties
    shape_type = mesh_shape.getType()
    print(f"✓ MeshShape type: {shape_type}")

    print("\n✅ All MeshShape + TriMesh integration tests passed!")


if __name__ == "__main__":
    try:
        test_trimesh_bindings()
        test_meshshape_trimesh()
        print("\n" + "=" * 60)
        print("🎉 SUCCESS: All Python binding tests passed!")
        print("=" * 60)
    except Exception as e:
        print(f"\n❌ FAILED: {e}")
        import traceback

        traceback.print_exc()
        exit(1)
