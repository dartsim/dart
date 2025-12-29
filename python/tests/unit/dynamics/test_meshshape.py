import dartpy as dart
import numpy as np
import pytest


def test_meshshape_trimesh_roundtrip():
    tri_mesh = dart.TriMesh()
    tri_mesh.add_vertex([0.0, 0.0, 0.0])
    tri_mesh.add_vertex([1.0, 0.0, 0.0])
    tri_mesh.add_vertex([0.0, 1.0, 0.0])
    tri_mesh.add_triangle(0, 1, 2)

    scale = [1.0, 2.0, 3.0]
    shape = dart.MeshShape(scale, tri_mesh)

    assert shape.get_type() == dart.MeshShape.get_static_type()
    np.testing.assert_allclose(np.asarray(shape.get_scale()), np.asarray(scale))

    retrieved = shape.get_tri_mesh()
    assert retrieved.get_num_vertices() == tri_mesh.get_num_vertices()
    assert retrieved.get_num_triangles() == tri_mesh.get_num_triangles()


if __name__ == "__main__":
    pytest.main()
