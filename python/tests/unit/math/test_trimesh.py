import dartpy as dart
import numpy as np
import pytest


def test_trimesh_basic():
    mesh = dart.TriMesh()
    mesh.reserve_vertices(3)
    mesh.reserve_triangles(1)
    mesh.reserve_vertex_normals(3)

    mesh.add_vertex([0.0, 0.0, 0.0])
    mesh.add_vertex([1.0, 0.0, 0.0])
    mesh.add_vertex([0.0, 1.0, 0.0])
    mesh.add_triangle(0, 1, 2)

    assert mesh.get_num_vertices() == 3
    assert mesh.get_num_triangles() == 1

    vertices = mesh.get_vertices()
    assert len(vertices) == 3
    np.testing.assert_allclose(np.asarray(vertices[1]), np.array([1.0, 0.0, 0.0]))

    assert not mesh.has_vertex_normals()
    mesh.add_vertex_normal([0.0, 0.0, 1.0])
    mesh.add_vertex_normal([0.0, 0.0, 1.0])
    mesh.add_vertex_normal([0.0, 0.0, 1.0])
    assert mesh.has_vertex_normals()

    normals = mesh.get_vertex_normals()
    assert len(normals) == 3
    np.testing.assert_allclose(np.asarray(normals[0]), np.array([0.0, 0.0, 1.0]))

    triangles = mesh.get_triangles()
    assert len(triangles) == 1
    np.testing.assert_array_equal(np.asarray(triangles[0]), np.array([0, 1, 2]))


def test_trimesh_clear():
    mesh = dart.TriMesh()
    mesh.add_vertex([0.0, 0.0, 0.0])
    mesh.add_vertex([1.0, 0.0, 0.0])
    mesh.add_vertex([0.0, 1.0, 0.0])
    mesh.add_triangle(0, 1, 2)

    assert mesh.get_num_vertices() == 3
    assert mesh.get_num_triangles() == 1

    mesh.clear()
    assert mesh.get_num_vertices() == 0
    assert mesh.get_num_triangles() == 0
    assert not mesh.has_vertex_normals()


if __name__ == "__main__":
    pytest.main()
