import gc
import weakref

import dartpy as dart
import numpy as np
import pytest


@pytest.mark.parametrize("layout", ["C", "F", "strided"])
def test_tri_mesh_owns_double_geometry_and_mesh_shape_retains_it(layout):
    vertices = np.array(
        [[0.123456789012345, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
        order="F" if layout == "F" else "C",
    )
    if layout == "strided":
        buffer = np.empty((8, 3))
        buffer[::2] = vertices
        vertices = buffer[::2]
    triangles = np.array([[0, 1, 2], [0, 3, 1]], dtype=np.int64)
    expected_vertices = vertices.copy()
    mesh = dart.math.TriMesh(vertices, triangles)
    assert mesh.num_vertices == 4
    assert mesh.num_triangles == 2
    np.testing.assert_array_equal(mesh.vertices, vertices)
    np.testing.assert_array_equal(mesh.triangles, triangles)
    shape = dart.dynamics.MeshShape([2, 3, 4], mesh)
    assert shape.tri_mesh is mesh
    np.testing.assert_array_equal(shape.getScale(), [2, 3, 4])
    vertices[:] = 99
    triangles[:] = 3
    mesh.vertices[:] = 42
    mesh.triangles[:] = 2
    np.testing.assert_array_equal(mesh.vertices, expected_vertices)
    np.testing.assert_array_equal(mesh.triangles, [[0, 1, 2], [0, 3, 1]])
    with pytest.raises(AttributeError):
        mesh.vertices = expected_vertices
    reference = weakref.ref(mesh)
    del mesh
    gc.collect()
    mesh = shape.tri_mesh
    np.testing.assert_array_equal(mesh.vertices, expected_vertices)
    del shape
    gc.collect()
    np.testing.assert_array_equal(mesh.triangles, [[0, 1, 2], [0, 3, 1]])
    # NumPy copies also survive the final native owner.
    vertices_copy = mesh.vertices
    del mesh
    gc.collect()
    assert reference() is None
    np.testing.assert_array_equal(vertices_copy, expected_vertices)


def test_empty_tri_mesh_has_shaped_independent_arrays():
    for mesh in (
        dart.math.TriMesh(),
        dart.math.TriMesh(np.empty((0, 3)), np.empty((0, 3), dtype=int)),
    ):
        assert mesh.vertices.shape == (0, 3)
        assert mesh.triangles.shape == (0, 3)
        assert mesh.num_vertices == mesh.num_triangles == 0


@pytest.mark.parametrize("index", [-1, 3, 0.5, np.nan, np.inf, 2**64 - 1])
def test_tri_mesh_rejects_unsafe_triangle_indices(index):
    vertices = np.eye(3)
    original = vertices.copy()
    with pytest.raises(ValueError, match="triangle indices"):
        dart.math.TriMesh(vertices, [[0, 1, index]])
    np.testing.assert_array_equal(vertices, original)


@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
def test_tri_mesh_rejects_nonfinite_vertices(value):
    with pytest.raises(ValueError, match="vertices must be finite"):
        dart.math.TriMesh([[value, 0, 0]], np.empty((0, 3)))


@pytest.mark.parametrize(
    "vertices, triangles",
    [
        ([[0, 0]], [[0, 0, 0]]),
        ([[0, 0, 0]], [[0, 0]]),
        ([[0, 0, 0]], [[[0, 0, 0]]]),
        (None, [[0, 0, 0]]),
        ([[0, 0, 0]], None),
    ],
)
def test_tri_mesh_rejects_incompatible_geometry_shapes(vertices, triangles):
    with pytest.raises(TypeError):
        dart.math.TriMesh(vertices, triangles)


def test_tri_mesh_rejects_triangles_without_vertices():
    with pytest.raises(ValueError, match="triangle indices"):
        dart.math.TriMesh(np.empty((0, 3)), [[0, 0, 0]])
