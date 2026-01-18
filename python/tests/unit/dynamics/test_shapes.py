import math

import dartpy as dart
import numpy as np
import pytest


class TestSphereShape:
    def test_create_sphere(self):
        radius = 0.5
        shape = dart.SphereShape(radius)

        assert shape is not None
        assert shape.get_type() == dart.SphereShape.get_static_type()

    def test_get_radius(self):
        radius = 0.75
        shape = dart.SphereShape(radius)

        assert shape.get_radius() == radius

    def test_set_radius(self):
        shape = dart.SphereShape(1.0)

        shape.set_radius(2.5)
        assert shape.get_radius() == 2.5

    def test_compute_inertia(self):
        radius = 1.0
        mass = 10.0
        shape = dart.SphereShape(radius)

        inertia = shape.compute_inertia(mass)
        assert inertia is not None
        assert inertia.shape == (3, 3)

    def test_static_compute_volume(self):
        radius = 1.0
        expected_volume = (4.0 / 3.0) * math.pi * radius**3

        volume = dart.SphereShape.compute_volume_of(radius)
        assert np.isclose(volume, expected_volume)

    def test_static_compute_inertia(self):
        radius = 1.0
        mass = 5.0

        inertia = dart.SphereShape.compute_inertia_of(radius, mass)
        assert inertia is not None
        assert inertia.shape == (3, 3)

    def test_repr(self):
        shape = dart.SphereShape(0.5)
        repr_str = repr(shape)

        assert "SphereShape" in repr_str
        assert "radius" in repr_str


class TestBoxShape:
    def test_create_box(self):
        size = np.array([1.0, 2.0, 3.0])
        shape = dart.BoxShape(size)

        assert shape is not None

    def test_get_size(self):
        size = np.array([0.5, 1.0, 1.5])
        shape = dart.BoxShape(size)

        retrieved_size = shape.get_size()
        assert np.allclose(retrieved_size, size)

    def test_set_size(self):
        shape = dart.BoxShape(np.array([1.0, 1.0, 1.0]))

        new_size = np.array([2.0, 3.0, 4.0])
        shape.set_size(new_size)

        assert np.allclose(shape.get_size(), new_size)

    def test_compute_inertia(self):
        size = np.array([1.0, 2.0, 3.0])
        mass = 10.0
        shape = dart.BoxShape(size)

        inertia = shape.compute_inertia(mass)
        assert inertia is not None
        assert inertia.shape == (3, 3)

    def test_static_compute_volume(self):
        size = np.array([2.0, 3.0, 4.0])
        expected_volume = 2.0 * 3.0 * 4.0

        volume = dart.BoxShape.compute_volume_of(size)
        assert np.isclose(volume, expected_volume)

    def test_static_compute_inertia(self):
        size = np.array([1.0, 2.0, 3.0])
        mass = 5.0

        inertia = dart.BoxShape.compute_inertia_of(size, mass)
        assert inertia is not None
        assert inertia.shape == (3, 3)

    def test_repr(self):
        shape = dart.BoxShape(np.array([1.0, 2.0, 3.0]))
        repr_str = repr(shape)

        assert "BoxShape" in repr_str
        assert "size" in repr_str


class TestMeshShape:
    @pytest.fixture
    def simple_triangle_mesh(self):
        mesh = dart.TriMesh()
        mesh.add_vertex([0.0, 0.0, 0.0])
        mesh.add_vertex([1.0, 0.0, 0.0])
        mesh.add_vertex([0.0, 1.0, 0.0])
        mesh.add_triangle(0, 1, 2)
        return mesh

    def test_create_mesh_shape(self, simple_triangle_mesh):
        scale = [1.0, 1.0, 1.0]
        shape = dart.MeshShape(scale, simple_triangle_mesh)

        assert shape is not None
        assert shape.get_type() == dart.MeshShape.get_static_type()

    def test_get_scale(self, simple_triangle_mesh):
        scale = [1.0, 2.0, 3.0]
        shape = dart.MeshShape(scale, simple_triangle_mesh)

        retrieved_scale = shape.get_scale()
        assert np.allclose(retrieved_scale, scale)

    def test_get_tri_mesh(self, simple_triangle_mesh):
        scale = [1.0, 1.0, 1.0]
        shape = dart.MeshShape(scale, simple_triangle_mesh)

        mesh = shape.get_tri_mesh()
        assert mesh is not None
        assert mesh.get_num_vertices() == 3
        assert mesh.get_num_triangles() == 1

    def test_non_uniform_scale(self, simple_triangle_mesh):
        scale = [2.0, 0.5, 1.5]
        shape = dart.MeshShape(scale, simple_triangle_mesh)

        assert np.allclose(shape.get_scale(), scale)


class TestShapeInBody:
    def test_add_sphere_to_body(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        shape = dart.SphereShape(0.5)
        shape_node = body.create_shape_node(shape)
        shape_node.create_visual_aspect()
        shape_node.create_collision_aspect()

        assert body.get_num_shape_nodes() == 1

    def test_add_box_to_body(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        shape = dart.BoxShape(np.array([1.0, 1.0, 1.0]))
        shape_node = body.create_shape_node(shape)
        shape_node.create_visual_aspect()

        assert body.get_num_shape_nodes() == 1

    def test_multiple_shapes_on_body(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        sphere = dart.SphereShape(0.5)
        shape_node1 = body.create_shape_node(sphere)
        shape_node1.create_visual_aspect()

        box = dart.BoxShape(np.array([1.0, 1.0, 1.0]))
        shape_node2 = body.create_shape_node(box)
        shape_node2.create_visual_aspect()

        assert body.get_num_shape_nodes() == 2


class TestShapeAspects:
    def test_visual_aspect(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        shape = dart.SphereShape(0.5)
        shape_node = body.create_shape_node(shape)
        shape_node.create_visual_aspect()

        assert shape_node.has_visual_aspect() is True
        assert shape_node.has_collision_aspect() is False

    def test_collision_aspect(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        shape = dart.SphereShape(0.5)
        shape_node = body.create_shape_node(shape)
        shape_node.create_collision_aspect()

        assert shape_node.has_visual_aspect() is False
        assert shape_node.has_collision_aspect() is True

    def test_dynamics_aspect(self):
        skel = dart.Skeleton()
        [joint, body] = skel.create_free_joint_and_body_node_pair()

        shape = dart.SphereShape(0.5)
        shape_node = body.create_shape_node(shape)
        shape_node.create_dynamics_aspect()

        assert shape_node.has_dynamics_aspect() is True


if __name__ == "__main__":
    pytest.main()
