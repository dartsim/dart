import numpy as np
import pytest
from pathlib import Path

import dartpy as dart


def test_experimental_stub_surface_is_backend_hidden():
    repo_root = Path(__file__).resolve().parents[4]
    gui_stub = repo_root / "python" / "stubs" / "dartpy" / "gui" / "__init__.pyi"
    experimental_stub = (
        repo_root / "python" / "stubs" / "dartpy" / "gui" / "experimental.pyi"
    )
    gui_docs = repo_root / "docs" / "python_api" / "modules" / "gui.rst"
    rtd_conf = repo_root / "docs" / "readthedocs" / "conf.py"

    assert "from . import experimental" in gui_stub.read_text()
    assert "automodule:: dartpy.gui.experimental" in gui_docs.read_text()
    assert "dartpy.gui.experimental" in rtd_conf.read_text()
    text = experimental_stub.read_text()
    for token in (
        "Filament",
        "GLFW",
        "ImGui",
        "OpenGL",
        "Vulkan",
        "Metal",
        "OSG",
        "Raylib",
        "osg::",
        "filament::",
    ):
        assert token not in text


def test_experimental_extract_renderables_from_world():
    world = dart.World.create("world")
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    shape = dart.BoxShape(np.array([1.0, 2.0, 3.0]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect()
    world.add_skeleton(skeleton)

    renderables = dart.gui.experimental.extract_renderables(world)

    assert len(renderables) == 1
    descriptor = renderables[0]
    assert descriptor.skeleton_name == "robot"
    assert descriptor.geometry.kind == dart.gui.experimental.ShapeKind.Box
    assert hasattr(dart.gui.experimental.ShapeKind, "Pyramid")
    assert hasattr(dart.gui.experimental.ShapeKind, "MultiSphere")
    assert hasattr(dart.gui.experimental.ShapeKind, "LineSegments")
    assert hasattr(dart.gui.experimental.ShapeKind, "ConvexMesh")
    assert hasattr(dart.gui.experimental.ShapeKind, "PointCloud")
    assert hasattr(dart.gui.experimental.ShapeKind, "Heightmap")
    assert hasattr(dart.gui.experimental.ShapeKind, "SoftMesh")
    assert hasattr(dart.gui.experimental.ShapeKind, "VoxelGrid")
    assert hasattr(dart.gui.experimental, "MeshAlphaMode")
    assert hasattr(dart.gui.experimental.MeshAlphaMode, "ShapeAlpha")
    geometry = dart.gui.experimental.GeometryDescriptor()
    assert hasattr(geometry, "voxel_centers")
    assert hasattr(geometry, "voxel_size")
    assert hasattr(geometry, "triangle_vertices")
    assert hasattr(geometry, "triangle_indices")
    assert hasattr(geometry, "triangle_normals")
    assert hasattr(geometry, "unsupported_reason")
    assert hasattr(geometry, "mesh_uses_material_colors")
    assert hasattr(geometry, "mesh_alpha_mode")
    assert hasattr(geometry, "mesh_texture_coord_components")
    assert hasattr(geometry, "mesh_texture_coordinates")
    assert hasattr(geometry, "mesh_materials")
    assert hasattr(geometry, "mesh_parts")
    material_descriptor = dart.gui.experimental.MeshMaterialDescriptor()
    assert hasattr(material_descriptor, "base_color_texture_path")
    assert hasattr(material_descriptor, "metallic_roughness_texture_path")
    active_state = dart.gui.experimental.ActiveRenderableState()
    assert hasattr(active_state, "shape_version")
    assert hasattr(active_state, "render_resource_version")
    part_descriptor = dart.gui.experimental.MeshPartDescriptor()
    assert hasattr(part_descriptor, "triangle_count")
    assert hasattr(part_descriptor, "material_index")
    assert hasattr(descriptor, "render_resource_version")
    assert np.allclose(descriptor.geometry.size, [1.0, 2.0, 3.0])
    assert descriptor.material.visible is True

    selection_lines = dart.gui.experimental.make_selection_debug_lines(descriptor)
    assert len(selection_lines) == 12
    assert selection_lines[0].label == "selection.bounds"

    ray = dart.gui.experimental.PickRay()
    ray.origin = np.array([-2.0, 0.0, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.experimental.pick_nearest_renderable(renderables, ray)
    assert hit is not None
    assert np.allclose(hit.point, [-0.5, 0.0, 0.0])
    assert np.allclose(hit.normal, [-1.0, 0.0, 0.0])


def test_experimental_plan_renderable_set_update():
    visible_a = dart.gui.experimental.RenderableDescriptor()
    visible_a.id = 1
    visible_a.material.visible = True

    hidden = dart.gui.experimental.RenderableDescriptor()
    hidden.id = 2
    hidden.material.visible = False

    visible_b = dart.gui.experimental.RenderableDescriptor()
    visible_b.id = 3
    visible_b.material.visible = True

    duplicate_visible_a = dart.gui.experimental.RenderableDescriptor()
    duplicate_visible_a.id = visible_a.id
    duplicate_visible_a.material.visible = True

    invalid_id = dart.gui.experimental.RenderableDescriptor()
    invalid_id.id = 0
    invalid_id.material.visible = True

    plan = dart.gui.experimental.plan_renderable_set_update(
        [visible_a, hidden, visible_b, duplicate_visible_a, invalid_id],
        [visible_a.id, hidden.id, 4, visible_a.id, 0],
    )

    assert plan.descriptor_indices_to_add == [2]
    assert plan.active_renderable_indices_to_remove == [1, 2, 3, 4]

    visible_a.shape_version = 2
    visible_a.render_resource_version = 20
    visible_b.shape_version = 4
    visible_b.render_resource_version = 40
    stale_a = dart.gui.experimental.ActiveRenderableState()
    stale_a.id = visible_a.id
    stale_a.shape_version = visible_a.shape_version
    stale_a.render_resource_version = 10
    current_b = dart.gui.experimental.ActiveRenderableState()
    current_b.id = visible_b.id
    current_b.shape_version = visible_b.shape_version
    current_b.render_resource_version = visible_b.render_resource_version

    versioned_plan = dart.gui.experimental.plan_renderable_set_update(
        [visible_a, visible_b],
        [stale_a, current_b],
    )

    assert versioned_plan.descriptor_indices_to_add == [0]
    assert versioned_plan.active_renderable_indices_to_remove == [0]


def test_experimental_pick_sphere_uses_surface_normal():
    world = dart.World.create("world")
    skeleton = dart.Skeleton("sphere_robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    shape_node = body.create_shape_node(dart.SphereShape(1.0))
    shape_node.create_visual_aspect()
    world.add_skeleton(skeleton)

    renderables = dart.gui.experimental.extract_renderables(world)
    ray = dart.gui.experimental.PickRay()
    ray.origin = np.array([-2.0, 0.5, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.experimental.pick_nearest_renderable(renderables, ray)

    expected_x = -np.sqrt(0.75)
    assert hit is not None
    assert np.isclose(hit.distance, 2.0 + expected_x)
    assert np.allclose(hit.point, [expected_x, 0.5, 0.0])
    assert np.allclose(hit.normal, [expected_x, 0.5, 0.0])


def test_experimental_pick_multi_sphere_uses_surface_normal():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.MultiSphere
    renderable.geometry.sphere_centers = [
        np.array([-1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    ]
    renderable.geometry.sphere_radii = [0.5, 0.25]
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.5, -0.5, -0.5])
    renderable.geometry.local_bounds_max = np.array([1.25, 0.5, 0.5])

    ray = dart.gui.experimental.PickRay()
    ray.origin = np.array([-3.0, 0.25, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.experimental.pick_nearest_renderable([renderable], ray)

    expected_x = -1.0 - np.sqrt(0.1875)
    expected_normal = np.array([expected_x + 1.0, 0.25, 0.0])
    expected_normal /= np.linalg.norm(expected_normal)
    assert hit is not None
    assert np.isclose(hit.distance, 3.0 + expected_x)
    assert np.allclose(hit.point, [expected_x, 0.25, 0.0])
    assert np.allclose(hit.normal, expected_normal)


def test_experimental_pick_cylinder_uses_surface_normal():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.Cylinder
    renderable.geometry.radius = 1.0
    renderable.geometry.height = 2.0
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -1.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 1.0])

    ray = dart.gui.experimental.PickRay()
    ray.origin = np.array([-2.0, 0.5, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.experimental.pick_nearest_renderable([renderable], ray)

    expected_x = -np.sqrt(0.75)
    assert hit is not None
    assert np.isclose(hit.distance, 2.0 + expected_x)
    assert np.allclose(hit.point, [expected_x, 0.5, 0.0])
    assert np.allclose(hit.normal, [expected_x, 0.5, 0.0])


def test_experimental_pick_capsule_uses_surface_normal():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.Capsule
    renderable.geometry.radius = 1.0
    renderable.geometry.height = 2.0
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -2.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 2.0])

    side_ray = dart.gui.experimental.PickRay()
    side_ray.origin = np.array([-2.0, 0.5, 0.0])
    side_ray.direction = np.array([1.0, 0.0, 0.0])
    side_hit = dart.gui.experimental.pick_nearest_renderable([renderable], side_ray)

    side_x = -np.sqrt(0.75)
    assert side_hit is not None
    assert np.isclose(side_hit.distance, 2.0 + side_x)
    assert np.allclose(side_hit.point, [side_x, 0.5, 0.0])
    assert np.allclose(side_hit.normal, [side_x, 0.5, 0.0])

    cap_ray = dart.gui.experimental.PickRay()
    cap_ray.origin = np.array([0.5, 0.0, 4.0])
    cap_ray.direction = np.array([0.0, 0.0, -1.0])
    cap_hit = dart.gui.experimental.pick_nearest_renderable([renderable], cap_ray)

    cap_z = 1.0 + np.sqrt(0.75)
    assert cap_hit is not None
    assert np.isclose(cap_hit.distance, 4.0 - cap_z)
    assert np.allclose(cap_hit.point, [0.5, 0.0, cap_z])
    assert np.allclose(cap_hit.normal, [0.5, 0.0, np.sqrt(0.75)])


def test_experimental_pick_cone_uses_surface_normal():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.Cone
    renderable.geometry.radius = 1.0
    renderable.geometry.height = 2.0
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -1.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 1.0])

    side_ray = dart.gui.experimental.PickRay()
    side_ray.origin = np.array([-2.0, 0.25, 0.0])
    side_ray.direction = np.array([1.0, 0.0, 0.0])
    side_hit = dart.gui.experimental.pick_nearest_renderable([renderable], side_ray)

    side_x = -np.sqrt(0.1875)
    side_normal = np.array([side_x, 0.25, 0.25])
    side_normal /= np.linalg.norm(side_normal)
    assert side_hit is not None
    assert np.isclose(side_hit.distance, 2.0 + side_x)
    assert np.allclose(side_hit.point, [side_x, 0.25, 0.0])
    assert np.allclose(side_hit.normal, side_normal)

    base_ray = dart.gui.experimental.PickRay()
    base_ray.origin = np.array([0.5, 0.0, -4.0])
    base_ray.direction = np.array([0.0, 0.0, 1.0])
    base_hit = dart.gui.experimental.pick_nearest_renderable([renderable], base_ray)

    assert base_hit is not None
    assert np.isclose(base_hit.distance, 3.0)
    assert np.allclose(base_hit.point, [0.5, 0.0, -1.0])
    assert np.allclose(base_hit.normal, [0.0, 0.0, -1.0])


def test_experimental_pick_pyramid_uses_surface_normal():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.Pyramid
    renderable.geometry.size = np.array([2.0, 2.0, 2.0])
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -1.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 1.0])

    side_ray = dart.gui.experimental.PickRay()
    side_ray.origin = np.array([0.0, -3.0, 0.0])
    side_ray.direction = np.array([0.0, 1.0, 0.0])
    side_hit = dart.gui.experimental.pick_nearest_renderable([renderable], side_ray)

    side_normal = np.array([0.0, -2.0, 1.0])
    side_normal /= np.linalg.norm(side_normal)
    assert side_hit is not None
    assert np.isclose(side_hit.distance, 2.5)
    assert np.allclose(side_hit.point, [0.0, -0.5, 0.0])
    assert np.allclose(side_hit.normal, side_normal)

    base_ray = dart.gui.experimental.PickRay()
    base_ray.origin = np.array([0.5, 0.25, -4.0])
    base_ray.direction = np.array([0.0, 0.0, 1.0])
    base_hit = dart.gui.experimental.pick_nearest_renderable([renderable], base_ray)

    assert base_hit is not None
    assert np.isclose(base_hit.distance, 3.0)
    assert np.allclose(base_hit.point, [0.5, 0.25, -1.0])
    assert np.allclose(base_hit.normal, [0.0, 0.0, -1.0])


def test_experimental_pick_plane_uses_finite_proxy_surface():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.Plane
    renderable.geometry.normal = np.array([0.0, 0.0, 1.0])
    renderable.geometry.offset = 0.25
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, 0.23])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 0.27])

    hit_ray = dart.gui.experimental.PickRay()
    hit_ray.origin = np.array([0.25, 0.5, 2.0])
    hit_ray.direction = np.array([0.0, 0.0, -1.0])
    hit = dart.gui.experimental.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.75)
    assert np.allclose(hit.point, [0.25, 0.5, 0.25])
    assert np.allclose(hit.normal, [0.0, 0.0, 1.0])

    miss_ray = dart.gui.experimental.PickRay()
    miss_ray.origin = np.array([1.5, 0.0, 2.0])
    miss_ray.direction = np.array([0.0, 0.0, -1.0])
    assert dart.gui.experimental.pick_nearest_renderable([renderable], miss_ray) is None


def test_experimental_pick_triangle_mesh_uses_surface_triangles():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.Mesh
    renderable.geometry.triangle_vertices = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
    ]
    renderable.geometry.triangle_indices = [np.array([0, 1, 2], dtype=np.int32)]
    renderable.geometry.triangle_normals = [
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    renderable.geometry.mesh_texture_coordinates = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
    ]
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([0.0, 0.0, -0.1])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 0.1])

    hit_ray = dart.gui.experimental.PickRay()
    hit_ray.origin = np.array([0.25, 0.25, 1.0])
    hit_ray.direction = np.array([0.0, 0.0, -1.0])
    hit = dart.gui.experimental.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.0)
    assert np.allclose(hit.point, [0.25, 0.25, 0.0])
    assert np.allclose(hit.normal, [0.0, 0.0, 1.0])

    miss_ray = dart.gui.experimental.PickRay()
    miss_ray.origin = np.array([0.75, 0.75, 1.0])
    miss_ray.direction = np.array([0.0, 0.0, -1.0])
    assert dart.gui.experimental.pick_nearest_renderable([renderable], miss_ray) is None


def test_experimental_pick_point_cloud_uses_per_point_box_surface():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.PointCloud
    renderable.geometry.point_cloud_points = [
        np.array([0.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
    ]
    renderable.geometry.point_size = 0.2
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-0.1, -0.1, -0.1])
    renderable.geometry.local_bounds_max = np.array([2.1, 0.1, 0.1])

    hit_ray = dart.gui.experimental.PickRay()
    hit_ray.origin = np.array([-2.0, 0.05, 0.0])
    hit_ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.experimental.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.9)
    assert np.allclose(hit.point, [-0.1, 0.05, 0.0])
    assert np.allclose(hit.normal, [-1.0, 0.0, 0.0])

    gap_ray = dart.gui.experimental.PickRay()
    gap_ray.origin = np.array([1.0, 0.0, -1.0])
    gap_ray.direction = np.array([0.0, 0.0, 1.0])
    assert dart.gui.experimental.pick_nearest_renderable([renderable], gap_ray) is None


def test_experimental_pick_voxel_grid_uses_per_voxel_box_surface():
    renderable = dart.gui.experimental.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.experimental.ShapeKind.VoxelGrid
    renderable.geometry.voxel_centers = [
        np.array([0.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
    ]
    renderable.geometry.voxel_size = 0.2
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-0.1, -0.1, -0.1])
    renderable.geometry.local_bounds_max = np.array([2.1, 0.1, 0.1])

    hit_ray = dart.gui.experimental.PickRay()
    hit_ray.origin = np.array([-2.0, 0.05, 0.0])
    hit_ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.experimental.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.9)
    assert np.allclose(hit.point, [-0.1, 0.05, 0.0])
    assert np.allclose(hit.normal, [-1.0, 0.0, 0.0])

    gap_ray = dart.gui.experimental.PickRay()
    gap_ray.origin = np.array([1.0, 0.0, -1.0])
    gap_ray.direction = np.array([0.0, 0.0, 1.0])
    assert dart.gui.experimental.pick_nearest_renderable([renderable], gap_ray) is None


def test_experimental_extract_renderables_from_simple_frame():
    world = dart.World.create("world")
    transform = dart.Isometry3()
    transform.set_translation(np.array([1.0, -2.0, 0.5]))
    frame = dart.SimpleFrame(dart.Frame.world(), "interactive_target", transform)
    frame.set_shape(dart.BoxShape(np.array([0.4, 0.5, 0.6])))
    frame.get_visual_aspect(True).set_rgba(np.array([0.9, 0.2, 0.1, 0.8]))
    world.add_simple_frame(frame)

    renderables = dart.gui.experimental.extract_renderables(world)

    assert len(renderables) == 1
    descriptor = renderables[0]
    assert descriptor.skeleton_name == ""
    assert descriptor.body_name == ""
    assert descriptor.shape_frame_name == "interactive_target"
    assert descriptor.shape_node_name == ""
    assert descriptor.shape_node_version == 0
    assert descriptor.geometry.kind == dart.gui.experimental.ShapeKind.Box
    assert np.allclose(descriptor.geometry.size, [0.4, 0.5, 0.6])
    assert np.allclose(descriptor.world_transform.translation(), [1.0, -2.0, 0.5])

    assert dart.gui.experimental.translate_simple_frame_renderable(
        descriptor, np.array([0.25, 0.5, -0.1])
    )
    moved_renderables = dart.gui.experimental.extract_renderables(world)
    assert np.allclose(
        moved_renderables[0].world_transform.translation(), [1.25, -1.5, 0.4]
    )

    assert dart.gui.experimental.translate_frame_renderable(
        moved_renderables[0], np.array([-0.25, -0.5, 0.1])
    )
    restored_renderables = dart.gui.experimental.extract_renderables(world)
    assert np.allclose(
        restored_renderables[0].world_transform.translation(), [1.0, -2.0, 0.5]
    )


def test_experimental_debug_grid_lines():
    options = dart.gui.experimental.DebugDrawOptions()
    options.grid_half_extent = 1.0
    options.grid_spacing = 1.0

    lines = dart.gui.experimental.make_grid_debug_lines(options)

    assert len(lines) == 6
    assert lines[0].label == "grid"
    assert np.allclose(lines[0].from_point, [-1.0, -1.0, 0.08])
    assert np.allclose(lines[0].to_point, [1.0, -1.0, 0.08])


def test_experimental_translate_free_joint_renderable():
    world = dart.World.create("world")
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    shape = dart.BoxShape(np.array([1.0, 1.0, 1.0]))
    shape_node = body.create_shape_node(shape)
    shape_node.create_visual_aspect()
    world.add_skeleton(skeleton)

    renderables = dart.gui.experimental.extract_renderables(world)
    assert len(renderables) == 1

    assert dart.gui.experimental.translate_free_joint_renderable(
        renderables[0], np.array([0.25, -0.5, 0.75])
    )
    moved_renderables = dart.gui.experimental.extract_renderables(world)
    assert np.allclose(
        moved_renderables[0].world_transform.translation(), [0.25, -0.5, 0.75]
    )

    assert not dart.gui.experimental.translate_free_joint_renderable(
        moved_renderables[0], np.array([np.nan, 0.0, 0.0])
    )


def test_experimental_plane_drag_helpers():
    previous_ray = dart.gui.experimental.PickRay()
    previous_ray.origin = np.array([0.0, 0.0, 1.0])
    previous_ray.direction = np.array([0.0, 0.0, -1.0])
    current_ray = dart.gui.experimental.PickRay()
    current_ray.origin = np.array([1.0, 2.0, 1.0])
    current_ray.direction = np.array([0.0, 0.0, -1.0])

    intersection = dart.gui.experimental.intersect_plane(
        previous_ray, np.zeros(3), np.array([0.0, 0.0, 1.0])
    )
    assert np.allclose(intersection, [0.0, 0.0, 0.0])

    translation = dart.gui.experimental.compute_plane_drag_translation(
        previous_ray, current_ray, np.zeros(3), np.array([0.0, 0.0, 1.0])
    )
    assert np.allclose(translation, [1.0, 2.0, 0.0])

    parallel_ray = dart.gui.experimental.PickRay()
    parallel_ray.origin = np.array([0.0, 0.0, 1.0])
    parallel_ray.direction = np.array([1.0, 0.0, 0.0])
    assert (
        dart.gui.experimental.intersect_plane(
            parallel_ray, np.zeros(3), np.array([0.0, 0.0, 1.0])
        )
        is None
    )


def test_experimental_center_of_mass_debug_lines():
    world = dart.World.create("world")
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    body.get_inertia().set_mass(2.0)
    world.add_skeleton(skeleton)

    options = dart.gui.experimental.DebugDrawOptions()
    options.draw_grid = False
    options.draw_world_frame = False
    options.draw_contacts = False
    options.draw_centers_of_mass = True
    options.center_of_mass_marker_radius = 0.2

    lines = dart.gui.experimental.extract_debug_lines(world, options)

    assert len(lines) == 3
    assert lines[0].label == "robot.com.x"
    assert np.allclose(lines[0].from_point, [-0.2, 0.0, 0.0])
    assert np.allclose(lines[0].to_point, [0.2, 0.0, 0.0])


def test_experimental_inertia_debug_options():
    world = dart.World.create("world")
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    body.get_inertia().set_mass(12.0)
    body.get_inertia().set_moment(np.diag([52.0, 40.0, 20.0]))
    world.add_skeleton(skeleton)

    options = dart.gui.experimental.DebugDrawOptions()
    options.draw_grid = False
    options.draw_world_frame = False
    options.draw_contacts = False
    options.draw_inertia_boxes = True
    options.inertia_box_scale = 0.5

    assert options.draw_inertia_boxes is True
    assert np.isclose(options.inertia_box_scale, 0.5)
    lines = dart.gui.experimental.make_inertia_debug_lines(
        body, options, "robot/body"
    )
    assert len(lines) == 12
    assert lines[0].label == "robot/body.inertia"

    extracted_lines = dart.gui.experimental.extract_debug_lines(world, options)
    assert len(extracted_lines) == 12
    assert extracted_lines[0].label.endswith(".inertia")


def test_experimental_collision_shape_debug_lines():
    world = dart.World.create("world")
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    shape_node = body.create_shape_node(dart.BoxShape(np.array([2.0, 4.0, 6.0])))
    shape_node.create_collision_aspect()
    world.add_skeleton(skeleton)

    assert len(dart.gui.experimental.extract_renderables(world)) == 0

    options = dart.gui.experimental.DebugDrawOptions()
    options.draw_grid = False
    options.draw_world_frame = False
    options.draw_contacts = False
    options.draw_collision_shape_bounds = True
    options.collision_bounds_padding = 0.1

    assert options.draw_collision_shape_bounds is True
    assert np.isclose(options.collision_bounds_padding, 0.1)
    lines = dart.gui.experimental.make_collision_shape_debug_lines(
        shape_node, options, "robot/body/collision"
    )
    assert len(lines) == 12
    assert lines[0].label == "robot/body/collision.collision_bounds"

    extracted_lines = dart.gui.experimental.extract_debug_lines(world, options)
    assert len(extracted_lines) == 12
    assert extracted_lines[0].label.endswith(".collision_bounds")


def test_experimental_support_polygon_debug_options():
    skeleton = dart.Skeleton("supportless")
    options = dart.gui.experimental.DebugDrawOptions()
    options.draw_support_polygons = True
    options.draw_support_centroids = True
    options.support_polygon_elevation = 0.03
    options.support_centroid_marker_radius = 0.04

    lines = dart.gui.experimental.make_support_polygon_debug_lines(
        skeleton, options, "supportless"
    )

    assert len(lines) == 0


def test_experimental_camera_and_run_helpers():
    options = dart.gui.experimental.RunOptions()
    options.width = 0
    options.height = -3
    options.gui_scale = float("nan")
    options.headless = True
    options.screenshot_path = "capture.ppm"

    dart.gui.experimental.normalize_run_options(options)

    assert options.width == 1
    assert options.height == 1
    assert np.isclose(options.gui_scale, 1.0)
    assert options.max_frames == 1
    assert options.headless is True
    assert dart.gui.experimental.should_request_screenshot(options, 0, False)
    assert not dart.gui.experimental.should_request_screenshot(options, 0, True)
    assert dart.gui.experimental.should_stop_after_frame(options, 1)

    lifecycle = dart.gui.experimental.ViewerLifecycleState()
    assert dart.gui.experimental.should_advance_simulation(lifecycle)
    dart.gui.experimental.toggle_paused(lifecycle)
    assert lifecycle.paused is True
    assert not dart.gui.experimental.should_advance_simulation(lifecycle)
    dart.gui.experimental.request_single_step(lifecycle)
    assert lifecycle.paused is True
    assert dart.gui.experimental.should_advance_simulation(lifecycle)
    dart.gui.experimental.mark_simulation_advanced(lifecycle)
    assert not lifecycle.step_once
    assert dart.gui.experimental.should_request_screenshot(options, lifecycle)
    dart.gui.experimental.mark_screenshot_requested(lifecycle)
    assert not dart.gui.experimental.should_request_screenshot(options, lifecycle)
    dart.gui.experimental.mark_frame_skipped(lifecycle)
    assert lifecycle.skipped_frames == 1
    dart.gui.experimental.mark_frame_rendered(lifecycle)
    assert lifecycle.rendered_frames == 1
    assert lifecycle.skipped_frames == 0
    assert dart.gui.experimental.should_stop_after_frame(options, lifecycle)

    window_only = dart.gui.experimental.RunOptions()
    window_only.gui_scale = 10.0
    dart.gui.experimental.normalize_run_options(window_only)
    assert np.isclose(window_only.gui_scale, 4.0)

    camera = dart.gui.experimental.OrbitCamera()
    camera.target = np.zeros(3)
    camera.yaw = 0.0
    camera.pitch = 0.0
    camera.distance = 2.0

    basis = dart.gui.experimental.make_orbit_camera_basis(camera)
    assert np.allclose(basis.eye, [2.0, 0.0, 0.0])
    assert np.allclose(basis.forward, [-1.0, 0.0, 0.0])

    nudge_input = dart.gui.experimental.DirectionalNudgeInput()
    nudge_input.right = True
    nudge_input.forward = True
    nudge_input.up = True
    nudge_input.fast = True
    nudge_input.step_size = 0.25
    nudge_input.fast_multiplier = 2.0
    nudge = dart.gui.experimental.compute_camera_relative_nudge(
        camera, nudge_input
    )
    assert np.allclose(nudge, [-0.5, 0.5, 0.5])

    nudge_input.step_size = float("nan")
    nudge = dart.gui.experimental.compute_camera_relative_nudge(
        camera, nudge_input
    )
    assert np.allclose(nudge, np.zeros(3))

    controller = dart.gui.experimental.OrbitCameraController()
    controller.camera = camera
    controller_input = dart.gui.experimental.OrbitCameraControllerInput()
    controller_input.cursor_x = 100.0
    controller_input.cursor_y = 50.0
    dart.gui.experimental.update_orbit_camera_controller(
        controller, controller_input
    )
    assert controller.has_last_cursor is True
    assert np.isclose(controller.last_cursor_x, 100.0)
    assert np.isclose(controller.last_cursor_y, 50.0)
    assert np.allclose(controller.camera.target, np.zeros(3))

    dart.gui.experimental.add_orbit_camera_scroll(controller, 1.0)
    controller_input.cursor_x = 110.0
    controller_input.cursor_y = 70.0
    controller_input.pan = True
    dart.gui.experimental.update_orbit_camera_controller(
        controller, controller_input
    )
    assert np.isclose(controller.scroll_delta, 0.0)
    assert np.allclose(controller.camera.target, [0.0, -0.03, 0.06])
    assert controller.camera.distance < 2.0

    controller_input.has_cursor = False
    dart.gui.experimental.update_orbit_camera_controller(
        controller, controller_input
    )
    assert controller.has_last_cursor is False

    ray = dart.gui.experimental.make_perspective_pick_ray(camera, 320, 240, 640, 480)
    assert np.allclose(ray.origin, basis.eye)
    assert np.allclose(ray.direction, basis.forward)

    update = dart.gui.experimental.OrbitCameraUpdate()
    update.scroll_delta = 100.0
    dart.gui.experimental.update_orbit_camera(camera, update)
    assert np.isclose(camera.distance, update.min_distance)

    camera.distance = 2.0
    projection = dart.gui.experimental.make_perspective_projection(camera, 640, 480)
    assert np.isclose(projection.vertical_fov_degrees, 45.0)
    assert np.isclose(projection.aspect_ratio, 4.0 / 3.0)
    assert np.isclose(projection.near_plane, 0.008)
    assert np.isclose(projection.far_plane, 37.0)

    projection_options = dart.gui.experimental.ProjectionOptions()
    projection_options.vertical_fov_degrees = float("nan")
    projection_options.near_plane = 0.1
    projection_options.far_plane = 0.05
    projection_options.min_far_plane = 10.0
    projection_options.far_padding = 5.0
    override_projection = dart.gui.experimental.make_perspective_projection(
        camera, 0, -20, projection_options
    )
    assert np.isclose(override_projection.vertical_fov_degrees, 45.0)
    assert np.isclose(override_projection.aspect_ratio, 1.0)
    assert np.isclose(override_projection.near_plane, 0.1)
    assert np.isclose(override_projection.far_plane, 10.0)


def test_experimental_write_rgba_ppm(tmp_path):
    path = tmp_path / "capture.ppm"
    rgba_pixels = [
        255,
        0,
        0,
        11,
        0,
        255,
        0,
        12,
        0,
        0,
        255,
        13,
        255,
        255,
        255,
        14,
    ]

    dart.gui.experimental.write_rgba_ppm(
        str(path), 2, 2, rgba_pixels, origin_bottom_left=True
    )

    assert path.read_bytes() == (
        b"P6\n2 2\n255\n"
        + bytes([0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 255, 0])
    )

    with pytest.raises(RuntimeError):
        dart.gui.experimental.write_rgba_ppm(
            str(path), 2, 2, [255, 0, 0, 255], origin_bottom_left=False
        )
