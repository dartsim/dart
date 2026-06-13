from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

requires_gui_bindings = pytest.mark.skipif(
    not hasattr(dart.gui, "describe_shape")
    or not hasattr(dart.gui, "RenderableDescriptor"),
    reason="dartpy GUI bindings are not built in this configuration",
)


def _renderable_from_shape(
    shape,
    *,
    renderable_id: int = 1,
    name: str = "shape",
    transform=None,
    rgba=(1.0, 1.0, 1.0, 1.0),
):
    geometry = dart.gui.describe_shape(shape)
    assert geometry is not None
    descriptor = dart.gui.RenderableDescriptor()
    descriptor.id = renderable_id
    descriptor.shape_frame_name = name
    descriptor.shape_node_name = name
    descriptor.geometry = geometry
    material = dart.gui.MaterialDescriptor()
    material.rgba = np.asarray(rgba, dtype=float)
    material.visible = True
    descriptor.material = material
    if transform is not None:
        descriptor.world_transform = np.asarray(transform, dtype=float)
    return descriptor


def test_gui_stub_surface_is_backend_hidden():
    repo_root = Path(__file__).resolve().parents[4]
    gui_stub = repo_root / "python" / "stubs" / "dartpy" / "gui" / "__init__.pyi"
    gui_docs = repo_root / "docs" / "python_api" / "modules" / "gui.rst"
    rtd_conf = repo_root / "docs" / "readthedocs" / "conf.py"

    gui_stub_text = gui_stub.read_text()
    assert "experimental" not in gui_stub_text
    assert "automodule:: dartpy.gui" in gui_docs.read_text()
    assert "dartpy.gui.experimental" not in gui_docs.read_text()
    assert "dartpy.gui" in rtd_conf.read_text()
    assert "dartpy.gui.experimental" not in rtd_conf.read_text()
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
        assert token not in gui_stub_text


@requires_gui_bindings
def test_gui_extract_renderables_from_world():
    shape = dart.BoxShape(np.array([1.0, 2.0, 3.0]))
    renderables = [_renderable_from_shape(shape, name="box")]

    assert len(renderables) == 1
    descriptor = renderables[0]
    assert descriptor.geometry.kind == dart.gui.ShapeKind.Box
    assert hasattr(dart.gui.ShapeKind, "Pyramid")
    assert hasattr(dart.gui.ShapeKind, "MultiSphere")
    assert hasattr(dart.gui.ShapeKind, "LineSegments")
    assert hasattr(dart.gui.ShapeKind, "ConvexMesh")
    assert hasattr(dart.gui.ShapeKind, "PointCloud")
    assert hasattr(dart.gui.ShapeKind, "Heightmap")
    assert hasattr(dart.gui.ShapeKind, "SoftMesh")
    assert hasattr(dart.gui.ShapeKind, "VoxelGrid")
    assert hasattr(dart.gui, "MeshAlphaMode")
    assert hasattr(dart.gui.MeshAlphaMode, "ShapeAlpha")
    geometry = dart.gui.GeometryDescriptor()
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
    material_descriptor = dart.gui.MeshMaterialDescriptor()
    assert hasattr(material_descriptor, "base_color_texture_path")
    assert hasattr(material_descriptor, "metallic_roughness_texture_path")
    material = dart.gui.MaterialDescriptor()
    assert material.metallic is None
    assert material.roughness is None
    assert material.reflectance is None
    material.metallic = 0.7
    material.roughness = 0.25
    material.reflectance = 0.45
    assert material.metallic == pytest.approx(0.7)
    assert material.roughness == pytest.approx(0.25)
    assert material.reflectance == pytest.approx(0.45)
    material.metallic = None
    assert material.metallic is None
    line = dart.gui.DebugLineDescriptor()
    line.thickness = 3.0
    assert line.thickness == pytest.approx(3.0)
    triangle = dart.gui.DebugTriangleDescriptor()
    triangle.b = np.array([1.0, 0.0, 0.0])
    assert np.allclose(triangle.b, [1.0, 0.0, 0.0])
    label = dart.gui.DebugLabelDescriptor()
    label.text = "debug label"
    assert label.text == "debug label"
    debug_scene = dart.gui.DebugScene()
    debug_scene.lines = [line]
    debug_scene.triangles = [triangle]
    debug_scene.labels = [label]
    assert len(debug_scene.lines) == 1
    assert len(debug_scene.triangles) == 1
    assert len(debug_scene.labels) == 1
    active_state = dart.gui.ActiveRenderableState()
    assert hasattr(active_state, "shape_version")
    assert hasattr(active_state, "render_resource_version")
    part_descriptor = dart.gui.MeshPartDescriptor()
    assert hasattr(part_descriptor, "triangle_count")
    assert hasattr(part_descriptor, "material_index")
    assert hasattr(descriptor, "render_resource_version")
    assert np.allclose(descriptor.geometry.size, [1.0, 2.0, 3.0])
    assert descriptor.material.visible is True

    selection_lines = dart.gui.make_selection_debug_lines(descriptor)
    assert len(selection_lines) == 12
    assert selection_lines[0].label == "selection.bounds"

    ray = dart.gui.PickRay()
    ray.origin = np.array([-2.0, 0.0, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.pick_nearest_renderable(renderables, ray)
    assert hit is not None
    assert np.allclose(hit.point, [-0.5, 0.0, 0.0])
    assert np.allclose(hit.normal, [-1.0, 0.0, 0.0])


@requires_gui_bindings
def test_gui_plan_renderable_set_update():
    visible_a = dart.gui.RenderableDescriptor()
    visible_a.id = 1
    visible_a.material.visible = True

    hidden = dart.gui.RenderableDescriptor()
    hidden.id = 2
    hidden.material.visible = False

    visible_b = dart.gui.RenderableDescriptor()
    visible_b.id = 3
    visible_b.material.visible = True

    duplicate_visible_a = dart.gui.RenderableDescriptor()
    duplicate_visible_a.id = visible_a.id
    duplicate_visible_a.material.visible = True

    invalid_id = dart.gui.RenderableDescriptor()
    invalid_id.id = 0
    invalid_id.material.visible = True

    plan = dart.gui.plan_renderable_set_update(
        [visible_a, hidden, visible_b, duplicate_visible_a, invalid_id],
        [visible_a.id, hidden.id, 4, visible_a.id, 0],
    )

    assert plan.descriptor_indices_to_add == [2]
    assert plan.active_renderable_indices_to_remove == [1, 2, 3, 4]

    visible_a.shape_version = 2
    visible_a.render_resource_version = 20
    visible_b.shape_version = 4
    visible_b.render_resource_version = 40
    stale_a = dart.gui.ActiveRenderableState()
    stale_a.id = visible_a.id
    stale_a.shape_version = visible_a.shape_version
    stale_a.render_resource_version = 10
    current_b = dart.gui.ActiveRenderableState()
    current_b.id = visible_b.id
    current_b.shape_version = visible_b.shape_version
    current_b.render_resource_version = visible_b.render_resource_version

    versioned_plan = dart.gui.plan_renderable_set_update(
        [visible_a, visible_b],
        [stale_a, current_b],
    )

    assert versioned_plan.descriptor_indices_to_add == [0]
    assert versioned_plan.active_renderable_indices_to_remove == [0]


@requires_gui_bindings
def test_gui_pick_sphere_uses_surface_normal():
    renderables = [_renderable_from_shape(dart.SphereShape(1.0), name="sphere")]
    ray = dart.gui.PickRay()
    ray.origin = np.array([-2.0, 0.5, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.pick_nearest_renderable(renderables, ray)

    expected_x = -np.sqrt(0.75)
    assert hit is not None
    assert np.isclose(hit.distance, 2.0 + expected_x)
    assert np.allclose(hit.point, [expected_x, 0.5, 0.0])
    assert np.allclose(hit.normal, [expected_x, 0.5, 0.0])


@requires_gui_bindings
def test_gui_pick_multi_sphere_uses_surface_normal():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.MultiSphere
    renderable.geometry.sphere_centers = [
        np.array([-1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    ]
    renderable.geometry.sphere_radii = [0.5, 0.25]
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.5, -0.5, -0.5])
    renderable.geometry.local_bounds_max = np.array([1.25, 0.5, 0.5])

    ray = dart.gui.PickRay()
    ray.origin = np.array([-3.0, 0.25, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.pick_nearest_renderable([renderable], ray)

    expected_x = -1.0 - np.sqrt(0.1875)
    expected_normal = np.array([expected_x + 1.0, 0.25, 0.0])
    expected_normal /= np.linalg.norm(expected_normal)
    assert hit is not None
    assert np.isclose(hit.distance, 3.0 + expected_x)
    assert np.allclose(hit.point, [expected_x, 0.25, 0.0])
    assert np.allclose(hit.normal, expected_normal)


@requires_gui_bindings
def test_gui_pick_cylinder_uses_surface_normal():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.Cylinder
    renderable.geometry.radius = 1.0
    renderable.geometry.height = 2.0
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -1.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 1.0])

    ray = dart.gui.PickRay()
    ray.origin = np.array([-2.0, 0.5, 0.0])
    ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.pick_nearest_renderable([renderable], ray)

    expected_x = -np.sqrt(0.75)
    assert hit is not None
    assert np.isclose(hit.distance, 2.0 + expected_x)
    assert np.allclose(hit.point, [expected_x, 0.5, 0.0])
    assert np.allclose(hit.normal, [expected_x, 0.5, 0.0])


@requires_gui_bindings
def test_gui_pick_capsule_uses_surface_normal():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.Capsule
    renderable.geometry.radius = 1.0
    renderable.geometry.height = 2.0
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -2.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 2.0])

    side_ray = dart.gui.PickRay()
    side_ray.origin = np.array([-2.0, 0.5, 0.0])
    side_ray.direction = np.array([1.0, 0.0, 0.0])
    side_hit = dart.gui.pick_nearest_renderable([renderable], side_ray)

    side_x = -np.sqrt(0.75)
    assert side_hit is not None
    assert np.isclose(side_hit.distance, 2.0 + side_x)
    assert np.allclose(side_hit.point, [side_x, 0.5, 0.0])
    assert np.allclose(side_hit.normal, [side_x, 0.5, 0.0])

    cap_ray = dart.gui.PickRay()
    cap_ray.origin = np.array([0.5, 0.0, 4.0])
    cap_ray.direction = np.array([0.0, 0.0, -1.0])
    cap_hit = dart.gui.pick_nearest_renderable([renderable], cap_ray)

    cap_z = 1.0 + np.sqrt(0.75)
    assert cap_hit is not None
    assert np.isclose(cap_hit.distance, 4.0 - cap_z)
    assert np.allclose(cap_hit.point, [0.5, 0.0, cap_z])
    assert np.allclose(cap_hit.normal, [0.5, 0.0, np.sqrt(0.75)])


@requires_gui_bindings
def test_gui_pick_cone_uses_surface_normal():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.Cone
    renderable.geometry.radius = 1.0
    renderable.geometry.height = 2.0
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -1.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 1.0])

    side_ray = dart.gui.PickRay()
    side_ray.origin = np.array([-2.0, 0.25, 0.0])
    side_ray.direction = np.array([1.0, 0.0, 0.0])
    side_hit = dart.gui.pick_nearest_renderable([renderable], side_ray)

    side_x = -np.sqrt(0.1875)
    side_normal = np.array([side_x, 0.25, 0.25])
    side_normal /= np.linalg.norm(side_normal)
    assert side_hit is not None
    assert np.isclose(side_hit.distance, 2.0 + side_x)
    assert np.allclose(side_hit.point, [side_x, 0.25, 0.0])
    assert np.allclose(side_hit.normal, side_normal)

    base_ray = dart.gui.PickRay()
    base_ray.origin = np.array([0.5, 0.0, -4.0])
    base_ray.direction = np.array([0.0, 0.0, 1.0])
    base_hit = dart.gui.pick_nearest_renderable([renderable], base_ray)

    assert base_hit is not None
    assert np.isclose(base_hit.distance, 3.0)
    assert np.allclose(base_hit.point, [0.5, 0.0, -1.0])
    assert np.allclose(base_hit.normal, [0.0, 0.0, -1.0])


@requires_gui_bindings
def test_gui_pick_pyramid_uses_surface_normal():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.Pyramid
    renderable.geometry.size = np.array([2.0, 2.0, 2.0])
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, -1.0])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 1.0])

    side_ray = dart.gui.PickRay()
    side_ray.origin = np.array([0.0, -3.0, 0.0])
    side_ray.direction = np.array([0.0, 1.0, 0.0])
    side_hit = dart.gui.pick_nearest_renderable([renderable], side_ray)

    side_normal = np.array([0.0, -2.0, 1.0])
    side_normal /= np.linalg.norm(side_normal)
    assert side_hit is not None
    assert np.isclose(side_hit.distance, 2.5)
    assert np.allclose(side_hit.point, [0.0, -0.5, 0.0])
    assert np.allclose(side_hit.normal, side_normal)

    base_ray = dart.gui.PickRay()
    base_ray.origin = np.array([0.5, 0.25, -4.0])
    base_ray.direction = np.array([0.0, 0.0, 1.0])
    base_hit = dart.gui.pick_nearest_renderable([renderable], base_ray)

    assert base_hit is not None
    assert np.isclose(base_hit.distance, 3.0)
    assert np.allclose(base_hit.point, [0.5, 0.25, -1.0])
    assert np.allclose(base_hit.normal, [0.0, 0.0, -1.0])


@requires_gui_bindings
def test_gui_pick_plane_uses_finite_proxy_surface():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.Plane
    renderable.geometry.normal = np.array([0.0, 0.0, 1.0])
    renderable.geometry.offset = 0.25
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-1.0, -1.0, 0.23])
    renderable.geometry.local_bounds_max = np.array([1.0, 1.0, 0.27])

    hit_ray = dart.gui.PickRay()
    hit_ray.origin = np.array([0.25, 0.5, 2.0])
    hit_ray.direction = np.array([0.0, 0.0, -1.0])
    hit = dart.gui.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.75)
    assert np.allclose(hit.point, [0.25, 0.5, 0.25])
    assert np.allclose(hit.normal, [0.0, 0.0, 1.0])

    miss_ray = dart.gui.PickRay()
    miss_ray.origin = np.array([1.5, 0.0, 2.0])
    miss_ray.direction = np.array([0.0, 0.0, -1.0])
    assert dart.gui.pick_nearest_renderable([renderable], miss_ray) is None


@requires_gui_bindings
def test_gui_pick_triangle_mesh_uses_surface_triangles():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.Mesh
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

    hit_ray = dart.gui.PickRay()
    hit_ray.origin = np.array([0.25, 0.25, 1.0])
    hit_ray.direction = np.array([0.0, 0.0, -1.0])
    hit = dart.gui.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.0)
    assert np.allclose(hit.point, [0.25, 0.25, 0.0])
    assert np.allclose(hit.normal, [0.0, 0.0, 1.0])

    miss_ray = dart.gui.PickRay()
    miss_ray.origin = np.array([0.75, 0.75, 1.0])
    miss_ray.direction = np.array([0.0, 0.0, -1.0])
    assert dart.gui.pick_nearest_renderable([renderable], miss_ray) is None


@requires_gui_bindings
def test_gui_pick_point_cloud_uses_per_point_box_surface():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.PointCloud
    renderable.geometry.point_cloud_points = [
        np.array([0.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
    ]
    renderable.geometry.point_size = 0.2
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-0.1, -0.1, -0.1])
    renderable.geometry.local_bounds_max = np.array([2.1, 0.1, 0.1])

    hit_ray = dart.gui.PickRay()
    hit_ray.origin = np.array([-2.0, 0.05, 0.0])
    hit_ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.9)
    assert np.allclose(hit.point, [-0.1, 0.05, 0.0])
    assert np.allclose(hit.normal, [-1.0, 0.0, 0.0])

    gap_ray = dart.gui.PickRay()
    gap_ray.origin = np.array([1.0, 0.0, -1.0])
    gap_ray.direction = np.array([0.0, 0.0, 1.0])
    assert dart.gui.pick_nearest_renderable([renderable], gap_ray) is None


@requires_gui_bindings
def test_gui_pick_voxel_grid_uses_per_voxel_box_surface():
    renderable = dart.gui.RenderableDescriptor()
    renderable.id = 1
    renderable.geometry.kind = dart.gui.ShapeKind.VoxelGrid
    renderable.geometry.voxel_centers = [
        np.array([0.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
    ]
    renderable.geometry.voxel_size = 0.2
    renderable.geometry.has_local_bounds = True
    renderable.geometry.local_bounds_min = np.array([-0.1, -0.1, -0.1])
    renderable.geometry.local_bounds_max = np.array([2.1, 0.1, 0.1])

    hit_ray = dart.gui.PickRay()
    hit_ray.origin = np.array([-2.0, 0.05, 0.0])
    hit_ray.direction = np.array([1.0, 0.0, 0.0])
    hit = dart.gui.pick_nearest_renderable([renderable], hit_ray)

    assert hit is not None
    assert np.isclose(hit.distance, 1.9)
    assert np.allclose(hit.point, [-0.1, 0.05, 0.0])
    assert np.allclose(hit.normal, [-1.0, 0.0, 0.0])

    gap_ray = dart.gui.PickRay()
    gap_ray.origin = np.array([1.0, 0.0, -1.0])
    gap_ray.direction = np.array([0.0, 0.0, 1.0])
    assert dart.gui.pick_nearest_renderable([renderable], gap_ray) is None


@requires_gui_bindings
def test_gui_extract_renderables_from_simple_frame():
    scene = dart.gui.DescriptorRenderScene(dart.World(), "world")
    transform = dart.Isometry3()
    transform.set_translation(np.array([1.0, -2.0, 0.5]))
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), "interactive_target", transform
    )
    frame.set_shape(dart.BoxShape(np.array([0.4, 0.5, 0.6])))
    visual = frame.get_visual_aspect(True)
    visual.set_rgba(np.array([0.9, 0.2, 0.1, 0.8]))
    visual.set_metallic(0.65)
    visual.set_roughness(0.2)
    visual.set_reflectance(-1.0)
    scene.add_simple_frame(frame)

    renderables = scene.renderable_provider()

    assert len(renderables) == 1
    descriptor = renderables[0]
    assert descriptor.skeleton_name == ""
    assert descriptor.body_name == ""
    assert descriptor.shape_frame_name == "interactive_target"
    assert descriptor.shape_node_name == "interactive_target"
    assert descriptor.shape_node_version > 0
    assert descriptor.geometry.kind == dart.gui.ShapeKind.Box
    assert np.allclose(descriptor.geometry.size, [0.4, 0.5, 0.6])
    assert descriptor.material.metallic == pytest.approx(0.65)
    assert descriptor.material.roughness == pytest.approx(0.2)
    assert descriptor.material.reflectance is None
    assert np.allclose(descriptor.world_transform.translation(), [1.0, -2.0, 0.5])

    assert not dart.gui.translate_simple_frame_renderable(
        descriptor, np.array([0.25, 0.5, -0.1])
    )
    frame.set_transform(transform)
    refreshed = scene.renderable_provider()
    assert np.allclose(refreshed[0].world_transform.translation(), [1.0, -2.0, 0.5])


@requires_gui_bindings
def test_gui_debug_grid_lines():
    options = dart.gui.DebugDrawOptions()
    options.grid_half_extent = 1.0
    options.grid_spacing = 1.0

    lines = dart.gui.make_grid_debug_lines(options)

    assert len(lines) == 6
    assert lines[0].label == "grid"
    assert np.allclose(lines[0].from_point, [-1.0, -1.0, 0.08])
    assert np.allclose(lines[0].to_point, [1.0, -1.0, 0.08])


@requires_gui_bindings
def test_gui_translate_free_joint_renderable():
    renderable = _renderable_from_shape(
        dart.BoxShape(np.array([1.0, 1.0, 1.0])),
        name="descriptor_only_box",
    )

    assert not dart.gui.translate_free_joint_renderable(
        renderable, np.array([0.25, -0.5, 0.75])
    )
    assert not dart.gui.translate_free_joint_renderable(
        renderable, np.array([np.nan, 0.0, 0.0])
    )


@requires_gui_bindings
def test_gui_plane_drag_helpers():
    previous_ray = dart.gui.PickRay()
    previous_ray.origin = np.array([0.0, 0.0, 1.0])
    previous_ray.direction = np.array([0.0, 0.0, -1.0])
    current_ray = dart.gui.PickRay()
    current_ray.origin = np.array([1.0, 2.0, 1.0])
    current_ray.direction = np.array([0.0, 0.0, -1.0])

    intersection = dart.gui.intersect_plane(
        previous_ray, np.zeros(3), np.array([0.0, 0.0, 1.0])
    )
    assert np.allclose(intersection, [0.0, 0.0, 0.0])

    translation = dart.gui.compute_plane_drag_translation(
        previous_ray, current_ray, np.zeros(3), np.array([0.0, 0.0, 1.0])
    )
    assert np.allclose(translation, [1.0, 2.0, 0.0])

    parallel_ray = dart.gui.PickRay()
    parallel_ray.origin = np.array([0.0, 0.0, 1.0])
    parallel_ray.direction = np.array([1.0, 0.0, 0.0])
    assert (
        dart.gui.intersect_plane(parallel_ray, np.zeros(3), np.array([0.0, 0.0, 1.0]))
        is None
    )


@requires_gui_bindings
def test_gui_axis_drag_helpers():
    previous_ray = dart.gui.PickRay()
    previous_ray.origin = np.array([0.0, 0.0, 1.0])
    previous_ray.direction = np.array([0.0, 0.0, -1.0])
    current_ray = dart.gui.PickRay()
    current_ray.origin = np.array([0.0, 1.0, 1.0])
    current_ray.direction = np.array([1.0, -1.0, -1.0])

    translation = dart.gui.compute_axis_drag_translation(
        previous_ray, current_ray, np.zeros(3), np.array([1.0, 0.0, 0.0])
    )
    assert np.allclose(translation, [1.0, 0.0, 0.0])

    parallel_ray = dart.gui.PickRay()
    parallel_ray.origin = np.array([-1.0, 0.0, 0.0])
    parallel_ray.direction = np.array([1.0, 0.0, 0.0])
    assert (
        dart.gui.compute_axis_drag_translation(
            parallel_ray, current_ray, np.zeros(3), np.array([1.0, 0.0, 0.0])
        )
        is None
    )


@requires_gui_bindings
def test_gui_center_of_mass_debug_lines():
    options = dart.gui.DebugDrawOptions()
    options.draw_grid = False
    options.draw_world_frame = False
    options.draw_contacts = False
    options.draw_centers_of_mass = True
    options.center_of_mass_marker_radius = 0.2

    assert options.draw_centers_of_mass is True
    assert np.isclose(options.center_of_mass_marker_radius, 0.2)
    assert dart.gui.extract_debug_lines(options) == []


@requires_gui_bindings
def test_gui_inertia_debug_options():
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    body.get_inertia().set_mass(12.0)
    body.get_inertia().set_moment(np.diag([52.0, 40.0, 20.0]))

    options = dart.gui.DebugDrawOptions()
    options.draw_grid = False
    options.draw_world_frame = False
    options.draw_contacts = False
    options.draw_inertia_boxes = True
    options.inertia_box_scale = 0.5

    assert options.draw_inertia_boxes is True
    assert np.isclose(options.inertia_box_scale, 0.5)
    lines = dart.gui.make_inertia_debug_lines(body, options, "robot/body")
    assert len(lines) == 12
    assert lines[0].label == "robot/body.inertia"


@requires_gui_bindings
def test_gui_collision_shape_debug_lines():
    skeleton = dart.Skeleton("robot")
    _, body = skeleton.create_free_joint_and_body_node_pair()
    shape_node = body.create_shape_node(dart.BoxShape(np.array([2.0, 4.0, 6.0])))
    shape_node.create_collision_aspect()

    options = dart.gui.DebugDrawOptions()
    options.draw_grid = False
    options.draw_world_frame = False
    options.draw_contacts = False
    options.draw_collision_shape_bounds = True
    options.collision_bounds_padding = 0.1

    assert options.draw_collision_shape_bounds is True
    assert np.isclose(options.collision_bounds_padding, 0.1)
    lines = dart.gui.make_collision_shape_debug_lines(
        shape_node, options, "robot/body/collision"
    )
    assert len(lines) == 12
    assert lines[0].label == "robot/body/collision.collision_bounds"


@requires_gui_bindings
def test_gui_support_polygon_debug_options():
    skeleton = dart.Skeleton("supportless")
    options = dart.gui.DebugDrawOptions()
    options.draw_support_polygons = True
    options.draw_support_centroids = True
    options.support_polygon_elevation = 0.03
    options.support_centroid_marker_radius = 0.04

    lines = dart.gui.make_support_polygon_debug_lines(skeleton, options, "supportless")

    assert len(lines) == 0


@requires_gui_bindings
def test_gui_joint_axis_and_velocity_debug_lines():
    skeleton = dart.Skeleton("debug_helpers")
    joint, body = skeleton.create_revolute_joint_and_body_node_pair()
    joint.set_axis(np.array([0.0, 0.0, 1.0]))

    options = dart.gui.DebugDrawOptions()
    options.draw_joint_axes = True
    options.joint_axis_length = 0.4
    assert options.draw_joint_axes is True
    assert np.isclose(options.joint_axis_length, 0.4)

    axis_lines = dart.gui.make_joint_axis_debug_lines(body, options, "debug")
    assert len(axis_lines) == 1
    assert axis_lines[0].label == "debug.joint_axis"
    assert np.isclose(
        np.linalg.norm(axis_lines[0].to_point - axis_lines[0].from_point), 0.8
    )

    options.draw_joint_axes = False
    assert dart.gui.make_joint_axis_debug_lines(body, options) == []

    joint.set_velocities(np.array([2.0]))
    options.draw_angular_velocities = True
    options.angular_velocity_scale = 0.25
    options.velocity_min_length = 0.02
    options.velocity_max_length = 1.0
    angular_lines = dart.gui.make_velocity_debug_lines(body, options, "debug")
    assert len(angular_lines) > 0
    assert angular_lines[0].label == "debug.vel_angular"

    free_joint, free_body = skeleton.create_free_joint_and_body_node_pair()
    velocities = np.zeros(6)
    velocities[3] = 2.0
    free_joint.set_velocities(velocities)
    options.draw_angular_velocities = False
    options.draw_linear_velocities = True
    options.linear_velocity_scale = 0.5
    linear_lines = dart.gui.make_velocity_debug_lines(free_body, options, "debug")
    assert len(linear_lines) > 0
    assert linear_lines[0].label == "debug.vel_linear"


@requires_gui_bindings
def test_gui_camera_and_run_helpers():
    options = dart.gui.RunOptions()
    options.width = 0
    options.height = -3
    options.gui_scale = float("nan")
    options.headless = True
    options.screenshot_path = "capture.ppm"

    dart.gui.normalize_run_options(options)

    assert options.width == 1
    assert options.height == 1
    assert np.isclose(options.gui_scale, 1.0)
    assert options.max_frames == 1
    assert options.headless is True
    assert dart.gui.should_request_screenshot(options, 0, False)
    assert not dart.gui.should_capture_frame_output(options)
    assert not dart.gui.should_request_screenshot(options, 0, True)
    assert dart.gui.should_stop_after_frame(options, 1)

    lifecycle = dart.gui.ViewerLifecycleState()
    assert dart.gui.should_advance_simulation(lifecycle)
    dart.gui.toggle_paused(lifecycle)
    assert lifecycle.paused is True
    assert not dart.gui.should_advance_simulation(lifecycle)
    dart.gui.request_single_step(lifecycle)
    assert lifecycle.paused is True
    assert dart.gui.should_advance_simulation(lifecycle)
    dart.gui.mark_simulation_advanced(lifecycle)
    assert not lifecycle.step_once
    assert dart.gui.should_request_screenshot(options, lifecycle)
    dart.gui.mark_screenshot_requested(lifecycle)
    assert not dart.gui.should_request_screenshot(options, lifecycle)
    dart.gui.mark_frame_skipped(lifecycle)
    assert lifecycle.skipped_frames == 1
    dart.gui.mark_frame_rendered(lifecycle)
    assert lifecycle.rendered_frames == 1
    assert lifecycle.skipped_frames == 0
    assert dart.gui.should_stop_after_frame(options, lifecycle)

    window_only = dart.gui.RunOptions()
    window_only.gui_scale = 10.0
    dart.gui.normalize_run_options(window_only)
    assert np.isclose(window_only.gui_scale, 4.0)

    sequence_output = dart.gui.RunOptions()
    sequence_output.frame_output_directory = "frames"
    dart.gui.normalize_run_options(sequence_output)
    assert sequence_output.max_frames == 1
    assert dart.gui.should_capture_frame_output(sequence_output)
    assert (
        Path(dart.gui.make_frame_output_path(sequence_output, 7)).name
        == "frame_000007.ppm"
    )

    camera = dart.gui.OrbitCamera()
    camera.target = np.zeros(3)
    camera.yaw = 0.0
    camera.pitch = 0.0
    camera.distance = 2.0

    basis = dart.gui.make_orbit_camera_basis(camera)
    assert np.allclose(basis.eye, [2.0, 0.0, 0.0])
    assert np.allclose(basis.forward, [-1.0, 0.0, 0.0])

    nudge_input = dart.gui.DirectionalNudgeInput()
    nudge_input.right = True
    nudge_input.forward = True
    nudge_input.up = True
    nudge_input.fast = True
    nudge_input.step_size = 0.25
    nudge_input.fast_multiplier = 2.0
    nudge = dart.gui.compute_camera_relative_nudge(camera, nudge_input)
    assert np.allclose(nudge, [-0.5, 0.5, 0.5])

    nudge_input.step_size = float("nan")
    nudge = dart.gui.compute_camera_relative_nudge(camera, nudge_input)
    assert np.allclose(nudge, np.zeros(3))

    controller = dart.gui.OrbitCameraController()
    controller.camera = camera
    controller_input = dart.gui.OrbitCameraControllerInput()
    controller_input.cursor_x = 100.0
    controller_input.cursor_y = 50.0
    dart.gui.update_orbit_camera_controller(controller, controller_input)
    assert controller.has_last_cursor is True
    assert np.isclose(controller.last_cursor_x, 100.0)
    assert np.isclose(controller.last_cursor_y, 50.0)
    assert np.allclose(controller.camera.target, np.zeros(3))

    dart.gui.add_orbit_camera_scroll(controller, 1.0)
    controller_input.cursor_x = 110.0
    controller_input.cursor_y = 70.0
    controller_input.pan = True
    dart.gui.update_orbit_camera_controller(controller, controller_input)
    assert np.isclose(controller.scroll_delta, 0.0)
    assert np.allclose(controller.camera.target, [0.0, -0.03, 0.06])
    assert controller.camera.distance < 2.0

    controller_input.has_cursor = False
    dart.gui.update_orbit_camera_controller(controller, controller_input)
    assert controller.has_last_cursor is False

    ray = dart.gui.make_perspective_pick_ray(camera, 320, 240, 640, 480)
    assert np.allclose(ray.origin, basis.eye)
    assert np.allclose(ray.direction, basis.forward)

    update = dart.gui.OrbitCameraUpdate()
    update.scroll_delta = 100.0
    dart.gui.update_orbit_camera(camera, update)
    assert np.isclose(camera.distance, update.min_distance)

    camera.distance = 2.0
    projection = dart.gui.make_perspective_projection(camera, 640, 480)
    assert np.isclose(projection.vertical_fov_degrees, 45.0)
    assert np.isclose(projection.aspect_ratio, 4.0 / 3.0)
    assert np.isclose(projection.near_plane, 0.002)
    assert np.isclose(projection.far_plane, 37.0)

    projection_options = dart.gui.ProjectionOptions()
    projection_options.vertical_fov_degrees = float("nan")
    projection_options.near_plane = 0.1
    projection_options.far_plane = 0.05
    projection_options.min_far_plane = 10.0
    projection_options.far_padding = 5.0
    override_projection = dart.gui.make_perspective_projection(
        camera, 0, -20, projection_options
    )
    assert np.isclose(override_projection.vertical_fov_degrees, 45.0)
    assert np.isclose(override_projection.aspect_ratio, 1.0)
    assert np.isclose(override_projection.near_plane, 0.1)
    assert np.isclose(override_projection.far_plane, 10.0)


@requires_gui_bindings
def test_gui_write_rgba_ppm(tmp_path):
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

    dart.gui.write_rgba_ppm(str(path), 2, 2, rgba_pixels, origin_bottom_left=True)

    assert path.read_bytes() == (
        b"P6\n2 2\n255\n" + bytes([0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 255, 0])
    )

    with pytest.raises(RuntimeError):
        dart.gui.write_rgba_ppm(
            str(path), 2, 2, [255, 0, 0, 255], origin_bottom_left=False
        )
