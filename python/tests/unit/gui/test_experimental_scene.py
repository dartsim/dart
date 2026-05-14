import numpy as np
import pytest

import dartpy as dart


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
    assert np.allclose(descriptor.geometry.size, [1.0, 2.0, 3.0])
    assert descriptor.material.visible is True

    selection_lines = dart.gui.experimental.make_selection_debug_lines(descriptor)
    assert len(selection_lines) == 12
    assert selection_lines[0].label == "selection.bounds"


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


def test_experimental_camera_and_run_helpers():
    options = dart.gui.experimental.RunOptions()
    options.width = 0
    options.height = -3
    options.headless = True
    options.screenshot_path = "capture.ppm"

    dart.gui.experimental.normalize_run_options(options)

    assert options.width == 1
    assert options.height == 1
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

    camera = dart.gui.experimental.OrbitCamera()
    camera.target = np.zeros(3)
    camera.yaw = 0.0
    camera.pitch = 0.0
    camera.distance = 2.0

    basis = dart.gui.experimental.make_orbit_camera_basis(camera)
    assert np.allclose(basis.eye, [2.0, 0.0, 0.0])
    assert np.allclose(basis.forward, [-1.0, 0.0, 0.0])

    ray = dart.gui.experimental.make_perspective_pick_ray(camera, 320, 240, 640, 480)
    assert np.allclose(ray.origin, basis.eye)
    assert np.allclose(ray.direction, basis.forward)

    update = dart.gui.experimental.OrbitCameraUpdate()
    update.scroll_delta = 100.0
    dart.gui.experimental.update_orbit_camera(camera, update)
    assert np.isclose(camera.distance, update.min_distance)


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
