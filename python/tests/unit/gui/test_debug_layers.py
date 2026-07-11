from __future__ import annotations

import numpy as np
import pytest

import dartpy as dart

# Guard on a C++ GUI binding, not the pure-Python helpers: reduced builds
# (e.g. the cuda environment) expose dartpy.gui without the descriptor and
# debug types these layers compose.
pytestmark = pytest.mark.skipif(
    not hasattr(dart, "gui") or not hasattr(dart.gui, "DebugDrawOptions"),
    reason="dartpy GUI debug bindings are not available in this build",
)


def _settled_world():
    world = dart.World()
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.05))
    ground.is_static = True
    ground.set_collision_shape(dart.CollisionShape.box((1.6, 1.6, 0.1)))
    box = world.add_rigid_body("box", position=(0.0, 0.0, 0.12))
    box.set_collision_shape(dart.CollisionShape.box((0.2, 0.2, 0.2)))
    for _ in range(50):
        world.step()
    return world


def test_unknown_layer_is_rejected():
    world = _settled_world()
    with pytest.raises(ValueError, match="unknown debug layers"):
        dart.gui.debug_scene_for_world(world, layers=("nonsense",))


def test_body_frames_layer_emits_three_axes_per_body():
    world = _settled_world()
    scene = dart.gui.debug_scene_for_world(world, layers=("body_frames",))
    assert len(scene.lines) == 3 * 2  # two bodies, three axis lines each
    labels = {line.label for line in scene.lines}
    assert {"box.x", "box.y", "box.z", "ground.x"} <= labels


def test_contacts_layer_marks_contact_points_and_normals():
    world = _settled_world()
    contacts = world.collide()
    assert contacts, "settled box should be in contact with the ground"
    scene = dart.gui.debug_scene_for_world(world, layers=("contacts",))
    labels = [line.label for line in scene.lines]
    assert labels.count("contact.point") == 2 * len(contacts)
    assert any(label == "contact.normal" for label in labels)
    # Contact markers must sit at the reported contact points.
    first_point = np.asarray(contacts[0].point, dtype=float)
    marker_lines = [line for line in scene.lines if line.label == "contact.point"]
    midpoints = [
        0.5 * (np.asarray(line.from_point) + np.asarray(line.to_point))
        for line in marker_lines
    ]
    assert any(np.allclose(mid, first_point, atol=1e-9) for mid in midpoints)


def test_explicit_contacts_input_is_used_verbatim():
    world = _settled_world()
    scene = dart.gui.debug_scene_for_world(world, layers=("contacts",), contacts=[])
    assert scene.lines == []


def test_trajectory_tracker_grows_polyline():
    # Track a body while it is actually falling so the polyline is non-empty
    # (a settled body would yield zero-length segments that are dropped).
    world = dart.World()
    faller = world.add_rigid_body("faller", position=(0.0, 0.0, 2.0))
    faller.set_collision_shape(dart.CollisionShape.sphere(0.1))
    tracker = dart.gui.TrajectoryTracker(world, bodies=["faller"])
    for _ in range(10):
        world.step()
        tracker.sample()
    lines = tracker.debug_lines()
    assert 1 <= len(lines) <= 9
    assert all(line.label == "faller.trajectory" for line in lines)
    scene = dart.gui.debug_scene_for_world(
        world, layers=("trajectories",), trajectories=tracker
    )
    assert len(scene.lines) == len(lines)


def test_trajectory_tracker_uses_world_body_enumeration(monkeypatch):
    from dartpy import _scene_dump

    world = _settled_world()
    monkeypatch.setattr(
        _scene_dump,
        "_tracked_rigid_bodies",
        lambda _world: (_ for _ in ()).throw(AssertionError("registry used")),
    )
    tracker = dart.gui.TrajectoryTracker(world)
    tracker.sample()
    assert set(tracker.history) == set(world.get_rigid_body_names())


def test_trajectories_layer_without_history_raises():
    world = _settled_world()
    with pytest.raises(ValueError, match="TrajectoryTracker"):
        dart.gui.debug_scene_for_world(world, layers=("trajectories",))


def test_trajectories_layer_rejects_fewer_than_two_samples():
    world = _settled_world()
    tracker = dart.gui.TrajectoryTracker(world, bodies=["box"])
    with pytest.raises(ValueError, match="at least two sampled positions"):
        dart.gui.debug_scene_for_world(
            world, layers=("trajectories",), trajectories=tracker
        )

    tracker.sample()
    with pytest.raises(ValueError, match="at least two sampled positions"):
        dart.gui.debug_scene_for_world(
            world, layers=("trajectories",), trajectories=tracker
        )

    with pytest.raises(ValueError, match="at least two sampled positions"):
        dart.gui.debug_scene_for_world(
            world, layers=("trajectories",), trajectories={"box": []}
        )

    with pytest.raises(ValueError, match="at least two sampled positions"):
        dart.gui.debug_scene_for_world(
            world,
            layers=("trajectories",),
            trajectories={"box": [np.zeros(3)]},
        )


def test_labels_layer_names_bodies():
    world = _settled_world()
    scene = dart.gui.debug_scene_for_world(world, layers=("labels",))
    assert sorted(label.text for label in scene.labels) == ["box", "ground"]


def test_velocities_layer_only_draws_moving_bodies():
    world = dart.World()
    body = world.add_rigid_body(
        "mover", position=(0.0, 0.0, 2.0), linear_velocity=(1.0, 0.0, 0.0)
    )
    body.set_collision_shape(dart.CollisionShape.sphere(0.1))
    still = world.add_rigid_body("still", position=(1.0, 0.0, 2.0))
    still.is_static = True
    still.set_collision_shape(dart.CollisionShape.sphere(0.1))
    scene = dart.gui.debug_scene_for_world(world, layers=("velocities",))
    labels = {line.label for line in scene.lines}
    assert "mover.vel_linear" in labels
    # The exclusion side of the claim: a static body draws no arrows.
    assert not any(label.startswith("still.") for label in labels)


def test_project_points_puts_camera_target_at_image_center():
    camera = dart.gui.orbit_camera(
        azimuth=0.7, elevation=0.4, distance=3.0, target=(0.2, -0.1, 0.5)
    )
    projected = dart.gui.project_points(camera, (640, 480), [(0.2, -0.1, 0.5)])
    assert projected.shape == (1, 3)
    assert projected[0, 0] == pytest.approx(320.0, abs=1.0)
    assert projected[0, 1] == pytest.approx(240.0, abs=1.0)
    assert projected[0, 2] == pytest.approx(3.0, abs=1e-6)


def test_draw_text_writes_opaque_pixels():
    pixels = np.zeros((40, 120, 4), dtype=np.uint8)
    dart.gui.draw_text(pixels, "BOX-1", (2, 2), scale=2)
    assert int((pixels[..., 3] == 255).sum()) > 0
    assert pixels[..., :3].max() == 255


def test_composite_labels_draws_visible_label():
    world = _settled_world()
    scene = dart.gui.debug_scene_for_world(world, layers=("labels",))
    camera = dart.gui.frame_body(world, "box")
    base = np.zeros((240, 320, 4), dtype=np.uint8)
    annotated = dart.gui.composite_labels(base, camera, scene.labels)
    assert annotated.shape == (240, 320, 4)
    assert int(annotated.max()) == 255
