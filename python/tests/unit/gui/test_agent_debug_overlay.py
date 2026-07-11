"""Tests for DART 6 debug-overlay composition and rasterization (no GL)."""

import sys
from pathlib import Path

import numpy as np
import pytest

import dartpy as dart

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import agent_debug_overlay as ado
import agent_view_quality as avq
from _image_tools import ImageData


def _settled_world():
    world = dart.simulation.World()

    def box(name, size, position, static=False):
        skeleton = dart.dynamics.Skeleton(name)
        if static:
            joint, body = skeleton.createWeldJointAndBodyNodePair()
        else:
            joint, body = skeleton.createFreeJointAndBodyNodePair()
        body.setName(name)
        shape_node = body.createShapeNode(dart.dynamics.BoxShape(np.asarray(size)))
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect()
        if static:
            transform = dart.math.Isometry3()
            transform.set_translation(list(position))
            joint.setTransformFromParentBodyNode(transform)
        else:
            joint.setPositions(np.concatenate([[0.0, 0.0, 0.0], position]))
        return skeleton

    world.addSkeleton(box("ground", [2.0, 2.0, 0.1], [0.0, 0.0, -0.05], static=True))
    world.addSkeleton(box("box", [0.2, 0.2, 0.2], [0.0, 0.0, 0.12]))
    for _ in range(100):
        world.step()
    return world


def test_unknown_layer_rejected():
    world = _settled_world()
    with pytest.raises(ValueError, match="unknown debug layers"):
        ado.build_overlay(world, layers=("nonsense",))


def test_body_frames_layer_emits_axes():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("body_frames",))
    assert len(scene.segments) == 3 * 2  # two bodies, three axes each
    colors = {segment[2] for segment in scene.segments}
    assert colors == {ado.AXIS_X_RGB, ado.AXIS_Y_RGB, ado.AXIS_Z_RGB}


def test_contacts_layer_marks_points_and_normals():
    world = _settled_world()
    contacts = list(world.getLastCollisionResult().getContacts())
    assert contacts, "settled box should contact the ground"
    plausible = [
        c
        for c in contacts
        if ado._is_plausible_point(np.asarray(c.point, dtype=float))
    ]
    assert plausible, "at least one plausible contact expected"
    scene = ado.build_overlay(world, layers=("contacts",))
    marker_segments = [
        s for s in scene.segments if s[2] == ado.CONTACT_POINT_RGB
    ]
    assert len(marker_segments) == 2 * len(plausible)
    assert scene.skipped_contacts == len(contacts) - len(plausible)
    first_point = np.asarray(plausible[0].point, dtype=float)
    midpoints = [0.5 * (s[0] + s[1]) for s in marker_segments]
    assert any(np.allclose(mid, first_point, atol=1e-9) for mid in midpoints)
    assert any(s[2] == ado.CONTACT_NORMAL_RGB for s in scene.segments)


def test_garbage_contact_points_are_skipped_not_drawn():
    # DART 6's detector can emit sentinel coordinates (~1e101); those must
    # not become overlay geometry.
    class FakeContact:
        def __init__(self, point):
            self.point = np.asarray(point, dtype=float)
            self.normal = np.array([0.0, 0.0, 1.0])
            self.force = np.zeros(3)

    world = _settled_world()
    scene = ado.build_overlay(
        world,
        layers=("contacts",),
        contacts=[FakeContact([0.0, 1.5e101, 0.0]), FakeContact([0.1, 0.0, 0.0])],
    )
    assert scene.skipped_contacts == 1
    marker_segments = [
        s for s in scene.segments if s[2] == ado.CONTACT_POINT_RGB
    ]
    assert len(marker_segments) == 2


def test_explicit_contacts_used_verbatim():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("contacts",), contacts=[])
    assert scene.segments == []


def test_trajectories_layer_requires_history():
    world = _settled_world()
    with pytest.raises(ValueError, match="TrajectoryTracker"):
        ado.build_overlay(world, layers=("trajectories",))


def test_trajectory_tracker_grows_polyline():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("faller")
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.setName("faller")
    shape_node = body.createShapeNode(dart.dynamics.BoxShape([0.1, 0.1, 0.1]))
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    joint.setPositions(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0]))
    world.addSkeleton(skeleton)

    tracker = ado.TrajectoryTracker(world, bodies=["faller"])
    for _ in range(10):
        world.step()
        tracker.sample()
    scene = ado.build_overlay(world, layers=("trajectories",), trajectories=tracker)
    assert 1 <= len(scene.segments) <= 9
    assert all(s[2] == ado.TRAJECTORY_RGB for s in scene.segments)


def test_velocities_layer_skips_static_bodies():
    world = _settled_world()  # settled: box at rest, ground static
    scene = ado.build_overlay(world, layers=("velocities",))
    # Neither body moves; no arrows should be drawn for the static ground.
    ground_origin = np.zeros(3)
    for start, _end, _rgb in scene.segments:
        assert not np.allclose(start[:2], ground_origin[:2], atol=1e-6) or (
            abs(start[2] - (-0.05)) > 1e-3
        )


def test_labels_layer_names_bodies():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("labels",))
    assert sorted(text for _anchor, text in scene.labels) == ["box", "ground"]


def test_composite_overlay_draws_pixels():
    world = _settled_world()
    camera = avq.frame_body(world, "box", margin=3.0)
    scene = ado.build_overlay(world, layers=("contacts", "body_frames", "labels"))
    blank = ImageData(
        path=Path("blank"),
        width=320,
        height=240,
        pixels=bytes(3 * 320 * 240),
    )
    annotated = ado.composite_overlay(blank, scene, camera)
    assert annotated.pixels != blank.pixels
    # Deterministic compositing: same inputs, same bytes.
    again = ado.composite_overlay(blank, scene, camera)
    assert annotated.pixels == again.pixels


def test_draw_line_clips_at_borders():
    pixels = bytearray(3 * 32 * 32)
    ado._draw_line_rgb(pixels, 32, 32, -10.0, -10.0, 60.0, 60.0, (255, 0, 0), 1)
    assert bytes((255, 0, 0)) in bytes(pixels)
