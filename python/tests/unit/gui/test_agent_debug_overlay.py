"""Tests for DART 6 debug-overlay composition and engine injection."""

import os
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

requires_display = pytest.mark.skipif(
    not os.environ.get("DISPLAY"),
    reason="off-screen GLX capture needs a DISPLAY (run under xvfb-run on "
    "headless hosts)",
)

_HAS_OSG = hasattr(dart.gui, "osg")


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


def test_inject_overlay_adds_and_removes_line_segment_frames():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("contacts", "body_frames"))
    before = world.getNumSimpleFrames()

    frames = ado.inject_overlay(world, scene)
    assert frames, "overlay with segments should create frames"
    assert world.getNumSimpleFrames() == before + len(frames)
    for frame in frames:
        shape = frame.getShape()
        assert isinstance(shape, dart.dynamics.LineSegmentShape)
        assert frame.getName().startswith(ado.OVERLAY_FRAME_PREFIX)
        assert frame.hasVisualAspect()

    ado.remove_overlay(world, frames)
    assert world.getNumSimpleFrames() == before


def test_inject_overlay_groups_segments_by_color():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("body_frames",))
    # Three axis colors across two bodies -> three color groups -> three frames.
    distinct_colors = {rgb for _s, _e, rgb in scene.segments}
    frames = ado.inject_overlay(world, scene)
    try:
        assert len(frames) == len(distinct_colors) == 3
    finally:
        ado.remove_overlay(world, frames)


def test_inject_overlay_empty_scene_adds_no_frames():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("contacts",), contacts=[])
    before = world.getNumSimpleFrames()
    frames = ado.inject_overlay(world, scene)
    assert frames == []
    assert world.getNumSimpleFrames() == before


@pytest.mark.skipif(not _HAS_OSG, reason="dartpy built without gui.osg")
def test_populate_labels_loads_text_overlay():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("labels",))
    overlay = dart.gui.osg.TextOverlay()
    count = ado.populate_labels(overlay, scene)
    assert count == len(scene.labels) == 2
    assert overlay.getNumLabels() == 2
    # Re-populating clears the previous labels first.
    again = ado.populate_labels(overlay, scene)
    assert again == 2
    assert overlay.getNumLabels() == 2


@pytest.mark.skipif(not _HAS_OSG, reason="dartpy built without gui.osg")
@requires_display
def test_engine_rendered_overlay_changes_pixels(tmp_path):
    world = _settled_world()
    viewer = dart.gui.osg.ImGuiViewer()
    viewer.addWorldNode(dart.gui.osg.WorldNode(world))
    overlay = dart.gui.osg.TextOverlay()
    font = ado.find_default_font()
    if font:
        overlay.setFont(font)
    viewer.addAttachment(overlay)
    camera = avq.frame_body(world, "box", margin=2.8)

    def shoot(name):
        path = tmp_path / name
        ok = viewer.captureOffscreen(
            str(path),
            camera.eye,
            camera.center,
            camera.up,
            width=200,
            height=150,
            fovYDeg=camera.fovy_deg,
            warmupFrames=10,
        )
        if not ok:
            pytest.skip("no off-screen GL context (needs a usable DISPLAY)")
        return path.read_bytes()

    base = shoot("base.png")

    # Geometry layers render through the engine and revert cleanly on removal.
    scene = ado.build_overlay(world, layers=("contacts", "body_frames"))
    frames = ado.inject_overlay(world, scene)
    with_geometry = shoot("geometry.png")
    ado.remove_overlay(world, frames)
    after_geometry = shoot("after_geometry.png")
    assert with_geometry != base, "engine-rendered geometry must change pixels"
    assert after_geometry == base, "removing the overlay must restore the scene"

    # osgText labels render through the engine and revert cleanly on clear.
    label_scene = ado.build_overlay(world, layers=("labels",))
    ado.populate_labels(overlay, label_scene)
    with_labels = shoot("labels.png")
    overlay.clear()
    after_labels = shoot("after_labels.png")
    assert with_labels != base, "engine-rendered labels must change pixels"
    assert after_labels == base, "clearing labels must restore the scene"
