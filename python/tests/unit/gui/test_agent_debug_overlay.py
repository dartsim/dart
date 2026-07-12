"""Tests for DART 6 debug-overlay composition and engine injection."""

import os
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

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

try:
    _osg = dart.gui.osg
except AttributeError:  # pragma: no cover - depends on build config
    pytest.skip("dartpy built without gui.osg", allow_module_level=True)

_HAS_OSG = True


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
        c for c in contacts if ado._is_plausible_point(np.asarray(c.point, dtype=float))
    ]
    assert plausible, "at least one plausible contact expected"
    scene = ado.build_overlay(world, layers=("contacts",))
    marker_segments = [s for s in scene.segments if s[2] == ado.CONTACT_POINT_RGB]
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
    marker_segments = [s for s in scene.segments if s[2] == ado.CONTACT_POINT_RGB]
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


def test_trajectory_histories_stay_separate_per_body():
    # Two skeletons whose bodies share the name "box" must record separate
    # histories — a name-keyed merge would draw polyline segments jumping
    # between the two COMs, fabricating motion evidence.
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, 0.0])
    for skeleton_name, x in (("skel_a", -0.5), ("skel_b", 0.5)):
        skeleton = dart.dynamics.Skeleton(skeleton_name)
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        body.setName("box")
        shape_node = body.createShapeNode(dart.dynamics.BoxShape([0.1, 0.1, 0.1]))
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect()
        joint.setPositions(np.array([0.0, 0.0, 0.0, x, 0.0, 0.2]))
        world.addSkeleton(skeleton)

    tracker = ado.TrajectoryTracker(world, bodies=["box"])
    for _ in range(3):
        world.step()
        tracker.sample()

    assert set(tracker.history) == {"skel_a:box", "skel_b:box"}
    # Each history stays on its own side of the world: no cross-body jumps.
    for key, positions in tracker.history.items():
        assert len(positions) == 3
        xs = [float(p[0]) for p in positions]
        expected = -0.5 if key == "skel_a:box" else 0.5
        assert xs == pytest.approx([expected] * 3, abs=1e-6)
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


def test_grid_layer_emits_ground_lattice():
    world = _settled_world()
    # half extent 4.0 / spacing 0.5 -> lines at -8..8 (17 offsets), one parallel
    # to x and one parallel to y per offset.
    scene = ado.build_overlay(world, layers=("grid",))
    assert len(scene.segments) == 2 * (2 * 8 + 1)
    assert {segment[2] for segment in scene.segments} == {ado.GRID_RGB}
    # every grid line lies on the z = 0 plane and stays within the half extent.
    for start, end, _rgb in scene.segments:
        assert abs(start[2]) < 1e-9 and abs(end[2]) < 1e-9
        assert np.all(np.abs(start) <= 4.0 + 1e-9)
        assert np.all(np.abs(end) <= 4.0 + 1e-9)


def test_grid_layer_respects_spacing_and_extent():
    world = _settled_world()
    scene = ado.build_overlay(
        world, layers=("grid",), grid_half_extent=1.0, grid_spacing=0.5, grid_z=0.2
    )
    # offsets at -1.0, -0.5, 0.0, 0.5, 1.0 -> 5 offsets, two lines each.
    assert len(scene.segments) == 2 * 5
    assert all(abs(start[2] - 0.2) < 1e-9 for start, _end, _rgb in scene.segments)


def test_world_frame_layer_emits_rgb_axes():
    world = _settled_world()
    scene = ado.build_overlay(
        world, layers=("world_frame",), world_frame_axis_length=0.5
    )
    assert len(scene.segments) == 3
    assert {segment[2] for segment in scene.segments} == {
        ado.AXIS_X_RGB,
        ado.AXIS_Y_RGB,
        ado.AXIS_Z_RGB,
    }
    for start, end, _rgb in scene.segments:
        assert np.allclose(start, np.zeros(3))
        assert np.isclose(np.linalg.norm(end), 0.5)


def test_coms_layer_marks_body_centers():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("coms",), com_marker_radius=0.05)
    assert len(scene.segments) == 2 * 3  # two bodies, a 3-axis cross each
    assert {segment[2] for segment in scene.segments} == {ado.COM_RGB}
    # each cross arm is centered on the body COM and spans 2 * radius.
    for start, end, _rgb in scene.segments:
        assert np.isclose(np.linalg.norm(end - start), 0.1)


def test_inertia_boxes_layer_reproduces_solid_box():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("solid")
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.setName("solid")
    body.createShapeNode(dart.dynamics.BoxShape([0.4, 0.7, 1.0])).createVisualAspect()
    world.addSkeleton(skeleton)

    mass = 3.0
    half = np.array([0.2, 0.35, 0.5])  # half extents of the solid box
    a, b, c = half
    moment = np.diag(
        [
            (mass / 3.0) * (b * b + c * c),
            (mass / 3.0) * (a * a + c * c),
            (mass / 3.0) * (a * a + b * b),
        ]
    )
    inertia = body.getInertia()
    inertia.setMass(mass)
    inertia.setMoment(moment)
    body.setInertia(inertia)

    scene = ado.build_overlay(world, layers=("inertia_boxes",))
    assert len(scene.segments) == 12  # a single body -> one 12-edge wireframe
    assert all(segment[2] == ado.INERTIA_RGB for segment in scene.segments)
    # the body is unrotated at the origin, so the recovered wireframe is the
    # axis-aligned solid box that produces this moment of inertia.
    points = np.array([p for segment in scene.segments for p in segment[:2]])
    recovered = 0.5 * (points.max(axis=0) - points.min(axis=0))
    assert np.allclose(np.sort(recovered), np.sort(half), atol=1e-9)


def test_inertia_boxes_layer_skips_massless_bodies():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("weightless")
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.createShapeNode(dart.dynamics.BoxShape([0.2, 0.2, 0.2])).createVisualAspect()
    inertia = body.getInertia()
    inertia.setMass(0.0)
    body.setInertia(inertia)
    world.addSkeleton(skeleton)
    scene = ado.build_overlay(world, layers=("inertia_boxes",))
    assert scene.segments == []


def test_collision_bounds_layer_matches_known_box():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("box")
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.setName("box")
    shape_node = body.createShapeNode(dart.dynamics.BoxShape([0.4, 0.7, 1.0]))
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    joint.setPositions(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.6]))
    world.addSkeleton(skeleton)

    scene = ado.build_overlay(world, layers=("collision_bounds",))
    assert len(scene.segments) == 12  # one shape node -> one 12-edge wireframe
    assert all(segment[2] == ado.COLLISION_BOUNDS_RGB for segment in scene.segments)
    points = np.array([p for segment in scene.segments for p in segment[:2]])
    extent = points.max(axis=0) - points.min(axis=0)
    center = 0.5 * (points.max(axis=0) + points.min(axis=0))
    assert np.allclose(extent, [0.4, 0.7, 1.0], atol=1e-9)
    assert np.allclose(center, [0.0, 0.0, 0.6], atol=1e-9)


def test_collision_bounds_layer_skips_visual_only_shapes():
    # A visual-only helper shape is invisible to the collision detector, so
    # the collision_bounds layer must not draw its bounds: only the shape
    # node with a CollisionAspect gets a 12-edge wireframe.
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("box")
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.setName("box")
    collision_node = body.createShapeNode(dart.dynamics.BoxShape([0.4, 0.7, 1.0]))
    collision_node.createVisualAspect()
    collision_node.createCollisionAspect()
    marker_node = body.createShapeNode(dart.dynamics.BoxShape([3.0, 3.0, 3.0]))
    marker_node.createVisualAspect()  # visual-only: no CollisionAspect
    joint.setPositions(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.6]))
    world.addSkeleton(skeleton)

    scene = ado.build_overlay(world, layers=("collision_bounds",))
    assert len(scene.segments) == 12  # only the collision shape's wireframe
    points = np.array([p for segment in scene.segments for p in segment[:2]])
    extent = points.max(axis=0) - points.min(axis=0)
    assert np.allclose(extent, [0.4, 0.7, 1.0], atol=1e-9)


@pytest.mark.skipif(not _HAS_OSG, reason="dartpy built without gui.osg")
def test_populate_overlay_loads_lines_and_labels():
    world = _settled_world()
    scene = ado.build_overlay(world, layers=("body_frames", "labels"))
    overlay = dart.gui.osg.DebugOverlay()
    lines, labels = ado.populate_overlay(overlay, scene)
    assert lines == len(scene.segments)
    assert labels == len(scene.labels) == 2
    assert overlay.getNumLines() == len(scene.segments)
    assert overlay.getNumLabels() == 2
    # Re-populating clears the previous content first.
    again_lines, again_labels = ado.populate_overlay(overlay, scene)
    assert (again_lines, again_labels) == (len(scene.segments), 2)
    assert overlay.getNumLines() == len(scene.segments)
    assert overlay.getNumLabels() == 2


@pytest.mark.skipif(not _HAS_OSG, reason="dartpy built without gui.osg")
def test_populate_overlay_empty_scene_clears_overlay():
    world = _settled_world()
    overlay = dart.gui.osg.DebugOverlay()
    ado.populate_overlay(overlay, ado.build_overlay(world, layers=("body_frames",)))
    assert overlay.getNumLines() > 0
    empty = ado.build_overlay(world, layers=("contacts",), contacts=[])
    lines, labels = ado.populate_overlay(overlay, empty)
    assert (lines, labels) == (0, 0)
    assert overlay.getNumLines() == 0
    assert overlay.getNumLabels() == 0


@pytest.mark.skipif(not _HAS_OSG, reason="dartpy built without gui.osg")
@requires_display
def test_engine_rendered_overlay_changes_pixels(tmp_path):
    world = _settled_world()
    viewer = dart.gui.osg.ImGuiViewer()
    viewer.addWorldNode(dart.gui.osg.WorldNode(world))
    overlay = dart.gui.osg.DebugOverlay()
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
            width=240,
            height=180,
            fovYDeg=camera.fovy_deg,
            warmupFrames=10,
        )
        if not ok:
            pytest.skip("no off-screen GL context (needs a usable DISPLAY)")
        return path.read_bytes()

    base = shoot("base.png")

    # Lines and labels render through the engine and revert cleanly on clear.
    scene = ado.build_overlay(world, layers=("contacts", "body_frames", "labels"))
    ado.populate_overlay(overlay, scene, character_size=0.05)
    with_overlay = shoot("overlay.png")
    overlay.clear()
    after_clear = shoot("after_clear.png")
    assert with_overlay != base, "engine-rendered overlay must change pixels"
    assert after_clear == base, "clearing the overlay must restore the scene"
