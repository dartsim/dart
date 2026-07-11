"""Geometry tests for the DART 6 agent view-quality helpers (no GL needed)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

import dartpy as dart

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import agent_view_quality as avq


def _box_skeleton(name, size, position, static=False):
    skeleton = dart.dynamics.Skeleton(name)
    if static:
        joint, body = skeleton.createWeldJointAndBodyNodePair()
    else:
        joint, body = skeleton.createFreeJointAndBodyNodePair()
    body.setName(name)
    shape_node = body.createShapeNode(dart.dynamics.BoxShape(np.asarray(size)))
    shape_node.createVisualAspect().setColor([0.2, 0.4, 0.8])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    if static:
        transform = dart.math.Isometry3()
        transform.set_translation(list(position))
        joint.setTransformFromParentBodyNode(transform)
    else:
        joint.setPositions(np.concatenate([[0.0, 0.0, 0.0], position]))
    return skeleton


def _world_with_marker():
    world = dart.simulation.World()
    world.addSkeleton(
        _box_skeleton("ground", [2.0, 2.0, 0.1], [0.0, 0.0, -0.05], static=True)
    )
    world.addSkeleton(_box_skeleton("marker", [0.16, 0.16, 0.16], [0.3, -0.2, 0.4]))
    return world


def _add_wall(world):
    world.addSkeleton(
        _box_skeleton(
            "marker_holder", [0.05, 2.4, 2.4], [0.8, 0.35, 0.4], static=True
        )
    )


def test_bounding_box_binding_roundtrip():
    # math::BoundingBox is now bound: getBoundingBox() returns a usable object,
    # and the class round-trips through its own constructor and setters.
    shape = dart.dynamics.BoxShape([0.2, 0.4, 0.6])
    box = shape.getBoundingBox()
    assert np.asarray(box.getMin()) == pytest.approx([-0.1, -0.2, -0.3])
    assert np.asarray(box.getMax()) == pytest.approx([0.1, 0.2, 0.3])
    assert np.asarray(box.computeCenter()) == pytest.approx([0.0, 0.0, 0.0])
    assert np.asarray(box.computeHalfExtents()) == pytest.approx([0.1, 0.2, 0.3])

    made = dart.math.BoundingBox([-1.0, -2.0, -3.0], [4.0, 5.0, 6.0])
    assert np.asarray(made.getMin()) == pytest.approx([-1.0, -2.0, -3.0])
    assert np.asarray(made.computeFullExtents()) == pytest.approx([5.0, 7.0, 9.0])
    made.setMin([0.0, 0.0, 0.0])
    made.setMax([2.0, 2.0, 2.0])
    assert np.asarray(made.computeCenter()) == pytest.approx([1.0, 1.0, 1.0])


def test_body_bounds_cover_shapes():
    world = _world_with_marker()
    bounds = {b.name: b for b in avq.body_bounds(world)}
    assert set(bounds) == {"ground", "marker"}
    assert bounds["marker"].aabb_min == pytest.approx([0.22, -0.28, 0.32])
    assert bounds["marker"].aabb_max == pytest.approx([0.38, -0.12, 0.48])


def test_clean_view_passes():
    world = _world_with_marker()
    camera = avq.frame_body(world, "marker", azimuth=0.8, elevation=0.45)
    report = avq.assess_view(world, camera, (320, 240), focus="marker")
    assert report.issues == []
    assert report.corner_coverage == 1.0
    assert report.occlusion_fraction == 0.0
    assert 0.0 < report.score <= 1.0


def test_prefix_sharing_body_is_an_occluder_not_focus():
    # "marker_holder" shares the "marker" prefix but is a distinct body; it
    # must stay an occluder rather than joining the focus set.
    world = _world_with_marker()
    _add_wall(world)
    camera = avq.orbit_camera([0.0, 0.0, 0.2], 2.2, azimuth=0.8, elevation=0.45)
    report = avq.assess_view(world, camera, (320, 240), focus="marker")
    assert report.focus == ["marker"]
    assert "occluded" in report.issues
    assert report.occlusion_fraction > 0.5


def test_reselection_avoids_occluder():
    world = _world_with_marker()
    _add_wall(world)
    choices = avq.select_viewpoints(world, (320, 240), focus="marker", count=2)
    assert len(choices) == 2
    for choice in choices:
        assert "occluded" not in choice.report.issues


def test_distance_and_crop_issues():
    world = _world_with_marker()
    far = avq.orbit_camera([0.0, 0.0, 0.2], 60.0, azimuth=0.8, elevation=0.45)
    assert "too-far" in avq.assess_view(world, far, (320, 240), focus="marker").issues

    off = avq.orbit_camera([4.0, 0.0, 0.2], 1.2, azimuth=0.8, elevation=0.45)
    off_report = avq.assess_view(world, off, (320, 240), focus="marker")
    assert {"cropped", "off-frame"} & set(off_report.issues)

    close = avq.orbit_camera([0.3, -0.2, 0.4], 0.2, azimuth=0.8, elevation=0.45)
    assert "too-close" in avq.assess_view(
        world, close, (320, 240), focus="marker"
    ).issues


def test_resting_contact_is_not_occlusion():
    # After settling, the box's bottom corners penetrate the ground slab by a
    # couple of millimetres; nearest-hit semantics must not read that as the
    # ground occluding the box.
    world = dart.simulation.World()
    world.addSkeleton(
        _box_skeleton("ground", [2.0, 2.0, 0.1], [0.0, 0.0, -0.05], static=True)
    )
    world.addSkeleton(_box_skeleton("box", [0.2, 0.2, 0.2], [0.0, 0.0, 0.35]))
    for _ in range(300):
        world.step()
    camera = avq.frame_body(world, "box", azimuth=0.8, elevation=0.45)
    report = avq.assess_view(world, camera, (320, 240), focus="box")
    assert "occluded" not in report.issues
    assert report.occlusion_fraction == 0.0


def test_selection_is_deterministic():
    world = _world_with_marker()
    _add_wall(world)
    first = avq.select_viewpoints(world, (320, 240), focus="marker", count=3)
    second = avq.select_viewpoints(world, (320, 240), focus="marker", count=3)
    assert [c.reason for c in first] == [c.reason for c in second]


def test_report_json_schema():
    world = _world_with_marker()
    camera = avq.frame_body(world, "marker")
    payload = avq.assess_view(world, camera, (320, 240), focus="marker").to_json()
    assert payload["schema_version"] == "dart.view_report/v1"
    assert set(payload["metrics"]) == {
        "corner_coverage",
        "subject_fraction",
        "center_visible",
        "occlusion_fraction",
        "ambiguity_iou",
    }
    assert payload["pass"] is True


def test_unknown_focus_lists_available():
    world = _world_with_marker()
    camera = avq.frame_body(world, "marker")
    with pytest.raises(ValueError, match="marker"):
        avq.assess_view(world, camera, (320, 240), focus="nonexistent")


def test_frame_region_margin_scales_distance():
    near = avq.frame_region([0.0, 0.0, 0.0], 1.0, margin=1.0)
    far = avq.frame_region([0.0, 0.0, 0.0], 1.0, margin=2.0)
    near_distance = float(np.linalg.norm(near.eye - near.center))
    far_distance = float(np.linalg.norm(far.eye - far.center))
    assert far_distance == pytest.approx(2.0 * near_distance)


def test_default_agent_camera_parity():
    # frame_region's distance law matches dart.gui.osg.defaultAgentCamera.
    if not hasattr(dart.gui, "osg"):
        pytest.skip("dartpy built without gui.osg")
    eye, center, up = dart.gui.osg.defaultAgentCamera(
        [0.0, 0.0, 0.0], 1.0, fovYDeg=30.0
    )
    theirs = float(np.linalg.norm(np.asarray(eye) - np.asarray(center)))
    ours = avq.frame_region([0.0, 0.0, 0.0], 1.0, fovy_deg=30.0, margin=1.0)
    assert float(np.linalg.norm(ours.eye - ours.center)) == pytest.approx(
        theirs, rel=1e-6
    )


def test_project_points_puts_target_at_image_center():
    camera = avq.orbit_camera([0.2, -0.1, 0.5], 3.0, azimuth=0.7, elevation=0.4)
    projected = avq.project_points(camera, (640, 480), [[0.2, -0.1, 0.5]])
    assert projected[0, 0] == pytest.approx(320.0, abs=1.0)
    assert projected[0, 1] == pytest.approx(240.0, abs=1.0)
    assert projected[0, 2] == pytest.approx(3.0, abs=1e-6)


def test_ray_aabb_distance():
    origin = np.array([0.0, 0.0, 0.0])
    direction = np.array([1.0, 0.0, 0.0])
    hit = avq._ray_aabb_distance(
        origin, direction, np.array([2.0, -1.0, -1.0]), np.array([3.0, 1.0, 1.0])
    )
    assert hit == pytest.approx(2.0)
    assert (
        avq._ray_aabb_distance(
            origin, direction, np.array([2.0, 2.0, -1.0]), np.array([3.0, 3.0, 1.0])
        )
        is None
    )
