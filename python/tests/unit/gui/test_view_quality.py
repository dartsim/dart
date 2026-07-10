from __future__ import annotations

import math

import pytest

import dartpy as dart

# Guard on the C++ GUI bindings the assessments consume (projection, pick
# rays, descriptors); reduced builds expose dartpy.gui without them.
pytestmark = pytest.mark.skipif(
    not hasattr(dart, "gui")
    or not hasattr(dart.gui, "ProjectionOptions")
    or not hasattr(dart.gui, "PickRay"),
    reason="dartpy GUI projection/picking bindings are not available in this build",
)


def _world_with_marker():
    world = dart.World()
    marker = world.add_rigid_body("marker", position=(0.3, -0.2, 0.4))
    marker.set_collision_shape(dart.CollisionShape.box((0.08, 0.08, 0.08)))
    return world


def _add_wall(world):
    wall = world.add_rigid_body("wall", position=(0.8, 0.35, 0.4))
    wall.is_static = True
    wall.set_collision_shape(dart.CollisionShape.box((0.05, 1.2, 1.2)))
    return wall


def test_clean_view_passes_geometry_checks():
    world = _world_with_marker()
    camera = dart.gui.frame_body(world, "marker", azimuth=0.8, elevation=0.45)
    report = dart.gui.assess_view(world, camera, (320, 240), focus="marker")
    assert report.issues == []
    assert report.corner_coverage == 1.0
    assert report.occlusion_fraction == 0.0
    assert report.center_visible
    assert 0.0 < report.score <= 1.0


def test_occlusion_detected_and_reselection_avoids_it():
    world = _world_with_marker()
    camera = dart.gui.orbit_camera(
        azimuth=0.8, elevation=0.45, distance=2.2, target=(0.0, 0.0, 0.2)
    )
    _add_wall(world)
    report = dart.gui.assess_view(world, camera, (320, 240), focus="marker")
    assert "occluded" in report.issues
    assert report.occlusion_fraction > 0.5

    choices = dart.gui.select_viewpoints(world, (320, 240), focus="marker", count=2)
    for choice in choices:
        assert "occluded" not in choice.report.issues
        assert choice.report.score > report.score


def test_cropped_and_distance_issues_detected():
    world = _world_with_marker()
    far = dart.gui.orbit_camera(
        azimuth=0.8, elevation=0.45, distance=40.0, target=(0.0, 0.0, 0.2)
    )
    assert "too-far" in dart.gui.assess_view(world, far, (320, 240), focus="marker").issues

    off = dart.gui.orbit_camera(
        azimuth=0.8, elevation=0.45, distance=1.2, target=(2.5, 0.0, 0.2)
    )
    off_report = dart.gui.assess_view(world, off, (320, 240), focus="marker")
    assert {"cropped", "off-frame"} & set(off_report.issues)

    close = dart.gui.orbit_camera(
        azimuth=0.8, elevation=0.45, distance=0.16, target=(0.3, -0.2, 0.4)
    )
    assert "too-close" in dart.gui.assess_view(
        world, close, (320, 240), focus="marker"
    ).issues


def test_selection_is_deterministic_and_reports_reasons():
    world = _world_with_marker()
    _add_wall(world)
    first = dart.gui.select_viewpoints(world, (320, 240), focus="marker", count=3)
    second = dart.gui.select_viewpoints(world, (320, 240), focus="marker", count=3)
    assert [c.reason for c in first] == [c.reason for c in second]
    assert len(first) == 3
    azimuths = [c.report.camera["azimuth"] for c in first]
    for index, azimuth in enumerate(azimuths):
        for other in azimuths[index + 1 :]:
            difference = (azimuth - other) % math.tau
            assert min(difference, math.tau - difference) >= math.tau / 8.0 - 1e-9


def test_report_json_schema():
    world = _world_with_marker()
    camera = dart.gui.frame_body(world, "marker")
    payload = dart.gui.assess_view(world, camera, (320, 240), focus="marker").to_json()
    assert payload["schema_version"] == "dart.view_report/v1"
    assert set(payload["metrics"]) == {
        "corner_coverage",
        "subject_fraction",
        "center_visible",
        "occlusion_fraction",
        "ambiguity_iou",
    }
    assert payload["pass"] is True
    assert payload["camera"]["distance"] > 0.0


def test_focus_validation_lists_available_names():
    world = _world_with_marker()
    camera = dart.gui.frame_body(world, "marker")
    with pytest.raises(ValueError, match="marker_0"):
        dart.gui.assess_view(world, camera, (320, 240), focus="nonexistent")


def test_frame_region_respects_margin():
    near = dart.gui.frame_region((0.0, 0.0, 0.0), 1.0, margin=1.0)
    far = dart.gui.frame_region((0.0, 0.0, 0.0), 1.0, margin=2.0)
    assert far.distance == pytest.approx(2.0 * near.distance)
