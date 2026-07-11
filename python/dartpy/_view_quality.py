"""View-quality assessment and adaptive viewpoint selection.

Geometry-first checks (projection and CPU ray picks, no GPU needed) that let
a headless agent detect inadequate captures — cropped, too close/far,
occluded, or ambiguous views — and deterministically pick better ones.
Pixel-level checks (blank/contrast) stay in the image tooling; this module
answers "is this camera worth rendering from?" before pixels exist, mirroring
the repo's text-first verification policy.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from typing import Any, Sequence

import dartpy as dart
import numpy as np

# Subject screen-area fractions outside this band flag too-far / too-close.
_MIN_SUBJECT_FRACTION = 0.015
_MAX_SUBJECT_FRACTION = 0.75
# A subject with corners past the viewport while its center is visible is
# cropped; below this on-screen corner fraction the view is discarded.
_MIN_CORNER_COVERAGE = 0.999
# Fraction of focus-sample rays that may hit another body before the view
# counts as occluded.
_MAX_OCCLUSION_FRACTION = 0.35
# Pairwise screen-box IoU above this for depth-separated bodies flags an
# ambiguous (axis-aligned, stacked) viewpoint.
_MAX_AMBIGUITY_IOU = 0.55


@dataclass
class ViewReport:
    """Machine-readable verdict for one camera against one scene."""

    camera: dict[str, Any]
    size: tuple[int, int]
    focus: list[str]
    corner_coverage: float = 0.0
    subject_fraction: float = 0.0
    center_visible: bool = False
    occlusion_fraction: float = 0.0
    ambiguity_iou: float = 0.0
    issues: list[str] = field(default_factory=list)
    score: float = 0.0

    @property
    def acceptable(self) -> bool:
        return not self.issues

    def to_json(self) -> dict[str, Any]:
        return {
            "schema_version": "dart.view_report/v1",
            "camera": self.camera,
            "size": list(self.size),
            "focus": self.focus,
            "metrics": {
                "corner_coverage": self.corner_coverage,
                "subject_fraction": self.subject_fraction,
                "center_visible": self.center_visible,
                "occlusion_fraction": self.occlusion_fraction,
                "ambiguity_iou": self.ambiguity_iou,
            },
            "issues": list(self.issues),
            "score": self.score,
            "pass": self.acceptable,
        }

    def to_json_text(self) -> str:
        return json.dumps(self.to_json(), indent=2, sort_keys=True)


@dataclass
class ViewpointChoice:
    camera: Any
    report: ViewReport
    reason: str


def _camera_params(camera: Any) -> dict[str, Any]:
    return {
        "azimuth": float(camera.yaw),
        "elevation": float(camera.pitch),
        "distance": float(camera.distance),
        "target": [float(v) for v in np.asarray(camera.target).reshape(3)],
    }


def _descriptor_world_corners(descriptor: Any) -> np.ndarray | None:
    geometry = getattr(descriptor, "geometry", None)
    if geometry is None or not bool(getattr(geometry, "has_local_bounds", False)):
        return None
    from . import _world_render_bridge

    bounds_min = np.asarray(geometry.local_bounds_min, dtype=float).reshape(3)
    bounds_max = np.asarray(geometry.local_bounds_max, dtype=float).reshape(3)
    transform = np.asarray(
        _world_render_bridge._isometry_to_matrix(descriptor.world_transform),
        dtype=float,
    ).reshape(4, 4)
    corners = np.array(
        [
            [x, y, z]
            for x in (bounds_min[0], bounds_max[0])
            for y in (bounds_min[1], bounds_max[1])
            for z in (bounds_min[2], bounds_max[2])
        ]
    )
    return corners @ transform[:3, :3].T + transform[:3, 3]


def _descriptor_name(descriptor: Any) -> str:
    for attribute in ("body_name", "shape_frame_name", "shape_node_name"):
        name = str(getattr(descriptor, attribute, "") or "")
        if name:
            return name
    return f"renderable_{int(getattr(descriptor, 'id', 0))}"


def _split_focus(
    descriptors: Sequence[Any], focus: str | Sequence[str] | None
) -> tuple[list[Any], list[str]]:
    if focus is None:
        return list(descriptors), sorted({_descriptor_name(d) for d in descriptors})
    wanted = {focus} if isinstance(focus, str) else set(focus)

    def matches(descriptor: Any) -> bool:
        names = {
            str(getattr(descriptor, "body_name", "") or ""),
            str(getattr(descriptor, "shape_frame_name", "") or ""),
            str(getattr(descriptor, "shape_node_name", "") or ""),
        }
        if wanted & names:
            return True
        # Render descriptors append a numeric shape index ("body" ->
        # "body_0"). Require exactly that form so a focus of "marker" cannot
        # swallow a distinct body named "marker_holder" — over-matching would
        # put occluders into the focus set and hide real occlusion.
        return any(
            re.fullmatch(rf"{re.escape(target)}_\d+", name)
            for name in names
            for target in wanted
        )

    matched = [d for d in descriptors if matches(d)]
    if not matched:
        raise ValueError(
            f"focus {sorted(wanted)} matched no renderable descriptors; "
            f"available: {sorted({_descriptor_name(d) for d in descriptors})}"
        )
    return matched, sorted({_descriptor_name(d) for d in matched})


def _screen_box(projected: np.ndarray) -> tuple[float, float, float, float] | None:
    in_front = projected[:, 2] > 0.0
    if not in_front.any():
        return None
    points = projected[in_front]
    return (
        float(points[:, 0].min()),
        float(points[:, 1].min()),
        float(points[:, 0].max()),
        float(points[:, 1].max()),
    )


def _box_iou(a: tuple[float, float, float, float], b) -> float:
    inter_w = min(a[2], b[2]) - max(a[0], b[0])
    inter_h = min(a[3], b[3]) - max(a[1], b[1])
    if inter_w <= 0.0 or inter_h <= 0.0:
        return 0.0
    inter = inter_w * inter_h
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


def _occlusion_fraction(
    descriptors: Sequence[Any],
    focus_ids: set[int],
    focus_samples: np.ndarray,
    eye: np.ndarray,
) -> float:
    if len(focus_samples) == 0:
        return 0.0
    blocked = 0
    total = 0
    for sample in focus_samples:
        offset = sample - eye
        distance = float(np.linalg.norm(offset))
        if distance <= 1e-9:
            continue
        ray = dart.gui.PickRay()
        ray.origin = eye
        ray.direction = offset / distance
        hit = dart.gui.pick_nearest_renderable(descriptors, ray)
        total += 1
        if hit is None:
            continue
        # A hit meaningfully closer than the sample that belongs to another
        # renderable blocks the line of sight to the focus subject.
        if int(hit.id) not in focus_ids and float(hit.distance) < distance - 1e-6:
            blocked += 1
    return blocked / total if total else 0.0


def assess_view(
    world: Any,
    camera: Any,
    size: tuple[int, int] = (640, 480),
    *,
    focus: str | Sequence[str] | None = None,
    projection_options: Any | None = None,
) -> ViewReport:
    """Assess one camera against the world's renderables, without rendering."""
    from . import _debug_layers, _world_render_bridge

    width, height = int(size[0]), int(size[1])
    descriptors = _world_render_bridge._renderables_from_world(world)
    focus_descriptors, focus_names = _split_focus(descriptors, focus)

    report = ViewReport(
        camera=_camera_params(camera), size=(width, height), focus=focus_names
    )

    corners_list = [
        corners
        for corners in (_descriptor_world_corners(d) for d in focus_descriptors)
        if corners is not None
    ]
    if not corners_list:
        report.issues.append("no-bounded-focus")
        return report
    corners = np.vstack(corners_list)
    projected = _debug_layers.project_points(
        camera, (width, height), corners, projection_options
    )

    in_front = projected[:, 2] > 0.0
    on_screen = (
        in_front
        & (projected[:, 0] >= 0.0)
        & (projected[:, 0] <= width)
        & (projected[:, 1] >= 0.0)
        & (projected[:, 1] <= height)
    )
    report.corner_coverage = float(on_screen.sum()) / float(len(projected))

    center = corners.mean(axis=0)
    center_projected = _debug_layers.project_points(
        camera, (width, height), [center], projection_options
    )[0]
    report.center_visible = bool(
        center_projected[2] > 0.0
        and 0.0 <= center_projected[0] <= width
        and 0.0 <= center_projected[1] <= height
    )

    box = _screen_box(projected)
    if box is not None:
        clipped_w = max(0.0, min(box[2], width) - max(box[0], 0.0))
        clipped_h = max(0.0, min(box[3], height) - max(box[1], 0.0))
        report.subject_fraction = (clipped_w * clipped_h) / float(width * height)

    focus_ids = {int(getattr(d, "id", 0)) for d in focus_descriptors}
    basis = dart.gui.make_orbit_camera_basis(camera)
    eye = np.asarray(basis.eye, dtype=float).reshape(3)
    samples = np.vstack([corners, center.reshape(1, 3)])
    report.occlusion_fraction = _occlusion_fraction(
        descriptors, focus_ids, samples, eye
    )

    # Ambiguity: distinct bodies whose screen boxes pile up while separated
    # in depth suggest a degenerate view axis (e.g. a stack seen from above).
    boxes = []
    for descriptor in descriptors:
        descriptor_corners = _descriptor_world_corners(descriptor)
        if descriptor_corners is None:
            continue
        descriptor_projected = _debug_layers.project_points(
            camera, (width, height), descriptor_corners, projection_options
        )
        # Bodies straddling the camera plane project to unbounded boxes and
        # signed depths; they cannot be judged for overlap, so skip them.
        descriptor_in_front = descriptor_projected[:, 2] > 0.0
        if not descriptor_in_front.all():
            continue
        descriptor_box = _screen_box(descriptor_projected)
        if descriptor_box is None:
            continue
        # Only overlap that is actually visible can make a view ambiguous.
        clipped_box = (
            max(descriptor_box[0], 0.0),
            max(descriptor_box[1], 0.0),
            min(descriptor_box[2], float(width)),
            min(descriptor_box[3], float(height)),
        )
        if clipped_box[2] <= clipped_box[0] or clipped_box[3] <= clipped_box[1]:
            continue
        depth = float(np.median(descriptor_projected[:, 2]))
        extent = float(
            np.linalg.norm(
                descriptor_corners.max(axis=0) - descriptor_corners.min(axis=0)
            )
        )
        boxes.append((clipped_box, depth, extent))
    worst_iou = 0.0
    for index, (box_a, depth_a, extent_a) in enumerate(boxes):
        for box_b, depth_b, extent_b in boxes[index + 1 :]:
            depth_gap = abs(depth_a - depth_b)
            min_extent = max(min(extent_a, extent_b), 1e-9)
            if depth_gap < 0.5 * min_extent:
                continue
            worst_iou = max(worst_iou, _box_iou(box_a, box_b))
    report.ambiguity_iou = worst_iou

    if report.corner_coverage < _MIN_CORNER_COVERAGE:
        report.issues.append("cropped" if report.center_visible else "off-frame")
    if report.subject_fraction < _MIN_SUBJECT_FRACTION:
        report.issues.append("too-far")
    elif report.subject_fraction > _MAX_SUBJECT_FRACTION:
        report.issues.append("too-close")
    if report.occlusion_fraction > _MAX_OCCLUSION_FRACTION:
        report.issues.append("occluded")
    if report.ambiguity_iou > _MAX_AMBIGUITY_IOU:
        report.issues.append("ambiguous")

    report.score = _score(report)
    return report


def _score(report: ViewReport) -> float:
    # Deterministic, monotone score in [0, 1]: coverage and line-of-sight
    # dominate; framing prefers a mid-band subject size; ambiguity discounts.
    framing_mid = math.sqrt(_MIN_SUBJECT_FRACTION * _MAX_SUBJECT_FRACTION)
    if report.subject_fraction <= 0.0:
        framing = 0.0
    else:
        framing = 1.0 - min(
            1.0, abs(math.log(report.subject_fraction / framing_mid)) / 4.0
        )
    visibility = 1.0 - report.occlusion_fraction
    clarity = 1.0 - report.ambiguity_iou
    return max(
        0.0, report.corner_coverage * visibility * (0.5 + 0.35 * framing + 0.15 * clarity)
    )


def frame_region(
    center: Sequence[float],
    radius: float,
    *,
    azimuth: float = 0.7853981633974483,
    elevation: float = 0.5235987755982988,
    size: tuple[int, int] = (640, 480),
    margin: float = 1.15,
    vertical_fov_degrees: float = 45.0,
) -> Any:
    """Build a camera that frames a sphere around ``center`` with margin."""
    width, height = int(size[0]), int(size[1])
    radius = max(float(radius), 1e-6)
    fov = math.radians(vertical_fov_degrees)
    aspect = max(float(width) / float(max(height, 1)), 1e-6)
    horizontal_fov = 2.0 * math.atan(math.tan(fov * 0.5) * aspect)
    distance = radius / math.sin(min(fov, horizontal_fov) * 0.5) * float(margin)
    return dart.gui.orbit_camera(
        azimuth=float(azimuth),
        elevation=float(elevation),
        distance=distance,
        target=np.asarray(center, dtype=float).reshape(3),
    )


def frame_body(
    world: Any,
    name: str | Sequence[str],
    *,
    azimuth: float = 0.7853981633974483,
    elevation: float = 0.5235987755982988,
    size: tuple[int, int] = (640, 480),
    margin: float = 1.6,
) -> Any:
    """Build a camera targeting one or more named bodies (region reframing)."""
    from . import _world_render_bridge

    descriptors = _world_render_bridge._renderables_from_world(world)
    focus_descriptors, _ = _split_focus(descriptors, name)
    corners_list = [
        corners
        for corners in (_descriptor_world_corners(d) for d in focus_descriptors)
        if corners is not None
    ]
    if not corners_list:
        raise ValueError(f"focus {name!r} has no bounded renderables to frame")
    corners = np.vstack(corners_list)
    center = 0.5 * (corners.min(axis=0) + corners.max(axis=0))
    radius = 0.5 * float(np.linalg.norm(corners.max(axis=0) - corners.min(axis=0)))
    return frame_region(
        center, radius, azimuth=azimuth, elevation=elevation, size=size, margin=margin
    )


_DEFAULT_AZIMUTHS = tuple(math.tau * i / 8.0 + math.tau / 16.0 for i in range(8))
_DEFAULT_ELEVATIONS = (0.35, 0.62, 0.95)
_DEFAULT_DISTANCE_SCALES = (1.0, 1.5)


def select_viewpoints(
    world: Any,
    size: tuple[int, int] = (640, 480),
    *,
    focus: str | Sequence[str] | None = None,
    count: int = 1,
    azimuths: Sequence[float] = _DEFAULT_AZIMUTHS,
    elevations: Sequence[float] = _DEFAULT_ELEVATIONS,
    distance_scales: Sequence[float] = _DEFAULT_DISTANCE_SCALES,
    min_azimuth_separation: float = math.tau / 8.0,
    projection_options: Any | None = None,
) -> list[ViewpointChoice]:
    """Deterministically pick the best viewpoint(s) from a candidate grid.

    Candidates are scored with :func:`assess_view`; selection greedily takes
    the highest score, then enforces azimuth diversity so additional views
    reveal new information instead of repeating the first. Everything is a
    pure function of the world state and the explicit candidate lists, so a
    recorded camera can be reproduced exactly.
    """
    from . import _world_render_bridge

    descriptors = _world_render_bridge._renderables_from_world(world)
    focus_descriptors, _ = _split_focus(descriptors, focus)
    corners_list = [
        corners
        for corners in (_descriptor_world_corners(d) for d in focus_descriptors)
        if corners is not None
    ]
    if not corners_list:
        raise ValueError("select_viewpoints requires bounded focus renderables")
    corners = np.vstack(corners_list)
    center = 0.5 * (corners.min(axis=0) + corners.max(axis=0))
    radius = 0.5 * float(np.linalg.norm(corners.max(axis=0) - corners.min(axis=0)))

    candidates: list[ViewpointChoice] = []
    for elevation in elevations:
        for azimuth in azimuths:
            for scale in distance_scales:
                camera = frame_region(
                    center,
                    radius * float(scale),
                    azimuth=float(azimuth),
                    elevation=float(elevation),
                    size=size,
                )
                report = assess_view(
                    world,
                    camera,
                    size,
                    focus=focus,
                    projection_options=projection_options,
                )
                reason = (
                    f"azimuth={azimuth:.3f} elevation={elevation:.3f} "
                    f"scale={scale:g} score={report.score:.3f}"
                    + (f" issues={','.join(report.issues)}" if report.issues else "")
                )
                candidates.append(ViewpointChoice(camera, report, reason))

    candidates.sort(key=lambda choice: (-choice.report.score, choice.reason))
    selected: list[ViewpointChoice] = []
    for choice in candidates:
        if len(selected) >= max(1, int(count)):
            break
        azimuth = choice.report.camera["azimuth"]
        too_close = any(
            _angular_distance(azimuth, other.report.camera["azimuth"])
            < min_azimuth_separation
            for other in selected
        )
        if too_close:
            continue
        choice.reason = (
            f"rank={len(selected) + 1} of {len(candidates)} candidates; "
            + choice.reason
        )
        selected.append(choice)
    # Relax diversity if it starved the selection (tiny candidate grids).
    if len(selected) < max(1, int(count)):
        for choice in candidates:
            if len(selected) >= max(1, int(count)):
                break
            if choice in selected:
                continue
            choice.reason = (
                f"rank={len(selected) + 1} (diversity relaxed); " + choice.reason
            )
            selected.append(choice)
    return selected


def _angular_distance(a: float, b: float) -> float:
    difference = (a - b) % math.tau
    return min(difference, math.tau - difference)


def install_view_quality_helpers(root: Any, gui: Any) -> None:
    gui.ViewReport = ViewReport
    gui.ViewpointChoice = ViewpointChoice
    gui.assess_view = assess_view
    gui.select_viewpoints = select_viewpoints
    gui.frame_body = frame_body
    gui.frame_region = frame_region
