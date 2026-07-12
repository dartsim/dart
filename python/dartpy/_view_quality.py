"""View-quality assessment and adaptive viewpoint selection.

Geometry-first checks (projection and CPU ray picks, no GPU needed) that let
a headless agent detect inadequate captures — cropped, too close/far,
occluded, or ambiguous views — and deterministically pick better ones. The
projection, occlusion, and ambiguity geometry lives in ``dart::gui``
(``gui.assess_view_quality``); this module is the thin Python orchestration
that resolves focus names to descriptor ids, adapts the core report into the
public ``ViewReport`` dataclass, and drives candidate selection. Pixel-level
checks (blank/contrast) stay in the image tooling; this module answers "is this
camera worth rendering from?" before pixels exist, mirroring the repo's
text-first verification policy.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from typing import Any, Sequence

import dartpy as dart
import numpy as np


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


def assess_view(
    world: Any,
    camera: Any,
    size: tuple[int, int] = (640, 480),
    *,
    focus: str | Sequence[str] | None = None,
    projection_options: Any | None = None,
) -> ViewReport:
    """Assess one camera against the world's renderables, without rendering.

    Resolves the focus names to descriptor ids, delegates the projection,
    occlusion, and ambiguity geometry to the core ``gui.assess_view_quality``,
    and adapts the result into the public :class:`ViewReport`.
    """
    from . import _world_render_bridge

    width, height = int(size[0]), int(size[1])
    descriptors = _world_render_bridge._renderables_from_world(world)
    focus_descriptors, focus_names = _split_focus(descriptors, focus)
    focus_ids = [int(getattr(d, "id", 0)) for d in focus_descriptors]

    if projection_options is None:
        projection_options = dart.gui.ProjectionOptions()

    core_report = dart.gui.assess_view_quality(
        descriptors, camera, width, height, focus_ids, projection_options
    )

    return ViewReport(
        camera=_camera_params(camera),
        size=(width, height),
        focus=focus_names,
        corner_coverage=float(core_report.corner_coverage),
        subject_fraction=float(core_report.subject_fraction),
        center_visible=bool(core_report.center_visible),
        occlusion_fraction=float(core_report.occlusion_fraction),
        ambiguity_iou=float(core_report.ambiguity_iou),
        issues=list(core_report.issues),
        score=float(core_report.score),
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

    # Acceptable views rank ahead of any failing view regardless of score:
    # a high-scoring-but-cropped candidate must not shadow a usable one and
    # make the downstream capture gate abort needlessly.
    candidates.sort(
        key=lambda choice: (
            not choice.report.acceptable,
            -choice.report.score,
            choice.reason,
        )
    )
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
