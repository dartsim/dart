"""View-quality assessment and adaptive viewpoint selection for DART 6.

Geometry-first checks that let a headless agent detect inadequate captures —
cropped, too far/close, occluded, or ambiguous views — and deterministically
pick better ones, mirroring the DART 7 `dart.gui.assess_view` /
`select_viewpoints` workflow over this branch's OSG offscreen path
(`Viewer.captureOffscreen(eye, center, up, fovYDeg, ...)` from WP-ASV).

Body bounds come from the core ``Shape.getBoundingBox()`` local AABB and
occlusion from core raycasts against the world's real collision geometry (the
Bullet backend, DART 6's only raycast provider); projection uses the same
look-at + vertical-FOV model the capture helper pins. No GL context is needed
to assess a view.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

SCHEMA_VERSION = "dart.view_report/v1"

# Subject screen-area fractions outside this band flag too-far / too-close.
MIN_SUBJECT_FRACTION = 0.015
MAX_SUBJECT_FRACTION = 0.75
MIN_CORNER_COVERAGE = 0.999
MAX_OCCLUSION_FRACTION = 0.35
MAX_AMBIGUITY_IOU = 0.55
# Contact bias as a fraction of the eye->sample ray: DART 6's contact solver
# leaves millimetre-scale penetration at rest, so a sampled corner can sit just
# inside a neighbor's surface. A nearest hit within this last stretch of the ray
# counts as contact, not occlusion; real occluders sit far closer to the eye.
OCCLUSION_CONTACT_BIAS = 0.02

DEFAULT_FOVY_DEG = 30.0


@dataclass
class AgentCamera:
    """Look-at camera matching Viewer.captureOffscreen's parameters."""

    eye: np.ndarray
    center: np.ndarray
    up: np.ndarray
    fovy_deg: float = DEFAULT_FOVY_DEG

    def to_json(self) -> dict[str, Any]:
        return {
            "eye": [float(v) for v in self.eye],
            "center": [float(v) for v in self.center],
            "up": [float(v) for v in self.up],
            "fovy_deg": float(self.fovy_deg),
        }


def orbit_camera(
    target: Sequence[float],
    distance: float,
    azimuth: float = math.tau / 8.0,
    elevation: float = math.tau / 12.0,
    fovy_deg: float = DEFAULT_FOVY_DEG,
) -> AgentCamera:
    """Build a z-up look-at camera from orbit parameters (radians)."""
    center = np.asarray(target, dtype=float).reshape(3)
    offset = distance * np.array(
        [
            math.cos(elevation) * math.cos(azimuth),
            math.cos(elevation) * math.sin(azimuth),
            math.sin(elevation),
        ]
    )
    return AgentCamera(
        eye=center + offset,
        center=center,
        up=np.array([0.0, 0.0, 1.0]),
        fovy_deg=float(fovy_deg),
    )


def frame_region(
    center: Sequence[float],
    radius: float,
    azimuth: float = math.tau / 8.0,
    elevation: float = math.tau / 12.0,
    fovy_deg: float = DEFAULT_FOVY_DEG,
    margin: float = 1.15,
) -> AgentCamera:
    """Camera framing a sphere, matching defaultAgentCamera's distance law."""
    radius = max(float(radius), 1e-6)
    distance = radius / math.sin(math.radians(fovy_deg) * 0.5) * float(margin)
    return orbit_camera(center, distance, azimuth, elevation, fovy_deg)


def _camera_basis(camera: AgentCamera) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = camera.center - camera.eye
    forward = forward / max(float(np.linalg.norm(forward)), 1e-12)
    right = np.cross(forward, camera.up)
    right = right / max(float(np.linalg.norm(right)), 1e-12)
    true_up = np.cross(right, forward)
    return forward, right, true_up


def project_points(
    camera: AgentCamera, size: tuple[int, int], points: Sequence[Any]
) -> np.ndarray:
    """Project world points to (x_px, y_px, view_depth); origin top-left."""
    width, height = int(size[0]), int(size[1])
    forward, right, true_up = _camera_basis(camera)
    focal = 1.0 / math.tan(math.radians(camera.fovy_deg) * 0.5)
    aspect = max(float(width) / float(max(height, 1)), 1e-6)

    array = np.asarray(
        [np.asarray(p, dtype=float).reshape(3) for p in points], dtype=float
    ).reshape(-1, 3)
    offsets = array - camera.eye
    depth = offsets @ forward
    safe_depth = np.where(np.abs(depth) < 1e-12, 1e-12, depth)
    ndc_x = (offsets @ right) / safe_depth * (focal / aspect)
    ndc_y = (offsets @ true_up) / safe_depth * focal
    pixel_x = (ndc_x * 0.5 + 0.5) * width
    pixel_y = (1.0 - (ndc_y * 0.5 + 0.5)) * height
    return np.column_stack([pixel_x, pixel_y, depth])


# --- World introspection ----------------------------------------------------


@dataclass
class BodyBounds:
    """World-space oriented bounds of one body (all its shape nodes)."""

    name: str
    corners: np.ndarray  # (8*k, 3) world-space corners
    aabb_min: np.ndarray
    aabb_max: np.ndarray


def _iter_shape_owners(world: Any):
    """Yield (name, shape-frame) pairs for skeleton bodies and simple frames."""
    for skeleton_index in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(skeleton_index)
        for body_index in range(skeleton.getNumBodyNodes()):
            body = skeleton.getBodyNode(body_index)
            for shape_node in body.getShapeNodes():
                yield str(body.getName()), shape_node
    for frame_index in range(world.getNumSimpleFrames()):
        frame = world.getSimpleFrame(frame_index)
        if frame.getShape() is not None:
            yield str(frame.getName()), frame


# Shapes that are not bounded solids: an infinite ground plane and the
# overlay's own line segments must never contribute assessment bounds.
_UNBOUNDED_SHAPE_TYPES = frozenset({"PlaneShape", "LineSegmentShape"})


def _shape_local_aabb(shape: Any) -> tuple[np.ndarray, np.ndarray] | None:
    """Shape-local AABB (min, max) from the core ``Shape.getBoundingBox()``.

    ``math::BoundingBox`` is bound in dartpy, so the engine-computed local
    bounds are the single source of truth. They may be off-center (e.g. a
    mesh), so the full min/max box is used rather than an origin-centered
    half-extent. Shapes whose core bounds are non-finite (an infinite plane)
    or degenerate (an empty shape reports inverted ``DBL_MAX`` bounds) return
    None and are skipped.
    """
    if type(shape).__name__ in _UNBOUNDED_SHAPE_TYPES:
        return None
    bounding_box = shape.getBoundingBox()
    low = np.asarray(bounding_box.getMin(), dtype=float).reshape(3)
    high = np.asarray(bounding_box.getMax(), dtype=float).reshape(3)
    if not (np.isfinite(low).all() and np.isfinite(high).all()):
        return None
    if not np.all(high >= low):
        return None
    return low, high


def _shape_frame_world_corners(shape_frame: Any) -> np.ndarray | None:
    shape = shape_frame.getShape()
    if shape is None:
        return None
    aabb = _shape_local_aabb(shape)
    if aabb is None:
        return None
    low, high = aabb
    transform = shape_frame.getWorldTransform()
    rotation = np.asarray(transform.rotation(), dtype=float).reshape(3, 3)
    translation = np.asarray(transform.translation(), dtype=float).reshape(3)
    local = np.array(
        [
            [x, y, z]
            for x in (low[0], high[0])
            for y in (low[1], high[1])
            for z in (low[2], high[2])
        ]
    )
    return local @ rotation.T + translation


def body_bounds(world: Any) -> list[BodyBounds]:
    corner_sets: dict[str, list[np.ndarray]] = {}
    for name, shape_frame in _iter_shape_owners(world):
        corners = _shape_frame_world_corners(shape_frame)
        if corners is not None:
            corner_sets.setdefault(name, []).append(corners)
    result: list[BodyBounds] = []
    for name in sorted(corner_sets):
        corners = np.vstack(corner_sets[name])
        result.append(
            BodyBounds(
                name=name,
                corners=corners,
                aabb_min=corners.min(axis=0),
                aabb_max=corners.max(axis=0),
            )
        )
    return result


def _split_focus(
    bounds: list[BodyBounds], focus: str | Sequence[str] | None
) -> tuple[list[BodyBounds], list[str]]:
    if focus is None:
        return bounds, sorted(b.name for b in bounds)
    wanted = {focus} if isinstance(focus, str) else set(focus)

    def matches(name: str) -> bool:
        if name in wanted:
            return True
        # Allow an indexed suffix only ("box" matches "box_0", never
        # "box_holder"): over-matching would hide real occluders.
        return any(
            re.fullmatch(rf"{re.escape(target)}_\d+", name) for target in wanted
        )

    matched = [b for b in bounds if matches(b.name)]
    if not matched:
        raise ValueError(
            f"focus {sorted(wanted)} matched no bodies; "
            f"available: {sorted(b.name for b in bounds)}"
        )
    return matched, sorted(b.name for b in matched)


def _dartpy() -> Any:
    import dartpy

    return dartpy


def _bullet_detector(dart: Any) -> Any:
    """Core Bullet collision detector, DART 6's only raycast-capable backend."""
    factory = getattr(dart.collision, "BulletCollisionDetector", None)
    if factory is None:
        raise RuntimeError(
            "view-quality occlusion uses core raycast, which DART 6 supports "
            "only through the Bullet collision backend; this dartpy build has "
            "no BulletCollisionDetector. Rebuild dartpy with Bullet enabled."
        )
    return factory()


def _hit_body_name(hit: Any) -> str | None:
    """Name of the body owning a ray hit, or None if it cannot be attributed."""
    shape_node = hit.mCollisionObject.getShapeFrame().asShapeNode()
    if shape_node is None:
        return None
    body = shape_node.getBodyNodePtr()
    return str(body.getName()) if body is not None else None


# --- Assessment -------------------------------------------------------------


@dataclass
class ViewReport:
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
            "schema_version": SCHEMA_VERSION,
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
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0.0 else 0.0


def _raycast_occlusion(
    world: Any,
    focus_names: set[str],
    samples: np.ndarray,
    eye: np.ndarray,
) -> float:
    """Fraction of sample sightlines a non-focus body blocks, via core raycast.

    Casts a core ray from the eye to each sampled focus point against the
    world's real collision geometry (Bullet backend) and takes the nearest hit.
    A neighbor blocks the sightline when the nearest hit belongs to another body
    and sits meaningfully in front of the sample; the contact bias ignores the
    last stretch of the ray so resting-contact penetration (a corner a couple of
    millimetres inside the ground) does not read as occlusion. This mirrors
    DART 7's nearest-renderable pick against the same engine geometry.
    """
    dart = _dartpy()
    group = _bullet_detector(dart).createCollisionGroup()
    for index in range(world.getNumSkeletons()):
        group.addShapeFramesOf(world.getSkeleton(index))
    option = dart.collision.RaycastOption()
    option.mSortByClosest = True

    blocked = 0
    total = 0
    origin = np.asarray(eye, dtype=float).reshape(3)
    for sample in samples:
        target = np.asarray(sample, dtype=float).reshape(3)
        if float(np.linalg.norm(target - origin)) <= 1e-9:
            continue
        total += 1
        result = dart.collision.RaycastResult()
        if not group.raycast(origin, target, option, result):
            continue
        if not result.hasHit():
            continue
        hit = result.mRayHits[0]
        body = _hit_body_name(hit)
        if body is None or body in focus_names:
            continue
        if float(hit.mFraction) < 1.0 - OCCLUSION_CONTACT_BIAS:
            blocked += 1
    return blocked / total if total else 0.0


def assess_view(
    world: Any,
    camera: AgentCamera,
    size: tuple[int, int] = (640, 480),
    focus: str | Sequence[str] | None = None,
) -> ViewReport:
    """Assess one camera against the world's body bounds, without rendering."""
    width, height = int(size[0]), int(size[1])
    bounds = body_bounds(world)
    focus_bounds, focus_names = _split_focus(bounds, focus)
    report = ViewReport(
        camera=camera.to_json(), size=(width, height), focus=focus_names
    )
    if not focus_bounds:
        report.issues.append("no-bounded-focus")
        return report

    corners = np.vstack([b.corners for b in focus_bounds])
    projected = project_points(camera, (width, height), corners)
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
    center_projected = project_points(camera, (width, height), [center])[0]
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

    samples = np.vstack([corners, center.reshape(1, 3)])
    report.occlusion_fraction = _raycast_occlusion(
        world, set(focus_names), samples, camera.eye
    )

    # Ambiguity: visible, depth-separated bodies stacking up on screen.
    boxes = []
    for body in bounds:
        body_projected = project_points(camera, (width, height), body.corners)
        if not (body_projected[:, 2] > 0.0).all():
            continue
        body_box = _screen_box(body_projected)
        if body_box is None:
            continue
        clipped = (
            max(body_box[0], 0.0),
            max(body_box[1], 0.0),
            min(body_box[2], float(width)),
            min(body_box[3], float(height)),
        )
        if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
            continue
        depth = float(np.median(body_projected[:, 2]))
        extent = float(np.linalg.norm(body.aabb_max - body.aabb_min))
        boxes.append((clipped, depth, extent))
    worst_iou = 0.0
    for index, (box_a, depth_a, extent_a) in enumerate(boxes):
        for box_b, depth_b, extent_b in boxes[index + 1 :]:
            if abs(depth_a - depth_b) < 0.5 * max(min(extent_a, extent_b), 1e-9):
                continue
            worst_iou = max(worst_iou, _box_iou(box_a, box_b))
    report.ambiguity_iou = worst_iou

    if report.corner_coverage < MIN_CORNER_COVERAGE:
        report.issues.append("cropped" if report.center_visible else "off-frame")
    if report.subject_fraction < MIN_SUBJECT_FRACTION:
        report.issues.append("too-far")
    elif report.subject_fraction > MAX_SUBJECT_FRACTION:
        report.issues.append("too-close")
    if report.occlusion_fraction > MAX_OCCLUSION_FRACTION:
        report.issues.append("occluded")
    if report.ambiguity_iou > MAX_AMBIGUITY_IOU:
        report.issues.append("ambiguous")
    report.score = _score(report)
    return report


def _score(report: ViewReport) -> float:
    framing_mid = math.sqrt(MIN_SUBJECT_FRACTION * MAX_SUBJECT_FRACTION)
    if report.subject_fraction <= 0.0:
        framing = 0.0
    else:
        framing = 1.0 - min(
            1.0, abs(math.log(report.subject_fraction / framing_mid)) / 4.0
        )
    visibility = 1.0 - report.occlusion_fraction
    clarity = 1.0 - report.ambiguity_iou
    return max(
        0.0,
        report.corner_coverage * visibility * (0.5 + 0.35 * framing + 0.15 * clarity),
    )


def frame_body(
    world: Any,
    name: str | Sequence[str],
    azimuth: float = math.tau / 8.0,
    elevation: float = math.tau / 12.0,
    fovy_deg: float = DEFAULT_FOVY_DEG,
    margin: float = 1.6,
) -> AgentCamera:
    """Camera targeting one or more named bodies (region reframing)."""
    bounds = body_bounds(world)
    focus_bounds, _ = _split_focus(bounds, name)
    corners = np.vstack([b.corners for b in focus_bounds])
    center = 0.5 * (corners.min(axis=0) + corners.max(axis=0))
    radius = 0.5 * float(np.linalg.norm(corners.max(axis=0) - corners.min(axis=0)))
    return frame_region(center, radius, azimuth, elevation, fovy_deg, margin)


@dataclass
class ViewpointChoice:
    camera: AgentCamera
    report: ViewReport
    reason: str


_DEFAULT_AZIMUTHS = tuple(math.tau * i / 8.0 + math.tau / 16.0 for i in range(8))
_DEFAULT_ELEVATIONS = (0.35, 0.62, 0.95)
_DEFAULT_DISTANCE_SCALES = (1.0, 1.5)


def select_viewpoints(
    world: Any,
    size: tuple[int, int] = (640, 480),
    focus: str | Sequence[str] | None = None,
    count: int = 1,
    azimuths: Sequence[float] = _DEFAULT_AZIMUTHS,
    elevations: Sequence[float] = _DEFAULT_ELEVATIONS,
    distance_scales: Sequence[float] = _DEFAULT_DISTANCE_SCALES,
    fovy_deg: float = DEFAULT_FOVY_DEG,
    min_azimuth_separation: float = math.tau / 8.0,
) -> list[ViewpointChoice]:
    """Deterministically pick the best viewpoint(s) from a candidate grid."""
    bounds = body_bounds(world)
    focus_bounds, _ = _split_focus(bounds, focus)
    corners = np.vstack([b.corners for b in focus_bounds])
    center = 0.5 * (corners.min(axis=0) + corners.max(axis=0))
    radius = 0.5 * float(np.linalg.norm(corners.max(axis=0) - corners.min(axis=0)))

    candidates: list[ViewpointChoice] = []
    for elevation in elevations:
        for azimuth in azimuths:
            for scale in distance_scales:
                camera = frame_region(
                    center, radius * float(scale), azimuth, elevation, fovy_deg
                )
                report = assess_view(world, camera, size, focus=focus)
                reason = (
                    f"azimuth={azimuth:.3f} elevation={elevation:.3f} "
                    f"scale={scale:g} score={report.score:.3f}"
                    + (f" issues={','.join(report.issues)}" if report.issues else "")
                )
                candidates.append(ViewpointChoice(camera, report, reason))

    candidates.sort(key=lambda choice: (-choice.report.score, choice.reason))

    def camera_azimuth(choice: ViewpointChoice) -> float:
        offset = np.asarray(choice.camera.eye) - np.asarray(choice.camera.center)
        return math.atan2(offset[1], offset[0])

    selected: list[ViewpointChoice] = []
    for choice in candidates:
        if len(selected) >= max(1, int(count)):
            break
        azimuth = camera_azimuth(choice)
        if any(
            _angular_distance(azimuth, camera_azimuth(other))
            < min_azimuth_separation
            for other in selected
        ):
            continue
        choice.reason = (
            f"rank={len(selected) + 1} of {len(candidates)} candidates; "
            + choice.reason
        )
        selected.append(choice)
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
