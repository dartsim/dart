"""Debug overlay compositing for DART 6 headless captures.

DART 6's OSG offscreen path (WP-ASV) renders the plain scene; this module
draws world-derived debug layers — contact markers/normals/forces, body
frames, velocity arrows, trajectory polylines, and name labels — onto the
captured PNG in image space. DART 7 renders its debug overlay unlit and
always-on-top, so 2D compositing after projection is visually equivalent
evidence. Colors and arrow geometry mirror dart::gui's debug producers so
both branches speak the same visual language.

Pure Python + numpy over the classic dartpy surface; pairs with
agent_view_quality.AgentCamera for the projection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np
from _image_tools import ImageData, draw_text_rgb, read_image, write_image
from agent_view_quality import AgentCamera, project_points

# Colors mirrored from dart/gui debug producers (DART 7 dart/gui/debug.cpp).
AXIS_X_RGB = (230, 71, 71)
AXIS_Y_RGB = (79, 191, 110)
AXIS_Z_RGB = (71, 120, 235)
CONTACT_POINT_RGB = (255, 235, 97)
CONTACT_NORMAL_RGB = (255, 191, 64)
CONTACT_FORCE_RGB = (237, 79, 171)
LINEAR_VELOCITY_RGB = (82, 189, 250)
ANGULAR_VELOCITY_RGB = (189, 133, 250)
TRAJECTORY_RGB = (250, 140, 64)
LABEL_RGB = (255, 255, 255)

DEBUG_LAYERS = (
    "body_frames",
    "contacts",
    "velocities",
    "trajectories",
    "labels",
)

# DART 6's collision detector can emit sentinel/garbage contact points (seen
# in practice: coordinates ~1e101). Drawing those would produce misleading
# evidence, so contacts outside this bound are skipped and counted; the text
# oracle (contact traces) is the place to surface such anomalies.
PLAUSIBLE_COORDINATE_LIMIT = 1.0e6


@dataclass
class OverlayScene:
    """World-space overlay primitives awaiting projection."""

    segments: list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]] = field(
        default_factory=list
    )
    labels: list[tuple[np.ndarray, str]] = field(default_factory=list)
    skipped_contacts: int = 0


def _is_plausible_point(point: np.ndarray) -> bool:
    return bool(
        np.isfinite(point).all()
        and (np.abs(point) < PLAUSIBLE_COORDINATE_LIMIT).all()
    )


def _append_segment(scene: OverlayScene, start, end, rgb) -> None:
    start = np.asarray(start, dtype=float).reshape(3)
    end = np.asarray(end, dtype=float).reshape(3)
    if not (np.isfinite(start).all() and np.isfinite(end).all()):
        return
    if float(np.dot(end - start, end - start)) <= 1e-18:
        return
    scene.segments.append((start, end, rgb))


def _append_arrow(scene: OverlayScene, start, end, rgb) -> None:
    """3D arrow (shaft + two barbs) mirroring appendArrowLines in dart::gui."""
    _append_segment(scene, start, end, rgb)
    start = np.asarray(start, dtype=float).reshape(3)
    end = np.asarray(end, dtype=float).reshape(3)
    vector = end - start
    length = float(np.linalg.norm(vector))
    if not math.isfinite(length) or length <= 1e-12:
        return
    direction = vector / length
    seed = (
        np.array([0.0, 0.0, 1.0])
        if abs(direction[2]) < 0.9
        else np.array([0.0, 1.0, 0.0])
    )
    side = np.cross(direction, seed)
    side_norm = float(np.linalg.norm(side))
    if side_norm <= 1e-12:
        return
    side /= side_norm
    head_length = length * 0.25
    head_width = head_length * 0.45
    base = end - direction * head_length
    _append_segment(scene, end, base + side * head_width, rgb)
    _append_segment(scene, end, base - side * head_width, rgb)


def _iter_bodies(world: Any):
    for skeleton_index in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(skeleton_index)
        for body_index in range(skeleton.getNumBodyNodes()):
            yield skeleton.getBodyNode(body_index)


class TrajectoryTracker:
    """Records per-body COM positions to draw motion-history polylines."""

    def __init__(
        self, world: Any, bodies: Sequence[str] | None = None, max_samples: int = 512
    ) -> None:
        self._world = world
        self._bodies = list(bodies) if bodies is not None else None
        self._max_samples = int(max_samples)
        self._history: dict[str, list[np.ndarray]] = {}

    def sample(self) -> None:
        for body in _iter_bodies(self._world):
            name = str(body.getName())
            if self._bodies is not None and name not in self._bodies:
                continue
            positions = self._history.setdefault(name, [])
            positions.append(np.asarray(body.getCOM(), dtype=float).reshape(3).copy())
            if len(positions) > self._max_samples:
                del positions[0 : len(positions) - self._max_samples]

    @property
    def history(self) -> dict[str, list[np.ndarray]]:
        return self._history


def build_overlay(
    world: Any,
    layers: Sequence[str] = ("contacts", "body_frames"),
    *,
    contacts: Iterable[Any] | None = None,
    trajectories: TrajectoryTracker | dict[str, Sequence[Any]] | None = None,
    frame_axis_length: float = 0.15,
    contact_marker_half_extent: float = 0.03,
    contact_normal_length: float = 0.12,
    velocity_scale: float = 0.25,
    velocity_min_length: float = 0.05,
    velocity_max_length: float = 0.6,
    force_scale: float = 0.01,
    force_min_length: float = 0.05,
    force_max_length: float = 0.5,
) -> OverlayScene:
    """Compose world-space overlay primitives from named layers."""
    unknown = sorted(set(layers) - set(DEBUG_LAYERS))
    if unknown:
        raise ValueError(
            f"unknown debug layers {unknown}; available: {list(DEBUG_LAYERS)}"
        )
    scene = OverlayScene()

    if "body_frames" in layers:
        for body in _iter_bodies(world):
            transform = body.getWorldTransform()
            origin = np.asarray(transform.translation(), dtype=float).reshape(3)
            rotation = np.asarray(transform.rotation(), dtype=float).reshape(3, 3)
            for column, rgb in enumerate((AXIS_X_RGB, AXIS_Y_RGB, AXIS_Z_RGB)):
                axis = rotation[:, column]
                norm = float(np.linalg.norm(axis))
                if norm > 1e-12:
                    _append_segment(
                        scene, origin, origin + axis / norm * frame_axis_length, rgb
                    )

    if "contacts" in layers:
        contact_list = (
            list(contacts)
            if contacts is not None
            else list(world.getLastCollisionResult().getContacts())
        )
        for contact in contact_list:
            point = np.asarray(contact.point, dtype=float).reshape(3)
            if not _is_plausible_point(point):
                scene.skipped_contacts += 1
                continue
            for axis in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])):
                _append_segment(
                    scene,
                    point - axis * contact_marker_half_extent,
                    point + axis * contact_marker_half_extent,
                    CONTACT_POINT_RGB,
                )
            normal = np.asarray(contact.normal, dtype=float).reshape(3)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-9:
                _append_arrow(
                    scene,
                    point,
                    point + normal / norm * contact_normal_length,
                    CONTACT_NORMAL_RGB,
                )
            force = np.asarray(contact.force, dtype=float).reshape(3)
            force_norm = float(np.linalg.norm(force))
            if force_norm > 1e-9:
                length = float(
                    np.clip(
                        force_norm * force_scale, force_min_length, force_max_length
                    )
                )
                _append_arrow(
                    scene, point, point + force / force_norm * length, CONTACT_FORCE_RGB
                )

    if "velocities" in layers:
        for body in _iter_bodies(world):
            com = np.asarray(body.getCOM(), dtype=float).reshape(3)
            linear = np.asarray(
                body.getCOMLinearVelocity(), dtype=float
            ).reshape(3)
            magnitude = float(np.linalg.norm(linear))
            if np.isfinite(linear).all() and magnitude > 1e-9:
                length = float(
                    np.clip(
                        magnitude * velocity_scale,
                        velocity_min_length,
                        velocity_max_length,
                    )
                )
                _append_arrow(
                    scene, com, com + linear / magnitude * length, LINEAR_VELOCITY_RGB
                )
            origin = np.asarray(
                body.getWorldTransform().translation(), dtype=float
            ).reshape(3)
            angular = np.asarray(body.getAngularVelocity(), dtype=float).reshape(3)
            magnitude = float(np.linalg.norm(angular))
            if np.isfinite(angular).all() and magnitude > 1e-9:
                length = float(
                    np.clip(
                        magnitude * velocity_scale,
                        velocity_min_length,
                        velocity_max_length,
                    )
                )
                _append_arrow(
                    scene,
                    origin,
                    origin + angular / magnitude * length,
                    ANGULAR_VELOCITY_RGB,
                )

    if "trajectories" in layers:
        if trajectories is None:
            raise ValueError(
                "the 'trajectories' layer needs recorded motion history: pass "
                "trajectories=TrajectoryTracker(world) (call sample() each "
                "step) or a {body_name: positions} mapping"
            )
        history = (
            trajectories.history
            if isinstance(trajectories, TrajectoryTracker)
            else trajectories
        )
        for name in sorted(history):
            positions = [
                np.asarray(p, dtype=float).reshape(3) for p in history[name]
            ]
            for start, end in zip(positions, positions[1:]):
                _append_segment(scene, start, end, TRAJECTORY_RGB)

    if "labels" in layers:
        for body in _iter_bodies(world):
            origin = np.asarray(
                body.getWorldTransform().translation(), dtype=float
            ).reshape(3)
            scene.labels.append((origin, str(body.getName())))

    return scene


# --- Rasterization ----------------------------------------------------------


def _draw_line_rgb(
    pixels: bytearray,
    width: int,
    height: int,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    rgb: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    xs = np.linspace(x0, x1, steps)
    ys = np.linspace(y0, y1, steps)
    half = max(int(thickness) // 2, 0)
    color = bytes(rgb)
    for x, y in zip(xs, ys):
        xi, yi = int(round(x)), int(round(y))
        for dy in range(-half, half + 1):
            py = yi + dy
            if py < 0 or py >= height:
                continue
            for dx in range(-half, half + 1):
                px = xi + dx
                if px < 0 or px >= width:
                    continue
                offset = (py * width + px) * 3
                pixels[offset : offset + 3] = color


def composite_overlay(
    image: ImageData,
    scene: OverlayScene,
    camera: AgentCamera,
    *,
    thickness: int = 2,
    label_scale: int = 2,
) -> ImageData:
    """Project overlay primitives with ``camera`` and draw them onto a copy."""
    width, height = image.width, image.height
    pixels = bytearray(image.pixels)
    for start, end, rgb in scene.segments:
        projected = project_points(camera, (width, height), [start, end])
        if projected[0, 2] <= 0.0 or projected[1, 2] <= 0.0:
            continue
        _draw_line_rgb(
            pixels,
            width,
            height,
            projected[0, 0],
            projected[0, 1],
            projected[1, 0],
            projected[1, 1],
            rgb,
            thickness,
        )
    for anchor, text in scene.labels:
        projected = project_points(camera, (width, height), [anchor])[0]
        if projected[2] <= 0.0:
            continue
        if not (math.isfinite(projected[0]) and math.isfinite(projected[1])):
            continue
        draw_text_rgb(
            pixels,
            width,
            height,
            text,
            (int(round(projected[0])) + 3, int(round(projected[1])) - 3 * label_scale),
            LABEL_RGB,
            scale=label_scale,
        )
    return ImageData(path=image.path, width=width, height=height, pixels=bytes(pixels))


def composite_overlay_file(
    png_path: Any,
    scene: OverlayScene,
    camera: AgentCamera,
    *,
    thickness: int = 2,
    label_scale: int = 2,
) -> None:
    """Read a captured PNG, composite the overlay, and write it back."""
    image = read_image(png_path)
    annotated = composite_overlay(
        image, scene, camera, thickness=thickness, label_scale=label_scale
    )
    write_image(png_path, annotated)
