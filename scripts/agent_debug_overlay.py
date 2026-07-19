"""Engine-rendered debug overlays for DART 6 headless captures.

This module builds world-derived debug layers — a ground grid, the world frame,
contact markers/normals/forces, body frames, velocity arrows, centers of mass,
inertia-equivalent boxes, collision bounding boxes, trajectory polylines, and
name labels — and renders them *through the DART core OSG pipeline* via a
``dart.gui.osg.DebugOverlay`` viewer attachment. Segments become always-on-top
overlay lines and labels become world-anchored osgText, both drawn unlit with
depth testing disabled in a late render bin, so the debug primitives stay
legible on top of the geometry they annotate (matching DART 7's core debug
overlay treatment) instead of being buried in depth or composited onto the PNG.
The capture harness populates the overlay, renders through ``captureOffscreen``,
then clears it. Colors and arrow geometry mirror dart::gui's debug producers so
DART 6 and DART 7 speak the same visual language.

``build_overlay`` and ``OverlayScene`` stay pure Python + numpy over the classic
dartpy surface; only the rendering backend (``populate_overlay``) touches the
OSG scene.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

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
# Parity layers mirrored from DART 7 dart/gui/debug.cpp float colors.
GRID_RGB = (117, 125, 128)  # rgba(0.46, 0.49, 0.5)
COM_RGB = (56, 209, 219)  # DART 7 COM teal (0.22, 0.82, 0.86)
INERTIA_RGB = (148, 112, 242)  # DART 7 inertia purple (0.58, 0.44, 0.95)
COLLISION_BOUNDS_RGB = (51, 219, 110)  # DART 7 bounds green (0.2, 0.86, 0.43)
# osgText labels render on the Viewer's light-gray (0.9) background, so a dark
# slate reads far more clearly than the white DART 7 uses on its dark overlay.
LABEL_RGB = (33, 33, 40)

DEBUG_LAYERS = (
    "grid",
    "world_frame",
    "body_frames",
    "contacts",
    "velocities",
    "coms",
    "inertia_boxes",
    "collision_bounds",
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
        np.isfinite(point).all() and (np.abs(point) < PLAUSIBLE_COORDINATE_LIMIT).all()
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


def _append_box(scene: OverlayScene, center, edges, rgb) -> None:
    """Draw the 12 edges of a box from its center and three half-extent edge
    vectors (each ``edges[k] = half_extent_k * axis_k`` in world coordinates).

    The eight corners are indexed by the three sign bits of ``center +/-
    edges[0] +/- edges[1] +/- edges[2]``; an edge joins two corners that differ
    in exactly one axis, giving the 12 wireframe segments.
    """
    center = np.asarray(center, dtype=float).reshape(3)
    vectors = [np.asarray(edge, dtype=float).reshape(3) for edge in edges]
    corners = []
    for bits in range(8):
        corner = center.copy()
        for axis in range(3):
            sign = 1.0 if (bits >> axis) & 1 else -1.0
            corner = corner + sign * vectors[axis]
        corners.append(corner)
    for bits in range(8):
        for axis in range(3):
            if not (bits >> axis) & 1:
                _append_segment(scene, corners[bits], corners[bits | (1 << axis)], rgb)


def _iter_bodies(world: Any):
    for skeleton_index in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(skeleton_index)
        for body_index in range(skeleton.getNumBodyNodes()):
            yield skeleton.getBodyNode(body_index)


class TrajectoryTracker:
    """Records per-body COM positions to draw motion-history polylines.

    Histories are keyed by the world-unique ``skeleton:body`` key so bodies
    that share a name across skeletons each get their own polyline instead of
    one merged sequence jumping between COMs. The ``bodies`` filter accepts
    either bare body names (matching every instance) or qualified keys.
    """

    def __init__(
        self, world: Any, bodies: Sequence[str] | None = None, max_samples: int = 512
    ) -> None:
        self._world = world
        self._bodies = list(bodies) if bodies is not None else None
        self._max_samples = int(max_samples)
        self._history: dict[str, list[np.ndarray]] = {}

    def sample(self) -> None:
        for skeleton_index in range(self._world.getNumSkeletons()):
            skeleton = self._world.getSkeleton(skeleton_index)
            skeleton_name = str(skeleton.getName())
            for body_index in range(skeleton.getNumBodyNodes()):
                body = skeleton.getBodyNode(body_index)
                name = str(body.getName())
                key = f"{skeleton_name}:{name}"
                if self._bodies is not None and not (
                    name in self._bodies or key in self._bodies
                ):
                    continue
                positions = self._history.setdefault(key, [])
                positions.append(
                    np.asarray(body.getCOM(), dtype=float).reshape(3).copy()
                )
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
    grid_half_extent: float = 4.0,
    grid_spacing: float = 0.5,
    grid_z: float = 0.0,
    world_frame_axis_length: float = 0.5,
    com_marker_radius: float = 0.05,
    inertia_box_scale: float = 1.0,
) -> OverlayScene:
    """Compose world-space overlay primitives from named layers."""
    unknown = sorted(set(layers) - set(DEBUG_LAYERS))
    if unknown:
        raise ValueError(
            f"unknown debug layers {unknown}; available: {list(DEBUG_LAYERS)}"
        )
    scene = OverlayScene()

    if "grid" in layers:
        half = float(grid_half_extent)
        spacing = float(grid_spacing)
        z = float(grid_z)
        if spacing > 1e-9 and half > 0.0:
            count = int(math.floor(half / spacing + 1e-9))
            for index in range(-count, count + 1):
                offset = index * spacing
                _append_segment(scene, [-half, offset, z], [half, offset, z], GRID_RGB)
                _append_segment(scene, [offset, -half, z], [offset, half, z], GRID_RGB)

    if "world_frame" in layers:
        origin = np.zeros(3)
        for axis, rgb in (
            (np.array([1.0, 0.0, 0.0]), AXIS_X_RGB),
            (np.array([0.0, 1.0, 0.0]), AXIS_Y_RGB),
            (np.array([0.0, 0.0, 1.0]), AXIS_Z_RGB),
        ):
            _append_segment(scene, origin, origin + axis * world_frame_axis_length, rgb)

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
            linear = np.asarray(body.getCOMLinearVelocity(), dtype=float).reshape(3)
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

    if "coms" in layers:
        for body in _iter_bodies(world):
            com = np.asarray(body.getCOM(), dtype=float).reshape(3)
            if not np.isfinite(com).all():
                continue
            for axis in (
                np.array([1.0, 0.0, 0.0]),
                np.array([0.0, 1.0, 0.0]),
                np.array([0.0, 0.0, 1.0]),
            ):
                _append_segment(
                    scene,
                    com - axis * com_marker_radius,
                    com + axis * com_marker_radius,
                    COM_RGB,
                )

    if "inertia_boxes" in layers:
        for body in _iter_bodies(world):
            mass = float(body.getMass())
            if not math.isfinite(mass) or mass <= 0.0:
                continue
            moment = np.asarray(body.getInertia().getMoment(), dtype=float).reshape(
                3, 3
            )
            moment = 0.5 * (moment + moment.T)
            if not np.isfinite(moment).all():
                continue
            # Eigendecompose the symmetric moment: eigenvalues are the principal
            # moments, eigenvectors the principal axes in the body frame. For a
            # solid box of mass m the principal moment about axis i is
            # (m/3)(sum(other half-extents^2)); inverting gives the box that
            # reproduces this inertia, half-extent_i = sqrt(6/m*(sum-2*I_i))/2.
            eigenvalues, eigenvectors = np.linalg.eigh(moment)
            total = float(np.sum(eigenvalues))
            half_extents = [
                math.sqrt(max(0.0, (6.0 / mass) * (total - 2.0 * value)))
                / 2.0
                * inertia_box_scale
                for value in eigenvalues
            ]
            com = np.asarray(body.getCOM(), dtype=float).reshape(3)
            rotation = np.asarray(
                body.getWorldTransform().rotation(), dtype=float
            ).reshape(3, 3)
            principal_world = rotation @ eigenvectors
            edges = [principal_world[:, axis] * half_extents[axis] for axis in range(3)]
            _append_box(scene, com, edges, INERTIA_RGB)

    if "collision_bounds" in layers:
        for body in _iter_bodies(world):
            for shape_index in range(body.getNumShapeNodes()):
                shape_node = body.getShapeNode(shape_index)
                # Visual-only helper shapes are invisible to the collision
                # detector; drawing their bounds would misrepresent what
                # contact/penetration reviews are actually checking.
                if shape_node.getCollisionAspect() is None:
                    continue
                shape = shape_node.getShape()
                if shape is None:
                    continue
                bounding = shape.getBoundingBox()
                minimum = np.asarray(bounding.getMin(), dtype=float).reshape(3)
                maximum = np.asarray(bounding.getMax(), dtype=float).reshape(3)
                if not (np.isfinite(minimum).all() and np.isfinite(maximum).all()):
                    continue
                if np.any(maximum < minimum):
                    continue
                transform = shape_node.getWorldTransform()
                origin = np.asarray(transform.translation(), dtype=float).reshape(3)
                rotation = np.asarray(transform.rotation(), dtype=float).reshape(3, 3)
                center_local = 0.5 * (minimum + maximum)
                half_local = 0.5 * (maximum - minimum)
                center_world = origin + rotation @ center_local
                edges = [rotation[:, axis] * half_local[axis] for axis in range(3)]
                _append_box(scene, center_world, edges, COLLISION_BOUNDS_RGB)

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
            positions = [np.asarray(p, dtype=float).reshape(3) for p in history[name]]
            for start, end in zip(positions, positions[1:]):
                _append_segment(scene, start, end, TRAJECTORY_RGB)

    if "labels" in layers:
        for body in _iter_bodies(world):
            origin = np.asarray(
                body.getWorldTransform().translation(), dtype=float
            ).reshape(3)
            scene.labels.append((origin, str(body.getName())))

    return scene


# --- Engine rendering backend -----------------------------------------------

# Default label height in world units when the caller does not scale it to the
# scene; comparable to the frame-axis length so labels read at the same scale as
# the axes they sit beside.
DEFAULT_LABEL_CHARACTER_SIZE = 0.1

# Font files tried, in order, when none is set explicitly. osgText renders
# nothing without a usable font, so the harness resolves one from the runtime
# environment (the pixi/conda env ships DejaVuSans).
_FONT_CANDIDATES = ("DejaVuSans.ttf", "Arial.ttf", "arial.ttf", "Vera.ttf")


def _color01(rgb: tuple[int, int, int]) -> list[float]:
    return [float(channel) / 255.0 for channel in rgb]


def find_default_font() -> str | None:
    """Locate a TrueType font for osgText labels, or None for osgText's default.

    Searches the active conda/pixi environment's font directory and the common
    system DejaVu locations. Returning an absolute path keeps label rendering
    deterministic across hosts.
    """
    import os

    search_dirs: list[Path] = []
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        search_dirs.append(Path(prefix) / "fonts")
    search_dirs += [
        Path("/usr/share/fonts/truetype/dejavu"),
        Path("/usr/share/fonts/dejavu"),
        Path("/usr/share/fonts"),
    ]
    for directory in search_dirs:
        for name in _FONT_CANDIDATES:
            candidate = directory / name
            if candidate.is_file():
                return str(candidate)
    # Last resort: a recursive scan of the env font dir for any candidate.
    if prefix:
        for name in _FONT_CANDIDATES:
            matches = sorted((Path(prefix)).rglob(name))
            if matches:
                return str(matches[0])
    return None


def populate_overlay(
    overlay: Any,
    scene: OverlayScene,
    *,
    label_color: tuple[int, int, int] = LABEL_RGB,
    character_size: float = DEFAULT_LABEL_CHARACTER_SIZE,
) -> tuple[int, int]:
    """Load an ``OverlayScene`` into a ``dart.gui.osg.DebugOverlay`` attachment.

    Segments become always-on-top overlay lines carrying their per-layer color,
    and labels become world-anchored osgText, both rendered by the engine unlit
    and depth-test disabled so debug primitives stay legible over the geometry
    they annotate. Clears any previous content first so the overlay tracks the
    current scene. Returns ``(num_lines, num_labels)``.
    """
    overlay.clear()
    for start, end, rgb in scene.segments:
        overlay.addLine(
            [float(v) for v in start],
            [float(v) for v in end],
            _color01(rgb) + [1.0],
        )
    label_rgba = _color01(label_color) + [1.0]
    for anchor, text in scene.labels:
        overlay.addLabel(
            [float(v) for v in anchor], str(text), label_rgba, float(character_size)
        )
    return len(scene.segments), len(scene.labels)
