"""Engine-rendered debug overlays for DART 6 headless captures.

This module builds world-derived debug layers — contact markers/normals/forces,
body frames, velocity arrows, trajectory polylines, and name labels — and
renders them *through the DART core OSG pipeline* rather than compositing them
onto the captured PNG in image space. Segments become
``dart.dynamics.LineSegmentShape`` geometry on ``SimpleFrame``s added to the
world (so ``WorldNode`` draws them as real, depth-correct scene geometry), and
labels become world-anchored text on a ``dart.gui.osg.TextOverlay`` viewer
attachment (osgText). The capture harness injects the overlay, renders through
``captureOffscreen``, then removes it. Colors and arrow geometry mirror
dart::gui's debug producers so DART 6 and DART 7 speak the same visual language.

``build_overlay`` and ``OverlayScene`` stay pure Python + numpy over the classic
dartpy surface; only the rendering backend (``inject_overlay`` /
``remove_overlay`` / ``populate_labels``) touches the OSG scene.
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
# osgText labels render on the Viewer's light-gray (0.9) background, so a dark
# slate reads far more clearly than the white DART 7 uses on its dark overlay.
LABEL_RGB = (33, 33, 40)

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


# --- Engine rendering backend -----------------------------------------------

# World SimpleFrames created for the overlay carry this name prefix so they are
# easy to identify; inject_overlay/remove_overlay manage their lifetime.
OVERLAY_FRAME_PREFIX = "agent_overlay"
DEFAULT_LINE_THICKNESS = 2.0
# Label size is in screen pixels (TextOverlay uses SCREEN_COORDS), so labels
# stay legible at any camera distance.
DEFAULT_LABEL_CHARACTER_SIZE = 20.0

# Font files tried, in order, when none is set explicitly. osgText renders
# nothing without a usable font, so the harness resolves one from the runtime
# environment (the pixi/conda env ships DejaVuSans).
_FONT_CANDIDATES = ("DejaVuSans.ttf", "Arial.ttf", "arial.ttf", "Vera.ttf")


def _dartpy() -> Any:
    import dartpy

    return dartpy


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


def inject_overlay(
    world: Any,
    scene: OverlayScene,
    *,
    thickness: float = DEFAULT_LINE_THICKNESS,
) -> list[Any]:
    """Render ``scene``'s segments through the engine as world geometry.

    Segments are grouped by color into one ``LineSegmentShape`` each, wrapped in
    a world ``SimpleFrame`` whose visual aspect carries that color. ``WorldNode``
    then draws them as real, depth-correct scene geometry in every capture.
    Returns the frames added; pass them to :func:`remove_overlay`.
    """
    dart = _dartpy()
    by_color: dict[tuple[int, int, int], list[tuple[np.ndarray, np.ndarray]]] = {}
    for start, end, rgb in scene.segments:
        by_color.setdefault(rgb, []).append((start, end))

    frames: list[Any] = []
    for index, rgb in enumerate(sorted(by_color)):
        line = dart.dynamics.LineSegmentShape(float(thickness))
        for start, end in by_color[rgb]:
            first = line.addVertex([float(v) for v in start])
            second = line.addVertex([float(v) for v in end])
            line.addConnection(first, second)
        frame = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), f"{OVERLAY_FRAME_PREFIX}_{index}"
        )
        frame.setShape(line)
        frame.createVisualAspect().setColor(_color01(rgb))
        world.addSimpleFrame(frame)
        frames.append(frame)
    return frames


def remove_overlay(world: Any, frames: Sequence[Any]) -> None:
    """Remove overlay frames previously added by :func:`inject_overlay`."""
    for frame in frames:
        world.removeSimpleFrame(frame)


def populate_labels(
    overlay: Any,
    scene: OverlayScene,
    *,
    color: tuple[int, int, int] = LABEL_RGB,
    character_size: float = DEFAULT_LABEL_CHARACTER_SIZE,
) -> int:
    """Load ``scene``'s labels into a ``dart.gui.osg.TextOverlay`` attachment.

    Clears any previous labels first so the overlay tracks the current scene.
    Each label is world-anchored, screen-facing text rendered by osgText.
    Returns the number of labels added.
    """
    overlay.clear()
    rgba = _color01(color) + [1.0]
    for anchor, text in scene.labels:
        overlay.addLabel(
            [float(v) for v in anchor], str(text), rgba, float(character_size)
        )
    return len(scene.labels)
