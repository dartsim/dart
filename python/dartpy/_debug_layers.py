"""World-level debug layer composition for headless agent rendering.

The debug-line and label geometry lives in ``dart::gui`` (C++): this module is
a thin Python orchestration layer that selects which core layers to extract for
a DART 7 rigid-body ``World`` and assembles them into a
``dart.gui.DebugScene``. Per-body layers (frames, centers of mass, inertia
boxes, collision bounds, velocity arrows) come from
``gui.extract_world_debug_lines``; contacts from ``gui.extract_contact_debug_lines``;
trajectories from ``gui.make_polyline_debug_lines``; and label text is
composited with the core ``gui.composite_debug_labels`` / ``gui.draw_debug_text``
font, so the Python layer never re-implements projection, arrow geometry, or a
glyph table.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Sequence

import dartpy as dart
import numpy as np

# Label color used for DebugLabelDescriptor construction; the line colors now
# live in dart/gui/debug.cpp and reach Python through the core extractors.
_TRAJECTORY_RGBA = (0.98, 0.55, 0.25, 0.95)
_LABEL_RGBA = (1.0, 1.0, 1.0, 1.0)

DEBUG_LAYERS = (
    "grid",
    "world_frame",
    "body_frames",
    "coms",
    "inertia_boxes",
    "collision_bounds",
    "velocities",
    "contacts",
    "trajectories",
    "labels",
)


def _tracked_rigid_bodies(world: Any) -> list[Any]:
    from . import _scene_dump

    return _scene_dump._tracked_rigid_bodies(world)


def _body_transform(body: Any) -> np.ndarray:
    return np.asarray(body.transform, dtype=float).reshape(4, 4)


def _default_options() -> Any:
    return dart.gui.DebugDrawOptions()


def _layer_lines(
    world: Any,
    source_options: Any,
    flags: Sequence[str],
    tunables: Sequence[str] = (),
) -> list:
    """Extract one per-body debug layer through the core world extractor.

    A fresh ``DebugDrawOptions`` with the static grid/world-frame layers off
    isolates a single per-body layer so ``extract_world_debug_lines`` returns
    only that layer's lines; the requested tunables are copied from the
    caller's options so per-layer sizing still applies.
    """
    layer_options = dart.gui.DebugDrawOptions()
    layer_options.draw_grid = False
    layer_options.draw_world_frame = False
    for flag in flags:
        setattr(layer_options, flag, True)
    for name in tunables:
        setattr(layer_options, name, getattr(source_options, name))
    return list(dart.gui.extract_world_debug_lines(world, layer_options))


def contact_debug_lines(contacts: Iterable[Any], options: Any | None = None) -> list:
    """Contact markers/normals for a list of ``simulation.Contact`` objects.

    Delegates to the core ``extract_contact_debug_lines`` overload that accepts
    the promoted simulation contact type returned by ``world.collide()``.
    """
    if options is None:
        options = _default_options()
    return list(dart.gui.extract_contact_debug_lines(list(contacts), options))


def _trajectory_lines(name: str, positions: Iterable[Any]) -> list:
    points = [np.asarray(point, dtype=float).reshape(3) for point in positions]
    return list(
        dart.gui.make_polyline_debug_lines(
            points,
            np.asarray(_TRAJECTORY_RGBA, dtype=float),
            f"{name}.trajectory",
        )
    )


class TrajectoryTracker:
    """Records per-body world positions to draw motion-history polylines."""

    def __init__(
        self, world: Any, bodies: Sequence[str] | None = None, max_samples: int = 512
    ) -> None:
        self._world = world
        self._bodies = list(bodies) if bodies is not None else None
        self._max_samples = int(max_samples)
        self._history: dict[str, list[np.ndarray]] = {}

    def sample(self) -> None:
        for body in _tracked_rigid_bodies(self._world):
            name = str(body.name)
            if self._bodies is not None and name not in self._bodies:
                continue
            positions = self._history.setdefault(name, [])
            transform = _body_transform(body)
            positions.append(transform[:3, 3].copy())
            if len(positions) > self._max_samples:
                del positions[0 : len(positions) - self._max_samples]

    @property
    def history(self) -> dict[str, list[np.ndarray]]:
        return self._history

    def debug_lines(self) -> list:
        lines: list = []
        for name, positions in sorted(self._history.items()):
            lines.extend(_trajectory_lines(name, positions))
        return lines


def body_labels(world: Any, *, offset: Sequence[float] = (0.0, 0.0, 0.0)) -> list:
    labels: list = []
    shift = np.asarray(offset, dtype=float).reshape(3)
    for name in world.get_rigid_body_names():
        body = world.get_rigid_body(name)
        if body is None:
            continue
        label = dart.gui.DebugLabelDescriptor()
        transform = _body_transform(body)
        label.position = transform[:3, 3] + shift
        label.text = str(name)
        label.rgba = np.asarray(_LABEL_RGBA, dtype=float)
        labels.append(label)
    return labels


def debug_scene_for_world(
    world: Any,
    options: Any | None = None,
    *,
    layers: Sequence[str] = ("contacts", "body_frames"),
    contacts: Iterable[Any] | None = None,
    trajectories: TrajectoryTracker | dict[str, Sequence[Any]] | None = None,
    extra_lines: Iterable[Any] | None = None,
    extra_labels: Iterable[Any] | None = None,
) -> Any:
    """Compose a DebugScene from named layers of a DART 7 Python world.

    ``contacts`` defaults to ``world.collide()`` when the contacts layer is
    requested; pass precomputed contacts to keep captures deterministic
    relative to a specific step.
    """
    unknown = sorted(set(layers) - set(DEBUG_LAYERS))
    if unknown:
        raise ValueError(
            f"unknown debug layers {unknown}; available: {list(DEBUG_LAYERS)}"
        )
    if options is None:
        options = _default_options()

    scene = dart.gui.DebugScene()
    lines: list = []
    labels: list = []

    if "grid" in layers or "world_frame" in layers:
        static_options = dart.gui.DebugDrawOptions()
        for attribute in (
            "grid_half_extent",
            "grid_spacing",
            "grid_z",
            "world_frame_axis_length",
        ):
            setattr(static_options, attribute, getattr(options, attribute))
        static_options.draw_grid = "grid" in layers
        static_options.draw_world_frame = "world_frame" in layers
        lines.extend(dart.gui.extract_debug_lines(static_options))

    if "body_frames" in layers:
        lines.extend(
            _layer_lines(
                world, options, ["draw_body_frames"], ["body_frame_axis_length"]
            )
        )

    if "coms" in layers:
        lines.extend(
            _layer_lines(
                world,
                options,
                ["draw_centers_of_mass"],
                ["center_of_mass_marker_radius"],
            )
        )

    if "inertia_boxes" in layers:
        lines.extend(
            _layer_lines(
                world, options, ["draw_inertia_boxes"], ["inertia_box_scale"]
            )
        )

    if "collision_bounds" in layers:
        lines.extend(
            _layer_lines(
                world,
                options,
                ["draw_collision_shape_bounds"],
                ["collision_bounds_padding"],
            )
        )

    if "velocities" in layers:
        lines.extend(
            _layer_lines(
                world,
                options,
                ["draw_linear_velocities", "draw_angular_velocities"],
                [
                    "linear_velocity_scale",
                    "angular_velocity_scale",
                    "velocity_min_length",
                    "velocity_max_length",
                ],
            )
        )

    if "contacts" in layers:
        contact_list = list(contacts) if contacts is not None else world.collide()
        lines.extend(contact_debug_lines(contact_list, options))

    if "trajectories" in layers:
        if trajectories is None:
            raise ValueError(
                "the 'trajectories' layer needs recorded motion history: pass "
                "trajectories=dart.gui.TrajectoryTracker(world) (call sample() "
                "each step) or a {body_name: positions} mapping"
            )
        if isinstance(trajectories, TrajectoryTracker):
            lines.extend(trajectories.debug_lines())
        else:
            for name, positions in sorted(trajectories.items()):
                lines.extend(_trajectory_lines(name, positions))

    if "labels" in layers:
        labels.extend(body_labels(world))

    if extra_lines is not None:
        lines.extend(extra_lines)
    if extra_labels is not None:
        labels.extend(extra_labels)

    scene.lines = lines
    scene.labels = labels
    return scene


# --- Screen-space projection and label compositing -------------------------


def project_points(
    camera: Any,
    size: tuple[int, int],
    points: Sequence[Any],
    projection_options: Any | None = None,
) -> np.ndarray:
    """Project world points to pixel coordinates for an orbit camera.

    Returns an (N, 3) array of (x_pixels, y_pixels, view_depth) by calling the
    core ``project_to_pixels`` per point; points behind the camera get a
    negative depth. Pixel origin is the top-left corner.
    """
    width, height = int(size[0]), int(size[1])
    if projection_options is None:
        projection_options = dart.gui.ProjectionOptions()
    array = [np.asarray(point, dtype=float).reshape(3) for point in points]
    result = np.empty((len(array), 3), dtype=float)
    for index, point in enumerate(array):
        result[index] = np.asarray(
            dart.gui.project_to_pixels(
                camera, width, height, point, projection_options
            ),
            dtype=float,
        ).reshape(3)
    return result


def draw_text(
    pixels: np.ndarray,
    text: str,
    origin: tuple[int, int],
    rgba: Sequence[float] = _LABEL_RGBA,
    scale: int = 2,
) -> None:
    """Draw uppercase bitmap text into an (H, W, 4) uint8 RGBA array in place.

    Delegates to the core ``draw_debug_text`` glyph rasterizer so the font
    table has a single owner in dart/gui/offscreen.cpp.
    """
    dart.gui.draw_debug_text(
        pixels,
        str(text),
        int(origin[0]),
        int(origin[1]),
        np.asarray(rgba, dtype=float).reshape(4),
        int(scale),
    )


def composite_labels(
    image: Any,
    camera: Any,
    labels: Iterable[Any],
    *,
    scale: int = 2,
    projection_options: Any | None = None,
) -> np.ndarray:
    """Composite DebugLabelDescriptors onto a rendered image.

    Accepts a RenderedImage or an (H, W, 4) uint8 array; returns the annotated
    array (a copy when given a RenderedImage). A RenderedImage is annotated by
    the core ``composite_debug_labels``; a raw array is projected here and
    rasterized through the core ``draw_debug_text`` font.
    """
    label_list = list(labels)
    if hasattr(image, "png_bytes"):
        if projection_options is None:
            projection_options = dart.gui.ProjectionOptions()
        dart.gui.composite_debug_labels(
            image, camera, label_list, scale, projection_options
        )
        return np.array(memoryview(image), copy=True)

    pixels = np.asarray(image)
    if not label_list:
        return pixels
    height, width = pixels.shape[:2]
    projected = project_points(
        camera,
        (width, height),
        [label.position for label in label_list],
        projection_options,
    )
    for label, (x, y, depth) in zip(label_list, projected):
        if depth <= 0.0:
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        draw_text(
            pixels,
            str(label.text),
            (int(round(x)) + 2, int(round(y)) - 3 * scale),
            np.asarray(label.rgba, dtype=float).reshape(4),
            scale=scale,
        )
    return pixels


def install_debug_layer_helpers(root: Any, gui: Any) -> None:
    gui.DEBUG_LAYERS = DEBUG_LAYERS
    gui.debug_scene_for_world = debug_scene_for_world
    gui.contact_debug_lines = contact_debug_lines
    gui.TrajectoryTracker = TrajectoryTracker
    gui.body_labels = body_labels
    gui.project_points = project_points
    gui.composite_labels = composite_labels
    gui.draw_text = draw_text
