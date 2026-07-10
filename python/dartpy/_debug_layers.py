"""World-level debug layer composition for headless agent rendering.

The C++ viewer exposes debug-line producers (`dart::gui::make*DebugLines`,
`extractContactDebugLines`) but nothing walks a DART 7 Python ``World`` and
aggregates them, and several producers only accept skeleton ``BodyNode``
types that the Python rigid-body ``World`` does not expose. This module
composes a ``dart.gui.DebugScene`` per layer directly from rigid-body state,
mirroring the C++ colors and arrow geometry so headless captures match the
interactive viewer's visual language. Labels are composited onto rendered
RGBA buffers here because the offscreen path has no ImGui text pass.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Sequence

import dartpy as dart
import numpy as np

# Colors mirrored from dart/gui/debug.cpp so overlays match the viewer.
_AXIS_X_RGBA = (0.9, 0.28, 0.28, 1.0)
_AXIS_Y_RGBA = (0.31, 0.75, 0.43, 1.0)
_AXIS_Z_RGBA = (0.28, 0.47, 0.92, 1.0)
_COM_RGBA = (0.22, 0.82, 0.86, 1.0)
_CONTACT_POINT_RGBA = (1.0, 0.92, 0.38, 1.0)
_CONTACT_NORMAL_RGBA = (1.0, 0.75, 0.25, 1.0)
_LINEAR_VELOCITY_RGBA = (0.32, 0.74, 0.98, 0.95)
_ANGULAR_VELOCITY_RGBA = (0.74, 0.52, 0.98, 0.95)
_INERTIA_RGBA = (0.95, 0.77, 0.06, 0.9)
_BOUNDS_RGBA = (0.55, 0.85, 0.55, 0.85)
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

_MIN_LINE_LENGTH_SQUARED = 1e-18


def _line(from_point: Any, to_point: Any, rgba: Sequence[float], label: str = ""):
    start = np.asarray(from_point, dtype=float).reshape(3)
    end = np.asarray(to_point, dtype=float).reshape(3)
    if not (np.isfinite(start).all() and np.isfinite(end).all()):
        return None
    if float(np.dot(end - start, end - start)) <= _MIN_LINE_LENGTH_SQUARED:
        return None
    line = dart.gui.DebugLineDescriptor()
    line.from_point = start
    line.to_point = end
    line.rgba = np.asarray(rgba, dtype=float).reshape(4)
    line.label = label
    return line


def _append_line(lines: list, from_point, to_point, rgba, label: str = "") -> None:
    line = _line(from_point, to_point, rgba, label)
    if line is not None:
        lines.append(line)


def _append_arrow(lines: list, from_point, to_point, rgba, label: str = "") -> None:
    """Mirror of appendArrowLines in dart/gui/debug.cpp (shaft + two barbs)."""
    _append_line(lines, from_point, to_point, rgba, label)
    start = np.asarray(from_point, dtype=float).reshape(3)
    end = np.asarray(to_point, dtype=float).reshape(3)
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
    _append_line(lines, end, base + side * head_width, rgba, label)
    _append_line(lines, end, base - side * head_width, rgba, label)


def _append_axis_marker(
    lines: list, center, radius: float, rgba, label: str = ""
) -> None:
    center = np.asarray(center, dtype=float).reshape(3)
    for axis in np.eye(3):
        _append_line(lines, center - axis * radius, center + axis * radius, rgba, label)


def _append_box_edges(lines: list, corners: np.ndarray, rgba, label: str = "") -> None:
    edges = (
        (0, 1), (1, 3), (3, 2), (2, 0),
        (4, 5), (5, 7), (7, 6), (6, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    for a, b in edges:
        _append_line(lines, corners[a], corners[b], rgba, label)


def _box_corners(center: np.ndarray, axes: np.ndarray, half_extents: np.ndarray):
    corners = np.empty((8, 3), dtype=float)
    index = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                offset = (
                    axes[:, 0] * (sx * half_extents[0])
                    + axes[:, 1] * (sy * half_extents[1])
                    + axes[:, 2] * (sz * half_extents[2])
                )
                corners[index] = center + offset
                index += 1
    return corners


def _tracked_rigid_bodies(world: Any) -> list[Any]:
    from . import _scene_dump

    return _scene_dump._tracked_rigid_bodies(world)


def _body_transform(body: Any) -> np.ndarray:
    return np.asarray(body.transform, dtype=float).reshape(4, 4)


def _default_options() -> Any:
    return dart.gui.DebugDrawOptions()


def _frame_lines_for_body(body: Any, axis_length: float) -> list:
    transform = _body_transform(body)
    lines: list = []
    origin = transform[:3, 3]
    rotation = transform[:3, :3]
    axis_colors = (_AXIS_X_RGBA, _AXIS_Y_RGBA, _AXIS_Z_RGBA)
    axis_names = ("x", "y", "z")
    for column, (color, axis_name) in enumerate(zip(axis_colors, axis_names)):
        axis = rotation[:, column]
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-12:
            continue
        _append_line(
            lines,
            origin,
            origin + axis / norm * axis_length,
            color,
            f"{body.name}.{axis_name}",
        )
    return lines


def _inertia_box_lines(body: Any, options: Any) -> list:
    lines: list = []
    mass = float(getattr(body, "mass", 0.0))
    if mass <= 0.0:
        return lines
    inertia = np.asarray(body.inertia, dtype=float).reshape(3, 3)
    if not np.isfinite(inertia).all():
        return lines
    moments, axes = np.linalg.eigh(0.5 * (inertia + inertia.T))
    if (moments <= 0.0).any():
        return lines
    # Solid-box equivalence: I_x = m/12 (b^2 + c^2) etc.
    sums = 6.0 / mass * (moments.sum() - 2.0 * moments)
    if (sums <= 0.0).any():
        return lines
    half_extents = np.sqrt(sums) * 0.5 * float(options.inertia_box_scale)
    transform = _body_transform(body)
    world_axes = transform[:3, :3] @ axes
    corners = _box_corners(transform[:3, 3], world_axes, half_extents)
    _append_box_edges(lines, corners, _INERTIA_RGBA, f"{body.name}.inertia")
    return lines


def _collision_bounds_lines(descriptors: Iterable[Any], padding: float) -> list:
    lines: list = []
    for descriptor in descriptors:
        geometry = getattr(descriptor, "geometry", None)
        if geometry is None or not bool(getattr(geometry, "has_local_bounds", False)):
            continue
        bounds_min = np.asarray(geometry.local_bounds_min, dtype=float).reshape(3)
        bounds_max = np.asarray(geometry.local_bounds_max, dtype=float).reshape(3)
        from . import _world_render_bridge

        center_local = 0.5 * (bounds_min + bounds_max)
        half_extents = 0.5 * (bounds_max - bounds_min) + padding
        transform = np.asarray(
            _world_render_bridge._isometry_to_matrix(descriptor.world_transform),
            dtype=float,
        ).reshape(4, 4)
        center = transform[:3, :3] @ center_local + transform[:3, 3]
        corners = _box_corners(center, transform[:3, :3], half_extents)
        label = str(getattr(descriptor, "shape_frame_name", "") or "bounds")
        _append_box_edges(lines, corners, _BOUNDS_RGBA, f"{label}.bounds")
    return lines


def _velocity_lines(body: Any, options: Any) -> list:
    lines: list = []

    def scaled(magnitude: float, scale: float) -> float:
        return float(
            np.clip(
                magnitude * scale,
                options.velocity_min_length,
                options.velocity_max_length,
            )
        )

    transform = _body_transform(body)
    origin = transform[:3, 3]
    linear = np.asarray(body.linear_velocity, dtype=float).reshape(3)
    magnitude = float(np.linalg.norm(linear))
    if np.isfinite(linear).all() and magnitude > 1e-9:
        _append_arrow(
            lines,
            origin,
            origin
            + linear / magnitude * scaled(magnitude, options.linear_velocity_scale),
            _LINEAR_VELOCITY_RGBA,
            f"{body.name}.vel_linear",
        )
    angular = np.asarray(body.angular_velocity, dtype=float).reshape(3)
    magnitude = float(np.linalg.norm(angular))
    if np.isfinite(angular).all() and magnitude > 1e-9:
        _append_arrow(
            lines,
            origin,
            origin
            + angular / magnitude * scaled(magnitude, options.angular_velocity_scale),
            _ANGULAR_VELOCITY_RGBA,
            f"{body.name}.vel_angular",
        )
    return lines


def contact_debug_lines(contacts: Iterable[Any], options: Any | None = None) -> list:
    """Python mirror of extractContactDebugLines for simulation.Contact lists.

    The C++ extractor requires a collision.CollisionResult, which the DART 7
    Python World does not expose; ``world.collide()`` returns
    ``simulation.Contact`` objects instead.
    """
    if options is None:
        options = _default_options()
    lines: list = []
    if not options.draw_contacts:
        return lines
    half_extent = float(options.contact_marker_half_extent)
    for contact in contacts:
        point = np.asarray(contact.point, dtype=float).reshape(3)
        for axis in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])):
            _append_line(
                lines,
                point - axis * half_extent,
                point + axis * half_extent,
                _CONTACT_POINT_RGBA,
                "contact.point",
            )
        if options.draw_contact_normals:
            normal = np.asarray(contact.normal, dtype=float).reshape(3)
            norm = float(np.linalg.norm(normal))
            if norm > 1e-9:
                _append_arrow(
                    lines,
                    point,
                    point + normal / norm * float(options.contact_normal_length),
                    _CONTACT_NORMAL_RGBA,
                    "contact.normal",
                )
    return lines


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
            for start, end in zip(positions, positions[1:]):
                _append_line(
                    lines, start, end, _TRAJECTORY_RGBA, f"{name}.trajectory"
                )
        return lines


def body_labels(world: Any, *, offset: Sequence[float] = (0.0, 0.0, 0.0)) -> list:
    labels: list = []
    shift = np.asarray(offset, dtype=float).reshape(3)
    for body in _tracked_rigid_bodies(world):
        label = dart.gui.DebugLabelDescriptor()
        transform = _body_transform(body)
        label.position = transform[:3, 3] + shift
        label.text = str(body.name)
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
    bodies = _tracked_rigid_bodies(world)

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
        for body in bodies:
            lines.extend(
                _frame_lines_for_body(body, float(options.body_frame_axis_length))
            )

    if "coms" in layers:
        for body in bodies:
            transform = _body_transform(body)
            _append_axis_marker(
                lines,
                transform[:3, 3],
                float(options.center_of_mass_marker_radius),
                _COM_RGBA,
                f"{body.name}.com",
            )

    if "inertia_boxes" in layers:
        for body in bodies:
            lines.extend(_inertia_box_lines(body, options))

    if "collision_bounds" in layers:
        from . import _world_render_bridge

        descriptors = _world_render_bridge._renderables_from_world(world)
        lines.extend(
            _collision_bounds_lines(
                descriptors, float(options.collision_bounds_padding)
            )
        )

    if "velocities" in layers:
        for body in bodies:
            lines.extend(_velocity_lines(body, options))

    if "contacts" in layers:
        contact_list = list(contacts) if contacts is not None else world.collide()
        lines.extend(contact_debug_lines(contact_list, options))

    if "trajectories" in layers and trajectories is not None:
        if isinstance(trajectories, TrajectoryTracker):
            lines.extend(trajectories.debug_lines())
        else:
            for name, positions in sorted(trajectories.items()):
                points = [np.asarray(p, dtype=float).reshape(3) for p in positions]
                for start, end in zip(points, points[1:]):
                    _append_line(
                        lines, start, end, _TRAJECTORY_RGBA, f"{name}.trajectory"
                    )

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

# Compact 3x5 uppercase bitmap font (columns-of-rows bitmask per glyph).
_FONT_3X5 = {
    "A": ("010", "101", "111", "101", "101"),
    "B": ("110", "101", "110", "101", "110"),
    "C": ("011", "100", "100", "100", "011"),
    "D": ("110", "101", "101", "101", "110"),
    "E": ("111", "100", "110", "100", "111"),
    "F": ("111", "100", "110", "100", "100"),
    "G": ("011", "100", "101", "101", "011"),
    "H": ("101", "101", "111", "101", "101"),
    "I": ("111", "010", "010", "010", "111"),
    "J": ("001", "001", "001", "101", "010"),
    "K": ("101", "110", "100", "110", "101"),
    "L": ("100", "100", "100", "100", "111"),
    "M": ("101", "111", "111", "101", "101"),
    "N": ("101", "111", "111", "111", "101"),
    "O": ("010", "101", "101", "101", "010"),
    "P": ("110", "101", "110", "100", "100"),
    "Q": ("010", "101", "101", "011", "001"),
    "R": ("110", "101", "110", "110", "101"),
    "S": ("011", "100", "010", "001", "110"),
    "T": ("111", "010", "010", "010", "010"),
    "U": ("101", "101", "101", "101", "111"),
    "V": ("101", "101", "101", "010", "010"),
    "W": ("101", "101", "111", "111", "101"),
    "X": ("101", "010", "010", "010", "101"),
    "Y": ("101", "101", "010", "010", "010"),
    "Z": ("111", "001", "010", "100", "111"),
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "011", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "001", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "-": ("000", "000", "111", "000", "000"),
    "_": ("000", "000", "000", "000", "111"),
    ".": ("000", "000", "000", "000", "010"),
    ":": ("000", "010", "000", "010", "000"),
    "/": ("001", "001", "010", "100", "100"),
    "+": ("000", "010", "111", "010", "000"),
    "(": ("010", "100", "100", "100", "010"),
    ")": ("010", "001", "001", "001", "010"),
    " ": ("000", "000", "000", "000", "000"),
}


def project_points(
    camera: Any,
    size: tuple[int, int],
    points: Sequence[Any],
    projection_options: Any | None = None,
) -> np.ndarray:
    """Project world points to pixel coordinates for an orbit camera.

    Returns an (N, 3) array of (x_pixels, y_pixels, view_depth); points behind
    the camera get a negative depth. Pixel origin is the top-left corner.
    """
    width, height = int(size[0]), int(size[1])
    if projection_options is None:
        projection_options = dart.gui.ProjectionOptions()
    projection = dart.gui.make_perspective_projection(
        camera, width, height, projection_options
    )
    basis = dart.gui.make_orbit_camera_basis(camera)
    eye = np.asarray(basis.eye, dtype=float).reshape(3)
    forward = np.asarray(basis.forward, dtype=float).reshape(3)
    right = np.asarray(basis.right, dtype=float).reshape(3)
    up = np.asarray(basis.up, dtype=float).reshape(3)

    fov = math.radians(float(projection.vertical_fov_degrees))
    focal = 1.0 / math.tan(0.5 * fov)
    aspect = float(projection.aspect_ratio)

    array = np.asarray(
        [np.asarray(p, dtype=float).reshape(3) for p in points], dtype=float
    ).reshape(-1, 3)
    offsets = array - eye
    depth = offsets @ forward
    safe_depth = np.where(np.abs(depth) < 1e-12, 1e-12, depth)
    ndc_x = (offsets @ right) / safe_depth * (focal / aspect)
    ndc_y = (offsets @ up) / safe_depth * focal
    pixel_x = (ndc_x * 0.5 + 0.5) * width
    pixel_y = (1.0 - (ndc_y * 0.5 + 0.5)) * height
    return np.column_stack([pixel_x, pixel_y, depth])


def draw_text(
    pixels: np.ndarray,
    text: str,
    origin: tuple[int, int],
    rgba: Sequence[float] = _LABEL_RGBA,
    scale: int = 2,
) -> None:
    """Draw uppercase bitmap text into an (H, W, 4) uint8 RGBA array."""
    height, width = pixels.shape[:2]
    color = (np.clip(np.asarray(rgba, dtype=float), 0.0, 1.0) * 255).astype(np.uint8)
    x_cursor = int(origin[0])
    y_origin = int(origin[1])
    for character in str(text).upper():
        glyph = _FONT_3X5.get(character, _FONT_3X5[" "])
        for row_index, row in enumerate(glyph):
            for column_index, bit in enumerate(row):
                if bit != "1":
                    continue
                y_start = y_origin + row_index * scale
                x_start = x_cursor + column_index * scale
                if y_start >= height or x_start >= width:
                    continue
                y_end = min(y_start + scale, height)
                x_end = min(x_start + scale, width)
                if y_start < 0 or x_start < 0:
                    continue
                pixels[y_start:y_end, x_start:x_end, :3] = color[:3]
                pixels[y_start:y_end, x_start:x_end, 3] = 255
        x_cursor += (3 + 1) * scale


def composite_labels(
    image: Any,
    camera: Any,
    labels: Iterable[Any],
    *,
    scale: int = 2,
    projection_options: Any | None = None,
) -> np.ndarray:
    """Composite DebugLabelDescriptors onto a rendered image.

    Accepts a RenderedImage or an (H, W, 4) uint8 array; returns the
    annotated array (a copy when given a RenderedImage).
    """
    if hasattr(image, "png_bytes"):
        pixels = np.array(memoryview(image), copy=True)
    else:
        pixels = np.asarray(image)
    height, width = pixels.shape[:2]
    label_list = list(labels)
    if not label_list:
        return pixels
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
