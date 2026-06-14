"""Descriptor-backed render bridge for DART 7 Python demo scenes."""

from __future__ import annotations

from collections import deque
from typing import Any

import dartpy as dart
import numpy as np


class _FallbackGeometryDescriptor:
    def __init__(self, shape: Any) -> None:
        shape_type = type(shape).__name__
        self.kind = shape_type.removesuffix("Shape") or "Unsupported"
        self.shape_type = shape_type
        self.unsupported_reason = ""
        self.size = _shape_vector(shape, "size")
        self.radius = _shape_scalar(shape, "radius")
        self.height = _shape_scalar(shape, "height")


class _FallbackMaterialDescriptor:
    def __init__(self) -> None:
        self.rgba = np.ones(4)
        self.visible = True
        self.casts_shadows = True
        self.receives_shadows = True
        self.metallic: float | None = None
        self.roughness: float | None = None
        self.reflectance: float | None = None


class _FallbackRenderableDescriptor:
    def __init__(self) -> None:
        self.id = 0
        self.skeleton_name = ""
        self.body_name = ""
        self.shape_frame_name = ""
        self.shape_node_name = ""
        self.geometry = None
        self.material = _FallbackMaterialDescriptor()
        self.world_transform = np.eye(4)
        self.shape_frame_version = 0
        self.shape_node_version = 0
        self.shape_version = 0
        self.render_resource_version = 0


def _shape_member(shape: Any, name: str) -> Any | None:
    snake_getter = getattr(shape, f"get_{name}", None)
    if snake_getter is not None:
        return snake_getter()
    camel_getter = getattr(shape, f"get{name.title().replace('_', '')}", None)
    if camel_getter is not None:
        return camel_getter()
    return getattr(shape, name, None)


def _shape_vector(shape: Any, name: str) -> np.ndarray:
    value = _shape_member(shape, name)
    if value is None:
        return np.zeros(3)
    try:
        return np.asarray(value, dtype=float).reshape(-1)
    except Exception:  # noqa: BLE001
        return np.zeros(3)


def _shape_scalar(shape: Any, name: str) -> float:
    value = _shape_member(shape, name)
    if value is None:
        return 0.0
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        return 0.0


def _visual_pbr_value(visual: Any, name: str) -> float | None:
    value = _shape_member(visual, name)
    if value is None:
        return None
    try:
        scalar = float(value)
    except Exception:  # noqa: BLE001
        return None
    if not np.isfinite(scalar) or scalar < 0.0:
        return None
    return scalar


def _copy_visual_pbr(material: Any, visual: Any) -> None:
    for name in ("metallic", "roughness", "reflectance"):
        if not hasattr(material, name):
            continue
        try:
            setattr(material, name, _visual_pbr_value(visual, name))
        except Exception:  # noqa: BLE001
            pass


def world_render_frame() -> Any:
    """Return the dynamics world frame used as a parent for render helpers."""

    from dartpy._dartpy import dynamics as _dyn

    return _dyn.Frame.world()


def _isometry_to_matrix(transform: Any) -> np.ndarray:
    if hasattr(transform, "matrix"):
        return np.asarray(transform.matrix())
    return np.asarray(transform)


def _translation(position: Any) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return transform


def _is_fixed_node(body: Any, index: int) -> bool:
    try:
        return bool(body.is_fixed_node(index))
    except Exception:  # noqa: BLE001
        return False


def _deformable_wireframe_edges(body: Any) -> list[tuple[int, int]]:
    edges: set[tuple[int, int]] = set()

    def add(u: int, v: int) -> None:
        edges.add((u, v) if u < v else (v, u))

    triangle_count = int(getattr(body, "surface_triangle_count", 0))
    if triangle_count > 0:
        for t in range(triangle_count):
            tri = body.surface_triangle(t)
            a, b, c = int(tri.node_a), int(tri.node_b), int(tri.node_c)
            add(a, b)
            add(b, c)
            add(c, a)
        return sorted(edges)

    for e in range(int(getattr(body, "edge_count", 0))):
        edge = body.edge(e)
        add(int(edge.node_a), int(edge.node_b))
    return sorted(edges)


class DescriptorRenderScene:
    """Small frame container that supplies GUI renderable descriptors."""

    def __init__(self, physics_world: Any, name: str) -> None:
        self._physics_world = physics_world
        self.name = name
        self.time_step = float(getattr(physics_world, "time_step", 0.001))
        self._frames: list[Any] = []
        self._render_version = 0

    def set_gravity(self, _gravity: Any) -> None:
        return

    def set_time_step(self, time_step: float) -> None:
        self.time_step = float(time_step)
        if hasattr(self._physics_world, "set_time_step"):
            try:
                self._physics_world.set_time_step(self.time_step)
                return
            except Exception:  # noqa: BLE001
                pass
        try:
            self._physics_world.time_step = self.time_step
        except Exception:  # noqa: BLE001
            pass

    def add_simple_frame(self, frame: Any) -> str:
        if frame not in self._frames:
            self._frames.append(frame)
        try:
            return str(frame.get_name())
        except Exception:  # noqa: BLE001
            try:
                return str(frame.getName())
            except Exception:  # noqa: BLE001
                return f"frame_{len(self._frames) - 1}"

    def step(self) -> None:
        return

    def renderable_provider(self) -> list[Any]:
        renderables: list[Any] = []
        self._render_version += 1
        describe_shape = getattr(dart.gui, "describe_shape", None)
        renderable_type = getattr(
            dart.gui, "RenderableDescriptor", _FallbackRenderableDescriptor
        )
        material_type = getattr(dart.gui, "MaterialDescriptor", _FallbackMaterialDescriptor)
        use_fallback_geometry = describe_shape is None
        for frame in self._frames:
            try:
                shape = frame.get_shape()
                visual = frame.get_visual_aspect(False)
            except Exception:  # noqa: BLE001
                continue
            if shape is None or visual is None:
                continue
            if use_fallback_geometry:
                geometry = _FallbackGeometryDescriptor(shape)
            else:
                try:
                    geometry = describe_shape(shape)
                except Exception:  # noqa: BLE001
                    continue
            if geometry is None:
                continue
            descriptor = renderable_type()
            descriptor.id = id(frame)
            try:
                descriptor.shape_frame_name = str(frame.get_name())
            except Exception:  # noqa: BLE001
                descriptor.shape_frame_name = f"frame_{len(renderables)}"
            descriptor.shape_node_name = descriptor.shape_frame_name
            descriptor.geometry = geometry
            material = material_type()
            try:
                material.rgba = np.asarray(visual.get_rgba(), dtype=float)
            except Exception:  # noqa: BLE001
                pass
            try:
                material.visible = not bool(visual.get_hidden())
            except Exception:  # noqa: BLE001
                pass
            try:
                shadowed = bool(visual.get_shadowed())
                material.casts_shadows = shadowed
                material.receives_shadows = shadowed
            except Exception:  # noqa: BLE001
                pass
            _copy_visual_pbr(material, visual)
            descriptor.material = material
            try:
                descriptor.world_transform = _isometry_to_matrix(frame.get_transform())
            except Exception:  # noqa: BLE001
                pass
            descriptor.shape_frame_version = self._render_version
            descriptor.shape_node_version = self._render_version
            descriptor.shape_version = self._render_version
            descriptor.render_resource_version = self._render_version
            renderables.append(descriptor)
        return renderables


class WorldRenderBridge:
    """Maps DART 7 physics objects to descriptor-backed GUI renderables."""

    _DRAG_DAMPING_KD: float = 4.0

    def __init__(self, physics_world: Any, name: str = "world_render") -> None:
        self._physics_world = physics_world
        self.render_world = DescriptorRenderScene(physics_world, name)
        self._mappings: list[tuple[Any, Any, np.ndarray]] = []
        self._surfaces: list[tuple[Any, Any, int]] = []
        self._pins: list[tuple[Any, list[tuple[int, Any]]]] = []
        self._by_name: dict[str, Any] = {}
        self._by_renderable_id: dict[int, Any] = {}
        self._drag: tuple[Any, np.ndarray, np.ndarray] | None = None
        self.force_drag_enabled = True
        self.force_drag_scale = 1.0
        self._last_drag_target = "none"
        self._last_drag_status = "idle"
        self._last_drag_magnitude = 0.0
        self._force_history: deque[float] = deque(maxlen=120)

    def _register_frame(self, frame: Any, physics_object: Any) -> str:
        actual_name = self.render_world.add_simple_frame(frame)
        self._by_name[actual_name] = physics_object
        self._refresh_renderable_ids()
        return actual_name

    def _refresh_renderable_ids(self) -> None:
        for renderable in self.renderable_provider():
            physics_object = self._by_name.get(renderable.shape_frame_name)
            if physics_object is not None:
                self._by_renderable_id[int(renderable.id)] = physics_object

    def add_link_visual(
        self,
        physics_link: Any,
        shape: Any,
        color: tuple[float, float, float],
        name: str | None = None,
        local_transform: Any | None = None,
    ) -> Any:
        frame_name = name or f"world_link_{len(self._mappings)}"
        frame = dart.SimpleFrame(world_render_frame(), frame_name, np.eye(4))
        frame.set_shape(shape)
        frame.create_visual_aspect().set_color(list(color))
        actual_name = self._register_frame(frame, physics_link)
        local = (
            np.eye(4)
            if local_transform is None
            else _isometry_to_matrix(local_transform)
        )
        self._mappings.append((physics_link, frame, local))
        self._by_name[actual_name] = physics_link
        return frame

    def add_rigid_body_visual(
        self,
        physics_body: Any,
        shape: Any,
        color: tuple[float, float, float],
        name: str | None = None,
        local_transform: Any | None = None,
    ) -> Any:
        return self.add_link_visual(
            physics_body,
            shape,
            color,
            name=name,
            local_transform=local_transform,
        )

    def add_deformable_visual(
        self,
        deformable_body: Any,
        color: tuple[float, float, float] = (0.25, 0.62, 0.85),
        radius: float = 0.018,
        fixed_color: tuple[float, float, float] | None = None,
        thickness: float = 2.5,
        name: str | None = None,
        edges: list[tuple[int, int]] | None = None,
    ) -> Any:
        body = deformable_body
        node_count = int(body.node_count)

        line = dart.LineSegmentShape(float(thickness))
        for i in range(node_count):
            line.add_vertex(np.asarray(body.node_position(i), dtype=float))
        for node_a, node_b in (
            edges if edges is not None else _deformable_wireframe_edges(body)
        ):
            line.add_connection(int(node_a), int(node_b))

        frame_name = name or f"world_surface_{len(self._surfaces)}"
        frame = dart.SimpleFrame(world_render_frame(), frame_name, np.eye(4))
        frame.set_shape(line)
        frame.create_visual_aspect().set_color(list(color))
        self._register_frame(frame, body)
        self._surfaces.append((body, line, node_count))

        pin_color = fixed_color if fixed_color is not None else color
        pin_frames: list[tuple[int, Any]] = []
        group = len(self._pins)
        for i in range(node_count):
            if not _is_fixed_node(body, i):
                continue
            sphere = dart.SimpleFrame(
                world_render_frame(),
                f"world_pin_{group}_{i}",
                _translation(body.node_position(i)),
            )
            sphere.set_shape(dart.SphereShape(radius))
            sphere.create_visual_aspect().set_color(list(pin_color))
            self._register_frame(sphere, body)
            pin_frames.append((i, sphere))
        self._pins.append((body, pin_frames))
        return frame

    def sync(self) -> None:
        for physics_object, frame, local_transform in self._mappings:
            tf = getattr(physics_object, "transform", None)
            if tf is None:
                continue
            try:
                frame.set_transform(_isometry_to_matrix(tf) @ local_transform)
            except Exception:  # noqa: BLE001
                pass

        for body, line, node_count in self._surfaces:
            for i in range(node_count):
                try:
                    line.set_vertex(i, np.asarray(body.node_position(i), dtype=float))
                except Exception:  # noqa: BLE001
                    pass

        for body, pin_frames in self._pins:
            for i, sphere in pin_frames:
                try:
                    sphere.set_transform(_translation(body.node_position(i)))
                except Exception:  # noqa: BLE001
                    pass

    def renderable_provider(self) -> list[Any]:
        self.sync()
        return self.render_world.renderable_provider()

    def _resolve_drag_target(self, event: dict[str, Any]) -> Any | None:
        renderable_id = int(event.get("renderable_id", 0) or 0)
        if renderable_id:
            physics_object = self._by_renderable_id.get(renderable_id)
            if physics_object is None:
                self._refresh_renderable_ids()
                physics_object = self._by_renderable_id.get(renderable_id)
            if physics_object is not None:
                return physics_object
        return self._by_name.get(event.get("renderable_name", ""))

    def _force_target_rejection(self, physics_object: Any | None) -> str | None:
        if physics_object is None:
            return "no mapped target"
        if not hasattr(physics_object, "apply_force"):
            return "target has no force input"
        try:
            if bool(getattr(physics_object, "is_static", False)):
                return "static target"
        except Exception:  # noqa: BLE001
            return "target state unavailable"
        return None

    def _force_target_label(self, physics_object: Any, frame_name: str) -> str:
        name = str(getattr(physics_object, "name", "") or "").strip()
        return name or frame_name

    def _force_target_hint(self) -> str:
        if not self.force_drag_enabled:
            return "disabled"
        labels: list[str] = []
        seen: set[int] = set()
        for frame_name, physics_object in self._by_name.items():
            identity = id(physics_object)
            if identity in seen:
                continue
            seen.add(identity)
            if self._force_target_rejection(physics_object) is None:
                labels.append(self._force_target_label(physics_object, frame_name))
        if not labels:
            return "none"
        preview = labels[:3]
        suffix = (
            ""
            if len(labels) <= len(preview)
            else f", +{len(labels) - len(preview)} more"
        )
        return ", ".join(preview) + suffix

    def _event_vector(self, event: dict[str, Any], key: str) -> np.ndarray:
        if key not in event:
            raise ValueError(f"missing {key}")
        vector = np.asarray(event[key], dtype=float).reshape(-1)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"invalid {key}")
        return vector

    def force_drag(self, event: dict[str, Any]) -> None:
        if not event.get("active", False):
            self._drag = None
            self._last_drag_status = "idle"
            self._last_drag_magnitude = 0.0
            return
        if not self.force_drag_enabled:
            self._drag = None
            self._last_drag_status = "disabled"
            self._last_drag_target = "disabled"
            self._last_drag_magnitude = 0.0
            return
        physics_object = self._resolve_drag_target(event)
        rejection = self._force_target_rejection(physics_object)
        if rejection is not None:
            self._drag = None
            self._last_drag_status = rejection
            self._last_drag_target = str(
                getattr(physics_object, "name", event.get("renderable_name", "none"))
            )
            self._last_drag_magnitude = 0.0
            return
        try:
            force = self._event_vector(event, "force") * float(self.force_drag_scale)
            point = self._event_vector(event, "application_point")
        except (TypeError, ValueError):
            self._drag = None
            self._last_drag_status = "invalid event"
            self._last_drag_target = str(
                getattr(physics_object, "name", event.get("renderable_name", "none"))
            )
            self._last_drag_magnitude = 0.0
            return
        self._drag = (physics_object, force, point)
        self._last_drag_target = str(getattr(physics_object, "name", "target"))
        self._last_drag_status = "applying"
        self._last_drag_magnitude = float(np.linalg.norm(force))
        self._force_history.append(self._last_drag_magnitude)

    def _apply_drag_force(
        self, physics_object: Any, force: np.ndarray, point: np.ndarray
    ) -> tuple[Any, np.ndarray | None, np.ndarray | None] | None:
        if hasattr(physics_object, "apply_torque"):
            previous_force = (
                np.asarray(getattr(physics_object, "force"), dtype=float).reshape(3)
                if hasattr(physics_object, "force")
                else None
            )
            previous_torque = (
                np.asarray(getattr(physics_object, "torque"), dtype=float).reshape(3)
                if hasattr(physics_object, "torque")
                else None
            )
            physics_object.apply_force(force)
            try:
                translation = np.asarray(
                    physics_object.translation, dtype=float
                ).reshape(3)
                physics_object.apply_torque(np.cross(point - translation, force))
            except Exception:  # noqa: BLE001
                pass
            return (physics_object, previous_force, previous_torque)

        physics_object.apply_force(
            force,
            point,
            force_in_world_frame=True,
            point_in_world_frame=True,
        )
        return None

    def _restore_rigid_force(
        self, restore: tuple[Any, np.ndarray | None, np.ndarray | None]
    ) -> None:
        physics_object, previous_force, previous_torque = restore
        try:
            if previous_force is not None:
                physics_object.force = previous_force
            elif hasattr(physics_object, "clear_force"):
                physics_object.clear_force()
            if previous_torque is not None:
                physics_object.torque = previous_torque
            elif hasattr(physics_object, "clear_torque"):
                physics_object.clear_torque()
        except Exception:  # noqa: BLE001
            pass

    def build_control_panel(self, builder: Any, _context: Any) -> None:
        builder.text("External force")
        builder.text("Drag a dynamic body in the viewport to apply force.")
        builder.text(f"drag target: {self._force_target_hint()}")
        changed, enabled = builder.checkbox(
            "Enable external force", self.force_drag_enabled
        )
        if changed:
            self.force_drag_enabled = bool(enabled)
            if not self.force_drag_enabled:
                self._drag = None
                self._last_drag_status = "disabled"
        changed, scale = builder.slider(
            "Force scale", float(self.force_drag_scale), 0.1, 5.0
        )
        if changed:
            self.force_drag_scale = float(scale)
        builder.text(f"status: {self._last_drag_status}")
        builder.text(f"target: {self._last_drag_target}")
        builder.text(f"magnitude: {self._last_drag_magnitude:.2f} N")
        if self._force_history:
            builder.plot_lines("Force magnitude", list(self._force_history))

    @staticmethod
    def _drag_velocity(physics_object: Any, point: np.ndarray) -> np.ndarray:
        linear = getattr(physics_object, "linear_velocity", None)
        if linear is None:
            return np.zeros(3)
        velocity = np.asarray(linear, dtype=float).reshape(3)
        angular = getattr(physics_object, "angular_velocity", None)
        if angular is not None:
            omega = np.asarray(angular, dtype=float).reshape(3)
            translation = getattr(physics_object, "translation", None)
            com = (
                np.zeros(3)
                if translation is None
                else np.asarray(translation, dtype=float).reshape(3)
            )
            velocity = velocity + np.cross(omega, point - com)
        return velocity

    def pre_step(self) -> None:
        restores: list[tuple[Any, np.ndarray | None, np.ndarray | None]] = []
        if self._drag is not None:
            physics_object, force, point = self._drag
            velocity = self._drag_velocity(physics_object, point)
            damped_force = force - self._DRAG_DAMPING_KD * velocity
            try:
                restore = self._apply_drag_force(physics_object, damped_force, point)
                if restore is not None:
                    restores.append(restore)
            except Exception:  # noqa: BLE001
                pass
        try:
            self._physics_world.step()
        except Exception:  # noqa: BLE001
            pass
        for restore in restores:
            self._restore_rigid_force(restore)
        self.sync()
