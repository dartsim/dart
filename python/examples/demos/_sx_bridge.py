"""Small bridge that lets sx::World scenes render through the C++ viewer.

The Filament viewer renders `dart.simulation.World`, not the experimental
`dartpy.simulation_experimental.World` (sx). The C++ experimental scenes
solve this by keeping BOTH worlds — sx for physics, a parallel
`dart.simulation.World` whose SimpleFrames are synced from sx body
transforms each frame for rendering. This module provides the same
pattern in Python.

Usage from an sx scene's `build()`::

    from .._sx_bridge import SxRenderBridge

    sx_world = sx.World()
    # ... add bodies/links to sx_world ...
    sx_world.enter_simulation_mode()

    bridge = SxRenderBridge(sx_world, name="sx_articulated")
    bridge.add_link_visual(link, dart.BoxShape(np.array([0.4, 0.1, 0.1])),
                           (0.20, 0.55, 0.85))
    bridge.add_rigid_body_visual(ground_body,
        dart.BoxShape(np.array([5.0, 5.0, 0.5])), (0.7, 0.7, 0.7))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": sx_world},
    )
"""

from __future__ import annotations

from collections import deque
from typing import Any

import dartpy as dart
import numpy as np


def _isometry_to_matrix(transform: Any) -> np.ndarray:
    """Convert an sx transform (Isometry3) to a numpy 4x4."""

    if hasattr(transform, "matrix"):
        return np.asarray(transform.matrix())
    return np.asarray(transform)


def _is_fixed_node(body: Any, index: int) -> bool:
    """``body.is_fixed_node(index)`` guarded against missing/odd bindings."""

    try:
        return bool(body.is_fixed_node(index))
    except Exception:  # noqa: BLE001
        return False


def _deformable_wireframe_edges(body: Any) -> list[tuple[int, int]]:
    """Unique undirected node-index pairs describing a body's mesh wireframe.

    Tetrahedral/surface bodies are drawn as the edges of their boundary
    triangles (a solid-looking faceted surface); mass-spring bodies are drawn
    as their spring network. The result mirrors the paper's surface rendering
    far better than an isolated per-node point cloud.
    """

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


class SxRenderBridge:
    """Maps sx::World bodies/links to dart::simulation::World SimpleFrames.

    The bridge owns a `render_world` (dart.simulation.World) containing one
    SimpleFrame per registered sx body / link. ``pre_step()`` advances the
    sx physics by one step and syncs every SimpleFrame's transform from its
    sx counterpart.
    """

    def __init__(self, sx_world: Any, name: str = "sx_render_world") -> None:
        self._sx_world = sx_world
        self.render_world = dart.World(name)
        self.render_world.set_gravity([0.0, 0.0, 0.0])
        # The render world steps too (the viewer's pump calls world.step()
        # each frame). With no skeletons it's a no-op + a time advance.
        self.render_world.set_time_step(getattr(sx_world, "time_step", 0.001))
        self._mappings: list[tuple[Any, dart.SimpleFrame]] = []
        # (deformable_body, line_shape, node_count) deforming wireframes.
        self._surfaces: list[tuple[Any, Any, int]] = []
        # (deformable_body, [(node_index, SimpleFrame)]) pinned-node markers.
        self._pins: list[tuple[Any, list[tuple[int, dart.SimpleFrame]]]] = []
        # SimpleFrame name / renderable id -> sx object, so the viewer's
        # external-force drag can resolve the picked renderable back to the sx
        # body/link that owns the physics.
        self._by_name: dict[str, Any] = {}
        self._by_renderable_id: dict[int, Any] = {}
        # Active mouse external force, if any: (sx_object, force, point).
        # Re-applied before each sx step because Link.apply_force is one-shot.
        self._drag: tuple[Any, np.ndarray, np.ndarray] | None = None
        self.force_drag_enabled = True
        self.force_drag_scale = 1.0
        self._last_drag_target = "none"
        self._last_drag_status = "idle"
        self._last_drag_magnitude = 0.0
        self._force_history: deque[float] = deque(maxlen=120)

    def _register_frame(self, frame: "dart.SimpleFrame", sx_object: Any) -> str:
        actual_name = str(self.render_world.add_simple_frame(frame))
        self._by_name[actual_name] = sx_object
        self._refresh_renderable_ids()
        return actual_name

    def _refresh_renderable_ids(self) -> None:
        gui = getattr(dart, "gui", None)
        if gui is None or not hasattr(gui, "extract_renderables"):
            return
        try:
            renderables = gui.extract_renderables(self.render_world)
        except Exception:  # noqa: BLE001
            return
        for renderable in renderables:
            sx_object = self._by_name.get(renderable.shape_frame_name)
            if sx_object is not None:
                self._by_renderable_id[int(renderable.id)] = sx_object

    def add_link_visual(
        self,
        sx_link: Any,
        shape: "dart.Shape",
        color: tuple[float, float, float],
        name: str | None = None,
    ) -> "dart.SimpleFrame":
        frame_name = name or f"sx_link_{len(self._mappings)}"
        frame = dart.SimpleFrame(dart.Frame.world(), frame_name, np.eye(4))
        frame.set_shape(shape)
        frame.create_visual_aspect().set_color(list(color))
        actual_name = self._register_frame(frame, sx_link)
        self._mappings.append((sx_link, frame))
        self._by_name[actual_name] = sx_link
        return frame

    def add_rigid_body_visual(
        self,
        sx_body: Any,
        shape: "dart.Shape",
        color: tuple[float, float, float],
        name: str | None = None,
    ) -> "dart.SimpleFrame":
        return self.add_link_visual(sx_body, shape, color, name=name)

    def add_deformable_visual(
        self,
        deformable_body: Any,
        color: tuple[float, float, float],
        radius: float = 0.018,
        fixed_color: tuple[float, float, float] | None = None,
        thickness: float = 2.5,
    ) -> "dart.SimpleFrame":
        """Render a deformable body as a live deforming surface wireframe.

        A single ``LineSegmentShape`` carries one vertex per node and one line
        per mesh edge — spring edges for mass-spring bodies (cloth/net) or
        boundary-triangle edges for tetrahedral bodies (the beam). Each frame
        the vertices are rewritten from ``deformable_body.node_position(i)``,
        which bumps the shape version so the viewer re-uploads the deformed
        geometry. Pinned nodes are marked with small ``fixed_color`` spheres so
        the constraints read clearly. Returns the wireframe SimpleFrame.
        """

        body = deformable_body
        node_count = int(body.node_count)

        line = dart.LineSegmentShape(float(thickness))
        for i in range(node_count):
            line.add_vertex(np.asarray(body.node_position(i), dtype=float))
        for node_a, node_b in _deformable_wireframe_edges(body):
            line.add_connection(node_a, node_b)

        frame = dart.SimpleFrame(
            dart.Frame.world(), f"sx_surface_{len(self._surfaces)}", np.eye(4)
        )
        frame.set_shape(line)
        frame.create_visual_aspect().set_color(list(color))
        self.render_world.add_simple_frame(frame)
        self._surfaces.append((body, line, node_count))

        pin_color = fixed_color if fixed_color is not None else color
        pin_frames: list[tuple[int, dart.SimpleFrame]] = []
        group = len(self._pins)
        for i in range(node_count):
            if not _is_fixed_node(body, i):
                continue
            sphere = dart.SimpleFrame(
                dart.Frame.world(), f"sx_pin_{group}_{i}", np.eye(4)
            )
            sphere.set_shape(dart.SphereShape(radius))
            sphere.create_visual_aspect().set_color(list(pin_color))
            self._register_frame(sphere, body)
            pin_frames.append((i, sphere))
        self._pins.append((body, pin_frames))
        return frame

    def sync(self) -> None:
        """Copy each sx body's world transform onto its SimpleFrame."""

        for sx_object, frame in self._mappings:
            tf = getattr(sx_object, "transform", None)
            if tf is None:
                continue
            try:
                frame.set_transform(_isometry_to_matrix(tf))
            except Exception:  # noqa: BLE001
                pass

        # Rewrite wireframe vertices in place (set_vertex bumps the shape
        # version, so the viewer re-uploads the deformed geometry).
        for body, line, node_count in self._surfaces:
            for i in range(node_count):
                try:
                    line.set_vertex(i, np.asarray(body.node_position(i), dtype=float))
                except Exception:  # noqa: BLE001
                    pass

        for body, pin_frames in self._pins:
            for i, sphere in pin_frames:
                try:
                    transform = np.eye(4)
                    transform[:3, 3] = np.asarray(body.node_position(i))
                    sphere.set_transform(transform)
                except Exception:  # noqa: BLE001
                    pass

    def _resolve_drag_target(self, event: dict[str, Any]) -> Any | None:
        renderable_id = int(event.get("renderable_id", 0) or 0)
        if renderable_id:
            sx_object = self._by_renderable_id.get(renderable_id)
            if sx_object is None:
                self._refresh_renderable_ids()
                sx_object = self._by_renderable_id.get(renderable_id)
            if sx_object is not None:
                return sx_object
        return self._by_name.get(event.get("renderable_name", ""))

    def _force_target_rejection(self, sx_object: Any | None) -> str | None:
        if sx_object is None:
            return "no mapped target"
        if not hasattr(sx_object, "apply_force"):
            return "target has no force input"
        try:
            if bool(getattr(sx_object, "is_static", False)):
                return "static target"
        except Exception:  # noqa: BLE001
            return "target state unavailable"
        return None

    def _force_target_label(self, sx_object: Any, frame_name: str) -> str:
        name = str(getattr(sx_object, "name", "") or "").strip()
        return name or frame_name

    def _force_target_hint(self) -> str:
        if not self.force_drag_enabled:
            return "disabled"

        labels: list[str] = []
        seen: set[int] = set()
        for frame_name, sx_object in self._by_name.items():
            identity = id(sx_object)
            if identity in seen:
                continue
            seen.add(identity)
            if self._force_target_rejection(sx_object) is not None:
                continue
            labels.append(self._force_target_label(sx_object, frame_name))

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
        """Viewer mouse external-force handler (see ``SceneSetup.force_drag``).

        On an active event, resolve the picked frame name to its sx object and
        store the world-frame force + application point; on an inactive event,
        clear the stored drag. The force is (re)applied in ``pre_step`` before
        each sx step because ``Link.apply_force`` is one-shot.
        """

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
        sx_object = self._resolve_drag_target(event)
        rejection = self._force_target_rejection(sx_object)
        if rejection is not None:
            self._drag = None
            self._last_drag_status = rejection
            self._last_drag_target = str(
                getattr(sx_object, "name", event.get("renderable_name", "none"))
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
                getattr(sx_object, "name", event.get("renderable_name", "none"))
            )
            self._last_drag_magnitude = 0.0
            return
        self._drag = (sx_object, force, point)
        self._last_drag_target = str(getattr(sx_object, "name", "target"))
        self._last_drag_status = "applying"
        self._last_drag_magnitude = float(np.linalg.norm(force))
        self._force_history.append(self._last_drag_magnitude)

    def _apply_drag_force(
        self, sx_object: Any, force: np.ndarray, point: np.ndarray
    ) -> tuple[Any, np.ndarray | None, np.ndarray | None] | None:
        if hasattr(sx_object, "apply_torque"):
            previous_force = (
                np.asarray(getattr(sx_object, "force"), dtype=float).reshape(3)
                if hasattr(sx_object, "force")
                else None
            )
            previous_torque = (
                np.asarray(getattr(sx_object, "torque"), dtype=float).reshape(3)
                if hasattr(sx_object, "torque")
                else None
            )
            sx_object.apply_force(force)
            try:
                translation = np.asarray(sx_object.translation, dtype=float).reshape(3)
                sx_object.apply_torque(np.cross(point - translation, force))
            except Exception:  # noqa: BLE001
                pass
            return (sx_object, previous_force, previous_torque)

        sx_object.apply_force(
            force,
            point,
            force_in_world_frame=True,
            point_in_world_frame=True,
        )
        return None

    def _restore_rigid_force(
        self, restore: tuple[Any, np.ndarray | None, np.ndarray | None]
    ) -> None:
        sx_object, previous_force, previous_torque = restore
        try:
            if previous_force is not None:
                sx_object.force = previous_force
            elif hasattr(sx_object, "clear_force"):
                sx_object.clear_force()
            if previous_torque is not None:
                sx_object.torque = previous_torque
            elif hasattr(sx_object, "clear_torque"):
                sx_object.clear_torque()
        except Exception:  # noqa: BLE001
            pass

    def build_control_panel(self, builder: Any, context: Any) -> None:
        """Render generic sx bridge controls into a DART ``PanelBuilder``."""

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

    def pre_step(self) -> None:
        """Advance sx physics by one step, then sync render frames.

        If a mouse external-force drag is active, (re)apply its one-shot force just
        before the step so the spring is consumed by this step.
        """

        restores: list[tuple[Any, np.ndarray | None, np.ndarray | None]] = []
        if self._drag is not None:
            sx_object, force, point = self._drag
            try:
                restore = self._apply_drag_force(sx_object, force, point)
                if restore is not None:
                    restores.append(restore)
            except Exception:  # noqa: BLE001
                pass
        try:
            self._sx_world.step()
        except Exception:  # noqa: BLE001
            pass
        for restore in restores:
            self._restore_rigid_force(restore)
        self.sync()
