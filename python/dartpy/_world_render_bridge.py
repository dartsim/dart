"""Small bridge that lets World scenes render through the C++ viewer.

The Filament viewer renders `dart.simulation.World`, while the current DART 7
World demos still build their physics state through
`dartpy.World`. The C++ scenes solve this by keeping
both worlds: one for physics, plus a parallel `dart.simulation.World` whose
SimpleFrames are synced from physics body transforms each frame for rendering.
This module provides the same pattern in Python.

Usage from a World scene's `build()`::

    from .._world_bridge import WorldRenderBridge

    physics_world = sx.World()
    # ... add bodies/links to physics_world ...
    physics_world.enter_simulation_mode()

    bridge = WorldRenderBridge(physics_world, name="articulated")
    bridge.add_link_visual(link, dart.BoxShape(np.array([0.4, 0.1, 0.1])),
                           (0.20, 0.55, 0.85))
    bridge.add_rigid_body_visual(ground_body,
        dart.BoxShape(np.array([5.0, 5.0, 0.5])), (0.7, 0.7, 0.7))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": physics_world},
    )
"""

from __future__ import annotations

from collections import deque
from typing import Any

import dartpy as dart
import numpy as np


def _isometry_to_matrix(transform: Any) -> np.ndarray:
    """Convert a physics transform (Isometry3) to a numpy 4x4."""

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


class WorldRenderBridge:
    """Maps physics World bodies/links to dart::simulation::World SimpleFrames.

    The bridge owns a `render_world` (dart.simulation.World) containing one
    SimpleFrame per registered physics body / link. ``pre_step()`` advances the
    physics world by one step and syncs every SimpleFrame's transform from its
    physics counterpart.
    """

    # Velocity-damping gain for the mouse force-drag. The C++ viewer can only
    # send the spring term ``F = kp*(target - app_point)`` for bridged
    # renderables (it can read a body's transform for picking but not its
    # velocity), so
    # the legacy ``-kd*v`` term is added here instead, mirroring the BodyNode
    # force-drag. Without it a light free body overshoots and rings under a
    # drag. Tunable: larger = more critically damped / heavier feel.
    _DRAG_DAMPING_KD: float = 4.0

    def __init__(self, physics_world: Any, name: str = "world_render") -> None:
        self._physics_world = physics_world
        # The render world is the legacy classic World, retained only for the
        # Filament viewer (dartpy.gui.RenderWorld). The physics_world is the
        # official ECS dartpy.simulation World.
        self.render_world = dart.gui.RenderWorld(name)
        self.render_world.set_gravity([0.0, 0.0, 0.0])
        # The render world steps too (the viewer's pump calls world.step()
        # each frame). With no skeletons it's a no-op + a time advance.
        self.render_world.set_time_step(getattr(physics_world, "time_step", 0.001))
        self._mappings: list[tuple[Any, dart.SimpleFrame, np.ndarray]] = []
        # (deformable_body, line_shape, node_count) deforming wireframes.
        self._surfaces: list[tuple[Any, Any, int]] = []
        # (deformable_body, [(node_index, SimpleFrame)]) pinned-node markers.
        self._pins: list[tuple[Any, list[tuple[int, dart.SimpleFrame]]]] = []
        # SimpleFrame name / renderable id -> physics object, so the viewer's
        # external-force drag can resolve the picked renderable back to the
        # body/link that owns the physics.
        self._by_name: dict[str, Any] = {}
        self._by_renderable_id: dict[int, Any] = {}
        # Active mouse external force, if any: (object, force, point).
        # Re-applied before each physics step because Link.apply_force is
        # one-shot.
        self._drag: tuple[Any, np.ndarray, np.ndarray] | None = None
        self.force_drag_enabled = True
        self.force_drag_scale = 1.0
        self._last_drag_target = "none"
        self._last_drag_status = "idle"
        self._last_drag_magnitude = 0.0
        self._force_history: deque[float] = deque(maxlen=120)

    def _register_frame(self, frame: "dart.SimpleFrame", physics_object: Any) -> str:
        actual_name = str(self.render_world.add_simple_frame(frame))
        self._by_name[actual_name] = physics_object
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
            physics_object = self._by_name.get(renderable.shape_frame_name)
            if physics_object is not None:
                self._by_renderable_id[int(renderable.id)] = physics_object

    def add_link_visual(
        self,
        physics_link: Any,
        shape: "dart.Shape",
        color: tuple[float, float, float],
        name: str | None = None,
        local_transform: Any | None = None,
    ) -> "dart.SimpleFrame":
        frame_name = name or f"world_link_{len(self._mappings)}"
        frame = dart.SimpleFrame(dart.gui.world_render_frame(), frame_name, np.eye(4))
        frame.set_shape(shape)
        frame.create_visual_aspect().set_color(list(color))
        actual_name = self._register_frame(frame, physics_link)
        local = np.eye(4) if local_transform is None else _isometry_to_matrix(local_transform)
        self._mappings.append((physics_link, frame, local))
        self._by_name[actual_name] = physics_link
        return frame

    def add_rigid_body_visual(
        self,
        physics_body: Any,
        shape: "dart.Shape",
        color: tuple[float, float, float],
        name: str | None = None,
        local_transform: Any | None = None,
    ) -> "dart.SimpleFrame":
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
            dart.gui.world_render_frame(), f"world_surface_{len(self._surfaces)}", np.eye(4)
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
                dart.gui.world_render_frame(), f"world_pin_{group}_{i}", np.eye(4)
            )
            sphere.set_shape(dart.SphereShape(radius))
            sphere.create_visual_aspect().set_color(list(pin_color))
            self._register_frame(sphere, body)
            pin_frames.append((i, sphere))
        self._pins.append((body, pin_frames))
        return frame

    def sync(self) -> None:
        """Copy each physics body's world transform onto its SimpleFrame."""

        for physics_object, frame, local_transform in self._mappings:
            tf = getattr(physics_object, "transform", None)
            if tf is None:
                continue
            try:
                frame.set_transform(_isometry_to_matrix(tf) @ local_transform)
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
            if self._force_target_rejection(physics_object) is not None:
                continue
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
        """Viewer mouse external-force handler (see ``SceneSetup.force_drag``).

        On an active event, resolve the picked frame name to its physics object and
        store the world-frame force + application point; on an inactive event,
        clear the stored drag. The force is (re)applied in ``pre_step`` before
        each physics step because ``Link.apply_force`` is one-shot.
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
                translation = np.asarray(physics_object.translation, dtype=float).reshape(3)
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

    def build_control_panel(self, builder: Any, context: Any) -> None:
        """Render generic World bridge controls into a DART ``PanelBuilder``."""

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
        """World-frame velocity of ``physics_object`` at ``point`` for drag damping.

        RigidBody exposes ``linear_velocity`` (+ ``angular_velocity``); when
        both are present the velocity is evaluated at the application point as
        ``v + omega x (point - com)`` so the damping opposes the actual motion
        of the dragged contact. Multibody Links expose no velocity accessor,
        so this returns zeros for them and the drag stays spring-only.
        """

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
        """Advance physics by one step, then sync render frames.

        If a mouse external-force drag is active, (re)apply its one-shot force
        just before the step so the spring is consumed by this step. The viewer
        sends the spring term, so the legacy ``-kd*v`` damping is subtracted
        here using the dragged body's velocity at the application point.
        """

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
