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

    # Velocity-damping gain for the mouse force-drag. The C++ viewer can only
    # send the spring term ``F = kp*(target - app_point)`` for sx renderables
    # (it can read an sx body's transform for picking but not its velocity), so
    # the legacy ``-kd*v`` term is added here instead, mirroring the BodyNode
    # force-drag. Without it a light free sx body overshoots and rings under a
    # drag. Tunable: larger = more critically damped / heavier feel.
    _DRAG_DAMPING_KD: float = 4.0

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
        # SimpleFrame name -> sx object, so the viewer's force-drag can resolve
        # the picked renderable (identified by its frame name) back to the sx
        # body/link that owns the physics.
        self._by_name: dict[str, Any] = {}
        # Active mouse force-drag, if any: (sx_object, force, point). Re-applied
        # before each sx step because Link.apply_force is one-shot.
        self._drag: tuple[Any, np.ndarray, np.ndarray] | None = None

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
        self.render_world.add_simple_frame(frame)
        self._mappings.append((sx_link, frame))
        self._by_name[frame_name] = sx_link
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
            line.addVertex(np.asarray(body.node_position(i), dtype=float))
        for node_a, node_b in _deformable_wireframe_edges(body):
            line.addConnection(node_a, node_b)

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
            self.render_world.add_simple_frame(sphere)
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

        # Rewrite wireframe vertices in place (setVertex bumps the shape
        # version, so the viewer re-uploads the deformed geometry).
        for body, line, node_count in self._surfaces:
            for i in range(node_count):
                try:
                    line.setVertex(i, np.asarray(body.node_position(i), dtype=float))
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

    def force_drag(self, event: dict[str, Any]) -> None:
        """Viewer mouse force-drag handler (see ``SceneSetup.force_drag``).

        On an active event, resolve the picked frame name to its sx object and
        store the world-frame force + application point; on an inactive event,
        clear the stored drag. The force is (re)applied in ``pre_step`` before
        each sx step because ``Link.apply_force`` is one-shot.
        """

        if not event.get("active", False):
            self._drag = None
            return
        sx_object = self._by_name.get(event.get("renderable_name", ""))
        if sx_object is None or not hasattr(sx_object, "apply_force"):
            self._drag = None
            return
        force = np.asarray(event["force"], dtype=float).reshape(3)
        point = np.asarray(event["application_point"], dtype=float).reshape(3)
        self._drag = (sx_object, force, point)

    @staticmethod
    def _drag_velocity(sx_object: Any, point: np.ndarray) -> np.ndarray:
        """World-frame velocity of ``sx_object`` at ``point`` for drag damping.

        sx RigidBody exposes ``linear_velocity`` (+ ``angular_velocity``); when
        both are present the velocity is evaluated at the application point as
        ``v + omega x (point - com)`` so the damping opposes the actual motion
        of the dragged contact. sx multibody Links expose no velocity accessor,
        so this returns zeros for them and the drag stays spring-only.
        """

        linear = getattr(sx_object, "linear_velocity", None)
        if linear is None:
            return np.zeros(3)
        velocity = np.asarray(linear, dtype=float).reshape(3)
        angular = getattr(sx_object, "angular_velocity", None)
        if angular is not None:
            omega = np.asarray(angular, dtype=float).reshape(3)
            translation = getattr(sx_object, "translation", None)
            com = (
                np.zeros(3)
                if translation is None
                else np.asarray(translation, dtype=float).reshape(3)
            )
            velocity = velocity + np.cross(omega, point - com)
        return velocity

    def pre_step(self) -> None:
        """Advance sx physics by one step, then sync render frames.

        If a mouse force-drag is active, (re)apply its one-shot force just
        before the step so the spring is consumed by this step. The viewer only
        sends the spring term, so the legacy ``-kd*v`` damping is subtracted
        here using the dragged body's velocity at the application point.
        """

        if self._drag is not None:
            sx_object, force, point = self._drag
            velocity = self._drag_velocity(sx_object, point)
            damped_force = force - self._DRAG_DAMPING_KD * velocity
            try:
                # RigidBody exposes linear_velocity; its apply_force takes only
                # (force,) in world coordinates. Link's apply_force accepts a
                # point + frame flags too. Duck-type on linear_velocity to pick
                # the right signature.
                if hasattr(sx_object, "linear_velocity"):
                    sx_object.apply_force(damped_force)
                else:
                    sx_object.apply_force(
                        damped_force,
                        point,
                        force_in_world_frame=True,
                        point_in_world_frame=True,
                    )
            except Exception:  # noqa: BLE001
                pass
        try:
            self._sx_world.step()
        except Exception:  # noqa: BLE001
            pass
        self.sync()
