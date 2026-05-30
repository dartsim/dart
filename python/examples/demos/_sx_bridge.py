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

import numpy as np

import dartpy as dart


def _isometry_to_matrix(transform: Any) -> np.ndarray:
    """Convert an sx transform (Isometry3) to a numpy 4x4."""

    if hasattr(transform, "matrix"):
        return np.asarray(transform.matrix())
    return np.asarray(transform)


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
        self.render_world.set_time_step(
            getattr(sx_world, "time_step", 0.001))
        self._mappings: list[tuple[Any, dart.SimpleFrame]] = []
        # (deformable_body, [per-node SimpleFrame]) for deformable point clouds.
        self._deformables: list[tuple[Any, list[dart.SimpleFrame]]] = []

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
    ) -> list["dart.SimpleFrame"]:
        """Render a deformable body as a per-node sphere point cloud.

        One SimpleFrame sphere is created per node and re-positioned from
        ``deformable_body.node_position(i)`` each frame, mirroring the C++
        deformable demo's "Point Masses" view. Pinned nodes use ``fixed_color``
        when given.
        """

        frames: list[dart.SimpleFrame] = []
        group = len(self._deformables)
        node_count = int(deformable_body.node_count)
        for i in range(node_count):
            frame = dart.SimpleFrame(
                dart.Frame.world(), f"sx_node_{group}_{i}", np.eye(4))
            frame.set_shape(dart.SphereShape(radius))
            node_color = color
            if fixed_color is not None:
                try:
                    if deformable_body.is_fixed_node(i):
                        node_color = fixed_color
                except Exception:  # noqa: BLE001
                    pass
            frame.create_visual_aspect().set_color(list(node_color))
            self.render_world.add_simple_frame(frame)
            frames.append(frame)
        self._deformables.append((deformable_body, frames))
        return frames

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

        for body, frames in self._deformables:
            for i, frame in enumerate(frames):
                try:
                    position = np.asarray(body.node_position(i))
                    transform = np.eye(4)
                    transform[:3, 3] = position
                    frame.set_transform(transform)
                except Exception:  # noqa: BLE001
                    pass

    def pre_step(self) -> None:
        """Advance sx physics by one step, then sync render frames."""

        try:
            self._sx_world.step()
        except Exception:  # noqa: BLE001
            pass
        self.sync()
