from __future__ import annotations

import numpy as np
import dartpy as dart

from demo_hub.core.scene_base import Scene, SceneMetadata


class HelloWorldScene(Scene):
    metadata = SceneMetadata(
        scene_id="hello_world",
        label="Hello World",
        summary="Minimal headless scene that steps a free box under gravity.",
        tags=("headless", "basics"),
    )

    def __init__(self) -> None:
        super().__init__()
        self._world: dart.simulation.World | None = None
        self._dt = 0.0

    def setup(self, dt: float) -> None:
        self._dt = dt
        self._world = dart.simulation.World()
        self._world.setTimeStep(dt)
        self._world.setGravity([0, 0, -9.81])

        box = dart.dynamics.Skeleton("box")
        _, body = box.createFreeJointAndBodyNodePair()

        shape = dart.dynamics.BoxShape([0.15, 0.15, 0.15])
        shape_node = body.createShapeNode(shape)
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect()

        box.getRootJoint().setPositions(np.array([0, 0, 0.5, 0, 0, 0], dtype=float))
        self._world.addSkeleton(box)

        self._set_last_dt(dt)

    def update(self, dt: float) -> None:
        if self._world is None:
            return
        if dt != self._dt:
            self._world.setTimeStep(dt)
            self._dt = dt
            self._set_last_dt(dt)
        self._world.step()

    def reset(self) -> None:
        self.setup(self._dt or 1.0 / 240.0)

    def export_state(self) -> dict:
        if self._world is None:
            return {}
        return {"time": float(self._world.getTime()), "frames": int(self._world.getSimFrames())}

    def debug_draw_2d(self):
        if self._world is None:
            return []
        # Visualize the box as a square footprint on the XZ plane.
        skel = self._world.getSkeleton(0)
        if skel is None:
            return []
        body = skel.getBodyNode(0)
        tf = body.getWorldTransform()
        # tf is a 4x4; extract translation
        pos = tf.translation()
        x, _, z = pos
        r = 0.1
        points = [
            (x - r, z - r),
            (x + r, z - r),
            (x + r, z + r),
            (x - r, z + r),
        ]
        color = (0.2, 0.8, 0.2)
        segments = []
        for i in range(len(points)):
            segments.append((points[i], points[(i + 1) % len(points)], color))
        return segments
