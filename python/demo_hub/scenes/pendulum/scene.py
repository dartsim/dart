from __future__ import annotations

import math
import numpy as np
from typing import Optional

import dartpy as dart

from demo_hub.core.scene_base import Scene, SceneMetadata


class PendulumScene(Scene):
    metadata = SceneMetadata(
        scene_id="pendulum",
        label="Chain Pendulum",
        summary="Three-link pendulum with light damping and a small drive torque.",
        tags=("dynamics", "chain"),
    )

    def __init__(self) -> None:
        super().__init__()
        self._world: Optional[dart.World] = None
        self._pendulum: Optional[dart.Skeleton] = None
        self._dt = 0.0

    def setup(self, dt: float) -> None:
        self._dt = dt
        self._world = dart.World()
        self._world.set_time_step(dt)
        self._world.set_gravity([0, 0, -9.81])

        self._pendulum = dart.Skeleton("pendulum")
        self._build_chain(self._pendulum)
        self._world.add_skeleton(self._pendulum)

        # Kick the first joint a bit so it starts moving.
        self._pendulum.get_dof(0).set_position(math.radians(45.0))
        self._set_last_dt(dt)

    def update(self, dt: float) -> None:
        if self._world is None or self._pendulum is None:
            return
        if dt != self._dt:
            self._world.set_time_step(dt)
            self._dt = dt
            self._set_last_dt(dt)

        # Apply a gentle sinusoidal torque on the root joint.
        t = self._world.get_time()
        torque = 0.5 * math.sin(2.0 * math.pi * 0.5 * t)
        self._pendulum.get_joint(0).set_command(0, torque)

        self._world.step()

    def reset(self) -> None:
        self.setup(self._dt or 1.0 / 240.0)

    def export_state(self) -> dict:
        if self._world is None or self._pendulum is None:
            return {}
        return {
            "time": float(self._world.get_time()),
            "frames": int(self._world.get_sim_frames()),
            "root_pos_rad": float(self._pendulum.get_positions()[0]),
        }

    def get_world(self):
        return self._world

    def debug_draw_2d(self):
        if self._pendulum is None:
            return []
        segments = []
        color = (0.8, 0.6, 0.2)
        prev = None
        for i in range(self._pendulum.get_num_body_nodes()):
            body = self._pendulum.get_body_node(i)
            pos = body.get_world_transform().translation()
            x, _, z = pos
            if prev is not None:
                segments.append((prev, (x, z), color))
            prev = (x, z)
        return segments

    def _build_chain(self, pendulum: dart.Skeleton) -> None:
        length = 0.6
        radius = 0.04

        # Root ball joint at the origin.
        root_joint, root_body = pendulum.create_ball_joint_and_body_node_pair(None)
        for i in range(3):
            root_joint.set_damping_coefficient(i, 0.02)
        self._attach_box_visual(root_body, radius, length)
        parent = root_body
        for idx in range(1, 3):
            joint, body = pendulum.create_revolute_joint_and_body_node_pair(parent)
            joint.set_axis(np.array([0.0, 1.0, 0.0], dtype=float))
            tf = dart.math.Isometry3()
            tf.set_translation(np.array([0.0, 0.0, -length], dtype=float))
            joint.set_transform_from_parent_body_node(tf)
            joint.set_damping_coefficient(0, 0.02)
            self._attach_box_visual(body, radius, length)

            parent = body

    @staticmethod
    def _attach_box_visual(body: dart.BodyNode, radius: float, length: float) -> None:
        dims = np.array([radius * 2.0, radius * 2.0, length], dtype=float)
        shape = dart.BoxShape(dims)
        shape_node = body.create_shape_node(shape)
        shape_node.create_visual_aspect()
        shape_node.create_collision_aspect()
        shape_node.create_dynamics_aspect()
        tf = dart.math.Isometry3()
        tf.set_translation(np.array([0.0, 0.0, -length * 0.5], dtype=float))
        shape_node.set_relative_transform(tf)
