from __future__ import annotations

import math
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
        self._world: Optional[dart.simulation.World] = None
        self._pendulum: Optional[dart.dynamics.Skeleton] = None
        self._dt = 0.0

    def setup(self, dt: float) -> None:
        self._dt = dt
        self._world = dart.simulation.World()
        self._world.setTimeStep(dt)
        self._world.setGravity([0, 0, -9.81])

        self._pendulum = dart.dynamics.Skeleton("pendulum")
        self._build_chain(self._pendulum)
        self._world.addSkeleton(self._pendulum)

        # Kick the first joint a bit so it starts moving.
        self._pendulum.getDof(0).setPosition(math.radians(45.0))
        self._set_last_dt(dt)

    def update(self, dt: float) -> None:
        if self._world is None or self._pendulum is None:
            return
        if dt != self._dt:
            self._world.setTimeStep(dt)
            self._dt = dt
            self._set_last_dt(dt)

        # Apply a gentle sinusoidal torque on the root joint.
        t = self._world.getTime()
        torque = 0.5 * math.sin(2.0 * math.pi * 0.5 * t)
        self._pendulum.getJoint(0).setCommand(0, torque)

        self._world.step()

    def reset(self) -> None:
        self.setup(self._dt or 1.0 / 240.0)

    def export_state(self) -> dict:
        if self._world is None or self._pendulum is None:
            return {}
        return {
            "time": float(self._world.getTime()),
            "frames": int(self._world.getSimFrames()),
            "root_pos_rad": float(self._pendulum.getPosition(0)),
        }

    def debug_draw_2d(self):
        if self._pendulum is None:
            return []
        segments = []
        pts = []
        for i in range(self._pendulum.getNumBodyNodes()):
            tf = self._pendulum.getBodyNode(i).getTransform()
            pos = tf.translation()
            pts.append((float(pos[0]), float(pos[2])))
        if pts:
            base = (0.0, 0.0)
            segments.append((base, pts[0], (1.0, 1.0, 0.6)))
            for i in range(len(pts) - 1):
                segments.append((pts[i], pts[i + 1], (0.6, 0.9, 0.4)))
        return segments

    def debug_draw_2d(self):
        if self._pendulum is None:
            return []
        segments = []
        color = (0.8, 0.6, 0.2)
        prev = None
        for i in range(self._pendulum.getNumBodyNodes()):
            body = self._pendulum.getBodyNode(i)
            pos = body.getWorldTransform().translation()
            x, _, z = pos
            if prev is not None:
                segments.append((prev, (x, z), color))
            prev = (x, z)
        return segments

    @staticmethod
    def _make_link_inertia(length: float, radius: float, mass: float) -> dart.dynamics.Inertia:
        box = dart.dynamics.BoxShape([radius * 2, radius * 2, length])
        inertia = dart.dynamics.Inertia()
        inertia.setMass(mass)
        inertia.setMoment(box.computeInertia(mass))
        return inertia

    def _build_chain(self, pendulum: dart.dynamics.Skeleton) -> None:
        length = 0.6
        radius = 0.04
        mass = 0.5
        inertia = self._make_link_inertia(length, radius, mass)

        # Root ball joint at the origin.
        ball_props = dart.dynamics.BallJointProperties()
        ball_props.mName = "root_joint"
        ball_props.mRestPositions = [0.0, 0.0, 0.0]
        ball_props.mSpringStiffnesses = [1.5, 1.5, 1.5]
        ball_props.mDampingCoefficients = [0.02, 0.02, 0.02]

        root_body_props = dart.dynamics.BodyNodeProperties()
        root_body_props.mName = "link0"
        root_body_props.mInertia = inertia

        root_joint, root_body = pendulum.createBallJointAndBodyNodePair(None, ball_props, root_body_props)
        self._attach_geometry(root_body, radius, length)

        parent = root_body
        for idx in range(1, 3):
            revolute_props = dart.dynamics.RevoluteJointProperties()
            revolute_props.mName = f"joint{idx}"
            revolute_props.mAxis = [0.0, 1.0, 0.0]
            revolute_props.mT_ParentBodyToJoint.set_translation([0.0, 0.0, -length])

            body_props = dart.dynamics.BodyNodeProperties()
            body_props.mName = f"link{idx}"
            body_props.mInertia = inertia

            joint, body = pendulum.createRevoluteJointAndBodyNodePair(parent, revolute_props, body_props)
            joint.setRestPosition(0, 0.0)
            joint.setDampingCoefficient(0, 0.02)
            joint.setSpringStiffness(0, 0.1)

            self._attach_geometry(body, radius, length)
            parent = body

    @staticmethod
    def _attach_geometry(body: dart.dynamics.BodyNode, radius: float, length: float) -> None:
        shape = dart.dynamics.CylinderShape(radius, length)
        shape_node = body.createShapeNode(shape)
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect()

        # Center the cylinder along -Z so the joint sits at the top.
        tf = dart.math.Isometry3()
        tf.set_translation([0.0, 0.0, -length * 0.5])
        shape_node.setRelativeTransform(tf)
