"""Control & IK > biped_stand.

Port of ``python/examples/biped_stand/main.py``: an SPD-balanced standing
biped with an ankle strategy and keyboard push tests.

Key remap: the original example uses keys 1-4 for the four push directions
(``InputHandler`` in the standalone example). The py-demos navigator reserves
0-9 for direct scene selection, so the pushes are remapped to the arrow keys
here (Up/Down/+-X, Right/Left/+-Z) -- same four directions, non-colliding
keys.

Note: ``self.pre_offset`` is set in ``__init__`` but never reassigned in the
original example's ankle-strategy step (so the ``kd * (pre_offset - offset)``
term always sees ``pre_offset == 0``). That looks like a dead variable in the
original, not something introduced by this port; kept as-is for parity.
"""

import dartpy as dart
import numpy as np

from ..registry import SceneHandle


class BipedStandNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, skel):
        super().__init__(world)
        self.skel = skel
        self.dofs = skel.getNumDofs()
        self.left_heel = skel.getBodyNode("h_heel_left")
        self.left_foot = [
            skel.getDof("j_heel_left_1").getIndexInSkeleton(),
            skel.getDof("j_toe_left").getIndexInSkeleton(),
        ]
        self.right_foot = [
            skel.getDof("j_heel_right_1").getIndexInSkeleton(),
            skel.getDof("j_toe_right").getIndexInSkeleton(),
        ]
        self.timestep = world.getTimeStep()

        self.Kp = np.eye(self.dofs)
        self.Kd = np.eye(self.dofs)
        for i in range(6, self.dofs):
            self.Kp[i, i] = 400.0
            self.Kd[i, i] = 40.0

        self.pre_offset = 0
        self.q_d = skel.getPositions()

        self.ext_force = np.zeros(3)
        self.ext_force_duration = 0
        self.ext_force_arrow_shape = dart.dynamics.ArrowShape([0, 0, 0], [0, 0, 0])
        self.ext_force_frame = dart.dynamics.SimpleFrame()
        self.ext_force_frame.setShape(self.ext_force_arrow_shape)
        self.ext_force_visual = self.ext_force_frame.createVisualAspect()
        self.ext_force_visual.setColor([1.0, 0.0, 0.0])
        self.ext_force_visual.hide()
        world.addSimpleFrame(self.ext_force_frame)

    def customPreStep(self):
        q = self.skel.getPositions()
        dq = self.skel.getVelocities()
        constraint_forces = self.skel.getConstraintForces()

        # SPD tracking.
        inv_m = np.linalg.inv(self.skel.getMassMatrix() + self.Kd * self.timestep)
        p = np.matmul(-self.Kp, q + dq * self.timestep - self.q_d)
        d = np.matmul(-self.Kd, dq)
        ddq = np.matmul(
            inv_m,
            -self.skel.getCoriolisAndGravityForces() + p + d + constraint_forces,
        )
        torques = p + d + np.matmul(-self.Kd, ddq) * self.timestep

        # Ankle strategy for the sagittal plane.
        com = self.skel.getCOM()
        cop = self.left_heel.getTransform().multiply([0.05, 0, 0])
        offset = com[0] - cop[0]

        gains = None
        if 0.0 < offset < 0.1:
            gains = (200, 100, 10)
        elif -0.2 < offset < -0.05:
            gains = (2000, 100, 100)

        if gains is not None:
            k1, k2, kd = gains
            correction = kd * (self.pre_offset - offset)
            torques[self.left_foot[0]] += -k1 * offset + correction
            torques[self.left_foot[1]] += -k2 * offset + correction
            torques[self.right_foot[0]] += -k1 * offset + correction
            torques[self.right_foot[1]] += -k2 * offset + correction

        # Root dofs must stay unactuated.
        for i in range(6):
            torques[i] = 0
        self.skel.setForces(torques * 0.8)

        # Apply/decay the push-test external force.
        self.ext_force_duration -= 1
        if self.ext_force_duration <= 0:
            self.ext_force_duration = 0
            self.ext_force = np.zeros(3)
        spine = self.skel.getBodyNode("h_spine")
        spine.addExtForce(self.ext_force)
        if self.ext_force_duration > 0:
            arrow_head = spine.getTransform().translation()
            arrow_tail = arrow_head - self.ext_force / 30
            self.ext_force_arrow_shape.setPositions(arrow_tail, arrow_head)
            self.ext_force_arrow_shape.setDataVariance(dart.dynamics.Shape.DYNAMIC)
            self.ext_force_visual.show()
        else:
            self.ext_force_arrow_shape.setDataVariance(dart.dynamics.Shape.STATIC)
            self.ext_force_visual.hide()

    def push(self, force):
        self.ext_force = np.array(force, dtype=float)
        self.ext_force_duration = 100


def build() -> SceneHandle:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    world.setGravity([0, -9.81, 0])

    biped = world.getSkeleton("fullbody1")
    biped.getDof("j_pelvis_rot_y").setPosition(-0.20)
    biped.getDof("j_thigh_left_z").setPosition(0.15)
    biped.getDof("j_shin_left").setPosition(-0.40)
    biped.getDof("j_heel_left_1").setPosition(0.25)
    biped.getDof("j_thigh_right_z").setPosition(0.15)
    biped.getDof("j_shin_right").setPosition(-0.40)
    biped.getDof("j_heel_right_1").setPosition(0.25)
    biped.getDof("j_abdomen_2").setPosition(0.00)

    node = BipedStandNode(world, biped)

    ea = dart.gui.osg.GUIEventAdapter
    key_actions = {
        ea.KEY_Up: ("Push +X", lambda: node.push([40, 0, 0])),
        ea.KEY_Down: ("Push -X", lambda: node.push([-40, 0, 0])),
        ea.KEY_Right: ("Push +Z", lambda: node.push([0, 0, 40])),
        ea.KEY_Left: ("Push -Z", lambda: node.push([0, 0, -40])),
    }

    return SceneHandle(
        node=node,
        key_actions=key_actions,
        camera_home=([3, 1.5, 3], [0, 0, 0], [0, 0, 0]),
        notes=(
            "Push-test keys remapped from 1-4 (original) to arrow keys: "
            "navigator reserves 0-9 for direct scene select."
        ),
    )
