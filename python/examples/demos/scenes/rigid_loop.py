"""Constraints & Joints > rigid_loop.

Straight port of ``python/examples/rigid_loop/main.py``: a chain closed into
a loop by a ball-joint constraint linking two non-adjacent links.
"""

import math

import dartpy as dart
import numpy as np

from ..registry import SceneHandle


def build() -> SceneHandle:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/chain.skel")
    world.setGravity([0, -9.81, 0])
    world.setTimeStep(1.0 / 2000)

    chain = world.getSkeleton(0)
    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    dof = chain.getNumDofs()
    init_pose = np.zeros(dof)
    init_pose[20] = math.pi * 0.4
    init_pose[23] = math.pi * 0.4
    init_pose[26] = math.pi * 0.4
    init_pose[29] = math.pi * 0.4
    chain.setPositions(init_pose)

    bd1 = chain.getBodyNode("link 6")
    bd2 = chain.getBodyNode("link 10")
    bd1.setColor([0, 1, 0])
    bd2.setColor([0, 1, 0])

    joint_pos = bd1.getTransform().multiply([0, 0.025, 0])
    constraint = dart.constraint.BallJointConstraint(bd1, bd2, joint_pos)
    world.getConstraintSolver().addConstraint(constraint)

    node = dart.gui.osg.RealTimeWorldNode(world)

    return SceneHandle(
        node=node,
        grid=True,
        camera_home=([0.6, 0.3, 0.6], [0, -0.2, 0], [0, 1, 0]),
    )
