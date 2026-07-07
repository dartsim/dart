"""Rigid Body > rigid_chain.

Straight port of ``python/examples/rigid_chain/main.py``: a damped
articulated chain loaded from a skeleton file with a randomized initial pose.
"""

import dartpy as dart
import numpy as np

from ..registry import SceneHandle


def build() -> SceneHandle:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/chain.skel")
    world.setGravity([0, -9.81, 0])

    chain = world.getSkeleton(0)
    dof = chain.getNumDofs()

    init_pose = np.zeros(dof)
    for i in range(dof):
        init_pose[i] = dart.math.Random.uniform(-0.5, 0.5)
    chain.setPositions(init_pose)

    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    node = dart.gui.osg.RealTimeWorldNode(world)

    return SceneHandle(
        node=node,
        grid=True,
        camera_home=([0.8, 0.0, 0.8], [0, -0.25, 0], [0, 0.5, 0]),
    )
