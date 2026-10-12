"""Constraints & Joints > rigid_loop.

Straight port of ``python/examples/rigid_loop/main.py``: a chain closed into
a loop by a ball-joint constraint linking two non-adjacent links.
"""

from __future__ import annotations

import math

import dartpy as dart
import numpy as np


def build() -> dart.viewer.SceneSetup:
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

    def on_activate(ctx):
        grid = dart.gui.GridVisual()
        grid.setPlaneType(dart.gui.GridVisual.PlaneType.ZX)
        grid.setOffset([0, -0.55, 0])
        ctx.add_attachment(grid)

    return dart.viewer.SceneSetup(
        world,
        on_activate=on_activate,
        shadows=False,
        camera_home=dart.viewer.CameraHome(
            eye=[0.6, 0.3, 0.6], center=[0, -0.2, 0], up=[0, 1, 0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "rigid_loop",
            title="Rigid Loop",
            category="Constraints & Joints",
            summary="A chain closed into a loop with a ball-joint constraint.",
            factory=build,
        )
    ]
