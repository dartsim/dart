"""Rigid Body > rigid_chain.

Straight port of ``python/examples/rigid_chain/main.py``: a damped
articulated chain loaded from a skeleton file with a randomized initial pose.
"""

from __future__ import annotations

import dartpy as dart
import numpy as np


def build() -> dart.viewer.SceneSetup:
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
            eye=[0.8, 0.0, 0.8], center=[0, -0.25, 0], up=[0, 0.5, 0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "rigid_chain",
            title="Rigid Chain",
            category="Rigid Body",
            summary="A damped articulated chain loaded from a skeleton file.",
            factory=build,
        )
    ]
