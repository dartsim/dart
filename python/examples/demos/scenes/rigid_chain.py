"""Articulated chain with the native scene's velocity damping law."""

from __future__ import annotations

import dartpy as dart
import numpy as np

from .common import reorient_z_up


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/chain.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/chain.skel")
    reorient_z_up(world)
    world.setTimeStep(1.0 / 2000)
    chain = world.getSkeleton(0)
    chain.setPositions(
        [dart.math.Random.uniform(-0.5, 0.5) for _ in range(chain.getNumDofs())]
    )

    panel = dart.viewer.Panel()
    dofs = panel.add_text(f"DOFs: {chain.getNumDofs()}")
    speed = panel.add_text("Speed: 0.000 rad/s")
    damping = panel.add_slider("Damping", 0.01, min=0.0, max=0.1, decimals=4)
    twist_scale = panel.add_slider("Twist scale", 0.1, min=0.0, max=1.0, decimals=3)
    damping_coefficient = 0.01
    twist_damping_scale = 0.1

    def change_damping():
        nonlocal damping_coefficient
        damping_coefficient = float(np.float32(damping.value))

    def change_twist_scale():
        nonlocal twist_damping_scale
        twist_damping_scale = float(np.float32(twist_scale.value))

    damping.on_change = change_damping
    twist_scale.on_change = change_twist_scale
    panel.add_text(
        "Every third dof (the twist dof of each joint) is damped by an "
        "extra scale factor relative to the other two.",
        wrap=True,
    )

    def pre_step():
        forces = -damping_coefficient * chain.getVelocities()
        forces[1::3] *= twist_damping_scale
        chain.setForces(forces)

    def pre_refresh():
        dofs.label = f"DOFs: {chain.getNumDofs()}"
        speed.label = f"Speed: {np.linalg.norm(chain.getVelocities()):.3f} rad/s"

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[2, 1, 2], center=[0, 0, 0], up=[0, 0, 1]
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
