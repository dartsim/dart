"""A damped Z-up chain closed by a ball-joint constraint."""

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
    dof = chain.getNumDofs()
    init_pose = np.zeros(dof)
    if dof > 29:
        init_pose[[20, 23, 26, 29]] = 3.14159 * 0.4
    chain.setPositions(init_pose)

    bd1 = chain.getBodyNode("link 6")
    bd2 = chain.getBodyNode("link 10")
    if bd1 is not None and bd2 is not None:
        bd1.setColor([1, 0, 0])
        bd2.setColor([1, 0, 0])
        joint_pos = bd1.getTransform().multiply([0, 0.025, 0])
        constraint = dart.constraint.BallJointConstraint(bd1, bd2, joint_pos)
        world.getConstraintSolver().addConstraint(constraint)

    damping_coeff = 0.01
    twist_damping_scale = 0.1
    panel = dart.viewer.Panel()
    panel.add_text(f"DOFs: {dof}")
    speed = panel.add_text("Speed: 0.000 rad/s")
    damping = panel.add_slider("Damping", damping_coeff, min=0.0, max=0.1, decimals=4)
    twist_scale = panel.add_slider("Twist scale", twist_damping_scale, min=0.0, max=1.0)
    panel.add_text(
        "The red links are connected by a ball-joint constraint, closing "
        "the chain into a loop.",
        wrap=True,
    )

    def set_damping():
        nonlocal damping_coeff
        damping_coeff = float(np.float32(damping.value))

    def set_twist_scale():
        nonlocal twist_damping_scale
        twist_damping_scale = float(np.float32(twist_scale.value))

    damping.on_change = set_damping
    twist_scale.on_change = set_twist_scale

    def pre_step():
        forces = -damping_coeff * chain.getVelocities()
        forces[1::3] *= twist_damping_scale
        chain.setForces(forces)

    def pre_refresh():
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
            "rigid_loop",
            title="Rigid Loop",
            category="Constraints & Joints",
            summary="A chain closed into a loop with a ball-joint constraint.",
            factory=build,
        )
    ]
