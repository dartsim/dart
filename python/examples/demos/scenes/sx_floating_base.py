"""Floating-base scene: a free body drifts and spins under SE(3) integration."""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    world = sx.World()
    robot = world.add_multibody("floating")
    base = robot.add_link("base")
    body = robot.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 1.5
    body.inertia = ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))

    joint = body.parent_joint
    # Free-joint velocity is [linear; angular]: drift along +X while spinning.
    joint.velocity = [1.0, 0.0, 0.0, 0.0, 0.0, 2.0]

    world.time_step = 0.01
    world.enter_simulation_mode()
    return SceneSetup(world=world, info={"dofs": robot.num_dofs})


SCENE = PythonDemoScene(
    id="sx_floating_base",
    title="Floating Base (sx)",
    category="Rigid Body (Experimental)",
    summary="A free-floating body drifts and spins under SE(3) integration.",
    build=build,
)
