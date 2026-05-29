"""Contact scene: a prismatic leg with a sphere foot drops onto static ground."""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    world = sx.World()
    robot = world.add_multibody("leg_robot")
    base = robot.add_link("base")
    leg = robot.add_link(
        "leg",
        parent=base,
        joint=sx.JointSpec(
            name="slider", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    leg.mass = 1.0
    leg.set_collision_shape(sx.CollisionShape.sphere(0.2))
    leg.parent_joint.position = [-0.25]

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -1.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.5)))

    world.time_step = 0.002
    world.enter_simulation_mode()
    return SceneSetup(world=world, info={"dofs": robot.num_dofs})


SCENE = PythonDemoScene(
    id="sx_contact",
    title="Contact (sx)",
    category="Experimental",
    summary="A sphere-footed leg drops and rests on static ground.",
    build=build,
)
