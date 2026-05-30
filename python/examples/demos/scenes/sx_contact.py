"""Contact scene: a prismatic leg with a sphere foot drops onto static ground.

sx::World owns the physics; SxRenderBridge renders a sphere foot + a box
ground in a parallel dart.simulation.World.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
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

    bridge = SxRenderBridge(world, name="sx_contact_render")
    bridge.add_link_visual(
        leg, dart.SphereShape(0.2), (0.20, 0.55, 0.90), name="leg_visual")
    bridge.add_rigid_body_visual(
        ground, dart.BoxShape(np.array([5.0, 5.0, 0.5])),
        (0.7, 0.7, 0.7), name="ground_visual")
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="sx_contact",
    title="Contact (sx)",
    category="Experimental",
    summary="A sphere-footed leg drops and rests on static ground.",
    build=build,
)
