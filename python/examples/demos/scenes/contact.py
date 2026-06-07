"""Contact scene: a prismatic leg with a sphere foot drops onto static ground.

World owns the physics; WorldRenderBridge renders a sphere foot + a box
ground in a parallel render World.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup


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

    bridge = WorldRenderBridge(world, name="contact_render")
    bridge.add_link_visual(
        leg, dart.SphereShape(0.2), (0.20, 0.55, 0.90), name="leg_visual"
    )
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(np.array([5.0, 5.0, 0.5])),
        (0.7, 0.7, 0.7),
        name="ground_visual",
    )
    bridge.sync()

    height_history: deque[float] = deque(maxlen=120)
    slider_speed_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        joint = leg.parent_joint
        height = float(np.asarray(leg.translation, dtype=float)[2])
        velocity = np.asarray(joint.velocity, dtype=float)
        slider_speed = float(abs(velocity[0])) if velocity.size else 0.0
        height_history.append(height)
        slider_speed_history.append(slider_speed)

        builder.text("solver: contact sx world")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"foot height: {height:.3f} m")
        builder.text(f"slider speed: {slider_speed:.3f} m/s")
        changed, friction = builder.slider(
            "Ground friction", float(ground.friction), 0.0, 1.0
        )
        if changed:
            ground.friction = float(friction)
        builder.plot_lines("Foot height", list(height_history))
        builder.plot_lines("Slider speed", list(slider_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Contact", build_panel)],
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="contact",
    title="Contact",
    category="World Rigid Body",
    summary="A sphere-footed leg drops and rests on static ground.",
    build=build,
)
