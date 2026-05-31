"""Floating-base scene: a free body drifts and spins under SE(3) integration.

sx::World owns the physics; SxRenderBridge mirrors the body's world transform
onto a SimpleFrame box visual each frame so the viewer renders the motion.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup


def build() -> SceneSetup:
    world = sx.World()
    # The C++ scene runs in zero-G so the body drifts/spins on its own; match.
    world.gravity = (0.0, 0.0, 0.0)
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

    bridge = SxRenderBridge(world, name="sx_floating_render")
    bridge.add_link_visual(
        body, dart.BoxShape(np.array([0.4, 0.3, 0.2])),
        (0.95, 0.50, 0.16), name="floating_body_visual")
    bridge.sync()

    linear_speed_history: deque[float] = deque(maxlen=120)
    angular_speed_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        joint = body.parent_joint
        velocity = np.asarray(joint.velocity, dtype=float)
        linear_speed = float(np.linalg.norm(velocity[:3]))
        angular_speed = float(np.linalg.norm(velocity[3:]))
        linear_speed_history.append(linear_speed)
        angular_speed_history.append(angular_speed)

        builder.text("solver: floating-base sx world")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text("gravity: zero")
        builder.text(f"linear speed: {linear_speed:.3f} m/s")
        builder.text(f"angular speed: {angular_speed:.3f} rad/s")
        changed, spin = builder.slider("Spin command", float(velocity[5]), -6.0, 6.0)
        if changed:
            next_velocity = velocity.copy()
            next_velocity[5] = float(spin)
            joint.velocity = next_velocity
        builder.plot_lines("Linear speed", list(linear_speed_history))
        builder.plot_lines("Angular speed", list(angular_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Floating Base sx", build_panel)],
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="sx_floating_base",
    title="Floating Base (sx)",
    category="Experimental",
    summary="A free-floating body drifts and spins under SE(3) integration.",
    build=build,
)
