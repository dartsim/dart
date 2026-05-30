"""Variational-integrator scene: a torque-free floating body tumbles cleanly.

A single free (floating) body with asymmetric inertia is given an initial spin
in zero gravity, integrated with the **variational integrator**. The VI's
manifold-correct SE(3)/SO(3) retraction (Phase B1) keeps the tumbling motion
energy- and momentum-conserving, so the body spins indefinitely without the
spurious spin-down or drift a non-symplectic step accrues. This exercises the
floating-base VI path that the default semi-implicit scenes do not.

sx::World owns the physics; SxRenderBridge mirrors the body's world transform
onto a SimpleFrame box visual each frame.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family="variational integrator"
    )
    # Torque-free: zero gravity so the only motion is the conserved tumble.
    world.gravity = (0.0, 0.0, 0.0)
    world.time_step = 0.005

    robot = world.add_multibody("tumbler")
    base = robot.add_link("base")
    body = robot.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 1.0
    # Asymmetric principal inertias (I_y is the intermediate axis) so the
    # tumble is visibly non-trivial while still conserving energy + momentum.
    body.inertia = ((0.08, 0.0, 0.0), (0.0, 0.16, 0.0), (0.0, 0.0, 0.10))

    joint = body.parent_joint
    # [linear; angular]: a gentle drift plus a spin dominated by the
    # intermediate axis with small perturbations about the others.
    joint.velocity = [0.15, 0.0, 0.0, 0.2, 4.0, 0.2]

    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="sx_variational_tumbler_render")
    bridge.add_link_visual(
        body,
        dart.BoxShape(np.array([0.55, 0.22, 0.38])),
        (0.16, 0.71, 0.78),
        name="tumbler_visual",
    )
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="sx_variational_tumbler",
    title="Variational Tumbler (sx)",
    category="Experimental",
    summary="A torque-free floating body tumbles without energy/momentum drift.",
    build=build,
)
