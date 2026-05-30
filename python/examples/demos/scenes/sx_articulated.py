"""Articulated dynamics scene: a 2-DOF arm on the experimental World.

Builds an sx::World (revolute shoulder + universal wrist) for physics, plus a
parallel dart.simulation.World (via SxRenderBridge) of SimpleFrame box
visuals so the C++ viewer can render the result.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def build() -> SceneSetup:
    world = sx.World()
    robot = world.add_multibody("arm")
    base = robot.add_link("base")
    upper = robot.add_link(
        "upper_arm",
        parent=base,
        joint=sx.JointSpec(
            name="shoulder",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=_translation(1.0, 0.0, 0.0),
        ),
    )
    upper.mass = 2.0
    upper.inertia = ((0.10, 0.0, 0.0), (0.0, 0.20, 0.0), (0.0, 0.0, 0.30))

    fore = robot.add_link(
        "forearm",
        parent=upper,
        joint=sx.JointSpec(
            name="wrist",
            type=sx.JointType.UNIVERSAL,
            axis=(0.0, 0.0, 1.0),
            axis2=(0.0, 1.0, 0.0),
            transform_from_parent=_translation(1.0, 0.0, 0.0),
        ),
    )
    fore.mass = 1.0
    fore.inertia = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))

    world.enter_simulation_mode()

    # Render bridge: a small box per link tracking the link's world transform.
    bridge = SxRenderBridge(world, name="sx_articulated_render")
    bridge.add_link_visual(
        base, dart.BoxShape(np.array([0.18, 0.18, 0.18])),
        (0.65, 0.65, 0.65), name="base_visual")
    bridge.add_link_visual(
        upper, dart.BoxShape(np.array([0.9, 0.12, 0.12])),
        (0.20, 0.55, 0.90), name="upper_visual")
    bridge.add_link_visual(
        fore, dart.BoxShape(np.array([0.9, 0.10, 0.10])),
        (0.30, 0.80, 0.45), name="fore_visual")
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="sx_articulated",
    title="Articulated Dynamics (sx)",
    category="Experimental",
    summary="A revolute + universal arm exercising the experimental World.",
    build=build,
)
