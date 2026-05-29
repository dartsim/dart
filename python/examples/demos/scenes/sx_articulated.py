"""Articulated dynamics scene: a 2-DOF arm on the experimental World."""

from __future__ import annotations

import numpy as np

import dartpy.simulation_experimental as sx

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
    return SceneSetup(world=world, info={"dofs": robot.num_dofs})


SCENE = PythonDemoScene(
    id="sx_articulated",
    title="Articulated Dynamics (sx)",
    category="Experimental",
    summary="A revolute + universal arm exercising the experimental World.",
    build=build,
)
