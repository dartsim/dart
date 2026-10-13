"""Rigid cube stack with directional, decaying force controls."""

from __future__ import annotations

import dartpy as dart
import numpy as np

from .common import reorient_z_up


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/cubes.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/cubes.skel")
    reorient_z_up(world)
    force = np.zeros(3)

    def pre_step():
        nonlocal force
        if world.getNumSkeletons() > 1:
            world.getSkeleton(1).getBodyNode(0).addExtForce(force)
        force /= 2.0

    def apply_force(value):
        force[:] = value

    panel = dart.viewer.Panel()
    skeletons = panel.add_text(f"Skeletons: {world.getNumSkeletons()}")
    panel.add_text(
        "Force keys apply a 500 N impulse to the second skeleton's first "
        "body, decaying by half each step. Contact forces are visualized "
        "host-wide via Diagnostics > Contact Visualizer.",
        wrap=True,
    )

    def pre_refresh():
        skeletons.label = f"Skeletons: {world.getNumSkeletons()}"

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction(
                key, label, lambda value=value: apply_force(value)
            )
            for key, label, value in [
                ("1", "Force -X", [-500, 0, 0]),
                ("2", "Force +X", [500, 0, 0]),
                ("3", "Force +Y", [0, 500, 0]),
                ("4", "Force -Y", [0, -500, 0]),
            ]
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[1.1, 1.1, 0.55], center=[0, 0, -0.3], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "rigid_cubes",
            title="Rigid Cubes",
            category="Rigid Body",
            summary=(
                "A loaded cube stack; apply directional impulses and visualize "
                "live contact forces."
            ),
            factory=build,
        )
    ]
