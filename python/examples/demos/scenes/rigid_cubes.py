"""Rigid Body > rigid_cubes.

Port of ``python/examples/rigid_cubes/main.py``: a cube-stack skeleton
dropped onto a ground plane. This keeps the standalone Python example's
behavior; the C++ scene's impulse controls arrive in a later parity port.
"""

from __future__ import annotations

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/cubes.skel")
    world.setGravity([0, -9.81, 0])

    return dart.viewer.SceneSetup(
        world,
        shadows=True,
        camera_home=dart.viewer.CameraHome(
            eye=[0.8, 0.0, 0.8], center=[0, -0.25, 0], up=[0, 0.5, 0]
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
