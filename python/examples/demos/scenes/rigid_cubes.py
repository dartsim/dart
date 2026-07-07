"""Rigid Body > rigid_cubes.

Port of ``python/examples/rigid_cubes/main.py``: a cube-stack skeleton
dropped onto a ground plane. Note: the C++ ``rigid_cubes`` demo scene adds
directional impulse forces, contact-force visualization, and playback on top
of this; this port keeps the original standalone example's simpler feature
set (id/category/title reused from the C++ catalog so the two runners agree
on what "rigid_cubes" means, but the summary below describes what the Python
port actually does).
"""

import dartpy as dart

from ..registry import SceneHandle


def build() -> SceneHandle:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/cubes.skel")
    world.setGravity([0, -9.81, 0])

    node = dart.gui.osg.RealTimeWorldNode(world)

    return SceneHandle(
        node=node,
        shadow=True,
        camera_home=([0.8, 0.0, 0.8], [0, -0.25, 0], [0, 0.5, 0]),
    )
