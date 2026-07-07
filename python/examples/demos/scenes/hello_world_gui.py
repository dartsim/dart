"""Getting Started > hello_world_gui.

Straight port of ``python/examples/hello_world_gui/main.py``: a KR5 arm and
ground plane with no active controller. The original defined every
customPre/PostStep/Refresh hook as a no-op; a bare ``RealTimeWorldNode``
already behaves identically, so this port skips the empty subclass.
"""

import dartpy as dart

from ..registry import SceneHandle


def build() -> SceneHandle:
    world = dart.simulation.World()

    urdf_parser = dart.utils.DartLoader()
    kr5 = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    node = dart.gui.osg.RealTimeWorldNode(world)

    return SceneHandle(
        node=node,
        grid=True,
        camera_home=([2.0, 1.0, 2.0], [0.0, 0.0, 0.0], [-0.24, 0.94, -0.25]),
    )
