"""Visualization > drag_and_drop.

Straight port of ``python/examples/drag_and_drop/main.py``: a gizmo-draggable
interactive frame carrying a draggable child box, plus X/Y/Z axis markers.
"""

import dartpy as dart

from ..registry import SceneHandle


def _marker(world, position, color, name):
    tf = dart.math.Isometry3()
    tf.set_translation(position)
    marker = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), name, tf)
    marker.setShape(dart.dynamics.BoxShape([0.2, 0.2, 0.2]))
    marker.getVisualAspect(True).setColor(color)
    world.addSimpleFrame(marker)


def build() -> SceneHandle:
    world = dart.simulation.World()

    tf = dart.math.Isometry3()
    tf.set_translation([4, -4, 0])
    frame = dart.gui.osg.InteractiveFrame(
        dart.dynamics.Frame.World(), "interactive frame", tf, 2
    )
    world.addSimpleFrame(frame)

    tf = dart.math.Isometry3()
    tf.set_translation([-4, 4, 0])
    draggable = dart.dynamics.SimpleFrame(frame, "draggable", tf)
    draggable.setShape(dart.dynamics.BoxShape([1, 1, 1]))
    draggable.getVisualAspect(True).setColor([0.9, 0, 0])
    world.addSimpleFrame(draggable)

    _marker(world, [8, 0, 0], [0.9, 0, 0], "X")
    _marker(world, [0, 8, 0], [0, 0.9, 0], "Y")
    _marker(world, [0, 0, 8], [0, 0, 0.9], "Z")

    node = dart.gui.osg.WorldNode(world)

    return SceneHandle(
        node=node,
        drag_and_drop=[frame, draggable],
        instructions=["Ctrl + Left-click: Rotate the box\n"],
        camera_home=([20, 17, 17], [0, 0, 0], [0, 0, 1]),
    )
