"""Visualization > drag_and_drop.

Straight port of ``python/examples/drag_and_drop/main.py``: a gizmo-draggable
interactive frame carrying a draggable child box, plus X/Y/Z axis markers.
"""

from __future__ import annotations

import dartpy as dart


def _marker(world, position, color, name):
    tf = dart.math.Isometry3()
    tf.set_translation(position)
    marker = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), name, tf)
    marker.setShape(dart.dynamics.BoxShape([0.2, 0.2, 0.2]))
    marker.getVisualAspect(True).setColor(color)
    world.addSimpleFrame(marker)


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()

    tf = dart.math.Isometry3()
    tf.set_translation([4, -4, 0])
    frame = dart.gui.InteractiveFrame(
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

    def on_activate(ctx):
        ctx.enable_drag_and_drop(frame)
        ctx.enable_drag_and_drop(draggable)

    panel = dart.viewer.Panel()
    panel.add_text("Ctrl + Left-click: Rotate the box", wrap=True)

    return dart.viewer.SceneSetup(
        world,
        on_activate=on_activate,
        panel=panel,
        shadows=False,
        camera_home=dart.viewer.CameraHome(
            eye=[20, 17, 17], center=[0, 0, 0], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "drag_and_drop",
            title="Drag and Drop",
            category="Visualization",
            summary=(
                "Drag and rotate a frame with a gizmo; click renderables to select."
            ),
            factory=build,
        )
    ]
