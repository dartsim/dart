"""Getting Started > empty: an empty world with a draggable gizmo."""

from __future__ import annotations

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    target = dart.gui.InteractiveFrame(dart.dynamics.Frame.World())
    world.addSimpleFrame(target)

    panel = dart.viewer.Panel()
    panel.add_text("Drag the gizmo target in the 3D view to move it around.", wrap=True)
    position = panel.add_text("")

    def pre_refresh():
        p = target.getWorldTransform().translation()
        position.label = f"Position: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        on_activate=lambda ctx: ctx.enable_drag_and_drop(target),
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[2.57, 3.14, 1.64], center=[0, 0, 0], up=[-0.24, -0.25, 0.94]
        ),
        shadows=False,
    )


def scenes():
    return [
        dart.viewer.Scene(
            "empty",
            title="Empty Scaffold",
            category="Getting Started",
            summary="A minimal empty world with a draggable InteractiveFrame gizmo.",
            factory=build,
        )
    ]
