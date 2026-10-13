"""Nested frames, ellipsoid pose markers, and an optional arrow."""

from __future__ import annotations

import math

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()

    tf1 = dart.math.Isometry3()
    tf1.set_translation([0.1, -0.1, 0])
    tf2 = dart.math.Isometry3()
    tf2.set_translation([0, 0.1, 0])
    tf2.set_rotation(dart.math.AngleAxis(math.radians(45), [1, 0, 0]).to_rotation_matrix())
    tf3 = dart.math.Isometry3()
    tf3.set_translation([0, 0, 0.1])
    tf3.set_rotation(dart.math.AngleAxis(math.radians(60), [0, 1, 0]).to_rotation_matrix())

    def frame(parent, name, transform, shape, color):
        result = dart.dynamics.SimpleFrame(parent, name, transform)
        result.setShape(shape)
        result.createVisualAspect().setColor(color)
        return result

    red = [0.85, 0.25, 0.25]
    green = [0.25, 0.75, 0.25]
    blue = [0.25, 0.45, 0.90]
    f1 = frame(
        dart.dynamics.Frame.World(),
        "F1",
        tf1,
        dart.dynamics.BoxShape([0.05, 0.05, 0.02]),
        red,
    )
    f2 = frame(f1, "F2", tf2, dart.dynamics.BoxShape([0.05, 0.05, 0.02]), green)
    f3 = frame(f2, "F3", tf3, dart.dynamics.BoxShape([0.05, 0.05, 0.02]), blue)
    world.addSimpleFrame(f1)

    a = frame(
        dart.dynamics.Frame.World(),
        "A",
        dart.math.Isometry3(),
        dart.dynamics.EllipsoidShape([0.02, 0.02, 0.02]),
        [0.85, 0.85, 0.85],
    )
    a1 = frame(
        a, "A1", f1.getTransform(a), dart.dynamics.EllipsoidShape([0.01] * 3), red
    )
    a2 = frame(
        a, "A2", f2.getTransform(a), dart.dynamics.EllipsoidShape([0.01] * 3), green
    )
    a3 = frame(
        a, "A3", f3.getTransform(a), dart.dynamics.EllipsoidShape([0.01] * 3), blue
    )
    world.addSimpleFrame(a)

    arrow = frame(
        dart.dynamics.Frame.World(),
        "arrow",
        dart.math.Isometry3(),
        dart.dynamics.ArrowShape(
            [0.1, -0.1, 0],
            [0.1, 0, 0],
            dart.dynamics.ArrowShapeProperties(0.002, 1.8),
            [1, 0.5, 0.5, 1],
        ),
        [1, 0.5, 0.5],
    )
    world.addSimpleFrame(arrow)

    # Frame child links do not own the nested frames.
    anchors = (f1, f2, f3, a, a1, a2, a3, arrow)

    def pre_step():
        _ = anchors

    panel = dart.viewer.Panel()
    panel.add_text(
        "F1 -> F2 -> F3 are nested box frames; A is an ellipsoid with "
        "child markers A1/A2/A3 tracking F1/F2/F3's poses.",
        wrap=True,
    )
    show_arrow = panel.add_checkbox("Show arrow", True)

    def toggle_arrow():
        arrow.getVisualAspect(True).setHidden(not show_arrow.value)

    show_arrow.on_change = toggle_arrow

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[0.55, 0.25, 0.5], center=[0.08, 0, 0.03], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "simple_frames",
            title="Simple Frames",
            category="Visualization",
            summary="Nested SimpleFrames with ellipsoid markers and an arrow shape.",
            factory=build,
        )
    ]
