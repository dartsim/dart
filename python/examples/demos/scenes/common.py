"""World-frame and panel helpers shared by the demo scenes."""

import dartpy as dart
import numpy as np


def reorient_z_up(world):
    """Rotate a Y-up world and its gravity by +90 degrees around X."""
    rotation = dart.math.Isometry3()
    rotation.set_rotation([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    for si in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(si)
        for ji in range(skeleton.getNumJoints()):
            joint = skeleton.getJoint(ji)
            if joint.getParentBodyNode() is None:
                joint.setTransformFromParentBodyNode(
                    rotation.multiply(joint.getTransformFromParentBodyNode())
                )
    world.setGravity(rotation.rotation() @ world.getGravity())


def add_grid_editor(panel, grid):
    """Add the native scenes' grid fields and return their refresh hook."""
    group = panel.add_group("Grid", open=False)
    show = group.add_checkbox("Show##Grid", grid.isDisplayed())
    planes = [
        dart.gui.GridVisual.PlaneType.XY,
        dart.gui.GridVisual.PlaneType.YZ,
        dart.gui.GridVisual.PlaneType.ZX,
    ]
    plane = group.add_choice(
        "Plane",
        ["XY-Plane", "YZ-Plane", "ZX-Plane"],
        index=planes.index(grid.getPlaneType()),
    )
    plane.on_change = lambda: grid.setPlaneType(planes[plane.value])
    fields = [plane]
    offset = np.asarray(grid.getOffset(), dtype=np.float32).copy()
    # C++ uses columns; full-width host inputs need separate rows.
    for index, label in enumerate(("X", "Y", "Z")):
        control = group.add_input(
            label, float(offset[index]), step=0.1, step_fast=0.5, decimals=1
        )

        def change_offset(index=index, control=control):
            if abs(control.value) > float(np.finfo(np.float32).max):
                control.value = float(offset[index])
                return
            offset[index] = np.float32(control.value)
            grid.setOffset(offset.astype(float))

        control.on_change = change_offset
        fields.append(control)

    def add_input(
        label,
        value,
        setter,
        *,
        step,
        step_fast,
        integer=False,
        minimum=-np.inf,
        decimals=3,
    ):
        control = group.add_input(
            label,
            value,
            step=step,
            step_fast=step_fast,
            integer=integer,
            min=minimum,
            decimals=decimals,
        )
        previous = value

        def change():
            nonlocal previous
            if not integer and abs(control.value) > float(np.finfo(np.float32).max):
                control.value = previous
                return
            value = control.value if integer else float(np.float32(control.value))
            # C++ clamps widths to zero, which GridVisual rejects; keep the last width.
            if not integer and minimum == 0 and value <= 0:
                control.value = previous
                return
            if value != previous:
                previous = value
                setter(value)

        control.on_change = change
        fields.append(control)

    add_input(
        "Line Count",
        grid.getNumCells(),
        grid.setNumCells,
        step=1,
        step_fast=5,
        integer=True,
        minimum=0,
    )
    add_input(
        "Line Step Size",
        float(np.float32(grid.getMinorLineStepSize())),
        grid.setMinorLineStepSize,
        step=0.001,
        step_fast=0.1,
    )
    add_input(
        "Minor Lines per Major Line",
        grid.getNumMinorLinesPerMajorLine(),
        grid.setNumMinorLinesPerMajorLine,
        step=1,
        step_fast=5,
        integer=True,
        minimum=0,
    )
    add_input(
        "Axis Line Width",
        grid.getAxisLineWidth(),
        grid.setAxisLineWidth,
        step=1,
        step_fast=2,
        minimum=0,
        decimals=0,
    )
    add_input(
        "Major Line Width",
        grid.getMajorLineWidth(),
        grid.setMajorLineWidth,
        step=1,
        step_fast=2,
        minimum=0,
        decimals=0,
    )

    def add_color(label, value, setter):
        rgba = np.asarray(value, dtype=np.float32).copy()
        control = group.add_color(label, rgba.astype(float), alpha=False)

        def change_color():
            rgba[:3] = np.asarray(control.value[:3], dtype=np.float32)
            setter(rgba.astype(float))

        control.on_change = change_color
        fields.append(control)

    add_color("Major Line Color", grid.getMajorLineColor(), grid.setMajorLineColor)
    add_input(
        "Minor Line Width",
        grid.getMinorLineWidth(),
        grid.setMinorLineWidth,
        step=1,
        step_fast=2,
        minimum=0,
        decimals=0,
    )
    add_color("Minor Line Color", grid.getMinorLineColor(), grid.setMinorLineColor)

    def refresh():
        show.value = grid.isDisplayed()
        plane.value = planes.index(grid.getPlaneType())
        for control in fields:
            control.visible = show.value

    def change_display():
        grid.display(show.value)
        refresh()

    show.on_change = change_display
    refresh()
    return refresh
