"""Visualization > heightmap: native float-random terrain and a grid editor."""

from __future__ import annotations

import dartpy as dart
import numpy as np

from .common import add_grid_editor


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0, 0, 0])
    terrain = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
    visual = terrain.createVisualAspect()
    world.addSimpleFrame(terrain)
    grid = dart.gui.GridVisual()

    panel = dart.viewer.Panel()
    group = panel.add_group("Heightmap")
    show = group.add_checkbox("Show##Terrain", not visual.isHidden())
    show.on_change = lambda: visual.show() if show.value else visual.hide()
    x_resolution = group.add_input(
        "X Resolution", 100, step=5, step_fast=10, integer=True, min=5
    )
    y_resolution = group.add_input(
        "Y Resolution", 100, step=5, step_fast=10, integer=True, min=5
    )
    group.add_separator()
    x_size = group.add_input(
        "X Size", 2.0, step=0.1, step_fast=0.2, min=float(np.float32(0.1))
    )
    y_size = group.add_input(
        "Y Size", 2.0, step=0.1, step_fast=0.2, min=float(np.float32(0.1))
    )
    group.add_separator()
    z_min = group.add_input("Z Min", 0.0, step=0.05, step_fast=0.1)
    z_max = group.add_input("Z Max", float(np.float32(0.1)), step=0.05, step_fast=0.1)
    group.add_separator()
    color = group.add_color("Color##Heightmap", visual.getRGBA())
    color.on_change = lambda: visual.setColor(
        np.asarray(color.value, dtype=np.float32).astype(float)
    )
    refresh_grid = add_grid_editor(panel, grid)
    geometry = (x_resolution, y_resolution, x_size, y_size, z_min, z_max)
    applied = None
    dirty = True

    def request_regeneration():
        nonlocal dirty
        dirty = True

    for control in geometry:
        control.on_change = request_regeneration

    def pre_refresh():
        nonlocal applied, dirty
        if dirty:
            parameters = [control.value for control in geometry]
            for index, control in enumerate(geometry[2:], start=2):
                if abs(parameters[index]) > float(np.finfo(np.float32).max):
                    control.value = applied[index]
                    parameters[index] = applied[index]
                parameters[index] = float(np.float32(parameters[index]))
            low, high = sorted(parameters[-2:])
            with np.errstate(over="ignore"):
                span = np.float32(high) - np.float32(low)
            # Native generation rejects overflowing float spans; keep the last range.
            if not np.isfinite(span):
                low, high = applied[-2:]
            parameters = (*parameters[:4], low, high)
            z_min.value, z_max.value = low, high
            if parameters != applied:
                terrain.setShape(dart._demos.create_heightmap_shape(*parameters))
                transform = terrain.getRelativeTransform()
                translation = transform.translation().copy()
                translation[:2] = [-parameters[2] / 2, parameters[3] / 2]
                transform.set_translation(translation)
                terrain.setRelativeTransform(transform)
                applied = parameters
            dirty = False
        show.value = not visual.isHidden()
        color.value = visual.getRGBA()
        refresh_grid()

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        on_activate=lambda ctx: ctx.add_attachment(grid),
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[2.57, 3.14, 1.64], center=[0, 0, 0.30], up=[-0.24, -0.25, 0.94]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "heightmap",
            title="Heightmap",
            category="Visualization",
            summary="An interactive procedural heightmap with a configurable grid.",
            factory=build,
        )
    ]
