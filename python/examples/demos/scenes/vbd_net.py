"""VBD net: a spring net pinned at its two top corners, solved by VBD.

Mirrors the C++ ``experimental_deformable`` net scene (without a ground), so the
contact-free body runs through the VBD inner solver. The net sags and sways
under gravity between its pinned corners.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup


def _make_net_options(columns: int, rows: int) -> "sx.DeformableBodyOptions":
    options = sx.DeformableBodyOptions()
    spacing = 0.26
    half_width = 0.5 * spacing * (columns - 1)

    positions = []
    velocities = []
    masses = []
    for row in range(rows):
        for col in range(columns):
            x = spacing * col - half_width
            y = 0.035 * math.sin(0.9 * col)
            z = 0.92 - 0.135 * row
            positions.append(np.array([x, y, z]))
            velocities.append(
                np.array([0.0, 0.16 * math.sin(0.7 * row + 0.3 * col), 0.0]))
            masses.append(0.08)
    options.positions = positions
    options.velocities = velocities
    options.masses = masses

    def index(col: int, row: int) -> int:
        return row * columns + col

    # Pin the two top corners.
    options.fixed_nodes = [index(0, 0), index(columns - 1, 0)]

    edges = []
    for row in range(rows):
        for col in range(columns):
            if col + 1 < columns:
                edges.append(
                    sx.DeformableEdge(index(col, row), index(col + 1, row), -1.0))
            if row + 1 < rows:
                edges.append(
                    sx.DeformableEdge(index(col, row), index(col, row + 1), -1.0))
            if col + 1 < columns and row + 1 < rows:
                edges.append(
                    sx.DeformableEdge(
                        index(col, row), index(col + 1, row + 1), -1.0))
                edges.append(
                    sx.DeformableEdge(
                        index(col + 1, row), index(col, row + 1), -1.0))
    options.edges = edges
    options.edge_stiffness = 65.0
    options.damping = 1.1
    return options


def build() -> SceneSetup:
    columns, rows = 9, 5
    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 1.0 / 120.0
    world.add_deformable_body("vbd_net", _make_net_options(columns, rows))

    solver = sx.DeformableSolverOptions()
    solver.iterations = 20
    world.configure_deformable_solver("vbd_net", solver)
    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="vbd_net_render")
    body = world.get_deformable_body("vbd_net")
    bridge.add_deformable_visual(
        body, (0.20, 0.70, 0.55), radius=0.03, fixed_color=(0.95, 0.50, 0.16))
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": columns * rows},
    )


SCENE = PythonDemoScene(
    id="vbd_net",
    title="VBD Net (sx)",
    category="Experimental",
    summary="A spring net pinned at its top corners, solved by VBD.",
    build=build,
)
