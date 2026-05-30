"""VBD cloth: a contact-free hanging curtain solved by Vertex Block Descent.

Mirrors the C++ ``experimental_vbd`` demo scene. The pinned top row holds a
spring-net curtain that billows out of plane from an initial gust and settles
under gravity. Because the body is contact-free, the experimental World routes
it through the VBD inner solver (selected with the public, solver-agnostic
``World.configure_deformable_solver``).
"""

from __future__ import annotations

import math

import numpy as np

import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, SceneSetup


def _make_cloth_options(columns: int, rows: int) -> "sx.DeformableBodyOptions":
    options = sx.DeformableBodyOptions()
    spacing = 0.06
    top_height = 1.25
    half_width = 0.5 * spacing * (columns - 1)

    positions = []
    velocities = []
    masses = []
    for row in range(rows):
        for col in range(columns):
            x = spacing * col - half_width
            z = top_height - spacing * row
            # A small out-of-plane billow so the rest shape reads as cloth.
            y = 0.02 * math.sin(1.7 * col)
            positions.append(np.array([x, y, z]))
            # A sideways gust, strongest at the free bottom edge.
            gust = 0.9 * row / (rows - 1)
            velocities.append(np.array([0.0, gust * math.sin(0.6 * col), 0.0]))
            masses.append(0.03)
    options.positions = positions
    options.velocities = velocities
    options.masses = masses
    # Pin the entire top row so the curtain hangs.
    options.fixed_nodes = list(range(columns))

    def index(col: int, row: int) -> int:
        return row * columns + col

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
    options.edge_stiffness = 120.0
    options.damping = 0.6
    return options


def build() -> SceneSetup:
    columns, rows = 17, 13
    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 1.0 / 120.0
    world.add_deformable_body("vbd_cloth", _make_cloth_options(columns, rows))

    solver = sx.DeformableSolverOptions()
    solver.iterations = 20
    world.configure_deformable_solver("vbd_cloth", solver)
    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="vbd_cloth_render")
    body = world.get_deformable_body("vbd_cloth")
    bridge.add_deformable_visual(
        body, (0.12, 0.57, 0.91), fixed_color=(0.95, 0.50, 0.16))
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": columns * rows},
    )


SCENE = PythonDemoScene(
    id="vbd_cloth",
    title="VBD Cloth (sx)",
    category="Experimental",
    summary="A contact-free hanging cloth solved by Vertex Block Descent.",
    build=build,
)
