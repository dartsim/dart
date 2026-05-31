"""VBD cloth: a contact-free hanging curtain solved by Vertex Block Descent.

Mirrors the C++ ``experimental_vbd`` demo scene. The pinned top row holds a
spring-net curtain that billows out of plane from an initial gust and settles
under gravity. Because the body is contact-free, the experimental World routes
it through the VBD inner solver (selected with the public, solver-agnostic
``World.configure_deformable_solver``).
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np

import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup


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

    top_indices = list(range(columns))
    sag_history = deque(maxlen=120)
    span_y_history = deque(maxlen=120)
    speed_history = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = np.asarray(
            [body.node_position(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        velocities = np.asarray(
            [body.node_velocity(i) for i in range(int(body.node_count))],
            dtype=float,
        )
        builder.text(f"grid: {columns} x {rows}")
        builder.text(f"pins: top row ({len(top_indices)} nodes)")
        if positions.size:
            pin_height = float(np.mean([positions[i, 2] for i in top_indices]))
            min_z = float(np.min(positions[:, 2]))
            sag = pin_height - min_z
            span_y = float(np.max(positions[:, 1]) - np.min(positions[:, 1]))
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            sag_history.append(sag)
            span_y_history.append(span_y)
            speed_history.append(mean_speed)
            builder.text(f"cloth sag: {sag:.3f} m")
            builder.text(f"out-of-plane span: {span_y:.3f} m")
            builder.text(f"mean node speed: {mean_speed:.3f} m/s")
        diagnostics = getattr(world, "last_deformable_solver_diagnostics", None)
        if diagnostics is not None:
            builder.text(f"solver iters: {diagnostics.solver_iterations}")
        if sag_history:
            builder.separator()
            builder.plot_lines("Cloth sag", list(sag_history))
            builder.plot_lines("Out-of-plane span", list(span_y_history))
            builder.plot_lines("Mean speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("VBD Cloth", build_panel)],
        info={"sx_world": world, "nodes": columns * rows},
    )


SCENE = PythonDemoScene(
    id="vbd_cloth",
    title="VBD Cloth (sx)",
    category="Experimental",
    summary="A contact-free hanging cloth solved by Vertex Block Descent.",
    build=build,
)
