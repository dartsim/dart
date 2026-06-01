"""VBD net: a spring net pinned at its two top corners, solved by VBD.

Mirrors the C++ ``experimental_deformable`` net scene (without a ground), so the
contact-free body runs through the VBD inner solver. The net sags and sways
under gravity between its pinned corners.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np

import dartpy.simulation_experimental as sx

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup


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

    pin_indices = (0, columns - 1)
    sag_history = deque(maxlen=120)
    sway_history = deque(maxlen=120)
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
        builder.text("pins: two top corners")
        if positions.size:
            pin_height = float(np.mean([positions[i, 2] for i in pin_indices]))
            min_z = float(np.min(positions[:, 2]))
            sag = pin_height - min_z
            sway = float(np.mean(positions[:, 1]))
            lateral_speed = (
                float(np.mean(np.linalg.norm(velocities[:, :2], axis=1)))
                if velocities.size
                else 0.0
            )
            sag_history.append(sag)
            sway_history.append(sway)
            speed_history.append(lateral_speed)
            builder.text(f"net sag: {sag:.3f} m")
            builder.text(f"mean lateral sway: {sway:.3f} m")
            builder.text(f"mean lateral speed: {lateral_speed:.3f} m/s")
        diagnostics = getattr(world, "last_deformable_solver_diagnostics", None)
        if diagnostics is not None:
            builder.text(f"solver iters: {diagnostics.solver_iterations}")
        if sag_history:
            builder.separator()
            builder.plot_lines("Net sag", list(sag_history))
            builder.plot_lines("Lateral sway", list(sway_history))
            builder.plot_lines("Lateral speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("VBD Net", build_panel)],
        info={"sx_world": world, "nodes": columns * rows},
    )


SCENE = PythonDemoScene(
    id="vbd_net",
    title="VBD Net (sx)",
    category="Vertex Block Descent (sx)",
    summary="A spring net pinned at its top corners, solved by VBD.",
    build=build,
)
