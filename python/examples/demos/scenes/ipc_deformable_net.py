"""Hanging spring net (experimental IPC deformable solver).

A 9x5 point-mass/spring net is pinned at its two top corners and sags under
gravity, swaying from small initial cross-velocities. This mirrors the C++
``experimental_deformable`` net scene. The sx world owns the IPC-style
deformable solve (clamped-log self-contact barriers, projected Newton); the
render bridge draws per-node spheres + a spring wireframe.

DART-native point-mass/spring showcase -- not a faithful IPC paper-figure
reproduction (no FEM/codimensional elasticity or obstacle contact).
"""

from __future__ import annotations

from collections import deque
import math

import dartpy.simulation_experimental as sx
import numpy as np

from .._ipc_deformable_bridge import (
    IpcDeformableBridge,
    build_grid_options,
    grid_index,
)
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_COLUMNS = 9
_ROWS = 5
_SPACING = 0.26


def build() -> SceneSetup:
    half_width = 0.5 * _SPACING * (_COLUMNS - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        x = _SPACING * col - half_width
        y = 0.035 * math.sin(0.9 * col)
        z = 0.92 - 0.135 * row
        return (x, y, z)

    def velocity(col: int, row: int) -> tuple[float, float, float]:
        return (0.0, 0.16 * math.sin(0.7 * row + 0.3 * col), 0.0)

    fixed = [
        grid_index(_COLUMNS, 0, 0),
        grid_index(_COLUMNS, _COLUMNS - 1, 0),
    ]
    options = build_grid_options(
        _COLUMNS,
        _ROWS,
        position_fn=position,
        velocity_fn=velocity,
        mass=0.08,
        edge_stiffness=65.0,
        damping=1.1,
        fixed_nodes=fixed,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.002
    body = world.add_deformable_body("deformable_net", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_net")
    bridge.add_deformable_visual(body, name="deformable_net")

    pin_indices = (
        grid_index(_COLUMNS, 0, 0),
        grid_index(_COLUMNS, _COLUMNS - 1, 0),
    )
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
        builder.text(f"grid: {_COLUMNS} x {_ROWS}")
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
        diagnostics = world.last_deformable_solver_diagnostics
        builder.text(
            f"solver iters: {diagnostics.solver_iterations} | "
            f"contacts: {diagnostics.self_contact_barrier_active_contacts}"
        )
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if sag_history:
            builder.separator()
            builder.plot_lines("Net sag", list(sag_history))
            builder.plot_lines("Lateral sway", list(sway_history))
            builder.plot_lines("Lateral speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC Net", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_net",
    title="Deformable Net (IPC)",
    category="IPC Deformable (sx)",
    summary="A pinned spring net sags and sways under the IPC deformable solver.",
    build=build,
)
