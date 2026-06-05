"""FEM cantilever bar (IPC deformable solver).

A tetrahedralized beam is pinned at one end and sags under gravity, held by
stable neo-Hookean *finite-element* volumetric elasticity (PLAN-081 M1) rather
than the mass-spring edge model the other scenes use. This is the first DART
deformable showcase driven by true FEM elasticity -- the keystone capability for
the IPC paper's volumetric scenes (e.g. Fig. 4 rod twist / Fig. 14 mat twist).

DART-native FEM showcase -- not a faithful IPC paper-figure reproduction (no
codimensional contact or friction here; this isolates the volumetric elasticity).
"""

from __future__ import annotations

from collections import deque

import numpy as np
import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_fem_bar
from ..runner import PythonDemoScene, ScenePanel, SceneSetup


def build() -> SceneSetup:
    options, edges = build_fem_bar(
        cells_x=10,
        cells_y=1,
        cells_z=1,
        cell_size=0.12,
        origin=(-0.6, -0.06, 0.75),
        youngs_modulus=5.0e6,
        poisson_ratio=0.3,
        density=1.0e3,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005
    body = world.add_deformable_body("fem_bar", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_fem_bar")
    bridge.add_deformable_visual(body, name="fem_bar", edges=edges)

    initial_positions = np.asarray(options.positions, dtype=float)
    free_end_indices = np.flatnonzero(
        np.isclose(initial_positions[:, 0], np.max(initial_positions[:, 0]))
    )
    free_end_height0 = float(np.mean(initial_positions[free_end_indices, 2]))
    tip_drop_history = deque(maxlen=120)
    span_z_history = deque(maxlen=120)
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
        builder.text("material: stable neo-Hookean FEM")
        builder.text(f"pins: clamped root face ({len(options.fixed_nodes)} nodes)")
        if positions.size:
            free_end_height = float(np.mean(positions[free_end_indices, 2]))
            tip_drop = free_end_height0 - free_end_height
            span_z = float(np.max(positions[:, 2]) - np.min(positions[:, 2]))
            mean_speed = (
                float(np.mean(np.linalg.norm(velocities, axis=1)))
                if velocities.size
                else 0.0
            )
            tip_drop_history.append(tip_drop)
            span_z_history.append(span_z)
            speed_history.append(mean_speed)
            builder.text(f"free-end height: {free_end_height:.3f} m")
            builder.text(f"tip drop: {tip_drop:.3f} m")
            builder.text(f"vertical span: {span_z:.3f} m")
            builder.text(f"mean node speed: {mean_speed:.3f} m/s")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if tip_drop_history:
            builder.separator()
            builder.plot_lines("Tip drop", list(tip_drop_history))
            builder.plot_lines("Span z", list(span_z_history))
            builder.plot_lines("Mean speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC FEM Bar", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_fem_bar",
    title="Deformable FEM Bar (IPC)",
    category="IPC Deformable",
    summary="A tetrahedral FEM cantilever sags under gravity via stable neo-Hookean elasticity.",
    build=build,
)
