"""Corner-pinned membrane trampoline (experimental IPC deformable solver).

A square point-mass/spring sheet is pinned at its four corners and released
horizontally. Gravity pulls the free interior into a sagging "pillow" that
overshoots and oscillates as the sparse projected-Newton solve resolves the
taut spring network each step; a gentle downward impulse concentrated near the
center gives the membrane its initial bounce. Unlike the two-corner net (a
hanging hammock), pinning all four corners keeps the sheet taut so the
oscillation reads as a trampoline rather than a sag.

DART-native point-mass/spring showcase -- not a faithful IPC paper-figure
reproduction (no FEM/codimensional elasticity or obstacle contact).
"""

from __future__ import annotations

from collections import deque

import dartpy.simulation_experimental as sx
import numpy as np

from .._ipc_deformable_bridge import (
    IpcDeformableBridge,
    build_grid_options,
    grid_index,
)
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_SIZE = 11  # 11x11 = 121 nodes
_SPACING = 0.07
_HEIGHT = 0.6
_CENTER = 0.5 * (_SIZE - 1)


def build() -> SceneSetup:
    half = 0.5 * _SPACING * (_SIZE - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        return (_SPACING * col - half, _SPACING * row - half, _HEIGHT)

    def velocity(col: int, row: int) -> tuple[float, float, float]:
        # A gentle downward impulse that tapers to zero at the pinned rim, so
        # the membrane launches into a centered bounce rather than a shock.
        dx = (col - _CENTER) / _CENTER
        dy = (row - _CENTER) / _CENTER
        falloff = max(0.0, 1.0 - (dx * dx + dy * dy))
        return (0.0, 0.0, -1.6 * falloff)

    corners = [
        grid_index(_SIZE, 0, 0),
        grid_index(_SIZE, _SIZE - 1, 0),
        grid_index(_SIZE, 0, _SIZE - 1),
        grid_index(_SIZE, _SIZE - 1, _SIZE - 1),
    ]
    options = build_grid_options(
        _SIZE,
        _SIZE,
        position_fn=position,
        velocity_fn=velocity,
        mass=0.04,
        edge_stiffness=140.0,  # taut enough to rebound, not rigid
        damping=0.6,  # under-damped so the bounce is visible
        fixed_nodes=corners,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.003
    body = world.add_deformable_body("deformable_trampoline", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_trampoline")
    bridge.add_deformable_visual(body, name="deformable_trampoline")

    center_index = grid_index(_SIZE, int(_CENTER), int(_CENTER))
    center_height_history = deque(maxlen=120)
    sag_history = deque(maxlen=120)
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
        builder.text(f"grid: {_SIZE} x {_SIZE}")
        builder.text("pins: four corners")
        if positions.size:
            center_height = float(positions[center_index, 2])
            rim_height = float(
                np.mean(
                    [
                        positions[grid_index(_SIZE, 0, 0), 2],
                        positions[grid_index(_SIZE, _SIZE - 1, 0), 2],
                        positions[grid_index(_SIZE, 0, _SIZE - 1), 2],
                        positions[grid_index(_SIZE, _SIZE - 1, _SIZE - 1), 2],
                    ]
                )
            )
            sag = rim_height - center_height
            center_speed = (
                float(velocities[center_index, 2]) if velocities.size else 0.0
            )
            center_height_history.append(center_height)
            sag_history.append(sag)
            speed_history.append(center_speed)
            builder.text(f"center height: {center_height:.3f} m")
            builder.text(f"center sag: {sag:.3f} m")
            builder.text(f"center vertical speed: {center_speed:.3f} m/s")
        diagnostics = world.last_deformable_solver_diagnostics
        builder.text(
            f"solver iters: {diagnostics.solver_iterations} | "
            f"contacts: {diagnostics.self_contact_barrier_active_contacts}"
        )
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if center_height_history:
            builder.separator()
            builder.plot_lines("Center height", list(center_height_history))
            builder.plot_lines("Center sag", list(sag_history))
            builder.plot_lines("Center vertical speed", list(speed_history))

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC Trampoline", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_trampoline",
    title="Deformable Trampoline (IPC)",
    category="IPC Deformable (sx)",
    summary="A corner-pinned membrane sags and rebounds under the projected-Newton IPC solver.",
    build=build,
)
