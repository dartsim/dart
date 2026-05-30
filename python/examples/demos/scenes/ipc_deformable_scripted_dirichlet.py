"""Scripted-boundary deformable banner (experimental IPC deformable solver).

A point-mass/spring banner hangs from two pinned top corners while the rest of
its top edge is driven, for a short time window, by a scripted Dirichlet
boundary condition that sweeps it out of plane; once the window closes the edge
is released and the banner billows and settles while gravity and the spring
network resolve the body. This showcases the scripted-boundary machinery
(`DeformableDirichletBoundaryCondition`) exposed to dartpy, coupled to the IPC
deformable solve.

DART-native point-mass/spring showcase -- not a faithful IPC paper-figure
reproduction.
"""

from __future__ import annotations

import numpy as np

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import (
    IpcDeformableBridge,
    build_grid_options,
    grid_index,
)
from ..runner import PythonDemoScene, SceneSetup

_COLUMNS = 11
_ROWS = 7
_SPACING = 0.12
_TOP_Z = 0.85


def build() -> SceneSetup:
    half_width = 0.5 * _SPACING * (_COLUMNS - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        x = _SPACING * col - half_width
        z = _TOP_Z - _SPACING * row
        return (x, 0.0, z)

    corners = [
        grid_index(_COLUMNS, 0, 0),
        grid_index(_COLUMNS, _COLUMNS - 1, 0),
    ]
    options = build_grid_options(
        _COLUMNS,
        _ROWS,
        position_fn=position,
        mass=0.06,
        edge_stiffness=90.0,
        damping=1.3,
        fixed_nodes=corners,
    )

    # Drive the interior of the top edge (the corners stay pinned) for a short
    # window with a scripted boundary velocity that sweeps it out of plane, then
    # release it so the banner billows and settles. The solver extrapolates the
    # angular velocity linearly (position += elapsed * omega x (pos - center)),
    # so the window is kept short to keep the sweep bounded rather than drifting.
    interior_top = [grid_index(_COLUMNS, col, 0) for col in range(1, _COLUMNS - 1)]
    scripted = sx.DeformableDirichletBoundaryCondition()
    scripted.nodes = interior_top
    scripted.angular_velocity = np.array([0.5, 0.0, 0.0])
    scripted.center = np.array([0.0, 0.0, _TOP_Z])
    scripted.start_time = 0.0
    scripted.end_time = 1.2
    options.dirichlet_boundary_conditions = [scripted]

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004
    body = world.add_deformable_body("deformable_banner", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_scripted")
    bridge.add_deformable_visual(body, name="deformable_banner")

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_scripted_dirichlet",
    title="Scripted Deformable Banner (IPC)",
    category="IPC Deformable (sx)",
    summary="A pinned banner billows under a scripted Dirichlet boundary condition.",
    build=build,
)
