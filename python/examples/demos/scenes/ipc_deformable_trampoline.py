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

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import (
    IpcDeformableBridge,
    build_grid_options,
    grid_index,
)
from ..runner import PythonDemoScene, SceneSetup

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

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_trampoline",
    title="Deformable Trampoline (IPC)",
    category="IPC Deformable (sx)",
    summary="A corner-pinned membrane sags and rebounds under the projected-Newton IPC solver.",
    build=build,
)
