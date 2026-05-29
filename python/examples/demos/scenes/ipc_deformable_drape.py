"""Cloth drape over a step (experimental IPC deformable solver).

A free point-mass/spring mat is released flat above a raised static box and
drapes over it onto the ground. Both the floor and the box are tagged as
deformable ground barriers (finite-footprint support height fields), so the
mat is held at the box top over its footprint and falls past the edges to the
surrounding ground -- exercising the landed IPC contact pipeline (ground
barrier + self-contact clamped-log barrier + sparse projected Newton). This
mirrors the C++ ``experimental_deformable`` drape scene at a viewer-friendly
mesh size.

DART-native point-mass/spring showcase -- not a faithful IPC paper-figure
reproduction (no FEM/codimensional elasticity).
"""

from __future__ import annotations

import dartpy.simulation_experimental as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_grid_options
from ..runner import PythonDemoScene, SceneSetup

# Viewer-friendly mat (252 nodes); the C++ benchmark drape is 26x22 = 572.
_COLUMNS = 18
_ROWS = 14
_SPACING = 0.05
_DROP_HEIGHT = 0.42  # above the obstacle top (0.24)

# Static ground-barrier boxes: (center, collision half-extents).
_GROUND_CENTER = (0.0, 0.0, -0.04)
_GROUND_HALF = (2.4, 1.2, 0.04)
_OBSTACLE_HALF = (0.30, 0.30, 0.12)
_OBSTACLE_CENTER = (0.0, 0.0, _OBSTACLE_HALF[2])


def _add_ground_barrier_box(world, name, center, half_extents):
    body = world.add_rigid_body(name, position=center)
    body.is_static = True
    body.set_collision_shape(sx.CollisionShape.box(half_extents))
    body.is_deformable_ground_barrier = True
    return body


def build() -> SceneSetup:
    half_width = 0.5 * _SPACING * (_COLUMNS - 1)
    half_depth = 0.5 * _SPACING * (_ROWS - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        x = _SPACING * col - half_width
        y = _SPACING * row - half_depth
        return (x, y, _DROP_HEIGHT)

    options = build_grid_options(
        _COLUMNS,
        _ROWS,
        position_fn=position,
        mass=0.05,
        edge_stiffness=25.0,  # floppy enough to drape over the step edges
        damping=1.5,
    )

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.005

    _add_ground_barrier_box(world, "ground", _GROUND_CENTER, _GROUND_HALF)
    _add_ground_barrier_box(
        world, "drape_obstacle", _OBSTACLE_CENTER, _OBSTACLE_HALF)
    body = world.add_deformable_body("deformable_drape", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_drape")
    # Full extents are twice the collision half-extents.
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_rigid_box_visual(
        _OBSTACLE_CENTER,
        tuple(2.0 * h for h in _OBSTACLE_HALF),
        (0.52, 0.45, 0.34),
        name="drape_obstacle_visual",
    )
    bridge.add_deformable_visual(body, name="deformable_drape")

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_drape",
    title="Deformable Drape (IPC)",
    category="Experimental",
    summary="A mat drapes over a step onto the ground via IPC ground + self-contact barriers.",
    build=build,
)
