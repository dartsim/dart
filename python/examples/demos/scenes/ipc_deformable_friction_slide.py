"""Frictional slide-to-rest (IPC deformable solver).

A point-mass/spring mat starts resting inside the ground barrier's activation
band with a horizontal launch velocity and skids to a halt as the lagged
smoothed-Coulomb ground friction (IPC Phase 4) dissipates its tangential
momentum. The mat carries a non-zero ``friction_coefficient`` on its deformable
material; with friction disabled it would slide indefinitely, so the visible
deceleration and stop is the friction term doing work. This is the cloth
counterpart of the paper's frictional-contact stress tests.

DART-native point-mass/spring showcase -- not a faithful IPC paper-figure
reproduction (no FEM/codimensional elasticity).
"""

from __future__ import annotations

import dartpy as sx

from .._ipc_deformable_bridge import IpcDeformableBridge, build_grid_options
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_COLUMNS = 12
_ROWS = 8
_SPACING = 0.06
_REST_HEIGHT = 0.012  # inside the ground barrier band (d_hat = 0.02)
_LAUNCH_SPEED = 1.8  # +x launch velocity (m/s)
_FRICTION = 0.8

# Static flat ground barrier (finite-footprint support height field).
_GROUND_CENTER = (0.0, 0.0, -0.05)
_GROUND_HALF = (2.5, 1.2, 0.05)  # top face at z = 0.0


def _add_ground_barrier_box(world, name, center, half_extents):
    body = world.add_rigid_body(name, position=center)
    body.is_static = True
    body.set_collision_shape(sx.CollisionShape.box(half_extents))
    policy = body.deformable_obstacle_policy
    policy.ground_barrier = True
    body.deformable_obstacle_policy = policy
    return body


def build() -> SceneSetup:
    half_width = 0.5 * _SPACING * (_COLUMNS - 1)
    half_depth = 0.5 * _SPACING * (_ROWS - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        x = _SPACING * col - half_width - 0.6  # start off to the -x side
        y = _SPACING * row - half_depth
        return (x, y, _REST_HEIGHT)

    def velocity(col: int, row: int) -> tuple[float, float, float]:
        return (_LAUNCH_SPEED, 0.0, 0.0)

    options = build_grid_options(
        _COLUMNS,
        _ROWS,
        position_fn=position,
        velocity_fn=velocity,
        mass=0.05,
        edge_stiffness=60.0,  # cohesive patch that still conforms to the floor
        damping=1.0,
    )
    material = options.material
    material.friction_coefficient = _FRICTION
    options.material = material

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 0.004

    _add_ground_barrier_box(world, "ground", _GROUND_CENTER, _GROUND_HALF)
    body = world.add_deformable_body("deformable_slider", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="ipc_deformable_friction_slide")
    bridge.add_rigid_box_visual(
        _GROUND_CENTER,
        tuple(2.0 * h for h in _GROUND_HALF),
        (0.37, 0.40, 0.43),
        name="ground_visual",
    )
    bridge.add_deformable_visual(body, name="deformable_slider")

    def build_panel(builder: object, context: object) -> None:
        builder.text(f"grid: {_COLUMNS} x {_ROWS}")
        builder.text(f"friction coefficient: {_FRICTION:.2f}")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel("IPC Friction Slide", build_panel)],
        info={"sx_world": world, "nodes": body.node_count},
    )


SCENE = PythonDemoScene(
    id="ipc_deformable_friction_slide",
    title="Deformable Friction Slide (IPC)",
    category="IPC Deformable",
    summary="A launched mat skids to rest under lagged Coulomb ground friction.",
    build=build,
)
