"""Rigid IPC inclined slide: a box slides down a tilted static ramp under
gravity and friction through the experimental rigid implicit-barrier (IPC)
solver (PLAN-082).

The ramp and box are both rotated about the y-axis so the contact is a tilted
face-face barrier; lagged Coulomb friction sets how fast the box accelerates
down-slope. SxRenderBridge mirrors the bodies into a parallel
dart.simulation.World for rendering.
"""

from __future__ import annotations

import math
from collections import deque

import dartpy as dart
import dartpy.simulation_experimental as sx
import numpy as np

from .._sx_bridge import SxRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_RAMP_HALF = (2.0, 1.0, 0.1)
_BOX_HALF = (0.2, 0.2, 0.2)
_TILT_RAD = math.radians(20.0)
_FRICTION = 0.2


def _full(half: tuple[float, float, float]) -> np.ndarray:
    return np.array([2.0 * half[0], 2.0 * half[1], 2.0 * half[2]])


def _tilt_quaternion() -> tuple[float, float, float, float]:
    # Rotation about the y-axis by the tilt angle, as (w, x, y, z).
    return (math.cos(_TILT_RAD / 2.0), 0.0, math.sin(_TILT_RAD / 2.0), 0.0)


def _down_slope_axis() -> np.ndarray:
    axis = np.array([math.cos(_TILT_RAD), 0.0, -math.sin(_TILT_RAD)])
    return axis / np.linalg.norm(axis)


def build() -> SceneSetup:
    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.time_step = 0.005

    tilt = _tilt_quaternion()

    ramp_options = sx.RigidBodyOptions()
    ramp_options.is_static = True
    ramp_options.position = (0.0, 0.0, 0.0)
    ramp_options.orientation = tilt
    ramp = world.add_rigid_body("ipc_incline_ramp", ramp_options)
    ramp.set_collision_shape(sx.CollisionShape.box(_RAMP_HALF))
    ramp.friction = _FRICTION

    # Place the box just above the ramp's top face along its (tilted) normal.
    normal = (math.sin(_TILT_RAD), 0.0, math.cos(_TILT_RAD))
    offset = _RAMP_HALF[2] + _BOX_HALF[2] + 0.004
    box_options = sx.RigidBodyOptions()
    box_options.mass = 1.0
    box_options.orientation = tilt
    box_options.position = (normal[0] * offset, 0.0, normal[2] * offset)
    box = world.add_rigid_body("ipc_incline_box", box_options)
    box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
    box.friction = _FRICTION

    world.enter_simulation_mode()

    bridge = SxRenderBridge(world, name="sx_rigid_ipc_incline_render")
    bridge.add_rigid_body_visual(
        ramp,
        dart.BoxShape(_full(_RAMP_HALF)),
        (0.7, 0.7, 0.7),
        name="ipc_incline_ramp_visual",
    )
    bridge.add_rigid_body_visual(
        box,
        dart.BoxShape(_full(_BOX_HALF)),
        (0.95, 0.65, 0.15),
        name="ipc_incline_box_visual",
    )
    bridge.sync()

    down_slope = _down_slope_axis()
    speed_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        velocity = np.asarray(box.linear_velocity, dtype=float)
        down_slope_speed = float(np.dot(velocity, down_slope))
        speed_history.append(down_slope_speed)
        builder.text("solver: rigid IPC")
        builder.text(f"tilt: {math.degrees(_TILT_RAD):.1f} deg")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"down-slope speed: {down_slope_speed:.3f} m/s")
        changed, friction = builder.slider("Friction", float(box.friction), 0.0, 1.0)
        if changed:
            box.friction = float(friction)
            ramp.friction = float(friction)
        builder.plot_lines("Speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid IPC Incline", build_panel)],
        info={"sx_world": world, "rigid_body_solver": "ipc"},
    )


SCENE = PythonDemoScene(
    id="sx_rigid_ipc_incline",
    title="Rigid IPC Inclined Slide (sx)",
    category="Experimental",
    summary="A box slides down a tilted ramp under friction via the rigid IPC barrier solver.",
    build=build,
)
