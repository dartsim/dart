"""Vehicle scene: PD-controlled four-wheel car on a flat ground.

Mirrors examples/demos/scenes/vehicle.cpp at a high level. The C++
scene exposes WASD-style keyboard actions for forward/back/steer; the
Python mirror drives the back wheels with a fixed forward velocity and
gently sweeps the steering, so the scene exercises the same PD
controller without interactive input.
"""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup
from ._z_up import reorient_to_z_up

_WORLD_URI = "dart://sample/skel/vehicle.skel"
_CAR_NAME = "car_skeleton"

_WHEEL_SPEED = math.radians(420.0)
_MAX_STEER = math.radians(30.0)
_STEERING_STIFFNESS = 0.01
_STEERING_DAMPING = 0.005
_WHEEL_DAMPING = 0.005


def build() -> SceneSetup:
    world = dart.io.SkelParser.read_world(_WORLD_URI)
    if world is None:
        raise RuntimeError(f"Failed to load vehicle world from {_WORLD_URI}")
    reorient_to_z_up(world)

    car = world.get_skeleton(_CAR_NAME)
    if car is None:
        raise RuntimeError("vehicle world is missing car_skeleton")

    state = {"steering_angle": 0.0, "wheel_velocity": -_WHEEL_SPEED}

    def pre_step() -> None:
        if car.get_num_dofs() < 12:
            return
        positions = car.get_positions()
        velocities = car.get_velocities()
        forces = np.zeros(car.get_num_dofs())

        target_steer = _MAX_STEER * math.sin(0.5 * world.get_time())
        state["steering_angle"] = target_steer
        forces[6] = (
            -_STEERING_STIFFNESS * (positions[6] - target_steer)
            - _STEERING_DAMPING * velocities[6]
        )
        forces[8] = (
            -_STEERING_STIFFNESS * (positions[8] - target_steer)
            - _STEERING_DAMPING * velocities[8]
        )
        forces[7] = -_WHEEL_DAMPING * (velocities[7] - state["wheel_velocity"])
        forces[9] = -_WHEEL_DAMPING * (velocities[9] - state["wheel_velocity"])
        forces[10] = -_WHEEL_DAMPING * (velocities[10] - state["wheel_velocity"])
        forces[11] = -_WHEEL_DAMPING * (velocities[11] - state["wheel_velocity"])

        car.set_forces(forces)

    return SceneSetup(world=world, pre_step=pre_step, info={})


SCENE = PythonDemoScene(
    id="vehicle",
    title="Vehicle",
    category="Control & IK",
    summary="Four-wheel car under PD steering and wheel-velocity control.",
    build=build,
)
