"""Robots > vehicle: the Y-up car's steering and wheel-velocity servos."""

import math

import dartpy as dart
import numpy as np


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/vehicle.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/vehicle.skel")
    world.setGravity([0.0, -9.81, 0.0])
    vehicle = world.getSkeleton("car_skeleton")
    if vehicle is None:
        raise RuntimeError("vehicle.skel: missing skeleton 'car_skeleton'")
    if vehicle.getNumDofs() < 12:
        raise RuntimeError("vehicle.skel: expected >= 12 dofs (car_skeleton layout)")

    back_wheel_velocity = 0.0
    steering_wheel_angle = 0.0
    deg_to_rad = math.pi / 180.0

    def pre_step():
        q = vehicle.getPositions()
        dq = vehicle.getVelocities()
        tau = np.zeros(vehicle.getNumDofs())
        for index in (6, 8):
            tau[index] = -0.01 * (q[index] - steering_wheel_angle) - 0.005 * dq[index]
        for index in (7, 9, 10, 11):
            tau[index] = -0.005 * (dq[index] - back_wheel_velocity)
        vehicle.setForces(tau)

    def drive(velocity):
        nonlocal back_wheel_velocity
        back_wheel_velocity = velocity

    def steer(increment):
        nonlocal steering_wheel_angle
        steering_wheel_angle = max(
            -30.0 * deg_to_rad,
            min(30.0 * deg_to_rad, steering_wheel_angle + increment),
        )

    panel = dart.viewer.Panel()
    steering_text = panel.add_text("Steering angle: 0.0 deg")
    velocity_text = panel.add_text("Back wheel target velocity: 0.0 deg/s")
    panel.add_text(
        "w/s/x drive the back wheels forward/stop/backward; a/d steer "
        "left/right in 10-degree steps (clamped to +-30 deg). This scene "
        "keeps vehicle.skel's original Y-up orientation (see file "
        "comment) rather than the host's usual Z-up convention.",
        wrap=True,
    )

    def pre_refresh():
        steering_text.label = (
            f"Steering angle: {steering_wheel_angle / deg_to_rad:.1f} deg"
        )
        velocity_text.label = (
            f"Back wheel target velocity: {back_wheel_velocity / deg_to_rad:.1f} deg/s"
        )

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction(
                "w", "Move forward", lambda: drive(-420.0 * deg_to_rad)
            ),
            dart.viewer.KeyAction("s", "Stop", lambda: drive(0.0)),
            dart.viewer.KeyAction(
                "x", "Move backward", lambda: drive(420.0 * deg_to_rad)
            ),
            dart.viewer.KeyAction("a", "Steer left", lambda: steer(10.0 * deg_to_rad)),
            dart.viewer.KeyAction(
                "d", "Steer right", lambda: steer(-10.0 * deg_to_rad)
            ),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[5.0, 3.0, 3.0], center=[0.0, 0.0, 0.0], up=[0.0, 1.0, 0.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "vehicle",
            title="Vehicle",
            category="Robots",
            summary="Drive a steerable car past obstacles.",
            factory=build,
        )
    ]
