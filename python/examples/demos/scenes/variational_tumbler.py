"""Variational-integrator scene: a torque-free floating body tumbles cleanly.

A single free (floating) body with asymmetric inertia is given an initial spin
in zero gravity, integrated with the **variational integrator**. The VI's
manifold-correct SE(3)/SO(3) retraction (Phase B1) keeps the tumbling motion
energy- and momentum-conserving, so the body spins indefinitely without the
spurious spin-down or drift a non-symplectic step accrues. This exercises the
floating-base VI path that the default semi-implicit scenes do not.

World owns the physics; WorldRenderBridge mirrors the body's world transform
onto a SimpleFrame box visual each frame.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup


def build() -> SceneSetup:
    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    # Torque-free: zero gravity so the only motion is the conserved tumble.
    world.gravity = (0.0, 0.0, 0.0)
    world.time_step = 0.005

    robot = world.add_multibody("tumbler")
    base = robot.add_link("base")
    body = robot.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 1.0
    # Asymmetric principal inertias (I_y is the intermediate axis) so the
    # tumble is visibly non-trivial while still conserving energy + momentum.
    body.inertia = ((0.08, 0.0, 0.0), (0.0, 0.16, 0.0), (0.0, 0.0, 0.10))

    joint = body.parent_joint
    # [linear; angular]: a gentle drift plus a spin dominated by the
    # intermediate axis with small perturbations about the others.
    joint.velocity = [0.15, 0.0, 0.0, 0.2, 4.0, 0.2]

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="variational_tumbler_render")
    bridge.add_link_visual(
        body,
        dart.BoxShape(np.array([0.55, 0.22, 0.38])),
        (0.16, 0.71, 0.78),
        name="tumbler_visual",
    )
    bridge.sync()

    angular_speed_history: deque[float] = deque(maxlen=120)
    drift_history: deque[float] = deque(maxlen=120)
    energy_drift_history: deque[float] = deque(maxlen=120)
    momentum_drift_history: deque[float] = deque(maxlen=120)
    inertia_diag = np.diag(np.asarray(body.inertia, dtype=float))
    initial_velocity = np.asarray(joint.velocity, dtype=float)
    initial_energy = 0.5 * body.mass * float(np.dot(initial_velocity[:3], initial_velocity[:3]))
    initial_energy += 0.5 * float(
        np.dot(inertia_diag * initial_velocity[3:], initial_velocity[3:])
    )
    initial_momentum = float(np.linalg.norm(inertia_diag * initial_velocity[3:]))

    def build_panel(builder: object, context: object) -> None:
        joint = body.parent_joint
        velocity = np.asarray(joint.velocity, dtype=float)
        angular_speed = float(np.linalg.norm(velocity[3:]))
        drift_speed = float(np.linalg.norm(velocity[:3]))
        energy = 0.5 * body.mass * float(np.dot(velocity[:3], velocity[:3]))
        energy += 0.5 * float(np.dot(inertia_diag * velocity[3:], velocity[3:]))
        momentum = float(np.linalg.norm(inertia_diag * velocity[3:]))
        energy_drift = energy - initial_energy
        momentum_drift = momentum - initial_momentum
        angular_speed_history.append(angular_speed)
        drift_history.append(drift_speed)
        energy_drift_history.append(energy_drift)
        momentum_drift_history.append(momentum_drift)

        builder.text("solver: variational integrator")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text("gravity: zero")
        builder.text(f"angular speed: {angular_speed:.3f} rad/s")
        builder.text(f"drift speed: {drift_speed:.3f} m/s")
        builder.text(f"energy drift: {energy_drift:+.3e} J")
        builder.text(f"angular momentum drift: {momentum_drift:+.3e}")
        changed, spin_y = builder.slider("Y spin", float(velocity[4]), -8.0, 8.0)
        if changed:
            next_velocity = velocity.copy()
            next_velocity[4] = float(spin_y)
            joint.velocity = next_velocity
        builder.plot_lines("Angular speed", list(angular_speed_history))
        builder.plot_lines("Drift speed", list(drift_history))
        builder.plot_lines("Energy drift", list(energy_drift_history))
        builder.plot_lines("Momentum drift", list(momentum_drift_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Variational Tumbler", build_panel)],
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="variational_tumbler",
    title="Variational Tumbler",
    category="Variational Integrators",
    summary="A torque-free floating body tumbles without energy/momentum drift.",
    build=build,
)
