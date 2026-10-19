"""Full-body SPD balance with timed pushes and a pelvis harness."""

from __future__ import annotations

import dartpy as dart
import numpy as np

from ..models.joint_constraints import JointConstraintsController
from .common import reorient_z_up


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/fullbody1.skel")
    reorient_z_up(world)
    if world.getNumSkeletons() < 2:
        raise RuntimeError("fullbody1.skel: expected biped at index 1")
    biped = world.getSkeleton(1)
    if biped.getNumDofs() < 22:
        raise RuntimeError("fullbody1.skel: expected >= 22 DOFs for the initial pose")
    biped.setPositions(
        [1, 4, 6, 9, 10, 13, 16, 17, 21],
        [-0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1],
    )
    controller = JointConstraintsController(biped, world.getTimeStep())
    perturbation_force = np.zeros(3)
    impulse_duration = 0
    harness_on = False
    weld_joint = None

    panel = dart.viewer.Panel()
    harness = panel.add_text("Harness: off")
    perturbation = panel.add_text("Perturbation active: 0 steps left")
    panel.add_text(
        "1/2 push the spine forward/backward, 3/4 push right/left "
        "(40-50 N for 100 steps); 'h' toggles a pelvis weld-joint harness.",
        wrap=True,
    )

    def push(axis, value):
        nonlocal impulse_duration
        perturbation_force[axis] = value
        impulse_duration = 100

    def toggle_harness():
        nonlocal harness_on, weld_joint
        harness_on = not harness_on
        if harness_on:
            pelvis = biped.getBodyNode("h_pelvis")
            if pelvis is None:
                return
            weld_joint = dart.constraint.WeldJointConstraint(pelvis)
            world.getConstraintSolver().addConstraint(weld_joint)
        elif weld_joint is not None:
            world.getConstraintSolver().removeConstraint(weld_joint)
            weld_joint = None

    def pre_step():
        nonlocal impulse_duration
        controller.timestep = world.getTimeStep()
        spine = biped.getBodyNode("h_spine")
        if spine is not None:
            spine.addExtForce(perturbation_force)
        biped.setForces(controller.compute_torques(biped))
        if impulse_duration > 0:
            impulse_duration -= 1
            if impulse_duration <= 0:
                perturbation_force.fill(0)

    def pre_refresh():
        harness.label = f"Harness: {'on' if harness_on else 'off'}"
        perturbation.label = f"Perturbation active: {impulse_duration} steps left"

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction("1", "Push forward", lambda: push(0, 40)),
            dart.viewer.KeyAction("2", "Push backward", lambda: push(0, -40)),
            dart.viewer.KeyAction("3", "Push right", lambda: push(1, -50)),
            dart.viewer.KeyAction("4", "Push left", lambda: push(1, 50)),
            dart.viewer.KeyAction("h", "Toggle harness", toggle_harness),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[5, -3, 3], center=[0, 0, 0], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "joint_constraints",
            title="Joint Constraints",
            category="Control & IK",
            summary="Balanced full-body PD control with a harness constraint.",
            factory=build,
        )
    ]
