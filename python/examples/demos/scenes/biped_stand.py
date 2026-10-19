"""SPD-balanced standing biped with an ankle strategy and push tests."""

from __future__ import annotations

import dartpy as dart

from ..models.biped_stand import BipedStandController as StandController
from .common import reorient_z_up


class BipedStandController:
    """Apply standing torques using the current world timestep."""

    def __init__(self, world, skel):
        self.world = world
        self.skel = skel
        self.controller = StandController(skel, world.getTimeStep())

    def pre_step(self):
        self.controller.timestep = self.world.getTimeStep()
        self.skel.setForces(self.controller.compute_torques())


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/fullbody1.skel")
    reorient_z_up(world)
    biped = world.getSkeleton("fullbody1")
    if biped is None:
        raise RuntimeError("fullbody1.skel: missing skeleton 'fullbody1'")
    for name, position in (
        ("j_pelvis_rot_y", -0.20),
        ("j_thigh_left_z", 0.15),
        ("j_shin_left", -0.40),
        ("j_heel_left_1", 0.25),
        ("j_thigh_right_z", 0.15),
        ("j_shin_right", -0.40),
        ("j_heel_right_1", 0.25),
        ("j_abdomen_2", 0.00),
    ):
        dof = biped.getDof(name)
        if dof is None:
            raise RuntimeError(f"fullbody1.skel: missing DOF {name}")
        dof.setPosition(position)

    controller = BipedStandController(world, biped)
    force = [0, 0, 0]
    duration = 0

    def perturb(value):
        nonlocal force, duration
        force = value
        duration = 100

    def pre_step():
        nonlocal force, duration
        spine = biped.getBodyNode("h_spine")
        if spine is not None:
            spine.addExtForce(force)
        controller.pre_step()
        if duration > 0:
            duration -= 1
            if duration <= 0:
                force = [0, 0, 0]

    panel = dart.viewer.Panel()
    active = panel.add_text("Perturbation active: 0 steps left")
    panel.add_text(
        "SPD tracking (Kp=400, Kd=40) plus a sagittal-plane ankle "
        "strategy hold the standing pose. 1-4 push the spine "
        "forward/backward/right/left with 50 N for 100 steps.",
        wrap=True,
    )

    def pre_refresh():
        active.label = f"Perturbation active: {duration} steps left"

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction(
                "1", "Push forward (+x, 50 N)", lambda: perturb([50, 0, 0])
            ),
            dart.viewer.KeyAction(
                "2", "Push backward (-x, 50 N)", lambda: perturb([-50, 0, 0])
            ),
            dart.viewer.KeyAction(
                "3", "Push right (-y, 50 N)", lambda: perturb([0, -50, 0])
            ),
            dart.viewer.KeyAction(
                "4", "Push left (+y, 50 N)", lambda: perturb([0, 50, 0])
            ),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[3, -3, 1.5], center=[0, 0, 0], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "biped_stand",
            title="Biped Stand",
            category="Control & IK",
            summary="SPD-balanced standing biped with an ankle strategy and push tests.",
            factory=build,
        )
    ]
