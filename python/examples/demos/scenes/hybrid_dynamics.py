"""Control & IK > hybrid_dynamics: sinusoidal gait and a pelvis harness."""

import math

import dartpy as dart

from .common import reorient_z_up


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/fullbody1.skel")
    reorient_z_up(world)
    if world.getNumSkeletons() < 2:
        raise RuntimeError("fullbody1.skel: expected biped at index 1")
    skel = world.getSkeleton(1)
    if skel.getNumDofs() <= 21:
        raise RuntimeError("fullbody1.skel: expected >= 22 DOFs for the initial pose")
    skel.setPositions(
        [1, 6, 9, 10, 13, 16, 17, 21],
        [-0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0],
    )

    actuator = dart.dynamics.ActuatorType
    joint0 = skel.getJoint(0)
    if joint0 is not None:
        joint0.setActuatorType(actuator.PASSIVE)
    for index in range(1, skel.getNumBodyNodes()):
        joint = skel.getJoint(index)
        if joint is not None:
            joint.setActuatorType(actuator.VELOCITY)

    harness_on = False

    def toggle_harness():
        nonlocal harness_on
        pelvis = skel.getBodyNode("h_pelvis")
        if pelvis is None:
            return
        joint = pelvis.getParentJoint()
        if joint is None:
            return
        harness_on = not harness_on
        joint.setActuatorType(actuator.LOCKED if harness_on else actuator.PASSIVE)

    def pre_step():
        for name, amplitude, frequency in (
            ("j_scapula_left", 1.0, 4.0),
            ("j_scapula_right", -1.0, 4.0),
            ("j_forearm_left", 0.8, 4.0),
            ("j_forearm_right", 0.8, 4.0),
            ("j_shin_left", 0.1, 2.0),
            ("j_shin_right", 0.1, 2.0),
        ):
            joint = skel.getJoint(name)
            if joint is not None:
                skel.setCommand(
                    joint.getIndexInSkeleton(0),
                    amplitude * math.sin(world.getTime() * frequency),
                )

    panel = dart.viewer.Panel()
    harness_text = panel.add_text("Harness: unlocked")
    panel.add_text(
        "Velocity-actuated sinusoidal gait (code-constant amplitudes/"
        "frequencies, no GUI). 'h' locks/unlocks the pelvis joint.",
        wrap=True,
    )

    def pre_refresh():
        harness_text.label = f"Harness: {'locked' if harness_on else 'unlocked'}"

    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        key_actions=[dart.viewer.KeyAction("h", "Toggle harness", toggle_harness)],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[5.0, -3.0, 3.0], center=[0.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "hybrid_dynamics",
            title="Hybrid Dynamics",
            category="Control & IK",
            summary="Velocity-driven sinusoidal gait with a lockable pelvis harness.",
            factory=build,
        )
    ]
