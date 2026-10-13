"""A hand-built revolute leg with keyboard pose controls."""

import dartpy as dart
import numpy as np


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, 0.0])
    skeleton = dart.dynamics.Skeleton()
    parent = None
    for name, axis, translation in (
        ("LHY", [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
        ("LHR", [1.0, 0.0, 0.0], [0.0, 0.0, 0.5]),
        ("LHP", [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
    ):
        joint_properties = dart.dynamics.RevoluteJointProperties()
        joint_properties.mName = name
        joint_properties.mAxis = axis
        joint_properties.mPositionLowerLimits = [-np.pi]
        joint_properties.mPositionUpperLimits = [np.pi]
        transform = dart.math.Isometry3()
        transform.set_translation(translation)
        joint_properties.mT_ParentBodyToJoint = transform
        _, body = skeleton.createRevoluteJointAndBodyNodePair(parent, joint_properties)
        body.setName(name)
        shape_node = body.createShapeNode(dart.dynamics.BoxShape([0.3, 0.3, 1.0]))
        shape_node.createVisualAspect()
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect()
        if parent is not None:
            shape_node.setRelativeTranslation([0.0, 0.0, 0.5])
            body.setLocalCOM(shape_node.getRelativeTranslation())
        body.setMass(1.0)
        parent = body
    world.addSkeleton(skeleton)

    inverse = False
    step = 0.1
    panel = dart.viewer.Panel()
    direction_text = panel.add_text("Direction: positive")
    step_size = panel.add_slider("Step size", step, min=0.01, max=0.5, decimals=3)
    pose_text = panel.add_text("Pose: [0.00, 0.00, 0.00]")
    panel.add_text(
        "This is a purely kinematic pose demo (gravity disabled): keys "
        "1/2/3 step the LHY/LHR/LHP joints, '-' flips the direction.",
        wrap=True,
    )

    def set_step():
        nonlocal step
        step = float(np.float32(step_size.value))

    step_size.on_change = set_step

    def invert_direction():
        nonlocal inverse
        inverse = not inverse

    def step_dof(index):
        pose = skeleton.getPositions().copy()
        pose[index] += -step if inverse else step
        skeleton.setPositions(pose)

    def pre_refresh():
        direction_text.label = f"Direction: {'negative' if inverse else 'positive'}"
        pose = skeleton.getPositions()
        pose_text.label = f"Pose: [{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}]"

    def on_activate(ctx):
        ctx.viewer.setWireframe(True)
        ctx.add_teardown(lambda: ctx.viewer.setWireframe(False))

    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        on_activate=on_activate,
        key_actions=[
            dart.viewer.KeyAction("-", "Invert direction", invert_direction),
            dart.viewer.KeyAction("1", "Move LHY", lambda: step_dof(0)),
            dart.viewer.KeyAction("2", "Move LHR", lambda: step_dof(1)),
            dart.viewer.KeyAction("3", "Move LHP", lambda: step_dof(2)),
        ],
        panel=panel,
        shadows=False,
        camera_home=dart.viewer.CameraHome(
            eye=[6.5, 6.5, 3.0], center=[0.0, 0.0, 1.0], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "hardcoded_design",
            title="Hardcoded Design",
            category="Constraints & Joints",
            summary="A hand-built revolute chain driven from the keyboard.",
            factory=build,
        )
    ]
