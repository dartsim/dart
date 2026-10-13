"""The native KR5 task-space controller with a draggable red-ball target."""

from __future__ import annotations

import dartpy as dart
import numpy as np
from dartpy import _demos


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    loader = dart.utils.DartLoader()
    robot = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    if robot is None:
        raise RuntimeError("failed to load dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    world.addSkeleton(robot)
    robot.getJoint(0).setTransformFromParentBodyNode(dart.math.Isometry3())

    ground = loader.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    if ground is None:
        raise RuntimeError("failed to load dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(ground)
    ground_tf = ground.getJoint(0).getTransformFromParentBodyNode()
    ground_tf.pretranslate([0, 0, 0.5])
    ground_tf.set_rotation(
        ground_tf.rotation() @ dart.math.AngleAxis(np.pi / 2, [1, 0, 0]).rotation()
    )
    ground.getJoint(0).setTransformFromParentBodyNode(ground_tf)

    for index in range(robot.getNumJoints()):
        joint = robot.getJoint(index)
        joint.setLimitEnforcement(False)
        joint.setDampingCoefficient(0, 0.5)

    ee = robot.getBodyNode(robot.getNumBodyNodes() - 1)
    offset = np.array([0.05, 0, 0])
    tf = ee.getWorldTransform()
    tf.pretranslate(offset)
    target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target", tf)
    target.setShape(dart.dynamics.SphereShape(0.025))
    target.getVisualAspect(True).setColor([0.9, 0, 0])
    world.addSimpleFrame(target)
    controller = _demos.OperationalSpaceController(robot, target)
    controller.offset = ee.getWorldTransform().rotation().transpose() @ offset

    constrained = [False, False, False]
    dnd = None
    world_node = None
    panel = dart.viewer.Panel()
    drag_constraint = panel.add_text("Drag constraint: (none)")
    panel.add_text(
        "Drag the red ball to move the operational-space controller's "
        "target. Hold 1/2/3 (alone or combined) to constrain the drag to "
        "an axis or plane. The original's fixed 10-steps-per-render-cycle "
        "pacing is superseded by the host's Target RTF slider.",
        wrap=True,
    )

    def pre_refresh():
        axes = "".join(axis + " " for axis, held in zip("xyz", constrained) if held)
        drag_constraint.label = "Drag constraint: " + (axes or "(none)")

    def handle_event(event):
        key = event.getKey()
        if key not in (ord("1"), ord("2"), ord("3")):
            return False
        if _demos.keyboard_capture_active():
            return False
        if event.getEventType() == dart.gui.GUIEventAdapter.KEYDOWN:
            constrained[key - ord("1")] = True
        elif event.getEventType() == dart.gui.GUIEventAdapter.KEYUP:
            constrained[key - ord("1")] = False
        else:
            return False
        if dnd is not None:
            count = sum(constrained)
            if count in (0, 3):
                dnd.unconstrain()
            elif count == 1:
                dnd.constrainToLine([float(held) for held in constrained])
            else:
                dnd.constrainToPlane([float(not held) for held in constrained])
        return True

    def on_activate(ctx):
        nonlocal dnd, world_node
        world_node = ctx.world_node
        dnd = ctx.enable_drag_and_drop(target)
        dnd.setObstructable(False)
        ctx.add_event_handler(handle_event)

        def teardown():
            nonlocal dnd, world_node
            dnd = None
            world_node = None

        ctx.add_teardown(teardown)

    def toggle_shadows():
        if world_node is not None:
            world_node.setShadowsEnabled(not world_node.isShadowed())

    return dart.viewer.SceneSetup(
        world,
        pre_step=controller.update,
        pre_refresh=pre_refresh,
        on_activate=on_activate,
        key_actions=[dart.viewer.KeyAction("s", "Toggle shadows", toggle_shadows)],
        camera_home=dart.viewer.CameraHome(
            eye=[2.57, 3.14, 1.64], center=[0, 0, 0], up=[-0.24, -0.25, 0.94]
        ),
        shadows=False,
        panel=panel,
    )


def scenes():
    return [
        dart.viewer.Scene(
            "operational_space_control",
            title="Operational Space Control",
            category="Control & IK",
            summary="KR5 arm chasing a draggable target with task-space control.",
            factory=build,
        )
    ]
