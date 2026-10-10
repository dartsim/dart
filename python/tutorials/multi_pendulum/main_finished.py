"""Multi-pendulum tutorial: forces, springs, constraints, and replay."""

import math

import dartpy as dart
import numpy as np

default_height = 1.0  # m
default_width = 0.2  # m
default_depth = 0.2  # m
default_torque = 15.0  # N-m
default_force = 15.0  # N
default_countdown = 200
default_playback_frame_step = 16
default_rest_position = 0.0
delta_rest_position = math.radians(10.0)
default_stiffness = 0.0
delta_stiffness = 10.0
default_damping = 5.0
delta_damping = 1.0


def set_geometry(body):
    box = dart.dynamics.BoxShape([default_width, default_depth, default_height])
    shape_node = body.createShapeNode(box)
    shape_node.createVisualAspect().setColor([0, 0, 1])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    transform = dart.math.Isometry3()
    center = [0, 0, default_height / 2.0]
    transform.set_translation(center)
    shape_node.setRelativeTransform(transform)
    body.setLocalCOM(center)


def make_root_body(pendulum, name):
    properties = dart.dynamics.BallJointProperties()
    properties.mName = name + "_joint"
    properties.mRestPositions = np.full(3, default_rest_position)
    properties.mSpringStiffnesses = np.full(3, default_stiffness)
    properties.mDampingCoefficients = np.full(3, default_damping)
    body_properties = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(name)
    )
    _, body = pendulum.createBallJointAndBodyNodePair(None, properties, body_properties)
    ball = dart.dynamics.EllipsoidShape(math.sqrt(2) * np.full(3, default_width))
    body.createShapeNode(ball).createVisualAspect().setColor([0, 0, 1])
    set_geometry(body)
    return body


def add_body(pendulum, parent, name):
    properties = dart.dynamics.RevoluteJointProperties()
    properties.mName = name + "_joint"
    properties.mAxis = [0, 1, 0]
    parent_transform = dart.math.Isometry3()
    parent_transform.set_translation([0, 0, default_height])
    properties.mT_ParentBodyToJoint = parent_transform
    body_properties = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(name)
    )
    joint, body = pendulum.createRevoluteJointAndBodyNodePair(
        parent, properties, body_properties
    )
    joint.setRestPosition(0, default_rest_position)
    joint.setSpringStiffness(0, default_stiffness)
    joint.setDampingCoefficient(0, default_damping)
    cylinder = dart.dynamics.CylinderShape(default_width / 2.0, default_depth)
    transform = dart.math.Isometry3()
    transform.set_rotation(dart.math.AngleAxis(math.pi / 2.0, [1, 0, 0]).rotation())
    shape_node = body.createShapeNode(cylinder)
    shape_node.createVisualAspect().setColor([0, 0, 1])
    shape_node.setRelativeTransform(transform)
    set_geometry(body)
    return body


class Controller:
    def __init__(self, pendulum, world):
        self.pendulum = pendulum
        self.world = world
        self.ball_constraint = None
        self.positive_sign = True
        self.body_force = False
        self.force_countdown = np.zeros(pendulum.getNumDofs(), dtype=int)
        self.dof_bodies = []
        properties = dart.dynamics.ArrowShapeProperties()
        properties.mRadius = 0.05
        self.arrow = dart.dynamics.ArrowShape(
            [-default_height, 0, default_height / 2.0],
            [-default_width / 2.0, 0, default_height / 2.0],
            properties,
            [1.0, 0.5, 0.0, 1.0],
        )
        self.arrow_visuals = []
        for i in range(pendulum.getNumBodyNodes()):
            body = pendulum.getBodyNode(i)
            self.dof_bodies.extend([body] * body.getParentJoint().getNumDofs())
            visual = body.createShapeNode(self.arrow).createVisualAspect()
            visual.setColor([1.0, 0.5, 0.0])
            visual.hide()
            self.arrow_visuals.append(visual)

    def change_direction(self):
        self.positive_sign = not self.positive_sign
        sign = 1 if self.positive_sign else -1
        self.arrow.setPositions(
            [-sign * default_height, 0, default_height / 2.0],
            [-sign * default_width / 2.0, 0, default_height / 2.0],
        )

    def apply_force(self, index):
        if 0 <= index < len(self.force_countdown):
            self.force_countdown[index] = default_countdown

    def change_rest_position(self, delta):
        # Lesson 2a: keep the spring rest positions within the stable range.
        for dof in self.pendulum.getDofs():
            dof.setRestPosition(
                np.clip(dof.getRestPosition() + delta, -math.pi / 2, math.pi / 2)
            )
        self.pendulum.getDof(0).setRestPosition(0.0)
        self.pendulum.getDof(2).setRestPosition(0.0)

    def change_stiffness(self, delta):
        # Lesson 2b
        for dof in self.pendulum.getDofs():
            dof.setSpringStiffness(max(0.0, dof.getSpringStiffness() + delta))

    def change_damping(self, delta):
        # Lesson 2c
        for dof in self.pendulum.getDofs():
            dof.setDampingCoefficient(max(0.0, dof.getDampingCoefficient() + delta))

    def add_constraint(self):
        # Lesson 3: attach the final link's tip at its current world position.
        if self.ball_constraint is None:
            tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
            location = tip.getTransform().multiply([0, 0, default_height])
            self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
            self.world.getConstraintSolver().addConstraint(self.ball_constraint)

    def remove_constraint(self):
        # Lesson 3
        if self.ball_constraint is not None:
            self.world.getConstraintSolver().removeConstraint(self.ball_constraint)
            self.ball_constraint = None

    def has_constraint(self):
        return self.ball_constraint is not None

    def toggle_body_force(self):
        self.body_force = not self.body_force

    def update(self):
        # Lesson 1a: reset colors and reuse the visual-only force arrows.
        for i in range(self.pendulum.getNumBodyNodes()):
            body = self.pendulum.getBodyNode(i)
            body.getShapeNode(0).getVisualAspect().setColor([0, 0, 1])
            body.getShapeNode(1).getVisualAspect().setColor([0, 0, 1])
            self.arrow_visuals[i].hide()

        sign = 1 if self.positive_sign else -1
        if not self.body_force:
            for i in range(self.pendulum.getNumDofs()):
                if self.force_countdown[i] > 0:
                    # Lesson 1b: apply a joint torque.
                    self.pendulum.getDof(i).setForce(sign * default_torque)
                    self.dof_bodies[i].getShapeNode(0).getVisualAspect().setColor(
                        [1, 0, 0]
                    )
                    self.force_countdown[i] -= 1
        else:
            for i in range(self.pendulum.getNumBodyNodes()):
                if self.force_countdown[i] > 0:
                    # Lesson 1c: apply force and its offset in the body frame.
                    body = self.pendulum.getBodyNode(i)
                    body.addExtForce(
                        [sign * default_force, 0, 0],
                        [-sign * default_width / 2.0, 0, default_height / 2.0],
                        True,
                        True,
                    )
                    body.getShapeNode(1).getVisualAspect().setColor([1, 0, 0])
                    self.arrow_visuals[i].show()
                    self.force_countdown[i] -= 1


class PendulumEventHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, world, controller):
        super().__init__()
        self.world = world
        self.controller = controller
        self.viewer = None
        self.playing_back = False
        self.play_frame = 0

    def handle(self, event, action):
        if event.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False
        key = event.getKey()
        if ord("0") <= key <= ord("9"):
            self.controller.apply_force((key - ord("1")) % 10)
        elif key == ord("-"):
            self.controller.change_direction()
        elif key == ord("q"):
            self.controller.change_rest_position(delta_rest_position)
        elif key == ord("a"):
            self.controller.change_rest_position(-delta_rest_position)
        elif key == ord("w"):
            self.controller.change_stiffness(delta_stiffness)
        elif key == ord("s"):
            self.controller.change_stiffness(-delta_stiffness)
        elif key == ord("e"):
            self.controller.change_damping(delta_damping)
        elif key == ord("d"):
            self.controller.change_damping(-delta_damping)
        elif key == ord("r"):
            if self.controller.has_constraint():
                self.controller.remove_constraint()
            else:
                self.controller.add_constraint()
        elif key == ord("f"):
            self.controller.toggle_body_force()
        elif key == ord("p"):
            self.toggle_playback()
        elif key == ord(" "):
            self.stop_playback()
            return False
        else:
            return False
        return True

    def toggle_playback(self):
        recording = self.world.getRecording()
        if recording.getNumFrames() == 0:
            print("No recorded frames are available for replay.")
            return
        self.playing_back = not self.playing_back
        if self.playing_back and self.viewer is not None:
            self.viewer.simulate(False)
        if self.play_frame >= recording.getNumFrames():
            self.play_frame = 0

    def stop_playback(self):
        self.playing_back = False

    def show_playback_frame(self):
        if not self.playing_back:
            return
        recording = self.world.getRecording()
        if (
            recording.getNumFrames() == 0
            or recording.getNumSkeletons() != self.world.getNumSkeletons()
        ):
            self.stop_playback()
            return
        if self.play_frame >= recording.getNumFrames():
            self.play_frame = 0
        for i in range(self.world.getNumSkeletons()):
            skeleton = self.world.getSkeleton(i)
            if recording.getNumDofs(i) != skeleton.getNumDofs():
                self.stop_playback()
                return
            skeleton.setPositions(recording.getConfig(self.play_frame, i))
        self.play_frame += default_playback_frame_step


class CustomWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, controller, handler):
        super().__init__(world)
        self.world = world
        self.controller = controller
        self.handler = handler

    def customPreRefresh(self):
        self.handler.show_playback_frame()

    def customPreStep(self):
        self.controller.update()

    def customPostStep(self):
        self.world.bake()


def build_scene():
    pendulum = dart.dynamics.Skeleton("pendulum")
    body = make_root_body(pendulum, "body1")
    for i in range(2, 6):
        body = add_body(pendulum, body, f"body{i}")
    pendulum.setPosition(1, math.radians(120.0))
    world = dart.simulation.World()
    world.addSkeleton(pendulum)
    controller = Controller(pendulum, world)
    handler = PendulumEventHandler(world, controller)
    return CustomWorldNode(world, controller, handler)


def main():
    node = build_scene()
    viewer = dart.gui.osg.Viewer()
    node.handler.viewer = viewer
    viewer.addWorldNode(node)
    viewer.addEventHandler(node.handler)
    viewer.addInstructionText(
        "Space: simulation on/off; p: replay\n"
        "1-9, 0: joint torque or body force; -: reverse force\n"
        "q/a: rest position; w/s: stiffness; e/d: damping\n"
        "r: tip constraint; f: joint/body force mode\n"
    )
    print(viewer.getInstructions())
    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([5, 3, 3], [0, 0, -2], [0, 0, 1])
    viewer.run()


if __name__ == "__main__":
    main()
