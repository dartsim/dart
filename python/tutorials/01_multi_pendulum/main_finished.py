#
# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

"""Interactive multi-pendulum tutorial implemented with dartpy."""

import math
from typing import List, Optional, Tuple

import dartpy as dart
import numpy as np

default_height = 1.0  # m
default_width = 0.2  # m
default_depth = 0.2  # m

default_torque = 15.0  # N-m
default_force = 15.0  # N
default_countdown = 200  # Number of timesteps for applying force

default_rest_position = 0.0
delta_rest_position = math.radians(10.0)

default_stiffness = 0.0
delta_stiffness = 10

default_damping = 5.0
delta_damping = 1.0


def _to_vector(values: Tuple[float, float, float]) -> np.ndarray:
    """Return a 3 × 1 numpy vector for dartpy API calls."""
    vec = np.zeros(3)
    vec[:] = values
    return vec


class Controller:
    """Owns the runtime state and interactions for the tutorial."""

    def __init__(
        self, pendulum: dart.dynamics.Skeleton, world: dart.simulation.World
    ):
        self.pendulum = pendulum
        self.world = world
        self.ball_constraint: Optional[dart.constraint.BallJointConstraint] = None
        self.positive_sign = True
        self.apply_body_force = False
        self.force_countdown: List[int] = [0] * self.pendulum.getNumDofs()
        self.body_force_visuals = self._create_force_visuals()

    def _create_force_visuals(self):
        """Create one hidden arrow per body so we can show applied forces."""
        visuals = []
        for idx in range(self.pendulum.getNumBodyNodes()):
            body = self.pendulum.getBodyNode(idx)
            frame = dart.dynamics.SimpleFrame(body, f"body_force_arrow_{idx}")
            frame.setRelativeTranslation(np.array([0.0, 0.0, default_height / 2.0]))
            arrow_props = dart.dynamics.ArrowShapeProperties(0.05)
            tail, head = self._arrow_positions()
            arrow = dart.dynamics.ArrowShape(tail, head, arrow_props)
            frame.setShape(arrow)
            visual = frame.createVisualAspect()
            visual.setColor([1.0, 0.5, 0.0, 0.9])
            visual.hide()
            self.world.addSimpleFrame(frame)
            visuals.append((arrow, visual))
        return visuals

    def _arrow_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.positive_sign:
            tail = _to_vector((-default_height, 0.0, default_height / 2.0))
            head = _to_vector((-default_width / 2.0, 0.0, default_height / 2.0))
        else:
            tail = _to_vector((default_height, 0.0, default_height / 2.0))
            head = _to_vector((default_width / 2.0, 0.0, default_height / 2.0))
        return tail, head

    def change_direction(self):
        self.positive_sign = not self.positive_sign
        tail, head = self._arrow_positions()
        for arrow, _ in self.body_force_visuals:
            arrow.setPositions(tail, head)

    def apply_force(self, index: int):
        if 0 <= index < len(self.force_countdown):
            self.force_countdown[index] = default_countdown

    def change_rest_position(self, delta: float):
        # snippet:py-lesson2a-rest-position-start
        for i in range(self.pendulum.getNumDofs()):
            dof = self.pendulum.getDof(i)
            q0 = dof.getRestPosition() + delta
            q0 = max(min(q0, math.radians(90.0)), math.radians(-90.0))
            dof.setRestPosition(q0)
        # Only curl along the middle axis of the BallJoint.
        self.pendulum.getDof(0).setRestPosition(0.0)
        self.pendulum.getDof(2).setRestPosition(0.0)
        # snippet:py-lesson2a-rest-position-end

    def change_stiffness(self, delta: float):
        # snippet:py-lesson2b-stiffness-start
        for i in range(self.pendulum.getNumDofs()):
            dof = self.pendulum.getDof(i)
            stiffness = max(0.0, dof.getSpringStiffness() + delta)
            dof.setSpringStiffness(stiffness)
        # snippet:py-lesson2b-stiffness-end

    def change_damping(self, delta: float):
        # snippet:py-lesson2c-damping-start
        for i in range(self.pendulum.getNumDofs()):
            dof = self.pendulum.getDof(i)
            damping = max(0.0, dof.getDampingCoefficient() + delta)
            dof.setDampingCoefficient(damping)
        # snippet:py-lesson2c-damping-end

    def add_constraint(self):
        # snippet:py-lesson3-add-constraint-start
        tip = self.pendulum.getBodyNode(self.pendulum.getNumBodyNodes() - 1)
        location = tip.getTransform().multiply([0.0, 0.0, default_height])
        self.ball_constraint = dart.constraint.BallJointConstraint(tip, location)
        self.world.getConstraintSolver().addConstraint(self.ball_constraint)
        # snippet:py-lesson3-add-constraint-end

    def remove_constraint(self):
        if self.ball_constraint is None:
            return
        # snippet:py-lesson3-remove-constraint-start
        self.world.getConstraintSolver().removeConstraint(self.ball_constraint)
        self.ball_constraint = None
        # snippet:py-lesson3-remove-constraint-end

    def has_constraint(self) -> bool:
        return self.ball_constraint is not None

    def toggle_body_force(self):
        self.apply_body_force = not self.apply_body_force

    def update(self):
        """Advance one controller iteration."""
        self._reset_visuals()
        if self.apply_body_force:
            self._apply_body_forces()
        else:
            self._apply_joint_torques()

    def _reset_visuals(self):
        """Lesson 1a: restore blue bodies and hide arrows."""
        # snippet:py-lesson1a-reset-start
        for idx in range(self.pendulum.getNumBodyNodes()):
            body = self.pendulum.getBodyNode(idx)
            num_visual_nodes = body.getNumShapeNodes()
            for j in range(min(2, num_visual_nodes)):
                visual = body.getShapeNode(j).getVisualAspect()
                visual.setColor([0.0, 0.0, 1.0, 1.0])
            # snippet:py-lesson1a-remove-arrow-start
            _, arrow_visual = self.body_force_visuals[idx]
            arrow_visual.hide()
            # snippet:py-lesson1a-remove-arrow-end
        # snippet:py-lesson1a-reset-end

    def _apply_joint_torques(self):
        """Lesson 1b: apply impulsive joint torques and highlight joints."""
        # snippet:py-lesson1b-joint-force-start
        torque = default_torque if self.positive_sign else -default_torque
        for i in range(self.pendulum.getNumDofs()):
            if self.force_countdown[i] <= 0:
                continue

            dof = self.pendulum.getDof(i)
            dof.setForce(torque)

            child = dof.getChildBodyNode()
            if child.getNumShapeNodes() > 0:
                joint_visual = child.getShapeNode(0).getVisualAspect()
                joint_visual.setColor([1.0, 0.0, 0.0, 1.0])

            self.force_countdown[i] -= 1
        # snippet:py-lesson1b-joint-force-end

    def _apply_body_forces(self):
        """Lesson 1c: apply body forces and show the arrow visuals."""
        # snippet:py-lesson1c-body-force-start
        num_slots = min(
            self.pendulum.getNumBodyNodes(), len(self.force_countdown)
        )
        tail, head = self._arrow_positions()
        for i in range(num_slots):
            if self.force_countdown[i] <= 0:
                continue

            body = self.pendulum.getBodyNode(i)
            force = np.array([default_force, 0.0, 0.0])
            location = np.array(
                [-default_width / 2.0, 0.0, default_height / 2.0]
            )
            if not self.positive_sign:
                force *= -1.0
                location[0] *= -1.0
            body.addExtForce(force, location, True, True)

            if body.getNumShapeNodes() > 1:
                body_shape = body.getShapeNode(1).getVisualAspect()
                body_shape.setColor([1.0, 0.0, 0.0, 1.0])

            arrow, arrow_visual = self.body_force_visuals[i]
            arrow.setPositions(tail, head)
            arrow_visual.show()

            self.force_countdown[i] -= 1
        # snippet:py-lesson1c-body-force-end


class PendulumEventHandler(dart.gui.GUIEventHandler):
    """Map keyboard input to controller actions."""

    def __init__(self, controller: Controller):
        super().__init__()
        self.controller = controller

    def handle(self, ea, _aa):
        if ea.getEventType() != dart.gui.GUIEventAdapter.KEYDOWN:
            return False

        key = ea.getKey()
        gea = dart.gui.GUIEventAdapter

        digit_keys = {
            gea.KEY_1: 0,
            gea.KEY_2: 1,
            gea.KEY_3: 2,
            gea.KEY_4: 3,
            gea.KEY_5: 4,
            gea.KEY_6: 5,
            gea.KEY_7: 6,
            gea.KEY_8: 7,
            gea.KEY_9: 8,
            gea.KEY_0: 9,
        }
        if key in digit_keys:
            self.controller.apply_force(digit_keys[key])
            return True

        if key == gea.KEY_Minus:
            self.controller.change_direction()
            return True
        if key == gea.KEY_Q:
            self.controller.change_rest_position(delta_rest_position)
            return True
        if key == gea.KEY_A:
            self.controller.change_rest_position(-delta_rest_position)
            return True
        if key == gea.KEY_W:
            self.controller.change_stiffness(delta_stiffness)
            return True
        if key == gea.KEY_S:
            self.controller.change_stiffness(-delta_stiffness)
            return True
        if key == gea.KEY_E:
            self.controller.change_damping(delta_damping)
            return True
        if key == gea.KEY_D:
            self.controller.change_damping(-delta_damping)
            return True
        if key == gea.KEY_R:
            if self.controller.has_constraint():
                self.controller.remove_constraint()
            else:
                self.controller.add_constraint()
            return True
        if key == gea.KEY_F:
            self.controller.toggle_body_force()
            return True
        return False


class CustomWorldNode(dart.gui.RealTimeWorldNode):
    """Call the controller before each physics step."""

    def __init__(
        self, world: dart.simulation.World, controller: Controller
    ):
        super().__init__(world)
        self.controller = controller

    def customPreStep(self):
        self.controller.update()


def set_geometry(body: dart.dynamics.BodyNode):
    """Attach the blue box geometry plus collision/dynamics aspects."""
    box = dart.dynamics.BoxShape([default_width, default_depth, default_height])
    shape_node = body.createShapeNode(box)
    visual = shape_node.createVisualAspect()
    visual.setColor([0.0, 0.0, 1.0, 1.0])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()

    box_tf = dart.math.Isometry3()
    center = np.array([0.0, 0.0, default_height / 2.0])
    box_tf.set_translation(center)
    shape_node.setRelativeTransform(box_tf)
    body.setLocalCOM(center)


def make_root_body(pendulum: dart.dynamics.Skeleton, name: str):
    """Create and return the ball-jointed root body."""
    joint_prop = dart.dynamics.BallJointProperties()
    joint_prop.mName = f"{name}_joint"
    joint_prop.mRestPositions = np.ones(3) * default_rest_position
    joint_prop.mSpringStiffnesses = np.ones(3) * default_stiffness
    joint_prop.mDampingCoefficients = np.ones(3) * default_damping

    body_aspect_prop = dart.dynamics.BodyNodeAspectProperties(name)
    body_prop = dart.dynamics.BodyNodeProperties(body_aspect_prop)

    joint, body = pendulum.createBallJointAndBodyNodePair(
        None, joint_prop, body_prop
    )

    r = default_width
    ball = dart.dynamics.EllipsoidShape(math.sqrt(2.0) * np.ones(3) * r)
    shape_node = body.createShapeNode(ball)
    visual = shape_node.createVisualAspect()
    visual.setColor([0.0, 0.0, 1.0, 1.0])

    set_geometry(body)
    return body


def add_body(pendulum: dart.dynamics.Skeleton, parent, name: str):
    """Append a revolute-jointed body and return it."""
    joint_prop = dart.dynamics.RevoluteJointProperties()
    joint_prop.mName = f"{name}_joint"
    joint_prop.mAxis = [0.0, 1.0, 0.0]
    joint_prop.mT_ParentBodyToJoint.set_translation([0.0, 0.0, default_height])

    body_aspect_prop = dart.dynamics.BodyNodeAspectProperties(name)
    body_prop = dart.dynamics.BodyNodeProperties(body_aspect_prop)

    joint, body = pendulum.createRevoluteJointAndBodyNodePair(
        parent, joint_prop, body_prop
    )
    joint.setRestPosition(0, default_rest_position)
    joint.setSpringStiffness(0, default_stiffness)
    joint.setDampingCoefficient(0, default_damping)

    r = default_width / 2.0
    h = default_depth
    cylinder = dart.dynamics.CylinderShape(r, h)

    tf = dart.math.Isometry3()
    rot = dart.math.eulerXYZToMatrix([math.pi / 2.0, 0.0, 0.0])
    tf.set_rotation(rot)

    shape_node = body.createShapeNode(cylinder)
    visual = shape_node.createVisualAspect()
    visual.setColor([0.0, 0.0, 1.0, 1.0])
    shape_node.setRelativeTransform(tf)

    set_geometry(body)
    return body


def main():
    """Build the pendulum demo and start the viewer."""
    pendulum = dart.dynamics.Skeleton("pendulum")

    body = make_root_body(pendulum, "body1")
    body = add_body(pendulum, body, "body2")
    body = add_body(pendulum, body, "body3")
    body = add_body(pendulum, body, "body4")
    body = add_body(pendulum, body, "body5")

    pendulum.getDof(1).setPosition(math.radians(120.0))

    world = dart.simulation.World()
    world.addSkeleton(pendulum)

    controller = Controller(pendulum, world)
    handler = PendulumEventHandler(controller)

    node = CustomWorldNode(world, controller)

    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)
    viewer.addEventHandler(handler)

    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.XY)
    grid.setOffset([0.0, 0.0, -5.0])
    viewer.addAttachment(grid)

    viewer.addInstructionText("space bar: simulation on/off")
    viewer.addInstructionText("'p': replay simulation")
    viewer.addInstructionText("'1' -> '9': apply torque to a pendulum body")
    viewer.addInstructionText("'-': change sign of applied joint torques")
    viewer.addInstructionText("'q'/'a': change joint rest positions")
    viewer.addInstructionText("'w'/'s': change joint spring stiffness")
    viewer.addInstructionText("'e'/'d': change joint damping")
    viewer.addInstructionText("'r': add/remove constraint on the chain tip")
    viewer.addInstructionText(
        "'f': toggle between joint torques and body forces"
    )
    print(viewer.getInstructions())

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition(
        [5.0, 3.0, 3.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    )

    viewer.run()


if __name__ == "__main__":
    main()
