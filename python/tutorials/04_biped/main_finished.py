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

import numpy as np

import dartpy as dart

DEFAULT_SPEED_INCREMENT = 0.5
DEFAULT_FORCE = 50.0
DEFAULT_COUNTDOWN = 100
DEFAULT_IK_ITERATIONS = 4500


class Controller:
    def __init__(self, biped: dart.dynamics.Skeleton):
        self.biped = biped
        self.speed = 0.0
        dofs = biped.getNumDofs()
        self.forces = np.zeros(dofs)
        self.Kp = np.eye(dofs)
        self.Kd = np.eye(dofs)

        # snippet:py-biped-lesson2-controller-gains-start
        for i in range(6):
            self.Kp[i, i] = 0.0
            self.Kd[i, i] = 0.0
        for i in range(6, dofs):
            self.Kp[i, i] = 1000.0
            self.Kd[i, i] = 50.0
        # snippet:py-biped-lesson2-controller-gains-end

        # snippet:py-biped-lesson2-controller-target-start
        self.set_target_positions(biped.getPositions())
        # snippet:py-biped-lesson2-controller-target-end

    def set_target_positions(self, pose: np.ndarray):
        # snippet:py-biped-lesson2-target-set-start
        self.target_positions = pose.copy()
        # snippet:py-biped-lesson2-target-set-end

    def clear_forces(self):
        self.forces[:] = 0.0

    def add_pd_forces(self):
        # snippet:py-biped-lesson2-pd-start
        q = self.biped.getPositions()
        dq = self.biped.getVelocities()
        p = -self.Kp @ (q - self.target_positions)
        d = -self.Kd @ dq
        self.forces += p + d
        self.biped.setForces(self.forces)
        # snippet:py-biped-lesson2-pd-end

    def add_spd_forces(self):
        # snippet:py-biped-lesson3-spd-start
        q = self.biped.getPositions()
        dq = self.biped.getVelocities()
        dt = self.biped.getTimeStep()

        inv_mass = np.linalg.inv(self.biped.getMassMatrix() + self.Kd * dt)
        p = -self.Kp @ (q + dq * dt - self.target_positions)
        d = -self.Kd @ dq
        qddot = inv_mass @ (
            -self.biped.getCoriolisAndGravityForces()
            + p
            + d
            + self.biped.getConstraintForces()
        )

        self.forces += p + d - (self.Kd @ qddot) * dt
        self.biped.setForces(self.forces)
        # snippet:py-biped-lesson3-spd-end

    def add_ankle_strategy_forces(self):
        # snippet:py-biped-lesson4-deviation-start
        com = self.biped.getCOM()
        offset = np.array([0.05, 0.0, 0.0])
        heel = self.biped.getBodyNode("h_heel_left")
        cop = heel.getTransform().multiply(offset)
        diff = com[0] - cop[0]
        # snippet:py-biped-lesson4-deviation-end

        # snippet:py-biped-lesson4-velocity-start
        dcom = self.biped.getCOMLinearVelocity()
        dcop = heel.getLinearVelocity(offset)
        ddiff = dcom[0] - dcop[0]
        # snippet:py-biped-lesson4-velocity-end

        dof_names = {
            "l_heel": "j_heel_left_1",
            "r_heel": "j_heel_right_1",
            "l_toe": "j_toe_left",
            "r_toe": "j_toe_right",
        }
        indices = {
            key: self.biped.getDof(name).getIndexInSkeleton()
            for key, name in dof_names.items()
        }

        if 0.0 <= diff < 0.1:
            k1, k2, kd = 200.0, 100.0, 10.0
        elif -0.2 < diff < -0.05:
            k1, k2, kd = 2000.0, 100.0, 100.0
        else:
            k1 = k2 = kd = None

        if k1 is not None:
            self.forces[indices["l_heel"]] += -k1 * diff - kd * ddiff
            self.forces[indices["r_heel"]] += -k1 * diff - kd * ddiff
            self.forces[indices["l_toe"]] += -k2 * diff - kd * ddiff
            self.forces[indices["r_toe"]] += -k2 * diff - kd * ddiff
            self.biped.setForces(self.forces)

    def set_wheel_commands(self):
        # snippet:py-biped-lesson6-wheel-commands-start
        wheel_names = [
            "joint_front_left_1",
            "joint_front_left_2",
            "joint_front_right_1",
            "joint_front_right_2",
            "joint_back_left",
            "joint_back_right",
        ]
        first = self.biped.getDof("joint_front_left_1").getIndexInSkeleton()
        for i in range(first, self.biped.getNumDofs()):
            self.Kp[i, i] = 0.0
            self.Kd[i, i] = 0.0

        for name in ["joint_front_left_2", "joint_front_right_2", "joint_back_left", "joint_back_right"]:
            self.biped.setCommand(
                self.biped.getDof(name).getIndexInSkeleton(), self.speed
            )
        # snippet:py-biped-lesson6-wheel-commands-end

    def change_wheel_speed(self, increment: float):
        self.speed += increment
        print(f"wheel speed = {self.speed:.2f}")


class BipedEventHandler(dart.gui.GUIEventHandler):
    def __init__(self, world: dart.World, controller: Controller):
        super().__init__()
        self.world = world
        self.controller = controller
        self.force_countdown = 0
        self.push_positive = True

    def handle(self, ea, _aa):
        if ea.getEventType() != dart.gui.GUIEventAdapter.KEYDOWN:
            return False

        gea = dart.gui.GUIEventAdapter
        key = ea.getKey()
        if key == gea.KEY_Comma:
            self.force_countdown = DEFAULT_COUNTDOWN
            self.push_positive = False
            return True
        if key == gea.KEY_Period:
            self.force_countdown = DEFAULT_COUNTDOWN
            self.push_positive = True
            return True
        if key in (ord("a"), ord("A")):
            self.controller.change_wheel_speed(DEFAULT_SPEED_INCREMENT)
            return True
        if key in (ord("s"), ord("S")):
            self.controller.change_wheel_speed(-DEFAULT_SPEED_INCREMENT)
            return True
        return False

    def update(self):
        self.controller.clear_forces()
        self.controller.add_spd_forces()
        self.controller.add_ankle_strategy_forces()
        self.controller.set_wheel_commands()

        if self.force_countdown > 0:
            biped = self.world.getSkeleton("biped")
            body = biped.getBodyNode("h_abdomen")
            direction = np.array([1.0, 0.0, 0.0])
            if not self.push_positive:
                direction *= -1
            body.addExtForce(DEFAULT_FORCE * direction, body.getCOM(), False, False)
            self.force_countdown -= 1


class CustomWorldNode(dart.gui.RealTimeWorldNode):
    def __init__(self, world, handler):
        super().__init__(world)
        self.handler = handler

    def customPreStep(self):
        self.handler.update()


def load_biped() -> dart.dynamics.Skeleton:
    # snippet:py-biped-lesson1-load-start
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/biped.skel")
    biped = world.getSkeleton("biped")
    # snippet:py-biped-lesson1-load-end

    # snippet:py-biped-lesson1-limits-start
    for i in range(biped.getNumJoints()):
        biped.getJoint(i).setLimitEnforcement(True)
    # snippet:py-biped-lesson1-limits-end

    # snippet:py-biped-lesson1-self-start
    biped.enableSelfCollisionCheck()
    biped.disableAdjacentBodyCheck()
    # snippet:py-biped-lesson1-self-end
    return biped


def set_initial_pose(biped: dart.dynamics.Skeleton):
    # snippet:py-biped-lesson2-initial-pose-start
    biped.getDof("j_thigh_left_z").setPosition(0.15)
    biped.getDof("j_thigh_right_z").setPosition(0.15)
    biped.getDof("j_shin_left").setPosition(-0.40)
    biped.getDof("j_shin_right").setPosition(-0.40)
    biped.getDof("j_heel_left_1").setPosition(0.25)
    biped.getDof("j_heel_right_1").setPosition(0.25)
    # snippet:py-biped-lesson2-initial-pose-end


def _add_wheel(
    biped: dart.dynamics.Skeleton,
    parent: dart.dynamics.BodyNode,
    name: str,
    translation,
):
    yaw_props = dart.dynamics.RevoluteJointProperties()
    yaw_props.mName = f"{name}_1"
    yaw_props.mAxis = np.array([0.0, 0.0, 1.0])
    yaw_props.mT_ParentBodyToJoint.set_translation(translation)
    _, yaw_body = biped.createRevoluteJointAndBodyNodePair(
        parent,
        yaw_props,
        dart.dynamics.BodyNodeProperties(
            dart.dynamics.BodyNodeAspectProperties(f"{name}_yaw")
        ),
    )

    roll_props = dart.dynamics.RevoluteJointProperties()
    roll_props.mName = f"{name}_2"
    roll_props.mAxis = np.array([0.0, 1.0, 0.0])
    joint, wheel_body = biped.createRevoluteJointAndBodyNodePair(
        yaw_body,
        roll_props,
        dart.dynamics.BodyNodeProperties(
            dart.dynamics.BodyNodeAspectProperties(f"{name}_wheel")
        ),
    )

    cyl = dart.dynamics.CylinderShape(0.05, 0.02)
    node = wheel_body.createShapeNode(cyl)
    node.createCollisionAspect()
    node.createDynamicsAspect()
    node.createVisualAspect().setColor([0.2, 0.2, 0.2, 1.0])
    return joint


def modify_biped_with_skateboard(biped: dart.dynamics.Skeleton):
    # snippet:py-biped-lesson5-skateboard-start
    props = dart.dynamics.EulerJointProperties()
    props.mName = "skateboard_mount"
    props.mT_ChildBodyToJoint.set_translation([0.0, 0.1, 0.0])
    _, board = biped.createEulerJointAndBodyNodePair(
        biped.getBodyNode("h_heel_left"),
        props,
        dart.dynamics.BodyNodeProperties(
            dart.dynamics.BodyNodeAspectProperties("skateboard")
        ),
    )

    deck = dart.dynamics.BoxShape([0.5, 0.02, 0.2])
    deck_node = board.createShapeNode(deck)
    deck_node.createCollisionAspect()
    deck_node.createDynamicsAspect()
    deck_node.createVisualAspect().setColor([0.4, 0.2, 0.1, 1.0])

    offsets = {
        "joint_front_left": np.array([0.2, -0.02, 0.08]),
        "joint_front_right": np.array([0.2, -0.02, -0.08]),
    }
    for name, translation in offsets.items():
        _add_wheel(biped, board, name, translation)

    back_left_props = dart.dynamics.RevoluteJointProperties()
    back_left_props.mName = "joint_back_left"
    back_left_props.mAxis = np.array([0.0, 1.0, 0.0])
    back_left_props.mT_ParentBodyToJoint.set_translation([-0.2, -0.02, 0.08])
    _, back_left = biped.createRevoluteJointAndBodyNodePair(
        board,
        back_left_props,
        dart.dynamics.BodyNodeProperties(
            dart.dynamics.BodyNodeAspectProperties("joint_back_left_wheel")
        ),
    )
    node_left = back_left.createShapeNode(dart.dynamics.CylinderShape(0.05, 0.02))
    node_left.createCollisionAspect()
    node_left.createDynamicsAspect()
    node_left.createVisualAspect().setColor([0.2, 0.2, 0.2, 1.0])

    back_right_props = dart.dynamics.RevoluteJointProperties()
    back_right_props.mName = "joint_back_right"
    back_right_props.mAxis = np.array([0.0, 1.0, 0.0])
    back_right_props.mT_ParentBodyToJoint.set_translation([-0.2, -0.02, -0.08])
    _, back_right = biped.createRevoluteJointAndBodyNodePair(
        board,
        back_right_props,
        dart.dynamics.BodyNodeProperties(
            dart.dynamics.BodyNodeAspectProperties("joint_back_right_wheel")
        ),
    )
    node_right = back_right.createShapeNode(dart.dynamics.CylinderShape(0.05, 0.02))
    node_right.createCollisionAspect()
    node_right.createDynamicsAspect()
    node_right.createVisualAspect().setColor([0.2, 0.2, 0.2, 1.0])
    # snippet:py-biped-lesson5-skateboard-end


def set_velocity_actuators(biped: dart.dynamics.Skeleton):
    # snippet:py-biped-lesson6-velocity-actuators-start
    for name in [
        "joint_front_left_2",
        "joint_front_right_2",
        "joint_back_left",
        "joint_back_right",
    ]:
        biped.getJoint(name).setActuatorType(dart.dynamics.Joint.VELOCITY)
    # snippet:py-biped-lesson6-velocity-actuators-end


def solve_ik(biped: dart.dynamics.Skeleton) -> np.ndarray:
    # snippet:py-biped-lesson7-ik-start
    biped.getDof("j_shin_right").setPosition(-1.4)
    biped.getDof("j_bicep_left_x").setPosition(0.8)
    biped.getDof("j_bicep_right_x").setPosition(-0.8)

    target = biped.getPositions().copy()
    left_heel = biped.getBodyNode("h_heel_left")
    left_toe = biped.getBodyNode("h_toe_left")
    initial_height = -0.8

    for _ in range(DEFAULT_IK_ITERATIONS):
        deviation = biped.getCOM() - left_heel.getCOM()
        local_com = left_heel.getCOM(left_heel)
        jacobian = biped.getCOMLinearJacobian() - biped.getLinearJacobian(
            left_heel, local_com
        )

        error = deviation[0]
        direction = -0.2 * error * jacobian[0]
        error = deviation[2]
        direction += -0.2 * error * jacobian[2]

        offset = np.array([0.0, -0.04, -0.03])
        error = (left_heel.getTransform().multiply(offset))[1] - initial_height
        direction += -0.2 * error * biped.getLinearJacobian(left_heel, offset)[1]

        offset[2] = 0.03
        error = (left_heel.getTransform().multiply(offset))[1] - initial_height
        direction += -0.2 * error * biped.getLinearJacobian(left_heel, offset)[1]

        offset[0] = 0.04
        error = (left_toe.getTransform().multiply(offset))[1] - initial_height
        direction += -0.2 * error * biped.getLinearJacobian(left_toe, offset)[1]

        offset[2] = -0.03
        error = (left_toe.getTransform().multiply(offset))[1] - initial_height
        direction += -0.2 * error * biped.getLinearJacobian(left_toe, offset)[1]

        target += direction
        biped.setPositions(target)
        biped.computeForwardKinematics(True, False, False)
    # snippet:py-biped-lesson7-ik-end
    return target


def create_floor() -> dart.dynamics.Skeleton:
    floor = dart.dynamics.Skeleton("floor")
    _, body = floor.createWeldJointAndBodyNodePair()
    shape = dart.dynamics.BoxShape([10.0, 0.01, 10.0])
    node = body.createShapeNode(shape)
    node.createVisualAspect().setColor([0.0, 0.0, 0.0, 1.0])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    tf = dart.math.Isometry3()
    tf.set_translation([0.0, -1.0, 0.0])
    body.getParentJoint().setTransformFromParentBodyNode(tf)
    return floor


def main():
    floor = create_floor()
    biped = load_biped()
    set_initial_pose(biped)
    modify_biped_with_skateboard(biped)
    set_velocity_actuators(biped)
    balanced_pose = solve_ik(biped)
    biped.setPositions(balanced_pose)

    world = dart.World()
    world.setGravity([0.0, -9.81, 0.0])
    world.addSkeleton(floor)
    world.addSkeleton(biped)

    controller = Controller(biped)
    handler = BipedEventHandler(world, controller)
    node = CustomWorldNode(world, handler)

    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)
    viewer.addEventHandler(handler)
    viewer.addInstructionText("space bar: simulation on/off")
    viewer.addInstructionText("',' or '.': apply a push backward/forward")
    viewer.addInstructionText("'a'/'s': increase or decrease wheel speed")
    viewer.addInstructionText("'p': replay simulation")
    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.run()


if __name__ == "__main__":
    main()
