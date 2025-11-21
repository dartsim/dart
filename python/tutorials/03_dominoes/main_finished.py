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

import math
from typing import List, Optional

import dartpy_nb as dart
import numpy as np

default_domino_height = 0.3
default_domino_width = 0.4 * default_domino_height
default_domino_depth = default_domino_width / 5.0

default_distance = default_domino_height / 2.0
default_angle = math.radians(20.0)

default_domino_density = 2.6e3
default_domino_mass = (
    default_domino_density
    * default_domino_height
    * default_domino_width
    * default_domino_depth
)

default_push_force = 8.0
default_force_duration = 200
default_push_duration = 1000

default_endeffector_offset = 0.05


class Controller:
    def __init__(
        self,
        manipulator: dart.dynamics.Skeleton,
        domino: dart.dynamics.Skeleton,
    ):
        self.manipulator = manipulator
        # snippet:py-dominoes-lesson3a-end-effector-start
        self.end_effector = self.manipulator.getBodyNode(
            self.manipulator.getNumBodyNodes() - 1
        )
        # snippet:py-dominoes-lesson3a-end-effector-end
        # snippet:py-dominoes-lesson3a-offset-start
        self.offset = np.array(
            [default_endeffector_offset, 0.0, 0.0], dtype=float
        )
        # snippet:py-dominoes-lesson3a-offset-end

        # snippet:py-dominoes-lesson2b-desired-positions-start
        self.q_desired = self.manipulator.getPositions().copy()
        # snippet:py-dominoes-lesson2b-desired-positions-end
        # snippet:py-dominoes-lesson3a-end-effector-start
        self.target = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "target"
        )
        # snippet:py-dominoes-lesson3a-end-effector-end

        # snippet:py-dominoes-lesson3a-target-offset-start
        target_tf = dart.math.Isometry3()
        target_tf.set_translation(
            [0.0, 0.0, default_domino_height / 2.0]
        )
        # snippet:py-dominoes-lesson3a-target-offset-end
        # snippet:py-dominoes-lesson3a-target-rotation-start
        relative = self.end_effector.getTransform(domino.getBodyNode(0))
        target_tf.set_rotation(relative.rotation())
        # snippet:py-dominoes-lesson3a-target-rotation-end
        # snippet:py-dominoes-lesson3a-target-set-start
        self.target.setTransform(target_tf, domino.getBodyNode(0))
        # snippet:py-dominoes-lesson3a-target-set-end

        self.kp_pd = 200.0
        self.kd_pd = 20.0
        self.kp_os = 5.0
        self.kd_os = 0.01
        self.forces = np.zeros(self.manipulator.getNumDofs())

    def set_pd_forces(self):
        if self.manipulator is None:
            return

        # snippet:py-dominoes-lesson2c-state-start
        dt = self.manipulator.getTimeStep()
        q = self.manipulator.getPositions().copy()
        dq = self.manipulator.getVelocities()
        # snippet:py-dominoes-lesson2c-state-end
        # snippet:py-dominoes-lesson2c-integrate-start
        q += dq * dt
        # snippet:py-dominoes-lesson2c-integrate-end
        # snippet:py-dominoes-lesson2c-q-error-start
        q_err = self.q_desired - q
        # snippet:py-dominoes-lesson2c-q-error-end
        # snippet:py-dominoes-lesson2c-dq-error-start
        dq_err = -dq
        # snippet:py-dominoes-lesson2c-dq-error-end
        # snippet:py-dominoes-lesson2d-cg-start
        cg = self.manipulator.getCoriolisAndGravityForces()
        # snippet:py-dominoes-lesson2d-cg-end
        # snippet:py-dominoes-lesson2c-mass-start
        mass = self.manipulator.getMassMatrix()
        # snippet:py-dominoes-lesson2c-mass-end

        # snippet:py-dominoes-lesson2c-force-law-start
        self.forces = mass @ (self.kp_pd * q_err + self.kd_pd * dq_err) + cg
        # snippet:py-dominoes-lesson2c-force-law-end
        # snippet:py-dominoes-lesson2c-apply-start
        self.manipulator.setForces(self.forces)
        # snippet:py-dominoes-lesson2c-apply-end

    def set_operational_space_forces(self):
        if self.manipulator is None:
            return

        # snippet:py-dominoes-lesson3b-mass-start
        mass = self.manipulator.getMassMatrix()
        # snippet:py-dominoes-lesson3b-mass-end
        # snippet:py-dominoes-lesson3b-jacobian-start
        J = self.end_effector.getWorldJacobian(self.offset)
        JJt = J @ J.T + 0.0025 * np.eye(6)
        pinv_J = J.T @ np.linalg.inv(JJt)
        # snippet:py-dominoes-lesson3b-jacobian-end

        # snippet:py-dominoes-lesson3b-jacobian-deriv-start
        dJ = self.end_effector.getJacobianClassicDeriv(self.offset)
        dJdJt = dJ @ dJ.T + 0.0025 * np.eye(6)
        pinv_dJ = dJ.T @ np.linalg.inv(dJdJt)
        # snippet:py-dominoes-lesson3b-jacobian-deriv-end

        # snippet:py-dominoes-lesson3b-linear-error-start
        target_tf = self.target.getWorldTransform()
        end_tf = self.end_effector.getWorldTransform()
        translation_error = (
            target_tf.translation() - end_tf.multiply(self.offset)
        )
        # snippet:py-dominoes-lesson3b-linear-error-end

        # snippet:py-dominoes-lesson3b-angular-error-start
        relative = self.target.getTransform(self.end_effector).rotation()
        aa = dart.math.AngleAxis(relative)
        axis = np.asarray(aa.axis()).reshape(3)
        angular_error = axis * aa.angle()
        # snippet:py-dominoes-lesson3b-angular-error-end

        # snippet:py-dominoes-lesson3b-error-derivative-start
        e = np.zeros(6)
        e[:3] = angular_error
        e[3:] = translation_error

        de = -self.end_effector.getSpatialVelocity(
            self.offset, self.target, dart.dynamics.Frame.World()
        )
        # snippet:py-dominoes-lesson3b-error-derivative-end

        # snippet:py-dominoes-lesson3b-cg-start
        cg = self.manipulator.getCoriolisAndGravityForces()
        # snippet:py-dominoes-lesson3b-cg-end
        # snippet:py-dominoes-lesson3b-gains-kp-start
        Kp = self.kp_os * np.eye(6)
        # snippet:py-dominoes-lesson3b-gains-kp-end
        # snippet:py-dominoes-lesson3b-gains-kd-start
        dofs = self.manipulator.getNumDofs()
        Kd = self.kd_os * np.eye(dofs)
        # snippet:py-dominoes-lesson3b-gains-kd-end

        # snippet:py-dominoes-lesson3b-feedforward-start
        f_desired = np.zeros(6)
        f_desired[3] = default_push_force
        feedforward = J.T @ f_desired
        # snippet:py-dominoes-lesson3b-feedforward-end
        # snippet:py-dominoes-lesson3b-control-law-start
        dq = self.manipulator.getVelocities()

        self.forces = (
            mass @ (pinv_J @ (Kp @ de) + pinv_dJ @ (Kp @ e))
            - Kd @ dq
            + Kd @ (pinv_J @ (Kp @ e))
            + cg
            + feedforward
        )
        # snippet:py-dominoes-lesson3b-apply-start
        self.manipulator.setForces(self.forces)
        # snippet:py-dominoes-lesson3b-apply-end
        # snippet:py-dominoes-lesson3b-control-law-end


class DominoEventHandler(dart.gui.GUIEventHandler):
    def __init__(
        self,
        world: dart.simulation.World,
        controller: Controller,
    ):
        super().__init__()
        self.world = world
        self.controller = controller
        self.first_domino = world.getSkeleton("domino")
        self.floor = world.getSkeleton("floor")
        self.dominoes: List[dart.dynamics.Skeleton] = []
        self.angles: List[float] = []
        self.total_angle = 0.0
        self.has_ever_run = False
        self.force_countdown = 0
        self.push_countdown = 0

    def handle(self, ea, _aa):
        if ea.getEventType() != dart.gui.GUIEventAdapter.KEYDOWN:
            return False

        gea = dart.gui.GUIEventAdapter
        key = ea.getKey()

        if not self.has_ever_run:
            if key == gea.KEY_Q:
                self.attempt_to_create_domino(default_angle)
                return True
            if key == gea.KEY_W:
                self.attempt_to_create_domino(0.0)
                return True
            if key == gea.KEY_E:
                self.attempt_to_create_domino(-default_angle)
                return True
            if key == gea.KEY_D:
                self.delete_last_domino()
                return True
            if key == gea.KEY_Space:
                self.has_ever_run = True
                return False
            return False
        if key == gea.KEY_F:
            self.force_countdown = default_force_duration
            return True
        if key == gea.KEY_R:
            self.push_countdown = default_push_duration
            return True
        return False

    def update(self):
        if self.force_countdown > 0:
            # snippet:py-dominoes-lesson1d-force-start
            force = default_push_force * np.array([1.0, 0.0, 0.0])
            location = np.array([0.0, 0.0, default_domino_height / 2.0])
            self.first_domino.getBodyNode(0).addExtForce(force, location)
            # snippet:py-dominoes-lesson1d-force-end
            self.force_countdown -= 1

        if self.push_countdown > 0:
            self.controller.set_operational_space_forces()
            self.push_countdown -= 1
        else:
            self.controller.set_pd_forces()

    def attempt_to_create_domino(self, angle: float):
        # snippet:py-dominoes-lesson1a-clone-start
        new_domino = self.first_domino.clone(
            f"domino_{len(self.dominoes) + 1}"
        )
        # snippet:py-dominoes-lesson1a-clone-end

        # snippet:py-dominoes-lesson1a-last-start
        last_domino = (
            self.dominoes[-1] if self.dominoes else self.first_domino
        )
        # snippet:py-dominoes-lesson1a-last-end
        # snippet:py-dominoes-lesson1a-offset-start
        displacement = default_distance * np.array(
            [math.cos(self.total_angle), math.sin(self.total_angle), 0.0]
        )
        # snippet:py-dominoes-lesson1a-offset-end

        # snippet:py-dominoes-lesson1a-copy-start
        pose = last_domino.getPositions().copy()
        # snippet:py-dominoes-lesson1a-copy-end
        # snippet:py-dominoes-lesson1a-translate-start
        pose[3:] += displacement
        # snippet:py-dominoes-lesson1a-translate-end
        # snippet:py-dominoes-lesson1a-angle-start
        pose[2] = self.total_angle + angle
        # snippet:py-dominoes-lesson1a-angle-end
        # snippet:py-dominoes-lesson1a-set-positions-start
        new_domino.setPositions(pose)
        # snippet:py-dominoes-lesson1a-set-positions-end

        # snippet:py-dominoes-lesson1b-collision-check-start
        solver = self.world.getConstraintSolver()
        collision_group = solver.getCollisionGroup()
        collision_detector = solver.getCollisionDetector()
        new_group = collision_detector.createCollisionGroup(new_domino)

        collision_group.removeShapeFramesOf(self.floor)
        domino_collision = collision_group.collide(new_group)
        collision_group.addShapeFramesOf(self.floor)
        # snippet:py-dominoes-lesson1b-collision-check-end

        # snippet:py-dominoes-lesson1b-result-start
        if not domino_collision:
            self.world.addSkeleton(new_domino)
            self.dominoes.append(new_domino)
            self.angles.append(angle)
            self.total_angle += angle
        else:
            print(
                "The new domino would penetrate something. Remove dominos "
                "with 'd' and try again."
            )
        # snippet:py-dominoes-lesson1b-result-end

    def delete_last_domino(self):
        # snippet:py-dominoes-lesson1c-delete-start
        if not self.dominoes:
            return
        last_domino = self.dominoes.pop()
        self.world.removeSkeleton(last_domino)
        self.total_angle -= self.angles.pop()
        # snippet:py-dominoes-lesson1c-delete-end


class CustomWorldNode(dart.gui.RealTimeWorldNode):
    def __init__(
        self,
        world: dart.simulation.World,
        handler: DominoEventHandler,
    ):
        super().__init__(world)
        self.handler = handler

    def customPreStep(self):
        self.handler.update()


def create_domino() -> dart.dynamics.Skeleton:
    domino = dart.dynamics.Skeleton("domino")
    _, body = domino.createFreeJointAndBodyNodePair()

    box = dart.dynamics.BoxShape(
        [default_domino_depth, default_domino_width, default_domino_height]
    )
    shape_node = body.createShapeNode(box)
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()

    inertia = dart.dynamics.Inertia()
    inertia.setMass(default_domino_mass)
    inertia.setMoment(box.computeInertia(default_domino_mass))
    body.setInertia(inertia)

    domino.getDof("Joint_pos_z").setPosition(default_domino_height / 2.0)
    return domino


def create_floor() -> dart.dynamics.Skeleton:
    floor = dart.dynamics.Skeleton("floor")
    _, body = floor.createWeldJointAndBodyNodePair()

    floor_width = 10.0
    floor_height = 0.01
    box = dart.dynamics.BoxShape(
        [floor_width, floor_width, floor_height]
    )
    shape_node = body.createShapeNode(box)
    shape_node.createVisualAspect().setColor([0.0, 0.0, 0.0, 1.0])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()

    tf = dart.math.Isometry3()
    tf.set_translation([0.0, 0.0, -floor_height / 2.0])
    body.getParentJoint().setTransformFromParentBodyNode(tf)
    return floor


def create_manipulator() -> dart.dynamics.Skeleton:
    # snippet:py-dominoes-lesson2a-loader-start
    loader = dart.utils.DartLoader()
    # snippet:py-dominoes-lesson2a-loader-end
    # snippet:py-dominoes-lesson2a-parse-start
    manipulator = loader.parseSkeleton(
        "dart://sample/urdf/KR5/KR5 sixx R650.urdf"
    )
    # snippet:py-dominoes-lesson2a-parse-end
    # snippet:py-dominoes-lesson2a-name-start
    manipulator.setName("manipulator")
    # snippet:py-dominoes-lesson2a-name-end

    # snippet:py-dominoes-lesson2a-base-start
    base_tf = dart.math.Isometry3()
    base_tf.set_translation([-0.65, 0.0, 0.0])
    manipulator.getJoint(0).setTransformFromParentBodyNode(base_tf)
    # snippet:py-dominoes-lesson2a-base-end

    # snippet:py-dominoes-lesson2a-configuration-start
    manipulator.getDof(1).setPosition(math.radians(140.0))
    manipulator.getDof(2).setPosition(math.radians(-140.0))
    # snippet:py-dominoes-lesson2a-configuration-end
    # snippet:py-dominoes-lesson2a-return-start
    return manipulator
    # snippet:py-dominoes-lesson2a-return-end
    # snippet:py-dominoes-lesson2a-return-end


def main():
    domino = create_domino()
    floor = create_floor()
    manipulator = create_manipulator()

    world = dart.simulation.World()
    world.addSkeleton(domino)
    world.addSkeleton(floor)
    world.addSkeleton(manipulator)

    controller = Controller(manipulator, domino)
    handler = DominoEventHandler(world, controller)
    node = CustomWorldNode(world, handler)

    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)
    viewer.addEventHandler(handler)

    viewer.addInstructionText(
        "Before the simulation starts, create dominoes with:\n"
    )
    viewer.addInstructionText("'q': left domino\n")
    viewer.addInstructionText("'w': forward domino\n")
    viewer.addInstructionText("'e': right domino\n")
    viewer.addInstructionText("'d': delete last domino\n")
    viewer.addInstructionText("\n")
    viewer.addInstructionText(
        "spacebar: begin simulation (domino editing disabled)\n"
    )
    viewer.addInstructionText("'p': replay simulation\n")
    viewer.addInstructionText("'f': push the first domino with a force\n")
    viewer.addInstructionText("'r': push the first domino with the arm\n")
    viewer.addInstructionText("'v': toggle contact force visualization\n")
    print(viewer.getInstructions())

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition(
        [2.0, 1.0, 2.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    )
    viewer.run()


if __name__ == "__main__":
    main()
