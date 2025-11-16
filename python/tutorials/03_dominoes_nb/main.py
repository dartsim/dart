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
        # Lesson 2b: Store the desired joint configuration for the PD controller.

        # Lesson 3a: Initialize the data needed for the operational space controller.

        self.kp_pd = 200.0
        self.kd_pd = 20.0
        self.kp_os = 5.0
        self.kd_os = 0.01
        self.forces = np.zeros(self.manipulator.getNumDofs())

    def set_pd_forces(self):
        # Lesson 2c: Write a stable PD controller.

        # Lesson 2d: Compensate for gravity and Coriolis forces.
        pass

    def set_operational_space_forces(self):
        # Lesson 3b: Build an operational space controller for the manipulator.
        pass


class DominoEventHandler(dart.gui.osg.GUIEventHandler):
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
        if ea.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False

        gea = dart.gui.osg.GUIEventAdapter
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
            # Lesson 1d: Apply an external force to push the first domino.
            self.force_countdown -= 1

        if self.push_countdown > 0:
            self.controller.set_operational_space_forces()
            self.push_countdown -= 1
        else:
            self.controller.set_pd_forces()

    def attempt_to_create_domino(self, angle: float):
        # Lesson 1a: Clone the base domino, move it, and adjust its yaw angle.

        # Lesson 1b: Use the collision detector to ensure the new domino
        #            does not intersect existing objects.
        pass

    def delete_last_domino(self):
        # Lesson 1c: Remove the most recently created domino from the scene.
        pass


class CustomWorldNode(dart.gui.osg.RealTimeWorldNode):
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
    # Lesson 2a: Load the manipulator from URDF and position its base.
    return dart.dynamics.Skeleton("manipulator")


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

    viewer = dart.gui.osg.Viewer()
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
