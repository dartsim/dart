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
import random
from typing import List, Optional, Tuple

import dartpy_nb as dart
import dartpy_nb.collision as dart_collision
import numpy as np

default_shape_density = 1000.0  # kg/m^3
default_shape_height = 0.1  # m
default_shape_width = 0.03  # m
default_skin_thickness = 1e-3  # m

default_start_height = 0.4  # m

minimum_start_v = 2.5  # m/s
maximum_start_v = 4.0  # m/s
default_start_v = 3.5  # m/s

minimum_launch_angle = math.radians(30.0)  # rad
maximum_launch_angle = math.radians(70.0)  # rad
default_launch_angle = math.radians(45.0)  # rad

maximum_start_w = 6 * math.pi  # rad/s
default_start_w = 3 * math.pi  # rad/s

ring_spring_stiffness = 0.5
ring_damping_coefficient = 0.05
default_damping_coefficient = 0.001

default_ground_width = 2.0
default_wall_thickness = 0.1
default_wall_height = 1.0
default_spawn_range = 0.9 * default_ground_width / 2.0

default_restitution = 0.6

SOFT_BOX = 0
SOFT_CYLINDER = 1
SOFT_ELLIPSOID = 2


def setup_ring(ring: dart.dynamics.Skeleton):
    dof_count = ring.getNumDofs()
    for idx in range(6, dof_count):
        dof = ring.getDof(idx)
        dof.setSpringStiffness(ring_spring_stiffness)
        dof.setDampingCoefficient(ring_damping_coefficient)

    num_edges = ring.getNumBodyNodes()
    angle = 2 * math.pi / num_edges

    for i in range(1, ring.getNumJoints()):
        joint = ring.getJoint(i)
        rotation = dart.math.eulerXYZToMatrix([0.0, angle, 0.0])
        rest = dart.dynamics.BallJoint.convertToPositions(rotation)
        for axis in range(3):
            joint.setRestPosition(axis, rest[axis])

    for idx in range(6, dof_count):
        dof = ring.getDof(idx)
        dof.setPosition(dof.getRestPosition())


class CollisionsEventHandler(dart.gui.osg.GUIEventHandler):
    def __init__(
        self,
        world: dart.simulation.World,
        ball: dart.dynamics.Skeleton,
        soft_body: dart.dynamics.Skeleton,
        hybrid_body: dart.dynamics.Skeleton,
        rigid_chain: dart.dynamics.Skeleton,
        rigid_ring: dart.dynamics.Skeleton,
    ):
        super().__init__()
        self.world = world
        self.randomize = True
        self.rng = random.Random()
        self.blueprint_ball = ball
        self.blueprint_soft = soft_body
        self.blueprint_hybrid = hybrid_body
        self.blueprint_chain = rigid_chain
        self.blueprint_ring = rigid_ring
        self.joint_constraints: List[dart.constraint.DynamicJointConstraint] = []
        self.spawn_index = 0

    def handle(self, ea, _aa):
        if ea.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False

        key = ea.getKey()
        gea = dart.gui.osg.GUIEventAdapter

        if key == gea.KEY_1:
            return self.add_object(self.blueprint_ball.clone("ball_clone"))
        if key == gea.KEY_2:
            return self.add_object(self.blueprint_soft.clone("soft_clone"))
        if key == gea.KEY_3:
            return self.add_object(self.blueprint_hybrid.clone("hybrid_clone"))
        if key == gea.KEY_4:
            return self.add_object(self.blueprint_chain.clone("chain_clone"))
        if key == gea.KEY_5:
            self.add_ring(self.blueprint_ring.clone("ring_clone"))
            return True
        if key == gea.KEY_D:
            if self.world.getNumSkeletons() > 2:
                self.remove_skeleton(self.world.getSkeleton(2))
            print(f"Remaining objects: {self.world.getNumSkeletons() - 2}")
            return True
        if key == gea.KEY_R:
            self.randomize = not self.randomize
            state = "on" if self.randomize else "off"
            print(f"Randomization: {state}")
            return True
        return False

    def _initial_position(self) -> np.ndarray:
        pos = np.zeros(6)
        if self.randomize:
            pos[4] = default_spawn_range * self.rng.uniform(-1.0, 1.0)
        pos[5] = default_start_height
        return pos

    def add_object(self, obj: dart.dynamics.Skeleton) -> bool:
        # Lesson 3a: Position the new object before launching it.

        # Lesson 3b: Give each spawned Skeleton a unique name.

        # Lesson 3c: Check whether the new object collides with existing ones.

        # Lesson 3d: Create reference frames to simplify velocity assignment.

        # Lesson 3e: Configure the linear and angular velocities.

        # Lesson 3f: Apply the velocity to the root joint.

        return True

    def _launch_object(self, obj: dart.dynamics.Skeleton):
        center_tf = dart.math.Isometry3()
        center_tf.set_translation(obj.getCOM())
        center = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "center", center_tf
        )

        angle = default_launch_angle
        speed = default_start_v
        angular_speed = default_start_w
        if self.randomize:
            angle = self.rng.uniform(0.0, 1.0) * (
                maximum_launch_angle - minimum_launch_angle
            ) + minimum_launch_angle
            speed = self.rng.uniform(minimum_start_v, maximum_start_v)
            angular_speed = self.rng.uniform(-maximum_start_w, maximum_start_w)

        linear = np.array([math.cos(angle), 0.0, math.sin(angle)]) * speed
        angular = np.array([0.0, 1.0, 0.0]) * angular_speed
        center.setClassicDerivatives(linear, angular)

        ref = dart.dynamics.SimpleFrame(center, "root_reference")
        ref.setRelativeTransform(
            obj.getBodyNode(0).getTransform(center)
        )
        obj.getJoint(0).setVelocities(ref.getSpatialVelocity())

    def add_ring(self, ring: dart.dynamics.Skeleton):
        setup_ring(ring)
        if not self.add_object(ring):
            return

        # Lesson 5: Close the chain by adding a constraint.

    def remove_skeleton(self, skel: dart.dynamics.Skeleton):
        solver = self.world.getConstraintSolver()
        survivors: List[dart.constraint.DynamicJointConstraint] = []
        for constraint in self.joint_constraints:
            nodes = [
                constraint.getBodyNode1(),
                constraint.getBodyNode2(),
            ]
            if any(node.getSkeleton() == skel for node in nodes if node):
                solver.removeConstraint(constraint)
            else:
                survivors.append(constraint)
        self.joint_constraints = survivors
        self.world.removeSkeleton(skel)


class CustomWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world):
        super().__init__(world)


def add_rigid_body(
    skeleton: dart.dynamics.Skeleton,
    joint_type: str,
    name: str,
    shape_type: str,
    parent: Optional[dart.dynamics.BodyNode] = None,
) -> dart.dynamics.BodyNode:
    # Lesson 1a: Configure joint properties.

    # Lesson 1b: Create the joint/body pair.

    # Lesson 1c: Attach the requested geometry for visualization/collision.

    # Lesson 1d: Compute the inertia tensor for the shape.

    # Lesson 1e: Tune restitution to make collisions bouncy.

    # Lesson 1f: Add joint damping for stability.

    return None


def add_soft_body(
    skeleton: dart.dynamics.Skeleton,
    joint_type: str,
    name: str,
    shape_type: int,
    parent: Optional[dart.dynamics.BodyNode] = None,
) -> dart.dynamics.BodyNode:
    # Lesson 2a: Position the soft body joint relative to neighbors.

    # Lesson 2b: Build the unique soft-body properties.

    # Lesson 2c: Create the soft body node and attach it to the skeleton.

    # Lesson 2d: Zero out the inertia on the underlying BodyNode.

    # Lesson 2e: Make the soft skin transparent.

    return None


def set_all_colors(skeleton: dart.dynamics.Skeleton, color: Tuple[float, float, float]):
    for i in range(skeleton.getNumBodyNodes()):
        body = skeleton.getBodyNode(i)
        for j in range(body.getNumShapeNodes()):
            visual = body.getShapeNode(j).getVisualAspect()
            visual.setColor(list(color) + [1.0])


def create_ball() -> dart.dynamics.Skeleton:
    ball = dart.dynamics.Skeleton("rigid_ball")
    add_rigid_body(ball, "free", "rigid_ball", "ellipsoid")
    set_all_colors(ball, (1.0, 0.0, 0.0))
    return ball


def create_rigid_chain() -> dart.dynamics.Skeleton:
    chain = dart.dynamics.Skeleton("rigid_chain")
    body = add_rigid_body(chain, "free", "rigid_box1", "box")
    body = add_rigid_body(chain, "ball", "rigid_cyl2", "cylinder", body)
    add_rigid_body(chain, "ball", "rigid_box3", "box", body)
    set_all_colors(chain, (1.0, 0.5, 0.0))
    return chain


def create_rigid_ring() -> dart.dynamics.Skeleton:
    ring = dart.dynamics.Skeleton("rigid_ring")
    body = add_rigid_body(ring, "free", "rigid_box1", "box")
    for idx in range(2, 7):
        shape = "box" if idx % 2 == 1 else "cylinder"
        body = add_rigid_body(
            ring,
            "ball",
            f"rigid_{shape}{idx}",
            shape,
            body,
        )
    set_all_colors(ring, (0.2, 0.2, 1.0))
    return ring


def create_soft_body() -> dart.dynamics.Skeleton:
    soft = dart.dynamics.Skeleton("soft_approx")
    add_soft_body(soft, "free", "soft_core", SOFT_BOX)

    # Lesson 2f: Add rigid collision geometry and inertia to the soft body.

    set_all_colors(soft, (1.0, 0.0, 1.0))
    return soft


def create_hybrid_body() -> dart.dynamics.Skeleton:
    hybrid = dart.dynamics.Skeleton("hybrid")

    # Lesson 2g: Attach a rigid geometry to the soft body with a WeldJoint.

    set_all_colors(hybrid, (0.0, 1.0, 0.0))
    return hybrid


def create_ground() -> dart.dynamics.Skeleton:
    ground = dart.dynamics.Skeleton("ground")
    body = ground.createWeldJointAndBodyNodePair()[1]
    dims = [
        default_ground_width,
        default_ground_width,
        default_wall_thickness,
    ]
    shape = dart.dynamics.BoxShape(dims)
    node = body.createShapeNode(shape)
    node.createVisualAspect().setColor([1.0, 1.0, 1.0, 1.0])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    return ground


def create_wall() -> dart.dynamics.Skeleton:
    wall = dart.dynamics.Skeleton("wall")
    body = wall.createWeldJointAndBodyNodePair()[1]
    dims = [
        default_wall_thickness,
        default_ground_width,
        default_wall_height,
    ]
    shape = dart.dynamics.BoxShape(dims)
    node = body.createShapeNode(shape)
    node.createVisualAspect().setColor([0.8, 0.8, 0.8, 1.0])
    node.createCollisionAspect()
    dyn = node.createDynamicsAspect()
    dyn.setRestitutionCoeff(0.2)

    tf = dart.math.Isometry3()
    tf.set_translation(
        [
            (default_ground_width + default_wall_thickness) / 2.0,
            0.0,
            (default_wall_height - default_wall_thickness) / 2.0,
        ]
    )
    body.getParentJoint().setTransformFromParentBodyNode(tf)
    return wall


def main():
    world = dart.simulation.World()
    world.addSkeleton(create_ground())
    world.addSkeleton(create_wall())

    handler = CollisionsEventHandler(
        world,
        create_ball(),
        create_soft_body(),
        create_hybrid_body(),
        create_rigid_chain(),
        create_rigid_ring(),
    )

    node = CustomWorldNode(world)

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.addEventHandler(handler)

    viewer.addInstructionText("space bar: simulation on/off")
    viewer.addInstructionText("'1': toss a rigid ball")
    viewer.addInstructionText("'2': toss a soft body")
    viewer.addInstructionText("'3': toss a hybrid body")
    viewer.addInstructionText("'4': toss a rigid chain")
    viewer.addInstructionText("'5': toss a rigid ring")
    viewer.addInstructionText("'d': delete the oldest object")
    viewer.addInstructionText("'r': toggle randomness")
    viewer.addInstructionText(
        "Warning: let objects settle before tossing a new one."
    )
    print(viewer.getInstructions())

    grid = dart.gui.osg.GridVisual()
    grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.XZ)
    grid.setOffset([0.0, -default_wall_thickness / 2.0, 0.0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition(
        [3.0, 2.0, 2.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    )

    viewer.run()


if __name__ == "__main__":
    main()
