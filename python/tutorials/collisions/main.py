# Copyright (c) 2011, The DART development contributors
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

"""Toss rigid, soft, and articulated bodies at a wall (exercise scaffold)."""

import math
import random

import dartpy as dart

default_shape_density = 1000.0  # kg/m^3
default_shape_height = 0.1  # m
default_shape_width = 0.03  # m
default_skin_thickness = 1e-3  # m
default_start_height = 0.4  # m
minimum_start_v = 2.5  # m/s
maximum_start_v = 4.0  # m/s
default_start_v = 3.5  # m/s
minimum_launch_angle = math.radians(30.0)
maximum_launch_angle = math.radians(70.0)
default_launch_angle = math.radians(45.0)
maximum_start_w = 6 * math.pi  # rad/s
default_start_w = 3 * math.pi  # rad/s
ring_spring_stiffness = 0.5
ring_damping_coefficient = 0.05
default_damping_coefficient = 0.001
default_ground_width = 2.0
default_wall_thickness = 0.1
default_wall_height = 1.0
default_spawn_range = 0.9 * default_ground_width / 2
default_restitution = 0.6
default_vertex_stiffness = 1000.0
default_edge_stiffness = 1.0
default_soft_damping = 5.0

SOFT_BOX, SOFT_CYLINDER, SOFT_ELLIPSOID = range(3)


def setup_ring(ring):
    # Lesson 4a: set the spring and damping coefficients after the root DOFs.
    # Lesson 4b: compute a ring angle and set BallJoint rest positions.
    # Lesson 4c: set the joints to their rest positions.
    raise NotImplementedError("Lesson 4a-c: configure the ring joints")


class CollisionsEventHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, world, ball, soft_body, hybrid_body, rigid_chain, rigid_ring):
        super().__init__()
        self.world = world
        self.randomize = True
        self.rng = random.Random()
        self.originals = (ball, soft_body, hybrid_body, rigid_chain, rigid_ring)
        self.joint_constraints = []
        self.skeleton_count = 0

    def handle(self, event, action):
        if event.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False
        key = event.getKey()
        if ord("1") <= key <= ord("5"):
            obj = self.originals[key - ord("1")].clone()
            if key == ord("5"):
                self.add_ring(obj)
            else:
                self.add_object(obj)
            return True
        if key == ord("d"):
            if self.world.getNumSkeletons() > 2:
                self.remove_skeleton(self.world.getSkeleton(2))
            print(f"Remaining objects: {self.world.getNumSkeletons() - 2}")
            return True
        if key == ord("r"):
            self.randomize = not self.randomize
            print(f"Randomization: {'on' if self.randomize else 'off'}")
            return True
        return False

    def add_object(self, obj):
        # Lesson 3a: set the object's starting position.
        # Lesson 3b: give it a unique name and add it to the world.
        # Lesson 3c: reject it if its collision group overlaps the world.
        # Lesson 3d: create reference frames for setting the initial velocity.
        # Lesson 3e: set the reference frames' linear and angular velocities.
        # Lesson 3f: set the root joint velocity from the root reference frame.
        raise NotImplementedError("Lesson 3a-f: launch an object without overlap")

    def add_ring(self, ring):
        setup_ring(ring)
        if not self.add_object(ring):
            return False
        # Lesson 5: close the ring with a BallJointConstraint and retain it.
        raise NotImplementedError("Lesson 5: close the ring with a constraint")

    def remove_skeleton(self, skeleton):
        for i, (ring, constraint) in enumerate(self.joint_constraints):
            if ring == skeleton:
                self.world.getConstraintSolver().removeConstraint(constraint)
                self.joint_constraints.pop(i)
                break
        self.world.removeSkeleton(skeleton)


class CustomWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, handler):
        super().__init__(world)
        self.world = world
        self.handler = handler


def add_rigid_body(chain, name, shape_type, parent=None):
    # Lesson 1a: set the FreeJoint or BallJoint properties and attachment frames.
    # Lesson 1b: create the Joint and BodyNode pair.
    # Lesson 1c: create a box, cylinder, or ellipsoid with all three aspects.
    # Lesson 1d: compute mass and inertia from the shape and density.
    # Lesson 1e: set the coefficient of restitution.
    # Lesson 1f: damp the child joints to stabilize the simulation.
    raise NotImplementedError("Lesson 1a-f: create a rigid body")


def add_soft_body(chain, name, shape_type, parent=None):
    # Lesson 2a: set the FreeJoint properties.
    # Lesson 2b: use SoftBodyNodeHelper to create the soft surface properties.
    # Lesson 2c: create the FreeJoint and SoftBodyNode pair.
    # Lesson 2d: give the underlying BodyNode negligible rigid inertia.
    # Lesson 2e: make the surface transparent.
    raise NotImplementedError("Lesson 2a-e: create a soft body")


def create_ball():
    ball = dart.dynamics.Skeleton("rigid_ball")
    add_rigid_body(ball, "rigid ball", "ellipsoid")
    ball.setColor([1, 0, 0])
    return ball


def create_rigid_chain():
    chain = dart.dynamics.Skeleton("rigid_chain")
    body = add_rigid_body(chain, "rigid box 1", "box")
    body = add_rigid_body(chain, "rigid cyl 2", "cylinder", body)
    add_rigid_body(chain, "rigid box 3", "box", body)
    chain.setColor([1, 0.5, 0])
    return chain


def create_rigid_ring():
    ring = dart.dynamics.Skeleton("rigid_ring")
    body = add_rigid_body(ring, "rigid box 1", "box")
    for i in range(2, 7):
        shape = "cylinder" if i % 2 == 0 else "box"
        name = f"rigid {'cyl' if i % 2 == 0 else 'box'} {i}"
        body = add_rigid_body(ring, name, shape, body)
    ring.setColor([0, 0, 1])
    return ring


def create_soft_body():
    soft = dart.dynamics.Skeleton("soft")
    add_soft_body(soft, "soft box", SOFT_BOX)
    # Lesson 2f: add a rigid core, with collision geometry and inertia.
    raise NotImplementedError("Lesson 2f: add the soft body's rigid core")


def create_hybrid_body():
    hybrid = dart.dynamics.Skeleton("hybrid")
    add_soft_body(hybrid, "soft sphere", SOFT_ELLIPSOID)
    # Lesson 2g: attach a rigid box using a WeldJoint and give it inertia.
    raise NotImplementedError("Lesson 2g: attach a rigid body to the soft body")


def create_ground():
    ground = dart.dynamics.Skeleton("ground")
    _, body = ground.createWeldJointAndBodyNodePair()
    shape = dart.dynamics.BoxShape(
        [default_ground_width, default_ground_width, default_wall_thickness]
    )
    shape_node = body.createShapeNode(shape)
    shape_node.createVisualAspect().setColor([1, 1, 1])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    return ground


def create_wall():
    wall = dart.dynamics.Skeleton("wall")
    joint, body = wall.createWeldJointAndBodyNodePair()
    shape = dart.dynamics.BoxShape(
        [default_wall_thickness, default_ground_width, default_wall_height]
    )
    shape_node = body.createShapeNode(shape)
    shape_node.createVisualAspect().setColor([0.8, 0.8, 0.8])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect().setRestitutionCoeff(0.2)
    tf = dart.math.Isometry3()
    tf.set_translation(
        [
            (default_ground_width + default_wall_thickness) / 2,
            0,
            (default_wall_height - default_wall_thickness) / 2,
        ]
    )
    joint.setTransformFromParentBodyNode(tf)
    return wall


def build_scene():
    world = dart.simulation.World()
    world.setGravity([0, 0, -9.81])
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
    return CustomWorldNode(world, handler)


def main():
    node = build_scene()
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.addEventHandler(node.handler)
    viewer.addInstructionText(
        "space bar: simulation on/off\n"
        "'1': toss a rigid ball\n"
        "'2': toss a soft body\n"
        "'3': toss a hybrid soft/rigid body\n"
        "'4': toss a rigid chain\n"
        "'5': toss a ring of rigid bodies\n"
        "'d': delete the oldest object\n"
        "'r': toggle randomness\n"
        "\nWarning: Let objects settle before tossing a new one, or the "
        "simulation could explode.\n"
        "         If the simulation freezes, you may need to force quit the application.\n"
    )
    print(viewer.getInstructions())
    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([3, 2, 2], [0, 0, 0], [0, 0, 1])
    viewer.run()


if __name__ == "__main__":
    main()
