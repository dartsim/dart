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

"""Toss rigid, soft, and articulated bodies at a wall (finished lessons)."""

import math
import random

import dartpy as dart
import numpy as np

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
    # Lesson 4a: leave the six floating root coordinates free.
    for i in range(6, ring.getNumDofs()):
        dof = ring.getDof(i)
        dof.setSpringStiffness(ring_spring_stiffness)
        dof.setDampingCoefficient(ring_damping_coefficient)

    # Lesson 4b: the BallJoint coordinates encode an angle-axis rotation.
    angle = 2 * math.pi / ring.getNumBodyNodes()
    rotation = dart.math.AngleAxis(angle, [0, 1, 0]).rotation()
    rest = dart.dynamics.BallJoint.convertToPositions(rotation)
    for i in range(1, ring.getNumJoints()):
        for j in range(3):
            ring.getJoint(i).setRestPosition(j, rest[j])

    # Lesson 4c
    for i in range(6, ring.getNumDofs()):
        dof = ring.getDof(i)
        dof.setPosition(dof.getRestPosition())


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
        # Lesson 3a: FreeJoint positions are rotation followed by translation.
        positions = np.zeros(6)
        if self.randomize:
            positions[4] = self.rng.uniform(-default_spawn_range, default_spawn_range)
        positions[5] = default_start_height
        obj.getJoint(0).setPositions(positions)
        obj.setName(obj.getName() + str(self.skeleton_count))
        self.skeleton_count += 1

        # Lessons 3b-c: check the proposed object before adding it to the world.
        solver = self.world.getConstraintSolver()
        new_group = solver.getCollisionDetector().createCollisionGroup()
        new_group.addShapeFramesOf(obj)
        option = dart.collision.CollisionOption()
        result = dart.collision.CollisionResult()
        if solver.getCollisionGroup().collide(new_group, option, result):
            print("The new object spawned in a collision. It will not be added.")
            return False
        self.world.addSkeleton(obj)

        # Lesson 3d: define the desired motion at the object's center of mass.
        center_tf = dart.math.Isometry3()
        center_tf.set_translation(obj.getCOM())
        center = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "center", center_tf
        )
        angle = default_launch_angle
        speed = default_start_v
        angular_speed = default_start_w
        if self.randomize:
            angle = self.rng.uniform(minimum_launch_angle, maximum_launch_angle)
            speed = self.rng.uniform(minimum_start_v, maximum_start_v)
            angular_speed = self.rng.uniform(-maximum_start_w, maximum_start_w)

        # Lesson 3e: the center frame uses world coordinates for these velocities.
        v = speed * np.array([math.cos(angle), 0.0, math.sin(angle)])
        w = np.array([0.0, angular_speed, 0.0])
        center.setClassicDerivatives(v, w)
        ref = dart.dynamics.SimpleFrame(center, "root_reference")
        ref.setRelativeTransform(obj.getBodyNode(0).getTransform(center))

        # Lesson 3f: include the angular-velocity offset from COM to root.
        obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
        return True

    def add_ring(self, ring):
        setup_ring(ring)
        if not self.add_object(ring):
            return False
        # Lesson 5: close the chain with a constraint at the tail's endpoint.
        head = ring.getBodyNode(0)
        tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
        offset = tail.getWorldTransform().multiply([0, 0, default_shape_height / 2])
        constraint = dart.constraint.BallJointConstraint(head, tail, offset)
        self.world.getConstraintSolver().addConstraint(constraint)
        self.joint_constraints.append((ring, constraint))
        return True

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
    # Lesson 1a: offset both joint frames to meet between the body centers.
    properties = (
        dart.dynamics.FreeJointProperties()
        if parent is None
        else dart.dynamics.BallJointProperties()
    )
    properties.mName = name + "_joint"
    if parent is not None:
        tf = dart.math.Isometry3()
        tf.set_translation([0, 0, default_shape_height / 2])
        properties.mT_ParentBodyToJoint = tf
        properties.mT_ChildBodyToJoint = tf.inverse()

    # Lesson 1b
    body_properties = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(name)
    )
    if parent is None:
        _, body = chain.createFreeJointAndBodyNodePair(
            parent, properties, body_properties
        )
    else:
        _, body = chain.createBallJointAndBodyNodePair(
            parent, properties, body_properties
        )

    # Lesson 1c
    if shape_type == "box":
        shape = dart.dynamics.BoxShape(
            [default_shape_width, default_shape_width, default_shape_height]
        )
    elif shape_type == "cylinder":
        shape = dart.dynamics.CylinderShape(
            default_shape_width / 2, default_shape_height
        )
    elif shape_type == "ellipsoid":
        shape = dart.dynamics.EllipsoidShape(default_shape_height * np.ones(3))
    else:
        raise ValueError(f"Unknown rigid shape: {shape_type}")
    shape_node = body.createShapeNode(shape)
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    dynamics = shape_node.createDynamicsAspect()

    # Lessons 1d-e
    mass = default_shape_density * shape.getVolume()
    inertia = dart.dynamics.Inertia()
    inertia.setMass(mass)
    inertia.setMoment(shape.computeInertia(mass))
    body.setInertia(inertia)
    dynamics.setRestitutionCoeff(default_restitution)

    # Lesson 1f
    if parent is not None:
        joint = body.getParentJoint()
        for i in range(joint.getNumDofs()):
            joint.setDampingCoefficient(i, default_damping_coefficient)
    return body


def add_soft_body(chain, name, shape_type, parent=None):
    # Lesson 2a
    joint_properties = dart.dynamics.FreeJointProperties()
    joint_properties.mName = name + "_joint"
    if parent is not None:
        tf = dart.math.Isometry3()
        tf.set_translation([0, 0, default_shape_height / 2])
        joint_properties.mT_ParentBodyToJoint = tf
        joint_properties.mT_ChildBodyToJoint = tf.inverse()

    # Lesson 2b: distribute skin mass over the deformable surface vertices.
    helper = dart.dynamics.SoftBodyNodeHelper
    if shape_type == SOFT_BOX:
        width, height = default_shape_height, 2 * default_shape_width
        dims = np.array([width, width, height])
        mass = 2 * (dims[0] * dims[1] + dims[0] * dims[2] + dims[1] * dims[2])
        mass *= default_shape_density * default_skin_thickness
        soft_properties = helper.makeBoxProperties(
            dims, dart.math.Isometry3(), [4, 4, 4], mass
        )
    elif shape_type == SOFT_CYLINDER:
        radius, height = default_shape_height / 2, 2 * default_shape_width
        area = height * 2 * math.pi * radius + 2 * math.pi * radius**2
        mass = default_shape_density * area * default_skin_thickness
        soft_properties = helper.makeCylinderProperties(radius, height, 8, 3, 2, mass)
    elif shape_type == SOFT_ELLIPSOID:
        radius = default_shape_height / 2
        mass = default_shape_density * 4 * math.pi * radius**2 * default_skin_thickness
        soft_properties = helper.makeEllipsoidProperties(
            2 * radius * np.ones(3), 6, 6, mass
        )
    else:
        raise ValueError(f"Unknown soft shape: {shape_type}")
    soft_properties.mKv = default_vertex_stiffness
    soft_properties.mKe = default_edge_stiffness
    soft_properties.mDampCoeff = default_soft_damping

    # Lesson 2c
    body_properties = dart.dynamics.SoftBodyNodeProperties(
        dart.dynamics.BodyNodeProperties(dart.dynamics.BodyNodeAspectProperties(name)),
        soft_properties,
    )
    _, body = chain.createFreeJointAndSoftBodyNodePair(
        parent, joint_properties, body_properties
    )

    # Lesson 2d: the point masses supply the soft body's mass and inertia.
    inertia = dart.dynamics.Inertia()
    inertia.setMoment(1e-8 * np.eye(3))
    inertia.setMass(1e-8)
    body.setInertia(inertia)

    # Lesson 2e
    body.setAlpha(0.4)
    return body


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
    body = add_soft_body(soft, "soft box", SOFT_BOX)
    # Lesson 2f: provide a rigid core to avoid collapsing the skin completely.
    dims = 0.6 * np.array(
        [default_shape_height, default_shape_height, 2 * default_shape_width]
    )
    box = dart.dynamics.BoxShape(dims)
    shape_node = body.createShapeNode(box)
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    inertia = dart.dynamics.Inertia()
    inertia.setMass(default_shape_density * box.getVolume())
    inertia.setMoment(box.computeInertia(inertia.getMass()))
    body.setInertia(inertia)
    soft.setColor([1, 0, 1])
    return soft


def create_hybrid_body():
    hybrid = dart.dynamics.Skeleton("hybrid")
    body = add_soft_body(hybrid, "soft sphere", SOFT_ELLIPSOID)
    # Lesson 2g: attach a rigid body to the soft body's underlying reference frame.
    joint, body = hybrid.createWeldJointAndBodyNodePair(body)
    body.setName("rigid box")
    box = dart.dynamics.BoxShape(default_shape_height * np.ones(3))
    shape_node = body.createShapeNode(box)
    shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    tf = dart.math.Isometry3()
    tf.set_translation([default_shape_height / 2, 0, 0])
    joint.setTransformFromParentBodyNode(tf)
    inertia = dart.dynamics.Inertia()
    inertia.setMass(default_shape_density * box.getVolume())
    inertia.setMoment(box.computeInertia(inertia.getMass()))
    body.setInertia(inertia)
    hybrid.setColor([0, 1, 0])
    return hybrid


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
