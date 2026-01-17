#
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

import math
import random
from typing import List, Optional, Tuple

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
    # snippet:py-collisions-lesson4a-ring-stiffness-start
    dof_count = ring.getNumDofs()
    for idx in range(6, dof_count):
        dof = ring.getDof(idx)
        dof.setSpringStiffness(ring_spring_stiffness)
        dof.setDampingCoefficient(ring_damping_coefficient)
    # snippet:py-collisions-lesson4a-ring-stiffness-end

    # snippet:py-collisions-lesson4b-ring-rest-start
    num_edges = ring.getNumBodyNodes()
    angle = 2 * math.pi / num_edges

    for i in range(1, ring.getNumJoints()):
        joint = ring.getJoint(i)
        rotation = dart.math.euler_xyz_to_matrix([0.0, angle, 0.0])
        rest = dart.dynamics.BallJoint.convertToPositions(rotation)
        for axis in range(3):
            joint.setRestPosition(axis, rest[axis])
    # snippet:py-collisions-lesson4b-ring-rest-end

    # snippet:py-collisions-lesson4c-ring-rest-state-start
    for idx in range(6, dof_count):
        dof = ring.getDof(idx)
        dof.setPosition(dof.getRestPosition())
    # snippet:py-collisions-lesson4c-ring-rest-state-end


class CollisionsEventHandler(dart.gui.GUIEventHandler):
    def __init__(
        self,
        world: dart.World,
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
        if ea.getEventType() != dart.gui.GUIEventAdapter.KEYDOWN:
            return False

        key = ea.getKey()
        gea = dart.gui.GUIEventAdapter

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

    # snippet:py-collisions-lesson3a-initial-position-start
    def _initial_position(self) -> np.ndarray:
        pos = np.zeros(6)
        if self.randomize:
            pos[4] = default_spawn_range * self.rng.uniform(-1.0, 1.0)
        pos[5] = default_start_height
        return pos

    def add_object(self, obj: dart.dynamics.Skeleton) -> bool:
        positions = self._initial_position()
        obj.getJoint(0).setPositions(positions)
    # snippet:py-collisions-lesson3a-initial-position-end

        # snippet:py-collisions-lesson3b-name-start
        obj.setName(f"{obj.getName()}_{self.spawn_index}")
        self.spawn_index += 1
        # snippet:py-collisions-lesson3b-name-end

        # snippet:py-collisions-lesson3c-collision-check-start
        constraint_solver = self.world.getConstraintSolver()
        collision_detector = constraint_solver.getCollisionDetector()
        world_group = constraint_solver.getCollisionGroup()
        new_group = collision_detector.createCollisionGroup(obj)

        option = dart.CollisionOption()
        result = dart.CollisionResult()
        collision = world_group.collide(new_group, option, result)

        if collision:
            print(
                "The new object spawned in collision and will not be added to the world."
            )
            return False

        self.world.addSkeleton(obj)
        self._launch_object(obj)
        # snippet:py-collisions-lesson3c-collision-check-end
        return True

    def _launch_object(self, obj: dart.dynamics.Skeleton):
        # snippet:py-collisions-lesson3d-reference-frame-start
        center_tf = dart.math.Isometry3()
        center_tf.set_translation(obj.getCOM())
        center = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "center", center_tf
        )
        # snippet:py-collisions-lesson3d-reference-frame-end

        # snippet:py-collisions-lesson3e-launch-velocity-start
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
        # snippet:py-collisions-lesson3e-launch-velocity-end

        # snippet:py-collisions-lesson3f-apply-velocity-start
        ref = dart.dynamics.SimpleFrame(center, "root_reference")
        ref.setRelativeTransform(
            obj.getBodyNode(0).getTransform(center)
        )
        obj.getJoint(0).setVelocities(ref.getSpatialVelocity())
        # snippet:py-collisions-lesson3f-apply-velocity-end

    def add_ring(self, ring: dart.dynamics.Skeleton):
        setup_ring(ring)
        if not self.add_object(ring):
            return

        # snippet:py-collisions-lesson5-closed-chain-start
        head = ring.getBodyNode(0)
        tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
        offset = np.array([0.0, 0.0, default_shape_height / 2.0])
        offset = tail.getWorldTransform().multiply(offset)
        constraint = dart.constraint.BallJointConstraint(head, tail, offset)
        self.world.getConstraintSolver().addConstraint(constraint)
        self.joint_constraints.append(constraint)
        # snippet:py-collisions-lesson5-closed-chain-end

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


class CustomWorldNode(dart.gui.RealTimeWorldNode):
    def __init__(self, world):
        super().__init__(world)


def add_rigid_body(
    skeleton: dart.dynamics.Skeleton,
    joint_type: str,
    name: str,
    shape_type: str,
    parent: Optional[dart.dynamics.BodyNode] = None,
) -> dart.dynamics.BodyNode:
    # snippet:py-collisions-lesson1a-properties-start
    joint_prop = None
    joint = None

    if joint_type == "free":
        joint_prop = dart.dynamics.FreeJointProperties()
    elif joint_type == "ball":
        joint_prop = dart.dynamics.BallJointProperties()
    elif joint_type == "revolute":
        joint_prop = dart.dynamics.RevoluteJointProperties()
        joint_prop.mAxis = [0.0, 1.0, 0.0]
    else:
        raise ValueError(f"Unsupported joint type: {joint_type}")

    joint_prop.mName = f"{name}_joint"
    if parent is not None:
        parent_tf = dart.math.Isometry3()
        parent_tf.set_translation([0.0, 0.0, default_shape_height / 2.0])
        child_tf = dart.math.Isometry3()
        child_tf.set_translation([0.0, 0.0, -default_shape_height / 2.0])
        joint_prop.mT_ParentBodyToJoint = parent_tf
        joint_prop.mT_ChildBodyToJoint = child_tf
    # snippet:py-collisions-lesson1a-properties-end

    body_prop = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(name)
    )

    # snippet:py-collisions-lesson1b-joint-pair-start
    if joint_type == "free":
        joint, body = skeleton.createFreeJointAndBodyNodePair(
            parent, joint_prop, body_prop
        )
    elif joint_type == "ball":
        joint, body = skeleton.createBallJointAndBodyNodePair(
            parent, joint_prop, body_prop
        )
    else:
        joint, body = skeleton.createRevoluteJointAndBodyNodePair(
            parent, joint_prop, body_prop
        )
    # snippet:py-collisions-lesson1b-joint-pair-end

    # snippet:py-collisions-lesson1c-shape-selection-start
    if shape_type == "box":
        dims = [default_shape_width, default_shape_width, default_shape_height]
        shape = dart.dynamics.BoxShape(dims)
    elif shape_type == "cylinder":
        shape = dart.dynamics.CylinderShape(
            default_shape_width / 2.0, default_shape_height
        )
    else:
        dims = default_shape_height * np.ones(3)
        shape = dart.dynamics.EllipsoidShape(dims)
    # snippet:py-collisions-lesson1c-shape-selection-end

    # snippet:py-collisions-lesson1c-shape-node-start
    shape_node = body.createShapeNode(shape)
    visual = shape_node.createVisualAspect()
    visual.setColor([0.8, 0.8, 0.8, 1.0])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    # snippet:py-collisions-lesson1c-shape-node-end

    # snippet:py-collisions-lesson1d-inertia-start
    inertia = dart.dynamics.Inertia()
    mass = default_shape_density * shape.getVolume()
    inertia.setMass(mass)
    inertia.setMoment(shape.computeInertia(mass))
    body.setInertia(inertia)
    # snippet:py-collisions-lesson1d-inertia-end

    # snippet:py-collisions-lesson1e-restitution-start
    shape_node.getDynamicsAspect().setRestitutionCoeff(default_restitution)
    # snippet:py-collisions-lesson1e-restitution-end

    # snippet:py-collisions-lesson1f-damping-start
    if parent is not None:
        parent_joint = body.getParentJoint()
        for idx in range(parent_joint.getNumDofs()):
            parent_joint.getDof(idx).setDampingCoefficient(
                default_damping_coefficient
            )
    # snippet:py-collisions-lesson1f-damping-end

    return body


def add_soft_body(
    skeleton: dart.dynamics.Skeleton,
    joint_type: str,
    name: str,
    shape_type: int,
    parent: Optional[dart.dynamics.BodyNode] = None,
) -> dart.dynamics.BodyNode:
    # snippet:py-collisions-lesson2b-soft-properties-start
    # dartpy does not yet expose SoftBodyNodeHelper, so we approximate the soft
    # shell with a standard BodyNode whose geometry matches the requested type.
    shape = "ellipsoid" if shape_type == SOFT_ELLIPSOID else "box"
    # snippet:py-collisions-lesson2b-soft-properties-end

    # snippet:py-collisions-lesson2c-soft-node-start
    body = add_rigid_body(skeleton, joint_type, name, shape, parent)
    # snippet:py-collisions-lesson2c-soft-node-end

    # snippet:py-collisions-lesson2d-soft-inertia-start
    inertia = dart.dynamics.Inertia()
    inertia.setMass(1e-8)
    inertia.setMoment(np.eye(3) * 1e-8)
    body.setInertia(inertia)
    # snippet:py-collisions-lesson2d-soft-inertia-end

    # snippet:py-collisions-lesson2e-soft-alpha-start
    body.setAlpha(0.4)
    # snippet:py-collisions-lesson2e-soft-alpha-end
    return body


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
    body = add_soft_body(soft, "free", "soft_core", SOFT_BOX)
    # snippet:py-collisions-lesson2f-rigid-core-start
    dims = np.array(
        [default_shape_height, default_shape_height, 2 * default_shape_width]
    )
    dims *= 0.6
    box = dart.dynamics.BoxShape(dims)
    node = body.createShapeNode(box)
    node.createVisualAspect().setColor([0.3, 0.3, 0.3, 1.0])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    inertia = dart.dynamics.Inertia()
    inertia.setMass(default_shape_density * box.getVolume())
    inertia.setMoment(box.computeInertia(inertia.getMass()))
    body.setInertia(inertia)
    # snippet:py-collisions-lesson2f-rigid-core-end
    for idx in range(2):
        body = add_soft_body(
            soft, "ball", f"soft_link_{idx}", SOFT_ELLIPSOID, body
        )
        for dof_idx in range(body.getParentJoint().getNumDofs()):
            dof = body.getParentJoint().getDof(dof_idx)
            dof.setSpringStiffness(5.0)
            dof.setDampingCoefficient(0.1)
    set_all_colors(soft, (1.0, 0.0, 1.0))
    return soft


def create_hybrid_body() -> dart.dynamics.Skeleton:
    hybrid = dart.dynamics.Skeleton("hybrid")
    body = add_soft_body(hybrid, "free", "soft_base", SOFT_ELLIPSOID)
    body = add_soft_body(hybrid, "ball", "soft_link", SOFT_BOX, body)
    # snippet:py-collisions-lesson2g-welded-rigid-start
    _, rigid = hybrid.createWeldJointAndBodyNodePair(body)
    rigid.setName("rigid_box")
    box = dart.dynamics.BoxShape(default_shape_height * np.ones(3))
    node = rigid.createShapeNode(box)
    node.createVisualAspect().setColor([0.0, 1.0, 0.0, 1.0])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    inertia = dart.dynamics.Inertia()
    inertia.setMass(default_shape_density * box.getVolume())
    inertia.setMoment(box.computeInertia(inertia.getMass()))
    rigid.setInertia(inertia)
    # snippet:py-collisions-lesson2g-welded-rigid-end
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
    world = dart.World()
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

    viewer = dart.gui.Viewer()
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

    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.XZ)
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
