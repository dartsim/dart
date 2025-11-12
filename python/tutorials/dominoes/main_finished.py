import math
from typing import List, Optional

import dartpy as dart
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
        self.end_effector = self.manipulator.getBodyNode(
            self.manipulator.getNumBodyNodes() - 1
        )
        self.offset = np.array(
            [default_endeffector_offset, 0.0, 0.0], dtype=float
        )

        self.q_desired = self.manipulator.getPositions().copy()
        self.target = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "target"
        )

        target_tf = dart.math.Isometry3()
        target_tf.set_translation(
            [0.0, 0.0, default_domino_height / 2.0]
        )
        relative = self.end_effector.getTransform(domino.getBodyNode(0))
        target_tf.set_rotation(relative.rotation())
        self.target.setTransform(target_tf, domino.getBodyNode(0))

        self.kp_pd = 200.0
        self.kd_pd = 20.0
        self.kp_os = 5.0
        self.kd_os = 0.01
        self.forces = np.zeros(self.manipulator.getNumDofs())

    def set_pd_forces(self):
        if self.manipulator is None:
            return

        dt = self.manipulator.getTimeStep()
        q = self.manipulator.getPositions().copy()
        dq = self.manipulator.getVelocities()
        q += dq * dt
        q_err = self.q_desired - q
        dq_err = -dq
        cg = self.manipulator.getCoriolisAndGravityForces()
        mass = self.manipulator.getMassMatrix()

        self.forces = mass @ (self.kp_pd * q_err + self.kd_pd * dq_err) + cg
        self.manipulator.setForces(self.forces)

    def set_operational_space_forces(self):
        if self.manipulator is None:
            return

        mass = self.manipulator.getMassMatrix()
        J = self.end_effector.getWorldJacobian(self.offset)
        JJt = J @ J.T + 0.0025 * np.eye(6)
        pinv_J = J.T @ np.linalg.inv(JJt)

        dJ = self.end_effector.getJacobianClassicDeriv(self.offset)
        dJdJt = dJ @ dJ.T + 0.0025 * np.eye(6)
        pinv_dJ = dJ.T @ np.linalg.inv(dJdJt)

        target_tf = self.target.getWorldTransform()
        end_tf = self.end_effector.getWorldTransform()
        translation_error = (
            target_tf.translation() - end_tf.multiply(self.offset)
        )

        relative = self.target.getTransform(self.end_effector).rotation()
        aa = dart.math.AngleAxis(relative)
        axis = np.asarray(aa.axis()).reshape(3)
        angular_error = axis * aa.angle()

        e = np.zeros(6)
        e[:3] = angular_error
        e[3:] = translation_error

        de = -self.end_effector.getSpatialVelocity(
            self.offset, self.target, dart.dynamics.Frame.World()
        )

        cg = self.manipulator.getCoriolisAndGravityForces()
        Kp = self.kp_os * np.eye(6)
        dofs = self.manipulator.getNumDofs()
        Kd = self.kd_os * np.eye(dofs)

        f_desired = np.zeros(6)
        f_desired[3] = default_push_force
        feedforward = J.T @ f_desired
        dq = self.manipulator.getVelocities()

        self.forces = (
            mass @ (pinv_J @ (Kp @ de) + pinv_dJ @ (Kp @ e))
            - Kd @ dq
            + Kd @ (pinv_J @ (Kp @ e))
            + cg
            + feedforward
        )
        self.manipulator.setForces(self.forces)


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
            force = default_push_force * np.array([1.0, 0.0, 0.0])
            location = np.array([0.0, 0.0, default_domino_height / 2.0])
            self.first_domino.getBodyNode(0).addExtForce(force, location)
            self.force_countdown -= 1

        if self.push_countdown > 0:
            self.controller.set_operational_space_forces()
            self.push_countdown -= 1
        else:
            self.controller.set_pd_forces()

    def attempt_to_create_domino(self, angle: float):
        new_domino = self.first_domino.clone(
            f"domino_{len(self.dominoes) + 1}"
        )

        last_domino = (
            self.dominoes[-1] if self.dominoes else self.first_domino
        )
        displacement = default_distance * np.array(
            [math.cos(self.total_angle), math.sin(self.total_angle), 0.0]
        )

        pose = last_domino.getPositions().copy()
        pose[3:] += displacement
        pose[2] = self.total_angle + angle
        new_domino.setPositions(pose)

        solver = self.world.getConstraintSolver()
        collision_group = solver.getCollisionGroup()
        collision_detector = solver.getCollisionDetector()
        new_group = collision_detector.createCollisionGroup(new_domino)

        collision_group.removeShapeFramesOf(self.floor)
        domino_collision = collision_group.collide(new_group)
        collision_group.addShapeFramesOf(self.floor)

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

    def delete_last_domino(self):
        if not self.dominoes:
            return
        last_domino = self.dominoes.pop()
        self.world.removeSkeleton(last_domino)
        self.total_angle -= self.angles.pop()


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
    loader = dart.utils.DartLoader()
    manipulator = loader.parseSkeleton(
        "dart://sample/urdf/KR5/KR5 sixx R650.urdf"
    )
    manipulator.setName("manipulator")

    base_tf = dart.math.Isometry3()
    base_tf.set_translation([-0.65, 0.0, 0.0])
    manipulator.getJoint(0).setTransformFromParentBodyNode(base_tf)

    manipulator.getDof(1).setPosition(math.radians(140.0))
    manipulator.getDof(2).setPosition(math.radians(-140.0))
    return manipulator


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
