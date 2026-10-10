"""Biped tutorial: PD/SPD balance, skateboard actuators, and Jacobian IK.

The sample models use Y as the vertical axis.
"""

import dartpy as dart
import numpy as np

default_speed_increment = 0.5
default_ik_iterations = 4500
default_force = 50.0  # N
default_countdown = 100


class Controller:
    def __init__(self, biped):
        self.biped = biped
        self.speed = 0.0
        dofs = biped.getNumDofs()
        self.forces = np.zeros(dofs)
        self.kp = np.diag([0.0] * 6 + [1000.0] * (dofs - 6))
        self.kd = np.diag([0.0] * 6 + [50.0] * (dofs - 6))
        self.set_target_positions(biped.getPositions())

    def set_target_positions(self, pose):
        self.target_positions = np.array(pose, copy=True)

    def clear_forces(self):
        self.forces.fill(0.0)

    def add_pd_forces(self):
        # Lesson 2: proportional position error and derivative damping.
        q = self.biped.getPositions()
        dq = self.biped.getVelocities()
        p = -self.kp @ (q - self.target_positions)
        d = -self.kd @ dq
        self.forces += p + d
        self.biped.setForces(self.forces)

    def add_spd_forces(self):
        # Lesson 3: account for the next position and acceleration implicitly.
        q = self.biped.getPositions()
        dq = self.biped.getVelocities()
        dt = self.biped.getTimeStep()
        p = -self.kp @ (q + dq * dt - self.target_positions)
        d = -self.kd @ dq
        acceleration = np.linalg.solve(
            self.biped.getMassMatrix() + self.kd * dt,
            -self.biped.getCoriolisAndGravityForces()
            + p
            + d
            + self.biped.getConstraintForces(),
        )
        self.forces += p + d - self.kd @ acceleration * dt
        self.biped.setForces(self.forces)

    def add_ankle_strategy_forces(self):
        # Lesson 4: recover sagittal pushes using heel and toe torques.
        heel = self.biped.getBodyNode("h_heel_left")
        offset = np.array([0.05, 0, 0])
        cop = heel.getTransform().multiply(offset)
        diff = self.biped.getCOM()[0] - cop[0]
        dcop = heel.getLinearVelocity(offset)
        ddiff = self.biped.getCOMLinearVelocity()[0] - dcop[0]
        gains = None
        if 0.0 <= diff < 0.1:
            gains = (200.0, 100.0, 10.0)
        elif -0.2 < diff < -0.05:
            gains = (2000.0, 100.0, 100.0)
        if gains is not None:
            k1, k2, kd = gains
            for name, gain in (
                ("j_heel_left_1", k1),
                ("j_heel_right_1", k1),
                ("j_toe_left", k2),
                ("j_toe_right", k2),
            ):
                index = self.biped.getDof(name).getIndexInSkeleton()
                self.forces[index] += -gain * diff - kd * ddiff
        self.biped.setForces(self.forces)

    def set_wheel_commands(self):
        # Lesson 6: exclude wheel DOFs from force control, then command velocity.
        first = self.biped.getDof("joint_front_left_1").getIndexInSkeleton()
        self.kp[first:, first:] = 0.0
        self.kd[first:, first:] = 0.0
        for name in (
            "joint_front_left_2",
            "joint_front_right_2",
            "joint_back_left",
            "joint_back_right",
        ):
            self.biped.getDof(name).setCommand(self.speed)

    def change_wheel_speed(self, increment):
        self.speed += increment
        print(f"wheel speed = {self.speed}")


class BipedEventHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, world, controller):
        super().__init__()
        self.world = world
        self.controller = controller
        self.force_countdown = 0
        self.positive_sign = True

    def handle(self, event, action):
        if event.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False
        key = event.getKey()
        if key in (ord(","), ord(".")):
            self.force_countdown = default_countdown
            self.positive_sign = key == ord(".")
        elif key in (ord("a"), ord("A")):
            self.controller.change_wheel_speed(default_speed_increment)
        elif key in (ord("s"), ord("S")):
            self.controller.change_wheel_speed(-default_speed_increment)
        else:
            return False
        return True

    def update(self):
        self.controller.clear_forces()
        self.controller.add_spd_forces()
        self.controller.add_ankle_strategy_forces()
        self.controller.set_wheel_commands()
        if self.force_countdown > 0:
            body = self.world.getSkeleton("biped").getBodyNode("h_abdomen")
            body.setColor([1, 0, 0])
            sign = 1 if self.positive_sign else -1
            body.addExtForce([sign * default_force, 0, 0], body.getCOM(), False, False)
            self.force_countdown -= 1


class CustomWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, controller, handler):
        super().__init__(world)
        self.world = world
        self.controller = controller
        self.handler = handler

    def customPreStep(self):
        self.handler.update()


def load_biped():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/biped.skel")
    if world is None or world.getSkeleton("biped") is None:
        raise RuntimeError("Could not load dart://sample/skel/biped.skel")
    biped = world.getSkeleton("biped")
    # Lesson 1: enforce limits and ignore adjacent bodies in self-collision.
    for i in range(biped.getNumJoints()):
        biped.getJoint(i).setLimitEnforcement(True)
    biped.enableSelfCollisionCheck()
    biped.disableAdjacentBodyCheck()
    return biped


def set_initial_pose(biped):
    # Lesson 2: begin with a crouched stance.
    for name, position in (
        ("j_thigh_left_z", 0.15),
        ("j_thigh_right_z", 0.15),
        ("j_shin_left", -0.4),
        ("j_shin_right", -0.4),
        ("j_heel_left_1", 0.25),
        ("j_heel_right_1", 0.25),
    ):
        biped.getDof(name).setPosition(position)


def modify_biped_with_skateboard(biped):
    # Lesson 5: move the skateboard tree under the biped's left heel.
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/skateboard.skel")
    if world is None or world.getNumSkeletons() == 0:
        raise RuntimeError("Could not load dart://sample/skel/skateboard.skel")
    skateboard = world.getSkeleton(0)
    properties = dart.dynamics.EulerJointProperties()
    child_transform = dart.math.Isometry3()
    child_transform.set_translation([0, 0.1, 0])
    properties.mT_ChildBodyToJoint = child_transform
    skateboard.getRootBodyNode().moveToEulerJoint(
        biped.getBodyNode("h_heel_left"), properties
    )


def set_velocity_actuators(biped):
    # Lesson 6
    for name in (
        "joint_front_left",
        "joint_front_right",
        "joint_back_left",
        "joint_back_right",
    ):
        biped.getJoint(name).setActuatorType(dart.dynamics.ActuatorType.VELOCITY)


def solve_ik(biped):
    # Lesson 7: balance COM over the left foot while keeping its corners level.
    biped.getDof("j_shin_right").setPosition(-1.4)
    biped.getDof("j_bicep_left_x").setPosition(0.8)
    biped.getDof("j_bicep_right_x").setPosition(-0.8)
    new_pose = biped.getPositions()
    heel = biped.getBodyNode("h_heel_left")
    toe = biped.getBodyNode("h_toe_left")
    initial_height = -0.8
    for _ in range(default_ik_iterations):
        deviation = biped.getCOM() - heel.getCOM()
        jacobian = biped.getCOMLinearJacobian() - biped.getLinearJacobian(
            heel, heel.getCOM(heel)
        )
        direction = -0.2 * deviation[0] * jacobian[0]
        direction += -0.2 * deviation[2] * jacobian[2]
        for body, offset in (
            (heel, [0, -0.04, -0.03]),
            (heel, [0, -0.04, 0.03]),
            (toe, [0.04, -0.04, 0.03]),
            (toe, [0.04, -0.04, -0.03]),
        ):
            error = body.getTransform().multiply(offset)[1] - initial_height
            gradient = biped.getLinearJacobian(body, offset)[1]
            direction += -0.2 * error * gradient
        new_pose += direction
        biped.setPositions(new_pose)
        biped.computeForwardKinematics(True, False, False)
    return new_pose


def create_floor():
    floor = dart.dynamics.Skeleton("floor")
    joint, body = floor.createWeldJointAndBodyNodePair()
    shape_node = body.createShapeNode(dart.dynamics.BoxShape([10, 0.01, 10]))
    shape_node.createVisualAspect().setColor([0, 0, 0])
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    transform = dart.math.Isometry3()
    transform.set_translation([0, -1, 0])
    joint.setTransformFromParentBodyNode(transform)
    return floor


def build_scene():
    floor = create_floor()
    biped = load_biped()
    set_initial_pose(biped)
    modify_biped_with_skateboard(biped)
    set_velocity_actuators(biped)
    biped.setPositions(solve_ik(biped))
    world = dart.simulation.World()
    world.setGravity([0, -9.81, 0])
    if hasattr(dart.collision, "BulletCollisionDetector"):
        world.getConstraintSolver().setCollisionDetector(
            dart.collision.BulletCollisionDetector()
        )
    world.addSkeleton(floor)
    world.addSkeleton(biped)
    controller = Controller(biped)
    handler = BipedEventHandler(world, controller)
    return CustomWorldNode(world, controller, handler)


def main():
    node = build_scene()
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.addEventHandler(node.handler)
    viewer.addInstructionText(
        "Space: simulation on/off\n"
        ".: forward push; ,: backward push\n"
        "a/A: increase wheel speed; s/S: decrease wheel speed\n"
    )
    print(viewer.getInstructions())
    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([5, 3, 3], [0, 0, 0], [0, 1, 0])
    viewer.run()


if __name__ == "__main__":
    main()
