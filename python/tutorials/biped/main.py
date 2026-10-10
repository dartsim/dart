"""Biped exercises. Complete the seven numbered TODO lessons.

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
        # TODO Lesson 2: add proportional and derivative torques to self.forces.
        pass

    def add_spd_forces(self):
        # TODO Lesson 3: compute stable PD torques using the mass matrix and timestep.
        pass

    def add_ankle_strategy_forces(self):
        # TODO Lesson 4: add heel/toe torques from COM/COP position and velocity errors.
        pass

    def set_wheel_commands(self):
        # TODO Lesson 6: zero wheel force-control gains and command wheel velocities.
        pass

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
    # TODO Lesson 1: enforce joint limits, enable self-collision,
    # and disable collision checking for adjacent bodies.
    return biped


def set_initial_pose(biped):
    # TODO Lesson 2: set thigh, shin, and heel DOFs to a crouched initial stance.
    pass


def modify_biped_with_skateboard(biped):
    # TODO Lesson 5: load skateboard.skel and move its root under the left heel
    # with an Euler joint, offsetting the child joint by [0, 0.1, 0].
    pass


def set_velocity_actuators(biped):
    # TODO Lesson 6: set each of the four wheel joints to VELOCITY actuation.
    pass


def solve_ik(biped):
    # TODO Lesson 7: use COM and foot-corner Jacobians to balance a one-foot stance.
    return biped.getPositions()


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
