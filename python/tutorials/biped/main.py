import dartpy as dart
import numpy as np

DEFAULT_FORCE = 40.0
DEFAULT_COUNTDOWN = 100


class InputHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, node):
        super(InputHandler, self).__init__()
        self.node = node
        self.force = np.zeros(3)
        self.impulse_duration = 0

    def handle(self, ea, aa):
        if ea.getEventType() != dart.gui.osg.GUIEventAdapter.KEYDOWN:
            return False

        gea = dart.gui.osg.GUIEventAdapter
        if ea.getKey() == gea.KEY_Comma:
            ext_force = np.zeros(3)
            ext_force[0] = -DEFAULT_FORCE
            self.node.set_external_force(ext_force, DEFAULT_COUNTDOWN)
            return True
        if ea.getKey() == gea.KEY_Period:
            ext_force = np.zeros(3)
            ext_force[0] = DEFAULT_FORCE
            self.node.set_external_force(ext_force, DEFAULT_COUNTDOWN)
            return True

        return False


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, skel):
        super(MyWorldNode, self).__init__(world)
        self.world = world
        self.skel = skel
        self.dofs = self.skel.getNumDofs()
        self.left_heel = self.skel.getBodyNode("h_heel_left")
        self.left_foot = [
            self.skel.getDof("j_heel_left_1").getIndexInSkeleton(),
            self.skel.getDof("j_toe_left").getIndexInSkeleton(),
        ]
        self.right_foot = [
            self.skel.getDof("j_heel_right_1").getIndexInSkeleton(),
            self.skel.getDof("j_toe_right").getIndexInSkeleton(),
        ]
        self.timestep = world.getTimeStep()
        self.Kp = np.eye(self.dofs)
        self.Kd = np.eye(self.dofs)

        self.torques = np.zeros(self.dofs)

        self.q_d = self.skel.getPositions()

        # Using SPD results in simple Kp coefficients
        for i in range(6):
            self.Kp[i, i] = 0.0
            self.Kd[i, i] = 0.0

        for i in range(6, self.dofs):
            self.Kp[i, i] = 400.0
            self.Kd[i, i] = 40.0

        self.pre_offset = 0

        self.ext_force = np.zeros(3)
        self.ext_force_duration = 0

        self.ext_force_arrow_shape = dart.dynamics.ArrowShape([0, 0, 0], [0, 0, 0])

        self.ext_force_simple_frame = dart.dynamics.SimpleFrame()
        self.ext_force_simple_frame.setShape(self.ext_force_arrow_shape)
        self.ext_force_visual = self.ext_force_simple_frame.createVisualAspect()
        self.ext_force_visual.setColor([1.0, 0.0, 0.0])
        self.ext_force_visual.hide()

        self.world.addSimpleFrame(self.ext_force_simple_frame)

    def customPreStep(self):
        # Lesson 3: Implement the SPD controller here.

        # Lesson 4: Apply the ankle strategy for balance recovery.

        # Lesson 5: Visualize and apply the external push forces.

    def set_external_force(self, force, duration=10):
        self.ext_force = force
        self.ext_force_duration = duration

    def external_force(self):
        return self.ext_force


def main():
    # Lesson 1: Load the biped model.
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    world.setGravity([0, -9.81, 0])

    biped = world.getSkeleton("fullbody1")
    # Lesson 2: Set the initial pose for the biped.
    biped.getDof("j_pelvis_rot_y").setPosition(-0.20)
    biped.getDof("j_thigh_left_z").setPosition(0.15)
    biped.getDof("j_shin_left").setPosition(-0.40)
    biped.getDof("j_heel_left_1").setPosition(0.25)
    biped.getDof("j_thigh_right_z").setPosition(0.15)
    biped.getDof("j_shin_right").setPosition(-0.40)
    biped.getDof("j_heel_right_1").setPosition(0.25)
    biped.getDof("j_abdomen_2").setPosition(0.00)

    node = MyWorldNode(world, biped)

    # Create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    input_handler = InputHandler(node)
    viewer.addEventHandler(input_handler)

    viewer.addInstructionText("space bar: simulation on/off")
    viewer.addInstructionText("'p': replay simulation")
    viewer.addInstructionText("'.': apply a forward push")
    viewer.addInstructionText("',': apply a backward push")
    print(viewer.getInstructions())

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([3, 1.5, 3], [0, 0, 0], [0, 0, 0])
    viewer.run()


if __name__ == "__main__":
    main()
