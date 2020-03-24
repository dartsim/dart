import numpy as np
import dartpy as dart


class InputHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, node):
        super(InputHandler, self).__init__()
        self.node = node
        self.force = np.zeros(3)
        self.impulse_duration = 0

    def handle(self, ea, aa):
        if ea.getEventType() == dart.gui.osg.GUIEventAdapter.KEYDOWN:
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_1:
                ext_force = np.zeros(3)
                ext_force[0] = 40
                self.node.set_external_force(ext_force, 100)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_2:
                ext_force = np.zeros(3)
                ext_force[0] = -40
                self.node.set_external_force(ext_force, 100)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_3:
                ext_force = np.zeros(3)
                ext_force[2] = 40
                self.node.set_external_force(ext_force, 100)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_4:
                ext_force = np.zeros(3)
                ext_force[2] = -40
                self.node.set_external_force(ext_force, 100)
                return True

        return False


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, skel):
        super(MyWorldNode, self).__init__(world)
        self.world = world
        self.skel = skel
        self.dofs = self.skel.getNumDofs()
        self.left_heel = self.skel.getBodyNode('h_heel_left')
        self.left_foot = [
            self.skel.getDof('j_heel_left_1').getIndexInSkeleton(),
            self.skel.getDof('j_toe_left').getIndexInSkeleton(),
        ]
        self.right_foot = [
            self.skel.getDof('j_heel_right_1').getIndexInSkeleton(),
            self.skel.getDof('j_toe_right').getIndexInSkeleton(),
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
        q = self.skel.getPositions();
        dq = self.skel.getVelocities();
        constraint_forces = self.skel.getConstraintForces()

        # SPD tracking
        invM = np.linalg.inv(self.skel.getMassMatrix() + self.Kd * self.timestep)
        p = np.matmul(-self.Kp, q + dq * self.timestep - self.q_d)
        d = np.matmul(-self.Kd, dq)
        ddq = np.matmul(invM, -self.skel.getCoriolisAndGravityForces() + p + d + constraint_forces)

        self.torques = p + d + np.matmul(-self.Kd, ddq) * self.timestep

        # Ankle strategy for sagital plane
        com = self.skel.getCOM()
        cop = self.left_heel.getTransform().multiply([0.05, 0, 0])

        offset = com[0] - cop[0]
        if offset < 0.1 and offset > 0.0:
             k1 = 200
             k2 = 100
             kd = 10
             self.torques[self.left_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
             self.torques[self.left_foot[1]] += -k2 * offset + kd * (self.pre_offset - offset)
             self.torques[self.right_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
             self.torques[self.right_foot[1]] += -k2 * offset + kd * (self.pre_offset - offset)
        elif offset > -0.2 and offset < -0.05:
            k1 = 2000
            k2 = 100
            kd = 100
            self.torques[self.left_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
            self.torques[self.left_foot[1]] += -k2 * offset + kd * (self.pre_offset - offset)
            self.torques[self.right_foot[0]] += -k1 * offset + kd * (self.pre_offset - offset)
            self.torques[self.right_foot[1]] += -k2 * offset + kd * (self.pre_offset - offset)

        # Just to make sure no illegal torque is used
        for i in range(6):
            self.torques[i] = 0

        self.skel.setForces(self.torques * 0.8)

        # Apply external force
        self.ext_force_duration = self.ext_force_duration - 1
        if self.ext_force_duration <= 0:
            self.ext_force_duration = 0
            self.ext_force = np.zeros(3)
        spine = self.skel.getBodyNode('h_spine')
        spine.addExtForce(self.ext_force)
        if self.ext_force_duration > 0:
            arrow_head = spine.getTransform().translation()
            arrow_tail = arrow_head - self.ext_force / 30
            self.ext_force_arrow_shape.setPositions(arrow_tail, arrow_head)
            self.ext_force_arrow_shape.setDataVariance(dart.dynamics.Shape.DYNAMIC)
            self.ext_force_visual.show()
        else:
            self.ext_force_arrow_shape.setDataVariance(dart.dynamics.Shape.STATIC)
            self.ext_force_visual.hide()

    def set_external_force(self, force, duration=10):
        self.ext_force = force
        self.ext_force_duration = duration

    def external_force(self):
        return self.ext_force


def main():
    world = dart.utils.SkelParser.readWorld('dart://sample/skel/fullbody1.skel')
    world.setGravity([0, -9.81, 0])

    biped = world.getSkeleton('fullbody1')
    biped.getDof('j_pelvis_rot_y').setPosition(-0.20)
    biped.getDof('j_thigh_left_z').setPosition(0.15)
    biped.getDof('j_shin_left').setPosition(-0.40)
    biped.getDof('j_heel_left_1').setPosition(0.25)
    biped.getDof('j_thigh_right_z').setPosition(0.15)
    biped.getDof('j_shin_right').setPosition(-0.40)
    biped.getDof('j_heel_right_1').setPosition(0.25)
    biped.getDof('j_abdomen_2').setPosition(0.00)

    node = MyWorldNode(world, biped)

    # Create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    input_handler = InputHandler(node)
    viewer.addEventHandler(input_handler)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([3, 1.5, 3], [0, 0, 0], [0, 0, 0])
    viewer.run()


if __name__ == "__main__":
    main()
