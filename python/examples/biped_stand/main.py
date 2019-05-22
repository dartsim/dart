import numpy as np
import dartpy as dart


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, skel):
        super(MyWorldNode, self).__init__(world)
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


def main():
    world = dart.utils.SkelParser.readWorld('dart://sample/skel/fullbody1.skel')
    world.setGravity([0, -9.81, 0])

    biped = world.getSkeleton('fullbody1')
    biped.getDof('j_pelvis_rot_y').setPosition(-0.20);
    biped.getDof('j_thigh_left_z').setPosition(0.15);
    biped.getDof('j_shin_left').setPosition(-0.40);
    biped.getDof('j_heel_left_1').setPosition(0.25);
    biped.getDof('j_thigh_right_z').setPosition(0.15);
    biped.getDof('j_shin_right').setPosition(-0.40);
    biped.getDof('j_heel_right_1').setPosition(0.25);
    biped.getDof('j_abdomen_2').setPosition(0.00);

    node = MyWorldNode(world, biped)

    # Create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([2.0, 1.0, 2.0],
                                 [0.00, 0.00, 0.00],
                                 [-0.24, 0.94, -0.25])
    viewer.run()


if __name__ == "__main__":
    main()
