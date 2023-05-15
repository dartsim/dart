import dartpy as dart
import numpy as np


class HelloWorldNode(dart.gui.RealTimeWorldNode):
    def __init__(self, world, kr5):
        super(HelloWorldNode, self).__init__(world)
        self.kr5 = kr5
        self.dofs = self.kr5.getNumDofs()
        self.ee = kr5.getBodyNode("palm")
        self.Kp = np.eye(3) * 50.0
        self.Kd = np.eye(self.dofs) * 5.0
        self.ee_offset = [0.05, 0, 0]

        tf = self.ee.getTransform()
        tf.pretranslate(self.ee_offset)
        self.target = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), "target", tf
        )

    def customPreStep(self):
        M = self.kr5.getMassMatrix()

        J = self.ee.getLinearJacobian()
        Jt = J.transpose()
        JJt = np.matmul(J, Jt)
        kI = 0.0025 * np.eye(3)
        invJ = np.matmul(Jt, np.linalg.inv(JJt + kI))

        dJ = self.ee.getLinearJacobianDeriv()
        dJt = dJ.transpose()
        dJdJt = np.matmul(dJ, dJt)
        invdJ = np.matmul(dJt, np.linalg.inv(dJdJt + kI))

        e = (
            self.target.getTransform().translation()
            - self.ee.getTransform().translation()
        )
        de = -self.ee.getLinearVelocity()

        cg = self.kr5.getCoriolisAndGravityForces()

        tmp1 = np.matmul(np.matmul(invJ, self.Kp), de)
        tmp2 = np.matmul(np.matmul(invdJ, self.Kp), e)

        forces1 = np.matmul(M, tmp1 + tmp2)
        forces2 = cg
        forces3 = np.matmul(np.matmul(np.matmul(self.Kd, invJ), self.Kp), e)

        forces = forces1 + forces2 + forces3

        self.kr5.setForces(forces)


def main():
    world = dart.simulation.World()

    urdfParser = dart.io.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    node = HelloWorldNode(world, kr5)

    # Create world node and add it to viewer
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)

    # Grid settings
    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.ZX)
    grid.setOffset([0, -0.55, 0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition(
        [2.0, 1.0, 2.0], [0.00, 0.00, 0.00], [-0.24, 0.94, -0.25]
    )
    viewer.run()


if __name__ == "__main__":
    main()
