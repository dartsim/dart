import dartpy as dart
import numpy as np


class MyWorldNode(dart.gui.RealTimeWorldNode):
    def __init__(self, world):
        super(MyWorldNode, self).__init__(world)

    def customPreStep(self):
        pass


def main():
    world = dart.io.SkelParser.readWorld("dart://sample/skel/chain.skel")
    world.setGravity([0, -9.81, 0])

    chain = world.getSkeleton(0)
    dof = chain.getNumDofs()

    # Set initial pose
    init_pose = np.zeros(dof)
    init_pose = dart.math.UniformFloat(dof, -0.5, 0.5)
    chain.setPositions(init_pose)

    # Set joint dampings
    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    node = MyWorldNode(world)

    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)

    # Grid settings
    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.ZX)
    grid.setOffset([0, -0.55, 0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([0.8, 0.0, 0.8], [0, -0.25, 0], [0, 0.5, 0])
    viewer.run()


if __name__ == "__main__":
    main()
