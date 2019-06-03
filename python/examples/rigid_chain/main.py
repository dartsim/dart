import numpy as np
import dartpy as dart


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world):
        super(MyWorldNode, self).__init__(world)

    def customPreStep(self):
        pass


def main():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/chain.skel")
    world.setGravity([0, -9.81, 0])

    chain = world.getSkeleton(0)
    dof = chain.getNumDofs()

    # Set initial pose
    init_pose = np.zeros(dof)
    for i in range(dof):
        init_pose[i] = dart.math.Random.uniform(-0.5, 0.5)
    chain.setPositions(init_pose)

    # Set joint dampings
    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    node = MyWorldNode(world)

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    # Grid settings
    grid = dart.gui.osg.GridVisual()
    grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.ZX)
    grid.setOffset([0, -0.55, 0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([0.8, 0.0, 0.8], [0, -0.25, 0], [0, 0.5, 0])
    viewer.run()


if __name__== "__main__":
    main()
