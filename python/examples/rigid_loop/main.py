import math
import numpy as np
import dartpy as dart


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, chain):
        super(MyWorldNode, self).__init__(world)
        self.chain = chain

    def customPreStep(self):
        pass


def main():
    world = dart.utils.SkelParser.readWorld('dart://sample/skel/chain.skel')
    world.setGravity([0, -9.81, 0])
    world.setTimeStep(1.0/2000)

    chain = world.getSkeleton(0)
    for i in range(chain.getNumJoints()):
        joint = chain.getJoint(i)
        for j in range(joint.getNumDofs()):
            joint.setDampingCoefficient(j, 0.01)

    dof = chain.getNumDofs()

    init_pose = np.zeros(dof)
    init_pose[20] = math.pi * 0.4
    init_pose[23] = math.pi * 0.4
    init_pose[26] = math.pi * 0.4
    init_pose[29] = math.pi * 0.4
    chain.setPositions(init_pose)

    # Create a ball joint contraint
    bd1 = chain.getBodyNode('link 6')
    bd2 = chain.getBodyNode('link 10')
    bd1.setColor([0, 1, 0])
    bd2.setColor([0, 1, 0])

    offset = [0, 0.025, 0]
    joint_pos = bd1.getTransform().multiply(offset)
    constraint = dart.constraint.BallJointConstraint(bd1, bd2, joint_pos)
    constraint_solver = world.getConstraintSolver()
    constraint_solver.addConstraint(constraint)

    node = MyWorldNode(world, chain)

    # Create world node and add it to viewer
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    # Grid settings
    grid = dart.gui.osg.GridVisual()
    grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.ZX)
    grid.setOffset([0, -0.55, 0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([0.6, 0.3, 0.6], [0, -0.2, 0], [0, 1, 0])
    viewer.run()


if __name__ == "__main__":
    main()
