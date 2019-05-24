import numpy as np
import dartpy as dart


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world):
        super(MyWorldNode, self).__init__(world)

    def customPreStep(self):
        pass


def main():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/cubes.skel")
    world.setGravity([0, -9.81, 0])

    node = MyWorldNode(world)

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([0.8, 0.0, 0.8], [0, -0.25, 0], [0, 0.5, 0])
    viewer.run()


if __name__== "__main__":
    main()
