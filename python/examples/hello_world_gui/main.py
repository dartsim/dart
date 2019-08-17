import dartpy as dart


class HelloWorldNode(dart.gui.osg.RealTimeWorldNode):
    # Use this function to execute custom code before each time that the
    # window is rendered. This function can be deleted if it does not need
    # to be used.
    def customPreRefresh(self):
        pass

    # Use this function to execute custom code after each time that the
    # window is rendered. This function can be deleted if it does not need
    # to be used.
    def customPostRefresh(self):
        pass

    # Use this function to execute custom code before each simulation time
    # step is performed. This function can be deleted if it does not need
    # to be used.
    def customPreStep(self):
        pass

    # Use this function to execute custom code after each simulation time
    # step is performed. This function can be deleted if it does not need
    # to be used.
    def customPostStep(self):
        pass


def main():
    world = dart.simulation.World()

    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    # Create world node and add it to viewer
    node = HelloWorldNode(world)

    # create a viewer with background color (red, green, blue, alpha), here: light grey
    viewer = dart.gui.osg.Viewer([1.0, 1.0, 1.0, 1.0])
    viewer.addWorldNode(node)

    # Grid settings
    grid = dart.gui.osg.GridVisual()
    grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.ZX)
    grid.setOffset([0, -0.55, 0])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 640, 480)
    viewer.setCameraHomePosition([2.0, 1.0, 2.0],
                                 [0.00, 0.00, 0.00],
                                 [-0.24, 0.94, -0.25])
    viewer.run()


if __name__ == "__main__":
    main()
