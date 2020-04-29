import dartpy as dart

class ContactVisualizingNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, pointCloudShape):
        super(ContactVisualizingNode, self).__init__(world)
        self._pointCloudShape = pointCloudShape
        # self._collisionOptions = dart.collision.CollisionOption(enableContact=False, maxNumContacts=100)
        # self._collisionResult = dart.collision.CollisionResult()

    
    def customPreRefresh(self):
        contactPoints = self._getListOfContactPoints()
        self._pointCloudShape.setPoint(contactPoints)

    def _getListOfContactPoints(self):
        return [c.point for c in self.getWorld().getLastCollisionResult().getContacts()]

def setAlpha(skeleton, alphaValue):
    for bodyNode in skeleton.getBodyNodes():
        bodyNode.getShapeNode(0).getVisualAspect().setAlpha(alphaValue)

def main():
    world = dart.simulation.World()

    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    setAlpha(kr5, 0.2) # set transparency, so that "inner" contact points will be visible
    ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    setAlpha(ground, 0.2)
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    # add point cloud shape for visualizing contacts
    pointCloudShape = dart.dynamics.PointCloudShape(0.02)
    pointCloudShape.setDataVariance(dart.dynamics.Shape.DataVariance.DYNAMIC)
    pointCloudShape.setColorMode(dart.dynamics.PointCloudShape.ColorMode.BIND_OVERALL)
    pointCloudShape.setOverallColor([1.0, 0.0, 0.0, 1.0]) # red, no transparency

    pointCloudSimpleFrame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "ContactsVisualization")
    world.addSimpleFrame(pointCloudSimpleFrame)

    # Create world node and add it to viewer
    node = ContactVisualizingNode(world, pointCloudShape)

    # create a viewer with background color (red, green, blue, alpha), here: white
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