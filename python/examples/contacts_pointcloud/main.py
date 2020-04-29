import dartpy as dart

"""
Displays Contacts between a Robot arm in a GUI as a pointcloud

Demonstrates the following dartpy functionality:
- Extracting contact points from simulation
- Drawing of a point cloud (with variable data during simulation)
- Hooking functionality to be executed in the viewer run() function
"""


class ContactVisualizingNode(dart.gui.osg.RealTimeWorldNode):
    """ WorldNode subclass, which extracts the contacts of the last simulation
    step and adds them to a pointloud for visualization.

    Note: Simulation must run for the contacts to be displayed.

     """

    def __init__(self, world, pointCloudShape):
        """
        Input:
        ------
        world : the dart.simulation.World object to visualize
        pointCloudShape : dart.dynamics.PointCloudShape to set contacts in
                          before each display refresh
        """
        super(ContactVisualizingNode, self).__init__(world)
        self._pointCloudShape = pointCloudShape

    def customPreRefresh(self):
        # here, we overload the function to add contact points to the
        # pointcloud before each GUI refresh step
        contactPoints = self._getListOfContactPoints()
        self._pointCloudShape.setPoint(contactPoints)

    def _getListOfContactPoints(self):
        return [
            c.point
            for c in self.getWorld().getLastCollisionResult().getContacts()
        ]


def setAlpha(skeleton, alphaValue):
    """ setAlpha(skeleton : dartpy.dynamics.Skeleton, alphaValue : float) -> None

    sets the transparency (alpha) value for each body node of the skeleton,
    assuming that the VisualAspect is contained within the first ShapeNode
    by convention.

    Input:
    ------
    skeleton : The skeleton to set transparency values for
    alphaValue: [0,1] transparency, where 1 is fully visible
    """
    for bodyNode in skeleton.getBodyNodes():
        bodyNode.getShapeNode(0).getVisualAspect().setAlpha(alphaValue)


def main():
    world = dart.simulation.World()

    # load KR5 robot and ground plane, set transparent:
    urdfParser = dart.utils.DartLoader()
    kr5 = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    # set transparency, so that "inner" contact points will be visible
    setAlpha(kr5, 0.3)
    ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    setAlpha(ground, 0.3)
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    # add point cloud shape for visualizing contacts
    pointCloudShape = dart.dynamics.PointCloudShape(0.02)

    # Since contact points may change during execution, dynamic data variance
    # is assumed for the pointcloud of contacts. Otherwise, OSG will not render
    # the new points.
    pointCloudShape.setDataVariance(dart.dynamics.Shape.DataVariance.DYNAMIC)

    pointCloudSimpleFrame = dart.dynamics.SimpleFrame(
        dart.dynamics.Frame.World(), "ContactsVisualization"
    )
    pointCloudSimpleFrame.setShape(pointCloudShape)
    pcVisualAspect = pointCloudSimpleFrame.createVisualAspect()
    pcVisualAspect.setRGBA([0.7, 0, 0, 1])
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
    viewer.setCameraHomePosition(
        [2.0, 1.0, 2.0], [0.00, 0.00, 0.00], [-0.24, 0.94, -0.25]
    )
    viewer.run()


if __name__ == "__main__":
    main()
