"""Visualization > contacts_pointcloud.

Straight port of ``python/examples/contacts_pointcloud/main.py``: a
transparent KR5 arm and ground plane whose live collision contacts are
rendered as a red point cloud, refreshed every frame via customPreRefresh.
No C++ demos scene counterpart (the C++ catalog's HAVE_OCTOMAP-gated
``point_cloud`` scene renders a sensor point cloud/voxel grid, not contact
points).
"""

import dartpy as dart

from ..registry import SceneHandle


class ContactsPointCloudNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, point_cloud_shape):
        super().__init__(world)
        self._point_cloud_shape = point_cloud_shape

    def customPreRefresh(self):
        contacts = self.getWorld().getLastCollisionResult().getContacts()
        self._point_cloud_shape.setPoint([c.point for c in contacts])


def build() -> SceneHandle:
    world = dart.simulation.World()

    urdf_parser = dart.utils.DartLoader()
    kr5 = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    kr5.setAlpha(0.3)
    ground = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    ground.setAlpha(0.3)
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    point_cloud_shape = dart.dynamics.PointCloudShape(0.02)
    point_cloud_shape.setDataVariance(dart.dynamics.Shape.DataVariance.DYNAMIC)

    point_cloud_frame = dart.dynamics.SimpleFrame(
        dart.dynamics.Frame.World(), "ContactsVisualization"
    )
    point_cloud_frame.setShape(point_cloud_shape)
    point_cloud_frame.createVisualAspect().setRGBA([0.7, 0, 0, 1])
    world.addSimpleFrame(point_cloud_frame)

    node = ContactsPointCloudNode(world, point_cloud_shape)

    return SceneHandle(
        node=node,
        grid=True,
        camera_home=([2.0, 1.0, 2.0], [0.0, 0.0, 0.0], [-0.24, 0.94, -0.25]),
        notes="Press space to start the simulation -- contacts only appear while it runs.",
    )
