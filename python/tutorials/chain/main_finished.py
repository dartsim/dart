import math

import dartpy as dart
import numpy as np

default_height = 1.0  # m
default_width = 0.2  # m
default_depth = 0.2  # m

default_torque = 15.0  # N-m
default_force = 15.0  # N
default_countdown = 200  # Number of timesteps for applying force

default_rest_position = 0.0
delta_rest_position = 10.0 * math.pi / 180.0

default_stiffness = 0.0
delta_stiffness = 10

default_damping = 5.0
delta_damping = 1.0


def set_geometry(body):
    # Create a BoxShape to be used for both visualization and collision checking
    box = dart.dynamics.BoxShape([default_width, default_depth, default_height])

    # Create a shape node for visualization and collision checking
    shape_node = body.createShapeNode(box)
    visual = shape_node.createVisualAspect()
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    visual.setColor([0, 0, 1])

    # Set the location of the shape node
    box_tf = dart.math.Isometry3()
    center = [0, 0, default_height / 2.0]
    box_tf.set_translation(center)
    shape_node.setRelativeTransform(box_tf)

    # Move the center of mass to the center of the object
    body.setLocalCOM(center)


def make_root_body(chain, name):
    joint_prop = dart.dynamics.BallJointProperties()
    joint_prop.mName = name + "_joint"
    joint_prop.mRestPositions = np.ones(3) * default_rest_position
    joint_prop.mSpringStiffnesses = np.ones(3) * default_stiffness
    joint_prop.mDampingCoefficients = np.ones(3) * default_damping

    body_aspect_prop = dart.dynamics.BodyNodeAspectProperties(name)
    body_prop = dart.dynamics.BodyNodeProperties(body_aspect_prop)

    [joint, body] = chain.createBallJointAndBodyNodePair(None, joint_prop, body_prop)

    # Make a shape for the Joint
    r = default_width
    ball = dart.dynamics.EllipsoidShape(math.sqrt(2) * np.ones(3) * r)
    shape_node = body.createShapeNode(ball)
    visual = shape_node.createVisualAspect()
    visual.setColor([0, 0, 1, 0.8])

    # Set the geometry of the BodyNode
    set_geometry(body)

    return body


def add_body(chain, parent, name):
    # Set up the properties for the Joint
    joint_prop = dart.dynamics.RevoluteJointProperties()
    joint_prop.mName = name + "_joint"
    joint_prop.mAxis = [0, 1, 0]
    joint_prop.mT_ParentBodyToJoint.set_translation([0, 0, default_height])

    # Set up the properties for the BodyNode
    body_aspect_prop = dart.dynamics.BodyNodeAspectProperties(name)
    body_prop = dart.dynamics.BodyNodeProperties(body_aspect_prop)

    # Create a new BodyNode, attached to its parent by a RevoluteJoint
    [joint, body] = chain.createRevoluteJointAndBodyNodePair(
        parent, joint_prop, body_prop
    )

    joint.setRestPosition(0, default_rest_position)
    joint.setSpringStiffness(0, default_stiffness)
    joint.setDampingCoefficient(0, default_damping)

    # Make a shape for the Joint
    r = default_width / 2.0
    h = default_depth
    cylinder = dart.dynamics.CylinderShape(r, h)

    # Line up the cylinder with the Joint axis
    tf = dart.math.Isometry3()
    angles = [math.pi / 2, 0, 0]

    rot = dart.math.eulerXYZToMatrix(angles)
    tf.set_rotation(rot)

    shape_node = body.createShapeNode(cylinder)
    visual = shape_node.createVisualAspect()
    visual.setColor([0, 0, 1, 0.8])
    shape_node.setRelativeTransform(tf)

    # Set the geometry of the Body
    set_geometry(body)

    return body


class MyWorldNode(dart.gui.RealTimeWorldNode):
    def __init__(self, world):
        super(MyWorldNode, self).__init__(world)
        pass

    def customPreStep(self):
        pass


def main():
    # Create an empty Skeleton with the name 'chain'
    chain = dart.dynamics.Skeleton("chain")

    # Add each body to the last BodyNode in the pendulum
    body = make_root_body(chain, "body1")
    body = add_body(chain, body, "body2")
    body = add_body(chain, body, "body3")
    body = add_body(chain, body, "body4")
    body = add_body(chain, body, "body5")

    # Set the initial position of the first DegreeOfFreedom so that the pendulum
    # starts to swing right away
    chain.setPosition(1, 120 * math.pi / 180.0)

    world = dart.simulation.World()
    world.addSkeleton(chain)

    node = MyWorldNode(world)

    # Create world node and add it to viewer
    viewer = dart.gui.Viewer()
    viewer.addWorldNode(node)

    # Grid settings
    grid = dart.gui.GridVisual()
    grid.setPlaneType(dart.gui.GridVisual.PlaneType.XY)
    grid.setOffset([0, 0, -5])
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(0, 0, 1280, 760)
    viewer.setCameraHomePosition([8.0, 8.0, 4.0], [0, 0, -2.5], [0, 0, 1])
    viewer.run()


if __name__ == "__main__":
    main()
