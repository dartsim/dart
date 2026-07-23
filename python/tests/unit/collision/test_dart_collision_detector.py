import dartpy as dart
import numpy as np


def _make_box_skeleton(name, size, position, mass=1.0):
    skel = dart.dynamics.Skeleton(name)
    joint, body = skel.createFreeJointAndBodyNodePair(None)
    body.setName(name)
    shape = dart.dynamics.BoxShape(np.array(size))
    shape_node = body.createShapeNode(shape)
    shape_node.createCollisionAspect()
    shape_node.createDynamicsAspect()
    moment = dart.dynamics.BoxShape.computeInertiaOf(np.array(size), mass)
    body.setInertia(dart.dynamics.Inertia(mass, np.zeros(3), moment))
    tf = dart.math.Isometry3(np.eye(3), np.array(position))
    joint.setTransform(tf)
    return skel


def test_dart_collision_detector_binding():
    detector = dart.collision.DARTCollisionDetector()
    assert detector.getType() == "dart"
    assert dart.collision.DARTCollisionDetector.getStaticType() == "dart"

    clone = detector.cloneWithoutCollisionObjects()
    assert clone.getType() == "dart"

    group = detector.createCollisionGroup()
    assert group is not None


def test_dart_collision_detector_detects_world_contacts():
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, -9.81])
    world.setTimeStep(0.001)

    ground = _make_box_skeleton("ground", (10.0, 10.0, 0.2), (0.0, 0.0, -0.1))
    ground.setMobile(False)
    world.addSkeleton(ground)
    world.addSkeleton(_make_box_skeleton("box", (0.2, 0.2, 0.2), (0.0, 0.0, 0.5)))

    world.getConstraintSolver().setCollisionDetector(
        dart.collision.DARTCollisionDetector()
    )
    assert world.getConstraintSolver().getCollisionDetector().getType() == "dart"

    for _ in range(1000):
        world.step()

    box = world.getSkeleton("box")
    positions = np.asarray(box.getPositions())
    assert np.isfinite(positions).all()
    # The box must come to rest on the ground (z ~ half box height), which
    # requires the detector to have produced support contacts.
    assert abs(positions[5] - 0.1) < 0.02
