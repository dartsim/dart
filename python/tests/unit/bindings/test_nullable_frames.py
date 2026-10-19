"""Nullable native arguments must not crash the Python interpreter."""

import runpy
from pathlib import Path

import pytest

from ._support import run_isolated


def test_nullable_inventory_frame_subclass_keep_list():
    inventory = runpy.run_path(
        str(Path(__file__).resolve().parents[4] / "scripts/nanobind/nullable_inventory.py")
    )["inventory"]()
    allowed = {
        ("collision.DistanceResult.shapeFrame1", "value"),
        ("collision.DistanceResult.shapeFrame2", "value"),
        ("dynamics.BodyNode.moveTo", "newParent"),
        ("dynamics.BodyNode.moveToEulerJoint", "newParent"),
        ("dynamics.BodyNode.copyTo", "newParent"),
        ("dynamics.Chain.__init__", "start"),
        ("dynamics.Chain.__init__", "target"),
        ("dynamics.ChainCriteria.__init__", "start"),
        ("dynamics.ChainCriteria.__init__", "target"),
        ("dynamics.FreeJoint.setTransformOf", "bodyNode"),
        ("dynamics.InverseKinematics.setTarget", "newTarget"),
        ("dynamics.InverseKinematicsTaskSpaceRegion.setReferenceFrame", "referenceFrame"),
        ("dynamics.InverseKinematicsTaskSpaceRegionProperties.mReferenceFrame", "value"),
        ("dynamics.InverseKinematicsTaskSpaceRegionUniqueProperties.__init__", "referenceFrame"),
        ("dynamics.InverseKinematicsTaskSpaceRegionUniqueProperties.mReferenceFrame", "value"),
        ("dynamics.LinkageCriteria.Target.__init__", "target"),
        ("dynamics.LinkageCriteria.Terminal.__init__", "terminal"),
        ("gui.PickInfo.frame", "value"),
    }
    plain_keep = {
        ("dynamics.Detachable.setParentFrame", "newParentFrame"),
        ("dynamics.Entity.descendsFrom", "someFrame"),
        ("dynamics.Joint.getWrenchToChildBodyNode", "withRespectTo"),
        ("dynamics.Joint.getWrenchToParentBodyNode", "withRespectTo"),
        ("dynamics.FreeJoint.setSpatialMotion", "withRespectTo"),
        ("dynamics.FreeJoint.setSpatialMotion", "velRelativeTo"),
        ("dynamics.FreeJoint.setSpatialMotion", "velInCoordinatesOf"),
        ("dynamics.FreeJoint.setSpatialMotion", "accRelativeTo"),
        ("dynamics.FreeJoint.setSpatialMotion", "accInCoordinatesOf"),
    }
    plain = [
        (row["path"].removeprefix("dartpy."), row["parameter"])
        for row in inventory["frame_parameters"]
        if row["type"] == "dartpy.dynamics.Frame"
    ]
    assert plain_keep <= set(plain)
    # Every other nullable Frame parameter defaults to Frame::World() in C++ and
    # maps None through dartnb::frameOrWorld; check a new one before updating.
    assert len([key for key in plain if key not in plain_keep]) == 141
    for row in inventory["frame_parameters"]:
        if row["type"] == "dartpy.dynamics.Frame":
            continue
        path = row["path"].removeprefix("dartpy.")
        if path.startswith("dynamics.Skeleton.create"):
            assert row["parameter"] == "parent"
        else:
            assert (path, row["parameter"]) in allowed, row
    paths = {row["path"] for row in inventory["frame_parameters"]}
    assert "dartpy.dynamics.LinkageCriteria.Target.__init__" in paths
    assert "dartpy.collision.DistanceResult.shapeFrame1" in paths


@pytest.mark.parametrize(
    "expression",
    [
        "body.getTransform(FRAME).matrix()",
        "body.getLinearVelocity(FRAME)",
        "body.getLinearVelocity(FRAME, FRAME)",
        "body.getLinearVelocity(np.ones(3), FRAME, FRAME)",
        "body.getAngularVelocity(FRAME, FRAME)",
        "body.getLinearAcceleration(FRAME, FRAME)",
        "skeleton.getCOM(FRAME)",
        "skeleton.getCOMSpatialVelocity(FRAME, FRAME)",
        "body.getLinearJacobian(FRAME)",
        "skeleton.getLinearJacobian(body, FRAME)",
        "skeleton.getCOMJacobian(FRAME)",
    ],
)
def test_none_means_world(expression):
    run_isolated(
        f"""
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        joint.setPositions(np.array([0.3, -0.2, 0.1, 1., 2., 3.]))
        joint.setVelocities(np.arange(6, dtype=float))
        joint.setAccelerations(np.arange(6, dtype=float) + 1)
        world = dart.dynamics.Frame.World()
        expected = {expression.replace('FRAME', 'world')}
        actual = {expression.replace('FRAME', 'None')}
        np.testing.assert_allclose(actual, expected)
        """
    )


@pytest.mark.parametrize(
    "expression",
    [
        "body.getTransform(None, world)",
        "body.getSpatialVelocity(None, world)",
        "body.getSpatialVelocity(world, None)",
        "body.getSpatialAcceleration(None, world)",
        "body.getJacobian(np.zeros(3), None)",
        "skeleton.getJacobian(body, None)",
        "dart.dynamics.MetaSkeleton.getJacobian(skeleton, body, None, world)",
        "dart.dynamics.MetaSkeleton.getJacobian(skeleton, None, body, world)",
        "skeleton.getJacobian(None, world)",
        "shape_node.copy(None)",
        "body.copy(None)",
        "body.duplicateNodes(None)",
        "body.matchNodes(None)",
        "skeleton.getIndexOf(bn=None)",
        "skeleton.hasBodyNode(None)",
        "skeleton.updateBiasImpulse(None)",
        "body.getOrCreateIK().clone(None)",
        "dart.dynamics.SoftMeshShape(None)",
        "dart.constraint.BallJointConstraint(None, np.zeros(3))",
        "dart.constraint.WeldJointConstraint(body, None)",
        "dart.simulation.World().addSimpleFrame(None)",
        "dart.simulation.World().removeSimpleFrame(None)",
        "dart.collision.DARTCollisionDetector().createCollisionGroup().removeShapeFrame(None)",
        "dart.collision.CollisionResult().inCollision(None)",
        'setattr(dart.dynamics.ContactInverseDynamics.Contact(), "bodyNode", None)',
        "frame.copy(None)",
        "frame.setRelativeSpatialVelocity(np.zeros(6), None)",
        "joint.setSpatialVelocity(np.zeros(6), None, world)",
        "dart.simulation.World().addSkeleton(None)",
        "dart.simulation.World().removeSkeleton(None)",
        "dart.collision.DARTCollisionDetector().createCollisionGroup().addShapeFrame(None)",
        "body.createShapeNode(None)",
        'body.createShapeNode(None, "shape")',
        "dart.gui.InteractiveFrame(None)",
    ],
)
def test_required_native_argument_rejects_none(expression):
    run_isolated(
        f"""
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        shape_node = body.createShapeNode(dart.dynamics.BoxShape(np.ones(3)))
        frame = dart.dynamics.SimpleFrame()
        world = dart.dynamics.Frame.World()
        try:
            {expression}
        except TypeError:
            pass
        else:
            raise AssertionError("None must raise TypeError")
        """
    )


def test_simple_frame_world_defaults_and_null_parent():
    run_isolated(
        """
        world = dart.dynamics.Frame.World()
        for args in [(None,), (None, "frame"), (None, "frame", np.eye(4))]:
            frame = dart.dynamics.SimpleFrame(*args)
            assert frame.getParentFrame().isWorld()
            frame.setTranslation(np.ones(3), None)
            np.testing.assert_allclose(frame.getRelativeTransform().translation(), np.ones(3))
            assert frame.clone(None).getParentFrame().isWorld()
            copied = dart.dynamics.SimpleFrame()
            copied.copy(frame, None)
            np.testing.assert_allclose(copied.getRelativeTransform().translation(), np.ones(3))
            assert frame.descendsFrom(None)
            frame.setParentFrame(None)
            assert frame.getParentFrame() is None
        class Sub(dart.dynamics.SimpleFrame):
            pass
        sub = Sub(None, "sub")
        assert sub.getName() == "sub" and sub.getParentFrame().isWorld()
        """
    )


def test_free_joint_spatial_motion_needs_frames_only_for_set_motions():
    run_isolated(
        """
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        world = dart.dynamics.Frame.World()
        transform = np.eye(4)
        transform[:3, 3] = [1.0, 2.0, 3.0]
        joint.setSpatialMotion(transform, world, None, None, None, None, None, None)
        np.testing.assert_allclose(body.getTransform().matrix(), transform)
        velocity = np.arange(6, dtype=float)
        joint.setSpatialMotion(None, None, velocity, world, world, None, None, None)
        np.testing.assert_allclose(body.getSpatialVelocity(world, world), velocity)
        acceleration = np.arange(6, dtype=float) + 1
        joint.setSpatialMotion(None, None, None, None, None, acceleration, world, world)
        np.testing.assert_allclose(
            body.getSpatialAcceleration(world, world), acceleration
        )
        for args, name in [
            ((transform, None, None, None, None, None, None, None), "withRespectTo"),
            ((None, None, velocity, None, world, None, None, None), "velRelativeTo"),
            (
                (None, None, None, None, None, acceleration, world, None),
                "accInCoordinatesOf",
            ),
        ]:
            try:
                joint.setSpatialMotion(*args)
            except TypeError as error:
                assert name in str(error), error
            else:
                raise AssertionError(f"a missing {name} must raise TypeError")
        """
    )


def test_referential_skeleton_and_jacobian_node_none_means_world():
    run_isolated(
        """
        skeleton = dart.dynamics.Skeleton()
        joint, start = skeleton.createFreeJointAndBodyNodePair()
        _, target = skeleton.createRevoluteJointAndBodyNodePair(start)
        joint.setPositions(np.array([0.3, -0.2, 0.1, 1., 2., 3.]))
        world = dart.dynamics.Frame.World()
        chain = dart.dynamics.Chain(start, target, False, "chain")
        for name, args in [
            ("getLinearJacobian", (target,)),
            ("getAngularJacobian", (target,)),
            ("getCOMJacobian", ()),
        ]:
            method = getattr(chain, name)
            np.testing.assert_allclose(method(*args, None), method(*args, world))
        node = dart.dynamics.JacobianNode
        np.testing.assert_allclose(
            node.getLinearJacobian(target, None), node.getLinearJacobian(target, world)
        )
        """
    )


def test_world_default_setters_and_meaningful_null_frames():
    run_isolated(
        """
        skeleton = dart.dynamics.Skeleton()
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        world = dart.dynamics.Frame.World()
        dart.dynamics.FreeJoint.setTransformOf(None, np.eye(4))
        dart.dynamics.FreeJoint.setTransformOf(None, np.eye(4), world)
        dart.dynamics.FreeJoint.setTransformOf(bodyNode=None, tf=np.eye(4))
        dart.dynamics.FreeJoint.setTransformOf(
            bodyNode=None, tf=np.eye(4), withRespectTo=None)
        joint.setTransform(np.eye(4), None)
        joint.setLinearVelocity(np.ones(3), None, None)
        np.testing.assert_allclose(body.getLinearVelocity(world, world), np.ones(3))
        np.testing.assert_allclose(
            joint.getWrenchToChildBodyNode(None), joint.getWrenchToChildBodyNode())
        np.testing.assert_allclose(
            joint.getWrenchToParentBodyNode(None), joint.getWrenchToParentBodyNode())
        properties = dart.dynamics.InverseKinematicsTaskSpaceRegionUniqueProperties(
            referenceFrame=None)
        assert properties.mReferenceFrame is None
        properties.mReferenceFrame = dart.dynamics.SimpleFrame()
        properties.mReferenceFrame = None
        ik = body.getOrCreateIK()
        ik.setTarget(None)
        assert ik.getTarget() is not None
        method = dart.dynamics.InverseKinematicsTaskSpaceRegion(ik)
        method.setReferenceFrame(dart.dynamics.SimpleFrame())
        method.setReferenceFrame(None)
        assert method.getReferenceFrame() is None
        """
    )
