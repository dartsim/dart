"""Shared regressions for the full non-GUI migration, using each backend."""

import gc

import dartpy as dart
import numpy as np
import pytest

from ._support import IS_NANOBIND, run_isolated


@pytest.mark.parametrize("value", [True, False])
def test_bool_parameters_accept_true_and_false(value):
    skeleton = dart.dynamics.Skeleton()
    skeleton.setMobile(value)
    assert skeleton.isMobile() is value


@pytest.mark.parametrize("value", [0, 1, None, np.bool_(True), np.bool_(False)])
def test_bool_conversion_difference(value):
    skeleton = dart.dynamics.Skeleton()
    if IS_NANOBIND:
        with pytest.raises(TypeError):
            skeleton.setMobile(value)
    else:
        skeleton.setMobile(value)
        assert skeleton.isMobile() is bool(value)


def test_bytes_are_not_strings_with_nanobind():
    skeleton = dart.dynamics.Skeleton()
    skeleton.setName("unicode_name")
    assert skeleton.getName() == "unicode_name"
    if IS_NANOBIND:
        with pytest.raises(TypeError):
            skeleton.setName(b"bytes_name")
    else:
        assert skeleton.setName(b"bytes_name") == "bytes_name"


@pytest.mark.parametrize("kind", ["BoxShape", "BoundingBox"])
def test_const_eigen_view_keeps_its_owner_alive(kind):
    run_isolated(
        f"""
        if {kind!r} == "BoxShape":
            owner = dart.dynamics.BoxShape([1, 2, 3])
            view = owner.getSize()
        else:
            owner = dart.math.BoundingBox([1, 2, 3], [4, 5, 6])
            view = owner.getMin()
        assert view.flags.writeable is False
        del owner
        gc.collect()
        np.testing.assert_allclose(view, [1, 2, 3])
        try:
            view[0] = 2
        except ValueError:
            pass
        else:
            raise AssertionError("const view accepted a write")
        """
    )


@pytest.mark.parametrize("order", ["C", "F"])
@pytest.mark.parametrize("readonly", [False, True])
def test_fixed_matrix_input_copy(order, readonly):
    value = np.eye(4, order=order)
    value[:3, 3] = [1, 2, 3]
    value.flags.writeable = not readonly
    transform = dart.math.Isometry3(value)
    assert np.array_equal(transform.matrix(), value)
    transform.set_matrix(value)
    assert np.array_equal(transform.translation(), [1, 2, 3])
    assert value.flags.writeable == (not readonly)


def test_shared_collision_filter_and_fallback():
    option = dart.collision.CollisionOption()
    child = dart.collision.BodyNodeCollisionFilter()
    option.collisionFilter = child
    assert option.collisionFilter is child
    option.collisionFilter = None
    assert option.collisionFilter is None


@pytest.mark.parametrize("field", ["nearestPoint1", "nearestPoint2"])
def test_distance_result_points_use_eigen_arrays(field):
    result = dart.collision.DistanceResult()
    setattr(result, field, [1, 2, 3])
    view = getattr(result, field)
    assert isinstance(view, np.ndarray)
    np.testing.assert_array_equal(view, [1, 2, 3])
    setattr(result, field, np.array([4.0, 5.0, 6.0]))
    np.testing.assert_array_equal(view, [4, 5, 6])
    np.testing.assert_array_equal(getattr(result, field), [4, 5, 6])
    del result
    gc.collect()
    np.testing.assert_array_equal(view, [4, 5, 6])


@pytest.mark.parametrize(
    ("kind", "field", "values"),
    [
        ("FreeJointProperties", "mDofNames", [f"dof{i}" for i in range(6)]),
        ("FreeJointProperties", "mPreserveDofNames", [False] * 6),
        ("UniversalJointProperties", "mAxis", [[1, 0, 0], [0, 1, 0]]),
    ],
)
def test_fixed_array_fields_preserve_list_and_length_contract(kind, field, values):
    properties = getattr(dart.dynamics, kind)()
    setattr(properties, field, values)
    result = getattr(properties, field)
    assert type(result) is list
    assert len(result) == len(values)
    with pytest.raises(TypeError):
        setattr(properties, field, values + values[:1])


def test_returned_joint_property_array_survives_temporary():
    skeleton = dart.dynamics.Skeleton()
    joint, body = skeleton.createUniversalJointAndBodyNodePair()
    joint.setAxis1([0, 0, 1])
    joint.setAxis2([0, 1, 0])
    axes = joint.getUniversalJointProperties().mAxis
    gc.collect()
    assert np.allclose(axes[0], [0, 0, 1])
    assert np.allclose(axes[1], [0, 1, 0])
    assert body.getSkeleton() is skeleton


def test_nullable_direct_member_arguments():
    skeleton = dart.dynamics.Skeleton()
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    assert np.array_equal(
        joint.getWrenchToChildBodyNode(None), joint.getWrenchToChildBodyNode()
    )
    region = body.getOrCreateIK().getErrorMethod()
    region.setReferenceFrame(None)
    assert region.getReferenceFrame() is None


def test_nonpolymorphic_secondary_property_fields():
    properties = dart.optimizer.GradientDescentSolverProperties()
    properties.mStepSize = 0.25
    properties.mMaxAttempts = 3
    properties.mPerturbationStep = 2
    properties.mMaxPerturbationFactor = 0.1
    properties.mMaxRandomizationStep = 0.2
    properties.mDefaultConstraintWeight = 0.5
    properties.mEqConstraintWeights = [1, 2]
    properties.mIneqConstraintWeights = [3, 4]
    assert properties.mStepSize == 0.25 and properties.mMaxAttempts == 3
    assert properties.mPerturbationStep == 2
    assert properties.mMaxPerturbationFactor == 0.1
    assert properties.mMaxRandomizationStep == 0.2
    assert properties.mDefaultConstraintWeight == 0.5
    assert np.array_equal(properties.mEqConstraintWeights, [1, 2])
    assert np.array_equal(properties.mIneqConstraintWeights, [3, 4])

    region = dart.dynamics.InverseKinematicsTaskSpaceRegionProperties()
    region.mComputeErrorFromCenter = False
    region.mReferenceFrame = dart.dynamics.SimpleFrame()
    assert region.mComputeErrorFromCenter is False
    assert isinstance(region.mReferenceFrame, dart.dynamics.SimpleFrame)
    region.mReferenceFrame = None
    assert region.mReferenceFrame is None
