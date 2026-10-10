import dartpy as dart
import numpy as np
import pytest


def make_soft_body(properties):
    skeleton = dart.dynamics.Skeleton("soft")
    body_properties = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties("skin")
    )
    joint, body = skeleton.createFreeJointAndSoftBodyNodePair(
        None,
        dart.dynamics.FreeJointProperties(),
        dart.dynamics.SoftBodyNodeProperties(body_properties, properties),
    )
    return skeleton, joint, body


@pytest.mark.parametrize(
    "factory, count",
    [
        (
            lambda: dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
                [0.2, 0.2, 0.1], dart.math.Isometry3(), [4, 4, 4], 0.5
            ),
            56,
        ),
        (
            lambda: dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
                [0.2, 0.2, 0.2], 6, 6, 0.5
            ),
            32,
        ),
        (
            lambda: dart.dynamics.SoftBodyNodeHelper.makeCylinderProperties(
                0.1, 0.1, 8, 3, 2, 0.5
            ),
            50,
        ),
    ],
)
def test_soft_factories_construct_native_bodies(factory, count):
    properties = factory()
    properties.mKv = 400
    properties.mKe = 100
    properties.mDampCoeff = 3
    skeleton, joint, body = make_soft_body(properties)
    assert isinstance(body, dart.dynamics.SoftBodyNode)
    assert skeleton.getNumSoftBodyNodes() == 1
    assert body.getNumPointMasses() == count
    assert body.getMass() == pytest.approx(0.5 + body.getInertia().getMass())
    assert body.getVertexSpringStiffness() == 400
    assert body.getEdgeSpringStiffness() == 100
    assert body.getDampingCoefficient() == 3
    shape = body.getShapeNode(0)
    assert shape.getShape().getType() == "SoftMeshShape"
    assert shape.hasVisualAspect()
    assert shape.hasCollisionAspect()
    assert shape.hasDynamicsAspect()
    world = dart.simulation.World()
    world.addSkeleton(skeleton)
    world.step()
    assert np.isfinite(skeleton.getPositions()).all()


def test_box_preserves_native_fragment_clamping():
    properties = dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
        [1, 1, 1], dart.math.Isometry3(), [1, 2, 3], 1
    )
    assert make_soft_body(properties)[2].getNumPointMasses() == 26


def test_lowest_safe_ellipsoid_and_cylinder_subdivisions():
    helper = dart.dynamics.SoftBodyNodeHelper
    ellipsoid = helper.makeEllipsoidProperties([1, 1, 1], 3, 2, 1)
    cylinder = helper.makeCylinderProperties(1, 1, 3, 2, 1, 1)
    assert make_soft_body(ellipsoid)[2].getNumPointMasses() == 5
    assert make_soft_body(cylinder)[2].getNumPointMasses() == 11


def test_box_rejects_nonfinite_transform():
    transform = dart.math.Isometry3()
    transform.set_translation([np.nan, 0, 0])
    with pytest.raises(ValueError):
        dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
            [1, 1, 1], transform, [4, 4, 4], 1
        )


@pytest.mark.parametrize("slices, stacks", [(0, 6), (2, 6), (6, 0), (6, 1), (-1, 6)])
def test_ellipsoid_rejects_unsafe_subdivisions(slices, stacks):
    with pytest.raises(ValueError):
        dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
            [1, 1, 1], slices, stacks, 1
        )


@pytest.mark.parametrize(
    "slices, stacks, rings",
    [(0, 3, 2), (2, 3, 2), (8, 0, 2), (8, 1, 2), (8, 3, 0), (8, 3, -1)],
)
def test_cylinder_rejects_unsafe_subdivisions(slices, stacks, rings):
    with pytest.raises(ValueError):
        dart.dynamics.SoftBodyNodeHelper.makeCylinderProperties(
            1, 1, slices, stacks, rings, 1
        )


@pytest.mark.parametrize("bad", [0, -1, np.nan, np.inf])
def test_soft_factories_reject_invalid_dimensions_and_mass(bad):
    helper = dart.dynamics.SoftBodyNodeHelper
    for factory in (
        lambda: helper.makeBoxProperties(
            [bad, 1, 1], dart.math.Isometry3(), [4, 4, 4], 1
        ),
        lambda: helper.makeEllipsoidProperties([bad, 1, 1], 6, 6, 1),
        lambda: helper.makeCylinderProperties(bad, 1, 8, 3, 2, 1),
        lambda: helper.makeCylinderProperties(1, bad, 8, 3, 2, 1),
        lambda: helper.makeBoxProperties(
            [1, 1, 1], dart.math.Isometry3(), [4, 4, 4], bad
        ),
        lambda: helper.makeEllipsoidProperties([1, 1, 1], 6, 6, bad),
        lambda: helper.makeCylinderProperties(1, 1, 8, 3, 2, bad),
    ):
        with pytest.raises(ValueError):
            factory()
