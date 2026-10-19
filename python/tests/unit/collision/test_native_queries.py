import dartpy as dart
import numpy as np
import pytest


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize(
    "shape_factory,axis,radius",
    [
        pytest.param(lambda: dart.dynamics.SphereShape(1), 0, 1, id="sphere"),
        pytest.param(lambda: dart.dynamics.BoxShape([2, 2, 2]), 0, 1, id="box"),
        pytest.param(lambda: dart.dynamics.CapsuleShape(0.5, 2), 0, 0.5, id="capsule"),
        pytest.param(lambda: dart.dynamics.CylinderShape(0.5, 2), 0, 0.5, id="cylinder"),
        pytest.param(lambda: dart.dynamics.PlaneShape([0, 0, 1], 0), 2, 0, id="plane"),
    ],
)
def test_native_raycast_geometry_and_finite_segment(shape_factory, axis, radius, direction):
    detector = dart.collision.DARTCollisionDetector()
    frame = dart.dynamics.SimpleFrame()
    frame.setShape(shape_factory())
    center = np.array([0.25, -0.5, 1.5])
    frame.setTranslation(center)
    group = detector.createCollisionGroup()
    group.addShapeFrame(frame)
    option = dart.collision.RaycastOption(False, True)
    result = dart.collision.RaycastResult()
    normal = np.zeros(3)
    normal[axis] = direction

    assert group.raycast(center + 3 * normal, center - 3 * normal, option, result)
    assert result.hasHit()
    assert len(result.mRayHits) == 1
    hit = result.mRayHits[0]
    assert hit.mCollisionObject.getShapeFrame() is frame
    np.testing.assert_allclose(hit.mPoint, center + radius * normal, atol=1e-12, rtol=0)
    np.testing.assert_allclose(hit.mNormal, normal, atol=1e-12, rtol=0)
    assert hit.mFraction == pytest.approx((3 - radius) / 6, abs=1e-12)

    assert not group.raycast(center + 3 * normal, center + 2 * normal, option, result)
    assert not result.hasHit()
    assert len(result.mRayHits) == 0
    assert not group.raycast(center, center, option, result)
    assert not result.hasHit()


def test_native_raycast_object_hits_sorting_and_python_filter():
    detector = dart.collision.DARTCollisionDetector()
    frames = [dart.dynamics.SimpleFrame(), dart.dynamics.SimpleFrame()]
    for frame, x in zip(frames, [-2, 2]):
        frame.setShape(dart.dynamics.SphereShape(1))
        frame.setTranslation([x, 0, 0])
    group = detector.createCollisionGroup()
    group.addShapeFrames(list(reversed(frames)))
    option = dart.collision.RaycastOption(True, True)
    result = dart.collision.RaycastResult()

    assert group.raycast([-5, 0, 0], [5, 0, 0], option, result)
    assert len(result.mRayHits) == 2
    for hit, frame, fraction in zip(result.mRayHits, frames, [0.2, 0.6]):
        assert hit.mCollisionObject.getShapeFrame() is frame
        assert hit.mFraction == pytest.approx(fraction, abs=1e-12)

    seen = []

    def keep_far_object(obj):
        frame = obj.getShapeFrame()
        seen.append(frame)
        return frame is frames[1]

    option.mFilter = keep_far_object
    assert group.raycast([-5, 0, 0], [5, 0, 0], option, result)
    assert set(seen) == set(frames)
    assert len(result.mRayHits) == 1
    assert result.mRayHits[0].mCollisionObject.getShapeFrame() is frames[1]
    assert result.mRayHits[0].mFraction == pytest.approx(0.6, abs=1e-12)

    option.mFilter = lambda obj: False
    assert not group.raycast([-5, 0, 0], [5, 0, 0], option, result)
    assert not result.hasHit()


@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("nearest_points", [False, True])
@pytest.mark.parametrize(
    "shape_factory,point_tolerance",
    [
        pytest.param(lambda: dart.dynamics.SphereShape(1), 1e-12, id="sphere"),
        pytest.param(lambda: dart.dynamics.BoxShape([2, 2, 2]), 1e-12, id="box"),
        pytest.param(lambda: dart.dynamics.CapsuleShape(1, 2), 1e-12, id="capsule"),
        # GJK's 1e-6 distance tolerance permits approximate cylinder witnesses.
        pytest.param(lambda: dart.dynamics.CylinderShape(1, 2), 1e-3, id="cylinder"),
    ],
)
def test_native_distance_nearest_points_clamping_and_empty_result(
    shape_factory, point_tolerance, nearest_points, reverse
):
    detector = dart.collision.DARTCollisionDetector()
    frames = [dart.dynamics.SimpleFrame(), dart.dynamics.SimpleFrame()]
    frames[0].setShape(shape_factory())
    frames[1].setShape(dart.dynamics.SphereShape(1))
    frames[1].setTranslation([4, 0, 0])
    group = detector.createCollisionGroup()
    group.addShapeFrames(list(reversed(frames)) if reverse else frames)
    option = dart.collision.DistanceOption(nearest_points, -np.inf, None)
    result = dart.collision.DistanceResult()

    for lower_bound in [-np.inf, 3.0]:
        option.distanceLowerBound = lower_bound
        assert group.distance(option, result) == pytest.approx(
            max(2, lower_bound), abs=1e-6, rel=0
        )
        assert result.found()
        assert result.minDistance == pytest.approx(max(2, lower_bound), abs=1e-6, rel=0)
        assert result.unclampedMinDistance == pytest.approx(2, abs=1e-6, rel=0)
        assert {result.shapeFrame1, result.shapeFrame2} == set(frames)
        for frame, point in [
            (result.shapeFrame1, result.nearestPoint1),
            (result.shapeFrame2, result.nearestPoint2),
        ]:
            expected = [1 if frame is frames[0] else 3, 0, 0] if nearest_points else [0] * 3
            np.testing.assert_allclose(point, expected, atol=point_tolerance, rtol=0)
        if nearest_points:
            assert np.linalg.norm(
                result.nearestPoint2 - result.nearestPoint1
            ) == pytest.approx(result.unclampedMinDistance, abs=1e-12, rel=0)

    group.removeAllShapeFrames()
    assert group.distance(option, result) == 3
    assert not result.found()
    assert result.minDistance == 0
    assert result.unclampedMinDistance == 0
    assert result.shapeFrame1 is None
    assert result.shapeFrame2 is None
    np.testing.assert_array_equal(result.nearestPoint1, [0, 0, 0])
    np.testing.assert_array_equal(result.nearestPoint2, [0, 0, 0])
