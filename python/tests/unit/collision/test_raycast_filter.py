import gc
import weakref

import dartpy as dart
import pytest


@pytest.mark.parametrize("all_hits", [False, True])
def test_raycast_filter_can_be_cleared_and_releases_callback(all_hits):
    detector = dart.collision.DARTCollisionDetector()
    frame = dart.dynamics.SimpleFrame()
    frame.setShape(dart.dynamics.SphereShape(1))
    group = detector.createCollisionGroup()
    group.addShapeFrame(frame)
    option = dart.collision.RaycastOption(all_hits, True, None)
    result = dart.collision.RaycastResult()
    assert option.mFilter is None

    class RejectHits:
        def __call__(self, obj):
            return False

    callback = RejectHits()
    retained = weakref.ref(callback)
    option.mFilter = callback
    del callback
    gc.collect()
    assert retained() is not None
    assert not group.raycast([-2, 0, 0], [2, 0, 0], option, result)
    assert not result.hasHit()

    option.mFilter = None
    assert option.mFilter is None
    gc.collect()
    assert retained() is None
    assert group.raycast([-2, 0, 0], [2, 0, 0], option, result)
    assert result.hasHit()
    assert len(result.mRayHits) == 1
    assert result.mRayHits[0].mFraction == pytest.approx(0.25, abs=1e-12)

    with pytest.raises(TypeError):
        option.mFilter = 42
    assert option.mFilter is None
