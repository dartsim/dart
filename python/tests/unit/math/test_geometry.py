import dartpy as dart
import numpy as np
import pytest


@pytest.mark.parametrize(
    "source,target",
    [
        ([0, 0, 2], [0, 3, 0]),
        ([0, 0, 1], [0, 0, -1]),
        ([0, 0, 1], [1e-10, 0, -1]),
        ([1, 2, 3], [-2, 1, 0.5]),
    ],
)
def test_quaternion_from_two_vectors_aligns_directions(source, target):
    quaternion = dart.math.Quaternion.from_two_vectors(source, target)
    source, target = np.asarray(source), np.asarray(target)
    np.testing.assert_allclose(
        quaternion.rotation() @ (source / np.linalg.norm(source)),
        target / np.linalg.norm(target),
        atol=1e-8,
    )
    assert np.linalg.norm(quaternion.wxyz()) == pytest.approx(1)


@pytest.mark.parametrize(
    "invalid",
    [[0, 0, 0], [np.nan, 0, 1], [np.inf, 0, 1], [1e308, 0, 0], [1e-161, 0, 0]],
)
def test_quaternion_from_two_vectors_rejects_invalid_directions(invalid):
    with pytest.raises(ValueError, match="finite and nonzero"):
        dart.math.Quaternion.from_two_vectors(invalid, [0, 0, 1])
    with pytest.raises(ValueError, match="finite and nonzero"):
        dart.math.Quaternion.from_two_vectors([0, 0, 1], invalid)
