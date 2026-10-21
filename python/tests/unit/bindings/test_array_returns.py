"""Owned Eigen returns are read-only snapshots on both NumPy export paths."""

import os
import subprocess
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest


def _import_root():
    # dartpy is an extension module today and a package with __init__.py once
    # the package layout lands; either way, return the sys.path entry it came
    # from.
    origin = Path(dart.__file__).resolve()
    return origin.parent.parent if origin.name == "__init__.py" else origin.parent


@pytest.mark.parametrize(
    "fallback",
    [
        pytest.param(
            False,
            id="numpy2",
            marks=pytest.mark.skipif(
                int(np.__version__.split(".")[0]) < 2,
                reason="the fast export needs NumPy 2",
            ),
        ),
        pytest.param(True, id="fallback"),
    ],
)
def test_owned_eigen_returns(fallback):
    # Select before importing dartpy: numpy2_export caches numpy.__version__.
    source = f"""
import numpy as np
if {fallback!r}:
    np.__version__ = "1.26.4"
import dartpy as dart
import pytest

skeleton = dart.dynamics.Skeleton()
joint, body = skeleton.createFreeJointAndBodyNodePair()
properties = dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
    [0.2, 0.2, 0.2], 3, 2, 0.5
)
soft_properties = dart.dynamics.SoftBodyNodeProperties(
    dart.dynamics.BodyNodeProperties(), properties
)
_, soft = skeleton.createFreeJointAndSoftBodyNodePair(
    None, dart.dynamics.FreeJointProperties(), soft_properties
)
heightmap = dart.dynamics.HeightmapShapef()
heightmap.setHeightField(np.arange(6, dtype=np.float32).reshape(2, 3))
positions = skeleton.getPositions()
# Distinguish the actual exporter, rather than just setting its selector.
assert isinstance(positions.base, memoryview) == {fallback!r}
arrays = [
    positions,
    body.getTransform().matrix(),
    body.getLinearJacobian(),
    soft.get_point_mass(0).positions,
    properties.faces,
    soft.get_face(0),
    heightmap.getScale(),
    heightmap.getHeightField(),
    dart.dynamics.Skeleton().getPositions(),
]
for array in arrays:
    assert isinstance(array, np.ndarray)
    assert not array.flags.writeable
    with pytest.raises(ValueError, match="assignment destination is read-only"):
        array[...] = 0
    writable = array.copy()
    assert writable.flags.writeable
    writable[...] = 42
    assert not np.shares_memory(array, writable)

skeleton.setPositions(np.ones(skeleton.getNumDofs()))
np.testing.assert_array_equal(positions, 0)
np.testing.assert_array_equal(skeleton.getPositions(), 1)

# Const reference fields and live views retain their existing behavior.
result = dart.collision.DistanceResult()
result.nearestPoint1 = [1, 2, 3]
view = result.nearestPoint1
assert not view.flags.writeable
with pytest.raises(ValueError, match="assignment destination is read-only"):
    view[0] = 4
result.nearestPoint1 = [4, 2, 3]
np.testing.assert_array_equal(view, [4, 2, 3])
np.testing.assert_array_equal(result.nearestPoint1, [4, 2, 3])
box = dart.dynamics.BoxShape([1, 2, 3])
view = box.getSize()
assert not view.flags.writeable
box.setSize([4, 5, 6])
np.testing.assert_array_equal(view, [4, 5, 6])

# Preserve NumPy's existing empty const-view behavior, too.
weights = dart.dynamics.InverseKinematicsGradientMethodProperties()
assert weights.mComponentWeights.size == 0
assert weights.mComponentWeights.flags.writeable
weights.mComponentWeights = [1, 2]
assert not weights.mComponentWeights.flags.writeable
"""
    result = subprocess.run(
        [sys.executable, "-c", source],
        env=os.environ | {"PYTHONPATH": str(_import_root())},
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
