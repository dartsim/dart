import pytest

import dartpy as dart

dart_utils = getattr(dart, "utils", None)
UsdParser = getattr(dart_utils, "UsdParser", None) if dart_utils else None

pytestmark = pytest.mark.skipif(
    UsdParser is None, reason="USD support is not enabled in this build"
)


def test_read_simple_chain_skeleton() -> None:
    skel = UsdParser.readSkeleton("dart://sample/usd/simple_chain.usda")
    assert skel is not None
    assert skel.getNumBodyNodes() == 2
    assert skel.getNumJoints() == 2


def test_read_unitree_world() -> None:
    world = UsdParser.readWorld("dart://sample/usd/unitree_h1_minimal.usda")
    assert world is not None
    assert world.getNumSkeletons() == 1
    humanoid = world.getSkeleton(0)
    assert humanoid.getNumBodyNodes() >= 10
