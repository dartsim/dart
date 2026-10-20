# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

import dartpy as dart
import pytest


@pytest.mark.skipif(
    hasattr(dart.utils.SdfParser, "Options"), reason="SDF loading is enabled"
)
@pytest.mark.parametrize("loader", ["readWorld", "readSkeleton"])
def test_sdf_disabled_raises(loader):
    with pytest.raises(RuntimeError, match="built without sdformat support"):
        getattr(dart.utils.SdfParser, loader)("missing.sdf")


requires_sdf = pytest.mark.skipif(
    not hasattr(dart.utils.SdfParser, "Options"), reason="SDF loading is disabled"
)


@requires_sdf
def test_read_world():
    assert (
        dart.utils.SdfParser.readWorld("dart://sample/sdf/double_pendulum.world")
        is not None
    )


@requires_sdf
def test_invalid_sdf_returns_none(tmp_path):
    document = tmp_path / "invalid.sdf"
    document.write_text(
        "<sdf version='1.9'><model name='invalid'><link name='body'>"
        "<pose rotation_format='quat_xyzw'>0 0 0 0 0 0 0</pose>"
        "</link></model></sdf>"
    )
    assert dart.utils.SdfParser.readSkeleton(str(document)) is None


@requires_sdf
def test_named_frames_and_world_gravity(tmp_path):
    document = tmp_path / "frames.world"
    document.write_text(
        "<sdf version='1.9'><world name='test_world'><gravity>1 2 -3</gravity>"
        "<frame name='origin'><pose>4 5 6 0 0 0</pose></frame>"
        "<model name='body'><pose relative_to='origin'>1 0 0 0 0 0</pose>"
        "<link name='link'/></model></world></sdf>"
    )
    world = dart.utils.SdfParser.readWorld(str(document))
    assert world is not None
    assert world.getGravity().tolist() == [1.0, 2.0, -3.0]
    body = world.getSkeleton(0).getBodyNode(0)
    assert body.getTransform().translation().tolist() == [5.0, 5.0, 6.0]


if __name__ == "__main__":
    pytest.main()
