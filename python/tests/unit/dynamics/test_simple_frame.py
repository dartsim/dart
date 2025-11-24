import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    world_frame = dart.dynamics.Frame.World()
    assert world_frame.is_world()

    frame1 = dart.dynamics.SimpleFrame()
    assert not frame1.is_world()
    assert not frame1.is_shape_node()
    assert frame1.is_shape_frame()

    assert frame1.get_transform().translation()[0] == pytest.approx(0)
    assert frame1.get_transform().translation()[1] == pytest.approx(0)
    assert frame1.get_transform().translation()[2] == pytest.approx(0)

    frame1.set_translation([1, 2, 3])
    assert frame1.get_transform().translation()[0] == pytest.approx(1)
    assert frame1.get_transform().translation()[1] == pytest.approx(2)
    assert frame1.get_transform().translation()[2] == pytest.approx(3)

    assert frame1.get_parent_frame().is_world()
    assert frame1.descends_from(None)
    assert frame1.descends_from(world_frame)
    assert frame1.descends_from(frame1)

    frame2 = frame1.spawn_child_simple_frame()
    assert not frame2.is_world()
    assert frame2.descends_from(None)
    assert frame2.descends_from(world_frame)
    assert frame2.descends_from(frame1)
    assert frame2.descends_from(frame2)

    assert frame2.get_transform().translation()[0] == pytest.approx(1)
    assert frame2.get_transform().translation()[1] == pytest.approx(2)
    assert frame2.get_transform().translation()[2] == pytest.approx(3)

    frame2.set_relative_translation([1, 2, 3])
    assert frame2.get_transform().translation()[0] == pytest.approx(2)
    assert frame2.get_transform().translation()[1] == pytest.approx(4)
    assert frame2.get_transform().translation()[2] == pytest.approx(6)

    frame2.set_translation([1, 2, 3])
    assert frame2.get_transform().translation()[0] == pytest.approx(1)
    assert frame2.get_transform().translation()[1] == pytest.approx(2)
    assert frame2.get_transform().translation()[2] == pytest.approx(3)


if __name__ == "__main__":
    pytest.main()
