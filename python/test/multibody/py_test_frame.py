# Copyright (c) 2011-2022, The DART development contributors

import pytest
import dartpy8 as dart


def test_inertial_frame():
    frame = dart.multibody.InertialFrame()

    # Properties
    assert frame.is_inertial_frame()
    assert frame.get_name() == dart.multibody.InertialFrame.GetName()

    # Positions == 0
    assert frame.get_pose().is_identity()
    assert frame.get_orientation().is_identity()
    assert frame.get_position().is_identity()

    # Velocities == 0
    assert frame.get_spatial_velocity().is_zero()
    assert frame.get_angular_velocity().is_zero()
    assert frame.get_linear_velocity().is_zero()

    # Accelerations == 0
    assert frame.get_spatial_acceleration().is_zero()
    assert frame.get_angular_acceleration().is_zero()
    assert frame.get_linear_acceleration().is_zero()


def test_simple_relative_frame():
    frame = dart.multibody.FreeRelativeFrame()

    assert frame.get_name() == ""
    assert frame.get_pose().is_identity()
    assert frame.get_spatial_velocity().is_zero()
    assert frame.get_spatial_acceleration().is_zero()


if __name__ == "__main__":
    pytest.main()
