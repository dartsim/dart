import os
import platform

import dartpy
import pytest
from dartpy.io import DartLoader
from tests.util import get_asset_path


def test_parse_skeleton_non_existing_path_returns_null():
    assert os.path.isfile(get_asset_path("skel/cubes.skel")) is True
    loader = DartLoader()
    assert loader.parseSkeleton(get_asset_path("skel/test/does_not_exist.urdf")) is None


def test_parse_skeleton_invalid_urdf_returns_null():
    loader = DartLoader()
    assert loader.parseSkeleton(get_asset_path("urdf/invalid.urdf")) is None


def test_parse_skeleton_missing_mesh_returns_null():
    loader = DartLoader()
    assert loader.parseSkeleton(get_asset_path("urdf/missing_mesh.urdf")) is None


def test_parse_skeleton_invalid_mesh_returns_null():
    loader = DartLoader()
    assert loader.parseSkeleton(get_asset_path("urdf/invalid_mesh.urdf")) is None


def test_parse_skeleton_missing_package_returns_null():
    loader = DartLoader()
    assert loader.parseSkeleton(get_asset_path("urdf/missing_package.urdf")) is None


def test_parse_skeleton_loads_primitive_geometry():
    loader = DartLoader()
    assert (
        loader.parseSkeleton(get_asset_path("urdf/test/primitive_geometry.urdf"))
        is not None
    )


# Failing with following errors:
# TypeError: No to_python (by-value) converter found for C++ type: std::shared_ptr<dart::simulation::World>
#
# def test_parse_world():
#     loader = DartLoader()
#     assert loader.parseWorld(get_asset_path('urdf/testWorld.urdf')) is not None


def test_parse_joint_properties():
    loader = DartLoader()
    robot = loader.parseSkeleton(get_asset_path("urdf/test/joint_properties.urdf"))
    assert robot is not None


#    joint1 = robot.getJoint(1)
#    assert joint1 is not None
#    assert joint1.getDampingCoefficient(0) == pytest.approx(1.2, 1e-12)
#    assert joint1.getCoulombFriction(0) == pytest.approx(2.3, 1e-12)

#    joint2 = robot.getJoint(2)
#    assert joint2 is not None
#    assert joint2.getPositionLowerLimit(0) == -float("inf")
#    assert joint2.getPositionUpperLimit(0) == float("inf")
#    if not platform.linux_distribution()[1] == '14.04':
#        assert joint2.isCyclic(0)


if __name__ == "__main__":
    pytest.main()
