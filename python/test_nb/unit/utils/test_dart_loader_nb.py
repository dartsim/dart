import os
import platform

import dartpy_nb as dart
import pytest
from dartpy_nb.utils import DartLoader
from util import get_asset_path


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


def test_parse_skeleton_loads_gearbox():
    loader = DartLoader()
    gear = loader.parseSkeleton(get_asset_path("urdf/test/gearbox.urdf"))
    assert gear is not None


def test_parse_skeleton_loads_kuka():
    loader = DartLoader()
    robot = loader.parseSkeleton(get_asset_path("urdf/test/kuka.urdf"))
    assert robot is not None


def test_parse_world_invalid_returns_null():
    loader = DartLoader()
    assert loader.parseWorld(get_asset_path("urdf/invalid.urdf")) is None


def test_parse_world_missing_mesh_returns_null():
    loader = DartLoader()
    assert loader.parseWorld(get_asset_path("urdf/missing_mesh.urdf")) is None


def test_parse_world_missing_package_returns_null():
    loader = DartLoader()
    assert loader.parseWorld(get_asset_path("urdf/missing_package.urdf")) is None


def test_parse_world_invalid_mesh_returns_null():
    loader = DartLoader()
    assert loader.parseWorld(get_asset_path("urdf/invalid_mesh.urdf")) is None


def test_parse_world_loads_world():
    loader = DartLoader()
    world = loader.parseWorld(get_asset_path("urdf/test/world.urdf"))
    assert world is not None
    assert world.getNumSkeletons() == 2


@pytest.mark.skipif(platform.system() == "Windows", reason="Windows path issues")
def test_parse_skeletons_with_include_directive():
    loader = DartLoader()
    skel = loader.parseSkeleton(get_asset_path("urdf/test/include_robot.urdf"))
    assert skel is not None
    assert skel.getName() == "include_robot"


if __name__ == "__main__":
    pytest.main()
