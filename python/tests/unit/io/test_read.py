import dartpy as dart
from tests.util import get_asset_path


def test_read_skeleton_loads_urdf_common_path():
    skeleton = dart.io.read_skeleton(
        get_asset_path("urdf/test/primitive_geometry.urdf")
    )

    assert skeleton is not None


def test_read_options_are_canonical_io_types():
    options = dart.io.ReadOptions()
    assert options.format == dart.io.ModelFormat.AUTO
    assert options.sdf_default_root_joint_type == dart.io.RootJointType.FLOATING

    options.format = dart.io.ModelFormat.URDF
    options.add_package_directory("unused", "/tmp")

    skeleton = dart.io.read_skeleton(
        get_asset_path("urdf/test/primitive_geometry.urdf"), options
    )

    assert skeleton is not None


def test_simulation_read_options_aliases_io_types():
    assert dart.simulation.ModelFormat is dart.io.ModelFormat
    assert dart.simulation.RootJointType is dart.io.RootJointType
    assert dart.simulation.ReadOptions is dart.io.ReadOptions
