import importlib.util
import sys
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def patch_gz_physics():
    repo_root = Path(__file__).resolve().parents[3]
    script_path = repo_root / "scripts" / "patch_gz_physics.py"

    spec = importlib.util.spec_from_file_location("patch_gz_physics", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_cmake(tmp_path: Path, version: str) -> Path:
    cmake_file = tmp_path / "CMakeLists.txt"
    cmake_file.write_text(
        "gz_find_package(DART REQUIRED COMPONENTS dynamics "
        f"VERSION {version})\n"
    )
    return cmake_file


@pytest.mark.parametrize("version", ["6.10", "6.10.1", "7.0", "7.0.0"])
def test_validate_accepts_compatible_versions(patch_gz_physics, tmp_path, version):
    assert patch_gz_physics.validate_gz_physics_cmake(
        write_cmake(tmp_path, version)
    )


@pytest.mark.parametrize("version", ["6.9.9", "7.0.1", "8.0"])
def test_validate_rejects_incompatible_versions(patch_gz_physics, tmp_path, version):
    assert not patch_gz_physics.validate_gz_physics_cmake(
        write_cmake(tmp_path, version)
    )


@pytest.mark.parametrize(
    ("lhs", "rhs", "expected"),
    [
        ((7, 0), (7, 0, 0), 0),
        ((7, 0, 1), (7, 0), 1),
        ((6, 10), (6, 10, 1), -1),
    ],
)
def test_compare_versions_normalizes_missing_components(
    patch_gz_physics, lhs, rhs, expected
):
    assert patch_gz_physics.compare_versions(lhs, rhs) == expected
