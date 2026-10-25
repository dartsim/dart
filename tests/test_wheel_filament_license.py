"""Fetched Filament licenses must stay intact for the native wheel.

An offline configure must stage them from disk and never download them.
tests/test_dartpy_backend.py covers their placement in the wheel.
"""

import hashlib
import re
import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("crlf", [False, True])
def test_filament_root_license_validation_preserves_upstream_bytes(tmp_path, crlf):
    root = Path(__file__).resolve().parents[1]
    canonical = (root / "tests/cmake/filament_notices/LICENSE").read_bytes()
    canonical = canonical.replace(b"\r\n", b"\n")
    expected = "02d70b593dd8e9355dd8e1ca8d13c303bee5bcaf3065b66ff6e4f534e1e6a2c1"
    assert hashlib.sha256(canonical).hexdigest() == expected
    contents = canonical.replace(b"\n", b"\r\n") if crlf else canonical
    license_file = tmp_path / "LICENSE"
    license_file.write_bytes(contents)
    script = tmp_path / "validate.cmake"
    script.write_text(
        f'include("{root.as_posix()}/cmake/DARTFilamentNotices.cmake")\n'
        f'dart_filament_validate_root_license("{license_file.as_posix()}" "{expected}")\n',
        encoding="utf-8",
    )
    subprocess.run(["cmake", "-P", str(script)], check=True, capture_output=True)
    assert license_file.read_bytes() == contents


@pytest.mark.parametrize("supplied_tree", [False, True])
def test_filament_notice_staging_rejects_unrelated_root_license(
    tmp_path, supplied_tree
):
    root = Path(__file__).resolve().parents[1]
    staging = tmp_path / "staging"
    staging.mkdir()
    license_file = staging / "LICENSE"
    license_file.write_bytes(b"Unrelated SDK or stale cached license\n")
    supplied = tmp_path / "notices"
    supplied.mkdir()
    source_argument = f' "{supplied.as_posix()}"' if supplied_tree else ""
    script = tmp_path / "stage.cmake"
    script.write_text(
        f'include("{root.as_posix()}/cmake/DARTFilamentNotices.cmake")\n'
        f'dart_filament_stage_notices("{staging.as_posix()}"{source_argument})\n',
        encoding="utf-8",
    )
    result = subprocess.run(
        ["cmake", "-P", str(script)], capture_output=True, text=True
    )
    assert result.returncode != 0
    assert "pinned Filament root license changed" in result.stderr
    assert license_file.read_bytes() == b"Unrelated SDK or stale cached license\n"


ROOT = Path(__file__).resolve().parents[1]
NOTICES = ROOT / "cmake/DARTFilamentNotices.cmake"
PINNED = re.findall(r'"([^"|]+)\|([a-f0-9]{64})"', NOTICES.read_text())
PROVIDER = ROOT / "cmake/DARTFilamentProvider.cmake"
# Fails any script that reaches file(DOWNLOAD), so no test needs a network.
DOWNLOAD_TRIPWIRE = """
macro(file)
  if("${ARGV0}" STREQUAL "DOWNLOAD")
    message(FATAL_ERROR "Tripwire: file(DOWNLOAD ${ARGV1})")
  endif()
  _file(${ARGV})
endmacro()
"""


def pin_fixture_notices(tmp_path):
    """Write a fixture source tree and a copy of the notices module pinning it.

    The real pins hash upstream files that this repository does not carry. The
    copy pins the fixture files instead, so the staging logic is what runs.
    """
    tree = tmp_path / "source"
    module = NOTICES.read_text()
    for relative, pinned in PINNED:
        contents = f"Fixture {relative}\n".encode()
        (tree / relative).parent.mkdir(parents=True, exist_ok=True)
        (tree / relative).write_bytes(contents)
        module = module.replace(pinned, hashlib.sha256(contents).hexdigest())
    modules = tmp_path / "modules"
    modules.mkdir()
    (modules / "DARTFilamentNotices.cmake").write_text(module)
    return tree, modules


def run_cmake(tmp_path, script, *, offline):
    path = tmp_path / "run.cmake"
    path.write_text(DOWNLOAD_TRIPWIRE + script, encoding="utf-8")
    disconnected = "ON" if offline else "OFF"
    result = subprocess.run(
        ["cmake", f"-DFETCHCONTENT_FULLY_DISCONNECTED={disconnected}", "-P", str(path)],
        capture_output=True,
        text=True,
    )
    # CMake wraps long diagnostics, so compare flattened output.
    return result, " ".join((result.stdout + result.stderr).split())


def stage(tmp_path, modules, staging, source=None, *, offline=True):
    argument = f' "{source.as_posix()}"' if source else ""
    return run_cmake(
        tmp_path,
        f'set(CMAKE_MODULE_PATH "{modules.as_posix()}")\n'
        "include(DARTFilamentNotices)\n"
        f'dart_filament_stage_notices("{staging.as_posix()}"{argument})\n',
        offline=offline,
    )


def stage_system_sdk(tmp_path, modules, sdk, *, seed=None, offline=True):
    """Run the provider's system-SDK notice staging against a fake SDK."""
    provider = PROVIDER.read_text()
    # Slice that branch out: the provider's other branches need a real SDK.
    start = provider.index("if(DART_USE_SYSTEM_FILAMENT)\n  # SDK packages")
    end = provider.index('file(\n  WRITE "${PROJECT_BINARY_DIR}/dartpy-filament')
    seeding = (
        f'set(FETCHCONTENT_SOURCE_DIR_DART_FILAMENT_SDK "{seed.as_posix()}")\n'
        if seed
        else ""
    )
    return run_cmake(
        tmp_path,
        f'set(CMAKE_MODULE_PATH "{modules.as_posix()}")\n'
        f'set(CMAKE_CURRENT_BINARY_DIR "{(tmp_path / "build").as_posix()}")\n'
        "set(DART_USE_SYSTEM_FILAMENT ON)\n"
        f'set(Filament_ROOT "{sdk.as_posix()}")\n'
        f'set(Filament_INCLUDE_DIR "{sdk.as_posix()}/include")\n'
        + seeding
        + provider[start:end],
        offline=offline,
    )


def staged(staging, notice):
    return staging / ("LICENSE" if notice == "LICENSE" else f"notices/{notice}")


def assert_staged_from(staging, tree):
    for notice, _ in PINNED:
        assert staged(staging, notice).read_bytes() == (tree / notice).read_bytes()


def assert_offline_notice_error(output, notice):
    assert "Tripwire" not in output
    assert f"Cannot fetch the Filament runtime notice {notice} with " in output
    assert "FETCHCONTENT_FULLY_DISCONNECTED=ON" in output
    assert "FETCHCONTENT_SOURCE_DIR_DART_FILAMENT_SDK" in output
    assert "DART_BUILD_GUI=OFF" in output


@pytest.mark.parametrize("offline", [True, False])
def test_notices_stage_from_a_local_tree_then_from_the_staged_files(tmp_path, offline):
    tree, modules = pin_fixture_notices(tmp_path)
    staging = tmp_path / "staging"
    # Neither run downloads: the tripwire fails the script if one starts.
    for source in (tree, None):
        result, output = stage(tmp_path, modules, staging, source, offline=offline)
        assert result.returncode == 0, output
        assert_staged_from(staging, tree)


@pytest.mark.parametrize("case", ["nothing staged", "stale copy", "tree lacks it"])
def test_offline_missing_notice_stops_and_names_the_options(tmp_path, case):
    tree, modules = pin_fixture_notices(tmp_path)
    staging = tmp_path / "staging"
    source, missing = None, "third_party/zstd/LICENSE"
    if case == "nothing staged":
        missing = "LICENSE"
    elif case == "stale copy":
        result, output = stage(tmp_path, modules, staging, tree)
        assert result.returncode == 0, output
        staged(staging, missing).write_text("Stale\n")
    else:
        (tree / missing).unlink()
        source = tree
    result, output = stage(tmp_path, modules, staging, source)
    assert result.returncode != 0
    assert_offline_notice_error(output, missing)


def test_online_missing_notice_is_still_downloaded_from_the_pinned_url(tmp_path):
    result, output = stage(
        tmp_path, NOTICES.parent, tmp_path / "staging", offline=False
    )
    assert result.returncode != 0
    assert (
        "Tripwire: file(DOWNLOAD "
        "https://raw.githubusercontent.com/google/filament/v1.77.3/LICENSE)"
    ) in output


@pytest.mark.parametrize("offline", [True, False])
def test_local_tree_notice_with_another_hash_is_rejected(tmp_path, offline):
    tree, modules = pin_fixture_notices(tmp_path)
    (tree / "third_party/zstd/LICENSE").write_text("Changed\n")
    result, output = stage(
        tmp_path, modules, tmp_path / "staging", tree, offline=offline
    )
    assert result.returncode != 0
    assert "The pinned Filament runtime notice changed: third_party/zstd/LICENSE" in (
        output
    )


@pytest.mark.parametrize("offline", [True, False])
def test_system_sdk_without_notices_stages_the_seeded_source_tree(tmp_path, offline):
    tree, modules = pin_fixture_notices(tmp_path)
    sdk = tmp_path / "sdk"
    sdk.mkdir()
    result, output = stage_system_sdk(
        tmp_path, modules, sdk, seed=tree, offline=offline
    )
    assert result.returncode == 0, output
    assert_staged_from(tmp_path / "build/filament_licenses", tree)


def test_system_sdk_without_notices_stops_offline_when_unseeded(tmp_path):
    _, modules = pin_fixture_notices(tmp_path)
    sdk = tmp_path / "sdk"
    sdk.mkdir()
    result, output = stage_system_sdk(tmp_path, modules, sdk)
    assert result.returncode != 0
    assert_offline_notice_error(output, "LICENSE")


@pytest.mark.parametrize("seeded", [False, True])
def test_system_sdk_with_notices_needs_no_seed_offline(tmp_path, seeded):
    tree, modules = pin_fixture_notices(tmp_path)
    sdk = tmp_path / "sdk"
    shutil.copytree(tree, sdk / "notices")
    (sdk / "notices/LICENSE").replace(sdk / "LICENSE")
    # The SDK's own tree comes first: an empty seed must not be consulted.
    seed = tmp_path / "empty-seed" if seeded else None
    if seed:
        seed.mkdir()
    result, output = stage_system_sdk(tmp_path, modules, sdk, seed=seed)
    assert result.returncode == 0, output
    assert_staged_from(tmp_path / "build/filament_licenses", tree)
