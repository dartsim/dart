"""The Gazebo lanes follow the main release target without changing packaging."""

import os
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/gz_dart_version.py"


def _write_source(tmp_path, package_version, rows):
    (tmp_path / "package.xml").write_text(
        f"<package><version>{package_version}</version></package>", encoding="utf-8"
    )
    docs = tmp_path / "docs/onboarding"
    docs.mkdir(parents=True)
    (docs / "release-management.md").write_text(rows, encoding="utf-8")


def _run(tmp_path, package_version, rows):
    _write_source(tmp_path, package_version, rows)
    return subprocess.run(
        [sys.executable, str(SCRIPT), str(tmp_path)], capture_output=True, text=True
    )


@pytest.mark.parametrize(
    "package_version,target,override",
    [
        ("6.20.0", "7.0.0", "7.0.0"),
        ("6.20.0", "7.4.2", "7.4.2"),
        ("7.4.2", "8.0.0", "8.0.0"),
        ("6.9.0", "6.10.0", "6.10.0"),
        ("7.0.0", "7.0.0", ""),
        ("7.1.0", "7.0.0", ""),
    ],
)
def test_override_tracks_rollovers_and_clears_stale_cache(
    tmp_path, package_version, target, override
):
    run = _run(tmp_path, package_version, f"| `main` | Development | `{target}` |\n")
    assert run.returncode == 0, run.stderr
    assert run.stdout == f"-DDART_VERSION_OVERRIDE:STRING={override}\n"


@pytest.mark.parametrize(
    "rows",
    [
        "",
        "| `main` | Development | `7.0` |\n",
        "| `main` | Development | `07.0.0` |\n",
        "| `main` | Development | `7.08.0` |\n",
        "| `main` | Development | `7.0.01` |\n",
        "| `main` | Development | `7.0.0` |\n" * 2,
        "| `release-6.20` | Maintenance | `6.20.0` |\n",
    ],
)
def test_unreadable_release_target_fails_loudly(tmp_path, rows):
    run = _run(tmp_path, "6.20.0", rows)
    assert run.returncode == 2
    assert "main | Development | X.Y.Z release target" in run.stderr
    assert run.stdout == ""


@pytest.mark.parametrize("package_version", ["6.20", "06.20.0", "6.020.0", "6.20.00"])
def test_malformed_package_version_fails_loudly(tmp_path, package_version):
    run = _run(tmp_path, package_version, "| `main` | Development | `7.0.0` |\n")
    assert run.returncode == 2
    assert "package.xml must contain a numeric X.Y.Z version" in run.stderr


@pytest.mark.skipif(sys.platform != "linux", reason="compat lanes require Linux")
@pytest.mark.parametrize("lane", ["forward", "unpatched"])
@pytest.mark.parametrize("valid", [True, False])
def test_gz_configures_receive_override_and_stop_on_selection_failure(
    tmp_path, lane, valid
):
    _write_source(
        tmp_path,
        "6.20.0",
        "| `main` | Development | `7.0.0` |\n" if valid else "",
    )
    (tmp_path / "CMakeLists.txt").write_text("# DART_VERSION_OVERRIDE\n")
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copy(SCRIPT, scripts)
    shim = tmp_path / "bin"
    shim.mkdir()
    cmake = shim / "cmake"
    cmake.write_text('#!/bin/bash\nprintf "%s\\n" "$@" >> "$CMAKE_CALLS"\n')
    cmake.chmod(0o755)
    calls = tmp_path / "cmake-calls"
    env = {
        **os.environ,
        "PATH": str(shim) + os.pathsep + os.environ["PATH"],
        "CMAKE_CALLS": str(calls),
        "CONDA_PREFIX": str(tmp_path / "prefix"),
        "PIXI_ENVIRONMENT_NAME": "gazebo",
        "GZ_COMPAT_DART_SOURCE": str(tmp_path),
        "GZ_COMPAT_DIR": str(tmp_path / "work"),
        "DART_PARALLEL_JOBS": "1",
    }
    if lane == "forward":
        manifest = tomllib.loads((ROOT / "pixi.toml").read_text(encoding="utf-8"))
        command = [
            "bash",
            "-c",
            manifest["feature"]["gazebo"]["tasks"]["config-dart-gz"]["cmd"],
        ]
    else:
        command = ["bash", str(ROOT / "tools/gazebo/compat/lane.sh"), "ionic", "dart"]
    run = subprocess.run(command, cwd=tmp_path, env=env, capture_output=True, text=True)
    if valid:
        assert run.returncode == 0, run.stdout + run.stderr
        assert "-DDART_VERSION_OVERRIDE:STRING=7.0.0" in calls.read_text().splitlines()
    else:
        assert run.returncode != 0
        assert "main | Development | X.Y.Z release target" in run.stderr
        assert not calls.exists(), "DART configure ran after version selection failed"
