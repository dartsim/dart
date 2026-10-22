"""Check package support boundaries and native baseline CI coverage."""

import os
import shlex
import subprocess
import sys
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet

ROOT = Path(__file__).resolve().parents[1]


def test_package_metadata_support_boundaries():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    metadata = project["project"]

    python = SpecifierSet(metadata["requires-python"])
    assert "3.11" not in python
    assert "3.12" in python
    assert "3.14" in python
    numpy = next(
        Requirement(value)
        for value in metadata["dependencies"]
        if Requirement(value).name == "numpy"
    ).specifier
    assert "1.21.4" not in numpy
    assert "1.21.5" in numpy
    assert "2.3.0" in numpy

    build_system = project["build-system"]
    assert build_system["build-backend"] == "dartpy_backend"
    assert build_system["backend-path"] == ["scripts/pep517"]
    build = {
        requirement.name: requirement.specifier
        for requirement in map(Requirement, build_system["requires"])
    }
    # scikit-build-core injects CMake and Ninja wheels when the host lacks the
    # pinned versions, so they stay out of build-system.requires.
    assert set(build) == {"scikit-build-core"}
    assert "1.1.0" not in build["scikit-build-core"]
    assert "1.1.1" in build["scikit-build-core"]
    skbuild = project["tool"]["scikit-build"]
    assert "3.25.9" not in SpecifierSet(skbuild["cmake"]["version"])
    assert "3.26.0" in SpecifierSet(skbuild["cmake"]["version"])
    assert "1.13.1" not in SpecifierSet(skbuild["ninja"]["version"])
    assert "1.13.2" in SpecifierSet(skbuild["ninja"]["version"])
    assert skbuild["wheel"]["py-api"] == "cp312"
    assert skbuild["cmake"]["define"]["DART_DARTPY_STABLE_ABI"] == "ON"
    assert skbuild["wheel"]["packages"] == []
    assert skbuild["install"]["components"] == ["python"]


def workflow_job(filename, job):
    workflow = yaml.load(
        (ROOT / ".github" / "workflows" / filename).read_text(), Loader=yaml.BaseLoader
    )
    return workflow["jobs"][job]


def test_linux_matrix_covers_baseline_and_forward_compilers():
    job = workflow_job("ci_toolchain.yml", "toolchain")
    rows = job["strategy"]["matrix"]["include"]
    baseline = [row for row in rows if row["runner"] == "ubuntu-22.04"]
    assert {(row["cc"], row["cxx"]) for row in baseline} == {
        ("gcc-11", "g++-11"),
        ("clang-13", "clang++-13"),
    }
    assert all(row["distribution"] == "true" for row in baseline)
    assert {row["name"] for row in rows if row["runner"] == "ubuntu-24.04"} == {
        "gcc 16",
        "clang 19",
        "Eigen 5 (gcc 13)",
    }
    # Packagers build against Eigen 5, which the lockfile does not cover.
    assert [row["eigen"] for row in rows if "eigen" in row] == ["5.*"]
    assert job["runs-on"] == "${{ matrix.runner }}"
    build = next(
        step for step in job["steps"] if "pixi run build-tests" in step.get("run", "")
    )
    assert build["env"] == {
        "CC": "${{ matrix.cc }}",
        "CXX": "${{ matrix.cxx }}",
        "SCCACHE_DIR": "${{ runner.temp }}/sccache-${{ matrix.runner }}-${{ matrix.cc }}",
    }
    # GitHub rejects the workflow if job-level env uses the runner context.
    assert "runner." not in str(job.get("env", {}))
    assert "pixi run test-build-requirements" in build["run"]


@pytest.mark.skipif(
    sys.platform != "linux", reason="The toolchain workflow runs on Linux"
)
def test_linux_toolchain_uses_pixi_runtime_only_for_baseline_compilers(tmp_path):
    job = workflow_job("ci_toolchain.yml", "toolchain")
    build = next(
        step for step in job["steps"] if "pixi run build-tests" in step.get("run", "")
    )
    words = shlex.split(build["run"], comments=True)
    shell = words.index("-euc")
    assert words[shell - 4 : shell] == ["pixi", "run", "--", "bash"]
    prefix = str(tmp_path / "active-pixi")
    for row in job["strategy"]["matrix"]["include"]:
        for inherited in ("", "-Wl,--as-needed"):
            runtime = "/inherited/runtime" if inherited else ""
            result = subprocess.run(
                [
                    "bash",
                    "-euc",
                    'pixi() { printf "%s|%s\\n" "${LDFLAGS-}" "${LD_LIBRARY_PATH-}"; }\n'
                    "ctest() { pixi; }\n" + words[shell + 1],
                ],
                env={
                    "PATH": os.defpath,
                    "CC": row["cc"],
                    "CONDA_PREFIX": prefix,
                    "PIXI_ENVIRONMENT_NAME": "default",
                    "LDFLAGS": inherited,
                    "LD_LIBRARY_PATH": runtime,
                },
                check=True,
                capture_output=True,
                text=True,
            )
            expected = inherited
            if row["runner"] == "ubuntu-22.04":
                expected = f"-L{prefix}/lib" + (f" {inherited}" if inherited else "")
                runtime = f"{prefix}/lib" + (f":{runtime}" if runtime else "")
            assert result.stdout.splitlines() == [f"{expected}|{runtime}"] * 2


def test_windows_matrix_preserves_required_checks_and_selects_vs2022():
    job = workflow_job("ci_windows.yml", "build")
    rows = {row["name"]: row for row in job["strategy"]["matrix"]["include"]}
    for name, task in [
        ("windows-Release-cpp", "test"),
        ("windows-Release-python", "test-py"),
    ]:
        assert rows[name]["runner"] == "windows-2025-vs2026"
        assert rows[name]["task"] == task
        assert rows[name]["vsversion"] == ""
    baseline = rows["windows-2022-Release-cpp"]
    assert baseline["runner"] == "windows-2022"
    assert baseline["vsversion"] == "2022"
    assert baseline["task"] == "test"
    assert job["runs-on"] == "${{ matrix.runner }}"
    msvc = next(step for step in job["steps"] if step["name"] == "Setup MSVC")
    assert msvc["with"]["vsversion"] == "${{ matrix.vsversion }}"
    cache = next(step for step in job["steps"] if step["name"] == "Initialize sccache")
    assert "${{ matrix.part }}${{ matrix.vsversion }}" in cache["run"]
    assert len({(row["part"], row["vsversion"]) for row in rows.values()}) == len(rows)


def test_eigen_dependency_boundaries_and_environment_baselines():
    manifest = tomllib.loads((ROOT / "pixi.toml").read_text())
    for dependencies in (
        manifest["dependencies"],
        manifest["feature"]["gazebo"]["dependencies"],
        manifest["feature"]["gz-compat-base"]["dependencies"],
    ):
        assert dependencies["eigen"] == ">=3.4.0,<6"
    for name, environment in manifest["environments"].items():
        features = (
            environment if isinstance(environment, list) else environment["features"]
        )
        assert "eigen34" in features, name
    # The Eigen 5 toolchain lane rewrites this baseline before running tests.
    assert manifest["feature"]["eigen34"]["dependencies"]["eigen"] in {
        "==3.4.0",
        "5.*",
    }
    eigen = ET.parse(ROOT / "package.xml").find("depend[.='eigen']")
    assert eigen.attrib == {"version_gte": "3.4.0", "version_lt": "6"}
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "eigen3 >= 3.4.0, eigen3 < 6" in cmake


@pytest.mark.skipif(
    sys.platform != "linux", reason="The toolchain workflow runs on Linux"
)
def test_eigen_ci_replaces_only_baseline_pin(tmp_path):
    manifest = (ROOT / "pixi.toml").read_text()
    (tmp_path / "pixi.toml").write_text(manifest)
    job = workflow_job("ci_toolchain.yml", "toolchain")
    step = next(step for step in job["steps"] if step.get("if") == "matrix.eigen")
    subprocess.run(
        ["bash", "-euc", step["run"]],
        cwd=tmp_path,
        env={"PATH": os.defpath, "EIGEN": "5.*"},
        check=True,
        capture_output=True,
        text=True,
    )
    assert (tmp_path / "pixi.toml").read_text() == manifest.replace(
        'eigen = "==3.4.0"', 'eigen = "5.*"'
    )
