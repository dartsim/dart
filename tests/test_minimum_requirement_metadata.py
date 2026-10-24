"""Check package support boundaries and native baseline CI coverage."""

import os
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


def test_urdfdom_requirement_metadata():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependency = next(
        dep for dep in package.findall("depend") if dep.text == "liburdfdom-dev"
    )
    assert dependency.get("version_gte") == "4.0.0"

    pixi = tomllib.loads((ROOT / "pixi.toml").read_text())
    for dependencies in (
        pixi["dependencies"],
        pixi["feature"]["gz-compat-base"]["dependencies"],
    ):
        versions = SpecifierSet(dependencies["urdfdom"])
        assert "3.1.1" not in versions
        assert "6.0.0" in versions
    assert "4.0.0" in SpecifierSet(
        pixi["feature"]["gz-compat-base"]["dependencies"]["urdfdom"]
    )


def test_openssl_requirement_metadata():
    pixi = tomllib.loads((ROOT / "pixi.toml").read_text())
    versions = SpecifierSet(pixi["dependencies"]["openssl"])
    assert "3.0.12" not in versions
    assert "3.0.13" in versions
    assert "4.0.0" not in versions


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
    assert "1.26.3" not in numpy
    assert "1.26.4" in numpy
    assert "2.3.0" in numpy

    pixi = tomllib.loads((ROOT / "pixi.toml").read_text())
    pixi_numpy = SpecifierSet(pixi["dependencies"]["numpy"])
    assert "1.26.3" not in pixi_numpy
    assert "1.26.4" in pixi_numpy
    assert "2.3.0" in pixi_numpy
    assert pixi["dependencies"]["nanobind"] == "3.1.*"
    assert not {"py310-wheel", "py311-wheel"} & pixi["environments"].keys()

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
    cmake = SpecifierSet(skbuild["cmake"]["version"])
    assert "3.26.0" not in cmake
    assert "3.27.9" not in cmake
    assert "3.28.0" in cmake
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
    baseline = [
        row
        for row in rows
        if row["runner"] == "ubuntu-24.04"
        and row["distribution"] == "true"
        and "eigen" not in row
    ]
    assert {(row["cc"], row["cxx"]) for row in baseline} == {
        ("gcc-13", "g++-13"),
        ("clang-18", "clang++-18"),
    }
    assert {row["runner"] for row in rows} == {"ubuntu-24.04"}
    assert {row["name"] for row in rows if row["distribution"] == "false"} == {
        "gcc 16",
        "clang 19",
    }
    # Packagers build against Eigen 5, which the lockfile does not cover.
    assert [row["eigen"] for row in rows if "eigen" in row] == ["5.*"]
    system_nanobind = [row for row in rows if row.get("system_nanobind") == "true"]
    assert len(system_nanobind) == 1
    assert "eigen" in system_nanobind[0]
    assert any(
        step.get("if") == "matrix.eigen" and "pixi run test-py" in step.get("run", "")
        for step in job["steps"]
    )
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
    assert "-DDART_USE_SYSTEM_NANOBIND=ON" in build["run"]
    assert "matrix.system_nanobind" in build["run"]


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
