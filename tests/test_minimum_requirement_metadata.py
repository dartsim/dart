"""Check package support boundaries and native baseline CI coverage."""

import ast
import os
import shlex
import subprocess
import sys
import tomllib
from pathlib import Path

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
