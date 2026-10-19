"""Check package support boundaries and native baseline CI coverage."""

import ast
import tomllib
from pathlib import Path

import yaml
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet

ROOT = Path(__file__).resolve().parents[1]


def test_package_metadata_support_boundaries():
    setup = next(
        node
        for node in ast.walk(
            ast.parse((ROOT / "setup.py").read_text(encoding="utf-8-sig"))
        )
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    )
    metadata = {
        keyword.arg: ast.literal_eval(keyword.value)
        for keyword in setup.keywords
        if keyword.arg in {"python_requires", "install_requires"}
    }

    python = SpecifierSet(metadata["python_requires"])
    assert "3.9" not in python
    assert "3.10" in python
    assert "3.14" in python
    numpy = next(
        Requirement(value)
        for value in metadata["install_requires"]
        if Requirement(value).name == "numpy"
    ).specifier
    assert "1.21.4" not in numpy
    assert "1.21.5" in numpy
    assert "2.3.0" in numpy

    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    assert project["tool"]["mypy"]["python_version"] == "3.10"
    build = {
        requirement.name: requirement.specifier
        for requirement in map(Requirement, project["build-system"]["requires"])
    }
    for name, rejected, accepted in [
        ("wheel", "0.47.0", "0.48.0"),
        ("ninja", "1.13.1", "1.13.2"),
        ("setuptools", "83.0.0", "84.0.0"),
        ("cmake", "4.4.3", "4.4.4"),
    ]:
        assert rejected not in build[name]
        assert accepted in build[name]
    assert "4.4.5" not in build["cmake"]


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
        if row["runner"] == "ubuntu-24.04" and row["distribution"] == "true"
    ]
    assert {(row["cc"], row["cxx"]) for row in baseline} == {
        ("gcc-13", "g++-13"),
        ("clang-18", "clang++-18"),
    }
    assert {row["runner"] for row in rows} == {"ubuntu-24.04"}
    assert {row["name"] for row in rows if row["distribution"] == "false"} == {
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
