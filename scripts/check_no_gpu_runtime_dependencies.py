#!/usr/bin/env python3
"""Check that core package manifests do not depend on GPU runtimes.

Phase 5 GPU support must ship as an optional sidecar. The default DART Pixi
dependencies, features included by the default Pixi environment, and official
dartpy wheel metadata must not acquire CUDA, SYCL, ROCm/HIP, or other
compute-runtime dependencies. Explicit opt-in Pixi features or environments may
carry those dependencies because they are not part of the default install
surface.
"""

from __future__ import annotations

import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]

MANIFESTS = (
    REPO_ROOT / "pixi.toml",
    REPO_ROOT / "pyproject.toml",
)

PROHIBITED_PACKAGE_TERMS = {
    "adaptivecpp",
    "cuda",
    "cuda-version",
    "cudatoolkit",
    "cupy",
    "dpcpp",
    "hip",
    "kokkos",
    "nvidia-cuda",
    "oneapi",
    "rocm",
    "sycl",
    "triton",
    "warp-lang",
}

PYPROJECT_SCANNED_PATHS = {
    ("build-system", "requires"),
    ("project", "dependencies"),
    ("project", "optional-dependencies"),
    ("tool", "scikit-build", "cmake", "define"),
}


@dataclass(frozen=True)
class Violation:
    path: str
    key_path: str
    value: str
    term: str


def _load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as f:
        return tomllib.load(f)


def _normalize(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")


def _matching_term(value: str) -> str | None:
    normalized = _normalize(value)
    for term in sorted(PROHIBITED_PACKAGE_TERMS):
        if term in normalized:
            return term
    return None


def _join_path(parts: tuple[str, ...]) -> str:
    return ".".join(parts)


def _iter_values(value: Any, path: tuple[str, ...]):
    if isinstance(value, dict):
        for key, nested in value.items():
            yield from _iter_values(nested, (*path, str(key)))
    elif isinstance(value, list):
        for index, nested in enumerate(value):
            yield from _iter_values(nested, (*path, str(index)))
    elif isinstance(value, str):
        yield path, value


def _environment_features(environment: Any) -> set[str]:
    if isinstance(environment, list):
        return {str(feature) for feature in environment}
    if isinstance(environment, dict):
        features = environment.get("features", [])
        if isinstance(features, str):
            return {features}
        if isinstance(features, list):
            return {str(feature) for feature in features}
    return set()


def _default_pixi_features(data: dict[str, Any]) -> set[str]:
    environments = data.get("environments", {})
    if not isinstance(environments, dict):
        return set()
    return _environment_features(environments.get("default"))


def _is_pixi_dependency_path(
    path: tuple[str, ...],
    data: dict[str, Any],
) -> bool:
    if path[:1] == ("feature",):
        return (
            len(path) > 2
            and path[1] in _default_pixi_features(data)
            and ("dependencies" in path[2:] or "pypi-dependencies" in path[2:])
        )
    return "dependencies" in path or "pypi-dependencies" in path


def _is_pyproject_scanned_path(path: tuple[str, ...]) -> bool:
    return any(path[: len(prefix)] == prefix for prefix in PYPROJECT_SCANNED_PATHS)


def _is_scanned_path(
    path: Path,
    data: dict[str, Any],
    key_path: tuple[str, ...],
) -> bool:
    if path.name == "pixi.toml":
        return _is_pixi_dependency_path(key_path, data)
    if path.name == "pyproject.toml":
        return _is_pyproject_scanned_path(key_path)
    return False


def find_manifest_violations(path: Path, data: dict[str, Any]) -> list[Violation]:
    violations: list[Violation] = []
    rel_path = (
        path.relative_to(REPO_ROOT).as_posix() if path.is_absolute() else str(path)
    )

    for key_path, value in _iter_values(data, ()):
        if not _is_scanned_path(path, data, key_path):
            continue

        key_term = _matching_term(_join_path(key_path))
        value_term = _matching_term(value)
        term = key_term or value_term
        if term is None:
            continue

        violations.append(
            Violation(
                path=rel_path,
                key_path=_join_path(key_path),
                value=value,
                term=term,
            )
        )

    return violations


def find_no_gpu_runtime_dependency_violations() -> list[Violation]:
    violations: list[Violation] = []
    for path in MANIFESTS:
        violations.extend(find_manifest_violations(path, _load_toml(path)))
    return violations


def run_self_tests() -> None:
    ok_pixi = {"dependencies": {"numpy": ">=2"}}
    if find_manifest_violations(REPO_ROOT / "pixi.toml", ok_pixi):
        raise AssertionError("benign pixi dependency self-test failed")

    bad_pixi = {"dependencies": {"cuda-version": "12.*"}}
    if not find_manifest_violations(REPO_ROOT / "pixi.toml", bad_pixi):
        raise AssertionError("pixi GPU dependency self-test failed")

    cuda_feature_pixi = {
        "feature": {"cuda": {"dependencies": {"cuda-version": "12.*"}}}
    }
    if find_manifest_violations(REPO_ROOT / "pixi.toml", cuda_feature_pixi):
        raise AssertionError("optional pixi CUDA feature self-test failed")

    default_feature_pixi = {
        "feature": {"py-default": {"dependencies": {"cuda-version": "12.*"}}},
        "environments": {"default": ["py-default"]},
    }
    if not find_manifest_violations(REPO_ROOT / "pixi.toml", default_feature_pixi):
        raise AssertionError("default pixi feature GPU dependency self-test failed")

    bad_pyproject = {"build-system": {"requires": ["cupy>=13"]}}
    if not find_manifest_violations(REPO_ROOT / "pyproject.toml", bad_pyproject):
        raise AssertionError("pyproject GPU dependency self-test failed")


def main() -> int:
    run_self_tests()
    violations = find_no_gpu_runtime_dependency_violations()
    if not violations:
        print("No-GPU runtime dependency check passed.")
        return 0

    print("No-GPU runtime dependency check failed:", file=sys.stderr)
    for violation in violations:
        print(
            f"{violation.path}: {violation.key_path}: "
            f"GPU runtime term '{violation.term}' is not allowed in core package "
            f"metadata\n  {violation.value}",
            file=sys.stderr,
        )
    return 1


if __name__ == "__main__":
    sys.exit(main())
