from __future__ import annotations

import importlib
import os
from pathlib import Path

import pytest

import dartpy as dart


def _cache_reports_experimental_disabled() -> bool:
    override = os.environ.get("DART_BUILD_SIMULATION_EXPERIMENTAL_OVERRIDE")
    if override and override.lower() in {"0", "false", "no", "off"}:
        return True

    repo_root = Path(__file__).resolve().parents[4]
    build_type = (
        os.environ.get("BUILD_TYPE")
        or os.environ.get("CMAKE_BUILD_TYPE")
        or "Release"
    )
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    candidates: list[Path] = []
    if os.environ.get("CMAKE_BUILD_DIR"):
        candidates.append(Path(os.environ["CMAKE_BUILD_DIR"]) / "CMakeCache.txt")
    candidates.extend(
        [
            repo_root / "build" / pixi_env / "cpp" / build_type / "CMakeCache.txt",
            repo_root / "build" / pixi_env / "cpp" / "CMakeCache.txt",
        ]
    )

    for cache in candidates:
        if not cache.is_file():
            continue
        for line in cache.read_text(encoding="utf-8").splitlines():
            if not line.startswith("DART_BUILD_SIMULATION_EXPERIMENTAL:"):
                continue
            return line.rsplit("=", 1)[-1].strip().lower() in {
                "0",
                "false",
                "no",
                "off",
            }
    return False


def _simulation_experimental():
    try:
        module = importlib.import_module("dartpy.simulation_experimental")
    except ModuleNotFoundError as exc:
        if _cache_reports_experimental_disabled():
            pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
        raise AssertionError(
            "dartpy.simulation_experimental should be available when "
            "dart-simulation-experimental is built"
        ) from exc
    if not hasattr(module, "World"):
        if _cache_reports_experimental_disabled():
            pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
        raise AssertionError(
            "dartpy.simulation_experimental imported but did not expose "
            "the experimental World binding"
        )
    return module


def test_experimental_world_module_is_separate_from_legacy_simulation():
    sx = _simulation_experimental()

    assert dart.simulation_experimental is sx
    assert sx is not dart.simulation
    assert not hasattr(dart, "next")


def test_experimental_world_smoke():
    sx = _simulation_experimental()

    world = sx.World()
    assert not world.is_simulation_mode()
    assert world.get_multi_body_count() == 0
    assert world.get_rigid_body_count() == 0

    multi_body = world.add_multi_body("robot")
    assert multi_body.get_name() == "robot"
    assert multi_body.get_link_count() == 0
    assert multi_body.get_joint_count() == 0
    assert multi_body.get_dof_count() == 0

    multi_body.set_name("renamed_robot")
    assert multi_body.get_name() == "renamed_robot"
    assert world.get_multi_body("renamed_robot").get_name() == "renamed_robot"
    assert world.get_multi_body("missing") is None
    assert world.get_multi_body_count() == 1

    rigid_body = world.add_rigid_body("box")
    assert rigid_body.get_name() == "box"
    assert world.has_rigid_body("box")
    assert not world.has_rigid_body("missing")
    assert world.get_rigid_body_count() == 1

    world.enter_simulation_mode()
    assert world.is_simulation_mode()
    world.update_kinematics()

    world.clear()
    assert not world.is_simulation_mode()
    assert world.get_multi_body_count() == 0
    assert world.get_rigid_body_count() == 0
