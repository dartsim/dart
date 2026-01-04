import importlib.util
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def run_cpp_example():
    repo_root = Path(__file__).resolve().parents[3]
    script_path = repo_root / "scripts" / "run_cpp_example.py"

    spec = importlib.util.spec_from_file_location("run_cpp_example", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("old_name", "new_name"),
    [
        ("add_delete_skels", "viz_add_delete_skels"),
        ("atlas_simbicon", "control_walking_humanoid"),
        ("biped_stand", "control_balance_biped"),
        ("box_stacking", "collision_box_stacking"),
        ("capsule_ground_contact", "collision_capsule_ground_contact"),
        ("csv_logger", "tool_csv_logger"),
        ("drag_and_drop", "viz_drag_and_drop"),
        ("empty", "viz_empty"),
        ("fetch", "model_fetch"),
        ("headless_simulation", "perf_headless_simulation"),
        ("heightmap", "collision_heightmap"),
        ("hybrid_dynamics", "control_actuator_modes"),
        ("imgui", "viz_imgui"),
        ("joint_lcp_solvers", "tool_lcp_solvers"),
        ("lcp_solvers", "tool_lcp_solvers"),
        ("mixed_chain", "hybrid_mixed_chain"),
        ("operational_space_control", "control_operational_space"),
        ("point_cloud", "viz_point_cloud"),
        ("polyhedron_visual", "viz_polyhedron_visual"),
        ("rigid_shapes", "collision_rigid_shapes"),
        ("tinkertoy", "viz_tinkertoy"),
        ("tool_point_cloud", "viz_point_cloud"),
        ("unified_loading", "io_unified_loading"),
        ("vehicle", "control_vehicle"),
        ("wam_ikfast", "ik_analytic_wam"),
    ],
)
def test_normalize_target_deprecates_renames(
    run_cpp_example, capsys, old_name, new_name
):
    assert run_cpp_example._normalize_target(old_name) == new_name
    captured = capsys.readouterr()
    assert old_name in captured.err
    assert new_name in captured.err


def test_normalize_target_passthrough(run_cpp_example, capsys):
    assert (
        run_cpp_example._normalize_target("control_balance_biped")
        == "control_balance_biped"
    )
    assert capsys.readouterr().err == ""


def test_list_examples_filters_cmake_lists(run_cpp_example, tmp_path):
    (tmp_path / "alpha").mkdir()
    (tmp_path / "alpha" / "CMakeLists.txt").write_text("cmake_minimum_required")
    (tmp_path / "beta").mkdir()
    (tmp_path / "beta" / "README.md").write_text("not a cmake example")
    (tmp_path / "gamma").mkdir()
    (tmp_path / "gamma" / "CMakeLists.txt").write_text("cmake_minimum_required")

    assert run_cpp_example._list_examples(tmp_path) == ["alpha", "gamma"]


def test_resolve_build_and_binary(run_cpp_example):
    resolved_build_target, resolved_binary = run_cpp_example._resolve_build_and_binary(
        "control_walking_humanoid"
    )
    assert resolved_build_target == "control_walking_humanoid"
    assert resolved_binary == "control_walking_humanoid"
