import importlib.util
import math
import pathlib
import tempfile
import types

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
RUNNER_PATH = ROOT / "scripts" / "run_fbf_mujoco_evidence.py"
BASELINE_PATH = (
    ROOT / "tests" / "benchmark" / "integration" / "fbf_paper_mujoco_baseline.py"
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = load(RUNNER_PATH, "run_fbf_mujoco_evidence_test")
BASELINE = load(BASELINE_PATH, "fbf_paper_mujoco_baseline_test")


def sample_row(**overrides):
    row = {
        "run_id": "backspin-r001",
        "repetition": 1,
        "step": 1,
        "sim_time_s": 1.0 / 60.0,
        "scenario": "backspin",
        "timed_step": 1,
        "step_wall_ms": 2.0,
        "step_wall_ns": 2_000_000,
        "solver_iterations_sum": 2,
        "contacts": 1,
        "z_m": 0.25,
        "x_m": 0.0,
        "vx_mps": BASELINE.BACKSPIN_ANALYTIC_V_INF,
        "wy_rad_s": BASELINE.BACKSPIN_ANALYTIC_OMEGA_INF,
        "down_slope_displacement_m": math.nan,
        "radial_distance_m": math.nan,
        "airborne": 0,
    }
    row.update(overrides)
    return row


def test_xml_encodes_only_the_published_solver_contract():
    for xml in (
        BASELINE.build_backspin_xml(BASELINE.PAPER_DT),
        BASELINE.build_incline_xml(BASELINE.PAPER_DT, 0.5),
        BASELINE.build_turntable_xml(BASELINE.PAPER_DT, 0.2),
    ):
        assert 'solver="Newton"' in xml
        assert 'cone="elliptic"' in xml
        assert 'iterations="500"' in xml
        assert "tolerance=" not in xml


def test_default_durations_separate_published_and_reconstructed_horizons():
    assert BASELINE.SCENARIO_DEFAULT_DURATIONS_S["backspin"] == pytest.approx(
        130.0 / 60.0
    )
    assert BASELINE.SCENARIO_DEFAULT_DURATIONS_S["incline_mu_0_4"] == 2.0
    assert BASELINE.SCENARIO_DEFAULT_DURATIONS_S["turntable_mu_0_5_omega_2"] == 4.0
    unknowns = RUNNER.SCENE_CONTRACTS["turntable_mu_0_5_omega_2"][
        "unknown_or_unpublished"
    ]
    assert "simulation duration" in unknowns


def test_cpu_list_parser_accepts_ranges_and_rejects_ambiguous_input():
    assert RUNNER.parse_cpu_list("4,6-8") == [4, 6, 7, 8]
    with pytest.raises(ValueError):
        RUNNER.parse_cpu_list("8-6")
    with pytest.raises(ValueError):
        RUNNER.parse_cpu_list("4,,6")


def test_percentile_uses_linear_interpolation():
    assert RUNNER.percentile([1.0, 2.0], 0.95) == pytest.approx(1.95)
    assert RUNNER.percentile([1.0, 2.0, 3.0], 0.5) == 2.0
    assert math.isnan(RUNNER.percentile([], 0.95))


def test_json_safe_emits_strict_nulls_for_nonfinite_metrics():
    value = RUNNER.json_safe({"missing": math.nan, "nested": [math.inf, 1.0]})
    assert value == {"missing": None, "nested": [None, 1.0]}


def test_summary_keeps_paper_numbers_reference_only():
    initial = sample_row(step=0, timed_step=0, step_wall_ms=0.0, step_wall_ns=0)
    final = sample_row(step=1, step_wall_ms=4.0, x_m=-1.0, z_m=1.0, airborne=1)
    summary = RUNNER.summarize_scenario("backspin", [initial, final])
    assert summary["mean_step_ms"] == 4.0
    assert summary["physical_outcome"] == "ejected_above_plane"
    assert summary["paper_implied_mujoco_ms_reference"] == pytest.approx(1.8)
    assert summary["paper_comparable"] is False
    assert summary["backspin_final_vx_abs_error_mps"] == pytest.approx(0.0)


def test_turntable_outcome_is_physical_not_a_timing_claim():
    initial = sample_row(
        scenario="turntable_mu_0_5_omega_2",
        run_id="turntable-r001",
        step=0,
        timed_step=0,
        step_wall_ms=0.0,
        step_wall_ns=0,
        radial_distance_m=1.0,
    )
    final = sample_row(
        scenario="turntable_mu_0_5_omega_2",
        run_id="turntable-r001",
        radial_distance_m=2.2,
        airborne=1,
    )
    assert (
        RUNNER.classify_outcome("turntable_mu_0_5_omega_2", [initial, final])
        == "ejected_or_off_support"
    )


def test_validation_checks_newton_elliptic_and_complete_steps():
    mujoco = types.SimpleNamespace(
        mjtSolver=types.SimpleNamespace(mjSOL_NEWTON=2),
        mjtCone=types.SimpleNamespace(mjCONE_ELLIPTIC=1),
    )
    baseline = types.SimpleNamespace(PAPER_MUJOCO_MAX_ITERATIONS=500)
    rows = []
    for step in (0, 1):
        row = {name: 0.0 for name in RUNNER.RAW_FIELDS}
        row.update(
            {
                "step": step,
                "sim_time_s": step / 60.0,
                "step_wall_ns": 0 if step == 0 else 1,
                "solver_iterations_max_island": 1,
                "quat_w": 1.0,
            }
        )
        rows.append(row)
    result = {
        "scenario": "backspin",
        "model_options": {
            "solver_enum": 2,
            "cone_enum": 1,
            "iterations": 500,
            "numeric_dtype": "float64",
        },
        "rows": rows,
        "steps": 1,
        "warnings": [],
    }
    assert RUNNER.validate_results(mujoco, baseline, ["backspin"], 1, [result]) == []
    result["model_options"]["cone_enum"] = 0
    assert "cone is not elliptic" in " ".join(
        RUNNER.validate_results(mujoco, baseline, ["backspin"], 1, [result])
    )


def test_report_leads_with_non_parity_verdict():
    metadata = {
        "software": {"mujoco_version": "3.10.0", "numeric_precision": "float64"},
        "host": {"cpu": {"model": "test", "process_affinity": [4]}},
        "invocation": {"repetitions": 1},
        "paper_contract": {"non_comparability_reasons": ["different precision"]},
    }
    summary = RUNNER.summarize_scenario(
        "backspin",
        [
            sample_row(step=0, timed_step=0, step_wall_ms=0.0, step_wall_ns=0),
            sample_row(),
        ],
    )
    report = RUNNER.report_text(metadata, [summary])
    assert "not paper-parity or apples-to-apples evidence" in report
    assert "No local/paper speed ratio is computed" in report


def test_force_invalidates_all_owned_artifacts_but_preserves_unrelated_files():
    with tempfile.TemporaryDirectory() as directory:
        output = pathlib.Path(directory)
        for name in RUNNER.OWNED_ARTIFACTS:
            (output / name).write_text("stale", encoding="utf-8")
        unrelated = output / "notes.txt"
        unrelated.write_text("keep", encoding="utf-8")

        with pytest.raises(SystemExit):
            RUNNER.prepare_output(output, force=False)
        RUNNER.prepare_output(output, force=True)

        assert not any((output / name).exists() for name in RUNNER.OWNED_ARTIFACTS)
        assert unrelated.read_text(encoding="utf-8") == "keep"
