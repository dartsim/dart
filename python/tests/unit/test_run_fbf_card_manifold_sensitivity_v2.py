import csv
import importlib.util
import io
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "scripts/run_fbf_card_manifold_sensitivity_v2.py"
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"


def _load_module():
    sys.path.insert(0, str(ROOT / "scripts"))
    spec = importlib.util.spec_from_file_location("card_manifold_runner", RUNNER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _string_literals(fragment: str) -> str:
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', fragment)
    return bytes("".join(literals), "utf-8").decode("unicode_escape")


def _header() -> list[str]:
    source = TRACE_SOURCE.read_text(encoding="utf-8")
    base_match = re.search(
        r"void printPerformanceHeader\(SolverContract contract\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*if \(contract",
        source,
        re.DOTALL,
    )
    assert base_match is not None
    suffix_match = re.search(
        r"if \(nativeManifoldSensitivityEnabled\(\)\) \{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert suffix_match is not None
    return (
        _string_literals(base_match.group(1)).strip()
        + _string_literals(suffix_match.group(1))
    ).split(",")


def _valid_row(module, mode: str, step: int) -> dict[str, str]:
    row = {field: "0" for field in _header()}
    row.update(
        {
            "step": str(step),
            "time": f"{step / 60.0:.17g}",
            "scenario": "card_house_26_settle_projectile_full",
            "solver": "exact_fbf",
            "solver_contract": "paper_cpu",
            "precision_contract": "float64_vs_paper_float32",
            "scene_contract": (
                "reconstructed_cards_density_1000_from_newton_default_"
                "cube_drop_author_overrides_unavailable"
            ),
            "baumgarte_contract": "split_impulse_no_velocity_baumgarte_vs_paper",
            "collision_frontend": "native",
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "10",
            "fixed_inner_sweeps_requested": "1",
            "step_size_persistence_enabled": "1",
            "step_size_recovery_growth_factor": "1.05",
            "step_size_persistence_used": "0",
            "step_size_persistence_request": "1",
            "row_operator_request": "contact_row_no_dense_snapshot",
            "row_operator_mode": "contact_row_no_dense_snapshot",
            "wall_ms": "2",
            "requested_threads": "1",
            "actual_threads": "1",
            "contacts": "2",
            "unique_colliding_body_pairs": "2",
            "penetration_depth_min": "0.001",
            "penetration_depth_median": "0.001",
            "penetration_depth_p95": "0.001",
            "penetration_depth_max": "0.001",
            "exact_diagnostics_contract": (
                "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
                "warm_fraction_over_step_contacts"
            ),
            "step_exact_solves": "1",
            "step_warm_starts": "1",
            "step_exact_failures": "0",
            "step_fallbacks": "0",
            "step_fbf_iterations": "1",
            "residual": "1e-8",
            "status": "success",
            "residual_primal_feasibility": "0",
            "residual_dual_feasibility": "1e-8",
            "residual_complementarity": "0",
            "accepted_gamma": "0.1",
            "safe_gamma": "0.1",
            "shrink_iterations": "0",
            "coupling_variation_ratio": "0",
            "warm_start_matched_contacts": "2",
            "warm_start_matched_fraction": "1",
            "phase": "settle" if step <= 402 else "projectile",
            "card_count": "26",
            "projectile_count": "0" if step <= 402 else "4",
            "finite_state": "1",
            "min_card_axis_up": "0.99",
            "min_center_height": "0.1",
            "max_card_horizontal_travel": "0",
            "max_projectile_speed": "0",
            "tracked_body": "card_house_l3_f0_left_body",
            "x": "0",
            "y": "0",
            "z": "1",
            "vx": "0",
            "vy": "0",
            "vz": "0",
            "up_z": "1",
            "max_outer_iterations": "200",
            "tolerance": "1e-6",
            "accept_outer_max_iterations": "1",
            "inner_local_iterations": "1",
            "adaptive_step_size_enabled": "1",
            "warm_start_enabled": "1",
            "projected_gradient_retry_enabled": "0",
            "dense_residual_polish_enabled": "0",
            "fallback_to_boxed_lcp_enabled": "0",
            "diagonal_seed_enabled": "0",
            "matrix_free_seed_enabled": "0",
            "step_size_scale": "1",
            "outer_relaxation": "1",
            "initial_gamma_contract": "automatic_safe_bound",
            "split_impulse_enabled": "1",
            "step_contact_row_delassus_products": "1",
            "step_parallel_contact_row_delassus_products": "0",
            "max_contact_row_participants_to_date": "1",
            "exact_contact_row_logical_cpus_to_date": "none",
            "max_phase_contact_row_logical_cpus_to_date": "none",
            "max_card_center_displacement_from_initial": "0.001",
            "min_card_orientation_alignment_from_initial": "0.99",
            "projectile_card_contacts": "0",
            "manifold_sensitivity_contract": (
                "card_house_native_manifold_sensitivity_v1"
            ),
            "requested_native_contact_manifold_mode": mode,
            "actual_native_contact_manifold_mode": mode,
            "collision_max_contacts": "512",
            "collision_max_contacts_per_pair": "4",
            "step_exact_max_iterations_accepted": "0",
            "step_internal_fbf_status": "success",
            "step_internal_fbf_best_iteration": "1",
            "step_internal_fbf_best_residual": "1e-8",
            "colliding_body_pair_labels": "a|b;c|d",
            "contact_multiplicity_by_body_pair": "a|b=1;c|d=1",
        }
    )
    return row


def _csv_text(rows: list[dict[str, str]]) -> str:
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=_header(), lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def test_frozen_protocol_hash_and_commands_are_exact():
    module = _load_module()

    assert module._protocol_contract_sha256() == (
        "eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9"
    )
    compact = module._command(Path("/tmp/fbf_paper_trace"), "compact")
    four = module._command(Path("/tmp/fbf_paper_trace"), "four_point_planar")
    assert compact[:3] == four[:3] == ["taskset", "--cpu-list", "8"]
    assert compact[-1] == "compact"
    assert four[-1] == "four_point_planar"
    assert compact[3:-1] == four[3:-1]


def test_parser_accepts_only_frozen_94_column_header():
    module = _load_module()
    text = _csv_text([_valid_row(module, "compact", 1)])

    header, rows = module._parse_trace(text)

    assert len(header) == 94
    assert len(rows) == 1
    assert tuple(header[-11:]) == module.SENSITIVITY_COLUMNS


def test_header_tampering_is_rejected():
    module = _load_module()
    text = _csv_text([_valid_row(module, "compact", 1)]).replace(
        "step,time,", "step,clock,", 1
    )

    with pytest.raises(module.EvidenceError, match="header hash"):
        module._parse_trace(text)


def test_pair_multiplicity_must_sum_to_contacts():
    module = _load_module()
    row = _valid_row(module, "compact", 1)
    row["contact_multiplicity_by_body_pair"] = "a|b=2;c|d=1"

    with pytest.raises(module.EvidenceError, match="multiplicities"):
        module._validate_pairs(row)


def test_pair_multiplicity_must_respect_per_pair_cap():
    module = _load_module()
    row = _valid_row(module, "compact", 1)
    row["contacts"] = "5"
    row["unique_colliding_body_pairs"] = "1"
    row["colliding_body_pair_labels"] = "a|b"
    row["contact_multiplicity_by_body_pair"] = "a|b=5"

    with pytest.raises(module.EvidenceError, match="1..4 cap"):
        module._validate_pairs(row)


def test_mode_readback_mismatch_is_rejected():
    module = _load_module()
    row = _valid_row(module, "compact", 1)
    row["actual_native_contact_manifold_mode"] = "four_point_planar"

    with pytest.raises(module.EvidenceError, match="actual_native"):
        module._validate_common_row(row, "compact", 1)


def test_nonfinite_physical_field_is_rejected():
    module = _load_module()
    row = _valid_row(module, "compact", 1)
    row["min_card_axis_up"] = "nan"

    with pytest.raises(module.EvidenceError, match="non-finite"):
        module._validate_common_row(row, "compact", 1)


def test_complete_strict_trajectory_is_valid():
    module = _load_module()
    rows = [_valid_row(module, "compact", step) for step in range(1, 601)]

    summary = module._mode_summary("compact", rows, 0)

    assert summary["strict_trajectory_valid"] is True
    assert summary["strict_success_rows"] == 600
    assert summary["accepted_cap_rows"] == 0


def test_aggregate_cap_counter_controls_multigroup_strictness():
    module = _load_module()
    rows = [_valid_row(module, "compact", step) for step in range(1, 601)]
    rows[0]["step_exact_solves"] = "2"
    rows[0]["step_exact_max_iterations_accepted"] = "1"

    summary = module._mode_summary("compact", rows, 0)

    assert summary["strict_trajectory_valid"] is False
    assert summary["accepted_cap_rows"] == 1
    assert summary["accepted_cap_exact_groups"] == 1
    assert summary["strict_success_rows"] == 599


def test_failed_prefix_requires_sole_terminal_failure():
    module = _load_module()
    rows = [_valid_row(module, "four_point_planar", step) for step in range(1, 4)]
    rows[-1].update(
        {
            "step_exact_solves": "0",
            "step_exact_failures": "1",
            "step_fbf_iterations": "183",
            "residual": "0.02",
            "status": "fbf_failed",
            "step_internal_fbf_status": "inner_solver_failed",
            "step_internal_fbf_best_iteration": "150",
            "step_internal_fbf_best_residual": "0.01",
        }
    )

    summary = module._mode_summary("four_point_planar", rows, 1)

    assert summary["strict_trajectory_valid"] is False
    assert summary["first_failed_step"] == 3
    assert summary["terminal"]["internal_fbf_status"] == "inner_solver_failed"


def test_full_return_one_terminal_convergence_gate_is_represented():
    module = _load_module()
    rows = [_valid_row(module, "four_point_planar", step) for step in range(1, 601)]
    rows[-1].update(
        {
            "step_exact_max_iterations_accepted": "1",
            "residual": "0.016",
            "status": "max_iterations_accepted",
            "step_internal_fbf_status": "max_iterations",
            "step_internal_fbf_best_residual": "0.01",
        }
    )

    summary = module._mode_summary("four_point_planar", rows, 1)

    assert summary["process_exit_class"] == (
        "complete_terminal_convergence_gate_failure"
    )
    assert summary["strict_trajectory_valid"] is False
    assert summary["first_failed_step"] is None


def test_full_return_one_with_passing_terminal_residual_is_rejected():
    module = _load_module()
    rows = [_valid_row(module, "compact", step) for step in range(1, 601)]

    with pytest.raises(module.EvidenceError, match="frozen v2 exit class"):
        module._mode_summary("compact", rows, 1)


def test_aggregate_failure_allows_later_last_group_success_status():
    module = _load_module()
    rows = [_valid_row(module, "compact", step) for step in range(1, 3)]
    rows[-1].update(
        {
            "step_exact_solves": "1",
            "step_exact_failures": "1",
            "status": "success",
            "step_internal_fbf_status": "success",
        }
    )

    summary = module._mode_summary("compact", rows, 1)

    assert summary["first_failed_step"] == 2
    assert summary["exact_failures"] == 1


def test_failed_prefix_rejects_multiple_terminal_failures():
    module = _load_module()
    rows = [_valid_row(module, "compact", step) for step in range(1, 3)]
    rows[-1].update(
        {
            "step_exact_solves": "1",
            "step_exact_failures": "2",
            "status": "fbf_failed",
            "step_internal_fbf_status": "inner_solver_failed",
        }
    )

    with pytest.raises(module.EvidenceError, match="sole failure"):
        module._mode_summary("compact", rows, 1)


def test_comparison_excludes_timing_from_verdict():
    module = _load_module()
    compact_rows = [_valid_row(module, "compact", step) for step in range(1, 601)]
    four_rows = [
        _valid_row(module, "four_point_planar", step) for step in range(1, 601)
    ]
    summaries = {
        "compact": module._mode_summary("compact", compact_rows, 0),
        "four_point_planar": module._mode_summary("four_point_planar", four_rows, 0),
    }

    comparison = module._comparison(summaries)

    assert comparison["comparison_artifact_integrity_valid"] is True
    assert comparison["timing_used_in_verdict"] is False
    assert "wall" not in comparison["deltas"]


def test_fresh_output_directory_rejects_existing_path(tmp_path):
    module = _load_module()

    with pytest.raises(module.EvidenceError, match="must be fresh"):
        module._prepare_output(tmp_path)


def test_main_writes_complete_indexed_pair_bundle(tmp_path, monkeypatch):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"binary")
    output = tmp_path / "evidence"
    compact = _csv_text([_valid_row(module, "compact", step) for step in range(1, 601)])
    four = _csv_text(
        [_valid_row(module, "four_point_planar", step) for step in range(1, 601)]
    )
    results = iter(((compact, "", 0), (four, "", 0)))
    identity = {"stable": True}
    monkeypatch.setattr(module, "_execution_identity", lambda unused: identity)
    monkeypatch.setattr(module, "_run_captured", lambda command, timeout: next(results))

    assert (
        module.main(
            ["--binary", str(binary), "--output-dir", str(output), "--timeout", "1"]
        )
        == 0
    )
    comparison = json.loads((output / "comparison.json").read_text())
    metadata = json.loads((output / "metadata.json").read_text())
    index = json.loads((output / "artifact-index.json").read_text())
    assert comparison["comparison_artifact_integrity_valid"] is True
    assert metadata["comparison_artifact_integrity_valid"] is True
    assert set(index["files"]) == {
        "REPORT.md",
        "compact/raw.csv",
        "compact/stderr.txt",
        "comparison.json",
        "four_point_planar/raw.csv",
        "four_point_planar/stderr.txt",
        "invocation.json",
        "metadata.json",
        "summary.json",
    }
    for relative, record in index["files"].items():
        assert record["sha256"] == module._sha256_file(output / relative)
