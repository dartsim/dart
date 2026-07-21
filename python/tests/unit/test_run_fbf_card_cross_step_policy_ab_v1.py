import csv
import importlib.util
import io
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "scripts/run_fbf_card_cross_step_policy_ab_v1.py"
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"


def _load_module():
    sys.path.insert(0, str(ROOT / "scripts"))
    spec = importlib.util.spec_from_file_location("card_policy_ab_runner", RUNNER)
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
        r"if \(crossStepPolicyEvidenceEnabled\(\)\) \{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert suffix_match is not None
    return (
        _string_literals(base_match.group(1)).strip()
        + _string_literals(suffix_match.group(1))
    ).split(",")


def _valid_row(policy: str, step: int) -> dict[str, str]:
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
            "baumgarte_contract": ("split_impulse_no_velocity_baumgarte_vs_paper"),
            "collision_frontend": "native",
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "10",
            "fixed_inner_sweeps_requested": "1",
            "step_size_persistence_enabled": "1",
            "step_size_recovery_growth_factor": "1.05",
            "step_size_persistence_used": "0",
            "step_size_persistence_request": "0.1",
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
            "coupling_variation_ratio": "0.1",
            "warm_start_matched_contacts": "2",
            "warm_start_matched_fraction": "1",
            "phase": "settle",
            "card_count": "26",
            "projectile_count": "0",
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
            "cross_step_policy_contract": "card_house_cross_step_policy_ab_v1",
            "requested_cross_step_policy": policy,
            "actual_cross_step_policy": policy,
            "requested_native_contact_manifold_mode": "compact",
            "actual_native_contact_manifold_mode": "compact",
            "collision_max_contacts": "512",
            "collision_max_contacts_per_pair": "4",
            "step_exact_attempts": "1",
            "step_exact_max_iterations_accepted": "0",
            "step_warm_start_gamma_caps": "0",
            "step_unconverged_warm_start_cache_skips": "0",
            "worst_exact_residual_to_date": "1e-8",
            "last_exact_diagnostics_contract": ("last_exact_group_only_single_group"),
            "last_exact_initial_natural_map_residual": "nan",
            "last_exact_final_natural_map_residual": "nan",
            "last_exact_uncapped_initial_gamma": "0.1",
            "last_exact_warm_start_gamma_cap_applied": "0",
            "warm_start_match_mode": "either_body_local_feature",
            "warm_start_match_distance": "0.025",
            "warm_start_normal_cosine": "0.9",
            "strict_warm_start_match_distance": "0",
            "warm_start_max_age": "-1",
            "persistent_gamma_safe_bound_scale": "1",
            "minimum_adaptive_gamma": "nan",
            "maximum_adaptive_gamma": "nan",
            "warm_start_gamma_natural_residual_threshold": "nan",
            "warm_start_gamma_cap": "nan",
            "persist_uncapped_gamma_after_warm_cap": "0",
            "require_residual_improvement_for_unconverged_cache_save": "0",
            "coupling_variation_tolerance": "0.9",
            "shrink_factor": "0.7",
            "max_step_shrink_iterations": "20",
        }
    )
    if policy == "author_policy_inspired_b3f3c5c":
        row.update(
            {
                "step_size_recovery_growth_factor": f"{1.0 / 0.7:.17g}",
                "step_size_scale": "10",
                "last_exact_initial_natural_map_residual": "1e-5",
                "last_exact_final_natural_map_residual": "1e-6",
                "warm_start_match_mode": "ordered_body_b_local_feature",
                "warm_start_match_distance": "0.02",
                "strict_warm_start_match_distance": "1",
                "warm_start_max_age": "3",
                "persistent_gamma_safe_bound_scale": "10",
                "minimum_adaptive_gamma": "1e-6",
                "maximum_adaptive_gamma": "1e6",
                "warm_start_gamma_natural_residual_threshold": "1e-4",
                "warm_start_gamma_cap": "1e4",
                "persist_uncapped_gamma_after_warm_cap": "1",
                "require_residual_improvement_for_unconverged_cache_save": "1",
                "max_step_shrink_iterations": "8",
            }
        )
    return row


def _csv_text(rows: list[dict[str, str]]) -> str:
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=_header(), lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def test_frozen_protocol_hash_command_and_order_are_exact():
    module = _load_module()

    assert module._protocol_contract_sha256() == (
        "a2babf6d28f802fce85d49abf4231c5ac94f3db62747f73a093cbba5ee3db4b5"
    )
    assert module.POLICIES == (
        "dart_current",
        "author_policy_inspired_b3f3c5c",
    )
    current = module._command(Path("/tmp/fbf_paper_trace"), "dart_current")
    author = module._command(
        Path("/tmp/fbf_paper_trace"), "author_policy_inspired_b3f3c5c"
    )
    assert (
        current[:4]
        == author[:4]
        == [
            "taskset",
            "--cpu-list",
            "8",
            "/tmp/fbf_paper_trace",
        ]
    )
    assert (
        current[4:-1]
        == author[4:-1]
        == [
            "card_house_26_settle_projectile_full",
            "exact_fbf",
            "1",
            "90",
            "nan",
            "performance",
            "default",
            "default",
            "1",
            "paper_cpu",
            "native",
            "default",
            "0",
            "0",
            "default",
        ]
    )
    assert current[-1] == "dart_current"
    assert author[-1] == "author_policy_inspired_b3f3c5c"


def test_parser_accepts_only_frozen_115_column_header():
    module = _load_module()
    text = _csv_text([_valid_row("dart_current", 1)])

    header, rows = module._parse_trace(text)

    assert len(header) == 115
    assert len(rows) == 1
    assert tuple(header[-32:]) == module.POLICY_COLUMNS


def test_header_tampering_is_rejected():
    module = _load_module()
    text = _csv_text([_valid_row("dart_current", 1)]).replace(
        "step,time,", "step,clock,", 1
    )

    with pytest.raises(module.EvidenceError, match="header hash"):
        module._parse_trace(text)


def test_current_policy_requires_intentional_nan_natural_diagnostics():
    module = _load_module()
    row = _valid_row("dart_current", 1)

    module._validate_common_row(row, "dart_current", 1)
    row["last_exact_initial_natural_map_residual"] = "1e-5"

    with pytest.raises(module.EvidenceError, match="is not NaN"):
        module._validate_common_row(row, "dart_current", 1)


def test_author_policy_requires_finite_natural_diagnostics():
    module = _load_module()
    row = _valid_row("author_policy_inspired_b3f3c5c", 1)
    row["last_exact_final_natural_map_residual"] = "nan"

    with pytest.raises(module.EvidenceError, match="non-finite"):
        module._validate_common_row(row, "author_policy_inspired_b3f3c5c", 1)


def test_actual_policy_readback_mismatch_is_rejected():
    module = _load_module()
    row = _valid_row("dart_current", 1)
    row["actual_cross_step_policy"] = "author_policy_inspired_b3f3c5c"

    with pytest.raises(module.EvidenceError, match="actual_cross_step_policy"):
        module._validate_common_row(row, "dart_current", 1)


def test_attempt_counter_identity_and_positive_contacts_are_structural():
    module = _load_module()
    row = _valid_row("dart_current", 1)
    row["step_exact_attempts"] = "2"

    with pytest.raises(module.EvidenceError, match="solves plus failures"):
        module._validate_common_row(row, "dart_current", 1)

    row = _valid_row("dart_current", 1)
    row["contacts"] = "0"
    with pytest.raises(module.EvidenceError, match="1..512 contacts"):
        module._validate_common_row(row, "dart_current", 1)


def test_zero_return_requires_all_90_rows():
    module = _load_module()
    rows = [_valid_row("dart_current", step) for step in range(1, 4)]

    with pytest.raises(module.EvidenceError, match="not a complete trace"):
        module._arm_summary("dart_current", rows, 0)


def test_return_one_accepts_terminal_exact_failure_prefix():
    module = _load_module()
    rows = [_valid_row("dart_current", step) for step in range(1, 4)]
    rows[-1].update(
        {
            "step_exact_attempts": "3",
            "step_exact_solves": "1",
            "step_exact_failures": "2",
            "status": "fbf_failed",
        }
    )

    summary = module._arm_summary("dart_current", rows, 1)

    assert summary["process_exit_class"] == "exact_failure_prefix"
    assert summary["first_failed_step"] == 3
    assert summary["strict_gate"] is False


def test_return_one_rejects_nonterminal_exact_failure():
    module = _load_module()
    rows = [_valid_row("dart_current", step) for step in range(1, 4)]
    rows[1].update(
        {
            "step_exact_attempts": "2",
            "step_exact_failures": "1",
        }
    )

    with pytest.raises(module.EvidenceError, match="sole failing row"):
        module._arm_summary("dart_current", rows, 1)


def test_return_one_accepts_complete_terminal_convergence_gate_failure():
    module = _load_module()
    policy = "author_policy_inspired_b3f3c5c"
    rows = [_valid_row(policy, step) for step in range(1, 91)]
    rows[-1].update(
        {
            "residual": "2e-6",
            "worst_exact_residual_to_date": "2e-6",
            "step_exact_max_iterations_accepted": "1",
            "status": "max_iterations_accepted",
        }
    )

    summary = module._arm_summary(policy, rows, 1)

    assert summary["process_exit_class"] == (
        "complete_terminal_convergence_gate_failure"
    )
    assert summary["strict_gate"] is False


def test_strict_gate_promotes_only_complete_author_arm():
    module = _load_module()
    current_rows = [_valid_row("dart_current", step) for step in range(1, 91)]
    author_rows = [
        _valid_row("author_policy_inspired_b3f3c5c", step) for step in range(1, 91)
    ]

    current = module._arm_summary("dart_current", current_rows, 0)
    author = module._arm_summary("author_policy_inspired_b3f3c5c", author_rows, 0)

    assert current["strict_gate"] is True
    assert current["promotion_to_separately_preregistered_600"] is False
    assert author["strict_gate"] is True
    assert author["promotion_to_separately_preregistered_600"] is True


def test_failures_fallbacks_caps_and_residual_are_strict_not_integrity_gates():
    module = _load_module()
    policy = "author_policy_inspired_b3f3c5c"
    rows = [_valid_row(policy, step) for step in range(1, 91)]
    rows[0].update(
        {
            "step_exact_attempts": "2",
            "step_exact_failures": "1",
            "step_fallbacks": "1",
            "step_exact_max_iterations_accepted": "1",
            "worst_exact_residual_to_date": "2e-6",
        }
    )

    summary = module._arm_summary(policy, rows, 0)

    assert summary["process_exit_class"] == "complete_exit_zero"
    assert summary["strict_gate"] is False
    assert summary["promotion_to_separately_preregistered_600"] is False


def test_gamma_caps_and_cache_skips_do_not_fail_strict_gate():
    module = _load_module()
    policy = "author_policy_inspired_b3f3c5c"
    rows = [_valid_row(policy, step) for step in range(1, 91)]
    rows[1].update(
        {
            "step_warm_start_gamma_caps": "2",
            "step_unconverged_warm_start_cache_skips": "3",
            "last_exact_warm_start_gamma_cap_applied": "1",
        }
    )

    summary = module._arm_summary(policy, rows, 0)

    assert summary["strict_gate"] is True
    assert summary["counters"]["warm_start_gamma_caps"] == 2
    assert summary["counters"]["unconverged_warm_start_cache_skips"] == 3


def test_comparison_excludes_timing_and_never_auto_launches_600():
    module = _load_module()
    summaries = {
        "dart_current": module._arm_summary(
            "dart_current",
            [_valid_row("dart_current", step) for step in range(1, 91)],
            0,
        ),
        "author_policy_inspired_b3f3c5c": module._arm_summary(
            "author_policy_inspired_b3f3c5c",
            [
                _valid_row("author_policy_inspired_b3f3c5c", step)
                for step in range(1, 91)
            ],
            0,
        ),
    }

    comparison = module._comparison(summaries)

    assert comparison["timing_used_in_comparison_or_verdict"] is False
    assert comparison["promotion_to_separately_preregistered_600"] is True
    assert comparison["auto_launched_600_step_child"] is False
    assert all("wall" not in key for key in comparison["deterministic_deltas"])


def test_fresh_output_directory_rejects_existing_path(tmp_path):
    module = _load_module()

    with pytest.raises(module.EvidenceError, match="must be fresh"):
        module._prepare_output(tmp_path)


def test_main_writes_complete_indexed_bundle_in_frozen_order(tmp_path, monkeypatch):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"binary")
    output = tmp_path / "evidence"
    traces = {
        "dart_current": _csv_text(
            [_valid_row("dart_current", step) for step in range(1, 91)]
        ),
        "author_policy_inspired_b3f3c5c": _csv_text(
            [
                _valid_row("author_policy_inspired_b3f3c5c", step)
                for step in range(1, 91)
            ]
        ),
    }
    calls: list[list[str]] = []

    def fake_run(command, timeout):
        del timeout
        calls.append(list(command))
        return traces[command[-1]], "", 0

    identity = {"stable": True}
    monkeypatch.setattr(module, "_execution_identity", lambda unused: identity)
    monkeypatch.setattr(module, "_run_captured", fake_run)

    assert (
        module.main(
            ["--binary", str(binary), "--output-dir", str(output), "--timeout", "1"]
        )
        == 0
    )
    assert [command[-1] for command in calls] == list(module.POLICIES)
    assert all(command[7] == "90" for command in calls)
    comparison = json.loads((output / "comparison.json").read_text())
    summary = json.loads((output / "summary.json").read_text())
    metadata = json.loads((output / "metadata.json").read_text())
    index = json.loads((output / "artifact-index.json").read_text())
    assert comparison["comparison_artifact_integrity_valid"] is True
    assert comparison["promotion_to_separately_preregistered_600"] is True
    assert comparison["auto_launched_600_step_child"] is False
    assert summary["promotion_to_separately_preregistered_600"] is True
    assert metadata["auto_launched_600_step_child"] is False
    assert set(index["files"]) == {
        "REPORT.md",
        "artifact-index.json",
        "author_policy_inspired_b3f3c5c/raw.csv",
        "author_policy_inspired_b3f3c5c/stderr.txt",
        "comparison.json",
        "dart_current/raw.csv",
        "dart_current/stderr.txt",
        "invocation.json",
        "metadata.json",
        "summary.json",
    } - {"artifact-index.json"}
    for relative, record in index["files"].items():
        assert record["sha256"] == module._sha256_file(output / relative)


def test_main_seals_valid_scientific_negative_with_successful_runner_exit(
    tmp_path, monkeypatch
):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"binary")
    output = tmp_path / "evidence"
    current = _csv_text([_valid_row("dart_current", step) for step in range(1, 91)])
    author_rows = [
        _valid_row("author_policy_inspired_b3f3c5c", step) for step in range(1, 4)
    ]
    author_rows[-1].update(
        {
            "step_exact_attempts": "2",
            "step_exact_failures": "1",
            "status": "fbf_failed",
        }
    )
    results = iter(((current, "", 0), (_csv_text(author_rows), "negative", 1)))
    monkeypatch.setattr(module, "_execution_identity", lambda unused: {"stable": True})
    monkeypatch.setattr(module, "_run_captured", lambda command, timeout: next(results))

    assert module.main(["--binary", str(binary), "--output-dir", str(output)]) == 0
    comparison = json.loads((output / "comparison.json").read_text())
    assert comparison["comparison_artifact_integrity_valid"] is True
    assert comparison["promotion_to_separately_preregistered_600"] is False


def test_main_attempts_both_arms_when_first_artifact_is_invalid(tmp_path, monkeypatch):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"binary")
    output = tmp_path / "evidence"
    author = _csv_text(
        [_valid_row("author_policy_inspired_b3f3c5c", step) for step in range(1, 91)]
    )
    results = iter((("not,csv\n", "bad", 0), (author, "", 0)))
    calls = []
    monkeypatch.setattr(module, "_execution_identity", lambda unused: {"stable": True})

    def fake_run(command, timeout):
        del timeout
        calls.append(command[-1])
        return next(results)

    monkeypatch.setattr(module, "_run_captured", fake_run)

    assert module.main(["--binary", str(binary), "--output-dir", str(output)]) == 1
    assert calls == list(module.POLICIES)
    summary = json.loads((output / "summary.json").read_text())
    assert summary["comparison_artifact_integrity_valid"] is False
    assert "dart_current" in summary["errors"]
