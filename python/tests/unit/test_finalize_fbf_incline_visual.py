import argparse
import copy
import csv
import importlib.util
import io
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_incline_visual.py"
TEST_EXECUTABLE = Path(sys.executable).resolve()


def _load_module():
    spec = importlib.util.spec_from_file_location("finalize_fbf_incline_visual", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _trace_rows(module, scenario):
    geometry = module._incline_geometry()
    if scenario == "incline_mu_0_4":
        terminal_displacement = module._analytical_slide_displacement()
    else:
        terminal_displacement = 0.005
    downhill_speed = terminal_displacement / (module.TOTAL_STEPS * module.DT_SECONDS)
    rows = []
    for step in range(module.TOTAL_STEPS + 1):
        fraction = step / module.TOTAL_STEPS
        displacement = terminal_displacement * fraction
        rows.append(
            {
                "step": str(step),
                "time": str(step / 60.0),
                "scenario": scenario,
                "solver": "exact_fbf",
                "body": "incline_cube_body",
                "x": str(geometry["initial_x"] + geometry["downhill_x"] * displacement),
                "y": "0.0",
                "z": str(geometry["initial_z"] + geometry["downhill_z"] * displacement),
                "vx": (
                    "0.0" if step == 0 else str(geometry["downhill_x"] * downhill_speed)
                ),
                "vy": "0.0",
                "vz": (
                    "0.0" if step == 0 else str(geometry["downhill_z"] * downhill_speed)
                ),
                "up_z": str(geometry["cos_theta"]),
                "contacts": "0" if step == 0 else "3",
                "exact_solves": str(step),
                "warm_starts": str(max(0, step - 1)),
                "fallbacks": "0",
                "residual": "nan" if step == 0 else "5e-7",
                "status": "not_run" if step == 0 else "success",
            }
        )
    return rows


def _trace_text(module, scenario, rows=None):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=module.TRACE_COLUMNS)
    writer.writeheader()
    writer.writerows(rows if rows is not None else _trace_rows(module, scenario))
    return stream.getvalue()


def _parsed_pair(module):
    return [
        module.parse_trace_text(_trace_text(module, scenario), scenario)
        for scenario in module.SCENARIOS
    ]


def test_v1_defaults_and_exact_membership_are_frozen():
    module = _load_module()

    assert module.DEFAULT_BUNDLE.name == "fig01_02_incline_current_v1"
    assert module.SCHEMA_VERSION == "dart.fbf_incline_visual_bundle/v1"
    assert module.MANUAL_SCHEMA_VERSION == "dart.fbf_incline_manual_inspection/v1"
    assert module.SCENARIOS == ("incline_mu_0_4", "incline_mu_0_5")
    assert module.CAPTURE_STEPS == tuple(range(0, 121, 2))
    assert len(module.CAPTURE_PATHS) == 77
    assert len(module.GENERATED_PATHS) == 11
    assert len(module.STAGING_PATHS) == 70
    assert len(module.DURABLE_STILL_PATHS) == 5
    assert len(module.EXPECTED_FINAL_PATHS) == 23
    assert module.MANUAL_ARTIFACT_PATHS >= module.DURABLE_STILL_PATHS
    assert not (module.STAGING_PATHS & module.EXPECTED_FINAL_PATHS)
    assert module.EXPECTED_METADATA_FLAGS["status"] == (
        "valid_current_source_nonpaper_incline"
    )


@pytest.mark.parametrize("scenario", ["incline_mu_0_4", "incline_mu_0_5"])
def test_trace_parser_accepts_fixture_derived_outcomes(scenario):
    module = _load_module()

    parsed = module.parse_trace_text(_trace_text(module, scenario), scenario)
    summary = parsed["summary"]

    assert summary["row_count"] == 121
    assert summary["completed_steps"] == 120
    assert summary["continuous_contact_post_initial"] is True
    assert summary["observed_contacts_per_step"] == 3
    assert summary["paper_reference_contacts"] == 4
    assert summary["paper_reference_contact_count_match"] is False
    assert summary["maximum_penetration_proven"] is False
    assert summary["physical_outcome_valid"] is True
    assert summary["trajectory_invariants"]["pass"] is True
    assert summary["trajectory_invariants"]["maximum_cross_slope_displacement_m"] == 0.0
    assert summary["trajectory_invariants"]["maximum_up_z_alignment_error"] == 0.0
    assert (
        summary["trajectory_invariants"][
            "maximum_velocity_position_component_error_m_s"
        ]
        <= module.INCLINE_VELOCITY_POSITION_CONSISTENCY_TOLERANCE
    )
    assert summary["trajectory_invariants"]["maximum_cross_slope_speed_m_s"] == 0.0
    assert (
        summary["trajectory_invariants"]["maximum_incline_normal_speed_m_s"]
        <= module.INCLINE_NORMAL_SPEED_TOLERANCE
    )
    assert summary["selected_panel_states"][0]["residual"] is None
    assert parsed["count_projection"][-1] == {
        "step": 120,
        "contacts": 3,
        "exact_solves": 120,
        "boxed_lcp_fallbacks": 0,
    }
    if scenario == "incline_mu_0_4":
        assert summary["downhill_displacement_m"] == pytest.approx(
            module._analytical_slide_displacement()
        )
    else:
        assert summary["downhill_displacement_m"] == pytest.approx(0.005)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda rows: rows.pop(), "expected 121 rows"),
        (lambda rows: rows[10].update(step="11"), "noncontiguous step"),
        (lambda rows: rows[10].update(time="1.0"), "is not step/60"),
        (lambda rows: rows[10].update(scenario="backspin"), "scenario"),
        (lambda rows: rows[10].update(solver="boxed_lcp"), "identity mismatch"),
        (lambda rows: rows[10].update(body="other"), "identity mismatch"),
        (lambda rows: rows[10].update(x="nan"), "nonfinite x"),
        (lambda rows: rows[10].update(status="failure"), "status"),
        (lambda rows: rows[10].update(residual="1.1e-6"), "exceeds"),
        (lambda rows: rows[10].update(residual="-1e-9"), "exceeds"),
        (lambda rows: rows[10].update(contacts="2"), "contacts 2 != 3"),
        (lambda rows: rows[10].update(exact_solves="9"), "exact solves 9 != 10"),
        (lambda rows: rows[10].update(warm_starts="11"), "warm starts exceed"),
        (lambda rows: rows[0].update(contacts="3"), "must be zero"),
        (lambda rows: rows[0].update(x="0.0"), "initial x"),
        (lambda rows: rows[5].update(step="05"), "noncanonical integer"),
    ],
)
def test_trace_parser_rejects_malformed_or_weakened_contracts(mutation, message):
    module = _load_module()
    rows = _trace_rows(module, "incline_mu_0_4")
    mutation(rows)

    with pytest.raises(ValueError, match=message):
        module.parse_trace_text(
            _trace_text(module, "incline_mu_0_4", rows), "incline_mu_0_4"
        )


def test_trace_parser_rejects_fallback_and_wrong_physical_outcomes():
    module = _load_module()
    rows = _trace_rows(module, "incline_mu_0_4")
    for row in rows[10:]:
        row["fallbacks"] = "1"
    with pytest.raises(ValueError, match="fallback"):
        module.parse_trace_text(
            _trace_text(module, "incline_mu_0_4", rows), "incline_mu_0_4"
        )

    for scenario in module.SCENARIOS:
        rows = _trace_rows(module, scenario)
        geometry = module._incline_geometry()
        wrong_displacement = 0.0 if scenario.endswith("0_4") else 0.5
        rows[-1]["x"] = str(
            geometry["initial_x"] + geometry["downhill_x"] * wrong_displacement
        )
        rows[-1]["z"] = str(
            geometry["initial_z"] + geometry["downhill_z"] * wrong_displacement
        )
        with pytest.raises(ValueError, match="displacement"):
            module.parse_trace_text(_trace_text(module, scenario, rows), scenario)


def test_trace_parser_gates_full_trajectory_stick_and_incline_invariants():
    module = _load_module()
    scenario = "incline_mu_0_5"

    rows = _trace_rows(module, scenario)
    rows[60]["y"] = "100.0"
    with pytest.raises(ValueError, match="cross-slope displacement"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)

    rows = _trace_rows(module, scenario)
    rows[60]["up_z"] = "-1.0"
    with pytest.raises(ValueError, match="up_z alignment"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)

    rows = _trace_rows(module, scenario)
    geometry = module._incline_geometry()
    rows[60]["x"] = str(float(rows[60]["x"]) - 100.0 * geometry["sin_theta"])
    rows[60]["z"] = str(float(rows[60]["z"]) + 100.0 * geometry["cos_theta"])
    with pytest.raises(ValueError, match="incline-normal center-offset"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)

    rows = _trace_rows(module, scenario)
    displacement = 0.1
    rows[60]["x"] = str(geometry["initial_x"] + geometry["downhill_x"] * displacement)
    rows[60]["z"] = str(geometry["initial_z"] + geometry["downhill_z"] * displacement)
    with pytest.raises(ValueError, match="stick trajectory displacement"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)


def test_trace_parser_rejects_downhill_regression_in_slide_trajectory():
    module = _load_module()
    scenario = "incline_mu_0_4"
    rows = _trace_rows(module, scenario)
    geometry = module._incline_geometry()
    displacement = 1.5
    rows[60]["x"] = str(geometry["initial_x"] + geometry["downhill_x"] * displacement)
    rows[60]["z"] = str(geometry["initial_z"] + geometry["downhill_z"] * displacement)

    with pytest.raises(ValueError, match="downhill displacement regressed"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)


def test_trace_parser_rejects_velocity_position_inconsistency_and_fast_stick():
    module = _load_module()
    scenario = "incline_mu_0_5"

    rows = _trace_rows(module, scenario)
    for row in rows[1:]:
        row.update(vx="1000.0", vy="1000.0", vz="1000.0")
    with pytest.raises(ValueError, match="velocity-position component consistency"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)

    rows = _trace_rows(module, scenario)
    geometry = module._incline_geometry()
    previous_displacement = 0.0
    for step, row in enumerate(rows[1:], start=1):
        displacement = 0.01 if step % 2 else -0.01
        downhill_speed = (displacement - previous_displacement) / module.DT_SECONDS
        row["x"] = str(geometry["initial_x"] + geometry["downhill_x"] * displacement)
        row["z"] = str(geometry["initial_z"] + geometry["downhill_z"] * displacement)
        row["vx"] = str(geometry["downhill_x"] * downhill_speed)
        row["vz"] = str(geometry["downhill_z"] * downhill_speed)
        previous_displacement = displacement
    with pytest.raises(ValueError, match="stick speed"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)


def test_trace_parser_rejects_kinematically_consistent_cross_and_normal_speed():
    module = _load_module()
    scenario = "incline_mu_0_5"

    def update_velocities(rows):
        for previous, current in zip(rows, rows[1:]):
            for position_field, velocity_field in (
                ("x", "vx"),
                ("y", "vy"),
                ("z", "vz"),
            ):
                current[velocity_field] = str(
                    (float(current[position_field]) - float(previous[position_field]))
                    / module.DT_SECONDS
                )

    rows = _trace_rows(module, scenario)
    for step, row in enumerate(rows[1:], start=1):
        row["y"] = str(0.01 if step % 2 else -0.01)
    update_velocities(rows)
    with pytest.raises(ValueError, match="cross-slope speed"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)

    rows = _trace_rows(module, scenario)
    geometry = module._incline_geometry()
    for step, row in enumerate(rows[1:], start=1):
        normal_offset = 0.01 if step % 2 else -0.01
        row["x"] = str(float(row["x"]) - geometry["sin_theta"] * normal_offset)
        row["z"] = str(float(row["z"]) + geometry["cos_theta"] * normal_offset)
    update_velocities(rows)
    with pytest.raises(ValueError, match="incline-normal speed"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)


def test_trace_parser_rejects_slide_that_stops_at_the_final_step():
    module = _load_module()
    scenario = "incline_mu_0_4"
    rows = _trace_rows(module, scenario)
    rows[-2]["x"] = rows[-1]["x"]
    rows[-2]["z"] = rows[-1]["z"]
    for previous, current in zip(rows, rows[1:]):
        current["vx"] = str(
            (float(current["x"]) - float(previous["x"])) / module.DT_SECONDS
        )
        current["vy"] = str(
            (float(current["y"]) - float(previous["y"])) / module.DT_SECONDS
        )
        current["vz"] = str(
            (float(current["z"]) - float(previous["z"])) / module.DT_SECONDS
        )

    with pytest.raises(ValueError, match="final downhill speed"):
        module.parse_trace_text(_trace_text(module, scenario, rows), scenario)


def test_trace_parser_rejects_noncanonical_header_and_unknown_scenario():
    module = _load_module()
    text = _trace_text(module, "incline_mu_0_4").replace("step,time", "time,step", 1)

    with pytest.raises(ValueError, match="unexpected trace columns"):
        module.parse_trace_text(text, "incline_mu_0_4")
    with pytest.raises(ValueError, match="unsupported incline"):
        module.parse_trace_text(text, "incline_mu_0_6")


def test_trace_pair_builds_only_the_additive_count_projection():
    module = _load_module()
    summary = module.summarize_trace_pair(_parsed_pair(module))

    assert summary["physical_outcome_valid"] is True
    assert summary["full_state_trace_equivalence"] is False
    assert summary["aggregate_projection_fields"] == [
        "step",
        "exact_solves",
        "boxed_lcp_fallbacks",
    ]
    assert summary["aggregate_count_projection"][0] == {
        "step": 0,
        "exact_solves": 0,
        "boxed_lcp_fallbacks": 0,
    }
    assert summary["aggregate_count_projection"][-1] == {
        "step": 120,
        "exact_solves": 240,
        "boxed_lcp_fallbacks": 0,
    }
    assert summary["trace_aggregate_contacts_per_post_initial_step"] == 6
    assert summary["threshold_separation"]["slide_minus_stick_m"] > 0.5
    assert summary["threshold_separation"]["pass"] is True
    assert "state, residual, status, warm-start" in summary["claim_boundary"]

    with pytest.raises(ValueError, match="incomplete"):
        module.summarize_trace_pair(_parsed_pair(module)[:1])
    duplicated = [_parsed_pair(module)[0], copy.deepcopy(_parsed_pair(module)[0])]
    with pytest.raises(ValueError, match="missing or duplicated"):
        module.summarize_trace_pair(duplicated)

    insufficient = _parsed_pair(module)
    insufficient[0]["summary"]["downhill_displacement_m"] = 0.4
    insufficient[1]["summary"]["downhill_displacement_m"] = 0.0
    with pytest.raises(ValueError, match="separation does not exceed"):
        module.summarize_trace_pair(insufficient)


def test_aggregate_count_projection_comparison_is_exact_and_non_stateful():
    module = _load_module()
    projection = module.summarize_trace_pair(_parsed_pair(module))[
        "aggregate_count_projection"
    ]

    result = module._compare_aggregate_count_projections(
        projection, copy.deepcopy(projection)
    )

    assert result["pass"] is True
    assert result["byte_identical"] is True
    assert result["aggregate_counts_only"] is True
    assert result["contact_counts_compared"] is False
    assert result["contact_count_match"] is False
    assert result["trace_aggregate_contacts_per_post_initial_step"] == 6
    assert result["capture_contacts_per_post_initial_step"] == 8
    assert result["full_state_trace_equivalence"] is False
    assert result["per_cell_trace_equivalence"] is False
    assert "state, residual, status" in result["limitation"]

    changed = copy.deepcopy(projection)
    changed[60]["exact_solves"] = 5
    with pytest.raises(ValueError, match="differs at 60"):
        module._compare_aggregate_count_projections(projection, changed)


def test_capture_count_projection_requires_exact_steps_and_strict_counts():
    module = _load_module()
    timeline = {
        "steps": {
            str(step): {
                "solver_diagnostics": {
                    "contacts": 0 if step == 0 else 8,
                    "exact_solves": 2 * step,
                    "boxed_lcp_fallbacks": 0,
                }
            }
            for step in range(module.TOTAL_STEPS + 1)
        }
    }

    projection = module._capture_count_projection(timeline)
    assert projection[-1]["exact_solves"] == 240
    assert "contacts" not in projection[-1]

    missing = copy.deepcopy(timeline)
    del missing["steps"]["60"]
    with pytest.raises(ValueError, match="incomplete"):
        module._capture_count_projection(missing)

    invalid = copy.deepcopy(timeline)
    invalid["steps"]["60"]["solver_diagnostics"]["contacts"] = True
    with pytest.raises(ValueError, match="contacts.*invalid"):
        module._capture_count_projection(invalid)

    invalid = copy.deepcopy(timeline)
    invalid["steps"]["60"]["solver_diagnostics"]["contacts"] = 6
    with pytest.raises(ValueError, match="contacts 6 != 8"):
        module._capture_count_projection(invalid)

    invalid = copy.deepcopy(timeline)
    del invalid["steps"]["60"]["solver_diagnostics"]["exact_solves"]
    with pytest.raises(ValueError, match="exact_solves.*invalid"):
        module._capture_count_projection(invalid)


def test_capture_final_diagnostics_require_exactly_two_groups_per_step():
    module = _load_module()
    diagnostics = {
        "available": True,
        "exact_attempts": 240,
        "exact_solves": 240,
        "accepted_at_cap": 0,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
        "status": "success",
        "fbf_status": "success",
        "worst_residual": 5e-7,
    }

    assert module._validate_capture_final_diagnostics(diagnostics) == diagnostics
    assert module.EXPECTED_CAPTURE_EXACT_GROUPS == 2 * module.TOTAL_STEPS

    for field, value in (("exact_attempts", 239), ("exact_solves", 241)):
        invalid = copy.deepcopy(diagnostics)
        invalid[field] = value
        with pytest.raises(ValueError, match="solver diagnostics changed"):
            module._validate_capture_final_diagnostics(invalid)


def _manual_record(module, root):
    artifacts = []
    for index, relative in enumerate(sorted(module.MANUAL_ARTIFACT_PATHS)):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"artifact-{index}".encode())
        artifacts.append(
            {
                "path": relative,
                "sha256": module.sha256(path),
                "observation": f"inspected {relative}",
            }
        )
    return {
        "schema_version": module.MANUAL_SCHEMA_VERSION,
        "manual_inspected": True,
        "pass": True,
        "verdicts": copy.deepcopy(module.MANUAL_VERDICTS),
        "representative_artifacts": artifacts,
    }


def _durable_still_fixture(module, root):
    frames = {}
    shots = []
    for step in module.DURABLE_STILL_STEPS:
        relative = f"incline/frames/step_{step:06d}.png"
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"raw-incline-{step}".encode())
        frames[str(step)] = {
            "path": str(path.resolve()),
            "sha256": module.sha256(path),
        }
        shots.append({"step": step, "path": str(path.resolve())})
    metadata = {
        "schedule": {"id": "incline"},
        "timeline_validation": {"frames": frames},
    }
    module.write_json(root / "incline/metadata.json", metadata)
    module.write_json(
        root / "run-summary.json",
        {"results": [copy.deepcopy(metadata)]},
    )
    module.write_json(root / "incline/timeline.json", {"shots": shots})
    return module._materialize_durable_stills(root)


def test_manual_inspection_binds_visual_outcomes_and_exact_artifacts(tmp_path):
    module = _load_module()
    record = _manual_record(module, tmp_path)
    module.write_json(tmp_path / "manual-inspection.json", record)

    assert module.validate_manual_inspection(tmp_path) == record
    assert record["verdicts"]["mu_0_4_downhill_slide_visible"] is True
    assert record["verdicts"]["mu_0_5_near_threshold_stick_visible"] is True
    assert record["verdicts"]["aggregate_count_projection_only"] is True
    assert record["verdicts"]["full_state_trace_equivalence"] is False
    assert record["verdicts"]["capture_sidecar_deliverable_validated"] is False
    assert record["verdicts"]["full_friction_sweep"] is False
    assert record["verdicts"]["approved_source_golden_diff"] is False
    assert record["verdicts"]["timing_verdict"] is None


def test_durable_stills_bind_timeline_and_run_summary_hashes(tmp_path):
    module = _load_module()
    records = _durable_still_fixture(module, tmp_path)

    assert module._validate_durable_stills(tmp_path, records) == records
    assert [item["step"] for item in records] == list(module.PANEL_STEPS)
    assert all(
        item["sha256"]
        == item["timeline_frame_sha256"]
        == item["run_summary_frame_sha256"]
        for item in records
    )


@pytest.mark.parametrize("mutation", ["missing", "tampered"])
def test_durable_stills_fail_closed_when_missing_or_tampered(tmp_path, mutation):
    module = _load_module()
    records = _durable_still_fixture(module, tmp_path)
    still = tmp_path / records[0]["path"]
    if mutation == "missing":
        still.unlink()
    else:
        still.write_bytes(b"tampered")

    with pytest.raises(ValueError, match="durable still"):
        module._validate_durable_stills(tmp_path, records)


def test_sealed_capture_provenance_binds_pruned_staging_and_stills(
    tmp_path, monkeypatch
):
    module = _load_module()
    runtime_identity = {"runtime": {"path": "/runtime", "sha256": "1" * 64}}
    binary_bindings = {"source": {"query_payload_sha256": "2" * 64}}
    monkeypatch.setattr(
        module,
        "_capture_runtime_identity",
        lambda **kwargs: copy.deepcopy(runtime_identity),
    )
    monkeypatch.setattr(
        module,
        "_binary_source_binding_identity",
        lambda **kwargs: copy.deepcopy(binary_bindings),
    )
    records = _durable_still_fixture(module, tmp_path)
    for relative in module.STAGING_PATHS:
        path = tmp_path / relative
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(relative.encode())
    manifest = module._build_staging_manifest(tmp_path)
    staging = {item["path"]: item for item in manifest["artifacts"]}
    executable = TEST_EXECUTABLE
    payload = {
        "argv": module._capture_argv(
            executable,
            executable,
            executable,
            tmp_path,
            executable,
            executable,
        ),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(tmp_path / "run-summary.json"),
        "run_summary_validated": True,
        "stdout_path": "capture.stdout.txt",
        "stdout_sha256": staging["capture.stdout.txt"]["sha256"],
        "stdout_ends_with_run_summary": True,
        "stderr_path": "capture.stderr.txt",
        "stderr_sha256": staging["capture.stderr.txt"]["sha256"],
        "runtime_inputs_before": runtime_identity,
        "runtime_inputs_after": runtime_identity,
        "binary_source_bindings_before": binary_bindings,
        "binary_source_bindings_after": binary_bindings,
        "durable_stills": records,
        "pruned_staging": manifest,
    }
    module._prune_staging_outputs(tmp_path, manifest)

    assert (
        module._validate_capture_provenance(
            payload,
            bundle=tmp_path,
            trace_binary=executable,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
            sealed=True,
        )
        == payload
    )

    tampered = copy.deepcopy(payload)
    tampered["durable_stills"][0]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="durable still provenance changed"):
        module._validate_capture_provenance(
            tampered,
            bundle=tmp_path,
            trace_binary=executable,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
            sealed=True,
        )


@pytest.mark.parametrize(
    "mutation",
    ["promote", "wrong_hash", "extra_path", "empty_observation", "extra_claim"],
)
def test_manual_inspection_fails_closed(tmp_path, mutation):
    module = _load_module()
    record = _manual_record(module, tmp_path)
    if mutation == "promote":
        record["verdicts"]["full_state_trace_equivalence"] = True
    elif mutation == "wrong_hash":
        record["representative_artifacts"][0]["sha256"] = "0" * 64
    elif mutation == "extra_path":
        record["representative_artifacts"].append(
            {
                "path": "unexpected.png",
                "sha256": "0" * 64,
                "observation": "unexpected",
            }
        )
    elif mutation == "empty_observation":
        record["representative_artifacts"][0]["observation"] = ""
    else:
        record["full_paper_parity"] = True
    module.write_json(tmp_path / "manual-inspection.json", record)

    with pytest.raises(ValueError):
        module.validate_manual_inspection(tmp_path)


def test_artifact_index_requires_exact_membership_and_hashes(tmp_path):
    module = _load_module()
    (tmp_path / "nested").mkdir()
    (tmp_path / "a.txt").write_text("a", encoding="utf-8")
    (tmp_path / "nested/b.txt").write_text("b", encoding="utf-8")
    index = module.artifact_index(tmp_path)

    module.validate_artifact_index(tmp_path, index)

    extra = copy.deepcopy(index)
    extra["paper_parity"] = True
    with pytest.raises(ValueError, match="malformed"):
        module.validate_artifact_index(tmp_path, extra)

    (tmp_path / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="membership mismatch"):
        module.validate_artifact_index(tmp_path, index)

    (tmp_path / "unexpected.txt").unlink()
    (tmp_path / "a.txt").write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="byte mismatch|hash mismatch"):
        module.validate_artifact_index(tmp_path, index)


def test_artifact_index_and_bundle_membership_reject_symlinks(tmp_path):
    module = _load_module()
    target = tmp_path / "target.txt"
    target.write_text("target", encoding="utf-8")
    (tmp_path / "link.txt").symlink_to(target)

    with pytest.raises(ValueError, match="symlink"):
        module.artifact_index(tmp_path)
    with pytest.raises(ValueError, match="symlink"):
        module._validate_bundle_paths(
            tmp_path,
            complete=False,
            allow_missing_capture=True,
            allow_missing_capture_provenance=True,
        )


def test_bundle_root_symlink_is_rejected_before_resolution(tmp_path):
    module = _load_module()
    target = tmp_path / "target"
    target.mkdir()
    bundle_link = tmp_path / "bundle-link"
    bundle_link.symlink_to(target, target_is_directory=True)

    with pytest.raises(ValueError, match="bundle root is a symlink"):
        module._validate_bundle_paths(
            bundle_link,
            complete=False,
            allow_missing_capture=True,
            allow_missing_capture_provenance=True,
        )
    with pytest.raises(ValueError, match="bundle root is a symlink"):
        module.finalize(
            argparse.Namespace(bundle=bundle_link, reuse_current_capture=True)
        )
    with pytest.raises(ValueError, match="bundle root is a symlink"):
        module.verify_only(argparse.Namespace(bundle=bundle_link))

    real_parent = tmp_path / "real-parent"
    real_parent.mkdir()
    (real_parent / "bundle").mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(ValueError, match="passes through a symlink"):
        module._validate_bundle_paths(
            linked_parent / "bundle",
            complete=False,
            allow_missing_capture=True,
            allow_missing_capture_provenance=True,
        )
    with pytest.raises(ValueError, match="passes through a symlink"):
        module._require_bundle_root(linked_parent / "new-bundle", create=True)
    assert not (real_parent / "new-bundle").exists()


def test_two_phase_bundle_membership_allows_only_the_manual_pause(tmp_path):
    module = _load_module()

    module._validate_bundle_paths(
        tmp_path,
        complete=False,
        allow_missing_capture=True,
        allow_missing_capture_provenance=True,
    )
    with pytest.raises(ValueError, match="capture bundle is incomplete"):
        module._validate_bundle_paths(tmp_path, complete=False)

    for relative in module.CAPTURE_PATHS - {"manual-inspection.json"}:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"captured")
    module._validate_bundle_paths(tmp_path, complete=False, allow_missing_manual=True)
    with pytest.raises(ValueError, match=r"\['manual-inspection.json'\]"):
        module._validate_bundle_paths(tmp_path, complete=False)

    (tmp_path / "unexpected.txt").write_text("bad", encoding="utf-8")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(
            tmp_path, complete=False, allow_missing_manual=True
        )


def test_sealed_membership_rejects_surviving_staging_and_extra_files(tmp_path):
    module = _load_module()
    for relative in module.EXPECTED_FINAL_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"durable")

    module._validate_bundle_paths(tmp_path, complete=True)

    (tmp_path / "capture.stderr.txt").write_bytes(b"staging")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(tmp_path, complete=True)
    (tmp_path / "capture.stderr.txt").unlink()

    (tmp_path / "unexpected.txt").write_bytes(b"extra")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(tmp_path, complete=True)


def test_pruned_staging_manifest_is_exact_and_rejects_survivors(tmp_path):
    module = _load_module()
    for relative in module.STAGING_PATHS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(relative.encode())
    manifest = module._build_staging_manifest(tmp_path)

    module._validate_staging_manifest(tmp_path, manifest, require_absent=False)
    tampered = copy.deepcopy(manifest)
    tampered["artifacts"][0]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="staging artifact changed"):
        module._validate_staging_manifest(tmp_path, tampered, require_absent=False)

    module._prune_staging_outputs(tmp_path, manifest)
    module._validate_staging_manifest(tmp_path, manifest, require_absent=True)
    (tmp_path / "capture.stderr.txt").write_bytes(b"survived")
    with pytest.raises(ValueError, match="survived seal"):
        module._validate_staging_manifest(tmp_path, manifest, require_absent=True)


def test_trace_capture_and_verification_argv_freeze_runtime_contract(tmp_path):
    module = _load_module()
    trace = Path("/trace")

    assert module._trace_argv(trace, "incline_mu_0_4") == [
        "/trace",
        "incline_mu_0_4",
        "exact_fbf",
        "1",
        "120",
        "nan",
        "tracked",
        "default",
        "default",
        "1",
        "dart_best",
        "dart",
        "default",
        "0",
        "0",
        "default",
    ]
    with pytest.raises(ValueError, match="unsupported incline"):
        module._trace_argv(trace, "incline_mu_0_6")

    python = Path("/python")
    runner = Path("/runner")
    demo = Path("/demo")
    ffmpeg = Path("/ffmpeg")
    ffprobe = Path("/ffprobe")
    capture = module._capture_argv(python, runner, demo, tmp_path, ffmpeg, ffprobe)
    verify = module._verification_argv(python, runner, demo, tmp_path, ffmpeg, ffprobe)
    assert capture[2:5] == ["run", "--scenario", "incline"]
    assert capture[-4:] == [
        "--python",
        str(python),
        "--out",
        str(tmp_path / "run-summary.json"),
    ]
    assert verify[2:5] == ["verify", "--scenario", "incline"]

    demo_argv = module._expected_demo_argv(demo, tmp_path)
    assert demo_argv[:16] == [
        "/demo",
        "--scene",
        "fbf_paper_incline",
        "--headless",
        "--steps",
        "120",
        "--width",
        "1280",
        "--height",
        "720",
        "--collision-detector",
        "dart",
        "--threads",
        "1",
        "--headless-sidecar",
        str(tmp_path / "incline/timeline.json"),
    ]
    assert demo_argv.count("--headless-shot-at") == 61
    assert demo_argv[-1].startswith("120:")


def test_capture_provenance_binds_source_and_all_runtime_inputs(tmp_path, monkeypatch):
    module = _load_module()
    runtime_dependencies = {
        "demo_dependency_lib__libdart.so": {
            "path": "/build/lib/libdart.so",
            "sha256": "1" * 64,
        }
    }
    binary_source_bindings = {
        "demo_source_binding": {"query_payload_sha256": "2" * 64},
        "trace_source_binding": {"query_payload_sha256": "3" * 64},
    }
    monkeypatch.setattr(
        module,
        "_in_tree_runtime_dependency_identity",
        lambda executable, *, label: copy.deepcopy(runtime_dependencies),
    )
    monkeypatch.setattr(
        module,
        "_binary_source_binding_identity",
        lambda **kwargs: copy.deepcopy(binary_source_bindings),
    )
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    module.write_json(bundle / "run-summary.json", {"pass": True})
    (bundle / "capture.stdout.txt").write_text(
        "renderer diagnostic\n"
        + json.dumps({"pass": True}, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    (bundle / "capture.stderr.txt").write_text("", encoding="utf-8")
    executable = TEST_EXECUTABLE
    identity = module._capture_runtime_identity(
        python=executable,
        runner=executable,
        demo=executable,
        ffmpeg=executable,
        ffprobe=executable,
    )
    assert set(identity) == {
        "visual_runner",
        "image_compose",
        "image_tools",
        "image_verdict",
        "demo_source",
        "demo_host_source",
        "demo_host_header",
        "registry_source",
        "scenes_header",
        "demo_binary",
        "ffmpeg_binary",
        "ffprobe_binary",
        "python_binary",
        "demo_dependency_lib__libdart.so",
    }
    payload = {
        "argv": module._capture_argv(
            executable, executable, executable, bundle, executable, executable
        ),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": module.sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "stdout_path": "capture.stdout.txt",
        "stdout_sha256": module.sha256(bundle / "capture.stdout.txt"),
        "stdout_ends_with_run_summary": True,
        "stderr_path": "capture.stderr.txt",
        "stderr_sha256": module.sha256(bundle / "capture.stderr.txt"),
        "runtime_inputs_before": identity,
        "runtime_inputs_after": identity,
        "binary_source_bindings_before": binary_source_bindings,
        "binary_source_bindings_after": binary_source_bindings,
    }

    assert (
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            trace_binary=executable,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
        )
        == payload
    )

    (bundle / "capture.stdout.txt").write_text(
        json.dumps({"pass": False}) + "\n", encoding="utf-8"
    )
    with pytest.raises(ValueError, match="does not end with"):
        module._validate_capture_provenance(
            payload,
            bundle=bundle,
            trace_binary=executable,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
        )
    (bundle / "capture.stdout.txt").write_text(
        "renderer diagnostic\n"
        + json.dumps({"pass": True}, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )

    tampered = copy.deepcopy(payload)
    tampered["runtime_inputs_before"]["visual_runner"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="capture-time provenance changed"):
        module._validate_capture_provenance(
            tampered,
            bundle=bundle,
            trace_binary=executable,
            python=executable,
            runner=executable,
            demo=executable,
            ffmpeg=executable,
            ffprobe=executable,
        )


def test_source_identity_binds_finalizer_tests_sources_and_binaries(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(
        module,
        "_in_tree_runtime_dependency_identity",
        lambda executable, *, label: {
            f"{label}_dependency_lib__libdart.so": {
                "path": f"/build/{label}/libdart.so",
                "sha256": ("1" if label == "demo" else "2") * 64,
            }
        },
    )
    binary_source_bindings = {
        "demo_source_binding": {"query_payload_sha256": "3" * 64},
        "trace_source_binding": {"query_payload_sha256": "4" * 64},
    }
    monkeypatch.setattr(
        module,
        "_binary_source_binding_identity",
        lambda **kwargs: copy.deepcopy(binary_source_bindings),
    )
    executable = TEST_EXECUTABLE
    sources = module._source_identity(
        trace_binary=executable,
        demo=executable,
        runner=module.DEFAULT_RUNNER,
        ffmpeg=executable,
        ffprobe=executable,
        python=executable,
    )

    assert set(sources) == {
        "finalizer",
        "finalizer_test",
        "visual_runner",
        "visual_runner_test",
        "image_compose",
        "image_tools",
        "image_verdict",
        "trace_source",
        "fixture_source",
        "demo_source",
        "demo_host_source",
        "demo_host_header",
        "registry_source",
        "scenes_header",
        "trace_binary",
        "demo_binary",
        "ffmpeg_binary",
        "ffprobe_binary",
        "python_binary",
        "demo_dependency_lib__libdart.so",
        "trace_dependency_lib__libdart.so",
        "binary_source_bindings",
    }
    assert sources["finalizer"] == {
        "path": str(SCRIPT.resolve()),
        "sha256": module.sha256(SCRIPT),
    }
    assert sources["finalizer_test"]["path"] == str(Path(__file__).resolve())
    assert sources["binary_source_bindings"] == binary_source_bindings


def test_binary_source_queries_bind_each_executable_to_current_cpp_source(
    tmp_path, monkeypatch
):
    module = _load_module()
    demo = tmp_path / "dart-demos"
    trace = tmp_path / "fbf-paper-trace"
    demo.write_bytes(b"demo")
    trace.write_bytes(b"trace")

    def run_query(argv, *, timeout):
        del timeout
        is_demo = "--fbf-author-turntable-contract" in argv
        role = "dart_demos" if is_demo else "fbf_paper_trace"
        source = module.DEMO_SOURCE if is_demo else module.TRACE_SOURCE
        payload = {
            "schema_version": "dart.fbf_author_turntable_physics_contract/v1",
            "kind": "physics_control",
            "scenario": {"trace_id": "turntable_author_mu_0_2_omega_2"},
            "binary_binding": {
                "role": role,
                "implementation_source_sha256": module.sha256(source),
            },
        }
        return subprocess.CompletedProcess(argv, 0, json.dumps(payload), "")

    monkeypatch.setattr(module, "_run_command", run_query)
    identity = module._binary_source_binding_identity(demo=demo, trace_binary=trace)

    assert set(identity) == {"demo_source_binding", "trace_source_binding"}
    assert identity["demo_source_binding"]["binary_sha256"] == module.sha256(demo)
    assert identity["trace_source_binding"][
        "implementation_source_sha256"
    ] == module.sha256(module.TRACE_SOURCE)
    assert identity["demo_source_binding"]["query_argv"][1:] == [
        "--fbf-author-turntable-contract",
        "fbf_author_turntable_mu_0_2_omega_2",
    ]

    def stale_query(argv, *, timeout):
        del timeout
        payload = {
            "schema_version": "dart.fbf_author_turntable_physics_contract/v1",
            "kind": "physics_control",
            "scenario": {"trace_id": "turntable_author_mu_0_2_omega_2"},
            "binary_binding": {
                "role": "dart_demos",
                "implementation_source_sha256": "0" * 64,
            },
        }
        return subprocess.CompletedProcess(argv, 0, json.dumps(payload), "")

    monkeypatch.setattr(module, "_run_command", stale_query)
    with pytest.raises(ValueError, match="does not bind the current"):
        module._binary_source_binding_identity(demo=demo, trace_binary=trace)


def test_report_and_metadata_preserve_count_only_and_nonpaper_boundaries():
    module = _load_module()
    pair = module.summarize_trace_pair(_parsed_pair(module))
    aggregate = pair.pop("aggregate_count_projection")
    pair["aggregate_count_projection_comparison"] = (
        module._compare_aggregate_count_projections(aggregate, aggregate)
    )
    capture = {
        "captured_frames": 61,
        "worst_residual": 5e-7,
        "observed_contacts_per_post_initial_step": 8,
        "media": {
            "stream_contract": {
                "width": 660,
                "height": 506,
                "frame_rate": "30/1",
                "frame_count": 61,
            }
        },
    }

    report = module._report_markdown(pair, capture)

    assert "additive count projection" in report
    assert "eight-versus-six" in report
    assert "Contact counts are explicitly excluded" in report
    assert "does not export a per-cell contact split" in report
    assert "explicitly not state or full-trace" in report
    assert "legacy three-contacts-per-cell mismatch note" in report
    assert "paper timing, or real-time verdict" in report
    assert (
        module.EXPECTED_METADATA_FLAGS["aggregate_count_projection_equivalent"] is True
    )
    assert (
        module.EXPECTED_METADATA_FLAGS["capture_trace_contact_count_equivalent"]
        is False
    )
    assert module.EXPECTED_METADATA_FLAGS["full_state_trace_equivalence"] is False
    assert module.EXPECTED_METADATA_FLAGS["maximum_penetration_proven"] is False
    assert (
        module.EXPECTED_METADATA_FLAGS["capture_sidecar_deliverable_validated"] is False
    )
    assert module.EXPECTED_METADATA_FLAGS["full_friction_sweep"] is False
    assert module.EXPECTED_METADATA_FLAGS["approved_source_golden_diff"] is False
    assert module.EXPECTED_FINAL_METADATA_KEYS == set(
        module.EXPECTED_METADATA_FLAGS
    ) | {
        "evidence_date",
        "claim_scope",
        "trace_summary",
        "capture_summary",
        "capture_provenance",
        "manual_inspection",
        "run_summary",
        "verification",
        "invocations",
        "report",
        "artifact_index",
        "source_identity",
        "claim_boundary",
    }
    assert "Only additive exact-solve/fallback counts" in (module.CLAIM_BOUNDARY)
    assert "capture contacts are eight" in module.CLAIM_BOUNDARY


def test_verify_only_replays_traces_without_disposable_capture_staging(
    tmp_path, monkeypatch
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    traces = {}
    for scenario in module.SCENARIOS:
        trace = f"stored-{scenario}\n"
        traces[scenario] = trace
        path = bundle / "traces" / f"{scenario}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(trace, encoding="utf-8")
        (bundle / "traces" / f"{scenario}.stderr.txt").write_text("", encoding="utf-8")
    monkeypatch.setattr(
        module,
        "_require_file",
        lambda path, label: Path(path).resolve(),
    )
    monkeypatch.setattr(
        module,
        "verify_finalized",
        lambda *args, **kwargs: {"pass": True, "status": "verified"},
    )
    processes = iter(
        [
            subprocess.CompletedProcess([], 0, traces[scenario], "")
            for scenario in module.SCENARIOS
        ]
    )
    monkeypatch.setattr(module, "_run_command", lambda *args, **kwargs: next(processes))
    args = argparse.Namespace(
        bundle=bundle,
        trace_binary=Path("/trace"),
        demo=Path("/demo"),
        runner=Path("/runner"),
        ffmpeg=Path("/ffmpeg"),
        ffprobe=Path("/ffprobe"),
        python=Path("/python"),
        trace_timeout=1.0,
        verification_timeout=1.0,
    )

    result = module.verify_only(args)

    assert result["live_trace_replays"] == {
        "incline_mu_0_4": True,
        "incline_mu_0_5": True,
    }
    assert result["live_capture_reverification"] is False
    assert result["durable_capture_reverification"] is True
    assert not any((bundle / relative).exists() for relative in module.STAGING_PATHS)


def test_verify_only_fails_on_live_trace_drift(tmp_path, monkeypatch):
    module = _load_module()
    bundle = tmp_path / "bundle"
    for scenario in module.SCENARIOS:
        path = bundle / "traces" / f"{scenario}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("stored\n", encoding="utf-8")
        (bundle / "traces" / f"{scenario}.stderr.txt").write_text("", encoding="utf-8")
    module.write_json(bundle / "verification.json", {"pass": True})
    monkeypatch.setattr(
        module, "_require_file", lambda path, label: Path(path).resolve()
    )
    monkeypatch.setattr(module, "verify_finalized", lambda *args, **kwargs: {})
    monkeypatch.setattr(
        module,
        "_run_command",
        lambda *args, **kwargs: subprocess.CompletedProcess([], 0, "drift\n", ""),
    )
    args = argparse.Namespace(
        bundle=bundle,
        trace_binary=Path("/trace"),
        demo=Path("/demo"),
        runner=Path("/runner"),
        ffmpeg=Path("/ffmpeg"),
        ffprobe=Path("/ffprobe"),
        python=Path("/python"),
        trace_timeout=1.0,
        verification_timeout=1.0,
    )

    with pytest.raises(ValueError, match="trace replay differs"):
        module.verify_only(args)


def test_bundle_transaction_restores_previous_bytes_after_trace_failure(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    old_files = {
        "run-summary.json": b"old capture\n",
        "metadata.json": b"old metadata\n",
        "artifact-index.json": b"old index\n",
        "traces/incline_mu_0_4.csv": b"old trace\n",
    }
    for relative, contents in old_files.items():
        path = bundle / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contents)

    with pytest.raises(ValueError, match="injected trace failure"):
        with module._bundle_transaction(bundle):
            module._clean_generated_outputs(bundle)
            partial = bundle / "traces/incline_mu_0_5.csv"
            partial.parent.mkdir(parents=True, exist_ok=True)
            partial.write_bytes(b"partial replacement\n")
            (bundle / "run-summary.json").write_bytes(b"mutated capture\n")
            raise ValueError("injected trace failure")

    restored = {
        path.relative_to(bundle).as_posix(): path.read_bytes()
        for path in bundle.rglob("*")
        if path.is_file()
    }
    assert restored == old_files


@pytest.mark.parametrize("replacement_kind", ["regular_file", "symlink"])
def test_bundle_transaction_restores_after_root_path_replacement(
    tmp_path, replacement_kind
):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "marker.txt").write_text("before\n", encoding="utf-8")
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel.txt"
    sentinel.write_text("outside\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="injected replacement failure"):
        with module._bundle_transaction(bundle):
            shutil.rmtree(bundle)
            if replacement_kind == "regular_file":
                bundle.write_text("replacement\n", encoding="utf-8")
            else:
                bundle.symlink_to(outside, target_is_directory=True)
            raise RuntimeError("injected replacement failure")

    assert bundle.is_dir()
    assert not bundle.is_symlink()
    assert (bundle / "marker.txt").read_text(encoding="utf-8") == "before\n"
    assert sentinel.read_text(encoding="utf-8") == "outside\n"
    assert not list(tmp_path.glob(".bundle.backup-*"))


def test_finalize_validates_prerequisites_before_deleting_previous_outputs(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    previous_report = b"previous report\n"
    (bundle / "REPORT.md").write_bytes(previous_report)
    args = argparse.Namespace(
        bundle=bundle,
        reuse_current_capture=False,
        trace_binary=tmp_path / "missing-trace-binary",
    )

    with pytest.raises(ValueError, match="trace binary is not a regular file"):
        module.finalize(args)

    assert (bundle / "REPORT.md").read_bytes() == previous_report


def test_fresh_capture_refuses_to_replace_finalized_bundle(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    previous = b"previous finalized metadata\n"
    (bundle / "metadata.json").write_bytes(previous)

    with pytest.raises(ValueError, match="refuses to replace"):
        module.finalize(argparse.Namespace(bundle=bundle, reuse_current_capture=False))

    assert (bundle / "metadata.json").read_bytes() == previous


def test_fresh_capture_cleanup_removes_stale_manual_record(tmp_path):
    module = _load_module()
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    stale_manual = bundle / "manual-inspection.json"
    stale_manual.write_bytes(b"stale manual record\n")
    generated = bundle / "REPORT.md"
    generated.write_bytes(b"generated\n")

    module._clean_capture_outputs(bundle)

    assert not stale_manual.exists()
    assert generated.read_bytes() == b"generated\n"


def test_run_command_timeout_terminates_the_descendant_process_group(tmp_path):
    module = _load_module()
    child_pid_path = tmp_path / "child.pid"
    child_code = (
        "import os, signal, time; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "os.close(1); os.close(2); time.sleep(60)"
    )
    parent_code = (
        "import pathlib, subprocess, sys, time; "
        f"child = subprocess.Popen([sys.executable, '-c', {child_code!r}]); "
        f"pathlib.Path({str(child_pid_path)!r}).write_text(str(child.pid)); "
        "time.sleep(60)"
    )

    with pytest.raises(subprocess.TimeoutExpired):
        module._run_command([sys.executable, "-c", parent_code], timeout=0.5)

    child_pid = int(child_pid_path.read_text(encoding="utf-8"))

    def process_is_live(pid):
        stat_path = Path(f"/proc/{pid}/stat")
        try:
            state = stat_path.read_text(encoding="utf-8").split()[2]
        except (FileNotFoundError, IndexError, ProcessLookupError):
            return False
        return state not in {"X", "Z"}

    for _ in range(20):
        if not process_is_live(child_pid):
            break
        time.sleep(0.05)
    assert not process_is_live(child_pid)


def test_interrupt_during_timeout_cleanup_still_kills_descendants(
    tmp_path, monkeypatch
):
    module = _load_module()
    child_pid_path = tmp_path / "timeout-interrupt-child.pid"
    child_code = (
        "import os, signal, time; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "os.close(1); os.close(2); time.sleep(60)"
    )
    parent_code = (
        "import pathlib, subprocess, sys, time; "
        f"child = subprocess.Popen([sys.executable, '-c', {child_code!r}]); "
        f"pathlib.Path({str(child_pid_path)!r}).write_text(str(child.pid)); "
        "time.sleep(60)"
    )
    real_sleep = time.sleep
    interrupted = False

    def interrupt_cleanup_sleep(duration):
        nonlocal interrupted
        if not interrupted:
            interrupted = True
            raise KeyboardInterrupt
        real_sleep(duration)

    monkeypatch.setattr(module.time, "sleep", interrupt_cleanup_sleep)
    with pytest.raises(KeyboardInterrupt):
        module._run_command([sys.executable, "-c", parent_code], timeout=0.5)

    child_pid = int(child_pid_path.read_text(encoding="utf-8"))
    for _ in range(20):
        stat_path = Path(f"/proc/{child_pid}/stat")
        if not stat_path.is_file():
            break
        if stat_path.read_text(encoding="utf-8").split()[2] in {"X", "Z"}:
            break
        real_sleep(0.05)
    if stat_path.is_file():
        assert stat_path.read_text(encoding="utf-8").split()[2] in {"X", "Z"}


def test_run_command_interrupt_cleans_descendants_before_reraising(
    tmp_path, monkeypatch
):
    module = _load_module()
    child_pid_path = tmp_path / "interrupt-child.pid"
    late_write_path = tmp_path / "late-write.txt"
    child_code = (
        "import os, pathlib, signal, time; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "os.close(1); os.close(2); time.sleep(1.5); "
        f"pathlib.Path({str(late_write_path)!r}).write_text('survived')"
    )
    parent_code = (
        "import pathlib, subprocess, sys, time; "
        f"child = subprocess.Popen([sys.executable, '-c', {child_code!r}]); "
        f"pathlib.Path({str(child_pid_path)!r}).write_text(str(child.pid)); "
        "time.sleep(60)"
    )
    real_popen = subprocess.Popen

    class InterruptingPopen:
        def __init__(self, *args, **kwargs):
            self._process = real_popen(*args, **kwargs)
            self._interrupted = False

        @property
        def pid(self):
            return self._process.pid

        @property
        def returncode(self):
            return self._process.returncode

        def communicate(self, *args, **kwargs):
            if not self._interrupted:
                self._interrupted = True
                deadline = time.monotonic() + 2.0
                while not child_pid_path.is_file() and time.monotonic() < deadline:
                    time.sleep(0.01)
                assert child_pid_path.is_file()
                raise KeyboardInterrupt
            return self._process.communicate(*args, **kwargs)

    monkeypatch.setattr(module.subprocess, "Popen", InterruptingPopen)
    with pytest.raises(KeyboardInterrupt):
        module._run_command([sys.executable, "-c", parent_code], timeout=30.0)

    child_pid = int(child_pid_path.read_text(encoding="utf-8"))
    time.sleep(0.6)
    assert not late_write_path.exists()
    stat_path = Path(f"/proc/{child_pid}/stat")
    if stat_path.is_file():
        assert stat_path.read_text(encoding="utf-8").split()[2] in {"X", "Z"}


def test_json_helpers_refuse_nonfinite_payloads(tmp_path):
    module = _load_module()

    with pytest.raises(ValueError, match="Out of range float"):
        module._payload_sha256({"bad": float("nan")})
    bad = tmp_path / "bad.json"
    bad.write_text('{"value": NaN}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="non-finite JSON number"):
        module.read_json(bad)


def test_main_rejects_mutually_exclusive_two_phase_modes(capsys):
    module = _load_module()

    assert module.main(["--verify-only", "--reuse-current-capture"]) == 2
    assert "mutually exclusive" in capsys.readouterr().err
