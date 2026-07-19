import importlib.util
import io
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/run_fbf_literal_arch101_v1.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("run_fbf_literal_arch101_v1", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _valid_row(module, step: int) -> dict[str, str]:
    row = {field: "0" for field in module.EXPECTED_HEADER}
    row.update(
        {
            "step": str(step),
            "time": format(step / 60.0, ".17g"),
            "scenario": module.SCENARIO,
            "solver": "exact_fbf",
            "solver_contract": module.SOLVER_CONTRACT,
            "precision_contract": "float64",
            "scene_contract": module.SCENE_CONTRACT,
            "baumgarte_contract": (
                "split_impulse_no_velocity_baumgarte_erp_zero_vs_paper"
            ),
            "collision_frontend": "native",
            "inner_local_solver": "exact_metric",
            "inner_sweeps_requested": "30",
            "fixed_inner_sweeps_requested": "1",
            "step_size_persistence_enabled": "0",
            "step_size_recovery_growth_factor": "1.05",
            "step_size_persistence_used": "0",
            "step_size_persistence_request": "nan",
            "row_operator_request": "contact_row_no_dense_snapshot",
            "row_operator_mode": "contact_row_no_dense_snapshot",
            "wall_ms": "2.5",
            "requested_threads": "4",
            "actual_threads": "4",
            "contacts": "400",
            "unique_colliding_body_pairs": "100",
            "penetration_depth_min": "1e-7",
            "penetration_depth_median": "2e-7",
            "penetration_depth_p95": "3e-7",
            "penetration_depth_max": "4e-7",
            "exact_diagnostics_contract": (
                "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
                "warm_fraction_over_step_contacts"
            ),
            "step_exact_solves": "1",
            "step_warm_starts": "0" if step == 1 else "1",
            "step_exact_failures": "0",
            "step_fallbacks": "0",
            "step_fbf_iterations": "12",
            "residual": "5e-7",
            "status": "success",
            "residual_primal_feasibility": "0",
            "residual_dual_feasibility": "4e-7",
            "residual_complementarity": "5e-7",
            "accepted_gamma": "0.1",
            "safe_gamma": "0.01",
            "shrink_iterations": "0",
            "coupling_variation_ratio": "0.2",
            "warm_start_matched_contacts": "0" if step == 1 else "400",
            "warm_start_matched_fraction": "0" if step == 1 else "1",
            "phase": "single_phase",
            "card_count": "0",
            "projectile_count": "0",
            "finite_state": "1",
            "min_card_axis_up": "nan",
            "min_center_height": "nan",
            "max_card_horizontal_travel": "0",
            "max_projectile_speed": "0",
            "tracked_body": "masonry_arch_stone_50_body",
            "x": "0",
            "y": "0",
            "z": "0.61",
            "vx": "0",
            "vy": "0",
            "vz": "0",
            "up_z": "1",
            "max_outer_iterations": "5000",
            "tolerance": "1e-6",
            "accept_outer_max_iterations": "0",
            "inner_local_iterations": "1",
            "adaptive_step_size_enabled": "1",
            "warm_start_enabled": "1",
            "projected_gradient_retry_enabled": "0",
            "dense_residual_polish_enabled": "0",
            "fallback_to_boxed_lcp_enabled": "0",
            "diagonal_seed_enabled": "0",
            "matrix_free_seed_enabled": "0",
            "step_size_scale": "35",
            "outer_relaxation": "1.1",
            "initial_gamma_contract": "automatic_safe_bound",
            "split_impulse_enabled": "1",
            "step_contact_row_delassus_products": "100",
            "step_parallel_contact_row_delassus_products": "0",
            "max_contact_row_participants_to_date": "1",
            "exact_contact_row_logical_cpus_to_date": "none",
            "max_phase_contact_row_logical_cpus_to_date": "none",
            "max_card_center_displacement_from_initial": "nan",
            "min_card_orientation_alignment_from_initial": "nan",
            "projectile_card_contacts": "0",
            "inner_bgs_schedule_contract": module.SCHEDULE_CONTRACT,
            "last_exact_colored_bgs_used": "1",
            "last_exact_colored_bgs_solves": "12",
            "last_exact_colored_bgs_dispatches": "1",
            "last_exact_colored_bgs_max_participants": "4",
            "last_exact_colored_bgs_manifolds": "100",
            "last_exact_colored_bgs_colors": "3",
            "last_exact_colored_bgs_max_manifolds_per_color": "34",
            "exact_colored_bgs_logical_cpus": "8;10;12;14",
            "max_phase_exact_colored_bgs_logical_cpus": "8;10;12;14",
            "max_arch_body_displacement_from_initial": "0.001",
            "min_arch_body_orientation_alignment_from_initial": "0.99",
        }
    )
    return row


def _csv_text(module, rows: list[dict[str, str]]) -> str:
    stream = io.StringIO()
    writer = __import__("csv").DictWriter(stream, fieldnames=module.EXPECTED_HEADER)
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().replace("\r\n", "\n")


def _probe_text() -> str:
    identity = (
        "stone_count=101,backend=native,policy=closure_1um,"
        "gap_policy=omitted_offsets"
    )
    lines = [
        (
            f"metadata,{identity},end_face_expansion_m=1e-6,"
            "downward_shift_m=0.001001,friction=0.8,pinned_springers=0:100,"
            "pinned_springers_valid=true,mobile_skeletons=99,"
            "collision_only=true,dynamic_claim=false,"
            "exact_polyhedral_inertia=true,source_density_kg_m3=1000,"
            "exact_volume_m3=0.01,nominal_mass_from_exact_volume_kg=10,"
            "convex_shape_aabb_volume_m3=0.02"
        )
    ]
    for repeat in range(2):
        lines.append(
            f"sample,{identity},repeat={repeat},contacts=102,unique_pairs=102,"
            "adjacent_stone_pairs=100,nonadjacent_stone_pairs=0,"
            "springer_ground_pairs=2,unexpected_ground_pairs=0,"
            "min_penetration_m=1e-6,max_penetration_m=1.5e-6,"
            "mean_penetration_m=1.1e-6,nonfinite_contacts=0"
        )
    for index in range(100):
        lines.append(
            f"pair,{identity},first=masonry_arch_wedge_{index}_shape,"
            f"second=masonry_arch_wedge_{index + 1}_shape,"
            "kind=adjacent_stones,contacts=1"
        )
    for index in (0, 100):
        lines.append(
            f"pair,{identity},first=masonry_arch_ground_shape,"
            f"second=masonry_arch_wedge_{index}_shape,"
            "kind=springer_ground,contacts=1"
        )
    lines.append(
        f"verdict,{identity},repeated_collision_stable=true,"
        "numerical_100_contact_target_observed=not_applicable,"
        "genuine_contact_graph=true,dynamic_path_candidate=not_applicable,"
        "paper_contract_proven=false"
    )
    return "\n".join(lines) + "\n"


def test_fixed_command_uses_frozen_scenario_threads_and_affinity():
    module = _load_module()

    assert module._build_command(Path("/tmp/fbf_paper_trace")) == [
        "taskset",
        "--cpu-list",
        "8,10,12,14",
        "/tmp/fbf_paper_trace",
        "masonry_arch_101_literal_wedge",
        "exact_fbf",
        "1",
        "600",
        "nan",
        "performance",
        "default",
        "default",
        "4",
        "dart_best_colored_bgs",
        "native",
        "default",
        "0",
        "0",
    ]


def test_collision_probe_command_uses_two_nonvacuous_repeats():
    module = _load_module()

    assert module._build_collision_probe_command(Path("/tmp/probe")) == [
        "/tmp/probe",
        "2",
    ]


def test_frozen_protocol_contract_hash_is_pinned():
    module = _load_module()

    assert module._protocol_contract_sha256() == (
        module.EXPECTED_PROTOCOL_CONTRACT_SHA256
    )


def test_protocol_contract_tampering_is_rejected(tmp_path, monkeypatch):
    module = _load_module()
    protocol = tmp_path / "protocol.md"
    protocol.write_text("changed\n\n## Frozen v1 result\nnot run\n", encoding="utf-8")
    monkeypatch.setattr(module, "PROTOCOL", protocol)

    with pytest.raises(module.EvidenceError, match="contract hash drifted"):
        module._protocol_contract_sha256()


def test_collision_probe_proves_only_constructed_compact_pair_identity():
    module = _load_module()

    summary = module._validate_collision_probe(_probe_text(), 0)

    assert summary["constructed_initial_scene_passed"] is True
    assert summary["repeat_count"] == 2
    assert summary["contacts"] == 102
    assert summary["adjacent_stone_pairs"] == 100
    assert summary["springer_ground_pairs"] == 2
    assert summary["dynamic_pair_identity_evidence"] is False
    assert "dynamic pair-identity" in summary["promotion_boundary"]


@pytest.mark.parametrize(
    ("old", "new", "message"),
    [
        ("contacts=102", "contacts=400", "contacts"),
        ("nonfinite_contacts=0", "nonfinite_contacts=1", "nonfinite"),
        (
            "repeated_collision_stable=true",
            "repeated_collision_stable=false",
            "verdict",
        ),
        ("min_penetration_m=1e-6", "min_penetration_m=nan", "non-finite"),
    ],
)
def test_collision_probe_tampering_is_rejected(old: str, new: str, message: str):
    module = _load_module()

    with pytest.raises(module.EvidenceError, match=message):
        module._validate_collision_probe(_probe_text().replace(old, new), 0)


def test_collision_probe_pair_identity_tampering_is_rejected():
    module = _load_module()
    tampered = _probe_text().replace(
        "second=masonry_arch_wedge_100_shape,kind=adjacent_stones",
        "second=masonry_arch_wedge_99_shape,kind=adjacent_stones",
        1,
    )

    with pytest.raises(module.EvidenceError, match="non-adjacent|identity"):
        module._validate_collision_probe(tampered, 0)


def test_fake_complete_600_row_positive_passes_all_gates():
    module = _load_module()
    rows = [_valid_row(module, step) for step in range(1, 601)]

    summary = module._validate_trace(
        header=module.EXPECTED_HEADER, rows=rows, returncode=0
    )

    assert summary["artifact_valid"] is True
    assert summary["standing_claim_passed"] is True
    assert summary["emitted_steps"] == 600
    assert summary["timing_evidence_eligible"] is False
    assert len(summary["normalized_scene_work_fingerprint_sha256"]) == 64


def test_expected_fail_fast_prefix_is_preserved_as_invalid_negative():
    module = _load_module()
    rows = [_valid_row(module, step) for step in range(1, 4)]
    rows[-1].update(
        {
            "step_exact_solves": "0",
            "step_exact_failures": "1",
            "step_fbf_iterations": "5000",
            "residual": "0.7815",
            "status": "fbf_failed",
            "last_exact_colored_bgs_solves": "5000",
        }
    )

    summary = module._validate_trace(
        header=module.EXPECTED_HEADER, rows=rows, returncode=1
    )

    assert summary["classification"] == "failed_prefix_scientific_negative"
    assert summary["artifact_valid"] is False
    assert summary["standing_claim_passed"] is False
    assert summary["first_failed_step"] == 3
    assert summary["timing_evidence_eligible"] is False
    assert summary["timing_statistics"] is None


def test_fail_fast_trace_cannot_continue_after_failed_gate():
    module = _load_module()
    rows = [_valid_row(module, step) for step in range(1, 4)]
    rows[1]["residual"] = "0.1"
    rows[-1].update(
        {
            "step_exact_solves": "0",
            "step_exact_failures": "1",
            "residual": "0.2",
            "status": "fbf_failed",
        }
    )

    with pytest.raises(module.EvidenceError, match="continued past"):
        module._validate_trace(header=module.EXPECTED_HEADER, rows=rows, returncode=1)


def test_parameter_tampering_is_rejected():
    module = _load_module()
    row = _valid_row(module, 1)
    row["step_size_scale"] = "34"

    with pytest.raises(module.EvidenceError, match="step_size_scale"):
        module._validate_trace(header=module.EXPECTED_HEADER, rows=[row], returncode=1)


@pytest.mark.parametrize(
    "field",
    [
        "residual",
        "x",
        "max_arch_body_displacement_from_initial",
        "min_arch_body_orientation_alignment_from_initial",
    ],
)
def test_nonfinite_required_metric_is_rejected(field: str):
    module = _load_module()
    row = _valid_row(module, 1)
    row[field] = "nan"

    with pytest.raises(module.EvidenceError, match="non-finite"):
        module._validate_trace(header=module.EXPECTED_HEADER, rows=[row], returncode=1)


def test_falsified_finite_state_boolean_is_rejected():
    module = _load_module()
    row = _valid_row(module, 1)
    row["finite_state"] = "0"

    with pytest.raises(module.EvidenceError, match="finite_state"):
        module._validate_trace(header=module.EXPECTED_HEADER, rows=[row], returncode=1)


@pytest.mark.parametrize(
    "field",
    [
        "exact_contact_row_logical_cpus_to_date",
        "max_phase_contact_row_logical_cpus_to_date",
    ],
)
def test_contact_row_residency_tampering_is_rejected(field: str):
    module = _load_module()
    row = _valid_row(module, 1)
    row[field] = "8"

    with pytest.raises(module.EvidenceError, match=field):
        module._validate_trace(header=module.EXPECTED_HEADER, rows=[row], returncode=1)


def test_normalized_work_fingerprint_drift_is_rejected():
    module = _load_module()
    rows = [_valid_row(module, 1), _valid_row(module, 2)]
    rows[-1]["contacts"] = "399"

    with pytest.raises(module.EvidenceError, match="contacts|fingerprint"):
        module._validate_trace(header=module.EXPECTED_HEADER, rows=rows, returncode=1)


def test_strict_schema_parser_accepts_only_frozen_95_columns():
    module = _load_module()
    text = _csv_text(module, [_valid_row(module, 1)])

    header, rows = module._parse_trace(text)

    assert tuple(header) == module.EXPECTED_HEADER
    assert len(header) == 95
    assert len(rows) == 1


def test_schema_header_tampering_is_rejected():
    module = _load_module()
    text = _csv_text(module, [_valid_row(module, 1)]).replace(
        "step,time,", "step,clock,", 1
    )

    with pytest.raises(module.EvidenceError, match="header hash mismatch"):
        module._parse_trace(text)


def test_fresh_output_directory_rejects_existing_path(tmp_path):
    module = _load_module()

    with pytest.raises(module.EvidenceError, match="must be fresh"):
        module._prepare_output(tmp_path)


def test_executable_identity_hashes_ldd_tool_build_libdart_and_all_libraries(
    tmp_path, monkeypatch
):
    module = _load_module()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    binary = tmp_path / "trace"
    binary.write_bytes(b"binary")
    ldd = tmp_path / "ldd"
    ldd.write_bytes(b"ldd tool")
    libdart = tmp_path / "build/lib/libdart.so.6.19"
    libdart.parent.mkdir(parents=True)
    libdart.write_bytes(b"dart")
    libc = tmp_path / "libc.so.6"
    libc.write_bytes(b"libc")
    monkeypatch.setattr(module.shutil, "which", lambda name: str(ldd))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0],
            0,
            f"libdart.so.6.19 => {libdart} (0x1)\n"
            f"libc.so.6 => {libc} (0x2)\nlinux-vdso.so.1 (0x3)\n",
            "",
        ),
    )

    identity = module._executable_identity(binary)

    assert identity["ldd_tool"]["sha256"] == module._sha256_file(ldd)
    assert identity["resolved_regular_shared_library_count"] == 2
    assert identity["resolved_build_libdart"]["sha256"] == module._sha256_file(libdart)
    assert {
        library["sha256"] for library in identity["resolved_regular_shared_libraries"]
    } == {
        module._sha256_file(libdart),
        module._sha256_file(libc),
    }
    assert len(identity["ldd_resolution_output_normalized_sha256"]) == 64


def test_executable_identity_rejects_unresolved_library(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    binary = tmp_path / "trace"
    binary.write_bytes(b"binary")
    ldd = tmp_path / "ldd"
    ldd.write_bytes(b"ldd tool")
    monkeypatch.setattr(module.shutil, "which", lambda name: str(ldd))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0], 0, "libmissing.so => not found\n", ""
        ),
    )

    with pytest.raises(module.EvidenceError, match="unresolved shared library"):
        module._executable_identity(binary)


def test_main_preserves_probe_output_and_fails_closed_on_probe_identity_drift(
    tmp_path, monkeypatch
):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"binary")
    probe = tmp_path / "collision_probe"
    probe.write_bytes(b"probe")
    output = tmp_path / "evidence"
    identities = iter(
        [
            {"trace_source_sha256": "before"},
            {"trace_source_sha256": "after"},
        ]
    )
    monkeypatch.setattr(module, "_affinity_metadata", lambda: {"checked": True})
    monkeypatch.setattr(
        module, "_execution_identity", lambda unused, also_unused: next(identities)
    )
    monkeypatch.setattr(
        module,
        "_run_captured",
        lambda command, timeout: ("probe stdout\n", "probe stderr\n", 0),
    )

    assert (
        module.main(
            [
                "--binary",
                str(binary),
                "--collision-probe",
                str(probe),
                "--output-dir",
                str(output),
            ]
        )
        == 1
    )
    assert (output / "raw.csv").read_text(encoding="utf-8") == ""
    assert (output / "collision_probe_stdout.txt").read_text(
        encoding="utf-8"
    ) == "probe stdout\n"
    assert (output / "collision_probe_stderr.txt").read_text(
        encoding="utf-8"
    ) == "probe stderr\n"
    summary = __import__("json").loads(
        (output / "summary.json").read_text(encoding="utf-8")
    )
    metadata = __import__("json").loads(
        (output / "metadata.json").read_text(encoding="utf-8")
    )
    assert summary["classification"] == "invalid_artifact"
    assert summary["artifact_valid"] is False
    assert summary["standing_claim_passed"] is False
    assert "drifted" in summary["error"]
    for name in (
        "raw.csv",
        "stderr.txt",
        "collision_probe_stdout.txt",
        "collision_probe_stderr.txt",
        "invocation.json",
        "summary.json",
        "REPORT.md",
    ):
        assert metadata["artifact_sha256"][name] == module._sha256_file(output / name)


def test_main_rechecks_full_identity_after_trace_child(tmp_path, monkeypatch):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"binary")
    probe = tmp_path / "collision_probe"
    probe.write_bytes(b"probe")
    output = tmp_path / "evidence"
    before = {"trace_executable": {"sha256": "before"}}
    identities = iter([before, before, {"trace_executable": {"sha256": "after"}}])
    captures = iter(
        [
            (_probe_text(), "probe warning\n", 0),
            ("trace stdout\n", "trace stderr\n", 0),
        ]
    )
    monkeypatch.setattr(module, "_affinity_metadata", lambda: {"checked": True})
    monkeypatch.setattr(
        module, "_execution_identity", lambda unused, also_unused: next(identities)
    )
    monkeypatch.setattr(
        module, "_run_captured", lambda command, timeout: next(captures)
    )

    assert (
        module.main(
            [
                "--binary",
                str(binary),
                "--collision-probe",
                str(probe),
                "--output-dir",
                str(output),
            ]
        )
        == 1
    )
    summary = __import__("json").loads(
        (output / "summary.json").read_text(encoding="utf-8")
    )
    assert "drifted after trace" in summary["error"]
    assert (output / "raw.csv").read_text(encoding="utf-8") == "trace stdout\n"
