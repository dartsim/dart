import hashlib
import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/run_fbf_literal_crown_impact_negative.py"
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "run_fbf_literal_crown_impact_negative", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _valid_rows(module):
    rows = []
    for step in range(1, 721):
        prefix = step <= 600
        row = {
            "step": str(step),
            "scenario": module.SCENARIO,
            "scene_contract": module.SCENE_CONTRACT,
            "impact_phase": "standing_prefix" if prefix else "crown_impact",
            "standing_prefix_comparable": "1" if prefix else "0",
            "impact_projectile_count": "0" if prefix else "3",
            "step_projectile_arch_contacts": "0",
            "step_projectile_ground_contacts": "0",
            "preimpact_standing_gate": "1" if step >= 600 else "-1",
            "final_gates_authoritative": "1" if step == 720 else "0",
            "preimpact_snapshot_captured": "1" if step >= 601 else "0",
            "first_projectile_arch_contact_step": "607" if step >= 607 else "-1",
            "first_projectile_arch_contact_time": (
                "10.116666666666667" if step >= 607 else "nan"
            ),
            "first_projectile_ground_contact_step": ("616" if step >= 616 else "-1"),
            "first_projectile_ground_contact_time": (
                "10.266666666666666" if step >= 616 else "nan"
            ),
            "exact_solves_to_date": "854" if step == 720 else str(step),
            "exact_failures_to_date": "0",
            "boxed_fallbacks_to_date": "0",
            "finite_state_to_date": "1",
            "max_iterations_accepted_to_date": "5" if step == 720 else "0",
            "worst_exact_residual_to_date": "9.1545317042653963e-05",
            "max_crown_displacement_to_date": "0.070939644312156866",
            "max_arch_body_displacement_from_preimpact": "0.070939644312156866",
            "min_arch_orientation_alignment_from_preimpact": "0.99194711995436347",
            "max_far_field_displacement_from_preimpact": "0.060523747030465196",
            "max_springer_displacement_from_preimpact": "0",
            "min_springer_orientation_alignment_from_preimpact": "1",
            "far_field_adjacent_pairs": "16" if step >= 601 else "0",
        }
        row.update(
            {
                field: expected if step == 720 else "0"
                for field, expected in module.EXPECTED_FINAL_GATES.items()
            }
        )
        if step >= 600:
            row["preimpact_standing_gate"] = "1"
        rows.append(row)
    return rows


def _printf_literals(block: str) -> str:
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', block)
    return bytes("".join(literals), "utf-8").decode("unicode_escape")


def test_current_trace_impact_header_is_frozen_136_column_schema():
    module = _load_module()
    source = TRACE_SOURCE.read_text(encoding="utf-8")
    function = re.search(
        r"void printPerformanceHeader\(SolverContract contract\)\s*\{(.*?)"
        r"std::printf\(\"\\n\"\);",
        source,
        re.DOTALL,
    )
    assert function is not None
    body = function.group(1)
    base = re.search(r"^\s*std::printf\((.*?)\);", body, re.DOTALL)
    colored = re.search(
        r"if \(contract == SolverContract::DartBestColoredBgs\).*?"
        r"std::printf\((.*?)\);\s*\}",
        body,
        re.DOTALL,
    )
    impact = re.search(
        r"if \(gAppendLiteralCrownImpactColumns\).*?" r"std::printf\((.*?)\);\s*\}",
        body,
        re.DOTALL,
    )
    assert base is not None and colored is not None and impact is not None
    header = (
        _printf_literals(base.group(1))
        + _printf_literals(colored.group(1))
        + _printf_literals(impact.group(1))
    )

    assert len(header.split(",")) == module.EXPECTED_COLUMNS
    assert hashlib.sha256((header + "\n").encode()).hexdigest() == (
        module.EXPECTED_HEADER_SHA256
    )


def test_fixed_command_uses_only_preregistered_contract():
    module = _load_module()
    taskset_identity = {
        "path": "/tmp/taskset-alias",
        "resolved_path": "/opt/pinned/taskset",
    }

    command = module._build_command(Path("/tmp/fbf_paper_trace"), 8, taskset_identity)

    assert command[:3] == ["/opt/pinned/taskset", "-c", "8"]
    assert command[3:] == [
        "/tmp/fbf_paper_trace",
        module.SCENARIO,
        "exact_fbf",
        "1",
        "720",
        "nan",
        "performance",
        "default",
        "default",
        "1",
        "dart_best_colored_bgs",
        "native",
        "default",
        "0",
        "0",
    ]

    reference = module._build_reference_command(
        Path("/tmp/fbf_paper_trace"), 8, taskset_identity
    )
    assert reference[:3] == ["/opt/pinned/taskset", "-c", "8"]
    assert reference[3:8] == [
        "/tmp/fbf_paper_trace",
        module.REFERENCE_SCENARIO,
        "exact_fbf",
        "1",
        "600",
    ]

    assert module._build_command(Path("/tmp/fbf_paper_trace"), None)[0] == (
        "/tmp/fbf_paper_trace"
    )
    with pytest.raises(module.EvidenceError, match="pinned taskset identity"):
        module._build_command(Path("/tmp/fbf_paper_trace"), 8)


def test_frozen_preregistration_contract_hash_is_pinned():
    module = _load_module()

    assert module._preregistration_contract_sha256() == (
        module.EXPECTED_PREREGISTRATION_CONTRACT_SHA256
    )


def test_taskset_identity_resolves_and_hashes_exact_executable(tmp_path, monkeypatch):
    module = _load_module()
    executable = tmp_path / "taskset-real"
    executable.write_bytes(b"taskset executable")
    alias = tmp_path / "taskset-alias"
    alias.symlink_to(executable)
    monkeypatch.setattr(module.shutil, "which", lambda name: str(alias))

    identity = module._tool_identity("taskset")

    assert identity == {
        "name": "taskset",
        "path": str(alias),
        "resolved_path": str(executable),
        "size_bytes": executable.stat().st_size,
        "sha256": module._sha256_file(executable),
    }


def test_taskset_identity_rejects_missing_tool(monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module.shutil, "which", lambda name: None)

    with pytest.raises(module.EvidenceError, match="required tool is unavailable"):
        module._tool_identity("taskset")


def test_taskset_identity_rejects_nonregular_tool(tmp_path, monkeypatch):
    module = _load_module()
    directory = tmp_path / "taskset-directory"
    directory.mkdir()
    monkeypatch.setattr(module.shutil, "which", lambda name: str(directory))

    with pytest.raises(module.EvidenceError, match="not a regular file"):
        module._tool_identity("taskset")


def test_taskset_identity_rejects_unresolvable_tool(tmp_path, monkeypatch):
    module = _load_module()
    alias = tmp_path / "taskset-alias"
    alias.symlink_to(tmp_path / "missing-taskset")
    monkeypatch.setattr(module.shutil, "which", lambda name: str(alias))

    with pytest.raises(module.EvidenceError, match="cannot be resolved"):
        module._tool_identity("taskset")


def test_main_rejects_missing_taskset_before_child_execution(tmp_path, monkeypatch):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    output = tmp_path / "evidence"
    child_commands = []
    monkeypatch.setattr(module, "_preregistration_contract_sha256", lambda: "b" * 64)
    monkeypatch.setattr(
        module,
        "_executable_identity",
        lambda unused: {"sha256": "stable", "size_bytes": binary.stat().st_size},
    )
    monkeypatch.setattr(module.shutil, "which", lambda name: None)
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: child_commands.append(command),
    )

    with pytest.raises(SystemExit, match="required tool is unavailable: taskset"):
        module.main(
            [
                "--binary",
                str(binary),
                "--output-dir",
                str(output),
                "--cpu",
                "8",
            ]
        )

    assert child_commands == []


def test_executable_identity_binds_binary_ldd_and_build_libdart(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    ldd = tmp_path / "ldd"
    ldd.write_bytes(b"ldd tool")
    libdart = tmp_path / "build/lib/libdart.so.6.19"
    libdart.parent.mkdir(parents=True)
    libdart.write_bytes(b"dart runtime")
    libc = tmp_path / "libc.so.6"
    libc.write_bytes(b"libc runtime")
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

    assert identity["path"] == str(binary)
    assert identity["size_bytes"] == binary.stat().st_size
    assert identity["sha256"] == module._sha256_file(binary)
    assert identity["ldd_tool"]["resolved_path"] == str(ldd)
    assert identity["ldd_tool"]["size_bytes"] == ldd.stat().st_size
    assert identity["ldd_tool"]["sha256"] == module._sha256_file(ldd)
    assert identity["resolved_regular_shared_library_count"] == 2
    assert identity["resolved_build_libdart"] == {
        "soname": "libdart.so.6.19",
        "reported_path": str(libdart),
        "resolved_path": str(libdart),
        "size_bytes": libdart.stat().st_size,
        "sha256": module._sha256_file(libdart),
    }
    assert {
        library["resolved_path"]
        for library in identity["resolved_regular_shared_libraries"]
    } == {str(libdart), str(libc)}


def test_executable_identity_rejects_unresolved_library(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    ldd = tmp_path / "ldd"
    ldd.write_bytes(b"ldd tool")
    monkeypatch.setattr(module.shutil, "which", lambda name: str(ldd))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0], 0, "libdart.so.6.19 => not found\n", ""
        ),
    )

    with pytest.raises(module.EvidenceError, match="unresolved shared library"):
        module._executable_identity(binary)


def test_executable_identity_requires_build_tree_libdart(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    ldd = tmp_path / "ldd"
    ldd.write_bytes(b"ldd tool")
    external_libdart = tmp_path / "external/libdart.so.6.19"
    external_libdart.parent.mkdir()
    external_libdart.write_bytes(b"external dart")
    monkeypatch.setattr(module.shutil, "which", lambda name: str(ldd))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0],
            0,
            f"libdart.so.6.19 => {external_libdart} (0x1)\n",
            "",
        ),
    )

    with pytest.raises(module.EvidenceError, match="one build-tree libdart"):
        module._executable_identity(binary)


def test_main_records_and_rechecks_runtime_closure(tmp_path, monkeypatch):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    output = tmp_path / "evidence"
    runtime_identity = {
        "path": str(binary),
        "size_bytes": binary.stat().st_size,
        "sha256": module._sha256_file(binary),
        "resolved_build_libdart": {
            "resolved_path": "/tmp/build/lib/libdart.so.6.19",
            "size_bytes": 12,
            "sha256": "a" * 64,
        },
    }
    taskset_identity = {
        "name": "taskset",
        "path": str(tmp_path / "taskset-alias"),
        "resolved_path": str(tmp_path / "taskset-real"),
        "size_bytes": 10,
        "sha256": "c" * 64,
    }
    identity_calls = []
    taskset_calls = []
    commands = []
    captures = iter(
        (
            module.subprocess.CompletedProcess([], 1, "impact trace\n", "negative\n"),
            module.subprocess.CompletedProcess([], 0, "reference trace\n", ""),
        )
    )
    monkeypatch.setattr(module, "_preregistration_contract_sha256", lambda: "b" * 64)
    monkeypatch.setattr(
        module,
        "_executable_identity",
        lambda unused: identity_calls.append(str(unused)) or runtime_identity,
    )
    monkeypatch.setattr(
        module,
        "_tool_identity",
        lambda name: taskset_calls.append(name) or taskset_identity,
    )
    monkeypatch.setattr(module.platform, "platform", lambda: "test-platform")
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda command, **kwargs: commands.append(command) or next(captures),
    )
    monkeypatch.setattr(module, "_parse_trace", lambda unused: (["step"], [{}]))
    monkeypatch.setattr(
        module,
        "_validate_negative",
        lambda **kwargs: {
            "schema_version": module.SCHEMA_VERSION,
            "classification": "valid_scientific_negative",
            "artifact_valid": True,
            "impact_claim_passed": False,
        },
    )
    monkeypatch.setattr(module, "_require_frozen_fingerprint", lambda unused: None)
    monkeypatch.setattr(
        module, "_parse_reference_trace", lambda unused: (["step"], [{}])
    )
    monkeypatch.setattr(
        module,
        "_compare_standing_prefix",
        lambda **kwargs: {
            "steps_compared": 600,
            "fields_compared": 88,
            "mismatches": 0,
            "pass": True,
        },
    )
    monkeypatch.setattr(module, "_report", lambda summary, metadata: "report\n")

    assert (
        module.main(
            [
                "--binary",
                str(binary),
                "--output-dir",
                str(output),
                "--cpu",
                "8",
            ]
        )
        == 0
    )

    metadata = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
    assert len(identity_calls) == 3
    assert taskset_calls == ["taskset", "taskset", "taskset"]
    assert [command[0] for command in commands] == [
        taskset_identity["resolved_path"],
        taskset_identity["resolved_path"],
    ]
    assert metadata["runtime_identity"] == runtime_identity
    assert metadata["executed_tool_closure"] == {"taskset": taskset_identity}
    assert metadata["binary_size_bytes"] == binary.stat().st_size
    assert metadata["binary_sha256"] == module._sha256_file(binary)
    assert metadata["selected_cpu"] == 8
    assert metadata["identity_rechecks"] == [
        "after_impact_trace",
        "after_standing_reference_trace",
    ]


def test_main_fails_closed_when_runtime_closure_drifts_after_impact(
    tmp_path, monkeypatch
):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    output = tmp_path / "evidence"
    identities = iter(
        (
            {"sha256": "before", "size_bytes": binary.stat().st_size},
            {"sha256": "after", "size_bytes": binary.stat().st_size},
        )
    )
    monkeypatch.setattr(module, "_preregistration_contract_sha256", lambda: "b" * 64)
    monkeypatch.setattr(module, "_executable_identity", lambda unused: next(identities))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0], 1, "impact trace\n", "negative\n"
        ),
    )

    with pytest.raises(SystemExit, match="runtime closure drifted after impact trace"):
        module.main(["--binary", str(binary), "--output-dir", str(output)])

    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["classification"] == "invalid_artifact"
    assert summary["artifact_valid"] is False
    assert "runtime closure drifted" in summary["error"]


def test_main_fails_closed_when_runtime_closure_drifts_after_reference(
    tmp_path, monkeypatch
):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    output = tmp_path / "evidence"
    stable = {"sha256": "stable", "size_bytes": binary.stat().st_size}
    identities = iter((stable, stable, {**stable, "sha256": "after"}))
    captures = iter(
        (
            module.subprocess.CompletedProcess([], 1, "impact trace\n", "negative\n"),
            module.subprocess.CompletedProcess([], 0, "reference trace\n", ""),
        )
    )
    monkeypatch.setattr(module, "_preregistration_contract_sha256", lambda: "b" * 64)
    monkeypatch.setattr(module, "_executable_identity", lambda unused: next(identities))
    monkeypatch.setattr(
        module.subprocess, "run", lambda *args, **kwargs: next(captures)
    )
    monkeypatch.setattr(module, "_parse_trace", lambda unused: (["step"], [{}]))
    monkeypatch.setattr(
        module,
        "_validate_negative",
        lambda **kwargs: {
            "normalized_trace_fingerprint": module.EXPECTED_NORMALIZED_FINGERPRINT
        },
    )

    with pytest.raises(
        SystemExit, match="runtime closure drifted after standing reference trace"
    ):
        module.main(["--binary", str(binary), "--output-dir", str(output)])

    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["classification"] == "invalid_artifact"
    assert summary["artifact_valid"] is False
    assert "runtime closure drifted" in summary["error"]


def test_main_fails_closed_when_taskset_identity_drifts_after_impact(
    tmp_path, monkeypatch
):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    output = tmp_path / "evidence"
    runtime_identity = {"sha256": "stable", "size_bytes": binary.stat().st_size}
    taskset_before = {
        "name": "taskset",
        "path": "/tmp/taskset-alias",
        "resolved_path": "/tmp/taskset-real",
        "size_bytes": 10,
        "sha256": "a" * 64,
    }
    taskset_after = {**taskset_before, "sha256": "b" * 64}
    taskset_identities = iter((taskset_before, taskset_after))
    monkeypatch.setattr(module, "_preregistration_contract_sha256", lambda: "c" * 64)
    monkeypatch.setattr(module, "_executable_identity", lambda unused: runtime_identity)
    monkeypatch.setattr(module, "_tool_identity", lambda name: next(taskset_identities))
    monkeypatch.setattr(
        module.subprocess,
        "run",
        lambda *args, **kwargs: module.subprocess.CompletedProcess(
            args[0], 1, "impact trace\n", "negative\n"
        ),
    )

    with pytest.raises(
        SystemExit, match="taskset executable identity drifted after impact trace"
    ):
        module.main(
            [
                "--binary",
                str(binary),
                "--output-dir",
                str(output),
                "--cpu",
                "8",
            ]
        )

    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["classification"] == "invalid_artifact"
    assert summary["artifact_valid"] is False
    assert "taskset executable identity drifted" in summary["error"]


def test_main_fails_closed_when_taskset_identity_drifts_after_reference(
    tmp_path, monkeypatch
):
    module = _load_module()
    binary = tmp_path / "fbf_paper_trace"
    binary.write_bytes(b"trace binary")
    output = tmp_path / "evidence"
    runtime_identity = {"sha256": "stable", "size_bytes": binary.stat().st_size}
    taskset_before = {
        "name": "taskset",
        "path": "/tmp/taskset-alias",
        "resolved_path": "/tmp/taskset-real",
        "size_bytes": 10,
        "sha256": "a" * 64,
    }
    taskset_after = {**taskset_before, "resolved_path": "/tmp/substituted-taskset"}
    taskset_identities = iter((taskset_before, taskset_before, taskset_after))
    captures = iter(
        (
            module.subprocess.CompletedProcess([], 1, "impact trace\n", "negative\n"),
            module.subprocess.CompletedProcess([], 0, "reference trace\n", ""),
        )
    )
    monkeypatch.setattr(module, "_preregistration_contract_sha256", lambda: "c" * 64)
    monkeypatch.setattr(module, "_executable_identity", lambda unused: runtime_identity)
    monkeypatch.setattr(module, "_tool_identity", lambda name: next(taskset_identities))
    monkeypatch.setattr(
        module.subprocess, "run", lambda *args, **kwargs: next(captures)
    )
    monkeypatch.setattr(module, "_parse_trace", lambda unused: (["step"], [{}]))
    monkeypatch.setattr(
        module,
        "_validate_negative",
        lambda **kwargs: {
            "normalized_trace_fingerprint": module.EXPECTED_NORMALIZED_FINGERPRINT
        },
    )

    with pytest.raises(
        SystemExit,
        match="taskset executable identity drifted after standing reference trace",
    ):
        module.main(
            [
                "--binary",
                str(binary),
                "--output-dir",
                str(output),
                "--cpu",
                "8",
            ]
        )

    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["classification"] == "invalid_artifact"
    assert summary["artifact_valid"] is False
    assert "taskset executable identity drifted" in summary["error"]


def test_expected_fail_closed_run_is_valid_artifact_but_not_passing_claim():
    module = _load_module()
    rows = _valid_rows(module)

    summary = module._validate_negative(
        header=list(rows[0]),
        rows=rows,
        returncode=1,
        stderr=module.EXPECTED_STDERR + "\n",
    )

    assert summary["artifact_valid"] is True
    assert summary["classification"] == "valid_scientific_negative"
    assert summary["impact_claim_passed"] is False
    assert summary["final_gate_values"]["final_impact_acceptance_gate"] is False


def test_zero_child_exit_is_rejected_even_if_rows_describe_negative():
    module = _load_module()
    rows = _valid_rows(module)

    with pytest.raises(module.EvidenceError, match="return code"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=0,
            stderr=module.EXPECTED_STDERR + "\n",
        )


def test_accidental_passing_impact_gate_is_rejected():
    module = _load_module()
    rows = _valid_rows(module)
    rows[-1]["final_impact_acceptance_gate"] = "1"

    with pytest.raises(module.EvidenceError, match="final_impact_acceptance_gate"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=1,
            stderr=module.EXPECTED_STDERR + "\n",
        )


def test_projectile_in_standing_prefix_is_rejected():
    module = _load_module()
    rows = _valid_rows(module)
    rows[599]["impact_projectile_count"] = "3"

    with pytest.raises(module.EvidenceError, match="standing prefix"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=1,
            stderr=module.EXPECTED_STDERR + "\n",
        )


def test_frozen_negative_rejects_cap_count_drift():
    module = _load_module()
    rows = _valid_rows(module)
    rows[-1]["max_iterations_accepted_to_date"] = "4"

    with pytest.raises(module.EvidenceError, match="cap count drifted"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=1,
            stderr=module.EXPECTED_STDERR + "\n",
        )


@pytest.mark.parametrize(
    "field",
    [
        "worst_exact_residual_to_date",
        "max_crown_displacement_to_date",
        "max_arch_body_displacement_from_preimpact",
        "min_arch_orientation_alignment_from_preimpact",
        "max_far_field_displacement_from_preimpact",
        "max_springer_displacement_from_preimpact",
        "min_springer_orientation_alignment_from_preimpact",
    ],
)
def test_nonfinite_gate_metric_is_rejected(field: str):
    module = _load_module()
    rows = _valid_rows(module)
    rows[-1][field] = "nan"

    with pytest.raises(module.EvidenceError, match="non-finite"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=1,
            stderr=module.EXPECTED_STDERR + "\n",
        )


def test_positive_gate_bit_cannot_hide_bad_metric():
    module = _load_module()
    rows = _valid_rows(module)
    rows[-1]["max_crown_displacement_to_date"] = "0"
    rows[-1]["crown_response_gate"] = "1"

    with pytest.raises(module.EvidenceError, match="crown_response_gate"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=1,
            stderr=module.EXPECTED_STDERR + "\n",
        )


def test_impact_phase_requires_new_exact_solve_progress():
    module = _load_module()
    rows = _valid_rows(module)
    rows[-1]["exact_solves_to_date"] = rows[599]["exact_solves_to_date"]

    with pytest.raises(module.EvidenceError, match="exact solve progress"):
        module._validate_negative(
            header=list(rows[0]),
            rows=rows,
            returncode=1,
            stderr=module.EXPECTED_STDERR + "\n",
        )


def test_standing_prefix_comparison_checks_88_exact_fields():
    module = _load_module()
    reference_header = [f"field_{index}" for index in range(95)]
    excluded = list(module.STANDING_PREFIX_COMPARISON_EXCLUSIONS)
    for index, field in enumerate(excluded):
        reference_header[index] = field
    impact_header = [*reference_header, *[f"impact_{index}" for index in range(41)]]
    reference_rows = [
        {field: f"{field}:{step}" for field in reference_header} for step in range(600)
    ]
    impact_rows = [
        {
            **reference,
            **{f"impact_{index}": "0" for index in range(41)},
        }
        for reference in reference_rows
    ]
    for row in impact_rows:
        row["scenario"] = module.SCENARIO
        row["scene_contract"] = module.SCENE_CONTRACT

    comparison = module._compare_standing_prefix(
        impact_header=impact_header,
        impact_rows=impact_rows,
        reference_header=reference_header,
        reference_rows=reference_rows,
    )

    assert comparison["pass"] is True
    assert comparison["fields_compared"] == 88
    assert comparison["mismatches"] == 0


def test_standing_prefix_comparison_rejects_field_drift():
    module = _load_module()
    reference_header = [f"field_{index}" for index in range(95)]
    for index, field in enumerate(module.STANDING_PREFIX_COMPARISON_EXCLUSIONS):
        reference_header[index] = field
    impact_header = [*reference_header, *[f"impact_{index}" for index in range(41)]]
    reference_rows = [{field: "same" for field in reference_header} for _ in range(600)]
    impact_rows = [
        {**reference, **{f"impact_{index}": "0" for index in range(41)}}
        for reference in reference_rows
    ]
    comparable = next(
        field
        for field in reference_header
        if field not in module.STANDING_PREFIX_COMPARISON_EXCLUSIONS
    )
    impact_rows[99][comparable] = "drift"

    with pytest.raises(module.EvidenceError, match="reference mismatch"):
        module._compare_standing_prefix(
            impact_header=impact_header,
            impact_rows=impact_rows,
            reference_header=reference_header,
            reference_rows=reference_rows,
        )


def test_frozen_fingerprint_drift_is_rejected():
    module = _load_module()

    with pytest.raises(module.EvidenceError, match="fingerprint drifted"):
        module._require_frozen_fingerprint({"normalized_trace_fingerprint": "0" * 64})
