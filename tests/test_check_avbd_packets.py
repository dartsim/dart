"""Tests for scripts/check_avbd_packets.py (PLAN-091 WP-091.1)."""

import hashlib
import importlib.util
import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_avbd_packets.py"
PACKET_DIR = ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_avbd_packets", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _identity(**overrides):
    identity = {
        "rigid_contact_solver": "sequential_impulse",
        "rigid_point_joint_solver": "sequential_impulse",
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "scene-construction audit (WP-091.1)",
        "rigid_contact_selection": "contact_solver_method",
    }
    identity.update(overrides)
    return identity


def _write_packet(tmp_path, name, packet):
    path = tmp_path / name
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")
    return path


def _source_provenance(root: Path, *relative_paths: str):
    combined = hashlib.sha256()
    files = []
    for relative_text in relative_paths:
        payload = (root / relative_text).read_bytes()
        encoded_path = relative_text.encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
        files.append(
            {
                "path": relative_text,
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    return {
        "algorithm": "sha256-length-prefixed-path-and-content-v1",
        "digest": combined.hexdigest(),
        "files": files,
    }


def _file_sha256(path: Path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_committed_packet_corpus_passes():
    module = _load_module()
    assert module.main([]) == 0


def test_new_version_packet_without_identity_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": module.AVBD_PACKET_SCHEMA_VERSION},
    )
    errors = module.packet_errors(path)
    assert any("requires resolved_solver_identity" in error for error in errors)


def test_new_version_packet_with_identity_passes(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(),
        },
    )
    assert module.packet_errors(path) == []
    assert module.main(["--packet", str(path)]) == 0


def test_avbd_contact_claim_with_wrong_selection_source_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="avbd",
                avbd_rigid_contact_config_emplaced=False,
            ),
        },
    )
    errors = module.packet_errors(path)
    assert any(
        "contact_solver_method" in error and "sequential_impulse" in error
        for error in errors
    ), errors


def test_public_avbd_contact_claim_without_private_config_passes(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="avbd",
                rigid_point_joint_solver="avbd",
                rigid_contact_selection="world_solver_family",
            ),
        },
    )
    assert module.packet_errors(path) == []


def test_public_world_family_with_crossed_point_joint_solver_is_rejected(
    tmp_path,
):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="vbd",
                rigid_contact_selection="world_solver_family",
                rigid_point_joint_solver="avbd",
            ),
        },
    )
    errors = module.packet_errors(path)
    assert any(
        "rigid_point_joint_solver to be 'none' or match" in error for error in errors
    )


def test_contact_method_with_vbd_point_rows_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_point_joint_solver="vbd",
            ),
        },
    )
    errors = module.packet_errors(path)
    assert any(
        "requires rigid_point_joint_solver 'none' or 'sequential_impulse'" in error
        for error in errors
    )


def test_source_provenance_rejects_file_and_digest_drift(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    source = tmp_path / "solver.cpp"
    source.write_text("first\n")
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(),
            "source_provenance": _source_provenance(tmp_path, "solver.cpp"),
        },
    )
    assert module.packet_errors(path) == []

    source.write_text("second\n")
    errors = module.packet_errors(path)
    assert any("sha256 drifted for solver.cpp" in error for error in errors)
    assert any("source_provenance.digest" in error for error in errors)


def test_source_provenance_rejects_digest_mutation(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    (tmp_path / "solver.cpp").write_text("stable\n")
    provenance = _source_provenance(tmp_path, "solver.cpp")
    provenance["digest"] = "0" * 64
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(),
            "source_provenance": provenance,
        },
    )
    errors = module.packet_errors(path)
    assert any("source_provenance.digest" in error for error in errors)


def test_paper_source_provenance_rejects_recomputed_incomplete_path_list(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    packet_name = "avbd-paper-breakable-wall-packet.json"
    for relative in ("solver.cpp", "writer.py"):
        (tmp_path / relative).write_text(f"{relative}\n")
    monkeypatch.setattr(
        module,
        "PAPER_PACKET_SOURCE_PATHS",
        {packet_name: ("solver.cpp", "writer.py")},
    )
    packet = {
        "source_provenance": _source_provenance(
            tmp_path,
            "solver.cpp",
            "writer.py",
        )
    }
    assert module._source_provenance_errors(packet, packet_name) == []

    packet["source_provenance"] = _source_provenance(tmp_path, "solver.cpp")
    errors = module._source_provenance_errors(packet, packet_name)

    assert any("canonical ordered" in error for error in errors), errors


def test_paper_packet_rejects_capture_and_benchmark_source_drift(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    packet_name = "avbd-paper-breakable-wall-packet.json"
    monkeypatch.setattr(
        module,
        "PAPER_PACKET_SOURCE_PATHS",
        {packet_name: (module.BENCHMARK_SOURCE_PATH.as_posix(),)},
    )
    benchmark_source = tmp_path / module.BENCHMARK_SOURCE_PATH
    benchmark_source.parent.mkdir(parents=True)
    benchmark_source.write_text("benchmark source\n")
    current = {
        "algorithm": module.CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "digest": "1" * 64,
        "file_count": 7,
        "git_head": "2" * 40,
        "roots": ["dart", "python"],
    }
    monkeypatch.setattr(
        module,
        "compute_capture_source_provenance",
        lambda _root: dict(current),
    )
    benchmark_hash = _file_sha256(benchmark_source)
    packet = {
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": _identity(
            rigid_contact_solver="avbd",
            rigid_point_joint_solver="avbd",
            rigid_contact_selection="world_solver_family",
        ),
        "source_provenance": _source_provenance(
            tmp_path,
            module.BENCHMARK_SOURCE_PATH.as_posix(),
        ),
        "visual_evidence": {
            role: {"source_provenance": dict(current)} for role in ("impact", "outcome")
        },
        "benchmark": {
            "context": {
                "benchmark_source_sha256": benchmark_hash,
                "capture_source_provenance_digest": current["digest"],
            },
            "timing": {"median_cpu_time_per_step_ns": 5.0},
            "source_provenance": {
                "benchmark_source_sha256": benchmark_hash,
                "capture_source_provenance_digest": current["digest"],
            },
        },
    }
    path = _write_packet(
        tmp_path,
        packet_name,
        packet,
    )
    assert module.packet_errors(path) == []

    source_provenance = packet.pop("source_provenance")
    path.write_text(json.dumps(packet))
    errors = module.packet_errors(path)
    assert any(
        "source_provenance must be an object for a current paper packet" in error
        for error in errors
    )

    packet["source_provenance"] = source_provenance
    packet["visual_evidence"]["impact"]["source_provenance"]["digest"] = "0" * 64
    path.write_text(json.dumps(packet))
    errors = module.packet_errors(path)
    assert any(
        "visual_evidence.impact.source_provenance.digest" in error for error in errors
    )

    packet["visual_evidence"]["impact"]["source_provenance"] = dict(current)
    packet["benchmark"]["source_provenance"]["benchmark_source_sha256"] = "0" * 64
    path.write_text(json.dumps(packet))
    errors = module.packet_errors(path)
    assert any("benchmark source hash" in error for error in errors)
    assert any("benchmark.context.benchmark_source_sha256" in error for error in errors)


def test_linked_packet_validation_reaches_transitive_avbd_source(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        module,
        "_paper_capture_source_binding_errors",
        lambda _packet, _name: [],
    )
    monkeypatch.setattr(
        module,
        "PAPER_PACKET_SOURCE_PATHS",
        {name: ("solver.cpp",) for name in module.PAPER_PACKET_SOURCE_PATHS},
    )
    (tmp_path / "solver.cpp").write_text("current\n")

    avbd = _write_packet(
        tmp_path,
        "avbd-paper-breakable-wall-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="avbd",
                rigid_point_joint_solver="avbd",
                rigid_contact_selection="world_solver_family",
            ),
            "source_provenance": _source_provenance(tmp_path, "solver.cpp"),
        },
    )
    vbd = _write_packet(
        tmp_path,
        "avbd-paper-vbd-comparison-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="vbd",
                rigid_point_joint_solver="vbd",
                rigid_contact_selection="world_solver_family",
            ),
            "source_provenance": _source_provenance(tmp_path, "solver.cpp"),
            "linked_avbd_evidence": {
                "file": avbd.name,
                "sha256": _file_sha256(avbd),
            },
        },
    )
    si = _write_packet(
        tmp_path,
        "avbd-paper-sequential-impulse-comparison-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(),
            "source_provenance": _source_provenance(tmp_path, "solver.cpp"),
            "linked_avbd_vbd_evidence": {
                "file": vbd.name,
                "sha256": _file_sha256(vbd),
            },
        },
    )
    assert module.packet_errors(si) == []

    (tmp_path / "solver.cpp").write_text("drifted\n")
    errors = module.packet_errors(si)
    assert any("sha256 drifted for solver.cpp" in error for error in errors)


def test_unknown_solver_name_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="warm_started_magic"
            ),
        },
    )
    errors = module.packet_errors(path)
    assert any("rigid_contact_solver" in error for error in errors)


def test_new_packet_file_below_current_version_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": 1},
    )
    errors = module.packet_errors(path)
    assert any("must be written at schema_version" in error for error in errors)


def test_new_schema_v3_packet_cannot_claim_retired_avbd_public_pair_rows(
    tmp_path,
):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-pre-v4-identity-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION - 1,
            "resolved_solver_identity": _identity(rigid_point_joint_solver="avbd"),
        },
    )

    errors = module.packet_errors(path)

    assert any(
        f"schema_version {module.AVBD_PACKET_SCHEMA_VERSION}" in error
        for error in errors
    ), errors


def test_legacy_allowlisted_packet_stays_readable(tmp_path):
    module = _load_module()
    name = "avbd-demo2d-pyramid-packet.json"
    assert name in module.LEGACY_IDENTITY_EXEMPT_PACKETS
    path = _write_packet(tmp_path, name, {"schema_version": 1})
    assert module.packet_errors(path) == []


def test_legacy_v3_allowlisted_packet_rejects_schema_downgrade(tmp_path):
    module = _load_module()
    name = "avbd-breakable-joint-scale-packet.json"
    assert module.LEGACY_PACKET_SCHEMA_VERSIONS[name] == 3
    path = _write_packet(tmp_path, name, {"schema_version": 1})

    errors = module.packet_errors(path)

    assert any(
        "legacy allowlist requires schema_version 3, got 1" in error for error in errors
    ), errors


def test_non_integer_schema_version_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": "two"},
    )
    errors = module.packet_errors(path)
    assert any("schema_version must be an integer" in error for error in errors)


def test_legacy_allowlist_and_current_packets_cover_committed_corpus():
    module = _load_module()
    for path in PACKET_DIR.glob("avbd-*-packet.json"):
        packet = json.loads(path.read_text())
        legacy_version = module.LEGACY_PACKET_SCHEMA_VERSIONS.get(path.name)
        if legacy_version is None:
            assert packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION
            assert "resolved_solver_identity" in packet
        else:
            assert packet["schema_version"] == legacy_version
        assert module.packet_errors(path) == []


_TIMING_GATE_PACKET_NAME = "avbd-paper-sequential-impulse-comparison-packet.json"


def _timing_gate_packet():
    return {
        "benchmark": {
            "method": {"timing": {"median_cpu_time_per_step_ns": 10.0}},
            "comparison": {
                "sequential_impulse_to_avbd_median_cpu_cost_ratio": 2.0,
                "sequential_impulse_to_vbd_median_cpu_cost_ratio": 1.25,
            },
        },
        "linked_avbd_vbd_evidence": {
            "benchmark_method_timings": {
                "avbd": {"median_cpu_time_per_step_ns": 5.0},
                "vbd": {"median_cpu_time_per_step_ns": 8.0},
            },
        },
    }


def test_paper_benchmark_timing_gate_accepts_consistent_medians():
    module = _load_module()
    errors = module._paper_benchmark_timing_errors(
        _timing_gate_packet(), _TIMING_GATE_PACKET_NAME
    )
    assert errors == []


def test_paper_benchmark_timing_gate_rejects_ratio_drift():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["benchmark"]["comparison"][
        "sequential_impulse_to_avbd_median_cpu_cost_ratio"
    ] = 3.0
    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)
    assert any("the embedded medians record" in error for error in errors)


def test_paper_benchmark_timing_gate_rejects_non_positive_median():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["benchmark"]["method"]["timing"]["median_cpu_time_per_step_ns"] = 0.0
    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)
    assert any("positive finite number" in error for error in errors)


def test_paper_benchmark_timing_gate_rejects_unreferenced_median():
    module = _load_module()
    packet = _timing_gate_packet()
    del packet["linked_avbd_vbd_evidence"]
    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)
    assert any(
        "references a median the packet does not embed" in error for error in errors
    )


def test_paper_benchmark_timing_gate_rejects_typoed_numerator_name():
    module = _load_module()
    packet = _timing_gate_packet()
    comparison = packet["benchmark"]["comparison"]
    value = comparison.pop("sequential_impulse_to_avbd_median_cpu_cost_ratio")
    comparison["typo_to_avbd_median_cpu_cost_ratio"] = value

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(
        "references a median the packet does not embed" in error for error in errors
    ), errors


def test_paper_benchmark_timing_gate_requires_exact_ratio_keys():
    module = _load_module()
    packet = _timing_gate_packet()
    comparison = packet["benchmark"]["comparison"]
    value = comparison.pop("sequential_impulse_to_avbd_median_cpu_cost_ratio")
    comparison["sequential_impulse_to_avbd_median_cpu_cost_rati0"] = value

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("median ratio keys must be exactly" in error for error in errors), errors


def test_paper_benchmark_timing_gate_requires_comparison_object():
    module = _load_module()
    packet = _timing_gate_packet()
    del packet["benchmark"]["comparison"]

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(
        "benchmark.comparison must contain exactly" in error for error in errors
    ), errors


def test_paper_benchmark_timing_gate_requires_avbd_self_median():
    module = _load_module()
    packet = {
        "benchmark": {
            "timing": {"median_cpu_time_per_step_ns": 5.0},
        }
    }
    packet["benchmark"].pop("timing")

    errors = module._paper_benchmark_timing_errors(
        packet, "avbd-paper-breakable-wall-packet.json"
    )

    assert any("must embed a positive finite avbd" in error for error in errors), errors


def test_paper_benchmark_timing_gate_ignores_non_paper_packets():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["benchmark"]["comparison"][
        "sequential_impulse_to_avbd_median_cpu_cost_ratio"
    ] = 3.0
    errors = module._paper_benchmark_timing_errors(
        packet, "avbd-demo2d-ground-packet.json"
    )
    assert errors == []
