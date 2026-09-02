"""Tests for scripts/check_avbd_packets.py (PLAN-091 WP-091.1)."""

import hashlib
import importlib.util
import json
import struct
import sys
from fractions import Fraction
from pathlib import Path

import pytest

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
        "multibody_integration_family": "none",
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


def _repo_source_provenance():
    """Provenance over a real repository file, valid against the live REPO_ROOT.

    Every packet at the current schema version must name the source it was
    produced from, so fixtures that only exercise identity still need one.
    """
    return _source_provenance(ROOT, "scripts/avbd_packet_schema.py")


def _bound_visual(*roles: str):
    return {
        role: {
            "image_verdict": {
                "image_sha256": f"{index:x}" * 64,
            },
            "screenshot": {
                "sha256": f"{index:x}" * 64,
            },
        }
        for index, role in enumerate(roles, start=1)
    }


def _schema6_bound_figure13_visual(packet_name: str):
    scenes_and_frames = {
        "avbd-paper-breakable-wall-packet.json": (
            "avbd_paper_breakable_wall",
            {"impact": 60, "outcome": 120, "long_horizon": 600},
        ),
        "avbd-paper-vbd-comparison-packet.json": (
            "vbd_paper_breakable_wall",
            {"bend": 18, "retention": 120, "long_horizon": 600},
        ),
        "avbd-paper-sequential-impulse-comparison-packet.json": (
            "sequential_impulse_paper_breakable_wall",
            {"fracture": 14, "collapse": 120, "long_horizon": 600},
        ),
    }
    scene, frames = scenes_and_frames[packet_name]
    visual = _bound_visual(*frames)
    for role, frame in frames.items():
        visual[role]["image_verdict"]["metadata"] = {
            "frame": str(frame),
            "scene": scene,
        }
    return visual


def _packet_errors(module, path, *, context=None):
    if context is not None:
        return module.packet_errors(path, context=context)
    return module.packet_errors(path, packet_dir=path.parent)


def _disable_figure13_consistency(module, monkeypatch):
    monkeypatch.setattr(module, "_paper_figure13_consistency_errors", lambda *_args: [])
    monkeypatch.setattr(module, "_paper_link_summary_errors", lambda *_args: [])
    monkeypatch.setattr(module, "_paper_benchmark_timing_errors", lambda *_args: [])


def test_committed_packet_corpus_passes():
    module = _load_module()
    assert module.main([]) == 0


@pytest.mark.parametrize(
    "payload",
    (
        b'{"schema_version": ' + b"1" * 5000 + b"}\n",
        b"\xff\xfe\x00",
    ),
)
def test_packet_parser_failures_are_reported_as_invalid_json(tmp_path, payload):
    module = _load_module()
    path = tmp_path / "avbd-malformed-packet.json"
    path.write_bytes(payload)

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert any("invalid JSON" in error for error in errors), errors


@pytest.mark.parametrize(
    "payload",
    (
        b'{"schema_version": 5, "schema_version": 5}',
        b'{"schema_version": 5, "nested": {"key": 1, "key": 2}}',
        b'{"schema_version": 5, "value": NaN}',
        b'{"schema_version": 5, "value": Infinity}',
        b'{"schema_version": 5, "value": -Infinity}',
        b'{"schema_version": 5, "value": 1e1000000}',
    ),
)
def test_packet_parser_rejects_duplicate_keys_and_non_finite_numbers(tmp_path, payload):
    module = _load_module()
    path = tmp_path / "avbd-strict-json-packet.json"
    path.write_bytes(payload)

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert any("invalid JSON" in error for error in errors), errors


def test_new_version_packet_without_identity_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": module.AVBD_PACKET_SCHEMA_VERSION},
    )
    errors = _packet_errors(module, path)
    assert any("requires resolved_solver_identity" in error for error in errors)


def test_new_version_packet_with_identity_passes(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {
            "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
            "resolved_solver_identity": _identity(),
            "source_provenance": _repo_source_provenance(),
        },
    )
    assert _packet_errors(module, path) == []
    assert module.main(["--packet-dir", str(tmp_path), "--packet", str(path)]) == 0


def test_public_checker_accepts_truthful_heterogeneous_row_identities(tmp_path):
    module = _load_module()
    avbd = _identity(
        rigid_contact_solver="avbd",
        rigid_point_joint_solver="avbd",
        rigid_contact_selection="world_solver_family",
    )
    variational = _identity(
        rigid_point_joint_solver="none",
        multibody_integration_family="variational",
    )
    packet = {
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": {
            "identity_scope": "per_benchmark_row",
            "recorded_from": "benchmark runtime counters",
        },
        "benchmark": {
            "rows": [
                {"run_name": "BM_Rigid/1", "resolved_solver_identity": avbd},
                {
                    "run_name": "BM_Articulated/1",
                    "resolved_solver_identity": variational,
                },
            ]
        },
        "source_provenance": _repo_source_provenance(),
    }
    path = _write_packet(tmp_path, "avbd-mixed-packet.json", packet)

    assert _packet_errors(module, path) == []


def test_public_checker_rejects_heterogeneous_marker_without_every_row_identity(
    tmp_path,
):
    module = _load_module()
    packet = {
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": {
            "identity_scope": "per_benchmark_row",
            "recorded_from": "benchmark runtime counters",
        },
        "benchmark": {
            "rows": [
                {
                    "run_name": "BM_First/1",
                    "resolved_solver_identity": _identity(),
                },
                {"run_name": "BM_Second/1", "benchmark": "identity omitted"},
            ]
        },
    }
    path = _write_packet(tmp_path, "avbd-mixed-packet.json", packet)

    errors = _packet_errors(module, path)

    assert any(
        "benchmark.rows[1].resolved_solver_identity" in error for error in errors
    )


def test_current_packet_validates_plan104_claims(tmp_path):
    module = _load_module()
    claim_id = "avbd.method.cpu_solver"
    packet = {
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": _identity(),
        "target": {"contract_rows": [claim_id]},
        "plan104_claims": {
            claim_id: {
                "status": "complete",
                "predicate_results": {
                    "artifact_valid": True,
                    "solver_contract_valid": True,
                    "physical_outcome_valid": True,
                    "performance_comparable": True,
                    "claim_valid": True,
                },
                "backend_results": {"cpu": True, "cuda": False},
            }
        },
    }
    path = _write_packet(tmp_path, "avbd-new-scene-packet.json", packet)

    errors = _packet_errors(module, path)

    assert any("backend_results must be true for: cuda" in error for error in errors)


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
    errors = _packet_errors(module, path)
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
            "source_provenance": _repo_source_provenance(),
        },
    )
    assert _packet_errors(module, path) == []


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
    errors = _packet_errors(module, path)
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
    errors = _packet_errors(module, path)
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
    assert _packet_errors(module, path) == []

    source.write_text("second\n")
    errors = _packet_errors(module, path)
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
    errors = _packet_errors(module, path)
    assert any("source_provenance.digest" in error for error in errors)


@pytest.mark.parametrize(
    ("target_kind", "message"),
    (
        ("final_symlink", "cannot be a symbolic link"),
        ("parent_symlink_escape", "resolves outside the repository"),
        ("directory", "must be a regular file"),
    ),
)
def test_source_provenance_path_confinement(
    tmp_path, monkeypatch, target_kind, message
):
    module = _load_module()
    repository = tmp_path / "repository"
    repository.mkdir()
    monkeypatch.setattr(module, "REPO_ROOT", repository)
    if target_kind == "final_symlink":
        target = repository / "target.cpp"
        target.write_text("target\n")
        source = repository / "solver.cpp"
        source.symlink_to(target.name)
        relative = "solver.cpp"
    elif target_kind == "parent_symlink_escape":
        outside = tmp_path / "outside"
        outside.mkdir()
        (outside / "solver.cpp").write_text("outside\n")
        (repository / "sources").symlink_to(outside, target_is_directory=True)
        relative = "sources/solver.cpp"
    else:
        (repository / "solver.cpp").mkdir()
        relative = "solver.cpp"
    packet = {
        "source_provenance": {
            "algorithm": module.SOURCE_PROVENANCE_ALGORITHM,
            "digest": "0" * 64,
            "files": [{"path": relative, "sha256": "0" * 64}],
        }
    }

    errors = module._source_provenance_errors(packet, "avbd-test-packet.json")

    assert any(message in error for error in errors), errors


def test_source_provenance_read_error_fails_closed(tmp_path, monkeypatch):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    source = tmp_path / "solver.cpp"
    source.write_text("stable\n")
    packet = {"source_provenance": _source_provenance(tmp_path, "solver.cpp")}
    real_read_bytes = Path.read_bytes

    def failing_read_bytes(path):
        if path.resolve() == source.resolve():
            raise PermissionError("denied")
        return real_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", failing_read_bytes)

    errors = module._source_provenance_errors(packet, "avbd-test-packet.json")

    assert any("cannot be read" in error and "denied" in error for error in errors)


@pytest.mark.parametrize("unsafe", ("bad\x00path", "bad\ud800path"))
def test_json_artifact_paths_reject_nul_and_unencodable_surrogates(
    tmp_path, monkeypatch, unsafe
):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    packet = {
        "source_provenance": {
            "algorithm": module.SOURCE_PROVENANCE_ALGORITHM,
            "digest": "0" * 64,
            "files": [{"path": unsafe, "sha256": "0" * 64}],
        }
    }

    errors = module._source_provenance_errors(packet, "avbd-test-packet.json")

    assert any("safe repository-relative path" in error for error in errors), errors
    assert module._safe_relative_path(unsafe) is None


@pytest.mark.parametrize("unsafe", ("avbd-bad\x00-packet.json", "avbd-bad\ud800.json"))
def test_public_packet_validation_returns_error_for_unencodable_path(tmp_path, unsafe):
    module = _load_module()
    path = tmp_path / unsafe

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert errors
    assert any("packet target" in error for error in errors), errors
    assert module.main(["--packet-dir", str(tmp_path), "--packet", str(path)]) == 1


def test_public_packet_validation_catches_json_numeric_overflow(tmp_path, monkeypatch):
    module = _load_module()
    path = _write_packet(
        tmp_path, "avbd-overflow-packet.json", _current_packet("overflow")
    )

    def raise_overflow(_payload):
        raise OverflowError("numeric conversion overflow")

    monkeypatch.setattr(module, "_strict_json_loads", raise_overflow)

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert any("invalid JSON" in error for error in errors), errors


@pytest.mark.parametrize(
    "exception", (ValueError("bad path"), UnicodeError("bad text"))
)
def test_public_packet_validation_catches_path_resolution_errors(
    tmp_path, monkeypatch, exception
):
    module = _load_module()
    path = _write_packet(
        tmp_path, "avbd-path-error-packet.json", _current_packet("path_error")
    )
    context = module.PacketValidationContext(packet_dir=tmp_path)
    real_resolve = Path.resolve

    def failing_resolve(candidate, *args, **kwargs):
        if candidate == path:
            raise exception
        return real_resolve(candidate, *args, **kwargs)

    monkeypatch.setattr(Path, "resolve", failing_resolve)

    errors = module.packet_errors(path, context=context)

    assert any("packet target cannot be resolved" in error for error in errors), errors


@pytest.mark.parametrize(
    "exception", (ValueError("bad path"), UnicodeError("bad text"))
)
def test_capture_source_validation_catches_path_exceptions(monkeypatch, exception):
    module = _load_module()

    def failing_capture_source(_root):
        raise exception

    monkeypatch.setattr(
        module, "compute_capture_source_provenance", failing_capture_source
    )

    errors = module._paper_capture_source_binding_errors(
        {}, "avbd-paper-breakable-wall-packet.json"
    )

    assert any(
        "cannot resolve current capture source provenance" in error for error in errors
    )


@pytest.mark.parametrize("unsafe", ("bad\x00.json", "bad\ud800.json"))
def test_linked_packet_paths_reject_nul_and_surrogates(tmp_path, unsafe):
    module = _load_module()
    parent = _write_packet(
        tmp_path,
        "avbd-parent-packet.json",
        _current_packet(
            "parent",
            linked_avbd_evidence={"file": unsafe, "sha256": "0" * 64},
        ),
    )

    errors = module.packet_errors(parent, packet_dir=tmp_path)

    assert any("must name one sibling packet file" in error for error in errors)


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


@pytest.mark.parametrize(
    ("packet_name", "roles"),
    tuple(
        (name, roles)
        for name, roles in (
            (
                "avbd-paper-breakable-wall-packet.json",
                ("impact", "outcome"),
            ),
            (
                "avbd-paper-vbd-comparison-packet.json",
                ("bend", "retention"),
            ),
            (
                "avbd-paper-sequential-impulse-comparison-packet.json",
                ("fracture", "collapse"),
            ),
        )
    ),
)
def test_paper_image_verdict_binding_accepts_every_figure13_capture(
    packet_name,
    roles,
):
    module = _load_module()
    packet = {"visual_evidence": _bound_visual(*roles)}

    assert module._paper_image_verdict_binding_errors(packet, packet_name) == []


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("tampered", "must match"),
        ("missing", "must be a lowercase SHA-256 digest"),
        ("uppercase", "must be a lowercase SHA-256 digest"),
    ),
)
def test_paper_image_verdict_binding_rejects_tamper_and_omission(
    mutation,
    message,
):
    module = _load_module()
    packet_name = "avbd-paper-breakable-wall-packet.json"
    packet = {"visual_evidence": _bound_visual("impact", "outcome")}
    image_verdict = packet["visual_evidence"]["impact"]["image_verdict"]
    if mutation == "tampered":
        image_verdict["image_sha256"] = "f" * 64
    elif mutation == "missing":
        del image_verdict["image_sha256"]
    else:
        image_verdict["image_sha256"] = "A" * 64

    errors = module._paper_image_verdict_binding_errors(packet, packet_name)

    assert any(message in error for error in errors), errors


def test_paper_packet_rejects_capture_and_benchmark_source_drift(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    path, packet = _complete_paper_packet(module, tmp_path, monkeypatch)
    current = dict(packet["visual_evidence"]["impact"]["source_provenance"])
    assert _packet_errors(module, path) == []

    source_provenance = packet.pop("source_provenance")
    path.write_text(json.dumps(packet))
    errors = _packet_errors(module, path)
    assert any(
        "source_provenance must be an object for a current paper packet" in error
        for error in errors
    )

    packet["source_provenance"] = source_provenance
    packet["visual_evidence"]["impact"]["source_provenance"]["digest"] = "0" * 64
    path.write_text(json.dumps(packet))
    errors = _packet_errors(module, path)
    assert any(
        "visual_evidence.impact.source_provenance.digest" in error for error in errors
    )

    packet["visual_evidence"]["impact"]["source_provenance"] = dict(current)
    packet["benchmark"]["source_provenance"]["benchmark_source_sha256"] = "0" * 64
    path.write_text(json.dumps(packet))
    errors = _packet_errors(module, path)
    assert any("benchmark source hash" in error for error in errors)


def test_linked_packet_validation_reaches_transitive_avbd_source(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    _disable_figure13_consistency(module, monkeypatch)
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        module,
        "_paper_capture_source_binding_errors",
        lambda _packet, _name: [],
    )
    monkeypatch.setattr(
        module,
        "_paper_image_verdict_binding_errors",
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
    assert _packet_errors(module, si) == []

    (tmp_path / "solver.cpp").write_text("drifted\n")
    errors = _packet_errors(module, si)
    assert any("sha256 drifted for solver.cpp" in error for error in errors)


def test_linked_packet_validation_reaches_nested_image_verdict_binding(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    _disable_figure13_consistency(module, monkeypatch)
    monkeypatch.setattr(module, "_source_provenance_errors", lambda *_args: [])
    monkeypatch.setattr(
        module, "_paper_capture_source_binding_errors", lambda *_args: []
    )

    avbd = _write_packet(
        tmp_path,
        "avbd-paper-breakable-wall-packet.json",
        _current_packet(
            "avbd_paper_breakable_wall",
            resolved_solver_identity=_identity(
                rigid_contact_solver="avbd",
                rigid_point_joint_solver="avbd",
                rigid_contact_selection="world_solver_family",
            ),
            visual_evidence=_schema6_bound_figure13_visual(
                "avbd-paper-breakable-wall-packet.json"
            ),
        ),
    )
    vbd = _write_packet(
        tmp_path,
        "avbd-paper-vbd-comparison-packet.json",
        _current_packet(
            "avbd_paper_vbd_comparison",
            resolved_solver_identity=_identity(
                rigid_contact_solver="vbd",
                rigid_point_joint_solver="vbd",
                rigid_contact_selection="world_solver_family",
            ),
            linked_avbd_evidence={
                "file": avbd.name,
                "sha256": _file_sha256(avbd),
            },
            visual_evidence=_schema6_bound_figure13_visual(
                "avbd-paper-vbd-comparison-packet.json"
            ),
        ),
    )
    si = _write_packet(
        tmp_path,
        "avbd-paper-sequential-impulse-comparison-packet.json",
        _current_packet(
            "avbd_paper_sequential_impulse_comparison",
            linked_avbd_vbd_evidence={
                "file": vbd.name,
                "sha256": _file_sha256(vbd),
            },
            visual_evidence=_schema6_bound_figure13_visual(
                "avbd-paper-sequential-impulse-comparison-packet.json"
            ),
        ),
    )
    assert _packet_errors(module, si) == []

    avbd_packet = json.loads(avbd.read_text())
    avbd_packet["visual_evidence"]["impact"]["image_verdict"]["image_sha256"] = "f" * 64
    avbd.write_text(json.dumps(avbd_packet))
    vbd_packet = json.loads(vbd.read_text())
    vbd_packet["linked_avbd_evidence"]["sha256"] = _file_sha256(avbd)
    vbd.write_text(json.dumps(vbd_packet))
    si_packet = json.loads(si.read_text())
    si_packet["linked_avbd_vbd_evidence"]["sha256"] = _file_sha256(vbd)
    si.write_text(json.dumps(si_packet))

    errors = _packet_errors(module, si)

    assert any(
        "visual_evidence.impact.image_verdict.image_sha256 must match" in error
        for error in errors
    ), errors


def _current_packet(packet_id, **extra):
    packet = {
        "schema_version": 6,
        "packet": packet_id,
        "resolved_solver_identity": _identity(),
    }
    packet.update(extra)
    return packet


def test_shared_context_loads_real_chain_once_and_caches_invalid_child(
    tmp_path, monkeypatch
):
    module = _load_module()
    child = _write_packet(
        tmp_path,
        "avbd-chain-child-packet.json",
        _current_packet(
            "chain_child",
            resolved_solver_identity=_identity(
                rigid_contact_solver="invalid_child_solver"
            ),
        ),
    )
    middle = _write_packet(
        tmp_path,
        "avbd-chain-middle-packet.json",
        _current_packet(
            "chain_middle",
            linked_avbd_evidence={
                "file": child.name,
                "sha256": _file_sha256(child),
            },
        ),
    )
    top = _write_packet(
        tmp_path,
        "avbd-chain-top-packet.json",
        _current_packet(
            "chain_top",
            linked_avbd_vbd_evidence={
                "file": middle.name,
                "sha256": _file_sha256(middle),
            },
        ),
    )
    real_loads = module._strict_json_loads
    loads = {}

    def counting_loads(payload):
        packet = real_loads(payload)
        packet_id = packet.get("packet") if isinstance(packet, dict) else None
        if packet_id is not None:
            loads[packet_id] = loads.get(packet_id, 0) + 1
        return packet

    monkeypatch.setattr(module, "_strict_json_loads", counting_loads)
    context = module.PacketValidationContext(packet_dir=tmp_path)

    top_errors = _packet_errors(module, top, context=context)
    child_errors = _packet_errors(module, child, context=context)
    repeated_top_errors = _packet_errors(module, top, context=context)

    assert any("invalid_child_solver" in error for error in top_errors), top_errors
    assert any("invalid_child_solver" in error for error in child_errors), child_errors
    assert repeated_top_errors == top_errors
    assert loads == {"chain_top": 1, "chain_middle": 1, "chain_child": 1}


def test_linked_packet_cycle_fails_closed(tmp_path):
    module = _load_module()
    first = _write_packet(
        tmp_path,
        "avbd-cycle-first-packet.json",
        _current_packet(
            "cycle_first",
            linked_avbd_evidence={
                "file": "avbd-cycle-second-packet.json",
                "sha256": "0" * 64,
            },
        ),
    )
    second = _write_packet(
        tmp_path,
        "avbd-cycle-second-packet.json",
        _current_packet(
            "cycle_second",
            linked_avbd_evidence={
                "file": first.name,
                "sha256": _file_sha256(first),
            },
        ),
    )
    packet = json.loads(first.read_text())
    packet["linked_avbd_evidence"]["sha256"] = _file_sha256(second)
    first.write_text(json.dumps(packet))

    errors = module.packet_errors(first, packet_dir=tmp_path)

    assert any("linked packet cycle detected" in error for error in errors), errors


def test_committed_si_vbd_avbd_chain_loads_each_packet_once(monkeypatch):
    module = _load_module()
    _disable_figure13_consistency(module, monkeypatch)
    monkeypatch.setattr(module, "_source_provenance_errors", lambda *_args: [])
    monkeypatch.setattr(
        module, "_paper_capture_source_binding_errors", lambda *_args: []
    )
    monkeypatch.setattr(
        module, "_paper_image_verdict_binding_errors", lambda *_args: []
    )
    avbd = PACKET_DIR / "avbd-paper-breakable-wall-packet.json"
    vbd = PACKET_DIR / "avbd-paper-vbd-comparison-packet.json"
    si = PACKET_DIR / "avbd-paper-sequential-impulse-comparison-packet.json"
    packet_ids = {json.loads(path.read_text())["packet"] for path in (avbd, vbd, si)}
    real_loads = module._strict_json_loads
    loads = {}

    def counting_loads(payload):
        packet = real_loads(payload)
        packet_id = packet.get("packet") if isinstance(packet, dict) else None
        if packet_id in packet_ids:
            loads[packet_id] = loads.get(packet_id, 0) + 1
        return packet

    monkeypatch.setattr(module, "_strict_json_loads", counting_loads)
    context = module.PacketValidationContext(packet_dir=PACKET_DIR)

    assert _packet_errors(module, si, context=context) == []
    assert _packet_errors(module, vbd, context=context) == []
    assert _packet_errors(module, avbd, context=context) == []
    assert loads == {packet_id: 1 for packet_id in packet_ids}


def test_direct_packet_symlink_escape_is_rejected(tmp_path):
    module = _load_module()
    packet_dir = tmp_path / "packets"
    packet_dir.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    target = _write_packet(outside, "avbd-target-packet.json", _current_packet("x"))
    link = packet_dir / "avbd-link-packet.json"
    link.symlink_to(target)

    errors = module.packet_errors(link, packet_dir=packet_dir)

    assert any("cannot be a symbolic link" in error for error in errors), errors


def test_direct_packet_in_directory_symlink_is_rejected(tmp_path):
    module = _load_module()
    target = _write_packet(
        tmp_path, "avbd-target-packet.json", _current_packet("target")
    )
    link = tmp_path / "avbd-link-packet.json"
    link.symlink_to(target.name)

    errors = module.packet_errors(link, packet_dir=tmp_path)

    assert any("cannot be a symbolic link" in error for error in errors), errors


def test_direct_packet_nested_below_packet_directory_is_rejected(tmp_path):
    module = _load_module()
    nested = tmp_path / "nested"
    nested.mkdir()
    path = _write_packet(nested, "avbd-nested-packet.json", _current_packet("nested"))

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert any("must resolve directly under" in error for error in errors), errors


def test_direct_packet_directory_is_rejected(tmp_path):
    module = _load_module()
    path = tmp_path / "avbd-directory-packet.json"
    path.mkdir()

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert any("must be a regular file" in error for error in errors), errors


@pytest.mark.parametrize(
    "target_kind", ("symlink_escape", "symlink_inside", "directory")
)
def test_linked_packet_target_confinement(tmp_path, target_kind):
    module = _load_module()
    packet_dir = tmp_path / "packets"
    packet_dir.mkdir()
    linked = packet_dir / "avbd-linked-packet.json"
    if target_kind == "symlink_escape":
        outside = tmp_path / "outside"
        outside.mkdir()
        target = _write_packet(
            outside, "avbd-target-packet.json", _current_packet("target")
        )
        linked.symlink_to(target)
    elif target_kind == "symlink_inside":
        target = _write_packet(
            packet_dir, "avbd-target-packet.json", _current_packet("target")
        )
        linked.symlink_to(target.name)
    else:
        linked.mkdir()
    parent = _write_packet(
        packet_dir,
        "avbd-parent-packet.json",
        _current_packet(
            "parent",
            linked_avbd_evidence={"file": linked.name, "sha256": "0" * 64},
        ),
    )

    errors = module.packet_errors(parent, packet_dir=packet_dir)

    expected = (
        "must resolve directly under"
        if target_kind.startswith("symlink")
        else "must be a regular file"
    )
    if target_kind.startswith("symlink"):
        expected = "cannot be a symbolic link"
    assert any(expected in error for error in errors), errors


def test_linked_packet_hash_read_error_fails_closed(tmp_path, monkeypatch):
    module = _load_module()
    child = _write_packet(tmp_path, "avbd-child-packet.json", _current_packet("child"))
    parent = _write_packet(
        tmp_path,
        "avbd-parent-packet.json",
        _current_packet(
            "parent",
            linked_avbd_evidence={
                "file": child.name,
                "sha256": _file_sha256(child),
            },
        ),
    )
    real_read_bytes = Path.read_bytes

    def failing_read_bytes(path):
        if path.resolve() == child.resolve():
            raise PermissionError("denied")
        return real_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", failing_read_bytes)

    errors = module.packet_errors(parent, packet_dir=tmp_path)

    assert any("cannot be read" in error and "denied" in error for error in errors)


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
    errors = _packet_errors(module, path)
    assert any("rigid_contact_solver" in error for error in errors)


def test_new_packet_file_below_current_version_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": 1},
    )
    errors = _packet_errors(module, path)
    assert any("must be written at schema_version" in error for error in errors)


def test_new_schema_v4_packet_is_rejected_outside_legacy_allowlist(
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

    errors = _packet_errors(module, path)

    assert any(
        f"schema_version {module.AVBD_PACKET_SCHEMA_VERSION}" in error
        for error in errors
    ), errors


def _legacy_non_evidence_boundary(module, name):
    return {
        "artifact_status": "legacy_unbound",
        "avbd_performance_claim_supported": False,
        "avbd_solver_evidence": False,
        "current_build_bound": False,
        "historical_identifiers_retained": True,
        "historical_measurements_preserved": True,
        "measurement_runtime_identity_recorded": False,
        "plan104_avbd_row_closure_supported": False,
        "reason": "Historical packet is not bound to the current runtime.",
        "supported_scope": module.LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES[name],
    }


def test_legacy_allowlisted_identity_free_skeleton_is_rejected(tmp_path):
    module = _load_module()
    name = "avbd-demo2d-pyramid-packet.json"
    assert name in module.LEGACY_IDENTITY_EXEMPT_PACKETS
    path = _write_packet(tmp_path, name, {"schema_version": 1})

    errors = _packet_errors(module, path)

    assert any("requires an evidence_boundary object" in error for error in errors)


def test_legacy_allowlisted_packet_with_explicit_boundary_stays_readable(tmp_path):
    module = _load_module()
    name = "avbd-demo2d-pyramid-packet.json"
    path = _write_packet(
        tmp_path,
        name,
        {
            "schema_version": 1,
            "evidence_boundary": _legacy_non_evidence_boundary(module, name),
        },
    )
    assert _packet_errors(module, path) == []


@pytest.mark.parametrize(
    ("field", "replacement"),
    (
        ("current_build_bound", True),
        ("avbd_solver_evidence", True),
        ("plan104_avbd_row_closure_supported", True),
        ("measurement_runtime_identity_recorded", True),
        ("supported_scope", "current_avbd_performance"),
        ("reason", ""),
    ),
)
def test_legacy_non_evidence_boundary_mutations_fail_closed(
    tmp_path, field, replacement
):
    module = _load_module()
    name = "avbd-demo2d-pyramid-packet.json"
    boundary = _legacy_non_evidence_boundary(module, name)
    boundary[field] = replacement
    path = _write_packet(
        tmp_path,
        name,
        {"schema_version": 1, "evidence_boundary": boundary},
    )

    errors = _packet_errors(module, path)

    assert any(f"evidence_boundary.{field}" in error for error in errors), errors


def test_misclassified_schema_v3_packet_requires_non_evidence_boundary(tmp_path):
    module = _load_module()
    name = "avbd-breakable-motor-scale-packet.json"
    path = _write_packet(
        tmp_path,
        name,
        {
            "schema_version": 3,
            "resolved_solver_identity": {
                "avbd_rigid_contact_config_emplaced": False,
                "recorded_from": "historical hand-authored identity",
                "rigid_contact_selection": "not_applicable",
                "rigid_contact_solver": "none",
                "rigid_point_joint_solver": "avbd",
            },
        },
    )

    errors = _packet_errors(module, path)

    assert any("requires an evidence_boundary object" in error for error in errors)


def test_legacy_v3_allowlisted_packet_rejects_schema_downgrade(tmp_path):
    module = _load_module()
    name = "avbd-breakable-joint-scale-packet.json"
    assert module.LEGACY_PACKET_SCHEMA_VERSIONS[name] == 3
    path = _write_packet(tmp_path, name, {"schema_version": 1})

    errors = _packet_errors(module, path)

    assert any(
        "legacy allowlist requires schema_version 3, got 1" in error for error in errors
    ), errors


def test_legacy_v5_allowlisted_packet_stays_readable(tmp_path, monkeypatch):
    module = _load_module()
    name = "avbd-paper-breakable-wall-packet.json"
    assert module.LEGACY_PACKET_SCHEMA_VERSIONS[name] == 5
    monkeypatch.setattr(module, "_paper_figure13_consistency_errors", lambda *_args: [])
    monkeypatch.setattr(module, "_source_provenance_errors", lambda *_args: [])
    monkeypatch.setattr(
        module, "_paper_capture_source_binding_errors", lambda *_args: []
    )
    monkeypatch.setattr(
        module, "_paper_image_verdict_binding_errors", lambda *_args: []
    )
    monkeypatch.setattr(module, "_paper_benchmark_timing_errors", lambda *_args: [])
    path = _write_packet(
        tmp_path,
        name,
        {
            "schema_version": 5,
            "resolved_solver_identity": _identity(
                rigid_contact_solver="avbd",
                rigid_point_joint_solver="avbd",
                rigid_contact_selection="world_solver_family",
            ),
        },
    )

    assert _packet_errors(module, path) == []


def test_legacy_figure13_version_cannot_bypass_filename_consistency():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["schema_version"] = 5
    packet["resolved_solver_identity"]["rigid_contact_solver"] = "avbd"

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("Figure 13 filename" in error for error in errors), errors


@pytest.mark.parametrize(
    "name",
    (
        "avbd-demo2d-pyramid-packet.json",
        "avbd-breakable-joint-scale-packet.json",
        "avbd-paper-breakable-wall-packet.json",
    ),
)
def test_legacy_filename_migrated_to_current_schema_is_accepted(
    tmp_path, monkeypatch, name
):
    module = _load_module()
    _disable_figure13_consistency(module, monkeypatch)
    assert (
        module.LEGACY_PACKET_SCHEMA_VERSIONS[name] < module.AVBD_PACKET_SCHEMA_VERSION
    )
    monkeypatch.setattr(module, "_source_provenance_errors", lambda *_args: [])
    monkeypatch.setattr(
        module, "_paper_capture_source_binding_errors", lambda *_args: []
    )
    monkeypatch.setattr(
        module, "_paper_image_verdict_binding_errors", lambda *_args: []
    )
    path = _write_packet(
        tmp_path,
        name,
        _current_packet(f"migrated_{name.removesuffix('-packet.json')}"),
    )

    assert _packet_errors(module, path) == []


def test_non_integer_schema_version_is_rejected(tmp_path):
    module = _load_module()
    path = _write_packet(
        tmp_path,
        "avbd-new-scene-packet.json",
        {"schema_version": "two"},
    )
    errors = _packet_errors(module, path)
    assert any("schema_version must be an integer" in error for error in errors)


def _historical_boundary(module, name):
    spec = module.HISTORICAL_HIGH_RATIO_BOUNDARIES[name]
    boundary = {
        "current_build_bound": False,
        "measurement_runtime_identity_recorded": False,
        "plan104_avbd_row_closure_supported": False,
        "avbd_solver_evidence": False,
        "avbd_performance_claim_supported": False,
        "supported_scope": spec["supported_scope"],
    }
    if spec["visual_boundary"]:
        boundary.update(
            {
                "capture_artifacts_accessible": False,
                "semantic_visual_review_recorded": False,
            }
        )
    return boundary


def test_historical_high_ratio_scope_names_bind_variational_multibody_evidence():
    module = _load_module()

    assert module.HISTORICAL_HIGH_RATIO_BOUNDARIES[
        "avbd-articulated-high-ratio-chain-packet.json"
    ]["supported_scope"] == (
        "historical_variational_multibody_capture_hash_and_cpu_metadata"
    )
    assert module.HISTORICAL_HIGH_RATIO_BOUNDARIES[
        "avbd-paper-scale-high-ratio-chain-packet.json"
    ]["supported_scope"] == (
        "historical_variational_multibody_capture_hash_and_cpu_metadata"
    )
    assert module.HISTORICAL_HIGH_RATIO_BOUNDARIES[
        "avbd-paper-scale-high-ratio-iteration-sweep-packet.json"
    ]["supported_scope"] == ("historical_variational_multibody_cpu_metadata_and_plot")


@pytest.mark.parametrize(
    "name",
    (
        "avbd-articulated-high-ratio-chain-packet.json",
        "avbd-paper-scale-high-ratio-chain-packet.json",
        "avbd-paper-scale-high-ratio-iteration-sweep-packet.json",
    ),
)
def test_historical_high_ratio_boundary_accepts_exact_claim_limits(name):
    module = _load_module()
    packet = {"evidence_boundary": _historical_boundary(module, name)}

    assert module._historical_high_ratio_boundary_errors(packet, name) == []


@pytest.mark.parametrize(
    ("name", "field", "replacement"),
    (
        (
            "avbd-articulated-high-ratio-chain-packet.json",
            "current_build_bound",
            True,
        ),
        (
            "avbd-articulated-high-ratio-chain-packet.json",
            "capture_artifacts_accessible",
            None,
        ),
        (
            "avbd-paper-scale-high-ratio-chain-packet.json",
            "semantic_visual_review_recorded",
            True,
        ),
        (
            "avbd-paper-scale-high-ratio-iteration-sweep-packet.json",
            "measurement_runtime_identity_recorded",
            True,
        ),
        (
            "avbd-paper-scale-high-ratio-iteration-sweep-packet.json",
            "supported_scope",
            "visual_evidence",
        ),
        (
            "avbd-paper-scale-high-ratio-iteration-sweep-packet.json",
            "avbd_performance_claim_supported",
            None,
        ),
    ),
)
def test_historical_high_ratio_boundary_rejects_missing_true_or_drifted_fields(
    name, field, replacement
):
    module = _load_module()
    boundary = _historical_boundary(module, name)
    if replacement is None:
        del boundary[field]
    else:
        boundary[field] = replacement

    errors = module._historical_high_ratio_boundary_errors(
        {"evidence_boundary": boundary}, name
    )

    assert any(f"evidence_boundary.{field}" in error for error in errors), errors


def test_legacy_allowlist_and_current_packets_cover_committed_corpus():
    module = _load_module()
    for path in PACKET_DIR.glob("avbd-*-packet.json"):
        packet = json.loads(path.read_text())
        legacy_version = module.LEGACY_PACKET_SCHEMA_VERSIONS.get(path.name)
        if legacy_version is None:
            assert packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION
        else:
            assert packet["schema_version"] in (
                legacy_version,
                module.AVBD_PACKET_SCHEMA_VERSION,
            )
        if packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION:
            assert "resolved_solver_identity" in packet
        assert _packet_errors(module, path) == []


_TIMING_GATE_PACKET_NAME = "avbd-paper-vbd-comparison-packet.json"


def _timing_gate_packet():
    return json.loads((PACKET_DIR / _TIMING_GATE_PACKET_NAME).read_text())


def _schema6_timing_gate_packet(module):
    packet = _with_current_outcome_oracles(
        module, _timing_gate_packet(), _TIMING_GATE_PACKET_NAME
    )
    packet["schema_version"] = module.AVBD_PACKET_SCHEMA_VERSION
    packet["resolved_solver_identity"]["multibody_integration_family"] = "none"
    configuration_fingerprints = {
        "avbd": int("fedcba9876543210", 16),
        "vbd": int("1020304050607080", 16),
    }
    for solver, entry in packet["benchmark"]["methods"].items():
        configuration = configuration_fingerprints[solver]
        entry["solver_configuration_fingerprint"] = f"{configuration:016x}"
        representative = {
            "contact_method_sequential_impulse": 1.0,
            "effective_scene_contract_passed": 1.0,
            "effective_scene_mutation_audit_passed": 1.0,
            "rigid_avbd_alpha": 0.95 if solver == "avbd" else 0.0,
            "rigid_avbd_beta": 10.0 if solver == "avbd" else 0.0,
            "rigid_avbd_gamma": 0.99 if solver == "avbd" else 0.0,
            "rigid_avbd_parameter_profile_paper_2025": (
                1.0 if solver == "avbd" else 0.0
            ),
            "runtime_identity_recorded": 1.0,
            "runtime_identity_applicable": 1.0,
            "runtime_identity_not_applicable": 0.0,
            "runtime_identity_public_avbd_rigid": (1.0 if solver == "avbd" else 0.0),
            "runtime_identity_variational_multibody": 0.0,
            "runtime_identity_contract_passed": 1.0,
            "scene_spec_matches_python": 1.0,
            "solver_projection_policies_match": 1.0,
            "solver_configuration_fingerprint_hi": float(configuration >> 32),
            "solver_configuration_fingerprint_lo": float(configuration & 0xFFFFFFFF),
        }
        stable_keys = {
            *module._PAPER_SCENE_COUNTERS,
            f"public_{solver}_family",
            f"resolved_rigid_body_{solver}",
            "resolved_rigid_constraint_iterations",
            f"resolved_rigid_contact_{solver}",
            f"resolved_rigid_pair_constraint_{solver}",
            "rigid_constraint_iterations",
            "runtime_contract_passed",
            "trajectory_frames",
            "scene_spec_fingerprint_hi",
            "scene_spec_fingerprint_lo",
            *representative,
        }
        for row in entry["rows"]:
            aggregate = row["aggregate_name"]
            if aggregate in ("mean", "median"):
                row.update(representative)
            elif aggregate == "stddev":
                row.update({key: 0.0 for key in stable_keys})
    return packet


def _synthetic_capture_loader_environment(module):
    """Mirror the attestation `_capture_loader_environment_policy` records."""
    return {
        "algorithm": module.CAPTURE_LOADER_POLICY_ALGORITHM,
        "forbidden_environment_prefixes": list(
            module.CAPTURE_LOADER_ENVIRONMENT_PREFIXES
        ),
        "passed": True,
        "present_environment_variables": [],
        "trusted_image_roots": ["/evidence", "/usr/lib"],
    }


def _synthetic_capture_runtime_image_inventory(module, *, extension, libraries):
    """Mirror the inventory `_capture_runtime_image_inventory` records."""
    images = sorted(
        (
            {key: binary[key] for key in ("file", "path", "sha256", "size_bytes")}
            for binary in (extension, *libraries)
        ),
        key=lambda image: image["path"],
    )
    payload = {
        "images": images,
        "required_images": {"dartpy_native_extension": extension["path"]},
    }
    return {
        "algorithm": module.CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": module._canonical_json_digest(payload),
    }


def _synthetic_capture_runtime_provenance(module):
    source_digest = "1" * 64
    source_git_head = "2" * 40
    build_configuration_digest = "5" * 64
    build_payload = {
        "build_configuration_digest": build_configuration_digest,
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15.2.0",
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": source_git_head,
        "source_provenance_digest": source_digest,
    }
    library_identity = {
        "algorithm": module.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        "digest": module._canonical_json_digest(build_payload),
        **build_payload,
    }
    library = {
        "build_identity": library_identity,
        "file": "libdart-simulation.so",
        "path": "/evidence/build/lib/libdart-simulation.so",
        "sha256": "3" * 64,
        "size_bytes": 4096,
    }
    extension = {
        "build_configuration_digest": build_configuration_digest,
        "file": "_dartpy.so",
        "module": "dartpy._dartpy",
        "path": "/evidence/build/python/dartpy/_dartpy.so",
        "sha256": "4" * 64,
        "size_bytes": 8192,
        "source_git_head": source_git_head,
        "source_provenance_digest": source_digest,
    }
    provenance = {
        "algorithm": module.CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
        "dart_library_linkage": "shared",
        "digest": "",
        "loaded_dart_libraries": [library],
        "loader_environment": _synthetic_capture_loader_environment(module),
        "native_extension": extension,
        "runtime_image_inventory": _synthetic_capture_runtime_image_inventory(
            module, extension=extension, libraries=[library]
        ),
        "source_git_head": source_git_head,
        "source_provenance_digest": source_digest,
    }
    provenance["digest"] = module._capture_runtime_manifest_digest(provenance)
    return provenance


def _synthetic_build_configuration(module):
    values = {key: "<UNDEFINED>" for key in module.BUILD_CONFIGURATION_KEYS}
    for definition in module.EVIDENCE_CMAKE_DEFINITIONS:
        name, value = definition.split("=", maxsplit=1)
        if name in values:
            values[name] = value
    values.update(
        {
            "CMAKE_CXX_COMPILER": "/usr/bin/c++",
            "CMAKE_CXX_COMPILER_ID": "GNU",
            "CMAKE_CXX_COMPILER_VERSION": "15.2.0",
            "CMAKE_GENERATOR": "Ninja",
            "CMAKE_SYSTEM_NAME": "Linux",
            "CMAKE_SYSTEM_PROCESSOR": "x86_64",
        }
    )
    record = "".join(
        [f"algorithm={module.BUILD_CONFIGURATION_ALGORITHM}\n"]
        + [f"{key}={values[key]}\n" for key in module.BUILD_CONFIGURATION_KEYS]
    )
    return {
        "algorithm": module.BUILD_CONFIGURATION_ALGORITHM,
        "digest": hashlib.sha256(record.encode("utf-8")).hexdigest(),
        "values": values,
    }


def _synthetic_benchmark_build_identity(module):
    build_configuration = _synthetic_build_configuration(module)
    capture_source_digest = "1" * 64
    capture_source_git_head = "2" * 40
    library_payload = {
        "build_configuration_digest": build_configuration["digest"],
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15.2.0",
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": capture_source_git_head,
        "source_provenance_digest": capture_source_digest,
    }
    library_identity = {
        "algorithm": module.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        "digest": module._canonical_json_digest(library_payload),
        **library_payload,
    }
    library = {
        "build_identity": library_identity,
        "file": "libdart-simulation.so",
        "path": "/evidence/build/lib/libdart-simulation.so",
        "sha256": "3" * 64,
        "size_bytes": 4096,
    }
    executable = {
        "file": "bm_avbd_rigid_fixed_joint",
        "path": "/evidence/build/bin/bm_avbd_rigid_fixed_joint",
        "sha256": "4" * 64,
        "size_bytes": 8192,
    }
    runtime_images = sorted(
        [
            executable,
            {key: library[key] for key in ("file", "path", "sha256", "size_bytes")},
            {
                "file": "ld-linux-x86-64.so.2",
                "path": "/lib64/ld-linux-x86-64.so.2",
                "sha256": "6" * 64,
                "size_bytes": 2048,
            },
            {
                "file": "libbenchmark.so.1",
                "path": "/usr/lib/libbenchmark.so.1",
                "sha256": "7" * 64,
                "size_bytes": 2048,
            },
            {
                "file": "libc.so.6",
                "path": "/usr/lib/libc.so.6",
                "sha256": "8" * 64,
                "size_bytes": 2048,
            },
            {
                "file": "libm.so.6",
                "path": "/usr/lib/libm.so.6",
                "sha256": "9" * 64,
                "size_bytes": 2048,
            },
            {
                "file": "libstdc++.so.6",
                "path": "/usr/lib/libstdc++.so.6",
                "sha256": "a" * 64,
                "size_bytes": 2048,
            },
        ],
        key=lambda image: image["path"],
    )
    runtime_inventory_payload = {
        "images": runtime_images,
        "required_roles": {
            "dynamic_loader": "/lib64/ld-linux-x86-64.so.2",
            "google_benchmark": "/usr/lib/libbenchmark.so.1",
            "libc": "/usr/lib/libc.so.6",
            "libm": "/usr/lib/libm.so.6",
            "libstdcxx": "/usr/lib/libstdc++.so.6",
        },
    }
    runtime_image_inventory = {
        "algorithm": module.RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        "digest": module._canonical_json_digest(runtime_inventory_payload),
        **runtime_inventory_payload,
    }
    benchmark_provenance = {
        "benchmark_source_sha256": "5" * 64,
        "capture_source_git_head": capture_source_git_head,
        "capture_source_provenance_digest": capture_source_digest,
        "executable": executable,
        "loaded_dart_libraries": [library],
        "runtime_image_inventory": runtime_image_inventory,
    }
    payload = {
        "benchmark_source_sha256": benchmark_provenance["benchmark_source_sha256"],
        "build_configuration": build_configuration,
        "capture_source_git_head": capture_source_git_head,
        "capture_source_provenance_digest": capture_source_digest,
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15.2.0",
        "executable_file": executable["file"],
        "executable_path": executable["path"],
        "executable_sha256": executable["sha256"],
        "executable_size_bytes": executable["size_bytes"],
        "loaded_dart_libraries": [library],
        "runtime_image_inventory": runtime_image_inventory,
        "ndebug": "1",
        "optimization_enabled": "1",
    }
    identity = {
        "algorithm": module.BENCHMARK_BUILD_IDENTITY_ALGORITHM,
        "digest": module._canonical_json_digest(payload),
        **payload,
    }
    return identity, benchmark_provenance


def test_schema6_capture_runtime_requires_a_shared_dart_library_inventory():
    module = _load_module()
    provenance = _synthetic_capture_runtime_provenance(module)
    capture = {"runtime_provenance": provenance}

    assert (
        module._paper_capture_runtime_errors(
            capture,
            expected_source_digest=provenance["source_provenance_digest"],
            expected_source_git_head=provenance["source_git_head"],
            label="capture",
        )
        == []
    )

    provenance["loaded_dart_libraries"] = []
    provenance["dart_library_linkage"] = "statically_embedded_in_extension"
    provenance["digest"] = module._capture_runtime_manifest_digest(provenance)
    errors = module._paper_capture_runtime_errors(
        capture,
        expected_source_digest=provenance["source_provenance_digest"],
        expected_source_git_head=provenance["source_git_head"],
        label="capture",
    )

    assert any("non-empty shared-library inventory" in error for error in errors)
    assert any("dart_library_linkage must be 'shared'" in error for error in errors)


def test_schema6_benchmark_build_identity_binds_configuration_and_executable():
    module = _load_module()
    identity, provenance = _synthetic_benchmark_build_identity(module)

    assert (
        module._paper_benchmark_build_identity_errors(
            identity,
            benchmark_provenance=provenance,
            label="benchmark.build_identity",
        )
        == []
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("release_flags", "CMAKE_CXX_FLAGS_RELEASE must be '-O3 -DNDEBUG'"),
        (
            "library_configuration",
            "build_configuration_digest must match the benchmark build identity",
        ),
        ("executable_hash", "executable_sha256 must match executable identity"),
    ),
)
def test_schema6_benchmark_build_identity_mutations_fail_closed(mutation, message):
    module = _load_module()
    identity, provenance = _synthetic_benchmark_build_identity(module)
    if mutation == "release_flags":
        configuration = identity["build_configuration"]
        configuration["values"]["CMAKE_CXX_FLAGS_RELEASE"] = "-O1 -DNDEBUG"
        record = "".join(
            [f"algorithm={module.BUILD_CONFIGURATION_ALGORITHM}\n"]
            + [
                f"{key}={configuration['values'][key]}\n"
                for key in module.BUILD_CONFIGURATION_KEYS
            ]
        )
        configuration["digest"] = hashlib.sha256(record.encode("utf-8")).hexdigest()
    elif mutation == "library_configuration":
        identity["loaded_dart_libraries"][0]["build_identity"][
            "build_configuration_digest"
        ] = ("9" * 64)
    else:
        identity["executable_sha256"] = "9" * 64
    payload = {
        key: value
        for key, value in identity.items()
        if key not in {"algorithm", "digest"}
    }
    identity["digest"] = module._canonical_json_digest(payload)

    errors = module._paper_benchmark_build_identity_errors(
        identity,
        benchmark_provenance=provenance,
        label="benchmark.build_identity",
    )

    assert any(message in error for error in errors), errors


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("missing_benchmark", "exactly one google_benchmark image"),
        ("ambiguous_benchmark", "exactly one google_benchmark image"),
        ("wrong_role_binding", "must bind its mapped image"),
        ("missing_executable", "must contain the exact executable"),
        ("bad_hash", "must be a lowercase SHA-256 digest"),
    ),
)
def test_schema6_runtime_image_inventory_mutations_fail_closed(mutation, message):
    module = _load_module()
    _identity, provenance = _synthetic_benchmark_build_identity(module)
    inventory = provenance["runtime_image_inventory"]
    images = inventory["images"]
    if mutation == "missing_benchmark":
        inventory["images"] = [
            image for image in images if image["file"] != "libbenchmark.so.1"
        ]
    elif mutation == "ambiguous_benchmark":
        inventory["images"].append(
            {
                "file": "libbenchmark.so.2",
                "path": "/usr/lib/libbenchmark.so.2",
                "sha256": "b" * 64,
                "size_bytes": 2048,
            }
        )
        inventory["images"].sort(key=lambda image: image["path"])
    elif mutation == "wrong_role_binding":
        inventory["required_roles"]["google_benchmark"] = "/tmp/unbound.so"
    elif mutation == "missing_executable":
        inventory["images"] = [
            image for image in images if image["file"] != "bm_avbd_rigid_fixed_joint"
        ]
    else:
        benchmark_image = next(
            image for image in images if image["file"] == "libbenchmark.so.1"
        )
        benchmark_image["sha256"] = "not-a-digest"
    payload = {
        "images": inventory["images"],
        "required_roles": inventory["required_roles"],
    }
    inventory["digest"] = module._canonical_json_digest(payload)

    errors = module._paper_runtime_image_inventory_errors(
        inventory,
        executable=provenance["executable"],
        label="benchmark.runtime_image_inventory",
    )

    assert any(message in error for error in errors), errors


def test_schema6_benchmark_rejects_omitted_mapped_dart_library_identity():
    module = _load_module()
    identity, provenance = _synthetic_benchmark_build_identity(module)
    inventory = identity["runtime_image_inventory"]
    inventory["images"].append(
        {
            "file": "libdart-collision.so",
            "path": "/evidence/build/lib/libdart-collision.so",
            "sha256": "b" * 64,
            "size_bytes": 4096,
        }
    )
    inventory["images"].sort(key=lambda image: image["path"])
    inventory_payload = {
        "images": inventory["images"],
        "required_roles": inventory["required_roles"],
    }
    inventory["digest"] = module._canonical_json_digest(inventory_payload)
    identity_payload = {
        key: value
        for key, value in identity.items()
        if key not in {"algorithm", "digest"}
    }
    identity["digest"] = module._canonical_json_digest(identity_payload)

    errors = module._paper_benchmark_build_identity_errors(
        identity,
        benchmark_provenance=provenance,
        label="benchmark.build_identity",
    )

    assert any(
        "must exactly cover every mapped libdart runtime image" in error
        for error in errors
    ), errors


def _synthetic_benchmark_run_evidence(module, build_identity):
    context = {"date": "2026-08-31T12:00:00Z", "host_name": "test-host"}
    host_payload = {
        "cpu_count": 8,
        "cpu_model": "test-cpu",
        "hostname": "test-host",
        "machine": "x86_64",
        "platform": "Linux-test",
        "system": "Linux",
    }
    host_token = module._canonical_json_digest(host_payload)
    gate = {
        "elapsed_seconds": 120.1,
        "finished_at": "2026-08-31T12:02:00Z",
        "max_normalized_load": 0.1,
        "normalized_load_limit": 0.25,
        "passed": True,
        "sample_count": 121,
        "sample_interval_seconds": 1.0,
        "started_at": "2026-08-31T12:00:00Z",
    }
    payload = {
        "benchmark_context_date": context["date"],
        "benchmark_policy": {
            "filter": module.FIGURE13_BENCHMARK_FILTER,
            "min_warmup_time_seconds": 1.0,
            "repetitions": 5,
            "report_aggregates_only": True,
        },
        "build_identity": build_identity,
        "host_identity": {**host_payload, "host_token": host_token},
        "host_token": host_token,
        "loader_environment": {
            "algorithm": module.LOADER_POLICY_ALGORITHM,
            "forbidden_environment_prefixes": list(module.LOADER_ENVIRONMENT_PREFIXES),
            "passed": True,
            "present_environment_variables": [],
        },
        "quiet_host": {**gate, "duration_seconds": 120.0},
        "run_token": "123e4567-e89b-42d3-a456-426614174000",
        "watchdog": {
            **gate,
            "elapsed_seconds": 3.0,
            "finished_at": "2026-08-31T12:02:03Z",
            "sample_count": 4,
            "started_at": "2026-08-31T12:02:00Z",
        },
    }
    evidence = {
        "schema_version": module.FIGURE13_BENCHMARK_RUN_SCHEMA,
        **payload,
        "digest": module._canonical_json_digest(payload),
    }
    return {"run_evidence": evidence}, context


@pytest.mark.parametrize(
    "mutation",
    ("missing", "false", "preload_present", "wrong_prefix_policy"),
)
def test_schema6_benchmark_run_rejects_loader_environment_mutations(mutation):
    module = _load_module()
    build_identity, _provenance = _synthetic_benchmark_build_identity(module)
    benchmark, context = _synthetic_benchmark_run_evidence(module, build_identity)
    evidence = benchmark["run_evidence"]
    if mutation == "missing":
        del evidence["loader_environment"]
    elif mutation == "false":
        evidence["loader_environment"]["passed"] = False
    elif mutation == "preload_present":
        evidence["loader_environment"]["present_environment_variables"] = ["LD_PRELOAD"]
    else:
        evidence["loader_environment"]["forbidden_environment_prefixes"] = ["LD_"]
    payload = {
        key: value
        for key, value in evidence.items()
        if key not in {"digest", "schema_version"}
    }
    evidence["digest"] = module._canonical_json_digest(payload)

    errors = module._paper_benchmark_run_evidence_errors(
        benchmark,
        context=context,
        build_identity=build_identity,
        packet_name="synthetic",
    )

    assert any("loader_environment" in error for error in errors), errors


def _with_current_outcome_oracles(module, packet, packet_name):
    """Refresh a corpus-derived fixture to the checker-owned outcome oracle.

    The committed packets predate the pinned oracle table, so fixtures cloned
    from them must adopt the current thresholds before asserting a clean run.
    """
    oracles = module.PAPER_OUTCOME_ORACLES.get(packet_name, {})
    visual = packet.get("visual_evidence")
    if isinstance(visual, dict):
        for role, capture in visual.items():
            expected = oracles.get(role)
            metrics = (
                capture.get("scene_metrics") if isinstance(capture, dict) else None
            )
            if expected is None or not isinstance(metrics, dict):
                continue
            oracle = metrics.get("outcome_oracle")
            if isinstance(oracle, dict):
                oracle.update(json.loads(json.dumps(expected)))
    return packet


def _with_synthetic_figure13_artifact_provenance(module, packet, packet_name):
    _with_current_outcome_oracles(module, packet, packet_name)
    spec = module.PAPER_FIGURE13_SPECS[packet_name]
    checkpoint_frames = sorted(
        capture_spec["frame"] for capture_spec in spec["captures"].values()
    )
    for role, capture_spec in spec["captures"].items():
        if capture_spec.get(
            "long_horizon"
        ) is True and not module._paper_requires_long_horizon(packet):
            continue
        capture = packet["visual_evidence"][role]
        frame_count = capture_spec["frame"]
        capture["capture"]["converted_frames"] = frame_count
        capture["scene_metrics"]["scene_contract"][
            "effective_scene_contract_passed"
        ] = True
        width = capture["capture"]["width"]
        height = capture["capture"]["height"]
        files = [
            {
                "file": f"frame_{index:06d}.png",
                "index": index,
                "sha256": hashlib.sha256(
                    f"{packet_name}:{role}:frame:{index}".encode()
                ).hexdigest(),
                "size_bytes": 1000 + index,
            }
            for index in range(1, frame_count + 1)
        ]
        png_frames = {
            "algorithm": module.CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM,
            "count": frame_count,
            "digest": module._ordered_png_manifest_digest(files),
            "files": files,
            "height": height,
            "width": width,
        }
        video_sha256 = hashlib.sha256(
            f"{packet_name}:{role}:video".encode()
        ).hexdigest()
        video = {
            "codec_name": "h264",
            "content_correspondence": {
                "algorithm": (module.CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM),
                "encoder": {
                    **module.CAPTURE_VIDEO_ENCODER,
                    "ffmpeg_version": "7.1.1",
                    "libx264_version": "164 r3108 31e19f9",
                },
                "expected_reencoded_sha256": video_sha256,
                "passed": True,
                "source_png_sequence_digest": png_frames["digest"],
            },
            "decoded_frame_count": frame_count,
            "duration_seconds": (
                f"{Fraction(frame_count, module._PAPER_CAPTURE_VIDEO_FPS).numerator}/"
                f"{Fraction(frame_count, module._PAPER_CAPTURE_VIDEO_FPS).denominator}"
            ),
            "file": f"{spec['scene']}_{role}.mp4",
            "fps": f"{module._PAPER_CAPTURE_VIDEO_FPS}/1",
            "height": height,
            "pixel_format": "yuv420p",
            "probe_algorithm": module.CAPTURE_VIDEO_PROBE_ALGORITHM,
            "sha256": video_sha256,
            "size_bytes": 100_000 + frame_count,
            "width": width,
        }
        terminal_frame = files[-1]
        provenance = {
            "algorithm": module.CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
            "artifact_count": frame_count + 3,
            "digest": "",
            "png_frames": png_frames,
            "scene_metrics_events_sha256": capture["scene_metrics_events"]["sha256"],
            "screenshot_png_frame_binding": {
                "algorithm": module.CAPTURE_SCREENSHOT_BINDING_ALGORITHM,
                "passed": True,
                "png_frame_file": terminal_frame["file"],
                "png_frame_index": frame_count,
                "png_frame_sha256": terminal_frame["sha256"],
            },
            "screenshot_sha256": capture["screenshot"]["sha256"],
            "video": video,
        }
        provenance["digest"] = module._capture_artifact_manifest_digest(provenance)
        capture["artifact_provenance"] = provenance
        capture["scene_metrics_events"]["prefix_sha256"] = {
            str(frame): hashlib.sha256(
                f"{packet_name}:event-prefix:{frame}".encode()
            ).hexdigest()
            for frame in checkpoint_frames
            if frame <= frame_count
        }
    return packet


def _schema6_long_horizon_packet(module):
    packet = _schema6_timing_gate_packet(module)
    visual = packet["visual_evidence"]
    long_horizon = json.loads(json.dumps(visual["retention"]))
    screenshot_hash = hashlib.sha256(b"schema6-vbd-long-horizon-still").hexdigest()
    event_hash = hashlib.sha256(b"schema6-vbd-long-horizon-events").hexdigest()
    long_horizon["label"] = "long_horizon"
    long_horizon["capture"]["requested_frames"] = 600
    long_horizon["scene_metrics_events"] = {
        "event_count": 600,
        "file": "scene_metrics.jsonl",
        "prefix_sha256": {},
        "sha256": event_hash,
    }
    long_horizon["manifest"] = {
        "file": "manifest.json",
        "sha256": hashlib.sha256(b"schema6-vbd-long-horizon-manifest").hexdigest(),
    }
    long_horizon["screenshot"] = {
        "file": "vbd_paper_breakable_wall_long_horizon.png",
        "sha256": screenshot_hash,
    }
    long_horizon["image_verdict"]["image_sha256"] = screenshot_hash
    long_horizon["image_verdict"]["metadata"] = {
        "frame": "600",
        "scene": "vbd_paper_breakable_wall",
        "view": "front-oblique",
    }
    metrics = long_horizon["scene_metrics"]
    metrics["event_count"] = 600
    metrics["frame"] = 600
    metrics["outcome"]["frame"] = 600
    metrics["outcome"]["world_time"] = 10.0
    visual["long_horizon"] = long_horizon
    packet = _with_synthetic_figure13_artifact_provenance(
        module, packet, _TIMING_GATE_PACKET_NAME
    )
    paper_figure = packet["paper_reference"]["figure"]
    inspected_images = [
        {
            "file": visual[role]["screenshot"]["file"],
            "role": module.PAPER_FIGURE13_SPECS[_TIMING_GATE_PACKET_NAME]["captures"][
                role
            ]["review_role"],
            "sha256": visual[role]["screenshot"]["sha256"],
        }
        for role in ("bend", "retention", "long_horizon")
    ]
    inspected_images.append(
        {
            "file": paper_figure["file"],
            "role": "paper_figure_13_reference",
            "sha256": paper_figure["sha256"],
        }
    )
    long_video = visual["long_horizon"]["artifact_provenance"]["video"]
    visual["semantic_review"] = {
        "assessment_assertions": {
            "capture_images_assessed": True,
            "long_horizon_video_assessed": True,
            "no_contradictions_found": True,
            "paper_reference_assessed": True,
            "text_oracle_agrees": True,
            "view_reports_agree": True,
        },
        "claim_assessments": dict(module.SEMANTIC_CLAIM_ASSESSMENTS),
        "file": "vbd-semantic-review.json",
        "inspected_images": inspected_images,
        "inspected_videos": [
            {
                "decoded_frame_count": long_video["decoded_frame_count"],
                "duration_seconds": long_video["duration_seconds"],
                "file": long_video["file"],
                "role": "long_horizon_video_600",
                "sha256": long_video["sha256"],
            }
        ],
        "reviewer_capabilities": {
            "image_semantic_review": True,
            "video_semantic_review": True,
        },
        "sha256": hashlib.sha256(b"schema6-vbd-semantic-review").hexdigest(),
        "structured_observations": dict(
            module.SEMANTIC_STRUCTURED_OBSERVATIONS["bent_retained_wall"]
        ),
        "temporal_assessment": {
            "checkpoint_sequence_agrees": True,
            "full_interval_viewed": True,
            "still_frames_only": False,
            "terminal_behavior": "bent_retained_wall",
        },
        "verdict": "pass",
    }
    return packet


def test_figure13_artifact_manifest_accepts_transitively_consistent_media():
    module = _load_module()
    packet = _with_synthetic_figure13_artifact_provenance(
        module, _timing_gate_packet(), _TIMING_GATE_PACKET_NAME
    )

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert errors == []


def test_schema6_figure13_requires_bound_600_frame_long_horizon_evidence():
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)
    errors.extend(
        module._paper_image_verdict_binding_errors(packet, _TIMING_GATE_PACKET_NAME)
    )

    assert errors == []


def test_schema6_figure13_rejects_missing_long_horizon_capture():
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)
    del packet["visual_evidence"]["long_horizon"]

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(
        "visual_evidence.long_horizon must be an object" in error for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda packet: packet["visual_evidence"]["long_horizon"][
                "capture"
            ].__setitem__("requested_frames", 599),
            "capture.requested_frames must be 600",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"]["scene_metrics"][
                "outcome"
            ].__setitem__("frame", 599),
            "outcome.frame must be 600",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"]["scene_metrics"][
                "outcome"
            ].__setitem__("evaluated", False),
            "outcome.evaluated must be True",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"]["scene_metrics"][
                "outcome"
            ].__setitem__("status", "fail"),
            "outcome.status must be 'pass'",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"]["scene_metrics"][
                "outcome"
            ].__setitem__("thresholds_pass", False),
            "outcome.thresholds_pass must be True",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"]["scene_metrics"][
                "outcome"
            ]["threshold_checks"].__setitem__("wall_retained", False),
            "threshold_checks must exactly match",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"]["scene_metrics"][
                "outcome"
            ].__setitem__("total_retained_fraction", 0.0),
            "total_retained_fraction must be >=",
        ),
        (
            lambda packet: packet["visual_evidence"]["long_horizon"][
                "artifact_provenance"
            ]["video"].__setitem__("decoded_frame_count", 599),
            "artifact_provenance.video.decoded_frame_count must be 600",
        ),
        (
            lambda packet: packet["visual_evidence"]["semantic_review"].__setitem__(
                "inspected_images",
                [
                    entry
                    for entry in packet["visual_evidence"]["semantic_review"][
                        "inspected_images"
                    ]
                    if entry["role"] != "long_horizon_frame_600"
                ],
            ),
            "must inspect 'long_horizon_frame_600' exactly once",
        ),
        (
            lambda packet: packet["visual_evidence"]["semantic_review"][
                "inspected_images"
            ][-2].__setitem__("sha256", "0" * 64),
            "long_horizon_frame_600.sha256 must match",
        ),
        (
            lambda packet: packet["visual_evidence"]["semantic_review"].__setitem__(
                "inspected_videos", []
            ),
            "must inspect exactly one long-horizon video",
        ),
        (
            lambda packet: packet["visual_evidence"]["semantic_review"][
                "inspected_videos"
            ][0].__setitem__("sha256", "0" * 64),
            "inspected video must bind the exact long-horizon MP4 bytes",
        ),
    ),
)
def test_schema6_figure13_long_horizon_mutations_fail_closed(mutation, message):
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)
    mutation(packet)

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(message in error for error in errors), errors


@pytest.mark.parametrize(
    ("path", "replacement", "message"),
    (
        (
            ("reviewer_capabilities", "video_semantic_review"),
            False,
            "reviewer_capabilities must positively attest",
        ),
        (
            ("assessment_assertions", "no_contradictions_found"),
            False,
            "assessment_assertions must be the exact positive contract",
        ),
        (
            ("claim_assessments", "cuda_parity"),
            "supported",
            "claim_assessments must use the exact authoritative",
        ),
        (
            ("claim_assessments", "other_figure13_solver_rows"),
            "supported",
            "claim_assessments must use the exact authoritative",
        ),
        (
            ("temporal_assessment", "full_interval_viewed"),
            False,
            "temporal_assessment must bind full-interval review",
        ),
        (
            ("temporal_assessment", "still_frames_only"),
            True,
            "temporal_assessment must bind full-interval review",
        ),
        (
            ("structured_observations", "view_report_relationship"),
            "contradicts",
            "structured_observations must be the exact authoritative",
        ),
        (
            ("reconciliation_and_verdict",),
            "I skipped the video; ViewReports contradict all evidence.",
            "semantic_review must contain exactly",
        ),
    ),
)
def test_schema6_figure13_semantic_review_structured_contract_fails_closed(
    path, replacement, message
):
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)
    _set_nested(packet["visual_evidence"]["semantic_review"], path, replacement)

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(message in error for error in errors), errors


@pytest.mark.parametrize(
    ("field", "replacement", "message"),
    (
        ("frame", "599", "metadata.frame must be '600'"),
        ("scene", "avbd_paper_breakable_wall", "metadata.scene must be"),
    ),
)
def test_schema6_figure13_long_horizon_verdict_metadata_is_exact(
    field, replacement, message
):
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)
    packet["visual_evidence"]["long_horizon"]["image_verdict"]["metadata"][
        field
    ] = replacement

    errors = module._paper_image_verdict_binding_errors(
        packet, _TIMING_GATE_PACKET_NAME
    )

    assert any(message in error for error in errors), errors


def test_schema6_figure13_long_horizon_verdict_binds_the_still_bytes():
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)
    packet["visual_evidence"]["long_horizon"]["image_verdict"]["image_sha256"] = (
        "0" * 64
    )

    errors = module._paper_image_verdict_binding_errors(
        packet, _TIMING_GATE_PACKET_NAME
    )

    assert any(
        "long_horizon.image_verdict.image_sha256 must match" in error
        for error in errors
    )


def test_schema6_figure13_long_horizon_requires_capture_source_provenance(
    monkeypatch,
):
    module = _load_module()
    packet = _schema6_long_horizon_packet(module)
    current = packet["visual_evidence"]["bend"]["source_provenance"]
    monkeypatch.setattr(
        module,
        "compute_capture_source_provenance",
        lambda _root: current,
    )
    del packet["visual_evidence"]["long_horizon"]["source_provenance"]

    errors = module._paper_capture_source_binding_errors(
        packet, _TIMING_GATE_PACKET_NAME
    )

    assert any(
        "long_horizon.source_provenance must be an object" in error for error in errors
    )


@pytest.mark.parametrize(
    ("path", "replacement", "message"),
    (
        (
            ("capture", "converted_frames"),
            17,
            "capture.converted_frames must be 18",
        ),
        (
            ("screenshot", "file"),
            "different-scene_bend.png",
            "screenshot.file must be",
        ),
        (
            ("artifact_provenance", "artifact_count"),
            20,
            "artifact_provenance.artifact_count must be 21",
        ),
        (
            ("artifact_provenance", "png_frames", "count"),
            17,
            "artifact_provenance.png_frames.count must be 18",
        ),
        (
            (
                "artifact_provenance",
                "png_frames",
                "files",
                0,
                "sha256",
            ),
            "f" * 64,
            "png_frames.digest must be derived",
        ),
        (
            ("artifact_provenance", "video", "sha256"),
            "e" * 64,
            "complete capture artifact manifest",
        ),
        (
            ("artifact_provenance", "video", "decoded_frame_count"),
            17,
            "artifact_provenance.video.decoded_frame_count must be 18",
        ),
        (
            ("artifact_provenance", "digest"),
            "d" * 64,
            "complete capture artifact manifest",
        ),
    ),
)
def test_figure13_artifact_manifest_one_field_mutations_fail_closed(
    path, replacement, message
):
    module = _load_module()
    packet = _with_synthetic_figure13_artifact_provenance(
        module, _timing_gate_packet(), _TIMING_GATE_PACKET_NAME
    )
    _set_nested(packet["visual_evidence"]["bend"], path, replacement)

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(message in error for error in errors), errors


def test_figure13_artifact_manifest_rejects_huge_numeric_fields_without_raising():
    module = _load_module()
    packet = _with_synthetic_figure13_artifact_provenance(
        module, _timing_gate_packet(), _TIMING_GATE_PACKET_NAME
    )
    entry = packet["visual_evidence"]["bend"]["artifact_provenance"]["png_frames"][
        "files"
    ][0]
    entry["size_bytes"] = 10**400

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("positive uint64" in error for error in errors), errors
    assert any("png_frames.digest must be derived" in error for error in errors), errors


def test_paper_benchmark_timing_gate_accepts_consistent_medians():
    module = _load_module()
    errors = module._paper_benchmark_timing_errors(
        _timing_gate_packet(), _TIMING_GATE_PACKET_NAME
    )
    assert errors == []


def test_schema6_paper_benchmark_gate_accepts_zero_configuration_stddev():
    module = _load_module()
    errors = module._paper_benchmark_timing_errors(
        _schema6_timing_gate_packet(module), _TIMING_GATE_PACKET_NAME
    )
    assert errors == []


@pytest.mark.parametrize(
    ("solver", "counter"),
    (
        ("avbd", "scene_spec_fingerprint_hi"),
        ("avbd", "solver_configuration_fingerprint_lo"),
        ("avbd", "rigid_avbd_beta"),
        ("avbd", "solver_projection_policies_match"),
        ("vbd", "public_vbd_family"),
        ("vbd", "contact_method_sequential_impulse"),
        ("vbd", "rigid_constraint_iterations"),
        ("vbd", "runtime_identity_contract_passed"),
    ),
)
def test_schema6_paper_benchmark_gate_rejects_nonzero_invariant_stddev(solver, counter):
    module = _load_module()
    packet = _schema6_timing_gate_packet(module)
    rows = packet["benchmark"]["methods"][solver]["rows"]
    stddev = next(row for row in rows if row["aggregate_name"] == "stddev")
    # Mean and median remain equal, so this models symmetric repetition drift
    # that the old two-aggregate check could not observe.
    stddev[counter] = 1.0

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(f"stddev.{counter} must be 0" in error for error in errors), errors


def test_paper_benchmark_timing_gate_rejects_ratio_drift():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["benchmark"]["comparison"]["vbd_to_avbd_median_cpu_cost_ratio"] = 3.0
    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)
    assert any("the embedded medians record" in error for error in errors)


def test_paper_benchmark_timing_gate_rejects_non_positive_median():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["benchmark"]["methods"]["vbd"]["rows"][1]["cpu_time"] = 0.0
    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)
    assert any("representative timings must be positive" in error for error in errors)


def test_paper_benchmark_timing_gate_rejects_unreferenced_median():
    module = _load_module()
    packet = _timing_gate_packet()
    del packet["benchmark"]["methods"]["avbd"]["rows"]
    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)
    assert any(
        "benchmark.methods.avbd.rows must be a list" in error for error in errors
    )
    assert any(
        "references a median the packet does not embed" in error for error in errors
    )


def test_paper_benchmark_timing_gate_rejects_typoed_numerator_name():
    module = _load_module()
    packet = _timing_gate_packet()
    comparison = packet["benchmark"]["comparison"]
    value = comparison.pop("vbd_to_avbd_median_cpu_cost_ratio")
    comparison["typo_to_avbd_median_cpu_cost_ratio"] = value

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(
        "references a median the packet does not embed" in error for error in errors
    ), errors


def test_paper_benchmark_timing_gate_requires_exact_ratio_keys():
    module = _load_module()
    packet = _timing_gate_packet()
    comparison = packet["benchmark"]["comparison"]
    value = comparison.pop("vbd_to_avbd_median_cpu_cost_ratio")
    comparison["vbd_to_avbd_median_cpu_cost_rati0"] = value

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
    packet_name = "avbd-paper-breakable-wall-packet.json"
    packet = json.loads((PACKET_DIR / packet_name).read_text())
    packet["benchmark"].pop("rows")

    errors = module._paper_benchmark_timing_errors(packet, packet_name)

    assert any("benchmark.rows must be a list" in error for error in errors), errors


def _set_nested(value, path, replacement):
    target = value
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = replacement


@pytest.mark.parametrize(
    ("path", "replacement", "message"),
    (
        (
            ("resolved_solver_identity", "rigid_contact_solver"),
            "avbd",
            "Figure 13 filename",
        ),
        (
            (
                "visual_evidence",
                "bend",
                "scene_metrics",
                "resolved_configuration",
                0,
                "resolved",
            ),
            "avbd",
            "resolved_configuration",
        ),
        (
            (
                "visual_evidence",
                "bend",
                "scene_metrics",
                "resolved_configuration",
                0,
                "reason",
            ),
            "contradictory solver explanation",
            "reason 'as requested'",
        ),
        (
            ("visual_evidence", "bend", "scene_metrics", "outcome", "status"),
            "fail",
            "outcome.status",
        ),
        (
            (
                "visual_evidence",
                "bend",
                "scene_metrics",
                "outcome",
                "threshold_checks",
                "wall_bends",
            ),
            False,
            "threshold_checks",
        ),
        (
            (
                "visual_evidence",
                "bend",
                "scene_metrics",
                "outcome",
                "maximum_wall_normal_displacement",
            ),
            0.0,
            "minimum_maximum_wall_normal_displacement",
        ),
        (
            ("benchmark", "methods", "vbd", "rows", 1, "resolved_rigid_body_vbd"),
            0.0,
            "resolved_rigid_body_vbd",
        ),
        (
            (
                "benchmark",
                "methods",
                "vbd",
                "rows",
                1,
                "scene_spec_fingerprint_hi",
            ),
            0.0,
            "scene_spec_fingerprint_hi",
        ),
        (
            ("benchmark", "methods", "vbd", "rows", 1, "trajectory_frames"),
            119.0,
            "trajectory_frames",
        ),
        (
            (
                "benchmark",
                "methods",
                "vbd",
                "timing",
                "median_cpu_time_per_step_ns",
            ),
            1.0,
            "derived from the embedded aggregate row",
        ),
        (
            (
                "benchmark",
                "methods",
                "vbd",
                "stability",
                "cpu_time_cv_fraction",
            ),
            0.09,
            "derived from the embedded cv aggregate row",
        ),
        (
            ("benchmark", "comparison", "vbd_to_avbd_median_cpu_cost_ratio"),
            2.0,
            "embedded medians record",
        ),
    ),
)
def test_figure13_one_field_contradictions_fail_closed(path, replacement, message):
    module = _load_module()
    packet = _timing_gate_packet()
    _set_nested(packet, path, replacement)

    errors = module._paper_figure13_consistency_errors(
        packet, _TIMING_GATE_PACKET_NAME
    ) + module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any(message in error for error in errors), errors


def test_figure13_rejects_duplicate_contradictory_resolved_note():
    module = _load_module()
    packet = _timing_gate_packet()
    resolved = packet["visual_evidence"]["bend"]["scene_metrics"][
        "resolved_configuration"
    ]
    contradictory = dict(resolved[0])
    contradictory["resolved"] = "avbd"
    resolved.append(contradictory)

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("exactly one rigid-body" in error for error in errors), errors


def test_figure13_rejects_fractional_fingerprint_counter_drift():
    module = _load_module()
    packet = _timing_gate_packet()
    row = packet["benchmark"]["methods"]["vbd"]["rows"][1]
    row["scene_spec_fingerprint_hi"] += 0.001

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("scene_spec_fingerprint_hi" in error for error in errors), errors


def test_figure13_rejects_unrepresentable_inactive_identity_counter():
    module = _load_module()
    packet = _timing_gate_packet()
    row = packet["benchmark"]["methods"]["vbd"]["rows"][1]
    row["resolved_rigid_body_avbd"] = 10**400

    errors = module._paper_benchmark_timing_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("must be a finite zero" in error for error in errors), errors


def test_figure13_rejects_oracle_threshold_drift_between_captures():
    module = _load_module()
    packet = _timing_gate_packet()
    oracle = packet["visual_evidence"]["retention"]["scene_metrics"]["outcome_oracle"]
    oracle["minimum_maximum_wall_normal_displacement"] /= 2.0

    errors = module._paper_figure13_consistency_errors(packet, _TIMING_GATE_PACKET_NAME)

    assert any("must exactly match visual_evidence.bend" in error for error in errors)


def test_figure13_requires_oracle_broken_joint_identity_digest():
    module = _load_module()
    packet_name = "avbd-paper-breakable-wall-packet.json"
    packet = json.loads((PACKET_DIR / packet_name).read_text())
    for role in ("impact", "outcome"):
        del packet["visual_evidence"][role]["scene_metrics"]["outcome_oracle"][
            "expected_broken_joint_ids_sha256"
        ]

    errors = module._paper_figure13_consistency_errors(packet, packet_name)

    assert any("must match the outcome oracle" in error for error in errors), errors


def test_figure13_rejects_broken_joint_record_identity_drift():
    module = _load_module()
    packet_name = "avbd-paper-breakable-wall-packet.json"
    packet = json.loads((PACKET_DIR / packet_name).read_text())
    records = packet["visual_evidence"]["impact"]["scene_metrics"]["outcome"][
        "broken_joint_records"
    ]
    records[0]["id"] += "_contradiction"

    errors = module._paper_figure13_consistency_errors(packet, packet_name)

    assert any("must be derived from broken_joint_records" in error for error in errors)


@pytest.mark.parametrize(
    ("parent_name", "link_key", "path", "replacement", "message"),
    (
        (
            "avbd-paper-vbd-comparison-packet.json",
            "linked_avbd_evidence",
            ("scene_spec_fingerprint",),
            "0" * 16,
            "must match the parsed linked AVBD packet",
        ),
        (
            "avbd-paper-vbd-comparison-packet.json",
            "linked_avbd_evidence",
            ("visual_evidence", "impact_screenshot", "sha256"),
            "f" * 64,
            "must match the parsed linked AVBD packet",
        ),
        (
            "avbd-paper-sequential-impulse-comparison-packet.json",
            "linked_avbd_vbd_evidence",
            (
                "benchmark_method_timings",
                "vbd",
                "median_cpu_time_per_step_ns",
            ),
            1.0,
            "derived from the linked packet's embedded median aggregate rows",
        ),
        (
            "avbd-paper-sequential-impulse-comparison-packet.json",
            "linked_avbd_vbd_evidence",
            ("nested_avbd_source_provenance_digest",),
            "f" * 64,
            "must match the parsed nested AVBD packet",
        ),
        (
            "avbd-paper-sequential-impulse-comparison-packet.json",
            "linked_avbd_vbd_evidence",
            ("resolved_solver_identity", "rigid_contact_solver"),
            "avbd",
            "must match the parsed linked VBD packet",
        ),
    ),
)
def test_figure13_link_summary_one_field_contradictions_fail_closed(
    parent_name, link_key, path, replacement, message
):
    module = _load_module()
    parent = json.loads((PACKET_DIR / parent_name).read_text())
    link = parent[link_key]
    _set_nested(link, path, replacement)
    linked_path = (PACKET_DIR / link["file"]).resolve()
    linked_packet = json.loads(linked_path.read_text())
    context = module.PacketValidationContext(packet_dir=PACKET_DIR)

    errors = module._paper_link_summary_errors(
        parent,
        parent_name,
        link_key,
        link,
        linked_packet,
        linked_path,
        context,
    )

    assert any(message in error for error in errors), errors


def test_sequential_impulse_ratio_uses_linked_raw_aggregate_rows():
    module = _load_module()
    parent_name = "avbd-paper-sequential-impulse-comparison-packet.json"
    parent = json.loads((PACKET_DIR / parent_name).read_text())
    parent["benchmark"]["comparison"][
        "sequential_impulse_to_avbd_median_cpu_cost_ratio"
    ] = 9.0
    link_key = "linked_avbd_vbd_evidence"
    link = parent[link_key]
    linked_path = (PACKET_DIR / link["file"]).resolve()
    linked_packet = json.loads(linked_path.read_text())
    context = module.PacketValidationContext(packet_dir=PACKET_DIR)

    errors = module._paper_link_summary_errors(
        parent,
        parent_name,
        link_key,
        link,
        linked_packet,
        linked_path,
        context,
    )

    assert any("embedded aggregate rows record" in error for error in errors), errors


def test_public_packet_validation_handles_huge_timing_integer(tmp_path, monkeypatch):
    module = _load_module()
    timing_validator = module._paper_benchmark_timing_errors
    _disable_figure13_consistency(module, monkeypatch)
    # Re-enable only the aggregate validator under test.
    monkeypatch.setattr(
        module,
        "_paper_benchmark_timing_errors",
        timing_validator,
    )
    packet = _timing_gate_packet()
    packet["benchmark"]["methods"]["vbd"]["rows"][1]["cpu_time"] = 10**400
    path = _write_packet(tmp_path, _TIMING_GATE_PACKET_NAME, packet)

    errors = module.packet_errors(path, packet_dir=tmp_path)

    assert errors
    assert any("timings must be finite" in error for error in errors), errors


def test_paper_benchmark_timing_gate_ignores_non_paper_packets():
    module = _load_module()
    packet = _timing_gate_packet()
    packet["benchmark"]["comparison"]["vbd_to_avbd_median_cpu_cost_ratio"] = 3.0
    errors = module._paper_benchmark_timing_errors(
        packet, "avbd-demo2d-ground-packet.json"
    )
    assert errors == []


def _complete_paper_packet(module, tmp_path, monkeypatch, packet_name=None):
    """Assemble a paper packet that `packet_errors` accepts with zero errors.

    Every sub-manifest is built from the same helpers the producers use, so a
    checker that drifts from `capture_runtime_provenance` or from the compiled
    benchmark identity fails here instead of only on the committed corpus.
    """
    packet_name = packet_name or "avbd-paper-breakable-wall-packet.json"
    _disable_figure13_consistency(module, monkeypatch)
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        module,
        "PAPER_PACKET_SOURCE_PATHS",
        {packet_name: (module.BENCHMARK_SOURCE_PATH.as_posix(),)},
    )
    benchmark_source = tmp_path / module.BENCHMARK_SOURCE_PATH
    benchmark_source.parent.mkdir(parents=True, exist_ok=True)
    benchmark_source.write_text("benchmark source\n")
    benchmark_hash = _file_sha256(benchmark_source)

    runtime = _synthetic_capture_runtime_provenance(module)
    current = {
        "algorithm": module.CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "digest": runtime["source_provenance_digest"],
        "file_count": 7,
        "git_head": runtime["source_git_head"],
        "ignored_paths": [],
        "roots": ["dart", "python"],
        "working_tree_clean": True,
    }
    monkeypatch.setattr(
        module, "compute_capture_source_provenance", lambda _root: dict(current)
    )

    visual_evidence = _schema6_bound_figure13_visual(packet_name)
    for capture in visual_evidence.values():
        capture["source_provenance"] = dict(current)
        capture["runtime_provenance"] = json.loads(json.dumps(runtime))

    build_identity, benchmark_provenance = _synthetic_benchmark_build_identity(module)
    identity_payload = {
        key: value
        for key, value in build_identity.items()
        if key not in {"algorithm", "digest"}
    }
    identity_payload["benchmark_source_sha256"] = benchmark_hash
    build_identity = {
        "algorithm": module.BENCHMARK_BUILD_IDENTITY_ALGORITHM,
        "digest": module._canonical_json_digest(identity_payload),
        **identity_payload,
    }
    run_evidence, context = _synthetic_benchmark_run_evidence(module, build_identity)

    source_provenance = dict(benchmark_provenance)
    source_provenance["algorithm"] = module.BENCHMARK_SOURCE_PROVENANCE_ALGORITHM
    source_provenance["benchmark_source_sha256"] = benchmark_hash
    source_provenance["capture_source_provenance_digest"] = current["digest"]
    source_provenance["capture_source_git_head"] = current["git_head"]
    source_provenance["build_identity"] = build_identity
    source_payload = {
        key: value
        for key, value in source_provenance.items()
        if key not in {"algorithm", "digest"}
    }
    source_provenance["digest"] = module._canonical_json_digest(source_payload)
    executable = source_provenance["executable"]

    packet = {
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": _identity(
            rigid_contact_solver="avbd",
            rigid_point_joint_solver="avbd",
            rigid_contact_selection="world_solver_family",
        ),
        "source_provenance": _source_provenance(
            tmp_path, module.BENCHMARK_SOURCE_PATH.as_posix()
        ),
        "visual_evidence": visual_evidence,
        "benchmark": {
            "context": {
                "benchmark_source_sha256": benchmark_hash,
                "capture_source_provenance_digest": current["digest"],
                "dart_benchmark_executable_path": executable["path"],
                "dart_benchmark_source_sha256": benchmark_hash,
                "dart_build_configuration_digest": (
                    build_identity["build_configuration"]["digest"]
                ),
                "dart_capture_source_git_head": current["git_head"],
                "dart_capture_source_provenance_digest": current["digest"],
                "dart_cmake_build_type": "Release",
                "dart_compiler_id": "GNU",
                "dart_compiler_version": "15.2.0",
                "dart_ndebug": "1",
                "dart_optimization_enabled": "1",
                "executable": executable["path"],
                **context,
            },
            "timing": {"median_cpu_time_per_step_ns": 5.0},
            "source_provenance": source_provenance,
            **run_evidence,
        },
    }
    path = _write_packet(tmp_path, packet_name, packet)
    return path, packet


def _rewrite(path, packet):
    path.write_text(json.dumps(packet))


def test_capture_runtime_manifest_round_trips_through_packet_errors(
    tmp_path, monkeypatch
):
    module = _load_module()
    path, packet = _complete_paper_packet(module, tmp_path, monkeypatch)

    assert _packet_errors(module, path) == []

    runtime = packet["visual_evidence"]["impact"]["runtime_provenance"]
    assert set(runtime) == set(module.CAPTURE_RUNTIME_PROVENANCE_KEYS)
    assert runtime["digest"] == module._capture_runtime_manifest_digest(runtime)


@pytest.mark.parametrize("omitted", ("loader_environment", "runtime_image_inventory"))
def test_capture_runtime_manifest_requires_every_producer_field(
    tmp_path, monkeypatch, omitted
):
    module = _load_module()
    path, packet = _complete_paper_packet(module, tmp_path, monkeypatch)
    runtime = packet["visual_evidence"]["impact"]["runtime_provenance"]
    del runtime[omitted]
    runtime["digest"] = module._capture_runtime_manifest_digest(runtime)
    _rewrite(path, packet)

    errors = _packet_errors(module, path)

    assert any("runtime_provenance must contain exactly" in e for e in errors), errors
    assert any(omitted in e for e in errors), errors


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("loader_failed", "loader_environment.passed"),
        ("loader_preload", "loader_environment.present_environment_variables"),
        ("loader_roots", "trusted_image_roots"),
        ("inventory_algorithm", "runtime_image_inventory.algorithm"),
        ("inventory_digest", "runtime_image_inventory.digest"),
        ("inventory_extension_bytes", "dartpy extension bytes verbatim"),
        ("inventory_drops_library", "exactly the loaded DART libraries"),
        ("inventory_required", "required_images must bind"),
    ],
)
def test_capture_runtime_manifest_mutations_fail_closed(
    tmp_path, monkeypatch, mutation, message
):
    module = _load_module()
    path, packet = _complete_paper_packet(module, tmp_path, monkeypatch)
    runtime = packet["visual_evidence"]["impact"]["runtime_provenance"]
    loader = runtime["loader_environment"]
    inventory = runtime["runtime_image_inventory"]
    if mutation == "loader_failed":
        loader["passed"] = False
    elif mutation == "loader_preload":
        loader["present_environment_variables"] = ["LD_PRELOAD"]
    elif mutation == "loader_roots":
        loader["trusted_image_roots"] = ["relative/path"]
    elif mutation == "inventory_algorithm":
        inventory["algorithm"] = "sha256-something-else-v1"
    elif mutation == "inventory_digest":
        inventory["digest"] = "0" * 64
    elif mutation == "inventory_extension_bytes":
        extension_path = runtime["native_extension"]["path"]
        for image in inventory["images"]:
            if image["path"] == extension_path:
                image["sha256"] = "0" * 64
    elif mutation == "inventory_drops_library":
        library_path = runtime["loaded_dart_libraries"][0]["path"]
        inventory["images"] = [
            image for image in inventory["images"] if image["path"] != library_path
        ]
    else:
        inventory["required_images"] = {"dartpy_native_extension": "/elsewhere.so"}
    if mutation not in {"inventory_digest"}:
        payload = {
            "images": inventory["images"],
            "required_images": inventory["required_images"],
        }
        inventory["digest"] = module._canonical_json_digest(payload)
    runtime["digest"] = module._capture_runtime_manifest_digest(runtime)
    _rewrite(path, packet)

    errors = _packet_errors(module, path)

    assert any(message in error for error in errors), errors


def test_paper_outcome_oracles_match_writer_constants():
    module = _load_module()
    writers = {
        "avbd-paper-breakable-wall-packet.json": (
            "write_avbd_paper_breakable_wall_packet"
        ),
        "avbd-paper-vbd-comparison-packet.json": (
            "write_avbd_paper_vbd_comparison_packet"
        ),
        "avbd-paper-sequential-impulse-comparison-packet.json": (
            "write_avbd_paper_sequential_impulse_comparison_packet"
        ),
    }
    assert set(module.PAPER_OUTCOME_ORACLES) == set(writers)
    for packet_name, writer_name in writers.items():
        spec = importlib.util.spec_from_file_location(
            f"outcome_oracle_source_{writer_name}",
            ROOT / "scripts" / f"{writer_name}.py",
        )
        assert spec is not None
        assert spec.loader is not None
        writer = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = writer
        spec.loader.exec_module(writer)
        checkpoints = module.PAPER_OUTCOME_ORACLES[packet_name]
        expected_checkpoints = {
            *module.PAPER_CAPTURE_ROLES[packet_name],
            module.PAPER_LONG_HORIZON_ROLE,
        }
        assert set(checkpoints) == expected_checkpoints
        for oracle in checkpoints.values():
            assert oracle == writer.OUTCOME_ORACLE, packet_name


@pytest.mark.parametrize(
    "packet_name",
    (
        "avbd-paper-breakable-wall-packet.json",
        "avbd-paper-vbd-comparison-packet.json",
        "avbd-paper-sequential-impulse-comparison-packet.json",
    ),
)
def test_packet_cannot_relax_the_outcome_oracle_it_is_graded_against(packet_name):
    module = _load_module()
    packet = json.loads((PACKET_DIR / packet_name).read_text())
    roles = module.PAPER_CAPTURE_ROLES[packet_name]
    relaxed_key = next(
        key
        for key, value in module.PAPER_OUTCOME_ORACLES[packet_name][roles[0]].items()
        if (key.startswith("minimum_") or key.startswith("maximum_"))
        and isinstance(value, (int, float))
        and not isinstance(value, bool)
        and value != 0
    )
    for role in roles:
        packet["visual_evidence"][role]["scene_metrics"]["outcome_oracle"][
            relaxed_key
        ] = 0

    errors = module._paper_figure13_consistency_errors(packet, packet_name)

    assert any(
        f"outcome_oracle.{relaxed_key} must be" in error
        and "cannot choose the threshold" in error
        for error in errors
    ), errors


def test_breakable_wall_packet_cannot_zero_minimum_broken_joints():
    module = _load_module()
    packet_name = "avbd-paper-breakable-wall-packet.json"
    packet = json.loads((PACKET_DIR / packet_name).read_text())
    for role in module.PAPER_CAPTURE_ROLES[packet_name]:
        packet["visual_evidence"][role]["scene_metrics"]["outcome_oracle"][
            "minimum_broken_joints"
        ] = 0

    errors = module._paper_figure13_consistency_errors(packet, packet_name)

    assert any(
        "outcome_oracle.minimum_broken_joints must be 150" in error for error in errors
    ), errors


def _minimal_non_paper_packet(module, **extra):
    packet = {
        "packet": "synthetic",
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": _identity(),
    }
    packet.update(extra)
    return packet


def test_source_provenance_is_required_at_the_current_schema_version(tmp_path):
    module = _load_module()
    packet = _minimal_non_paper_packet(module)
    path = _write_packet(tmp_path, "avbd-forged-current-packet.json", packet)

    errors = _packet_errors(module, path)

    assert any(
        "source_provenance must be an object for a packet written at "
        f"schema_version {module.AVBD_PACKET_SCHEMA_VERSION}" in error
        for error in errors
    ), errors


def test_source_provenance_is_required_for_any_plan104_claim_packet(tmp_path):
    module = _load_module()
    packet = _minimal_non_paper_packet(module)
    packet["schema_version"] = module.PLAN104_CLAIMS_MIN_SCHEMA_VERSION
    packet[module.PLAN104_CLAIMS_KEY] = {
        "avbd.rigid.contact": {
            "status": "complete",
            "predicate_results": {},
            "backend_results": {},
        }
    }
    path = _write_packet(tmp_path, "avbd-forged-claims-packet.json", packet)

    errors = _packet_errors(module, path)

    assert any(
        f"source_provenance must be an object for a packet that records "
        f"{module.PLAN104_CLAIMS_KEY}" in error
        for error in errors
    ), errors


def test_legacy_pinned_packet_keeps_its_provenance_free_shape():
    module = _load_module()
    legacy_name = next(iter(module.LEGACY_PACKET_SCHEMA_VERSIONS))
    packet = {
        "packet": "legacy",
        "schema_version": module.LEGACY_PACKET_SCHEMA_VERSIONS[legacy_name],
    }

    assert module._source_provenance_errors(packet, legacy_name) == []


def test_image_verdict_binding_covers_packets_outside_figure13():
    module = _load_module()
    screenshot_sha256 = "b" * 64
    packet = {
        "visual_capture": {
            "screenshot": {"file": "capture.png", "sha256": screenshot_sha256},
            "image_verdict": {
                "file": "image_verdict.json",
                "image_sha256": screenshot_sha256,
                "pass": True,
            },
        }
    }

    assert (
        module._image_verdict_binding_errors(packet, "avbd-synthetic-packet.json") == []
    )

    packet["visual_capture"]["image_verdict"]["image_sha256"] = "c" * 64
    errors = module._image_verdict_binding_errors(packet, "avbd-synthetic-packet.json")
    assert any(
        "visual_capture.image_verdict.image_sha256 must match" in error
        for error in errors
    ), errors

    del packet["visual_capture"]["image_verdict"]["image_sha256"]
    errors = module._image_verdict_binding_errors(packet, "avbd-synthetic-packet.json")
    assert any(
        "visual_capture.image_verdict.image_sha256 must be a lowercase" in error
        for error in errors
    ), errors


def test_image_verdict_binding_is_reached_by_packet_errors(tmp_path):
    module = _load_module()
    screenshot_sha256 = "b" * 64
    packet = _minimal_non_paper_packet(
        module,
        visual_capture={
            "screenshot": {"file": "capture.png", "sha256": screenshot_sha256},
            "image_verdict": {
                "file": "image_verdict.json",
                "image_sha256": "c" * 64,
                "pass": True,
            },
        },
    )
    path = _write_packet(tmp_path, "avbd-forged-verdict-packet.json", packet)

    errors = _packet_errors(module, path)

    assert any(
        "visual_capture.image_verdict.image_sha256 must match" in error
        for error in errors
    ), errors


def test_paper_image_verdict_binding_is_not_double_reported(tmp_path, monkeypatch):
    module = _load_module()
    path, packet = _complete_paper_packet(module, tmp_path, monkeypatch)
    packet["visual_evidence"]["impact"]["image_verdict"]["image_sha256"] = "c" * 64
    _rewrite(path, packet)

    errors = _packet_errors(module, path)
    matching = [
        error
        for error in errors
        if "visual_evidence.impact.image_verdict.image_sha256 must match" in error
    ]

    assert len(matching) == 1, errors


@pytest.mark.parametrize(
    ("validator", "message"),
    [
        ("capture_source_binding", "does not match current source state"),
        ("benchmark_run_evidence", "benchmark.run_evidence"),
        ("capture_digest_compare", "does not match current source state"),
        ("benchmark_source_hash_compare", "benchmark source hash does not match"),
    ],
)
def test_each_paper_validator_is_wired_into_packet_errors(
    tmp_path, monkeypatch, validator, message
):
    """One injected defect per validator, observed through `packet_errors`.

    Deleting any of these calls or comparisons used to leave the suite green,
    so each defect is asserted at the public entry point rather than against
    the rule function.
    """
    module = _load_module()
    path, packet = _complete_paper_packet(module, tmp_path, monkeypatch)
    assert _packet_errors(module, path) == []

    if validator == "capture_source_binding":
        packet["visual_evidence"]["impact"]["source_provenance"]["roots"] = ["dart"]
    elif validator == "benchmark_run_evidence":
        del packet["benchmark"]["run_evidence"]
    elif validator == "capture_digest_compare":
        packet["visual_evidence"]["impact"]["source_provenance"]["digest"] = "0" * 64
    else:
        packet["benchmark"]["source_provenance"]["benchmark_source_sha256"] = "0" * 64
    _rewrite(path, packet)

    errors = _packet_errors(module, path)

    assert any(message in error for error in errors), errors
