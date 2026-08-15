"""Tests for the release-6.20 citation evidence validator (fail-closed).

Every required provenance field must fail validation when missing or
degraded; a complete packet must pass; negative-control packets must fail.
"""

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_citation_evidence.py"
DESIGN_DIR = ROOT / "docs" / "design" / "dart6_citation_driven_contact_trust"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_citation_evidence", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()


def complete_packet() -> dict:
    return {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-001",
        "title": "Test packet",
        "source": {"url": "https://example.org/claim", "claim": "A claim."},
        "target": {"branch": "release-6.20", "commit": "0" * 40},
        "scene": {
            "id": "test_scene",
            "digest": "sha256:" + "a" * 64,
            "description": "A scene.",
        },
        "configuration": {
            "requested": {"collision_detector": "fcl", "solver": "boxed-lcp-dantzig"},
            "resolved": {"collision_detector": "fcl", "solver": "boxed-lcp-dantzig"},
            "resolved_provenance": "ConstraintSolver/CollisionDetector readback",
            "detector": "fcl (PRIMITIVE default)",
            "timestep": 0.001,
            "substeps": 1,
            "iterations": "defaults",
            "fallback_policy": "boxed LCP secondary fallback; none observed",
        },
        "ensemble": {
            "kind": "deterministic-repeats",
            "deterministic_repeats": 2,
            "measurement_window": {"start_s": 0.0, "end_s": 1.0},
        },
        "metrics": {
            "physical": {
                "method": "sweep",
                "lateral_drift_m": 0.0,
                "measured_zero_fields": ["lateral_drift_m"],
            },
            "numerical": {"method": "penetration probe", "max_penetration_m": 1e-5},
            "performance": {
                "status": "unsupported",
                "reason": "no timing methodology",
            },
            "allocation": {
                "status": "unsupported",
                "reason": "no allocation methodology on this row",
            },
        },
        "evidence": {
            "commands": ["pixi run python scripts/example.py"],
            "raw_rows": [{"angle_deg": 0.0, "lateral_drift_m": 0.0}],
            "visual": {
                "status": "not-applicable",
                "reason": "numeric oracle only",
            },
        },
        "result": {
            "disposition": "unresolved",
            "claim_boundary": "This commit, this scene only.",
            "limitations": ["Single fixture."],
        },
        "review": {"passes": []},
    }


def test_complete_packet_passes():
    assert MODULE.packet_errors(complete_packet()) == []


def test_wrong_branch_fails():
    packet = complete_packet()
    packet["target"]["branch"] = "main"
    errors = MODULE.packet_errors(packet)
    assert any("target.branch" in error for error in errors)


@pytest.mark.parametrize(
    "path",
    [
        ("target", "commit"),
        ("scene", "digest"),
        ("configuration", "requested"),
        ("configuration", "resolved"),
        ("configuration", "resolved_provenance"),
        ("configuration", "detector"),
        ("configuration", "timestep"),
        ("configuration", "fallback_policy"),
        ("ensemble", "measurement_window"),
        ("evidence", "commands"),
        ("result", "disposition"),
        ("result", "claim_boundary"),
        ("result", "limitations"),
    ],
)
def test_each_required_field_fails_closed(path):
    packet = complete_packet()
    section, field = path
    del packet[section][field]
    errors = MODULE.packet_errors(packet)
    assert errors, f"deleting {section}.{field} must fail validation"
    assert any(field in error for error in errors)


def test_single_run_ensemble_fails():
    packet = complete_packet()
    packet["ensemble"]["deterministic_repeats"] = 1
    errors = MODULE.packet_errors(packet)
    assert any("single runs are not evidence" in error for error in errors)


def test_unsupported_metric_requires_reason():
    packet = complete_packet()
    packet["metrics"]["performance"] = {"status": "unsupported"}
    errors = MODULE.packet_errors(packet)
    assert any("no non-empty reason" in error for error in errors)


def test_measured_metric_requires_method():
    packet = complete_packet()
    packet["metrics"]["physical"] = {"lateral_drift_m": 0.0}
    errors = MODULE.packet_errors(packet)
    assert any("measurement 'method'" in error for error in errors)


def test_unsupported_metric_cannot_carry_values():
    packet = complete_packet()
    packet["metrics"]["performance"] = {
        "status": "unsupported",
        "reason": "no methodology",
        "wall_time_ms": 0.0,
    }
    errors = MODULE.packet_errors(packet)
    assert any("mixes unsupported status" in error for error in errors)


def test_null_and_nan_metric_values_fail():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_penetration_m"] = None
    assert any("contains null" in e for e in MODULE.packet_errors(packet))
    packet["metrics"]["numerical"]["max_penetration_m"] = float("nan")
    assert any("non-finite" in e for e in MODULE.packet_errors(packet))


def test_unknown_top_level_key_fails():
    packet = complete_packet()
    packet["extra_notes"] = "sneaky"
    errors = MODULE.packet_errors(packet)
    assert any("unknown top-level keys" in error for error in errors)


def _minimal_manifest(ids):
    return {
        "schema": "dart.citation_claim_manifest/v1",
        "branch": "release-6.20",
        "corpus_reference": {
            "path": (
                "docs/plans/123-citation-driven-simulation-trust/"
                "citation-claim-corpus.md"
            ),
            "branch": "main",
        },
        "claims": [
            {
                "id": claim_id,
                "title": f"Claim {claim_id}",
                "source": "somewhere",
                "lanes": {
                    "dart6": {
                        "owner": "dev task",
                        "status": "audit-required",
                        "disposition": None,
                        "evidence": [],
                    },
                },
            }
            for claim_id in ids
        ],
    }


def test_manifest_passes():
    assert MODULE.manifest_errors(_minimal_manifest(["CT-001"])) == []


def test_manifest_requires_branch_and_corpus_reference():
    manifest = _minimal_manifest(["CT-001"])
    manifest["branch"] = "main"
    errors = MODULE.manifest_errors(manifest)
    assert any("release-6.20" in error for error in errors)
    manifest = _minimal_manifest(["CT-001"])
    del manifest["corpus_reference"]
    errors = MODULE.manifest_errors(manifest)
    assert any("corpus_reference" in error for error in errors)


def test_manifest_rejects_extra_lane():
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart7"] = {
        "owner": "x",
        "status": "audit-required",
        "disposition": None,
        "evidence": [],
    }
    errors = MODULE.manifest_errors(manifest)
    assert any("lanes must define exactly" in error for error in errors)


def test_closed_lane_requires_disposition_and_evidence():
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart6"]["status"] = "closed"
    errors = MODULE.manifest_errors(manifest)
    assert any("without a valid disposition" in error for error in errors)
    assert any("prose cannot close a row" in error for error in errors)


def test_not_applicable_lane_requires_reason():
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart6"]["status"] = "not-applicable"
    errors = MODULE.manifest_errors(manifest)
    assert any("must record a reason" in error for error in errors)


def _write_tree(tmp_path, *, packet=None, negative=None, manifest=None):
    design_dir = tmp_path / "design"
    evidence = design_dir / "evidence"
    negative_dir = evidence / "negative-controls"
    negative_dir.mkdir(parents=True)
    if manifest is None:
        manifest = _minimal_manifest(["CT-001"])
        if packet is not None:
            manifest["claims"][0]["lanes"]["dart6"]["status"] = "in-progress"
            manifest["claims"][0]["lanes"]["dart6"]["evidence"] = [
                "evidence/packet.json"
            ]
    (design_dir / "claims-manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    if packet is not None:
        (evidence / "packet.json").write_text(json.dumps(packet), encoding="utf-8")
    if negative is not None:
        (negative_dir / "incomplete.json").write_text(
            json.dumps(negative), encoding="utf-8"
        )
    return design_dir


def test_validate_tree_accepts_complete_state(tmp_path):
    incomplete = {"schema": "dart.citation_claim_evidence/v1"}
    design_dir = _write_tree(tmp_path, packet=complete_packet(), negative=incomplete)
    assert MODULE.validate_tree(design_dir) == []


def test_validate_tree_requires_negative_control(tmp_path):
    design_dir = _write_tree(tmp_path, packet=complete_packet())
    errors = MODULE.validate_tree(design_dir)
    assert any("negative-control" in error for error in errors)


def test_validate_tree_rejects_passing_negative_control(tmp_path):
    design_dir = _write_tree(
        tmp_path, packet=complete_packet(), negative=complete_packet()
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("fail-closed proof is vacuous" in error for error in errors)


def test_validate_tree_rejects_unreferenced_packet(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    design_dir = _write_tree(
        tmp_path,
        packet=complete_packet(),
        negative={"schema": "x"},
        manifest=manifest,
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("not referenced by any manifest lane" in error for error in errors)


def test_unacknowledged_zero_fails():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_solver_residual"] = 0.0
    errors = MODULE.packet_errors(packet)
    assert any("without acknowledgement" in error for error in errors)


def test_acknowledged_measured_zero_passes():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_solver_residual"] = 0.0
    packet["metrics"]["numerical"]["measured_zero_fields"] = ["max_solver_residual"]
    assert MODULE.packet_errors(packet) == []


def test_typed_unsupported_leaf_passes_and_needs_reason():
    packet = complete_packet()
    packet["metrics"]["numerical"]["solver_residual"] = {
        "status": "unsupported",
        "reason": "never computed on this path",
    }
    assert MODULE.packet_errors(packet) == []
    packet["metrics"]["numerical"]["solver_residual"] = {"status": "unsupported"}
    errors = MODULE.packet_errors(packet)
    assert any("typed unsupported" in error for error in errors)


def test_stale_measured_zero_declaration_fails():
    packet = complete_packet()
    packet["metrics"]["numerical"]["measured_zero_fields"] = ["not_zero_here"]
    errors = MODULE.packet_errors(packet)
    assert any("which are not zero" in error for error in errors)


def test_nested_zero_is_caught_by_path():
    packet = complete_packet()
    packet["metrics"]["physical"]["per_solver_summary"] = {
        "BOXED_LCP": {"max_penetration_m": 0.0}
    }
    errors = MODULE.packet_errors(packet)
    assert any(
        "per_solver_summary.BOXED_LCP.max_penetration_m" in error for error in errors
    )


def test_spelled_placeholder_fails():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_penetration_m"] = "n/a"
    errors = MODULE.packet_errors(packet)
    assert any("placeholder" in error for error in errors)


def test_only_empty_containers_fails():
    packet = complete_packet()
    packet["metrics"]["numerical"] = {"method": "m", "values": {}}
    errors = MODULE.packet_errors(packet)
    assert any("only empty containers" in error for error in errors)


def test_empty_list_alongside_real_values_passes():
    packet = complete_packet()
    packet["metrics"]["numerical"]["violations"] = []
    assert MODULE.packet_errors(packet) == []


def test_dangling_raw_path_fails(tmp_path):
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["does/not/exist.csv"]
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any("does not resolve" in error for error in errors)
    (tmp_path / "real.csv").write_text("x", encoding="utf-8")
    packet["evidence"]["raw_paths"] = ["real.csv"]
    assert MODULE.packet_errors(packet, base_dir=tmp_path) == []


def test_empty_measurement_window_fails():
    packet = complete_packet()
    packet["ensemble"]["measurement_window"] = {}
    errors = MODULE.packet_errors(packet)
    assert any("measurement_window" in error for error in errors)


def test_validate_tree_validates_packets_in_subdirectories(tmp_path):
    """A lane may not close a row with a file the packet checks never reach."""
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/sub/prose.json"]
    design_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    sub = design_dir / "evidence" / "sub"
    sub.mkdir(parents=True)
    (sub / "prose.json").write_text(
        json.dumps({"this is": "not a packet"}), encoding="utf-8"
    )
    errors = MODULE.validate_tree(design_dir)
    assert errors, "a nested non-packet must not close a lane"
    assert any("missing required top-level keys" in error for error in errors)


def test_validate_tree_rejects_negative_control_as_lane_evidence(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/negative-controls/incomplete.json"]
    design_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    errors = MODULE.validate_tree(design_dir)
    assert any("negative control" in error for error in errors)


def test_validate_tree_rejects_non_json_lane_evidence(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/notes.md"]
    design_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    (design_dir / "evidence" / "notes.md").write_text("prose", encoding="utf-8")
    errors = MODULE.validate_tree(design_dir)
    assert any("not a .json packet" in error for error in errors)


def test_validate_tree_rejects_scalar_lane_evidence(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart6"]["evidence"] = 7
    design_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    errors = MODULE.validate_tree(design_dir)
    assert any("evidence must be a list" in error for error in errors)


def test_validate_tree_rejects_shared_packet_owner(tmp_path):
    manifest = _minimal_manifest(["CT-001", "CT-002"])
    for claim in manifest["claims"]:
        claim["lanes"]["dart6"]["status"] = "in-progress"
        claim["lanes"]["dart6"]["evidence"] = ["evidence/packet.json"]
    design_dir = _write_tree(
        tmp_path,
        packet=complete_packet(),
        negative={"schema": "x"},
        manifest=manifest,
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("one packet has one owner" in error for error in errors)


def test_validate_tree_closed_lane_needs_two_review_passes(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [{"reviewer": "first", "summary": "clean"}]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/packet.json"]
    design_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("at least two recorded review passes" in error for error in errors)
    packet["review"]["passes"].append({"reviewer": "second", "summary": "clean"})
    (design_dir / "evidence" / "packet.json").write_text(
        json.dumps(packet), encoding="utf-8"
    )
    assert MODULE.validate_tree(design_dir) == []


def test_validate_tree_freshness_flags_stale_commit(tmp_path):
    design_dir = _write_tree(
        tmp_path, packet=complete_packet(), negative={"schema": "x"}
    )
    errors = MODULE.validate_tree(design_dir, freshness_head="1" * 40)
    assert any("--freshness" in error for error in errors)


def test_repository_tree_validates():
    """The committed manifest and negative controls must pass the gate."""
    errors = MODULE.validate_tree(DESIGN_DIR)
    assert errors == []
