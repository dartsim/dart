"""Tests for the PLAN-123 citation evidence validator (fail-closed contract).

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
PLAN_DIR = ROOT / "docs" / "plans" / "123-citation-driven-simulation-trust"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_citation_evidence", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()
LANE = "dart7"


def complete_packet() -> dict:
    return {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-001",
        "title": "Test packet",
        "source": {"url": "https://example.org/claim", "claim": "A claim."},
        "target": {"branch": "main", "commit": "0" * 40},
        "scene": {
            "id": "test_scene",
            "digest": "sha256:" + "a" * 64,
            "description": "A scene.",
        },
        "configuration": {
            "requested": {"contact_solver_method": "BOXED_LCP"},
            "resolved": {"contact_solver_method": "BOXED_LCP"},
            "resolved_provenance": "World property readback",
            "detector": "dart7 native pipeline",
            "timestep": 0.002,
            "substeps": 1,
            "iterations": "defaults",
            "fallback_policy": "World defaults; none observed",
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
            "numerical": {"method": "step metrics", "max_penetration_m": 1e-5},
            "performance": {
                "status": "unsupported",
                "reason": "no timing methodology",
            },
            "allocation": {
                "status": "unsupported",
                "reason": "PLAN-122 tooling owns allocation gates",
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


def test_non_object_packet_fails():
    assert MODULE.packet_errors([1, 2, 3]) == ["packet must be a JSON object"]


@pytest.mark.parametrize(
    "path",
    [
        ("target", "commit"),
        ("target", "branch"),
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


@pytest.mark.parametrize(
    "section",
    [
        "source",
        "target",
        "scene",
        "configuration",
        "ensemble",
        "metrics",
        "evidence",
        "result",
        "review",
    ],
)
def test_each_required_section_fails_closed(section):
    packet = complete_packet()
    del packet[section]
    errors = MODULE.packet_errors(packet)
    assert any("missing required top-level keys" in error for error in errors)


def test_unknown_top_level_key_fails():
    packet = complete_packet()
    packet["extra_notes"] = "sneaky"
    errors = MODULE.packet_errors(packet)
    assert any("unknown top-level keys" in error for error in errors)


def test_short_commit_fails():
    packet = complete_packet()
    packet["target"]["commit"] = "abc123"
    assert any("40-hex" in error for error in MODULE.packet_errors(packet))


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


def test_null_metric_value_fails():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_penetration_m"] = None
    errors = MODULE.packet_errors(packet)
    assert any("contains null" in error for error in errors)


def test_nan_metric_value_fails():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_penetration_m"] = float("nan")
    errors = MODULE.packet_errors(packet)
    assert any("non-finite" in error for error in errors)


def test_missing_metric_group_fails():
    packet = complete_packet()
    del packet["metrics"]["allocation"]
    errors = MODULE.packet_errors(packet)
    assert any("metrics.allocation is required" in error for error in errors)


def test_untyped_visual_fails():
    packet = complete_packet()
    packet["evidence"]["visual"] = []
    errors = MODULE.packet_errors(packet)
    assert any("visual" in error for error in errors)


def test_invalid_disposition_fails():
    packet = complete_packet()
    packet["result"]["disposition"] = "looks-fine"
    errors = MODULE.packet_errors(packet)
    assert any("disposition" in error for error in errors)


def test_unknown_claim_id_fails_when_manifest_known():
    packet = complete_packet()
    errors = MODULE.packet_errors(packet, known_claim_ids={"CT-999"})
    assert any("not in the claims manifest" in error for error in errors)


def test_review_pass_needs_reviewer_and_summary():
    packet = complete_packet()
    packet["review"]["passes"] = [{"reviewer": "someone"}]
    errors = MODULE.packet_errors(packet)
    assert any("review.passes[0]" in error for error in errors)


def _minimal_manifest(ids):
    return {
        "schema": "dart.citation_claim_manifest/v1",
        "first_wave_families": [
            "family_a",
            "family_b",
            "family_c",
            "family_d",
            "family_e",
            "family_f",
        ],
        "claims": [
            {
                "id": claim_id,
                "title": f"Claim {claim_id}",
                "source": "somewhere",
                "first_wave_family": None,
                "lanes": {
                    "dart7": {
                        "owner": "PLAN-123",
                        "status": "audit-required",
                        "disposition": None,
                        "evidence": [],
                    },
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


def test_manifest_matches_corpus_ids():
    manifest = _minimal_manifest(["CT-001", "CT-002"])
    assert MODULE.manifest_errors(manifest, ["CT-001", "CT-002"]) == []
    errors = MODULE.manifest_errors(manifest, ["CT-001", "CT-002", "CT-003"])
    assert any("missing from manifest" in error for error in errors)


def test_manifest_rejects_wrong_family_count():
    manifest = _minimal_manifest(["CT-001"])
    manifest["first_wave_families"] = ["only_one"]
    errors = MODULE.manifest_errors(manifest, ["CT-001"])
    assert any("exactly 6" in error for error in errors)


def test_manifest_rejects_unknown_family_reference():
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["first_wave_family"] = "family_x"
    errors = MODULE.manifest_errors(manifest, ["CT-001"])
    assert any("capped family list" in error for error in errors)


def test_closed_lane_requires_disposition_and_evidence():
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart7"]["status"] = "closed"
    errors = MODULE.manifest_errors(manifest, ["CT-001"])
    assert any("without a valid disposition" in error for error in errors)
    assert any("prose cannot close a row" in error for error in errors)


def test_not_applicable_lane_requires_reason():
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart6"]["status"] = "not-applicable"
    errors = MODULE.manifest_errors(manifest, ["CT-001"])
    assert any("must record a reason" in error for error in errors)


def test_corpus_claim_ids_parses_table_rows():
    text = "| CT-001 | x |\n| CT-002 | y |\nno row\n| CT-002 | dup |\n"
    assert MODULE.corpus_claim_ids(text) == ["CT-001", "CT-002"]


def _write_tree(tmp_path, *, packet=None, negative=None, manifest=None):
    plan_dir = tmp_path / "plan"
    evidence = plan_dir / "evidence"
    negative_dir = evidence / "negative-controls"
    negative_dir.mkdir(parents=True)
    (plan_dir / "citation-claim-corpus.md").write_text(
        "| CT-001 | claim |\n", encoding="utf-8"
    )
    if manifest is None:
        manifest = _minimal_manifest(["CT-001"])
        if packet is not None:
            manifest["claims"][0]["lanes"]["dart7"]["status"] = "in-progress"
            manifest["claims"][0]["lanes"]["dart7"]["evidence"] = [
                "evidence/packet.json"
            ]
    (plan_dir / "claims-manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    if packet is not None:
        (evidence / "packet.json").write_text(json.dumps(packet), encoding="utf-8")
    if negative is not None:
        (negative_dir / "incomplete.json").write_text(
            json.dumps(negative), encoding="utf-8"
        )
    return plan_dir


def test_validate_tree_accepts_complete_state(tmp_path):
    incomplete = {"schema": "dart.citation_claim_evidence/v1"}
    plan_dir = _write_tree(tmp_path, packet=complete_packet(), negative=incomplete)
    assert MODULE.validate_tree(plan_dir) == []


def test_validate_tree_requires_negative_control(tmp_path):
    plan_dir = _write_tree(tmp_path, packet=complete_packet())
    errors = MODULE.validate_tree(plan_dir)
    assert any("negative-control" in error for error in errors)


def test_validate_tree_rejects_passing_negative_control(tmp_path):
    plan_dir = _write_tree(
        tmp_path, packet=complete_packet(), negative=complete_packet()
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("fail-closed proof is vacuous" in error for error in errors)


def test_validate_tree_rejects_unreferenced_packet(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    plan_dir = _write_tree(
        tmp_path,
        packet=complete_packet(),
        negative={"schema": "x"},
        manifest=manifest,
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("not referenced by any manifest lane" in error for error in errors)


def test_validate_tree_rejects_branch_lane_mismatch(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["target"]["branch"] = "release-6.20"
    plan_dir = _write_tree(tmp_path, packet=packet, negative={"schema": "x"})
    errors = MODULE.validate_tree(plan_dir)
    assert any("does not match lane dart7" in error for error in errors)


def test_validate_tree_rejects_missing_referenced_packet(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart7"]["evidence"] = ["evidence/ghost.json"]
    plan_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    errors = MODULE.validate_tree(plan_dir)
    assert any("missing packet" in error for error in errors)


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
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/sub/prose.json"]
    plan_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    sub = plan_dir / "evidence" / "sub"
    sub.mkdir(parents=True)
    (sub / "prose.json").write_text(
        json.dumps({"this is": "not a packet"}), encoding="utf-8"
    )
    errors = MODULE.validate_tree(plan_dir)
    assert errors, "a nested non-packet must not close a lane"
    assert any("missing required top-level keys" in error for error in errors)


def test_validate_tree_rejects_negative_control_as_lane_evidence(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/negative-controls/incomplete.json"]
    plan_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    errors = MODULE.validate_tree(plan_dir)
    assert any("negative control" in error for error in errors)


def test_validate_tree_rejects_non_json_lane_evidence(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/notes.md"]
    plan_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    (plan_dir / "evidence" / "notes.md").write_text("prose", encoding="utf-8")
    errors = MODULE.validate_tree(plan_dir)
    assert any("not a .json packet" in error for error in errors)


def test_validate_tree_rejects_scalar_lane_evidence(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart7"]["evidence"] = 7
    plan_dir = _write_tree(tmp_path, negative={"schema": "x"}, manifest=manifest)
    errors = MODULE.validate_tree(plan_dir)
    assert any("evidence must be a list" in error for error in errors)


def test_validate_tree_rejects_shared_packet_owner(tmp_path):
    manifest = _minimal_manifest(["CT-001", "CT-002"])
    for claim in manifest["claims"]:
        claim["lanes"]["dart7"]["status"] = "in-progress"
        claim["lanes"]["dart7"]["evidence"] = ["evidence/packet.json"]
    plan_dir = _write_tree(
        tmp_path,
        packet=complete_packet(),
        negative={"schema": "x"},
        manifest=manifest,
    )
    (plan_dir / "citation-claim-corpus.md").write_text(
        "| CT-001 | claim |\n| CT-002 | claim |\n", encoding="utf-8"
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("one packet has one owner" in error for error in errors)


def test_validate_tree_closed_lane_needs_two_review_passes(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [{"reviewer": "first", "summary": "clean"}]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/packet.json"]
    plan_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("at least two recorded review passes" in error for error in errors)
    packet["review"]["passes"].append({"reviewer": "second", "summary": "clean"})
    (plan_dir / "evidence" / "packet.json").write_text(
        json.dumps(packet), encoding="utf-8"
    )
    assert MODULE.validate_tree(plan_dir) == []


def test_validate_tree_freshness_flags_stale_commit(tmp_path):
    plan_dir = _write_tree(tmp_path, packet=complete_packet(), negative={"schema": "x"})
    errors = MODULE.validate_tree(plan_dir, freshness_head="1" * 40)
    assert any("--freshness" in error for error in errors)


def test_repository_tree_validates():
    """The committed manifest, packets, and negative controls must pass."""
    errors = MODULE.validate_tree(PLAN_DIR)
    assert errors == []


def test_non_string_lane_evidence_entry_fails(tmp_path):
    """A lane must not be closable by an entry the packet checks cannot read."""
    for bad in ({"path": "evidence/x.json"}, None, 42, True, ["evidence/x.json"]):
        manifest = _minimal_manifest(["CT-001"])
        lane = manifest["claims"][0]["lanes"]["dart7"]
        lane["status"] = "closed"
        lane["disposition"] = "reproduced"
        lane["evidence"] = [bad]
        tree = _write_tree(
            tmp_path / f"case{abs(hash(str(bad)))}",
            negative={"schema": "x"},
            manifest=manifest,
        )
        errors = MODULE.validate_tree(tree)
        assert any("non-string entry" in error for error in errors), bad


def test_raw_path_pointing_at_a_directory_fails(tmp_path):
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    (tmp_path / "somedir").mkdir()
    packet["evidence"]["raw_paths"] = ["somedir"]
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any("does not resolve" in error for error in errors)


def test_scene_digest_must_match_published_parameters():
    packet = complete_packet()
    packet["scene"]["parameters"] = {"a": 1}
    errors = MODULE.packet_errors(packet)
    assert any("does not match the digest" in error for error in errors)
    import hashlib as _h
    import json as _j

    packet["scene"]["digest"] = (
        "sha256:"
        + _h.sha256(
            _j.dumps({"a": 1}, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    )
    assert MODULE.packet_errors(packet) == []


def test_nested_negative_control_is_enumerated(tmp_path):
    """A control in a subdirectory must still be required to fail."""
    tree = _write_tree(tmp_path, negative={"schema": "x"})
    nested = tree / "evidence" / "negative-controls" / "deep"
    nested.mkdir(parents=True)
    (nested / "passing.json").write_text(
        json.dumps(complete_packet()), encoding="utf-8"
    )
    errors = MODULE.validate_tree(tree)
    assert any("fail-closed proof is vacuous" in error for error in errors)
