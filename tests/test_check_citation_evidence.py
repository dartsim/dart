"""Tests for the PLAN-123 citation evidence validator (fail-closed contract).

Every required provenance field must fail validation when missing or
degraded; a complete packet must pass; negative-control packets must fail.
"""

import copy
import hashlib
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
        "target": {
            "branch": "main",
            "commit": "0" * 40,
            "fetch_hint": (
                "git fetch origin pull/3445/head && git checkout " + "0" * 40
            ),
        },
        "scene": {
            "id": "test_scene",
            "digest": (
                "sha256:"
                + hashlib.sha256(
                    json.dumps(
                        {"gravity": -9.81}, sort_keys=True, separators=(",", ":")
                    ).encode("utf-8")
                ).hexdigest()
            ),
            "parameters": {"gravity": -9.81},
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
            "deterministic_repeats_identical": True,
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
            "commands": [
                "pixi run build",
                "pixi run python scripts/example.py",
            ],
            "raw_rows": [
                {
                    "angle_deg": 0.0,
                    "lateral_drift_m": 0.0,
                    "trajectory_sha256": "d" * 64,
                }
            ],
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
        "host": {
            "platform": "test-host",
            "python": "3.14",
            "performance_valid": False,
        },
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
        (negative_dir / "incomplete.expected-errors.json").write_text(
            json.dumps(["missing required top-level keys"]), encoding="utf-8"
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
    (tmp_path / "real.csv").write_text("a,b\n1,2\n", encoding="utf-8")
    packet["evidence"]["raw_paths"] = ["real.csv"]
    # Path-based evidence pins its artifact bytes; repeat verification is
    # bound separately by an explicit trajectory digest (artifact digests
    # prove file identity, not repeat determinism).
    packet["evidence"]["artifact_digests"] = {
        "real.csv": "sha256:"
        + hashlib.sha256((tmp_path / "real.csv").read_bytes()).hexdigest()
    }
    packet["ensemble"]["repeat_trajectory_sha256"] = "a" * 64
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


def _bound_pass(packet: dict, reviewer: str) -> dict:
    """A review pass bound to the packet's current non-review content."""
    return {
        "reviewer": reviewer,
        "summary": "clean",
        "verdict": "pass",
        "content_digest": MODULE._packet_content_digest(packet),
    }


def test_validate_tree_closed_lane_needs_two_review_passes(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [_bound_pass(packet, "first")]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "unresolved"
    lane["evidence"] = ["evidence/packet.json"]
    plan_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("at least two recorded review passes" in error for error in errors)
    packet["review"]["passes"].append(_bound_pass(packet, "second"))
    (plan_dir / "evidence" / "packet.json").write_text(
        json.dumps(packet), encoding="utf-8"
    )
    assert MODULE.validate_tree(plan_dir) == []


def test_validate_tree_closed_lane_needs_distinct_reviewers(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [
        _bound_pass(packet, "same"),
        _bound_pass(packet, "same"),
    ]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "unresolved"
    lane["evidence"] = ["evidence/packet.json"]
    plan_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("INDEPENDENT" in error for error in errors)


def test_validate_tree_closed_lane_disposition_must_match_packet(tmp_path):
    packet = copy.deepcopy(complete_packet())  # result.disposition: unresolved
    packet["review"]["passes"] = [
        _bound_pass(packet, "first"),
        _bound_pass(packet, "second"),
    ]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/packet.json"]
    plan_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any(
        "cannot publish a conclusion its evidence does not support" in error
        for error in errors
    )


def test_review_pass_must_bind_to_packet_content():
    packet = complete_packet()
    packet["review"]["passes"] = [
        {
            "reviewer": "first",
            "summary": "clean",
            "content_digest": "sha256:" + "0" * 64,
        }
    ]
    errors = MODULE.packet_errors(packet)
    assert any("not bound to this packet's content" in error for error in errors)
    packet["review"]["passes"] = [_bound_pass(packet, "first")]
    assert MODULE.packet_errors(packet) == []
    # Changing any non-review content invalidates the binding.
    packet["result"]["claim_boundary"] = "Changed after review."
    errors = MODULE.packet_errors(packet)
    assert any("not bound to this packet's content" in error for error in errors)


def test_validate_tree_freshness_flags_stale_commit(tmp_path):
    plan_dir = _write_tree(tmp_path, packet=complete_packet(), negative={"schema": "x"})
    errors = MODULE.validate_tree(plan_dir, freshness_head="1" * 40)
    assert any("--freshness" in error for error in errors)


def test_repository_tree_validates():
    """The committed manifest, packets, and negative controls must pass."""
    errors = MODULE.validate_tree(PLAN_DIR)
    assert errors == []


def test_fetch_hint_is_required():
    packet = complete_packet()
    del packet["target"]["fetch_hint"]
    errors = MODULE.packet_errors(packet)
    assert any("fetch_hint" in error for error in errors)


def test_raw_rows_placeholders_fail():
    for bad in ([None], ["prose"], [{}], [{"k": 1}, None]):
        packet = complete_packet()
        packet["evidence"]["raw_rows"] = bad
        errors = MODULE.packet_errors(packet)
        assert any(
            "raw_rows" in error and "structured record" in error for error in errors
        ), bad


def test_raw_paths_must_stay_inside_evidence_roots(tmp_path):
    for bad in ("/etc/passwd", "../escape.json", "C:\\evil.json", "a/../../b"):
        packet = complete_packet()
        del packet["evidence"]["raw_rows"]
        packet["evidence"]["raw_paths"] = [bad]
        errors = MODULE.packet_errors(packet)
        assert any(
            "relative path inside the repository" in error for error in errors
        ), bad
    # A relative path escaping via symlink-free resolution is caught with a
    # base_dir even when the file exists outside the roots.
    outside = tmp_path / "outside.json"
    outside.write_text("{}", encoding="utf-8")
    plan = tmp_path / "plan"
    plan.mkdir()
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["missing/nowhere.json"]
    errors = MODULE.packet_errors(packet, base_dir=plan)
    assert any("does not resolve to an existing file" in error for error in errors)


def test_ensemble_sweep_and_seed_entries_must_be_valid_and_distinct():
    base = complete_packet()
    del base["ensemble"]["deterministic_repeats"]
    for bad, needle in (
        ([None, None], "sweep[0]"),
        ([{"a": 1}, {"a": 1}], "DISTINCT points"),
        ([{}, {"a": 1}], "sweep[0]"),
    ):
        packet = copy.deepcopy(base)
        packet["ensemble"]["sweep"] = bad
        errors = MODULE.packet_errors(packet)
        assert any(needle in error for error in errors), (bad, errors)
    for bad, needle in (
        ([None, None], "seeds[0]"),
        ([7, 7], "DISTINCT seeds"),
        ([True, False], "seeds[0]"),
    ):
        packet = copy.deepcopy(base)
        packet["ensemble"]["seeds"] = bad
        errors = MODULE.packet_errors(packet)
        assert any(needle in error for error in errors), (bad, errors)
    good = copy.deepcopy(base)
    good["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    good["evidence"]["raw_rows"] = [
        {"angle_deg": 0.0, "lateral_drift_m": 0.0, "trajectory_sha256": "d" * 64},
        {"angle_deg": 15.0, "lateral_drift_m": 0.1, "trajectory_sha256": "e" * 64},
    ]
    assert MODULE.packet_errors(good) == []


def test_visual_entries_are_validated():
    for bad in ([None], [{"path": "x.png"}], [123]):
        packet = complete_packet()
        packet["evidence"]["visual"] = bad
        errors = MODULE.packet_errors(packet)
        assert any("evidence.visual[0]" in error for error in errors), bad
    packet = complete_packet()
    packet["evidence"]["visual"] = ["/abs/frame.png"]
    errors = MODULE.packet_errors(packet)
    assert any("relative path inside the repository" in error for error in errors)


def test_prose_cannot_masquerade_as_measurement():
    packet = complete_packet()
    packet["metrics"]["numerical"]["max_penetration_m"] = "not measured yet"
    errors = MODULE.packet_errors(packet)
    assert any("prose where a measurement is expected" in error for error in errors)
    # Semantic annotation keys stay allowed.
    packet = complete_packet()
    packet["metrics"]["numerical"]["penetration_semantics"] = "clamped at zero"
    packet["metrics"]["numerical"]["clamp_note"] = "runtime clamps depth"
    assert MODULE.packet_errors(packet) == []


def test_configuration_placeholder_objects_fail():
    packet = complete_packet()
    packet["configuration"]["resolved"] = {"placeholder": None}
    errors = MODULE.packet_errors(packet)
    assert any("null values" in error for error in errors)
    assert any("no recognizable" in error for error in errors)


def test_nonstandard_json_constants_fail_at_load(tmp_path):
    packet = complete_packet()
    packet["host"] = {"skew": float("nan")}
    plan_dir = _write_tree(tmp_path, packet=None, negative={"schema": "x"})
    (plan_dir / "evidence" / "packet.json").write_text(
        json.dumps(packet, allow_nan=True), encoding="utf-8"
    )
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"]["dart7"]["status"] = "in-progress"
    manifest["claims"][0]["lanes"]["dart7"]["evidence"] = ["evidence/packet.json"]
    (plan_dir / "claims-manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("unreadable JSON" in error for error in errors)


def test_lane_evidence_paths_are_canonicalized(tmp_path):
    packet = copy.deepcopy(complete_packet())
    manifest = _minimal_manifest(["CT-001", "CT-002"])
    first = manifest["claims"][0]["lanes"]["dart7"]
    second = manifest["claims"][1]["lanes"]["dart7"]
    first["status"] = "in-progress"
    first["evidence"] = ["evidence/packet.json"]
    second["status"] = "in-progress"
    second["evidence"] = ["evidence/./packet.json"]
    plan_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    (plan_dir / "citation-claim-corpus.md").write_text(
        "| CT-001 | claim |\n| CT-002 | claim |\n", encoding="utf-8"
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("one packet has one owner" in error for error in errors)


def test_lane_evidence_paths_cannot_escape(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart7"]
    lane["status"] = "in-progress"
    lane["evidence"] = ["../outside.json"]
    plan_dir = _write_tree(
        tmp_path, packet=None, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(plan_dir)
    assert any("escapes the plan directory" in error for error in errors)


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
    (tmp_path / "somedir.csv").mkdir()
    packet["evidence"]["raw_paths"] = ["somedir.csv"]
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


def test_scene_parameters_are_required():
    packet = complete_packet()
    del packet["scene"]["parameters"]
    errors = MODULE.packet_errors(packet)
    assert any("binds nothing" in error for error in errors)


def test_boolean_metric_leaves_are_valid_but_odd_types_fail():
    packet = complete_packet()
    packet["metrics"]["physical"]["pyramid_signature"] = True
    assert MODULE.packet_errors(packet) == []


def test_reviewer_identities_are_normalized_before_counting(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [
        _bound_pass(packet, "reviewer-a"),
        _bound_pass(packet, "  Reviewer-A "),
    ]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"][LANE]
    lane["status"] = "closed"
    lane["disposition"] = "unresolved"
    lane["evidence"] = ["evidence/packet.json"]
    tree_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(tree_dir)
    assert any("INDEPENDENT" in error for error in errors)


def test_visual_entries_must_be_media_artifacts():
    packet = complete_packet()
    packet["evidence"]["visual"] = ["CHANGELOG.md"]
    errors = MODULE.packet_errors(packet)
    assert any("not a recognized visual media artifact" in error for error in errors)


def test_review_entries_require_a_passing_verdict():
    packet = complete_packet()
    entry = _bound_pass(packet, "first")
    del entry["verdict"]
    packet["review"]["passes"] = [entry]
    errors = MODULE.packet_errors(packet)
    assert any("verdict 'pass'" in error for error in errors)
    entry["verdict"] = "fail"
    errors = MODULE.packet_errors(packet)
    assert any("verdict 'pass'" in error for error in errors)


def test_identity_values_must_be_non_empty():
    for bad in ("", [], False, {}):
        packet = complete_packet()
        packet["configuration"]["resolved"] = {"solver": bad}
        errors = MODULE.packet_errors(packet)
        assert any("no recognizable" in error for error in errors), bad


def test_measurement_window_placeholders_fail():
    for bad in (True, 1, [None], {"a": None}, {"start_s": 2.0, "end_s": 1.0}):
        packet = complete_packet()
        packet["ensemble"]["measurement_window"] = bad
        errors = MODULE.packet_errors(packet)
        assert any("measurement_window" in error for error in errors), bad
    packet = complete_packet()
    packet["ensemble"]["measurement_window"] = {
        "warmup_steps": 250,
        "continuation_steps": 100,
    }
    assert MODULE.packet_errors(packet) == []
    packet["ensemble"]["measurement_window"] = {"foo": 1.0}
    errors = MODULE.packet_errors(packet)
    assert any("must name its bounds" in error for error in errors)
    packet["ensemble"]["measurement_window"] = "full 1 s horizon"
    errors = MODULE.packet_errors(packet)
    assert any("measurement_window" in error for error in errors)


def test_visual_artifacts_are_verified_by_content(tmp_path):
    fake = tmp_path / "design"
    fake.mkdir()
    (fake / "capture.png").write_text("not an image", encoding="utf-8")
    packet = complete_packet()
    packet["evidence"]["visual"] = ["capture.png"]
    errors = MODULE.packet_errors(packet, base_dir=fake)
    assert any("structurally complete" in error for error in errors)
    png = b"\x89PNG\r\n\x1a\n" + bytes.fromhex(
        "0000000d49484452000000010000000108060000001f15c489"
        "0000000a49444154789c63000100000500010d0a2db4"
        "0000000049454e44ae426082"
    )
    (fake / "real.png").write_bytes(png)
    packet["evidence"]["visual"] = ["real.png"]
    packet["evidence"]["artifact_digests"] = {
        "real.png": "sha256:" + hashlib.sha256(png).hexdigest()
    }
    assert MODULE.packet_errors(packet, base_dir=fake) == []


def test_not_applicable_lane_cannot_conclude(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"][LANE]
    lane["status"] = "not-applicable"
    lane["reason"] = "does not apply here"
    lane["disposition"] = "fixed"
    lane["evidence"] = ["evidence/packet.json"]
    errors = MODULE.manifest_errors(manifest, ["CT-001"])
    assert any("concludes nothing" in error for error in errors)
    assert any("must not hold" in error for error in errors)


def test_metadata_keys_are_not_identities():
    packet = complete_packet()
    packet["configuration"]["resolved"] = {"method_note": "not measured"}
    errors = MODULE.packet_errors(packet)
    assert any("no recognizable" in error for error in errors)


def test_raw_rows_need_measurement_content():
    packet = complete_packet()
    packet["evidence"]["raw_rows"] = [{"note": "pending"}]
    errors = MODULE.packet_errors(packet)
    assert any("metadata-only record" in error for error in errors)
    packet["evidence"]["raw_rows"] = [
        {"lateral_drift_m": 1.5e-3, "trajectory_sha256": "f" * 64}
    ]
    assert MODULE.packet_errors(packet) == []


def test_metric_group_needs_a_real_measurement():
    packet = complete_packet()
    packet["metrics"]["numerical"] = {"method": "manual", "note": "not measured"}
    errors = MODULE.packet_errors(packet)
    assert any("only semantic annotations" in error for error in errors)


def test_identity_tokens_are_whole_words():
    packet = complete_packet()
    packet["configuration"]["resolved"] = {"methodology": "pending"}
    errors = MODULE.packet_errors(packet)
    assert any("no recognizable" in error for error in errors)


def test_asserted_repeats_need_recorded_verification():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats_identical"]
    errors = MODULE.packet_errors(packet)
    assert any("did not verify bit-identical" in error for error in errors)


def test_fetch_hint_must_be_the_durable_pr_ref_command():
    packet = complete_packet()
    packet["target"]["fetch_hint"] = "not a command"
    errors = MODULE.packet_errors(packet)
    assert any("runnable durable PR-ref" in error for error in errors)
    packet["target"]["fetch_hint"] = "git fetch origin pull/999/head"
    errors = MODULE.packet_errors(packet)
    assert any("runnable durable PR-ref" in error for error in errors)


def test_signature_only_media_is_rejected(tmp_path):
    root = tmp_path / "design"
    root.mkdir()
    (root / "stub.png").write_bytes(b"\x89PNG\r\n\x1a\n")
    packet = complete_packet()
    packet["evidence"]["visual"] = ["stub.png"]
    errors = MODULE.packet_errors(packet, base_dir=root)
    assert any("structurally complete" in error for error in errors)


def test_identity_placeholder_values_are_rejected():
    for bad in ("unknown", "n/a", "not measured", "pending"):
        packet = complete_packet()
        packet["configuration"]["resolved"] = {"solver": bad}
        errors = MODULE.packet_errors(packet)
        assert any("no recognizable" in error for error in errors), bad


def test_fetch_hint_suffix_or_prefix_variants_fail():
    for bad in (
        "git fetch origin pull/3445/head",
        "git fetch origin pull/3445/head; echo no-checkout",
        "git fetch origin pull/3445/head-wrong && git checkout <target.commit>",
    ):
        packet = complete_packet()
        packet["target"]["fetch_hint"] = bad
        errors = MODULE.packet_errors(packet)
        assert any("runnable durable PR-ref" in error for error in errors), bad


def test_negative_control_sidecar_pins_each_seeded_defect(tmp_path):
    tree_dir = _write_tree(tmp_path, packet=None, negative={"schema": "x"})
    sidecar = (
        tree_dir / "evidence" / "negative-controls" / "incomplete.expected-errors.json"
    )
    sidecar.write_text(
        json.dumps(["an error text that no check produces"]), encoding="utf-8"
    )
    errors = MODULE.validate_tree(tree_dir)
    assert any("seeded defect no longer detected" in error for error in errors)
    sidecar.unlink()
    errors = MODULE.validate_tree(tree_dir)
    assert any("no .expected-errors.json sidecar" in error for error in errors)


def test_fetch_hint_must_check_out_the_target_commit():
    packet = complete_packet()
    packet["target"]["fetch_hint"] = (
        "git fetch origin pull/3445/head && git checkout " + "1" * 40
    )
    errors = MODULE.packet_errors(packet)
    assert any("must reproduce THIS packet's target" in error for error in errors)


def test_orphan_sidecar_does_not_count_as_a_control(tmp_path):
    tree_dir = _write_tree(tmp_path, packet=None, negative={"schema": "x"})
    (tree_dir / "evidence" / "negative-controls" / "incomplete.json").unlink()
    errors = MODULE.validate_tree(tree_dir)
    assert any("at least one intentionally incomplete" in error for error in errors)


def test_step_window_bounds_must_be_sane_integers():
    packet = complete_packet()
    packet["ensemble"]["measurement_window"] = {
        "warmup_steps": -1.5,
        "continuation_steps": -2,
    }
    errors = MODULE.packet_errors(packet)
    assert any("non-negative integers" in error for error in errors)


def test_commands_must_be_the_reproducible_pixi_form():
    for bad in ("pending", "echo success", "bash -c 'anything'"):
        packet = complete_packet()
        packet["evidence"]["commands"] = [bad]
        errors = MODULE.packet_errors(packet)
        assert any("reproducible repository form" in error for error in errors), bad
    packet = complete_packet()
    packet["evidence"]["commands"] = [
        "pixi run build",
        "PYTHONPATH=build/x pixi run python scripts/write.py",
    ]
    assert MODULE.packet_errors(packet) == []
    packet["evidence"]["commands"] = [
        "PYTHONPATH=build/x pixi run python scripts/write.py"
    ]
    errors = MODULE.packet_errors(packet)
    assert any("must include the build step" in error for error in errors)


def test_bookkeeping_only_rows_fail():
    packet = complete_packet()
    packet["evidence"]["raw_rows"] = [{"seed": 1, "trajectory_sha256": "e" * 64}]
    errors = MODULE.packet_errors(packet)
    assert any("beyond bookkeeping" in error for error in errors)


def test_repeats_need_hash_bearing_evidence():
    packet = complete_packet()
    packet["evidence"]["raw_rows"] = [{"angle_deg": 0.0, "lateral_drift_m": 0.5}]
    errors = MODULE.packet_errors(packet)
    assert any(
        "binding the repeats to recorded trajectories" in error for error in errors
    )


def test_open_lane_disposition_must_match_packet(tmp_path):
    packet = copy.deepcopy(complete_packet())  # result.disposition: unresolved
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"][LANE]
    lane["status"] = "in-progress"
    lane["disposition"] = "fixed"
    lane["evidence"] = ["evidence/packet.json"]
    tree_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(tree_dir)
    assert any(
        "cannot publish a conclusion its evidence does not support" in error
        for error in errors
    )


def test_commands_with_shell_tails_fail():
    for bad in (
        "pixi run test; false",
        "pixi run test && rm -rf /",
        "pixi run test | tee log",
        "pixi run test `id`",
    ):
        packet = complete_packet()
        packet["evidence"]["commands"] = [bad]
        errors = MODULE.packet_errors(packet)
        assert any("reproducible repository form" in error for error in errors), bad


def test_duplicate_json_keys_fail_at_load(tmp_path):
    tree_dir = _write_tree(tmp_path, packet=None, negative={"schema": "x"})
    (tree_dir / "evidence" / "packet.json").write_text(
        '{"schema": "a", "schema": "b"}', encoding="utf-8"
    )
    manifest = _minimal_manifest(["CT-001"])
    manifest["claims"][0]["lanes"][LANE]["status"] = "in-progress"
    manifest["claims"][0]["lanes"][LANE]["evidence"] = ["evidence/packet.json"]
    (tree_dir / "claims-manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    errors = MODULE.validate_tree(tree_dir)
    assert any("duplicate JSON object key" in error for error in errors)


def test_prose_raw_paths_fail(tmp_path):
    (tmp_path / "AGENTS.md").write_text("prose", encoding="utf-8")
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["AGENTS.md"]
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any("must name a raw-data artifact" in error for error in errors)


def test_placeholder_hash_values_do_not_bind_repeats():
    packet = complete_packet()
    packet["evidence"]["raw_rows"] = [{"lateral_drift_m": 0.5, "hash": "pending"}]
    errors = MODULE.packet_errors(packet)
    assert any(
        "binding the repeats to recorded trajectories" in error for error in errors
    )


def test_placeholder_source_urls_fail():
    packet = complete_packet()
    packet["source"]["url"] = "pending"
    errors = MODULE.packet_errors(packet)
    assert any("retrievable http(s) URL" in error for error in errors)


def test_commands_with_embedded_newlines_fail():
    for bad in ("pixi run check\nfalse", "pixi run check\necho hacked\n"):
        packet = complete_packet()
        packet["evidence"]["commands"] = [bad]
        errors = MODULE.packet_errors(packet)
        assert any("reproducible repository form" in error for error in errors), bad


def test_large_repeat_claims_need_per_repeat_hash_lists():
    packet = complete_packet()
    packet["ensemble"]["deterministic_repeats"] = 1000
    errors = MODULE.packet_errors(packet)
    assert any("must show their repeats" in error for error in errors)
    packet["evidence"]["repeat_hashes_sha256"] = ["a" * 64] * 1000
    assert MODULE.packet_errors(packet) == []


def test_metadata_only_scene_parameters_fail():
    packet = complete_packet()
    params = {"note": "pending"}
    packet["scene"]["parameters"] = params
    packet["scene"]["digest"] = (
        "sha256:"
        + hashlib.sha256(
            json.dumps(params, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    )
    errors = MODULE.packet_errors(packet)
    assert any("metadata-only parameters" in error for error in errors)


def test_prose_inside_raw_data_files_fails(tmp_path):
    (tmp_path / "rows.csv").write_text("totally unstructured prose", encoding="utf-8")
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["rows.csv"]
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "does not parse as its claimed raw-data format" in error for error in errors
    )


def test_declared_sweep_points_need_matching_rows():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    errors = MODULE.packet_errors(packet)
    assert any(
        "every declared point needs at least one recorded sample" in e for e in errors
    )


def test_swapped_artifacts_invalidate_the_packet(tmp_path):
    (tmp_path / "real.csv").write_text("a,b\n1,2\n", encoding="utf-8")
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["real.csv"]
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any("artifact_digests must map" in e for e in errors)
    packet["evidence"]["artifact_digests"] = {"real.csv": "sha256:" + "0" * 64}
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any("does not match the referenced file's bytes" in e for e in errors)


def test_sweep_points_must_be_observed_by_rows():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    packet["evidence"]["raw_rows"] = [
        {"angle_deg": 0.0, "lateral_drift_m": 0.0, "trajectory_sha256": "d" * 64},
        {"angle_deg": 0.0, "lateral_drift_m": 0.0, "trajectory_sha256": "d" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("has no matching row" in error for error in errors)


def test_declared_seeds_must_be_observed_by_rows():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["seeds"] = [7, 11]
    packet["evidence"]["raw_rows"] = [
        {"seed": 7, "lateral_drift_m": 0.1, "trajectory_sha256": "d" * 64},
        {"seed": 7, "lateral_drift_m": 0.1, "trajectory_sha256": "d" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("has no row recording it" in error for error in errors)


def test_structurally_empty_json_artifacts_fail(tmp_path):
    (tmp_path / "rows.json").write_text('"just prose"', encoding="utf-8")
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["rows.json"]
    packet["evidence"]["artifact_digests"] = {
        "rows.json": "sha256:"
        + hashlib.sha256((tmp_path / "rows.json").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "carries no numeric or boolean measurement content" in error for error in errors
    )


def test_scalar_sweep_points_are_rejected():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [0.0, 15.0]
    packet["evidence"]["raw_rows"] = [
        {"angle_deg": 0.0, "lateral_drift_m": 0.0, "trajectory_sha256": "d" * 64},
        {"angle_deg": 15.0, "lateral_drift_m": 0.0, "trajectory_sha256": "d" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("naming its coordinates" in error for error in errors)


def test_sweep_ensembles_require_rows(tmp_path):
    (tmp_path / "rows.csv").write_text("a,b\n1,2\n", encoding="utf-8")
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["rows.csv"]
    packet["evidence"]["artifact_digests"] = {
        "rows.csv": "sha256:"
        + hashlib.sha256((tmp_path / "rows.csv").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any("requires inline evidence.raw_rows" in error for error in errors)


def test_host_provenance_is_required():
    packet = complete_packet()
    del packet["host"]
    errors = MODULE.packet_errors(packet)
    assert any("missing required top-level keys" in error for error in errors)
    packet = complete_packet()
    del packet["host"]["performance_valid"]
    errors = MODULE.packet_errors(packet)
    assert any(
        "performance_valid must be an explicit boolean" in error for error in errors
    )


def test_metadata_only_csv_fails(tmp_path):
    (tmp_path / "rows.csv").write_text("note,status\nfoo,pending\n", encoding="utf-8")
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["rows.csv"]
    packet["evidence"]["artifact_digests"] = {
        "rows.csv": "sha256:"
        + hashlib.sha256((tmp_path / "rows.csv").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "carries no numeric or boolean measurement content" in error for error in errors
    )


def test_signature_only_webp_and_mp4_fail(tmp_path):
    root = tmp_path / "d"
    root.mkdir()
    (root / "clip.webp").write_bytes(
        b"RIFF" + b"\x00\x01\x02\x03" + b"WEBP" + b"p" * 60
    )
    packet = complete_packet()
    packet["evidence"]["visual"] = ["clip.webp"]
    packet["evidence"]["artifact_digests"] = {
        "clip.webp": "sha256:"
        + hashlib.sha256((root / "clip.webp").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=root)
    assert any("structurally complete" in error for error in errors)


def test_unicode_variant_reviewers_count_once(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [
        _bound_pass(packet, "Jos\u00e9"),
        _bound_pass(packet, "Jose\u0301"),
    ]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"][LANE]
    lane["status"] = "closed"
    lane["disposition"] = "unresolved"
    lane["evidence"] = ["evidence/packet.json"]
    tree_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(tree_dir)
    assert any("INDEPENDENT" in error for error in errors)


def test_stub_binary_artifacts_fail(tmp_path):
    import zipfile as _zip

    empty_zip = tmp_path / "empty.npz"
    with _zip.ZipFile(empty_zip, "w"):
        pass
    stub_npy = tmp_path / "stub.npy"
    stub_npy.write_bytes(b"\x93NUMPY" + b"x" * 100)
    for name in ("empty.npz", "stub.npy"):
        packet = complete_packet()
        del packet["evidence"]["raw_rows"]
        packet["evidence"]["raw_paths"] = [name]
        packet["evidence"]["artifact_digests"] = {
            name: "sha256:" + hashlib.sha256((tmp_path / name).read_bytes()).hexdigest()
        }
        errors = MODULE.packet_errors(packet, base_dir=tmp_path)
        assert any(
            "does not parse as its claimed raw-data format" in error for error in errors
        ), name


def test_stub_webm_fails(tmp_path):
    root = tmp_path / "d"
    root.mkdir()
    (root / "clip.webm").write_bytes(b"\x1a\x45\xdf\xa3" + b"p" * 96)
    packet = complete_packet()
    packet["evidence"]["visual"] = ["clip.webm"]
    packet["evidence"]["artifact_digests"] = {
        "clip.webm": "sha256:"
        + hashlib.sha256((root / "clip.webm").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=root)
    assert any("structurally complete" in error for error in errors)


def test_raw_artifacts_under_evidence_raw_are_not_packets(tmp_path):
    tree_dir = _write_tree(tmp_path, packet=None, negative={"schema": "x"})
    raw_dir = tree_dir / "evidence" / "raw"
    raw_dir.mkdir()
    (raw_dir / "rows.json").write_text(
        json.dumps([{"lateral_drift_m": 0.5}]), encoding="utf-8"
    )
    errors = MODULE.validate_tree(tree_dir)
    assert not any("rows.json" in error for error in errors)
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"][LANE]
    lane["status"] = "in-progress"
    lane["evidence"] = ["evidence/raw/rows.json"]
    (tree_dir / "claims-manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    errors = MODULE.validate_tree(tree_dir)
    assert any("holds raw artifacts, not packets" in error for error in errors)


def test_overlapping_sweep_points_need_distinct_rows():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [
        {"angle_deg": 0.0},
        {"angle_deg": 0.0, "detector": "fcl"},
    ]
    packet["evidence"]["raw_rows"] = [
        {
            "angle_deg": 0.0,
            "detector": "fcl",
            "lateral_drift_m": 0.1,
            "trajectory_sha256": "d" * 64,
        },
        {"unrelated_metric": 1.0, "trajectory_sha256": "e" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("DISTINCT" in error and "own" in error for error in errors)


def test_empty_npy_arrays_fail(tmp_path):
    header = b"{'descr': '<f8', 'fortran_order': False, 'shape': (0,), }"
    header += b" " * (63 - len(header) % 64) + b"\n"
    payload = b"\x93NUMPY\x01\x00" + len(header).to_bytes(2, "little") + header
    (tmp_path / "empty.npy").write_bytes(payload)
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["empty.npy"]
    packet["evidence"]["artifact_digests"] = {
        "empty.npy": "sha256:"
        + hashlib.sha256((tmp_path / "empty.npy").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "does not parse as its claimed raw-data format" in error for error in errors
    )


def test_npz_members_must_be_valid_arrays(tmp_path):
    import zipfile as _zip

    bad = tmp_path / "bad.npz"
    with _zip.ZipFile(bad, "w") as archive:
        archive.writestr("member.npy", b"not an array")
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["bad.npz"]
    packet["evidence"]["artifact_digests"] = {
        "bad.npz": "sha256:" + hashlib.sha256(bad.read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "does not parse as its claimed raw-data format" in error for error in errors
    )


def test_seeds_need_distinct_rows():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["seeds"] = [7, 11]
    packet["evidence"]["raw_rows"] = [
        {
            "seed": 7,
            "other_seed": 11,
            "metric": 1.0,
            "trajectory_sha256": "d" * 64,
        },
        {"metric": 2.0, "trajectory_sha256": "e" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("DISTINCT rows" in error for error in errors)


def test_zero_duration_windows_fail():
    for bad in (
        {"start_s": 1.0, "end_s": 1.0},
        {"start_s": -2.0, "end_s": -1.0},
    ):
        packet = complete_packet()
        packet["ensemble"]["measurement_window"] = bad
        errors = MODULE.packet_errors(packet)
        assert any(
            "non-empty" in error and "interval" in error for error in errors
        ), bad


def test_invalid_host_forbids_measured_performance():
    packet = complete_packet()
    packet["metrics"]["performance"] = {
        "method": "wall clock",
        "step_time_ms": 1.2,
    }
    errors = MODULE.packet_errors(packet)
    assert any("uncontrolled host" in error for error in errors)


def test_parquet_stub_fails(tmp_path):
    (tmp_path / "rows.parquet").write_bytes(b"PAR1PAR1")
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["rows.parquet"]
    packet["evidence"]["artifact_digests"] = {
        "rows.parquet": "sha256:"
        + hashlib.sha256((tmp_path / "rows.parquet").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "does not parse as its claimed raw-data format" in error for error in errors
    )


def test_all_unsupported_measured_groups_fail():
    packet = complete_packet()
    packet["metrics"]["numerical"] = {
        "method": "not actually measured",
        "value": {"status": "unsupported", "reason": "instrument absent"},
    }
    errors = MODULE.packet_errors(packet)
    assert any("dressing" in error for error in errors)


def test_string_dtype_npy_fails(tmp_path):
    header = b"{'descr': '<U12', 'fortran_order': False, 'shape': (2,), }"
    header += b" " * (63 - len(header) % 64) + b"\n"
    payload = (
        b"\x93NUMPY\x01\x00" + len(header).to_bytes(2, "little") + header + b"x" * 96
    )
    (tmp_path / "strings.npy").write_bytes(payload)
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["strings.npy"]
    packet["evidence"]["artifact_digests"] = {
        "strings.npy": "sha256:"
        + hashlib.sha256((tmp_path / "strings.npy").read_bytes()).hexdigest()
    }
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "does not parse as its claimed raw-data format" in error for error in errors
    )


def test_allocation_needs_a_valid_host_too():
    packet = complete_packet()
    packet["metrics"]["allocation"] = {"method": "counter", "allocs": 12.0}
    errors = MODULE.packet_errors(packet)
    assert any("allocation counts" in error for error in errors)


def test_null_sweep_coordinates_fail():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": None}, {"other": None}]
    packet["evidence"]["raw_rows"] = [
        {"metric": 1.0, "trajectory_sha256": "d" * 64},
        {"metric": 2.0, "trajectory_sha256": "e" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("null" in error and "coordinates" in error for error in errors)


def test_seed_fields_match_by_token():
    packet = complete_packet()
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["seeds"] = [7, 11]
    packet["evidence"]["raw_rows"] = [
        {"unseeded_metric": 7.0, "trajectory_sha256": "d" * 64},
        {"unseeded_metric": 11.0, "trajectory_sha256": "e" * 64},
    ]
    errors = MODULE.packet_errors(packet)
    assert any("has no row recording it" in error for error in errors)


def test_build_must_precede_the_evidence_command():
    packet = complete_packet()
    packet["evidence"]["commands"] = [
        "pixi run python scripts/example.py",
        "pixi run build",
    ]
    errors = MODULE.packet_errors(packet)
    assert any("BEFORE the evidence command" in error for error in errors)


def test_artifact_digests_do_not_prove_repeats():
    packet = complete_packet()
    packet["evidence"]["raw_rows"] = [{"angle_deg": 0.0, "lateral_drift_m": 0.5}]
    packet["evidence"]["artifact_digests"] = {"x.csv": "sha256:" + "a" * 64}
    errors = MODULE.packet_errors(packet)
    assert any(
        "binding the repeats to recorded trajectories" in error for error in errors
    )


def test_env_prefixed_build_commands_are_recognized():
    packet = complete_packet()
    packet["evidence"]["commands"] = [
        "DART_PARALLEL_JOBS=4 pixi run build",
        "pixi run python scripts/example.py",
    ]
    assert MODULE.packet_errors(packet) == []


def test_truncated_npy_payloads_fail(tmp_path):
    header = b"{'descr': '<f8', 'fortran_order': False, 'shape': (1000000,), }"
    header += b" " * (63 - len(header) % 64) + b"\n"
    payload = b"\x93NUMPY\x01\x00" + len(header).to_bytes(2, "little") + header + b"x"
    (tmp_path / "big.npy").write_bytes(payload)
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["big.npy"]
    packet["evidence"]["artifact_digests"] = {
        "big.npy": "sha256:"
        + hashlib.sha256((tmp_path / "big.npy").read_bytes()).hexdigest()
    }
    packet["ensemble"]["repeat_trajectory_sha256"] = "a" * 64
    errors = MODULE.packet_errors(packet, base_dir=tmp_path)
    assert any(
        "does not parse as its claimed raw-data format" in error for error in errors
    )


def test_cased_bookkeeping_keys_are_still_bookkeeping():
    for row in ({"Seed": 1}, {"run_id": 1}, {"Repeat-2": 3}):
        packet = complete_packet()
        row = dict(row)
        row["trajectory_sha256"] = "d" * 64
        packet["evidence"]["raw_rows"] = [row]
        errors = MODULE.packet_errors(packet)
        assert any("beyond bookkeeping" in error for error in errors), row
