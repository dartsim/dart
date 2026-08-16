"""Tests for the release-6.20 citation evidence validator (fail-closed).

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
DESIGN_DIR = ROOT / "docs" / "design" / "dart6_citation_driven_contact_trust"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_citation_evidence", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()
LANE = "dart6"


def complete_packet() -> dict:
    return {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-001",
        "title": "Test packet",
        "source": {"url": "https://example.org/claim", "claim": "A claim."},
        "target": {
            "branch": "release-6.20",
            "commit": "0" * 40,
            "fetch_hint": (
                "git fetch origin pull/3444/head && git checkout " + "0" * 40
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
            "deterministic_repeats_identical": True,
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
        (negative_dir / "incomplete.expected-errors.json").write_text(
            json.dumps(["missing required top-level keys"]), encoding="utf-8"
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
    # raw_paths carry no hash leaf, so the repeats claim needs one recorded
    # elsewhere; a sweep ensemble sidesteps that requirement here.
    del packet["ensemble"]["deterministic_repeats"]
    del packet["ensemble"]["deterministic_repeats_identical"]
    packet["ensemble"]["sweep"] = [{"angle_deg": 0.0}, {"angle_deg": 15.0}]
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
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "unresolved"
    lane["evidence"] = ["evidence/packet.json"]
    design_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("at least two recorded review passes" in error for error in errors)
    packet["review"]["passes"].append(_bound_pass(packet, "second"))
    (design_dir / "evidence" / "packet.json").write_text(
        json.dumps(packet), encoding="utf-8"
    )
    assert MODULE.validate_tree(design_dir) == []


def test_validate_tree_closed_lane_needs_distinct_reviewers(tmp_path):
    packet = copy.deepcopy(complete_packet())
    packet["review"]["passes"] = [
        _bound_pass(packet, "same"),
        _bound_pass(packet, "same"),
    ]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "unresolved"
    lane["evidence"] = ["evidence/packet.json"]
    design_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("INDEPENDENT" in error for error in errors)


def test_validate_tree_closed_lane_disposition_must_match_packet(tmp_path):
    packet = copy.deepcopy(complete_packet())  # result.disposition: unresolved
    packet["review"]["passes"] = [
        _bound_pass(packet, "first"),
        _bound_pass(packet, "second"),
    ]
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "closed"
    lane["disposition"] = "reproduced"
    lane["evidence"] = ["evidence/packet.json"]
    design_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(design_dir)
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
    packet["result"]["claim_boundary"] = "Changed after review."
    errors = MODULE.packet_errors(packet)
    assert any("not bound to this packet's content" in error for error in errors)


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
    design = tmp_path / "design"
    design.mkdir()
    packet = complete_packet()
    del packet["evidence"]["raw_rows"]
    packet["evidence"]["raw_paths"] = ["missing/nowhere.json"]
    errors = MODULE.packet_errors(packet, base_dir=design)
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


def test_corpus_reference_must_pin_the_canonical_owner():
    manifest = _minimal_manifest(["CT-001"])
    manifest["corpus_reference"]["path"] = "wrong/path.md"
    errors = MODULE.manifest_errors(manifest)
    assert any(
        "claim identity is owned by the DART 7 corpus" in error for error in errors
    )
    manifest = _minimal_manifest(["CT-001"])
    manifest["corpus_reference"]["branch"] = "release-6.20"
    errors = MODULE.manifest_errors(manifest)
    assert any("corpus_reference.branch" in error for error in errors)


def test_lane_evidence_paths_are_canonicalized(tmp_path):
    packet = copy.deepcopy(complete_packet())
    manifest = _minimal_manifest(["CT-001", "CT-002"])
    first = manifest["claims"][0]["lanes"]["dart6"]
    second = manifest["claims"][1]["lanes"]["dart6"]
    first["status"] = "in-progress"
    first["evidence"] = ["evidence/packet.json"]
    second["status"] = "in-progress"
    second["evidence"] = ["evidence/./packet.json"]
    design_dir = _write_tree(
        tmp_path, packet=packet, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("one packet has one owner" in error for error in errors)


def test_lane_evidence_paths_cannot_escape(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"]["dart6"]
    lane["status"] = "in-progress"
    lane["evidence"] = ["../outside.json"]
    design_dir = _write_tree(
        tmp_path, packet=None, negative={"schema": "x"}, manifest=manifest
    )
    errors = MODULE.validate_tree(design_dir)
    assert any("escapes the sidecar directory" in error for error in errors)


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


def test_non_string_lane_evidence_entry_fails(tmp_path):
    """A lane must not be closable by an entry the packet checks cannot read."""
    for bad in ({"path": "evidence/x.json"}, None, 42, True, ["evidence/x.json"]):
        manifest = _minimal_manifest(["CT-001"])
        lane = manifest["claims"][0]["lanes"]["dart6"]
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
    assert MODULE.packet_errors(packet, base_dir=fake) == []


def test_not_applicable_lane_cannot_conclude(tmp_path):
    manifest = _minimal_manifest(["CT-001"])
    lane = manifest["claims"][0]["lanes"][LANE]
    lane["status"] = "not-applicable"
    lane["reason"] = "does not apply here"
    lane["disposition"] = "fixed"
    lane["evidence"] = ["evidence/packet.json"]
    errors = MODULE.manifest_errors(manifest)
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
        "git fetch origin pull/3444/head",
        "git fetch origin pull/3444/head; echo no-checkout",
        "git fetch origin pull/3444/head-wrong && git checkout <target.commit>",
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
        "git fetch origin pull/3444/head && git checkout " + "1" * 40
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
        "PYTHONPATH=build/x pixi run python scripts/write.py"
    ]
    assert MODULE.packet_errors(packet) == []


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
