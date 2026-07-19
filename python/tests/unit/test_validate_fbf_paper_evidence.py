"""Unit coverage for the FBF paper-evidence manifest gate."""

from __future__ import annotations

import copy
import hashlib
import json
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import validate_fbf_paper_evidence as validator

MANIFEST = (
    ROOT
    / "docs"
    / "dev_tasks"
    / "fbf_exact_coulomb_friction"
    / "paper-evidence-manifest.json"
)


@pytest.fixture
def manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def _requirement(manifest: dict, requirement_id: str) -> dict:
    return next(
        requirement
        for requirement in manifest["requirements"]
        if requirement["id"] == requirement_id
    )


def _truth(manifest: dict, key: str) -> dict:
    return manifest["current_truth"][key]


@pytest.mark.parametrize(
    ("resolver_name", "leaf_kind"),
    [("_artifact_path", "file"), ("_evidence_directory", "directory")],
)
@pytest.mark.parametrize("symlink_kind", ["direct", "ancestor"])
def test_repository_evidence_paths_reject_symlink_components_before_resolution(
    tmp_path: Path,
    resolver_name: str,
    leaf_kind: str,
    symlink_kind: str,
) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    real_parent = repo / "real-parent"
    real_parent.mkdir()
    target = real_parent / "evidence"
    if leaf_kind == "file":
        target.write_text("evidence\n", encoding="utf-8")
    else:
        target.mkdir()

    if symlink_kind == "direct":
        relative = Path("evidence-link")
        (repo / relative).symlink_to(
            target, target_is_directory=leaf_kind == "directory"
        )
    else:
        relative = Path("parent-link/evidence")
        (repo / "parent-link").symlink_to(real_parent, target_is_directory=True)

    errors: list[str] = []
    resolver = getattr(validator, resolver_name)
    resolved = resolver(relative.as_posix(), "test.evidence", repo, errors)

    assert resolved is None
    assert errors == [
        "test.evidence: repository evidence paths must not contain symlinks: "
        f"{relative.as_posix()}"
    ]


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _patch_backspin_bundle_json(
    monkeypatch: pytest.MonkeyPatch, name: str, mutate
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, requested_name, location, errors):
        payload = original(bundle, requested_name, location, errors)
        if (
            payload is not None
            and requested_name == name
            and bundle is not None
            and bundle.as_posix().endswith(validator.BACKSPIN_V3_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)


def _patch_incline_bundle_json(
    monkeypatch: pytest.MonkeyPatch, name: str, mutate
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, requested_name, location, errors):
        payload = original(bundle, requested_name, location, errors)
        if (
            payload is not None
            and requested_name == name
            and bundle is not None
            and bundle.as_posix().endswith(validator.INCLINE_V1_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)


def _patch_author_card_house_bundle_json(
    monkeypatch: pytest.MonkeyPatch, name: str, mutate
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, requested_name, location, errors):
        payload = original(bundle, requested_name, location, errors)
        if (
            payload is not None
            and requested_name == name
            and bundle is not None
            and bundle.as_posix().endswith(validator.AUTHOR_CARD_HOUSE_V1_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)


def _author_card_house_truth(manifest: dict) -> dict:
    return _truth(manifest, "author_card_house_5_construction_only_v1")


def _author_card_house_requirement_map(manifest: dict) -> dict[str, dict]:
    return {
        requirement["id"]: copy.deepcopy(requirement)
        for requirement in manifest["requirements"]
        if requirement["id"] in {"fig.06", "video.06_card_house"}
    }


def _incline_requirement_map(manifest: dict) -> dict[str, dict]:
    return {
        requirement["id"]: copy.deepcopy(requirement)
        for requirement in manifest["requirements"]
        if requirement["id"] in {"fig.01", "fig.02", "video.03_incline"}
    }


def _turntable_bundle() -> Path:
    return ROOT / validator.TURNTABLE_V1_BUNDLE


def _turntable_hashes() -> dict[str, str]:
    hashes = {}
    for key, relative in validator.TURNTABLE_V1_ARTIFACT_TARGETS.items():
        path = ROOT / relative
        if not path.is_file():
            pytest.skip(f"turntable finalization has not produced {relative}")
        hashes[key] = _digest(path)
    return hashes


def _turntable_truth_payload(*, finalized: bool = True) -> dict:
    return {
        "bundle": validator.TURNTABLE_V1_BUNDLE,
        "status": "valid_author_source_pinned_nonpaper_turntable_matrix",
        "artifact_valid": True,
        "solver_contract_valid": True,
        "physical_outcome_valid": True,
        "render_binding_lane": "current_visual",
        "separate_diagnostic_lane": "paper_cpu_native",
        "cross_lane_substitution_allowed": False,
        "paper_cpu_native_all_solver_contract_valid": False,
        "paper_parity": False,
        "paper_comparable": False,
        "external_solver_parity": False,
        "approved_source_golden": False,
        "timing_verdict": None,
        "realtime_verdict": None,
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "manual_visual_outcome_validated": True,
        "trace_equivalence_to_rendered_demo": False,
        "solver_projection_equivalent": True,
        "core_solver_contact_projection_equivalent": True,
        "source_order_validated": True,
        "author_source_pinned": True,
        "author_source_commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
        "author_configuration_port": True,
        "steps": 360,
        "trace_rows_per_scenario": 361,
        "member_frames": 181,
        "group_video_frames": 181,
        "group_video_fps": 30,
        "group_dimensions": [1320, 1060],
        "group_panel_dimensions": [1344, 1076],
        "manual_inspected": True,
        "artifact_count": 58,
        "artifact_hashes": _turntable_hashes() if finalized else {},
        "claim_scope": validator.TURNTABLE_V1_CLAIM_SCOPE,
        "claim_boundary": validator.TURNTABLE_V1_CLAIM_BOUNDARY,
    }


def _patch_turntable_bundle_json(
    monkeypatch: pytest.MonkeyPatch, name: str, mutate
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, requested_name, location, errors):
        payload = original(bundle, requested_name, location, errors)
        if (
            payload is not None
            and requested_name == name
            and bundle is not None
            and bundle.as_posix().endswith(validator.TURNTABLE_V1_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)


def _turntable_requirement_map(manifest: dict) -> dict[str, dict]:
    by_id = {
        requirement["id"]: copy.deepcopy(requirement)
        for requirement in manifest["requirements"]
    }
    expected = {
        "fig.04": {
            "exact_fixture": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
            "trace_csv": (
                f"{validator.TURNTABLE_V1_BUNDLE}/traces/current_visual/"
                "turntable_author_mu_0_5_omega_2.csv"
            ),
            "still_image": (
                f"{validator.TURNTABLE_V1_BUNDLE}/groups/turntable_author/panel.png"
            ),
        },
        "video.04_turntable": {
            "exact_fixture": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
            "video_clip": (
                f"{validator.TURNTABLE_V1_BUNDLE}/groups/turntable_author/clip.mp4"
            ),
        },
    }
    for requirement_id, deliverables in expected.items():
        requirement = by_id[requirement_id]
        requirement["status"] = "partial"
        requirement["fallback_count"] = 0
        by_kind = {
            item["kind"]: item
            for item in requirement["deliverables"]
            if isinstance(item, dict) and "kind" in item
        }
        for kind, path in deliverables.items():
            by_kind[kind] = {"kind": kind, "path": path, "validated": True}
        requirement["deliverables"] = list(by_kind.values())
        for kind in (
            "external_baseline",
            "approved_golden",
            "golden_diff",
            "capture_sidecar",
            "claim_map",
        ):
            if kind in by_kind:
                by_kind[kind]["validated"] = False
    return by_id


def _patch_turntable_verification_for_group_test(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original = validator._read_bundle_json
    bundle = _turntable_bundle()

    def read_bundle(root, name, location, errors):
        if name == "verification.json":
            return {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "verification",
                "pass": True,
                "results": [
                    {"metadata_path": str(bundle / capture_id / "metadata.json")}
                    for capture_id in validator.TURNTABLE_V1_CAPTURE_IDS
                ],
                "group_outputs": [{"group_id": "turntable_author", "pass": True}],
            }
        return original(root, name, location, errors)

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)


def _isolated_complete_teaser(manifest: dict, repo_root: Path) -> tuple[dict, dict]:
    data = copy.deepcopy(manifest)
    evidence_path = repo_root / "evidence.txt"
    evidence_path.write_text("existing partial evidence\n", encoding="utf-8")
    for requirement in data["requirements"]:
        requirement["deliverables"] = []
        for evidence in requirement["current_evidence"]:
            evidence["path"] = evidence_path.name

    png = (
        b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01"
        b"\x00\x00\x00\x01\x08\x06\x00\x00\x00\x1f\x15\xc4\x89"
        b"\x00\x00\x00\rIDAT\x08\xd7c\xf8\xcf\xc0\xf0\x1f\x00\x05"
        b"\x00\x01\xff\x89\x99=\x1d\x00\x00\x00\x00IEND\xaeB`\x82"
    )
    (repo_root / "still.png").write_bytes(png)
    (repo_root / "golden.png").write_bytes(png)
    _write_json(
        repo_root / "golden-diff.json",
        {
            "schema_version": "dart.image_diff/v1",
            "requirement_ids": ["teaser"],
            "different_pixels": 0,
            "total_pixels": 1,
            "difference_fraction": 0.0,
            "allowed_difference_fraction": 0.0,
            "pass": True,
        },
    )
    _write_json(
        repo_root / "capture.json",
        {
            "schema_version": "dart.demos_headless_timeline/v1",
            "requirement_ids": ["teaser"],
            "total_steps": 1,
            "completed_steps": 1,
            "actions": [],
            "shots": [{"step": 1, "success": True}],
            "solver_contract": {
                "schema_version": validator.FULL_TRAJECTORY_SOLVER_SCHEMA,
                "exact_fbf_required": True,
                "tolerance": validator.PAPER_RESIDUAL_TOLERANCE,
                "steps": [
                    {
                        "step": 1,
                        "contacts": 1,
                        "constraint_group_count": 1,
                        "groups": [
                            {
                                "group_index": 0,
                                "contact_count": 1,
                                "solver": "ExactCoulombFbfConstraintSolver",
                                "status": "success",
                                "fbf_status": "success",
                                "residual": 5e-7,
                                "exact_failures": 0,
                                "boxed_lcp_fallbacks": 0,
                            }
                        ],
                    }
                ],
                "summary": {
                    "steps_observed": 1,
                    "groups_observed": 1,
                    "fallback_count": 0,
                    "exact_failure_count": 0,
                    "maximum_residual": 5e-7,
                    "status": "success",
                    "pass": True,
                },
            },
        },
    )
    teaser_claim = _requirement(data, "teaser")["claim"]
    _write_json(
        repo_root / "claim-map.json",
        {
            "schema_version": "dart.fbf_paper_claim_map/v1",
            "requirement_ids": ["teaser"],
            "manifest_claim": teaser_claim,
            "status": "complete",
            "remaining_blockers": [],
            "claims": [
                {
                    "claim": teaser_claim,
                    "support": "The listed runtime and media artifacts support it.",
                }
            ],
        },
    )
    _write_json(
        repo_root / "citation.json",
        {
            "schema_version": "dart.fbf_citation_record/v1",
            "requirement_ids": ["teaser"],
            "status": "complete",
            "sources": [
                {
                    "url": "https://example.test/paper",
                    "locator": "teaser",
                    "verified": True,
                }
            ],
        },
    )

    teaser = _requirement(data, "teaser")
    teaser["status"] = "complete"
    teaser["fallback_count"] = 0
    teaser["blockers"] = []
    teaser["capture_plan"]["blockers"] = []
    paths = {
        "still_image": repo_root / "still.png",
        "approved_golden": repo_root / "golden.png",
        "golden_diff": repo_root / "golden-diff.json",
        "capture_sidecar": repo_root / "capture.json",
        "claim_map": repo_root / "claim-map.json",
        "citation_record": repo_root / "citation.json",
    }
    teaser["deliverables"] = [
        {
            "kind": kind,
            "path": path.name,
            "validated": True,
            "sha256": _digest(path),
        }
        for kind, path in paths.items()
    ]
    return data, teaser


def test_repository_manifest_covers_the_canonical_source_set(
    manifest: dict,
) -> None:
    assert validator.validate_manifest(manifest, ROOT) == []
    ids = [requirement["id"] for requirement in manifest["requirements"]]
    assert ids == list(validator.CANONICAL_REQUIREMENT_IDS)
    assert manifest["overall_status"] == "partial"
    assert all(
        requirement["status"] != "complete" for requirement in manifest["requirements"]
    )


def test_current_truth_rejects_a_nonexistent_current_bundle(manifest: dict) -> None:
    card = _truth(manifest, "card_house_manifold_sensitivity_v2_nonpaper")
    card["bundle"] = (
        "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
        "missing_card_v2_bundle"
    )

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "card_house_manifold_sensitivity_v2_nonpaper.bundle: expected current path"
        in error
        for error in errors
    )
    assert any("current evidence bundle does not exist" in error for error in errors)


def test_current_truth_rejects_an_artifact_hash_mismatch(manifest: dict) -> None:
    card = _truth(manifest, "card_house_manifold_sensitivity_v2_nonpaper")
    card["artifact_hashes"]["summary.json"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "card_house_manifold_sensitivity_v2_nonpaper.artifact_hashes['summary.json']"
        in error
        and "digest mismatch" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: _truth(data, "card_house_manifold_sensitivity_v2_nonpaper")[
                "compact"
            ].update({"strict_trajectory_valid": True}),
            "compact.strict_trajectory_valid: expected False, got True",
        ),
        (
            lambda data: _truth(
                data, "card_house_manifold_sensitivity_v2_nonpaper"
            ).update({"physical_verdict": "passed"}),
            "physical_verdict: expected None, got 'passed'",
        ),
    ],
)
def test_current_truth_rejects_promoted_card_verdicts(
    manifest: dict, mutate, message: str
) -> None:
    mutate(manifest)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("key", "stale_bundle", "current_bundle"),
    [
        (
            "literal_wedge_crown_impact_v1_nonpaper",
            (
                "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
                "fig07_arch25_literal_impact_v1_negative_final_v5"
            ),
            validator.IMPACT_V7_BUNDLE,
        ),
        (
            "literal_arch_101_v1_nonpaper",
            (
                "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
                "fig08_arch101_literal_v1_negative_final_v3"
            ),
            validator.ARCH101_V5_BUNDLE,
        ),
    ],
)
def test_current_truth_rejects_superseded_negative_bundle_paths(
    manifest: dict, key: str, stale_bundle: str, current_bundle: str
) -> None:
    _truth(manifest, key)["bundle"] = stale_bundle

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{key}.bundle: expected current path {current_bundle!r}" in error
        for error in errors
    )


def test_prior_source_truth_is_bound_to_the_repository_archive(manifest: dict) -> None:
    prior = _truth(manifest, "prior_source_strict_card_paper_cpu")
    prior["artifact_hashes"]["metadata.json"] = "f" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "prior_source_strict_card_paper_cpu.artifact_hashes['metadata.json']" in error
        and "digest mismatch" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("requested_steps", 601, "invocations[0].expected_rows"),
        ("completed_steps", 88, "invocations[0].rows"),
        ("contact_range", [91, 155], "contact_range: bound raw range drifted"),
        ("contact_range", "malformed", "summary[0].min_contacts"),
        ("outer_cap_per_exact_group", 199, "raw.max_outer_iterations"),
        ("tolerance", 2e-6, "tolerance: bound raw value drifted"),
        ("tolerance", 10**400, "tolerance: bound raw value drifted"),
        ("terminal_residual", 0.0, "terminal_residual: bound raw value drifted"),
        (
            "mean_step_ms_before_failure",
            0.0,
            "mean_step_ms_before_failure: bound raw mean drifted",
        ),
        ("exact_fbf_failures", 2, "exact_fbf_failures: bound raw total drifted"),
        ("accepted_cap_steps", 0, "accepted_cap_steps: bound raw total drifted"),
        (
            "original_output_path",
            "/forged/path",
            "original_output_path: expected "
            "'/tmp/fbf_cpu_paper_postreview_20260712_card600'",
        ),
    ],
)
def test_prior_source_semantic_claims_are_bound_to_archived_rows(
    manifest: dict, field: str, value, message: str
) -> None:
    _truth(manifest, "prior_source_strict_card_paper_cpu")[field] = value

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


def test_cpu_truth_rejects_a_noncurrent_or_missing_bundle(manifest: dict) -> None:
    cpu = _truth(manifest, "literal_wedge_exact_dynamics_nonpaper")
    cpu["bundle"] = (
        "docs/dev_tasks/fbf_exact_coulomb_friction/assets/dart_cpu_evidence/"
        "missing_native25_bundle"
    )

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "literal_wedge_exact_dynamics_nonpaper.bundle: expected current path" in error
        for error in errors
    )
    assert any("current evidence bundle does not exist" in error for error in errors)


def test_cpu_truth_rejects_an_artifact_hash_mismatch(manifest: dict) -> None:
    cpu = _truth(manifest, "literal_wedge_exact_dynamics_nonpaper")
    cpu["artifact_hashes"]["summary.json"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "literal_wedge_exact_dynamics_nonpaper.artifact_hashes['summary.json']" in error
        and "digest mismatch" in error
        for error in errors
    )


def test_current_small_cpu_truth_rejects_a_noncurrent_bundle(manifest: dict) -> None:
    current = _truth(manifest, "current_small_paper_cpu_v1")
    current["bundle"] = (
        "docs/dev_tasks/fbf_exact_coulomb_friction/assets/dart_cpu_evidence/"
        "missing_current_small_bundle"
    )

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "current_small_paper_cpu_v1.bundle: expected current path" in error
        for error in errors
    )


def test_current_small_cpu_truth_rejects_an_artifact_hash_mismatch(
    manifest: dict,
) -> None:
    current = _truth(manifest, "current_small_paper_cpu_v1")
    current["artifact_hashes"]["raw.csv"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "current_small_paper_cpu_v1.artifact_hashes['raw.csv']" in error
        and "digest mismatch" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: _truth(data, "current_small_paper_cpu_v1").update(
                {"overall_strict_matrix_valid": True}
            ),
            "overall_strict_matrix_valid: expected False, got True",
        ),
        (
            lambda data: _truth(data, "current_small_paper_cpu_v1")["rows"][
                "incline_mu_0_5"
            ].update({"strict_solver_valid": True}),
            "summary['incline_mu_0_5'].all_solver_steps_successful",
        ),
        (
            lambda data: _truth(data, "current_small_paper_cpu_v1")["rows"][
                "turntable_mu_0_5_omega_5"
            ].update({"max_residual": 1e-7}),
            "summary['turntable_mu_0_5_omega_5'].max_residual",
        ),
        (
            lambda data: _truth(data, "current_small_paper_cpu_v1")["rows"][
                "turntable_mu_0_5_omega_5"
            ].update({"failed_processes": 0}),
            "rows['turntable_mu_0_5_omega_5'].failed_processes",
        ),
    ],
)
def test_current_small_cpu_truth_rejects_promoted_or_drifted_claims(
    manifest: dict, mutate, message: str
) -> None:
    mutate(manifest)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


def test_current_small_cpu_runtime_recheck_is_bound(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.CURRENT_SMALL_CPU_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            payload["identity_recheck"]["binary_sha256"] = "0" * 64
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "current_small_paper_cpu_v1.metadata.identity_recheck.binary_sha256" in error
        for error in errors
    )


def test_mark26_cpu_runtime_recheck_is_bound(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.CPU_EVIDENCE_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            payload["identity_recheck"]["binary_sha256"] = "0" * 64
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "literal_wedge_exact_dynamics_nonpaper.metadata.identity_recheck."
        "binary_sha256" in error
        for error in errors
    )


def test_current_small_cpu_bundle_generation_is_r7() -> None:
    assert validator.CURRENT_SMALL_CPU_BUNDLE.endswith(
        "2026-07-19_current_source_paper_cpu_small_r7"
    )


@pytest.mark.parametrize(
    ("bundle_name", "truth_key", "message"),
    [
        (
            "CPU_EVIDENCE_BUNDLE",
            "literal_wedge_exact_dynamics_nonpaper",
            "invocations[0].command[0]",
        ),
        (
            "CURRENT_SMALL_CPU_BUNDLE",
            "current_small_paper_cpu_v1",
            "invocations[0].command",
        ),
    ],
)
def test_cpu_affinity_commands_use_the_recorded_taskset_identity(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    bundle_name: str,
    truth_key: str,
    message: str,
) -> None:
    original = validator._read_json_array
    bundle_suffix = getattr(validator, bundle_name)

    def read_array(path, location, errors):
        payload = original(path, location, errors)
        if path.parent.as_posix().endswith(bundle_suffix) and path.name == (
            "invocations.json"
        ):
            payload = copy.deepcopy(payload)
            payload[0]["command"][0] = "/tmp/forged-taskset"
        return payload

    monkeypatch.setattr(validator, "_read_json_array", read_array)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(f"{truth_key}.{message}" in error for error in errors)


@pytest.mark.parametrize(
    ("bundle_name", "truth_key"),
    [
        ("CPU_EVIDENCE_BUNDLE", "literal_wedge_exact_dynamics_nonpaper"),
        ("CURRENT_SMALL_CPU_BUNDLE", "current_small_paper_cpu_v1"),
    ],
)
def test_cpu_taskset_identity_and_per_invocation_rechecks_are_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    bundle_name: str,
    truth_key: str,
) -> None:
    original = validator._read_bundle_json
    bundle_suffix = getattr(validator, bundle_name)

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(bundle_suffix)
        ):
            payload = copy.deepcopy(payload)
            payload["executed_tool_closure"]["taskset"]["sha256"] = "0" * 64
            payload["executed_tool_identity_rechecks"].pop()
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{truth_key}.metadata.executed_tool_closure.taskset.sha256" in error
        for error in errors
    )
    assert any(
        f"{truth_key}.metadata.executed_tool_identity_rechecks" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("bundle_name", "truth_key"),
    [
        ("CPU_EVIDENCE_BUNDLE", "literal_wedge_exact_dynamics_nonpaper"),
        ("CURRENT_SMALL_CPU_BUNDLE", "current_small_paper_cpu_v1"),
    ],
)
def test_cpu_runtime_file_identity_rejects_noncanonical_symlink_paths(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    bundle_name: str,
    truth_key: str,
) -> None:
    original = validator._read_bundle_json
    bundle_suffix = getattr(validator, bundle_name)

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(bundle_suffix)
        ):
            payload = copy.deepcopy(payload)
            files = payload["runtime_identity"]["resolved_regular_files"]
            canonical = next(
                path for path in files if path.endswith("/ld-linux-x86-64.so.2")
            )
            files["/lib64/ld-linux-x86-64.so.2"] = files.pop(canonical)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{truth_key}.metadata.runtime_identity.resolved_regular_files" in error
        and "recorded runtime path must be canonical" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda payload: payload.update(selected_cpu="8"),
            "metadata.selected_cpu",
        ),
        (
            lambda payload: payload.update(executed_tool_closure={}),
            "metadata.executed_tool_closure",
        ),
        (
            lambda payload: payload["command"].__setitem__(0, "/tmp/forged-taskset"),
            "metadata.command",
        ),
    ],
)
def test_crown_impact_affinity_cpu_tool_and_command_are_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutate,
    message: str,
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.IMPACT_V7_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"literal_wedge_crown_impact_v1_nonpaper.{message}" in error for error in errors
    )


def test_current_small_cpu_summary_cannot_promote_failed_turntable(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_json_array

    def read_array(path, location, errors):
        payload = original(path, location, errors)
        if location == "current_truth.current_small_paper_cpu_v1.summary":
            payload = copy.deepcopy(payload)
            row = next(
                item
                for item in payload
                if item["scenario"] == "turntable_mu_0_5_omega_5"
            )
            row["all_solver_steps_successful"] = True
        return payload

    monkeypatch.setattr(validator, "_read_json_array", read_array)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "summary['turntable_mu_0_5_omega_5'].all_solver_steps_successful" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper").update(
                {"paper_comparable": True}
            ),
            "paper_comparable: expected False, got True",
        ),
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper")[
                "one_thread"
            ].update({"all_steps_realtime": True}),
            "one_thread.all_steps_realtime: expected False, got True",
        ),
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper").update(
                {"validated_speedup": 99.0}
            ),
            "validated_speedup: expected 1.4341328993236115, got 99.0",
        ),
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper")[
                "four_threads"
            ].update({"mean_ms": 5.0}),
            "summary[4].mean_step_ms: expected 5.0",
        ),
    ],
)
def test_cpu_truth_rejects_promoted_or_drifted_performance_claims(
    manifest: dict, mutate, message: str
) -> None:
    mutate(manifest)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


def test_current_truth_hash_targets_do_not_require_local_build_artifacts() -> None:
    target_maps = (
        validator.CARD_V2_ARTIFACT_TARGETS,
        validator.BACKSPIN_V3_ARTIFACT_TARGETS,
        validator.TURNTABLE_V1_ARTIFACT_TARGETS,
        validator.PAINLEVE_V1_ARTIFACT_TARGETS,
        validator.CURRENT_SMALL_CPU_ARTIFACT_TARGETS,
        validator.VISUAL_ARTIFACT_TARGETS,
        validator.IMPACT_V7_ARTIFACT_TARGETS,
        validator.ARCH101_V5_ARTIFACT_TARGETS,
    )

    assert all(
        not path.startswith("build/")
        for targets in target_maps
        for path in targets.values()
    )


@pytest.mark.parametrize(
    ("truth_key", "hash_key", "message"),
    [
        (
            "card_house_manifold_sensitivity_v2_nonpaper",
            "trace_binary",
            "metadata.source_identity.trace_executable.sha256",
        ),
        (
            "literal_wedge_visual_nonpaper",
            "current_trace_binary",
            "provenance.source.trace_executable_sha256",
        ),
        (
            "painleve_proxy_visual_v1_nonpaper",
            "trace_binary",
            "metadata.source_identity.trace_binary.sha256",
        ),
        (
            "painleve_proxy_visual_v1_nonpaper",
            "demo_binary",
            "metadata.source_identity.demo_binary.sha256",
        ),
        (
            "backspin_visual_v3_nonpaper",
            "trace_binary",
            "metadata.source_identity.trace_binary.sha256",
        ),
        (
            "backspin_visual_v3_nonpaper",
            "demo_binary",
            "metadata.source_identity.demo_binary.sha256",
        ),
        (
            "literal_wedge_crown_impact_v1_nonpaper",
            "trace_binary",
            "metadata.binary_sha256",
        ),
        (
            "literal_arch_101_v1_nonpaper",
            "trace_binary",
            "metadata.source_identity.trace_executable.sha256",
        ),
        (
            "literal_arch_101_v1_nonpaper",
            "collision_probe_binary",
            "metadata.source_identity.collision_probe_executable.sha256",
        ),
    ],
)
def test_recorded_binary_hashes_are_bound_to_repository_metadata(
    manifest: dict, truth_key: str, hash_key: str, message: str
) -> None:
    _truth(manifest, truth_key)["artifact_hashes"][hash_key] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize("mutation", ["missing", "unexpected"])
def test_prior_source_archive_index_requires_exact_membership(
    manifest: dict, monkeypatch: pytest.MonkeyPatch, mutation: str
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "artifact-index.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.PRIOR_SOURCE_ARCHIVE)
        ):
            payload = copy.deepcopy(payload)
            if mutation == "missing":
                payload["files"].pop("invocations.json")
            else:
                payload["files"]["unexpected.txt"] = {
                    "sha256": "0" * 64,
                    "size_bytes": 0,
                }
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "prior_source_strict_card_paper_cpu.artifact_index.files" in error
        and "exact membership" in error
        for error in errors
    )


@pytest.mark.parametrize("mutation", ["missing", "unexpected"])
def test_cpu_artifact_index_requires_exact_disk_membership(
    manifest: dict, monkeypatch: pytest.MonkeyPatch, mutation: str
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "artifact-index.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.CPU_EVIDENCE_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            if mutation == "missing":
                payload["files"].pop("raw.csv")
            else:
                payload["files"]["unexpected.txt"] = {
                    "sha256": "0" * 64,
                    "size_bytes": 0,
                }
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "literal_wedge_exact_dynamics_nonpaper.artifact_index.files" in error
        and "exact disk membership" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper").update(
                {"contacts_each_step": 95}
            ),
            ".raw[1].contacts",
        ),
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper")[
                "solver_options"
            ].update({"inner_fixed_sweeps": 29}),
            ".raw[1].inner_sweeps_requested",
        ),
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper").update(
                {"max_arch_body_displacement_from_initial_m": 0.0}
            ),
            ".raw[1].max_arch_body_displacement_from_initial",
        ),
    ],
)
def test_cpu_claim_antecedents_are_recomputed_from_raw_rows(
    manifest: dict, mutate, message: str
) -> None:
    mutate(manifest)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("returncode", "invocations[0].returncode"),
        ("warmup_repetition", "invocations[1].warmup.repetition"),
        ("measured_repetition", "invocations[1].repetition"),
        ("affinity_identity", "invocations[4].cpu_affinity"),
        ("affinity_mapping", "invocations[4].cpu_affinity"),
        ("cpu_list", "invocations[0].cpu_list"),
        ("command_cpu", "invocations[0].command[2]"),
        ("command_steps", "invocations[0].command[7]"),
        ("command_threads", "invocations[0].command[12]"),
        ("topology_core", "invocations[0].cpu_affinity"),
        ("topology_package", "invocations[0].cpu_affinity"),
        ("topology_smt", "invocations[0].cpu_affinity"),
        ("topology_frequency", "invocations[0].cpu_affinity"),
        ("topology_governor", "invocations[0].cpu_affinity"),
    ],
)
def test_cpu_process_records_bind_completion_and_cpu_identity(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    original = validator._read_json_array

    def read_array(path, location, errors):
        payload = original(path, location, errors)
        if path.as_posix().endswith(
            f"{validator.CPU_EVIDENCE_BUNDLE}/invocations.json"
        ):
            payload = copy.deepcopy(payload)
            if mutation == "returncode":
                payload[0]["returncode"] = 1
            elif mutation == "warmup_repetition":
                payload[0]["repetition"] = 2
            elif mutation == "measured_repetition":
                payload[1]["repetition"] = 9
            elif mutation == "affinity_identity":
                payload[4]["cpu_affinity"]["logical_cpus"] = [8, 8, 12, 14]
            elif mutation == "affinity_mapping":
                payload[4]["cpu_affinity"]["logical_cpu_physical_core_keys"][
                    "8"
                ] = "forged:core"
            elif mutation == "cpu_list":
                payload[0]["cpu_list"] = "999"
            elif mutation == "command_cpu":
                payload[0]["command"][2] = "999"
            elif mutation == "command_steps":
                payload[0]["command"][7] = "999"
            elif mutation == "command_threads":
                payload[0]["command"][12] = "999"
            else:
                topology = payload[0]["cpu_affinity"]["logical_cpu_topology"]["8"]
                if mutation == "topology_core":
                    topology["core_id"] = "999"
                elif mutation == "topology_package":
                    topology["physical_package_id"] = "999"
                elif mutation == "topology_smt":
                    topology["thread_sibling_count"] = 999
                elif mutation == "topology_frequency":
                    topology["cpuinfo_max_frequency_khz"] = 999
                else:
                    topology["scaling_governor"] = "forged"
        return payload

    monkeypatch.setattr(validator, "_read_json_array", read_array)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("process_returncode", "1", ".raw[1].process_returncode"),
        ("repetition", "2", ".raw[1].warmup"),
        ("affinity_logical_cpus", "999", ".raw[1].affinity_logical_cpus"),
        (
            "affinity_physical_core_keys",
            "forged:core",
            ".raw[1].affinity_physical_core_keys",
        ),
        ("affinity_source", "forged", ".raw[1].affinity_source"),
        (
            "affinity_logical_cpu_physical_core_keys",
            "forged",
            ".raw[1].affinity_logical_cpu_physical_core_keys",
        ),
        ("affinity_package_ids", "forged", ".raw[1].affinity_package_ids"),
        (
            "affinity_smt_sibling_counts",
            "forged",
            ".raw[1].affinity_smt_sibling_counts",
        ),
        (
            "max_phase_exact_colored_bgs_logical_cpus",
            None,
            ".raw[1].rows[0].max_phase_exact_colored_bgs_logical_cpus",
        ),
    ],
)
def test_cpu_raw_rows_bind_process_repetition_and_cpu_identity(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    field: str,
    value,
    message: str,
) -> None:
    original = validator._read_csv

    def read_csv(path, location, errors):
        payload = original(path, location, errors)
        if payload is not None and path.as_posix().endswith(
            f"{validator.CPU_EVIDENCE_BUNDLE}/raw.csv"
        ):
            fieldnames, rows = payload
            rows = copy.deepcopy(rows)
            if field == "repetition":
                row = next(
                    item
                    for item in rows
                    if item["warmup"] == "1" and item["requested_threads"] == "1"
                )
            else:
                row = next(
                    item
                    for item in rows
                    if item["warmup"] == "0" and item["requested_threads"] == "1"
                )
            row[field] = value
            return fieldnames, rows
        return payload

    monkeypatch.setattr(validator, "_read_csv", read_csv)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


def test_csv_reader_rejects_rows_with_missing_trailing_cells(tmp_path: Path) -> None:
    path = tmp_path / "short-row.csv"
    path.write_text("first,second\nvalue\n", encoding="utf-8")
    errors: list[str] = []

    parsed = validator._read_csv(path, "short_row", errors)

    assert parsed is None
    assert errors == ["short_row: CSV row has fewer values than the header"]


@pytest.mark.parametrize(
    "field",
    [
        "exact_colored_bgs_logical_cpus",
        "max_phase_exact_colored_bgs_logical_cpus",
    ],
)
def test_cpu_residency_ids_match_taskset_affinity(
    manifest: dict, monkeypatch: pytest.MonkeyPatch, field: str
) -> None:
    original = validator._read_csv

    def read_csv(path, location, errors):
        payload = original(path, location, errors)
        if payload is not None and path.as_posix().endswith(
            f"{validator.CPU_EVIDENCE_BUNDLE}/raw.csv"
        ):
            fieldnames, rows = payload
            rows = copy.deepcopy(rows)
            for row in rows:
                if row["warmup"] == "0" and row["requested_threads"] == "4":
                    row[field] = "999;998;997;996"
            return fieldnames, rows
        return payload

    monkeypatch.setattr(validator, "_read_csv", read_csv)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(f".raw[4].{field}" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("actual_threads", "1", "summary[4].actual_threads"),
        (
            "affinity_physical_core_count",
            1,
            "summary[4].affinity_physical_core_count",
        ),
        (
            "observed_exact_colored_bgs_logical_cpu_count",
            0,
            "summary[4].observed_exact_colored_bgs_logical_cpu_count",
        ),
        ("colored_bgs_dispatches", 0, "summary[4].colored_bgs_dispatches"),
    ],
)
def test_cpu_summary_cannot_falsify_multicore_antecedents(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    field: str,
    value,
    message: str,
) -> None:
    original = validator._read_json_array

    def read_array(path, location, errors):
        payload = original(path, location, errors)
        if path.as_posix().endswith(f"{validator.CPU_EVIDENCE_BUNDLE}/summary.json"):
            payload = copy.deepcopy(payload)
            payload[1][field] = value
        return payload

    monkeypatch.setattr(validator, "_read_json_array", read_array)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: _truth(data, "card_house_manifold_sensitivity_v2_nonpaper")[
                "compact"
            ].update({"accepted_cap_exact_groups": 0}),
            "compact.accepted_cap_exact_groups",
        ),
        (
            lambda data: _truth(data, "literal_wedge_visual_nonpaper").update(
                {"contacts_each_step": 95}
            ),
            "trace_equivalence.expected_reference_int.contacts",
        ),
        (
            lambda data: _truth(data, "painleve_proxy_visual_v1_nonpaper").update(
                {"shorter_pre_tumble_margin_m": 0.0}
            ),
            "shorter_pre_tumble_margin_m",
        ),
        (
            lambda data: _truth(data, "literal_wedge_crown_impact_v1_nonpaper").update(
                {"accepted_caps": 0}
            ),
            "accepted_caps: expected 5, got 0",
        ),
        (
            lambda data: _truth(data, "literal_wedge_crown_impact_v1_nonpaper").update(
                {"final_max_far_field_displacement_m": 0.0}
            ),
            "summary.preservation.final_maximum_far_field_displacement",
        ),
        (
            lambda data: _truth(data, "literal_arch_101_v1_nonpaper").update(
                {"terminal_residual": 0.0}
            ),
            "summary.terminal_residual",
        ),
        (
            lambda data: _truth(data, "literal_arch_101_v1_nonpaper")[
                "collision_probe"
            ].update({"genuine_contact_graph": False}),
            "summary.collision_probe.genuine_contact_graph",
        ),
        (
            lambda data: _truth(data, "literal_wedge_exact_dynamics_nonpaper").update(
                {"closure_meters": 0.0}
            ),
            "closure_meters: expected 1e-06, got 0.0",
        ),
    ],
)
def test_declared_current_truth_values_are_bound_to_evidence(
    manifest: dict, mutate, message: str
) -> None:
    mutate(manifest)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (
            "standing_reference_fields",
            0,
            "standing_reference_fields: expected 88, got 0",
        ),
        (
            "standing_reference_mismatches",
            1,
            "standing_reference_mismatches: expected 0, got 1",
        ),
    ],
)
def test_impact_standing_reference_claims_are_bound_to_summary(
    manifest: dict, field: str, value: int, message: str
) -> None:
    _truth(manifest, "literal_wedge_crown_impact_v1_nonpaper")[field] = value

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("scope", "forged scope", "runtime_provenance.scope"),
        ("identity_rechecks", [], "runtime_provenance.identity_rechecks"),
    ],
)
def test_arch_runtime_provenance_scope_and_rechecks_are_bound(
    manifest: dict, field: str, value, message: str
) -> None:
    _truth(manifest, "literal_arch_101_v1_nonpaper")["runtime_provenance"][
        field
    ] = value

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


def test_arch_runtime_rechecks_are_bound_to_archived_invocation(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "invocation.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.ARCH101_V5_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            payload["identity_rechecks"] = ["after_trace"]
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("invocation.identity_rechecks" in error for error in errors)
    assert any("runtime_provenance.identity_rechecks" in error for error in errors)


@pytest.mark.parametrize(
    ("bundle_name", "identity_path", "message"),
    [
        (
            "CARD_V2_BUNDLE",
            ("source_identity", "trace_executable"),
            "card_house_manifold_sensitivity_v2_nonpaper.metadata.source_identity."
            "trace_executable.resolved_regular_shared_libraries[0].sha256",
        ),
        (
            "IMPACT_V7_BUNDLE",
            ("runtime_identity",),
            "literal_wedge_crown_impact_v1_nonpaper.metadata.runtime_identity."
            "resolved_regular_shared_libraries[0].sha256",
        ),
        (
            "ARCH101_V5_BUNDLE",
            ("source_identity", "collision_probe_executable"),
            "literal_arch_101_v1_nonpaper.metadata.source_identity."
            "collision_probe_executable.resolved_regular_shared_libraries[0]."
            "sha256",
        ),
    ],
)
def test_structured_runtime_library_closures_are_checked_live(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    bundle_name: str,
    identity_path: tuple[str, ...],
    message: str,
) -> None:
    original = validator._read_bundle_json
    bundle_suffix = getattr(validator, bundle_name)

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(bundle_suffix)
        ):
            payload = copy.deepcopy(payload)
            identity = payload
            for key in identity_path:
                identity = identity[key]
            identity["resolved_regular_shared_libraries"][0]["sha256"] = "0" * 64
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("bundle_name", "filename", "mutate", "message"),
    [
        (
            "CARD_V2_BUNDLE",
            "invocation.json",
            lambda payload: payload.update(identity_rechecks=["after_compact"]),
            "card_house_manifold_sensitivity_v2_nonpaper.invocation."
            "identity_rechecks",
        ),
        (
            "IMPACT_V7_BUNDLE",
            "metadata.json",
            lambda payload: payload.update(identity_rechecks=["after_impact_trace"]),
            "literal_wedge_crown_impact_v1_nonpaper.metadata.identity_rechecks",
        ),
    ],
)
def test_successor_runtime_recheck_stages_are_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    bundle_name: str,
    filename: str,
    mutate,
    message: str,
) -> None:
    original = validator._read_bundle_json
    bundle_suffix = getattr(validator, bundle_name)

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == filename
            and bundle is not None
            and bundle.as_posix().endswith(bundle_suffix)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("bundle_name", "mutate", "message"),
    [
        (
            "CARD_V2_BUNDLE",
            lambda payload: payload["commands"]["compact"].__setitem__(
                0, "/tmp/forged-taskset"
            ),
            "card_house_manifold_sensitivity_v2_nonpaper.invocation.commands."
            "compact[0]",
        ),
        (
            "CARD_V2_BUNDLE",
            lambda payload: payload["commands"]["four_point_planar"].__setitem__(
                3, "/tmp/forged-trace"
            ),
            "card_house_manifold_sensitivity_v2_nonpaper.invocation.commands."
            "four_point_planar[1:][2]",
        ),
        (
            "ARCH101_V5_BUNDLE",
            lambda payload: payload["command"].__setitem__(0, "/tmp/forged-taskset"),
            "literal_arch_101_v1_nonpaper.invocation.command[0]",
        ),
        (
            "ARCH101_V5_BUNDLE",
            lambda payload: payload["command"].__setitem__(3, "/tmp/forged-trace"),
            "literal_arch_101_v1_nonpaper.invocation.command[1:][2]",
        ),
        (
            "ARCH101_V5_BUNDLE",
            lambda payload: payload["collision_probe_command"].__setitem__(
                0, "/tmp/forged-probe"
            ),
            "literal_arch_101_v1_nonpaper.invocation.collision_probe_command[0]",
        ),
    ],
)
def test_card_and_arch_invocation_executables_are_identity_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    bundle_name: str,
    mutate,
    message: str,
) -> None:
    original = validator._read_bundle_json
    bundle_suffix = getattr(validator, bundle_name)

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "invocation.json"
            and bundle is not None
            and bundle.as_posix().endswith(bundle_suffix)
        ):
            payload = copy.deepcopy(payload)
            mutate(payload)
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    "truth_key",
    [
        "card_house_manifold_sensitivity_v2_nonpaper",
        "literal_wedge_exact_dynamics_nonpaper",
        "current_small_paper_cpu_v1",
        "literal_wedge_crown_impact_v1_nonpaper",
        "literal_arch_101_v1_nonpaper",
    ],
)
def test_successor_runtime_provenance_summaries_are_bound(
    manifest: dict, truth_key: str
) -> None:
    _truth(manifest, truth_key)["runtime_provenance"]["ldd_tool_sha256"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{truth_key}.runtime_provenance.ldd_tool_sha256" in error for error in errors
    )


@pytest.mark.parametrize(
    "truth_key",
    [
        "card_house_manifold_sensitivity_v2_nonpaper",
        "literal_wedge_exact_dynamics_nonpaper",
        "current_small_paper_cpu_v1",
        "literal_wedge_crown_impact_v1_nonpaper",
        "literal_arch_101_v1_nonpaper",
    ],
)
def test_runtime_provenance_rejects_extra_claims(
    manifest: dict, truth_key: str
) -> None:
    _truth(manifest, truth_key)["runtime_provenance"]["paper_parity"] = True

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{truth_key}.runtime_provenance" in error
        and (
            "exact runtime-provenance contract changed" in error
            or "unexpected keys" in error
        )
        for error in errors
    )


def test_unreadable_impact_metadata_fails_closed_without_crashing(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        if (
            name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.IMPACT_V7_BUNDLE)
        ):
            errors.append(f"{location}: simulated unreadable metadata")
            return None
        return original(bundle, name, location, errors)

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("simulated unreadable metadata" in error for error in errors)


def test_malformed_visual_video_stream_fails_closed_without_crashing(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "video-probe.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.VISUAL_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            payload["streams"] = ["not-an-object"]
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "video_probe.streams: expected one video stream" in error for error in errors
    )


def test_visual_artifact_index_materializes_every_final_artifact(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "artifact-index.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.VISUAL_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            payload["artifacts"].pop()
            payload["artifact_count"] -= 1
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "literal_wedge_visual_nonpaper.artifact_index" in error
        and "expected 19" in error
        for error in errors
    )


def test_incline_truth_and_requirement_boundaries_accept_sealed_bundle(
    manifest: dict,
) -> None:
    truth_errors: list[str] = []
    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, truth_errors)
    boundary_errors: list[str] = []
    validator._validate_incline_requirement_boundaries(
        _incline_requirement_map(manifest), boundary_errors
    )
    truth = _truth(manifest, "incline_visual_v1_nonpaper")

    assert truth_errors == []
    assert boundary_errors == []
    assert truth["capture_contacts_per_post_initial_step"] == 8
    assert truth["trace_aggregate_contacts_per_post_initial_step"] == 6
    assert validator.validate_manifest(manifest, ROOT) == []


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("claim_valid", True),
        ("paper_parity", True),
        ("paper_comparable", True),
        ("external_solver_parity", True),
        ("approved_source_golden", True),
        ("approved_source_golden_diff", True),
        ("timing_verdict", True),
        ("realtime_verdict", True),
        ("generated_imagery", True),
        ("automated_semantic_outcome_validated", True),
        ("trace_equivalence_to_rendered_demo", True),
        ("capture_trace_contact_count_equivalent", True),
        ("per_cell_trace_equivalence", True),
        ("full_state_trace_equivalence", True),
        ("paper_reference_contact_count_match", True),
        ("maximum_penetration_proven", True),
        ("full_friction_sweep", True),
        ("capture_sidecar_deliverable_validated", True),
    ],
)
def test_incline_truth_rejects_promoted_claims(
    manifest: dict, field: str, value
) -> None:
    _truth(manifest, "incline_visual_v1_nonpaper")[field] = value
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(f"incline_visual_v1_nonpaper.{field}" in error for error in errors)


def test_incline_truth_rejects_noncurrent_bundle_and_hash(manifest: dict) -> None:
    incline = _truth(manifest, "incline_visual_v1_nonpaper")
    incline["bundle"] += "-missing"
    incline["artifact_hashes"]["metadata.json"] = "0" * 64
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "incline_visual_v1_nonpaper.bundle: expected current path" in error
        for error in errors
    )
    assert any(
        "incline_visual_v1_nonpaper.artifact_hashes['metadata.json']" in error
        for error in errors
    )


def test_incline_truth_rejects_unrecognized_claim_key(manifest: dict) -> None:
    _truth(manifest, "incline_visual_v1_nonpaper")["paper_contact_count_match"] = True
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any("exact truth schema changed" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "scenario", "result_field", "value"),
    [
        (
            "slide_downhill_displacement_m",
            "incline_mu_0_4",
            "downhill_displacement_m",
            1.0,
        ),
        ("trace_max_residual", None, "max_residual", 5e-7),
    ],
)
def test_incline_truth_metrics_bind_recomputed_trace_values(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    field: str,
    scenario: str | None,
    result_field: str,
    value: float,
) -> None:
    original = validator._validate_incline_v1_trace

    def validate_trace(bundle, requested_scenario, location, errors):
        result = original(bundle, requested_scenario, location, errors)
        if result is not None and (scenario is None or requested_scenario == scenario):
            result = copy.deepcopy(result)
            result[result_field] = value
        return result

    monkeypatch.setattr(validator, "_validate_incline_v1_trace", validate_trace)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        f"incline_visual_v1_nonpaper.{field}: raw traces changed" in error
        for error in errors
    )


def test_incline_source_identity_binds_dependency_and_query_hash(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    dependency = "demo_dependency_lib__libdart.so.6.19.3"

    def mutate(payload):
        identity = payload["source_identity"]
        identity[dependency] = {"path": "/forged", "sha256": "0" * 64}
        identity["binary_source_bindings"]["demo_source_binding"][
            "query_payload_sha256"
        ] = ("1" * 64)

    _patch_incline_bundle_json(monkeypatch, "metadata.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        f"source_identity.{dependency}.path: expected" in error for error in errors
    )
    assert any(
        f"source_identity.{dependency}.sha256: expected" in error for error in errors
    )
    assert any(
        "demo_source_binding.query_payload_sha256: expected" in error
        for error in errors
    )


def test_incline_capture_provenance_rejects_nonzero_returncode(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and bundle is not None
            and bundle.as_posix().endswith(validator.INCLINE_V1_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            if name == "capture-provenance.json":
                payload["returncode"] = 99
            elif name == "metadata.json":
                payload["capture_provenance"]["returncode"] = 99
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "capture_provenance.returncode: expected 0, got 99" in error for error in errors
    )


def test_incline_manual_inspection_rejects_duplicate_representatives(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(payload):
        representative = payload["representative_artifacts"][0]
        payload["representative_artifacts"] = [
            copy.deepcopy(representative) for _ in range(7)
        ]

    _patch_incline_bundle_json(monkeypatch, "manual-inspection.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "representative_artifacts: expected exact ordered unique paths" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("filename", "mutation", "message"),
    [
        (
            "invocations.json",
            lambda payload: payload["traces"][0].update(returncode=99),
            "invocations.traces.incline_mu_0_4.returncode: expected 0, got 99",
        ),
        (
            "run-summary.json",
            lambda payload: payload.update({"pass": False}),
            "run_summary.pass: expected True, got False",
        ),
    ],
)
def test_incline_invocation_and_run_summary_semantics_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    filename: str,
    mutation,
    message: str,
) -> None:
    _patch_incline_bundle_json(monkeypatch, filename, mutation)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(message in error for error in errors)


def test_incline_capture_timeline_recomputes_every_step_and_frame(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(payload):
        timeline = payload["results"][0]["timeline_validation"]
        diagnostics = timeline["steps"]["60"]["solver_diagnostics"]
        diagnostics["boxed_lcp_fallbacks"] = 1
        diagnostics["status"] = "fallback"
        diagnostics["fbf_status"] = "fallback"
        timeline["frames"] = {}

    _patch_incline_bundle_json(monkeypatch, "run-summary.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "steps[60].solver_diagnostics.boxed_lcp_fallbacks: expected 0, got 1" in error
        for error in errors
    )
    assert any(
        "timeline_validation.steps: aggregate count projection changed" in error
        for error in errors
    )
    assert any(
        "timeline_validation.frames: expected every captured even step" in error
        for error in errors
    )


def test_incline_capture_timeline_binds_terminal_diagnostics(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(payload):
        payload["results"][0]["timeline_validation"]["final_solver_diagnostics"][
            "total_iterations"
        ] += 1

    _patch_incline_bundle_json(monkeypatch, "run-summary.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "final_solver_diagnostics: step 120 binding changed" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("fallback", "rows[120].fallbacks: expected 0"),
        ("residual", "residual: 0.01 exceeds 1e-06"),
        ("contact", "rows[30].contacts: expected 3"),
        ("displacement", "slide displacement does not exceed 0.5 m"),
        ("velocity", "rows[50].vx: finite-difference mismatch"),
    ],
)
def test_incline_trace_semantics_are_recomputed_from_raw_csv(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    original = validator._read_csv

    def read_csv(path, location, errors):
        payload = original(path, location, errors)
        if payload is not None and path.as_posix().endswith(
            "fig01_02_incline_current_v1/traces/incline_mu_0_4.csv"
        ):
            fields, rows = payload
            rows = copy.deepcopy(rows)
            if mutation == "fallback":
                rows[120]["fallbacks"] = "1"
            elif mutation == "residual":
                rows[20]["residual"] = "0.01"
            elif mutation == "contact":
                rows[30]["contacts"] = "0"
            elif mutation == "displacement":
                initial = rows[0]
                for row in rows:
                    for field in ("x", "y", "z", "up_z"):
                        row[field] = initial[field]
                    for field in ("vx", "vy", "vz"):
                        row[field] = "0"
            elif mutation == "velocity":
                rows[50]["vx"] = str(float(rows[50]["vx"]) + 1.0)
            return fields, rows
        return payload

    monkeypatch.setattr(validator, "_read_csv", read_csv)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing", "exact membership changed"),
        ("unexpected", "exact membership changed"),
        ("hash", ".sha256: expected"),
    ],
)
def test_incline_artifact_index_is_exact_and_content_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "missing":
            payload["artifacts"].pop()
        elif mutation == "unexpected":
            payload["artifacts"][0]["path"] = "unexpected.txt"
        elif mutation == "hash":
            payload["artifacts"][0]["sha256"] = "0" * 64

    _patch_incline_bundle_json(monkeypatch, "artifact-index.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "incline_visual_v1_nonpaper.artifact_index" in error and message in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("verdict", ".verdicts.paper_parity: expected False"),
        ("hash", ".sha256: expected"),
        ("observation", ".observation: expected non-empty text"),
    ],
)
def test_incline_manual_inspection_is_exact_and_artifact_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "verdict":
            payload["verdicts"]["paper_parity"] = True
        elif mutation == "hash":
            payload["representative_artifacts"][0]["sha256"] = "0" * 64
        elif mutation == "observation":
            payload["representative_artifacts"][0]["observation"] = ""

    _patch_incline_bundle_json(monkeypatch, "manual-inspection.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(
        "incline_visual_v1_nonpaper.manual_inspection" in error and message in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("truth", "aggregate_count_projection_sha256: raw traces changed"),
        ("capture", ".capture_sha256: expected"),
    ],
)
def test_incline_aggregate_projection_mismatch_fails_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    if mutation == "truth":
        _truth(manifest, "incline_visual_v1_nonpaper")[
            "aggregate_count_projection_sha256"
        ] = ("0" * 64)
    else:

        def mutate(payload):
            payload["aggregate_count_projection_comparison"]["capture_sha256"] = (
                "0" * 64
            )

        _patch_incline_bundle_json(monkeypatch, "trace-summary.json", mutate)
    errors: list[str] = []

    validator._validate_incline_v1_truth(manifest["current_truth"], ROOT, errors)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("requirement_id", "kind"),
    [
        ("fig.01", "exact_fixture"),
        ("fig.01", "trace_csv"),
        ("fig.02", "exact_fixture"),
        ("fig.02", "trace_csv"),
        ("fig.02", "still_image"),
        ("video.03_incline", "exact_fixture"),
        ("video.03_incline", "video_clip"),
    ],
)
def test_incline_requirements_bind_all_current_positive_deliverables(
    manifest: dict, requirement_id: str, kind: str
) -> None:
    by_id = _incline_requirement_map(manifest)
    deliverable = next(
        item for item in by_id[requirement_id]["deliverables"] if item["kind"] == kind
    )
    deliverable["path"] += ".stale"
    errors: list[str] = []

    validator._validate_incline_requirement_boundaries(by_id, errors)

    assert any(
        f"{requirement_id}.incline_current_boundary.{kind}.path" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("requirement_id", "kind"),
    [
        (requirement_id, kind)
        for requirement_id in ("fig.01", "fig.02", "video.03_incline")
        for kind in (
            "external_baseline",
            "approved_golden",
            "golden_diff",
            "capture_sidecar",
            "claim_map",
        )
    ]
    + [("fig.01", "comparison_plot")],
)
def test_incline_requirements_reject_forbidden_promotions(
    manifest: dict, requirement_id: str, kind: str
) -> None:
    by_id = _incline_requirement_map(manifest)
    deliverable = next(
        (
            item
            for item in by_id[requirement_id]["deliverables"]
            if item["kind"] == kind
        ),
        None,
    )
    if deliverable is None:
        deliverable = {"kind": kind, "path": "unsupported", "validated": True}
        by_id[requirement_id]["deliverables"].append(deliverable)
    else:
        deliverable["validated"] = True
    errors: list[str] = []

    validator._validate_incline_requirement_boundaries(by_id, errors)

    assert any(
        f"{requirement_id}.incline_current_boundary.{kind}: "
        "promotion remains unsupported" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("field", "value"), [("status", "complete"), ("fallback_count", 1)]
)
def test_incline_requirements_remain_partial_and_fallback_free(
    manifest: dict, field: str, value
) -> None:
    by_id = _incline_requirement_map(manifest)
    by_id["fig.01"][field] = value
    errors: list[str] = []

    validator._validate_incline_requirement_boundaries(by_id, errors)

    assert any(f"fig.01.incline_current_boundary.{field}" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("manual_inspected", False),
        ("claim_valid", True),
        ("paper_parity", True),
        ("paper_comparable", True),
        ("external_solver_parity", True),
        ("approved_source_golden", True),
        ("timing_verdict", True),
        ("realtime_verdict", True),
        ("generated_imagery", True),
        ("automated_semantic_outcome_validated", True),
        ("manual_visual_outcome_validated", False),
        ("trace_equivalence_to_rendered_demo", True),
        ("solver_projection_equivalent", False),
        ("strict_rigid_body_rest_proven", True),
        ("signed_angular_direction_proven", True),
        ("continuous_contact_proven", True),
        ("capture_sidecar_deliverable_validated", True),
    ],
)
def test_backspin_truth_rejects_promoted_claims(
    manifest: dict, field: str, value
) -> None:
    _truth(manifest, "backspin_visual_v3_nonpaper")[field] = value

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(f"backspin_visual_v3_nonpaper.{field}" in error for error in errors)


def test_backspin_truth_rejects_noncurrent_bundle_and_hash(manifest: dict) -> None:
    backspin = _truth(manifest, "backspin_visual_v3_nonpaper")
    backspin["bundle"] += "-missing"
    backspin["artifact_hashes"]["metadata.json"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "backspin_visual_v3_nonpaper.bundle: expected current path" in error
        for error in errors
    )
    assert any(
        "backspin_visual_v3_nonpaper.artifact_hashes['metadata.json']" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing_row", "expected 131 rows including step 0"),
        ("time", "rows[50].time: expected step/60"),
        ("no_reversal", "translational velocity never reversed"),
        ("residual", "residual: 0.01 exceeds 1e-06"),
        ("fallback", "boxed-LCP fallback observed"),
        ("final_contact", "final state is not in contact"),
    ],
)
def test_backspin_trace_semantics_are_recomputed_from_raw_csv(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    original = validator._read_csv

    def read_csv(path, location, errors):
        payload = original(path, location, errors)
        if payload is not None and path.as_posix().endswith(
            "fig03_backspin_current_v3/traces/backspin.csv"
        ):
            fields, rows = payload
            rows = copy.deepcopy(rows)
            if mutation == "missing_row":
                rows.pop()
            elif mutation == "time":
                rows[50]["time"] = "999"
            elif mutation == "no_reversal":
                for row in rows:
                    row["vx"] = "1"
            elif mutation == "residual":
                rows[1]["residual"] = "0.01"
            elif mutation == "fallback":
                rows[-1]["fallbacks"] = "1"
            elif mutation == "final_contact":
                rows[-1]["contacts"] = "0"
            return fields, rows
        return payload

    monkeypatch.setattr(validator, "_read_csv", read_csv)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


def test_backspin_trace_metrics_are_bound_to_declared_truth(manifest: dict) -> None:
    backspin = _truth(manifest, "backspin_visual_v3_nonpaper")
    backspin["maximum_forward_x_m"] = 0.0
    backspin["final_x_m"] = 0.0
    backspin["solver_projection_sha256"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("maximum_forward_x_m" in error for error in errors)
    assert any("final_x_m" in error for error in errors)
    assert any("solver_projection_sha256" in error for error in errors)


def test_backspin_trace_and_capture_solver_projection_must_match(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(payload):
        payload["timeline_validation"]["steps"]["120"]["solver_diagnostics"][
            "contacts"
        ] = 1

    _patch_backspin_bundle_json(monkeypatch, "backspin/metadata.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "trace/capture solver projection differs at 120" in error for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing", "exact membership changed"),
        ("unexpected", "exact membership changed"),
        ("bytes", ".bytes: expected"),
        ("sha256", ".sha256: expected"),
    ],
)
def test_backspin_artifact_index_is_exact_and_content_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "missing":
            payload["artifacts"].pop()
            payload["artifact_count"] -= 1
        elif mutation == "unexpected":
            payload["artifacts"].append(
                {"path": "unexpected.txt", "bytes": 0, "sha256": "0" * 64}
            )
            payload["artifact_count"] += 1
        elif mutation == "bytes":
            payload["artifacts"][0]["bytes"] += 1
        elif mutation == "sha256":
            payload["artifacts"][0]["sha256"] = "0" * 64

    _patch_backspin_bundle_json(monkeypatch, "artifact-index.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "backspin_visual_v3_nonpaper.artifact_index" in error and message in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("verdict", ".verdicts: expected"),
        ("hash", ".sha256: expected"),
        ("observation", ".observation: expected non-empty text"),
    ],
)
def test_backspin_manual_inspection_is_exact_and_artifact_bound(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "verdict":
            payload["verdicts"]["signed_angular_direction_proven_by_media"] = True
        elif mutation == "hash":
            payload["representative_artifacts"][0]["sha256"] = "0" * 64
        elif mutation == "observation":
            payload["representative_artifacts"][0]["observation"] = ""

    _patch_backspin_bundle_json(monkeypatch, "manual-inspection.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "backspin_visual_v3_nonpaper.manual_inspection" in error and message in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("contract", "media_validation.mp4.stream_contract"),
        ("probe", "media_validation.mp4.probe.streams[0].nb_frames"),
        ("malformed", "media_validation: expected objects"),
    ],
)
def test_backspin_media_contract_fails_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "contract":
            payload["media_validation"][0]["stream_contract"]["width"] = 1
        elif mutation == "probe":
            payload["media_validation"][0]["probe"]["streams"][0]["nb_frames"] = "1"
        elif mutation == "malformed":
            payload["media_validation"] = ["not-an-object"]

    _patch_backspin_bundle_json(monkeypatch, "backspin/metadata.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    "source_key",
    [
        "demo_source",
        "backspin_checker_mesh",
        "backspin_checker_material",
        "backspin_checker_texture",
    ],
)
def test_backspin_source_identity_is_bound_to_repository_source(
    manifest: dict, monkeypatch: pytest.MonkeyPatch, source_key: str
) -> None:
    def mutate(payload):
        payload["source_identity"][source_key]["sha256"] = "0" * 64

    _patch_backspin_bundle_json(monkeypatch, "metadata.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"metadata.source_identity.{source_key}.sha256" in error for error in errors
    )


def test_backspin_capture_provenance_binds_runtime_checker_hashes(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(payload):
        payload["capture_provenance"]["runtime_resources_before"][
            "backspin_checker_mesh"
        ]["sha256"] = ("0" * 64)

    _patch_backspin_bundle_json(monkeypatch, "metadata.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "metadata.capture_provenance.runtime_resources_before" in error
        for error in errors
    )


def test_backspin_checker_structure_is_derived_from_repository_assets() -> None:
    mesh = (ROOT / "data/obj/fbf_backspin_checker_sphere.obj").read_text(
        encoding="ascii"
    )
    texture = (ROOT / "data/obj/fbf_backspin_checker.ppm").read_text(encoding="ascii")

    result = validator._validate_backspin_checker_asset_structure(mesh, texture)

    assert result == {
        "mesh_vertex_count": 559,
        "mesh_uv_count": 559,
        "mesh_normal_count": 559,
        "mesh_triangle_count": 960,
        "mesh_outward_winding": True,
        "mesh_uv_triangles_non_degenerate": True,
        "texture_column_width_range": [10, 11],
        "texture_row_height_range": [8, 8],
        "checker_layout_validated": True,
        "registration_tile_coordinates": [0, 1],
        "registration_tile_count": 1,
    }


@pytest.mark.parametrize("asset", ["winding", "texture_layout"])
def test_backspin_checker_structure_fails_closed_on_semantic_mutation(
    asset: str,
) -> None:
    mesh = (ROOT / "data/obj/fbf_backspin_checker_sphere.obj").read_text(
        encoding="ascii"
    )
    texture = (ROOT / "data/obj/fbf_backspin_checker.ppm").read_text(encoding="ascii")
    if asset == "winding":
        lines = mesh.splitlines()
        first_face = next(
            index for index, line in enumerate(lines) if line.startswith("f ")
        )
        tag, a, b, c = lines[first_face].split()
        lines[first_face] = f"{tag} {a} {c} {b}"
        mesh = "\n".join(lines) + "\n"
        message = "face winding is not outward"
    else:
        texture = texture.replace("244 241 228", "32 36 43", 1)
        message = "tile layout changed"

    with pytest.raises(ValueError, match=message):
        validator._validate_backspin_checker_asset_structure(mesh, texture)


def test_backspin_invocations_are_bound_to_exact_commands_and_outputs(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(payload):
        payload["trace"]["argv"][4] = "129"

    _patch_backspin_bundle_json(monkeypatch, "invocations.json", mutate)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "backspin_visual_v3_nonpaper.invocations.trace.argv" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("requirement_id", "kind"),
    [
        ("fig.03", "exact_fixture"),
        ("fig.03", "trace_csv"),
        ("fig.03", "still_image"),
        ("video.02_backspin", "exact_fixture"),
        ("video.02_backspin", "video_clip"),
    ],
)
def test_backspin_requirements_bind_current_positive_deliverables(
    manifest: dict, requirement_id: str, kind: str
) -> None:
    requirement = _requirement(manifest, requirement_id)
    deliverable = next(
        item for item in requirement["deliverables"] if item["kind"] == kind
    )
    deliverable["path"] += ".stale"

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{requirement_id}.backspin_current_boundary.{kind}.path" in error
        for error in errors
    )


@pytest.mark.parametrize("requirement_id", ["fig.03", "video.02_backspin"])
def test_backspin_requirements_cannot_promote_generic_capture_sidecar(
    manifest: dict, requirement_id: str
) -> None:
    requirement = _requirement(manifest, requirement_id)
    sidecar = next(
        item
        for item in requirement["deliverables"]
        if item["kind"] == "capture_sidecar"
    )
    sidecar["validated"] = True
    sidecar["sha256"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{requirement_id}.backspin_current_boundary.capture_sidecar" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("field", "value"), [("status", "complete"), ("fallback_count", 1)]
)
def test_backspin_requirements_remain_partial_and_fallback_free(
    manifest: dict, field: str, value
) -> None:
    _requirement(manifest, "fig.03")[field] = value

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(f"fig.03.backspin_current_boundary.{field}" in error for error in errors)


def test_turntable_truth_accepts_finalized_bundle_before_manifest_integration() -> None:
    errors: list[str] = []

    validator._validate_turntable_v1_truth(
        {"turntable_author_visual_v1_nonpaper": _turntable_truth_payload()},
        ROOT,
        errors,
    )

    assert errors == []


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("cross_lane_substitution_allowed", True),
        ("paper_cpu_native_all_solver_contract_valid", True),
        ("paper_parity", True),
        ("paper_comparable", True),
        ("external_solver_parity", True),
        ("approved_source_golden", True),
        ("timing_verdict", True),
        ("realtime_verdict", True),
        ("automated_semantic_outcome_validated", True),
        ("trace_equivalence_to_rendered_demo", True),
        ("claim_scope", "paper parity"),
        ("claim_boundary", "all equivalent"),
    ],
)
def test_turntable_truth_rejects_promoted_or_erased_claim_boundaries(
    monkeypatch: pytest.MonkeyPatch, field: str, value
) -> None:
    truth = _turntable_truth_payload(finalized=False)
    truth[field] = value
    monkeypatch.setattr(
        validator, "_validate_current_path", lambda *args, **kwargs: None
    )
    monkeypatch.setattr(
        validator, "_validate_artifact_hashes", lambda *args, **kwargs: {}
    )
    errors: list[str] = []

    validator._validate_turntable_v1_truth(
        {"turntable_author_visual_v1_nonpaper": truth}, ROOT, errors
    )

    assert any(
        f"turntable_author_visual_v1_nonpaper.{field}" in error for error in errors
    )


def test_turntable_trace_matrix_recomputes_current_outcomes_and_strict_failure() -> (
    None
):
    bundle = _turntable_bundle()
    errors: list[str] = []
    computed: dict[str, dict[str, dict]] = {
        lane: {} for lane in validator.TURNTABLE_V1_LANES
    }
    for lane in validator.TURNTABLE_V1_LANES:
        for scenario in validator.TURNTABLE_V1_SCENARIOS:
            path = bundle / "traces" / lane / f"{scenario}.csv"
            parsed = validator._read_csv(path, f"test.{lane}.{scenario}", errors)
            metrics = validator._turntable_v1_trace_metrics(
                parsed, scenario, lane, f"test.{lane}.{scenario}", errors
            )
            assert metrics is not None
            computed[lane][scenario] = metrics

    assert errors == []
    assert [
        computed["current_visual"][scenario]["classified_outcome"]
        for scenario in validator.TURNTABLE_V1_SCENARIOS
    ] == ["ejected", "ejected", "captured", "ejected"]
    assert all(
        computed["current_visual"][scenario]["solver_contract_valid"]
        for scenario in validator.TURNTABLE_V1_SCENARIOS
    )
    strict_failures = {
        scenario: result
        for scenario, result in computed["paper_cpu_native"].items()
        if not result["solver_contract_valid"]
    }
    assert set(strict_failures) == {"turntable_author_mu_0_5_omega_2"}
    assert strict_failures["turntable_author_mu_0_5_omega_2"][
        "accepted_at_cap_steps"
    ] == [40]


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("render_lane", "render_binding_lane"),
        ("authority", "physical_outcome_authority"),
        ("comparison", "paper_cpu_native_projection_compared_to_capture"),
    ],
)
def test_turntable_two_lane_separation_mutations_fail_closed(
    mutation: str, message: str
) -> None:
    summary = {
        "render_binding_lane": "current_visual",
        "separate_diagnostic_lane": "paper_cpu_native",
        "cross_lane_substitution_allowed": False,
        "paper_cpu_native_projection_compared_to_capture": False,
        "lanes": {
            lane: {
                "contract": copy.deepcopy(contract),
                "physical_outcome_authority": lane == "current_visual",
            }
            for lane, contract in validator.TURNTABLE_V1_LANES.items()
        },
    }
    if mutation == "render_lane":
        summary["render_binding_lane"] = "paper_cpu_native"
    elif mutation == "authority":
        summary["lanes"]["paper_cpu_native"]["physical_outcome_authority"] = True
    else:
        summary["paper_cpu_native_projection_compared_to_capture"] = True
    errors: list[str] = []

    validator._validate_turntable_v1_lane_separation(summary, "test.lanes", errors)

    assert any(message in error for error in errors)


def test_turntable_strict_diagnostic_erasure_is_detected() -> None:
    bundle = _turntable_bundle()
    errors: list[str] = []
    strict: dict[str, dict] = {}
    for scenario in validator.TURNTABLE_V1_SCENARIOS:
        parsed = validator._read_csv(
            bundle / "traces/paper_cpu_native" / f"{scenario}.csv",
            f"test.strict.{scenario}",
            errors,
        )
        assert parsed is not None
        fields, rows = parsed
        rows = copy.deepcopy(rows)
        if scenario == "turntable_author_mu_0_5_omega_2":
            rows[40]["status"] = "success"
            rows[40]["residual"] = "0"
        metrics = validator._turntable_v1_trace_metrics(
            (fields, rows),
            scenario,
            "paper_cpu_native",
            f"test.strict.{scenario}",
            errors,
        )
        assert metrics is not None
        strict[scenario] = metrics
    validator._validate_turntable_v1_strict_diagnostic(
        {"paper_cpu_native": strict}, "test.strict", errors
    )

    assert any(
        "expected only mu=.5/omega=2 strict failure" in error for error in errors
    )


@pytest.mark.parametrize("mutation", ["fallback", "outcome"])
def test_turntable_current_trace_semantic_mutations_fail_closed(
    mutation: str,
) -> None:
    scenario = "turntable_author_mu_0_2_omega_2"
    path = _turntable_bundle() / "traces/current_visual" / f"{scenario}.csv"
    read_errors: list[str] = []
    parsed = validator._read_csv(path, "test.turntable", read_errors)
    assert parsed is not None and read_errors == []
    fields, rows = parsed
    rows = copy.deepcopy(rows)
    if mutation == "fallback":
        rows[-1]["fallbacks"] = "1"
        message = "boxed-LCP fallback observed"
    else:
        rows[-1]["x"] = "1.0"
        rows[-1]["y"] = "0.0"
        rows[-1]["z"] = "0.25"
        rows[-1]["contacts"] = "1"
        message = "current visual outcome"
    errors: list[str] = []

    validator._turntable_v1_trace_metrics(
        (fields, rows), scenario, "current_visual", "test.turntable", errors
    )

    assert any(message in error for error in errors)


def test_turntable_current_trace_projection_equals_rendered_capture() -> None:
    bundle = _turntable_bundle()
    errors: list[str] = []
    for scenario, contract in validator.TURNTABLE_V1_SCENARIOS.items():
        parsed = validator._read_csv(
            bundle / "traces/current_visual" / f"{scenario}.csv",
            f"test.{scenario}.trace",
            errors,
        )
        metrics = validator._turntable_v1_trace_metrics(
            parsed, scenario, "current_visual", f"test.{scenario}.trace", errors
        )
        timeline = json.loads(
            (bundle / contract["capture_id"] / "timeline.json").read_text(
                encoding="utf-8"
            )
        )
        projection = validator._turntable_v1_capture_projection(
            timeline, f"test.{scenario}.timeline", errors
        )
        assert metrics is not None
        assert metrics["projection"] == projection

    assert errors == []


def test_turntable_projection_mutation_is_detected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    truth = _turntable_truth_payload()

    def mutate(payload):
        payload["steps"][120]["solver_diagnostics"]["contacts"] += 1

    _patch_turntable_bundle_json(
        monkeypatch, "turntable_author_mu02_omega2/timeline.json", mutate
    )
    errors: list[str] = []

    validator._validate_turntable_v1_truth(
        {"turntable_author_visual_v1_nonpaper": truth}, ROOT, errors
    )

    assert any(
        "current trace/capture full projection differs" in error for error in errors
    )


def test_turntable_group_binds_media_panel_order_and_outcome_steps(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_turntable_verification_for_group_test(monkeypatch)
    bundle = _turntable_bundle()
    group = json.loads(
        (bundle / "groups/turntable_author/metadata.json").read_text(encoding="utf-8")
    )
    hashes = {
        key: _digest(ROOT / relative)
        for key, relative in validator.TURNTABLE_V1_ARTIFACT_TARGETS.items()
        if (ROOT / relative).is_file()
    }
    errors: list[str] = []

    validator._validate_turntable_v1_group(
        group, bundle, hashes, "test.turntable", errors
    )

    assert errors == []


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("member_order", "member_order"),
        ("panel_step", "source_frames"),
        ("panel_label", "panel_validation.labels"),
        ("media_width", "probe.streams[0].width"),
    ],
)
def test_turntable_group_semantic_mutations_fail_closed(
    monkeypatch: pytest.MonkeyPatch, mutation: str, message: str
) -> None:
    _patch_turntable_verification_for_group_test(monkeypatch)
    bundle = _turntable_bundle()
    group = json.loads(
        (bundle / "groups/turntable_author/metadata.json").read_text(encoding="utf-8")
    )
    if mutation == "member_order":
        group["member_order"][0], group["member_order"][1] = (
            group["member_order"][1],
            group["member_order"][0],
        )
    elif mutation == "panel_step":
        group["panel_validation"]["source_frames"][0]["step"] += 1
    elif mutation == "panel_label":
        group["panel_validation"]["labels"][0] = "stale"
    else:
        stream = next(
            item
            for item in group["media_validation"]["probe"]["streams"]
            if item.get("width")
        )
        stream["width"] = 1
    hashes = {
        key: _digest(ROOT / relative)
        for key, relative in validator.TURNTABLE_V1_ARTIFACT_TARGETS.items()
        if (ROOT / relative).is_file()
    }
    errors: list[str] = []

    validator._validate_turntable_v1_group(
        group, bundle, hashes, "test.turntable", errors
    )

    assert any(message in error for error in errors)


def test_turntable_segmented_renderer_assets_are_structurally_bound() -> None:
    obj = (ROOT / "data/obj/fbf_author_turntable_disc.obj").read_text(encoding="utf-8")
    mtl = (ROOT / "data/obj/fbf_author_turntable_disc.mtl").read_text(encoding="utf-8")

    result = validator._validate_turntable_v1_visual_resources(obj, mtl)

    assert result["vertex_count"] == 130
    assert result["face_count"] == 192
    assert result["top_wedge_count"] == 8
    assert result["top_triangles_per_wedge"] == 8
    assert result["registration_material"] == "FbfTurntableCoral"


@pytest.mark.parametrize("mutation", ["sector_order", "diffuse_color"])
def test_turntable_renderer_asset_semantic_mutations_fail_closed(
    mutation: str,
) -> None:
    obj = (ROOT / "data/obj/fbf_author_turntable_disc.obj").read_text(encoding="utf-8")
    mtl = (ROOT / "data/obj/fbf_author_turntable_disc.mtl").read_text(encoding="utf-8")
    if mutation == "sector_order":
        obj = obj.replace("usemtl FbfTurntableCoral", "usemtl FbfTurntableDark", 1)
        message = "material sector order changed"
    else:
        mtl = mtl.replace("Kd 0.940000 0.310000 0.270000", "Kd 0 0 0", 1)
        message = "diffuse colors changed"

    with pytest.raises(ValueError, match=message):
        validator._validate_turntable_v1_visual_resources(obj, mtl)


def test_turntable_physics_contract_excludes_renderer_identity() -> None:
    scenario = "turntable_author_mu_0_2_omega_2"
    timeline = json.loads(
        (
            _turntable_bundle()
            / validator.TURNTABLE_V1_SCENARIOS[scenario]["capture_id"]
            / "timeline.json"
        ).read_text(encoding="utf-8")
    )
    source_hashes = {
        key: _digest(ROOT / relative)
        for key, relative in validator.TURNTABLE_V1_ARTIFACT_TARGETS.items()
        if not relative.startswith(validator.TURNTABLE_V1_BUNDLE)
    }
    errors: list[str] = []

    validator._validate_turntable_v1_physics_contract(
        timeline["physics_contract"],
        scenario,
        "dart_best",
        "dart_demos",
        source_hashes,
        "test.physics",
        errors,
    )

    assert errors == []
    assert timeline["physics_contract"]["visual_asset_identity"] is None


def test_turntable_physics_contract_rejects_renderer_asset_injection() -> None:
    scenario = "turntable_author_mu_0_2_omega_2"
    timeline = json.loads(
        (
            _turntable_bundle()
            / validator.TURNTABLE_V1_SCENARIOS[scenario]["capture_id"]
            / "timeline.json"
        ).read_text(encoding="utf-8")
    )
    timeline["physics_contract"]["visual_asset_identity"] = {"sha256": "0" * 64}
    source_hashes = {
        key: _digest(ROOT / relative)
        for key, relative in validator.TURNTABLE_V1_ARTIFACT_TARGETS.items()
        if not relative.startswith(validator.TURNTABLE_V1_BUNDLE)
    }
    errors: list[str] = []

    validator._validate_turntable_v1_physics_contract(
        timeline["physics_contract"],
        scenario,
        "dart_best",
        "dart_demos",
        source_hashes,
        "test.physics",
        errors,
    )

    assert any("visual_asset_identity" in error for error in errors)


@pytest.mark.parametrize(
    ("source_key", "message"),
    [
        ("demo_source", "source_identity.demo_source.sha256"),
        (
            "author_turntable_visual_obj",
            "source_identity.author_turntable_visual_obj.sha256",
        ),
        (
            "author_turntable_visual_mtl",
            "source_identity.author_turntable_visual_mtl.sha256",
        ),
    ],
)
def test_turntable_source_identity_mutations_fail_closed(
    monkeypatch: pytest.MonkeyPatch, source_key: str, message: str
) -> None:
    truth = _turntable_truth_payload()

    def mutate(payload):
        payload["source_identity"][source_key]["sha256"] = "0" * 64

    _patch_turntable_bundle_json(monkeypatch, "metadata.json", mutate)
    errors: list[str] = []

    validator._validate_turntable_v1_truth(
        {"turntable_author_visual_v1_nonpaper": truth}, ROOT, errors
    )

    assert any(message in error for error in errors)


def test_turntable_cross_lane_physics_mutation_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    truth = _turntable_truth_payload()

    def mutate(payload):
        contract = payload["capture_source_contract"]["physics_contract_queries"][
            "trace"
        ]["paper_cpu"]["turntable_author_mu_0_2_omega_2"]
        contract["control"]["duration_seconds"] = 5

    _patch_turntable_bundle_json(monkeypatch, "metadata.json", mutate)
    errors: list[str] = []

    validator._validate_turntable_v1_truth(
        {"turntable_author_visual_v1_nonpaper": truth}, ROOT, errors
    )

    assert any(
        "control.duration_seconds" in error
        or "diagnostic lane changed non-solver physics" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing", "expected 58 entries"),
        ("bytes", ".bytes: expected"),
        ("sha256", ".sha256: expected"),
    ],
)
def test_turntable_artifact_index_is_exact_and_content_bound(
    monkeypatch: pytest.MonkeyPatch, mutation: str, message: str
) -> None:
    truth = _turntable_truth_payload()

    def mutate(payload):
        if mutation == "missing":
            payload["artifacts"].pop()
            payload["artifact_count"] -= 1
        elif mutation == "bytes":
            payload["artifacts"][0]["bytes"] += 1
        else:
            payload["artifacts"][0]["sha256"] = "0" * 64

    _patch_turntable_bundle_json(monkeypatch, "artifact-index.json", mutate)
    errors: list[str] = []

    validator._validate_turntable_v1_truth(
        {"turntable_author_visual_v1_nonpaper": truth}, ROOT, errors
    )

    assert any(
        "turntable_author_visual_v1_nonpaper.artifact_index" in error
        and message in error
        for error in errors
    )


def test_turntable_manual_record_binds_verdicts_and_representative_artifacts() -> None:
    errors: list[str] = []

    validator._validate_turntable_v1_manual(
        _turntable_bundle(), {}, "test.turntable", errors
    )

    assert errors == []


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("verdict", "verdicts"),
        ("hash", "sha256"),
        ("observation", "observation"),
    ],
)
def test_turntable_manual_record_mutations_fail_closed(
    monkeypatch: pytest.MonkeyPatch, mutation: str, message: str
) -> None:
    def mutate(payload):
        if mutation == "verdict":
            payload["verdicts"]["turntable_rotation_visible"] = False
        elif mutation == "hash":
            payload["representative_artifacts"][0]["sha256"] = "0" * 64
        else:
            payload["representative_artifacts"][0]["observation"] = ""

    _patch_turntable_bundle_json(monkeypatch, "manual-inspection.json", mutate)
    errors: list[str] = []

    validator._validate_turntable_v1_manual(
        _turntable_bundle(), {}, "test.turntable", errors
    )

    assert any(message in error for error in errors)


def test_turntable_requirement_boundaries_accept_fig4_and_video4_paths(
    manifest: dict,
) -> None:
    errors: list[str] = []

    validator._validate_turntable_requirement_boundaries(
        _turntable_requirement_map(manifest), errors
    )

    assert errors == []


@pytest.mark.parametrize(
    ("requirement_id", "kind"),
    [
        ("fig.04", "exact_fixture"),
        ("fig.04", "trace_csv"),
        ("fig.04", "still_image"),
        ("video.04_turntable", "exact_fixture"),
        ("video.04_turntable", "video_clip"),
    ],
)
def test_turntable_requirement_boundaries_reject_stale_positive_paths(
    manifest: dict, requirement_id: str, kind: str
) -> None:
    by_id = _turntable_requirement_map(manifest)
    deliverable = next(
        item for item in by_id[requirement_id]["deliverables"] if item["kind"] == kind
    )
    deliverable["path"] += ".stale"
    errors: list[str] = []

    validator._validate_turntable_requirement_boundaries(by_id, errors)

    assert any(
        f"{requirement_id}.turntable_current_boundary.{kind}.path" in error
        for error in errors
    )


@pytest.mark.parametrize("requirement_id", ["fig.04", "video.04_turntable"])
def test_turntable_requirement_boundaries_reject_promoted_capture_sidecar(
    manifest: dict, requirement_id: str
) -> None:
    by_id = _turntable_requirement_map(manifest)
    sidecar = next(
        item
        for item in by_id[requirement_id]["deliverables"]
        if item["kind"] == "capture_sidecar"
    )
    sidecar["validated"] = True
    errors: list[str] = []

    validator._validate_turntable_requirement_boundaries(by_id, errors)

    assert any(
        f"{requirement_id}.turntable_current_boundary.capture_sidecar" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("field", "value"), [("status", "complete"), ("fallback_count", 1)]
)
def test_turntable_requirement_boundaries_remain_partial_and_fallback_free(
    manifest: dict, field: str, value
) -> None:
    by_id = _turntable_requirement_map(manifest)
    by_id["fig.04"][field] = value
    errors: list[str] = []

    validator._validate_turntable_requirement_boundaries(by_id, errors)

    assert any(
        f"fig.04.turntable_current_boundary.{field}" in error for error in errors
    )


def test_turntable_requirement_boundary_rejects_zero_slip_claim(
    manifest: dict,
) -> None:
    by_id = _turntable_requirement_map(manifest)
    by_id["fig.04"]["claim"] = "The retained cell proves zero-slip co-rotation."
    errors: list[str] = []

    validator._validate_turntable_requirement_boundaries(by_id, errors)

    assert any("fig.04.turntable_current_boundary.claim" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("paper_parity", True),
        ("paper_comparable", True),
        ("external_solver_parity", True),
        ("approved_source_golden", True),
        ("timing_verdict", True),
        ("realtime_verdict", True),
        ("automated_semantic_outcome_validated", True),
        ("trace_equivalence_to_rendered_demo", True),
        ("strict_rigid_body_rest_proven", True),
        ("capture_sidecar_deliverable_validated", True),
    ],
)
def test_painleve_truth_rejects_promoted_claims(
    manifest: dict, field: str, value
) -> None:
    _truth(manifest, "painleve_proxy_visual_v1_nonpaper")[field] = value

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"painleve_proxy_visual_v1_nonpaper.{field}" in error for error in errors
    )


def test_painleve_truth_rejects_noncurrent_bundle_and_hash(manifest: dict) -> None:
    painleve = _truth(manifest, "painleve_proxy_visual_v1_nonpaper")
    painleve["bundle"] += "-missing"
    painleve["artifact_hashes"]["metadata.json"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "painleve_proxy_visual_v1_nonpaper.bundle: expected current path" in error
        for error in errors
    )
    assert any(
        "painleve_proxy_visual_v1_nonpaper.artifact_hashes['metadata.json']" in error
        for error in errors
    )


def test_unreadable_painleve_metadata_fails_closed_without_crashing(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        if (
            name == "metadata.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.PAINLEVE_V1_BUNDLE)
        ):
            errors.append(f"{location}: simulated unreadable metadata")
            return None
        return original(bundle, name, location, errors)

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("simulated unreadable metadata" in error for error in errors)


def test_painleve_trace_semantics_are_recomputed_from_raw_csv(
    manifest: dict, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = validator._read_csv

    def read_csv(path, location, errors):
        payload = original(path, location, errors)
        if payload is not None and path.as_posix().endswith(
            "fig05_painleve_proxy_current_v1/traces/painleve_mu_0_55.csv"
        ):
            fields, rows = payload
            rows = copy.deepcopy(rows)
            for row in rows:
                row["up_z"] = "0.9"
                row["z"] = "0.5"
            return fields, rows
        return payload

    monkeypatch.setattr(validator, "_read_csv", read_csv)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("mu=.55 never crossed the tumble threshold" in error for error in errors)


@pytest.mark.parametrize("mutation", ["missing", "unexpected"])
def test_painleve_artifact_index_requires_exact_membership(
    manifest: dict, monkeypatch: pytest.MonkeyPatch, mutation: str
) -> None:
    original = validator._read_bundle_json

    def read_bundle(bundle, name, location, errors):
        payload = original(bundle, name, location, errors)
        if (
            payload is not None
            and name == "artifact-index.json"
            and bundle is not None
            and bundle.as_posix().endswith(validator.PAINLEVE_V1_BUNDLE)
        ):
            payload = copy.deepcopy(payload)
            if mutation == "missing":
                payload["artifacts"].pop()
                payload["artifact_count"] -= 1
            else:
                payload["artifacts"].append(
                    {"path": "unexpected.txt", "bytes": 0, "sha256": "0" * 64}
                )
                payload["artifact_count"] += 1
        return payload

    monkeypatch.setattr(validator, "_read_bundle_json", read_bundle)

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "painleve_proxy_visual_v1_nonpaper.artifact_index" in error for error in errors
    )


@pytest.mark.parametrize(
    ("requirement_id", "kind"),
    [
        ("fig.05", "trace_csv"),
        ("fig.05", "still_image"),
        ("video.05_painleve", "video_clip"),
    ],
)
def test_painleve_requirements_bind_current_positive_deliverables(
    manifest: dict, requirement_id: str, kind: str
) -> None:
    requirement = _requirement(manifest, requirement_id)
    deliverable = next(
        item for item in requirement["deliverables"] if item["kind"] == kind
    )
    deliverable["path"] = deliverable["path"].replace("current_v1", "missing_v1")

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{requirement_id}.painleve_current_boundary.{kind}.path" in error
        for error in errors
    )


@pytest.mark.parametrize("requirement_id", ["fig.05", "video.05_painleve"])
def test_painleve_requirements_cannot_promote_generic_capture_sidecar(
    manifest: dict, requirement_id: str
) -> None:
    requirement = _requirement(manifest, requirement_id)
    sidecar = next(
        item
        for item in requirement["deliverables"]
        if item["kind"] == "capture_sidecar"
    )
    sidecar["validated"] = True
    sidecar["sha256"] = "0" * 64

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        f"{requirement_id}.painleve_current_boundary.capture_sidecar" in error
        for error in errors
    )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: data["requirements"].append(
                copy.deepcopy(data["requirements"][0])
            ),
            "duplicate ids",
        ),
        (
            lambda data: data["requirements"].pop(1),
            "missing canonical ids ['fig.01']",
        ),
        (
            lambda data: _requirement(data, "fig.01").update({"status": "verified"}),
            "unsupported status 'verified'",
        ),
    ],
)
def test_canonical_coverage_and_statuses_are_enforced(
    manifest: dict, mutate, message: str
) -> None:
    mutate(manifest)
    assert any(
        message in error for error in validator.validate_manifest(manifest, ROOT)
    )


@pytest.mark.parametrize(
    ("object_name", "location"),
    [
        ("manifest", "manifest"),
        ("current_truth", "current_truth"),
        ("requirement", "requirements[0]"),
        ("deliverable", "requirements[1].deliverables[0]"),
        ("current_evidence", "requirements[0].current_evidence[0]"),
        ("source", "requirements[0].source"),
        ("configuration", "requirements[0].configuration"),
        ("capture_plan", "requirements[0].capture_plan"),
    ],
)
def test_manifest_claim_objects_reject_unknown_completion_keys(
    manifest: dict,
    object_name: str,
    location: str,
) -> None:
    requirement = manifest["requirements"][0]
    targets = {
        "manifest": manifest,
        "current_truth": manifest["current_truth"],
        "requirement": requirement,
        "deliverable": manifest["requirements"][1]["deliverables"][0],
        "current_evidence": requirement["current_evidence"][0],
        "source": requirement["source"],
        "configuration": requirement["configuration"],
        "capture_plan": requirement["capture_plan"],
    }
    targets[object_name]["paper_parity_complete"] = True

    errors = validator.validate_manifest(manifest, ROOT)

    assert f"{location}: unexpected keys ['paper_parity_complete']" in errors


@pytest.mark.parametrize(
    ("object_name", "location"),
    [
        ("sources", "sources"),
        ("paper_source", "sources.paper"),
        (
            "author_scene_hashes",
            "sources.author_code.scene_source_sha256",
        ),
        (
            "author_solver_hashes",
            "sources.author_code.solver_source_sha256",
        ),
        ("source_audit", "source_audit"),
        ("configuration_profiles", "configuration_profiles"),
        (
            "nested_configuration_profile",
            "configuration_profiles.turntable.support",
        ),
        ("capture_profiles", "capture_profiles"),
        ("capture_profile", "capture_profiles.video"),
    ],
)
def test_other_manifest_schema_objects_reject_unknown_completion_keys(
    manifest: dict,
    object_name: str,
    location: str,
) -> None:
    targets = {
        "sources": manifest["sources"],
        "paper_source": manifest["sources"]["paper"],
        "author_scene_hashes": manifest["sources"]["author_code"][
            "scene_source_sha256"
        ],
        "author_solver_hashes": manifest["sources"]["author_code"][
            "solver_source_sha256"
        ],
        "source_audit": manifest["source_audit"],
        "configuration_profiles": manifest["configuration_profiles"],
        "nested_configuration_profile": manifest["configuration_profiles"]["turntable"][
            "support"
        ],
        "capture_profiles": manifest["capture_profiles"],
        "capture_profile": manifest["capture_profiles"]["video"],
    }
    targets[object_name]["paper_parity_complete"] = True

    errors = validator.validate_manifest(manifest, ROOT)

    assert f"{location}: unexpected keys ['paper_parity_complete']" in errors


@pytest.mark.parametrize(
    ("object_name", "location"),
    [
        (
            "backspin_truth",
            "current_truth.backspin_visual_v3_nonpaper",
        ),
        (
            "card_compact",
            "current_truth.card_house_manifold_sensitivity_v2_nonpaper.compact",
        ),
        (
            "one_thread",
            "current_truth.literal_wedge_exact_dynamics_nonpaper.one_thread",
        ),
        (
            "solver_options",
            "current_truth.literal_wedge_exact_dynamics_nonpaper.solver_options",
        ),
        (
            "painleve_mu05",
            "current_truth.painleve_proxy_visual_v1_nonpaper.mu05",
        ),
    ],
)
def test_nested_current_truth_objects_reject_unknown_completion_keys(
    manifest: dict,
    object_name: str,
    location: str,
) -> None:
    current_truth = manifest["current_truth"]
    targets = {
        "backspin_truth": current_truth["backspin_visual_v3_nonpaper"],
        "card_compact": current_truth["card_house_manifold_sensitivity_v2_nonpaper"][
            "compact"
        ],
        "one_thread": current_truth["literal_wedge_exact_dynamics_nonpaper"][
            "one_thread"
        ],
        "solver_options": current_truth["literal_wedge_exact_dynamics_nonpaper"][
            "solver_options"
        ],
        "painleve_mu05": current_truth["painleve_proxy_visual_v1_nonpaper"]["mu05"],
    }
    targets[object_name]["paper_parity_complete"] = True

    errors = validator.validate_manifest(manifest, ROOT)

    assert f"{location}: unexpected keys ['paper_parity_complete']" in errors


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("completion_rule", "completion_rule: expected a non-empty string"),
        ("snapshot_date", "snapshot_date: expected a non-empty string"),
        ("author_code", "sources.author_code: expected an object"),
        ("pr3377", "sources.pr3377: expected an object"),
        (
            "author_scene_hashes",
            "sources.author_code.scene_source_sha256: expected an object",
        ),
        (
            "author_solver_hashes",
            "sources.author_code.solver_source_sha256: expected an object",
        ),
        ("pr3377_status", "sources.pr3377.status: expected a scalar value"),
        (
            "source_audit_status",
            "source_audit.local_capture_status: expected a scalar value",
        ),
        (
            "capture_toolchain",
            "capture_profiles.video.toolchain: expected a flat list",
        ),
        (
            "requirement_orientation",
            "requirements[3].configuration.orientation_cue: expected a scalar value",
        ),
        (
            "profile_support",
            "configuration_profiles.turntable.support: expected an object",
        ),
        (
            "profile_cells_scalar",
            "configuration_profiles.turntable.cells: expected a list of objects",
        ),
        (
            "profile_cells_nested_list",
            "configuration_profiles.turntable.cells[0]: expected an object",
        ),
    ],
)
def test_manifest_object_contracts_reject_object_smuggling(
    manifest: dict,
    mutation: str,
    message: str,
) -> None:
    claim = {"paper_parity_complete": True}
    if mutation in {"completion_rule", "snapshot_date"}:
        manifest[mutation] = claim
    elif mutation in {"author_code", "pr3377"}:
        manifest["sources"][mutation] = "paper parity complete"
    elif mutation == "author_scene_hashes":
        manifest["sources"]["author_code"][
            "scene_source_sha256"
        ] = "paper parity complete"
    elif mutation == "author_solver_hashes":
        manifest["sources"]["author_code"][
            "solver_source_sha256"
        ] = "paper parity complete"
    elif mutation == "pr3377_status":
        manifest["sources"]["pr3377"]["status"] = claim
    elif mutation == "source_audit_status":
        manifest["source_audit"]["local_capture_status"] = claim
    elif mutation == "capture_toolchain":
        manifest["capture_profiles"]["video"]["toolchain"] = claim
    elif mutation == "requirement_orientation":
        _requirement(manifest, "fig.03")["configuration"]["orientation_cue"] = claim
    elif mutation == "profile_support":
        manifest["configuration_profiles"]["turntable"][
            "support"
        ] = "paper parity complete"
    elif mutation == "profile_cells_scalar":
        manifest["configuration_profiles"]["turntable"][
            "cells"
        ] = "paper parity complete"
    else:
        manifest["configuration_profiles"]["turntable"]["cells"] = [[claim]]

    errors = validator.validate_manifest(manifest, ROOT)

    assert message in errors


def test_manifest_key_contracts_reject_missing_members(manifest: dict) -> None:
    manifest.pop("snapshot_date")
    _requirement(manifest, "teaser")["capture_plan"].pop("shots")

    errors = validator.validate_manifest(manifest, ROOT)

    assert "manifest: missing keys ['snapshot_date']" in errors
    assert "requirements[0].capture_plan: missing keys ['shots']" in errors


def test_recursive_object_contracts_cover_every_live_manifest_object(
    manifest: dict,
) -> None:
    def collect_paths(value, path: str, delegated: set[str]) -> set[str]:
        paths: set[str] = set()
        if isinstance(value, dict):
            paths.add(path)
            if path in delegated:
                return paths
            for field, child in value.items():
                paths.update(collect_paths(child, f"{path}.{field}", delegated))
        elif isinstance(value, list):
            for item in value:
                paths.update(collect_paths(item, f"{path}[]", delegated))
        return paths

    profile_paths: set[str] = set()
    for profile_id, profile in manifest["configuration_profiles"].items():
        profile_paths.update(collect_paths(profile, profile_id, set()))
    assert profile_paths <= set(validator.CONFIGURATION_PROFILE_OBJECT_KEYS)

    truth_paths: set[str] = set()
    for record_id, record in manifest["current_truth"].items():
        if isinstance(record, dict):
            truth_paths.update(
                collect_paths(
                    record,
                    record_id,
                    validator.CURRENT_TRUTH_DELEGATED_OBJECT_PATHS,
                )
            )
    assert truth_paths <= (
        set(validator.CURRENT_TRUTH_OBJECT_KEYS)
        | validator.CURRENT_TRUTH_DELEGATED_OBJECT_PATHS
    )


def test_partial_requirement_needs_reproduction_commands(manifest: dict) -> None:
    requirement = _requirement(manifest, "fig.01")
    requirement["commands"] = []
    requirement["capture_plan"]["commands"] = []

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("fig.01" not in error and ".commands" in error for error in errors)
    assert any("capture_plan.commands" in error for error in errors)


def test_every_listed_local_evidence_path_must_exist(manifest: dict) -> None:
    evidence = _requirement(manifest, "fig.03")["current_evidence"][0]
    evidence["path"] = "docs/dev_tasks/fbf_exact_coulomb_friction/missing.csv"

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("local evidence does not exist" in error for error in errors)


def test_complete_requires_all_validated_deliverables_and_zero_fallback(
    manifest: dict,
) -> None:
    requirement = _requirement(manifest, "teaser")
    requirement["status"] = "complete"
    requirement["fallback_count"] = 3

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("complete status is missing deliverables" in error for error in errors)
    assert any(
        "complete status requires fallback_count == 0" in error for error in errors
    )
    assert any("complete status cannot retain blockers" in error for error in errors)


def test_video_segments_are_fixed_and_cover_all_82_seconds(
    manifest: dict,
) -> None:
    segments = [
        _requirement(manifest, requirement_id)["source"]
        for requirement_id in validator.VIDEO_SEGMENTS
    ]
    assert segments[0]["start_seconds"] == 0
    assert segments[-1]["end_seconds"] == 82
    assert all(
        left["end_seconds"] == right["start_seconds"]
        for left, right in zip(segments, segments[1:])
    )

    segments[1]["start_seconds"] = 3
    errors = validator.validate_manifest(manifest, ROOT)
    assert any(
        "video.02_backspin.source: expected timeline" in error for error in errors
    )


def test_readme_cannot_self_attest_as_every_complete_deliverable(
    manifest: dict,
) -> None:
    requirement = _requirement(manifest, "teaser")
    requirement["status"] = "complete"
    requirement["fallback_count"] = 0
    requirement["blockers"] = []
    requirement["capture_plan"]["blockers"] = []
    readme = ROOT / "README.md"
    requirement["deliverables"] = [
        {
            "kind": kind,
            "path": str(readme.relative_to(ROOT)),
            "validated": True,
            "sha256": _digest(readme),
        }
        for kind in requirement["required_deliverables"]
    ]

    errors = validator.validate_manifest(manifest, ROOT)

    assert any("still_image requires one of" in error for error in errors)
    assert any("capture_sidecar requires one of" in error for error in errors)
    assert any("must use distinct artifacts" in error for error in errors)


def test_fabricated_one_step_one_pixel_completion_is_rejected(
    manifest: dict, tmp_path: Path
) -> None:
    complete, _ = _isolated_complete_teaser(manifest, tmp_path)

    errors = validator.validate_manifest(complete, tmp_path)

    assert any(
        "hand-authored solver records are not accepted" in error for error in errors
    )
    assert any("non-self-attesting production attestation" in error for error in errors)
    assert any("at least two completed runtime steps" in error for error in errors)


def test_native_looking_hand_authored_capture_still_fails_closed(
    manifest: dict, tmp_path: Path
) -> None:
    complete, teaser = _isolated_complete_teaser(manifest, tmp_path)
    deliverable = next(
        item for item in teaser["deliverables"] if item["kind"] == "capture_sidecar"
    )
    media = tmp_path / "still.png"
    diagnostics = {
        "available": True,
        "solver": "ExactCoulombFbfConstraintSolver",
        "status": "success",
        "fbf_status": "success",
        "contacts": 1,
        "total_iterations": 1,
        "exact_solves": 1,
        "exact_attempts": 1,
        "accepted_at_cap": 0,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
        "warm_starts": 0,
        "worst_residual": 5e-7,
    }
    payload = {
        "schema_version": "dart.demos_headless_timeline/v1",
        "requirement_ids": ["teaser"],
        "runtime_command": "dart-demos --headless --headless-sidecar capture.json",
        "build": {"dart_version": "forged", "mode": "Release"},
        "scene": "forged_scene",
        "active_scene": "forged_scene",
        "total_steps": 2,
        "completed_steps": 2,
        "actions": [],
        "steps": [
            {"step": 0, "solver_diagnostics": diagnostics},
            {"step": 1, "solver_diagnostics": diagnostics},
            {"step": 2, "solver_diagnostics": diagnostics},
        ],
        "shots": [
            {
                "step": 2,
                "success": True,
                "path": media.name,
                "sha256": _digest(media),
                "solver_diagnostics": diagnostics,
            }
        ],
        "solver_diagnostics": diagnostics,
    }
    path = tmp_path / deliverable["path"]
    _write_json(path, payload)
    deliverable["sha256"] = _digest(path)

    errors = validator.validate_manifest(complete, tmp_path)

    assert any("non-self-attesting production attestation" in error for error in errors)


def test_complete_deliverable_hash_tampering_is_rejected(
    manifest: dict, tmp_path: Path
) -> None:
    complete, _ = _isolated_complete_teaser(manifest, tmp_path)
    still = tmp_path / "still.png"
    still.write_bytes(still.read_bytes() + b"tamper")

    errors = validator.validate_manifest(complete, tmp_path)

    assert any("sha256: digest mismatch" in error for error in errors)


def test_every_complete_deliverable_requires_a_sha256(
    manifest: dict, tmp_path: Path
) -> None:
    complete, teaser = _isolated_complete_teaser(manifest, tmp_path)
    teaser["deliverables"][0].pop("sha256")

    errors = validator.validate_manifest(complete, tmp_path)

    assert any(
        "validated deliverable requires lowercase SHA-256" in error for error in errors
    )


def test_validated_partial_deliverable_requires_a_sha256(manifest: dict) -> None:
    deliverable = _requirement(manifest, "fig.01")["deliverables"][0]
    assert deliverable["validated"] is True
    deliverable.pop("sha256")

    errors = validator.validate_manifest(manifest, ROOT)

    assert any(
        "validated deliverable requires lowercase SHA-256" in error for error in errors
    )


def test_trace_validation_accepts_native_literal_capture_aliases() -> None:
    fieldnames = [
        "step",
        "contacts",
        "boxed_lcp_fallbacks_delta",
        "exact_status",
        "residual",
    ]
    rows = [
        {
            "step": "0",
            "contacts": "0",
            "boxed_lcp_fallbacks_delta": "0",
            "exact_status": "not_run",
            "residual": "nan",
        },
        {
            "step": "1",
            "contacts": "96",
            "boxed_lcp_fallbacks_delta": "0",
            "exact_status": "success",
            "residual": "9.9e-7",
        },
    ]
    errors: list[str] = []

    validator._validate_trace_csv((fieldnames, rows), "literal_trace", errors)

    assert errors == []


def test_trace_validation_rejects_native_literal_capture_fallback() -> None:
    fieldnames = [
        "step",
        "contacts",
        "boxed_lcp_fallbacks_delta",
        "exact_status",
        "residual",
    ]
    rows = [
        {
            "step": "0",
            "contacts": "0",
            "boxed_lcp_fallbacks_delta": "0",
            "exact_status": "not_run",
            "residual": "nan",
        },
        {
            "step": "1",
            "contacts": "96",
            "boxed_lcp_fallbacks_delta": "1",
            "exact_status": "success",
            "residual": "9.9e-7",
        },
    ]
    errors: list[str] = []

    validator._validate_trace_csv((fieldnames, rows), "literal_trace", errors)

    assert any("completion requires 0" in error for error in errors)


@pytest.mark.parametrize(
    ("kind", "mutate", "message"),
    [
        (
            "golden_diff",
            lambda payload: payload.update({"pass": False}),
            "pass: expected computed value True",
        ),
        (
            "golden_diff",
            lambda payload: payload.update({"schema_version": "unrecognized/v1"}),
            "schema_version: expected 'dart.image_diff/v1'",
        ),
        (
            "claim_map",
            lambda payload: payload.update({"requirement_ids": ["fig.01"]}),
            "claim map must bind exactly ['teaser']",
        ),
        (
            "capture_sidecar",
            lambda payload: None,
            "non-self-attesting production attestation",
        ),
    ],
)
def test_complete_structured_payload_failures_are_rejected(
    manifest: dict,
    tmp_path: Path,
    kind: str,
    mutate,
    message: str,
) -> None:
    complete, teaser = _isolated_complete_teaser(manifest, tmp_path)
    deliverable = next(item for item in teaser["deliverables"] if item["kind"] == kind)
    path = tmp_path / deliverable["path"]
    payload = json.loads(path.read_text(encoding="utf-8"))
    mutate(payload)
    _write_json(path, payload)
    deliverable["sha256"] = _digest(path)

    errors = validator.validate_manifest(complete, tmp_path)

    assert any(message in error for error in errors)


def test_claim_map_rejects_unrelated_or_altered_manifest_claim(
    manifest: dict, tmp_path: Path
) -> None:
    complete, teaser = _isolated_complete_teaser(manifest, tmp_path)
    deliverable = next(
        item for item in teaser["deliverables"] if item["kind"] == "claim_map"
    )
    path = tmp_path / deliverable["path"]
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["manifest_claim"] = "An unrelated claim."
    payload["claims"][0]["claim"] = "An altered claim."
    _write_json(path, payload)
    deliverable["sha256"] = _digest(path)

    errors = validator.validate_manifest(complete, tmp_path)

    assert any("must exactly match the manifest claim" in error for error in errors)
    assert any(
        "exact manifest claim with non-empty support" in error for error in errors
    )


def test_author_card_house_truth_profile_and_boundaries_accept_sealed_bundle(
    manifest: dict,
) -> None:
    truth_errors: list[str] = []
    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, truth_errors
    )
    profile_errors: list[str] = []
    validator._validate_author_card_house_v1_profile(
        manifest["configuration_profiles"], profile_errors
    )
    boundary_errors: list[str] = []
    validator._validate_author_card_house_requirement_boundaries(
        _author_card_house_requirement_map(manifest), boundary_errors
    )

    assert truth_errors == []
    assert profile_errors == []
    assert boundary_errors == []
    assert manifest["configuration_profiles"]["card_house_26"] == {
        "plates": 26,
        "levels": 4,
        "friction_coefficient": 0.8,
        "time_step_seconds": 1.0 / 60.0,
        "timeline": [
            "gravity-settled initial state",
            "standing at 6.7 s",
            "four projectiles strike one side",
            "post-impact at 10 s",
        ],
        "paper_contact_count_for_timing": 214,
        "paper_precision": "float32",
        "comparison_solvers": ["FBF", "Kamino", "MuJoCo"],
    }


@pytest.mark.parametrize(
    ("path", "replacement"),
    [
        (("evidence_scope",), "trajectory"),
        (("provenance", "author_commit"), "0" * 40),
        (("scene", "levels"), 4),
        (("scene", "cards"), 26),
        (("cards", "source_mobile"), False),
        (("cards", "lean_from_horizontal_degrees"), 64),
        (("cubes", "source_initially_kinematic"), False),
        (("contact", "gap_m"), 0.0),
        (("schedule", "release_substep"), 1599),
        (("schedule", "total_substeps"), 0),
        (("solver", "type"), "boxed_lcp"),
        (("solver", "max_outer"), 201),
        (("solver", "termination_tol"), 1.0e-5),
    ],
)
def test_author_card_house_profile_mutations_fail_closed(
    manifest: dict, path: tuple[str, ...], replacement
) -> None:
    profile = copy.deepcopy(
        manifest["configuration_profiles"][validator.AUTHOR_CARD_HOUSE_V1_PROFILE]
    )
    cursor = profile
    for field in path[:-1]:
        cursor = cursor[field]
    cursor[path[-1]] = replacement
    profiles = {validator.AUTHOR_CARD_HOUSE_V1_PROFILE: profile}
    errors: list[str] = []

    validator._validate_author_card_house_v1_profile(profiles, errors)

    assert errors
    assert any(".".join(path) in error for error in errors)


@pytest.mark.parametrize("field", sorted(validator.AUTHOR_CARD_HOUSE_V1_NEGATIVE_FLAGS))
def test_author_card_house_negative_claim_flags_cannot_be_promoted(
    manifest: dict, field: str
) -> None:
    _author_card_house_truth(manifest)[field] = True
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(f".{field}: expected False" in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("extra_truth_key", "unexpected keys"),
        ("wrong_status", ".status"),
        ("wrong_requirement", ".requirement_ids"),
        ("wrong_level", ".levels"),
        ("nonzero_steps", ".total_steps"),
        ("wrong_artifact_count", ".artifact_count"),
        ("wrong_indexed_count", ".indexed_artifact_count"),
        ("wrong_exclusions", ".artifact_index_exclusions"),
        ("wrong_contract_payload", "payload digest mismatch"),
        ("wrong_still_path", ".durable_still_path"),
        ("wrong_still_hash", ".durable_still_sha256"),
        ("malformed_artifact_hashes", ".artifact_hashes: expected an object"),
        ("missing_artifact_hash", "missing current keys"),
        ("extra_artifact_hash", "unexpected current keys"),
        ("wrong_artifact_hash", "digest mismatch"),
    ],
)
def test_author_card_house_truth_and_hash_mutations_fail_closed(
    manifest: dict, mutation: str, message: str
) -> None:
    truth = _author_card_house_truth(manifest)
    if mutation == "extra_truth_key":
        truth["unsupported"] = False
    elif mutation == "wrong_status":
        truth["status"] = "complete"
    elif mutation == "wrong_requirement":
        truth["requirement_ids"] = ["fig.06"]
    elif mutation == "wrong_level":
        truth["levels"] = 4
    elif mutation == "nonzero_steps":
        truth["total_steps"] = 1
    elif mutation == "wrong_artifact_count":
        truth["artifact_count"] = 13
    elif mutation == "wrong_indexed_count":
        truth["indexed_artifact_count"] = 13
    elif mutation == "wrong_exclusions":
        truth["artifact_index_exclusions"] = ["artifact-index.json"]
    elif mutation == "wrong_contract_payload":
        truth["contract_payload_sha256"] = "0" * 64
    elif mutation == "wrong_still_path":
        truth["durable_still_path"] = "panel.png"
    elif mutation == "wrong_still_hash":
        truth["durable_still_sha256"] = "0" * 64
    elif mutation == "malformed_artifact_hashes":
        truth["artifact_hashes"] = []
    elif mutation == "missing_artifact_hash":
        truth["artifact_hashes"].pop("REPORT.md")
    elif mutation == "extra_artifact_hash":
        truth["artifact_hashes"]["extra"] = "0" * 64
    elif mutation == "wrong_artifact_hash":
        truth["artifact_hashes"]["REPORT.md"] = "0" * 64
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing", "missing current keys"),
        ("extra", "unexpected current keys"),
        ("digest", "manifest identity differs"),
    ],
)
def test_author_card_house_source_identity_manifest_mutations_fail_closed(
    manifest: dict, mutation: str, message: str
) -> None:
    identities = _author_card_house_truth(manifest)["source_identity_sha256"]
    if mutation == "missing":
        identities.pop("configuration_spec")
    elif mutation == "extra":
        identities["extra"] = "0" * 64
    else:
        identities["configuration_spec"] = "0" * 64
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("path", "captured path suffix changed"),
        ("hash", "manifest identity differs"),
        ("membership", "exact identity membership changed"),
    ],
)
def test_author_card_house_captured_source_identity_mutations_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        identities = payload["source_identity"]
        if mutation == "path":
            identities["configuration_spec"]["path"] = "/tmp/wrong.hpp"
        elif mutation == "hash":
            identities["configuration_spec"]["sha256"] = "0" * 64
        else:
            identities["extra"] = copy.deepcopy(identities["configuration_spec"])

    _patch_author_card_house_bundle_json(monkeypatch, "metadata.json", mutate)
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("extra_top", "unexpected keys"),
        ("schema", ".schema_version"),
        ("kind", ".kind"),
        ("author", ".author_source.commit"),
        ("author_hash", ".author_source.card_house_run_py_sha256"),
        ("spec", ".configuration_spec_source_sha256"),
        ("binding", ".binary_binding.implementation_source_sha256"),
        ("scene", ".source_configuration.scene.cards"),
        ("solver", ".source_configuration.solver.max_outer"),
        ("world", ".dart_observation.world.time_step_seconds"),
        ("collision", ".dart_observation.collision.detector"),
        ("card_mobile", ".dart_observation.cards[0].mobile"),
        ("card_velocity", ".dart_observation.cards[0].initial_linear_velocity_m_s"),
        ("card_pose", "payload digest mismatch"),
        ("cube_mobile", ".dart_observation.cubes[0].mobile"),
        ("cube_velocity", ".dart_observation.cubes[0].initial_angular_velocity_rad_s"),
        ("adapter", ".adapter_boundaries.source_contact_gap_semantics_implemented"),
        ("claim", ".claim_boundary.trajectory_valid"),
        ("visual", ".visual_style.renderer_only"),
        ("malformed_observation", ".dart_observation: expected an object"),
    ],
)
def test_author_card_house_runtime_contract_mutations_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "extra_top":
            payload["extra"] = None
        elif mutation == "schema":
            payload["schema_version"] = "wrong"
        elif mutation == "kind":
            payload["kind"] = "trajectory"
        elif mutation == "author":
            payload["author_source"]["commit"] = "0" * 40
        elif mutation == "author_hash":
            payload["author_source"]["card_house_run_py_sha256"] = "0" * 64
        elif mutation == "spec":
            payload["configuration_spec_source_sha256"] = "0" * 64
        elif mutation == "binding":
            payload["binary_binding"]["implementation_source_sha256"] = "0" * 64
        elif mutation == "scene":
            payload["source_configuration"]["scene"]["cards"] = 26
        elif mutation == "solver":
            payload["source_configuration"]["solver"]["max_outer"] = 201
        elif mutation == "world":
            payload["dart_observation"]["world"]["time_step_seconds"] = 1 / 60
        elif mutation == "collision":
            payload["dart_observation"]["collision"]["detector"] = "bullet"
        elif mutation == "card_mobile":
            payload["dart_observation"]["cards"][0]["mobile"] = False
        elif mutation == "card_velocity":
            payload["dart_observation"]["cards"][0]["initial_linear_velocity_m_s"][
                0
            ] = 1
        elif mutation == "card_pose":
            payload["dart_observation"]["cards"][0]["initial_pose"]["translation"][
                0
            ] += 0.1
        elif mutation == "cube_mobile":
            payload["dart_observation"]["cubes"][0]["mobile"] = True
        elif mutation == "cube_velocity":
            payload["dart_observation"]["cubes"][0]["initial_angular_velocity_rad_s"][
                2
            ] = 1
        elif mutation == "adapter":
            payload["adapter_boundaries"][
                "source_contact_gap_semantics_implemented"
            ] = True
        elif mutation == "claim":
            payload["claim_boundary"]["trajectory_valid"] = True
        elif mutation == "visual":
            payload["visual_style"]["renderer_only"] = False
        elif mutation == "malformed_observation":
            payload["dart_observation"] = []

    _patch_author_card_house_bundle_json(monkeypatch, "contract.json", mutate)
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing", ".physics_contract: expected an object"),
        (
            "binding",
            ".physics_contract.binary_binding.implementation_source_sha256",
        ),
        ("runtime", "differs semantically from the separately queried contract"),
        ("digest_only", "canonical payload digest differs"),
    ],
)
def test_author_card_house_timeline_physics_contract_mutations_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "missing":
            payload.pop("physics_contract")
        elif mutation == "binding":
            payload["physics_contract"]["binary_binding"][
                "implementation_source_sha256"
            ] = ("0" * 64)
        elif mutation == "runtime":
            payload["physics_contract"]["dart_observation"]["world"][
                "time_step_seconds"
            ] = (1 / 60)
        else:
            payload["physics_contract"]["source_configuration"]["scene"]["levels"] = 5.0

    _patch_author_card_house_bundle_json(
        monkeypatch,
        f"{validator.AUTHOR_CARD_HOUSE_V1_CAPTURE}/timeline.json",
        mutate,
    )
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("total_steps", ".total_steps"),
        ("completed_steps", ".completed_steps"),
        ("collision", ".collision_detector"),
        ("actions", ".actions"),
        ("diagnostics", ".solver_diagnostics"),
        ("extra_step", ".steps"),
        ("shot_step", ".shots[0].step"),
        ("shot_path", ".shots[0].path"),
        ("event", ".events[0].type"),
        ("command", ".runtime_command"),
        ("malformed_shot", ".shots[0]: expected an object"),
    ],
)
def test_author_card_house_zero_step_timeline_mutations_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "total_steps":
            payload["total_steps"] = 1
        elif mutation == "completed_steps":
            payload["completed_steps"] = 1
        elif mutation == "collision":
            payload["collision_detector"] = "bullet"
        elif mutation == "actions":
            payload["actions"] = [{"step": 0}]
        elif mutation == "diagnostics":
            payload["solver_diagnostics"]["exact_solves"] = 1
        elif mutation == "extra_step":
            payload["steps"].append(copy.deepcopy(payload["steps"][0]))
        elif mutation == "shot_step":
            payload["shots"][0]["step"] = 1
        elif mutation == "shot_path":
            payload["shots"][0]["path"] = "/tmp/wrong.png"
        elif mutation == "event":
            payload["events"][0]["type"] = "action"
        elif mutation == "command":
            payload["runtime_command"] = payload["runtime_command"].replace(
                "--steps 0", "--steps 1"
            )
        elif mutation == "malformed_shot":
            payload["shots"][0] = []

    _patch_author_card_house_bundle_json(
        monkeypatch,
        f"{validator.AUTHOR_CARD_HOUSE_V1_CAPTURE}/timeline.json",
        mutate,
    )
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("filename", "mutation", "message"),
    [
        ("metadata.json", "metadata_trajectory", ".metadata.trajectory_valid"),
        (
            "metadata.json",
            "metadata_canonical",
            ".metadata.canonical_fig06_deliverable",
        ),
        ("capture-provenance.json", "capture_returncode", ".capture_returncode"),
        ("capture-provenance.json", "contract_hash", ".contract_sha256"),
        (
            "capture-provenance.json",
            "contract_payload",
            ".contract_payload_sha256_before",
        ),
        ("capture-provenance.json", "durable_hash", ".durable_still_sha256"),
        ("capture-provenance.json", "staging", ".staging_pruned"),
        ("run-summary.json", "run_outcome", ".automated_semantic_outcome_validated"),
        ("run-summary.json", "run_media", ".media_validation"),
        ("run-summary.json", "run_frame_hash", "durable still differs from capture"),
        ("verification.json", "verification_media", ".media"),
        ("verification.json", "verification_schedule", ".schedule"),
        ("invocations.json", "invocation_returncode", ".returncode"),
        ("invocations.json", "invocation_payload", ".payload_sha256"),
    ],
)
def test_author_card_house_capture_record_mutations_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    filename: str,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "metadata_trajectory":
            payload["trajectory_valid"] = True
        elif mutation == "metadata_canonical":
            payload["canonical_fig06_deliverable"] = True
        elif mutation == "capture_returncode":
            payload["capture_returncode"] = 1
        elif mutation == "contract_hash":
            payload["contract_sha256"] = "0" * 64
        elif mutation == "contract_payload":
            payload["contract_payload_sha256_before"] = "0" * 64
        elif mutation == "durable_hash":
            payload["durable_still_sha256"] = "0" * 64
        elif mutation == "staging":
            payload["staging_pruned"] = False
        elif mutation == "run_outcome":
            payload["results"][0]["automated_semantic_outcome_validated"] = True
        elif mutation == "run_media":
            payload["results"][0]["media_validation"] = [{"kind": "mp4"}]
        elif mutation == "run_frame_hash":
            payload["results"][0]["timeline_validation"]["frames"]["0"]["sha256"] = (
                "0" * 64
            )
        elif mutation == "verification_media":
            payload["results"][0]["media"] = [{"kind": "mp4"}]
        elif mutation == "verification_schedule":
            payload["results"][0]["schedule"] = "card_house_26"
        elif mutation == "invocation_returncode":
            payload["contract_query"]["returncode"] = 1
        elif mutation == "invocation_payload":
            payload["contract_query"]["payload_sha256"] = "0" * 64

    _patch_author_card_house_bundle_json(monkeypatch, filename, mutate)
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("schema", ".schema_version"),
        ("pass", ".pass"),
        ("verdict", ".verdicts"),
        ("order", "exact artifact order changed"),
        ("hash", "artifact digest changed"),
        ("observation", ".observation"),
    ],
)
def test_author_card_house_manual_record_mutations_fail_closed(
    manifest: dict,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    message: str,
) -> None:
    def mutate(payload):
        if mutation == "schema":
            payload["schema_version"] = "wrong"
        elif mutation == "pass":
            payload["pass"] = False
        elif mutation == "verdict":
            payload["verdicts"]["step_zero_only_confirmed"] = False
        elif mutation == "order":
            payload["representative_artifacts"].reverse()
        elif mutation == "hash":
            payload["representative_artifacts"][0]["sha256"] = "0" * 64
        elif mutation == "observation":
            payload["representative_artifacts"][0]["observation"] = ""

    _patch_author_card_house_bundle_json(monkeypatch, "manual-inspection.json", mutate)
    errors: list[str] = []

    validator._validate_author_card_house_v1_truth(
        manifest["current_truth"], ROOT, errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("schema", ".schema_version"),
        ("count", ".artifact_count"),
        ("excluded", ".excluded"),
        ("missing", ".artifacts"),
        ("extra", ".artifacts"),
        ("hash", ".sha256"),
        ("size", ".size_bytes"),
    ],
)
def test_author_card_house_artifact_index_mutations_fail_closed(
    mutation: str, message: str
) -> None:
    bundle = ROOT / validator.AUTHOR_CARD_HOUSE_V1_BUNDLE
    payload = json.loads((bundle / "artifact-index.json").read_text())
    if mutation == "schema":
        payload["schema_version"] = "wrong"
    elif mutation == "count":
        payload["artifact_count"] = 13
    elif mutation == "excluded":
        payload["excluded"] = ["artifact-index.json"]
    elif mutation == "missing":
        payload["artifacts"].pop()
    elif mutation == "extra":
        payload["artifacts"].append(copy.deepcopy(payload["artifacts"][0]))
    elif mutation == "hash":
        payload["artifacts"][0]["sha256"] = "0" * 64
    elif mutation == "size":
        payload["artifacts"][0]["size_bytes"] += 1
    errors: list[str] = []

    validator._validate_author_card_house_v1_artifact_index(
        bundle, payload, "test.card_house", errors
    )

    assert any(message in error for error in errors)


@pytest.mark.parametrize("mutation", ["missing", "extra", "symlink"])
def test_author_card_house_bundle_membership_is_exact(
    tmp_path: Path, mutation: str
) -> None:
    source = ROOT / validator.AUTHOR_CARD_HOUSE_V1_BUNDLE
    bundle = tmp_path / "bundle"
    shutil.copytree(source, bundle)
    if mutation == "missing":
        (bundle / "REPORT.md").unlink()
    elif mutation == "extra":
        shutil.copyfile(bundle / "REPORT.md", bundle / "extra.txt")
    else:
        (bundle / "linked-report").symlink_to(bundle / "REPORT.md")
    index = json.loads((bundle / "artifact-index.json").read_text())
    errors: list[str] = []

    validator._validate_author_card_house_v1_artifact_index(
        bundle, index, "test.card_house", errors
    )

    assert any("membership" in error for error in errors)


@pytest.mark.parametrize(
    ("requirement_id", "mutation", "message"),
    [
        ("fig.06", "status", ".status"),
        ("video.06_card_house", "status", ".status"),
        ("fig.06", "profile", ".configuration.profile"),
        ("video.06_card_house", "profile", ".configuration.profile"),
        ("fig.06", "fallback", ".fallback_count"),
        ("video.06_card_house", "fallback", ".fallback_count"),
        ("fig.06", "deliverable", "must remain empty"),
        ("video.06_card_house", "deliverable", "must remain empty"),
        ("fig.06", "evidence", "missing construction-only report"),
        ("video.06_card_house", "evidence", "missing construction-only report"),
        ("fig.06", "command", "missing bundle verification command"),
        ("video.06_card_house", "command", "missing bundle verification command"),
        ("fig.06", "blocker", "missing construction boundary"),
        ("video.06_card_house", "blocker", "missing construction boundary"),
    ],
)
def test_author_card_house_requirement_boundary_mutations_fail_closed(
    manifest: dict, requirement_id: str, mutation: str, message: str
) -> None:
    by_id = _author_card_house_requirement_map(manifest)
    requirement = by_id[requirement_id]
    if mutation == "status":
        requirement["status"] = "complete"
    elif mutation == "profile":
        requirement["configuration"]["profile"] = validator.AUTHOR_CARD_HOUSE_V1_PROFILE
    elif mutation == "fallback":
        requirement["fallback_count"] = 0
    elif mutation == "deliverable":
        requirement["deliverables"] = [
            {
                "kind": "still_image",
                "path": (
                    f"{validator.AUTHOR_CARD_HOUSE_V1_BUNDLE}/"
                    f"{validator.AUTHOR_CARD_HOUSE_V1_CAPTURE}/"
                    "construction-step-0.png"
                ),
                "validated": True,
            }
        ]
    elif mutation == "evidence":
        requirement["current_evidence"] = [
            item
            for item in requirement["current_evidence"]
            if not item["path"].endswith(
                "card_house_author_5_construction_current_v1/REPORT.md"
            )
        ]
    elif mutation == "command":
        requirement["commands"] = []
        requirement["capture_plan"]["commands"] = []
    elif mutation == "blocker":
        requirement["blockers"] = ["No evidence."]
        requirement["capture_plan"]["blockers"] = ["No evidence."]
    errors: list[str] = []

    validator._validate_author_card_house_requirement_boundaries(by_id, errors)

    assert any(message in error for error in errors)


def test_cli_accepts_the_repository_manifest(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert validator.main([str(MANIFEST), "--repo-root", str(ROOT)]) == 0
    assert "29 canonical requirements" in capsys.readouterr().out
