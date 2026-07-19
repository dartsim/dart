import copy
import errno
import gzip
import importlib.util
import io
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_author_masonry_arch_reference.py"
BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "author_masonry_arch_reference_v1"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_author_masonry_arch_reference", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def module():
    return _load_module()


def _hardlink_bundle(destination):
    def link_or_copy(source, target):
        try:
            os.link(source, target)
        except OSError as error:
            if error.errno != errno.EXDEV:
                raise
            shutil.copy2(source, target)

    shutil.copytree(BUNDLE, destination, copy_function=link_or_copy)
    return destination


def test_repository_bundle_validates_scientific_negative(module):
    report = module.validate_bundle(BUNDLE)

    assert report == {
        "schema_version": module.VERIFICATION_SCHEMA_VERSION,
        "pass": True,
        "status": "valid_current_source_scientific_negative",
        "source_commit": module.SOURCE_COMMIT,
        "artifact_count": 7,
        "stored_bytes": 8117829,
        "author_converged_flag_count": 157,
        "author_nonconverged_flag_count": 1843,
        "dart_dynamics_parity_valid": False,
    }


def test_manifest_binds_source_environment_and_literal_invocation(module):
    manifest = module.validate_manifest(module.read_json(BUNDLE / "manifest.json"))

    assert manifest["source"] == {
        "repository": module.SOURCE_REPOSITORY,
        "commit": module.SOURCE_COMMIT,
        "tree": module.SOURCE_TREE,
        "runner_path": module.RUNNER_PATH,
        "runner_sha256": module.RUNNER_SHA256,
        "runner_git_blob": module.RUNNER_GIT_BLOB,
        "requirements_path": module.REQUIREMENTS_PATH,
        "requirements_sha256": module.REQUIREMENTS_SHA256,
        "requirements_git_blob": module.REQUIREMENTS_GIT_BLOB,
        "clean_pinned_checkout_observed_at_finalization": True,
        "pre_and_post_run_cleanliness_captured": False,
    }
    assert manifest["invocation"]["argv"] == list(module.INVOCATION)
    assert manifest["invocation"]["cwd"] == "/tmp/fbf-sca-2026-author"
    assert manifest["invocation"]["returncode"] == 0
    assert manifest["invocation"]["observation_source"] == "orchestration_record"
    assert manifest["invocation"]["argv_embedded_in_source_result"] is False
    assert manifest["environment"] == module.EXPECTED_ENVIRONMENT
    assert manifest["environment"]["observation_timing"] == (
        "post_run_only_at_finalization"
    )
    history = next(
        item
        for item in manifest["retained_source_artifacts"]
        if item["bundle_path"] == module.HISTORY_PATH
    )
    assert history["encoding"] == "claim-projection-json-gzip-n9-mtime0"
    assert history["source_identity"] == {
        "bytes": 382753953,
        "sha256": ("cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1"),
        "retained": False,
    }
    assert history["projection"] == {
        "schema_version": module.HISTORY_PROJECTION_SCHEMA_VERSION,
        "stored_bytes": 8080187,
        "stored_sha256": (
            "8a19be46feae8bfbbfc57f3aff194bc18a9fa695f286cc4d13f3c819f9dc701b"
        ),
        "decompressed_bytes": 28606169,
        "decompressed_sha256": (
            "8c9d245b5820950b36fe9b319cde0c256ab1cb404d9dfa1fc7af5fb53a9e0f98"
        ),
        "step_fields": [
            "step_idx",
            "num_contacts",
            "outer_iters",
            "converged",
            "initial_residual",
            "final_residual",
            "warmup",
            "outer",
        ],
        "outer_fields": ["k", "residual", "r_coulomb"],
    }


def test_manifest_distinguishes_source_default_from_observed_override(module):
    manifest = module.read_json(BUNDLE / "manifest.json")

    assert manifest["run"]["source_default"] == {
        "frames": 400,
        "drop_frame": 400,
        "releases_cubes": False,
    }
    assert manifest["run"]["observed_override"] == {
        "frames": 500,
        "drop_frame": 400,
        "releases_cubes": True,
    }
    assert manifest["claim_boundary"]["historical_or_paper_invocation"] is False


def test_summary_separates_configured_and_natural_residual_semantics(module):
    summary = module.read_json(BUNDLE / "summary.json")
    assert summary == module.expected_summary()
    history = summary["history"]

    assert history["author_converged_flag_count"] == 157
    assert history["author_nonconverged_flag_count"] == 1843
    assert history["initial_natural_residual_shortcut_converged_count"] == 40
    assert history["configured_coulomb_rel_outer_gate_converged_count"] == 117
    assert history["natural_final_residual_at_or_below_tolerance_count"] == 47
    assert history["pre_release"] == {
        "substeps": 1600,
        "converged": 142,
        "nonconverged": 1458,
        "initial_natural_residual_shortcut_converged": 40,
        "configured_coulomb_rel_outer_gate_converged": 102,
    }
    assert history["post_release"] == {
        "substeps": 400,
        "converged": 15,
        "nonconverged": 385,
        "initial_natural_residual_shortcut_converged": 0,
        "configured_coulomb_rel_outer_gate_converged": 15,
    }
    assert summary["residual_semantics"] == {
        "configured_termination_residual": "coulomb_rel",
        "configured_termination_tolerance": 1.0e-6,
        "author_converged_flag_is_exclusively_configured_gate": False,
        "initial_natural_residual_shortcut_converged_count": 40,
        "configured_coulomb_rel_outer_gate_converged_count": 117,
        "final_residual_field_is_natural_residual": True,
        "natural_threshold_count_is_not_configured_convergence_count": True,
    }
    assert summary["predicates"]["claim_history_projection_valid"] is True
    assert summary["predicates"]["raw_source_history_retained"] is False


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (-1.0e-12, False),
        (0.0, True),
        (np.nextafter(1.0e-6, 0.0), True),
        (1.0e-6, False),
    ],
)
def test_configured_coulomb_rel_gate_matches_source_strict_boundary(
    module, value, expected
):
    assert module._configured_coulomb_rel_gate_passes(value) is expected


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (np.nextafter(1.0e-6, 0.0), True),
        (1.0e-6, False),
    ],
)
def test_initial_natural_residual_shortcut_is_strict(module, value, expected):
    assert module._initial_natural_residual_shortcut_passes(value) is expected


def test_summary_keeps_contact_count_change_inferred_only(module):
    summary = module.read_json(BUNDLE / "summary.json")
    history = summary["history"]

    assert history["release_substep"] == {
        "step_idx": 1600,
        "num_contacts": 100,
        "converged": False,
        "outer_iters": 200,
        "natural_final_residual": 0.017456069692858667,
    }
    assert history["first_inferred_contact_count_increase_after_release"] == {
        "step_idx": 1944,
        "previous_count": 100,
        "count": 102,
    }
    assert history["peak_contact_count"] == {"step_idx": 1947, "count": 109}
    assert history["contact_pair_identity_available"] is False
    assert (
        summary["predicates"]["contact_increase_is_pair_identified_cube_arch_contact"]
        is False
    )


def test_artifact_index_covers_all_durable_artifacts_without_cycle(module):
    index = module.read_json(BUNDLE / "artifact-index.json")
    module.validate_artifact_index(BUNDLE, index)

    paths = {item["path"] for item in index["artifacts"]}
    assert paths == module.CORE_FILES
    assert module.HISTORY_PATH in paths
    assert index["excluded"] == ["artifact-index.json", "verification.json"]
    history = next(
        item for item in index["artifacts"] if item["path"] == module.HISTORY_PATH
    )
    assert history["bytes"] == 8080187
    assert len(history["sha256"]) == 64


def test_report_is_concise_and_preserves_claim_boundary(module):
    report = (BUNDLE / "REPORT.md").read_text(encoding="utf-8")
    normalized = " ".join(report.split())

    assert report == module.report_markdown()
    assert "valid current-source scientific negative" in report
    assert "not a historical" in report
    assert "49 MB raw archive" in report
    assert "not definitive cube-arch contact evidence" in normalized
    assert len(report.splitlines()) < 50


def test_repository_projection_recomputes_exact_history_metrics(module):
    projection = BUNDLE / module.HISTORY_PATH

    assert module.summarize_projected_gzip_history(projection) == (
        module.EXPECTED_HISTORY_METRICS
    )


def test_projected_history_source_identity_fails_closed(module):
    payload = {
        "schema_version": module.HISTORY_PROJECTION_SCHEMA_VERSION,
        "source_history": {"bytes": 1, "sha256": "0" * 64},
        "meta": module.EXPECTED_HISTORY_META,
        "steps": [],
    }

    with pytest.raises(ValueError, match="projection source identity"):
        module.summarize_history(
            io.StringIO(json.dumps(payload)), label="projection", projected=True
        )


@pytest.mark.parametrize(
    ("section", "field", "replacement"),
    [
        ("source_identity", "bytes", 1),
        ("source_identity", "sha256", "0" * 64),
        ("source_identity", "retained", True),
        ("projection", "schema_version", "invalid/v1"),
        ("projection", "stored_bytes", 1),
        ("projection", "stored_sha256", "0" * 64),
        ("projection", "decompressed_bytes", 1),
        ("projection", "decompressed_sha256", "0" * 64),
        ("projection", "step_fields", []),
        ("projection", "outer_fields", []),
    ],
)
def test_manifest_projection_identity_and_schema_fail_closed(
    module, section, field, replacement
):
    manifest = copy.deepcopy(module.expected_manifest())
    history = next(
        item
        for item in manifest["retained_source_artifacts"]
        if item["bundle_path"] == module.HISTORY_PATH
    )
    history[section][field] = replacement

    with pytest.raises(ValueError, match="manifest or claim boundary"):
        module.validate_manifest(manifest)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("historical_or_paper_invocation", True),
        ("dart_dynamics_parity", True),
        ("cross_solver_parity", True),
        ("all_substeps_solver_contract_valid", True),
        ("contact_pairs_recorded", True),
        ("timing_verdict", "valid"),
        ("runtime_attestation_complete", True),
    ],
)
def test_manifest_rejects_forbidden_claim_promotion(module, field, value):
    manifest = copy.deepcopy(module.expected_manifest())
    manifest["claim_boundary"][field] = value

    with pytest.raises(ValueError, match="claim boundary"):
        module.validate_manifest(manifest)


def test_bundle_rejects_same_size_hash_mutation_before_history_parse(module, tmp_path):
    bundle = _hardlink_bundle(tmp_path / "bundle")
    target = bundle / module.RESULT_PATH
    data = bytearray(target.read_bytes())
    target.unlink()
    data[20] ^= 1
    target.write_bytes(data)

    with pytest.raises(ValueError, match="artifact index"):
        module.validate_bundle(bundle)


def test_bundle_rejects_unexpected_path(module, tmp_path):
    bundle = _hardlink_bundle(tmp_path / "bundle")
    (bundle / "unexpected.txt").write_text("unexpected\n", encoding="utf-8")

    with pytest.raises(ValueError, match="file membership changed"):
        module.validate_bundle(bundle)


def test_bundle_rejects_internal_symlink(module, tmp_path):
    bundle = _hardlink_bundle(tmp_path / "bundle")
    target = bundle / module.RESULT_PATH
    target.unlink()
    target.symlink_to(BUNDLE / module.RESULT_PATH)

    with pytest.raises(ValueError, match="contains symlink"):
        module.validate_bundle(bundle)


def test_bundle_rejects_root_and_ancestor_symlinks(module, tmp_path):
    root_link = tmp_path / "root-link"
    root_link.symlink_to(BUNDLE, target_is_directory=True)
    with pytest.raises(ValueError, match="path contains symlink"):
        module.validate_bundle(root_link)

    real_parent = tmp_path / "real"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(ValueError, match="path contains symlink"):
        module.validate_bundle(linked_parent / "missing")


def test_deterministic_gzip_round_trip_and_trailing_data_rejection(module, tmp_path):
    raw = (b'{"meta":{},"steps":[]}\n' * 1000) + b"complete\n"
    source = tmp_path / "history.json"
    compressed = tmp_path / "history.json.gz"
    source.write_bytes(raw)
    module._gzip_n9_mtime0(source, compressed)
    module.validate_gzip_source_identity(
        compressed, raw_bytes=len(raw), raw_sha256=module.sha256_bytes(raw)
    )

    compressed.write_bytes(compressed.read_bytes() + b"trailing")
    with pytest.raises(ValueError, match="trailing data or members|invalid gzip"):
        module.validate_gzip_source_identity(
            compressed, raw_bytes=len(raw), raw_sha256=module.sha256_bytes(raw)
        )


def test_gzip_decompression_is_size_bounded(module, tmp_path):
    raw = b"x" * 65
    compressed = tmp_path / "history.json.gz"
    compressed.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))

    with pytest.raises(ValueError, match="exceeds size bound"):
        module.validate_gzip_source_identity(
            compressed, raw_bytes=64, raw_sha256=module.sha256_bytes(raw[:64])
        )


def test_trajectory_npz_disables_pickle_loading(module, monkeypatch):
    real_load = module.np.load
    observed = []

    def checked_load(*args, **kwargs):
        observed.append(kwargs.get("allow_pickle"))
        return real_load(*args, **kwargs)

    monkeypatch.setattr(module.np, "load", checked_load)
    module.load_trajectory(BUNDLE / module.TRAJECTORY_PATH)

    assert observed == [False]


def test_trajectory_npz_rejects_unexpected_member(module, tmp_path):
    arrays = module.load_trajectory(BUNDLE / module.TRAJECTORY_PATH)
    arrays["unexpected"] = np.zeros(1, dtype=np.float64)
    path = tmp_path / "trajectory.npz"
    np.savez(path, **arrays)

    with pytest.raises(ValueError, match="unexpected NPZ member set"):
        module.load_trajectory(path)


def test_result_config_rejects_boolean_integer_substitution(module):
    arrays = module.load_trajectory(BUNDLE / module.TRAJECTORY_PATH)
    trajectory = module.summarize_trajectory(arrays)
    result = module.read_json(BUNDLE / module.RESULT_PATH)
    result["config"]["warm_start"] = 1

    with pytest.raises(ValueError, match="solver config"):
        module.validate_result(result, trajectory)


def test_streaming_json_cursor_rejects_duplicate_keys(module):
    cursor = module._JsonCursor(io.StringIO('{"a": 1, "a": 2}'), label="test")

    with pytest.raises(ValueError, match="duplicate JSON key"):
        cursor.decode()


@pytest.mark.parametrize("path", ["/absolute", "../escape", "a\\b", "a//b"])
def test_bundle_paths_must_be_safe_and_canonical(module, path):
    with pytest.raises(ValueError, match="unsafe or noncanonical"):
        module._safe_relative_path(path)


def test_publish_rolls_back_existing_bundle_on_post_promotion_failure(module, tmp_path):
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "sentinel.txt").write_text("old\n", encoding="utf-8")
    stage = tmp_path / "stage"
    stage.mkdir()
    (stage / "sentinel.txt").write_text("new\n", encoding="utf-8")

    def fail_after_promotion(path):
        assert (path / "sentinel.txt").read_text(encoding="utf-8") == "new\n"
        raise RuntimeError("forced post-promotion failure")

    with pytest.raises(RuntimeError, match="forced post-promotion failure"):
        module._publish_staged_bundle(stage, bundle, validator=fail_after_promotion)

    assert (bundle / "sentinel.txt").read_text(encoding="utf-8") == "old\n"
    assert not stage.exists()
    assert not list(tmp_path.glob(".*.backup-*"))


def test_final_bundle_has_no_reserved_transient_directories(module):
    _, directories = module._bundle_entries(BUNDLE)

    assert ".staging" not in directories
    assert "frames" not in directories
    assert "panel_frames" not in directories
    assert not any("backup-" in value for value in directories)
