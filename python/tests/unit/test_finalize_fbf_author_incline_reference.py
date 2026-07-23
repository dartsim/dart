"""Focused tests for the current-author incline sweep finalizer."""

from __future__ import annotations

import copy
import csv
import gzip
import importlib.util
import io
import json
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_author_incline_reference.py"


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_author_incline_reference", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


@pytest.fixture(scope="module")
def evidence_bundle(module):
    bundle = module.DEFAULT_BUNDLE
    if not bundle.exists():
        pytest.skip(
            "requires the ignored local author-incline evidence bundle at " f"{bundle}"
        )
    return bundle


@pytest.fixture(scope="module")
def sealed_runs(module, evidence_bundle):
    return module._load_bundle_runs(evidence_bundle)


def _materialize_source_repo(module, root: Path) -> Path:
    """Rebuild the retained source layout without relying on a host clone."""

    source_repo = (root / "source").resolve()
    for bundle_relative in module.SOURCE_ARTIFACT_PATHS:
        source_relative = module._expected_source_path(bundle_relative)
        destination = source_repo / source_relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        data = (module.DEFAULT_BUNDLE / bundle_relative).read_bytes()
        if bundle_relative.endswith(".json.gz"):
            data = gzip.decompress(data)
        destination.write_bytes(data)
    return source_repo


def test_schema_names_are_stable(module):
    assert module.SCHEMA_VERSION == "dart.fbf_author_incline_sweep_reference/v1"
    assert module.VALIDATION_SCHEMA_VERSION == (
        "dart.fbf_author_incline_sweep_validation/v1"
    )


def test_default_bundle_verifies(module, evidence_bundle):
    report = module.validate_bundle(module.DEFAULT_BUNDLE)

    assert report == {
        "status": "valid_current_source_scientific_negative",
        "bundle": str(module.DEFAULT_BUNDLE.absolute()),
        "artifact_count": 37,
        "physical_file_count": 39,
        "cell_count": 21,
        "fbf_configured_converged_flags": 839,
        "fbf_configured_nonconverged_flags": 1,
        "timing_verdict": None,
        "paper_comparable": False,
    }


def test_default_bundle_has_exact_regular_members(module, evidence_bundle):
    assert module._regular_members(module.DEFAULT_BUNDLE) == set(
        module.EXACT_BUNDLE_MEMBERS
    )
    assert len(module.EXACT_BUNDLE_MEMBERS) == 39
    assert len(module.SOURCE_ARTIFACT_PATHS) == 34


def test_retained_metrics_keep_configured_and_natural_residuals_distinct(
    module, evidence_bundle
):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    metrics = module._metrics(runs)

    assert metrics["fbf"]["converged_flags"] == 839
    assert metrics["fbf"]["nonconverged_flags"] == 1
    assert metrics["fbf"]["initial_natural_shortcut_accepts"] == 235
    assert metrics["fbf"]["configured_outer_gate_accepts"] == 604
    assert metrics["fbf"]["true_flags_natural_residual_at_or_below_tolerance"] == 456
    assert metrics["fbf"]["true_flags_natural_residual_above_tolerance"] == 383
    assert metrics["fbf"]["per_mu"][4]["nonconverged_steps"] == [1]


def test_comparison_contact_evidence_is_fbf_only(module, sealed_runs):
    metrics = module._metrics(copy.deepcopy(sealed_runs))
    rows = list(
        csv.DictReader(
            io.StringIO(module._comparison_csv(sealed_runs, metrics).decode("utf-8"))
        )
    )

    assert metrics["fbf_contacts_per_step"] == 4
    assert all(row["fbf_contacts_per_step"] == "4" for row in rows[:7])
    assert all(row["fbf_contacts_per_step"] == "" for row in rows[7:])


def test_false_flag_cannot_be_promoted_from_small_natural_residual(
    module, evidence_bundle
):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    failed_step = runs["fbf"]["histories"][4]["steps"][1]
    assert failed_step["final_residual"] < module.TERMINATION_TOLERANCE
    assert failed_step["outer"][-1]["r_coulomb"] > module.TERMINATION_TOLERANCE
    failed_step["converged"] = True

    with pytest.raises(ValueError, match="configured outer accept"):
        module._metrics(runs)


def test_initial_shortcut_rejects_tolerance_equality(module, sealed_runs):
    history = copy.deepcopy(sealed_runs["fbf"]["histories"][0])
    step = next(item for item in history["steps"] if item["outer_iters"] == 0)
    step["initial_residual"] = module.TERMINATION_TOLERANCE
    step["final_residual"] = module.TERMINATION_TOLERANCE

    with pytest.raises(ValueError, match="invalid initial natural-residual shortcut"):
        module._validate_history(history, 0.3, "20260719T151337Z", "history")


def test_outer_solve_requires_failed_initial_shortcut(module, sealed_runs):
    history = copy.deepcopy(sealed_runs["fbf"]["histories"][0])
    step = next(item for item in history["steps"] if item["outer_iters"] > 0)
    step["initial_residual"] = 0.0

    with pytest.raises(ValueError, match="outer solve has a passing initial residual"):
        module._validate_history(history, 0.3, "20260719T151337Z", "history")


def test_negative_natural_residual_fails_closed(module, sealed_runs):
    history = copy.deepcopy(sealed_runs["fbf"]["histories"][0])
    history["steps"][0]["initial_residual"] = -1.0

    with pytest.raises(ValueError, match="natural residuals must be nonnegative"):
        module._validate_history(history, 0.3, "20260719T151337Z", "history")


def test_preterminal_configured_gate_cannot_pass(module, sealed_runs):
    history = copy.deepcopy(sealed_runs["fbf"]["histories"][0])
    checked = history["steps"][0]["outer"][4]
    for key in ("eps_vel", "eps_force", "eps_gap", "r_coulomb"):
        checked[key] = 0.0

    with pytest.raises(ValueError, match="preterminal configured gate passes"):
        module._validate_history(history, 0.3, "20260719T151337Z", "history")


def test_contact_count_mutation_fails_closed(module, evidence_bundle):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    runs["fbf"]["histories"][0]["steps"][0]["num_contacts"] = 3

    with pytest.raises(ValueError, match="num_contacts: expected 4"):
        module._metrics(runs)


def test_mu_0_55_nonconvergence_location_is_sealed(module, evidence_bundle):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    runs["fbf"]["histories"][4]["steps"][1]["step_idx"] = 2

    with pytest.raises(ValueError, match="step_idx: expected 1"):
        module._metrics(runs)


@pytest.mark.parametrize(
    "value",
    ("", ".", "./a", "a/./b", "a/", "a//b", "a/../b", "/a", "a\\b"),
)
def test_noncanonical_relative_paths_fail_closed(module, value):
    with pytest.raises(ValueError, match="unsafe or noncanonical"):
        module._safe_relative(value, "test path")


def test_canonical_relative_path_is_accepted(module):
    assert str(module._safe_relative("runs/id/result.json", "test path")) == (
        "runs/id/result.json"
    )


def test_metadata_schema_and_types_fail_closed(module, sealed_runs):
    metadata = copy.deepcopy(sealed_runs["fbf"]["metadata"])
    metadata["extra"] = True
    with pytest.raises(ValueError, match="exact keys changed"):
        module._validate_metadata(metadata, "fbf", "20260719T151337Z")

    metadata = copy.deepcopy(sealed_runs["fbf"]["metadata"])
    metadata["num_steps"] = 120.0
    with pytest.raises(ValueError, match="exact JSON value or type changed"):
        module._validate_metadata(metadata, "fbf", "20260719T151337Z")


def test_result_schema_config_and_external_sentinel_fail_closed(module, sealed_runs):
    run_id = "20260719T151337Z"
    result = copy.deepcopy(sealed_runs["fbf"]["results"][0])
    result["extra"] = True
    with pytest.raises(ValueError, match="exact keys changed"):
        module._validate_result(result, "fbf", 0.3, run_id, "result")

    result = copy.deepcopy(sealed_runs["fbf"]["results"][0])
    result["config"]["warm_start"] = False
    with pytest.raises(ValueError, match="exact JSON value or type changed"):
        module._validate_result(result, "fbf", 0.3, run_id, "result")

    result = copy.deepcopy(sealed_runs["fbf"]["results"][0])
    result["config"]["extra"] = None
    with pytest.raises(ValueError, match="exact keys changed"):
        module._validate_result(result, "fbf", 0.3, run_id, "result")

    result = copy.deepcopy(sealed_runs["fbf"]["results"][0])
    result["config"]["history_path"] = module._history_source_path(run_id, 0.3)
    with pytest.raises(ValueError, match="exact JSON value or type changed"):
        module._validate_result(result, "fbf", 0.3, run_id, "result")

    external = copy.deepcopy(sealed_runs["mujoco"]["results"][0])
    external["final_residual"] = 0.0
    with pytest.raises(ValueError, match="expected JSON NaN"):
        module._validate_result(external, "mujoco", 0.3, "20260719T151857Z", "result")


@pytest.mark.parametrize(
    "mutation,match",
    (
        ("meta_extra", "exact keys changed"),
        ("inner_solver", "exact JSON value or type changed"),
        ("config_extra", "exact keys changed"),
        ("config_path", "exact JSON value or type changed"),
        ("bench_extra", "exact keys changed"),
        ("bench_warmup_type", "exact JSON value or type changed"),
        ("scene_extra", "exact JSON value or type changed"),
        ("step_extra", "exact keys changed"),
        ("warmup", "expected false"),
        ("outer_extra", "exact keys changed"),
        ("outer_index", "expected 0"),
        ("outer_length", "expected exactly 45 records"),
        ("gpu_time", "exact JSON value or type changed"),
        ("unchecked_residual", "exact JSON value or type changed"),
    ),
)
def test_history_schema_config_and_iteration_mutations_fail_closed(
    module, sealed_runs, mutation, match
):
    history = copy.deepcopy(sealed_runs["fbf"]["histories"][0])
    step = history["steps"][0]
    outer = step["outer"]
    if mutation == "meta_extra":
        history["meta"]["extra"] = True
    elif mutation == "inner_solver":
        history["meta"]["inner_solver"] = "changed"
    elif mutation == "config_extra":
        history["meta"]["config"]["extra"] = None
    elif mutation == "config_path":
        history["meta"]["config"]["history_path"] = str(
            module.DEFAULT_SOURCE_REPO
            / module._history_source_path("20260719T151337Z", 0.3)
        )
    elif mutation == "bench_extra":
        history["meta"]["bench"]["extra"] = None
    elif mutation == "bench_warmup_type":
        history["meta"]["bench"]["warmup_steps"] = False
    elif mutation == "scene_extra":
        history["meta"]["bench"]["scene_params"]["extra"] = None
    elif mutation == "step_extra":
        step["extra"] = None
    elif mutation == "warmup":
        step["warmup"] = True
    elif mutation == "outer_extra":
        outer[0]["extra"] = None
    elif mutation == "outer_index":
        outer[0]["k"] = 1
    elif mutation == "outer_length":
        outer.pop(0)
    elif mutation == "gpu_time":
        step["t_total_gpu_ms"] = 0.0
    elif mutation == "unchecked_residual":
        outer[0]["residual"] = 0.0
    else:  # pragma: no cover - keeps the mutation table exhaustive
        raise AssertionError(mutation)

    with pytest.raises(ValueError, match=match):
        module._validate_history(history, 0.3, "20260719T151337Z", "history")


def test_result_history_summary_disconnect_fails_closed(module, sealed_runs):
    runs = copy.deepcopy(sealed_runs)
    runs["fbf"]["results"][0]["total_outer_iters"] += 1
    runs["fbf"]["results"][0]["avg_outer_iters_per_step"] = (
        runs["fbf"]["results"][0]["total_outer_iters"] / module.STEPS_PER_CELL
    )

    with pytest.raises(ValueError, match="differs from retained history"):
        module._metrics(runs)


def test_derived_artifacts_are_exact_regenerations(module, evidence_bundle):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    metrics = module._metrics(runs)

    assert (module.DEFAULT_BUNDLE / "comparison.csv").read_bytes() == (
        module._comparison_csv(runs, metrics)
    )
    assert (module.DEFAULT_BUNDLE / "comparison.svg").read_bytes() == (
        module._comparison_svg(metrics)
    )
    assert (module.DEFAULT_BUNDLE / "REPORT.md").read_bytes() == (
        module._report(metrics)
    )


def test_sweep_result_copy_uses_exact_json_types(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    sweep_path = bundle / "runs/20260719T151337Z/sweep_results.json"
    sweep = json.loads(sweep_path.read_text(encoding="utf-8"))
    sweep[0]["num_steps"] = 120.0
    sweep_path.write_bytes(module._json_bytes(sweep))

    with pytest.raises(ValueError, match="differs from result.json"):
        module._load_bundle_runs(bundle)


def test_stored_artifact_tamper_fails_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    with (bundle / "comparison.csv").open("ab") as stream:
        stream.write(b"tamper\n")

    with pytest.raises(ValueError, match="deterministic derived content changed"):
        module.validate_bundle(bundle)


def test_manifest_parity_promotion_fails_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["predicates"]["fig01_parity"] = True
    manifest_path.write_bytes(module._json_bytes(manifest))

    with pytest.raises(ValueError, match="manifest.predicates: exact JSON"):
        module.validate_bundle(bundle)


def test_manifest_source_path_alias_fails_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["retained_source_artifacts"][0]["source_path"] = "runs//result.json"
    manifest_path.write_bytes(module._json_bytes(manifest))

    with pytest.raises(ValueError, match="unsafe or noncanonical"):
        module.validate_bundle(bundle)


@pytest.mark.parametrize(
    "mutation,match",
    (
        ("source_extra", "manifest.source: exact keys changed"),
        ("python_sha", "manifest.source: exact JSON value or type changed"),
        ("canonical_source_path", "source path differs"),
    ),
)
def test_manifest_source_provenance_is_exact(
    module, evidence_bundle, tmp_path, mutation, match
):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    manifest_path = bundle / "manifest.json"
    verification_path = bundle / "verification.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if mutation == "source_extra":
        manifest["source"]["extra"] = None
    elif mutation == "python_sha":
        manifest["source"]["recorded_python_sha256"] = "0" * 64
    elif mutation == "canonical_source_path":
        manifest["retained_source_artifacts"][0]["source_path"] = "bogus.json"
    else:  # pragma: no cover - keeps the mutation table exhaustive
        raise AssertionError(mutation)
    manifest_path.write_bytes(module._json_bytes(manifest))
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    verification["manifest_sha256"] = module._sha256(manifest_path)
    verification_path.write_bytes(module._json_bytes(verification))

    with pytest.raises(ValueError, match=match):
        module.validate_bundle(bundle)


@pytest.mark.parametrize(
    "mutation,match",
    (
        ("predicate_bool", "manifest.predicates: exact JSON"),
        ("workload_bool", "manifest.workload: exact JSON"),
        ("run_bool", "manifest.runs: exact JSON"),
        ("metric_int", "manifest.metrics: exact JSON"),
        ("source_bytes", "stored_bytes: exact JSON"),
        ("derived_duplicate", "exact list length changed"),
        ("verification_int", "verification: exact JSON"),
    ),
)
def test_manifest_and_verification_scalar_types_are_exact(
    module, evidence_bundle, tmp_path, mutation, match
):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    manifest_path = bundle / "manifest.json"
    verification_path = bundle / "verification.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    if mutation == "predicate_bool":
        manifest["predicates"]["fig01_parity"] = 0
    elif mutation == "workload_bool":
        manifest["workload"]["independent_solver_runs"] = 1
    elif mutation == "run_bool":
        manifest["runs"][0]["profile_flag"] = 1
    elif mutation == "metric_int":
        manifest["metrics"]["cell_count"] = 21.0
    elif mutation == "source_bytes":
        manifest["retained_source_artifacts"][0]["stored_bytes"] = 455.0
    elif mutation == "derived_duplicate":
        manifest["derived_artifacts"].append(
            copy.deepcopy(manifest["derived_artifacts"][0])
        )
    elif mutation == "verification_int":
        verification["artifact_count"] = 37.0
    else:  # pragma: no cover - keeps the mutation table exhaustive
        raise AssertionError(mutation)
    if mutation != "verification_int":
        manifest_path.write_bytes(module._json_bytes(manifest))
        verification["manifest_sha256"] = module._sha256(manifest_path)
    verification_path.write_bytes(module._json_bytes(verification))

    with pytest.raises(ValueError, match=match):
        module.validate_bundle(bundle)


def test_duplicate_json_keys_fail_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    result_path = bundle / "runs/20260719T151337Z/mu0.3000_fbf/result.json"
    text = result_path.read_text(encoding="utf-8")
    text = text.replace(
        '  "num_steps": 120,',
        '  "num_steps": 999,\n  "num_steps": 120,',
        1,
    )
    result_path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate JSON object key 'num_steps'"):
        module._load_bundle_runs(bundle)


def test_duplicate_gzip_history_keys_fail_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    history_path = bundle / ("runs/20260719T151337Z/mu0.3000_fbf/history.json.gz")
    raw = gzip.decompress(history_path.read_bytes())
    raw = raw.replace(
        b'      "step_idx": 0,',
        b'      "step_idx": 999,\n      "step_idx": 0,',
        1,
    )
    history_path.write_bytes(module._gzip_bytes(raw))

    with pytest.raises(ValueError, match="duplicate JSON object key 'step_idx'"):
        module._load_bundle_runs(bundle)


def test_infinite_json_constant_fails_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    result_path = bundle / "runs/20260719T151857Z/mu0.3000_mujoco/result.json"
    text = result_path.read_text(encoding="utf-8").replace(
        '"final_residual": NaN', '"final_residual": Infinity', 1
    )
    result_path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="unsupported JSON constant 'Infinity'"):
        module._load_bundle_runs(bundle)


def test_extra_member_and_symlink_fail_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    (bundle / "unexpected.txt").write_text("unexpected\n", encoding="utf-8")

    with pytest.raises(ValueError, match="bundle membership changed"):
        module.validate_bundle(bundle)

    (bundle / "unexpected.txt").unlink()
    (bundle / "unexpected.txt").symlink_to(bundle / "REPORT.md")
    with pytest.raises(ValueError, match="contains symlink"):
        module.validate_bundle(bundle)


def test_symlinked_ancestor_fails_before_bundle_reads(
    module, evidence_bundle, tmp_path
):
    real_parent = tmp_path / "real"
    real_parent.mkdir()
    shutil.copytree(module.DEFAULT_BUNDLE, real_parent / "bundle")
    alias = tmp_path / "alias"
    alias.symlink_to(real_parent, target_is_directory=True)

    with pytest.raises(ValueError, match="path contains symlink component"):
        module.validate_bundle(alias / "bundle")


def test_source_membership_extra_file_fails_closed(module, evidence_bundle, tmp_path):
    source_copy = _materialize_source_repo(module, tmp_path)
    extra = module._source_run_root(source_copy, "20260719T151337Z") / "extra.txt"
    extra.write_text("unexpected\n", encoding="utf-8")

    with pytest.raises(ValueError, match="source membership changed"):
        module._load_source_runs(source_copy)


def test_finalize_rejects_symlinked_destination_ancestor(
    module, evidence_bundle, tmp_path
):
    source_repo = _materialize_source_repo(module, tmp_path)
    real_parent = tmp_path / "real"
    real_parent.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(real_parent, target_is_directory=True)

    with pytest.raises(ValueError, match="path contains symlink component"):
        module.finalize_bundle(source_repo, alias / "bundle")
    assert list(real_parent.iterdir()) == []


def test_finalize_builds_and_verifies_missing_destination(
    module, evidence_bundle, tmp_path, monkeypatch
):
    source_repo = _materialize_source_repo(module, tmp_path)
    source_identity = module._expected_source_identity()
    monkeypatch.setattr(module, "_source_identity", lambda unused: source_identity)
    bundle = tmp_path / "missing" / "bundle"
    report = module.finalize_bundle(source_repo, bundle)

    assert report["status"] == "valid_current_source_scientific_negative"
    assert module._regular_members(bundle) == set(module.EXACT_BUNDLE_MEMBERS)
    assert not list(bundle.parent.glob(".bundle.staging-*"))
    assert not list(bundle.parent.glob(".bundle.backup-*"))


def test_post_promotion_failure_restores_previous_bundle(module, tmp_path, monkeypatch):
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "sentinel.txt").write_text("old\n", encoding="utf-8")
    staging = tmp_path / ".bundle.staging-test"
    staging.mkdir()
    (staging / "sentinel.txt").write_text("new\n", encoding="utf-8")

    def fail_validation(_bundle):
        raise ValueError("forced post-promotion failure")

    monkeypatch.setattr(module, "validate_bundle", fail_validation)
    with pytest.raises(ValueError, match="forced post-promotion failure"):
        module._publish_staged_bundle(staging, bundle)

    assert (bundle / "sentinel.txt").read_text(encoding="utf-8") == "old\n"
    assert not list(tmp_path.glob(".bundle.staging-*"))
    assert not list(tmp_path.glob(".bundle.backup-*"))


def test_cli_verify_only_succeeds(module, evidence_bundle, capsys):
    assert module.main(["--bundle", str(module.DEFAULT_BUNDLE), "--verify-only"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "valid_current_source_scientific_negative"
