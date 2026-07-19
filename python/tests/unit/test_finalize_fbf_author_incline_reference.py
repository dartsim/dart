"""Focused tests for the current-author incline sweep finalizer."""

from __future__ import annotations

import importlib.util
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


def test_schema_names_are_stable(module):
    assert module.SCHEMA_VERSION == "dart.fbf_author_incline_sweep_reference/v1"
    assert module.VALIDATION_SCHEMA_VERSION == (
        "dart.fbf_author_incline_sweep_validation/v1"
    )


def test_default_bundle_verifies(module):
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


def test_default_bundle_has_exact_regular_members(module):
    assert module._regular_members(module.DEFAULT_BUNDLE) == set(
        module.EXACT_BUNDLE_MEMBERS
    )
    assert len(module.EXACT_BUNDLE_MEMBERS) == 39
    assert len(module.SOURCE_ARTIFACT_PATHS) == 34


def test_retained_metrics_keep_configured_and_natural_residuals_distinct(module):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    metrics = module._metrics(runs)

    assert metrics["fbf"]["converged_flags"] == 839
    assert metrics["fbf"]["nonconverged_flags"] == 1
    assert metrics["fbf"]["initial_natural_shortcut_accepts"] == 235
    assert metrics["fbf"]["configured_outer_gate_accepts"] == 604
    assert metrics["fbf"]["true_flags_natural_residual_at_or_below_tolerance"] == 456
    assert metrics["fbf"]["true_flags_natural_residual_above_tolerance"] == 383
    assert metrics["fbf"]["per_mu"][4]["nonconverged_steps"] == [1]


def test_false_flag_cannot_be_promoted_from_small_natural_residual(module):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    failed_step = runs["fbf"]["histories"][4]["steps"][1]
    assert failed_step["final_residual"] < module.TERMINATION_TOLERANCE
    assert failed_step["outer"][-1]["r_coulomb"] > module.TERMINATION_TOLERANCE
    failed_step["converged"] = True

    with pytest.raises(ValueError, match="configured outer accept"):
        module._metrics(runs)


def test_contact_count_mutation_fails_closed(module):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    runs["fbf"]["histories"][0]["steps"][0]["num_contacts"] = 3

    with pytest.raises(ValueError, match="num_contacts: expected 4"):
        module._metrics(runs)


def test_mu_0_55_nonconvergence_location_is_sealed(module):
    runs = module._load_bundle_runs(module.DEFAULT_BUNDLE)
    runs["fbf"]["histories"][4]["steps"][1]["step_idx"] = 2

    with pytest.raises(ValueError, match="step_idx: expected 1"):
        module._metrics(runs)


def test_derived_artifacts_are_exact_regenerations(module):
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


def test_stored_artifact_tamper_fails_closed(module, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    with (bundle / "comparison.csv").open("ab") as stream:
        stream.write(b"tamper\n")

    with pytest.raises(ValueError, match="deterministic derived content changed"):
        module.validate_bundle(bundle)


def test_manifest_parity_promotion_fails_closed(module, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["predicates"]["fig01_parity"] = True
    manifest_path.write_bytes(module._json_bytes(manifest))

    with pytest.raises(ValueError, match="manifest predicates changed"):
        module.validate_bundle(bundle)


def test_extra_member_and_symlink_fail_closed(module, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(module.DEFAULT_BUNDLE, bundle)
    (bundle / "unexpected.txt").write_text("unexpected\n", encoding="utf-8")

    with pytest.raises(ValueError, match="bundle membership changed"):
        module.validate_bundle(bundle)

    (bundle / "unexpected.txt").unlink()
    (bundle / "unexpected.txt").symlink_to(bundle / "REPORT.md")
    with pytest.raises(ValueError, match="contains symlink"):
        module.validate_bundle(bundle)


def test_cli_verify_only_succeeds(module, capsys):
    assert module.main(["--bundle", str(module.DEFAULT_BUNDLE), "--verify-only"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "valid_current_source_scientific_negative"
