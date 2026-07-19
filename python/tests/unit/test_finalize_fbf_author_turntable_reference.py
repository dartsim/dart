import copy
import gzip
import importlib.util
import json
import shutil
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_author_turntable_reference.py"
BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "author_turntable_reference_v1"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_author_turntable_reference", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def module():
    return _load_module()


def _case(module, case_id):
    return module.CASE_BY_ID[case_id]


def _history(module, case):
    record = next(
        item for item in module.EXPECTED_ARTIFACTS if item["path"] == case.history_path
    )
    raw = module.decompress_history(
        BUNDLE / case.history_path,
        raw_bytes=record["raw_bytes"],
        raw_sha256=record["raw_sha256"],
    )
    return module.parse_json_bytes(raw, label=case.history_path)


def _write_trajectory(path, arrays):
    np.savez(path, **{key: arrays[key] for key in arrays})


def test_repository_bundle_validates_expected_partial_negative(module):
    report = module.validate_bundle(BUNDLE)

    assert report["pass"] is True
    assert report["artifact_count"] == 14
    assert report["stored_artifact_bytes"] == 832388
    assert report["predicates"] == module.EXPECTED_PREDICATES
    assert report["predicates"]["artifact_valid"] is True
    assert report["predicates"]["physical_outcome_matrix_valid"] is True
    assert report["predicates"]["all_solver_contract_valid"] is False
    assert report["predicates"]["timing_verdict"] is None
    assert report["predicates"]["runtime_attestation_complete"] is False
    assert {
        item["case_id"]: item["solver_contract_valid"] for item in report["cases"]
    } == {
        "mu_0_5_omega_2": True,
        "mu_0_5_omega_5": True,
        "mu_0_2_omega_2": False,
        "mu_0_2_omega_5": True,
    }


def test_manifest_binds_source_invocations_environment_and_nonclaims(module):
    manifest = module.read_json(BUNDLE / "manifest.json")

    assert module.validate_manifest_contract(manifest) == manifest
    assert manifest["source"]["commit"] == module.SOURCE_COMMIT
    assert manifest["source"]["pre_and_post_run_cleanliness_captured"] is False
    assert len(manifest["invocations"]) == 2
    assert all(item["returncode"] == 0 for item in manifest["invocations"])
    assert all(
        item["timing_includes_first_use_compilation"] is True
        for item in manifest["invocations"]
    )
    assert manifest["environment"]["installed_distribution_count"] == 65
    assert manifest["environment"]["requirements_exact_match"] is True
    assert manifest["nonclaims"] == list(module.NONCLAIMS)


def test_bundle_rejects_same_size_hash_mutation(module, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(BUNDLE, bundle)
    target = bundle / module.CASES[0].result_path
    data = bytearray(target.read_bytes())
    data[20] ^= 1
    target.write_bytes(data)

    with pytest.raises(ValueError, match="artifact content changed"):
        module.validate_bundle(bundle)


def test_bundle_rejects_unexpected_path(module, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(BUNDLE, bundle)
    (bundle / "unexpected.txt").write_text("unexpected", encoding="utf-8")

    with pytest.raises(ValueError, match="file membership changed"):
        module.validate_bundle(bundle)


def test_bundle_rejects_internal_symlink(module, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(BUNDLE, bundle)
    target = bundle / module.CASES[0].result_path
    target.unlink()
    target.symlink_to(BUNDLE / module.CASES[0].result_path)

    with pytest.raises(ValueError, match="contains symlink"):
        module.validate_bundle(bundle)


def test_bundle_rejects_root_symlink(module, tmp_path):
    link = tmp_path / "bundle-link"
    link.symlink_to(BUNDLE, target_is_directory=True)

    with pytest.raises(ValueError, match="path contains symlink"):
        module.validate_bundle(link)


def test_bundle_rejects_ancestor_directory_symlink(module, tmp_path):
    real_parent = tmp_path / "real"
    real_parent.mkdir()
    shutil.copytree(BUNDLE, real_parent / "bundle")
    linked_parent = tmp_path / "linked"
    linked_parent.symlink_to(real_parent, target_is_directory=True)

    with pytest.raises(ValueError, match="path contains symlink"):
        module.validate_bundle(linked_parent / "bundle")


@pytest.mark.parametrize("symlink_location", ["root", "ancestor"])
def test_finalize_rejects_symlink_path_before_mutation(
    module, monkeypatch, tmp_path, symlink_location
):
    source_repo = tmp_path / "source"
    source_repo.mkdir()
    real_parent = tmp_path / "real"
    real_parent.mkdir()
    sentinel = real_parent / "sentinel.txt"
    sentinel.write_text("outside\n", encoding="utf-8")
    linked = tmp_path / "linked"

    if symlink_location == "root":
        linked.symlink_to(real_parent, target_is_directory=True)
        bundle = linked
    else:
        linked.symlink_to(real_parent, target_is_directory=True)
        bundle = linked / "bundle"

    monkeypatch.setattr(module, "_validate_source_repo", lambda _: None)

    with pytest.raises(ValueError, match="path contains symlink"):
        module.finalize_bundle(source_repo, bundle)

    assert sentinel.read_text(encoding="utf-8") == "outside\n"
    assert not (real_parent / "bundle").exists()


def test_finalize_cleanup_unlinks_swapped_root_symlink(module, monkeypatch, tmp_path):
    source_repo = tmp_path / "source"
    source_repo.mkdir()
    bundle = tmp_path / "bundle"
    outside = tmp_path / "outside"
    outside.mkdir()
    sentinel = outside / "sentinel.txt"
    sentinel.write_text("outside\n", encoding="utf-8")
    validation_count = 0

    def swap_bundle_on_second_validation(_):
        nonlocal validation_count
        validation_count += 1
        if validation_count == 2:
            shutil.rmtree(bundle)
            bundle.symlink_to(outside, target_is_directory=True)
            raise RuntimeError("forced failure")

    monkeypatch.setattr(module, "RUNS", {})
    monkeypatch.setattr(
        module, "_validate_source_repo", swap_bundle_on_second_validation
    )

    with pytest.raises(RuntimeError, match="forced failure"):
        module.finalize_bundle(source_repo, bundle)

    assert not bundle.exists()
    assert not bundle.is_symlink()
    assert sentinel.read_text(encoding="utf-8") == "outside\n"


@pytest.mark.parametrize("path", ["/absolute", "../escape", "a\\b", "a//b"])
def test_manifest_paths_must_be_safe_and_canonical(module, path):
    with pytest.raises(ValueError, match="unsafe or noncanonical"):
        module._safe_relative_path(path)


def test_history_decompression_is_bounded_and_rejects_trailing_data(module, tmp_path):
    expanded = b"x" * 33
    path = tmp_path / "history.json.gz"
    path.write_bytes(gzip.compress(expanded, compresslevel=9, mtime=0))

    with pytest.raises(ValueError, match="exceeds size bound"):
        module.decompress_history(
            path, raw_bytes=32, raw_sha256=module.sha256_bytes(expanded[:32])
        )

    path.write_bytes(gzip.compress(expanded, compresslevel=9, mtime=0) + b"tail")
    with pytest.raises(ValueError, match="trailing data"):
        module.decompress_history(
            path, raw_bytes=len(expanded), raw_sha256=module.sha256_bytes(expanded)
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda arrays: arrays.update(unexpected=np.zeros(360, dtype=np.float64)),
            "unexpected NPZ member set",
        ),
        (
            lambda arrays: arrays.update(t=np.arange(720, dtype=np.float32)),
            "t dtype must be float64",
        ),
        (
            lambda arrays: arrays.update(
                t=np.arange(360, dtype=np.float64).reshape(180, 2)
            ),
            "t shape must be",
        ),
    ],
)
def test_trajectory_npz_rejects_member_dtype_and_shape_mutations(
    module, tmp_path, mutation, message
):
    case = _case(module, "mu_0_5_omega_2")
    arrays = module.load_trajectory(BUNDLE / case.trajectory_path)
    mutation(arrays)
    path = tmp_path / "trajectory.npz"
    _write_trajectory(path, arrays)

    with pytest.raises(ValueError, match=message):
        module.load_trajectory(path)


def test_trajectory_npz_disables_pickle_loading(module, monkeypatch):
    case = _case(module, "mu_0_5_omega_2")
    real_load = module.np.load
    observed = []

    def checked_load(*args, **kwargs):
        observed.append(kwargs.get("allow_pickle"))
        return real_load(*args, **kwargs)

    monkeypatch.setattr(module.np, "load", checked_load)
    module.load_trajectory(BUNDLE / case.trajectory_path)

    assert observed == [False]


def test_result_metric_must_recompute_from_trajectory(module):
    case = _case(module, "mu_0_5_omega_2")
    arrays = module.load_trajectory(BUNDLE / case.trajectory_path)
    trajectory = module.summarize_trajectory(arrays, case)
    result = module.read_result(BUNDLE / case.result_path, case)
    result["v_slip_tan_mean_last_window"] += 0.01

    with pytest.raises(ValueError, match="does not match trajectory"):
        module.validate_result(result, trajectory, case)


def test_result_config_rejects_boolean_integer_substitution(module):
    case = _case(module, "mu_0_5_omega_2")
    arrays = module.load_trajectory(BUNDLE / case.trajectory_path)
    trajectory = module.summarize_trajectory(arrays, case)
    result = module.read_result(BUNDLE / case.result_path, case)
    result["config"]["warm_start"] = 1

    with pytest.raises(ValueError, match="solver config changed"):
        module.validate_result(result, trajectory, case)


def test_trajectory_outcome_exit_index_is_source_derived(module):
    case = _case(module, "mu_0_5_omega_5")
    arrays = module.load_trajectory(BUNDLE / case.trajectory_path)
    arrays["r"][case.first_exit_index] = module.EXIT_RADIUS
    arrays["radial_disp"][case.first_exit_index] = (
        module.EXIT_RADIUS - module.CUBE_OFFSET_RADIUS
    )

    with pytest.raises(ValueError, match="first author exit index"):
        module.summarize_trajectory(arrays, case)


def test_history_uses_initial_and_coulomb_residual_convergence_rules(module):
    case = _case(module, "mu_0_5_omega_5")
    history = _history(module, case)
    summary = module.summarize_history(history, case)

    assert summary["initial_converged_steps"] == 21
    assert summary["natural_residual_over_tolerance_converged_steps"] == 26
    natural_over_tolerance = next(
        step
        for step in history["steps"]
        if step["num_contacts"] > 0
        and step["converged"]
        and step["outer_iters"] > 0
        and step["final_residual"] >= module.TERMINATION_TOLERANCE
    )
    assert natural_over_tolerance["outer"][-1]["r_coulomb"] < (
        module.TERMINATION_TOLERANCE
    )

    mutated = copy.deepcopy(history)
    step = mutated["steps"][natural_over_tolerance["step_idx"]]
    step["outer"][-1]["r_coulomb"] = 10.0 * module.TERMINATION_TOLERANCE
    with pytest.raises(ValueError, match="violates coulomb_rel tolerance"):
        module.summarize_history(mutated, case)


def test_history_preserves_expected_nonconverged_plateau_not_cap(module):
    case = _case(module, "mu_0_2_omega_2")
    history = _history(module, case)
    summary = module.summarize_history(history, case)
    terminal = history["steps"][130]

    assert summary["solver_contract_valid"] is False
    assert summary["nonconverged_steps"] == [130]
    assert summary["outer_cap_hit_steps"] == []
    assert terminal["outer_iters"] == 40
    assert history["meta"]["config"]["max_outer"] == 200
    assert terminal["outer"][-1]["r_coulomb"] == pytest.approx(0.00014589023205034332)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("all_solver_contract_valid", True),
        ("paper_parity", True),
        ("dart_equivalence", True),
        ("runtime_attestation_complete", True),
        ("timing_verdict", "valid"),
        ("realtime_verdict", "valid"),
        ("claim_valid", True),
    ],
)
def test_manifest_rejects_forbidden_claim_promotion(module, field, value):
    manifest = module.read_json(BUNDLE / "manifest.json")
    manifest["predicates"][field] = value

    with pytest.raises(ValueError, match="promotes a forbidden claim"):
        module.validate_manifest_contract(manifest)


def test_manifest_rejects_nonclaim_removal(module):
    manifest = module.read_json(BUNDLE / "manifest.json")
    manifest["nonclaims"].pop()

    with pytest.raises(ValueError, match="promotes a forbidden claim"):
        module.validate_manifest_contract(manifest)


def test_manifest_rejects_boolean_integer_substitution(module):
    manifest = module.read_json(BUNDLE / "manifest.json")
    manifest["predicates"]["artifact_valid"] = 1

    with pytest.raises(ValueError, match="promotes a forbidden claim"):
        module.validate_manifest_contract(manifest)


def test_cli_verify_only_prints_strict_json(module, capsys):
    assert module.main(["--verify-only", "--bundle", str(BUNDLE)]) == 0
    captured = capsys.readouterr()
    report = json.loads(captured.out)

    assert report["pass"] is True
    assert "NaN" not in captured.out
