"""Focused tests for the current-author backspin source finalizer."""

from __future__ import annotations

import copy
import gzip
import importlib.util
import json
import shutil
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/finalize_fbf_author_backspin_reference.py"


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location(
        "finalize_fbf_author_backspin_reference", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


@pytest.fixture(scope="module")
def evidence_bundle(module):
    if not module.DEFAULT_BUNDLE.exists():
        pytest.skip(
            "requires the ignored local author-backspin evidence bundle at "
            f"{module.DEFAULT_BUNDLE}"
        )
    return module.DEFAULT_BUNDLE


@pytest.fixture(scope="module")
def source_run(module):
    run = module.DEFAULT_SOURCE_REPO / module.SOURCE_RUN_DIRECTORY
    if not run.exists():
        pytest.skip(f"requires the pinned local author backspin run at {run}")
    return run


def _valid_arrays(module) -> dict[str, np.ndarray]:
    vx = np.full(module.TOTAL_STEPS, module.SOURCE_TERMINAL["vx"], dtype=np.float64)
    wy = np.full(module.TOTAL_STEPS, module.SOURCE_TERMINAL["wy"], dtype=np.float64)
    z = np.zeros(module.TOTAL_STEPS, dtype=np.float64)
    z[-1] = module.SOURCE_TERMINAL["z"]
    return {
        "t": np.cumsum(
            np.full(module.TOTAL_STEPS, module.DT_SECONDS, dtype=np.float64)
        ),
        "vx": vx,
        "wy": wy,
        "vc": vx - module.RADIUS * wy,
        "z": z,
    }


def _write_npz(path: Path, arrays: dict[str, np.ndarray]) -> None:
    np.savez(path, **arrays)


def _valid_result(module) -> dict[str, object]:
    return {
        "T_seconds": 4.0,
        "avg_step_time_ms": 1.0,
        "config": module._expected_config(module.RESULT_HISTORY_PATH),
        "dt": module.DT_SECONDS,
        "mu": 0.5,
        "num_steps": module.TOTAL_STEPS,
        "omega0": -200.0,
        "solver": "fbf",
        "step_times_ms": [1.0] * module.TOTAL_STEPS,
        "usd": None,
        "v0": 4.0,
        "vx_final": module.SOURCE_TERMINAL["vx"],
        "wy_final": module.SOURCE_TERMINAL["wy"],
    }


def test_schema_and_source_identity_are_stable(module):
    assert module.SCHEMA_VERSION == "dart.fbf_author_backspin_reference/v1"
    assert module.VALIDATION_SCHEMA_VERSION == "dart.fbf_author_backspin_validation/v1"
    assert module.SOURCE_COMMIT == "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
    assert module.SOURCE_TREE == "ffcdafb61adeda2239c8366d054b548b50d26685"
    assert module.RUNNER_GIT_BLOB == "82d8916233df8db1cacc5915699dc53a5d08ea17"
    assert module.RUNNER_SHA256 == (
        "c9174e88bf18dbe050d72568639de50b8477f2bc57ea9558637087da4268409a"
    )


def test_invocation_and_configuration_are_literal(module):
    manifest = module._expected_manifest()
    assert manifest["invocation"]["argv"] == list(module.RECORDED_INVOCATION)
    assert manifest["invocation"]["returncode"] == 0
    assert manifest["configuration"]["result"] == module._expected_config(
        module.RESULT_HISTORY_PATH
    )
    assert manifest["configuration"]["history"] == module._expected_config(
        module.SOURCE_HISTORY_PATH
    )


def test_terminal_projection_and_analytic_state_are_sealed(module):
    assert module.SOURCE_TERMINAL == {
        "vx": -11.428571701049805,
        "wy": -45.71428680419922,
        "vc": 0.0,
        "z": -1.3714094161987305,
    }
    impulse = (
        module.RADIUS * module.EXPECTED_METADATA["omega0"]
        - module.EXPECTED_METADATA["v0"]
    ) / (1.0 + 1.0 / module.MASS_NORMALIZED_INERTIA)
    analytic_vx = module.EXPECTED_METADATA["v0"] + impulse
    analytic_wy = module.EXPECTED_METADATA["omega0"] - impulse / (
        module.MASS_NORMALIZED_INERTIA * module.RADIUS
    )
    assert impulse == module.ANALYTIC_STATE["impulse_per_mass"]
    assert analytic_vx == module.ANALYTIC_STATE["vx"]
    assert analytic_wy == module.ANALYTIC_STATE["wy"]
    assert module.ANALYTIC_STATE["vc"] == 0.0


def test_valid_synthetic_trajectory_is_accepted(module, tmp_path):
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(module)
    _write_npz(path, arrays)

    loaded = module.load_trajectory(path)

    assert tuple(loaded) == module.TRAJECTORY_KEYS
    assert all(array.shape == (240,) for array in loaded.values())
    assert loaded["vx"][-1] == module.SOURCE_TERMINAL["vx"]


def test_missing_trajectory_array_fails_closed(module, tmp_path):
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(module)
    del arrays["z"]
    _write_npz(path, arrays)

    with pytest.raises(ValueError, match="missing, reordered, or unexpected"):
        module.load_trajectory(path)


@pytest.mark.parametrize("key", ("t", "vx", "wy", "vc", "z"))
def test_nonfinite_trajectory_array_fails_closed(key, tmp_path, request):
    loaded_module = request.getfixturevalue("module")
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(loaded_module)
    arrays[key][7] = np.nan
    _write_npz(path, arrays)

    with pytest.raises(ValueError, match="non-finite"):
        loaded_module.load_trajectory(path)


def test_trajectory_dtype_and_shape_fail_closed(module, tmp_path):
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(module)
    arrays["z"] = arrays["z"].astype(np.float32)
    _write_npz(path, arrays)
    with pytest.raises(ValueError, match="expected float64"):
        module.load_trajectory(path)

    arrays = _valid_arrays(module)
    arrays["z"] = arrays["z"][:-1]
    _write_npz(path, arrays)
    with pytest.raises(ValueError, match=r"expected shape \(240,\)"):
        module.load_trajectory(path)


def test_time_grid_mutation_fails_closed(module, tmp_path):
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(module)
    arrays["t"][10] += np.finfo(np.float64).eps
    _write_npz(path, arrays)

    with pytest.raises(ValueError, match="exact post-step time grid"):
        module.load_trajectory(path)


def test_slip_projection_mutation_fails_closed(module, tmp_path):
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(module)
    arrays["vx"][10] += 1.0
    _write_npz(path, arrays)

    with pytest.raises(ValueError, match="vc is inconsistent"):
        module.load_trajectory(path)


def test_terminal_mutation_fails_closed_even_if_slip_is_recomputed(module, tmp_path):
    path = tmp_path / "trajectory.npz"
    arrays = _valid_arrays(module)
    arrays["vx"][-1] += 1.0
    arrays["vc"] = arrays["vx"] - module.RADIUS * arrays["wy"]
    _write_npz(path, arrays)

    with pytest.raises(ValueError, match="source terminal value changed"):
        module.load_trajectory(path)


def test_metadata_schema_and_exact_types_fail_closed(module):
    module._validate_metadata(copy.deepcopy(module.EXPECTED_METADATA))

    metadata = copy.deepcopy(module.EXPECTED_METADATA)
    metadata["extra"] = True
    with pytest.raises(ValueError, match="exact keys changed"):
        module._validate_metadata(metadata)

    metadata = copy.deepcopy(module.EXPECTED_METADATA)
    metadata["num_steps"] = 240.0
    with pytest.raises(ValueError, match="exact JSON value or type changed"):
        module._validate_metadata(metadata)


def test_result_outcome_config_and_timings_fail_closed(module):
    module._validate_result(_valid_result(module))

    result = _valid_result(module)
    result["config"]["max_outer"] = 201
    with pytest.raises(ValueError, match="result.config"):
        module._validate_result(result)

    result = _valid_result(module)
    result["vx_final"] += 1.0
    with pytest.raises(ValueError, match="result.vx_final"):
        module._validate_result(result)

    result = _valid_result(module)
    result["step_times_ms"][0] = float("nan")
    with pytest.raises(ValueError, match="finite JSON float"):
        module._validate_result(result)

    result = _valid_result(module)
    result["avg_step_time_ms"] = 2.0
    with pytest.raises(ValueError, match="inconsistent with step samples"):
        module._validate_result(result)


def test_manifest_cannot_promote_forbidden_claims(module):
    manifest = module._expected_manifest()
    manifest["predicates"]["paper_parity"] = True

    with pytest.raises(ValueError, match="promotes a forbidden claim"):
        module._validate_manifest(manifest)


@pytest.mark.parametrize(
    "value",
    ("", ".", "./a", "a/./b", "a/", "a//b", "a/../b", "/a", "a\\b"),
)
def test_noncanonical_relative_paths_fail_closed(module, value):
    with pytest.raises(ValueError, match="unsafe or noncanonical"):
        module._safe_relative(value, "test path")


def test_pinned_source_run_and_history_are_valid(module, source_run):
    payloads = module._load_source_run(module.DEFAULT_SOURCE_REPO)
    history = module._parse_json_bytes(payloads["fbf/history.json"], "history")

    assert module._validate_history(history) == module.EXPECTED_HISTORY_SUMMARY


def test_history_outcome_mutations_fail_closed(module, source_run):
    history = module._read_json(source_run / "fbf/history.json")
    history["steps"][0]["converged"] = False
    with pytest.raises(ValueError, match="configured terminal gate"):
        module._validate_history(history)

    history = module._read_json(source_run / "fbf/history.json")
    del history["steps"][0]["final_residual"]
    with pytest.raises(ValueError, match="exact keys changed"):
        module._validate_history(history)

    history = module._read_json(source_run / "fbf/history.json")
    history["steps"][0]["outer"][-1]["r_coulomb"] = 2.0e-6
    with pytest.raises(ValueError, match="configured terminal gate"):
        module._validate_history(history)


def test_source_artifact_same_size_mutation_fails_before_semantics(
    module, source_run, tmp_path
):
    source = tmp_path / "source"
    copied_run = source / module.SOURCE_RUN_DIRECTORY
    shutil.copytree(source_run, copied_run)
    metadata = copied_run / "metadata.json"
    data = bytearray(metadata.read_bytes())
    data[data.index(b"backspin_ball")] ^= 1
    metadata.write_bytes(data)

    with pytest.raises(ValueError, match="source artifact identity changed"):
        module._load_source_run(source)


def test_default_bundle_verifies(module, evidence_bundle):
    report = module.validate_bundle(evidence_bundle)

    assert report["schema_version"] == module.VALIDATION_SCHEMA_VERSION
    assert report["artifact_count"] == 4
    assert report["trajectory_state_count"] == 240
    assert report["converged_steps"] == 240
    assert report["terminal"] == module.SOURCE_TERMINAL
    assert report["paper_parity"] is False
    assert report["dart_equivalence"] is False


def test_bundle_membership_and_symlinks_fail_closed(module, evidence_bundle, tmp_path):
    bundle = tmp_path / "bundle"
    shutil.copytree(evidence_bundle, bundle)
    (bundle / "unexpected.txt").write_text("unexpected", encoding="utf-8")
    with pytest.raises(ValueError, match="file membership changed"):
        module.validate_bundle(bundle)

    bundle = tmp_path / "symlink-bundle"
    shutil.copytree(evidence_bundle, bundle)
    target = bundle / "runs" / module.RUN_ID / "metadata.json"
    target.unlink()
    target.symlink_to(evidence_bundle / "runs" / module.RUN_ID / "metadata.json")
    with pytest.raises(ValueError, match="contains symlink"):
        module.validate_bundle(bundle)


def test_bundle_artifact_same_size_mutation_fails_closed(
    module, evidence_bundle, tmp_path
):
    bundle = tmp_path / "bundle"
    shutil.copytree(evidence_bundle, bundle)
    result = bundle / "runs" / module.RUN_ID / "fbf/result.json"
    data = bytearray(result.read_bytes())
    data[data.index(b'"solver": "fbf"')] ^= 1
    result.write_bytes(data)

    with pytest.raises(ValueError, match="bundle artifact identity changed"):
        module.validate_bundle(bundle)


def test_concatenated_gzip_history_fails_closed(module, evidence_bundle, tmp_path):
    source = evidence_bundle / "runs" / module.RUN_ID / "fbf/history.json.gz"
    path = tmp_path / "history.json.gz"
    path.write_bytes(source.read_bytes() + gzip.compress(b"trailing", mtime=0))
    record = next(
        record
        for record in module.EXPECTED_ARTIFACTS
        if record["path"].endswith("history.json.gz")
    )

    with pytest.raises(ValueError, match="truncated or concatenated"):
        module._decompress_history(path, record)


def test_atomic_publish_restores_previous_bundle_on_validation_failure(
    module, tmp_path, monkeypatch
):
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    (bundle / "old.txt").write_text("old", encoding="utf-8")
    staging = tmp_path / "staging"
    staging.mkdir()
    (staging / "new.txt").write_text("new", encoding="utf-8")

    def fail_validation(_bundle):
        raise ValueError("injected validation failure")

    monkeypatch.setattr(module, "validate_bundle", fail_validation)
    with pytest.raises(ValueError, match="injected validation failure"):
        module._publish_staged_bundle(staging, bundle)

    assert (bundle / "old.txt").read_text(encoding="utf-8") == "old"
    assert not (bundle / "new.txt").exists()
    assert not staging.exists()
