import argparse
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_fbf_literal_wedge_visual_capture.py"


def _load_module():
    name = "run_fbf_literal_wedge_visual_capture"
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _write_bytes(path, content=b"fixture\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


def _raw_frame_fixture(module, tmp_path):
    from _image_tools import write_png

    output = tmp_path / "capture"
    output.mkdir()
    runtime_frames = []
    validated_frames = []
    for index, step in enumerate(module.TIMELINE_STILL_STEPS):
        relative = f"frames/step_{step:06d}.png"
        path = output / relative
        pixels = bytes((30 + index * 20, 90 + index * 10, 180 - index * 20)) * 12
        write_png(path, 4, 3, pixels)
        runtime_frames.append({"step": step, "path": relative})
        validated_frames.append(
            {
                "step": step,
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": module.sha256(path),
                "pixel_sha256": hashlib.sha256(pixels).hexdigest(),
                "non_blank": {"pass": True},
                "contrast": {"pass": True},
            }
        )
    runtime = {
        "steps_completed": 600,
        "frame_stride": 150,
        "width": 4,
        "height": 3,
        "frames": runtime_frames,
    }
    frame_validation = {
        "schema_version": "dart.fbf_literal_wedge_frame_validation/v1",
        "pass": True,
        "frame_count": len(validated_frames),
        "unique_pixel_frames": len(validated_frames),
        "motion_required": False,
        "motion_note": "stability fixture",
        "frames": validated_frames,
    }
    return output, runtime, frame_validation


def _synthetic_pending_bundle(module, tmp_path, monkeypatch):
    root = tmp_path / "repo"
    output = root / "bundle"
    output.mkdir(parents=True)
    monkeypatch.setattr(module, "ROOT", root)

    capture = root / "bin" / "capture"
    trace = root / "bin" / "trace"
    source = root / "synthetic" / "source.cpp"
    cmake_cache = root / "build" / "CMakeCache.txt"
    recorded_tool = root / "bin" / "recorded-tool"
    for path in (capture, trace, source, cmake_cache, recorded_tool):
        _write_bytes(path)

    runtime = {
        "steps_completed": 1,
        "frame_stride": 1,
        "width": 2,
        "height": 2,
        "simulation_threads": 1,
    }
    frame_validation = {
        "schema_version": "synthetic-frame-validation/v1",
        "pass": True,
        "frame_count": 1,
        "frames": [],
    }
    trace_equivalence = {
        "schema_version": "synthetic-trace-equivalence/v1",
        "pass": True,
        "rows_compared": 1,
        "mismatches": [],
        "max_abs_difference": {"sim_time": 0.0},
        "expected_reference_text": {"scenario": "synthetic"},
        "expected_reference_int": {"contacts": 1},
        "expected_reference_float": {"tolerance": 1e-6},
        "defining_source_contract": {"pass": True},
    }
    video_probe = {"schema_version": "synthetic-video-probe/v1", "pass": True}

    module.write_json(output / "capture-runtime.json", runtime)
    module.write_json(output / "frame-validation.json", frame_validation)
    module.write_json(output / "trace-equivalence.json", trace_equivalence)
    module.write_json(output / "video-probe.json", video_probe)
    _write_bytes(output / "trajectory.csv", b"step\n0\n")
    _write_bytes(output / "reference-fbf-paper-trace.csv", b"step\n1\n")
    _write_bytes(output / "fig07_literal_wedge_stability.mp4")
    _write_bytes(output / "fig07_literal_wedge_timeline.png")
    _write_bytes(output / "fig07_literal_wedge_timeline.compose.json")
    _write_bytes(output / "decoded" / "video_midpoint_t3.0.png")

    provenance = {
        "schema_version": "synthetic-provenance/v1",
        "generation_mode": "revalidated_existing_raw_assets",
        "evidence_state": module.PENDING_STATUS,
        "source": {
            "capture_executable": str(capture),
            "capture_executable_sha256": module.sha256(capture),
            "trace_executable": str(trace),
            "trace_executable_sha256": module.sha256(trace),
            "binary_and_raw_source_continuity": {
                "capture_executable_matches_original_raw": True,
                "capture_source_matches_original_raw": True,
                "current_trace_revalidation": module.trace_revalidation_contract(
                    trace_equivalence, expected_rows=1
                ),
            },
            "files": [
                {
                    "path": "synthetic/source.cpp",
                    "sha256": module.sha256(source),
                }
            ],
        },
        "environment": {
            "build": {
                "cmake_cache_path": str(cmake_cache),
                "cmake_cache_sha256": module.sha256(cmake_cache),
                "cmake_cache": {"DART_DISABLE_COMPILER_CACHE": "ON"},
            },
            "tools": {
                "recorded": {
                    "available": True,
                    "executable": str(recorded_tool),
                    "executable_sha256": module.sha256(recorded_tool),
                }
            },
        },
        "runtime": {
            "path": "capture-runtime.json",
            "sha256": module.sha256(output / "capture-runtime.json"),
            "trajectory_path": "trajectory.csv",
            "trajectory_sha256": module.sha256(output / "trajectory.csv"),
            "reference_trace_path": "reference-fbf-paper-trace.csv",
            "reference_trace_sha256": module.sha256(
                output / "reference-fbf-paper-trace.csv"
            ),
        },
        "validation": {
            "automated_validation_pass": True,
            "manual_inspection_bound": False,
            "frame_validation_path": "frame-validation.json",
            "frame_validation_sha256": module.sha256(output / "frame-validation.json"),
            "trace_equivalence_path": "trace-equivalence.json",
            "trace_equivalence_sha256": module.sha256(
                output / "trace-equivalence.json"
            ),
            "video_probe_path": "video-probe.json",
            "video_probe_sha256": module.sha256(output / "video-probe.json"),
            "trace_timeline_binding": {
                "capture_runtime_sha256": module.sha256(
                    output / "capture-runtime.json"
                ),
                "frame_validation_sha256": module.sha256(
                    output / "frame-validation.json"
                ),
                "trajectory_sha256": module.sha256(output / "trajectory.csv"),
                "reference_trace_sha256": module.sha256(
                    output / "reference-fbf-paper-trace.csv"
                ),
                "trace_equivalence_sha256": module.sha256(
                    output / "trace-equivalence.json"
                ),
                "timeline_compose_manifest_sha256": module.sha256(
                    output / "fig07_literal_wedge_timeline.compose.json"
                ),
                "durable_stills": [],
            },
        },
        "media": {
            "video": {
                "path": "fig07_literal_wedge_stability.mp4",
                "sha256": module.sha256(output / "fig07_literal_wedge_stability.mp4"),
            },
            "decoded_midpoint": {
                "path": "decoded/video_midpoint_t3.0.png",
                "sha256": module.sha256(output / "decoded" / "video_midpoint_t3.0.png"),
            },
            "timeline_panel": {
                "path": "fig07_literal_wedge_timeline.png",
                "sha256": module.sha256(output / "fig07_literal_wedge_timeline.png"),
            },
            "durable_stills": [],
        },
        "capture_staging": {},
    }
    module.write_json(output / "provenance.json", provenance)

    excluded = {
        "artifact-index.json",
        "metadata.json",
        "pending-metadata.json",
        "manual-inspection.json",
    }
    module.write_json(
        output / "artifact-index.json", module.artifact_index(output, excluded)
    )
    timing = {
        "simulation_duration_seconds": 1.0,
        "captured_frames": 1,
        "playback_duration_seconds": 1.0,
        "playback_acceleration_factor": 1.0,
    }
    metadata = {
        "schema_version": "synthetic-bundle/v1",
        "status": module.PENDING_STATUS,
        "claim_valid": False,
        "manual_inspected": False,
        "manual_inspection_bound": False,
        "artifact_index_sha256": module.sha256(output / "artifact-index.json"),
        "provenance_sha256": module.sha256(output / "provenance.json"),
        "runtime_sha256": module.sha256(output / "capture-runtime.json"),
        "trace_equivalence_sha256": module.sha256(output / "trace-equivalence.json"),
        "frame_validation_sha256": module.sha256(output / "frame-validation.json"),
        "video_probe_sha256": module.sha256(output / "video-probe.json"),
        "video_sha256": module.sha256(output / "fig07_literal_wedge_stability.mp4"),
        "timeline_panel_sha256": module.sha256(
            output / "fig07_literal_wedge_timeline.png"
        ),
        "decoded_midpoint_sha256": module.sha256(
            output / "decoded" / "video_midpoint_t3.0.png"
        ),
        "durable_stills": [],
        "timing": timing,
    }
    module.write_json(output / "metadata.json", metadata)
    module.write_json(output / "pending-metadata.json", metadata)
    module.write_json(output / "manual-inspection.json", {})

    monkeypatch.setattr(module, "validate_runtime", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        module,
        "validate_frame_validation_contract",
        lambda *args, **kwargs: {},
    )
    monkeypatch.setattr(
        module,
        "_validate_staging_provenance",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        module,
        "validate_durable_stills",
        lambda *args, **kwargs: [],
    )
    monkeypatch.setattr(
        module,
        "validate_timeline_bundle",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        module,
        "_validate_bundle_paths",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        module,
        "compare_trace",
        lambda *args, **kwargs: trace_equivalence,
    )
    return output, metadata


@pytest.mark.parametrize(
    "generation_mode", ["fresh_capture", "revalidated_existing_raw_assets"]
)
def test_pending_bundle_never_marks_fresh_or_revalidated_claim_valid(
    tmp_path, generation_mode, monkeypatch
):
    module = _load_module()
    output = tmp_path / generation_mode
    output.mkdir()
    runtime_path = output / "capture-runtime.json"
    _write_bytes(runtime_path)
    for relative in (
        "trace-equivalence.json",
        "frame-validation.json",
        "video-probe.json",
        "fig07_literal_wedge_stability.mp4",
        "fig07_literal_wedge_timeline.png",
        "decoded/video_midpoint_t3.0.png",
    ):
        _write_bytes(output / relative)

    video_probe = {
        "sha256": module.sha256(output / "fig07_literal_wedge_stability.mp4"),
        "timing": {"playback_acceleration_factor": 1.0},
    }
    panel = {"sha256": module.sha256(output / "fig07_literal_wedge_timeline.png")}
    midpoint = {
        "path": "decoded/video_midpoint_t3.0.png",
        "sha256": module.sha256(output / "decoded/video_midpoint_t3.0.png"),
    }
    provenance = {
        "generation_mode": generation_mode,
        "evidence_state": module.PENDING_STATUS,
        "media": {"durable_stills": []},
    }
    args = argparse.Namespace(output=output)
    monkeypatch.setattr(
        module,
        "_validate_bundle_paths",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        module,
        "validate_artifact_index",
        lambda *args, **kwargs: None,
    )

    metadata = module.write_pending_bundle(
        args,
        runtime_path=runtime_path,
        provenance=provenance,
        trace_equivalence={"pass": True},
        frame_validation={"pass": True},
        video_probe=video_probe,
        panel=panel,
        midpoint=midpoint,
    )

    retained = json.loads((output / "pending-metadata.json").read_text())
    assert metadata == retained
    assert (output / "metadata.json").read_bytes() == (
        output / "pending-metadata.json"
    ).read_bytes()
    assert retained["status"] == module.PENDING_STATUS
    assert retained["claim_valid"] is False
    assert retained["manual_inspection_bound"] is False
    assert retained["staging_pruned"] is True
    assert retained["raw_frame_staging_required"] is False


def test_pending_hash_dag_requires_retained_pending_metadata(tmp_path, monkeypatch):
    module = _load_module()
    output, metadata = _synthetic_pending_bundle(module, tmp_path, monkeypatch)
    (output / "pending-metadata.json").unlink()

    with pytest.raises(ValueError, match="immutable pending-metadata.json is missing"):
        module.verify_pending_hash_dag(output, metadata)


def test_pending_hash_dag_accepts_complete_unchanged_fixture(tmp_path, monkeypatch):
    module = _load_module()
    output, metadata = _synthetic_pending_bundle(module, tmp_path, monkeypatch)

    result = module.verify_pending_hash_dag(output, metadata)

    assert result == {
        "pass": True,
        "artifact_count": 11,
        "source_count": 1,
        "frame_count": 1,
        "trace_rows": 1,
        "pending_metadata_sha256": module.sha256(output / "pending-metadata.json"),
    }


def test_mutated_pending_artifact_aborts_finalization_before_writes(
    tmp_path, monkeypatch
):
    module = _load_module()
    output, _ = _synthetic_pending_bundle(module, tmp_path, monkeypatch)
    control_paths = (
        output / "metadata.json",
        output / "pending-metadata.json",
        output / "provenance.json",
        output / "artifact-index.json",
    )
    before = {path: path.read_bytes() for path in control_paths}
    _write_bytes(output / "trajectory.csv", b"step\n0\nmutated\n")

    def unexpected_manual_verification(*args, **kwargs):
        raise AssertionError("manual verification ran before the hash DAG passed")

    monkeypatch.setattr(
        module, "verify_manual_inspection", unexpected_manual_verification
    )
    args = argparse.Namespace(
        output=output,
        manual_inspection_record=output / "manual-inspection.json",
    )

    with pytest.raises(ValueError, match="pending hash DAG verification failed"):
        module.finalize_existing(args)

    assert {path: path.read_bytes() for path in control_paths} == before


def test_trace_revalidation_requires_complete_zero_difference_contract():
    module = _load_module()
    comparison = {
        "pass": True,
        "rows_compared": 600,
        "mismatches": [],
        "max_abs_difference": {"sim_time": 0.0, "residual": 0.0},
        "expected_reference_text": {"scenario": "literal"},
        "expected_reference_int": {"contacts": 96},
        "expected_reference_float": {"tolerance": 1e-6},
        "defining_source_contract": {"pass": True},
    }

    accepted = module.trace_revalidation_contract(comparison, expected_rows=600)
    comparison["max_abs_difference"]["residual"] = 1e-16
    rejected = module.trace_revalidation_contract(comparison, expected_rows=600)

    assert accepted["pass"] is True
    assert accepted["row_count_matches"] is True
    assert accepted["complete_contract_sections_present"] is True
    assert rejected["pass"] is False
    assert rejected["zero_numeric_difference"] is False


def test_raw_frames_are_promoted_to_hash_bound_durable_stills_then_pruned(tmp_path):
    module = _load_module()
    output, runtime, frame_validation = _raw_frame_fixture(module, tmp_path)
    _write_bytes(output / "capture.stdout.txt", b"capture log\n")

    bindings = module.promote_durable_stills(output, runtime, frame_validation)
    staging = module._record_staging(output)
    module._prune_staging(output)
    staging["staging_pruned"] = True

    assert [item["step"] for item in bindings] == list(module.TIMELINE_STILL_STEPS)
    assert not (output / "frames").exists()
    assert not (output / "capture.stdout.txt").exists()
    assert (
        module.validate_durable_stills(output, runtime, frame_validation, bindings)
        == bindings
    )
    module._validate_staging_provenance(staging, frame_validation)


@pytest.mark.parametrize("failure", ["missing", "tampered"])
def test_durable_still_binding_rejects_missing_or_tampered_still(tmp_path, failure):
    module = _load_module()
    output, runtime, frame_validation = _raw_frame_fixture(module, tmp_path)
    bindings = module.promote_durable_stills(output, runtime, frame_validation)
    target = output / module.STILL_PATH_BY_STEP[300]
    if failure == "missing":
        target.unlink()
    else:
        target.write_bytes(b"tampered\n")

    with pytest.raises(ValueError, match="durable still differs"):
        module.validate_durable_stills(
            output,
            runtime,
            frame_validation,
            bindings,
        )


def test_sealed_membership_rejects_ignored_staging_and_unknown_files(tmp_path):
    module = _load_module()
    output = tmp_path / "sealed"
    for relative in module.FINAL_DURABLE_PATHS:
        _write_bytes(output / relative)

    module._validate_bundle_paths(output, complete=True, final=True)
    _write_bytes(output / "frames" / "step_000010.png")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(output, complete=True, final=True)
    (output / "frames" / "step_000010.png").unlink()
    (output / "frames").rmdir()
    _write_bytes(output / "editor-backup.tmp")
    with pytest.raises(ValueError, match="unexpected entries"):
        module._validate_bundle_paths(output, complete=True, final=True)


def test_artifact_index_requires_exact_sealed_membership(tmp_path):
    module = _load_module()
    output = tmp_path / "sealed"
    for relative in module.FINAL_DURABLE_PATHS:
        if relative != "artifact-index.json":
            _write_bytes(output / relative, relative.encode())
    index = module.artifact_index(output, module.INDEX_EXCLUSIONS_FINAL)
    module.write_json(output / "artifact-index.json", index)

    module.validate_artifact_index(
        output,
        index,
        excluded=module.INDEX_EXCLUSIONS_FINAL,
    )
    _write_bytes(output / "unindexed.txt")
    with pytest.raises(ValueError, match="membership mismatch"):
        module.validate_artifact_index(
            output,
            index,
            excluded=module.INDEX_EXCLUSIONS_FINAL,
        )


def test_bundle_transaction_restores_exact_tree_after_failure(tmp_path):
    module = _load_module()
    output = tmp_path / "bundle"
    _write_bytes(output / "metadata.json", b"before metadata\n")
    _write_bytes(output / "nested" / "artifact.bin", b"before artifact\n")
    before = {
        path.relative_to(output).as_posix(): path.read_bytes()
        for path in output.rglob("*")
        if path.is_file()
    }

    with pytest.raises(RuntimeError, match="injected"):
        with module._bundle_transaction(output):
            (output / "metadata.json").write_bytes(b"after\n")
            (output / "nested" / "artifact.bin").unlink()
            _write_bytes(output / "new.txt")
            raise RuntimeError("injected failure")

    after = {
        path.relative_to(output).as_posix(): path.read_bytes()
        for path in output.rglob("*")
        if path.is_file()
    }
    assert after == before


@pytest.mark.parametrize(
    "entrypoint", ["generate_pending", "finalize_existing", "verify_only"]
)
def test_entrypoints_reject_root_and_ancestor_symlinks(tmp_path, entrypoint):
    module = _load_module()

    def invoke(bundle, *, ancestor):
        if entrypoint == "generate_pending":
            return module.generate_pending(
                argparse.Namespace(
                    output=bundle,
                    revalidate_existing=not ancestor,
                )
            )
        return getattr(module, entrypoint)(argparse.Namespace(output=bundle))

    target = tmp_path / "target"
    target.mkdir()
    root_link = tmp_path / "bundle-link"
    root_link.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match="bundle root is a symlink"):
        invoke(root_link, ancestor=False)

    real_parent = tmp_path / "real-parent"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    ancestor_bundle = linked_parent / (
        "new-bundle" if entrypoint == "generate_pending" else "bundle"
    )
    if entrypoint != "generate_pending":
        (real_parent / "bundle").mkdir()
    with pytest.raises(ValueError, match="passes through a symlink"):
        invoke(ancestor_bundle, ancestor=True)
    assert not (real_parent / "new-bundle").exists()


def test_verify_only_is_a_distinct_mode_without_manual_record():
    module = _load_module()

    args = module.parse_args(["--verify-only"])

    assert args.verify_only is True
    assert args.finalize_existing is False
    assert args.revalidate_existing is False
    assert args.manual_inspection_record is None
