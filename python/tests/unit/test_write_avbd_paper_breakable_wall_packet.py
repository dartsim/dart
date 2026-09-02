from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from fractions import Fraction
from hashlib import sha256
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_paper_breakable_wall_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_paper_breakable_wall_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_png(
    path: Path, width: int = 8, height: int = 6, *, red_offset: int = 0
) -> None:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    rows = b"".join(
        b"\x00" + bytes((32 + row + red_offset, 64, 128)) * width
        for row in range(height)
    )
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _build_configuration(module) -> dict[str, object]:
    values = {key: "<UNDEFINED>" for key in module.BUILD_CONFIGURATION_KEYS}
    for definition in module.EVIDENCE_CMAKE_DEFINITIONS:
        name, value = definition.split("=", maxsplit=1)
        if name in values:
            values[name] = value
    values.update(
        {
            "CMAKE_CXX_COMPILER": "/usr/bin/c++",
            "CMAKE_CXX_COMPILER_ID": "GNU",
            "CMAKE_CXX_COMPILER_VERSION": "15.2.0",
            "CMAKE_GENERATOR": "Ninja",
            "CMAKE_SYSTEM_NAME": "Linux",
            "CMAKE_SYSTEM_PROCESSOR": "x86_64",
        }
    )
    record = "".join(
        [f"algorithm={module.BUILD_CONFIGURATION_ALGORITHM}\n"]
        + [f"{key}={values[key]}\n" for key in module.BUILD_CONFIGURATION_KEYS]
    )
    return {
        "algorithm": module.BUILD_CONFIGURATION_ALGORITHM,
        "digest": sha256(record.encode("utf-8")).hexdigest(),
        "values": values,
    }


def _reseal_benchmark_evidence(module, data: dict[str, object]) -> None:
    evidence = data["dart_evidence_run"]
    build_identity = evidence["build_identity"]
    build_payload = {
        key: value
        for key, value in build_identity.items()
        if key not in {"algorithm", "digest"}
    }
    build_identity["digest"] = module._canonical_json_digest(build_payload)
    run_payload = {
        key: value
        for key, value in evidence.items()
        if key not in {"schema_version", "digest"}
    }
    evidence["digest"] = module._canonical_json_digest(run_payload)


def test_artifact_path_resolves_supported_path_forms(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_packet_module()
    repo_root = tmp_path / "repo"
    capture_dir = repo_root / "build" / "evidence" / "capture"
    capture_dir.mkdir(parents=True)
    manifest = capture_dir / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")
    manifest_relative = capture_dir / "manifest-relative.png"
    repository_relative = capture_dir / "repository-relative.png"
    absolute = tmp_path / "absolute.png"
    for artifact in (manifest_relative, repository_relative, absolute):
        _write_png(artifact)
    monkeypatch.setattr(module, "REPO_ROOT", repo_root)

    assert (
        module._artifact_path(
            manifest,
            manifest_relative.name,
            "artifacts.screenshot",
        )
        == manifest_relative
    )
    assert (
        module._artifact_path(
            manifest,
            repository_relative.relative_to(repo_root).as_posix(),
            "artifacts.screenshot",
        )
        == repository_relative
    )
    assert (
        module._artifact_path(
            manifest,
            str(absolute),
            "artifacts.screenshot",
        )
        == absolute
    )


def test_artifact_path_rejects_ambiguous_relative_path(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_packet_module()
    repo_root = tmp_path / "repo"
    capture_dir = repo_root / "captures"
    manifest_candidate = capture_dir / "shared" / "frame.png"
    repository_candidate = repo_root / "shared" / "frame.png"
    manifest_candidate.parent.mkdir(parents=True)
    repository_candidate.parent.mkdir(parents=True)
    _write_png(manifest_candidate)
    _write_png(repository_candidate)
    monkeypatch.setattr(module, "REPO_ROOT", repo_root)

    with pytest.raises(
        module.AvbdPaperBreakableWallPacketError,
        match="artifacts.screenshot path is ambiguous",
    ):
        module._artifact_path(
            capture_dir / "manifest.json",
            "shared/frame.png",
            "artifacts.screenshot",
        )


@pytest.mark.parametrize("artifact_kind", ["missing", "directory", "symlink"])
def test_artifact_path_rejects_unsafe_or_non_file_targets(
    tmp_path: Path,
    monkeypatch,
    artifact_kind: str,
) -> None:
    module = _load_packet_module()
    repo_root = tmp_path / "repo"
    capture_dir = repo_root / "capture"
    capture_dir.mkdir(parents=True)
    artifact = capture_dir / "artifact"
    if artifact_kind == "directory":
        artifact.mkdir()
        message = "must be a regular file"
    elif artifact_kind == "symlink":
        target = capture_dir / "target.png"
        _write_png(target)
        artifact.symlink_to(target)
        message = "must not be a symlink"
    else:
        message = "does not resolve to a regular file"
    monkeypatch.setattr(module, "REPO_ROOT", repo_root)

    with pytest.raises(module.AvbdPaperBreakableWallPacketError, match=message):
        module._artifact_path(
            capture_dir / "manifest.json",
            artifact.name,
            "artifacts.screenshot",
        )


@pytest.mark.parametrize(
    ("unsafe", "message"),
    (
        ("bad\x00path.png", "must not contain NUL"),
        ("bad\ud800path.png", "must be UTF-8-encodable text"),
    ),
)
def test_artifact_paths_reject_nul_and_surrogates(
    tmp_path: Path,
    unsafe: str,
    message: str,
) -> None:
    module = _load_packet_module()
    manifest = tmp_path / "manifest.json"

    with pytest.raises(module.AvbdPaperBreakableWallPacketError, match=message):
        module._artifact_path(manifest, unsafe, "artifacts.screenshot")
    with pytest.raises(module.AvbdPaperBreakableWallPacketError, match=message):
        module._artifact_directory(manifest, unsafe, "artifacts.frames")


def _broken_joint_records(
    *,
    broken_count: int,
    impact_region_counts: tuple[int, int, int],
) -> list[dict[str, Any]]:
    records = []
    impact_region_count = sum(impact_region_counts)
    impact_indices = [
        impact_index
        for impact_index, count in enumerate(impact_region_counts)
        for _ in range(count)
    ]
    for index in range(broken_count):
        within_region = index < impact_region_count
        impact_index = impact_indices[index] if within_region else index % 3
        records.append(
            {
                "angular_residual_radians": 0.01 + 0.0001 * index,
                "child": {
                    "body": "brick",
                    "column": (index + 1) % 21,
                    "row": (index // 21) % 12,
                },
                "id": f"fixture_joint_{index:03d}",
                "initial_anchor": [
                    float(index % 21),
                    0.0,
                    float((index // 21) % 12),
                ],
                "kind": "horizontal" if index % 2 == 0 else "vertical",
                "linear_residual": 0.001 + 0.000001 * index,
                "nearest_impact_distance": 0.5 if within_region else 2.0,
                "nearest_impact_index": impact_index,
                "parent": {
                    "body": "brick",
                    "column": index % 21,
                    "row": (index // 21) % 12,
                },
                "within_impact_band": within_region,
                "within_impact_region": within_region,
            }
        )
    return records


def _broken_joint_ids_sha256(records: list[dict[str, Any]]) -> str:
    digest = sha256()
    for joint_id in sorted(record["id"] for record in records):
        encoded_id = joint_id.encode("utf-8")
        digest.update(struct.pack("<Q", len(encoded_id)))
        digest.update(encoded_id)
    return digest.hexdigest()


def _outcome(module, frame: int) -> dict[str, Any]:
    checkpoints = {
        60: {
            "bands": [0, 0, 0],
            "broken": 36,
            "damage": True,
            "evaluated": False,
            "outside": 1.0,
            "status": "pre-evaluation",
            "thresholds_pass": False,
            "total": 1.0,
            "unbroken": 676,
        },
        120: {
            "bands": [0, 0, 0],
            "broken": 36,
            "damage": True,
            "evaluated": True,
            "outside": 1.0,
            "status": "pass",
            "thresholds_pass": True,
            "total": 1.0,
            "unbroken": 676,
        },
        600: {
            "bands": [0, 0, 0],
            "broken": 36,
            "damage": True,
            "evaluated": True,
            "outside": 1.0,
            "status": "pass",
            "thresholds_pass": True,
            "total": 1.0,
            "unbroken": 676,
        },
    }
    expected = checkpoints.get(frame)
    if expected is None:
        expected = (
            checkpoints[120]
            if frame >= 120
            else {
                "bands": [0, 0, 0],
                "broken": 0,
                "damage": False,
                "evaluated": False,
                "outside": 1.0,
                "status": "pre-evaluation",
                "thresholds_pass": False,
                "total": 1.0,
                "unbroken": 712,
            }
        )
    records = (
        _broken_joint_records(
            broken_count=expected["broken"],
            impact_region_counts=(5, 5, 5),
        )
        if frame in (60, 120, 600)
        else []
    )
    identity_digest = _broken_joint_ids_sha256(
        _broken_joint_records(
            broken_count=expected["broken"],
            impact_region_counts=(5, 5, 5),
        )
    )
    return {
        "ball_positions": [[0.0, 1.0, 2.0]] * 3,
        "ball_velocities": [[3.0, 4.0, 5.0]] * 3,
        "broken_joint_identity_count": expected["broken"],
        "broken_joint_ids_sha256": identity_digest,
        "broken_joint_impact_region_counts": (
            [5, 5, 5] if expected["broken"] else [0, 0, 0]
        ),
        "broken_joint_records": records,
        "broken_joints": expected["broken"],
        "broken_joints_outside_impact_regions": (
            expected["broken"] - 15 if expected["broken"] else 0
        ),
        "contact_count": 12,
        "checkpoint": "outcome",
        "evaluated": expected["evaluated"],
        "frame": frame,
        "impact_band_displaced_counts": expected["bands"],
        "last_step_iterations": 20,
        "max_brick_displacement": 3.0,
        "maximum_outside_impact_unbroken_joint_angular_residual_radians": (0.0003),
        "maximum_outside_impact_unbroken_joint_linear_residual": 0.001,
        "maximum_unbroken_joint_angular_residual_radians": 0.0003,
        "maximum_unbroken_joint_linear_residual": 0.001,
        "outside_brick_count": 181,
        "outside_impact_unbroken_joint_residual_count": (
            463 if expected["broken"] else 484
        ),
        "outside_retained_fraction": expected["outside"],
        "joint_residuals_finite": True,
        "rms_outside_impact_unbroken_joint_angular_residual_radians": 0.0001,
        "rms_outside_impact_unbroken_joint_linear_residual": 0.0005,
        "rms_unbroken_joint_angular_residual_radians": 0.0001,
        "rms_unbroken_joint_linear_residual": 0.0005,
        "status": expected["status"],
        "threshold_checks": {
            "finite_state": True,
            "fracture_activated": True,
            "fracture_count_bounded": True,
            "fracture_identity_matches": True,
            "fracture_in_three_impact_regions": expected["damage"],
            "outside_wall_retained": True,
            "retained_joint_rows_satisfied": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": expected["thresholds_pass"],
        "total_retained_fraction": expected["total"],
        "unbroken_joints": expected["unbroken"],
        "unbroken_joint_residual_count": expected["unbroken"],
        "world_time": frame / 60.0,
    }


def _metrics(module, frame: int) -> dict[str, Any]:
    return {
        "ball_count": 3,
        "break_force": module.BREAK_FORCE,
        "breakable_joints": 712,
        "brick_count": 252,
        "collision_shapes": 256,
        "executor": "World.step default",
        "effective_scene_contract_passed": True,
        "outcome": _outcome(module, frame),
        "outcome_oracle": dict(module.OUTCOME_ORACLE),
        "paper_locator": module.PAPER_LOCATOR,
        "rigid_bodies": 256,
        "rigid_body_solver": "AVBD",
        "rigid_constraint_options": {"iterations": 20},
        "resolved_configuration": [
            {
                "domain": "rigid-body",
                "reason": "as requested",
                "requested": "avbd",
                "resolved": "avbd",
            },
            {
                "domain": "rigid-contact",
                "reason": "as requested",
                "requested": "avbd",
                "resolved": "avbd",
            },
            {
                "domain": "rigid-pair-constraint",
                "reason": "as requested",
                "requested": "avbd",
                "resolved": "avbd",
            },
            {
                "domain": "rigid-constraint-iterations",
                "reason": "public rigid constraint iteration budget",
                "requested": "20",
                "resolved": "20",
            },
        ],
        "row": module.SCENE_ID,
        "scene_spec_fingerprint": "0123456789abcdef",
        "solver": "public_avbd",
        "time_step_ms": 1000.0 / 60.0,
        "view_report": {
            "camera": {
                "azimuth": module.CAMERA_AZIMUTH,
                "distance": module.CAMERA_DISTANCE,
                "elevation": module.CAMERA_ELEVATION,
                "target": list(module.CAMERA_TARGET),
            },
            "focus": list(module.VIEW_FOCUS),
            "issues": [],
            "metrics": {
                "ambiguity_iou": 0.0,
                "center_visible": True,
                "corner_coverage": 1.0,
                "occlusion_fraction": 0.0,
                "subject_fraction": 0.5,
            },
            "pass": True,
            "schema_version": "dart.view_report/v1",
            "score": 1.0,
            "size": [8, 6],
        },
    }


def _write_capture(module, tmp_path: Path, frame: int, label: str) -> tuple[Path, Path]:
    directory = tmp_path / label
    directory.mkdir()
    screenshot = directory / f"{module.SCENE_ID}_{label}.png"
    _write_png(screenshot)
    frames = directory / "png_frames"
    frames.mkdir()
    frame_paths = []
    for index in range(1, frame + 1):
        frame_path = frames / f"frame_{index:06d}.png"
        _write_png(frame_path)
        frame_paths.append(frame_path)
    video = directory / f"{module.SCENE_ID}_{label}.mp4"
    video.write_bytes(f"{module.SCENE_ID}:{label}:mp4".encode())
    metrics_log = directory / "scene_metrics.jsonl"
    events = [
        {
            "event": "scene_capture_metrics",
            "frame": event_frame,
            "metrics": _metrics(module, event_frame),
            "scene": module.SCENE_ID,
            "source": "py-demo-scene",
        }
        for event_frame in range(1, frame + 1)
    ]
    latest_event = events[-1]
    metrics_log.write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )
    # The fixture models a sealed capture taken from a clean tree; the live
    # test checkout may be dirty, so the recorded flags are forced here.
    capture_source = {
        **module.compute_capture_source_provenance(module.REPO_ROOT),
        "ignored_paths": [],
        "working_tree_clean": True,
    }
    build_configuration_digest = _build_configuration(module)["digest"]
    runtime_library = tmp_path / "libdart-simulation.so"
    runtime_library.write_bytes(b"capture DART shared library")
    runtime_library_identity_payload = {
        "build_configuration_digest": build_configuration_digest,
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15.2.0",
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": capture_source["git_head"],
        "source_provenance_digest": capture_source["digest"],
    }
    runtime_payload = {
        "dart_library_linkage": "shared",
        "loaded_dart_libraries": [
            {
                "build_identity": {
                    "algorithm": module.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
                    **runtime_library_identity_payload,
                    "digest": module._canonical_json_digest(
                        runtime_library_identity_payload
                    ),
                },
                "file": runtime_library.name,
                "path": str(runtime_library),
                "sha256": _sha256(runtime_library),
                "size_bytes": runtime_library.stat().st_size,
            }
        ],
        "native_extension": {
            "build_configuration_digest": build_configuration_digest,
            "file": "_dartpy.so",
            "module": "dartpy._dartpy",
            "path": str(tmp_path / "_dartpy.so"),
            "sha256": "1" * 64,
            "size_bytes": 1,
            "source_git_head": capture_source["git_head"],
            "source_provenance_digest": capture_source["digest"],
        },
        "source_git_head": capture_source["git_head"],
        "source_provenance_digest": capture_source["digest"],
    }
    manifest = {
        "artifacts": {
            "events": None,
            "frames": str(frames),
            "scene_metrics_events": str(metrics_log),
            "screenshot": str(screenshot),
            "video": str(video),
        },
        "camera": {
            "azimuth": module.math.degrees(module.CAMERA_AZIMUTH),
            "distance": module.CAMERA_DISTANCE,
            "elevation": module.math.degrees(module.CAMERA_ELEVATION),
            "target": list(module.CAMERA_TARGET),
            "view": module.CAMERA_PRESET,
        },
        "capture": {
            "converted_frames": frame,
            "height": 6,
            "requested_frames": frame,
            "width": 8,
        },
        "capture_artifact_provenance": module.capture_artifact_provenance(
            scene_metrics_events=metrics_log,
            screenshot=screenshot,
            png_frames=frame_paths,
            video=video,
            video_fps=module.CAPTURE_VIDEO_FPS,
            video_width=8,
            video_height=6,
            screenshot_png_frame_index=frame,
        ),
        "capture_runtime_provenance": {
            "algorithm": module.CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
            "digest": module._canonical_json_digest(runtime_payload),
            **runtime_payload,
        },
        "capture_source_provenance": capture_source,
        "capture_label": label,
        "force_drag": None,
        "resolved_solver_identity": {
            "executor": "World.step default",
            "solver": "public_avbd",
            "source": "scene_capture_metrics.latest.metrics",
        },
        "scene": module.SCENE_ID,
        "scene_metrics": {
            "event_count": frame,
            "latest": latest_event,
        },
        "schema_version": 1,
    }
    manifest_path = directory / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    verdict = {
        "checks": {
            "contrast": {"pass": frame in (120, 600)},
            "non_blank": {"pass": True},
        },
        "image": {
            "height": 6,
            "path": str(screenshot),
            "sha256": _sha256(screenshot),
            "width": 8,
        },
        "machine_scope": "pixel-integrity",
        "metadata": {
            "frame": str(frame),
            "scene": module.SCENE_ID,
            "view": module.CAMERA_VIEW,
        },
        "pass": True,
        "schema_version": "dart.image_verdict/v1",
        "semantic_review": {
            "performed_by_this_tool": False,
            "required": True,
        },
    }
    verdict_path = directory / "image_verdict.json"
    verdict_path.write_text(json.dumps(verdict), encoding="utf-8")
    return manifest_path, verdict_path


def _write_benchmark(module, tmp_path: Path) -> Path:
    fingerprint = int("0123456789abcdef", 16)
    configuration_fingerprint = int("fedcba9876543210", 16)
    rows = []
    for aggregate, real_time, cpu_time in (
        ("mean", 7_800_000.0, 7_790_000.0),
        ("median", 7_750_000.0, 7_740_000.0),
        ("stddev", 110_000.0, 109_000.0),
        ("cv", 0.014, 0.014),
    ):
        row = {
            "aggregate_name": aggregate,
            "aggregate_unit": "percentage" if aggregate == "cv" else "time",
            "cpu_time": cpu_time,
            "iterations": 5,
            "name": f"{module.BENCHMARK_RUN}_{aggregate}",
            "real_time": real_time,
            "repetitions": 5,
            "run_name": module.BENCHMARK_RUN,
            "run_type": "aggregate",
            "time_unit": "ns",
        }
        if aggregate in ("mean", "median"):
            row.update(
                {
                    "breakable_joints": 712.0,
                    "collision_shapes": 256.0,
                    "contact_method_sequential_impulse": 1.0,
                    "effective_scene_contract_passed": 1.0,
                    "effective_scene_mutation_audit_passed": 1.0,
                    "impacting_balls": 3.0,
                    "public_avbd_family": 1.0,
                    "resolved_rigid_body_avbd": 1.0,
                    "resolved_rigid_constraint_iterations": 1.0,
                    "resolved_rigid_contact_avbd": 1.0,
                    "resolved_rigid_pair_constraint_avbd": 1.0,
                    "rigid_constraint_iterations": 20.0,
                    "rigid_bodies": 256.0,
                    "rigid_body_joints": 712.0,
                    "rigid_avbd_alpha": 0.95,
                    "rigid_avbd_beta": 10.0,
                    "rigid_avbd_gamma": 0.99,
                    "rigid_avbd_parameter_profile_paper_2025": 1.0,
                    "runtime_contract_passed": 1.0,
                    "runtime_identity_recorded": 1.0,
                    "runtime_identity_applicable": 1.0,
                    "runtime_identity_not_applicable": 0.0,
                    "runtime_identity_public_avbd_rigid": 1.0,
                    "runtime_identity_variational_multibody": 0.0,
                    "runtime_identity_contract_passed": 1.0,
                    "scene_spec_matches_python": 1.0,
                    "solver_projection_policies_match": 1.0,
                    "scene_spec_fingerprint_hi": float(fingerprint >> 32),
                    "scene_spec_fingerprint_lo": float(fingerprint & 0xFFFFFFFF),
                    "solver_configuration_fingerprint_hi": float(
                        configuration_fingerprint >> 32
                    ),
                    "solver_configuration_fingerprint_lo": float(
                        configuration_fingerprint & 0xFFFFFFFF
                    ),
                    "trajectory_frames": 120.0,
                }
            )
        elif aggregate == "stddev":
            row.update(
                {
                    key: 0.0
                    for key in (
                        "breakable_joints",
                        "collision_shapes",
                        "contact_method_sequential_impulse",
                        "effective_scene_contract_passed",
                        "effective_scene_mutation_audit_passed",
                        "impacting_balls",
                        "public_avbd_family",
                        "resolved_rigid_body_avbd",
                        "resolved_rigid_constraint_iterations",
                        "resolved_rigid_contact_avbd",
                        "resolved_rigid_pair_constraint_avbd",
                        "rigid_constraint_iterations",
                        "rigid_bodies",
                        "rigid_body_joints",
                        "rigid_avbd_alpha",
                        "rigid_avbd_beta",
                        "rigid_avbd_gamma",
                        "rigid_avbd_parameter_profile_paper_2025",
                        "runtime_contract_passed",
                        "runtime_identity_recorded",
                        "runtime_identity_applicable",
                        "runtime_identity_not_applicable",
                        "runtime_identity_public_avbd_rigid",
                        "runtime_identity_variational_multibody",
                        "runtime_identity_contract_passed",
                        "scene_spec_matches_python",
                        "solver_projection_policies_match",
                        "scene_spec_fingerprint_hi",
                        "scene_spec_fingerprint_lo",
                        "solver_configuration_fingerprint_hi",
                        "solver_configuration_fingerprint_lo",
                        "trajectory_frames",
                    )
                }
            )
        rows.append(row)
    source = module.compute_capture_source_provenance(module.REPO_ROOT)
    executable = tmp_path / "bm_avbd_rigid_fixed_joint"
    executable.write_bytes(b"\x7fELFtest benchmark executable")
    library = tmp_path / "libdart-simulation.so"
    library.write_bytes(b"\x7fELFtest DART shared library")
    build_configuration = _build_configuration(module)
    library_identity_payload = {
        "build_configuration_digest": build_configuration["digest"],
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15.2.0",
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": source["git_head"],
        "source_provenance_digest": source["digest"],
    }
    library_identity = {
        "algorithm": module.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        **library_identity_payload,
        "digest": module._canonical_json_digest(library_identity_payload),
    }
    libraries = [
        {
            "build_identity": library_identity,
            "file": library.name,
            "path": str(library),
            "sha256": _sha256(library),
            "size_bytes": library.stat().st_size,
        }
    ]
    runtime_paths = [executable, library]
    for name in (
        "ld-linux-x86-64.so.2",
        "libbenchmark.so.1",
        "libc.so.6",
        "libm.so.6",
        "libstdc++.so.6",
    ):
        path = tmp_path / name
        path.write_bytes(b"\x7fELF" + name.encode("utf-8"))
        runtime_paths.append(path)
    runtime_images = [
        {
            "file": path.name,
            "path": str(path),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted(runtime_paths, key=str)
    ]
    runtime_inventory_payload = {
        "images": runtime_images,
        "required_roles": module._runtime_image_roles(runtime_images),
    }
    runtime_image_inventory = {
        "algorithm": module.RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **runtime_inventory_payload,
        "digest": module._canonical_json_digest(runtime_inventory_payload),
    }
    context = {
        "dart_benchmark_executable_path": str(executable),
        "dart_benchmark_source_sha256": _sha256(
            module.REPO_ROOT / module.BENCHMARK_SOURCE_PATH
        ),
        "dart_build_configuration_digest": build_configuration["digest"],
        "dart_capture_source_git_head": source["git_head"],
        "dart_capture_source_provenance_digest": source["digest"],
        "dart_cmake_build_type": "Release",
        "dart_compiler_id": "GNU",
        "dart_compiler_version": "15.2.0",
        "dart_ndebug": "1",
        "dart_optimization_enabled": "1",
        "date": "2026-08-31T12:00:00Z",
        "executable": str(executable),
        "host_name": "test-host",
        "json_schema_version": 1,
        "library_build_type": "release",
        "library_version": "test",
        "mhz_per_cpu": 5000,
        "num_cpus": 8,
    }
    build_payload = {
        "benchmark_source_sha256": context["dart_benchmark_source_sha256"],
        "build_configuration": build_configuration,
        "capture_source_git_head": source["git_head"],
        "capture_source_provenance_digest": source["digest"],
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15.2.0",
        "executable_file": executable.name,
        "executable_path": str(executable),
        "executable_sha256": _sha256(executable),
        "executable_size_bytes": executable.stat().st_size,
        "loaded_dart_libraries": libraries,
        "runtime_image_inventory": runtime_image_inventory,
        "ndebug": "1",
        "optimization_enabled": "1",
    }
    build_identity = {
        "algorithm": module.BENCHMARK_BUILD_IDENTITY_ALGORITHM,
        **build_payload,
        "digest": module._canonical_json_digest(build_payload),
    }
    host_payload = {
        "cpu_count": 8,
        "cpu_model": "test-cpu",
        "hostname": "test-host",
        "machine": "x86_64",
        "platform": "Linux-test",
        "system": "Linux",
    }
    host_token = module._canonical_json_digest(host_payload)
    gate_common = {
        "elapsed_seconds": 120.1,
        "finished_at": "2026-08-31T12:02:00Z",
        "max_normalized_load": 0.1,
        "normalized_load_limit": 0.25,
        "passed": True,
        "sample_count": 121,
        "sample_interval_seconds": 1.0,
        "started_at": "2026-08-31T12:00:00Z",
    }
    run_payload = {
        "benchmark_context_date": context["date"],
        "benchmark_policy": {
            "filter": module.FIGURE13_BENCHMARK_FILTER,
            "min_warmup_time_seconds": 1.0,
            "repetitions": 5,
            "report_aggregates_only": True,
        },
        "build_identity": build_identity,
        "host_identity": {**host_payload, "host_token": host_token},
        "host_token": host_token,
        "loader_environment": {
            "algorithm": module.LOADER_POLICY_ALGORITHM,
            "forbidden_environment_prefixes": list(module.LOADER_ENVIRONMENT_PREFIXES),
            "passed": True,
            "present_environment_variables": [],
        },
        "quiet_host": {**gate_common, "duration_seconds": 120.0},
        "run_token": "123e4567-e89b-42d3-a456-426614174000",
        "capture_ignored_paths": [],
        "capture_working_tree_clean": True,
        "watchdog": {
            **gate_common,
            "elapsed_seconds": 3.0,
            "finished_at": "2026-08-31T12:02:03Z",
            "sample_count": 4,
            "started_at": "2026-08-31T12:02:00Z",
        },
    }
    evidence = {
        "schema_version": module.FIGURE13_BENCHMARK_RUN_SCHEMA,
        **run_payload,
        "digest": module._canonical_json_digest(run_payload),
    }
    path = tmp_path / "benchmark.json"
    path.write_text(
        json.dumps(
            {
                "benchmarks": rows,
                "context": context,
                "dart_evidence_run": evidence,
            }
        ),
        encoding="utf-8",
    )
    return path


def _write_inputs(module, tmp_path: Path, monkeypatch) -> dict[str, Path]:
    capture_module = sys.modules[module.capture_artifact_provenance.__module__]

    def fake_video_probe(video, **kwargs):
        frame_count = kwargs["expected_frame_count"]
        fps = kwargs["expected_fps"]
        return {
            "codec_name": "h264",
            "content_correspondence": {
                "algorithm": (
                    capture_module.CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM
                ),
                "encoder": dict(capture_module.CAPTURE_VIDEO_ENCODER),
                "expected_reencoded_sha256": _sha256(Path(video)),
                "passed": True,
                "source_png_sequence_digest": kwargs["png_sequence_digest"],
            },
            "decoded_frame_count": frame_count,
            "duration_seconds": (
                f"{Fraction(frame_count, fps).numerator}/"
                f"{Fraction(frame_count, fps).denominator}"
            ),
            "fps": f"{fps}/1",
            "height": kwargs["expected_height"],
            "pixel_format": "yuv420p",
            "probe_algorithm": capture_module.CAPTURE_VIDEO_PROBE_ALGORITHM,
            "width": kwargs["expected_width"],
        }

    monkeypatch.setattr(capture_module, "probe_decoded_video", fake_video_probe)
    monkeypatch.setattr(
        module,
        "validate_capture_runtime_provenance",
        lambda recorded, **_kwargs: recorded,
    )
    source = module.compute_capture_source_provenance(module.REPO_ROOT)

    def fake_library_identity(
        _path,
        *,
        expected_source_digest=None,
        expected_source_git_head=None,
    ):
        payload = {
            "build_configuration_digest": _build_configuration(module)["digest"],
            "build_target": "dart-simulation",
            "cmake_build_type": "Release",
            "compiler_id": "GNU",
            "compiler_version": "15.2.0",
            "ndebug": True,
            "optimization_enabled": True,
            "source_git_head": expected_source_git_head or source["git_head"],
            "source_provenance_digest": (expected_source_digest or source["digest"]),
        }
        return {
            "algorithm": capture_module.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
            **payload,
            "digest": capture_module._canonical_json_digest(payload),
        }

    monkeypatch.setattr(module, "dart_library_build_identity", fake_library_identity)
    fixture_records = _broken_joint_records(
        broken_count=36,
        impact_region_counts=(5, 5, 5),
    )
    module.OUTCOME_ORACLE["expected_broken_joint_ids_sha256"] = (
        _broken_joint_ids_sha256(fixture_records)
    )
    impact_manifest, impact_verdict = _write_capture(module, tmp_path, 60, "impact")
    outcome_manifest, outcome_verdict = _write_capture(module, tmp_path, 120, "outcome")
    long_horizon_manifest, long_horizon_verdict = _write_capture(
        module, tmp_path, 600, "long_horizon"
    )
    paper_pdf = tmp_path / "avbd.pdf"
    paper_pdf.write_bytes(b"test AVBD paper")
    paper_figure = tmp_path / "paper-10.png"
    _write_png(paper_figure, width=10, height=12)
    monkeypatch.setattr(module, "PAPER_PDF_SHA256", _sha256(paper_pdf))
    monkeypatch.setattr(module, "PAPER_FIGURE_SHA256", _sha256(paper_figure))

    impact_screenshot = impact_manifest.parent / f"{module.SCENE_ID}_impact.png"
    outcome_screenshot = outcome_manifest.parent / f"{module.SCENE_ID}_outcome.png"
    long_horizon_screenshot = (
        long_horizon_manifest.parent / f"{module.SCENE_ID}_long_horizon.png"
    )
    long_horizon_video = (
        long_horizon_manifest.parent / f"{module.SCENE_ID}_long_horizon.mp4"
    )
    review = {
        "assessment_assertions": {
            "capture_images_assessed": True,
            "long_horizon_video_assessed": True,
            "no_contradictions_found": True,
            "paper_reference_assessed": True,
            "text_oracle_agrees": True,
            "view_reports_agree": True,
        },
        "claim_assessments": dict(
            module.SEMANTIC_CLAIM_ASSESSMENTS_BY_TERMINAL_BEHAVIOR[
                "retained_damaged_wall"
            ]
        ),
        "inspected_images": [
            {
                "file": str(impact_screenshot),
                "role": "impact_frame_60",
                "sha256": _sha256(impact_screenshot),
            },
            {
                "file": str(outcome_screenshot),
                "role": "outcome_frame_120",
                "sha256": _sha256(outcome_screenshot),
            },
            {
                "file": str(long_horizon_screenshot),
                "role": "long_horizon_frame_600",
                "sha256": _sha256(long_horizon_screenshot),
            },
            {
                "file": str(paper_figure),
                "role": "paper_figure_13_reference",
                "sha256": _sha256(paper_figure),
            },
        ],
        "inspected_videos": [
            {
                "decoded_frame_count": 600,
                "duration_seconds": "10/1",
                "file": str(long_horizon_video),
                "role": "long_horizon_video_600",
                "sha256": _sha256(long_horizon_video),
            }
        ],
        "reviewer_capabilities": {
            "image_semantic_review": True,
            "video_semantic_review": True,
        },
        "scene": module.SCENE_ID,
        "schema_version": "dart.visual_semantic_review/v1",
        "structured_observations": dict(
            module.SEMANTIC_STRUCTURED_OBSERVATIONS["retained_damaged_wall"]
        ),
        "temporal_assessment": {
            "checkpoint_sequence_agrees": True,
            "full_interval_viewed": True,
            "still_frames_only": False,
            "terminal_behavior": "retained_damaged_wall",
        },
        "verdict": "pass",
    }
    review_path = tmp_path / "visual_review.json"
    review_path.write_text(json.dumps(review), encoding="utf-8")
    return {
        "benchmark_json": _write_benchmark(module, tmp_path),
        "impact_capture_manifest": impact_manifest,
        "impact_image_verdict_json": impact_verdict,
        "outcome_capture_manifest": outcome_manifest,
        "outcome_image_verdict_json": outcome_verdict,
        "long_horizon_capture_manifest": long_horizon_manifest,
        "long_horizon_image_verdict_json": long_horizon_verdict,
        "visual_review_json": review_path,
        "paper_pdf": paper_pdf,
        "paper_figure_image": paper_figure,
    }


def test_make_packet_records_public_avbd_figure13_evidence(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_packet_module()
    inputs = _write_inputs(module, tmp_path, monkeypatch)

    packet = module.make_packet(**inputs)

    assert packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": (
            "engine World.resolved_configuration in both captures plus "
            "benchmark runtime resolved-configuration counters"
        ),
        "rigid_contact_selection": "world_solver_family",
        "rigid_contact_solver": "avbd",
        "rigid_point_joint_solver": "avbd",
        "multibody_integration_family": "none",
    }
    assert packet["visual_evidence"]["impact"]["scene_metrics"]["frame"] == 60
    assert (
        packet["visual_evidence"]["outcome"]["scene_metrics"]["outcome"]["status"]
        == "pass"
    )
    long_horizon = packet["visual_evidence"]["long_horizon"]
    assert long_horizon["scene_metrics"]["frame"] == 600
    assert long_horizon["capture"]["converted_frames"] == 600
    assert long_horizon["artifact_provenance"]["video"]["decoded_frame_count"] == 600
    assert long_horizon["image_verdict"]["metadata"]["frame"] == "600"
    assert any(
        entry["role"] == "long_horizon_frame_600"
        for entry in packet["visual_evidence"]["semantic_review"]["inspected_images"]
    )
    assert packet["benchmark"]["stability"]["repetitions"] == 5
    assert packet["benchmark"]["scene_spec_fingerprint"] == "0123456789abcdef"
    assert packet["benchmark"]["solver_configuration_fingerprint"] == "fedcba9876543210"
    stddev = next(
        row for row in packet["benchmark"]["rows"] if row["aggregate_name"] == "stddev"
    )
    assert stddev["solver_configuration_fingerprint_hi"] == 0.0
    assert stddev["runtime_identity_contract_passed"] == 0.0
    provenance_paths = {entry["path"] for entry in packet["source_provenance"]["files"]}
    assert {"scripts/_image_tools.py", "scripts/image_verdict.py"} <= (provenance_paths)
    assert (
        packet["visual_evidence"]["impact"]["image_verdict"]["image_sha256"]
        == packet["visual_evidence"]["impact"]["screenshot"]["sha256"]
    )
    impact_artifacts = packet["visual_evidence"]["impact"]["artifact_provenance"]
    assert impact_artifacts["artifact_count"] == 63
    assert impact_artifacts["png_frames"]["count"] == 60
    assert impact_artifacts["video"]["decoded_frame_count"] == 60
    assert impact_artifacts["video"]["fps"] == f"{module.CAPTURE_VIDEO_FPS}/1"
    assert packet["visual_evidence"]["impact"]["capture"]["converted_frames"] == 60
    assert packet["target"]["complete_paper_reproduction"] is False
    rendered = json.dumps(packet)
    assert str(tmp_path) in rendered
    assert "no reference ratio or speedup claim is made" in rendered


@pytest.mark.parametrize(
    "counter",
    (
        "scene_spec_fingerprint_hi",
        "solver_configuration_fingerprint_lo",
        "public_avbd_family",
        "rigid_constraint_iterations",
        "rigid_avbd_beta",
        "contact_method_sequential_impulse",
        "solver_projection_policies_match",
        "runtime_identity_contract_passed",
    ),
)
def test_benchmark_rejects_nonzero_invariant_stddev_with_equal_mean_and_median(
    tmp_path: Path,
    counter: str,
) -> None:
    module = _load_packet_module()
    benchmark_path = _write_benchmark(module, tmp_path)
    data = json.loads(benchmark_path.read_text())
    stddev = next(
        row for row in data["benchmarks"] if row["aggregate_name"] == "stddev"
    )
    # Equal mean/median words can hide symmetric per-repetition drift; the
    # nonzero dispersion is the independent fail-closed signal.
    # Fingerprint words may carry double-precision aggregation noise (tens on
    # a ~1e9 word), so inject a drift far above that bound for them.
    stddev[counter] = 1.0e6 if counter.endswith(("_fingerprint_hi", "_fingerprint_lo")) else 1.0
    benchmark_path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(
        module.AvbdPaperBreakableWallPacketError,
        match=rf"stddev {counter} must be 0",
    ):
        module._validate_benchmark(
            benchmark_path,
            expected_scene_spec_fingerprint="0123456789abcdef",
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("camera", "serialized camera"),
        ("capture_source", "capture source provenance digest"),
        ("capture_dirty_tree", "working_tree_clean must be true"),
        ("capture_ignored_paths", "ignored_paths must be empty"),
        ("capture_artifact", "capture artifact screenshot_sha256"),
        ("artifact_manifest_digest", "capture artifact provenance"),
        ("png_manifest_count", "capture artifact provenance"),
        ("video_frame_count", "capture artifact provenance"),
        ("png_frame", "capture artifact provenance"),
        ("extra_png", "final PNG sequence must contain exactly"),
        ("video_artifact", "capture artifact provenance"),
        ("stale_image_verdict", "image verdict image sha256"),
        ("long_horizon_frame", "capture requested_frames"),
        ("long_horizon_video", "capture artifact provenance"),
        ("long_horizon_review", "visual review must inspect"),
        ("semantic_claim", "claim_assessments"),
        ("semantic_still_only", "temporal_assessment"),
        ("semantic_video_path", "video file does not match capture"),
        ("semantic_contradictory_narrative", "exact structured semantic-review"),
        ("view_report_gate", "ambiguity_iou fails"),
        ("solver", "resolve public_avbd"),
        ("oracle", "passing outcome has a false threshold check"),
        ("joint_identity", "broken-joint identity digest"),
        ("joint_residual", "retained-joint linear residual exceeds"),
        ("benchmark", "public_avbd_family"),
        ("benchmark_profile", "rigid_avbd_alpha"),
        ("benchmark_missing_evidence", "missing dart_evidence_run"),
        ("benchmark_quiet_false", "quiet-host gate passed"),
        ("benchmark_quiet_zero", "benchmark quiet-host gate"),
        ("benchmark_watchdog_false", "in-run watchdog passed"),
        ("benchmark_loader_missing", "unexpected field set"),
        ("benchmark_loader_false", "loader environment"),
        (
            "benchmark_runtime_missing_benchmark",
            "missing required roles",
        ),
        (
            "benchmark_runtime_benchmark_substitution",
            "benchmark runtime image",
        ),
        (
            "benchmark_runtime_omitted_dart_identity",
            "exactly cover every mapped libdart runtime image",
        ),
        ("benchmark_warmup_zero", "benchmark evidence policy"),
        ("benchmark_debug", "compiled CMake build type"),
        (
            "benchmark_build_config_missing",
            "compiled build configuration digest",
        ),
        (
            "benchmark_library_build_config_mismatch",
            "loaded DART library 0",
        ),
        ("benchmark_bad_run_token", "run_token must be a canonical UUIDv4"),
        ("benchmark_bad_host_token", "host token"),
        ("benchmark_executable_substitution", "benchmark runtime image"),
        ("benchmark_library_substitution", "benchmark runtime image"),
        ("benchmark_library_omission", "inventory loaded DART shared libraries"),
        (
            "benchmark_configuration",
            "mean/median solver_configuration_fingerprint_lo counters must match",
        ),
        (
            "benchmark_source",
            "benchmark compiled capture source provenance digest",
        ),
        ("events_empty", "expected 60 scene metric events"),
        ("events_unrelated", "event must be"),
        ("events_count", "expected 60 scene metric events"),
        ("events_order", "frames must be exactly"),
        ("events_inner_frame", "outcome frame"),
        ("events_inner_time", "outcome world_time"),
        ("events_intermediate_solver", "full-stream scene/solver/oracle invariants"),
        (
            "events_intermediate_fingerprint",
            "full-stream scene/solver/oracle invariants",
        ),
        ("events_intermediate_effective_scene", "effective scene contract"),
        ("events_intermediate_view_report", "ambiguity_iou fails"),
        ("events_checkpoint_threshold", "evaluated outcome state"),
        ("events_prefix_mismatch", "exact scene-metric event prefix"),
        ("events_manifest_mismatch", "manifest latest scene metric event"),
        ("review", "sha256"),
        ("paper", "paper PDF sha256"),
    ],
)
def test_make_packet_fails_closed_on_mismatched_evidence(
    tmp_path: Path,
    monkeypatch,
    mutation: str,
    message: str,
) -> None:
    module = _load_packet_module()
    inputs = _write_inputs(module, tmp_path, monkeypatch)

    if mutation == "camera":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        del data["camera"]
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "capture_source":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_source_provenance"]["digest"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "capture_dirty_tree":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_source_provenance"]["working_tree_clean"] = False
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "capture_ignored_paths":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_source_provenance"]["ignored_paths"] = ["dart/local.hpp"]
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "capture_artifact":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_artifact_provenance"]["screenshot_sha256"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "artifact_manifest_digest":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_artifact_provenance"]["digest"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "png_manifest_count":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_artifact_provenance"]["png_frames"]["count"] = 59
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "video_frame_count":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_artifact_provenance"]["video"]["decoded_frame_count"] = 59
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "png_frame":
        manifest = json.loads(inputs["impact_capture_manifest"].read_text())
        frame_path = next(Path(manifest["artifacts"]["frames"]).glob("*.png"))
        frame_path.write_bytes(frame_path.read_bytes() + b"png-mutation")
    elif mutation == "extra_png":
        manifest = json.loads(inputs["impact_capture_manifest"].read_text())
        _write_png(Path(manifest["artifacts"]["frames"]) / "unbound.PNG")
    elif mutation == "video_artifact":
        manifest = json.loads(inputs["impact_capture_manifest"].read_text())
        video_path = Path(manifest["artifacts"]["video"])
        video_path.write_bytes(video_path.read_bytes() + b"video-mutation")
    elif mutation == "stale_image_verdict":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        screenshot = Path(data["artifacts"]["screenshot"])
        terminal_frame = Path(data["artifacts"]["frames"]) / "frame_000060.png"
        _write_png(screenshot, red_offset=1)
        _write_png(terminal_frame, red_offset=1)
        updated_hash = _sha256(screenshot)
        data["capture_artifact_provenance"] = module.capture_artifact_provenance(
            scene_metrics_events=Path(data["artifacts"]["scene_metrics_events"]),
            screenshot=screenshot,
            png_frames=sorted(Path(data["artifacts"]["frames"]).glob("*.png")),
            video=Path(data["artifacts"]["video"]),
            video_fps=module.CAPTURE_VIDEO_FPS,
            video_width=8,
            video_height=6,
            screenshot_png_frame_index=60,
        )
        path.write_text(json.dumps(data), encoding="utf-8")

        review_path = inputs["visual_review_json"]
        review = json.loads(review_path.read_text())
        impact = next(
            entry
            for entry in review["inspected_images"]
            if entry["role"] == "impact_frame_60"
        )
        impact["sha256"] = updated_hash
        review_path.write_text(json.dumps(review), encoding="utf-8")
    elif mutation == "long_horizon_frame":
        path = inputs["long_horizon_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture"]["requested_frames"] = 599
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "long_horizon_video":
        manifest = json.loads(inputs["long_horizon_capture_manifest"].read_text())
        video_path = Path(manifest["artifacts"]["video"])
        video_path.write_bytes(video_path.read_bytes() + b"long-horizon-mutation")
    elif mutation == "long_horizon_review":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["inspected_images"] = [
            entry
            for entry in data["inspected_images"]
            if entry["role"] != "long_horizon_frame_600"
        ]
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "semantic_claim":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["claim_assessments"]["cuda_parity"] = "supported"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "semantic_still_only":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["temporal_assessment"]["full_interval_viewed"] = False
        data["temporal_assessment"]["still_frames_only"] = True
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "semantic_video_path":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["inspected_videos"][0]["file"] = str(
            tmp_path / "unrelated" / Path(data["inspected_videos"][0]["file"]).name
        )
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "semantic_contradictory_narrative":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["narrative"] = "I skipped the video; ViewReports contradict evidence"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "view_report_gate":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["scene_metrics"]["latest"]["metrics"]["view_report"]["metrics"][
            "ambiguity_iou"
        ] = 0.75
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line) for line in events_path.read_text().splitlines() if line
        ]
        events[-1]["metrics"]["view_report"]["metrics"]["ambiguity_iou"] = 0.75
        events_path.write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8",
        )
    elif mutation == "solver":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["resolved_solver_identity"]["solver"] = "sequential_impulse"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "oracle":
        path = inputs["outcome_capture_manifest"]
        data = json.loads(path.read_text())
        data["scene_metrics"]["latest"]["metrics"]["outcome"]["threshold_checks"][
            "fracture_count_bounded"
        ] = False
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line) for line in events_path.read_text().splitlines() if line
        ]
        events[-1]["metrics"]["outcome"]["threshold_checks"][
            "fracture_count_bounded"
        ] = False
        events_path.write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8",
        )
    elif mutation == "benchmark":
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        data["benchmarks"][0]["public_avbd_family"] = 0.0
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "benchmark_profile":
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        data["benchmarks"][0]["rigid_avbd_alpha"] = 0.0
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation.startswith("benchmark_") and mutation not in {
        "benchmark_configuration",
        "benchmark_source",
    }:
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        evidence = data.get("dart_evidence_run")
        if mutation == "benchmark_missing_evidence":
            del data["dart_evidence_run"]
        elif mutation == "benchmark_quiet_false":
            evidence["quiet_host"]["passed"] = False
        elif mutation == "benchmark_quiet_zero":
            evidence["quiet_host"]["duration_seconds"] = 0.0
            evidence["quiet_host"]["elapsed_seconds"] = 0.0
            evidence["quiet_host"]["sample_count"] = 1
        elif mutation == "benchmark_watchdog_false":
            evidence["watchdog"]["passed"] = False
        elif mutation == "benchmark_loader_missing":
            del evidence["loader_environment"]
        elif mutation == "benchmark_loader_false":
            evidence["loader_environment"]["passed"] = False
        elif mutation == "benchmark_runtime_missing_benchmark":
            inventory = evidence["build_identity"]["runtime_image_inventory"]
            inventory["images"] = [
                image
                for image in inventory["images"]
                if image["file"] != "libbenchmark.so.1"
            ]
            inventory_payload = {
                "images": inventory["images"],
                "required_roles": inventory["required_roles"],
            }
            inventory["digest"] = module._canonical_json_digest(inventory_payload)
        elif mutation == "benchmark_runtime_benchmark_substitution":
            inventory = evidence["build_identity"]["runtime_image_inventory"]
            Path(inventory["required_roles"]["google_benchmark"]).write_bytes(
                b"\x7fELFsubstituted benchmark library"
            )
        elif mutation == "benchmark_runtime_omitted_dart_identity":
            inventory = evidence["build_identity"]["runtime_image_inventory"]
            extra_library = tmp_path / "libdart-collision.so"
            extra_library.write_bytes(b"\x7fELFunbound DART image")
            inventory["images"].append(
                {
                    "file": extra_library.name,
                    "path": str(extra_library),
                    "sha256": _sha256(extra_library),
                    "size_bytes": extra_library.stat().st_size,
                }
            )
            inventory["images"].sort(key=lambda image: image["path"])
            inventory_payload = {
                "images": inventory["images"],
                "required_roles": inventory["required_roles"],
            }
            inventory["digest"] = module._canonical_json_digest(inventory_payload)
        elif mutation == "benchmark_warmup_zero":
            evidence["benchmark_policy"]["min_warmup_time_seconds"] = 0.0
        elif mutation == "benchmark_debug":
            data["context"]["dart_cmake_build_type"] = "Debug"
            evidence["build_identity"]["cmake_build_type"] = "Debug"
        elif mutation == "benchmark_build_config_missing":
            del data["context"]["dart_build_configuration_digest"]
        elif mutation == "benchmark_library_build_config_mismatch":
            library_identity = evidence["build_identity"]["loaded_dart_libraries"][0][
                "build_identity"
            ]
            library_identity["build_configuration_digest"] = "9" * 64
            library_payload = {
                key: value
                for key, value in library_identity.items()
                if key not in {"algorithm", "digest"}
            }
            library_identity["digest"] = module._canonical_json_digest(library_payload)
        elif mutation == "benchmark_bad_run_token":
            evidence["run_token"] = "not-a-uuid"
        elif mutation == "benchmark_bad_host_token":
            evidence["host_token"] = "0" * 64
        elif mutation == "benchmark_executable_substitution":
            Path(data["context"]["dart_benchmark_executable_path"]).write_bytes(
                b"\x7fELFsubstituted benchmark executable"
            )
        elif mutation == "benchmark_library_substitution":
            Path(
                evidence["build_identity"]["loaded_dart_libraries"][0]["path"]
            ).write_bytes(b"\x7fELFsubstituted DART shared library")
        elif mutation == "benchmark_library_omission":
            evidence["build_identity"]["loaded_dart_libraries"] = []
        if mutation not in {
            "benchmark_missing_evidence",
            "benchmark_executable_substitution",
            "benchmark_library_substitution",
        }:
            _reseal_benchmark_evidence(module, data)
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "benchmark_configuration":
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        data["benchmarks"][1]["solver_configuration_fingerprint_lo"] += 1.0
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "benchmark_source":
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        data["context"]["dart_capture_source_provenance_digest"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation in {"joint_identity", "joint_residual"}:
        path = inputs["outcome_capture_manifest"]
        data = json.loads(path.read_text())
        latest_outcome = data["scene_metrics"]["latest"]["metrics"]["outcome"]
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line) for line in events_path.read_text().splitlines() if line
        ]
        event_outcome = events[-1]["metrics"]["outcome"]
        if mutation == "joint_identity":
            latest_outcome["broken_joint_records"][0]["id"] = "mutated"
            event_outcome["broken_joint_records"][0]["id"] = "mutated"
        else:
            latest_outcome["maximum_unbroken_joint_linear_residual"] = 0.1
            event_outcome["maximum_unbroken_joint_linear_residual"] = 0.1
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path.write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8",
        )
    elif mutation.startswith("events_"):
        capture_key = (
            "long_horizon_capture_manifest"
            if mutation == "events_checkpoint_threshold"
            else (
                "outcome_capture_manifest"
                if mutation == "events_prefix_mismatch"
                else "impact_capture_manifest"
            )
        )
        manifest_path = inputs[capture_key]
        manifest = json.loads(manifest_path.read_text())
        events_path = Path(manifest["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line) for line in events_path.read_text().splitlines() if line
        ]
        if mutation == "events_empty":
            events = []
        elif mutation == "events_unrelated":
            events[0]["event"] = "unrelated"
        elif mutation == "events_count":
            events.pop()
        elif mutation == "events_order":
            events[0], events[1] = events[1], events[0]
        elif mutation == "events_inner_frame":
            events[0]["metrics"]["outcome"]["frame"] = 2
        elif mutation == "events_inner_time":
            events[0]["metrics"]["outcome"]["world_time"] = 1.0
        elif mutation == "events_intermediate_solver":
            events[29]["metrics"]["solver"] = "sequential_impulse"
        elif mutation == "events_intermediate_fingerprint":
            events[29]["metrics"]["scene_spec_fingerprint"] = "0" * 16
        elif mutation == "events_intermediate_effective_scene":
            events[29]["metrics"]["effective_scene_contract_passed"] = False
        elif mutation == "events_intermediate_view_report":
            events[29]["metrics"]["view_report"]["metrics"]["ambiguity_iou"] = 0.9
        elif mutation == "events_checkpoint_threshold":
            events[119]["metrics"]["outcome"]["thresholds_pass"] = False
            events[119]["metrics"]["outcome"]["status"] = "pre-evaluation"
        elif mutation == "events_prefix_mismatch":
            events[0]["metrics"]["outcome"]["ball_positions"][0][0] = 0.25
        elif mutation == "events_manifest_mismatch":
            manifest["scene_metrics"]["latest"]["metrics"]["breakable_joints"] = 0
            inputs["impact_capture_manifest"].write_text(
                json.dumps(manifest),
                encoding="utf-8",
            )
        events_path.write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8",
        )
        if mutation == "events_prefix_mismatch":
            manifest["capture_artifact_provenance"] = (
                module.capture_artifact_provenance(
                    scene_metrics_events=events_path,
                    screenshot=Path(manifest["artifacts"]["screenshot"]),
                    png_frames=sorted(
                        Path(manifest["artifacts"]["frames"]).glob("*.png")
                    ),
                    video=Path(manifest["artifacts"]["video"]),
                    video_fps=module.CAPTURE_VIDEO_FPS,
                    video_width=8,
                    video_height=6,
                    screenshot_png_frame_index=120,
                )
            )
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    elif mutation == "review":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["inspected_images"][0]["sha256"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "paper":
        inputs["paper_pdf"].write_bytes(b"not the validated paper")

    with pytest.raises(module.AvbdPaperBreakableWallPacketError, match=message):
        module.make_packet(**inputs)


def test_main_writes_validated_packet(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_packet_module()
    inputs = _write_inputs(module, tmp_path, monkeypatch)
    output = tmp_path / "packet.json"
    args = []
    for name, path in inputs.items():
        args.extend(("--" + name.replace("_", "-"), str(path)))
    args.extend(("--output", str(output)))

    assert module.main(args) == 0
    assert json.loads(output.read_text())["packet"] == "avbd_paper_breakable_wall"
