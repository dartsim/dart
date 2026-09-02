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
SCRIPT = ROOT / "scripts" / "write_avbd_paper_vbd_comparison_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_paper_vbd_comparison_packet",
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
        + chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0),
        )
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _build_configuration(module) -> dict[str, object]:
    shared = module.shared
    values = {key: "<UNDEFINED>" for key in shared.BUILD_CONFIGURATION_KEYS}
    for definition in shared.EVIDENCE_CMAKE_DEFINITIONS:
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
        [f"algorithm={shared.BUILD_CONFIGURATION_ALGORITHM}\n"]
        + [f"{key}={values[key]}\n" for key in shared.BUILD_CONFIGURATION_KEYS]
    )
    return {
        "algorithm": shared.BUILD_CONFIGURATION_ALGORITHM,
        "digest": sha256(record.encode("utf-8")).hexdigest(),
        "values": values,
    }


def _source_provenance(module) -> dict[str, Any]:
    combined = sha256()
    files = []
    for relative in module.shared.SOURCE_PATHS:
        path = module.shared.REPO_ROOT / relative
        payload = path.read_bytes()
        encoded_path = relative.as_posix().encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
        files.append(
            {
                "path": relative.as_posix(),
                "sha256": sha256(payload).hexdigest(),
            }
        )
    return {
        "algorithm": "sha256-length-prefixed-path-and-content-v1",
        "digest": combined.hexdigest(),
        "files": files,
    }


def _empty_joint_evidence(
    *,
    maximum_angular: float,
    maximum_linear: float,
    rms_angular: float,
    rms_linear: float,
) -> dict[str, Any]:
    return {
        "broken_joint_identity_count": 0,
        "broken_joint_ids_sha256": sha256().hexdigest(),
        "broken_joint_impact_region_counts": [0, 0, 0],
        "broken_joint_records": [],
        "broken_joints_outside_impact_regions": 0,
        "joint_residuals_finite": True,
        "maximum_outside_impact_unbroken_joint_angular_residual_radians": (
            maximum_angular
        ),
        "maximum_outside_impact_unbroken_joint_linear_residual": maximum_linear,
        "maximum_unbroken_joint_angular_residual_radians": maximum_angular,
        "maximum_unbroken_joint_linear_residual": maximum_linear,
        "outside_impact_unbroken_joint_residual_count": 484,
        "rms_outside_impact_unbroken_joint_angular_residual_radians": (rms_angular),
        "rms_outside_impact_unbroken_joint_linear_residual": rms_linear,
        "rms_unbroken_joint_angular_residual_radians": rms_angular,
        "rms_unbroken_joint_linear_residual": rms_linear,
        "unbroken_joint_residual_count": 712,
    }


def _avbd_joint_records() -> list[dict[str, Any]]:
    impact_indices = [0] * 18 + [1] * 44 + [2] * 13
    records = []
    for index in range(154):
        within_region = index < len(impact_indices)
        records.append(
            {
                "angular_residual_radians": 0.01 + index * 0.0001,
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
                "linear_residual": 0.001 + index * 0.000001,
                "nearest_impact_distance": 0.5 if within_region else 2.0,
                "nearest_impact_index": (
                    impact_indices[index] if within_region else index % 3
                ),
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


def _joint_ids_sha256(records: list[dict[str, Any]]) -> str:
    digest = sha256()
    for joint_id in sorted(record["id"] for record in records):
        encoded = joint_id.encode("utf-8")
        digest.update(struct.pack("<Q", len(encoded)))
        digest.update(encoded)
    return digest.hexdigest()


def _linked_avbd_outcome(module, frame: int) -> dict[str, Any]:
    records = _avbd_joint_records()
    expected = module.shared.EXPECTED_OUTCOMES.get(
        frame, module.shared.EXPECTED_OUTCOMES[120]
    )
    return dict(expected) | {
        "broken_joint_identity_count": 154,
        "broken_joint_ids_sha256": _joint_ids_sha256(records),
        "broken_joint_impact_region_counts": [18, 44, 13],
        "broken_joint_records": records,
        "broken_joints_outside_impact_regions": 79,
        "checkpoint": "outcome",
        "contact_count": 12,
        "frame": frame,
        "joint_residuals_finite": True,
        "last_step_iterations": 20,
        "max_brick_displacement": 3.0,
        "maximum_outside_impact_unbroken_joint_angular_residual_radians": (0.0003),
        "maximum_outside_impact_unbroken_joint_linear_residual": 0.001,
        "maximum_unbroken_joint_angular_residual_radians": 0.0003,
        "maximum_unbroken_joint_linear_residual": 0.001,
        "outside_impact_unbroken_joint_residual_count": 405,
        "rms_outside_impact_unbroken_joint_angular_residual_radians": 0.0001,
        "rms_outside_impact_unbroken_joint_linear_residual": 0.0005,
        "rms_unbroken_joint_angular_residual_radians": 0.0001,
        "rms_unbroken_joint_linear_residual": 0.0005,
        "unbroken_joint_residual_count": 558,
        "world_time": frame / 60.0,
    }


def _outcome(module, frame: int) -> dict[str, Any]:
    generic = {
        "ball_positions": [[0.0, 1.0, 2.0]] * 3,
        "ball_velocities": [[3.0, 4.0, 5.0]] * 3,
        "bent_brick_count": 0,
        "broken_joints": 0,
        "checkpoint": "bend",
        "contact_count": 0,
        "evaluated": False,
        "frame": frame,
        "impact_band_displaced_counts": [0, 0, 0],
        "last_step_iterations": 20,
        "max_brick_displacement": 0.0,
        "maximum_wall_normal_displacement": 0.0,
        "outside_brick_count": 181,
        "outside_retained_fraction": 1.0,
        "rms_wall_normal_displacement": 0.0,
        "status": "pre-evaluation" if frame < 18 else "between-checkpoints",
        "threshold_checks": {"finite_state": True},
        "thresholds_pass": False,
        "total_retained_fraction": 1.0,
        "unbroken_joints": 712,
        "world_time": frame / 60.0,
    } | _empty_joint_evidence(
        maximum_angular=0.0,
        maximum_linear=0.0,
        rms_angular=0.0,
        rms_linear=0.0,
    )
    if frame == 18:
        return (
            generic
            | {
                "bent_brick_count": 120,
                "evaluated": True,
                "max_brick_displacement": 0.13266478096099812,
                "maximum_wall_normal_displacement": 0.13266478096099812,
                "rms_wall_normal_displacement": 0.06264039944471685,
                "status": "pass",
                "threshold_checks": dict(module.CHECKPOINTS[18]["threshold_checks"]),
                "thresholds_pass": True,
            }
            | _empty_joint_evidence(
                maximum_angular=0.013,
                maximum_linear=0.016,
                rms_angular=0.003,
                rms_linear=0.004,
            )
        )
    if frame >= 120:
        return (
            generic
            | {
                "checkpoint": "retention",
                "evaluated": True,
                "max_brick_displacement": 0.0012,
                "maximum_wall_normal_displacement": 0.00086,
                "rms_wall_normal_displacement": 0.00047,
                "status": "pass",
                "threshold_checks": dict(
                    module.CHECKPOINTS[600 if frame >= 600 else 120]["threshold_checks"]
                ),
                "thresholds_pass": True,
            }
            | _empty_joint_evidence(
                maximum_angular=0.0013,
                maximum_linear=0.011,
                rms_angular=0.00012,
                rms_linear=0.0033,
            )
        )
    return generic


def _metrics(module, frame: int) -> dict[str, Any]:
    return {
        "ball_count": 3,
        "break_force": module.BREAK_FORCE,
        "breakable_joints": 712,
        "brick_count": 252,
        "collision_shapes": 256,
        "effective_scene_contract_passed": True,
        "executor": "World.step default",
        "outcome": _outcome(module, frame),
        "outcome_oracle": dict(module.OUTCOME_ORACLE),
        "paper_locator": module.PAPER_LOCATOR,
        "rigid_bodies": 256,
        "rigid_body_solver": "VBD",
        "rigid_constraint_options": {"iterations": 20},
        "resolved_configuration": [
            {
                "domain": "rigid-body",
                "reason": "as requested",
                "requested": "vbd",
                "resolved": "vbd",
            },
            {
                "domain": "rigid-contact",
                "reason": "as requested",
                "requested": "vbd",
                "resolved": "vbd",
            },
            {
                "domain": "rigid-pair-constraint",
                "reason": "as requested",
                "requested": "vbd",
                "resolved": "vbd",
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
        "solver": "public_vbd",
        "time_step_ms": 1000.0 / 60.0,
        "view_report": {
            "camera": {
                "azimuth": module.shared.CAMERA_AZIMUTH,
                "distance": module.shared.CAMERA_DISTANCE,
                "elevation": module.shared.CAMERA_ELEVATION,
                "target": list(module.shared.CAMERA_TARGET),
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


def _write_capture(
    module,
    tmp_path: Path,
    frame: int,
    label: str,
) -> tuple[Path, Path]:
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
    metrics_log.write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )
    capture_source = module.shared.compute_capture_source_provenance(
        module.shared.REPO_ROOT
    )
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
                    "algorithm": module.shared.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
                    **runtime_library_identity_payload,
                    "digest": module.shared._canonical_json_digest(
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
            "azimuth": module.math.degrees(module.shared.CAMERA_AZIMUTH),
            "distance": module.shared.CAMERA_DISTANCE,
            "elevation": module.math.degrees(module.shared.CAMERA_ELEVATION),
            "target": list(module.shared.CAMERA_TARGET),
            "view": module.shared.CAMERA_PRESET,
        },
        "capture": {
            "converted_frames": frame,
            "height": 6,
            "requested_frames": frame,
            "width": 8,
        },
        "capture_artifact_provenance": (
            module.shared.capture_artifact_provenance(
                scene_metrics_events=metrics_log,
                screenshot=screenshot,
                png_frames=frame_paths,
                video=video,
                video_fps=module.shared.CAPTURE_VIDEO_FPS,
                video_width=8,
                video_height=6,
                screenshot_png_frame_index=frame,
            )
        ),
        "capture_runtime_provenance": {
            "algorithm": module.shared.CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
            "digest": module.shared._canonical_json_digest(runtime_payload),
            **runtime_payload,
        },
        "capture_source_provenance": capture_source,
        "capture_label": label,
        "force_drag": None,
        "resolved_solver_identity": {
            "executor": "World.step default",
            "solver": "public_vbd",
            "source": "scene_capture_metrics.latest.metrics",
        },
        "scene": module.SCENE_ID,
        "scene_metrics": {
            "event_count": frame,
            "latest": events[-1],
        },
        "schema_version": 1,
    }
    manifest_path = directory / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    verdict = {
        "checks": {"non_blank": {"pass": True}},
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
    configuration_fingerprints = {
        "avbd": int("fedcba9876543210", 16),
        "vbd": int("1020304050607080", 16),
    }
    rows = []
    for solver_key in ("avbd", "vbd"):
        run = module.BENCHMARK_RUNS[solver_key]
        scale = 1.0 if solver_key == "avbd" else 1.25
        for aggregate, real_time, cpu_time in (
            ("mean", 7_800_000.0 * scale, 7_790_000.0 * scale),
            ("median", 7_750_000.0 * scale, 7_740_000.0 * scale),
            ("stddev", 110_000.0, 109_000.0),
            ("cv", 0.014, 0.014),
        ):
            row = {
                "aggregate_name": aggregate,
                "aggregate_unit": ("percentage" if aggregate == "cv" else "time"),
                "cpu_time": cpu_time,
                "iterations": 5,
                "name": f"{run}_{aggregate}",
                "real_time": real_time,
                "repetitions": 5,
                "run_name": run,
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
                        f"public_{solver_key}_family": 1.0,
                        f"resolved_rigid_body_{solver_key}": 1.0,
                        "resolved_rigid_constraint_iterations": 1.0,
                        f"resolved_rigid_contact_{solver_key}": 1.0,
                        (f"resolved_rigid_pair_constraint_{solver_key}"): 1.0,
                        "rigid_constraint_iterations": 20.0,
                        "rigid_bodies": 256.0,
                        "rigid_body_joints": 712.0,
                        "rigid_avbd_alpha": 0.95 if solver_key == "avbd" else 0.0,
                        "rigid_avbd_beta": 10.0 if solver_key == "avbd" else 0.0,
                        "rigid_avbd_gamma": 0.99 if solver_key == "avbd" else 0.0,
                        "rigid_avbd_parameter_profile_paper_2025": (
                            1.0 if solver_key == "avbd" else 0.0
                        ),
                        "runtime_contract_passed": 1.0,
                        "runtime_identity_recorded": 1.0,
                        "runtime_identity_applicable": 1.0,
                        "runtime_identity_not_applicable": 0.0,
                        "runtime_identity_public_avbd_rigid": (
                            1.0 if solver_key == "avbd" else 0.0
                        ),
                        "runtime_identity_variational_multibody": 0.0,
                        "runtime_identity_contract_passed": 1.0,
                        "scene_spec_matches_python": 1.0,
                        "solver_projection_policies_match": 1.0,
                        "scene_spec_fingerprint_hi": float(fingerprint >> 32),
                        "scene_spec_fingerprint_lo": float(fingerprint & 0xFFFFFFFF),
                        "solver_configuration_fingerprint_hi": float(
                            configuration_fingerprints[solver_key] >> 32
                        ),
                        "solver_configuration_fingerprint_lo": float(
                            configuration_fingerprints[solver_key] & 0xFFFFFFFF
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
                            f"public_{solver_key}_family",
                            f"resolved_rigid_body_{solver_key}",
                            "resolved_rigid_constraint_iterations",
                            f"resolved_rigid_contact_{solver_key}",
                            f"resolved_rigid_pair_constraint_{solver_key}",
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
    source = module.shared.compute_capture_source_provenance(module.shared.REPO_ROOT)
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
        "algorithm": module.shared.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        **library_identity_payload,
        "digest": module.shared._canonical_json_digest(library_identity_payload),
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
        "required_roles": module.shared._runtime_image_roles(runtime_images),
    }
    runtime_image_inventory = {
        "algorithm": module.shared.RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **runtime_inventory_payload,
        "digest": module.shared._canonical_json_digest(runtime_inventory_payload),
    }
    context = {
        "dart_benchmark_executable_path": str(executable),
        "dart_benchmark_source_sha256": _sha256(
            module.shared.REPO_ROOT / module.shared.BENCHMARK_SOURCE_PATH
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
        "algorithm": module.shared.BENCHMARK_BUILD_IDENTITY_ALGORITHM,
        **build_payload,
        "digest": module.shared._canonical_json_digest(build_payload),
    }
    host_payload = {
        "cpu_count": 8,
        "cpu_model": "test-cpu",
        "hostname": "test-host",
        "machine": "x86_64",
        "platform": "Linux-test",
        "system": "Linux",
    }
    host_token = module.shared._canonical_json_digest(host_payload)
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
            "filter": module.shared.FIGURE13_BENCHMARK_FILTER,
            "min_warmup_time_seconds": 1.0,
            "repetitions": 5,
            "report_aggregates_only": True,
        },
        "build_identity": build_identity,
        "host_identity": {**host_payload, "host_token": host_token},
        "host_token": host_token,
        "loader_environment": {
            "algorithm": module.shared.LOADER_POLICY_ALGORITHM,
            "forbidden_environment_prefixes": list(
                module.shared.LOADER_ENVIRONMENT_PREFIXES
            ),
            "passed": True,
            "present_environment_variables": [],
        },
        "quiet_host": {**gate_common, "duration_seconds": 120.0},
        "run_token": "123e4567-e89b-42d3-a456-426614174000",
        "watchdog": {
            **gate_common,
            "elapsed_seconds": 3.0,
            "finished_at": "2026-08-31T12:02:03Z",
            "sample_count": 4,
            "started_at": "2026-08-31T12:02:00Z",
        },
    }
    evidence = {
        "schema_version": module.shared.FIGURE13_BENCHMARK_RUN_SCHEMA,
        **run_payload,
        "digest": module.shared._canonical_json_digest(run_payload),
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
    capture_module = sys.modules[module.shared.capture_artifact_provenance.__module__]

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
        module.shared,
        "validate_capture_runtime_provenance",
        lambda recorded, **_kwargs: recorded,
    )
    source = module.shared.compute_capture_source_provenance(module.shared.REPO_ROOT)

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

    monkeypatch.setattr(
        module.shared, "dart_library_build_identity", fake_library_identity
    )
    module.shared.OUTCOME_ORACLE["expected_broken_joint_ids_sha256"] = (
        _joint_ids_sha256(_avbd_joint_records())
    )
    bend_manifest, bend_verdict = _write_capture(
        module,
        tmp_path,
        18,
        "bend",
    )
    retention_manifest, retention_verdict = _write_capture(
        module,
        tmp_path,
        120,
        "retention",
    )
    long_horizon_manifest, long_horizon_verdict = _write_capture(
        module,
        tmp_path,
        600,
        "long_horizon",
    )
    benchmark = _write_benchmark(module, tmp_path)

    paper_pdf = tmp_path / "avbd.pdf"
    paper_pdf.write_bytes(b"test AVBD paper")
    paper_figure = tmp_path / "paper-10.png"
    _write_png(paper_figure, width=10, height=12)
    monkeypatch.setattr(
        module.shared,
        "PAPER_PDF_SHA256",
        _sha256(paper_pdf),
    )
    monkeypatch.setattr(
        module.shared,
        "PAPER_FIGURE_SHA256",
        _sha256(paper_figure),
    )

    bend_screenshot = bend_manifest.parent / f"{module.SCENE_ID}_bend.png"
    retention_screenshot = (
        retention_manifest.parent / f"{module.SCENE_ID}_retention.png"
    )
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
        "claim_assessments": dict(module.shared.SEMANTIC_CLAIM_ASSESSMENTS),
        "inspected_images": [
            {
                "file": str(bend_screenshot),
                "role": "bend_frame_18",
                "sha256": _sha256(bend_screenshot),
            },
            {
                "file": str(retention_screenshot),
                "role": "retention_frame_120",
                "sha256": _sha256(retention_screenshot),
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
            module.shared.SEMANTIC_STRUCTURED_OBSERVATIONS["bent_retained_wall"]
        ),
        "temporal_assessment": {
            "checkpoint_sequence_agrees": True,
            "full_interval_viewed": True,
            "still_frames_only": False,
            "terminal_behavior": "bent_retained_wall",
        },
        "verdict": "pass",
    }
    review_path = tmp_path / "visual_review.json"
    review_path.write_text(json.dumps(review), encoding="utf-8")

    impact_hash = "1" * 64
    outcome_hash = "2" * 64
    long_horizon_hash = "6" * 64
    semantic_hash = "3" * 64
    paper_figure_hash = _sha256(paper_figure)
    avbd_visual = {
        "impact": {
            "image_verdict": {
                "image_sha256": impact_hash,
                "pass": True,
                "sha256": "4" * 64,
            },
            "scene_metrics": {
                "frame": 60,
                "outcome": _linked_avbd_outcome(module, 60),
                "outcome_oracle": dict(module.shared.OUTCOME_ORACLE),
                "scene_spec_fingerprint": "0123456789abcdef",
            },
            "screenshot": {"file": "impact.png", "sha256": impact_hash},
        },
        "outcome": {
            "image_verdict": {
                "image_sha256": outcome_hash,
                "pass": True,
                "sha256": "5" * 64,
            },
            "scene_metrics": {
                "frame": 120,
                "outcome": _linked_avbd_outcome(module, 120),
                "outcome_oracle": dict(module.shared.OUTCOME_ORACLE),
                "scene_spec_fingerprint": "0123456789abcdef",
            },
            "screenshot": {"file": "outcome.png", "sha256": outcome_hash},
        },
        "long_horizon": {
            "image_verdict": {
                "image_sha256": long_horizon_hash,
                "pass": True,
                "sha256": "7" * 64,
            },
            "scene_metrics": {
                "frame": 600,
                "outcome": _linked_avbd_outcome(module, 600),
                "outcome_oracle": dict(module.shared.OUTCOME_ORACLE),
                "scene_spec_fingerprint": "0123456789abcdef",
            },
            "screenshot": {
                "file": "long_horizon.png",
                "sha256": long_horizon_hash,
            },
        },
        "semantic_review": {
            "file": "review.json",
            "inspected_images": [
                {
                    "file": "impact.png",
                    "role": "impact_frame_60",
                    "sha256": impact_hash,
                },
                {
                    "file": "outcome.png",
                    "role": "outcome_frame_120",
                    "sha256": outcome_hash,
                },
                {
                    "file": "long_horizon.png",
                    "role": "long_horizon_frame_600",
                    "sha256": long_horizon_hash,
                },
                {
                    "file": paper_figure.name,
                    "role": "paper_figure_13_reference",
                    "sha256": paper_figure_hash,
                },
            ],
            "sha256": semantic_hash,
            "verdict": "pass",
        },
    }
    avbd_packet = {
        "benchmark": {
            "json_sha256": _sha256(benchmark),
            "run_evidence": json.loads(benchmark.read_text())["dart_evidence_run"],
            "scene_spec_fingerprint": "0123456789abcdef",
        },
        "correctness": {
            "determinism_test": (
                "test_avbd_paper_breakable_wall_outcome_is_deterministic"
            ),
            "outcome_test": (
                "test_avbd_paper_breakable_wall_matches_figure13_contract"
            ),
        },
        "packet": "avbd_paper_breakable_wall",
        "paper_reference": {
            "figure": {
                "file": paper_figure.name,
                "sha256": paper_figure_hash,
            }
        },
        "resolved_solver_identity": {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "test",
            "rigid_contact_selection": "world_solver_family",
            "rigid_contact_solver": "avbd",
            "rigid_point_joint_solver": "avbd",
        },
        "scene": module.AVBD_SCENE_ID,
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "source_provenance": _source_provenance(module),
        "visual_evidence": avbd_visual,
    }
    avbd_packet_path = tmp_path / "avbd-packet.json"
    avbd_packet_path.write_text(json.dumps(avbd_packet), encoding="utf-8")

    return {
        "benchmark_json": benchmark,
        "bend_capture_manifest": bend_manifest,
        "bend_image_verdict_json": bend_verdict,
        "retention_capture_manifest": retention_manifest,
        "retention_image_verdict_json": retention_verdict,
        "long_horizon_capture_manifest": long_horizon_manifest,
        "long_horizon_image_verdict_json": long_horizon_verdict,
        "visual_review_json": review_path,
        "paper_pdf": paper_pdf,
        "paper_figure_image": paper_figure,
        "avbd_packet": avbd_packet_path,
    }


def test_make_packet_records_matched_vbd_and_avbd_evidence(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_packet_module()
    inputs = _write_inputs(module, tmp_path, monkeypatch)

    packet = module.make_packet(**inputs)

    assert packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION
    assert packet["resolved_solver_identity"]["rigid_contact_solver"] == "vbd"
    assert (
        packet["linked_avbd_evidence"]["resolved_solver_identity"][
            "rigid_contact_solver"
        ]
        == "avbd"
    )
    assert (
        packet["visual_evidence"]["bend"]["scene_metrics"]["outcome"][
            "bent_brick_count"
        ]
        == 120
    )
    assert (
        packet["visual_evidence"]["retention"]["scene_metrics"]["outcome"][
            "unbroken_joints"
        ]
        == 712
    )
    long_horizon = packet["visual_evidence"]["long_horizon"]
    assert long_horizon["scene_metrics"]["frame"] == 600
    assert long_horizon["artifact_provenance"]["video"]["decoded_frame_count"] == 600
    assert any(
        entry["role"] == "long_horizon_frame_600"
        for entry in packet["visual_evidence"]["semantic_review"]["inspected_images"]
    )
    assert packet["benchmark"]["comparison"][
        "vbd_to_avbd_median_cpu_cost_ratio"
    ] == pytest.approx(1.25)
    assert (
        packet["benchmark"]["methods"]["avbd"]["solver_configuration_fingerprint"]
        == "fedcba9876543210"
    )
    assert (
        packet["benchmark"]["methods"]["vbd"]["solver_configuration_fingerprint"]
        == "1020304050607080"
    )
    for solver_key in ("avbd", "vbd"):
        stddev = next(
            row
            for row in packet["benchmark"]["methods"][solver_key]["rows"]
            if row["aggregate_name"] == "stddev"
        )
        assert stddev["solver_configuration_fingerprint_hi"] == 0.0
        assert stddev["runtime_identity_contract_passed"] == 0.0
    assert packet["correctness"]["determinism_test"] == (
        "test_vbd_paper_breakable_wall_checkpoints_are_deterministic"
    )
    assert packet["publication_observation"].endswith("does not break.")
    assert packet["target"]["contract_rows"] == ["avbd.paper.fig.13"]
    assert packet["target"]["complete_paper_reproduction"] is False
    bend_artifacts = packet["visual_evidence"]["bend"]["artifact_provenance"]
    assert bend_artifacts["artifact_count"] == 21
    assert bend_artifacts["png_frames"]["count"] == 18
    assert bend_artifacts["video"]["decoded_frame_count"] == 18
    assert bend_artifacts["video"]["fps"] == f"{module.shared.CAPTURE_VIDEO_FPS}/1"
    assert packet["visual_evidence"]["bend"]["capture"]["converted_frames"] == 18
    provenance_paths = {entry["path"] for entry in packet["source_provenance"]["files"]}
    assert {
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/tests/integration/test_demos_cycle.py",
        "scripts/_image_tools.py",
        "scripts/image_verdict.py",
        "scripts/write_avbd_paper_breakable_wall_packet.py",
        "tests/unit/simulation/world/test_world.cpp",
    } <= provenance_paths
    assert str(tmp_path) in json.dumps(packet)


@pytest.mark.parametrize(
    ("solver_key", "counter"),
    (
        ("avbd", "solver_configuration_fingerprint_hi"),
        ("avbd", "rigid_avbd_alpha"),
        ("avbd", "solver_projection_policies_match"),
        ("vbd", "scene_spec_fingerprint_lo"),
        ("vbd", "contact_method_sequential_impulse"),
        ("vbd", "public_vbd_family"),
        ("vbd", "rigid_constraint_iterations"),
        ("vbd", "runtime_identity_contract_passed"),
    ),
)
def test_benchmark_rows_reject_nonzero_invariant_stddev(
    tmp_path: Path,
    solver_key: str,
    counter: str,
) -> None:
    module = _load_packet_module()
    benchmark_path = _write_benchmark(module, tmp_path)
    data = json.loads(benchmark_path.read_text())
    run = module.BENCHMARK_RUNS[solver_key]
    stddev = next(
        row
        for row in data["benchmarks"]
        if row["run_name"] == run and row["aggregate_name"] == "stddev"
    )
    stddev[counter] = 1.0

    with pytest.raises(
        module.AvbdPaperVbdComparisonPacketError,
        match=rf"stddev {counter} must be 0",
    ):
        module._benchmark_rows(
            data,
            solver_key=solver_key,
            expected_fingerprint="0123456789abcdef",
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("solver", "capture resolved_solver_identity"),
        ("capture_source", "capture source provenance digest"),
        ("png_frame", "capture artifact provenance"),
        ("video_artifact", "capture artifact provenance"),
        ("stale_image_verdict", "image verdict image sha256"),
        ("long_horizon_frame", "capture requested_frames"),
        ("long_horizon_video", "capture artifact provenance"),
        ("long_horizon_review", "visual review must inspect"),
        ("fracture", "broken_joints"),
        ("joint_residual", "retained-joint linear residual exceeds"),
        ("benchmark", "expected aggregates"),
        ("benchmark_runtime_inventory", "runtime-image inventory"),
        ("benchmark_loader_false", "loader environment"),
        ("linked_hash", "linked AVBD benchmark JSON hash"),
        ("linked_outcome", "frame 120 outcome status"),
        ("linked_long_horizon", "long-horizon outcome frame"),
        ("linked_verdict_binding", "image verdict screenshot binding"),
        ("linked_verdict_binding_missing", "image_sha256 must be"),
        ("linked_screenshot", "semantic review impact_frame_60 sha256"),
        ("linked_review_hash", "semantic review sha256"),
        ("semantic_claim", "claim_assessments"),
        ("semantic_still_only", "temporal_assessment"),
        ("semantic_video_path", "video file does not match capture"),
    ],
)
def test_make_packet_fails_closed_on_mismatched_comparison_evidence(
    tmp_path: Path,
    monkeypatch,
    mutation: str,
    message: str,
) -> None:
    module = _load_packet_module()
    inputs = _write_inputs(module, tmp_path, monkeypatch)

    if mutation == "solver":
        path = inputs["bend_capture_manifest"]
        data = json.loads(path.read_text())
        data["resolved_solver_identity"]["solver"] = "public_avbd"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "capture_source":
        path = inputs["bend_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_source_provenance"]["digest"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "png_frame":
        manifest = json.loads(inputs["bend_capture_manifest"].read_text())
        frame_path = next(Path(manifest["artifacts"]["frames"]).glob("*.png"))
        frame_path.write_bytes(frame_path.read_bytes() + b"png-mutation")
    elif mutation == "video_artifact":
        manifest = json.loads(inputs["bend_capture_manifest"].read_text())
        video_path = Path(manifest["artifacts"]["video"])
        video_path.write_bytes(video_path.read_bytes() + b"video-mutation")
    elif mutation == "stale_image_verdict":
        path = inputs["bend_capture_manifest"]
        data = json.loads(path.read_text())
        screenshot = Path(data["artifacts"]["screenshot"])
        terminal_frame = Path(data["artifacts"]["frames"]) / "frame_000018.png"
        _write_png(screenshot, red_offset=1)
        _write_png(terminal_frame, red_offset=1)
        updated_hash = _sha256(screenshot)
        data["capture_artifact_provenance"] = module.shared.capture_artifact_provenance(
            scene_metrics_events=Path(data["artifacts"]["scene_metrics_events"]),
            screenshot=screenshot,
            png_frames=sorted(Path(data["artifacts"]["frames"]).glob("*.png")),
            video=Path(data["artifacts"]["video"]),
            video_fps=module.shared.CAPTURE_VIDEO_FPS,
            video_width=8,
            video_height=6,
            screenshot_png_frame_index=18,
        )
        path.write_text(json.dumps(data), encoding="utf-8")

        review_path = inputs["visual_review_json"]
        review = json.loads(review_path.read_text())
        bend = next(
            entry
            for entry in review["inspected_images"]
            if entry["role"] == "bend_frame_18"
        )
        bend["sha256"] = updated_hash
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
    elif mutation == "fracture":
        path = inputs["bend_capture_manifest"]
        data = json.loads(path.read_text())
        data["scene_metrics"]["latest"]["metrics"]["outcome"]["broken_joints"] = 1
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line) for line in events_path.read_text().splitlines() if line
        ]
        events[-1]["metrics"]["outcome"]["broken_joints"] = 1
        events_path.write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8",
        )
    elif mutation == "benchmark":
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        data["benchmarks"] = [
            row
            for row in data["benchmarks"]
            if not row["run_name"].startswith("BM_Avbd")
        ]
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation in {"benchmark_runtime_inventory", "benchmark_loader_false"}:
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        evidence = data["dart_evidence_run"]
        if mutation == "benchmark_runtime_inventory":
            del evidence["build_identity"]["runtime_image_inventory"]
        else:
            evidence["loader_environment"]["passed"] = False
        build_identity = evidence["build_identity"]
        build_payload = {
            key: value
            for key, value in build_identity.items()
            if key not in {"algorithm", "digest"}
        }
        build_identity["digest"] = module.shared._canonical_json_digest(build_payload)
        run_payload = {
            key: value
            for key, value in evidence.items()
            if key not in {"schema_version", "digest"}
        }
        evidence["digest"] = module.shared._canonical_json_digest(run_payload)
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "joint_residual":
        path = inputs["bend_capture_manifest"]
        data = json.loads(path.read_text())
        data["scene_metrics"]["latest"]["metrics"]["outcome"][
            "maximum_unbroken_joint_linear_residual"
        ] = 0.03
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line) for line in events_path.read_text().splitlines() if line
        ]
        events[-1]["metrics"]["outcome"][
            "maximum_unbroken_joint_linear_residual"
        ] = 0.03
        events_path.write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8",
        )
    elif mutation == "linked_hash":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["benchmark"]["json_sha256"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_outcome":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["outcome"]["scene_metrics"]["outcome"][
            "status"
        ] = "fail"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_long_horizon":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["long_horizon"]["scene_metrics"]["outcome"][
            "frame"
        ] = 599
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_verdict_binding":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["impact"]["image_verdict"]["image_sha256"] = "6" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_verdict_binding_missing":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        del data["visual_evidence"]["outcome"]["image_verdict"]["image_sha256"]
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_screenshot":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["impact"]["screenshot"]["sha256"] = "6" * 64
        data["visual_evidence"]["impact"]["image_verdict"]["image_sha256"] = "6" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_review_hash":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["semantic_review"]["sha256"] = "replaced"
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

    with pytest.raises(
        (
            module.AvbdPaperVbdComparisonPacketError,
            module.shared.AvbdPaperBreakableWallPacketError,
        ),
        match=message,
    ):
        module.make_packet(**inputs)
