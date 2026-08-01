from __future__ import annotations

from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import struct
import sys
from typing import Any
import zlib

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    ROOT
    / "scripts"
    / "write_avbd_paper_sequential_impulse_comparison_packet.py"
)


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_paper_sequential_impulse_comparison_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_png(path: Path, width: int = 8, height: int = 6) -> None:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    rows = b"".join(
        b"\x00" + bytes((32 + row, 64, 128)) * width
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


def _si_broken_joint_records() -> list[dict[str, Any]]:
    impact_indices = (0, 0, 1, 2, 2)
    records = []
    for index, impact_index in enumerate(impact_indices):
        records.append(
            {
                "angular_residual_radians": 0.01 + index * 0.001,
                "child": {
                    "body": "brick",
                    "column": index + 1,
                    "row": index + 2,
                },
                "id": f"fixture_si_joint_{index}",
                "initial_anchor": [float(index), 0.0, float(index + 1)],
                "kind": "horizontal" if index < 2 else "vertical",
                "linear_residual": 0.001 + index * 0.0001,
                "nearest_impact_distance": 0.3 + index * 0.05,
                "nearest_impact_index": impact_index,
                "parent": {
                    "body": "brick",
                    "column": index,
                    "row": index + 2,
                },
                "within_impact_band": True,
                "within_impact_region": True,
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


def _joint_evidence(
    *,
    broken: int,
    records: list[dict[str, Any]],
    maximum_angular: float,
    maximum_linear: float,
    rms_angular: float,
    rms_linear: float,
) -> dict[str, Any]:
    return {
        "broken_joint_identity_count": broken,
        "broken_joint_ids_sha256": _joint_ids_sha256(records),
        "broken_joint_impact_region_counts": (
            [2, 1, 2] if broken else [0, 0, 0]
        ),
        "broken_joint_records": records,
        "broken_joints_outside_impact_regions": 0,
        "joint_residuals_finite": True,
        "maximum_outside_impact_unbroken_joint_angular_residual_radians": (
            maximum_angular
        ),
        "maximum_outside_impact_unbroken_joint_linear_residual": maximum_linear,
        "maximum_unbroken_joint_angular_residual_radians": maximum_angular,
        "maximum_unbroken_joint_linear_residual": maximum_linear,
        "outside_impact_unbroken_joint_residual_count": 484,
        "rms_outside_impact_unbroken_joint_angular_residual_radians": (
            rms_angular
        ),
        "rms_outside_impact_unbroken_joint_linear_residual": rms_linear,
        "rms_unbroken_joint_angular_residual_radians": rms_angular,
        "rms_unbroken_joint_linear_residual": rms_linear,
        "unbroken_joint_residual_count": 712 - broken,
    }


def _outcome(module, frame: int) -> dict[str, Any]:
    generic = {
        "ball_positions": [[0.0, 1.0, 2.0]] * 3,
        "ball_velocities": [[3.0, 4.0, 5.0]] * 3,
        "bent_brick_count": 0,
        "broken_joints": 0,
        "checkpoint": "fracture",
        "contact_count": 0,
        "evaluated": False,
        "frame": frame,
        "impact_band_displaced_counts": [0, 0, 0],
        "impact_damage_displacement_threshold": 0.1,
        "last_step_iterations": 20,
        "max_brick_displacement": 0.0,
        "maximum_wall_normal_displacement": 0.0,
        "outside_brick_count": 181,
        "outside_retained_fraction": 1.0,
        "retained_displacement_threshold": 0.5,
        "rms_wall_normal_displacement": 0.0,
        "status": "pre-evaluation",
        "threshold_checks": {"finite_state": True},
        "thresholds_pass": False,
        "total_retained_fraction": 1.0,
        "unbroken_joints": 712,
        "world_time": frame / 60.0,
    } | _joint_evidence(
        broken=0,
        records=[],
        maximum_angular=0.0,
        maximum_linear=0.0,
        rms_angular=0.0,
        rms_linear=0.0,
    )
    if frame == 14:
        expected = module.EXPECTED_OUTCOMES[14]
        return generic | expected | {
            "bent_brick_count": 103,
            "contact_count": 17,
            "evaluated": True,
            "max_brick_displacement": 0.22658181580080475,
            "maximum_wall_normal_displacement": 0.22658181580080475,
            "rms_wall_normal_displacement": 0.08221218581583968,
            "status": "pass",
            "threshold_checks": dict(
                module.CHECKPOINTS[14]["threshold_checks"]
            ),
            "thresholds_pass": True,
        } | _joint_evidence(
            broken=5,
            records=_si_broken_joint_records(),
            maximum_angular=0.095,
            maximum_linear=0.025,
            rms_angular=0.025,
            rms_linear=0.012,
        )
    if frame == 120:
        expected = module.EXPECTED_OUTCOMES[120]
        return generic | expected | {
            "bent_brick_count": 230,
            "checkpoint": "collapse",
            "contact_count": 298,
            "evaluated": True,
            "max_brick_displacement": 2.9062896239039815,
            "maximum_wall_normal_displacement": 2.9062896239039815,
            "rms_wall_normal_displacement": 1.564993114358864,
            "status": "pass",
            "threshold_checks": dict(
                module.CHECKPOINTS[120]["threshold_checks"]
            ),
            "thresholds_pass": True,
        } | _joint_evidence(
            broken=5,
            records=_si_broken_joint_records(),
            maximum_angular=0.88,
            maximum_linear=0.12,
            rms_angular=0.24,
            rms_linear=0.04,
        )
    return generic


def _metrics(module, frame: int) -> dict[str, Any]:
    return {
        "ball_count": 3,
        "break_force": module.BREAK_FORCE,
        "breakable_joints": 712,
        "brick_count": 252,
        "collision_shapes": 256,
        "executor": "World.step default",
        "outcome": _outcome(module, frame),
        "outcome_oracle": dict(module.OUTCOME_ORACLE),
        "paper_locator": module.PAPER_LOCATOR,
        "rigid_bodies": 256,
        "rigid_body_solver": "SEQUENTIAL_IMPULSE",
        "rigid_constraint_options": {"iterations": 20},
        "resolved_configuration": [
            {
                "domain": "rigid-body",
                "reason": "as requested",
                "requested": "sequential-impulse",
                "resolved": "sequential-impulse",
            },
            {
                "domain": "rigid-contact",
                "reason": "as requested",
                "requested": "sequential-impulse",
                "resolved": "sequential-impulse",
            },
            {
                "domain": "rigid-pair-constraint",
                "reason": "as requested",
                "requested": "sequential-impulse",
                "resolved": "sequential-impulse",
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
        "solver": "public_sequential-impulse",
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
    manifest = {
        "artifacts": {
            "events": None,
            "frames": str(directory / "png_frames"),
            "scene_metrics_events": str(metrics_log),
            "screenshot": str(screenshot),
            "video": None,
        },
        "camera": {
            "azimuth": module.shared.math.degrees(
                module.shared.CAMERA_AZIMUTH
            ),
            "distance": module.shared.CAMERA_DISTANCE,
            "elevation": module.shared.math.degrees(
                module.shared.CAMERA_ELEVATION
            ),
            "target": list(module.shared.CAMERA_TARGET),
            "view": module.shared.CAMERA_PRESET,
        },
        "capture": {
            "converted_frames": frame,
            "height": 6,
            "requested_frames": frame,
            "width": 8,
        },
        "capture_artifact_provenance": {
            "algorithm": module.shared.CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
            "scene_metrics_events_sha256": _sha256(metrics_log),
            "screenshot_sha256": _sha256(screenshot),
        },
        "capture_source_provenance": (
            module.shared.compute_capture_source_provenance(
                module.shared.REPO_ROOT
            )
        ),
        "capture_label": label,
        "force_drag": None,
        "resolved_solver_identity": {
            "executor": "World.step default",
            "solver": "public_sequential-impulse",
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


def _write_benchmark(module, tmp_path: Path) -> tuple[Path, dict[str, Any]]:
    fingerprint = int("0123456789abcdef", 16)
    rows = []
    scales = {"avbd": 1.0, "vbd": 1.25, "sequential-impulse": 0.5}
    for solver_key, scale in scales.items():
        run = module.comparison.BENCHMARK_RUNS[solver_key]
        for aggregate, real_time, cpu_time in (
            ("mean", 7_800_000.0 * scale, 7_790_000.0 * scale),
            ("median", 7_750_000.0 * scale, 7_740_000.0 * scale),
            ("stddev", 110_000.0, 109_000.0),
            ("cv", 0.014, 0.014),
        ):
            row = {
                "aggregate_name": aggregate,
                "aggregate_unit": (
                    "percentage" if aggregate == "cv" else "time"
                ),
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
                        "impacting_balls": 3.0,
                        f"public_{solver_key}_family": 1.0,
                        f"resolved_rigid_body_{solver_key}": 1.0,
                        "resolved_rigid_constraint_iterations": 1.0,
                        f"resolved_rigid_contact_{solver_key}": 1.0,
                        (
                            f"resolved_rigid_pair_constraint_{solver_key}"
                        ): 1.0,
                        "rigid_constraint_iterations": 20.0,
                        "rigid_bodies": 256.0,
                        "rigid_body_joints": 712.0,
                        "runtime_contract_passed": 1.0,
                        "scene_spec_fingerprint_hi": float(
                            fingerprint >> 32
                        ),
                        "scene_spec_fingerprint_lo": float(
                            fingerprint & 0xFFFFFFFF
                        ),
                        "trajectory_frames": 120.0,
                    }
                )
            rows.append(row)
    data = {
        "benchmarks": rows,
        "context": {
            "benchmark_source_sha256": _sha256(
                module.shared.REPO_ROOT
                / module.shared.BENCHMARK_SOURCE_PATH
            ),
            "capture_source_provenance_digest": (
                module.shared.compute_capture_source_provenance(
                    module.shared.REPO_ROOT
                )["digest"]
            ),
            "executable": (
                "build/default/cpp/Release/bin/"
                "bm_avbd_rigid_fixed_joint"
            ),
            "host_name": "test-host",
            "json_schema_version": 1,
            "library_build_type": "release",
            "library_version": "test",
            "mhz_per_cpu": 5000,
            "num_cpus": 8,
        },
    }
    path = tmp_path / "benchmark.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path, data


def _linked_capture(
    module,
    *,
    frame: int,
    label: str,
    screenshot_hash: str,
) -> dict[str, Any]:
    return {
        "image_verdict": {"pass": True, "sha256": "a" * 64},
        "scene_metrics": {
            "frame": frame,
            "outcome": {
                "broken_joints": 0,
                "broken_joint_identity_count": 0,
                "broken_joint_ids_sha256": sha256().hexdigest(),
                "broken_joint_impact_region_counts": [0, 0, 0],
                "broken_joint_records": [],
                "broken_joints_outside_impact_regions": 0,
                "evaluated": True,
                "joint_residuals_finite": True,
                "maximum_outside_impact_unbroken_joint_angular_residual_radians": 0.013,
                "maximum_outside_impact_unbroken_joint_linear_residual": 0.016,
                "maximum_unbroken_joint_angular_residual_radians": 0.013,
                "maximum_unbroken_joint_linear_residual": 0.016,
                "outside_impact_unbroken_joint_residual_count": 484,
                "rms_outside_impact_unbroken_joint_angular_residual_radians": 0.003,
                "rms_outside_impact_unbroken_joint_linear_residual": 0.004,
                "rms_unbroken_joint_angular_residual_radians": 0.003,
                "rms_unbroken_joint_linear_residual": 0.004,
                "status": "pass",
                "threshold_checks": dict(
                    module.comparison.CHECKPOINTS[frame][
                        "threshold_checks"
                    ]
                ),
                "thresholds_pass": True,
                "unbroken_joints": 712,
                "unbroken_joint_residual_count": 712,
            },
            "scene_spec_fingerprint": "0123456789abcdef",
        },
        "screenshot": {
            "file": f"{label}.png",
            "sha256": screenshot_hash,
        },
    }


def _write_linked_packet(
    module,
    tmp_path: Path,
    benchmark: Path,
    benchmark_data: dict[str, Any],
    paper_figure: Path,
) -> Path:
    bend_hash = "1" * 64
    retention_hash = "2" * 64
    paper_hash = _sha256(paper_figure)
    avbd_identity = {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "test",
        "rigid_contact_selection": "world_solver_family",
        "rigid_contact_solver": "avbd",
        "rigid_point_joint_solver": "avbd",
    }
    avbd_packet = {
        "benchmark": {
            "scene_spec_fingerprint": "0123456789abcdef",
        },
        "packet": "avbd_paper_breakable_wall",
        "resolved_solver_identity": avbd_identity,
        "scene": module.shared.SCENE_ID,
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "source_provenance": module.shared._source_provenance(),
    }
    avbd_packet_path = (
        tmp_path / "avbd-paper-breakable-wall-packet.json"
    )
    avbd_packet_path.write_text(json.dumps(avbd_packet), encoding="utf-8")
    methods = {
        solver_key: module.comparison._benchmark_rows(
            benchmark_data,
            solver_key=solver_key,
            expected_fingerprint="0123456789abcdef",
        )
        for solver_key in ("avbd", "vbd")
    }
    packet = {
        "benchmark": {
            "json_sha256": _sha256(benchmark),
            "methods": methods,
            "scene_spec_fingerprint": "0123456789abcdef",
        },
        "correctness": {
            "determinism_test": (
                "test_vbd_paper_breakable_wall_checkpoints_are_deterministic"
            ),
            "outcome_test": (
                "test_vbd_paper_breakable_wall_matches_figure13_contract"
            ),
        },
        "linked_avbd_evidence": {
            "file": avbd_packet_path.name,
            "resolved_solver_identity": avbd_identity,
            "scene_spec_fingerprint": "0123456789abcdef",
            "sha256": _sha256(avbd_packet_path),
        },
        "packet": module.LINKED_PACKET_ID,
        "paper_reference": {
            "figure": {
                "file": paper_figure.name,
                "sha256": paper_hash,
            }
        },
        "resolved_solver_identity": {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "test",
            "rigid_contact_selection": "world_solver_family",
            "rigid_contact_solver": "vbd",
            "rigid_point_joint_solver": "vbd",
        },
        "scene": module.LINKED_SCENE_ID,
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "source_provenance": module.comparison._source_provenance(),
        "visual_evidence": {
            "bend": _linked_capture(
                module,
                frame=18,
                label="bend",
                screenshot_hash=bend_hash,
            ),
            "retention": _linked_capture(
                module,
                frame=120,
                label="retention",
                screenshot_hash=retention_hash,
            ),
            "semantic_review": {
                "inspected_images": [
                    {
                        "file": "bend.png",
                        "role": "bend_frame_18",
                        "sha256": bend_hash,
                    },
                    {
                        "file": "retention.png",
                        "role": "retention_frame_120",
                        "sha256": retention_hash,
                    },
                    {
                        "file": paper_figure.name,
                        "role": "paper_figure_13_reference",
                        "sha256": paper_hash,
                    },
                ],
                "sha256": "3" * 64,
                "verdict": "pass",
            },
        },
    }
    path = tmp_path / "avbd-vbd-packet.json"
    path.write_text(json.dumps(packet), encoding="utf-8")
    return path


def _write_inputs(module, tmp_path: Path, monkeypatch) -> dict[str, Path]:
    module.OUTCOME_ORACLE["expected_broken_joint_ids_sha256"] = (
        _joint_ids_sha256(_si_broken_joint_records())
    )
    fracture_manifest, fracture_verdict = _write_capture(
        module,
        tmp_path,
        14,
        "fracture",
    )
    collapse_manifest, collapse_verdict = _write_capture(
        module,
        tmp_path,
        120,
        "collapse",
    )
    benchmark, benchmark_data = _write_benchmark(module, tmp_path)

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
    linked_packet = _write_linked_packet(
        module,
        tmp_path,
        benchmark,
        benchmark_data,
        paper_figure,
    )

    fracture_screenshot = (
        fracture_manifest.parent / f"{module.SCENE_ID}_fracture.png"
    )
    collapse_screenshot = (
        collapse_manifest.parent / f"{module.SCENE_ID}_collapse.png"
    )
    review = {
        "claim_and_expected_observation": (
            "Sequential Impulse first fractures locally and later collapses."
        ),
        "inspected_images": [
            {
                "file": str(fracture_screenshot),
                "role": "fracture_frame_14",
                "sha256": _sha256(fracture_screenshot),
            },
            {
                "file": str(collapse_screenshot),
                "role": "collapse_frame_120",
                "sha256": _sha256(collapse_screenshot),
            },
            {
                "file": str(paper_figure),
                "role": "paper_figure_13_reference",
                "sha256": _sha256(paper_figure),
            },
        ],
        "not_proven_and_limitations": (
            "Exact unpublished constants, XPBD, CUDA, and published/source "
            "timing remain unproven."
        ),
        "reconciliation_and_verdict": (
            "PASS: both ViewReports and both numeric checkpoints agree with "
            "the inspected images."
        ),
        "reviewer_capability": "native image input at original detail",
        "scene": module.SCENE_ID,
        "schema_version": "dart.visual_semantic_review/v1",
        "text_oracle": (
            "Figure 13 shows initial fracture, slow bending, and collapse."
        ),
        "verdict": "pass",
        "visible_observation": (
            "The early wall remains mostly placed around a local fracture; "
            "the final wall has collapsed."
        ),
    }
    review_path = tmp_path / "visual_review.json"
    review_path.write_text(json.dumps(review), encoding="utf-8")

    return {
        "benchmark_json": benchmark,
        "fracture_capture_manifest": fracture_manifest,
        "fracture_image_verdict_json": fracture_verdict,
        "collapse_capture_manifest": collapse_manifest,
        "collapse_image_verdict_json": collapse_verdict,
        "visual_review_json": review_path,
        "paper_pdf": paper_pdf,
        "paper_figure_image": paper_figure,
        "avbd_vbd_packet": linked_packet,
    }


def _mutate_latest_capture(
    manifest_path: Path,
    mutation,
) -> None:
    manifest = json.loads(manifest_path.read_text())
    mutation(manifest["scene_metrics"]["latest"])
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    events_path = Path(manifest["artifacts"]["scene_metrics_events"])
    events = [
        json.loads(line)
        for line in events_path.read_text().splitlines()
        if line
    ]
    mutation(events[-1])
    events_path.write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )


def test_make_packet_records_matched_three_method_evidence(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_packet_module()
    inputs = _write_inputs(module, tmp_path, monkeypatch)

    packet = module.make_packet(**inputs)

    identity = packet["resolved_solver_identity"]
    assert identity["rigid_contact_solver"] == "sequential_impulse"
    assert identity["rigid_point_joint_solver"] == "sequential_impulse"
    assert identity["rigid_contact_selection"] == "contact_solver_method"
    assert packet["linked_avbd_vbd_evidence"]["resolved_solver_identity"][
        "rigid_contact_solver"
    ] == "vbd"
    assert packet["linked_avbd_vbd_evidence"][
        "nested_avbd_resolved_solver_identity"
    ]["rigid_contact_solver"] == "avbd"
    assert packet["visual_evidence"]["fracture"]["scene_metrics"]["outcome"][
        "broken_joints"
    ] == 5
    assert packet["visual_evidence"]["collapse"]["scene_metrics"]["outcome"][
        "total_retained_fraction"
    ] == pytest.approx(0.19047619047619047)
    comparison = packet["benchmark"]["comparison"]
    assert comparison[
        "sequential_impulse_to_avbd_median_cpu_cost_ratio"
    ] == pytest.approx(0.5)
    assert comparison[
        "sequential_impulse_to_vbd_median_cpu_cost_ratio"
    ] == pytest.approx(0.4)
    assert packet["target"]["contract_rows"] == ["avbd.paper.fig.13"]
    assert packet["target"]["complete_paper_reproduction"] is False
    provenance_paths = {
        entry["path"] for entry in packet["source_provenance"]["files"]
    }
    assert {
        "scripts/avbd_packet_schema.py",
        (
            "python/examples/demos/scenes/"
            "sequential_impulse_paper_breakable_wall.py"
        ),
        "tests/unit/simulation/contact/test_boxed_lcp_contact.cpp",
    } <= provenance_paths
    assert str(tmp_path) not in json.dumps(packet)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("solver", "capture resolved_solver_identity"),
        ("capture_source", "capture source provenance digest"),
        ("fracture", "broken_joints"),
        (
            "retained_residual",
            "rms_outside_impact_unbroken_joint_angular_residual_radians "
            "must be",
        ),
        ("benchmark", "expected aggregates"),
        ("linked_hash", "linked AVBD/VBD benchmark JSON hash"),
        ("linked_outcome", "retention outcome status"),
        ("linked_source", "source provenance"),
        ("nested_link_hash", "nested linked AVBD current file sha256"),
        ("nested_link_source", "nested linked AVBD source provenance"),
        ("linked_review", "paper figure sha256"),
        ("limitations", "limitations must name xpbd"),
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

    if mutation == "solver":
        path = inputs["fracture_capture_manifest"]
        data = json.loads(path.read_text())
        data["resolved_solver_identity"]["solver"] = "public_avbd"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "capture_source":
        path = inputs["fracture_capture_manifest"]
        data = json.loads(path.read_text())
        data["capture_source_provenance"]["digest"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "fracture":
        _mutate_latest_capture(
            inputs["fracture_capture_manifest"],
            lambda event: event["metrics"]["outcome"].__setitem__(
                "broken_joints",
                6,
            ),
            )
    elif mutation == "retained_residual":
        _mutate_latest_capture(
            inputs["collapse_capture_manifest"],
            lambda event: event["metrics"]["outcome"].__setitem__(
                "rms_outside_impact_unbroken_joint_angular_residual_radians",
                0.1,
            ),
        )
    elif mutation == "benchmark":
        path = inputs["benchmark_json"]
        data = json.loads(path.read_text())
        data["benchmarks"] = [
            row
            for row in data["benchmarks"]
            if not row["run_name"].startswith("BM_SequentialImpulse")
        ]
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_hash":
        path = inputs["avbd_vbd_packet"]
        data = json.loads(path.read_text())
        data["benchmark"]["json_sha256"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_outcome":
        path = inputs["avbd_vbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["retention"]["scene_metrics"]["outcome"][
            "status"
        ] = "fail"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_source":
        path = inputs["avbd_vbd_packet"]
        data = json.loads(path.read_text())
        data["source_provenance"]["digest"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "nested_link_hash":
        path = inputs["avbd_vbd_packet"]
        data = json.loads(path.read_text())
        data["linked_avbd_evidence"]["sha256"] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "nested_link_source":
        path = inputs["avbd_vbd_packet"]
        data = json.loads(path.read_text())
        nested_path = path.parent / data["linked_avbd_evidence"]["file"]
        nested = json.loads(nested_path.read_text())
        nested["source_provenance"]["digest"] = "0" * 64
        nested_path.write_text(json.dumps(nested), encoding="utf-8")
        data["linked_avbd_evidence"]["sha256"] = _sha256(nested_path)
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_review":
        path = inputs["avbd_vbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["semantic_review"]["inspected_images"][2][
            "sha256"
        ] = "0" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "limitations":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["not_proven_and_limitations"] = (
            "Exact unpublished constants, CUDA, and published/source timing "
            "remain unproven."
        )
        path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(
        (
            module.AvbdPaperSequentialImpulseComparisonPacketError,
            module.comparison.AvbdPaperVbdComparisonPacketError,
            module.shared.AvbdPaperBreakableWallPacketError,
        ),
        match=message,
    ):
        module.make_packet(**inputs)
