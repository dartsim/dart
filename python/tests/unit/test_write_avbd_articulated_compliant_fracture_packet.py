from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from hashlib import sha256
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    ROOT / "scripts" / "write_avbd_articulated_compliant_fracture_packet.py"
)


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_articulated_compliant_fracture_packet",
        SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", checksum)
    )


def _write_png(path: Path, width: int = 4, height: int = 3) -> None:
    rows = [b"\x00" + (b"\x10\x20\x30" * width) for _ in range(height)]
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0),
        )
        + _png_chunk(b"IDAT", zlib.compress(b"".join(rows), 6))
        + _png_chunk(b"IEND", b"")
    )


def _scene_metrics(*, saw_intact: float = 1.0) -> dict[str, object]:
    metrics = {
        "angular_stiffness": 100.0,
        "constraint": "finite_and_motor_physical_load_fracture_lifecycle",
        "history": {
            "cycles": 0.0,
            "max_transverse_residual": 0.18,
            "samples": 100.0,
            "saw_broken": 1.0,
            "saw_intact": saw_intact,
            "transitions": 3.0,
        },
        "linear_stiffness": 100.0,
        "max_effort": 6.0,
        "phase": "weak re-arm: break",
        "prestrain": 0.08,
        "start_stiffness": 100.0,
        "strong_break_force": 100.0,
        "time_step_ms": 5.0,
        "weak_break_force": 9.0,
    }
    latest = {
        "event": "scene_capture_metrics",
        "frame": 100,
        "metrics": metrics,
        "scene": "avbd_articulated_compliant_breakable_motor",
    }
    return {
        "event_count": 100,
        "first": latest,
        "latest": latest,
        "metric_key_counts": {key: 100 for key in metrics},
        "numeric_ranges": {},
    }


def _write_capture_manifest(
    tmp_path: Path,
    *,
    docked_workspace: bool = True,
    saw_intact: float = 1.0,
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True, exist_ok=True)
    screenshot = capture / "avbd_articulated_compliant_breakable_motor.png"
    _write_png(screenshot)
    for index in range(1, 101):
        _write_png(frames / f"frame_{index:06d}.png")
    metrics_events = capture / "scene_metrics.jsonl"
    metrics_events.write_text(
        json.dumps(_scene_metrics(saw_intact=saw_intact)["latest"]) + "\n",
        encoding="utf-8",
    )
    manifest = {
        "artifacts": {
            "events": None,
            "frames": str(frames),
            "scene_metrics_events": str(metrics_events),
            "screenshot": str(screenshot),
        },
        "force_drag": None,
        "scene": "avbd_articulated_compliant_breakable_motor",
        "scene_metrics": _scene_metrics(saw_intact=saw_intact),
        "schema_version": 1,
        "show_ui": True,
        "switch_frame": None,
        "switch_scene": None,
        "ui_ready": {
            "dropped_warmup_frames": 1,
            "required": True,
        },
        "visual_evidence": {
            "first_frame": {"docked_workspace": docked_workspace},
            "screenshot": {"docked_workspace": docked_workspace},
        },
    }
    path = capture / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _write_image_verdict(tmp_path: Path) -> Path:
    path = tmp_path / "capture" / "image_verdict.json"
    path.write_text(
        json.dumps(
            {
                "checks": {
                    "contrast": {"pass": True},
                    "non_blank": {"pass": True},
                },
                "image": {
                    "height": 3,
                    "path": "avbd_articulated_compliant_breakable_motor.png",
                    "width": 4,
                },
                "machine_scope": "pixel-integrity",
                "metadata": {
                    "scene": "avbd_articulated_compliant_breakable_motor",
                    "view": "three-quarter",
                },
                "pass": True,
                "schema_version": "dart.image_verdict/v1",
            }
        ),
        encoding="utf-8",
    )
    return path


def _write_visual_review(tmp_path: Path) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    selected = {
        "start": frames / "frame_000001.png",
        "middle": frames / "frame_000050.png",
        "end": frames / "frame_000100.png",
    }
    path = capture / "visual_review.json"
    path.write_text(
        json.dumps(
            {
                "inspected_frames": [
                    {
                        "file": f"png_frames/{frame.name}",
                        "role": role,
                        "sha256": _sha256(frame),
                    }
                    for role, frame in selected.items()
                ],
                "limitations": (
                    "Images corroborate the lifecycle but do not measure "
                    "individual physical row loads."
                ),
                "observations": (
                    "The connector is broken, intact after reset, and broken "
                    "again in the three inspected phases."
                ),
                "reviewer_capability": "native image input",
                "scene": "avbd_articulated_compliant_breakable_motor",
                "schema_version": "dart.visual_semantic_review/v1",
                "verdict": "pass",
            }
        ),
        encoding="utf-8",
    )
    return path


def _write_benchmark_json(
    tmp_path: Path,
    *,
    missing_arg: int | None = None,
) -> Path:
    rows = []
    for arg in (1, 4, 16):
        if arg == missing_arg:
            continue
        rows.append(
            {
                "aggregate_name": "median",
                "breakable_motors": float(2 * arg),
                "cpu_time": float(1000 * arg),
                "family_instances": float(arg),
                "iterations": 10,
                "name": (
                    "BM_AvbdArticulatedCompliantBreakableMotorStep/"
                    f"{arg}_median"
                ),
                "prismatic_motors": float(arg),
                "real_time": float(1100 * arg),
                "revolute_motors": float(arg),
                "run_name": (
                    "BM_AvbdArticulatedCompliantBreakableMotorStep/"
                    f"{arg}"
                ),
                "run_type": "aggregate",
                "time_unit": "ns",
            }
        )
    benchmark = {
        "benchmarks": rows,
        "context": {
            "executable": "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint",
            "json_schema_version": 1,
            "library_build_type": "release",
            "library_version": "v1.9.5",
            "mhz_per_cpu": 5300,
            "num_cpus": 32,
        },
    }
    path = tmp_path / "benchmark.json"
    path.write_text(json.dumps(benchmark), encoding="utf-8")
    return path


def _write_exact_parent_mutation(
    tmp_path: Path,
    *,
    parent_commit: str = "9ebd9b895b17e982bd1ed9287287d589b029409f",
    omit_case: str | None = None,
) -> Path:
    cases = (
        "finite_only",
        "motor_dt_0.005",
        "motor_dt_0.01",
        "combined_load",
        "reset_lifecycle",
    )
    path = tmp_path / "exact_parent_mutation.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "dart.exact_parent_mutation/v1",
                "parent_commit": parent_commit,
                "tests": [
                    "VariationalIntegration."
                    "AvbdCompliantPublicArticulatedJointAggregatesFiniteAndMotor"
                    "BreakLoads",
                    "VariationalIntegration."
                    "AvbdCompliantPublicArticulatedJointBreakResetRearmsFiniteRows",
                ],
                "mutation": (
                    "Apply the candidate finite-load tests to the exact parent "
                    "without the load-accounting implementation."
                ),
                "command": (
                    "./test_variational_integration --gtest_filter="
                    "VariationalIntegration.AvbdCompliantPublicArticulated*"
                ),
                "exit_code": 1,
                "expected_result": "fail",
                "observed_result": "fail",
                "failed_assertions": [
                    {
                        "actual": False,
                        "case": case,
                        "expected": True,
                    }
                    for case in cases
                    if case != omit_case
                ],
                "interpretation": (
                    "The exact parent does not fracture from finite or "
                    "finite-plus-motor physical loads."
                ),
            }
        ),
        encoding="utf-8",
    )
    return path


def _write_valid_inputs(tmp_path: Path) -> dict[str, Path]:
    return {
        "capture_manifest": _write_capture_manifest(tmp_path),
        "image_verdict_json": _write_image_verdict(tmp_path),
        "visual_review_json": _write_visual_review(tmp_path),
        "benchmark_json": _write_benchmark_json(tmp_path),
        "exact_parent_mutation_json": _write_exact_parent_mutation(tmp_path),
    }


def test_packet_records_finite_load_fracture_evidence(tmp_path: Path) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(inputs["capture_manifest"]),
                "--image-verdict-json",
                str(inputs["image_verdict_json"]),
                "--visual-review-json",
                str(inputs["visual_review_json"]),
                "--benchmark-json",
                str(inputs["benchmark_json"]),
                "--exact-parent-mutation-json",
                str(inputs["exact_parent_mutation_json"]),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text(encoding="utf-8"))
    assert packet["schema_version"] == 2
    assert packet["resolved_solver_identity"]["rigid_contact_solver"] == "none"
    assert packet["resolved_solver_identity"]["rigid_point_joint_solver"] == "avbd"
    assert packet["load_contract"]["joint_aggregation"] == (
        "L2 norm across finite and projection rows"
    )
    assert packet["visual_capture"]["scene_metrics"]["event_count"] == 100
    assert len(packet["benchmark"]["scale_data"]) == 3
    assert packet["target"]["complete_paper_reproduction"] is False


def test_packet_rejects_capture_without_intact_reset_phase(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["capture_manifest"] = _write_capture_manifest(
        tmp_path,
        saw_intact=0.0,
    )

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match="saw_intact",
    ):
        module.make_packet(**inputs)


def test_packet_rejects_missing_benchmark_scale(tmp_path: Path) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["benchmark_json"] = _write_benchmark_json(tmp_path, missing_arg=4)

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match="missing breakable finite-motor scale rows",
    ):
        module.make_packet(**inputs)


def test_packet_rejects_wrong_exact_parent(tmp_path: Path) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["exact_parent_mutation_json"] = _write_exact_parent_mutation(
        tmp_path,
        parent_commit="deadbeef",
    )

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match="parent_commit",
    ):
        module.make_packet(**inputs)


def test_packet_rejects_incomplete_exact_parent_failures(tmp_path: Path) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["exact_parent_mutation_json"] = _write_exact_parent_mutation(
        tmp_path,
        omit_case="combined_load",
    )

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match="combined_load",
    ):
        module.make_packet(**inputs)


def test_packet_rejects_undocked_visual_capture(tmp_path: Path) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["capture_manifest"] = _write_capture_manifest(
        tmp_path,
        docked_workspace=False,
    )

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match="docked workspace",
    ):
        module.make_packet(**inputs)
