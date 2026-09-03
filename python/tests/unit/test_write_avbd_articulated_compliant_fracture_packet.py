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


def _write_image_verdict(
    tmp_path: Path,
    *,
    image_sha256: str | None = None,
    omit_image_sha256: bool = False,
) -> Path:
    screenshot = (
        tmp_path / "capture" / "avbd_articulated_compliant_breakable_motor.png"
    )
    image: dict[str, object] = {
        "height": 3,
        "path": "avbd_articulated_compliant_breakable_motor.png",
        "width": 4,
    }
    if not omit_image_sha256:
        image["sha256"] = (
            _sha256(screenshot) if image_sha256 is None else image_sha256
        )
    path = tmp_path / "capture" / "image_verdict.json"
    path.write_text(
        json.dumps(
            {
                "checks": {
                    "contrast": {"pass": True},
                    "non_blank": {"pass": True},
                },
                "image": image,
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
                    "individual solver-row load coordinates."
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


VARIATIONAL_RUNTIME_COUNTERS = {
    "runtime_identity_recorded": 1.0,
    "runtime_identity_applicable": 1.0,
    "runtime_identity_not_applicable": 0.0,
    "runtime_identity_public_avbd_rigid": 0.0,
    "runtime_identity_variational_multibody": 1.0,
    "runtime_identity_contract_passed": 1.0,
    "public_avbd_family": 0.0,
    "public_sequential_impulse_family": 1.0,
    "resolved_rigid_body_avbd": 0.0,
    "resolved_rigid_contact_avbd": 0.0,
    "resolved_rigid_body_sequential_impulse": 1.0,
    "resolved_rigid_contact_sequential_impulse": 1.0,
    "resolved_rigid_pair_constraint_sequential_impulse": 1.0,
    "resolved_rigid_pair_constraint_not_applicable": 0.0,
    "configured_multibody_variational": 1.0,
    "resolved_multibody_variational": 1.0,
}


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
                **VARIATIONAL_RUNTIME_COUNTERS,
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
                    (
                        "VariationalIntegration."
                        + "AvbdCompliantPublicArticulatedJointAggregatesFiniteAndMotor"
                        + "BreakLoads"
                    ),
                    (
                        "VariationalIntegration."
                        + "AvbdCompliantPublicArticulatedJointBreakResetRearmsFiniteRows"
                    ),
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
                    "finite-plus-motor solver-row metrics."
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
    assert packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION
    assert (
        packet["resolved_solver_identity"]["rigid_contact_selection"]
        == "contact_solver_method"
    )
    assert (
        packet["resolved_solver_identity"]["rigid_contact_solver"]
        == "sequential_impulse"
    )
    assert (
        packet["resolved_solver_identity"]["rigid_point_joint_solver"]
        == "sequential_impulse"
    )
    assert (
        packet["resolved_solver_identity"]["multibody_integration_family"]
        == "variational"
    )
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


def test_packet_binds_image_verdict_to_screenshot_sha256(
    tmp_path: Path,
) -> None:
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
    screenshot = (
        tmp_path / "capture" / "avbd_articulated_compliant_breakable_motor.png"
    )
    image_verdict = packet["visual_capture"]["image_verdict"]
    assert image_verdict["image_sha256"] == _sha256(screenshot)
    assert (
        image_verdict["image_sha256"]
        == packet["visual_capture"]["screenshot"]["sha256"]
    )


def test_packet_rejects_image_verdict_for_a_foreign_image(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["image_verdict_json"] = _write_image_verdict(
        tmp_path,
        image_sha256="0" * 64,
    )

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match=(
            "image verdict image sha256 does not match capture screenshot"
        ),
    ):
        module.make_packet(**inputs)


@pytest.mark.parametrize(
    ("verdict_kwargs", "expected_message"),
    [
        (
            {"omit_image_sha256": True},
            "image verdict image sha256 must be a 64-character lowercase "
            "hexadecimal string",
        ),
        (
            {"image_sha256": "z" * 64},
            "image verdict image sha256 must be hexadecimal",
        ),
        (
            {"image_sha256": "0" * 63},
            "image verdict image sha256 must be a 64-character lowercase "
            "hexadecimal string",
        ),
        (
            {"image_sha256": "A" * 64},
            "image verdict image sha256 must use canonical lowercase "
            "hexadecimal",
        ),
    ],
)
def test_packet_rejects_malformed_image_verdict_sha256(
    tmp_path: Path,
    verdict_kwargs: dict[str, object],
    expected_message: str,
) -> None:
    module = _load_packet_module()
    inputs = _write_valid_inputs(tmp_path)
    inputs["image_verdict_json"] = _write_image_verdict(
        tmp_path,
        **verdict_kwargs,
    )

    with pytest.raises(
        module.AvbdArticulatedCompliantFracturePacketError,
        match=expected_message,
    ):
        module.make_packet(**inputs)
