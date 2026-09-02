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
    ROOT / "scripts" / "write_avbd_articulated_compliant_motors_packet.py"
)


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_articulated_compliant_motors_packet",
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
        struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)
    )


def _write_png(path: Path, width: int = 4, height: int = 3) -> None:
    rows = [b"\x00" + (b"\x10\x20\x30" * width) for _ in range(height)]
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(b"".join(rows), 6))
        + _png_chunk(b"IEND", b"")
    )


def _write_capture_manifest(
    tmp_path: Path,
    *,
    scene: str = "avbd_articulated_compliant_motors",
    docked_workspace: bool = True,
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_articulated_compliant_motors.png"
    _write_png(screenshot)
    _write_png(frames / "frame_000001.png")
    _write_png(frames / "frame_000002.png")
    manifest = {
        "artifacts": {
            "events": None,
            "frames": str(frames),
            "screenshot": str(screenshot),
        },
        "force_drag": None,
        "scene": scene,
        "schema_version": 1,
        "show_ui": True,
        "switch_frame": None,
        "switch_scene": None,
        "ui_ready": {
            "dropped_warmup_frames": 0,
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
    screenshot = tmp_path / "capture" / "avbd_articulated_compliant_motors.png"
    image: dict[str, object] = {
        "height": 3,
        "path": "avbd_articulated_compliant_motors.png",
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
                    "contrast": {"pass": False},
                    "non_blank": {"pass": True},
                },
                "image": image,
                "machine_scope": "pixel-integrity",
                "metadata": {
                    "scene": "avbd_articulated_compliant_motors",
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
        "middle": frames / "frame_000001.png",
        "end": frames / "frame_000002.png",
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
                "limitations": "Images corroborate bounded motion but do not measure row residuals.",
                "observations": "All three bodies remain finite and aligned with their visible guides.",
                "reviewer_capability": "native image input",
                "scene": "avbd_articulated_compliant_motors",
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
    bad_compliant_count_arg: int | None = None,
) -> Path:
    rows = []
    for arg in (1, 4, 16):
        if arg == missing_arg:
            continue
        compliant_motors = 2 * arg
        if arg == bad_compliant_count_arg:
            compliant_motors += 1
        rows.append(
            {
                **VARIATIONAL_RUNTIME_COUNTERS,
                "aggregate_name": "median",
                "compliant_motors": float(compliant_motors),
                "cpu_time": float(1000 * arg),
                "family_instances": float(arg),
                "iterations": 10,
                "name": f"BM_AvbdArticulatedCompliantMotorStep/{arg}_median",
                "prismatic_motors": float(arg),
                "real_time": float(1100 * arg),
                "revolute_motors": float(arg),
                "run_name": f"BM_AvbdArticulatedCompliantMotorStep/{arg}",
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
    parent_commit: str = "761263bbd41190536ce699439ebc64f5776757b6",
) -> Path:
    path = tmp_path / "exact_parent_mutation.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "dart.exact_parent_mutation/v1",
                "parent_commit": parent_commit,
                "test": (
                    "VariationalIntegration."
                    "AvbdCompliantPublicArticulatedOneDofMotors"
                    "DriveMovableLinkPairs"
                ),
                "mutation": (
                    "Apply the candidate behavior test to the exact parent "
                    "without the solver implementation."
                ),
                "command": (
                    "./test_variational_integration "
                    "--gtest_filter=VariationalIntegration."
                    "AvbdCompliantPublicArticulatedOneDofMotors"
                    "DriveMovableLinkPairs"
                ),
                "exit_code": 1,
                "expected_result": "fail",
                "observed_result": "fail",
                "failed_assertions": [
                    {
                        "coordinate": coordinate,
                        "actual": 0.0,
                        "expected": expected,
                    }
                    for coordinate, expected in (
                        ("strong finite revolute motor angle", 0.04),
                        ("strong finite prismatic motor travel", 0.03),
                    )
                ],
                "interpretation": (
                    "The exact parent skips the finite articulated rows."
                ),
            }
        ),
        encoding="utf-8",
    )
    return path


def test_avbd_articulated_compliant_motors_packet_records_evidence(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["schema_version"] == module.AVBD_PACKET_SCHEMA_VERSION
    assert packet["scene"] == "avbd_articulated_compliant_motors"
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "multibody_integration_family": "variational",
        "recorded_from": (
            "articulated compliant-motor benchmark runtime identity counters"
        ),
        "rigid_contact_selection": "contact_solver_method",
        "rigid_contact_solver": "sequential_impulse",
        "rigid_point_joint_solver": "sequential_impulse",
    }
    assert packet["scene_invariants"]["motor_types"] == ["revolute", "prismatic"]
    assert packet["scene_invariants"]["endpoint_topology"] == (
        "same-multibody movable-link pairs"
    )
    assert packet["target"]["contract_rows"] == [
        "avbd.method.joints_and_attachments",
        "avbd.method.finite_stiffness_ramping",
        "avbd.method.motors",
    ]
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["benchmark"]["benchmark"] == (
        "BM_AvbdArticulatedCompliantMotorStep"
    )
    assert [row["compliant_motors"] for row in packet["benchmark"]["scale_data"]] == [
        2,
        8,
        32,
    ]
    assert packet["benchmark"]["scale_data"][0]["cpu_time_per_motor_ns"] == (
        pytest.approx(1000.0 / 2.0)
    )
    assert packet["correctness"]["exact_parent_mutation"]["parent_commit"] == (
        "761263bbd41190536ce699439ebc64f5776757b6"
    )
    assert packet["correctness"]["exact_parent_mutation"]["observed_result"] == (
        "fail"
    )
    assert "finite articulated one-DOF velocity-motor coupling" not in packet[
        "remaining_gates"
    ]
    assert "same-multibody finite articulated endpoint pairs" not in packet[
        "remaining_gates"
    ]
    assert (
        "finite-row break-force accounting and fracture lifecycle"
        in packet["remaining_gates"]
    )


def test_avbd_articulated_compliant_motors_packet_rejects_wrong_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(
        tmp_path, scene="avbd_articulated_high_ratio_chain"
    )
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)

    with pytest.raises(
        SystemExit,
        match="capture scene must be avbd_articulated_compliant_motors",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )


def test_avbd_articulated_compliant_motors_packet_rejects_undocked_capture(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(
        tmp_path, docked_workspace=False
    )
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)

    with pytest.raises(
        SystemExit,
        match="capture must verify the docked workspace",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )


def test_avbd_articulated_compliant_motors_packet_rejects_missing_scale_row(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, missing_arg=4)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)

    with pytest.raises(
        SystemExit,
        match="missing compliant-motor scale rows: 4",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )


def test_avbd_articulated_compliant_motors_packet_rejects_wrong_counter(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, bad_compliant_count_arg=16)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)

    with pytest.raises(
        SystemExit,
        match="expected compliant_motors=32, got 33",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )


def test_avbd_articulated_compliant_motors_packet_rejects_wrong_parent(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(
        tmp_path, parent_commit="deadbeef"
    )

    with pytest.raises(
        SystemExit,
        match="exact-parent mutation parent_commit must be 761263bbd4",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )


def test_avbd_articulated_compliant_motors_packet_binds_image_verdict_sha256(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    screenshot = tmp_path / "capture" / "avbd_articulated_compliant_motors.png"
    recorded = packet["visual_capture"]["image_verdict"]["image_sha256"]
    assert recorded == _sha256(screenshot)
    assert recorded == packet["visual_capture"]["screenshot"]["sha256"]


def test_avbd_articulated_compliant_motors_packet_rejects_foreign_image_verdict(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path, image_sha256="0" * 64)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)

    with pytest.raises(
        SystemExit,
        match="image verdict image sha256 does not match capture screenshot",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )


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
            "image verdict image sha256 must use canonical lowercase hexadecimal",
        ),
    ],
)
def test_avbd_articulated_compliant_motors_packet_rejects_malformed_image_sha256(
    tmp_path: Path,
    verdict_kwargs: dict[str, object],
    expected_message: str,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    image_verdict = _write_image_verdict(tmp_path, **verdict_kwargs)
    visual_review = _write_visual_review(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    exact_parent_mutation = _write_exact_parent_mutation(tmp_path)

    with pytest.raises(SystemExit, match=expected_message):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--image-verdict-json",
                str(image_verdict),
                "--visual-review-json",
                str(visual_review),
                "--benchmark-json",
                str(benchmark_json),
                "--exact-parent-mutation-json",
                str(exact_parent_mutation),
            ]
        )
