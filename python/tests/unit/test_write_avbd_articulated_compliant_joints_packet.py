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
    ROOT / "scripts" / "write_avbd_articulated_compliant_joints_packet.py"
)


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_articulated_compliant_joints_packet",
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
    scene: str = "avbd_articulated_compliant_joints",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_articulated_compliant_joints.png"
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
        "show_ui": False,
        "switch_frame": None,
        "switch_scene": None,
        "ui_ready": {
            "dropped_warmup_frames": 0,
            "required": False,
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
                    "contrast": {"pass": False},
                    "non_blank": {"pass": True},
                },
                "image": {
                    "height": 3,
                    "path": "avbd_articulated_compliant_joints.png",
                    "width": 4,
                },
                "machine_scope": "pixel-integrity",
                "metadata": {
                    "scene": "avbd_articulated_compliant_joints",
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
                "scene": "avbd_articulated_compliant_joints",
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
    bad_compliant_count_arg: int | None = None,
) -> Path:
    rows = []
    for arg in (1, 4, 16):
        if arg == missing_arg:
            continue
        compliant_joints = 3 * arg
        if arg == bad_compliant_count_arg:
            compliant_joints += 1
        rows.append(
            {
                "aggregate_name": "median",
                "compliant_joints": float(compliant_joints),
                "cpu_time": float(1000 * arg),
                "family_instances": float(arg),
                "iterations": 10,
                "name": f"BM_AvbdArticulatedCompliantJointStep/{arg}_median",
                "prismatic_joints": float(arg),
                "real_time": float(1100 * arg),
                "revolute_joints": float(arg),
                "run_name": f"BM_AvbdArticulatedCompliantJointStep/{arg}",
                "run_type": "aggregate",
                "spherical_joints": float(arg),
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
    parent_commit: str = "0b0154573b842c142d31ef9feef4567c5677c4fd",
) -> Path:
    path = tmp_path / "exact_parent_mutation.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "dart.exact_parent_mutation/v1",
                "parent_commit": parent_commit,
                "test": (
                    "VariationalIntegration."
                    "AvbdCompliantPublicArticulatedJointMasks"
                    "ResistOnlyConstrainedMotion"
                ),
                "mutation": (
                    "Apply the candidate behavior test to the exact parent "
                    "without the solver implementation."
                ),
                "command": (
                    "./test_variational_integration "
                    "--gtest_filter=VariationalIntegration."
                    "AvbdCompliantPublicArticulatedJointMasks"
                    "ResistOnlyConstrainedMotion"
                ),
                "exit_code": 1,
                "expected_result": "fail",
                "observed_result": "fail",
                "failed_assertions": [
                    {
                        "coordinate": coordinate,
                        "actual": 1.0,
                        "upper_bound": 0.5,
                    }
                    for coordinate in (
                        "spherical constrained translation",
                        "revolute constrained translation",
                        "revolute constrained axis tilt",
                        "prismatic constrained transverse translation",
                        "prismatic constrained rotation",
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


def test_avbd_articulated_compliant_joints_packet_records_evidence(
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
    assert packet["schema_version"] == 3
    assert packet["scene"] == "avbd_articulated_compliant_joints"
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": (
            "contact-free py-demo and articulated compliant-joint "
            "benchmark row family"
        ),
        "rigid_contact_selection": "not_applicable",
        "rigid_contact_solver": "none",
        "rigid_point_joint_solver": "avbd",
    }
    assert packet["scene_invariants"]["joint_types"] == [
        "spherical",
        "revolute",
        "prismatic",
    ]
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["benchmark"]["benchmark"] == (
        "BM_AvbdArticulatedCompliantJointStep"
    )
    assert [row["compliant_joints"] for row in packet["benchmark"]["scale_data"]] == [
        3,
        12,
        48,
    ]
    assert packet["benchmark"]["scale_data"][0]["cpu_time_per_joint_ns"] == (
        pytest.approx(1000.0 / 3.0)
    )
    assert packet["correctness"]["exact_parent_mutation"]["parent_commit"] == (
        "0b0154573b842c142d31ef9feef4567c5677c4fd"
    )
    assert packet["correctness"]["exact_parent_mutation"]["observed_result"] == (
        "fail"
    )
    assert "finite articulated one-DOF velocity-motor coupling" in packet[
        "remaining_gates"
    ]


def test_avbd_articulated_compliant_joints_packet_rejects_wrong_scene(
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
        match="capture scene must be avbd_articulated_compliant_joints",
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


def test_avbd_articulated_compliant_joints_packet_rejects_missing_scale_row(
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
        match="missing compliant-joint scale rows: 4",
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


def test_avbd_articulated_compliant_joints_packet_rejects_wrong_counter(
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
        match="expected compliant_joints=48, got 49",
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


def test_avbd_articulated_compliant_joints_packet_rejects_wrong_parent(
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
        match="exact-parent mutation parent_commit must be 0b0154573b8",
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
