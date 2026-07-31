from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
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


def _write_png(path: Path, width: int = 8, height: int = 6) -> None:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    rows = b"".join(
        b"\x00" + bytes((32 + row, 64, 128)) * width for row in range(height)
    )
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


def _sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _outcome(frame: int) -> dict[str, Any]:
    expected = {
        60: {
            "bands": [3, 9, 4],
            "broken": 359,
            "damage": False,
            "evaluated": False,
            "outside": 0.9337016574585635,
            "status": "pre-evaluation",
            "thresholds_pass": False,
            "total": 0.8611111111111112,
            "unbroken": 353,
        },
        120: {
            "bands": [4, 10, 6],
            "broken": 359,
            "damage": True,
            "evaluated": True,
            "outside": 0.9116022099447514,
            "status": "pass",
            "thresholds_pass": True,
            "total": 0.8253968253968254,
            "unbroken": 353,
        },
    }.get(
        frame,
        {
            "bands": [0, 0, 0],
            "broken": 0,
            "damage": False,
            "evaluated": False,
            "outside": 1.0,
            "status": "pre-evaluation",
            "thresholds_pass": False,
            "total": 1.0,
            "unbroken": 712,
        },
    )
    return {
        "ball_positions": [[0.0, 1.0, 2.0]] * 3,
        "ball_velocities": [[3.0, 4.0, 5.0]] * 3,
        "broken_joints": expected["broken"],
        "contact_count": 12,
        "evaluated": expected["evaluated"],
        "frame": frame,
        "impact_band_displaced_counts": expected["bands"],
        "last_step_iterations": 20,
        "max_brick_displacement": 3.0,
        "outside_brick_count": 181,
        "outside_retained_fraction": expected["outside"],
        "status": expected["status"],
        "threshold_checks": {
            "finite_state": True,
            "fracture_activated": True,
            "fracture_localized": True,
            "outside_wall_retained": True,
            "damage_in_three_impact_bands": expected["damage"],
            "total_wall_retained": True,
        },
        "thresholds_pass": expected["thresholds_pass"],
        "total_retained_fraction": expected["total"],
        "unbroken_joints": expected["unbroken"],
        "world_time": frame / 60.0,
    }


def _metrics(module, frame: int) -> dict[str, Any]:
    return {
        "ball_count": 3,
        "break_force": 8500.0,
        "breakable_joints": 712,
        "brick_count": 252,
        "collision_shapes": 256,
        "executor": "World.step default",
        "outcome": _outcome(frame),
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
                "azimuth": -1.5707963267948966,
                "distance": 11.0,
                "elevation": 0.0,
                "target": [0.0, 0.0, 1.4],
            },
            "focus": [
                "avbd_paper_wall_brick_00_00_visual",
                "avbd_paper_wall_brick_00_20_visual",
                "avbd_paper_wall_brick_11_00_visual",
                "avbd_paper_wall_brick_11_20_visual",
            ],
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
    manifest = {
        "artifacts": {
            "events": None,
            "frames": str(directory / "png_frames"),
            "scene_metrics_events": str(metrics_log),
            "screenshot": str(screenshot),
            "video": None,
        },
        "camera": {
            "distance": 11.0,
            "target": [0.0, 0.0, 1.4],
            "view": "front",
        },
        "capture": {
            "converted_frames": frame,
            "height": 6,
            "requested_frames": frame,
            "width": 8,
        },
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
            "contrast": {"pass": frame == 120},
            "non_blank": {"pass": True},
        },
        "image": {
            "height": 6,
            "path": str(screenshot),
            "width": 8,
        },
        "machine_scope": "pixel-integrity",
        "metadata": {
            "frame": str(frame),
            "scene": module.SCENE_ID,
            "view": "front",
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
                    "impacting_balls": 3.0,
                    "public_avbd_family": 1.0,
                    "resolved_rigid_body_avbd": 1.0,
                    "resolved_rigid_constraint_iterations": 1.0,
                    "resolved_rigid_contact_avbd": 1.0,
                    "resolved_rigid_pair_constraint_avbd": 1.0,
                    "rigid_constraint_iterations": 20.0,
                    "rigid_bodies": 256.0,
                    "rigid_body_joints": 712.0,
                    "runtime_contract_passed": 1.0,
                    "scene_spec_fingerprint_hi": float(fingerprint >> 32),
                    "scene_spec_fingerprint_lo": float(
                        fingerprint & 0xFFFFFFFF
                    ),
                    "trajectory_frames": 120.0,
                }
            )
        rows.append(row)
    path = tmp_path / "benchmark.json"
    path.write_text(
        json.dumps(
            {
                "benchmarks": rows,
                "context": {
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
        ),
        encoding="utf-8",
    )
    return path


def _write_inputs(module, tmp_path: Path, monkeypatch) -> dict[str, Path]:
    impact_manifest, impact_verdict = _write_capture(
        module, tmp_path, 60, "impact"
    )
    outcome_manifest, outcome_verdict = _write_capture(
        module, tmp_path, 120, "outcome"
    )
    paper_pdf = tmp_path / "avbd.pdf"
    paper_pdf.write_bytes(b"test AVBD paper")
    paper_figure = tmp_path / "paper-10.png"
    _write_png(paper_figure, width=10, height=12)
    monkeypatch.setattr(module, "PAPER_PDF_SHA256", _sha256(paper_pdf))
    monkeypatch.setattr(module, "PAPER_FIGURE_SHA256", _sha256(paper_figure))

    impact_screenshot = (
        impact_manifest.parent / f"{module.SCENE_ID}_impact.png"
    )
    outcome_screenshot = (
        outcome_manifest.parent / f"{module.SCENE_ID}_outcome.png"
    )
    review = {
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
                "file": str(paper_figure),
                "role": "paper_figure_13_reference",
                "sha256": _sha256(paper_figure),
            },
        ],
        "claim_and_expected_observation": (
            "Three impact bands should show damage while large outside wall "
            "regions remain attached and standing."
        ),
        "not_proven_and_limitations": (
            "The images do not prove exact unpublished source constants, "
            "other solver rows, CUDA parity, or timing parity."
        ),
        "reconciliation_and_verdict": (
            "PASS: both engine ViewReports pass, the numeric oracle passes at "
            "frame 120, and the visible retained-wall damage agrees."
        ),
        "reviewer_capability": "native image input at original detail",
        "scene": module.SCENE_ID,
        "schema_version": "dart.visual_semantic_review/v1",
        "text_oracle": (
            "At frame 120 all finite, fracture, impact-band damage, and "
            "retention threshold checks pass."
        ),
        "verdict": "pass",
        "visible_observation": (
            "The fixed front views show damage at each target band and "
            "substantial upper and side wall regions still standing."
        ),
    }
    review_path = tmp_path / "visual_review.json"
    review_path.write_text(json.dumps(review), encoding="utf-8")
    return {
        "benchmark_json": _write_benchmark(module, tmp_path),
        "impact_capture_manifest": impact_manifest,
        "impact_image_verdict_json": impact_verdict,
        "outcome_capture_manifest": outcome_manifest,
        "outcome_image_verdict_json": outcome_verdict,
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

    assert packet["schema_version"] == 3
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": (
            "engine World.resolved_configuration in both captures plus "
            "benchmark runtime resolved-configuration counters"
        ),
        "rigid_contact_selection": "world_solver_family",
        "rigid_contact_solver": "avbd",
        "rigid_point_joint_solver": "avbd",
    }
    assert packet["visual_evidence"]["impact"]["scene_metrics"]["frame"] == 60
    assert (
        packet["visual_evidence"]["outcome"]["scene_metrics"]["outcome"]["status"]
        == "pass"
    )
    assert packet["benchmark"]["stability"]["repetitions"] == 5
    assert packet["benchmark"]["scene_spec_fingerprint"] == "0123456789abcdef"
    assert packet["source_provenance"]["files"]
    assert packet["target"]["complete_paper_reproduction"] is False
    rendered = json.dumps(packet)
    assert str(tmp_path) not in rendered
    assert "no reference ratio or speedup claim is made" in rendered


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("camera", "serialized camera"),
        ("view_report_gate", "ambiguity_iou fails"),
        ("solver", "resolve public_avbd"),
        ("oracle", "outcome threshold_checks"),
        ("benchmark", "public_avbd_family"),
        ("events_empty", "expected 60 scene metric events"),
        ("events_unrelated", "event must be"),
        ("events_count", "expected 60 scene metric events"),
        ("events_order", "frames must be exactly"),
        ("events_inner_frame", "outcome frame"),
        ("events_inner_time", "outcome world_time"),
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
    elif mutation == "view_report_gate":
        path = inputs["impact_capture_manifest"]
        data = json.loads(path.read_text())
        data["scene_metrics"]["latest"]["metrics"]["view_report"]["metrics"][
            "ambiguity_iou"
        ] = 0.75
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line)
            for line in events_path.read_text().splitlines()
            if line
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
            "fracture_localized"
        ] = False
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line)
            for line in events_path.read_text().splitlines()
            if line
        ]
        events[-1]["metrics"]["outcome"]["threshold_checks"][
            "fracture_localized"
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
    elif mutation.startswith("events_"):
        manifest = json.loads(inputs["impact_capture_manifest"].read_text())
        events_path = Path(manifest["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line)
            for line in events_path.read_text().splitlines()
            if line
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
