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
        "status": "pre-evaluation",
        "threshold_checks": {"finite_state": True},
        "thresholds_pass": False,
        "total_retained_fraction": 1.0,
        "unbroken_joints": 712,
        "world_time": frame / 60.0,
    }
    if frame == 14:
        return generic | {
            "bent_brick_count": 131,
            "contact_count": 26,
            "evaluated": True,
            "max_brick_displacement": 0.22961765643338428,
            "maximum_wall_normal_displacement": 0.22961612378147464,
            "rms_wall_normal_displacement": 0.12274369318326252,
            "status": "pass",
            "threshold_checks": dict(
                module.CHECKPOINTS[14]["threshold_checks"]
            ),
            "thresholds_pass": True,
        }
    if frame == 120:
        return generic | {
            "checkpoint": "retention",
            "evaluated": True,
            "max_brick_displacement": 0.0012,
            "maximum_wall_normal_displacement": 0.00086,
            "rms_wall_normal_displacement": 0.00047,
            "status": "pass",
            "threshold_checks": dict(
                module.CHECKPOINTS[120]["threshold_checks"]
            ),
            "thresholds_pass": True,
        }
    return generic


def _metrics(module, frame: int) -> dict[str, Any]:
    return {
        "ball_count": 3,
        "break_force": 8500.0,
        "breakable_joints": 712,
        "brick_count": 252,
        "collision_shapes": 256,
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
                "azimuth": -1.5707963267948966,
                "distance": 11.0,
                "elevation": 0.0,
                "target": [0.0, 0.0, 1.4],
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
    bend_manifest, bend_verdict = _write_capture(
        module,
        tmp_path,
        14,
        "bend",
    )
    retention_manifest, retention_verdict = _write_capture(
        module,
        tmp_path,
        120,
        "retention",
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

    bend_screenshot = (
        bend_manifest.parent / f"{module.SCENE_ID}_bend.png"
    )
    retention_screenshot = (
        retention_manifest.parent / f"{module.SCENE_ID}_retention.png"
    )
    review = {
        "claim_and_expected_observation": "VBD bends without fracture.",
        "inspected_images": [
            {
                "file": str(bend_screenshot),
                "role": "bend_frame_14",
                "sha256": _sha256(bend_screenshot),
            },
            {
                "file": str(retention_screenshot),
                "role": "retention_frame_120",
                "sha256": _sha256(retention_screenshot),
            },
            {
                "file": str(paper_figure),
                "role": "paper_figure_13_reference",
                "sha256": _sha256(paper_figure),
            },
        ],
        "not_proven_and_limitations": (
            "Exact unpublished constants, Sequential Impulse, XPBD, and CUDA "
            "remain unproven."
        ),
        "reconciliation_and_verdict": (
            "PASS: both ViewReports and both numeric checkpoints pass."
        ),
        "reviewer_capability": "native image input at original detail",
        "scene": module.SCENE_ID,
        "schema_version": "dart.visual_semantic_review/v1",
        "text_oracle": "Figure 13 says VBD bends but does not break.",
        "verdict": "pass",
        "visible_observation": "The wall bends and later remains intact.",
    }
    review_path = tmp_path / "visual_review.json"
    review_path.write_text(json.dumps(review), encoding="utf-8")

    impact_hash = "1" * 64
    outcome_hash = "2" * 64
    semantic_hash = "3" * 64
    paper_figure_hash = _sha256(paper_figure)
    avbd_visual = {
        "impact": {
            "image_verdict": {"pass": True, "sha256": "4" * 64},
            "scene_metrics": {
                "frame": 60,
                "outcome": dict(module.shared.EXPECTED_OUTCOMES[60])
                | {
                    "frame": 60,
                    "last_step_iterations": 20,
                },
                "outcome_oracle": dict(module.shared.OUTCOME_ORACLE),
                "scene_spec_fingerprint": "0123456789abcdef",
            },
            "screenshot": {"file": "impact.png", "sha256": impact_hash},
        },
        "outcome": {
            "image_verdict": {"pass": True, "sha256": "5" * 64},
            "scene_metrics": {
                "frame": 120,
                "outcome": dict(module.shared.EXPECTED_OUTCOMES[120])
                | {
                    "frame": 120,
                    "last_step_iterations": 20,
                },
                "outcome_oracle": dict(module.shared.OUTCOME_ORACLE),
                "scene_spec_fingerprint": "0123456789abcdef",
            },
            "screenshot": {"file": "outcome.png", "sha256": outcome_hash},
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

    assert packet["schema_version"] == 3
    assert packet["resolved_solver_identity"]["rigid_contact_solver"] == "vbd"
    assert packet["linked_avbd_evidence"]["resolved_solver_identity"][
        "rigid_contact_solver"
    ] == "avbd"
    assert (
        packet["visual_evidence"]["bend"]["scene_metrics"]["outcome"][
            "bent_brick_count"
        ]
        == 131
    )
    assert packet["visual_evidence"]["retention"]["scene_metrics"]["outcome"][
        "unbroken_joints"
    ] == 712
    assert packet["benchmark"]["comparison"][
        "vbd_to_avbd_median_cpu_cost_ratio"
    ] == pytest.approx(1.25)
    assert packet["correctness"]["determinism_test"] == (
        "test_vbd_paper_breakable_wall_checkpoints_are_deterministic"
    )
    assert packet["publication_observation"].endswith("does not break.")
    assert packet["target"]["contract_rows"] == ["avbd.paper.fig.13"]
    assert packet["target"]["complete_paper_reproduction"] is False
    provenance_paths = {
        entry["path"] for entry in packet["source_provenance"]["files"]
    }
    assert {
        "dart/simulation/detail/world_step_schedule.hpp",
        "python/tests/integration/test_demos_cycle.py",
        "tests/unit/simulation/world/test_world.cpp",
    } <= provenance_paths
    assert str(tmp_path) not in json.dumps(packet)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("solver", "capture resolved_solver_identity"),
        ("fracture", "broken_joints"),
        ("benchmark", "expected aggregates"),
        ("linked_hash", "linked AVBD benchmark JSON hash"),
        ("linked_outcome", "frame 120 outcome status"),
        ("linked_screenshot", "semantic review impact_frame_60 sha256"),
        ("linked_review_hash", "semantic review sha256"),
        ("limitations", "limitations must name xpbd"),
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
    elif mutation == "fracture":
        path = inputs["bend_capture_manifest"]
        data = json.loads(path.read_text())
        data["scene_metrics"]["latest"]["metrics"]["outcome"][
            "broken_joints"
        ] = 1
        path.write_text(json.dumps(data), encoding="utf-8")
        events_path = Path(data["artifacts"]["scene_metrics_events"])
        events = [
            json.loads(line)
            for line in events_path.read_text().splitlines()
            if line
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
    elif mutation == "linked_screenshot":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["impact"]["screenshot"]["sha256"] = "6" * 64
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "linked_review_hash":
        path = inputs["avbd_packet"]
        data = json.loads(path.read_text())
        data["visual_evidence"]["semantic_review"]["sha256"] = "replaced"
        path.write_text(json.dumps(data), encoding="utf-8")
    elif mutation == "limitations":
        path = inputs["visual_review_json"]
        data = json.loads(path.read_text())
        data["not_proven_and_limitations"] = (
            "Exact unpublished constants, Sequential Impulse, and CUDA remain "
            "unproven."
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
