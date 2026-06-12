from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "capture_py_demo", _ROOT / "scripts" / "capture_py_demo.py"
)
assert _SPEC is not None
assert _SPEC.loader is not None
capture_py_demo = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(capture_py_demo)


def _write_ppm(
    path: pathlib.Path, pixels: bytes, width: int = 2, height: int = 1
) -> None:
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)


def _workspace_pixels(width: int = 320, height: int = 240) -> bytes:
    pixels = bytearray([150] * width * height * 3)

    def fill(x0: int, x1: int, y0: int, y1: int, value: int) -> None:
        for y in range(y0, y1):
            for x in range(x0, x1):
                offset = (y * width + x) * 3
                pixels[offset : offset + 3] = bytes([value, value, value])

    fill(0, width, 0, int(height * 0.18), 25)
    fill(0, int(width * 0.24), int(height * 0.22), int(height * 0.90), 45)
    fill(int(width * 0.73), width, int(height * 0.22), int(height * 0.90), 45)
    fill(0, width, int(height * 0.89), height, 25)
    fill(
        int(width * 0.30),
        int(width * 0.68),
        int(height * 0.30),
        int(height * 0.78),
        170,
    )
    return bytes(pixels)


def test_ppm_to_png_conversion_and_blank_guard(tmp_path: pathlib.Path) -> None:
    ppm = tmp_path / "capture.ppm"
    png = tmp_path / "capture.png"
    _write_ppm(ppm, bytes([0, 0, 0, 255, 0, 0]))

    assert capture_py_demo.ppm_has_nonzero_pixels(ppm)
    capture_py_demo.convert_ppm_to_png(ppm, png)
    assert png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")

    blank = tmp_path / "blank.ppm"
    _write_ppm(blank, bytes(6))
    assert not capture_py_demo.ppm_has_nonzero_pixels(blank)


def test_ppm_image_evidence_reports_machine_checkable_stats(
    tmp_path: pathlib.Path,
) -> None:
    ppm = tmp_path / "evidence.ppm"
    _write_ppm(
        ppm,
        bytes(
            [
                0,
                0,
                0,
                255,
                255,
                255,
                255,
                0,
                0,
                0,
                255,
                0,
            ]
        ),
        width=2,
        height=2,
    )

    evidence = capture_py_demo.ppm_image_evidence(ppm)

    assert evidence["width"] == 2
    assert evidence["height"] == 2
    assert evidence["pixel_count"] == 4
    assert evidence["nonzero_pixels"] == 3
    assert evidence["nonzero_channels"] == 5
    assert evidence["unique_rgb_count"] == 4
    assert evidence["rgb_channel_variance"] > 0.0
    assert evidence["luminance_variance"] > 0.0
    assert evidence["docked_workspace"] is False


def test_show_ui_detector_rejects_scene_only_frames(
    tmp_path: pathlib.Path,
) -> None:
    no_ui = tmp_path / "no_ui.ppm"
    workspace = tmp_path / "workspace.ppm"
    width, height = 320, 240
    _write_ppm(no_ui, bytes([150] * width * height * 3), width, height)
    _write_ppm(workspace, _workspace_pixels(width, height), width, height)

    assert not capture_py_demo.ppm_has_docked_workspace_regions(no_ui)
    assert capture_py_demo.ppm_has_docked_workspace_regions(workspace)


def test_show_ui_frame_sequence_drops_warmup_frames(
    tmp_path: pathlib.Path,
) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    width, height = 320, 240
    _write_ppm(
        frames / "frame_000001.ppm",
        bytes([150] * width * height * 3),
        width,
        height,
    )
    _write_ppm(
        frames / "frame_000002.ppm", _workspace_pixels(width, height), width, height
    )
    _write_ppm(
        frames / "frame_000003.ppm", _workspace_pixels(width, height), width, height
    )

    ready_frames, dropped = capture_py_demo._prepare_frame_sequence(frames, True)

    assert dropped == 1
    assert [frame.name for frame in ready_frames] == [
        "frame_000001.ppm",
        "frame_000002.ppm",
    ]
    assert all(
        capture_py_demo.ppm_has_docked_workspace_regions(frame)
        for frame in ready_frames
    )


def test_visual_capture_manifest_records_image_evidence(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "capture"
    width, height = 320, 240
    workflow_guidance = {
        "workflow_index": 1,
        "workflow_count": 36,
        "workflow_label": "Baseline",
        "user_question": "What is the baseline DART 7 World rigid-body path?",
        "try_first": "Start here before moving to specialized rows.",
        "inspect": ["Solver/material controls", "Contacts, energy, step timing"],
        "healthy_signal": "Contacts settle and energy remains bounded.",
        "scope": "Baseline front door; focused edge cases stay specialized.",
    }

    def fake_run_demo(demo_args: list[str]) -> int:
        screenshot = pathlib.Path(
            demo_args[demo_args.index("--screenshot") + 1]
        )
        frames = pathlib.Path(demo_args[demo_args.index("--out") + 1])
        scene_metrics = pathlib.Path(
            demo_args[demo_args.index("--capture-metrics-event-log") + 1]
        )
        frames.mkdir(parents=True)
        _write_ppm(
            screenshot,
            _workspace_pixels(width, height),
            width,
            height,
        )
        _write_ppm(
            frames / "frame_000001.ppm",
            bytes([150] * width * height * 3),
            width,
            height,
        )
        _write_ppm(
            frames / "frame_000002.ppm",
            _workspace_pixels(width, height),
            width,
            height,
        )
        _write_ppm(
            frames / "frame_000003.ppm",
            _workspace_pixels(width, height),
            width,
            height,
        )
        scene_metrics.write_text(
            "\n".join(
                [
                    json.dumps(
                        {
                            "event": "scene_capture_metadata",
                            "metadata": {
                                "replay_timeline": {
                                    "has_markers": True,
                                    "has_signal": True,
                                    "panel": "Replay",
                                    "signal_label": "Diagnostic gap",
                                }
                            },
                            "scene": "rigid_body",
                            "source": "py-demo-scene",
                        },
                        sort_keys=True,
                    ),
                    json.dumps(
                        {
                            "event": "scene_capture_metrics",
                            "frame": 1,
                            "metrics": {
                                "contact_count": 0,
                                "status": "settling",
                                "step_ms": 1.2,
                            },
                            "scene": "rigid_body",
                            "source": "py-demo-scene",
                        },
                        sort_keys=True,
                    ),
                    json.dumps(
                        {
                            "event": "scene_capture_metrics",
                            "frame": 3,
                            "metrics": {"step_ms": 2.4, "status": "standing"},
                            "scene": "rigid_body",
                            "source": "py-demo-scene",
                        },
                        sort_keys=True,
                    ),
                ]
            )
            + "\n"
        )
        return 0

    monkeypatch.setattr(
        capture_py_demo,
        "_rigid_workflow_guidance_by_scene",
        lambda: {"rigid_body": workflow_guidance},
    )
    monkeypatch.setattr(capture_py_demo, "_run_demo", fake_run_demo)

    rc = capture_py_demo.main(
        [
            "--scene",
            "rigid_body",
            "--frames",
            "3",
            "--width",
            str(width),
            "--height",
            str(height),
            "--show-ui",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["scene_metadata"] == {
        "replay_timeline": {
            "has_markers": True,
            "has_signal": True,
            "panel": "Replay",
            "signal_label": "Diagnostic gap",
        }
    }
    assert manifest["workflow_guidance"] == workflow_guidance
    assert manifest["capture"] == {
        "converted_frames": 2,
        "height": height,
        "requested_frames": 3,
        "width": width,
    }
    assert manifest["ui_ready"] == {
        "dropped_warmup_frames": 1,
        "required": True,
    }
    assert pathlib.Path(manifest["artifacts"]["screenshot"]).is_file()
    assert pathlib.Path(manifest["artifacts"]["scene_metrics_events"]).is_file()
    assert len(list((output / "png_frames").glob("frame_*.png"))) == 2
    assert manifest["scene_metrics"] == {
        "event_count": 2,
        "first": {
            "event": "scene_capture_metrics",
            "frame": 1,
            "metrics": {
                "contact_count": 0,
                "status": "settling",
                "step_ms": 1.2,
            },
            "scene": "rigid_body",
            "source": "py-demo-scene",
        },
        "latest": {
            "event": "scene_capture_metrics",
            "frame": 3,
            "metrics": {"status": "standing", "step_ms": 2.4},
            "scene": "rigid_body",
            "source": "py-demo-scene",
        },
        "metric_key_counts": {
            "contact_count": 1,
            "status": 2,
            "step_ms": 2,
        },
        "numeric_ranges": {
            "contact_count": {"max": 0.0, "min": 0.0},
            "step_ms": {"max": 2.4, "min": 1.2},
        },
    }
    evidence = manifest["visual_evidence"]
    for key in ("screenshot", "first_frame"):
        stats = evidence[key]
        assert stats["width"] == width
        assert stats["height"] == height
        assert stats["pixel_count"] == width * height
        assert stats["nonzero_pixels"] == width * height
        assert stats["unique_rgb_count"] > 1
        assert stats["rgb_channel_variance"] > 0.0
        assert stats["luminance_variance"] > 0.0
        assert stats["docked_workspace"] is True


def test_visual_capture_manifest_records_video_artifact(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "capture"
    width, height = 2, 1

    def fake_run_demo(demo_args: list[str]) -> int:
        screenshot = pathlib.Path(demo_args[demo_args.index("--screenshot") + 1])
        frames = pathlib.Path(demo_args[demo_args.index("--out") + 1])
        scene_metrics = pathlib.Path(
            demo_args[demo_args.index("--capture-metrics-event-log") + 1]
        )
        frames.mkdir(parents=True)
        _write_ppm(screenshot, bytes([255, 0, 0, 0, 255, 0]), width, height)
        _write_ppm(
            frames / "frame_000001.ppm",
            bytes([255, 0, 0, 0, 255, 0]),
            width,
            height,
        )
        scene_metrics.write_text("")
        return 0

    def fake_encode_video(
        frames: pathlib.Path, video: pathlib.Path, fps: int
    ) -> bool:
        assert frames == output / "frames"
        assert fps == 12
        video.write_bytes(b"fake-mp4")
        return True

    monkeypatch.setattr(capture_py_demo, "_run_demo", fake_run_demo)
    monkeypatch.setattr(capture_py_demo, "_encode_video", fake_encode_video)

    rc = capture_py_demo.main(
        [
            "--scene",
            "rigid_body",
            "--frames",
            "1",
            "--width",
            str(width),
            "--height",
            str(height),
            "--video",
            "--fps",
            "12",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    video = pathlib.Path(manifest["artifacts"]["video"])
    assert video == output / "rigid_body.mp4"
    assert video.read_bytes() == b"fake-mp4"


def test_rigid_workflow_dry_run_writes_capture_plan(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (
        ("rigid_body", 24, 960, 540, True),
        ("rigid_solver_compare", 24, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        ["--rigid-workflow", "--dry-run", "--output-dir", str(output)]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["workflow"] == "rigid_visual_verification"
    assert manifest["include_related"] is False
    assert manifest["include_ipc_shelf"] is False
    assert manifest["include_packets"] is False
    assert manifest["selected_include_related"] is False
    assert manifest["selected_include_ipc_shelf"] is False
    assert manifest["selected_include_packets"] is False
    assert manifest["dry_run"] is True
    assert manifest["status"] == "planned"
    assert manifest["capture_count"] == len(specs)
    assert manifest["completed_count"] == 0
    assert manifest["failed_rows"] == []
    assert manifest["guidance_complete"] is True
    assert manifest["guidance_missing_count"] == 0
    assert manifest["guidance_missing_rows"] == []
    assert pathlib.Path(manifest["artifacts"]["review_index"]).is_file()
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_body",
        "rigid_solver_compare",
    ]
    assert manifest["captures"][0]["command"].startswith(
        "pixi run py-demo-capture -- --scene rigid_body"
    )
    assert (
        manifest["captures"][0]["viewer_command"]
        == "pixi run py-demos -- --scene rigid_body --width 960 --height 540"
    )
    assert (
        manifest["captures"][0]["workflow_rerun_command"]
        == "pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 1 "
        "--workflow-end-row 1 --output-dir "
        f"{output}/reruns/01_rigid_body"
    )
    assert manifest["captures"][0]["workflow_label"] == "Baseline"
    assert (
        manifest["captures"][0]["user_question"]
        == "What is the baseline DART 7 World rigid-body path?"
    )
    assert "Solver/material controls" in manifest["captures"][0]["inspect"]
    assert manifest["captures"][1]["workflow_label"] == "Solver family"
    assert manifest["captures"][0]["manifest"].endswith(
        "scenes/01_rigid_body/manifest.json"
    )
    review_index = pathlib.Path(manifest["artifacts"]["review_index"])
    review_html = review_index.read_text()
    assert "DART rigid workflow review index" in review_html
    assert "requested groups" in review_html
    assert "selected groups" in review_html
    assert "<strong>guidance</strong> complete" in review_html
    assert "Rows Missing Guidance" not in review_html
    assert "1-2 / 2" in review_html
    assert "numbered" in review_html
    assert "rerun workflow row" in review_html
    assert "--workflow-start-row 1 --workflow-end-row 1" in review_html
    assert f"{output}/reruns/01_rigid_body" in review_html
    assert "What is the baseline DART 7 World rigid-body path?" in review_html
    assert "Healthy: contacts settle" in review_html
    assert "rigid_solver_compare" in review_html
    assert "scenes/01_rigid_body/manifest.json" in review_html
    assert "open live" in review_html
    assert "capture evidence" in review_html
    assert "pixi run py-demos -- --scene rigid_body --width 960 --height 540" in (
        review_html
    )
    assert "pixi run py-demo-capture -- --scene rigid_body" in review_html


def test_rigid_workflow_dry_run_can_include_related_evidence(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_body", 24, 960, 540, True),)
    related_specs = (
        ("floating_base", 72, 960, 540, True),
        ("rigid_ipc_tunnel", 24, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_RELATED_CAPTURE_SPECS", related_specs
    )

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--include-related",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["include_related"] is True
    assert manifest["selected_include_related"] is True
    assert manifest["capture_count"] == len(specs) + len(related_specs)
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_body",
        "floating_base",
        "rigid_ipc_tunnel",
    ]
    assert [capture["workflow_group"] for capture in manifest["captures"]] == [
        "numbered",
        "related_evidence",
        "related_evidence",
    ]
    assert manifest["captures"][1]["workflow_label"] == "Related evidence"
    assert manifest["captures"][1]["related_source_row"] == "rigid_free_flight"
    assert manifest["captures"][1]["related_shelf"] == "World Rigid Body"
    assert "broader floating-joint row" in manifest["captures"][1]["user_question"]
    assert "floating-joint SE(3)" in manifest["captures"][1]["scope"]
    assert manifest["captures"][2]["related_source_row"] == "rigid_solver_compare"
    assert manifest["captures"][2]["related_shelf"] == "Rigid IPC"
    assert "focused no-tunneling view" in manifest["captures"][2]["user_question"]
    assert manifest["captures"][2]["manifest"].endswith(
        "scenes/03_rigid_ipc_tunnel/manifest.json"
    )
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "Related evidence" in review_html
    assert "broader floating-joint row" in review_html
    assert "focused no-tunneling view" in review_html
    assert "rigid_ipc_tunnel" in review_html
    assert "related_evidence" in review_html


def test_rigid_workflow_dry_run_can_include_direct_ipc_shelf(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_body", 24, 960, 540, True),)
    ipc_shelf_specs = (
        ("rigid_ipc", 72, 960, 540, True),
        ("rigid_ipc_pile", 72, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_IPC_SHELF_CAPTURE_SPECS", ipc_shelf_specs
    )

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--include-ipc-shelf",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["include_ipc_shelf"] is True
    assert manifest["selected_include_ipc_shelf"] is True
    assert manifest["capture_count"] == len(specs) + len(ipc_shelf_specs)
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_body",
        "rigid_ipc",
        "rigid_ipc_pile",
    ]
    assert [capture["workflow_group"] for capture in manifest["captures"]] == [
        "numbered",
        "rigid_ipc_shelf",
        "rigid_ipc_shelf",
    ]
    assert manifest["captures"][1]["workflow_label"] == "Rigid IPC shelf"
    assert "free box settle" in manifest["captures"][1]["user_question"]
    assert "barrier gap" in manifest["captures"][1]["inspect"]
    assert "Direct Rigid IPC shelf row" in manifest["captures"][1]["scope"]
    assert manifest["captures"][2]["workflow_label"] == "Rigid IPC shelf"
    assert "multi-box pile" in manifest["captures"][2]["user_question"]
    assert "minimum clearance" in manifest["captures"][2]["inspect"]
    assert manifest["captures"][2]["manifest"].endswith(
        "scenes/03_rigid_ipc_pile/manifest.json"
    )
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "Rigid IPC shelf" in review_html
    assert "Can direct Rigid IPC captures show multi-box pile" in review_html
    assert "Direct Rigid IPC shelf row" in review_html
    assert "rigid_ipc_pile" in review_html
    assert "rigid_ipc_shelf" in review_html


def test_rigid_workflow_dry_run_can_include_capture_first_packets(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_body", 24, 960, 540, True),)
    related_specs = (("rigid_ipc_tunnel", 24, 960, 540, True),)
    ipc_shelf_specs = (("rigid_ipc", 72, 960, 540, True),)
    packet_specs = (("rigid_ipc_stack_packet", 24, 960, 540, True),)
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_RELATED_CAPTURE_SPECS", related_specs
    )
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_IPC_SHELF_CAPTURE_SPECS", ipc_shelf_specs
    )
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_PACKET_CAPTURE_SPECS", packet_specs
    )

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--include-related",
            "--include-ipc-shelf",
            "--include-packets",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["include_related"] is True
    assert manifest["include_ipc_shelf"] is True
    assert manifest["include_packets"] is True
    assert manifest["selected_include_related"] is True
    assert manifest["selected_include_ipc_shelf"] is True
    assert manifest["selected_include_packets"] is True
    assert manifest["capture_count"] == (
        len(specs) + len(related_specs) + len(ipc_shelf_specs) + len(packet_specs)
    )
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_body",
        "rigid_ipc_tunnel",
        "rigid_ipc",
        "rigid_ipc_stack_packet",
    ]
    assert [capture["workflow_group"] for capture in manifest["captures"]] == [
        "numbered",
        "related_evidence",
        "rigid_ipc_shelf",
        "capture_first_packet",
    ]
    assert manifest["captures"][1]["workflow_label"] == "Related evidence"
    assert manifest["captures"][2]["workflow_label"] == "Rigid IPC shelf"
    assert manifest["captures"][3]["workflow_label"] == "Capture-first packet"
    assert (
        "four-box IPC stack"
        in manifest["captures"][3]["user_question"]
    )
    assert "benchmark pointer" in manifest["captures"][3]["inspect"]
    assert "solver-performance parity claim" in manifest["captures"][3]["scope"]
    assert manifest["captures"][3]["manifest"].endswith(
        "scenes/04_rigid_ipc_stack_packet/manifest.json"
    )
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "Capture-first packet" in review_html
    assert "four-box IPC stack" in review_html
    assert "solver-performance parity claim" in review_html
    assert "rigid_ipc_stack_packet" in review_html
    assert "capture_first_packet" in review_html


def test_rigid_workflow_dry_run_can_select_row_range(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (
        ("rigid_body", 24, 960, 540, True),
        ("rigid_solver_compare", 24, 960, 540, True),
        ("rigid_loop_closure", 72, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--workflow-start-row",
            "2",
            "--workflow-end-row",
            "2",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["capture_count"] == 1
    assert manifest["workflow_total_count"] == len(specs)
    assert manifest["workflow_row_start"] == 2
    assert manifest["workflow_row_end"] == 2
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_solver_compare"
    ]
    assert manifest["captures"][0]["order"] == 2
    assert manifest["captures"][0]["count"] == len(specs)
    assert manifest["captures"][0]["manifest"].endswith(
        "scenes/02_rigid_solver_compare/manifest.json"
    )
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "2/3 rigid_solver_compare" in review_html


def test_rigid_workflow_dry_run_can_request_video_commands(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_solver_compare", 24, 960, 540, True),)
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--backend",
            "opengl",
            "--video",
            "--fps",
            "12",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["capture_count"] == 1
    assert (
        manifest["captures"][0]["command"]
        == "pixi run py-demo-capture -- --scene rigid_solver_compare "
        "--frames 24 --width 960 --height 540 --output-dir "
        f"{output}/scenes/01_rigid_solver_compare --show-ui "
        "--backend opengl --video --fps 12"
    )
    assert (
        manifest["captures"][0]["viewer_command"]
        == "pixi run py-demos -- --scene rigid_solver_compare --width 960 "
        "--height 540 --backend opengl"
    )


def test_rigid_workflow_row_range_preserves_requested_extra_groups(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_body", 24, 960, 540, True),)
    related_specs = (("rigid_ipc_tunnel", 24, 960, 540, True),)
    ipc_shelf_specs = (("rigid_ipc", 72, 960, 540, True),)
    packet_specs = (("rigid_ipc_stack_packet", 24, 960, 540, True),)
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_RELATED_CAPTURE_SPECS", related_specs
    )
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_IPC_SHELF_CAPTURE_SPECS", ipc_shelf_specs
    )
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_PACKET_CAPTURE_SPECS", packet_specs
    )

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--include-related",
            "--include-ipc-shelf",
            "--include-packets",
            "--workflow-start-row",
            "3",
            "--workflow-end-row",
            "4",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["include_related"] is True
    assert manifest["include_ipc_shelf"] is True
    assert manifest["include_packets"] is True
    assert manifest["selected_include_related"] is False
    assert manifest["selected_include_ipc_shelf"] is True
    assert manifest["selected_include_packets"] is True
    assert manifest["capture_count"] == 2
    assert manifest["workflow_total_count"] == 4
    assert manifest["workflow_row_start"] == 3
    assert manifest["workflow_row_end"] == 4
    assert [capture["workflow_group"] for capture in manifest["captures"]] == [
        "rigid_ipc_shelf",
        "capture_first_packet",
    ]
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_ipc",
        "rigid_ipc_stack_packet",
    ]
    assert manifest["captures"][0]["workflow_label"] == "Rigid IPC shelf"
    assert manifest["captures"][1]["workflow_label"] == "Capture-first packet"
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "3-4 / 4" in review_html
    assert "numbered, related, ipc shelf, packets" in review_html
    assert "ipc shelf, packets" in review_html
    assert "Rigid IPC shelf" in review_html
    assert "Capture-first packet" in review_html


def test_rigid_workflow_full_extended_plan_has_complete_guidance(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--include-related",
            "--include-ipc-shelf",
            "--include-packets",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    expected_count = (
        len(capture_py_demo.rigid_workflow_capture_specs())
        + len(capture_py_demo.rigid_workflow_related_capture_specs())
        + len(capture_py_demo.rigid_workflow_ipc_shelf_capture_specs())
        + len(capture_py_demo.rigid_workflow_packet_capture_specs())
    )
    assert manifest["capture_count"] == expected_count
    assert manifest["workflow_total_count"] == expected_count
    assert manifest["guidance_complete"] is True
    assert manifest["guidance_missing_count"] == 0
    assert manifest["guidance_missing_rows"] == []
    for capture in manifest["captures"]:
        assert capture["workflow_label"]
        assert capture["user_question"]
        assert capture["try_first"]
        assert capture["inspect"]
        assert capture["healthy_signal"]
        assert capture["scope"]
        assert capture["viewer_command"].startswith("pixi run py-demos -- --scene ")

    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "<strong>guidance</strong> complete" in review_html
    assert "Rows Missing Guidance" not in review_html
    assert "open live" in review_html
    assert "capture evidence" in review_html
    assert "Related evidence" in review_html
    assert "Rigid IPC shelf" in review_html
    assert "Capture-first packet" in review_html
    heavy_packet = next(
        capture
        for capture in manifest["captures"]
        if capture["scene"] == "rigid_ipc_heavy_stack_packet"
    )
    assert heavy_packet["workflow_group"] == "capture_first_packet"
    assert heavy_packet["workflow_label"] == "Capture-first packet"
    assert "top-heavy IPC stack" in heavy_packet["user_question"]
    assert "top mass" in heavy_packet["inspect"]
    assert "Taller capture-first stress packet" in heavy_packet["scope"]
    assert "rigid_ipc_heavy_stack_packet" in review_html
    assert "top-heavy IPC stack" in review_html


def test_rigid_workflow_manifest_reports_missing_guidance(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("unlabeled_scene", 1, 320, 180, False),)
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)

    def fail_run(_argv: list[str]) -> int:
        raise AssertionError("dry-run should not render scenes")

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fail_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["guidance_complete"] is False
    assert manifest["guidance_missing_count"] == 1
    assert manifest["guidance_missing_rows"] == [
        {
            "order": 1,
            "scene": "unlabeled_scene",
            "workflow_group": "numbered",
            "missing_fields": [
                "workflow_label",
                "user_question",
                "try_first",
                "inspect",
                "healthy_signal",
                "scope",
            ],
        }
    ]
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "<strong>guidance</strong> missing 1" in review_html
    assert "Rows Missing Guidance" in review_html
    assert "unlabeled_scene" in review_html
    assert "workflow_label, user_question, try_first" in review_html


def test_rigid_workflow_run_aggregates_scene_manifests(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (
        ("rigid_body", 24, 960, 540, True),
        ("rigid_executor_equivalence", 24, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    rendered: list[str] = []

    def fake_run(argv: list[str]) -> int:
        scene = argv[argv.index("--scene") + 1]
        frames = int(argv[argv.index("--frames") + 1])
        output_dir = pathlib.Path(argv[argv.index("--output-dir") + 1])
        output_dir.mkdir(parents=True)
        screenshot = output_dir / f"{scene}.png"
        screenshot.write_bytes(b"fake-png")
        (output_dir / "manifest.json").write_text(
            json.dumps(
                {
                    "scene": scene,
                    "capture": {"requested_frames": frames},
                    "artifacts": {
                        "frames": str(output_dir / "png_frames"),
                        "screenshot": str(screenshot),
                    },
                    "scene_metrics": {
                        "latest": {
                            "metrics": {
                                "comparison_axis": "test_axis",
                                "held_fixed": {
                                    "executor": "Sequential",
                                    "solver": "si",
                                },
                                "controls": {
                                    "friction": 0.72,
                                    "solver_index": 1,
                                },
                                "solver_pair": ["SEQUENTIAL_IMPULSE", "IPC"],
                                "executor_pair": [
                                    "Sequential",
                                    "Parallel (2 workers)",
                                ],
                                "position_divergence": 1.0e-9,
                                "divergence": {
                                    "current_x": 0.125,
                                    "max_x": 0.25,
                                    "samples": 2.0,
                                },
                            },
                        },
                        "metric_key_counts": {
                            "comparison_axis": 1,
                            "controls": 1,
                            "divergence": 1,
                            "executor_pair": 1,
                            "held_fixed": 1,
                            "position_divergence": 1,
                            "solver_pair": 1,
                        },
                    },
                    "scene_metadata": {
                        "replay_timeline": {
                            "has_markers": True,
                            "has_signal": True,
                            "panel": "Replay",
                            "signal_label": "Diagnostic gap",
                        }
                    },
                },
                sort_keys=True,
            )
            + "\n"
        )
        rendered.append(scene)
        return 0

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fake_run)

    rc = capture_py_demo.main(["--rigid-workflow", "--output-dir", str(output)])

    assert rc == 0
    assert rendered == ["rigid_body", "rigid_executor_equivalence"]
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["dry_run"] is False
    assert manifest["status"] == "complete"
    assert manifest["completed_count"] == len(specs)
    assert manifest["failed_count"] == 0
    assert [capture["status"] for capture in manifest["captures"]] == [
        "captured",
        "captured",
    ]
    assert all(capture["manifest_exists"] for capture in manifest["captures"])
    review_index = pathlib.Path(manifest["artifacts"]["review_index"])
    assert review_index.is_file()
    review_html = review_index.read_text()
    assert "scenes/01_rigid_body/rigid_body.png" in review_html
    assert review_html.count('alt="rigid_body screenshot"') == 1
    assert review_html.count('alt="rigid_executor_equivalence screenshot"') == 1
    assert "scenes/02_rigid_executor_equivalence/manifest.json" in review_html
    assert "test_axis" in review_html
    assert "Diagnostic gap (signal, markers)" in review_html
    assert "comparison_axis, controls, divergence, executor_pair" in review_html
    assert "executor=Sequential, solver=si" in review_html
    assert "friction=0.72, solver index=1" in review_html
    assert "latest signals" in review_html
    assert "solver pair: SEQUENTIAL_IMPULSE / IPC" in review_html
    assert "executor pair: Sequential / Parallel (2 workers)" in review_html
    assert "position divergence: 1e-09" in review_html
    assert "divergence: current x=0.125, max x=0.25, samples=2" in review_html


def test_rigid_workflow_latest_signals_prioritize_normal_push_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "case_pair": [
                "IPC normal-push caveat",
                "IPC heavy-target caveat",
                "Sequential impulse push",
            ],
            "executor": "Sequential",
            "ipc_heavy_max_depth": 0.125147,
            "ipc_normal_max_depth": 0.125147,
            "ipc_normal_target_travel": 0.0,
            "si_caveat_contact_count": 1,
            "si_caveat_target_travel": 0.122797,
            "solver": "ipc_penetration_caveat_vs_sequential_impulse_push",
            "solver_pair": [
                "IPC",
                "IPC",
                "SEQUENTIAL_IMPULSE",
            ],
            "target_travel_divergence": 0.122797,
        }
    )

    assert highlights[:6] == [
        "target travel divergence: 0.122797",
        "si caveat target travel: 0.122797",
        "ipc normal max depth: 0.125147",
        "solver pair: IPC / IPC / SEQUENTIAL_IMPULSE",
        (
            "case pair: IPC normal-push caveat / IPC heavy-target caveat / "
            "Sequential impulse push"
        ),
        "solver: ipc_penetration_caveat_vs_sequential_impulse_push",
    ]


def test_rigid_workflow_latest_signals_prioritize_contact_push_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "case_pair": ["Sequential impulse", "IPC barrier"],
            "executor": "Sequential",
            "ipc_contact_count": 2.0,
            "ipc_max_contact_count": 3.0,
            "ipc_min_gap": 0.004,
            "ipc_target_travel": 0.134,
            "sequential_impulse_contact_count": 1.0,
            "sequential_impulse_max_contact_count": 2.0,
            "sequential_impulse_target_travel": 0.142,
            "solver": "sequential_impulse_vs_ipc",
            "solver_pair": ["SEQUENTIAL_IMPULSE", "IPC"],
            "travel_divergence": 0.008,
        }
    )

    assert highlights[:6] == [
        "travel divergence: 0.008",
        "sequential impulse target travel: 0.142",
        "ipc target travel: 0.134",
        "sequential impulse max contact count: 2",
        "ipc min gap: 0.004",
        "solver pair: SEQUENTIAL_IMPULSE / IPC",
    ]


def test_rigid_workflow_latest_signals_prioritize_kinematic_driver_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "case_pair": [
                "IPC kinematic grip",
                "IPC low-friction slip",
                "Sequential impulse caveat",
            ],
            "executor": "Sequential",
            "ipc_grip_box_travel": 0.087,
            "ipc_grip_contact_count": 3.0,
            "ipc_grip_min_abs_support_gap": 0.0007,
            "ipc_grip_speed_ratio": 0.82,
            "ipc_slip_slip": 0.106,
            "si_caveat_driver_travel": 0.0,
            "solver": "ipc_kinematic_driver_with_si_caveat",
            "solver_pair": ["IPC", "IPC", "SEQUENTIAL_IMPULSE"],
        }
    )

    assert highlights[:6] == [
        "ipc grip box travel: 0.087",
        "ipc grip speed ratio: 0.82",
        "ipc slip slip: 0.106",
        "si caveat driver travel: 0",
        "ipc grip min abs support gap: 0.0007",
        "solver pair: IPC / IPC / SEQUENTIAL_IMPULSE",
    ]


def test_rigid_workflow_latest_signals_prioritize_fixed_joint_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "comparison_axis": "fixed_relative_transform_recovery",
            "constraint": "fixed_relative_transform",
            "fixed_joint_orientation_error": 0.00012,
            "fixed_joint_payload_angular_speed": 0.0034,
            "fixed_joint_payload_speed": 0.0023,
            "fixed_joint_translation_error": 0.00025,
            "solver": "sequential_rigid_joints",
        }
    )

    assert highlights[:5] == [
        "fixed joint translation error: 0.00025",
        "fixed joint orientation error: 0.00012",
        "fixed joint payload speed: 0.0023",
        "fixed joint payload angular speed: 0.0034",
        "solver: sequential_rigid_joints",
    ]


def test_rigid_workflow_latest_signals_prioritize_breakage_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "breakage_broken": 1.0,
            "breakage_captured_offset_error": 0.028,
            "breakage_payload_release_distance": 0.41,
            "breakage_payload_speed": 0.72,
            "breakage_status": "broken",
            "comparison_axis": "fixed_break_force_lifecycle",
            "solver": "avbd_rigid_joints",
        }
    )

    assert highlights[:6] == [
        "breakage payload release distance: 0.41",
        "breakage broken: 1",
        "breakage captured offset error: 0.028",
        "breakage payload speed: 0.72",
        "breakage status: broken",
        "solver: avbd_rigid_joints",
    ]


def test_rigid_workflow_latest_signals_prioritize_distance_spring_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "comparison_axis": "distance_spring_response_family",
            "distance_spring_free_abs_stretch": 0.33,
            "distance_spring_max_sprung_abs_stretch": 0.35,
            "distance_spring_offset_abs_stretch": 0.002,
            "distance_spring_offset_angular_speed": 1.96,
            "distance_spring_soft_abs_stretch": 0.113,
            "distance_spring_stiff_abs_stretch": 0.125,
            "solver": "sequential_impulse_avbd_distance_spring",
        }
    )

    assert highlights[:7] == [
        "distance spring free abs stretch: 0.33",
        "distance spring soft abs stretch: 0.113",
        "distance spring stiff abs stretch: 0.125",
        "distance spring offset abs stretch: 0.002",
        "distance spring offset angular speed: 1.96",
        "distance spring max sprung abs stretch: 0.35",
        "solver: sequential_impulse_avbd_distance_spring",
    ]


def test_rigid_workflow_latest_signals_prioritize_one_dof_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "comparison_axis": "one_dof_joint_constraint_family",
            "one_dof_hinge_angular_speed": 1.4,
            "one_dof_hinge_radius_error": 0.000001,
            "one_dof_hinge_yaw": 0.21,
            "one_dof_hinge_z_error": 0.000002,
            "one_dof_slider_axis_speed": 0.45,
            "one_dof_slider_axis_travel": 0.72,
            "one_dof_slider_orthogonal_error": 0.000003,
            "solver": "sequential_rigid_joints",
        }
    )

    assert highlights[:8] == [
        "one dof hinge radius error: 1e-06",
        "one dof hinge z error: 2e-06",
        "one dof slider orthogonal error: 3e-06",
        "one dof hinge yaw: 0.21",
        "one dof slider axis travel: 0.72",
        "one dof hinge angular speed: 1.4",
        "one dof slider axis speed: 0.45",
        "solver: sequential_rigid_joints",
    ]


def test_rigid_workflow_latest_signals_prioritize_joint_motor_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "comparison_axis": "world_multibody_actuator_limit_family",
            "joint_motor_expected_speed": 0.3,
            "joint_motor_force_acceleration_gap": 6.0,
            "joint_motor_force_position_gap": 0.6984,
            "joint_motor_position_limit_angle": 0.35,
            "joint_motor_position_limit_error": 0.0,
            "joint_motor_speed": 0.3,
            "joint_motor_speed_error": 0.0,
            "solver": "world_multibody_joint_actuators",
        }
    )

    assert highlights[:8] == [
        "joint motor speed: 0.3",
        "joint motor expected speed: 0.3",
        "joint motor speed error: 0",
        "joint motor position limit angle: 0.35",
        "joint motor position limit error: 0",
        "joint motor force position gap: 0.6984",
        "joint motor force acceleration gap: 6",
        "solver: world_multibody_joint_actuators",
    ]


def test_rigid_workflow_latest_signals_prioritize_passive_joint_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "executor": "Sequential",
            "passive_joint_armature_acceleration_gap": 2.4,
            "passive_joint_armature_position_gap": 0.37,
            "passive_joint_damped_energy": 0.52,
            "passive_joint_damped_energy_ratio": 0.24,
            "passive_joint_slip_speed": 0.73,
            "passive_joint_spring_energy": 2.16,
            "solver": "world_multibody_passive_joint_parameters",
        }
    )

    assert highlights[:8] == [
        "passive joint spring energy: 2.16",
        "passive joint damped energy: 0.52",
        "passive joint damped energy ratio: 0.24",
        "passive joint slip speed: 0.73",
        "passive joint armature position gap: 0.37",
        "passive joint armature acceleration gap: 2.4",
        "solver: world_multibody_passive_joint_parameters",
        "executor: Sequential",
    ]


def test_rigid_workflow_latest_signals_prioritize_screw_joint_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "executor": "Sequential",
            "screw_joint_coarse_fine_travel_gap": 0.14,
            "screw_joint_coarse_pitch": 0.56,
            "screw_joint_fine_acceleration_error": 0.0,
            "screw_joint_fine_pitch": 0.28,
            "screw_joint_reverse_angle": 0.83,
            "screw_joint_reverse_pitch": -0.28,
            "screw_joint_zero_pitch_axial_travel": 0.0,
            "solver": "world_multibody_screw_joint_pitch",
        }
    )

    assert highlights[:8] == [
        "screw joint zero pitch axial travel: 0",
        "screw joint fine pitch: 0.28",
        "screw joint coarse pitch: 0.56",
        "screw joint reverse pitch: -0.28",
        "screw joint coarse fine travel gap: 0.14",
        "screw joint reverse angle: 0.83",
        "screw joint fine acceleration error: 0",
        "solver: world_multibody_screw_joint_pitch",
    ]


def test_rigid_workflow_latest_signals_prioritize_multibody_dynamics_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "executor": "Sequential",
            "multibody_dynamics_coupled_coupling": 0.19,
            "multibody_dynamics_coupled_heavy_response_gap": 1.25,
            "multibody_dynamics_heavy_response_ratio": 0.42,
            "multibody_dynamics_heavy_tau_gap": 2.5,
            "multibody_dynamics_max_impulse_residual": 0.0,
            "multibody_dynamics_max_inverse_residual": 0.0,
            "multibody_dynamics_single_mass_diag0": 0.33,
            "solver": "world_multibody_dynamics_terms",
        }
    )

    assert highlights[:8] == [
        "multibody dynamics single mass diag0: 0.33",
        "multibody dynamics coupled coupling: 0.19",
        "multibody dynamics heavy tau gap: 2.5",
        "multibody dynamics coupled heavy response gap: 1.25",
        "multibody dynamics heavy response ratio: 0.42",
        "multibody dynamics max inverse residual: 0",
        "multibody dynamics max impulse residual: 0",
        "solver: world_multibody_dynamics_terms",
    ]


def test_rigid_workflow_latest_signals_prioritize_link_com_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "executor": "Sequential",
            "link_com_centered_gravity_torque": 0.0,
            "link_com_high_acceleration_ratio": 0.42,
            "link_com_high_mass_matrix_ratio": 2.25,
            "link_com_max_acceleration_error": 0.0,
            "link_com_negative_gravity_torque": -3.53,
            "link_com_positive_gravity_torque": 3.53,
            "link_com_positive_negative_angle_sum": 0.0,
            "solver": "world_multibody_inertial_offsets",
        }
    )

    assert highlights[:8] == [
        "link com centered gravity torque: 0",
        "link com positive gravity torque: 3.53",
        "link com negative gravity torque: -3.53",
        "link com positive negative angle sum: 0",
        "link com high mass matrix ratio: 2.25",
        "link com high acceleration ratio: 0.42",
        "link com max acceleration error: 0",
        "solver: world_multibody_inertial_offsets",
    ]


def test_rigid_workflow_latest_signals_prioritize_link_jacobian_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "link_jacobian_angular_speed": 0.62,
            "link_jacobian_finite_difference_error": 1.0e-8,
            "link_jacobian_linear_speed": 0.56,
            "link_jacobian_power_error": 0.0,
            "link_jacobian_tau0": -0.86,
            "link_jacobian_tau1": -0.29,
            "link_jacobian_world_body_gap": 0.13,
            "solver": "world_multibody_link_jacobian",
        }
    )

    assert highlights[:8] == [
        "link jacobian linear speed: 0.56",
        "link jacobian angular speed: 0.62",
        "link jacobian world body gap: 0.13",
        "link jacobian finite difference error: 1e-08",
        "link jacobian tau0: -0.86",
        "link jacobian tau1: -0.29",
        "link jacobian power error: 0",
        "solver: world_multibody_link_jacobian",
    ]


def test_rigid_workflow_latest_signals_prioritize_multibody_solver_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "multibody_solver_max_step_ms": 0.03,
            "multibody_solver_residual_only_residual": 0.62,
            "multibody_solver_residual_solve_ratio": 31.0,
            "multibody_solver_semi_residual": 0.35,
            "multibody_solver_solved_residual": 0.02,
            "multibody_solver_solved_tip_error": 0.0,
            "multibody_solver_variational_residual": 0.62,
            "solver": "world_multibody_integration_family",
        }
    )

    assert highlights[:8] == [
        "multibody solver residual only residual: 0.62",
        "multibody solver solved residual: 0.02",
        "multibody solver residual solve ratio: 31",
        "multibody solver semi residual: 0.35",
        "multibody solver variational residual: 0.62",
        "multibody solver solved tip error: 0",
        "multibody solver max step ms: 0.03",
        "solver: world_multibody_integration_family",
    ]


def test_rigid_workflow_latest_signals_prioritize_loop_closure_values() -> None:
    highlights = capture_py_demo._workflow_metric_highlights(
        {
            "loop_closure_distance_residual_ratio": 7.0,
            "loop_closure_distance_solved_distance_error": 0.0,
            "loop_closure_distance_solved_tip_error": 0.2,
            "loop_closure_max_step_ms": 0.04,
            "loop_closure_point_residual_ratio": 9.0,
            "loop_closure_rigid_residual_orientation_error": 0.12,
            "loop_closure_rigid_residual_ratio": 11.0,
            "loop_closure_rigid_solved_orientation_error": 0.0,
            "solver": "variational_rigid_multibody_loop_closure",
        }
    )

    assert highlights[:8] == [
        "loop closure point residual ratio: 9",
        "loop closure distance residual ratio: 7",
        "loop closure rigid residual ratio: 11",
        "loop closure distance solved distance error: 0",
        "loop closure distance solved tip error: 0.2",
        "loop closure rigid residual orientation error: 0.12",
        "loop closure rigid solved orientation error: 0",
        "solver: variational_rigid_multibody_loop_closure",
    ]


def test_rigid_workflow_run_links_scene_videos(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_body", 24, 960, 540, True),)
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    rendered: list[list[str]] = []

    def fake_run(argv: list[str]) -> int:
        rendered.append(argv)
        scene = argv[argv.index("--scene") + 1]
        output_dir = pathlib.Path(argv[argv.index("--output-dir") + 1])
        output_dir.mkdir(parents=True)
        screenshot = output_dir / f"{scene}.png"
        video = output_dir / f"{scene}.mp4"
        screenshot.write_bytes(b"fake-png")
        video.write_bytes(b"fake-mp4")
        (output_dir / "manifest.json").write_text(
            json.dumps(
                {
                    "scene": scene,
                    "artifacts": {
                        "frames": str(output_dir / "png_frames"),
                        "screenshot": str(screenshot),
                        "video": str(video),
                    },
                },
                sort_keys=True,
            )
            + "\n"
        )
        return 0

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fake_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--video",
            "--fps",
            "12",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    assert rendered
    assert "--video" in rendered[0]
    assert rendered[0][rendered[0].index("--fps") + 1] == "12"
    manifest = json.loads((output / "manifest.json").read_text())
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "scenes/01_rigid_body/rigid_body.mp4" in review_html
    assert ">video</a>" in review_html


def test_rigid_workflow_fails_when_scene_manifest_is_missing(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (
        ("rigid_body", 24, 960, 540, True),
        ("rigid_solver_compare", 24, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    rendered: list[str] = []

    def fake_run(argv: list[str]) -> int:
        rendered.append(argv[argv.index("--scene") + 1])
        return 0

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fake_run)

    rc = capture_py_demo.main(["--rigid-workflow", "--output-dir", str(output)])

    assert rc == 1
    assert rendered == ["rigid_body"]
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["dry_run"] is False
    assert manifest["continue_on_failure"] is False
    assert manifest["status"] == "failed"
    assert manifest["completed_count"] == 0
    assert manifest["failed_count"] == 1
    assert manifest["failed_rows"] == [
        {
            "order": 1,
            "count": 2,
            "scene": "rigid_body",
            "workflow_group": "numbered",
            "workflow_label": "Baseline",
            "failure_reason": "missing_manifest",
            "return_code": 0,
            "manifest_exists": False,
            "manifest": str(output / "scenes" / "01_rigid_body" / "manifest.json"),
            "command": manifest["captures"][0]["command"],
            "workflow_rerun_command": manifest["captures"][0][
                "workflow_rerun_command"
            ],
        }
    ]
    first_capture = manifest["captures"][0]
    assert first_capture["scene"] == "rigid_body"
    assert first_capture["status"] == "failed"
    assert first_capture["manifest_exists"] is False
    assert first_capture["failure_reason"] == "missing_manifest"
    review_index = pathlib.Path(manifest["artifacts"]["review_index"])
    assert review_index.is_file()
    review_html = review_index.read_text()
    assert "Failed Rows" in review_html
    assert "1/2 rigid_body" in review_html
    assert "missing_manifest" in review_html
    assert "rerun workflow row" in review_html
    assert "capture scene directly" in review_html
    assert "--workflow-start-row 1" in review_html


def test_rigid_workflow_can_continue_after_scene_failure(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (
        ("rigid_body", 24, 960, 540, True),
        ("rigid_solver_compare", 24, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    rendered: list[str] = []

    def fake_run(argv: list[str]) -> int:
        scene = argv[argv.index("--scene") + 1]
        rendered.append(scene)
        if scene == "rigid_body":
            return 9
        output_dir = pathlib.Path(argv[argv.index("--output-dir") + 1])
        output_dir.mkdir(parents=True)
        (output_dir / "manifest.json").write_text(
            json.dumps({"scene": scene, "artifacts": {}}, sort_keys=True) + "\n"
        )
        return 0

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fake_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--continue-on-failure",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 9
    assert rendered == ["rigid_body", "rigid_solver_compare"]
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["dry_run"] is False
    assert manifest["continue_on_failure"] is True
    assert manifest["status"] == "failed"
    assert manifest["completed_count"] == 1
    assert manifest["failed_count"] == 1
    assert len(manifest["failed_rows"]) == 1
    assert manifest["failed_rows"][0]["order"] == 1
    assert manifest["failed_rows"][0]["scene"] == "rigid_body"
    assert manifest["failed_rows"][0]["failure_reason"] == "return_code"
    assert manifest["failed_rows"][0]["return_code"] == 9
    assert manifest["failed_rows"][0]["manifest_exists"] is False
    assert "--continue-on-failure" in manifest["failed_rows"][0][
        "workflow_rerun_command"
    ]
    assert [capture["status"] for capture in manifest["captures"]] == [
        "failed",
        "captured",
    ]
    first_capture = manifest["captures"][0]
    assert first_capture["scene"] == "rigid_body"
    assert first_capture["return_code"] == 9
    assert first_capture["failure_reason"] == "return_code"
    assert first_capture["manifest_exists"] is False
    review_index = pathlib.Path(manifest["artifacts"]["review_index"])
    assert review_index.is_file()
    review_html = review_index.read_text()
    assert "failure mode" in review_html
    assert "continue" in review_html
    assert "Failed Rows" in review_html
    assert "1/2 rigid_body" in review_html
    assert "rerun workflow row" in review_html
    assert "capture scene directly" in review_html
    assert "return_code" in review_html
    assert "rigid_solver_compare" in review_html


def test_rigid_workflow_failed_row_rerun_preserves_packet_flags(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    monkeypatch.setattr(
        capture_py_demo,
        "RIGID_WORKFLOW_CAPTURE_SPECS",
        (("rigid_body", 24, 960, 540, True),),
    )
    monkeypatch.setattr(
        capture_py_demo,
        "RIGID_WORKFLOW_RELATED_CAPTURE_SPECS",
        (("floating_base", 72, 960, 540, True),),
    )
    monkeypatch.setattr(
        capture_py_demo,
        "RIGID_WORKFLOW_IPC_SHELF_CAPTURE_SPECS",
        (("rigid_ipc", 72, 960, 540, True),),
    )
    monkeypatch.setattr(
        capture_py_demo,
        "RIGID_WORKFLOW_PACKET_CAPTURE_SPECS",
        (("rigid_ipc_stack_packet", 24, 960, 540, True),),
    )

    def fake_run(argv: list[str]) -> int:
        assert argv[argv.index("--scene") + 1] == "rigid_ipc_stack_packet"
        return 7

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fake_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--include-related",
            "--include-ipc-shelf",
            "--include-packets",
            "--continue-on-failure",
            "--workflow-start-row",
            "4",
            "--workflow-end-row",
            "4",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 7
    manifest = json.loads((output / "manifest.json").read_text())
    assert len(manifest["failed_rows"]) == 1
    rerun_command = manifest["failed_rows"][0]["workflow_rerun_command"]
    assert "--include-related" in rerun_command
    assert "--include-ipc-shelf" in rerun_command
    assert "--include-packets" in rerun_command
    assert "--continue-on-failure" in rerun_command
    assert "--workflow-start-row 4 --workflow-end-row 4" in rerun_command
    assert str(output / "reruns" / "04_rigid_ipc_stack_packet") in rerun_command
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "rerun workflow row" in review_html
    assert "--include-related --include-ipc-shelf --include-packets" in review_html


def test_rigid_workflow_run_can_resume_from_selected_row(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (
        ("rigid_body", 24, 960, 540, True),
        ("rigid_solver_compare", 24, 960, 540, True),
        ("rigid_loop_closure", 72, 960, 540, True),
    )
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    rendered: list[str] = []

    def fake_run(argv: list[str]) -> int:
        scene = argv[argv.index("--scene") + 1]
        output_dir = pathlib.Path(argv[argv.index("--output-dir") + 1])
        output_dir.mkdir(parents=True)
        (output_dir / "manifest.json").write_text(
            json.dumps({"scene": scene, "artifacts": {}}, sort_keys=True) + "\n"
        )
        rendered.append(scene)
        return 0

    monkeypatch.setattr(capture_py_demo, "_run_scene_capture_from_argv", fake_run)

    rc = capture_py_demo.main(
        [
            "--rigid-workflow",
            "--workflow-start-row",
            "2",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    assert rendered == ["rigid_solver_compare", "rigid_loop_closure"]
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["status"] == "complete"
    assert manifest["capture_count"] == 2
    assert manifest["workflow_total_count"] == len(specs)
    assert manifest["workflow_row_start"] == 2
    assert manifest["workflow_row_end"] == 3
    assert [capture["status"] for capture in manifest["captures"]] == [
        "captured",
        "captured",
    ]


def test_rigid_workflow_scene_capture_runs_in_child_process(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    class _Completed:
        returncode = 7

    def fake_run(command: list[str], *, check: bool) -> _Completed:
        captured["command"] = command
        captured["check"] = check
        return _Completed()

    monkeypatch.setattr(capture_py_demo.subprocess, "run", fake_run)

    rc = capture_py_demo._run_scene_capture_from_argv(["--scene", "rigid_body"])

    command = captured["command"]
    assert rc == 7
    assert captured["check"] is False
    assert command[0] == capture_py_demo.sys.executable
    assert command[1].endswith("scripts/capture_py_demo.py")
    assert command[2:] == ["--scene", "rigid_body"]


@pytest.mark.parametrize(
    ("flag", "message"),
    (
        ("--include-related", "--include-related requires --rigid-workflow"),
        ("--include-ipc-shelf", "--include-ipc-shelf requires --rigid-workflow"),
        ("--include-packets", "--include-packets requires --rigid-workflow"),
        (
            "--continue-on-failure",
            "--continue-on-failure requires --rigid-workflow",
        ),
        (
            "--workflow-end-row",
            "--workflow-start-row/--workflow-end-row require --rigid-workflow",
        ),
    ),
)
def test_rigid_workflow_extra_groups_require_workflow(flag: str, message: str) -> None:
    args = [flag, "--dry-run"]
    if flag == "--workflow-end-row":
        args.insert(1, "1")
    with pytest.raises(SystemExit, match=message):
        capture_py_demo.main(args)


@pytest.mark.parametrize(
    ("args", "message"),
    (
        (["--workflow-start-row", "0"], "--workflow-start-row must be >= 1"),
        (["--workflow-end-row", "0"], "--workflow-end-row must be >= 1"),
        (
            ["--workflow-start-row", "3", "--workflow-end-row", "2"],
            "--workflow-start-row must be <= --workflow-end-row",
        ),
        (
            ["--workflow-end-row", "3"],
            "--workflow-end-row must be <= 1 for this workflow packet",
        ),
    ),
)
def test_rigid_workflow_row_selection_validates_bounds(
    args: list[str], message: str, monkeypatch: pytest.MonkeyPatch, tmp_path: pathlib.Path
) -> None:
    monkeypatch.setattr(
        capture_py_demo,
        "RIGID_WORKFLOW_CAPTURE_SPECS",
        (("rigid_body", 24, 960, 540, True),),
    )
    with pytest.raises(SystemExit, match=message):
        capture_py_demo.main(
            ["--rigid-workflow", "--dry-run", "--output-dir", str(tmp_path), *args]
        )


def test_linux_render_env_defaults_to_software_gl(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(capture_py_demo.sys, "platform", "linux")
    monkeypatch.delenv("LIBGL_ALWAYS_SOFTWARE", raising=False)
    monkeypatch.delenv("MESA_LOADER_DRIVER_OVERRIDE", raising=False)

    capture_py_demo._apply_stable_linux_render_env()

    assert capture_py_demo.os.environ["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert capture_py_demo.os.environ["MESA_LOADER_DRIVER_OVERRIDE"] == "llvmpipe"


def test_linux_render_env_preserves_overrides(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(capture_py_demo.sys, "platform", "linux")
    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "0")
    monkeypatch.setenv("MESA_LOADER_DRIVER_OVERRIDE", "custom")

    capture_py_demo._apply_stable_linux_render_env()

    assert capture_py_demo.os.environ["LIBGL_ALWAYS_SOFTWARE"] == "0"
    assert capture_py_demo.os.environ["MESA_LOADER_DRIVER_OVERRIDE"] == "custom"


def test_visual_capture_default_scene_matches_py_demos_front_door() -> None:
    from examples.demos.runner import DEFAULT_INITIAL_SCENE_ID

    assert capture_py_demo.parse_args([]).scene == DEFAULT_INITIAL_SCENE_ID
    assert DEFAULT_INITIAL_SCENE_ID == "rigid_body"


def test_visual_capture_rejects_noop_backend(tmp_path: pathlib.Path) -> None:
    args = argparse.Namespace(
        backend="noop",
        allow_noop=False,
        scene="articulated",
        frames=1,
        width=2,
        height=1,
        show_ui=False,
    )

    with pytest.raises(ValueError, match="noop"):
        capture_py_demo.build_demo_args(
            args, tmp_path / "capture.ppm", tmp_path / "frames"
        )


def test_visual_capture_forwards_scripted_demo_switch(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_ipc_slide",
        frames=6,
        width=640,
        height=360,
        show_ui=True,
        switch_scene="rigid_ipc_incline",
        switch_frame=2,
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-demo-switch" in demo_args
    assert "2:rigid_ipc_incline" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args


def test_visual_capture_forwards_scene_metrics_log(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_body",
        frames=3,
        width=640,
        height=360,
        show_ui=True,
        capture_metrics_event_log=tmp_path / "scene_metrics.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--capture-metrics-event-log" in demo_args
    assert str(tmp_path / "scene_metrics.jsonl") in demo_args


def test_visual_capture_forwards_scripted_force_drag(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_ipc_slide",
        frames=12,
        width=640,
        height=360,
        show_ui=True,
        force_drag_target="ipc_slide_box_visual",
        force_drag_frame=2,
        force_drag_frames=5,
        force_drag_delta=(0.8, 0.0, 0.2),
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-force-drag" in demo_args
    assert "2:ipc_slide_box_visual:0.8,0,0.2:5" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args


def test_visual_capture_forwards_scripted_pointer_force_drag(
    tmp_path: pathlib.Path,
) -> None:
    args = argparse.Namespace(
        backend="opengl",
        allow_noop=False,
        scene="rigid_ipc_slide",
        frames=12,
        width=640,
        height=360,
        show_ui=True,
        force_drag_target="",
        force_drag_pixel=(320.0, 180.0),
        force_drag_delta_pixels=(140.0, -50.0),
        force_drag_frame=2,
        force_drag_frames=5,
        force_drag_delta=(0.8, 0.0, 0.2),
        event_log=tmp_path / "events.jsonl",
    )

    demo_args = capture_py_demo.build_demo_args(
        args, tmp_path / "capture.ppm", tmp_path / "frames"
    )

    assert "--scripted-pointer-force-drag" in demo_args
    assert "2:320,180:140,-50:5" in demo_args
    assert "--scripted-demo-event-log" in demo_args
    assert str(tmp_path / "events.jsonl") in demo_args
