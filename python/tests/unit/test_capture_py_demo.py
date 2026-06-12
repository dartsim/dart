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
    assert manifest["include_packets"] is False
    assert manifest["dry_run"] is True
    assert manifest["status"] == "planned"
    assert manifest["capture_count"] == len(specs)
    assert manifest["completed_count"] == 0
    assert pathlib.Path(manifest["artifacts"]["review_index"]).is_file()
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_body",
        "rigid_solver_compare",
    ]
    assert manifest["captures"][0]["command"].startswith(
        "pixi run py-demo-capture -- --scene rigid_body"
    )
    assert manifest["captures"][0]["manifest"].endswith(
        "scenes/01_rigid_body/manifest.json"
    )
    review_index = pathlib.Path(manifest["artifacts"]["review_index"])
    review_html = review_index.read_text()
    assert "DART rigid workflow review index" in review_html
    assert "rigid_solver_compare" in review_html
    assert "scenes/01_rigid_body/manifest.json" in review_html


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
    assert manifest["captures"][2]["manifest"].endswith(
        "scenes/03_rigid_ipc_tunnel/manifest.json"
    )
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
    assert "rigid_ipc_tunnel" in review_html
    assert "related_evidence" in review_html


def test_rigid_workflow_dry_run_can_include_capture_first_packets(
    tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "rigid_workflow"
    specs = (("rigid_body", 24, 960, 540, True),)
    related_specs = (("rigid_ipc_tunnel", 24, 960, 540, True),)
    packet_specs = (("rigid_ipc_stack_packet", 24, 960, 540, True),)
    monkeypatch.setattr(capture_py_demo, "RIGID_WORKFLOW_CAPTURE_SPECS", specs)
    monkeypatch.setattr(
        capture_py_demo, "RIGID_WORKFLOW_RELATED_CAPTURE_SPECS", related_specs
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
            "--include-packets",
            "--dry-run",
            "--output-dir",
            str(output),
        ]
    )

    assert rc == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["include_related"] is True
    assert manifest["include_packets"] is True
    assert manifest["capture_count"] == len(specs) + len(related_specs) + len(
        packet_specs
    )
    assert [capture["scene"] for capture in manifest["captures"]] == [
        "rigid_body",
        "rigid_ipc_tunnel",
        "rigid_ipc_stack_packet",
    ]
    assert [capture["workflow_group"] for capture in manifest["captures"]] == [
        "numbered",
        "related_evidence",
        "capture_first_packet",
    ]
    assert manifest["captures"][2]["manifest"].endswith(
        "scenes/03_rigid_ipc_stack_packet/manifest.json"
    )
    review_html = pathlib.Path(manifest["artifacts"]["review_index"]).read_text()
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
                                "held_fixed": {"solver": "si"},
                            },
                        },
                        "metric_key_counts": {
                            "comparison_axis": 1,
                            "held_fixed": 1,
                        },
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
    assert "scenes/02_rigid_executor_equivalence/manifest.json" in review_html
    assert "test_axis" in review_html
    assert "comparison_axis, held_fixed" in review_html


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
    assert manifest["status"] == "failed"
    assert manifest["completed_count"] == 0
    assert manifest["failed_count"] == 1
    first_capture = manifest["captures"][0]
    assert first_capture["scene"] == "rigid_body"
    assert first_capture["status"] == "failed"
    assert first_capture["manifest_exists"] is False
    assert first_capture["failure_reason"] == "missing_manifest"
    review_index = pathlib.Path(manifest["artifacts"]["review_index"])
    assert review_index.is_file()
    assert "missing_manifest" in review_index.read_text()


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
        ("--include-packets", "--include-packets requires --rigid-workflow"),
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
