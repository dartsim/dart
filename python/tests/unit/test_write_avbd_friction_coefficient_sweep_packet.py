from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
PACKET_SCRIPT = ROOT / "scripts" / "write_avbd_friction_coefficient_sweep_packet.py"
PLOT_SCRIPT = ROOT / "scripts" / "write_avbd_friction_coefficient_sweep_plot.py"
REFERENCE_SCRIPT = ROOT / "scripts" / "run_avbd_demo2d_reference_timing.py"


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _benchmark_row(max_friction: float) -> dict[str, object]:
    arg = int(round(max_friction * 10.0))
    name = f"BM_AvbdDemo2dFrictionCoefficientSweep/{arg}"
    return {
        "collision_shapes": 12.0,
        "cpu_time": 100.0 + arg,
        "friction_samples": 11.0,
        "iterations": 10,
        "max_friction": max_friction,
        "min_friction": 0.0,
        "name": name,
        "real_time": 120.0 + arg,
        "rigid_bodies": 12.0,
        "rigid_body_joints": 0.0,
        "run_name": name,
        "run_type": "iteration",
        "source_scene_index": 2.0,
        "time_unit": "ns",
    }


def _write_benchmark_json(
    tmp_path: Path,
    *,
    omit: float | None = None,
    wrong_rigid_bodies: bool = False,
) -> Path:
    rows = []
    for max_friction in (0.0, 0.5, 1.0, 2.5, 5.0):
        if omit is not None and max_friction == omit:
            continue
        row = _benchmark_row(max_friction)
        if wrong_rigid_bodies and max_friction == 1.0:
            row["rigid_bodies"] = 13.0
        rows.append(row)
    benchmark = {
        "benchmarks": rows,
        "context": {
            "executable": "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint",
            "json_schema_version": 1,
            "library_build_type": "release",
            "library_version": "v1.9.5",
            "mhz_per_cpu": 3200,
            "num_cpus": 8,
        },
    }
    path = tmp_path / "benchmark.json"
    path.write_text(json.dumps(benchmark), encoding="utf-8")
    return path


def _write_reference_timing_json(
    tmp_path: Path,
    max_friction: float,
    *,
    cpu_time_per_step_ns: float | None = None,
    wrong_dynamic_bodies: bool = False,
) -> Path:
    arg = int(round(max_friction * 10.0))
    payload = {
        "box_count": 11,
        "collision_shapes": 12,
        "compile_command": ["c++", "<temp-dir>/reference_runner.cpp"],
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "c++",
        "cpu_time_per_step_ns": (
            200.0 + arg
            if cpu_time_per_step_ns is None
            else cpu_time_per_step_ns
        ),
        "dynamic_bodies": 10 if wrong_dynamic_bodies else 11,
        "dynamic_max_friction": max_friction,
        "dynamic_min_friction": 0.0,
        "elapsed_ns": 200_000.0 + arg,
        "final_time": 166.6666667,
        "friction_samples": 11,
        "initial_speed": 10.0,
        "joints": 0,
        "max_friction": max(0.5, max_friction),
        "min_friction": 0.0,
        "repository": "https://github.com/savant117/avbd-demo2d",
        "requested_max_friction": max_friction,
        "rigid_bodies": 12,
        "scene_builder": "sceneDynamicFriction",
        "scene_index": 2,
        "scene_name": "Dynamic Friction",
        "schema_version": 1,
        "source_demo": "avbd-demo2d",
        "source_revision": "74699a11f858",
        "static_bodies": 1,
        "steps": 1000,
        "warmup_steps": 8,
    }
    path = tmp_path / f"reference-{arg}.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _write_reference_sweep_jsons(tmp_path: Path) -> list[Path]:
    return [
        _write_reference_timing_json(tmp_path, max_friction)
        for max_friction in (0.0, 0.5, 1.0, 2.5, 5.0)
    ]


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", checksum)
    )


def _write_png(path: Path, width: int = 2, height: int = 1) -> None:
    pixels = b"\xff\x00\x00\x00\xff\x00" * width * height
    rows = [b"\x00" + pixels[y * width * 3 : (y + 1) * width * 3] for y in range(height)]
    raw = b"".join(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(raw, 6))
        + _png_chunk(b"IEND", b"")
    )


def _write_capture_manifest(
    tmp_path: Path,
    max_friction: float,
    *,
    env_max_friction: str | None = None,
    wrong_scene: bool = False,
) -> Path:
    arg = int(round(max_friction * 10.0))
    capture_dir = tmp_path / f"capture-{arg}"
    frames = capture_dir / "png_frames"
    screenshot = capture_dir / "avbd_demo2d_dynamic_friction.png"
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
        "metadata": {"max_friction": f"{max_friction:g}"},
        "scene": "wrong" if wrong_scene else "avbd_demo2d_dynamic_friction",
        "scene_environment": {
            "DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION": (
                f"{max_friction:g}" if env_max_friction is None else env_max_friction
            )
        },
        "schema_version": 1,
        "show_ui": False,
        "switch_frame": None,
        "switch_scene": None,
        "ui_ready": {"dropped_warmup_frames": 0, "required": False},
    }
    path = capture_dir / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def _write_capture_manifests(tmp_path: Path) -> list[Path]:
    return [
        _write_capture_manifest(tmp_path, max_friction)
        for max_friction in (0.0, 0.5, 1.0, 2.5, 5.0)
    ]


def test_avbd_friction_coefficient_sweep_packet_records_rows(
    tmp_path: Path,
) -> None:
    module = _load_module(PACKET_SCRIPT, "write_avbd_friction_coefficient_sweep_packet")
    benchmark_json = _write_benchmark_json(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--benchmark-json",
                str(benchmark_json),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text(encoding="utf-8"))
    assert packet["schema_version"] == 2
    assert packet["packet"] == "avbd_friction_coefficient_sweep"
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "friction coefficient sweep benchmark scene counters",
        "rigid_contact_solver": "sequential_impulse",
        "rigid_point_joint_solver": "none",
    }
    assert packet["scene"] == "avbd_demo2d_dynamic_friction"
    assert packet["target"]["paper_gap"] == "friction coefficient comparison"
    assert packet["target"]["complete_paper_reproduction"] is False
    assert packet["visual_anchor"]["scene"] == "avbd_demo2d_dynamic_friction"
    assert (
        packet["visual_anchor"]["existing_packet"]
        == "avbd-demo2d-dynamic-friction-packet.json"
    )
    assert packet["benchmark"]["benchmark"] == "BM_AvbdDemo2dFrictionCoefficientSweep"
    assert packet["benchmark"]["invariants"]["max_friction"] == [
        0.0,
        0.5,
        1.0,
        2.5,
        5.0,
    ]
    assert [
        row["max_friction"] for row in packet["benchmark"]["plot_data"]
    ] == [0.0, 0.5, 1.0, 2.5, 5.0]
    assert "rendered friction-sweep plot" in packet["remaining_gates"]


def test_avbd_friction_coefficient_sweep_packet_records_visual_sweep(
    tmp_path: Path,
) -> None:
    module = _load_module(PACKET_SCRIPT, "write_avbd_friction_sweep_packet_visual")
    benchmark_json = _write_benchmark_json(tmp_path)
    output = tmp_path / "packet.json"
    args = [
        "--benchmark-json",
        str(benchmark_json),
        "--output",
        str(output),
    ]
    for manifest in _write_capture_manifests(tmp_path):
        args.extend(["--capture-manifest", str(manifest)])

    assert module.main(args) == 0

    packet = json.loads(output.read_text(encoding="utf-8"))
    assert [row["max_friction"] for row in packet["visual_sweep"]["captures"]] == [
        0.0,
        0.5,
        1.0,
        2.5,
        5.0,
    ]
    assert packet["visual_sweep"]["captures"][0]["screenshot"]["width"] == 2
    assert "per-coefficient visual capture or video evidence" not in packet[
        "remaining_gates"
    ]


def test_avbd_friction_coefficient_sweep_packet_accepts_equivalent_capture_env(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT,
        "write_avbd_friction_sweep_packet_equivalent_capture_env",
    )
    manifest = _write_capture_manifest(tmp_path, 1.0, env_max_friction="1.0")

    capture = module._validate_capture_manifest(manifest, 1.0)

    assert capture["max_friction"] == 1.0


def test_avbd_friction_coefficient_sweep_packet_rejects_mismatched_capture_env(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT,
        "write_avbd_friction_sweep_packet_mismatched_capture_env",
    )
    manifest = _write_capture_manifest(tmp_path, 1.0, env_max_friction="1.5")

    with pytest.raises(
        module.AvbdFrictionCoefficientSweepPacketError,
        match="Dynamic Friction scene env",
    ):
        module._validate_capture_manifest(manifest, 1.0)


def test_avbd_friction_coefficient_sweep_packet_rejects_nonnumeric_capture_env(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT,
        "write_avbd_friction_sweep_packet_nonnumeric_capture_env",
    )
    manifest = _write_capture_manifest(tmp_path, 1.0, env_max_friction="fast")

    with pytest.raises(
        module.AvbdFrictionCoefficientSweepPacketError,
        match="Dynamic Friction scene env",
    ):
        module._validate_capture_manifest(manifest, 1.0)


def test_avbd_friction_coefficient_sweep_packet_records_reference_sweep(
    tmp_path: Path,
) -> None:
    module = _load_module(PACKET_SCRIPT, "write_avbd_friction_sweep_packet_ref")
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_jsons = _write_reference_sweep_jsons(tmp_path)
    output = tmp_path / "packet.json"

    args = [
        "--benchmark-json",
        str(benchmark_json),
        "--output",
        str(output),
    ]
    for reference_json in reference_jsons:
        args.extend(["--reference-timing-json", str(reference_json)])
    assert module.main(args) == 0

    packet = json.loads(output.read_text(encoding="utf-8"))
    assert packet["reference_sweep"]["scene_builder"] == "sceneDynamicFriction"
    assert [
        row["max_friction"] for row in packet["reference_sweep"]["plot_data"]
    ] == [0.0, 0.5, 1.0, 2.5, 5.0]
    assert packet["reference_sweep"]["all_dart_faster_than_reference"] is True
    assert [
        row["dart_faster_than_reference"]
        for row in packet["reference_sweep"]["comparison"]
    ] == [True, True, True, True, True]
    assert "source/reference friction-sweep timing comparison" not in packet[
        "remaining_gates"
    ]


def test_avbd_friction_coefficient_sweep_packet_records_mixed_reference_sweep(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT, "write_avbd_friction_sweep_packet_mixed_ref"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_jsons = [
        _write_reference_timing_json(tmp_path, 0.0, cpu_time_per_step_ns=50.0),
        *[
            _write_reference_timing_json(tmp_path, max_friction)
            for max_friction in (0.5, 1.0, 2.5, 5.0)
        ],
    ]
    output = tmp_path / "packet.json"

    args = [
        "--benchmark-json",
        str(benchmark_json),
        "--output",
        str(output),
    ]
    for reference_json in reference_jsons:
        args.extend(["--reference-timing-json", str(reference_json)])
    assert module.main(args) == 0

    packet = json.loads(output.read_text(encoding="utf-8"))
    assert packet["reference_sweep"]["all_dart_faster_than_reference"] is False
    assert [
        row["dart_faster_than_reference"]
        for row in packet["reference_sweep"]["comparison"]
    ] == [False, True, True, True, True]
    assert packet["reference_sweep"]["comparison"][0][
        "dart_to_reference_cpu_time_ratio"
    ] == 2.0
    assert (
        "DART CPU performance must beat the native source sweep before a CPU win is claimed for every friction coefficient"
        in packet["remaining_gates"]
    )


def test_avbd_friction_coefficient_sweep_packet_rejects_missing_value(
    tmp_path: Path,
) -> None:
    module = _load_module(PACKET_SCRIPT, "write_avbd_friction_coefficient_sweep_packet")
    benchmark_json = _write_benchmark_json(tmp_path, omit=2.5)

    with pytest.raises(SystemExit, match="missing max_friction values: 2.5"):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_friction_coefficient_sweep_packet_rejects_wrong_counts(
    tmp_path: Path,
) -> None:
    module = _load_module(PACKET_SCRIPT, "write_avbd_friction_coefficient_sweep_packet")
    benchmark_json = _write_benchmark_json(tmp_path, wrong_rigid_bodies=True)

    with pytest.raises(SystemExit, match="expected rigid_bodies=12"):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_friction_coefficient_sweep_packet_rejects_missing_reference_value(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT, "write_avbd_friction_sweep_packet_missing_ref"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_jsons = [
        _write_reference_timing_json(tmp_path, max_friction)
        for max_friction in (0.0, 0.5, 1.0, 5.0)
    ]
    args = ["--benchmark-json", str(benchmark_json)]
    for reference_json in reference_jsons:
        args.extend(["--reference-timing-json", str(reference_json)])

    with pytest.raises(
        SystemExit, match="reference timing missing max_friction values: 2.5"
    ):
        module.main(args)


def test_avbd_friction_coefficient_sweep_packet_rejects_reference_counts(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT, "write_avbd_friction_sweep_packet_wrong_ref"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_jsons = _write_reference_sweep_jsons(tmp_path)
    reference_jsons[2] = _write_reference_timing_json(
        tmp_path, 1.0, wrong_dynamic_bodies=True
    )
    args = ["--benchmark-json", str(benchmark_json)]
    for reference_json in reference_jsons:
        args.extend(["--reference-timing-json", str(reference_json)])

    with pytest.raises(SystemExit, match="expected 11 dynamic bodies"):
        module.main(args)


def test_avbd_friction_coefficient_sweep_packet_rejects_missing_capture_value(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT, "write_avbd_friction_sweep_packet_missing_capture"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    manifests = [
        _write_capture_manifest(tmp_path, max_friction)
        for max_friction in (0.0, 0.5, 1.0, 5.0)
    ]
    args = ["--benchmark-json", str(benchmark_json)]
    for manifest in manifests:
        args.extend(["--capture-manifest", str(manifest)])

    with pytest.raises(
        SystemExit, match="capture manifests missing max_friction values: 2.5"
    ):
        module.main(args)


def test_avbd_friction_coefficient_sweep_packet_rejects_capture_scene(
    tmp_path: Path,
) -> None:
    module = _load_module(
        PACKET_SCRIPT, "write_avbd_friction_sweep_packet_wrong_capture"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    manifests = _write_capture_manifests(tmp_path)
    manifests[2] = _write_capture_manifest(tmp_path, 1.0, wrong_scene=True)
    args = ["--benchmark-json", str(benchmark_json)]
    for manifest in manifests:
        args.extend(["--capture-manifest", str(manifest)])

    with pytest.raises(
        SystemExit, match="capture scene must be avbd_demo2d_dynamic_friction"
    ):
        module.main(args)


def test_avbd_friction_coefficient_sweep_plot_and_packet_link(
    tmp_path: Path,
) -> None:
    packet_module = _load_module(
        PACKET_SCRIPT,
        "write_avbd_friction_coefficient_sweep_packet_for_plot",
    )
    plot_module = _load_module(PLOT_SCRIPT, "write_avbd_friction_coefficient_sweep_plot")
    benchmark_json = _write_benchmark_json(tmp_path)
    packet_path = tmp_path / "packet.json"
    plot_path = tmp_path / "plot.svg"
    linked_packet_path = tmp_path / "linked_packet.json"

    assert (
        packet_module.main(
            [
                "--benchmark-json",
                str(benchmark_json),
                "--output",
                str(packet_path),
            ]
        )
        == 0
    )
    assert (
        plot_module.main(
            ["--packet", str(packet_path), "--output", str(plot_path)]
        )
        == 0
    )
    assert (
        packet_module.main(
            [
                "--benchmark-json",
                str(benchmark_json),
                "--plot-svg",
                str(plot_path),
                "--output",
                str(linked_packet_path),
            ]
        )
        == 0
    )

    svg = plot_path.read_text(encoding="utf-8")
    assert "<svg" in svg
    assert "AVBD Friction Coefficient Sweep" in svg
    packet = json.loads(linked_packet_path.read_text(encoding="utf-8"))
    assert packet["rendered_plot"]["file"] == str(plot_path)
    assert "rendered friction-sweep plot" not in packet["remaining_gates"]


def test_avbd_friction_coefficient_sweep_plot_draws_reference_sweep(
    tmp_path: Path,
) -> None:
    packet_module = _load_module(
        PACKET_SCRIPT,
        "write_avbd_friction_coefficient_sweep_packet_ref_plot",
    )
    plot_module = _load_module(
        PLOT_SCRIPT, "write_avbd_friction_coefficient_sweep_plot_ref"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    packet_path = tmp_path / "packet.json"
    plot_path = tmp_path / "plot.svg"
    args = [
        "--benchmark-json",
        str(benchmark_json),
        "--output",
        str(packet_path),
    ]
    for reference_json in _write_reference_sweep_jsons(tmp_path):
        args.extend(["--reference-timing-json", str(reference_json)])

    assert packet_module.main(args) == 0
    assert (
        plot_module.main(["--packet", str(packet_path), "--output", str(plot_path)])
        == 0
    )

    svg = plot_path.read_text(encoding="utf-8")
    assert "DART" in svg
    assert "Native source" in svg
    assert "#dc2626" in svg


def test_avbd_friction_coefficient_sweep_plot_rejects_wrong_packet(
    tmp_path: Path,
) -> None:
    plot_module = _load_module(
        PLOT_SCRIPT, "write_avbd_friction_coefficient_sweep_plot_reject"
    )
    packet_path = tmp_path / "packet.json"
    packet_path.write_text(
        json.dumps({"schema_version": 1, "packet": "wrong"}),
        encoding="utf-8",
    )

    with pytest.raises(
        SystemExit, match="packet must be avbd_friction_coefficient_sweep"
    ):
        plot_module.main(["--packet", str(packet_path)])


def test_avbd_demo2d_reference_timing_dynamic_friction_max_is_threaded() -> None:
    module = _load_module(
        REFERENCE_SCRIPT, "run_avbd_demo2d_reference_timing_for_sweep"
    )
    args = module.parse_args(
        [
            "--source-dir",
            "/tmp/unused",
            "--scene",
            "dynamic_friction",
            "--dynamic-friction-max-friction",
            "2.5",
            "--steps",
            "1",
            "--output",
            "/tmp/unused.json",
        ]
    )

    source = module._runner_source(args)
    assert "const float requestedMaxFriction = 2.5f;" in source
    assert "body->friction *= frictionScale;" in source
    assert "dynamic_max_friction" in source


def test_avbd_demo2d_reference_timing_rejects_non_dynamic_friction_max() -> None:
    module = _load_module(
        REFERENCE_SCRIPT, "run_avbd_demo2d_reference_timing_reject_sweep"
    )
    args = module.parse_args(
        [
            "--source-dir",
            "/tmp/unused",
            "--scene",
            "ground",
            "--dynamic-friction-max-friction",
            "2.5",
            "--steps",
            "1",
            "--output",
            "/tmp/unused.json",
        ]
    )

    with pytest.raises(
        module.ReferenceTimingError,
        match="only valid with --scene dynamic_friction",
    ):
        module.run_reference_timing(args)
