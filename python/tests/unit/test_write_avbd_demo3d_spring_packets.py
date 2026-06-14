from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def _load_packet_module(script_name: str):
    script = ROOT / "scripts" / script_name
    spec = importlib.util.spec_from_file_location(script_name.removesuffix(".py"), script)
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


def _write_capture_manifest(tmp_path: Path, *, scene: str) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / f"{scene}.png"
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


def _write_benchmark_json(
    tmp_path: Path,
    *,
    name: str,
    rigid_bodies: float,
    distance_springs: float,
    collision_shapes: float,
    source_scene_index: float,
    ignored_collision_pairs: float | None = None,
    cpu_time: float = 120.0,
) -> Path:
    if ignored_collision_pairs is None:
        ignored_collision_pairs = distance_springs
    benchmark = {
        "benchmarks": [
            {
                "collision_shapes": collision_shapes,
                "cpu_time": cpu_time,
                "distance_springs": distance_springs,
                "ignored_collision_pairs": ignored_collision_pairs,
                "iterations": 10,
                "name": name,
                "real_time": 125.0,
                "rigid_bodies": rigid_bodies,
                "rigid_body_joints": 0.0,
                "run_name": name,
                "run_type": "iteration",
                "source_scene_index": source_scene_index,
                "time_unit": "ns",
            }
        ],
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
    *,
    scene_index: int,
    scene_name: str,
    scene_builder: str,
    rigid_bodies: int,
    dynamic_bodies: int,
    static_bodies: int,
    distance_springs: int,
    collision_shapes: int,
    min_spring_stiffness: float,
    max_spring_stiffness: float,
    spring_rest_length: float,
    source_demo: str = "avbd-demo3d",
    source_revision: str = "7701bd427d55",
    cpu_time: float = 40.0,
    spring_link_count: int | None = None,
    high_stiffness_springs: int | None = None,
    low_stiffness_springs: int | None = None,
) -> Path:
    timing = {
        "collision_shapes": collision_shapes,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "distance_springs": distance_springs,
        "dynamic_bodies": dynamic_bodies,
        "elapsed_ns": cpu_time * 1000.0,
        "final_time": 18.8,
        "joints": 0,
        "max_friction": 0.5,
        "max_spring_stiffness": max_spring_stiffness,
        "min_friction": 0.5,
        "min_spring_stiffness": min_spring_stiffness,
        "repository": "https://github.com/savant117/avbd-demo3d",
        "rigid_bodies": rigid_bodies,
        "scene_builder": scene_builder,
        "scene_index": scene_index,
        "scene_name": scene_name,
        "schema_version": 1,
        "source_demo": source_demo,
        "source_revision": source_revision,
        "spring_rest_length": spring_rest_length,
        "static_bodies": static_bodies,
        "steps": 1000,
        "warmup_steps": 128,
    }
    if spring_link_count is not None:
        timing["spring_link_count"] = spring_link_count
    if high_stiffness_springs is not None:
        timing["high_stiffness_springs"] = high_stiffness_springs
    if low_stiffness_springs is not None:
        timing["low_stiffness_springs"] = low_stiffness_springs
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(timing), encoding="utf-8")
    return path


def test_avbd_demo3d_spring_packet_records_evidence(tmp_path: Path) -> None:
    module = _load_packet_module("write_avbd_demo3d_spring_packet.py")
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_demo3d_spring")
    benchmark_json = _write_benchmark_json(
        tmp_path,
        name="BM_AvbdDemo3dSpringStep",
        rigid_bodies=3.0,
        distance_springs=1.0,
        collision_shapes=3.0,
        source_scene_index=7.0,
    )
    reference_json = _write_reference_timing_json(
        tmp_path,
        scene_index=7,
        scene_name="Spring",
        scene_builder="sceneSpring",
        rigid_bodies=3,
        dynamic_bodies=1,
        static_bodies=2,
        distance_springs=1,
        collision_shapes=3,
        min_spring_stiffness=100.0,
        max_spring_stiffness=100.0,
        spring_rest_length=4.0,
    )
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["scene"] == "avbd_demo3d_spring"
    assert packet["source_demo_reference_row"]["scene_name"] == "Spring"
    assert packet["source_demo_reference_row"]["source_constraints"][
        "radial_distance_springs"
    ] == {
        "child_anchor": [0.0, 0.0, 0.0],
        "count": 1,
        "parent_anchor": [0.0, 0.0, 0.0],
        "rest_length": 4.0,
        "stiffness": 100.0,
    }
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo3dSpringStep"
    assert packet["dart_benchmark"]["invariants"] == {
        "collision_shapes": 3,
        "distance_springs": 1,
        "ignored_collision_pairs": 1,
        "rigid_bodies": 3,
        "rigid_body_joints": 0,
        "source_scene_index": 7,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["source_demo"] == "avbd-demo3d"
    assert packet["reference_timing"]["invariants"]["dynamic_bodies"] == 1
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0
    assert (
        packet["comparison"]["performance_claim"]
        == "DART does not yet beat the native avbd-demo3d Spring row"
    )
    assert "run_avbd_demo3d_reference_timing.py" in packet["reproduction"][
        "reference_timing_command"
    ]


def test_avbd_demo3d_spring_ratio_packet_records_evidence(tmp_path: Path) -> None:
    module = _load_packet_module("write_avbd_demo3d_spring_ratio_packet.py")
    capture_manifest = _write_capture_manifest(
        tmp_path,
        scene="avbd_demo3d_spring_ratio",
    )
    benchmark_json = _write_benchmark_json(
        tmp_path,
        name="BM_AvbdDemo3dSpringRatioStep",
        rigid_bodies=9.0,
        distance_springs=7.0,
        collision_shapes=9.0,
        source_scene_index=8.0,
    )
    reference_json = _write_reference_timing_json(
        tmp_path,
        scene_index=8,
        scene_name="Spring Ratio",
        scene_builder="sceneSpringsRatio",
        rigid_bodies=9,
        dynamic_bodies=6,
        static_bodies=3,
        distance_springs=7,
        collision_shapes=9,
        min_spring_stiffness=10.0,
        max_spring_stiffness=1.0e4,
        spring_rest_length=3.0,
        spring_link_count=8,
        high_stiffness_springs=4,
        low_stiffness_springs=3,
    )
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["scene"] == "avbd_demo3d_spring_ratio"
    assert packet["source_demo_reference_row"]["scene_name"] == "Spring Ratio"
    assert packet["source_demo_reference_row"]["source_constraints"][
        "radial_distance_springs"
    ]["stiffness_pattern"] == [
        1.0e4,
        10.0,
        1.0e4,
        10.0,
        1.0e4,
        10.0,
        1.0e4,
    ]
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo3dSpringRatioStep"
    assert packet["reference_timing"]["invariants"]["spring_link_count"] == 8
    assert packet["reference_timing"]["invariants"]["high_stiffness_springs"] == 4
    assert packet["reference_timing"]["invariants"]["low_stiffness_springs"] == 3
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0


def test_avbd_demo3d_spring_packet_rejects_wrong_scene(tmp_path: Path) -> None:
    module = _load_packet_module("write_avbd_demo3d_spring_packet.py")
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_demo3d_ground")
    benchmark_json = _write_benchmark_json(
        tmp_path,
        name="BM_AvbdDemo3dSpringStep",
        rigid_bodies=3.0,
        distance_springs=1.0,
        collision_shapes=3.0,
        source_scene_index=7.0,
    )
    reference_json = _write_reference_timing_json(
        tmp_path,
        scene_index=7,
        scene_name="Spring",
        scene_builder="sceneSpring",
        rigid_bodies=3,
        dynamic_bodies=1,
        static_bodies=2,
        distance_springs=1,
        collision_shapes=3,
        min_spring_stiffness=100.0,
        max_spring_stiffness=100.0,
        spring_rest_length=4.0,
    )

    with pytest.raises(SystemExit, match="capture scene must be avbd_demo3d_spring"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
            ]
        )


def test_avbd_demo3d_spring_packet_rejects_wrong_reference_demo(
    tmp_path: Path,
) -> None:
    module = _load_packet_module("write_avbd_demo3d_spring_packet.py")
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_demo3d_spring")
    benchmark_json = _write_benchmark_json(
        tmp_path,
        name="BM_AvbdDemo3dSpringStep",
        rigid_bodies=3.0,
        distance_springs=1.0,
        collision_shapes=3.0,
        source_scene_index=7.0,
    )
    reference_json = _write_reference_timing_json(
        tmp_path,
        scene_index=7,
        scene_name="Spring",
        scene_builder="sceneSpring",
        rigid_bodies=3,
        dynamic_bodies=1,
        static_bodies=2,
        distance_springs=1,
        collision_shapes=3,
        min_spring_stiffness=100.0,
        max_spring_stiffness=100.0,
        spring_rest_length=4.0,
        source_demo="avbd-demo2d",
    )

    with pytest.raises(
        SystemExit,
        match="reference timing expected source_demo='avbd-demo3d'",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
            ]
        )


def test_avbd_demo3d_spring_ratio_packet_rejects_wrong_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module("write_avbd_demo3d_spring_ratio_packet.py")
    capture_manifest = _write_capture_manifest(
        tmp_path,
        scene="avbd_demo3d_spring_ratio",
    )
    benchmark_json = _write_benchmark_json(
        tmp_path,
        name="BM_AvbdDemo3dSpringRatioStep",
        rigid_bodies=8.0,
        distance_springs=7.0,
        collision_shapes=9.0,
        source_scene_index=8.0,
    )
    reference_json = _write_reference_timing_json(
        tmp_path,
        scene_index=8,
        scene_name="Spring Ratio",
        scene_builder="sceneSpringsRatio",
        rigid_bodies=9,
        dynamic_bodies=6,
        static_bodies=3,
        distance_springs=7,
        collision_shapes=9,
        min_spring_stiffness=10.0,
        max_spring_stiffness=1.0e4,
        spring_rest_length=3.0,
        spring_link_count=8,
        high_stiffness_springs=4,
        low_stiffness_springs=3,
    )

    with pytest.raises(SystemExit, match="expected rigid_bodies=9"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
            ]
        )
