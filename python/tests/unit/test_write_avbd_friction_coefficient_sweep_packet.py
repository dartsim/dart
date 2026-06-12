from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
PACKET_SCRIPT = ROOT / "scripts" / "write_avbd_friction_coefficient_sweep_packet.py"
PLOT_SCRIPT = ROOT / "scripts" / "write_avbd_friction_coefficient_sweep_plot.py"


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
    assert packet["packet"] == "avbd_friction_coefficient_sweep"
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
