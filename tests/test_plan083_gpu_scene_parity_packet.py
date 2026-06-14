import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_gpu_scene_parity_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_gpu_scene_parity_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(name: str, real_time: float, max_error: float = 0.0) -> dict[str, object]:
    return {
        "name": f"{name}_median",
        "run_name": f"{name}_median",
        "aggregate_name": "median",
        "real_time": real_time,
        "cpu_time": real_time,
        "time_unit": "ns",
        "worlds": 4096,
        "bodies": 7,
        "scene_body_count": 7,
        "steps": 64,
        "max_result_abs_error": max_error,
    }


def _packet_rows(max_error: float = 1e-12) -> dict[str, object]:
    return {
        "benchmarks": [
            _row("BM_Plan083SceneParityCpu/4096/64", 1_000_000.0),
            _row("BM_Plan083SceneParityCuda/4096/64", 500_000.0, max_error),
        ]
    }


def test_scene_parity_packet_accepts_reduced_same_scene_rows() -> None:
    module = _load_module()

    packet = module.make_packet(
        _packet_rows(),
        world_count=4096,
        step_count=64,
        tolerance=1e-9,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_scene_parity_packet"]
    assert row["row_id"] == "scene-parity-speedup"
    assert row["same_scene_cpu_gpu"] is True
    assert row["world_count"] == 4096
    assert row["body_count"] == 7
    assert row["step_count"] == 64
    assert row["speedup"] == 2.0
    assert row["meets_speedup_gate"] is True
    assert "Reduced state-batch scene parity only" in row["limitation_status"]


def test_scene_parity_packet_rejects_missing_gpu_row() -> None:
    module = _load_module()
    data = {"benchmarks": [_row("BM_Plan083SceneParityCpu/4096/64", 1_000_000.0)]}

    try:
        module.make_packet(
            data,
            world_count=4096,
            step_count=64,
            tolerance=1e-9,
            speedup_gate=1.25,
        )
    except module.SceneParityPacketError as exc:
        assert "missing median benchmark row: BM_Plan083SceneParityCuda/4096/64" in str(
            exc
        )
    else:
        raise AssertionError("missing GPU benchmark row should fail")


def test_scene_parity_packet_rejects_accuracy_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _packet_rows(max_error=1e-3),
            world_count=4096,
            step_count=64,
            tolerance=1e-9,
            speedup_gate=1.25,
        )
    except module.SceneParityPacketError as exc:
        assert "scene parity max error" in str(exc)
    else:
        raise AssertionError("high scene parity error should fail")
