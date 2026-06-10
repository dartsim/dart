import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_gpu_psd_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_gpu_psd_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(name, real_time, **counters):
    row = {
        "name": f"{name}_median",
        "run_name": f"{name}_median",
        "aggregate_name": "median",
        "real_time": real_time,
        "cpu_time": real_time,
        "time_unit": "ns",
    }
    row.update(counters)
    return row


def _benchmark_packet(*, max_error=1e-12, gpu_time=100.0):
    return {
        "benchmarks": [
            _row("BM_Plan083PsdProjectionCpu/4096", 200.0),
            _row(
                "BM_Plan083PsdProjectionCuda/4096",
                gpu_time,
                dimension=12.0,
                max_result_abs_error=max_error,
                host_setup_ns=1.0,
                host_to_device_ns=2.0,
                kernel_ns=3.0,
                device_to_host_ns=4.0,
            ),
        ]
    }


def test_plan083_gpu_psd_packet_accepts_valid_benchmark_json(tmp_path):
    module = _load_module()

    packet = module.make_packet(
        _benchmark_packet(),
        block_count=4096,
        tolerance=1e-9,
        min_speedup=1.25,
    )
    output = tmp_path / "packet.json"
    module.write_packet(output, packet)

    written = json.loads(output.read_text(encoding="utf-8"))
    row = written["plan083_gpu_psd_projection_packet"]
    assert row["same_scene_cpu_gpu"] is True
    assert row["block_dimension"] == 12
    assert row["block_count"] == 4096
    assert row["speedup"] == 2.0
    assert row["timing_ns"] == {
        "setup": 1.0,
        "host_to_device": 2.0,
        "kernel": 3.0,
        "solve": 0.0,
        "device_to_host": 4.0,
        "readback": 0.0,
    }


def test_plan083_gpu_psd_packet_rejects_accuracy_failure():
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_packet(max_error=1e-6),
            block_count=4096,
            tolerance=1e-9,
            min_speedup=1.25,
        )
    except module.Plan083GpuPsdPacketError as exc:
        assert "exceeds tolerance" in str(exc)
    else:
        raise AssertionError("expected accuracy failure")


def test_plan083_gpu_psd_packet_rejects_speedup_failure():
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_packet(gpu_time=180.0),
            block_count=4096,
            tolerance=1e-9,
            min_speedup=1.25,
        )
    except module.Plan083GpuPsdPacketError as exc:
        assert "below required" in str(exc)
    else:
        raise AssertionError("expected speedup failure")


def test_plan083_gpu_psd_packet_requires_representative_rows():
    module = _load_module()

    try:
        module.make_packet(
            {"benchmarks": [_row("BM_Plan083PsdProjectionCpu/1024", 200.0)]},
            block_count=4096,
            tolerance=1e-9,
            min_speedup=1.25,
        )
    except module.Plan083GpuPsdPacketError as exc:
        assert "missing median benchmark row" in str(exc)
    else:
        raise AssertionError("expected missing row failure")
