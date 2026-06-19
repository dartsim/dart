import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan091_cross_family_metrics_packet.py"
MANIFEST = (
    ROOT
    / "docs"
    / "plans"
    / "091-architecture-hardening"
    / "cross-family-metrics-corpus.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan091_cross_family_metrics_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _benchmark_row(name: str) -> dict[str, object]:
    return {
        "name": f"{name}_median",
        "run_name": f"{name}_median",
        "aggregate_name": "median",
        "real_time": 1.0,
        "cpu_time": 1.0,
        "time_unit": "ms",
        "step_count": 32,
        "kinetic_energy": 1.0,
        "potential_energy": -0.5,
        "total_energy": 0.5,
        "linear_momentum_x": 0.1,
        "linear_momentum_y": 0.0,
        "linear_momentum_z": 0.0,
        "angular_momentum_x": 0.0,
        "angular_momentum_y": 0.2,
        "angular_momentum_z": 0.0,
        "active_contact_count": 0,
        "max_penetration_depth": 0.0,
        "last_step_iterations": 0,
        "last_step_residual": 0.0,
    }


def _benchmark_data(manifest: dict[str, object]) -> dict[str, object]:
    rows = manifest["rows"]
    assert isinstance(rows, list)
    return {"benchmarks": [_benchmark_row(row["benchmark"]) for row in rows]}


def test_plan091_cross_family_packet_accepts_manifest_rows():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)

    packet = module.make_packet(_benchmark_data(manifest), manifest)

    metadata = packet[module.PACKET_KEY]
    assert metadata["metric_source"] == "dart::simulation::World::computeStepMetrics"
    assert metadata["row_count"] == len(manifest["rows"])
    first = metadata["rows"][0]
    assert first["scene_id"] == "rigid_head_on"
    assert first["resolved_solver_identity"] == "rigid-contact:sequential-impulse"
    assert first["metrics"]["total_energy"] == 0.5


def test_plan091_cross_family_packet_rejects_missing_row():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    data["benchmarks"].pop()

    with pytest.raises(module.Plan091CrossFamilyPacketError, match="missing"):
        module.make_packet(data, manifest)


def test_plan091_cross_family_packet_prefers_median_over_raw_rows():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    first_name = manifest["rows"][0]["benchmark"]

    median_row = data["benchmarks"][0]
    median_row["total_energy"] = 0.5
    raw_row = _benchmark_row(first_name)
    raw_row["name"] = first_name
    raw_row["run_name"] = first_name
    raw_row.pop("aggregate_name")
    raw_row["total_energy"] = 99.0
    data["benchmarks"].append(raw_row)

    packet = module.make_packet(data, manifest)

    first = packet[module.PACKET_KEY]["rows"][0]
    assert first["metrics"]["total_energy"] == 0.5


def test_plan091_cross_family_packet_rejects_missing_metric_counter():
    module = _load_module()
    manifest = module.load_manifest(MANIFEST)
    data = _benchmark_data(manifest)
    data["benchmarks"][0].pop("linear_momentum_x")

    with pytest.raises(
        module.Plan091CrossFamilyPacketError,
        match="linear_momentum_x",
    ):
        module.make_packet(data, manifest)
