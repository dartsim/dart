from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_breakable_joint_scale_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_breakable_joint_scale_packet",
        SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BENCHMARKS = (
    "BM_AvbdRigidBreakableJointStep",
    "BM_AvbdRigidSphericalBreakableJointStep",
    "BM_AvbdArticulatedBreakableJointStep",
    "BM_AvbdArticulatedWorldSphericalBreakableJointStep",
    "BM_AvbdArticulatedSphericalPairBreakableJointStep",
)
BENCHMARK_ARGS = (1, 8, 32)


def _benchmark_row(
    benchmark: str,
    arg: int,
    *,
    index: int = 0,
    run_type: str = "iteration",
    aggregate_name: str | None = None,
    breakable_joints: float | None = None,
) -> dict[str, object]:
    name = f"{benchmark}/{arg}"
    row: dict[str, object] = {
        "breakable_joints": float(arg if breakable_joints is None else breakable_joints),
        "cpu_time": float(arg * 1000 + index * 100),
        "iterations": 10,
        "name": name,
        "real_time": float(arg * 1100 + index * 100),
        "run_name": name,
        "run_type": run_type,
        "time_unit": "ns",
    }
    if aggregate_name is not None:
        row["aggregate_name"] = aggregate_name
        row["aggregate_unit"] = "time"
        row["name"] = f"{name}_{aggregate_name}"
    return row


def _write_benchmark_json(
    tmp_path: Path,
    *,
    benchmarks: tuple[str, ...] = BENCHMARKS,
    args: tuple[int, ...] = BENCHMARK_ARGS,
    breakable_joints: float | None = None,
    include_unexpected_arg: bool = False,
) -> Path:
    rows = []
    for index, benchmark in enumerate(benchmarks):
        for arg in args:
            rows.append(
                _benchmark_row(
                    benchmark,
                    arg,
                    index=index,
                    breakable_joints=breakable_joints,
                )
            )
            rows.append(
                _benchmark_row(
                    benchmark,
                    arg,
                    index=index,
                    run_type="aggregate",
                    aggregate_name="median",
                    breakable_joints=breakable_joints,
                )
            )
    if include_unexpected_arg:
        rows.append(_benchmark_row(BENCHMARKS[0], 16))
    benchmark_json = {
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
    path.write_text(json.dumps(benchmark_json), encoding="utf-8")
    return path


def test_avbd_breakable_joint_scale_packet_records_scale_data(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
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

    packet = json.loads(output.read_text())
    assert packet["schema_version"] == 2
    assert packet["packet"] == "avbd_breakable_joint_scale"
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "breakable joint scale benchmark row family",
        "rigid_contact_solver": "none",
        "rigid_point_joint_solver": "avbd",
    }
    assert packet["scene"] == "avbd_breakable_joint_scale"
    assert packet["target"]["broad_breakable_constraint_complete"] is False
    assert packet["target"]["scope"] == (
        "benchmark-only scale evidence for public fixed and spherical "
        "breakable point-joint rows over 1, 8, and 32 constraints"
    )
    assert packet["benchmark"]["benchmark"] == "avbd_breakable_joint_scale"
    assert packet["benchmark"]["benchmarks"] == list(BENCHMARKS)
    assert packet["benchmark"]["invariants"] == {
        "break_force_n": 1.0e12,
        "breakable_joints": [1, 8, 32],
        "gravity_m_per_s2": [0.0, -9.81, 0.0],
        "joint_anchor_scopes": [
            "rigid_body_chain",
            "same_multibody_pair",
            "world_link",
        ],
        "time_step": 0.005,
    }
    assert len(packet["benchmark"]["scale_data"]) == len(BENCHMARKS) * len(
        BENCHMARK_ARGS
    )
    assert packet["benchmark"]["scale_data"][0] == {
        "benchmark": "BM_AvbdRigidBreakableJointStep/1",
        "breakable_joints": 1,
        "cpu_time_per_step_ns": 1000.0,
        "joint_anchor_scope": "rigid_body_chain",
        "real_time_per_step_ns": 1100.0,
        "time_unit": "ns",
        "variant": "rigid_fixed",
    }
    assert packet["benchmark"]["scale_data"][-1] == {
        "benchmark": "BM_AvbdArticulatedSphericalPairBreakableJointStep/32",
        "breakable_joints": 32,
        "cpu_time_per_step_ns": 32400.0,
        "joint_anchor_scope": "same_multibody_pair",
        "real_time_per_step_ns": 35600.0,
        "time_unit": "ns",
        "variant": "articulated_spherical_pair",
    }
    assert "visual breakable-wall or fracture corpus scene" in packet[
        "remaining_gates"
    ]


def test_avbd_breakable_joint_scale_packet_rejects_missing_row(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, args=(1, 8))

    with pytest.raises(
        SystemExit,
        match="missing breakable-joint scale rows: .*BM_AvbdRigidBreakableJointStep/32",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_joint_scale_packet_rejects_wrong_counter(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, breakable_joints=2.0)

    with pytest.raises(SystemExit, match="expected breakable_joints=1"):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_joint_scale_packet_rejects_unexpected_arg(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, include_unexpected_arg=True)

    with pytest.raises(
        SystemExit,
        match="unexpected breakable_joints argument 16",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_joint_scale_packet_rejects_missing_family(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, benchmarks=BENCHMARKS[:-1])

    with pytest.raises(
        SystemExit,
        match="BM_AvbdArticulatedSphericalPairBreakableJointStep/1",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])
