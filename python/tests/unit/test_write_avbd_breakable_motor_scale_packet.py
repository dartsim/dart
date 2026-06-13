from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_breakable_motor_scale_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_breakable_motor_scale_packet",
        SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BENCHMARKS = (
    "BM_AvbdArticulatedBreakableMotorStep",
    "BM_AvbdArticulatedPrismaticBreakableMotorStep",
    "BM_AvbdArticulatedWorldPrismaticBreakableMotorStep",
    "BM_AvbdArticulatedWorldRevoluteBreakableMotorStep",
)
BENCHMARK_ARGS = (1, 8, 32)


def _benchmark_row(
    benchmark: str,
    arg: int,
    *,
    index: int = 0,
    run_type: str = "iteration",
    aggregate_name: str | None = None,
    motors: float | None = None,
    breakable_motors: float | None = None,
) -> dict[str, object]:
    name = f"{benchmark}/{arg}"
    row: dict[str, object] = {
        "breakable_motors": float(
            arg if breakable_motors is None else breakable_motors
        ),
        "cpu_time": float(arg * 1000 + index * 100),
        "iterations": 10,
        "motors": float(arg if motors is None else motors),
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
    motors: float | None = None,
    breakable_motors: float | None = None,
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
                    motors=motors,
                    breakable_motors=breakable_motors,
                )
            )
            rows.append(
                _benchmark_row(
                    benchmark,
                    arg,
                    index=index,
                    run_type="aggregate",
                    aggregate_name="median",
                    motors=motors,
                    breakable_motors=breakable_motors,
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


def test_avbd_breakable_motor_scale_packet_records_scale_data(
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
    assert packet["packet"] == "avbd_breakable_motor_scale"
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "breakable motor scale benchmark row family",
        "rigid_contact_solver": "none",
        "rigid_point_joint_solver": "avbd",
    }
    assert packet["scene"] == "avbd_breakable_motor_scale"
    assert packet["target"]["broad_motor_lifecycle_complete"] is False
    assert packet["target"]["scope"] == (
        "benchmark-only scale evidence for public articulated "
        "breakable motor rows over 1, 8, and 32 motors"
    )
    assert packet["benchmark"]["benchmark"] == "avbd_breakable_motor_scale"
    assert packet["benchmark"]["benchmarks"] == list(BENCHMARKS)
    assert packet["benchmark"]["invariants"] == {
        "break_force_n": 1.0e12,
        "breakable_motors": [1, 8, 32],
        "joint_anchor_scopes": [
            "same_multibody_pair",
            "world_link",
        ],
        "motor_kinds": ["prismatic", "revolute"],
        "motors": [1, 8, 32],
        "prismatic_max_force_n": 800.0,
        "prismatic_target_speed_m_per_s": 0.35,
        "revolute_max_torque_nm": 800.0,
        "revolute_target_speed_rad_per_s": 0.5,
        "time_step": 0.005,
    }
    assert len(packet["benchmark"]["scale_data"]) == len(BENCHMARKS) * len(
        BENCHMARK_ARGS
    )
    assert packet["benchmark"]["scale_data"][0] == {
        "benchmark": "BM_AvbdArticulatedBreakableMotorStep/1",
        "breakable_motors": 1,
        "cpu_time_per_step_ns": 1000.0,
        "joint_anchor_scope": "same_multibody_pair",
        "motor_kind": "revolute",
        "motors": 1,
        "real_time_per_step_ns": 1100.0,
        "time_unit": "ns",
        "variant": "same_multibody_revolute",
    }
    assert packet["benchmark"]["scale_data"][-1] == {
        "benchmark": "BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/32",
        "breakable_motors": 32,
        "cpu_time_per_step_ns": 32300.0,
        "joint_anchor_scope": "world_link",
        "motor_kind": "revolute",
        "motors": 32,
        "real_time_per_step_ns": 35500.0,
        "time_unit": "ns",
        "variant": "world_revolute",
    }
    assert (
        "source-demo and paper/site/video motor scene comparisons"
        in packet["remaining_gates"]
    )


def test_avbd_breakable_motor_scale_packet_rejects_missing_row(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, args=(1, 8))

    with pytest.raises(
        SystemExit,
        match="missing breakable-motor scale rows: .*BM_AvbdArticulatedBreakableMotorStep/32",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_motor_scale_packet_rejects_wrong_motor_count(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, motors=2.0)

    with pytest.raises(SystemExit, match="expected motors=1"):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_motor_scale_packet_rejects_wrong_breakable_count(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, breakable_motors=2.0)

    with pytest.raises(SystemExit, match="expected breakable_motors=1"):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_motor_scale_packet_rejects_unexpected_arg(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, include_unexpected_arg=True)

    with pytest.raises(
        SystemExit,
        match="unexpected breakable_motors argument 16",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_breakable_motor_scale_packet_rejects_missing_family(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, benchmarks=BENCHMARKS[:-1])

    with pytest.raises(
        SystemExit,
        match="BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/1",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])
