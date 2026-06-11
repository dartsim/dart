#!/usr/bin/env python3
"""Run and validate PLAN-083's reduced CPU scene corpus packet."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from benchmark_packet_utils import (  # noqa: E402
    benchmark_row_name,
    benchmark_timing_field_errors,
    benchmark_timing_ns,
    canonical_benchmark_name,
)

DEFAULT_BENCHMARK_OUTPUT = Path(
    ".benchmark_results/plan083/cpu_scene_corpus/hanging_bridge_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/cpu_scene_corpus/hanging_bridge.json"
)

HANGING_BRIDGE_ROW = "BM_Plan083CpuScene_hanging_bridge_reduced_world_step"
DEFAULT_MAX_EQUALITY_RESIDUAL = 1e-8


class Plan083CpuScenePacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        default=DEFAULT_BENCHMARK_OUTPUT,
        help="Google Benchmark JSON path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_PACKET_OUTPUT,
        help="Validated packet output path.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type for the benchmark target.",
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="0.01s",
        help="Google Benchmark --benchmark_min_time value.",
    )
    parser.add_argument(
        "--benchmark-repetitions",
        type=int,
        default=3,
        help="Google Benchmark --benchmark_repetitions value.",
    )
    parser.add_argument(
        "--max-equality-residual",
        type=float,
        default=DEFAULT_MAX_EQUALITY_RESIDUAL,
        help="Maximum accepted final equality residual for the reduced scene.",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Validate an existing Google Benchmark JSON file.",
    )
    return parser.parse_args(argv)


def run_benchmark(args: argparse.Namespace) -> None:
    args.benchmark_json.parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_cpu_scene_corpus",
        "--build-type",
        args.build_type,
        "--",
        f"--benchmark_filter=^{HANGING_BRIDGE_ROW}$",
        f"--benchmark_min_time={args.benchmark_min_time}",
        f"--benchmark_repetitions={args.benchmark_repetitions}",
        "--benchmark_report_aggregates_only=true",
        f"--benchmark_out={args.benchmark_json.as_posix()}",
        "--benchmark_out_format=json",
    ]
    subprocess.run(command, check=True)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
    except json.JSONDecodeError as exc:
        raise Plan083CpuScenePacketError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise Plan083CpuScenePacketError(f"{path}: JSON root must be an object")
    return data


def _finite_number(row: Mapping[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise Plan083CpuScenePacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise Plan083CpuScenePacketError(
            f"{benchmark_row_name(row)} has non-finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _representative_row(rows: list[Mapping[str, Any]]) -> Mapping[str, Any]:
    found: Mapping[str, Any] | None = None
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical != HANGING_BRIDGE_ROW:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found = row
        errors.extend(benchmark_timing_field_errors(row, name))

    if found is None:
        errors.append(f"missing median benchmark row: {HANGING_BRIDGE_ROW}")

    if errors:
        raise Plan083CpuScenePacketError("\n".join(errors))
    return found


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    max_equality_residual: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083CpuScenePacketError("benchmark JSON has no benchmark rows")
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083CpuScenePacketError("benchmark JSON has non-object rows")

    row = _representative_row(typed_rows)
    timing_ns = benchmark_timing_ns(row)
    if not math.isfinite(timing_ns) or timing_ns <= 0.0:
        raise Plan083CpuScenePacketError("benchmark timing is not positive")

    failed_steps = _finite_number(row, "failed_steps")
    if failed_steps != 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced hanging bridge reported {failed_steps:g} failed steps"
        )

    final_residual = _finite_number(row, "final_equality_residual_norm")
    if final_residual > max_equality_residual:
        raise Plan083CpuScenePacketError(
            "reduced hanging bridge equality residual "
            f"{final_residual:.3g} exceeds {max_equality_residual:.3g}"
        )

    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    fixed_joint_count = int(_finite_number(row, "fixed_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 7:
        raise Plan083CpuScenePacketError(f"expected 7 rigid bodies, got {body_count}")
    if dynamic_body_count != 5:
        raise Plan083CpuScenePacketError(
            f"expected 5 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if fixed_joint_count != 4:
        raise Plan083CpuScenePacketError(
            f"expected 4 point-connection joints, got {fixed_joint_count}"
        )
    if active_articulation_constraints < 12:
        raise Plan083CpuScenePacketError(
            "expected fixed-joint articulation rows for the reduced bridge"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-02",
            "scene_id": "plan083_hanging_bridge",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "fixed_joint_count": fixed_joint_count,
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "traveler_height_m": _finite_number(row, "traveler_height_m"),
            "max_board_sag_m": _finite_number(row, "max_board_sag_m"),
            "limitation_status": (
                "Reduced runtime smoke packet only; paper-scale rods, "
                "codimensional deformables, and full Table 2 counts remain planned."
            ),
        },
        "benchmarks": rows,
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if not args.skip_run:
        run_benchmark(args)
    try:
        packet = make_packet(
            _load_json(args.benchmark_json),
            max_equality_residual=args.max_equality_residual,
        )
    except Plan083CpuScenePacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_cpu_scene_packet"]
    print(
        "PLAN-083 CPU scene packet OK: "
        f"{row['scene_id']} bodies={row['body_count']} "
        f"residual={row['final_equality_residual_norm']:.3g} "
        f"time={row['wall_time_ns']:.3g} ns"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
