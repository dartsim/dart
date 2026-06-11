#!/usr/bin/env python3
"""Run and validate the private GPU reduced scene-parity packet."""

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
    ".benchmark_results/plan083/gpu/scene_parity_profile.json"
)
DEFAULT_PACKET_OUTPUT = Path(".benchmark_results/plan083/gpu/scene_parity_speedup.json")

DEFAULT_SCENE_ID = "plan083_hanging_bridge_reduced_state_batch"
DEFAULT_WORLD_COUNT = 4096
DEFAULT_STEP_COUNT = 64
DEFAULT_TOLERANCE = 1e-9
DEFAULT_SPEEDUP_GATE = 1.25


class SceneParityPacketError(RuntimeError):
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
        "--world-count",
        type=int,
        default=DEFAULT_WORLD_COUNT,
        help="Replicated reduced scene count.",
    )
    parser.add_argument(
        "--step-count",
        type=int,
        default=DEFAULT_STEP_COUNT,
        help="Rigid state-batch rollout step count.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU state absolute error.",
    )
    parser.add_argument(
        "--speedup-gate",
        type=float,
        default=DEFAULT_SPEEDUP_GATE,
        help="Policy speedup gate recorded in the packet; parity does not require it.",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Validate an existing Google Benchmark JSON file.",
    )
    return parser.parse_args(argv)


def run_benchmark(args: argparse.Namespace) -> None:
    args.benchmark_json.parent.mkdir(parents=True, exist_ok=True)
    filter_expr = (
        "^BM_Plan083SceneParity(Cpu|Cuda)"
        f"/{args.world_count}/{args.step_count}(/real_time)?$"
    )
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_gpu_scene_parity",
        "--build-type",
        args.build_type,
        "--",
        f"--benchmark_filter={filter_expr}",
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
        raise SceneParityPacketError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise SceneParityPacketError(f"{path}: JSON root must be an object")
    return data


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _counter(row: Mapping[str, Any], key: str) -> float:
    value = _finite_number(row.get(key))
    if value is None:
        raise SceneParityPacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _representative_rows(
    rows: list[Mapping[str, Any]], world_count: int, step_count: int
) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    cpu_name = f"BM_Plan083SceneParityCpu/{world_count}/{step_count}"
    gpu_name = f"BM_Plan083SceneParityCuda/{world_count}/{step_count}"
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in {cpu_name, gpu_name}:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[canonical] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for expected in (cpu_name, gpu_name):
        if expected not in found:
            errors.append(f"missing median benchmark row: {expected}")

    if errors:
        raise SceneParityPacketError("\n".join(errors))

    return found[cpu_name], found[gpu_name]


def _matching_int_counter(
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    key: str,
) -> int:
    cpu_count = int(_counter(cpu_row, key))
    gpu_count = int(_counter(gpu_row, key))
    if cpu_count != gpu_count:
        raise SceneParityPacketError(
            f"{key} count {cpu_count} != GPU {key} count {gpu_count}"
        )
    return cpu_count


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    world_count: int,
    step_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise SceneParityPacketError("benchmark JSON has no benchmark rows")
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise SceneParityPacketError("benchmark JSON has non-object rows")

    cpu_row, gpu_row = _representative_rows(typed_rows, world_count, step_count)
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise SceneParityPacketError("CPU benchmark timing is not positive")
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise SceneParityPacketError("GPU benchmark timing is not positive")

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise SceneParityPacketError(
            f"scene parity max error {max_error:.3g} exceeds {tolerance:.3g}"
        )

    worlds = _matching_int_counter(cpu_row, gpu_row, "worlds")
    if worlds != world_count:
        raise SceneParityPacketError(f"expected {world_count} worlds, got {worlds}")
    steps = _matching_int_counter(cpu_row, gpu_row, "steps")
    if steps != step_count:
        raise SceneParityPacketError(f"expected {step_count} steps, got {steps}")
    bodies = _matching_int_counter(cpu_row, gpu_row, "bodies")
    scene_bodies = _matching_int_counter(cpu_row, gpu_row, "scene_body_count")
    speedup = cpu_ns / gpu_ns

    return {
        "plan083_gpu_scene_parity_packet": {
            "row_id": "scene-parity-speedup",
            "scene_id": DEFAULT_SCENE_ID,
            "same_scene_cpu_gpu": True,
            "state_path": "private rigid-body state-batch CUDA rollout",
            "world_count": worlds,
            "scene_body_count": scene_bodies,
            "body_count": bodies,
            "step_count": steps,
            "time_step": 0.005,
            "max_result_abs_error": max_error,
            "result_abs_error_tolerance": tolerance,
            "speedup": speedup,
            "speedup_gate": speedup_gate,
            "meets_speedup_gate": speedup >= speedup_gate,
            "cpu_benchmark_row": _packet_row_name(cpu_row),
            "gpu_benchmark_row": _packet_row_name(gpu_row),
            "limitation_status": (
                "Reduced state-batch scene parity only; this does not run a "
                "GPU World::step path, contact candidate construction, CCD, "
                "barrier/friction assembly, sparse equality reduction, or "
                "global Newton solve."
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
            world_count=args.world_count,
            step_count=args.step_count,
            tolerance=args.tolerance,
            speedup_gate=args.speedup_gate,
        )
    except SceneParityPacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_gpu_scene_parity_packet"]
    print(
        "Reduced scene parity packet OK: "
        f"scene={row['scene_id']} worlds={row['world_count']} "
        f"bodies={row['body_count']} steps={row['step_count']} "
        f"max_error={row['max_result_abs_error']:.3g} "
        f"speedup={row['speedup']:.3f}x "
        f"meets_gate={row['meets_speedup_gate']}"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
