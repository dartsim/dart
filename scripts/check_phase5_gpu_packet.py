#!/usr/bin/env python3
"""Validate the Phase 5 GPU go/no-go benchmark packet.

The packet is a JSON object that combines Google Benchmark rows with the
extra metadata that Google Benchmark cannot prove by itself:

{
  "phase5_gpu_packet": {
    "world_count": 4096,
    "body_count": 128,
    "step_count": 100,
    "includes_transfer_setup_compute_readback": true,
    "max_final_state_abs_error": 1e-12,
    "gpu_build_import_gate_passed": true,
    "compute_backend_boundaries_passed": true,
    "no_gpu_runtime_dependencies_passed": true,
    "phase5_benchmark_contract_passed": true
  },
  "benchmarks": [...]
}

This checker is intentionally not wired into routine CI; the GPU go/no-go is run
manually on a CUDA host rather than on a project-owned GPU runner. It is the
executable gate for that manual Phase 5 benchmark packet.

Use `--write-template <packet.json>` to create the manual evidence-packet shape,
then fill in the measured benchmark rows and metadata from the CUDA host before
running the validator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from benchmark_packet_utils import (
    batched_benchmark_row_schema_errors,
    benchmark_packet_timing_schema_errors,
    benchmark_row_name,
    canonical_benchmark_name,
    median_timing_by_name,
)

DEFAULT_CPU_PREFIX = "BM_Phase5RigidBodyBatchCpuBaseline"
DEFAULT_GPU_PREFIX = "BM_Phase5RigidBodyBatchGpu"
DEFAULT_MIN_WORLD_COUNT = 1024
DEFAULT_BODY_COUNT = 128
DEFAULT_STEP_COUNT = 100
DEFAULT_MIN_SPEEDUP = 1.25
DEFAULT_MAX_FINAL_STATE_ABS_ERROR = 1e-10
REQUIRED_EVIDENCE_FLAGS = (
    "gpu_build_import_gate_passed",
    "compute_backend_boundaries_passed",
    "no_gpu_runtime_dependencies_passed",
    "phase5_benchmark_contract_passed",
)


class Phase5PacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Phase 5 packet JSON file.",
    )
    parser.add_argument(
        "--write-template",
        type=Path,
        default=None,
        help="Write a Phase 5 packet template and exit.",
    )
    parser.add_argument(
        "--cpu-prefix",
        default=DEFAULT_CPU_PREFIX,
        help="Google Benchmark prefix for the CPU fallback row.",
    )
    parser.add_argument(
        "--gpu-prefix",
        default=DEFAULT_GPU_PREFIX,
        help="Google Benchmark prefix for the GPU prototype row.",
    )
    parser.add_argument(
        "--min-world-count",
        type=int,
        default=DEFAULT_MIN_WORLD_COUNT,
        help="Smallest allowed worldCount for a go decision.",
    )
    parser.add_argument(
        "--body-count",
        type=int,
        default=DEFAULT_BODY_COUNT,
        help="Required bodyCount for the representative workload.",
    )
    parser.add_argument(
        "--step-count",
        type=int,
        default=DEFAULT_STEP_COUNT,
        help="Required stepCount for the representative workload.",
    )
    parser.add_argument(
        "--min-speedup",
        type=float,
        default=DEFAULT_MIN_SPEEDUP,
        help="Required CPU median / GPU median full-workload speedup.",
    )
    parser.add_argument(
        "--max-final-state-abs-error",
        type=float,
        default=DEFAULT_MAX_FINAL_STATE_ABS_ERROR,
        help="Maximum allowed CPU/GPU final-state absolute error.",
    )
    return parser.parse_args(argv)


def _require_bool(metadata: dict, key: str) -> bool:
    value = metadata.get(key)
    if not isinstance(value, bool):
        raise Phase5PacketError(f"phase5_gpu_packet.{key} must be a boolean")
    return value


def _require_int(metadata: dict, key: str) -> int:
    value = metadata.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise Phase5PacketError(f"phase5_gpu_packet.{key} must be an integer")
    return value


def _require_float(metadata: dict, key: str) -> float:
    value = metadata.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise Phase5PacketError(f"phase5_gpu_packet.{key} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise Phase5PacketError(f"phase5_gpu_packet.{key} must be finite")
    return value


def _is_power_of_two(value: int) -> bool:
    return value > 0 and (value & (value - 1)) == 0


def _load_packet(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise Phase5PacketError("packet root must be a JSON object")
    return data


def make_packet_template(
    *,
    cpu_prefix: str = DEFAULT_CPU_PREFIX,
    gpu_prefix: str = DEFAULT_GPU_PREFIX,
    world_count: int = 4096,
    body_count: int = DEFAULT_BODY_COUNT,
    step_count: int = DEFAULT_STEP_COUNT,
) -> dict:
    cpu_name = f"{cpu_prefix}/{world_count}/{body_count}/{step_count}_median"
    gpu_name = f"{gpu_prefix}/{world_count}/{body_count}/{step_count}_median"
    return {
        "phase5_gpu_packet": {
            "world_count": world_count,
            "body_count": body_count,
            "step_count": step_count,
            "includes_transfer_setup_compute_readback": False,
            "max_final_state_abs_error": None,
            "gpu_build_import_gate_passed": False,
            "compute_backend_boundaries_passed": False,
            "no_gpu_runtime_dependencies_passed": False,
            "phase5_benchmark_contract_passed": False,
            "notes": [
                "Run on a CUDA host (the project does not maintain GPU CI).",
                (
                    "Set the evidence booleans to true only after the GPU "
                    + "build/import gate, backend-boundary check, "
                    + "no-GPU default/core dependency check, and Phase 5 "
                    + "benchmark-contract check have passed for the same change."
                ),
                (
                    "Set includes_transfer_setup_compute_readback to true only "
                    + "when the GPU row includes host/device transfer, setup, "
                    + "kernel/compute, and readback needed for final-state "
                    + "comparison."
                ),
                (
                    "Replace the placeholder median rows with Google Benchmark "
                    + "aggregate median rows from the CPU fallback and matching "
                    + "GPU prototype workload."
                ),
            ],
        },
        "benchmarks": [
            {
                "name": cpu_name,
                "run_name": cpu_name,
                "run_type": "aggregate",
                "aggregate_name": "median",
                "real_time": None,
                "time_unit": "ms",
                "backend": "cpu",
                "precision": "double-reference",
                "includes_transfer_time": False,
                "lane_count": world_count,
                "resolved_execution_shape": "homogeneous-batch",
                "step_count": step_count,
            },
            {
                "name": gpu_name,
                "run_name": gpu_name,
                "run_type": "aggregate",
                "aggregate_name": "median",
                "real_time": None,
                "time_unit": "ms",
                "backend": "cuda",
                "precision": "double-reference",
                "includes_transfer_time": True,
                "lane_count": world_count,
                "resolved_execution_shape": "homogeneous-batch",
                "step_count": step_count,
            },
        ],
    }


def write_packet_template(path: Path, template: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(template, indent=2) + "\n", encoding="utf-8")


def _validate_metadata(
    metadata: dict,
    min_world_count: int,
    body_count: int,
    step_count: int,
    max_final_state_abs_error: float,
) -> tuple[int, int, int, float]:
    world_count = _require_int(metadata, "world_count")
    actual_body_count = _require_int(metadata, "body_count")
    actual_step_count = _require_int(metadata, "step_count")
    timing_schema_errors = benchmark_packet_timing_schema_errors(
        metadata, "phase5_gpu_packet"
    )
    if timing_schema_errors:
        raise Phase5PacketError(timing_schema_errors[0])
    includes_full_workload = _require_bool(
        metadata, "includes_transfer_setup_compute_readback"
    )
    max_error = _require_float(metadata, "max_final_state_abs_error")

    if world_count < min_world_count:
        raise Phase5PacketError(
            f"world_count {world_count} is below the go-decision minimum "
            f"{min_world_count}"
        )
    if not _is_power_of_two(world_count):
        raise Phase5PacketError("world_count must be a power of two")
    if actual_body_count != body_count:
        raise Phase5PacketError(
            f"body_count {actual_body_count} does not match required {body_count}"
        )
    if actual_step_count != step_count:
        raise Phase5PacketError(
            f"step_count {actual_step_count} does not match required {step_count}"
        )
    if not includes_full_workload:
        raise Phase5PacketError(
            "packet must measure transfer, setup, kernel/compute, and readback"
        )
    if max_error > max_final_state_abs_error:
        raise Phase5PacketError(
            f"final-state error {max_error:.3g} exceeds "
            f"{max_final_state_abs_error:.3g}"
        )
    for key in REQUIRED_EVIDENCE_FLAGS:
        if not _require_bool(metadata, key):
            raise Phase5PacketError(
                f"phase5_gpu_packet.{key} must be true for a go/no-go packet"
            )

    return world_count, actual_body_count, actual_step_count, max_error


def _find_median_row(rows: list, canonical_name: str) -> dict | None:
    for row in rows:
        if not isinstance(row, dict):
            continue
        if row.get("aggregate_name") != "median":
            continue
        if canonical_benchmark_name(benchmark_row_name(row)) == canonical_name:
            return row
    return None


def validate_packet(
    data: dict,
    *,
    cpu_prefix: str = DEFAULT_CPU_PREFIX,
    gpu_prefix: str = DEFAULT_GPU_PREFIX,
    min_world_count: int = DEFAULT_MIN_WORLD_COUNT,
    body_count: int = DEFAULT_BODY_COUNT,
    step_count: int = DEFAULT_STEP_COUNT,
    min_speedup: float = DEFAULT_MIN_SPEEDUP,
    max_final_state_abs_error: float = DEFAULT_MAX_FINAL_STATE_ABS_ERROR,
) -> dict:
    metadata = data.get("phase5_gpu_packet")
    if not isinstance(metadata, dict):
        raise Phase5PacketError("packet is missing phase5_gpu_packet metadata")

    world_count, actual_body_count, actual_step_count, max_error = _validate_metadata(
        metadata,
        min_world_count,
        body_count,
        step_count,
        max_final_state_abs_error,
    )

    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise Phase5PacketError("packet is missing a Google Benchmark row list")

    cpu_name = f"{cpu_prefix}/{world_count}/{actual_body_count}/{actual_step_count}"
    gpu_name = f"{gpu_prefix}/{world_count}/{actual_body_count}/{actual_step_count}"
    timings = median_timing_by_name(rows)
    missing = [name for name in (cpu_name, gpu_name) if name not in timings]
    if missing:
        raise Phase5PacketError("missing median benchmark rows: " + ", ".join(missing))

    for name, expected_backend in ((cpu_name, "cpu"), (gpu_name, "cuda")):
        row = _find_median_row(rows, name)
        if row is None:
            raise Phase5PacketError(f"missing median benchmark row: {name}")
        errors = batched_benchmark_row_schema_errors(
            row, f"phase5_gpu_packet.benchmarks[{name}]"
        )
        if errors:
            raise Phase5PacketError(errors[0])
        if row["backend"] != expected_backend:
            raise Phase5PacketError(
                f"phase5_gpu_packet.benchmarks[{name}].backend must be "
                f"{expected_backend!r}"
            )
        if row["lane_count"] != world_count:
            raise Phase5PacketError(
                f"phase5_gpu_packet.benchmarks[{name}].lane_count must match "
                "phase5_gpu_packet.world_count"
            )
        if row["step_count"] != actual_step_count:
            raise Phase5PacketError(
                f"phase5_gpu_packet.benchmarks[{name}].step_count must match "
                "phase5_gpu_packet.step_count"
            )
    gpu_row = _find_median_row(rows, gpu_name)
    if not gpu_row["includes_transfer_time"]:
        raise Phase5PacketError(
            f"phase5_gpu_packet.benchmarks[{gpu_name}].includes_transfer_time "
            "must be true for the full GPU workload"
        )

    cpu_ns = timings[cpu_name]
    gpu_ns = timings[gpu_name]
    speedup = cpu_ns / gpu_ns
    if speedup < min_speedup:
        raise Phase5PacketError(
            f"GPU speedup {speedup:.3f} is below required {min_speedup:.3f}"
        )

    return {
        "world_count": world_count,
        "body_count": actual_body_count,
        "step_count": actual_step_count,
        "cpu_median_ns": cpu_ns,
        "gpu_median_ns": gpu_ns,
        "speedup": speedup,
        "max_final_state_abs_error": max_error,
    }


def _print_summary(summary: dict) -> None:
    print(
        "Phase 5 GPU packet accepted: "
        f"worldCount={summary['world_count']} "
        f"bodyCount={summary['body_count']} "
        f"stepCount={summary['step_count']} "
        f"speedup={summary['speedup']:.3f} "
        f"maxFinalStateAbsError={summary['max_final_state_abs_error']:.3g}"
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.write_template is not None:
        write_packet_template(
            args.write_template,
            make_packet_template(
                cpu_prefix=args.cpu_prefix,
                gpu_prefix=args.gpu_prefix,
                body_count=args.body_count,
                step_count=args.step_count,
            ),
        )
        print(f"Wrote Phase 5 GPU packet template: {args.write_template}")
        return 0

    if args.input is None:
        raise SystemExit("--input is required unless --write-template is used")

    try:
        summary = validate_packet(
            _load_packet(args.input),
            cpu_prefix=args.cpu_prefix,
            gpu_prefix=args.gpu_prefix,
            min_world_count=args.min_world_count,
            body_count=args.body_count,
            step_count=args.step_count,
            min_speedup=args.min_speedup,
            max_final_state_abs_error=args.max_final_state_abs_error,
        )
    except Phase5PacketError as exc:
        raise SystemExit(str(exc)) from exc

    _print_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
