#!/usr/bin/env python3
"""Write a validated Phase 5 CUDA go/no-go packet from benchmark JSON."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_phase5_gpu_packet as packet_check
from benchmark_packet_utils import canonical_benchmark_row_name

DEFAULT_OUTPUT = Path(".benchmark_results/phase5_cuda_packet.json")


class Phase5CudaPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        required=True,
        help="Google Benchmark JSON from bm_cuda_rigid_body_state_batch.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output Phase 5 packet JSON path.",
    )
    parser.add_argument(
        "--cpu-prefix",
        default=packet_check.DEFAULT_CPU_PREFIX,
        help="Google Benchmark prefix for the CPU fallback row.",
    )
    parser.add_argument(
        "--gpu-prefix",
        default=packet_check.DEFAULT_GPU_PREFIX,
        help="Google Benchmark prefix for the GPU prototype row.",
    )
    parser.add_argument(
        "--world-count",
        type=int,
        default=4096,
        help="Representative workload worldCount.",
    )
    parser.add_argument(
        "--body-count",
        type=int,
        default=packet_check.DEFAULT_BODY_COUNT,
        help="Representative workload bodyCount.",
    )
    parser.add_argument(
        "--step-count",
        type=int,
        default=packet_check.DEFAULT_STEP_COUNT,
        help="Representative workload stepCount.",
    )
    parser.add_argument(
        "--includes-transfer-setup-compute-readback",
        action="store_true",
        help="Assert the GPU row measured setup, transfer, compute, and readback.",
    )
    parser.add_argument(
        "--gpu-build-import-gate-passed",
        action="store_true",
        help="Assert the CUDA build/import gate passed for this change.",
    )
    parser.add_argument(
        "--compute-backend-boundaries-passed",
        action="store_true",
        help="Assert check-compute-backend-boundaries passed for this change.",
    )
    parser.add_argument(
        "--no-gpu-runtime-dependencies-passed",
        action="store_true",
        help="Assert check-no-gpu-runtime-dependencies passed for this change.",
    )
    parser.add_argument(
        "--phase5-benchmark-contract-passed",
        action="store_true",
        help="Assert check-phase5-cuda-benchmark-contract passed for this change.",
    )
    return parser.parse_args(argv)


def _load_benchmark_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise Phase5CudaPacketError("benchmark JSON root must be an object")
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise Phase5CudaPacketError("benchmark JSON has no benchmark row list")
    return data


def _matching_rows(
    rows: list[dict],
    *,
    prefix: str,
    world_count: int,
    body_count: int,
    step_count: int,
) -> list[dict]:
    target = f"{prefix}/{world_count}/{body_count}/{step_count}"
    return [row for row in rows if canonical_benchmark_row_name(row) == target]


def _finite_counter(row: dict, key: str) -> float | None:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def extract_max_final_state_abs_error(
    rows: list[dict],
    *,
    gpu_prefix: str,
    world_count: int,
    body_count: int,
    step_count: int,
) -> float:
    matches = _matching_rows(
        rows,
        prefix=gpu_prefix,
        world_count=world_count,
        body_count=body_count,
        step_count=step_count,
    )
    if not matches:
        raise Phase5CudaPacketError(
            "benchmark JSON is missing the representative GPU row "
            f"{gpu_prefix}/{world_count}/{body_count}/{step_count}"
        )

    median_values = [
        value
        for row in matches
        if row.get("aggregate_name") == "median"
        for value in [_finite_counter(row, "max_final_state_abs_error")]
        if value is not None
    ]
    if median_values:
        return max(median_values)

    values = [
        value
        for row in matches
        for value in [_finite_counter(row, "max_final_state_abs_error")]
        if value is not None
    ]
    if values:
        return max(values)

    raise Phase5CudaPacketError(
        "benchmark JSON is missing max_final_state_abs_error for the "
        "representative GPU row"
    )


def make_packet(data: dict, args: argparse.Namespace) -> dict:
    rows = data["benchmarks"]
    max_error = extract_max_final_state_abs_error(
        rows,
        gpu_prefix=args.gpu_prefix,
        world_count=args.world_count,
        body_count=args.body_count,
        step_count=args.step_count,
    )
    return {
        "phase5_gpu_packet": {
            "world_count": args.world_count,
            "body_count": args.body_count,
            "step_count": args.step_count,
            "includes_transfer_setup_compute_readback": (
                args.includes_transfer_setup_compute_readback
            ),
            "max_final_state_abs_error": max_error,
            "gpu_build_import_gate_passed": args.gpu_build_import_gate_passed,
            "compute_backend_boundaries_passed": (
                args.compute_backend_boundaries_passed
            ),
            "no_gpu_runtime_dependencies_passed": (
                args.no_gpu_runtime_dependencies_passed
            ),
            "phase5_benchmark_contract_passed": (args.phase5_benchmark_contract_passed),
        },
        "benchmarks": rows,
    }


def write_packet(path: Path, packet: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(_load_benchmark_json(args.benchmark_json), args)
        summary = packet_check.validate_packet(
            packet,
            cpu_prefix=args.cpu_prefix,
            gpu_prefix=args.gpu_prefix,
            body_count=args.body_count,
            step_count=args.step_count,
        )
    except (Phase5CudaPacketError, packet_check.Phase5PacketError) as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    print(f"Wrote Phase 5 CUDA packet: {args.output}")
    packet_check._print_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
