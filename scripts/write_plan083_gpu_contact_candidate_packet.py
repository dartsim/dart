#!/usr/bin/env python3
"""Run and validate PLAN-083's private GPU contact-candidate packet."""

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
    ".benchmark_results/plan083/gpu/contact_candidates_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/contact_candidates_parity.json"
)

DEFAULT_STENCIL_COUNT = 65536
DEFAULT_TOLERANCE = 1e-10
DEFAULT_SPEEDUP_GATE = 1.25
REQUIRED_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "solve",
    "device_to_host",
    "readback",
}


class Plan083GpuContactCandidatePacketError(RuntimeError):
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
        "--stencil-count",
        type=int,
        default=DEFAULT_STENCIL_COUNT,
        help="Representative point-triangle stencil count for the packet row.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU squared-distance absolute error.",
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
        "^BM_Plan083(EdgeEdge)?ContactCandidate(Cpu|Cuda)"
        f"/{args.stencil_count}(/real_time)?$"
    )
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_gpu_contact_candidates",
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
        raise Plan083GpuContactCandidatePacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise Plan083GpuContactCandidatePacketError(
            f"{path}: JSON root must be an object"
        )
    return data


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _counter(row: Mapping[str, Any], key: str) -> float:
    value = _finite_number(row.get(key))
    if value is None:
        raise Plan083GpuContactCandidatePacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _expected_row_names(stencil_count: int) -> dict[str, tuple[str, str]]:
    return {
        "point_triangle": (
            f"BM_Plan083ContactCandidateCpu/{stencil_count}",
            f"BM_Plan083ContactCandidateCuda/{stencil_count}",
        ),
        "edge_edge": (
            f"BM_Plan083EdgeEdgeContactCandidateCpu/{stencil_count}",
            f"BM_Plan083EdgeEdgeContactCandidateCuda/{stencil_count}",
        ),
    }


def _representative_rows(
    rows: list[Mapping[str, Any]], stencil_count: int
) -> dict[str, Mapping[str, Any]]:
    expected_rows = _expected_row_names(stencil_count)
    expected_names = {name for names in expected_rows.values() for name in names}
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in expected_names:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[canonical] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for expected in expected_names:
        if expected not in found:
            errors.append(f"missing median benchmark row: {expected}")

    if errors:
        raise Plan083GpuContactCandidatePacketError("\n".join(errors))

    return found


def _validate_primitive_family(
    *,
    family: str,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    stencil_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} candidate max error {max_error:.3g} exceeds tolerance "
            f"{tolerance:.3g}"
        )

    cpu_accepted = _counter(cpu_row, "accepted_count")
    gpu_accepted = _counter(gpu_row, "gpu_accepted_count")
    if int(cpu_accepted) != int(gpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{family} CPU accepted count {cpu_accepted:g} != GPU accepted "
            f"count {gpu_accepted:g}"
        )

    cpu_stencils = int(_counter(cpu_row, "stencils"))
    gpu_stencils = int(_counter(gpu_row, "stencils"))
    if cpu_stencils != stencil_count or gpu_stencils != stencil_count:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} expected {stencil_count} stencils, got "
            f"CPU={cpu_stencils}, GPU={gpu_stencils}"
        )

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - timing_ns.keys()
    if missing:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} packet timing is missing {sorted(missing)}"
        )

    return {
        "stencil_count": stencil_count,
        "accepted_count": int(cpu_accepted),
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    stencil_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083GpuContactCandidatePacketError(
            "benchmark JSON has no benchmark rows"
        )
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083GpuContactCandidatePacketError(
            "benchmark JSON has non-object rows"
        )

    representative_rows = _representative_rows(typed_rows, stencil_count)
    primitive_families = {}
    for family, (cpu_name, gpu_name) in _expected_row_names(stencil_count).items():
        primitive_families[family] = _validate_primitive_family(
            family=family,
            cpu_row=representative_rows[cpu_name],
            gpu_row=representative_rows[gpu_name],
            stencil_count=stencil_count,
            tolerance=tolerance,
            speedup_gate=speedup_gate,
        )

    point_triangle = primitive_families["point_triangle"]
    max_error = max(
        family["max_result_abs_error"] for family in primitive_families.values()
    )
    speedup = min(family["speedup"] for family in primitive_families.values())
    meets_speedup_gate = all(
        family["meets_speedup_gate"] for family in primitive_families.values()
    )

    return {
        "plan083_gpu_contact_candidate_packet": {
            "row_id": "contact-stencils-candidate-filtering",
            "same_scene_cpu_gpu": True,
            "primitive_families": primitive_families,
            "stencil_count": stencil_count * len(primitive_families),
            "accepted_count": sum(
                family["accepted_count"] for family in primitive_families.values()
            ),
            "max_result_abs_error": max_error,
            "result_abs_error_tolerance": tolerance,
            "speedup": speedup,
            "speedup_gate": speedup_gate,
            "meets_speedup_gate": meets_speedup_gate,
            "timing_ns": point_triangle["timing_ns"],
            "cpu_benchmark_row": point_triangle["cpu_benchmark_row"],
            "gpu_benchmark_row": point_triangle["gpu_benchmark_row"],
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
            stencil_count=args.stencil_count,
            tolerance=args.tolerance,
            speedup_gate=args.speedup_gate,
        )
    except Plan083GpuContactCandidatePacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_gpu_contact_candidate_packet"]
    print(
        "PLAN-083 GPU contact candidate packet OK: "
        f"stencils={row['stencil_count']} max_error={row['max_result_abs_error']:.3g} "
        f"speedup={row['speedup']:.3f}x "
        f"meets_gate={row['meets_speedup_gate']}"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
