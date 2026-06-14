#!/usr/bin/env python3
"""Run and validate PLAN-083's private GPU CCD/line-search packet."""

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
    ".benchmark_results/plan083/gpu/ccd_line_search_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/ccd_line_search_parity.json"
)

DEFAULT_PAIR_COUNT = 65536
DEFAULT_TOLERANCE = 1e-6
DEFAULT_SPEEDUP_GATE = 1.25
REQUIRED_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "solve",
    "device_to_host",
    "readback",
}


class Plan083GpuCcdLineSearchPacketError(RuntimeError):
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
        "--pair-count",
        type=int,
        default=DEFAULT_PAIR_COUNT,
        help="Representative CCD pair count per primitive family.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU step-bound absolute error.",
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
        "^BM_Plan083"
        "("
        "EdgeEdge|RigidCurvedPointTriangle|RigidCurvedEdgeEdge|"
        "SceneRuntimePointTriangle|SceneRuntimeEdgeEdge|SceneRuntimeCombined"
        ")?"
        "CcdLineSearch(Cpu|Cuda)"
        f"/{args.pair_count}(/real_time)?$"
    )
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_gpu_ccd_line_search",
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
        raise Plan083GpuCcdLineSearchPacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise Plan083GpuCcdLineSearchPacketError(f"{path}: JSON root must be an object")
    return data


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _counter(row: Mapping[str, Any], key: str) -> float:
    value = _finite_number(row.get(key))
    if value is None:
        raise Plan083GpuCcdLineSearchPacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _expected_row_names(pair_count: int) -> dict[str, tuple[str, str]]:
    return {
        "point_triangle": (
            f"BM_Plan083CcdLineSearchCpu/{pair_count}",
            f"BM_Plan083CcdLineSearchCuda/{pair_count}",
        ),
        "edge_edge": (
            f"BM_Plan083EdgeEdgeCcdLineSearchCpu/{pair_count}",
            f"BM_Plan083EdgeEdgeCcdLineSearchCuda/{pair_count}",
        ),
        "rigid_curved_point_triangle": (
            f"BM_Plan083RigidCurvedPointTriangleCcdLineSearchCpu/{pair_count}",
            f"BM_Plan083RigidCurvedPointTriangleCcdLineSearchCuda/{pair_count}",
        ),
        "rigid_curved_edge_edge": (
            f"BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCpu/{pair_count}",
            f"BM_Plan083RigidCurvedEdgeEdgeCcdLineSearchCuda/{pair_count}",
        ),
        "scene_runtime_point_triangle": (
            f"BM_Plan083SceneRuntimePointTriangleCcdLineSearchCpu/{pair_count}",
            f"BM_Plan083SceneRuntimePointTriangleCcdLineSearchCuda/{pair_count}",
        ),
        "scene_runtime_edge_edge": (
            f"BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCpu/{pair_count}",
            f"BM_Plan083SceneRuntimeEdgeEdgeCcdLineSearchCuda/{pair_count}",
        ),
        "scene_runtime_combined": (
            f"BM_Plan083SceneRuntimeCombinedCcdLineSearchCpu/{pair_count}",
            f"BM_Plan083SceneRuntimeCombinedCcdLineSearchCuda/{pair_count}",
        ),
    }


def _representative_rows(
    rows: list[Mapping[str, Any]], pair_count: int
) -> dict[str, Mapping[str, Any]]:
    expected_rows = _expected_row_names(pair_count)
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
        raise Plan083GpuCcdLineSearchPacketError("\n".join(errors))

    return found


def _validate_primitive_family(
    *,
    family: str,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    pair_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} CCD max error {max_error:.3g} exceeds tolerance "
            f"{tolerance:.3g}"
        )

    cpu_hits = _counter(cpu_row, "hits")
    gpu_hits = _counter(gpu_row, "gpu_hits")
    if int(cpu_hits) != int(gpu_hits):
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} CPU hit count {cpu_hits:g} != GPU hit count {gpu_hits:g}"
        )

    cpu_pairs = int(_counter(cpu_row, "pairs"))
    gpu_pairs = int(_counter(gpu_row, "pairs"))
    is_scene_runtime = family.startswith("scene_runtime_")
    if cpu_pairs != gpu_pairs:
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} CPU pair count {cpu_pairs} != GPU pair count {gpu_pairs}"
        )
    if is_scene_runtime:
        if cpu_pairs <= 0:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} must record at least one runtime CCD pair"
            )
    elif cpu_pairs != pair_count:
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} expected {pair_count} pairs, got "
            f"CPU={cpu_pairs}, GPU={gpu_pairs}"
        )

    scene_counters: dict[str, int] = {}
    analytic_reference_counters: dict[str, float | int | str] = {}
    if is_scene_runtime:
        required_scene_counters = [
            "scene_bodies",
            "scene_nodes",
            "scene_triangles",
        ]
        if family == "scene_runtime_point_triangle":
            required_scene_counters.extend(
                [
                    "runtime_point_triangle_candidates",
                    "static_triangle_point_triangle_candidates",
                    "moving_triangle_point_triangle_candidates",
                ]
            )
        if family == "scene_runtime_edge_edge":
            required_scene_counters.append("runtime_edge_edge_candidates")
        if family == "scene_runtime_combined":
            required_scene_counters.extend(
                [
                    "point_triangle_pairs",
                    "edge_edge_pairs",
                    "point_triangle_hits",
                    "edge_edge_hits",
                    "runtime_point_triangle_candidates",
                    "static_triangle_point_triangle_candidates",
                    "moving_triangle_point_triangle_candidates",
                    "runtime_edge_edge_candidates",
                ]
            )
        for key in required_scene_counters:
            cpu_value = int(_counter(cpu_row, key))
            gpu_value = int(_counter(gpu_row, key))
            if cpu_value != gpu_value:
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} CPU {key} {cpu_value} != GPU {key} {gpu_value}"
                )
            scene_counters[key] = cpu_value
        if (
            family == "scene_runtime_point_triangle"
            and scene_counters["runtime_point_triangle_candidates"] != cpu_pairs
        ):
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} runtime point-triangle candidate count "
                f"{scene_counters['runtime_point_triangle_candidates']} "
                f"!= pairs {cpu_pairs}"
            )
        if (
            family == "scene_runtime_point_triangle"
            and (
                scene_counters["static_triangle_point_triangle_candidates"]
                + scene_counters["moving_triangle_point_triangle_candidates"]
            )
            != cpu_pairs
        ):
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} static/moving point-triangle candidate counts "
                f"{scene_counters['static_triangle_point_triangle_candidates']} "
                f"+ {scene_counters['moving_triangle_point_triangle_candidates']} "
                f"!= pairs {cpu_pairs}"
            )
        if (
            family == "scene_runtime_edge_edge"
            and scene_counters["runtime_edge_edge_candidates"] != cpu_pairs
        ):
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} runtime edge-edge candidate count "
                f"{scene_counters['runtime_edge_edge_candidates']} "
                f"!= pairs {cpu_pairs}"
            )
        if family == "scene_runtime_combined":
            if (
                scene_counters["point_triangle_pairs"]
                + scene_counters["edge_edge_pairs"]
                != cpu_pairs
            ):
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} point-triangle/edge-edge pair counts "
                    f"{scene_counters['point_triangle_pairs']} + "
                    f"{scene_counters['edge_edge_pairs']} != pairs {cpu_pairs}"
                )
            if scene_counters["point_triangle_hits"] + scene_counters[
                "edge_edge_hits"
            ] != int(cpu_hits):
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} point-triangle/edge-edge hit counts "
                    f"{scene_counters['point_triangle_hits']} + "
                    f"{scene_counters['edge_edge_hits']} != hits {int(cpu_hits)}"
                )
            if (
                scene_counters["runtime_point_triangle_candidates"]
                != scene_counters["point_triangle_pairs"]
            ):
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} runtime point-triangle candidate count "
                    f"{scene_counters['runtime_point_triangle_candidates']} "
                    f"!= point-triangle pairs "
                    f"{scene_counters['point_triangle_pairs']}"
                )
            if (
                scene_counters["static_triangle_point_triangle_candidates"]
                + scene_counters["moving_triangle_point_triangle_candidates"]
                != scene_counters["point_triangle_pairs"]
            ):
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} static/moving point-triangle candidate counts "
                    f"{scene_counters['static_triangle_point_triangle_candidates']} "
                    f"+ {scene_counters['moving_triangle_point_triangle_candidates']} "
                    f"!= point-triangle pairs "
                    f"{scene_counters['point_triangle_pairs']}"
                )
            if (
                scene_counters["runtime_edge_edge_candidates"]
                != scene_counters["edge_edge_pairs"]
            ):
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} runtime edge-edge candidate count "
                    f"{scene_counters['runtime_edge_edge_candidates']} "
                    f"!= edge-edge pairs {scene_counters['edge_edge_pairs']}"
                )

    if family.startswith("rigid_curved_"):
        required_reference_counters = [
            "analytic_reference_pairs",
            "analytic_reference_hits",
            "analytic_reference_hit_mismatches",
            "analytic_reference_min_step_bound",
            "max_sampled_reference_overshoot",
            "max_sampled_reference_conservative_gap",
        ]
        for key in required_reference_counters:
            cpu_value = _counter(cpu_row, key)
            gpu_value = _counter(gpu_row, key)
            if abs(cpu_value - gpu_value) > tolerance:
                raise Plan083GpuCcdLineSearchPacketError(
                    f"{family} CPU {key} {cpu_value:g} != GPU {key} {gpu_value:g}"
                )
            analytic_reference_counters[key] = (
                int(cpu_value)
                if key
                in {
                    "analytic_reference_pairs",
                    "analytic_reference_hits",
                    "analytic_reference_hit_mismatches",
                }
                else cpu_value
            )

        if analytic_reference_counters["analytic_reference_pairs"] <= 0:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} must record analytic reference pairs"
            )
        if analytic_reference_counters["analytic_reference_hits"] <= 0:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} must record analytic reference hits"
            )
        if analytic_reference_counters["analytic_reference_hit_mismatches"] != 0:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} analytic reference hit mismatches must be zero"
            )
        if analytic_reference_counters["max_sampled_reference_overshoot"] > tolerance:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} sampled curved CCD overshoots analytic reference by "
                f"{analytic_reference_counters['max_sampled_reference_overshoot']:.3g}"
            )
        analytic_reference_counters["analytic_reference_method"] = (
            "rigid_ipc_curved_accd_prefix"
        )

    segment_count = cpu_pairs
    samples_per_pair = 1
    has_segment_counters = (
        "segments" in cpu_row or "segments" in gpu_row or "gpu_segments" in gpu_row
    )
    if has_segment_counters:
        cpu_segments = int(_counter(cpu_row, "segments"))
        gpu_segments_key = "gpu_segments" if "gpu_segments" in gpu_row else "segments"
        gpu_segments = int(_counter(gpu_row, gpu_segments_key))
        cpu_samples_per_pair = int(_counter(cpu_row, "samples_per_pair"))
        gpu_samples_per_pair = int(_counter(gpu_row, "samples_per_pair"))
        if cpu_segments != gpu_segments:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} CPU segment count {cpu_segments} != "
                f"GPU segment count {gpu_segments}"
            )
        if cpu_samples_per_pair != gpu_samples_per_pair:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} CPU samples_per_pair {cpu_samples_per_pair} != "
                f"GPU samples_per_pair {gpu_samples_per_pair}"
            )
        if cpu_samples_per_pair <= 0:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} samples_per_pair must be positive"
            )
        if cpu_segments != cpu_pairs * cpu_samples_per_pair:
            raise Plan083GpuCcdLineSearchPacketError(
                f"{family} expected {cpu_pairs * cpu_samples_per_pair} "
                f"segments, got {cpu_segments}"
            )
        segment_count = cpu_segments
        samples_per_pair = cpu_samples_per_pair

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
        raise Plan083GpuCcdLineSearchPacketError(
            f"{family} packet timing is missing {sorted(missing)}"
        )

    family_packet = {
        "pair_count": cpu_pairs,
        "segment_count": segment_count,
        "samples_per_pair": samples_per_pair,
        "hit_count": int(cpu_hits),
        "min_step_bound": _counter(cpu_row, "min_step_bound"),
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }
    if is_scene_runtime:
        family_packet["nominal_pair_count"] = pair_count
        family_packet.update(scene_counters)
    if analytic_reference_counters:
        family_packet.update(analytic_reference_counters)
    return family_packet


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    pair_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083GpuCcdLineSearchPacketError("benchmark JSON has no benchmark rows")
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083GpuCcdLineSearchPacketError("benchmark JSON has non-object rows")

    representative_rows = _representative_rows(typed_rows, pair_count)
    primitive_families = {}
    for family, (cpu_name, gpu_name) in _expected_row_names(pair_count).items():
        primitive_families[family] = _validate_primitive_family(
            family=family,
            cpu_row=representative_rows[cpu_name],
            gpu_row=representative_rows[gpu_name],
            pair_count=pair_count,
            tolerance=tolerance,
            speedup_gate=speedup_gate,
        )

    scene_point_triangle = primitive_families["scene_runtime_point_triangle"]
    scene_edge_edge = primitive_families["scene_runtime_edge_edge"]
    scene_combined = primitive_families["scene_runtime_combined"]
    if (
        scene_combined["point_triangle_pairs"] != scene_point_triangle["pair_count"]
        or scene_combined["point_triangle_hits"] != scene_point_triangle["hit_count"]
        or scene_combined["runtime_point_triangle_candidates"]
        != scene_point_triangle["runtime_point_triangle_candidates"]
        or scene_combined["static_triangle_point_triangle_candidates"]
        != scene_point_triangle["static_triangle_point_triangle_candidates"]
        or scene_combined["moving_triangle_point_triangle_candidates"]
        != scene_point_triangle["moving_triangle_point_triangle_candidates"]
    ):
        raise Plan083GpuCcdLineSearchPacketError(
            "combined scene runtime CCD row point-triangle counters do not "
            "match the point-triangle scene runtime row"
        )
    if (
        scene_combined["edge_edge_pairs"] != scene_edge_edge["pair_count"]
        or scene_combined["edge_edge_hits"] != scene_edge_edge["hit_count"]
        or scene_combined["runtime_edge_edge_candidates"]
        != scene_edge_edge["runtime_edge_edge_candidates"]
    ):
        raise Plan083GpuCcdLineSearchPacketError(
            "combined scene runtime CCD row edge-edge counters do not match "
            "the edge-edge scene runtime row"
        )
    for key in ("scene_bodies", "scene_nodes", "scene_triangles"):
        if (
            scene_combined[key] != scene_point_triangle[key]
            or scene_combined[key] != scene_edge_edge[key]
        ):
            raise Plan083GpuCcdLineSearchPacketError(
                "combined scene runtime CCD row scene counters do not match "
                "the per-family scene runtime rows"
            )

    point_triangle = primitive_families["point_triangle"]
    max_error = max(
        family["max_result_abs_error"] for family in primitive_families.values()
    )
    speedup = min(family["speedup"] for family in primitive_families.values())
    meets_speedup_gate = all(
        family["meets_speedup_gate"] for family in primitive_families.values()
    )
    analytic_reference_families = [
        family
        for family in primitive_families.values()
        if "analytic_reference_pairs" in family
    ]

    return {
        "plan083_gpu_ccd_line_search_packet": {
            "row_id": "ccd-line-search",
            "same_scene_cpu_gpu": True,
            "primitive_families": primitive_families,
            "pair_count": sum(
                family["pair_count"] for family in primitive_families.values()
            ),
            "segment_count": sum(
                family["segment_count"] for family in primitive_families.values()
            ),
            "hit_count": sum(
                family["hit_count"] for family in primitive_families.values()
            ),
            "analytic_reference_pair_count": sum(
                family["analytic_reference_pairs"]
                for family in analytic_reference_families
            ),
            "analytic_reference_hit_count": sum(
                family["analytic_reference_hits"]
                for family in analytic_reference_families
            ),
            "max_sampled_reference_overshoot": max(
                (
                    family["max_sampled_reference_overshoot"]
                    for family in analytic_reference_families
                ),
                default=0.0,
            ),
            "max_sampled_reference_conservative_gap": max(
                (
                    family["max_sampled_reference_conservative_gap"]
                    for family in analytic_reference_families
                ),
                default=0.0,
            ),
            "min_step_bound": min(
                family["min_step_bound"] for family in primitive_families.values()
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
            pair_count=args.pair_count,
            tolerance=args.tolerance,
            speedup_gate=args.speedup_gate,
        )
    except Plan083GpuCcdLineSearchPacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_gpu_ccd_line_search_packet"]
    print(
        "PLAN-083 GPU CCD line-search packet OK: "
        f"pairs={row['pair_count']} max_error={row['max_result_abs_error']:.3g} "
        f"speedup={row['speedup']:.3f}x "
        f"meets_gate={row['meets_speedup_gate']}"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
