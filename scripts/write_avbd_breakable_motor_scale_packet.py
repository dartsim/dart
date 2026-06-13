#!/usr/bin/env python3
"""Write a validated AVBD breakable-motor scale benchmark packet."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import AVBD_PACKET_SCHEMA_VERSION  # noqa: E402
from write_avbd_demo3d_static_friction_packet import (  # noqa: E402
    _canonical_name,
    _load_json,
    _row_name,
    _sha256,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-breakable-motor-scale-packet.json"
)
BENCHMARK_ARGS = (1, 8, 32)
TIME_STEP = 0.005
BENCHMARK_BREAK_FORCE_N = 1.0e12
REVOLUTE_TARGET_SPEED_RAD_PER_S = 0.5
REVOLUTE_MAX_TORQUE_NM = 800.0
PRISMATIC_TARGET_SPEED_M_PER_S = 0.35
PRISMATIC_MAX_FORCE_N = 800.0
RESOLVED_SOLVER_IDENTITY = {
    "avbd_rigid_contact_config_emplaced": False,
    "recorded_from": "breakable motor scale benchmark row family",
    "rigid_contact_solver": "none",
    "rigid_point_joint_solver": "avbd",
}


@dataclass(frozen=True)
class Variant:
    key: str
    benchmark: str
    motor_kind: str
    joint_anchor_scope: str
    row_family: str


VARIANTS = (
    Variant(
        key="same_multibody_revolute",
        benchmark="BM_AvbdArticulatedBreakableMotorStep",
        motor_kind="revolute",
        joint_anchor_scope="same_multibody_pair",
        row_family="public same-multibody articulated revolute motor break/reset",
    ),
    Variant(
        key="same_multibody_prismatic",
        benchmark="BM_AvbdArticulatedPrismaticBreakableMotorStep",
        motor_kind="prismatic",
        joint_anchor_scope="same_multibody_pair",
        row_family="public same-multibody articulated prismatic motor break/reset",
    ),
    Variant(
        key="world_prismatic",
        benchmark="BM_AvbdArticulatedWorldPrismaticBreakableMotorStep",
        motor_kind="prismatic",
        joint_anchor_scope="world_link",
        row_family="public world-link articulated prismatic motor break/reset",
    ),
    Variant(
        key="world_revolute",
        benchmark="BM_AvbdArticulatedWorldRevoluteBreakableMotorStep",
        motor_kind="revolute",
        joint_anchor_scope="world_link",
        row_family="public world-link articulated revolute motor break/reset",
    ),
)
_VARIANT_BY_BENCHMARK = {variant.benchmark: variant for variant in VARIANTS}


class AvbdBreakableMotorScalePacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _benchmark_identity(row: dict[str, Any]) -> str:
    return _canonical_name(_row_name(row))


def _match_variant_arg(row: dict[str, Any]) -> tuple[Variant, int] | None:
    identity = _benchmark_identity(row)
    if "/" not in identity:
        return None
    benchmark, arg_text = identity.rsplit("/", 1)
    variant = _VARIANT_BY_BENCHMARK.get(benchmark)
    if variant is None:
        return None
    try:
        arg = int(arg_text)
    except ValueError as exc:
        raise AvbdBreakableMotorScalePacketError(
            f"{benchmark}: non-integer benchmark argument {arg_text!r}"
        ) from exc
    if arg not in BENCHMARK_ARGS:
        raise AvbdBreakableMotorScalePacketError(
            f"{benchmark}: unexpected breakable_motors argument {arg}"
        )
    return variant, arg


def _finite_counter(row: dict[str, Any], key: str, benchmark: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdBreakableMotorScalePacketError(f"{benchmark}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdBreakableMotorScalePacketError(f"{benchmark}: non-finite {key}")
    return value


def _require_counter(
    row: dict[str, Any], key: str, expected: float, benchmark: str
) -> None:
    value = _finite_counter(row, key, benchmark)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1e-15):
        raise AvbdBreakableMotorScalePacketError(
            f"{benchmark}: expected {key}={expected:g}, got {value:g}"
        )


def _is_representative(row: dict[str, Any]) -> bool:
    run_type = row.get("run_type", "iteration")
    aggregate_name = row.get("aggregate_name")
    return run_type == "iteration" or aggregate_name in ("mean", "median")


def _timing_row(rows: list[dict[str, Any]]) -> dict[str, Any]:
    median_rows = [row for row in rows if row.get("aggregate_name") == "median"]
    if median_rows:
        return median_rows[0]
    mean_rows = [row for row in rows if row.get("aggregate_name") == "mean"]
    if mean_rows:
        return mean_rows[0]
    return rows[0]


def _validate_representative_row(
    row: dict[str, Any], variant: Variant, arg: int
) -> None:
    _require_counter(row, "motors", float(arg), variant.benchmark)
    _require_counter(row, "breakable_motors", float(arg), variant.benchmark)
    for key in ("real_time", "cpu_time"):
        _finite_counter(row, key, variant.benchmark)


def _context(data: dict[str, Any]) -> dict[str, Any]:
    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    return {
        key: context[key]
        for key in (
            "executable",
            "num_cpus",
            "mhz_per_cpu",
            "library_version",
            "library_build_type",
            "json_schema_version",
        )
        if key in context
    }


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdBreakableMotorScalePacketError(
            "benchmark JSON missing benchmarks list"
        )

    representative_by_key: dict[tuple[str, int], list[dict[str, Any]]] = {
        (variant.key, arg): [] for variant in VARIANTS for arg in BENCHMARK_ARGS
    }
    packet_rows: list[dict[str, Any]] = []

    for row in rows:
        if not isinstance(row, dict):
            continue
        match = _match_variant_arg(row)
        if match is None:
            continue
        variant, arg = match
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key, variant.benchmark)
        packet_rows.append(row)
        if _is_representative(row):
            _validate_representative_row(row, variant, arg)
            representative_by_key[(variant.key, arg)].append(row)

    if not packet_rows:
        raise AvbdBreakableMotorScalePacketError(
            "benchmark JSON missing AVBD breakable-motor scale rows"
        )

    missing = [
        f"{variant.benchmark}/{arg}"
        for variant in VARIANTS
        for arg in BENCHMARK_ARGS
        if not representative_by_key[(variant.key, arg)]
    ]
    if missing:
        raise AvbdBreakableMotorScalePacketError(
            "benchmark JSON missing breakable-motor scale rows: " + ", ".join(missing)
        )

    scale_data = []
    for variant in VARIANTS:
        for arg in BENCHMARK_ARGS:
            timing_row = _timing_row(representative_by_key[(variant.key, arg)])
            scale_data.append(
                {
                    "variant": variant.key,
                    "benchmark": f"{variant.benchmark}/{arg}",
                    "motor_kind": variant.motor_kind,
                    "joint_anchor_scope": variant.joint_anchor_scope,
                    "motors": arg,
                    "breakable_motors": arg,
                    "cpu_time_per_step_ns": _finite_counter(
                        timing_row, "cpu_time", variant.benchmark
                    ),
                    "real_time_per_step_ns": _finite_counter(
                        timing_row, "real_time", variant.benchmark
                    ),
                    "time_unit": timing_row.get("time_unit", "ns"),
                }
            )

    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": "avbd_breakable_motor_scale",
        "benchmarks": [variant.benchmark for variant in VARIANTS],
        "context": _context(data),
        "rows": packet_rows,
        "scale_data": scale_data,
        "invariants": {
            "motors": list(BENCHMARK_ARGS),
            "breakable_motors": list(BENCHMARK_ARGS),
            "motor_kinds": sorted({variant.motor_kind for variant in VARIANTS}),
            "joint_anchor_scopes": sorted(
                {variant.joint_anchor_scope for variant in VARIANTS}
            ),
            "time_step": TIME_STEP,
            "revolute_target_speed_rad_per_s": REVOLUTE_TARGET_SPEED_RAD_PER_S,
            "revolute_max_torque_nm": REVOLUTE_MAX_TORQUE_NM,
            "prismatic_target_speed_m_per_s": PRISMATIC_TARGET_SPEED_M_PER_S,
            "prismatic_max_force_n": PRISMATIC_MAX_FORCE_N,
            "break_force_n": BENCHMARK_BREAK_FORCE_N,
        },
    }


def make_packet(benchmark_json: Path) -> dict[str, Any]:
    return {
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "packet": "avbd_breakable_motor_scale",
        "scene": "avbd_breakable_motor_scale",
        "target": {
            "broad_motor_lifecycle_complete": False,
            "scope": (
                "benchmark-only scale evidence for public articulated "
                "breakable motor rows over 1, 8, and 32 motors"
            ),
            "row_families": [variant.row_family for variant in VARIANTS],
        },
        "benchmark": _validate_benchmark(benchmark_json),
        "reproduction": {
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter='BM_AvbdArticulated(BreakableMotor|"
                "PrismaticBreakableMotor|WorldPrismaticBreakableMotor|"
                "WorldRevoluteBreakableMotor)Step/(1|8|32)$' "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            )
        },
        "remaining_gates": [
            "visual motor command-update and weak re-arm corpus beyond the narrow demos",
            "source-demo and paper/site/video motor scene comparisons",
            "visual breakable-wall or fracture corpus scene",
            "GPU AVBD row parity and same-hardware benchmark packets",
            "paper/site/video scene visual and performance packets",
        ],
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(args.benchmark_json)
    except AvbdBreakableMotorScalePacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD breakable-motor scale packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
