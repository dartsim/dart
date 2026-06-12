#!/usr/bin/env python3
"""Write a validated AVBD friction-coefficient sweep benchmark packet."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-friction-coefficient-sweep-packet.json"
)
SCENE_ID = "avbd_demo2d_dynamic_friction"
BENCHMARK_NAME = "BM_AvbdDemo2dFrictionCoefficientSweep"
EXPECTED_MAX_FRICTIONS = (0.0, 0.5, 1.0, 2.5, 5.0)
BOX_COUNT = 11
RIGID_BODIES = 12
COLLISION_SHAPES = 12
SOURCE_SCENE_INDEX = 2
TIME_STEP = 1.0 / 60.0
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_SVG_SIZE_RE = re.compile(
    r"<svg\b[^>]*\bwidth=\"([0-9.]+)\"[^>]*\bheight=\"([0-9.]+)\""
)


class AvbdFrictionCoefficientSweepPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--plot-svg", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{path}: file not found"
        ) from exc
    except json.JSONDecodeError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{path}: top-level JSON value must be an object"
        )
    return data


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_plot(plot_svg: Path) -> dict[str, Any]:
    try:
        text = plot_svg.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{plot_svg}: plot SVG not found"
        ) from exc
    if "<svg" not in text:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{plot_svg}: plot artifact must be SVG"
        )
    match = _SVG_SIZE_RE.search(text)
    if match is None:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{plot_svg}: plot SVG missing width/height"
        )
    return {
        "file": str(plot_svg),
        "sha256": _sha256(plot_svg),
        "width": float(match.group(1)),
        "height": float(match.group(2)),
    }


def _row_name(row: dict[str, Any]) -> str:
    name = row.get("run_name", row.get("name", ""))
    return name if isinstance(name, str) else ""


def _canonical_name(name: str) -> str:
    return _AGGREGATE_SUFFIX_RE.sub("", _REPEATS_SUFFIX_RE.sub("", name))


def _benchmark_family(name: str) -> str:
    return _canonical_name(name).split("/", 1)[0]


def _finite_counter(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: missing {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: non-finite {key}"
        )
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1e-15):
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
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


def _friction_key(value: float) -> int:
    return int(round(value * 10.0))


def _expected_friction_by_key() -> dict[int, float]:
    return {_friction_key(value): value for value in EXPECTED_MAX_FRICTIONS}


def _validate_representative_row(row: dict[str, Any], max_friction: float) -> None:
    _require_counter(row, "rigid_bodies", float(RIGID_BODIES))
    _require_counter(row, "rigid_body_joints", 0.0)
    _require_counter(row, "collision_shapes", float(COLLISION_SHAPES))
    _require_counter(row, "source_scene_index", float(SOURCE_SCENE_INDEX))
    _require_counter(row, "max_friction", max_friction)
    _require_counter(row, "min_friction", 0.0)
    _require_counter(row, "friction_samples", float(BOX_COUNT))
    for key in ("real_time", "cpu_time"):
        _finite_counter(row, key)


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdFrictionCoefficientSweepPacketError(
            "benchmark JSON missing benchmarks list"
        )

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _benchmark_family(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdFrictionCoefficientSweepPacketError(
            f"benchmark JSON missing {BENCHMARK_NAME}"
        )

    expected_by_key = _expected_friction_by_key()
    representative_by_key: dict[int, list[dict[str, Any]]] = {
        key: [] for key in expected_by_key
    }
    packet_rows: list[dict[str, Any]] = []
    for row in matching_rows:
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

        if not _is_representative(row):
            continue
        max_friction = _finite_counter(row, "max_friction")
        friction_key = _friction_key(max_friction)
        if friction_key not in representative_by_key:
            raise AvbdFrictionCoefficientSweepPacketError(
                f"{BENCHMARK_NAME}: unexpected max_friction={max_friction:g}"
            )
        expected = expected_by_key[friction_key]
        _validate_representative_row(row, expected)
        representative_by_key[friction_key].append(row)

    missing = [
        expected_by_key[key]
        for key, rows_for_key in representative_by_key.items()
        if not rows_for_key
    ]
    if missing:
        missing_text = ", ".join(f"{value:g}" for value in missing)
        raise AvbdFrictionCoefficientSweepPacketError(
            f"{BENCHMARK_NAME}: missing max_friction values: {missing_text}"
        )

    plot_data = []
    for friction_key in sorted(representative_by_key):
        timing_row = _timing_row(representative_by_key[friction_key])
        max_friction = expected_by_key[friction_key]
        plot_data.append(
            {
                "max_friction": max_friction,
                "cpu_time_per_step_ns": _finite_counter(timing_row, "cpu_time"),
                "real_time_per_step_ns": _finite_counter(timing_row, "real_time"),
                "friction_samples": _finite_counter(timing_row, "friction_samples"),
                "time_unit": timing_row.get("time_unit", "ns"),
            }
        )

    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": BENCHMARK_NAME,
        "context": {
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
        },
        "rows": packet_rows,
        "plot_data": plot_data,
        "invariants": {
            "source_demo": "avbd-demo2d",
            "source_scene": "Dynamic Friction",
            "source_scene_index": SOURCE_SCENE_INDEX,
            "time_step": TIME_STEP,
            "rigid_bodies": RIGID_BODIES,
            "collision_shapes": COLLISION_SHAPES,
            "friction_samples": BOX_COUNT,
            "max_friction": list(EXPECTED_MAX_FRICTIONS),
            "min_friction": 0.0,
        },
    }


def make_packet(benchmark_json: Path, plot_svg: Path | None = None) -> dict[str, Any]:
    packet = {
        "schema_version": 1,
        "packet": "avbd_friction_coefficient_sweep",
        "scene": SCENE_ID,
        "target": {
            "paper_gap": "friction coefficient comparison",
            "scope": (
                "benchmark-selected sweep over the source-shaped "
                "avbd-demo2d Dynamic Friction scene"
            ),
            "complete_paper_reproduction": False,
        },
        "scene_invariants": {
            "source_demo": "avbd-demo2d",
            "scene_name": "Dynamic Friction",
            "scene_index": SOURCE_SCENE_INDEX,
            "box_count": BOX_COUNT,
            "time_step": TIME_STEP,
            "initial_velocity": [10.0, 0.0, 0.0],
            "max_friction": list(EXPECTED_MAX_FRICTIONS),
            "min_friction": 0.0,
        },
        "visual_anchor": {
            "scene": SCENE_ID,
            "existing_packet": "avbd-demo2d-dynamic-friction-packet.json",
            "note": (
                "This sweep reuses the source-shaped Dynamic Friction visual "
                "scene; the new artifact is benchmark/plot evidence only."
            ),
        },
        "benchmark": _validate_benchmark(benchmark_json),
        "reproduction": {
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdDemo2dFrictionCoefficientSweep "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            )
        },
    }
    if plot_svg is not None:
        packet["rendered_plot"] = _validate_plot(plot_svg)
    remaining_gates = [
        "source/reference friction-sweep timing comparison",
        "per-coefficient visual capture or video evidence",
        "same-hardware paper/site friction comparison",
        "GPU AVBD row parity and benchmark packets",
    ]
    if plot_svg is None:
        remaining_gates.insert(0, "rendered friction-sweep plot")
    packet["remaining_gates"] = remaining_gates
    return packet


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(args.benchmark_json, args.plot_svg)
    except AvbdFrictionCoefficientSweepPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD friction-coefficient sweep packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
