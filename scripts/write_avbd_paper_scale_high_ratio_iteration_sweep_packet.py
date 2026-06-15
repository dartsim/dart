#!/usr/bin/env python3
"""Write a validated AVBD high-ratio iteration-sweep benchmark packet."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    make_resolved_solver_identity,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-scale-high-ratio-iteration-sweep-packet.json"
)
SCENE_ID = "avbd_paper_scale_high_ratio_chain"
BENCHMARK_NAME = "BM_AvbdPaperScaleHighRatioChainIterationSweep"
LINKS = 50
LIGHT_LINK_MASS = 1.0
TIP_LINK_MASS = 50000.0
MASS_RATIO = TIP_LINK_MASS / LIGHT_LINK_MASS
TIME_STEP = 0.005
REPLAY_STEPS = 32
REPLAY_SECONDS = 0.16
TOLERANCE = 1e-9
EXPECTED_MAX_ITERATIONS = (25, 50, 100, 200)
RESOLVED_SOLVER_IDENTITY = make_resolved_solver_identity(
    resolved_rigid_contact_family=None,
    rigid_point_joint_solver="avbd",
    avbd_rigid_contact_config_emplaced=False,
    recorded_from="paper-scale high-ratio iteration benchmark row family",
)
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_SVG_SIZE_RE = re.compile(
    r"<svg\b[^>]*\bwidth=\"([0-9.]+)\"[^>]*\bheight=\"([0-9.]+)\""
)


class AvbdPaperScaleHighRatioIterationSweepPacketError(RuntimeError):
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
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{path}: file not found"
        ) from exc
    except json.JSONDecodeError as exc:
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
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
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{plot_svg}: plot SVG not found"
        ) from exc
    if "<svg" not in text:
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{plot_svg}: plot artifact must be SVG"
        )
    match = _SVG_SIZE_RE.search(text)
    if match is None:
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
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
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{BENCHMARK_NAME}: missing {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{BENCHMARK_NAME}: non-finite {key}"
        )
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1e-15):
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _require_positive_counter(row: dict[str, Any], key: str) -> float:
    value = _finite_counter(row, key)
    if value <= 0.0:
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{BENCHMARK_NAME}: expected positive {key}, got {value:g}"
        )
    return value


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


def _validate_representative_row(row: dict[str, Any], max_iterations: int) -> None:
    _require_counter(row, "links", float(LINKS))
    _require_counter(row, "mass_ratio", MASS_RATIO)
    _require_counter(row, "max_iterations", float(max_iterations))
    _require_counter(row, "tolerance", TOLERANCE)
    _require_counter(row, "replay_seconds", REPLAY_SECONDS)
    _require_counter(row, "replay_steps", float(REPLAY_STEPS))
    _require_counter(row, "finite_replay", 1.0)
    _require_positive_counter(row, "max_abs_position")
    for key in ("real_time", "cpu_time"):
        _finite_counter(row, key)


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            "benchmark JSON missing benchmarks list"
        )

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _benchmark_family(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"benchmark JSON missing {BENCHMARK_NAME}"
        )

    packet_rows: list[dict[str, Any]] = []
    representative_by_budget: dict[int, list[dict[str, Any]]] = {
        budget: [] for budget in EXPECTED_MAX_ITERATIONS
    }
    for row in matching_rows:
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

        if not _is_representative(row):
            continue
        max_iterations = int(round(_finite_counter(row, "max_iterations")))
        if max_iterations not in representative_by_budget:
            raise AvbdPaperScaleHighRatioIterationSweepPacketError(
                f"{BENCHMARK_NAME}: unexpected max_iterations={max_iterations}"
            )
        _validate_representative_row(row, max_iterations)
        representative_by_budget[max_iterations].append(row)

    missing_budgets = [
        budget
        for budget in EXPECTED_MAX_ITERATIONS
        if not representative_by_budget[budget]
    ]
    if missing_budgets:
        missing = ", ".join(str(budget) for budget in missing_budgets)
        raise AvbdPaperScaleHighRatioIterationSweepPacketError(
            f"{BENCHMARK_NAME}: missing max_iterations budgets: {missing}"
        )

    plot_data = []
    for budget in EXPECTED_MAX_ITERATIONS:
        timing_row = _timing_row(representative_by_budget[budget])
        plot_data.append(
            {
                "max_iterations": budget,
                "cpu_time_per_step_ns": _finite_counter(timing_row, "cpu_time"),
                "real_time_per_step_ns": _finite_counter(timing_row, "real_time"),
                "max_abs_position_rad": _finite_counter(timing_row, "max_abs_position"),
                "finite_replay": _finite_counter(timing_row, "finite_replay"),
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
            "links": LINKS,
            "mass_ratio": MASS_RATIO,
            "time_step": TIME_STEP,
            "replay_steps": REPLAY_STEPS,
            "replay_seconds": REPLAY_SECONDS,
            "max_iterations": list(EXPECTED_MAX_ITERATIONS),
            "tolerance": TOLERANCE,
            "finite_replay": True,
        },
    }


def make_packet(benchmark_json: Path, plot_svg: Path | None = None) -> dict[str, Any]:
    packet = {
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "packet": "avbd_paper_scale_high_ratio_iteration_sweep",
        "scene": SCENE_ID,
        "target": {
            "paper_gap": "iteration-count sweep for the 50-link 50,000:1 chain",
            "scope": (
                "benchmark-only max-iteration sweep over the paper-scale "
                "50-link articulated chain"
            ),
            "complete_paper_reproduction": False,
        },
        "scene_invariants": {
            "links": LINKS,
            "light_link_mass": LIGHT_LINK_MASS,
            "tip_link_mass": TIP_LINK_MASS,
            "mass_ratio": MASS_RATIO,
            "time_step": TIME_STEP,
            "replay_steps": REPLAY_STEPS,
            "replay_seconds": REPLAY_SECONDS,
            "max_iterations": list(EXPECTED_MAX_ITERATIONS),
            "tolerance": TOLERANCE,
        },
        "benchmark": _validate_benchmark(benchmark_json),
        "reproduction": {
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdPaperScaleHighRatioChainIterationSweep "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            )
        },
    }
    if plot_svg is not None:
        packet["rendered_plot"] = _validate_plot(plot_svg)
    remaining_gates = [
        "same-hardware paper-number comparison for the 50-link 50,000:1 chain",
        "two-heavy-ball chain visual and invariant",
        "broad articulated hard-constraint stability coverage",
        "GPU AVBD row parity and same-hardware benchmark packets",
        "paper/site/video scene visual and performance packets",
    ]
    if plot_svg is None:
        remaining_gates.insert(
            0, "rendered convergence/stability plot for the iteration-count sweep"
        )
    packet["remaining_gates"] = remaining_gates
    return packet


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(args.benchmark_json, args.plot_svg)
    except AvbdPaperScaleHighRatioIterationSweepPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD paper-scale high-ratio iteration-sweep packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
