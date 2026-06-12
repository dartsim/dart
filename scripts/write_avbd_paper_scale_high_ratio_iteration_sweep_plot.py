#!/usr/bin/env python3
"""Render the AVBD high-ratio iteration-sweep packet as an SVG plot."""

from __future__ import annotations

import argparse
import html
import json
import math
import sys
from pathlib import Path
from typing import Any

DEFAULT_PACKET = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-scale-high-ratio-iteration-sweep-packet.json"
)
DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-scale-high-ratio-iteration-sweep-plot.svg"
)
WIDTH = 720
HEIGHT = 420
PLOT_LEFT = 76
PLOT_RIGHT = 682
TOP_PANEL_TOP = 76
TOP_PANEL_BOTTOM = 198
BOTTOM_PANEL_TOP = 256
BOTTOM_PANEL_BOTTOM = 378


class AvbdPaperScaleHighRatioIterationSweepPlotError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet", type=Path, default=DEFAULT_PACKET)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            f"{path}: file not found"
        ) from exc
    except json.JSONDecodeError as exc:
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            f"{path}: top-level JSON value must be an object"
        )
    return data


def _finite(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(f"missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(f"non-finite {key}")
    return value


def _validate_packet(packet_path: Path) -> list[dict[str, float]]:
    packet = _load_json(packet_path)
    if packet.get("schema_version") != 1:
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            "packet schema_version must be 1"
        )
    if packet.get("packet") != "avbd_paper_scale_high_ratio_iteration_sweep":
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            "packet must be avbd_paper_scale_high_ratio_iteration_sweep"
        )
    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, dict):
        raise AvbdPaperScaleHighRatioIterationSweepPlotError("packet missing benchmark")
    raw_plot_data = benchmark.get("plot_data")
    if not isinstance(raw_plot_data, list):
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            "packet benchmark missing plot_data"
        )

    rows: list[dict[str, float]] = []
    for raw_row in raw_plot_data:
        if not isinstance(raw_row, dict):
            raise AvbdPaperScaleHighRatioIterationSweepPlotError(
                "plot_data rows must be objects"
            )
        finite_replay = _finite(raw_row, "finite_replay")
        if finite_replay != 1.0:
            raise AvbdPaperScaleHighRatioIterationSweepPlotError(
                "plot_data row is not finite"
            )
        rows.append(
            {
                "max_iterations": _finite(raw_row, "max_iterations"),
                "cpu_time_ms": _finite(raw_row, "cpu_time_per_step_ns") / 1e6,
                "max_abs_position_rad": _finite(raw_row, "max_abs_position_rad"),
            }
        )
    if len(rows) < 2:
        raise AvbdPaperScaleHighRatioIterationSweepPlotError(
            "plot_data must contain at least two rows"
        )
    rows.sort(key=lambda row: row["max_iterations"])
    return rows


def _scale(
    value: float,
    domain_min: float,
    domain_max: float,
    range_min: float,
    range_max: float,
) -> float:
    if domain_max == domain_min:
        return (range_min + range_max) * 0.5
    ratio = (value - domain_min) / (domain_max - domain_min)
    return range_min + ratio * (range_max - range_min)


def _polyline(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def _format_tick(value: float) -> str:
    if abs(value) >= 10:
        return f"{value:.0f}"
    return f"{value:.3g}"


def _panel_svg(
    *,
    rows: list[dict[str, float]],
    metric: str,
    label: str,
    color: str,
    top: int,
    bottom: int,
) -> list[str]:
    x_values = [row["max_iterations"] for row in rows]
    y_values = [row[metric] for row in rows]
    x_min = min(x_values)
    x_max = max(x_values)
    y_min = min(0.0, min(y_values))
    y_max = max(y_values)
    y_padding = (y_max - y_min) * 0.08 if y_max > y_min else max(y_max, 1.0) * 0.08
    y_max += y_padding

    def point(row: dict[str, float]) -> tuple[float, float]:
        x = _scale(row["max_iterations"], x_min, x_max, PLOT_LEFT, PLOT_RIGHT)
        y = _scale(row[metric], y_min, y_max, bottom, top)
        return x, y

    points = [point(row) for row in rows]
    lines = [
        f'<text x="76" y="{top - 20}" class="panel-title">{html.escape(label)}</text>',
        f'<line x1="{PLOT_LEFT}" y1="{bottom}" x2="{PLOT_RIGHT}" y2="{bottom}" class="axis"/>',
        f'<line x1="{PLOT_LEFT}" y1="{top}" x2="{PLOT_LEFT}" y2="{bottom}" class="axis"/>',
        f'<text x="18" y="{top + 7}" class="tick">{_format_tick(y_max)}</text>',
        f'<text x="18" y="{bottom + 4}" class="tick">{_format_tick(y_min)}</text>',
        f'<polyline points="{_polyline(points)}" fill="none" stroke="{color}" stroke-width="3"/>',
    ]
    for row, (x, y) in zip(rows, points):
        budget = int(row["max_iterations"])
        lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="{color}"/>')
        lines.append(
            f'<text x="{x:.2f}" y="{bottom + 22}" class="tick" text-anchor="middle">{budget}</text>'
        )
    return lines


def render_svg(rows: list[dict[str, float]]) -> str:
    svg_lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
        "<style>",
        "text{font-family:Arial,Helvetica,sans-serif;fill:#111827}",
        ".title{font-size:22px;font-weight:700}",
        ".subtitle{font-size:13px;fill:#4b5563}",
        ".panel-title{font-size:14px;font-weight:700}",
        ".axis{stroke:#6b7280;stroke-width:1.4}",
        ".tick{font-size:11px;fill:#4b5563}",
        "</style>",
        '<rect width="720" height="420" fill="#ffffff"/>',
        '<text x="32" y="34" class="title">AVBD Paper-Scale High-Ratio Iteration Sweep</text>',
        '<text x="32" y="54" class="subtitle">50 links, 50,000:1 tip mass, 32-step replay envelope; lower CPU time is better, finite replay required.</text>',
    ]
    svg_lines.extend(
        _panel_svg(
            rows=rows,
            metric="cpu_time_ms",
            label="CPU time per step (ms)",
            color="#2563eb",
            top=TOP_PANEL_TOP,
            bottom=TOP_PANEL_BOTTOM,
        )
    )
    svg_lines.extend(
        _panel_svg(
            rows=rows,
            metric="max_abs_position_rad",
            label="Replay stability: max absolute joint position (rad)",
            color="#16a34a",
            top=BOTTOM_PANEL_TOP,
            bottom=BOTTOM_PANEL_BOTTOM,
        )
    )
    svg_lines.append(
        '<text x="379" y="412" class="subtitle" text-anchor="middle">Max solver iterations</text>'
    )
    svg_lines.append("</svg>")
    return "\n".join(svg_lines) + "\n"


def write_plot(path: Path, svg: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg, encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        rows = _validate_packet(args.packet)
    except AvbdPaperScaleHighRatioIterationSweepPlotError as exc:
        raise SystemExit(str(exc)) from exc
    write_plot(args.output, render_svg(rows))
    print(f"Wrote AVBD paper-scale high-ratio iteration-sweep plot: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
