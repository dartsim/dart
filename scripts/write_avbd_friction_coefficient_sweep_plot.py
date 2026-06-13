#!/usr/bin/env python3
"""Render the AVBD friction-coefficient sweep packet as an SVG plot."""

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
    "avbd-friction-coefficient-sweep-packet.json"
)
DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-friction-coefficient-sweep-plot.svg"
)
WIDTH = 720
HEIGHT = 360
PLOT_LEFT = 76
PLOT_RIGHT = 682
PLOT_TOP = 84
PLOT_BOTTOM = 292
SUPPORTED_SCHEMA_VERSIONS = (1, 2)


class AvbdFrictionCoefficientSweepPlotError(RuntimeError):
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
        raise AvbdFrictionCoefficientSweepPlotError(f"{path}: file not found") from exc
    except json.JSONDecodeError as exc:
        raise AvbdFrictionCoefficientSweepPlotError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise AvbdFrictionCoefficientSweepPlotError(
            f"{path}: top-level JSON value must be an object"
        )
    return data


def _finite(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdFrictionCoefficientSweepPlotError(f"missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdFrictionCoefficientSweepPlotError(f"non-finite {key}")
    return value


def _validate_packet(packet_path: Path) -> list[dict[str, float]]:
    packet = _load_json(packet_path)
    if packet.get("schema_version") not in SUPPORTED_SCHEMA_VERSIONS:
        raise AvbdFrictionCoefficientSweepPlotError(
            "packet schema_version must be 1 or 2"
        )
    if packet.get("packet") != "avbd_friction_coefficient_sweep":
        raise AvbdFrictionCoefficientSweepPlotError(
            "packet must be avbd_friction_coefficient_sweep"
        )
    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, dict):
        raise AvbdFrictionCoefficientSweepPlotError("packet missing benchmark")
    raw_plot_data = benchmark.get("plot_data")
    if not isinstance(raw_plot_data, list):
        raise AvbdFrictionCoefficientSweepPlotError(
            "packet benchmark missing plot_data"
        )

    reference_by_friction: dict[float, float] = {}
    reference_sweep = packet.get("reference_sweep")
    if reference_sweep is not None:
        if not isinstance(reference_sweep, dict):
            raise AvbdFrictionCoefficientSweepPlotError(
                "packet reference_sweep must be an object"
            )
        raw_reference_rows = reference_sweep.get("plot_data")
        if not isinstance(raw_reference_rows, list):
            raise AvbdFrictionCoefficientSweepPlotError(
                "packet reference_sweep missing plot_data"
            )
        for raw_row in raw_reference_rows:
            if not isinstance(raw_row, dict):
                raise AvbdFrictionCoefficientSweepPlotError(
                    "reference plot_data rows must be objects"
                )
            reference_by_friction[_finite(raw_row, "max_friction")] = (
                _finite(raw_row, "cpu_time_per_step_ns") / 1000.0
            )

    rows: list[dict[str, float]] = []
    for raw_row in raw_plot_data:
        if not isinstance(raw_row, dict):
            raise AvbdFrictionCoefficientSweepPlotError(
                "plot_data rows must be objects"
            )
        max_friction = _finite(raw_row, "max_friction")
        row = {
            "max_friction": max_friction,
            "dart_cpu_time_us": _finite(raw_row, "cpu_time_per_step_ns") / 1000.0,
        }
        if max_friction in reference_by_friction:
            row["reference_cpu_time_us"] = reference_by_friction[max_friction]
        rows.append(row)
    if len(rows) < 2:
        raise AvbdFrictionCoefficientSweepPlotError(
            "plot_data must contain at least two rows"
        )
    rows.sort(key=lambda row: row["max_friction"])
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


def render_svg(rows: list[dict[str, float]]) -> str:
    x_values = [row["max_friction"] for row in rows]
    y_values = [row["dart_cpu_time_us"] for row in rows]
    y_values.extend(
        row["reference_cpu_time_us"] for row in rows if "reference_cpu_time_us" in row
    )
    x_min = min(x_values)
    x_max = max(x_values)
    y_min = min(0.0, min(y_values))
    y_max = max(y_values)
    y_padding = (y_max - y_min) * 0.08 if y_max > y_min else max(y_max, 1.0) * 0.08
    y_max += y_padding

    points = [
        (
            _scale(row["max_friction"], x_min, x_max, PLOT_LEFT, PLOT_RIGHT),
            _scale(row["dart_cpu_time_us"], y_min, y_max, PLOT_BOTTOM, PLOT_TOP),
        )
        for row in rows
    ]
    reference_points = [
        (
            _scale(row["max_friction"], x_min, x_max, PLOT_LEFT, PLOT_RIGHT),
            _scale(
                row["reference_cpu_time_us"],
                y_min,
                y_max,
                PLOT_BOTTOM,
                PLOT_TOP,
            ),
        )
        for row in rows
        if "reference_cpu_time_us" in row
    ]
    has_reference = len(reference_points) == len(rows)
    svg_lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
        "<style>",
        "text{font-family:Arial,Helvetica,sans-serif;fill:#111827}",
        ".title{font-size:22px;font-weight:700}",
        ".subtitle{font-size:13px;fill:#4b5563}",
        ".axis{stroke:#6b7280;stroke-width:1.4}",
        ".tick{font-size:11px;fill:#4b5563}",
        "</style>",
        f'<rect width="{WIDTH}" height="{HEIGHT}" fill="#ffffff"/>',
        '<text x="32" y="36" class="title">AVBD Friction Coefficient Sweep</text>',
        (
            '<text x="32" y="58" class="subtitle">'
            "avbd-demo2d Dynamic Friction; 11 boxes, lower CPU time is better."
            "</text>"
        ),
        f'<line x1="{PLOT_LEFT}" y1="{PLOT_BOTTOM}" x2="{PLOT_RIGHT}" y2="{PLOT_BOTTOM}" class="axis"/>',
        f'<line x1="{PLOT_LEFT}" y1="{PLOT_TOP}" x2="{PLOT_LEFT}" y2="{PLOT_BOTTOM}" class="axis"/>',
        f'<text x="18" y="{PLOT_TOP + 7}" class="tick">{_format_tick(y_max)} us</text>',
        f'<text x="18" y="{PLOT_BOTTOM + 4}" class="tick">{_format_tick(y_min)} us</text>',
        (
            f'<polyline points="{_polyline(points)}" fill="none" '
            'stroke="#2563eb" stroke-width="3"/>'
        ),
    ]
    if has_reference:
        svg_lines.extend(
            [
                (
                    f'<polyline points="{_polyline(reference_points)}" fill="none" '
                    'stroke="#dc2626" stroke-width="3" stroke-dasharray="8 5"/>'
                ),
                '<line x1="514" y1="34" x2="548" y2="34" stroke="#2563eb" stroke-width="3"/>',
                '<text x="556" y="38" class="tick">DART</text>',
                (
                    '<line x1="514" y1="54" x2="548" y2="54" '
                    'stroke="#dc2626" stroke-width="3" stroke-dasharray="8 5"/>'
                ),
                '<text x="556" y="58" class="tick">Native source</text>',
            ]
        )
    for row, (x, y) in zip(rows, points):
        label = html.escape(_format_tick(row["max_friction"]))
        svg_lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="#2563eb"/>')
        if has_reference:
            reference_y = _scale(
                row["reference_cpu_time_us"],
                y_min,
                y_max,
                PLOT_BOTTOM,
                PLOT_TOP,
            )
            svg_lines.append(
                f'<circle cx="{x:.2f}" cy="{reference_y:.2f}" r="5" fill="#dc2626"/>'
            )
        svg_lines.append(
            f'<text x="{x:.2f}" y="{PLOT_BOTTOM + 24}" class="tick" text-anchor="middle">{label}</text>'
        )
    svg_lines.append(
        '<text x="379" y="344" class="subtitle" text-anchor="middle">Maximum Coulomb friction coefficient</text>'
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
    except AvbdFrictionCoefficientSweepPlotError as exc:
        raise SystemExit(str(exc)) from exc
    write_plot(args.output, render_svg(rows))
    print(f"Wrote AVBD friction-coefficient sweep plot: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
