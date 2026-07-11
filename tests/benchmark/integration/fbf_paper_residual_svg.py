#!/usr/bin/env python3
"""Render fbf_paper_trace residual_history CSV files as simple SVG plots."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

COLORS = [
    "#005f73",
    "#9b2226",
    "#0a9396",
    "#ca6702",
    "#3a0ca3",
    "#2d6a4f",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a residual_history CSV as an SVG convergence plot."
    )
    parser.add_argument("input_csv", nargs="?", type=Path)
    parser.add_argument("output_svg", nargs="?", type=Path)
    parser.add_argument("--title", default="Exact-FBF residual history")
    parser.add_argument(
        "--output",
        type=Path,
        help="Output SVG for multi-panel mode.",
    )
    parser.add_argument(
        "--panel",
        action="append",
        nargs=2,
        metavar=("INPUT_CSV", "PANEL_TITLE"),
        help="Add one residual-history CSV to a multi-panel SVG.",
    )
    return parser.parse_args()


def load_series(path: Path) -> dict[int, list[tuple[float, float]]]:
    series: dict[int, list[tuple[float, float]]] = {}
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            residual = float(row["residual"])
            if not math.isfinite(residual) or residual <= 0.0:
                continue
            solve_index = int(row["solve_index"])
            outer_iteration = float(row["outer_iteration"])
            series.setdefault(solve_index, []).append(
                (outer_iteration, math.log10(residual))
            )
    if not series:
        raise SystemExit(f"{path} did not contain positive finite residuals")
    return series


def escape_xml(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def render_svg(series: dict[int, list[tuple[float, float]]], title: str) -> str:
    width = 920
    height = 520
    margin_left = 78
    margin_right = 26
    margin_top = 54
    margin_bottom = 72
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    xs = [point[0] for points in series.values() for point in points]
    ys = [point[1] for points in series.values() for point in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    if min_x == max_x:
        max_x = min_x + 1.0
    if min_y == max_y:
        max_y = min_y + 1.0

    def map_x(value: float) -> float:
        return margin_left + (value - min_x) / (max_x - min_x) * plot_width

    def map_y(value: float) -> float:
        return margin_top + (max_y - value) / (max_y - min_y) * plot_height

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        (
            f'<text x="{width / 2:.1f}" y="28" text-anchor="middle" '
            'font-family="sans-serif" font-size="18" fill="#111111">'
            f"{escape_xml(title)}</text>"
        ),
        (
            f'<line x1="{margin_left}" y1="{margin_top + plot_height}" '
            f'x2="{margin_left + plot_width}" y2="{margin_top + plot_height}" '
            'stroke="#333333" stroke-width="1"/>'
        ),
        (
            f'<line x1="{margin_left}" y1="{margin_top}" '
            f'x2="{margin_left}" y2="{margin_top + plot_height}" '
            'stroke="#333333" stroke-width="1"/>'
        ),
    ]

    for tick in range(6):
        fraction = tick / 5.0
        x = margin_left + fraction * plot_width
        value = min_x + fraction * (max_x - min_x)
        lines.extend(
            [
                (
                    f'<line x1="{x:.1f}" y1="{margin_top}" '
                    f'x2="{x:.1f}" y2="{margin_top + plot_height}" '
                    'stroke="#dddddd" stroke-width="1"/>'
                ),
                (
                    f'<text x="{x:.1f}" y="{margin_top + plot_height + 24}" '
                    'text-anchor="middle" font-family="sans-serif" '
                    f'font-size="12" fill="#333333">{value:.0f}</text>'
                ),
            ]
        )

    for tick in range(6):
        fraction = tick / 5.0
        y = margin_top + fraction * plot_height
        value = max_y - fraction * (max_y - min_y)
        lines.extend(
            [
                (
                    f'<line x1="{margin_left}" y1="{y:.1f}" '
                    f'x2="{margin_left + plot_width}" y2="{y:.1f}" '
                    'stroke="#eeeeee" stroke-width="1"/>'
                ),
                (
                    f'<text x="{margin_left - 10}" y="{y + 4:.1f}" '
                    'text-anchor="end" font-family="sans-serif" '
                    f'font-size="12" fill="#333333">{value:.1f}</text>'
                ),
            ]
        )

    legend_y = margin_top + 18
    for index, (solve_index, points) in enumerate(sorted(series.items())):
        color = COLORS[index % len(COLORS)]
        coordinates = " ".join(
            f"{map_x(x):.1f},{map_y(y):.1f}" for x, y in points
        )
        lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2" '
            f'points="{coordinates}"/>'
        )
        legend_x = margin_left + plot_width - 170
        y = legend_y + 20 * index
        lines.extend(
            [
                (
                    f'<line x1="{legend_x}" y1="{y - 4}" '
                    f'x2="{legend_x + 28}" y2="{y - 4}" '
                    f'stroke="{color}" stroke-width="3"/>'
                ),
                (
                    f'<text x="{legend_x + 36}" y="{y}" '
                    'font-family="sans-serif" font-size="12" fill="#111111">'
                    f"solve {solve_index}</text>"
                ),
            ]
        )

    lines.extend(
        [
            (
                f'<text x="{margin_left + plot_width / 2:.1f}" '
                f'y="{height - 24}" text-anchor="middle" '
                'font-family="sans-serif" font-size="13" fill="#111111">'
                "Outer FBF iteration</text>"
            ),
            (
                '<text transform="translate(22 260) rotate(-90)" '
                'text-anchor="middle" font-family="sans-serif" '
                'font-size="13" fill="#111111">log10 residual</text>'
            ),
            "</svg>",
        ]
    )
    return "\n".join(lines) + "\n"


def render_panel_svg(
    panels: list[tuple[str, dict[int, list[tuple[float, float]]]]], title: str
) -> str:
    width = 1120
    columns = 2
    panel_width = 520
    panel_height = 320
    panel_gap_x = 40
    panel_gap_y = 50
    margin_left = 78
    margin_top = 74
    plot_left_pad = 58
    plot_right_pad = 18
    plot_top_pad = 38
    plot_bottom_pad = 54
    rows = (len(panels) + columns - 1) // columns
    height = margin_top + rows * panel_height + (rows - 1) * panel_gap_y + 36

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        (
            f'<text x="{width / 2:.1f}" y="30" text-anchor="middle" '
            'font-family="sans-serif" font-size="20" fill="#111111">'
            f"{escape_xml(title)}</text>"
        ),
    ]

    for panel_index, (panel_title, series) in enumerate(panels):
        row = panel_index // columns
        column = panel_index % columns
        origin_x = margin_left + column * (panel_width + panel_gap_x)
        origin_y = margin_top + row * (panel_height + panel_gap_y)
        plot_x = origin_x + plot_left_pad
        plot_y = origin_y + plot_top_pad
        plot_width = panel_width - plot_left_pad - plot_right_pad
        plot_height = panel_height - plot_top_pad - plot_bottom_pad

        xs = [point[0] for points in series.values() for point in points]
        ys = [point[1] for points in series.values() for point in points]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        if min_x == max_x:
            max_x = min_x + 1.0
        if min_y == max_y:
            max_y = min_y + 1.0

        def map_x(value: float) -> float:
            return plot_x + (value - min_x) / (max_x - min_x) * plot_width

        def map_y(value: float) -> float:
            return plot_y + (max_y - value) / (max_y - min_y) * plot_height

        lines.extend(
            [
                (
                    f'<text x="{origin_x + panel_width / 2:.1f}" '
                    f'y="{origin_y + 18}" text-anchor="middle" '
                    'font-family="sans-serif" font-size="15" fill="#111111">'
                    f"{escape_xml(panel_title)}</text>"
                ),
                (
                    f'<line x1="{plot_x}" y1="{plot_y + plot_height}" '
                    f'x2="{plot_x + plot_width}" y2="{plot_y + plot_height}" '
                    'stroke="#333333" stroke-width="1"/>'
                ),
                (
                    f'<line x1="{plot_x}" y1="{plot_y}" '
                    f'x2="{plot_x}" y2="{plot_y + plot_height}" '
                    'stroke="#333333" stroke-width="1"/>'
                ),
            ]
        )

        for tick in range(5):
            fraction = tick / 4.0
            x = plot_x + fraction * plot_width
            value = min_x + fraction * (max_x - min_x)
            lines.extend(
                [
                    (
                        f'<line x1="{x:.1f}" y1="{plot_y}" '
                        f'x2="{x:.1f}" y2="{plot_y + plot_height}" '
                        'stroke="#dddddd" stroke-width="1"/>'
                    ),
                    (
                        f'<text x="{x:.1f}" y="{plot_y + plot_height + 20}" '
                        'text-anchor="middle" font-family="sans-serif" '
                        f'font-size="11" fill="#333333">{value:.0f}</text>'
                    ),
                ]
            )

        for tick in range(5):
            fraction = tick / 4.0
            y = plot_y + fraction * plot_height
            value = max_y - fraction * (max_y - min_y)
            lines.extend(
                [
                    (
                        f'<line x1="{plot_x}" y1="{y:.1f}" '
                        f'x2="{plot_x + plot_width}" y2="{y:.1f}" '
                        'stroke="#eeeeee" stroke-width="1"/>'
                    ),
                    (
                        f'<text x="{plot_x - 8}" y="{y + 4:.1f}" '
                        'text-anchor="end" font-family="sans-serif" '
                        f'font-size="11" fill="#333333">{value:.1f}</text>'
                    ),
                ]
            )

        legend_x = plot_x + plot_width - 120
        legend_y = plot_y + 16
        for index, (solve_index, points) in enumerate(sorted(series.items())):
            color = COLORS[index % len(COLORS)]
            coordinates = " ".join(
                f"{map_x(x):.1f},{map_y(y):.1f}" for x, y in points
            )
            lines.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2" '
                f'points="{coordinates}"/>'
            )
            if len(series) > 1:
                y = legend_y + 18 * index
                lines.extend(
                    [
                        (
                            f'<line x1="{legend_x}" y1="{y - 4}" '
                            f'x2="{legend_x + 22}" y2="{y - 4}" '
                            f'stroke="{color}" stroke-width="3"/>'
                        ),
                        (
                            f'<text x="{legend_x + 28}" y="{y}" '
                            'font-family="sans-serif" font-size="11" '
                            f'fill="#111111">solve {solve_index}</text>'
                        ),
                    ]
                )

        lines.extend(
            [
                (
                    f'<text x="{plot_x + plot_width / 2:.1f}" '
                    f'y="{origin_y + panel_height - 18}" '
                    'text-anchor="middle" font-family="sans-serif" '
                    'font-size="12" fill="#111111">Outer FBF iteration</text>'
                ),
                (
                    f'<text transform="translate({origin_x + 16} '
                    f'{plot_y + plot_height / 2:.1f}) rotate(-90)" '
                    'text-anchor="middle" font-family="sans-serif" '
                    'font-size="12" fill="#111111">log10 residual</text>'
                ),
            ]
        )

    lines.append("</svg>")
    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()
    if args.panel:
        if args.output is None:
            raise SystemExit("--output is required with --panel")
        panels = [(panel_title, load_series(Path(path))) for path, panel_title in args.panel]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(render_panel_svg(panels, args.title), encoding="utf-8")
        return

    if args.input_csv is None or args.output_svg is None:
        raise SystemExit("input_csv and output_svg are required without --panel")

    series = load_series(args.input_csv)
    args.output_svg.parent.mkdir(parents=True, exist_ok=True)
    args.output_svg.write_text(render_svg(series, args.title), encoding="utf-8")


if __name__ == "__main__":
    main()
