#!/usr/bin/env python3
"""Render a PR-thread benchmark comparison comment.

Compares a PR's Google Benchmark results (the merged ``combined.json`` produced by
``merge_benchmark_results.py``) against the Performance Dashboard's published
baseline (the ``data.js`` that ``benchmark-action/github-action-benchmark`` writes
to the ``gh-pages`` branch). Emits a Markdown comment with a stable marker so the
PR workflow can create-or-update a single comment in the review thread.

This is a read-only consumer of the dashboard's gh-pages history (PLAN-092); it
never writes benchmark history. The reconciliation decision behind it (extend the
dashboard infra rather than stand up a parallel pipeline) lives in
``docs/plans/dashboard.md`` (PLAN-092).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

COMMENT_MARKER = "<!-- dart-benchmark-pr-comparison -->"

# Smaller is better for timing metrics; normalize every unit to seconds so a
# baseline recorded in one unit still compares to a PR result in another.
_UNIT_SCALE = {
    "ns": 1e-9,
    "ns/iter": 1e-9,
    "us": 1e-6,
    "us/iter": 1e-6,
    "ms": 1e-3,
    "ms/iter": 1e-3,
    "s": 1.0,
    "s/iter": 1.0,
}


def _to_seconds(value: float, unit: str) -> float | None:
    scale = _UNIT_SCALE.get((unit or "").strip())
    if scale is None:
        return None
    return value * scale


def parse_current(path: Path) -> dict[str, float]:
    """Map benchmark name -> real_time in seconds from a Google Benchmark JSON."""
    data = json.loads(path.read_text(encoding="utf-8"))
    result: dict[str, float] = {}
    for row in data.get("benchmarks", []):
        if row.get("run_type") == "aggregate" and row.get("aggregate_name") not in (
            None,
            "median",
        ):
            # The real pipeline already selects the median row via
            # merge_benchmark_results.py; this guard just skips any stray
            # non-median aggregate rows.
            continue
        name = row.get("name") or row.get("run_name")
        seconds = _to_seconds(
            row.get("real_time", float("nan")), row.get("time_unit", "")
        )
        if name and seconds is not None:
            result[name] = seconds
    return result


def parse_baseline(path: Path, series: str) -> dict[str, float]:
    """Map benchmark name -> value in seconds from a github-action-benchmark data.js.

    The file looks like ``window.BENCHMARK_DATA = { ... };``. The most recent
    entry for the named series is the baseline.
    """
    text = path.read_text(encoding="utf-8").strip()
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        return {}
    data = json.loads(text[start : end + 1])
    entries = data.get("entries", {}).get(series, [])
    if not entries:
        return {}
    latest = entries[-1]
    result: dict[str, float] = {}
    for bench in latest.get("benches", []):
        name = bench.get("name")
        seconds = _to_seconds(bench.get("value", float("nan")), bench.get("unit", ""))
        if name and seconds is not None:
            result[name] = seconds
    return result


def render_comment(
    current: dict[str, float],
    baseline: dict[str, float],
    *,
    series: str,
    alert_ratio: float,
    baseline_available: bool,
) -> str:
    lines = [
        COMMENT_MARKER,
        f"## Benchmark comparison — {series}",
        "",
    ]
    if not baseline_available:
        lines += [
            (
                "_No published baseline was found on `gh-pages` yet, so only the "
                "current PR measurements are shown. The comparison populates once "
                "the Performance Dashboard has published history for this series._"
            ),
            "",
        ]
    if not current:
        lines += ["_No benchmark rows were produced for this run._", ""]
        return "\n".join(lines)

    rows = []
    for name in sorted(current):
        cur = current[name]
        base = baseline.get(name)
        if base is None or base == 0:
            rows.append((name, base, cur, None))
            continue
        ratio = cur / base
        rows.append((name, base, cur, ratio))

    # Sort largest-ratio (worst regression) first; new benchmarks (no baseline)
    # sort around the neutral boundary, then alphabetical tiebreak.
    def _sort_key(row):
        ratio = row[3]
        return (-(ratio if ratio is not None else 0.0), row[0])

    rows.sort(key=_sort_key)

    regressed = [r for r in rows if r[3] is not None and r[3] >= alert_ratio]
    threshold_pct = round((alert_ratio - 1.0) * 100)
    if regressed:
        lines.append(
            f"⚠️ {len(regressed)} benchmark(s) slower than the baseline by "
            f"≥ {threshold_pct}% (alert threshold)."
        )
    else:
        lines.append(f"✅ No benchmark slower than the baseline by ≥ {threshold_pct}%.")
    lines += [
        "",
        "| Benchmark | Baseline | PR | Δ | |",
        "| --- | ---: | ---: | ---: | :-: |",
    ]
    for name, base, cur, ratio in rows:
        cur_s = _fmt_time(cur)
        if ratio is None:
            lines.append(f"| `{name}` | — | {cur_s} | new | 🆕 |")
            continue
        delta_pct = (ratio - 1.0) * 100.0
        if -1.0 < delta_pct < 1.0:
            flag = "➖"  # within noise band
        elif ratio >= alert_ratio:
            flag = "🔴"  # regression past the alert threshold
        elif delta_pct < 0:
            flag = "🟢"  # improvement
        else:
            flag = "🟡"  # sub-alert regression
        sign = "+" if delta_pct >= 0 else ""
        lines.append(
            f"| `{name}` | {_fmt_time(base)} | {cur_s} | {sign}{delta_pct:.1f}% | {flag} |"
        )
    missing = sorted(set(baseline) - set(current))
    if missing:
        lines += ["", f"_Baseline-only (not run in this PR): {len(missing)}._"]
    lines += [
        "",
        (
            "<sub>Smaller is better. Baseline is the latest `gh-pages` dashboard "
            "entry; GitHub-hosted runners are noisy, so treat small deltas as "
            "noise.</sub>"
        ),
    ]
    return "\n".join(lines)


def _fmt_time(seconds: float) -> str:
    for unit, scale in (("s", 1.0), ("ms", 1e-3), ("us", 1e-6), ("ns", 1e-9)):
        if seconds >= scale:
            return f"{seconds / scale:.3g} {unit}"
    return f"{seconds / 1e-9:.3g} ns"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument(
        "--series", required=True, help="github-action-benchmark series name"
    )
    parser.add_argument("--alert-threshold", type=float, default=1.5)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    current = parse_current(args.current) if args.current.exists() else {}
    baseline_available = args.baseline.exists()
    baseline = parse_baseline(args.baseline, args.series) if baseline_available else {}
    comment = render_comment(
        current,
        baseline,
        series=args.series,
        alert_ratio=args.alert_threshold,
        baseline_available=baseline_available and bool(baseline),
    )
    args.output.write_text(comment + "\n", encoding="utf-8")
    print(f"Wrote benchmark PR comparison comment: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
