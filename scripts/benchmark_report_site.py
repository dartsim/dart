#!/usr/bin/env python3
"""Generate benchmark dashboard data from committed reports."""
from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timezone
from pathlib import Path

UNIT_SCALE = {
    "ns": 1e-9,
    "us": 1e-6,
    "ms": 1e-3,
    "s": 1.0,
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reports-dir",
        type=Path,
        default=Path("benchmarks") / "reports",
        help="Root directory containing benchmark reports.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("benchmarks") / "site" / "data.json",
        help="Output JSON file for the dashboard.",
    )
    return parser.parse_args(argv)


def _time_seconds(entry: dict) -> float | None:
    value = entry.get("real_time")
    unit = entry.get("time_unit", "ns")
    if value is None:
        value = entry.get("cpu_time")
    if value is None:
        return None
    scale = UNIT_SCALE.get(unit)
    if scale is None:
        return None
    return float(value) * scale


def _bench_key(entry: dict) -> str | None:
    name = entry.get("name")
    if not name:
        return None
    aggregate = entry.get("aggregate_name")
    if aggregate:
        return f"{name}/{aggregate}"
    return name


def _parse_timestamp(name: str) -> str | None:
    match = re.match(r"(\d{8}-\d{6}Z)", name)
    if not match:
        return None
    ts = datetime.strptime(match.group(1), "%Y%m%d-%H%M%SZ").replace(tzinfo=timezone.utc)
    return ts.isoformat()


def _report_metadata(path: Path, data: dict) -> dict:
    context = data.get("context", {})
    meta = context.get("dart_bench", {}) if isinstance(context, dict) else {}
    slug = meta.get("benchmark") or path.parts[-3]
    runner_id = meta.get("runner_id") or path.parts[-2]
    timestamp = meta.get("timestamp") or _parse_timestamp(path.name) or ""
    return {
        "slug": slug,
        "runner_id": runner_id,
        "timestamp": timestamp,
        "git_sha": meta.get("git_sha", ""),
        "git_branch": meta.get("git_branch", ""),
        "build_type": meta.get("build_type", ""),
        "pixi_env": meta.get("pixi_env", ""),
    }


def _load_reports(root: Path) -> list[dict]:
    reports: list[dict] = []
    for path in sorted(root.rglob("*.json")):
        data = json.loads(path.read_text())
        meta = _report_metadata(path, data)
        entries: dict[str, float] = {}
        for entry in data.get("benchmarks", []):
            if entry.get("error_occurred"):
                continue
            key = _bench_key(entry)
            if not key:
                continue
            seconds = _time_seconds(entry)
            if seconds is None:
                continue
            entries[key] = seconds
        reports.append(
            {
                **meta,
                "source": path.as_posix(),
                "entries": entries,
            }
        )
    reports.sort(key=lambda item: (item.get("slug", ""), item.get("runner_id", ""), item.get("timestamp", "")))
    return reports


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    reports = _load_reports(args.reports_dir)
    output = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "reports": reports,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    import sys

    raise SystemExit(main(sys.argv[1:]))
