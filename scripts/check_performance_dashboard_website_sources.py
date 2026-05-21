#!/usr/bin/env python3
"""Verify Read the Docs sources expose the performance dashboard guide."""

from __future__ import annotations

import argparse
from pathlib import Path

GUIDE_DOC = "community/performance_dashboard"
PUBLIC_DASHBOARD_URL = "https://dartsim.github.io/dart/performance/"
REQUIRED_GUIDE_SNIPPETS = [
    PUBLIC_DASHBOARD_URL,
    "https://dartsim.github.io/dart/performance/status.json",
    "https://dartsim.github.io/dart/performance/data.json",
    "summary.md",
    "Quick Start",
    "Open the dashboard page",
    "Use ``status.json``",
    "Download ``data.json``",
    "Use ``summary.md``",
    "Add competitor rows",
    "Maintainers launch",
    "publication_contract",
    "schedule_crons",
    "latest_run_id",
    "latest successful ``main`` workflow run",
    "feature-branch artifacts cannot seed the",
    "expected update interval",
    "stale-after window",
    "Performance Dashboard",
    "refs/heads/main",
    "actions: read",
    "contents: write",
    "pages: write",
    "comparison_input_contract",
    "comparison_metric_contract",
    "External Competitor Status",
    "trend_summary",
    "testbed_summary",
    "curl -fsSL https://dartsim.github.io/dart/performance/status.json",
    "python -m json.tool",
    "backend_ns / primary_ns",
    "native_ns / best_reference_ns",
    "ratio < 1",
    "BM_Distance_BoxSphere_MuJoCo",
    "BM_KR5ForwardDynamics_Drake/1",
    "Supported external competitor suffixes",
    "github-action-benchmark",
    "Airspeed Velocity",
    "LLVM LNT",
    "pixi run check-bm-dashboard-launch-preflight",
    "pixi run check-bm-dashboard-pages-branch",
    "pixi run check-bm-dashboard-pages-build",
    "pixi run check-bm-dashboard-workflow-run",
    "pixi run check-bm-dashboard-launch-live",
    "gh workflow run performance_dashboard.yml --ref main",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--docs-root",
        type=Path,
        default=Path("docs/readthedocs"),
        help="Read the Docs source root.",
    )
    return parser.parse_args()


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Failed to read {path}: {exc}") from exc


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def verify_website_sources(docs_root: Path) -> None:
    index_path = docs_root / "index.rst"
    guide_path = docs_root / f"{GUIDE_DOC}.rst"
    index = _read(index_path)
    guide = _read(guide_path)

    _require(
        f"<{GUIDE_DOC}>" in index and "DART Performance Dashboard" in index,
        f"{index_path} does not visibly link to the dashboard guide.",
    )
    _require(
        PUBLIC_DASHBOARD_URL in index
        and "https://dartsim.github.io/dart/performance/status.json" in index,
        f"{index_path} does not directly expose the hosted dashboard URLs.",
    )
    _require(
        any(line.strip() == GUIDE_DOC for line in index.splitlines()),
        f"{index_path} does not include {GUIDE_DOC!r} in a toctree.",
    )

    missing = [snippet for snippet in REQUIRED_GUIDE_SNIPPETS if snippet not in guide]
    _require(
        not missing,
        f"{guide_path} is missing dashboard launch guidance: {', '.join(missing)}",
    )


def main() -> int:
    args = parse_args()
    try:
        verify_website_sources(args.docs_root)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    print(f"performance dashboard website sources verified: {args.docs_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
