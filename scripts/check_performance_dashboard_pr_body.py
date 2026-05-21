#!/usr/bin/env python3
"""Verify the performance dashboard PR body draft stays launch-ready."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

DEFAULT_PR_BODY = Path("docs/dev_tasks/performance_dashboard/PR_BODY.md")
DEFAULT_TEMPLATE = Path(".github/PULL_REQUEST_TEMPLATE.md")

REQUIRED_SNIPPETS = [
    ".github/workflows/performance_dashboard.yml",
    "gh-pages/performance/",
    "https://dartsim.github.io/dart/performance/",
    "direct hosted-dashboard/status link",
    "Quick Start",
    "https://dart.readthedocs.io/en/latest/",
    "docs/readthedocs/community/performance_dashboard.rst",
    "docs/plans/080-performance-dashboard.md",
    "trend_summary",
    "testbed_summary",
    "service_decision_summary",
    "comparison_metric_contract",
    "schedule_crons",
    "GitHub Pages is the selected DART-owned host",
    "Bencher",
    "CodSpeed",
    "github-action-benchmark",
    "Airspeed Velocity",
    "LLVM LNT",
    "Conbench",
    "OpenBenchmarking.org/Phoronix Test Suite",
    "canonical website dashboard/status/guide links",
    "public `latest_run_id` and SHA must match",
    "first-launch blockers",
    "exact hosted status/data/HTML/summary",
    "require `gh-pages/performance/` to exist",
    "Filters first-run seed artifacts by branch",
    "simulation world-step",
    "compute graph surfaces",
    "MuJoCo",
    "Drake",
    "Gazebo/gz-physics",
    "PyBullet",
    "pixi run --locked lint",
    "170 passed",
    "pixi run --locked bm-dashboard && pixi run --locked check-bm-dashboard",
    "pixi run --locked check-bm-dashboard-service-sources",
    "pixi run --locked check-bm-dashboard-launch-preflight",
    "pixi run --locked docs-build",
    "git diff --check origin/main..HEAD",
    "pixi run check-bm-dashboard-pages-branch",
    "pixi run check-bm-dashboard-pages-build",
    "pixi run check-bm-dashboard-workflow-registration",
    "pixi run check-bm-dashboard-workflow-run",
    "gh workflow run performance_dashboard.yml --ref main",
    "pixi run check-bm-dashboard-launch-live",
    "404 until this workflow lands",
    "Remote approval boundary",
    "explicit maintainer approval",
    "dispatching the workflow",
    "Milestone set",
]

FORBIDDEN_SNIPPETS = [
    "build/performance-dashboard-pr-body.md",
    "118 passed",
    "109 passed",
    "122 passed",
    "128 passed",
    "129 passed",
    "134 passed",
    "139 passed",
    "152 passed",
    "154 passed",
    "155 passed",
    "156 passed",
    "158 passed",
    "159 passed",
    "161 passed",
    "162 passed",
    "165 passed",
    "166 passed",
    "167 passed",
    "169 passed",
    "4474f967",
    "2f15425",
    "46cea557",
    "1bc6ccf",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pr-body",
        type=Path,
        default=DEFAULT_PR_BODY,
        help="PR body draft to verify.",
    )
    parser.add_argument(
        "--template",
        type=Path,
        default=DEFAULT_TEMPLATE,
        help="Repository pull request template.",
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


def _template_headers(template: str) -> list[str]:
    headers: list[str] = []
    for line in template.splitlines():
        match = re.match(r"^(#{2,4})\s+(.+?)\s*$", line)
        if not match:
            continue
        text = match.group(2)
        if text == "Checklist":
            headers.append("#### Checklist")
        elif match.group(1) == "##":
            headers.append(f"## {text}")
    return headers


def verify_pr_body(pr_body: Path, template: Path) -> None:
    body_text = _read(pr_body)
    template_text = _read(template)
    normalized_body = re.sub(r"\s+", " ", body_text)

    _require(body_text.strip(), f"{pr_body} is empty")
    for header in _template_headers(template_text):
        _require(
            header in body_text, f"{pr_body} is missing template header {header!r}"
        )

    missing = [
        snippet
        for snippet in REQUIRED_SNIPPETS
        if re.sub(r"\s+", " ", snippet) not in normalized_body
    ]
    _require(
        not missing,
        f"{pr_body} is missing launch-ready dashboard details: {', '.join(missing)}",
    )

    forbidden = [snippet for snippet in FORBIDDEN_SNIPPETS if snippet in body_text]
    _require(
        not forbidden,
        f"{pr_body} still contains stale dashboard PR body details: {', '.join(forbidden)}",
    )


def main() -> int:
    args = parse_args()
    try:
        verify_pr_body(args.pr_body, args.template)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    print(f"performance dashboard PR body verified: {args.pr_body}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
