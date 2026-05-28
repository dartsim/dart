# Resume: Per-PR Benchmark Comparison Comments

## Last Session Summary

Captured the per-PR benchmark-comparison comment design from the retired
`bm-report` branch as a tracked dev task. The prototype built a parallel
benchmark pipeline (committed JSON reports + custom site + workflows) that
overlapped with the existing Performance Dashboard (PLAN-080, Complete).
Phase 1 is blocked on a reconciliation decision: extend the dashboard
runner with PR-comment hooks, or stand up the prototype's parallel
pipeline.

## Current Branch

`main` — no implementation work after the prototype.

## Immediate Next Step

Write `docs/dev_tasks/benchmark_pr_comparison/01-reconciliation.md` choosing
between (a) extending the existing
`.github/workflows/performance_dashboard.yml` runner with a PR-comment
hook that reads gh-pages history for the baseline, or (b) standing up the
prototype's parallel `benchmark_reports.yml` workflow with committed JSON
reports. Phase 1 is blocked until that decision is recorded.

## Context That Would Be Lost

- The Performance Dashboard already publishes per-benchmark history via
  `benchmark-action/github-action-benchmark` to
  `dartsim.github.io/dart/performance/`. Do not run two pipelines.
- The prototype's report sanitization (strip hostnames and executable
  paths, record a runner id under `context.dart_bench`) is the right shape;
  reuse it.
- PR comparisons match a single runner id; cross-machine comparison is out
  of scope.

## How to Resume

```bash
git checkout main
git pull --ff-only origin main
git checkout -b feature/bm-comparison-reconciliation

# Inspect the prototype (while reflog still holds it):
git show 3a36c6130a1 --stat
git show 3a36c6130a1:scripts/benchmark_report_compare.py
git show 3a36c6130a1:.github/workflows/benchmark_reports.yml
```

Then write `01-reconciliation.md` and unblock Phase 1.
