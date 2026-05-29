# Per-PR Benchmark Comparison Comments — Dev Task

## Current Status

- [ ] Phase 0: Capture design intent and reconciliation with PLAN-080 (this PR)
- [ ] Phase 1: Decide whether to extend the existing Performance Dashboard
      runner with PR-comment hooks (preferred) or stand up a parallel
      "committed JSON in `benchmarks/reports/`" pipeline as the prototype
      proposed
- [ ] Phase 2: Implement the chosen mechanism; comment template should
      reference benchmark name, runner id, baseline ref, delta with
      confidence interval, and link to the dashboard for history
- [ ] Phase 3: Roll out gradually — start with a single benchmark suite on a
      stable runner; expand once review-noise tradeoffs are visible

## Goal

Surface a PR-level "benchmark moved by N%" signal in the review thread so
performance regressions are caught at review time rather than after merge,
without contradicting the Performance Dashboard (PLAN-080, Complete) that
already owns long-form benchmark history on GitHub Pages.

## Non-Goals (for early phases)

- Replacing the Performance Dashboard. PLAN-080 owns time-series history;
  this work adds an orthogonal PR-time signal.
- Comparing across runners. PR comparisons match a single runner id only;
  cross-machine noise is out of scope.
- Publishing the comparison from forks. Initial rollout uses the
  protected-branch runner pattern already used by other DART CI workflows.

## Key Decisions

- **Reconcile with the Performance Dashboard before implementing**. PLAN-080
  uses `benchmark-action/github-action-benchmark` and publishes per-benchmark
  history to `dartsim.github.io/dart/performance/`. The prototype branch
  built a parallel pipeline (committed JSON reports + custom site + custom
  PR-comment workflow). Pick one source of truth; do not run both.
- **PR comment is the deliverable**, not the dashboard. The comment cites
  benchmark name, runner id, baseline ref / SHA, delta, and links back to
  the dashboard for time-series context.
- **Match runner ids, not hostnames**. The prototype's report-sanitization
  step (strip hostnames and executable paths, record a non-sensitive runner
  id under `context.dart_bench`) is the right shape and should be reused.
- **Open question, blocks Phase 1**: whether the comparison data should be
  produced by extending the existing dashboard's published JSON (read from
  gh-pages) or by re-running on the PR and comparing against a cached
  baseline. The first reuses existing infrastructure; the second matches the
  prototype's design.

## Prototype Reference

The original prototype lived on the now-deleted `bm-report` branch. Commit
SHAs (recoverable from reflog and `git fsck --unreachable` until natural
expiry):

- `3a36c6130a1 Add benchmark report workflows and dashboard` — full
  prototype, including two workflows (`benchmark_publish.yml`,
  `benchmark_reports.yml`), a site at `benchmarks/site/{index.html,site.js,
styles.css}`, three scripts (`benchmark_report_compare.py`,
  `benchmark_report_site.py`, `run_cpp_benchmark.py`), and the
  `pixi run bm-report` task.
- `f6fc8257e33` and `3d8ffde9037` — added and relocated the resume prompt.

To inspect the prototype while reflog still holds it:

```bash
git show 3a36c6130a1 --stat
git show 3a36c6130a1:scripts/benchmark_report_compare.py
git show 3a36c6130a1:.github/workflows/benchmark_reports.yml
git show 3a36c6130a1:docs/dev_tasks/bm_report/README.md
```

## Related Plans

- PLAN-080 Performance Dashboard
  (`docs/readthedocs/community/performance_dashboard.rst`,
  `.github/workflows/performance_dashboard.yml`) — the existing time-series
  surface this work must reconcile with.

## Immediate Next Steps

1. Land this dev-task folder.
2. Before any Phase 1 implementation, write `01-reconciliation.md` here
   recording the answer to the "extend the dashboard runner or stand up a
   parallel pipeline" decision. Phase 1 is blocked until that lands.

## Verification Gates

- Phase 2: `pixi run lint`, `pixi run check-lint-yaml` for the workflow,
  manual dry-run on a PR comparing the comment to the dashboard's history
  for the same benchmark.
- Phase 3: regression test — at least one stable benchmark must be wired
  before broader rollout, and the comment must remain quiet on noise below
  a documented threshold.
