## Summary

- Adds a DART-owned static performance dashboard pipeline that turns Google
  Benchmark JSON into durable `gh-pages/performance/` history and a public
  dashboard.
- Links the hosted dashboard, `status.json`, and dashboard guide from the new
  Read the Docs website and documents why GitHub Pages is the canonical host
  with optional Bencher reporting.

## Motivation / Problem

- DART's current benchmark evidence is split across local benchmark commands,
  expiring GitHub Actions artifacts, and narrow collision checks.
- The project needs a hosted dashboard that shows performance changes over time
  and keeps native/reference comparisons available after artifacts expire.

## Changes / Key Changes

- Adds `scripts/generate_performance_dashboard.py` plus verifiers, endpoint
  checks, artifact seeding, workflow dry-run, workflow registration check,
  Actions summary, Bencher command construction, and bounded benchmark-surface
  runner scripts.
- Adds `.github/workflows/performance_dashboard.yml` for path-scoped `main`
  push, scheduled, and manual benchmark collection, dashboard rendering,
  optional Bencher reporting, and `gh-pages/performance/` publication from
  `main`, including a Pages configuration preflight and explicit Pages build
  request after the workflow updates the legacy branch-source site. The
  publisher loads existing history first, then cleans stale files from
  `gh-pages/performance/` before writing the current static payload.
- Adds Pixi tasks for local rendering, verification, workflow-shaped dry runs,
  publication preflight, artifact seeding, bounded dashboard surfaces,
  `gh-pages` dashboard-path verification, latest-workflow-run verification,
  and pre/post-merge launch checks.
- Collects bounded non-collision dashboard rows for common allocator,
  dynamics/kinematics, real robot dynamics I/O, LCP, math helpers, SIMD,
  simulation world-step, and compute graph surfaces.
- Makes the generated `data.json` and `status.json` self-describing with
  canonical Read the Docs, dashboard, status, data, summary, and guide URLs,
  plus the main-push/scheduled/manual update triggers.
- Adds a compact `trend_summary` to `data.json`, `status.json`, dashboard
  HTML, `summary.md`, and the Actions summary so maintainers can inspect
  primary/reference trend row counts, status/threshold counts, and largest
  improvement/regression snapshots without downloading full history.
- Adds a compact `testbed_summary` to `data.json`, `status.json`, dashboard
  HTML, `summary.md`, and the Actions summary so over-time movement can be
  interpreted against the latest benchmark host, CPU count, and
  CPU-scaling/ASLR state.
- Adds a compact `external_competitor_status` to `data.json`, `status.json`,
  dashboard HTML, `summary.md`, and the Actions summary so the hosted page and
  publisher run summary show whether external competitor harness rows are live,
  which backends/surfaces are present, and the live-claim rule used for
  advanced comparisons.
- Includes the hosted-site artifact contract in the generated publication
  metadata, dashboard HTML, public `summary.md`, and Actions summary so the
  post-merge workflow-run check can prove the first dashboard payload was
  archived before treating the hosted endpoint as live.
- Makes the post-merge live launch gate pass the latest successful `main`
  workflow run identity into the publication verifier, so public
  `latest_run_id` and SHA must match that run before the hosted dashboard is
  accepted as live.
- Aggregates canonical website, guide, status, data, dashboard HTML, and
  `summary.md` failures in the live publication check so first-launch blockers
  are reported together.
- Makes the live launch gate require `gh-pages/performance/` to exist and
  contain dashboard files, while the no-push preflight still accepts an absent
  path before first publication.
- Makes workflow registration require exact hosted status/data/HTML/summary
  checks and the Pages propagation retry budget in the publisher workflow.
- Filters first-run seed artifacts by branch so PR or feature-branch artifacts
  cannot enter the public `main` dashboard history.
- Includes canonical website dashboard/status/guide links in the generated
  publication contract so public payloads advertise the same website wiring
  that the launch checks enforce.
- Includes the checked `schedule_crons`, expected update interval, and
  stale-after window in the generated publication contract, and makes workflow
  registration reject schedule drift from the twice-weekly dashboard cadence.
- Adds a generated `comparison_metric_contract` so native/reference and
  competitor ratio formulas stay machine-readable in the same public payloads
  as the rendered dashboard.
- Carries the dashboard service decision through generated `data.json`,
  `status.json`, `index.html`, `summary.md`, and the Actions summary: GitHub
  Pages is the selected DART-owned host, Bencher is the opt-in external
  history/threshold companion, CodSpeed is a deferred microbenchmark pilot,
  `github-action-benchmark` is a not-selected off-the-shelf Pages action,
  Airspeed Velocity is a not-selected static benchmark dashboard framework,
  LLVM LNT is a not-selected self-hosted performance-tracking server option,
  Conbench is the self-hosted fallback, and OpenBenchmarking.org/Phoronix Test
  Suite is a not-selected future public ecosystem-campaign option.
- Adds compact `service_decision_summary` metadata to `data.json`,
  `status.json`, dashboard HTML, `summary.md`, and the Actions summary so the
  hosted endpoint states the recommended DART option directly: GitHub Pages as
  the canonical dashboard/data host, with Bencher as the opt-in external
  history/threshold companion after maintainer approval.
- Shows live DART reference comparisons only when backed by reproducible JSON,
  while listing deferred competitor/reference harness candidates such as
  MuJoCo, Drake, Gazebo/gz-physics, PyBullet, and Simbody as roadmap entries.
- Adds a durable service decision in `docs/plans/080-performance-dashboard.md`,
  a direct hosted-dashboard/status link from the Read the Docs homepage, and a
  public Read the Docs guide at
  `docs/readthedocs/community/performance_dashboard.rst`.
- Adds a Quick Start section to the public guide so readers know which hosted
  surface to use first: dashboard HTML, `status.json`, `data.json`,
  `summary.md`, competitor JSON rows, or the maintainer launch checks.
- Updates `CHANGELOG.md` for the new Tooling/Docs dashboard pipeline.

## Testing

- `pixi run --locked lint`
- `pixi run --locked python -m pytest $(find python/tests/unit -maxdepth 1 -name 'test_*performance_dashboard*.py' | sort) -q` (170 passed)
- `pixi run --locked bm-dashboard && pixi run --locked check-bm-dashboard`
- `pixi run --locked check-bm-dashboard-service-sources`
- `pixi run --locked check-bm-dashboard-launch-preflight`
- `pixi run --locked docs-build` (passed with existing dartpy autodoc warnings)
- `git diff --check origin/main..HEAD`

## Breaking Changes

- [x] None

## Related Issues / PRs (backports)

- None.

---

#### Checklist

- [ ] Milestone set (DART 7.0 for `main`, DART 6.16.x for `release-6.16`)
- [x] CHANGELOG.md updated if required
- [x] Add unit tests for new functionality
- [x] Document new methods and classes
- [x] Add Python bindings (dartpy) if applicable (not applicable)

Notes:

- This PR does not add public C++ APIs or dartpy bindings.
- The public dashboard URL is expected to return 404 until this workflow lands
  on `main` and publishes `gh-pages/performance/` once.
- Current launch-preflight state: local Pages config, Read the Docs source
  wiring, `gh-pages` dashboard-path state, workflow shape, seed artifact
  download, workflow-shaped dashboard render, local endpoint checks, and
  publication preflight pass; the dry run renders 197 measurements.
- Current live endpoint state: `https://dart.readthedocs.io/en/latest/`
  returns 200, while the new dashboard guide and
  `https://dartsim.github.io/dart/performance/` endpoints return 404 until the
  workflow is merged and run on `main`.
- `pixi run check-bm-dashboard-launch-live` currently fails before merge for
  the expected reason: GitHub Actions has not registered
  `.github/workflows/performance_dashboard.yml` on `main` yet, so the latest
  workflow-run check reports that workflow runs are not available yet.
- Remote approval boundary: pushing this branch, opening this PR, merging it,
  dispatching the workflow, and publishing `gh-pages/performance/` require
  explicit maintainer approval.
- First launch after merge: run
  `pixi run check-bm-dashboard-pages-branch` and
  `pixi run check-bm-dashboard-pages-build` and
  `pixi run check-bm-dashboard-workflow-registration`, dispatch with
  `gh workflow run performance_dashboard.yml --ref main`, confirm the latest
  `main` publisher completed and uploaded the hosted-site dashboard artifact
  with `pixi run check-bm-dashboard-workflow-run`, then run
  `pixi run check-bm-dashboard-launch-live`.
