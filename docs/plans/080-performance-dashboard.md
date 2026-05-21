# DART Performance Dashboard

- Operating state: `PLAN-080` in `docs/plans/dashboard.md`
- Outcome: a public, continuously updated performance dashboard that shows
  DART benchmark history, row-level detail, variance, run environment, and
  native/reference comparisons without depending on short-lived CI artifacts.
- Website integration: <https://dart.readthedocs.io/en/latest/> is the new
  DART website and should link directly to the hosted dashboard plus the
  dashboard guide.
- Hosted dashboard target: <https://dartsim.github.io/dart/performance/> via
  the repository's existing `gh-pages` GitHub Pages site. Local generation is
  only a verification path.
- Current evidence: DART already has Google Benchmark targets under
  `tests/benchmark/`, `pixi run bm`, `pixi run bm-collision-check`, JSON
  comparison scripts, scheduled/manual CI benchmark artifacts, a new Read the
  Docs website at <https://dart.readthedocs.io/en/latest/>, and an existing
  GitHub Pages branch that can host generated dashboard files.
- Source status checked on 2026-05-21: GitHub Pages remains available for
  public repositories, GitHub Actions artifacts remain retention-limited,
  Bencher remains free for public/open-source projects and supports project API
  keys plus the `cpp_google` adapter, and CodSpeed remains free on public
  repositories but still requires its own C++ Google Benchmark integration path;
  `github-action-benchmark` remains an open-source GitHub Pages action with
  Google Benchmark support; Airspeed Velocity remains an open-source static
  history-dashboard framework; LLVM LNT remains an open-source self-hosted
  performance-tracking server with JSON submission and custom test-suite
  support; Conbench remains an open-source self-hosted API/dashboard option;
  OpenBenchmarking.org remains a free public Phoronix Test Suite
  result-storage and comparison ecosystem.

## Requirements

- **Live history:** main-push, scheduled, and manual CI runs must append
  durable benchmark data. GitHub Actions artifacts alone are insufficient
  because public repository artifacts expire within the GitHub retention
  window.
- **Consolidated view:** one dashboard should cover collision, dynamics, math,
  LCP, SIMD, allocator, simulation, compute graph, and future GPU benchmark
  families.
- **Competitor/reference comparison:** native DART rows should be comparable to
  explicit reference implementations where DART already has reproducible
  benchmark harnesses. Historical external comparisons, such as the OSRF Atlas
  real-time-factor result on the Read the Docs website gallery, should be
  contextual links, not live competitor data unless they are backed by a
  reproducible harness. The generated coverage table may name deferred
  competitor/reference candidates, but those names are a harness roadmap, not a
  dashboard result.
- **Advanced detail:** the dashboard must retain benchmark name, family,
  backend/reference label, metric, median/mean/stddev when available, runner
  context, commit SHA, branch, run ID, source artifact, and threshold status.
- **Low maintainer burden:** the first version should use the existing Pixi,
  Google Benchmark, GitHub Actions, and GitHub Pages surfaces before adding a
  service that changes the build system.

## Operating Model

The dashboard is a static site, not a long-running server. Benchmark jobs write
Google Benchmark JSON into `.benchmark_results/`; the generator normalizes those
files, appends them to durable `data.json` history, and writes the static site
files under `gh-pages/performance/`. The generated `status.json` endpoint also
reports freshness for the latest run so a public dashboard can be monitored for
stale data, not only for file availability.

Local usage is for verification and preview:

```bash
pixi run bm-dashboard-surfaces --continue-on-error
pixi run bm-dashboard
pixi run check-bm-dashboard
pixi run check-bm-dashboard-pages
pixi run check-bm-dashboard-service-sources
pixi run check-bm-dashboard-workflow-dry-run
pixi run check-bm-dashboard-publication
```

Open `build/performance-dashboard/index.html` to inspect the same static
payload that the web workflow publishes. `pixi run check-bm-dashboard-empty`
covers the first-publication fallback where the workflow creates the hosted
dashboard shell even if benchmark collection produced no JSON.
`check-bm-dashboard-publication` verifies the generated local payload, confirms
the canonical website is reachable, and reports whether the public dashboard
endpoint is already live.
`check-bm-dashboard-service-sources` verifies that the external service
evidence URLs behind the GitHub Pages/Bencher/CodSpeed/
github-action-benchmark/Airspeed-Velocity/LLVM-LNT/Conbench/OpenBenchmarking decision remain reachable
without redirects and still contain the source phrases DART relies on for
free/public repository availability, GitHub Actions integration, Google
Benchmark ingestion, static publishing, or self-hosted/API support. It uses
bounded retries for transient network failures and HTTP 5xx responses, but
redirect, unconfigured evidence text, missing evidence text, and non-transient
evidence drift still fail immediately. `check-bm-dashboard-workflow-dry-run` renders the
workflow-shaped
`gh-pages/performance/` tree under ignored `build/` state, verifies the
dashboard files, and endpoint-checks the generated local `status.json` before
maintainers approve the real Pages publish. `check-bm-dashboard-pages` verifies
that the repository's public Pages settings still point at the expected
`gh-pages` branch-source site before the workflow depends on that host.

Hosted usage is through the main-push/scheduled/manual GitHub Actions workflow.
On `main`, `.github/workflows/performance_dashboard.yml` checks out
`gh-pages`, optionally seeds history from recent benchmark artifacts for the
active branch when no durable history exists, runs collision plus bounded
non-collision benchmark surfaces, renders `gh-pages/performance/`, commits
that directory to the `gh-pages` branch when the generated files changed,
requests a Pages build for the branch-source site on every `main` publisher
run, and then verifies the public `status.json` and `data.json` endpoints
against the generated local files. The publisher loads existing `data.json`
history before cleaning the target `performance/` directory, so stale files
from older dashboard payloads do not stay published.
After the workflow lands on `main`, maintainers can first confirm that GitHub
Actions registered it as an active main-push/scheduled/manual workflow:

```bash
pixi run check-bm-dashboard-workflow-registration
```

After the first successful publication, maintainers can run:

```bash
pixi run check-bm-dashboard-launch-live
```

That public launch check requires workflow registration, Pages configuration,
a valid local workflow-shaped render, the Read the Docs guide page, the latest
successful `main` dashboard workflow run with the hosted-site artifact,
`status.json` `latest_run_id` and source SHA values that match that workflow
run, `freshness.state == "fresh"`, and exact hosted status/data/HTML/summary
matches against the generated payload, so stale, partially published,
unregistered, unlinked, or mismatched dashboard data fails instead of looking
like a healthy static page.

The public dashboard link belongs on the canonical Read the Docs website at
<https://dart.readthedocs.io/en/latest/>. Until the first `main` workflow run
publishes `gh-pages/performance/`, the target URL is expected to return 404.
The published website also has a user-facing dashboard guide at
`docs/readthedocs/community/performance_dashboard.rst`, so the homepage can
expose the hosted dashboard URL and `status.json` health endpoint while linking
to DART-owned usage and interpretation guidance.

## Current Repo Evidence

- `scripts/run_cpp_benchmark.py` wraps C++ benchmark targets for `pixi run bm`.
- `scripts/check_collision_benchmarks.py` consumes Google Benchmark JSON and
  compares native rows against explicit reference rows.
- `pixi.toml` defines the broad collision benchmark guard as
  `bm-collision-check` and the bounded non-collision dashboard slice as
  `bm-dashboard-surfaces`. It also defines `check-bm-dashboard-public` for the
  hosted `status.json` endpoint after the first Pages publication; that check
  requires a fresh status endpoint, reachable `data.json` history, dashboard
  HTML, and `summary.md`. The companion
  `check-bm-dashboard-publication` task is the local launch preflight that
  allows the endpoint to be unpublished before the first `main` workflow run.
  `check-bm-dashboard-publication-live` is the lower-level live endpoint check
  that also requires the Read the Docs dashboard guide to be published, while
  `check-bm-dashboard-launch-live` is the post-merge completion gate that
  includes workflow registration, latest successful workflow-run identity, and
  Pages configuration checks.
- `pixi run check-bm-dashboard-service-sources` verifies every
  `service_decision[*].evidence_urls` link and rejects redirects, so stale
  free/open-source service-source URLs fail before launch, including the
  Airspeed Velocity static-dashboard framework, LLVM LNT self-hosted
  performance-tracking server, and OpenBenchmarking/Phoronix future-campaign
  options. The checker retries transient network failures and HTTP 5xx
  responses so temporary upstream service errors do not hide real launch
  readiness.
- `scripts/run_performance_dashboard_benchmarks.py` runs representative
  common allocator, dynamics/kinematics, real robot dynamics I/O, LCP smoke,
  math helper, SIMD add, simulation world-step, and compute graph rows through
  `scripts/run_cpp_benchmark.py` and writes Google Benchmark JSON under
  `.benchmark_results/`.
- `scripts/generate_performance_dashboard.py` is the initial normalizer and
  static renderer for dashboard-ready `data.json`, `index.html`, and
  `summary.md`, plus a compact `status.json` publication endpoint. The
  renderer includes latest native/reference comparisons, per-surface
  reference-backend summaries with geometric mean ratios, an all-backend
  comparison matrix with primary/reference ratios and variance fields,
  explicit reference/competitor coverage, previous-run deltas, compact
  testbed/environment metadata, per-row native and reference-backend history
  sparklines, recent run history, and embedded JSON for deeper inspection. The
  coverage view distinguishes live
  reproducible DART reference backend comparisons from primary-only surfaces
  and deferred external competitor claims. Reproducible external competitor
  rows are recognized when Google Benchmark row names use backend suffixes such
  as `_MuJoCo`, `_Drake`, `_Gazebo`, or `_PyBullet`; those rows become live
  comparison coverage instead of roadmap-only candidates. The generated
  `comparison_input_contract` in `data.json`, `status.json`, and `summary.md`
  publishes the exact contract for external harness authors: Google Benchmark
  row names use a shared benchmark prefix plus a `_<Backend>` suffix before
  optional Google Benchmark parameters, and the supported external suffixes are
  Chrono, Drake, Gazebo, GzPhysics, Isaac, IsaacGym, MuJoCo, PyBullet, RaiSim,
  and Simbody. Deferred
  competitor/reference candidates are still listed per surface with the
  comparison target, entrypoint, and next step so future MuJoCo, Drake,
  Gazebo/gz-physics, PyBullet, Simbody, and solver baselines have an explicit
  integration path without being presented as live data. The compact
  `status.json` endpoint also carries live external-competitor row totals,
  backend names, surfaces, and sample comparison groups so monitoring can
  inspect competitor-harness coverage without downloading the full history.
  The compact `testbed_summary` reports latest-run source files, host names,
  CPU counts, and CPU-scaling/ASLR state counts so over-time movement can be
  interpreted against the benchmark environment.
  Trend rows also carry an explicit informational threshold policy so
  slower/faster labels are traceable without becoming required PR gates, and
  the compact `status.json` endpoint reports whether the latest run is fresh or
  stale relative to the scheduled publisher
  cadence. When no durable dashboard history exists yet, it can seed history
  from real downloaded benchmark artifact JSON before appending the current
  run. Seeded run IDs include the artifact/source directory so multiple
  artifacts from the same CI timestamp do not overwrite each other. The
  generated HTML links directly to `status.json`, `data.json`, `summary.md`,
  and the Read the Docs dashboard guide so the hosted page exposes both its
  machine-readable endpoints and interpretation guidance. Empty zero-byte JSON
  files from failed benchmark attempts are skipped so one failed surface does
  not prevent publishing the remaining valid dashboard data. The freshness
  metadata advertises the path-scoped `main` push, scheduled, and manual
  workflow triggers so endpoint checks prove the hosted dashboard is wired to
  the intended live update path.
- The generated `data.json` and compact `status.json` endpoint include
  self-describing public links for the canonical Read the Docs website, the
  GitHub Pages dashboard, `status.json`, `data.json`, `summary.md`, and the
  dashboard guide. The endpoint verifier checks those links so a live status
  response proves the dashboard is wired to the new website and intended Pages
  host.
- The generated `data.json`, compact `status.json`, dashboard HTML, and
  `summary.md` also include `publication_contract`: the GitHub Pages host,
  `gh-pages` source branch, `/performance/` dashboard path, branch-source build
  type, publisher workflow, `refs/heads/main` publish ref, required
  `schedule_crons`, expected update interval, stale-after window, required
  Actions permissions, required public endpoints, required canonical website
  links to the dashboard, health endpoint, and guide, launch checks, and
  explicit maintainer approval boundary. The workflow-registration preflight
  rejects schedule drift from the expected twice-weekly cadence, and the local
  and hosted verifiers reject missing or drifting publication-contract
  metadata.
- The generated dashboard also carries the service decision in `data.json`,
  `status.json`, `index.html`, `summary.md`, and the Actions summary: GitHub
  Pages is the selected DART-owned dashboard/data host, Bencher is the
  recommended external history/threshold service after maintainer approval,
  CodSpeed is a deferred microbenchmark pilot, `github-action-benchmark` is a
  not-selected off-the-shelf Pages action, Airspeed Velocity is a not-selected
  static benchmark dashboard framework, LLVM LNT is a not-selected
  self-hosted performance-tracking server option, Conbench is the self-hosted
  fallback, and OpenBenchmarking.org/Phoronix Test Suite is a not-selected
  future public ecosystem-campaign option. The endpoint verifier checks these
  roles so the public status endpoint cannot silently drift from the intended
  architecture.
- The generated `service_decision_summary` in `data.json`, `status.json`, the
  dashboard page, `summary.md`, and the Actions summary states the selected
  architecture directly: DART-owned GitHub Pages is the canonical static
  dashboard/data host, Bencher is the opt-in external history/threshold
  companion after maintainer approval, and the other evaluated services remain
  deferred or fallback options.
- `pixi run bm-dashboard` renders existing `.benchmark_results/*.json` into
  `build/performance-dashboard/` without publishing.
- `scripts/download_performance_dashboard_artifacts.py` can download recent
  non-expired benchmark artifacts, such as `collision-benchmark-guard-*` and
  `performance-dashboard-raw-*`, into `.benchmark_seed/` so the first published
  dashboard can show trend mechanics from real prior CI JSON when those
  artifacts are still available.
- `.github/workflows/ci_ubuntu.yml` runs `Collision Benchmark Guard` only on
  scheduled and manually dispatched Linux CI when full CI is active, then
  uploads `.benchmark_results/collision_check_*.json` as a short-lived
  artifact.
- `.github/workflows/performance_dashboard.yml` is the main-push,
  scheduled/manual dashboard publisher draft: it optionally downloads recent
  real benchmark artifacts from the active branch for first-run history
  seeding, runs collision plus bounded non-collision dashboard benchmarks,
  renders `gh-pages/performance/`, uploads raw evidence artifacts, publishes
  only from `main`, and includes an optional Bencher reporting step that is
  inert unless maintainers configure
  `BENCHER_PROJECT` and `BENCHER_API_KEY`. It can render an empty dashboard
  shell so the public URL exists even if a first benchmark collection run
  produces no JSON or `.benchmark_results/` is absent, preserves the last good
  run if a later collection produces no JSON, cleans stale files from the
  target dashboard directory after loading existing history, and verifies the
  generated files before upload or publication. It writes a GitHub Actions
  summary with public dashboard links, latest run counts, and the publication
  contract so maintainers can inspect the generated payload and hosted-path
  assumptions from the workflow UI. After the publish step, it explicitly
  requests a GitHub Pages build on every `main` publisher run because the
  repository uses legacy branch-source Pages and `GITHUB_TOKEN` pushes do not
  automatically trigger a Pages build. Requesting the build even when no new
  `gh-pages` commit was needed lets a rerun repair a missed Pages deployment.
  It then retries the public `status.json` endpoint and checks that the hosted
  status, `data.json` history, dashboard HTML, and `summary.md` match the
  generated local publication intent, including freshness metadata, and fails
  if the hosted endpoint is stale.
- `scripts/check_performance_dashboard_pages_config.py` verifies the live
  GitHub Pages API configuration before publication. It expects the current
  DART site to be a usable legacy branch-source Pages site served from
  `gh-pages` at `/`, so settings drift is caught before a workflow writes
  dashboard history to an unserved branch.
- `scripts/summarize_performance_dashboard.py` reads generated `status.json`
  and appends a concise workflow summary containing the dashboard URL,
  `status.json`, `data.json`, `summary.md`, run identity, measurement counts,
  comparison counts, backend detail row counts, whether Bencher reporting was
  configured for the run, freshness and update-trigger metadata, and the
  publication contract used by the main-branch Pages publisher. It also
  renders the comparison metric contract so the workflow summary explains the
  backend matrix ratio (`backend_ns / primary_ns`) and latest comparison ratio
  (`native_ns / best_reference_ns`) using the same metadata as `status.json`,
  plus the compact `trend_summary` status counts and largest-change snapshots.
- `scripts/check_performance_dashboard_publication.py` is the local publication
  preflight. It verifies the generated dashboard directory, checks the canonical
  website plus dashboard guide, and verifies or reports the public
  `status.json`, `data.json`, dashboard HTML, and `summary.md` endpoint state.
  Its local-match mode compares all four public surfaces to the generated
  files and uses the endpoint verifier to prove hosted `data.json` still
  contains every expected comparison surface plus external competitor
  candidates, the reproducible-harness requirement, live external-competitor
  backend/surface metadata, a comparison input contract, and a comparison
  metric contract that agree with `status.json`, plus a compact
  `trend_summary` that agrees with the latest trend rows. The hosted HTML and
  `summary.md` checks also require
  service-decision metadata, publication-contract metadata, comparison input
  rules, ratio semantics, trend-summary metadata, and
  reference/competitor coverage instead of accepting a title-only page. Before
  this PR lands it reports the guide and dashboard endpoints as unpublished
  instead of hiding that launch dependency.
- `scripts/check_performance_dashboard_website_sources.py` is the source-level
  Read the Docs wiring check. It verifies that the website index visibly links
  to the dashboard guide, directly exposes the hosted dashboard URL and
  `status.json` health endpoint, includes the guide in the Community toctree,
  and that the guide names the public dashboard endpoints plus the preflight
  and live launch commands.
- `scripts/check_performance_dashboard_workflow_registration.py` is the
  post-merge workflow registration verifier. It checks that the local workflow
  still has `push`, `schedule`, and `workflow_dispatch` triggers, including a
  `main` push trigger and the dashboard-relevant path filters that make
  benchmark, source, model data, CMake, Pixi, workflow, setup-action, and
  dashboard script changes rerun the publisher. It also verifies the
  `actions: read`, `contents: write`, and `pages: write` permissions needed to
  seed artifacts, update `gh-pages`, and request a Pages build. It rejects a
  floating Bencher setup action ref so the optional external reporting path
  uses a release tag instead of `main`, checks that the workflow requests a
  Pages build on every `main` publisher run, checks that seed artifacts are
  filtered by `GITHUB_REF_NAME`, and checks that hosted endpoint verification
  still requires fresh `status.json`, `data.json`, dashboard HTML, and
  `summary.md`. It then confirms GitHub Actions has registered
  `Performance Dashboard` as an active workflow at
  `.github/workflows/performance_dashboard.yml`.
- `scripts/check_performance_dashboard_workflow_dry_run.py` is the local
  workflow-publish rehearsal. It writes `gh-pages/performance/` under a local
  dry-run directory, runs the same generator arguments used by the workflow,
  verifies the static dashboard, and checks the generated local `status.json`
  endpoint. It also verifies the generated workflow summary so the first
  Actions run cannot lose public links, service-decision metadata, or the
  publication contract without failing local preflight.
- `scripts/check_performance_dashboard_launch_preflight.py` is the consolidated
  local no-push launch gate. It verifies Pages configuration, Read the Docs
  source wiring, local workflow registration shape, seed artifact download,
  workflow-shaped dashboard rendering, and publication endpoint state from the
  generated dry-run payload. Its seed-artifact step filters to `main` so
  preflight exercises the same public-history boundary expected after merge.
  `pixi run check-bm-dashboard-launch-preflight` is the command to run before
  asking maintainers to approve the remote push and PR creation.
  `pixi run check-bm-dashboard-launch-live` is the post-merge variant; it uses
  the same checks but requires the GitHub Actions workflow and public
  Read-the-Docs/Pages endpoints to be live, and requires the public
  `latest_run_id`/SHA to match the latest successful `main` dashboard workflow
  run.
- `scripts/report_performance_to_bencher.py` owns the optional Bencher command
  construction so maintainers can dry-run the `cpp_google` reports locally
  before enabling secrets.
- `docs/readthedocs/index.rst` links to the eventual public dashboard URL under
  the project stats section.
- Live GitHub inspection on 2026-05-21 found the repository public, Pages
  enabled from the legacy `gh-pages` branch, and a scheduled
  `collision-benchmark-guard-26144293889-1` artifact from 2026-05-20 expiring
  on 2026-06-03.
- Downloading that artifact showed nine Google Benchmark JSON files with
  context metadata, repeated rows, aggregate rows, CPU details, and one-thread
  benchmark rows. The distance check reported native rows leading all current
  reference rows.

## External Options

| Option                                     | Fit for DART | Strengths                                                                                                                                                    | Risks / gaps                                                                                                                                                      | Decision                                                                                       |
| ------------------------------------------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| DART-owned GitHub Pages dashboard          | High         | Free for the public repo, already enabled, no extra account, can preserve competitor/reference ratios and DART-specific taxonomy exactly.                    | Needs a normalizer, static renderer, and a safe `gh-pages` publish workflow; GitHub-hosted runners are noisy for absolute wall-clock performance.                 | Primary dashboard and canonical data owner.                                                    |
| Bencher Cloud or Bencher Self-Hosted       | High         | Free tier for public/open-source use, supports GitHub Actions, has a `cpp_google` adapter for Google Benchmark JSON, REST API, public plots, and thresholds. | Requires project setup and an API key; built-in Google adapter tracks rows over time but does not own DART-specific strongest-reference ratio grouping by itself. | Recommended external service after maintainer approval; use beside the DART-owned dashboard.   |
| CodSpeed                                   | Medium       | Free/unlimited on public repositories, GitHub Action integration, lower-noise instrumentation, C++ Google Benchmark integration, public project examples.    | Requires CodSpeed's Google Benchmark integration or build mode; CPU-simulation results are not the same as DART's existing wall-clock reference-engine JSON.      | Pilot later for pure microbenchmarks or PR regression checks, not the first canonical backend. |
| `benchmark-action/github-action-benchmark` | Medium       | Supports Google Benchmark output and can write a Pages dashboard directly to `gh-pages`.                                                                     | Dashboard/data model is generic, history is action-owned, and DART would still need custom ratio/variance taxonomy for advanced competitor views.                 | Acceptable fallback prototype, but weaker than a DART-owned renderer.                          |
| Airspeed Velocity                          | Medium       | Open-source static benchmark history dashboard with JSON results, regression tooling, and GitHub Pages publishing support.                                   | Primarily an ASV/Python benchmark-suite workflow; DART would need an adapter or suite migration to consume existing C++ Google Benchmark JSON.                    | Keep as a future option for Python/dartpy-heavy or ASV-style suites, not the first backend.    |
| LLVM LNT                                   | Medium       | Open-source performance-tracking server with a web UI, JSON submissions, REST access, and custom test-suite support.                                         | Requires self-hosting plus adapter/schema ownership; does not directly preserve DART's current Google Benchmark JSON and native/reference/competitor schema.      | Keep as a future lab-scale performance server option, not the first dashboard.                 |
| Conbench                                   | Medium       | Open-source continuous benchmarking framework with an API and dashboard; proven by Apache Arrow.                                                             | Requires hosting and operational ownership; heavier than needed for the first DART dashboard.                                                                     | Keep as a self-hosted option if Bencher or Pages becomes limiting.                             |
| OpenBenchmarking.org / Phoronix Test Suite | Low          | Free public benchmark result storage and comparison ecosystem.                                                                                               | Best fit for Phoronix test profiles, not DART's existing Google Benchmark and PR workflow; would require a separate profile/suite layer.                          | Use only for future public ecosystem benchmark campaigns.                                      |
| Workflow Metrics / Actions cost tools      | Low          | Useful for GitHub Actions duration, reliability, and cost analytics.                                                                                         | Measures CI workflow performance, not DART physics/runtime performance.                                                                                           | Out of scope for the performance dashboard.                                                    |

## Recommended Architecture

1. **Canonical DART data store**

   Add a small normalizer that reads Google Benchmark JSON from
   `.benchmark_results/` or downloaded CI artifacts and emits a versioned DART
   performance schema. The schema should preserve raw row identity and add
   DART-owned fields: family, benchmark surface, backend/reference label,
   comparable group key, metric, aggregate, unit, run ID, branch, SHA, testbed,
   source file, and environment context.

2. **Static dashboard linked from the new website**

   Render the normalized data into a static dashboard under the existing
   `gh-pages` site at `/performance/`, making the canonical dashboard a public
   web page rather than a local artifact. Link that hosted dashboard and its
   `status.json` endpoint directly from the canonical Read the Docs homepage at
   <https://dart.readthedocs.io/en/latest/>, and keep a guide page for
   interpretation and launch instructions. The first dashboard screen should
   show the health summary and latest regressions; deeper views should show
   time-series charts, row tables, native/reference ratios,
   reference/competitor coverage, informational threshold status, variance,
   and links to source CI runs/artifacts.

3. **CI update path**

   Add a dedicated workflow after the normalizer exists:
   - `workflow_dispatch` for controlled runs;
   - path-scoped `main` push runs for dashboard-relevant code/build/benchmark
     changes;
   - scheduled runs on `main`;
   - optional path-scoped PR runs once variance and runtime cost are known;
   - raw JSON upload as short-term evidence;
   - normalized data and rendered dashboard publish to Pages as the durable
     history;
   - optional Bencher reporting from `main` only after maintainer project and
     key configuration.

4. **Bencher integration**

   After the DART-owned dashboard works locally, create a public Bencher project
   and add a maintainer-approved `BENCHER_PROJECT` variable and
   `BENCHER_API_KEY` secret. Start with base-branch scheduled/main reporting
   using the existing Google Benchmark JSON and the `cpp_google` adapter. Enable
   thresholds gradually after enough baseline samples exist. PR comments or
   check mutations should be enabled only by explicit maintainer policy.

5. **Benchmark expansion**

   Start with collision because it already has native/reference JSON. The first
   non-collision CI slice should stay bounded to common allocator,
   dynamics/kinematics, LCP smoke, math helper, SIMD add, simulation
   world-step, and compute graph rows until runtime and variance are visible on
   Pages. Add broader dynamics, LCP, math, SIMD, allocator, simulation, compute
   graph, and GPU/accelerator benchmarks as separate surfaces with explicit
   comparability rules. Do not claim competitor comparisons unless the
   competitor/reference run is reproducible in CI.

## First Implementation Phases

1. Create `docs/dev_tasks/performance_dashboard/` for multi-session handoff.
2. Add fixture verification for the normalizer/renderer using a small sample of
   current Google Benchmark JSON.
3. Add a non-publishing local command or Pixi task for rendering the dashboard.
4. Add the main-push/scheduled/manual CI workflow that produces raw JSON and
   dashboard output without making benchmark checks required.
5. Run the workflow from `main` and verify the first durable
   `gh-pages/performance/` update.
6. Configure and verify Bencher base-branch reporting after maintainer
   account/secrets approval.
7. Broaden benchmark coverage and add thresholds only after variance is
   characterized.

The initial dashboard threshold is informational only: it labels primary and
reference-backend trend rows as improvement, regression, or within-threshold
using the recorded percentage cutoff, but it does not fail workflows or PRs.
Required checks should wait until enough scheduled samples exist to quantify
GitHub-hosted runner variance.

The initial freshness policy records the path-scoped `main` push, scheduled,
and manual workflow triggers while treating the twice-weekly scheduled
publisher as the maximum expected cadence. It marks dashboard data stale after
the recorded grace window. Freshness is an endpoint health signal; it does not
infer benchmark correctness.

## Acceptance Gates

- The dashboard URL is public and linked from DART docs.
- At least one scheduled `main` run has appended durable benchmark data.
- The static dashboard can be regenerated locally from raw Google Benchmark
  JSON.
- Collision native/reference rows show time history and strongest-reference
  ratios.
- At least one non-collision benchmark family is represented or explicitly
  queued with a benchmark-surface rationale.
- CI artifacts remain raw evidence only; durable history survives artifact
  expiration.
- Any Bencher/CodSpeed/GitHub Pages mutation path is documented and approved by
  maintainers before it is made required.

## Revision Triggers

- GitHub Pages source or site ownership changes.
- A benchmark service changes free/open-source terms, API behavior, or required
  integration steps.
- Benchmark variance makes GitHub-hosted runner data unsuitable for a required
  threshold.
- A new reference/competitor harness becomes reproducible in CI.
- Dashboard data volume outgrows a static Pages-only history.

## Source Links

- GitHub Pages availability:
  <https://docs.github.com/en/pages/getting-started-with-github-pages>
- GitHub Pages publishing source:
  <https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site>
- GitHub Actions artifact retention:
  <https://docs.github.com/en/organizations/managing-organization-settings/configuring-the-retention-period-for-github-actions-artifacts-and-logs-in-your-organization>
- GitHub Actions artifacts API:
  <https://docs.github.com/en/rest/actions/artifacts>
- Bencher pricing and free/self-hosted terms source:
  <https://github.com/bencherdev/bencher/blob/main/services/console/src/pages/pricing.astro>
- Bencher GitHub Actions integration source:
  <https://github.com/bencherdev/bencher/blob/main/services/console/src/content/docs-how-to/en/github-actions.mdx>
- Bencher base-branch reporting source:
  <https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-how-to/github-actions/en/base-branch.mdx>
- Bencher Google Benchmark adapter source:
  <https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-explanation/adapters/en/cpp-google.mdx>
- CodSpeed public repository billing:
  <https://codspeed.io/docs/features/seats-and-billing>
- CodSpeed GitHub Actions integration:
  <https://codspeed.io/docs/integrations/ci/github-actions>
- CodSpeed C++ Google Benchmark guide:
  <https://codspeed.io/docs/guides/how-to-benchmark-cpp-with-google-benchmark>
- github-action-benchmark:
  <https://github.com/benchmark-action/github-action-benchmark>
- github-action-benchmark marketplace listing:
  <https://github.com/marketplace/actions/continuous-benchmark>
- Airspeed Velocity:
  <https://github.com/airspeed-velocity/asv>
- Airspeed Velocity usage docs:
  <https://asv.readthedocs.io/en/stable/using.html>
- Airspeed Velocity command docs:
  <https://asv.readthedocs.io/en/stable/commands.html>
- LLVM LNT:
  <https://llvm.org/docs/lnt/>
- LLVM LNT concepts:
  <https://llvm.org/docs/lnt/concepts.html>
- LLVM LNT importing data:
  <https://llvm.org/docs/lnt/importing_data.html>
- LLVM LNT source:
  <https://github.com/llvm/llvm-lnt>
- Conbench:
  <https://conbench.github.io/conbench/>
- Conbench source:
  <https://github.com/conbench/conbench>
- OpenBenchmarking.org:
  <https://openbenchmarking.org/>
- OpenBenchmarking.org features:
  <https://openbenchmarking.org/features>
