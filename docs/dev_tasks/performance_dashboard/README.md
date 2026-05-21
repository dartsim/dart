# Performance Dashboard - Dev Task

## Current Status

- [x] Phase 0: Evidence inventory and service recommendation documented in
      `docs/plans/080-performance-dashboard.md`.
- [x] Phase 1: Initial canonical data schema and normalizer for Google
      Benchmark JSON, with fixture verification.
- [x] Phase 2: Initial static dashboard renderer and non-publishing Pixi render
      task, including latest-run deltas and native history sparklines.
- [ ] Phase 3: Scheduled/manual CI workflow and first durable Pages update
      (workflow definition added locally; first main run pending).
- [ ] Phase 4: Maintainer-approved Bencher integration for public history and
      thresholds (optional workflow step added locally; project variable and
      API key still pending).
- [ ] Phase 5: Broaden coverage beyond collision and add stable threshold
      policy (bounded common, dynamics, LCP, math, SIMD, simulation, and
      compute graph JSON runner added locally; GPU/accelerator coverage
      queued; first CI run pending).

## Launch Requirement Audit

| Requirement                     | Current evidence                                                                                                                                           | State                                           |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------- |
| Public hosted dashboard         | Workflow target is `https://dartsim.github.io/dart/performance/`; live endpoint returns 404 until the workflow lands and publishes from `main`.            | Blocked on approved push, PR, merge, first run. |
| New website integration         | Read the Docs source links to the dashboard, `status.json`, and guide; the live root is reachable, but the new guide/link set is not published yet.        | Source ready, live site pending merge/build.    |
| Continuous updates              | `.github/workflows/performance_dashboard.yml` has path-scoped `main` push, scheduled, and manual triggers; GitHub Actions has not registered it on `main`. | Workflow ready locally, registration pending.   |
| Durable over-time history       | Workflow dry run renders `gh-pages/performance/` with seeded/current history and exact status/data/page/summary checks; no durable Pages history exists.   | Local proof ready, first main run pending.      |
| Advanced comparisons and detail | Generated `data.json` carries trend, testbed, reference/backend, external-competitor, service-decision, and publication/comparison contracts.              | Implemented and covered by dashboard tests.     |
| External service decision       | Service-source preflight verifies GitHub Pages, Bencher, CodSpeed, github-action-benchmark, Airspeed Velocity, LLVM LNT, Conbench, and OpenBenchmarking.   | Implemented; Bencher remains opt-in.            |
| Completion gate                 | `check-bm-dashboard-launch-preflight` passes; `check-bm-dashboard-launch-live` fails on missing workflow registration and missing public endpoints.        | Not complete until live gate passes.            |

Latest local verification on 2026-05-21 confirmed the full lint gate, focused
dashboard script tests, generated static payload, empty-publication smoke path,
workflow-shaped Pages dry run, actual `origin/gh-pages` first-publication
rehearsal, publication preflight, diff whitespace check, and Read the Docs
build for the published dashboard guide. This resume pass also reconfirmed the
referenced GitHub Actions tags and current service docs for GitHub Pages,
Actions artifacts, Bencher, CodSpeed, Airspeed Velocity, LLVM LNT, Conbench,
and OpenBenchmarking. A
follow-up pass fixed the branch-source Pages publication path by requesting a
Pages build on every `main` publisher run after the workflow publish step, then
added a Pages-configuration preflight and reran the focused dashboard tests and
publication checks. A later product-fit pass made deferred external
competitor/reference candidates explicit in the generated coverage table while
preserving the rule that live comparisons require reproducible dashboard JSON.
The dashboard implementation is now committed locally as one unpushed commit on
top of current `origin/main`. A short all-surface
non-collision benchmark smoke run also caught and fixed overly strict dashboard
filters that otherwise left zero-byte JSON files for common, math, and SIMD
rows. The remaining user-visible blocker is not local generation; it is
publishing `gh-pages/performance/` through the reviewed workflow so the hosted
URL stops returning 404. The generated status endpoint now also exposes
freshness so the hosted dashboard can be checked for stale data after
publication. A continuation check on 2026-05-21 reran
`pixi run check-bm-dashboard-launch-preflight` successfully, then confirmed
that `pixi run check-bm-dashboard-launch-live` still fails at workflow
registration because `.github/workflows/performance_dashboard.yml` has not
landed on `main` yet. The latest continuation reconfirmed the same live state:
Read the Docs itself is reachable, the dashboard guide and all
`https://dartsim.github.io/dart/performance/` endpoints still return 404, the
repository Pages settings still serve `gh-pages` at
`https://dartsim.github.io/dart/`, the workflow is still unregistered on
`main`, and the no-push launch preflight still renders a fresh workflow-shaped
dashboard with 197 measurements. The public Read the Docs guide now names
`pixi run check-bm-dashboard-launch-preflight` as the hosted-path rehearsal to
run before the remote publish approval boundary. The latest local pass changed
the workflow to request a GitHub Pages build on every `main` publisher run
after the publish step, so reruns can repair missed branch-source Pages
deployments even when no new `gh-pages` commit is needed; focused workflow
dry-run tests, launch preflight, diff whitespace check, and lint passed.
This continuation hardened `check-bm-dashboard-workflow-registration` so the
launch preflight also rejects missing source/CMake/data push paths, a Pages
build request gated on changed `gh-pages` output, or hosted endpoint
verification that no longer requires fresh status/data/HTML/summary surfaces;
the focused registration suite and launch preflight passed with the stronger
checks. This continuation hardened the generated-dashboard and public-endpoint
verifiers so hosted `data.json` must keep every expected comparison surface,
external competitor candidates, and the reproducible-harness requirement; the
focused endpoint/verifier suites and launch preflight passed.
This continuation added compute graph and GPU/accelerator rows to the generated
comparison-coverage catalog as queued future surfaces, kept live claims tied to
reproducible dashboard JSON, and reran the focused dashboard suites plus the
consolidated launch preflight successfully.
This continuation made each generated reference/competitor coverage row carry
the comparison target, dashboard entrypoint, and next-step metadata directly,
then taught local and hosted verifiers to reject missing comparison-scope
metadata so public `data.json` remains self-describing; the full performance
dashboard unit suite, consolidated launch preflight, and lint gate passed.
This continuation added first-class external competitor backend recognition for
reproducible Google Benchmark rows with suffixes such as `_MuJoCo`, `_Drake`,
`_Gazebo`, and `_PyBullet`; generated coverage now reports live external
competitor backends/counts when those rows exist instead of treating every
competitor engine as a deferred roadmap candidate. The full performance
dashboard unit suite and consolidated launch preflight passed with this
competitor-row fixture.
This continuation made the workflow summary report the total external
competitor rows and taught the local/hosted verifiers to reject drift between
that status count and the generated comparison-coverage rows.
This continuation added a machine-readable `comparison_input_contract` to
`data.json` and `status.json`, documented the supported external competitor
row naming rule in the Read the Docs guide, and made source/endpoint verifiers
reject missing or drifting contract metadata.
This continuation made the external-service choice machine-checkable: generated
service-decision rows now include cost/free-OSS fit, data owner, CI integration,
approval gate, and evidence URLs for the selected and evaluated services;
local/hosted verifiers reject missing service-evaluation metadata.
This continuation added a machine-readable `publication_contract` to
`data.json`, `status.json`, dashboard HTML, and `summary.md`, documenting the
GitHub Pages host, `gh-pages` source branch, `performance/` dashboard path,
publisher workflow, `refs/heads/main` publish ref, required Actions
permissions, required endpoints, canonical website dashboard/status/guide
links, launch checks, and maintainer approval boundary; local/hosted verifiers
and Read the Docs source checks reject drift.
The full performance-dashboard unit suite, consolidated launch preflight, lint
gate, and diff whitespace check passed after the change.
This continuation also made the Actions summary helper render that publication
contract and fixed its fallback `summary.md` URL when status links are missing.
This continuation tightened the workflow dry-run preflight so it now rejects a
generated Actions summary that omits public links, service-decision metadata,
the publication contract, required permissions, launch checks, or the approval
boundary.
This continuation tightened rendered-surface verification: hosted endpoint
checks and the local dashboard verifier now reject dashboard HTML or
`summary.md` that only keep the title but lose service-decision metadata,
publication-contract metadata, comparison input rules, or reference/competitor
coverage.
This continuation reran the consolidated no-push launch preflight from current
state; it passed, downloaded the latest available seed artifact, and rendered a
workflow-shaped dashboard with 197 measurements. The post-merge live gate still
fails at workflow registration, which is expected because the local
`.github/workflows/performance_dashboard.yml` has not landed on `main`.
This continuation added
`scripts/check_performance_dashboard_service_sources.py` plus
`pixi run check-bm-dashboard-service-sources`, updated the CodSpeed C++ guide
evidence URL to its current canonical path, and wired the service-source check
into launch preflight so stale or redirecting GitHub Pages, Bencher, CodSpeed,
or Conbench evidence links fail before the hosted dashboard launch. The full
performance-dashboard unit suite, service-source check, website-source check,
consolidated launch preflight, docs build, lint gate, and diff whitespace check
passed; the live guide and Pages dashboard endpoints still return 404 until the
workflow lands and publishes from `main`.
This continuation added `github-action-benchmark` as an explicit evaluated
external option in the generated service decision and docs. It remains
not-selected because DART needs custom native/reference ratios, competitor
coverage metadata, publication contracts, and exact endpoint verification that
the off-the-shelf Pages action does not own.
This continuation reconfirmed the launch boundary from current external state:
`pixi run check-bm-dashboard-launch-preflight` still passes, the service-source
evidence URLs still resolve, `https://dart.readthedocs.io/en/latest/` is live,
and `pixi run check-bm-dashboard-launch-live` still fails at workflow
registration because the local publisher has not landed on `main`.
This continuation made workflow dependency readiness repeatable by teaching the
workflow-registration preflight to resolve local composite actions and verify
all external GitHub Action tags, including nested setup action dependencies.
The focused workflow-registration suite and consolidated launch preflight
passed with those stronger checks.
This continuation also hardened the optional Bencher path: workflow
registration now rejects Bencher setup/reporting steps that are not main-only,
not `continue-on-error`, missing the `BENCHER_API_KEY` secret binding, or
missing skip flags for unconfigured/no-input runs. The focused
workflow-registration suite and consolidated launch preflight passed.
This continuation also made the workflow-registration checker use
`gh auth token` when `GITHUB_TOKEN` is unset, so local launch preflight avoids
GitHub's unauthenticated REST rate limits while remaining a read-only check.
The direct workflow-registration check and consolidated launch preflight passed
with that fallback.
This continuation reran the current launch evidence from the clean local
branch: service-source URLs, Read the Docs source wiring, local workflow
registration with `--allow-missing`, the full performance-dashboard
unit suite, and `pixi run --locked check-bm-dashboard-launch-preflight` all
passed. The live launch gate still fails at workflow registration because the
publisher is not on `main`, confirming the remaining blocker is the approved
push/PR/merge and first main-branch workflow run.
This continuation promoted the already-documented OpenBenchmarking.org /
Phoronix Test Suite option into the generated `service_decision` contract. It
is now a not-selected future public ecosystem-campaign option in generated
`data.json`, `status.json`, dashboard HTML, `summary.md`, and the endpoint
verifier, with source URLs covered by the service-source preflight.
This continuation made the compact `status.json` endpoint report live external
competitor backends, surfaces, and sample groups in addition to the row total,
then made local and hosted verifiers reject drift between that compact status
metadata and the full `data.json` comparison coverage.
This continuation verified the website-facing path from current state without
publishing: the Read the Docs source includes the dashboard guide in the
Community toctree, `pixi run --locked docs-build` succeeds with the known
dartpy autodoc stub warnings, and
`pixi run --locked check-bm-dashboard-launch-preflight` still passes while
reporting the expected unpublished guide/dashboard endpoints.
This continuation made the post-merge live launch gate report all current
launch blockers in one run. `check-bm-dashboard-launch-live` now continues past
the expected unregistered workflow, probes the RTD guide plus status/data/HTML/
summary Pages endpoints, reports missing surfaces together, and prints workflow
registration failures without a Python traceback. The full 112-test
performance-dashboard unit suite and no-push launch preflight passed with the
stronger diagnostics.
This continuation added a machine-readable `comparison_metric_contract` to
the generated dashboard schema, compact status endpoint, rendered HTML,
`summary.md`, and Actions summary so advanced native/reference and competitor
ratios carry explicit formulas and status meanings. Local/hosted verifiers and
Read the Docs source checks now reject missing or drifting ratio semantics;
the focused 66-test slice, full 115-test performance-dashboard suite, local
dashboard regeneration, consolidated no-push launch preflight, and docs build
passed with the stronger contract.
This continuation added a compact machine-readable `trend_summary` to
`data.json` and `status.json`, plus rendered Trend Summary sections in the
dashboard HTML, `summary.md`, and GitHub Actions summary. Local and hosted
verifiers now reject missing or drifting trend counts, status/threshold count
summaries, or largest improvement/regression snapshots. Focused trend tests,
the full 122-test performance-dashboard suite,
`pixi run --locked bm-dashboard && pixi run --locked check-bm-dashboard`,
`pixi run --locked check-bm-dashboard-launch-preflight`, and
`pixi run --locked docs-build` passed before the final lint/amend pass.
This continuation added a compact machine-readable `testbed_summary` to
`data.json` and `status.json`, plus rendered Testbed Summary sections in the
dashboard HTML, `summary.md`, and GitHub Actions summary. Local and hosted
verifiers now reject missing or drifting latest-run environment metadata, so
over-time performance changes can be interpreted against the latest benchmark
testbed, host, CPU count, CPU-scaling state, and ASLR state. The full
performance-dashboard unit suite now reports 154 passing tests.
This continuation hardened the external service-source verifier so it now
checks source-specific evidence text, not only HTTP reachability, for the
GitHub Pages, Bencher, CodSpeed, github-action-benchmark, Airspeed Velocity,
LLVM LNT, Conbench, and OpenBenchmarking sources behind the service decision.
It also rejects any service-decision evidence URL that lacks configured
evidence text, and added the current Bencher base-branch reporting source so
the optional `BENCHER_API_KEY` path is backed by the upstream docs. This
continuation also made the local and hosted endpoint verifiers reject
service-decision evidence URLs that drift away from that verified
service-source contract. That pass reported 158 passing tests.
This continuation made the PR handoff checker require the remote approval
boundary explicitly in the launch-ready PR body, so pushes, PR creation,
merges, workflow dispatches, and `gh-pages/performance/` publication cannot be
presented without maintainer approval. The full performance-dashboard unit
suite now reports 159 passing tests.
This continuation added compact `service_decision_summary` metadata to the
generated data/status payloads, dashboard HTML, `summary.md`, and Actions
summary so the hosted endpoint states the selected DART option directly:
GitHub Pages as the canonical dashboard/data host, with Bencher as the opt-in
external history/threshold companion after maintainer approval. The full
performance-dashboard unit suite now reports 161 passing tests.
This continuation added a Quick Start section to the public Read the Docs guide
and made the website-source preflight require the user-facing hosted-surface
sequence: dashboard HTML, `status.json`, `data.json`, `summary.md`,
competitor JSON rows, and maintainer launch checks.
This continuation made the twice-weekly dashboard schedule part of the checked
publication contract. `publication_contract` now records `schedule_crons`, the
expected update interval, and the stale-after window, while workflow
registration rejects cron drift from `30 3 * * 0,3`. The full
performance-dashboard unit suite now reports 162 passing tests.
This continuation added a read-only latest-workflow-run verifier and wired it
into the post-merge live launch gate, so `check-bm-dashboard-launch-live` now
requires the dashboard workflow to be registered, the latest `main` run to have
completed successfully, and the public Read the Docs/GitHub Pages endpoints to
be live.
This continuation also made that workflow-run verifier translate GitHub's
pre-merge HTTP 404 into an explicit "workflow runs are not available yet"
message, so the live gate names the actual approval/merge blocker instead of a
raw API failure.
This continuation added a read-only `gh-pages` dashboard-path preflight so the
launch gate now verifies the Pages source branch exists and that
`gh-pages/performance/` is either absent for first publication or already
contains dashboard files, rather than colliding with an unrelated path.
This continuation added a read-only latest GitHub Pages build preflight and
folded the expanded launch checks into the generated `publication_contract`, so
public dashboard payloads now name the Pages branch, Pages build, workflow
registration, workflow run, and live endpoint gates.
This continuation refreshed the read-only remote state from the local dashboard
branch: `origin/main` and `origin/gh-pages` were reachable, service-source and
Read the Docs source checks passed, and the post-merge live launch gate still
failed only at the expected
approval/publication boundary. Current public endpoints are unchanged:
`https://dart.readthedocs.io/en/latest/` returns 200, while the dashboard guide
and all `https://dartsim.github.io/dart/performance/` surfaces return 404 until
the workflow lands on `main`, runs successfully, and publishes
`gh-pages/performance/`.
This continuation added Airspeed Velocity to the external-service evaluation
as a not-selected static benchmark dashboard framework. The generated
`service_decision`, endpoint verifier, Read the Docs source check, PR-body
check, service-source preflight, durable plan, and public guide now keep ASV
covered alongside GitHub Pages, Bencher, CodSpeed, github-action-benchmark,
Conbench, and OpenBenchmarking.org.
This continuation added LLVM LNT to the external-service evaluation as a
not-selected self-hosted performance-tracking server option. The generated
`service_decision`, endpoint verifier, Read the Docs source check, PR-body
check, service-source preflight, durable plan, and public guide now keep LNT
covered alongside GitHub Pages, Bencher, CodSpeed, github-action-benchmark,
Airspeed Velocity, Conbench, and OpenBenchmarking.org.
It also switched the Bencher pricing, GitHub Actions, and C++ Google Benchmark
adapter evidence URLs to stable official Bencher repository sources because the
rendered `bencher.dev/docs/...` pages returned transient 52x responses from the
service-source preflight environment.
This continuation hardened the service-source preflight with bounded retries
for transient network errors and HTTP 5xx responses while keeping redirects and
non-transient evidence drift as immediate failures.
This continuation added a rendered and machine-readable
`external_competitor_status` block to generated `data.json`, `status.json`,
dashboard HTML, and `summary.md`. Local and hosted verifiers now reject drift
between that compact status and the full comparison-coverage rows, and the Read
the Docs guide explains that readers can inspect external competitor liveness
without downloading raw history.
This continuation also rendered `external_competitor_status` in the GitHub
Actions summary and tightened the workflow dry-run verifier so publisher runs
cannot drop the live/queued competitor state, live backends/surfaces/sample
groups, or live-claim rule. The full performance-dashboard unit suite now
reports 152 passing tests; local dashboard render/check, PR-body source check,
website-source check, workflow dry run, launch preflight, and docs build passed
after the latest website-source check update.
This continuation tightened the live publication verifier so the canonical
Read the Docs website must link to the hosted dashboard, `status.json`, and
dashboard guide once the dashboard is expected to be live. It also added those
same required website links to the generated `publication_contract`, dashboard
HTML, public `summary.md`, GitHub Actions summary, endpoint verifier, PR-body
checker, public guide, and durable plan. Focused publication/generation/
endpoint/summary/workflow-dry-run tests passed, the full performance-dashboard
unit suite now reports 152 passing tests, local dashboard render/check passed,
launch preflight passed with the expected pre-merge 404s, and docs build
passed with the known dartpy stub autodoc warnings.
This continuation promoted bounded simulation world-step and compute graph rows
into `bm-dashboard-surfaces`, so the scheduled publisher can collect those
surfaces as live representative dashboard data instead of only listing them as
queued coverage.
The latest continuation added a direct hosted-dashboard and `status.json` link
to the Read the Docs homepage, added a copyable hosted `status.json`/
`summary.md` terminal check to the Read the Docs guide, and made the
website-source preflight, durable plan, and PR body require both usage
surfaces. It also reconfirmed the current public state:
`https://dart.readthedocs.io/en/latest/` returns HTTP 200, while the
dashboard guide and all `https://dartsim.github.io/dart/performance/`
surfaces still return HTTP 404 until the reviewed workflow publishes from
`main`.
This continuation tightened the live launch gate so the `gh-pages` source
branch check remains permissive before first publication but requires
`gh-pages/performance/` to exist once `check-bm-dashboard-launch-live` is
proving the hosted dashboard. The live gate still aggregates the missing Pages
source path, workflow registration, workflow run, Read the Docs guide, and
hosted endpoint blockers in one run. The full performance-dashboard unit suite
now reports 170 passing tests.

## Goal

Establish a consolidated, public, continuously updated DART performance
dashboard that tracks benchmark history and detailed native/reference
comparisons over time while preserving DART-owned data and avoiding
short-lived-artifact dependence.

The canonical DART website is `https://dart.readthedocs.io/en/latest/`. The
target dashboard is the web-hosted GitHub Pages URL
`https://dartsim.github.io/dart/performance/`, linked from that Read the Docs
site. Local generation is only the verification path for the same static site
payload.

## Non-Goals

- No required PR performance gate until benchmark variance and runtime cost are
  characterized.
- No public claim against an external physics engine unless that competitor or
  reference run is reproducible in the dashboard input data.
- No replacement of the existing Google Benchmark harness for the first
  dashboard slice.
- No GitHub Pages, Bencher, or CodSpeed remote mutation without maintainer
  approval through normal PR/workflow review. The first public hosting event
  must come from the reviewed workflow running on `main` or another explicitly
  approved maintainer action.

## Key Decisions

- **DART-owned dashboard first:** link the dashboard from the new Read the Docs
  website, while using the existing public GitHub Pages branch as the generated
  dashboard host because DART needs native/reference ratio grouping and
  benchmark-family taxonomy that generic services do not own.
- **Bencher as the recommended external service:** Bencher can ingest current
  Google Benchmark JSON through its `cpp_google` adapter and offers public
  history, thresholds, REST API access, and a free/self-hosted path without
  changing DART's benchmark build.
- **CodSpeed as a later pilot:** CodSpeed is attractive for lower-noise
  microbenchmark regression checks, but its C++ path requires a CodSpeed
  Google Benchmark integration/build mode and should not be the first canonical
  source for DART's existing native/reference wall-clock JSON.
- **Artifacts are evidence, not storage:** current CI artifacts expire and
  cannot be the dashboard history.

## Current Evidence

- Local benchmark surfaces: `tests/benchmark/`, `scripts/run_cpp_benchmark.py`,
  `scripts/check_collision_benchmarks.py`, and `pixi.toml` benchmark tasks.
- Dashboard prototype:
  `pixi run bm-dashboard` renders existing `.benchmark_results/*.json` into
  normalized `data.json`, `index.html`, `summary.md`, and `status.json` under
  `build/performance-dashboard`.
- Seed artifact importer:
  `pixi run bm-dashboard-artifacts` downloads recent non-expired
  `collision-benchmark-guard-*` and `performance-dashboard-raw-*` artifacts into
  `.benchmark_seed/` when the GitHub artifact API exposes them. The generated
  dashboard can use those real JSON files to seed initial history before the
  first scheduled dashboard run creates durable `gh-pages` history. Seeded run
  IDs include the artifact/source directory so multiple artifacts from the same
  CI timestamp stay distinct. Local runs can fall back to `gh auth token` when
  `GITHUB_TOKEN` is unset, and artifact archive redirects drop the GitHub
  authorization header before following the signed storage URL. The workflow
  still provides `GITHUB_TOKEN` with `actions: read`. Main publisher runs and
  launch preflight filter seed artifacts by branch, so PR or feature-branch
  artifacts cannot seed the public dashboard history.
- Dashboard surface runner: `pixi run bm-dashboard-surfaces` emits bounded
  Google Benchmark JSON for common allocator, dynamics/kinematics, KR5/Atlas
  real robot dynamics I/O, LCP smoke, math helper, SIMD add, simulation
  world-step, and compute graph rows under `.benchmark_results/`. The runner
  validates that each selected surface writes at least one benchmark row, so a
  Google Benchmark no-match that leaves an empty output file is reported as a
  surface failure instead of silently shrinking the dashboard.
- The generated dashboard surfaces latest native/reference comparisons,
  per-surface reference-backend summaries with geometric mean ratios,
  an all-backend comparison matrix with primary/reference ratios and variance
  fields, reference/competitor coverage that distinguishes live reproducible
  references from deferred external competitor claims across collision,
  dynamics, LCP, common allocator, math, SIMD, simulation, compute graph, and
  GPU/accelerator surfaces, with each coverage row carrying its comparison
  target, benchmark entrypoint, and next step. Reproducible competitor rows
  named with supported backend suffixes such as `_MuJoCo`, `_Drake`,
  `_Gazebo`, or `_PyBullet` are parsed as live competitor/reference rows and
  counted separately from deferred candidates. The generated `data.json`,
  `status.json`, and `summary.md` publish the comparison input contract:
  Google Benchmark rows should use a shared benchmark prefix plus a
  `_<Backend>` suffix, and supported external competitor suffixes include
  Chrono, Drake, Gazebo, GzPhysics, Isaac, IsaacGym, MuJoCo, PyBullet, RaiSim,
  and Simbody. The dashboard also shows
  previous-run primary deltas, previous-run reference-backend deltas, per-row
  native/reference history sparklines, explicit informational threshold labels,
  and the recent run list. The compact status endpoint reports freshness
  relative to the scheduled publisher cadence, advertises the path-scoped
  `main` push, scheduled, and manual workflow triggers, summarizes live
  external competitor backends/surfaces/sample groups, and includes a compact
  `testbed_summary` for latest-run environment interpretation. The generated
  HTML links to `status.json`, `data.json`, `summary.md`, and the Read the Docs
  dashboard guide. The generated
  `data.json` and `status.json` files also carry
  self-describing
  public links for the canonical website, dashboard, status, data, summary, and
  guide URLs, and the endpoint verifier checks those links so a live response
  proves the dashboard is wired to the intended DART web surfaces. Empty
  zero-byte benchmark JSON files are skipped so a failed surface does not break
  dashboard generation when other valid inputs exist.
- The generated dashboard carries the service decision in `data.json`,
  `status.json`, `index.html`, `summary.md`, and the workflow summary: GitHub
  Pages is the selected DART-owned host, Bencher is the recommended external
  history/threshold service after maintainer approval, CodSpeed is a deferred
  microbenchmark pilot, `github-action-benchmark` is a not-selected
  off-the-shelf Pages action, Airspeed Velocity is a not-selected static
  benchmark dashboard framework, LLVM LNT is a not-selected self-hosted
  performance-tracking server option, Conbench is the self-hosted fallback,
  and OpenBenchmarking.org/Phoronix Test Suite is a not-selected future public
  ecosystem-campaign option. Each row now
  carries cost/free-OSS fit, data ownership, CI integration, maintainer
  approval gate, and evidence URLs. The endpoint verifier checks the expected
  service roles plus required service-evaluation metadata.
- The generated `data.json`, `status.json`, dashboard HTML, and `summary.md`
  publish the `publication_contract`, including the GitHub Pages host,
  `gh-pages` source branch, branch-source build type, `performance/` path,
  publisher workflow, `refs/heads/main` publish ref, required Actions
  permissions, required hosted endpoints, canonical website
  dashboard/status/guide links, launch checks, and maintainer approval
  boundary. Local and hosted verifiers reject contract drift so the published
  endpoint proves how the dashboard is hosted and updated.
- CI surface: `.github/workflows/ci_ubuntu.yml` has a scheduled/manual
  `Collision Benchmark Guard` job that uploads
  `.benchmark_results/collision_check_*.json`.
- Dashboard CI draft: `.github/workflows/performance_dashboard.yml` optionally
  downloads recent real benchmark artifacts from the active branch for
  first-run history seeding, runs the collision benchmarks plus the bounded
  dashboard surface runner on
  path-scoped `main` pushes, schedule, and manual dispatch, renders
  `gh-pages/performance/`, publishes only from `main`, and can report Google
  Benchmark JSON to Bencher only when maintainers configure `BENCHER_PROJECT`
  and `BENCHER_API_KEY`. The publisher allows an empty render so the public
  dashboard URL can be created even if a
  first benchmark collection run produces no JSON or `.benchmark_results/` is
  absent, preserves the last good run if a later collection produces no JSON,
  then verifies the generated files before uploading artifacts or committing to
  `gh-pages`. It writes a GitHub
  Actions summary with public dashboard links, run identity, measurement
  counts, comparison counts, backend detail row counts, and Bencher
  configuration state, including reference/competitor coverage row counts and
  threshold policy plus freshness and update-trigger metadata. Because this
  repository still uses legacy branch-source Pages, the workflow requests a
  Pages build on every `main` publisher run after the publish step; otherwise
  GitHub does not guarantee a Pages build for workflow-created `gh-pages`
  commits, and reruns cannot repair a missed deployment when the generated
  files already match. After publication, it retries the public `status.json`
  endpoint and verifies that hosted status,
  `data.json` history, dashboard HTML, and `summary.md` match the generated
  local publication intent while requiring the hosted endpoint to be fresh.
- Endpoint verifier: `scripts/check_performance_dashboard_endpoint.py` checks a
  public `status.json` URL for schema, run identity, commit SHA, and required
  dashboard file manifest. It can also require the public `data.json` endpoint
  and compare it to generated local history, including comparison coverage
  surfaces, external competitor candidates, and the reproducible-harness
  requirement, while also checking that the compact status external-competitor
  row, backend, surface, sample-group, and comparison input metadata match the
  coverage payload. Its
  `--require-dashboard-page` and `--require-summary` modes verify the hosted
  HTML and Markdown surfaces and require their launch metadata, including
  service-decision rows, the publication contract, comparison input rules, and
  reference/competitor coverage;
  `--expect-dashboard-file` and `--expect-summary-file` compare those surfaces
  to generated local files exactly. Its `--require-fresh` mode fails stale
  published dashboards. The `check-bm-dashboard-public` Pixi task uses all four
  hosted checks. The workflow uses the exact-match checks after publishing so a
  hosted dashboard failure is not mistaken for a successful local render.
- Local dashboard verifier: `scripts/verify_performance_dashboard.py` checks
  that generated `data.json` and `status.json` are internally consistent and
  carry the expected DART public URLs, selected service-decision roles,
  comparison coverage surfaces, external competitor candidates, the
  reproducible-harness requirement, external-competitor row totals, testbed
  summary metadata, comparison input contract, main-push/scheduled/manual
  update-trigger metadata, and launch metadata in the rendered HTML and
  `summary.md` before a render is treated as publishable.
- Publication preflight: `scripts/check_performance_dashboard_publication.py`
  verifies the local generated dashboard, canonical website reachability,
  dashboard guide reachability, public status, and public data history in one
  command. The
  `check-bm-dashboard-publication` Pixi task allows the public endpoint to be
  unpublished, which is expected before the first reviewed `main` workflow run.
  In that pre-merge mode it also reports the dashboard guide URL when Read the
  Docs has not built the new page yet.
  `check-bm-dashboard-publication-live` is the lower-level live endpoint
  variant that requires the Read the Docs guide and public dashboard endpoints
  to be live. The post-merge completion gate is
  `check-bm-dashboard-launch-live`, which also requires workflow registration,
  the latest successful `main` dashboard workflow run, public `latest_run_id`
  and SHA values that match that run, exact hosted status/data/HTML/summary
  matches against the generated payload, and Pages configuration.
  In live mode the publication checker aggregates canonical website-link,
  guide, status, data, HTML, and summary failures so maintainers get the full
  first-publication blocker list in one run.
- Website source preflight:
  `scripts/check_performance_dashboard_website_sources.py` verifies that the
  Read the Docs index visibly links to the dashboard guide, directly exposes
  the hosted dashboard URL and `status.json` health endpoint, includes the
  guide in the Community toctree, and that the guide names the public dashboard
  endpoints plus the no-push and live launch commands. This catches local
  website wiring regressions while the live Read the Docs guide is still
  allowed to return 404 before merge.
- Service-source preflight:
  `scripts/check_performance_dashboard_service_sources.py` verifies every
  `service_decision[*].evidence_urls` link and rejects redirects, so the
  generated GitHub Pages/Bencher/CodSpeed/github-action-benchmark/
  Airspeed-Velocity/LLVM-LNT/Conbench/OpenBenchmarking recommendation cannot
  carry stale service-source URLs into the public launch. It retries transient
  network errors and HTTP 5xx responses before failing so temporary service
  errors do not produce a traceback or mask actual evidence drift.
- Pages configuration preflight:
  `scripts/check_performance_dashboard_pages_config.py` verifies that the live
  repository Pages configuration is still the expected legacy branch-source
  site at `https://dartsim.github.io/dart/` from `gh-pages` path `/`. The
  workflow runs this on `main` before depending on that host.
- Pages branch preflight:
  `scripts/check_performance_dashboard_pages_branch.py` verifies that the
  `gh-pages` source branch exists and that `performance/` is either absent
  before first publication or already contains the expected dashboard files.
  This catches source-branch path collisions before the first approved
  publisher run. The post-merge live launch gate runs the same check in strict
  mode, so completion requires `gh-pages/performance/` to exist and contain the
  dashboard files.
- Pages build preflight:
  `scripts/check_performance_dashboard_pages_build.py` verifies that the latest
  GitHub Pages build is `built`, has no reported error, and was built from the
  current `gh-pages` branch head. This catches a missed or failed branch-source
  Pages build after the workflow pushes dashboard files.
- Workflow registration preflight:
  `scripts/check_performance_dashboard_workflow_registration.py` verifies that
  the local dashboard workflow has `push`, `schedule`, and
  `workflow_dispatch` triggers, including a `main` push trigger and the
  dashboard-relevant path filters that keep benchmark, source, model data,
  CMake, Pixi, workflow, setup-action, and dashboard script changes from
  silently skipping the publisher. It also checks the `actions: read`,
  `contents: write`, and `pages: write` permissions required for artifact
  seeding, `gh-pages` publication, and the explicit Pages build request. It
  rejects floating Bencher action refs so the optional external reporting path
  stays pinned to a release tag, verifies that the Pages build request is not
  gated on changed `gh-pages` output, verifies that seed artifacts are filtered
  by `GITHUB_REF_NAME`, and verifies that hosted endpoint checks still require
  fresh status, data, HTML, and summary surfaces. It then
  confirms GitHub Actions has registered the active `Performance Dashboard`
  workflow after the PR lands on `main`.
- Workflow dry-run preflight:
  `scripts/check_performance_dashboard_workflow_dry_run.py` renders a local
  `gh-pages/performance/` tree under ignored `build/` state, then verifies the
  generated files, local `status.json` endpoint, and workflow summary output.
  The summary check requires public links, service-decision metadata, the
  publication contract, required permissions, launch checks, and the approval
  boundary. The `check-bm-dashboard-workflow-dry-run` Pixi task is the closest
  local rehearsal for the first Pages publish without mutating GitHub.
- Launch preflight:
  `scripts/check_performance_dashboard_launch_preflight.py` is the consolidated
  no-push gate for the approval boundary. It verifies Pages configuration,
  the `gh-pages` source-branch dashboard path, the latest Pages build, local
  Read the Docs source wiring, the launch-ready PR body draft, external
  service-source evidence URLs, checks local workflow registration shape with
  `--allow-missing`, downloads `main` seed benchmark artifacts into ignored
  `build/` state when available, renders the workflow-shaped
  `gh-pages/performance/` dry run, and runs the publication preflight against
  that generated payload. Use
  `pixi run check-bm-dashboard-launch-preflight` before requesting approval to
  push or open the dashboard PR.
  `pixi run check-bm-dashboard-launch-live` is the post-merge variant that
  requires workflow registration, the latest successful `main` workflow run
  with the hosted-site dashboard artifact, public `latest_run_id` and SHA
  values that match that run, plus live Read-the-Docs and Pages endpoints.
- Workflow summary helper: `scripts/summarize_performance_dashboard.py` turns
  generated `status.json` into the Actions job summary so the first hosted run
  exposes the public dashboard, `status.json`, `data.json`, `summary.md`, and
  guide URLs, the publication contract, and the data shape maintainers need to
  inspect.
- Empty publication smoke check: `pixi run check-bm-dashboard-empty` renders,
  verifies, and endpoint-checks a temporary zero-measurement dashboard using
  the same explicit missing `.benchmark_results` input shape that the workflow
  uses. This protects the first public URL creation path when benchmark
  collection fails before writing JSON.
- Bencher reporting helper: `scripts/report_performance_to_bencher.py` expands
  benchmark JSON globs, builds `cpp_google` report commands, supports dry-runs,
  skips zero-byte benchmark JSON files, and skips cleanly when maintainer
  configuration is absent.
- Website surface: `https://dart.readthedocs.io/en/latest/` is the canonical
  DART website, and GitHub Pages is available as a generated dashboard host from
  the legacy `gh-pages` branch.
- Website link draft: `docs/readthedocs/index.rst` on the new website directly
  links to `https://dartsim.github.io/dart/performance/` and `status.json`,
  and links to the published dashboard guide under
  `docs/readthedocs/community/`. That guide explains the dashboard URL,
  status/data endpoints, freshness, native/reference comparisons, and the
  first-publication 404 state.
- Endpoint check on 2026-05-21: `https://dart.readthedocs.io/en/latest/`
  returned HTTP 200, GitHub Pages is enabled at
  `https://dartsim.github.io/dart/` from the `gh-pages` branch, and
  `https://dartsim.github.io/dart/performance/` plus
  `https://dartsim.github.io/dart/performance/status.json` returned HTTP 404
  because no `gh-pages/performance/` publication has landed yet.
- Launch check on 2026-05-21: the no-push launch preflight passed with the
  current branch, while `pixi run --locked check-bm-dashboard-launch-live`
  failed at workflow registration because `Performance Dashboard` is not
  registered on `main` yet. This confirms the remaining launch blocker is the
  reviewed remote publish path, not local dashboard generation.
- First-publication rehearsal on 2026-05-21 fetched the current
  `origin/gh-pages` ref, confirmed it has zero `performance/` files, rendered
  the dashboard into an ignored copy of that branch, and verified the generated
  local endpoint with freshness required. The rehearsal produced 193
  measurements, six backend summary rows, nine comparison coverage rows, and
  the expected `index.html`, `data.json`, `summary.md`, and `status.json`
  files. This confirms the remaining blocker is the approved remote publish,
  not local `gh-pages` content shape.
- GitHub's current `GITHUB_TOKEN` docs state that workflow-created commits do
  not trigger a GitHub Pages build. The workflow now grants `pages: write`,
  records whether the `gh-pages` push changed anything, requests
  `/repos/${GITHUB_REPOSITORY}/pages/builds` on every `main` publisher run
  after the publish step, and gives the hosted endpoint up to 15 minutes to
  become fresh.
- Live Pages API check on 2026-05-21 still reports `build_type: legacy`,
  `status: built`, `html_url: https://dartsim.github.io/dart/`, and source
  `gh-pages` at `/`; the latest Pages builds are for commits on `gh-pages`.
- The generated reference/competitor coverage table now includes deferred
  candidates such as MuJoCo, Drake, Gazebo/gz-physics, PyBullet, Simbody, and
  solver/allocator baselines per surface. These are explicitly shown as harness
  candidates, not live performance claims, until reproducible benchmark JSON is
  available.
- The public GitHub Actions artifact REST endpoint can list repository
  artifacts, but artifact archive downloads require authenticated access and
  redirect to signed storage URLs. The importer now uses `GITHUB_TOKEN` or
  local `gh auth token` and drops the GitHub authorization header on
  cross-host redirects; the dashboard workflow therefore keeps explicit
  `actions: read` permission for its `GITHUB_TOKEN`.
- Live artifact sample: CI run `26144293889` produced
  `collision-benchmark-guard-26144293889-1` on 2026-05-20 with nine Google
  Benchmark JSON files and expiration on 2026-06-03. A local seed preflight on
  2026-05-21 downloaded that artifact into ignored
  `build/performance-dashboard-seed-check/` state.
- Downloaded sample files are currently available in ignored local
  `.benchmark_results/` state for non-publishing dashboard checks.
- A short local all-surface smoke run built and executed the common allocator,
  kinematics, `dynamics_cache_io`, LCP smoke, math helper, SIMD, simulation
  world-step, and compute graph dashboard slices. It produced valid ignored JSON
  for all eight dashboard surface files:
  32 common rows, four kinematics rows, four KR5/Atlas real-robot dynamics
  rows, one LCP row, 16 math rows, eight SIMD rows, two simulation rows, and
  two compute graph rows. The generated local dashboard now reports collision
  plus all wired non-collision slices as live surfaces.

## Immediate Next Steps

1. Get explicit maintainer approval to push the local branch and open the
   dashboard publisher PR, using `docs/dev_tasks/performance_dashboard/PR_BODY.md`
   as the launch-ready PR body draft.
2. Review and merge the main-push/scheduled/manual CI workflow.
3. Run `pixi run check-bm-dashboard-workflow-registration` after the merge to
   confirm GitHub Actions has registered the workflow as active on `main`.
4. Run `pixi run check-bm-dashboard-pages-branch` to confirm
   `gh-pages/performance/` is absent or already dashboard-owned before the
   first publisher run. The later live launch gate requires that path to exist.
5. Run `pixi run check-bm-dashboard-pages-build` to confirm the current
   `gh-pages` source commit has a successful Pages build.
6. Run the workflow once from `main` through the GitHub Actions UI or
   `gh workflow run performance_dashboard.yml --ref main` so
   `https://dartsim.github.io/dart/performance/` is actually hosted.
7. Run `pixi run check-bm-dashboard-workflow-run` after the workflow
   completes to confirm the latest `main` publisher succeeded and uploaded the
   hosted-site dashboard artifact, then run
   `pixi run check-bm-dashboard-launch-live` to confirm
   `gh-pages/performance/status.json` is reachable and that
   `gh-pages/performance/data.json` preserves durable history instead of
   relying on expiring GitHub Actions artifacts.
8. Inspect the first non-collision rows for runtime/variance before broadening
   dashboard filters or adding thresholds.
9. Configure maintainer-approved `BENCHER_PROJECT` and `BENCHER_API_KEY`, then
   confirm Bencher receives the base-branch benchmark reports without enabling
   PR comments or required checks.

## Completion Criteria

- `docs/plans/080-performance-dashboard.md` remains the durable plan owner.
- The dashboard can be regenerated locally from raw benchmark JSON.
- The public Pages dashboard updates from at least one CI run.
- Collision native/reference comparisons show history and strongest-reference
  ratios.
- A Bencher integration path is either live with maintainer-approved secrets or
  explicitly deferred with a blocker.
- This dev-task folder is deleted in the completing change after durable
  implementation docs and dashboard links are promoted.

## Verification

Plan/docs-only updates:

```bash
pixi run lint-md
pixi run check-lint-md
pixi run check-docs-policy
pixi run check-lint-spell
```

Implementation updates should add focused script tests or fixture checks, then
run:

```bash
pixi run --locked pytest -q python/tests/unit/test_check_performance_dashboard_endpoint.py python/tests/unit/test_check_performance_dashboard_publication.py python/tests/unit/test_check_performance_dashboard_workflow_dry_run.py python/tests/unit/test_check_performance_dashboard_workflow_registration.py python/tests/unit/test_download_performance_dashboard_artifacts.py python/tests/unit/test_generate_performance_dashboard.py python/tests/unit/test_report_performance_to_bencher.py python/tests/unit/test_run_cpp_benchmark.py python/tests/unit/test_run_performance_dashboard_benchmarks.py python/tests/unit/test_summarize_performance_dashboard.py python/tests/unit/test_verify_performance_dashboard.py
pixi run bm-dashboard-artifacts
pixi run --locked python scripts/check_performance_dashboard_workflow_registration.py --allow-missing
pixi run --locked python scripts/run_performance_dashboard_benchmarks.py --dry-run
pixi run --locked python scripts/run_performance_dashboard_benchmarks.py --benchmark-min-time=1ms --benchmark-repetitions=1 --continue-on-error
pixi run bm-dashboard
pixi run check-bm-dashboard
pixi run check-bm-dashboard-pr-body
pixi run check-bm-dashboard-empty
pixi run check-bm-dashboard-workflow-dry-run
pixi run check-bm-dashboard-publication
pixi run check-lint-py
pixi run check-lint-yaml
pixi run check-lint-rst
pixi run check-docs-policy
pixi run check-lint-spell
pixi run lint-toml
pixi run check-lint-md
pixi run lint
pixi run docs-build
pixi run bm-collision-check
```

Use the stronger build/test gates from `docs/ai/verification.md` if any C++,
Python binding, or CI workflow behavior changes.

## Related

- `docs/dev_tasks/performance_dashboard/PR_BODY.md` - launch-ready PR body
  draft matching `.github/PULL_REQUEST_TEMPLATE.md`.
- `docs/plans/080-performance-dashboard.md` - durable option analysis and
  selected architecture.
- `docs/plans/035-native-collision-dashboard.md` - existing
  native-collision feature/performance dashboard.
- `docs/plans/035-native-collision/coverage-matrix.md` - current
  native-collision coverage taxonomy and benchmark row map.
- `docs/onboarding/ci-cd.md` - CI workflow architecture and artifact handling.
