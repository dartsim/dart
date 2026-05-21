# Resume: Performance Dashboard

## Last Session Summary

Created the durable performance dashboard plan, active dev-task handoff,
dashboard generator/verifier, bounded dashboard benchmark runner, optional
Bencher reporter, real-artifact history seed importer, public endpoint
verifier, workflow summary helper, and main-push/scheduled/manual GitHub Pages publisher
draft. Focused script tests, local dashboard generation, the empty-publication
smoke path, workflow-shaped Pages dry run, publication preflight, full lint,
docs build, live endpoint checks, and a short all-surface non-collision
benchmark smoke run confirmed the expected state on 2026-05-21: the generated
dashboard is ready for review, but the hosted dashboard URL still returns 404
until `gh-pages/performance/` is published. Later local updates added explicit
reference/competitor coverage rows, informational threshold metadata,
freshness metadata, explicit update-trigger metadata, the publication preflight
script, direct generated-dashboard resource links, zero-byte benchmark JSON
skipping, real robot dynamics I/O rows, broadened dashboard-surface filters,
no-row output validation, and a Read the Docs performance dashboard guide so
the generated dashboard distinguishes live reproducible DART reference
comparisons from deferred external competitor claims and can report stale
published data.
This resume pass reran the focused dashboard tests, workflow dry run,
publication preflight, local render checks, empty-publication smoke check, diff
whitespace check, full lint gate, and Read the Docs build; all passed. It also
reconfirmed the official service docs and the referenced GitHub Actions tags.
A later no-push rehearsal fetched the current `origin/gh-pages` branch,
confirmed it has no existing `performance/` files, rendered
`gh-pages/performance/` into ignored build state, and verified the generated
fresh `status.json` endpoint with 197 measurements.
A follow-up pass fixed the branch-source Pages build trigger by adding
`pages: write`, requesting a Pages build after the publish step on every
`main` publisher run, and extending hosted endpoint verification to 15
minutes. It also added a Pages configuration preflight so the workflow confirms
the live repository settings still point at `gh-pages` before publishing
dashboard history there.
A later product-fit pass added explicit deferred competitor/reference
candidates to the generated coverage table while keeping the live-comparison
rule tied to reproducible dashboard JSON.
The latest local pass made the generated `data.json` and `status.json`
self-describing with canonical Read the Docs, dashboard, status, data, summary,
and guide URLs, then taught the endpoint and dashboard verifiers to reject
missing or mismatched public links. It reran the focused dashboard suite, the
workflow dry run, local dashboard regeneration, publication preflight, Pages
configuration preflight, workflow-registration preflight, lint, and docs build.
A follow-up local pass made the generated dashboard artifacts carry the service
decision as first-class data: GitHub Pages is the selected DART-owned host,
Bencher is the recommended external history/threshold companion after
maintainer approval, CodSpeed is a deferred microbenchmark pilot, Airspeed
Velocity is a not-selected static benchmark dashboard framework, LLVM LNT is a
not-selected self-hosted performance-tracking server option, Conbench is the
self-hosted fallback, and OpenBenchmarking.org/Phoronix Test Suite is a
not-selected future public ecosystem-campaign option. The dashboard,
status/data endpoints, workflow summary, local verifier, endpoint verifier, and
focused unit tests now check that decision path.
A later no-push launch-readiness pass added
`scripts/check_performance_dashboard_launch_preflight.py` and
`pixi run check-bm-dashboard-launch-preflight` to consolidate the Pages
configuration check, local workflow registration shape, seed artifact download,
workflow-shaped dashboard dry run, and publication endpoint preflight.
A continuation pass on 2026-05-21 reran that launch preflight successfully,
then ran `pixi run check-bm-dashboard-launch-live` and confirmed it still fails
at workflow registration because the dashboard workflow is not present on
`main` yet. The branch remained clean and no PR existed for
`task/performance-dashboard`.
A later continuation on 2026-05-21 reconfirmed the live external state:
`https://dart.readthedocs.io/en/latest/` returned HTTP 200, the new dashboard
guide and all `https://dartsim.github.io/dart/performance/` endpoints returned
HTTP 404, GitHub Pages still serves the legacy `gh-pages` branch, and GitHub
Actions still has no registered `Performance Dashboard` workflow on `main`.
`pixi run check-bm-dashboard-launch-preflight` still passed and rendered the
workflow-shaped dashboard with 197 measurements. The public dashboard guide now
also names `pixi run check-bm-dashboard-launch-preflight` as the no-push
hosted-path rehearsal before the approval boundary.
A subsequent local hardening pass pinned the optional Bencher setup action to
`bencherdev/bencher@v0.6.6` after checking the current Bencher release tags,
then taught `scripts/check_performance_dashboard_workflow_registration.py` to
reject floating Bencher action refs such as `bencherdev/bencher@main`.
The latest local pass added
`scripts/check_performance_dashboard_website_sources.py` plus
`pixi run check-bm-dashboard-website-sources`, then wired it into the launch
preflight so pre-merge checks verify the Read the Docs source index links to
the dashboard guide, includes it in the Community toctree, and keeps the guide
launch commands and public endpoint URLs intact even while the live guide is
still allowed to return 404 before merge.
A follow-up hardening pass taught
`scripts/check_performance_dashboard_workflow_registration.py` to verify the
dashboard-relevant `main` push path filters, so future edits cannot silently
drop source, model data, CMake, script, Pixi, benchmark, workflow, or
setup-action paths and leave the hosted dashboard stale after relevant changes.
The latest local pass made the workflow request a GitHub Pages build on every
`main` dashboard run after the publish step, rather than only after a changed
`gh-pages` commit, so a rerun can repair a missed branch-source Pages
deployment. It also updated the workflow dry-run test and handoff docs for that
behavior, then reran the focused workflow dry-run tests, launch preflight, diff
whitespace check, and full lint gate successfully.
A subsequent local hardening pass moved those launch-sensitive publication
rules into the workflow-registration verifier itself: local preflight now also
checks the source/CMake/data path filters, the unconditional main-only Pages
build request, and the hosted endpoint verification flags that require fresh
status, data, dashboard HTML, and summary surfaces. The focused workflow
registration suite and consolidated launch preflight passed with those stronger
checks.
A later continuation hardened the generated-dashboard and public-endpoint
verifiers so a hosted `data.json` payload must keep every expected comparison
surface, external competitor candidates, and the reproducible-harness
requirement. The focused endpoint/verifier suites and consolidated launch
preflight passed with those stronger checks.
The latest local pass added the public `summary.md` link to the GitHub Actions
summary helper so maintainers can inspect all four hosted dashboard surfaces
from the first publisher run summary.
This continuation promoted compute graph and GPU/accelerator into the generated
comparison-coverage catalog as queued future surfaces, updated tests and docs
for nine comparison coverage rows, and reran the focused dashboard suites plus
the consolidated launch preflight successfully.
This continuation also made reference/competitor coverage rows carry the
comparison target, benchmark entrypoint, and next-step metadata directly in
`data.json`, HTML, and `summary.md`, with local and hosted verifiers rejecting
missing comparison-scope metadata. The full performance-dashboard unit suite,
consolidated launch preflight, and lint gate passed after the change.
This continuation added real external competitor backend recognition for
reproducible Google Benchmark rows named with suffixes such as `_MuJoCo`,
`_Drake`, `_Gazebo`, and `_PyBullet`. Generated coverage now reports live
external competitor backends/counts when those rows exist, while still keeping
candidate-only engines clearly deferred. The full performance-dashboard unit
suite and consolidated launch preflight passed with this competitor-row
fixture.
This continuation made external competitor row totals visible in the workflow
summary and made local/hosted verifiers reject status/data drift for that
count.
This continuation later expanded that compact status metadata to include live
external competitor backends, surfaces, and sample groups, and made
local/hosted verifiers reject drift between those status fields and the full
`data.json` comparison coverage.
This continuation added a machine-readable `comparison_input_contract` to the
generated status/data artifacts, documented the exact external competitor row
naming rule in the Read the Docs guide, and made source plus endpoint checks
reject missing or drifting contract metadata.
This continuation made the external-service recommendation machine-checkable:
generated service-decision rows now include cost/free-OSS fit, data owner, CI
integration, approval gate, and source URLs for Pages, Bencher, CodSpeed, and
Conbench, with verifier coverage for those fields.
This continuation added machine-readable `publication_contract` metadata to
`data.json`, `status.json`, dashboard HTML, and `summary.md`. The contract
records the GitHub Pages host, `gh-pages` source branch, branch-source build
type, `performance/` path, publisher workflow, `refs/heads/main` publish ref,
required Actions permissions, required endpoints, canonical website
dashboard/status/guide links, launch checks, and the maintainer approval
boundary; local/hosted verifiers and Read the Docs source checks now reject
drift. The full performance-dashboard unit suite,
consolidated launch preflight, lint gate, and diff whitespace check passed
after the change.
This continuation also made `scripts/summarize_performance_dashboard.py` render
that publication contract into the Actions summary and fixed its default
`summary.md` URL fallback for status payloads without links.
This continuation tightened the workflow dry-run preflight so it now rejects a
generated Actions summary that omits public links, service-decision metadata,
the publication contract, required permissions, launch checks, or the approval
boundary.
This continuation tightened rendered-surface verification so both hosted
endpoint checks and the local dashboard verifier reject dashboard HTML or
`summary.md` that keep only the title while dropping service-decision metadata,
publication-contract metadata, comparison input rules, or reference/competitor
coverage.
This continuation reran the no-push launch preflight from current state; it
passed, downloaded the current seed artifact, and rendered a workflow-shaped
dashboard with 197 measurements. The post-merge live launch gate still fails
at workflow registration, which is the expected pre-merge state because
`.github/workflows/performance_dashboard.yml` has not landed on `main`.
This continuation added a service-source URL preflight for the generated
GitHub Pages/Bencher/CodSpeed/github-action-benchmark/Airspeed-Velocity/
Conbench/OpenBenchmarking recommendation, updated the CodSpeed C++ Google
Benchmark guide evidence URL to the current canonical path, and wired that check into
launch preflight so stale or redirecting external evidence links fail before
launch. The focused service-source tests, full performance-dashboard unit
suite, service-source check, website-source check, consolidated launch
preflight, docs build, lint gate, and diff whitespace
check passed; the live guide and Pages dashboard endpoints still return 404
until publication from `main`.
This continuation added `github-action-benchmark` as an explicit evaluated
external option in the generated service decision and docs. It remains
not-selected because DART needs custom native/reference ratios, competitor
coverage metadata, publication contracts, and exact endpoint verification that
the off-the-shelf Pages action does not own.
This continuation reconfirmed the launch boundary from current external state:
the no-push launch preflight still passes, service-source evidence URLs still
resolve, `https://dart.readthedocs.io/en/latest/` is live, the dashboard guide
and GitHub Pages dashboard endpoints still return 404, and the post-merge live
gate still fails at workflow registration because the local publisher has not
landed on `main`.
This continuation made workflow dependency readiness repeatable by teaching the
workflow-registration preflight to resolve local composite actions and verify
all external GitHub Action tags, including nested setup action dependencies.
The focused workflow-registration suite, direct workflow-registration check,
and consolidated launch preflight passed with those stronger checks.
This continuation also hardened the optional Bencher path: workflow
registration now rejects Bencher setup/reporting steps that are not main-only,
not `continue-on-error`, missing the `BENCHER_API_KEY` secret binding, or
missing skip flags for unconfigured/no-input runs. The focused
workflow-registration suite, direct workflow-registration check, and
consolidated launch preflight passed.
This continuation also made the workflow-registration checker use
`gh auth token` when `GITHUB_TOKEN` is unset, so local launch preflight avoids
GitHub's unauthenticated REST rate limits while remaining a read-only check.
The direct workflow-registration check and consolidated launch preflight passed
with that fallback.
This continuation added a tracked launch PR body draft at
`docs/dev_tasks/performance_dashboard/PR_BODY.md` so the approval handoff
matches the current launch hardening, service decision, comparison metric
contract, trend summary, and 122-test performance-dashboard suite.
This continuation added `scripts/check_performance_dashboard_pr_body.py` plus
`pixi run check-bm-dashboard-pr-body`, then wired that check into launch
preflight so the tracked PR body must keep the template sections, launch
commands, hosted endpoint notes, current test evidence, and service/competitor
metadata before maintainers approve a remote push.
This continuation reran current launch-readiness checks from the clean local
branch. Service-source URLs, Read the Docs source wiring, local workflow
registration with `--allow-missing`, the full performance-dashboard
unit suite, and `pixi run --locked check-bm-dashboard-launch-preflight` all
passed. `pixi run --locked check-bm-dashboard-launch-live` still fails at
workflow registration, which is expected before the publisher lands on `main`.
This continuation promoted the OpenBenchmarking.org / Phoronix Test Suite
option from the durable service matrix into the generated `service_decision`
contract as a not-selected future public ecosystem-campaign option. The
generated data/status/HTML/summary surfaces and endpoint verifier now check it,
and the service-source preflight covers its current official source URLs.
This continuation verified the website-facing path without publishing:
`pixi run --locked docs-build` succeeded with the known dartpy autodoc stub
warnings, the Read the Docs source/build includes
`community/performance_dashboard` in navigation, and
`pixi run --locked check-bm-dashboard-launch-preflight` passed while still
reporting the expected unpublished RTD guide and Pages dashboard endpoints.
This continuation also made the post-merge live launch gate report all current
launch blockers in one run. `pixi run --locked check-bm-dashboard-launch-live`
now continues after the expected unregistered workflow, checks the RTD guide
and all Pages dashboard surfaces, reports every missing endpoint together, and
prints the workflow-registration failure without a Python traceback. The full
112-test performance-dashboard unit suite and no-push launch preflight passed
with the stronger diagnostics.
This continuation added a generated `comparison_metric_contract` for advanced
native/reference and competitor comparisons. The contract records that backend
matrix rows use `backend_vs_primary_ratio = backend_ns / primary_ns`, latest
native/reference rows use `ratio = native_ns / best_reference_ns`, and ratio
interpretation/status meanings are published in `data.json`, `status.json`,
HTML, `summary.md`, and the Actions summary. Focused endpoint/verifier/summary
tests, the full 115-test performance-dashboard suite,
`pixi run --locked bm-dashboard && pixi run --locked check-bm-dashboard`,
`pixi run --locked check-bm-dashboard-launch-preflight`, and
`pixi run --locked docs-build` passed; the launch preflight still reports the
expected unpublished RTD guide and GitHub Pages dashboard endpoints.
This continuation added a compact `trend_summary` to `data.json` and
`status.json`, then rendered it in dashboard HTML, `summary.md`, and the
Actions summary. It reports latest run ID, history run count, primary and
reference trend row counts, status/threshold count summaries, and largest
improvement/regression snapshots so maintainers can inspect performance change
over time without downloading full trend rows. Focused trend tests, the full
122-test performance-dashboard suite,
`pixi run --locked bm-dashboard && pixi run --locked check-bm-dashboard`,
`pixi run --locked check-bm-dashboard-launch-preflight`, and
`pixi run --locked docs-build` passed before the final lint/amend pass.
This continuation added `scripts/check_performance_dashboard_workflow_run.py`
and `pixi run check-bm-dashboard-workflow-run`, then wired that read-only check
into the post-merge live launch gate so the hosted dashboard is not considered
complete until the latest `main` dashboard workflow run has completed
successfully.
This continuation also made that checker report GitHub's pre-merge HTTP 404 as
"workflow runs are not available yet", clarifying that the workflow has not
landed on `main` or been registered rather than exposing a raw REST failure.
This continuation added `scripts/check_performance_dashboard_pages_branch.py`
and `pixi run check-bm-dashboard-pages-branch` so launch preflight verifies the
`gh-pages` source branch and dashboard path before the first approved
publication.
This continuation added `scripts/check_performance_dashboard_pages_build.py`
and `pixi run check-bm-dashboard-pages-build`, then expanded the generated
`publication_contract` launch checks so public dashboard payloads advertise the
Pages branch, Pages build, workflow registration, workflow run, and live
endpoint gates.
This continuation refreshed remote refs and launch evidence from the local
dashboard commit: `origin/main` was `d8f198b63a9`, `origin/gh-pages` was
`f60a366857a4`, service-source and Read the Docs source checks passed, no PR
existed for `task/performance-dashboard`, and the post-merge live launch gate
still failed only because the workflow is not registered on `main` and the RTD
guide plus Pages dashboard endpoints are unpublished.
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
This continuation added generated `external_competitor_status` metadata and a
rendered External Competitor Status section to `data.json`, `status.json`,
dashboard HTML, `summary.md`, and the GitHub Actions summary. The status is
derived from `comparison_coverage`, carries live/queued state, live
backends/surfaces/sample groups, supported external backend names, and the
live-claim rule, and local plus hosted verifiers and the workflow dry-run
verifier now reject drift. The full performance-dashboard unit suite now
reports 151 passed; local dashboard render/check, PR-body check,
website-source check, workflow dry run, launch preflight, and docs build
passed after the latest website-source check update.
The latest continuation exposes the required hosted-site workflow artifact in
the generated public `summary.md` and GitHub Actions summary publication
contract, and the workflow dry-run plus endpoint verifiers now reject summaries
that omit `performance-dashboard-site-<run_id>-<run_attempt>`.
This continuation added `--clean-output` to the dashboard generator and
publisher so existing `data.json` history is loaded before stale files under
`gh-pages/performance/` are removed and the current static payload is written.
The workflow registration check now requires that flag.
This continuation added a direct hosted-dashboard and `status.json` link to
the Read the Docs homepage, added a copyable hosted terminal check for
`status.json` and `summary.md` to the Read the Docs guide, and made the
website-source preflight, durable plan, and PR body require those usage
surfaces so future edits cannot leave readers with only raw URLs.
This continuation tightened live publication verification so the canonical
Read the Docs website must link to the hosted dashboard, `status.json`, and
dashboard guide once publication is expected. The generated
`publication_contract` now carries those required website links in
`data.json`, `status.json`, dashboard HTML, public `summary.md`, and the
GitHub Actions summary. Focused publication/generation/endpoint/summary/
workflow-dry-run tests passed, the full performance-dashboard unit suite now
reports 152 passing tests, local dashboard render/check passed, launch
preflight passed with the expected pre-merge 404s, and docs build passed with
the known dartpy stub autodoc warnings.
This continuation promoted bounded simulation world-step and compute graph rows
into `bm-dashboard-surfaces`, so the scheduled publisher can collect those
surfaces as live representative dashboard data instead of only listing them as
queued coverage.
This continuation added a compact machine-readable `testbed_summary` to
`data.json` and `status.json`, rendered it in dashboard HTML, public
`summary.md`, and the GitHub Actions summary, and tightened local/hosted
verifiers so over-time performance changes retain latest-run testbed, host,
CPU count, CPU-scaling, and ASLR context. The full performance-dashboard unit
suite now reports 154 passing tests.
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
so users can pick the right hosted surface first: dashboard HTML for
inspection, `status.json` for health, `data.json` for scripts, `summary.md`
for compact reports, competitor JSON rows for advanced comparisons, and
maintainer launch checks for first publication. The website-source preflight
now requires that guidance.
This continuation made the workflow schedule part of the publication contract:
generated payloads now record `schedule_crons`, the expected update interval,
and the stale-after window, and workflow registration rejects cron drift from
`30 3 * * 0,3`. The full performance-dashboard unit suite now reports 162
passing tests.
This continuation hardened the live launch gate so the latest successful
`main` dashboard workflow run writes a machine-readable run identity, and
`check-bm-dashboard-launch-live` requires the public `status.json`
`latest_run_id` and source SHA to match that run before treating the hosted
dashboard as live. The full performance-dashboard unit suite now reports 165
passing tests.
This continuation made `scripts/check_performance_dashboard_publication.py`
aggregate canonical website-link failures with guide/status/data/page/summary
endpoint failures in live mode, so a first-publication check reports all public
launch blockers instead of stopping at the first missing Read the Docs link.
The full performance-dashboard unit suite now reports 166 passing tests.
This continuation tightened workflow-registration verification so the publisher
must keep exact hosted `status.json`, `data.json`, dashboard HTML, and
`summary.md` checks, plus the Pages propagation retry budget, before the hosted
dashboard can be treated as verified. The full performance-dashboard unit suite
now reports 167 passing tests.
This continuation hardened first-run artifact seeding so the downloader,
workflow, launch preflight, Pixi task, workflow-registration check, PR body,
Read the Docs guide, and durable plan all require branch-filtered seed
artifacts. Public `main` dashboard history cannot be seeded from PR or
feature-branch artifacts.
The focused branch-filter tests, full performance-dashboard unit suite, no-push
launch preflight, docs build, lint gate, and diff whitespace check passed. The
post-merge live launch gate still fails for the expected external blockers:
the workflow is not registered on `main`, the live Read the Docs guide and
canonical website links have not been published yet, and
`gh-pages/performance/` still returns 404.
This continuation added a concise launch-requirement audit to the dev-task
tracker so the remaining remote/publication blockers stay explicit: local
readiness is not the same as the requested hosted dashboard being live.
This continuation refreshed the no-push launch evidence: service-source URLs,
Read the Docs source wiring, local workflow registration with `--allow-missing`,
and `check-bm-dashboard-launch-preflight` all passed; the dry run rendered 197
measurements. The live launch gate still fails only on the expected unpublished
state: no registered `Performance Dashboard` workflow on `main`, no latest
workflow run, no live Read the Docs guide links, and 404 responses for the
Pages dashboard HTML, `status.json`, `data.json`, and `summary.md`.
This continuation refreshed the dashboard contract tests and local render:
the full performance-dashboard unit slice reported 169 passing tests, and
`pixi run --locked bm-dashboard && pixi run --locked check-bm-dashboard`
renders and verifies 197 local measurements.
This continuation rechecked the launch boundary from the clean
`task/performance-dashboard` branch: no PR exists for the branch, GitHub
Actions still has no registered `Performance Dashboard` workflow on `main`,
`https://dart.readthedocs.io/en/latest/` returns HTTP 200, the Read the Docs
dashboard guide returns HTTP 404, and the hosted
`https://dartsim.github.io/dart/performance/` HTML, `status.json`,
`data.json`, and `summary.md` endpoints all return HTTP 404.
`pixi run --locked check-bm-dashboard-launch-preflight` passed again,
downloaded `collision-benchmark-guard-26144293889-1`, and rendered the
workflow-shaped dashboard with 197 measurements. `pixi run --locked
check-bm-dashboard-launch-live` still fails only on the expected unpublished
state: missing workflow registration, missing latest workflow run, missing
Read the Docs guide links, and missing Pages dashboard endpoints.
This continuation tightened the post-merge live launch gate so the Pages
branch check accepts an absent `gh-pages/performance/` path before first
publication but requires that path to exist once
`check-bm-dashboard-launch-live` is proving the dashboard is hosted. The live
gate still aggregates the Pages source-path, workflow-registration,
latest-workflow-run, Read the Docs guide, and Pages endpoint blockers in one
run. The full performance-dashboard unit suite now reports 170 passing tests.
This continuation also made the tracked PR body and PR-body verifier name that
strict live-path requirement explicitly, so the approval handoff cannot omit
the fact that no-push preflight may accept an absent `gh-pages/performance/`
path but the post-publication live gate must require it.

## Current Branch

`task/performance-dashboard` - one local unpushed commit on top of
`origin/main`, with the dashboard generator, fixture tests, Pixi render/surface
tasks, dashboard workflow draft, optional Bencher helper, and planning docs
committed locally. The comparison metric contract, compact `trend_summary`,
compact `testbed_summary`, external competitor status, latest-workflow-run
checks, Pages source/build checks, publication contract, Actions-summary
artifact contract, clean-output stale-file protection, hosted usage-guide
checks, public Quick Start guidance, checked schedule-cadence metadata,
canonical website-link verifier, branch-filtered artifact seeding, and bounded
simulation/compute surface rows are folded into that same local commit after
the latest amend.

## Immediate Next Step

Get explicit maintainer approval to push the local branch and open the
dashboard publisher PR, then review/merge the main-push/scheduled/manual CI
workflow, confirm it is registered with
`pixi run check-bm-dashboard-workflow-registration`, confirm the `gh-pages`
source path with `pixi run check-bm-dashboard-pages-branch`, confirm the
current Pages build with `pixi run check-bm-dashboard-pages-build`, run it once
from `main`, either from the GitHub Actions UI or
`gh workflow run performance_dashboard.yml --ref main`, confirm the latest
`main` workflow run succeeded and uploaded the hosted-site artifact with
`pixi run check-bm-dashboard-workflow-run`, confirm
`https://dartsim.github.io/dart/performance/` is hosted with
`pixi run check-bm-dashboard-launch-live`, `gh-pages/performance/status.json`,
`data.json`, `summary.md`, and the dashboard HTML reachable, then inspect the
first common/dynamics/LCP/math/SIMD/simulation/compute surface rows for runtime
and variance, plus the queued GPU coverage row for expected next-step labels.

## Context That Would Be Lost

- GitHub Pages is already enabled at `https://dartsim.github.io/dart/` from
  the legacy `gh-pages` branch.
- The canonical DART website is `https://dart.readthedocs.io/en/latest/`; it
  should link to the generated dashboard once the first Pages publication
  exists. A local Read the Docs guide now lives at
  `docs/readthedocs/community/performance_dashboard.rst` and explains the
  dashboard URL, first-use Quick Start, `status.json`, `data.json`,
  `summary.md`, freshness, native/reference comparisons, machine-readable
  ratio formulas, compact `trend_summary`, compact `testbed_summary`, rendered
  `external_competitor_status` metadata, local preview commands, and the
  consolidated
  `check-bm-dashboard-launch-preflight` hosted-path rehearsal.
- Endpoint check on 2026-05-21: Read the Docs returned HTTP 200, GitHub Pages
  is enabled at `https://dartsim.github.io/dart/` from the `gh-pages` branch,
  and `https://dartsim.github.io/dart/performance/` plus
  `https://dartsim.github.io/dart/performance/status.json` returned HTTP 404,
  so the remaining hosting blocker is publishing `gh-pages/performance/` from
  the reviewed workflow. After publication, `status.json` is the smallest
  endpoint to verify first.
- The post-launch gate currently fails for the expected reason:
  `Performance Dashboard` is not registered in GitHub Actions at
  `.github/workflows/performance_dashboard.yml` because the local workflow has
  not been pushed, reviewed, and merged to `main`.
- Fresh launch gate check on 2026-05-21: service-source URLs, Read the Docs
  source wiring, local workflow registration with `--allow-missing`, the full
  performance-dashboard unit suite, and the consolidated no-push
  preflight passed. The post-merge live gate failed at workflow registration.
  Treat that as evidence that the local hosted-path rehearsal is still healthy
  and the remaining blocker is remote approval/publication.
- Fresh endpoint recon on 2026-05-21 from the current local branch: the branch
  was clean, one commit ahead of `origin/main`, no PR existed for
  `task/performance-dashboard`, GitHub Actions still had no registered
  `Performance Dashboard` workflow on `main`,
  `https://dart.readthedocs.io/en/latest/` returned HTTP 200, and the
  dashboard guide plus all `https://dartsim.github.io/dart/performance/`
  surfaces returned HTTP 404.
- Fresh launch gate check on 2026-05-21 from the clean local branch: the
  no-push launch preflight passed, downloaded
  `collision-benchmark-guard-26144293889-1`, and rendered 197 measurements.
  The live launch gate aggregated the remaining public blockers: no registered
  workflow, no latest successful workflow run, missing canonical website links,
  missing Read the Docs guide, and 404 responses for the hosted dashboard
  HTML, `status.json`, `data.json`, and `summary.md`.
- Previous clean local-branch launch gate check on 2026-05-21: the
  consolidated no-push preflight passed, downloaded
  `collision-benchmark-guard-26144293889-1`, rendered a workflow-shaped
  dashboard with 197 measurements, and the post-merge live gate still failed
  at workflow registration because the workflow is not registered on `main`.
- Workflow action refs checked on 2026-05-21: the local workflow-registration
  preflight now verifies local composite actions plus external GitHub Action
  tags. Current tags resolve for `actions/checkout@v6`,
  `actions/upload-artifact@v6`, `awalsh128/cache-apt-pkgs-action@v1.6.0`,
  `bencherdev/bencher@v0.6.6`, `prefix-dev/setup-pixi@v0.9.4`, and
  `mozilla-actions/sccache-action@v0.0.9`.
- Optional Bencher reporting is guarded as launch-safe: setup/reporting must
  stay on `main`, stay `continue-on-error`, read `BENCHER_API_KEY` from the
  Actions secret, and pass `--skip-if-unconfigured` plus
  `--skip-if-no-input`, so missing maintainer Bencher configuration cannot
  block the GitHub Pages dashboard publication.
- Local workflow-registration checks now default to `GITHUB_TOKEN` and then
  `gh auth token`; this avoids unauthenticated GitHub REST rate limits for the
  workflow-list and action-tag checks without mutating GitHub.
- A no-push rehearsal against the actual current `origin/gh-pages` branch
  rendered and verified the first `performance/` tree creation path under
  `build/performance-dashboard-gh-pages-rehearsal.*/performance`. The branch
  currently has zero `performance/` files, so the first approved workflow run
  is expected to create the directory rather than append to existing public
  history.
- GitHub's `GITHUB_TOKEN` behavior is a critical publication detail: commits
  pushed by Actions with `GITHUB_TOKEN` do not trigger a Pages build by
  themselves. The workflow now uses the Pages builds REST endpoint after the
  publish step so the legacy branch-source site should actually deploy.
- `pixi run check-bm-dashboard-pages` checks the live Pages API configuration.
  The current expected state is `build_type: legacy`, source branch
  `gh-pages`, path `/`, and `html_url: https://dartsim.github.io/dart/`.
- `pixi run check-bm-dashboard-pages-branch` checks the live `gh-pages` branch
  and the `performance/` path without mutating GitHub. Before first
  publication it should report that `performance/` is absent and ready for
  first publication; after launch it should report that the path contains
  dashboard files. `check-bm-dashboard-launch-live` runs this check in strict
  mode and reports a missing `gh-pages/performance/` path as a launch blocker.
- `pixi run check-bm-dashboard-pages-build` checks the latest GitHub Pages
  build without mutating GitHub. It requires status `built`, no Pages build
  error message, and a build commit matching the current `gh-pages` branch
  head. After the workflow publishes dashboard files, this is the read-only
  proof that Pages built the current source commit.
- `pixi run check-bm-dashboard-website-sources` checks local Read the Docs
  source wiring for the dashboard homepage links and dashboard guide. It is now
  part of launch preflight so the branch can fail before PR if the website
  stops linking to the hosted dashboard, `status.json`, or guide while the live
  Read the Docs page is still unpublished.
- `pixi run check-bm-dashboard-service-sources` checks the external evidence
  URLs in the generated service decision and rejects redirects. It is part of
  launch preflight so the Pages/Bencher/CodSpeed/github-action-benchmark/
  Airspeed-Velocity/LLVM-LNT/Conbench/OpenBenchmarking recommendation cannot
  launch with stale source links. It retries transient network errors and HTTP
  5xx responses before failing.
- `pixi run check-bm-dashboard-workflow-registration` is intended for after the
  workflow lands on `main`. It verifies GitHub Actions has registered
  `Performance Dashboard` as an active workflow at
  `.github/workflows/performance_dashboard.yml` and that the local workflow
  still has the main-push/scheduled/manual triggers, the required
  `30 3 * * 0,3` scheduled cadence, dashboard-relevant push path filters, the
  main-only Pages build request, hosted endpoint
  verification flags, and the `actions: read`, `contents: write`, and
  `pages: write` permissions required for publication. It also rejects
  floating Bencher setup action refs; use the script's `--allow-missing` mode
  only for local pre-merge checks.
- `pixi run check-bm-dashboard-workflow-run` is intended for after the
  workflow lands on `main` and has run at least once. It queries dashboard
  workflow runs without mutating GitHub and requires the latest `main` run to
  have `status: completed`, `conclusion: success`, a run URL, and the
  `performance-dashboard-site-<run_id>-<run_attempt>` hosted-site artifact.
  Before the workflow lands, GitHub returns HTTP 404 for the workflow-runs
  endpoint; the script reports that as "workflow runs are not available yet".
- `publication_contract` is now part of the generated dashboard API. Keep it
  in sync with `.github/workflows/performance_dashboard.yml`, the live Pages
  source (`gh-pages` at `/`), the Read the Docs homepage dashboard links, and
  the public endpoint verifier whenever the hosted publication path changes.
- Live publication verification now requires the canonical Read the Docs
  website to expose the hosted dashboard URL, `status.json`, and the dashboard
  guide. Pre-merge checks still use `--allow-unpublished`, so the root website
  only needs to be reachable until the first reviewed publication lands.
- The public GitHub Actions artifact API can list repository artifacts, but
  archive downloads require authenticated access and redirect to signed storage
  URLs. The importer now falls back to `gh auth token` for local checks and
  drops the GitHub authorization header on cross-host redirects. The workflow's
  explicit `actions: read` permission remains important for artifact seeding.
- A PR body draft matching `.github/PULL_REQUEST_TEMPLATE.md` is tracked at
  `docs/dev_tasks/performance_dashboard/PR_BODY.md`. It includes the action-ref
  checks, Bencher guardrails, authenticated workflow-registration fallback,
  `github-action-benchmark` service-decision row, comparison metric contract,
  compact `trend_summary`, required website links, and the latest
  performance-dashboard suite result.
  Use it when the maintainer approves `git push -u origin HEAD` and PR
  creation.
- GitHub Actions artifacts are short-lived; a locally downloaded live sample
  from run `26144293889` expires on 2026-06-03 and contains nine benchmark
  JSON files under ignored `build/performance-dashboard-seed-check/` state.
- `scripts/generate_performance_dashboard.py` can already render
  `build/performance-dashboard/{data.json,index.html,summary.md,status.json}`
  from the downloaded sample artifact, including best-reference rows, an
  all-backend comparison matrix, per-surface reference-backend summaries,
  explicit reference/competitor coverage, primary and reference-backend trend
  deltas, informational threshold status, freshness metadata, sparklines, and
  direct links to `status.json`, `data.json`, `summary.md`, and the Read the
  Docs dashboard guide. Its generated `data.json` and `status.json` also carry
  self-describing public links for the canonical website, dashboard, status,
  data, summary, and guide URLs. It also carries the selected service decision
  in every generated public artifact so maintainers and endpoint checks can
  see that Pages is the canonical host and Bencher remains an opt-in companion
  service. Its comparison-coverage catalog now has nine expected surfaces:
  collision, dynamics, LCP, common allocator, math, SIMD, simulation, compute
  graph, and GPU/accelerators. The dashboard surface runner now collects
  representative simulation world-step and compute graph rows as part of the
  scheduled slice. Each coverage row includes the surface's
  comparison target, entrypoint, and next step, so hosted `data.json` is enough
  to understand what a future competitor/reference harness must measure.
  Reproducible competitor rows with supported backend suffixes such as
  `_MuJoCo`, `_Drake`, `_Gazebo`, and `_PyBullet` are parsed as live external
  competitor coverage and counted separately from deferred candidates. It skips
  empty zero-byte benchmark JSON inputs so a failed benchmark command that
  leaves an empty output file does not prevent publication of valid rows from
  other surfaces.
- `pixi run bm-dashboard` renders existing `.benchmark_results/*.json` into the
  same output directory for local, non-publishing checks.
- `pixi run check-bm-dashboard-publication` verifies the local generated
  dashboard, confirms `https://dart.readthedocs.io/en/latest/` is reachable,
  reports whether the Read the Docs dashboard guide page has been published,
  and reports the public endpoint state while allowing it to be unpublished
  before the first reviewed `main` workflow run.
- `pixi run check-bm-dashboard-publication-live` is the lower-level live
  endpoint variant: it requires the Read the Docs dashboard guide plus public
  status/data, dashboard HTML, and summary endpoints to be live and fresh.
  The post-merge completion gate remains
  `pixi run check-bm-dashboard-launch-live` because it also checks workflow
  registration and Pages configuration.
- `pixi run check-bm-dashboard-workflow-dry-run` renders and verifies a local
  workflow-shaped `gh-pages/performance/` tree under ignored
  `build/performance-dashboard-workflow-dry-run/` state, including the
  generated workflow summary file. That summary must include public links,
  service-decision metadata, the publication contract, required permissions,
  launch checks, and the approval boundary. This is the local rehearsal for the
  workflow's Pages publish path.
- `pixi run check-bm-dashboard-launch-preflight` is the consolidated no-push
  launch gate to run before requesting approval for `git push -u origin HEAD`
  and PR creation. It verifies Pages settings, local Read the Docs source
  wiring, the launch PR body draft, service-source evidence URLs, local
  workflow registration shape, live seed artifact download, workflow-shaped
  dashboard rendering, and publication endpoint state from the generated
  dry-run payload.
- `pixi run check-bm-dashboard-launch-live` is the post-merge completion gate.
  It uses the same launch preflight script but requires the workflow to be
  registered and the Read-the-Docs guide plus public Pages dashboard endpoints
  to be live.
- `pixi run check-bm-dashboard-empty` renders and verifies a temporary
  zero-measurement dashboard with explicit missing `.benchmark_results` input.
  Use it to validate the first public URL creation path without benchmark JSON.
- `pixi run bm-dashboard-artifacts` downloads recent non-expired real benchmark
  artifacts, including `collision-benchmark-guard-*` and
  `performance-dashboard-raw-*`, into `.benchmark_seed/` when the GitHub
  Actions artifact API exposes them. The default Pixi task filters to `main`,
  and the workflow filters to `GITHUB_REF_NAME`; artifacts without matching
  branch metadata are skipped when a branch is requested. The dashboard
  generator uses those files only when durable `gh-pages/performance/data.json`
  history does not exist yet. Seeded run IDs include the artifact/source
  directory so same-timestamp artifacts do not overwrite each other. Local runs
  without an artifact token may skip downloads after listing the artifact; the
  workflow provides `GITHUB_TOKEN` with `actions: read`.
- `pixi run bm-dashboard-surfaces` dry-runs or emits bounded non-collision
  Google Benchmark JSON for common allocator, dynamics/kinematics, KR5/Atlas
  real robot dynamics I/O, LCP smoke, math helper, SIMD add, simulation
  world-step, and compute graph rows.
- A local short all-surface smoke run with `--benchmark-min-time=1ms` and one
  repetition built the common allocator, kinematics, `bm_dynamics_cache_io`,
  LCP, math helper, SIMD, simulation world-step, and compute graph dashboard
  slice targets, then produced valid ignored JSON for all eight dashboard
  surface files. It reported 32 common rows, four kinematics rows, four
  KR5/Atlas real-robot rows, one LCP row, 16 math rows, eight SIMD rows, two
  simulation rows, and two compute graph rows. This run exposed and fixed the
  common/math/SIMD no-match filters and the runner now treats zero-row JSON
  output as a surface failure.
- `.github/workflows/performance_dashboard.yml` is a local workflow draft that
  checks out `gh-pages`, downloads recent real benchmark artifacts from the
  active branch for first-run history seeding, runs collision plus bounded
  non-collision dashboard benchmarks, renders `performance/`, uploads
  short-lived raw/site artifacts, pushes only when running on `main`, and can
  report to Bencher from `main` only after `BENCHER_PROJECT` and
  `BENCHER_API_KEY` are configured. It uses an empty render fallback so the
  hosted dashboard URL can be created even if the first benchmark collection
  produces no JSON or `.benchmark_results/` is absent, preserves the last good
  run when later collections produce no JSON, and verifies the generated
  dashboard files before upload or publication. It
  writes a GitHub Actions summary with public dashboard URLs and status counts,
  explicitly requests a GitHub Pages build on every `main` run after the
  publish step because branch-source Pages does not automatically build commits
  created with `GITHUB_TOKEN`, and because reruns need to repair missed
  deployments even when no new `gh-pages` commit is needed. It then verifies
  the public `status.json`, `data.json`, dashboard HTML, and `summary.md`
  endpoints against the generated local files and requires the hosted endpoint
  to be fresh.
- `scripts/check_performance_dashboard_endpoint.py` is the public endpoint
  verifier. It can be run against `file://` URLs in tests and against
  `https://dartsim.github.io/dart/performance/status.json` in the workflow.
  Use `--require-fresh` for hosted checks that should fail stale dashboard
  data. Use `--expect-data-file` when the hosted `data.json` history should
  match a generated local file exactly. Use `--require-dashboard-page` and
  `--require-summary` when the hosted HTML and Markdown surfaces must be
  reachable; use `--expect-dashboard-file` and `--expect-summary-file` when the
  hosted surfaces should match generated local files exactly. It also verifies
  that `status.json` advertises the selected DART public URLs, including the
  new Read the Docs website, the selected GitHub Pages plus Bencher service
  roles, and that hosted `data.json` carries every expected comparison surface
  with external competitor candidates and the reproducible-harness requirement.
- `scripts/verify_performance_dashboard.py` is the local generated-dashboard
  verifier. It now rejects renders whose `data.json`/`status.json` drop the
  expected public links, selected service roles, comparison coverage surfaces,
  external competitor candidates, reproducible-harness requirement, or
  main-push/scheduled/manual update-trigger metadata. It also rejects rendered
  HTML or `summary.md` that lose service-decision metadata, the publication
  contract, comparison input rules, or reference/competitor coverage.
- `scripts/check_performance_dashboard_pages_config.py` is the Pages settings
  verifier. The workflow runs it on `main` before publishing so a future switch
  away from branch-source Pages fails explicitly instead of producing an
  unreachable dashboard.
- The generated reference/competitor coverage table lists deferred candidate
  harnesses per surface, including MuJoCo, Drake, Gazebo/gz-physics, PyBullet,
  Simbody, solver baselines, allocator baselines, and math/SIMD baselines. Do
  not treat those as results unless a future harness emits comparable dashboard
  JSON.
- `scripts/summarize_performance_dashboard.py` is the Actions summary helper.
  It reads generated `status.json` and writes the canonical website,
  dashboard URL, status/data URLs, dashboard guide, run identity,
  measurement/detail row counts, comparison states, and Bencher configuration
  state into `$GITHUB_STEP_SUMMARY`, including the advertised main-push,
  scheduled, and manual update triggers plus the publication contract and
  `external_competitor_status`.
- `scripts/report_performance_to_bencher.py --dry-run` can validate the Bencher
  command shape without sending data or exposing the API key. It skips zero-byte
  benchmark JSON files so optional Bencher reporting follows the dashboard
  generator's failed-surface behavior.
- Bencher is the preferred external service because it can ingest current
  Google Benchmark JSON with `cpp_google` and does not require changing DART's
  C++ benchmark build.
- CodSpeed should be a later pilot for pure microbenchmarks because its C++
  integration requires a CodSpeed Google Benchmark build mode.
- The remote `origin/task/native-collision-performance-complete` branch has a
  useful native-collision benchmark-manifest generator, but it is narrower than
  the repo-wide dashboard and includes unrelated AI-doc changes versus current
  `main`; reuse ideas selectively instead of merging the branch wholesale.

## How To Resume

```bash
git checkout task/performance-dashboard
git status -sb
git log -3 --oneline
```

Then continue with the publishing/first-run work described in
`docs/dev_tasks/performance_dashboard/README.md`.

Do not push, open a PR, trigger the workflow, or mutate GitHub state until the
maintainer/user explicitly approves that remote action.
