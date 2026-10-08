# CI And DART 6 Checks

Use GitHub Actions as the hosted source of truth after a PR is opened. Locally,
run the smallest gate that proves the touched surface, then broaden when shared
runtime, package, or downstream behavior changes.

## Workflow Index

All files live in `.github/workflows/` on this branch. "PR, push" means pull
requests and pushes to `main` and `release-*`; "nightly" means the `Nightly`
workflow below. Note that `gh pr checks` lists job-level check names (for example
`Release` under the `CI Linux` workflow); map a failing check to its workflow
via the run's workflow name shown here (`gh pr checks` exposes it in the
`workflow` JSON field).

| Workflow file                     | Workflow name                | Runs                            | Purpose |
| --------------------------------- | ---------------------------- | ------------------------------- | ------- |
| `ci_ubuntu.yml`                   | CI Linux                     | PR, push, nightly               | AI checks, lint, Release build + test, no-OSG assertions build + test; nightly adds install, ASan, coverage (the Debug build), and Eigen 64-byte alignment |
| `ci_macos.yml`                    | CI macOS                     | PR, push, nightly               | arm64 Release build + test; nightly adds install |
| `ci_windows.yml`                  | CI Windows                   | PR, push, nightly               | MSVC Release build + test |
| `ci_gz_physics.yml`               | CI gz-physics                | PR, push, nightly               | Gazebo/gz-physics downstream integration |
| `api_doc.yml`                     | API Documentation            | PR, push, nightly               | Doxygen API docs build (validation only; not published) |
| `ci_simd.yml`                     | CI SIMD Multi-Arch           | PR/push touching SIMD, nightly  | SIMD instruction-level matrix (scalar/SSE4.2/AVX/AVX2) on x86_64; NEON is covered by `ci_macos.yml` arm64 jobs |
| `ci_freebsd.yml`                  | CI FreeBSD (VM)              | nightly, dispatch               | FreeBSD build + test in a VM |
| `ci_toolchain.yml`                | CI Toolchain (Linux)         | nightly, dispatch               | Newest gcc/clang build + test |
| `codeql.yml`                      | CodeQL                       | nightly, dispatch               | Static security analysis |
| `publish_dartpy.yml`              | Publish dartpy               | nightly, version tags, dispatch | Build, repair, verify, and test wheels; publish from version tags |
| `nightly.yml`                     | Nightly                      | daily, on demand                | Everything above on `main`; files `nightly-failure` issues |
| `performance_dashboard_dart6.yml` | DART 6 Performance Dashboard | push, dispatch                  | Advisory wall-time dashboard on `ubuntu-24.04`, profiler and alerts off |
| `perf.yml`                        | Performance regression       | PR/push to main touching perf paths, nightly, dispatch | Advisory `Perf A/B` counts and guards; merge records and Ir/allocation chart; nightly absolute values and generated S1–S6 guards |
| `update_lockfiles.yml`            | Update Lock Files            | weekly                          | Pixi lockfile refresh PRs against `main`; an update removes their `maintainer-approved` label |
| `maintainer_approval.yml`         | Maintainer Approval          | PR pushes, retargets, reopens   | Removes the `maintainer-approved` label when a PR changes after approval; pushes of conflict-free base merges keep it ([PR Lifecycle](ai-tools.md#pr-lifecycle)) |

To acknowledge an intended regression, add `Perf-Regression-Rationale: <rows>: <reason>` (or `Rebaseline-Rationale: <rows>: <reason>` for changed guards, including a signed Ir percentage when above +1%) to the PR body and run `gh run rerun <run-id> --failed`; editing the body alone does not trigger a run.

Required checks on `main`: `Release` and
`Asserts enabled (no -DNDEBUG)` (CI Linux), `arm64-Release` (CI macOS),
`windows-Release` (CI Windows), `ubuntu-latest` (CI gz-physics),
`API Documentation`, and the two Read the Docs builds. Never require a
nightly-only job: it never reports on PRs, so it would block every merge.

## Performance Records And Guards

`perf.yml` extends the PR harness for two hosted tiers on `ubuntu-24.04`:

- The merge tier compares a qualifying push to `main` from its pre-update SHA
  (`github.event.before`) to HEAD, covering every commit in a rebase merge.
  The base must be an ancestor of HEAD; only a zero or unavailable pre-update
  SHA falls back to HEAD's first parent. Manual dispatch uses that first parent
  and validates the optional `base` input against it.
  Measurement uses the base's harness and thresholds, in HEAD's pixi environment.
  Harness/workflow changes also run a head smoke capture.
  Pixi-only changes are skipped. It finds the merged PR through the commit's
  pull-request API endpoint and checks its current body for rationale lines.
  A gated regression without an applicable `Perf-Regression-Rationale` or
  `Rebaseline-Rationale` gets a comment on that PR and fails the writer job.
  After changing a merged PR's rationale, rerun the full workflow with
  `gh run rerun <run-id>` so measurement reads the new body; rerunning only the
  writer reuses the saved verdict. A passing rerun updates an existing marked
  failure comment; it does not create a new comment. Comments identify their
  measurement time so a writer that reads a newer verdict leaves it untouched.
- The nightly tier calls the same workflow from `nightly.yml`, measuring HEAD
  only. It runs the quick rows, canonical S1–S6 native guards, layout
  perturbations, scene estimated cycles, and the opt-in matrix-free row. S3
  and S4 include 1- and 4-thread captures alongside their canonical 16-thread
  captures. Failed perturbation checks make the row diagnostic and fail the
  nightly `perf` group. Every `contact_benchmark` detector row must report its
  final contact pair count; a missing count fails the nightly.

Both tiers publish in this repository's `gh-pages` branch:

| Path | Content |
| --- | --- |
| `performance/records/main/<yyyy>/<date>-<sha12>-<tier>.json` | Plain JSON `dart-perf/1` record: revisions, runner/host metadata, environment fingerprint, accepted rationale lines, per-row values, guards and advisory wall time |
| `performance/dart6-ir/` | Merge-only Ir and allocation chart, alerts off; input changes split series, other continuity changes are annotated, and the chart retains 250 points |
| `performance/guards/main.md` | Latest generated nightly S1–S6 guard table, with revision and fingerprint; replaces manual live baseline tables |

The nightly record is added when the newest record by measurement time differs
in HEAD, environment fingerprint, or results (including guards and advisory
metrics). Identical results at a later time do not add another record. The guard
table advances by commit ancestry, then measurement time for the same HEAD;
rerunning the same artifact leaves it untouched, preserving drift annotations.
Records keep full history beyond the chart window. Repeated merge runs with the
same HEAD and fingerprint must reproduce identical per-row Ir, allocations,
requested bytes, guards (hash, contacts, pairs, resting, finite, cap hit and
penetration), penetration checkpoints, time advancement, perturbation evidence,
gate qualification, micro instrumentation and measurement inputs.
Perturbation qualification, multithread parity and A/B guard comparison also
include separately stored penetration and checkpoint evidence.
The chart writer validates this evidence too; legacy chart points can only
validate the plotted counts they retained. A nightly with changed results is
retained even if its timestamp matches an earlier record.
Unchanged verdict/rationale reruns are deduplicated; a new accepted rationale or verdict
keeps another immutable
record without another chart point. The `<date>`
filename component is a UTC timestamp with microseconds, for example
`2026-10-08T080000000000Z-aaaaaaaaaaaa-merge.json`, so a fingerprint change
on the same day preserves both records.
Publication retries regenerate against the fetched `gh-pages` tip, including
deduplication and derived chart/table data, before attempting a normal push.
Chart points use the full measurement time, with commit/fingerprint tie breaks,
so equal-time arrivals have consistent order and annotations. The stock page
uses series names such as `s3w/dart@1:01234567 Ir`, including the first eight
hex characters of the row's `input_sha`. Workload or data input changes start
a new series even without a version bump, so incomparable inputs are never
connected. Older series without input identity remain separate. Hover
annotations mark the first point after an environment fingerprint, thread
count, warm-up/step window, measurement method, collection signature or micro
instrumentation changes. Annotations compare each series with its last observed
point, including across missing rows, and are recomputed when older evidence
arrives. Older points without continuity metadata are shown as `unknown` at
the transition. Row names, detectors and versions also
form distinct series names. Hover annotations preserve the stock page; they do
not remove the connecting line, so annotated transitions are incomparable.
The fingerprint includes valgrind and its guest CPU,
compiler, glibc, `pixi.lock`, preset and harness identity. Read records directly
with `git show origin/gh-pages:performance/records/main/<yyyy>/<file>.json`.
The canonical commands and historical baseline evidence remain in
[`01-baseline-evidence.md`](../dev_tasks/dart6_performance_generalization/01-baseline-evidence.md).

This compact merge-record example uses illustrative values and one row:

```json
{
  "schema": "dart-perf/1",
  "run": {
    "tier": "merge", "commit": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    "parent": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", "branch": "main", "pr": 3570,
    "describe": "v6.19.4-270-gaaaaaaaaaaaa", "time": "2026-10-08T08:00:00+00:00",
    "env": {
      "runner": {"environment": "github-hosted", "name": "GitHub Actions 1", "image": "ubuntu24/20261004.1"},
      "host_cpu": "AMD EPYC 7763", "valgrind": "3.22.0", "valgrind_guest_cpu": "Intel Core i7-4910MQ",
      "compiler": "GCC 13.3.0", "glibc": "2.39", "preset": "perf-1",
      "pixi_lock_sha": "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
      "harness_sha": "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff",
      "fingerprint": "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"
    },
    "accepted": []
  },
  "results": [{
    "row": "s3w", "det": "dart", "version": 1, "gated": true, "status": "ok", "method": "slope",
    "input_sha": "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
    "window": {"warmup": 5, "steps": 5},
    "parent": {
      "ir_per_step": 167316103, "allocs_per_step": 9009, "max_rss_kb": 147960, "wall_ms_per_step": 53.1,
      "guards": {"hash": "0x3250ca4b2debb3f8", "finite": true, "contacts": 3003, "cap_hit": false, "resting": "0/3003"}
    },
    "head": {
      "ir_per_step": 167316103, "allocs_per_step": 9009, "max_rss_kb": 147960, "wall_ms_per_step": 47.9,
      "guards": {"hash": "0x3250ca4b2debb3f8", "finite": true, "contacts": 3003, "cap_hit": false, "resting": "0/3003"}
    },
    "delta": {"ir": 0, "allocs": 0, "bytes": 0, "guards_equal": true, "class": "gated"},
    "wall_ms_per_step": {"parent": 53.1, "head": 47.9, "advisory": true}
  }],
  "verdict": {"status": "PASS", "failures": [], "warnings": [], "ir_geomean": 0}
}
```

Nightly records have `parent: null` and absolute head metrics; S6 checkpoint
penetration values remain in JSON and the generated guard table reports the
final maximum penetration.

PR measurements use a read-only token and checkout with
`persist-credentials: false`; the separate verdict job never runs PR code.
Merge and nightly measurements also run in read-only jobs (`record-measure`
and `nightly-measure`) with `persist-credentials: false` and upload evidence
artifacts. Merge measurement has `pull-requests: read` for rationale lookup;
its `GH_TOKEN` exists only on that lookup step. The separate writers (`record`
and `nightly`) download the evidence and run main's publisher without building
DART or running benchmark binaries. Both have `contents: write`; only the merge
writer has `pull-requests: write`, with `GH_TOKEN` scoped to its sticky failure
comment step. Its main publisher checkout uses `persist-credentials: false`;
the dedicated `gh-pages` checkout retains credentials for the git publication
loop. The merge writer publishes and comments before enforcing the saved
verdict. Merge evidence is retained per measurement attempt; writer reruns use
that attempt's artifact name from the measurement job's outputs.
Merge publication requires a push or dispatch on `main`; nightly publication
requires a schedule or dispatch on `main`.
A CI-change PR or off-main dispatch can measure but writes
nothing. The writer refuses any runner whose environment is not
`github-hosted`. Every write fetches `origin/gh-pages`, rebases onto it and
pushes with bounded retries after rejection; it never force-pushes. The
repository ruleset also blocks force-push and branch deletion.

The existing wall-time dashboard continues at `performance/dart6/`, pinned
to `ubuntu-24.04`, with `DART_BUILD_PROFILE=OFF`, alerts and alert comments
disabled. Removing profiler overhead intentionally introduces a one-time
level shift; historical points across that shift are not directly comparable.
CI has no quiet hardware: wall time and estimated cycles are advisory.
Cache, prefetch, SIMD and threading claims require hand-run `perf stat`
evidence. Backfill and release/tag records are P4; `release-6.19` has no
per-commit tracking in P3.

After merging workflow changes, verify the hosted behavior: dispatch the
merge tier twice for the same `main` SHA and check identical Ir or different
fingerprints (including host CPU metadata); inspect the record schema;
compare the generated guard table with a manual S1–S6 capture; confirm that
a CI-change PR nightly dry run writes nothing and a pixi-only merge is
skipped.

For the repeated-SHA check, use
`gh workflow run perf.yml --ref main -f tier=merge -f head=<main-sha>` twice.
Manual dispatches can name an ancestor of `main` that provides the selected
tier's harness (`nightly` requires P3); the merge base, when supplied, must be
that commit's first parent. Historical backfill remains P4.

## Caching

Build jobs save and restore per-run sccache snapshots, as described for
Windows below; Linux, macOS and gz-physics share `.github/actions/sccache`,
and each workflow's `Prune compiler caches` job deletes superseded snapshots.
Each job's `SCCACHE_CACHE_SIZE` holds about two full builds, so snapshots stay
small and fresh, and each job prints `sccache --show-stats`. Pixi environment
caches are written only from `main`.

Windows also splits its build, because each MSVC cache miss is expensive. Its
build runs as two parallel jobs, `windows-Release-cpp` (C++ tests) and
`windows-Release-python` (dartpy), reported together as the required
`windows-Release` check. Every main push saves a snapshot of the whole cache.
Every same-repository PR run saves only the objects that PR's runs compiled,
even after a failure or cancellation (fork PRs may not save caches). A PR
restores the newest main snapshot plus its own, so it rebuilds only what
changed since its last push. `windows-Release` keeps the newest run's snapshot
per job in its ref and deletes the rest; on main pushes it also deletes PR
snapshots unused for a day. Cache keys include the MSVC version. Each MSVC
compile spends most of its time parsing headers, so Windows CI builds each
target as unity translation units (`CMAKE_UNITY_BUILD`). The Linux assertions
gate does too, so a collision also fails on Linux. Keep file-local names
unique within a target for this ([Code Style](code-style.md)).
CTest runs in parallel (`CTEST_PARALLEL_LEVEL`). Nightly-only configurations
never save, so they build cold. Pixi build tasks pin `BUILD_TYPE=Release`, so
a job that needs another build type configures CMake itself, as the assertions
gate does.

## Nightly

`nightly.yml` runs every workflow in the index except the wall-time performance
dashboard, lockfile refresh, and maintainer approval against `main` each night
at 08:17 UTC, including the nightly-only jobs. It is scheduled directly on
`main`, the default branch, with no dispatcher. Run it on demand with
`gh workflow run nightly.yml --ref main`.

Its `report` job (`scripts/nightly_ci_report.py`) groups jobs by their
`nightly.yml` caller (`linux`, `macos`, `freebsd`, `perf`, ...) and keeps at most one
open `nightly-failure` issue per failing group tracking `main`. It opens the
issue with log excerpts, fixing steps, and a prompt for an AI agent; comments
on it each night the group still fails; and closes it on the first night the group
succeeds. PRs do not run it, so the full matrix never delays the per-PR
checks; to try a CI change against it, run
`gh workflow run nightly.yml --ref <branch>` (off `main` the report only
dry-runs). Test the reporter with
`pixi run python -I scripts/run_pytest.py tests/test_nightly_ci_report.py`.

Useful commands:

```bash
pixi run lint
pixi run build
pixi run test
pixi run test-py
pixi run -e gazebo test-gz
```

`ci_gz_physics.yml` runs the forward lane, which patches gz-physics before
testing and configures it once, so gz-physics' contact-callback test
expectations are not compiled in. The unpatched Gazebo lanes
(`pixi run gz-compat-ionic`, `gz-compat-jetty`, `gz-compat-harmonic`; see
`tools/gazebo/README.md`) are not in CI yet: each builds DART, gz-physics, and
gz-sim from source and runs a serial suite, and on release-6.20 they
currently report the known DART 6.20 Gazebo regressions from issue #3056. Run
them locally for downstream-sensitive changes and before releases.

For failing CI, inspect the exact run and job logs before changing code. Prefer
reproducing locally, but document when a hosted-platform failure cannot be
reproduced on the current machine.
