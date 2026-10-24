# CI And DART Checks

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
| `ci_ubuntu.yml`                   | CI Linux                     | PR, push, nightly               | AI checks, build-requirement checks, lint, Release build + test, nanobind Python tests, GUI-disabled assertions build + test; nightly adds install, ASan, coverage (the Debug build), Eigen 64-byte alignment, and a unity name-clash check |
| `ci_macos.yml`                    | CI macOS                     | PR, push, nightly               | arm64 Release build + test and nanobind Python tests; nightly adds install |
| `ci_windows.yml`                  | CI Windows                   | PR, push, nightly               | MSVC Release C++ tests and nanobind Python tests on VS 2026; unity builds; nightly adds the build-requirement checks and the VS 2022 (v143) baseline C++ leg |
| `ci_filament.yml`                 | CI Filament                  | PR, push, nightly, dispatch     | GUI test lane: `ubuntu-24.04-Filament-functional` and `macos-15-Filament-functional`, plus `windows-2022-Filament-functional` nightly, configure with `pixi run -e gui-ci config -- -DDART_ENABLE_GUI_SMOKE_TESTS=ON` and run the Filament C++ tests (display-free Vulkan and Xvfb native-window tests on Linux, native tests on Windows and macOS), the Python GUI and integration tests, and a relocated C++17 consumer check. The CI Linux, macOS, and Windows Release jobs build the GUI but skip the `gui-device`-labeled CTest tests and the Python rendering modules; device-free GUI tests still run. CI Linux also runs the display-free agent-capture smoke and `test-agent-debug-overlay` on Lavapipe. The `nvidia-Filament-hardware` job runs only on `main` pushes and manual dispatch; see [Hardware GPU job](#hardware-gpu-job) |
| `ci_gz_physics.yml`               | CI gz-physics                | PR, push, nightly               | Gazebo/gz-physics downstream integration |
| `api_doc.yml`                     | API Documentation            | PR, push, nightly               | Doxygen API docs build (validation only; not published) |
| `ci_simd.yml`                     | CI SIMD Multi-Arch           | PR/push touching SIMD or DART collision sources, nightly | SIMD instruction-level matrix (scalar/SSE4.2/AVX/AVX2) on x86_64; NEON is covered by `ci_macos.yml` arm64 jobs |
| `ci_freebsd.yml`                  | CI FreeBSD (VM)              | nightly, dispatch               | FreeBSD build + test in a VM |
| `ci_toolchain.yml`                | CI Toolchain (Linux)         | nightly, dispatch               | Ubuntu 24.04 floor (gcc 13, clang 18) and newest (gcc 16, clang 19) build + test |
| `codeql.yml`                      | CodeQL                       | nightly, dispatch               | Static security analysis |
| `publish_dartpy.yml`              | Publish dartpy               | nightly, version tags, dispatch | Build, repair, verify, and test wheels; publish from version tags |
| `nightly.yml`                     | Nightly                      | daily, on demand                | Everything above, on `main`; files `nightly-failure` issues |
| `performance_dashboard_dart6.yml` | DART 6 Performance Dashboard | push, dispatch                  | Advisory `release-6.*` wall-time dashboard on `ubuntu-24.04`, profiler and alerts off |
| `perf.yml`                        | Performance regression       | PR/push to main touching perf paths, nightly, dispatch | Advisory `Perf A/B` counts and guards; merge records and Ir/allocation chart; nightly absolute values and generated S1–S7 guards; release records per tag |
| `update_lockfiles.yml`            | Update Lock Files            | weekly                          | Pixi lockfile refresh PRs against `main`; an update removes their `maintainer-approved` label |
| `maintainer_approval.yml`         | Maintainer Approval          | PR pushes, retargets, reopens   | Removes the `maintainer-approved` label when a PR changes after approval; pushes of conflict-free base merges keep it ([PR Lifecycle](ai-tools.md#pr-lifecycle)) |
| `pr_text.yml`                     | PR Text                      | PR opens, edits, reopens, pushes | Checks PR title, body, messages, file names and added lines of every PR commit with only the base branch's local-path checker, skipping in PRs into `main` the changes of commits that a protected `release-*` branch already published; fetches PR git objects without checking out or executing PR code; skips with a notice until the checker exists on the base |

To acknowledge an intended regression, add `Perf-Regression-Rationale: <rows>: <reason>` (or `Rebaseline-Rationale: <rows>: <reason>` for changed guards, including a signed Ir percentage when above +1%) to the PR body and run `gh run rerun <run-id> --failed`; editing the body alone does not trigger a run.

Required checks on `main`: `Release` and
`Asserts enabled (no -DNDEBUG)` (CI Linux), `arm64-Release` (CI macOS),
`windows-Release-cpp` and `windows-Release-python` (CI Windows),
`ubuntu-latest` (CI gz-physics),
`API Documentation`, and the two Read the Docs builds. `CI Filament` jobs
are not yet required. Never require a nightly-only job: it never reports on
PRs, so it would block every merge.

Once the PR Text workflow is on `main`, the maintainer should add
`No local paths in PR text` to the `main` ruleset's required checks. The
repository ruleset is configured by the maintainer, not in code.

## Performance Records And Guards

`perf.yml` extends the PR harness for three hosted tiers on `ubuntu-24.04`:

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
  only. It runs the quick rows, canonical S1–S7 native guards, layout
  perturbations, scene estimated cycles, and the opt-in matrix-free row. S3
  and S4 include 1- and 4-thread captures alongside their canonical 16-thread
  captures. Failed perturbation checks make the row diagnostic and fail the
  nightly `perf` group. Every `contact_benchmark` detector row must report its
  final contact pair count; a missing count fails the nightly.
- The release tier is dispatched with
  `gh workflow run perf.yml --ref main -f tier=release -f head=<candidate-sha>`.
  Candidates must belong to the first-parent history of `main` or a
  `release-X.Y` branch. The tag is `v<package.xml version>` at the candidate;
  a supplied tag must match. With no `head`, `-f tag=vX.Y.Z` selects an existing
  tag's commit. An existing tag must name the selected candidate.
  The base is the newest earlier release tag merged into the candidate's parent;
  `-f base=vX.Y.Z` selects another eligible baseline. Both arms use `main`'s
  harness and Pixi environment. Comparisons involving a 6.19.x tag run the
  portable `gzb/ode` and `robot/dart` rows. A policy FAIL is published and the
  writer stays green; an ERROR publishes nothing. Only `main` candidates include
  a change ledger; `release-X.Y` candidates carry the tag comparison only.
  Hosted records take precedence over local tag records even when their
  environment fingerprints match. Identical-or-refuse checks apply only within
  the same `runner.environment`.
  Within that producer, a newer candidate or later measurement can replace the
  record before tagging. Once the tag exists, its commit's record replaces any
  stored candidate, including a newer or diverged one, and later candidates
  from that producer cannot replace it.

These tiers publish in this repository's `gh-pages` branch:

| Path | Content |
| --- | --- |
| `performance/records/main/<yyyy>/<date>-<sha12>-<tier>.json` | Plain JSON `dart-perf/1` record: revisions, runner/host metadata, environment fingerprint, accepted rationale lines, per-row values, guards and advisory wall time |
| `performance/dart7-ir/` | Merge-only Ir and allocation chart for main's DART 7 line, alerts off; input changes split series, other continuity changes are annotated, and the chart retains 250 points. `performance/dart6-ir/` keeps the frozen pre-7.0 history |
| `performance/guards/main.md` | Latest generated nightly S1–S7 guard table, with revision and fingerprint; replaces manual live baseline tables |
| `performance/records/main/<yyyy>/<date>-<sha12>-backfill.json` | Local A/B record with `runner.environment: local`, pushed once by a maintainer after review; never charted |
| `performance/releases/<tag>.{json,md}` and `index.md` | Comparison with the previous tag; main candidates include a ledger of changes; per-row release index; JSON and markdown attached by hand to the GitHub release |

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
### Canonical guard scenes

`scripts/perf_regression.py` owns the exact `NIGHTLY_ROWS` arguments and
measurement windows. Current values are published in
[the generated guard table](https://dartsim.github.io/dart/performance/guards/main.md).

| Scene | Purpose |
| --- | --- |
| S1 | Active container; primary fixture, 60 and 120 objects at 1 and 16 threads |
| S2 | Settled 3k shapes; sleeping regime |
| S3 | Active 3k shapes; deactivation disabled |
| S4 | 900 generated objects |
| S5 | 90 generated objects, serial |
| S6 | Penetration-creep and pile-sleep reproducer |
| S7 | Settled 3k shapes through `--gz-preset` with the DART, Bullet and ODE detectors at 1 and 4 threads; sleeping through custom-filter decision replay |

Read hash, contacts, pairs, resting, finite-state, cap-hit, and maximum
penetration columns together. Hashes compare only within one detector.
The S1–S6 guard settings do not substitute for
[Gazebo-path measurements](profiling.md#preserve-downstream-compatibility).
S7 runs the `--gz-preset` command those measurements use on `3k_shapes.sdf`,
but its table rows leave out the contact-demand, starved-pair, sunk-body and
changed-pose census; the row logs in the nightly evidence artifact keep it.
S7's ODE rows hit the contact cap by design: they keep the 10000-contact cap
that gz-physics sets, and this scene's ODE contact demand exceeds it.
Every S7 row must finish with at least one resting body. Within each record,
each detector's 1-thread and 4-thread guards must match; a mismatch marks both
rows broken. These checks reject replay-off regressions even in head-only
nightly smoke runs, which compare the record with itself.
Historical baseline rows remain in Git history.

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
Merge, nightly and release measurements also run in read-only jobs (`record-measure`,
`nightly-measure` and `release-measure`) with `persist-credentials: false` and upload evidence
artifacts. Merge measurement has `pull-requests: read` for rationale lookup;
its `GH_TOKEN` exists only on that lookup step. The separate writers (`record`,
`nightly` and `release`) download the evidence and run main's publisher without building
DART or running benchmark binaries. All have `contents: write`; only the merge
writer has `pull-requests: write`, with `GH_TOKEN` scoped to its sticky failure
comment step. Its main publisher checkout uses `persist-credentials: false`;
the dedicated `gh-pages` checkout retains credentials for the git publication
loop. The merge writer publishes and comments before enforcing the saved
verdict. Merge evidence is retained per measurement attempt; writer reruns use
that attempt's artifact name from the measurement job's outputs.
Merge publication requires a push or dispatch on `main`; nightly publication
requires a schedule or dispatch on `main`.
Release publication requires a dispatch on `main`; its writer has only
`contents: write` and no `GH_TOKEN`. A CI-change PR or off-main nightly or
release dispatch can measure but writes nothing; merge measurement runs only
on `main`. Hosted writers refuse any runner whose environment is not
`github-hosted`. Backfill publication refuses to run inside GitHub Actions.
It uses the maintainer's own git credentials after validating a local runner,
one fingerprint, `main` commits and a harness commit on `main`.
Every write fetches `origin/gh-pages`, resets to and regenerates against it and
pushes with bounded retries after rejection; it never force-pushes. The
repository ruleset also blocks force-push and branch deletion.

The existing wall-time dashboard continues at `performance/dart6/` for the
`release-6.*` maintenance line only (main is the DART 7 line), pinned
to `ubuntu-24.04`, with `DART_BUILD_PROFILE=OFF`, alerts and alert comments
disabled. Removing profiler overhead intentionally introduces a one-time
level shift; historical points across that shift are not directly comparable.
CI has no quiet hardware: wall time and estimated cycles are advisory.
Cache, prefetch, SIMD and threading claims require hand-run `perf stat`
evidence. `release-X.Y` branches are tracked by release records only.

After merging workflow changes, verify the hosted behavior: dispatch the
merge tier twice for the same `main` SHA and check identical Ir or different
fingerprints (including host CPU metadata); inspect the record schema;
compare the generated guard table with a manual S1–S7 capture run with the
harness's `LC_ALL` and `GLIBC_TUNABLES` settings; confirm that
a CI-change PR nightly dry run writes nothing and a pixi-only merge is
skipped.

For the repeated-SHA check, use
`gh workflow run perf.yml --ref main -f tier=merge -f head=<main-sha>` twice.
Manual dispatches can name an ancestor of `main` that provides the selected
tier's harness; the merge base, when supplied, must be that commit's first
parent. For an existing release tag, use
`gh workflow run perf.yml --ref main -f tier=release -f tag=vX.Y.Z`.
Historical comparisons use [`perf-backfill`](profiling.md#revision-comparisons).

Measurements stage every arm under one fixed path with a minimal environment,
so counts on one host do not depend on the checkout location, output
directory or environment, and per-revision deltas within one host are exact.
Across hosts, system libraries that load at startup still shift the heap
layout slightly: with identical fingerprints, a local run of a hosted PR
measurement matched every guard, allocation and byte count exactly and 11 of
17 rows' Ir exactly, while the rest differed by at most 0.1% (`s3w/dart`
+0.10%, `s3w/ode` -0.09%, the others under 0.01%). The local-to-hosted
overlap check therefore allows 0.1% relative Ir differences and requires
exact guards, allocations and bytes.

After publishing the GitHub release, attach its performance record manually:

```bash
(
set -e
git fetch origin gh-pages
git show origin/gh-pages:performance/releases/vX.Y.Z.json > dart-perf-vX.Y.Z.json
tag_commit=$(git rev-parse --verify 'vX.Y.Z^{commit}')
record_commit=$(jq -er .run.commit dart-perf-vX.Y.Z.json)
if [ "$record_commit" != "$tag_commit" ]; then
  echo 'record names another candidate; dispatch -f tag=vX.Y.Z first' >&2
  exit 1
fi
git show origin/gh-pages:performance/releases/vX.Y.Z.md > dart-perf-vX.Y.Z.md
gh release upload vX.Y.Z dart-perf-vX.Y.Z.json dart-perf-vX.Y.Z.md
)
```

If the commit check fails, dispatch again with `-f tag=vX.Y.Z` and no `head`
before uploading; the tagged commit's record replaces the stored candidate
even if that candidate is newer or diverged. A FAIL needs an explanation of
its changed guards or gated regressions; it does not block release by itself.

## Caching

Build jobs save and restore per-run sccache snapshots, as described for
Windows below; Linux, macOS and gz-physics share `.github/actions/sccache`.
After saving, each job runs `.github/actions/sccache/prune.sh`: in its ref it
keeps its part's newest snapshot and deletes the rest, and on main pushes it
also deletes its part's PR snapshots unused for a day. Pruning runs inside the
build jobs: a separate job would wait for a runner of its own, and on a
cancelled run it would hold the PR's concurrency group, delaying the next
push's run. Each job's `SCCACHE_CACHE_SIZE` bounds its snapshot, and each job
prints `sccache --show-stats`. Pixi environment caches are written only from
`main`.

Windows also splits its build, because each MSVC cache miss is expensive. Its
build runs as two parallel jobs, `windows-Release-cpp` (C++ tests) and
`windows-Release-python` (dartpy), both required. Every main push saves a
snapshot of the whole cache. Every same-repository PR run saves only the
objects that PR's runs compiled, even after a failure or cancellation (fork
PRs may not save caches). A PR restores the newest main snapshot plus its own,
so it rebuilds only what changed since its last push. Cache keys include the
MSVC version. Each MSVC
compile spends most of its time parsing headers, so Windows CI builds each
target as unity translation units (`CMAKE_UNITY_BUILD`). The Linux Release and
assertions jobs do too, so a collision also fails on Linux. Keep file-local
names unique within a target for this ([Code Style](code-style.md)).
CTest runs in parallel (`CTEST_PARALLEL_LEVEL`). Nightly-only configurations
never save, so they build cold. Pixi build tasks pin `BUILD_TYPE=Release`, so
a job that needs another build type configures CMake itself, as the assertions
gate does.

## Hardware GPU job

The hosted Linux `CI Filament` job uses software Vulkan (Lavapipe), and Windows
uses software WGL; macOS runs native GUI tests. The
`nvidia-Filament-hardware` job adds real NVIDIA Vulkan on
one self-hosted runner: the headless `test_FilamentBackend`, `test_GuiParity`,
and `test_GuiMesh` CTests, plus a `box_on_ground` `agent-capture` (250 steps,
contacts, collision bounds, auto-view focused on the box) accepted by
`image-verdict`. The
job asserts that `nvidia-smi` lists exactly one GPU and that the capture
sidecar reports `graphics_backend` `vulkan` with a `graphics_device` equal to
that GPU's name, so a silent software fallback fails. This adds NVIDIA driver
coverage; hosted jobs stay the broad-platform and PR gate.

- **Trigger**: `push` to `main` and `workflow_dispatch` on `main` in
  `dartsim/dart`. Pull requests, forks, and release branches never reach it,
  so it cannot be exercised from a PR branch before merge.
- **Server-side restriction**: the runner group `dart-gpu-trusted` must stay
  limited to this repository and to the workflow
  `.github/workflows/ci_filament.yml@refs/heads/main`. The job's `if` is
  defense in depth; labels are not a trust boundary. Keep the restriction when
  renaming or moving the workflow.
- **Label**: `dart-gpu-nvidia` selects the runner and exists only on this
  hardware. One runner means one GPU job at a time; a job-level concurrency
  group retains the newest pending run without interrupting a running GPU job.
- **Limits**: the runner container is unprivileged with 4 CPUs, 16 GiB of
  memory, and 512 PIDs, so the build uses `--parallel 4`. It has no host
  credentials, Docker socket, or desktop session, and the job must not need
  `sudo` or `apt`.
- **Diagnostics**: the `filament-nvidia` artifact holds the
  `nvidia-smi` report, Vulkan environment, capture PNG and sidecar, verdicts,
  and `LastTest.log`.

## Nightly

`nightly.yml` runs every workflow in the index except the
wall-time performance dashboard, lockfile refresh, maintainer approval, and PR
text against `main` each night at 08:17 UTC, including the nightly-only jobs.
It is scheduled directly on `main`, the default branch, with no dispatcher. Run
it on demand with `gh workflow run nightly.yml --ref main`.

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

CI for a PR push or a `main` commit should finish within 30 minutes. A job
that would push it past that, without catching failures the per-PR jobs miss,
runs nightly instead: the VS 2022 baseline, the Filament Windows leg, and the
Windows build-requirement checks (CI Linux runs those on every PR).

Useful commands:

```bash
pixi run lint
pixi run build
pixi run test
pixi run test-py
pixi run test-gui
pixi run -e gazebo test-gz
```

To reproduce the `CI Filament` capture smoke test, run
`pixi run config -- -DDART_ENABLE_GUI_SMOKE_TESTS=ON` before
`pixi run test-gui`. This option defaults to `OFF`, needs a graphics device,
and replaces `DART_ENABLE_GUI_OSG_SMOKE_TESTS`. The GUI task selects
`-L gui -LE filament-native`; run native-window tests separately as described
in [testing guidance](testing.md). Filament lanes build dartpy and the
retained GUI, private-host, and memory-diagnostics test targets. The demo
catalog's cycle and capture coverage comes from Python tests; there is no
C++ demo app or scene-cycle CTest target.

`ci_gz_physics.yml` runs the forward lane, which patches gz-physics before
testing and configures it once, so gz-physics' contact-callback test
expectations are not compiled in. The unpatched Gazebo lanes
(`pixi run gz-compat-ionic` and `gz-compat-jetty` on `main`, plus
`gz-compat-harmonic` on `release-6.20`; see `tools/gazebo/README.md`) are not
in CI yet: each builds DART, gz-physics, and
gz-sim from source and runs a serial suite. Release verification of DART
6.20 at `48eb618bb81` passed all three unpatched gz-physics and gz-sim
compatibility gates, with results equal to or better than DART 6.19.5.
The only gz-physics failure is the accepted ray-intersection expectation.
Run the lanes locally for downstream-sensitive changes and before releases;
[testing guidance](testing.md) covers the sleep oracle and raycast probe.

For failing CI, inspect the exact run and job logs before changing code. Prefer
reproducing locally, but document when a hosted-platform failure cannot be
reproduced on the current machine.
