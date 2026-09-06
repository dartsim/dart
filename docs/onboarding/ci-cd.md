# CI/CD System

## Overview

DART uses GitHub Actions for continuous integration and deployment. The CI system validates code quality, runs tests across multiple platforms, builds documentation, and publishes Python wheels.

## Quickstart

- Start here next time:
  - Local build/test entry points: [building.md](building.md) and [testing.md](testing.md)
  - Gazebo / gz-physics workflow: [build-system.md](build-system.md#gazebo-integration-feature)
  - PR template checklist: [`.github/PULL_REQUEST_TEMPLATE.md`](../../.github/PULL_REQUEST_TEMPLATE.md)
  - Asserts-enabled CI build (no `-DNDEBUG`): see [Asserts-Enabled CI Build](#asserts-enabled-ci-build-no--dndebug)
  - Eigen over-alignment CI build: see [Eigen Over-Alignment CI Build](#eigen-over-alignment-ci-build)
  - ASAN testing (memory errors): `pixi run test-asan`; the Linux `ASAN Tests`
    CI job runs it on protected branch pushes, scheduled runs, and manual
    dispatch. Trigger `CI Linux` via workflow_dispatch on a branch when a PR
    needs ASAN evidence before merge.
  - CI monitoring commands: see [CI Monitoring (CLI)](#ci-monitoring-cli) and [CI Monitoring (API)](#ci-monitoring-api)
  - Common CI failure fixes: see [Common CI Failure Modes](#common-ci-failure-modes)
- Fast CI fail-fast loop:
  - Suggested (Unverified): `gh pr checks <PR_NUMBER> --watch --interval 30 --fail-fast`
  - Suggested (Unverified): `gh run view --job <JOB_ID> --log-failed`
- Gotchas:
  - Feature branch pushes run a core pre-PR tier only; full required coverage
    still runs from `pull_request`, protected branch pushes, release tags,
    schedules, and manual runs.
  - `gh run watch` is blocking and can run for a long time; use a persistent shell and re-run it if your terminal session times out.
  - `gh run view --job <JOB_ID> --log-failed` only works after the job completes; use the REST logs endpoint (or wait) when a run is still in progress.
  - If a PR is not mergeable due to conflicts, CI checks may be blocked or fail early; resolve conflicts locally and push before re-running CI.
  - FreeBSD VM tests can take over 1 hour to complete; this is expected, not a sign of failure.
  - The Linux `ASAN Tests` job is compile-dominated (~2h of instrumented build
    for ~2min of ctest) and runs on protected branch pushes, schedules, and
    manual dispatch rather than on PRs. Check recent successful `main` run
    durations before treating it as stuck.
  - FreeBSD VM startup can timeout (~5 min); this is transient—re-run the job.
  - `dynamic_cast` can fail silently on FreeBSD across shared library boundaries; use type enums + `static_cast`.
  - macOS ARM64 sporadic SEGFAULT from `alloca`/VLA alignment; use `std::vector<T>` instead.
  - macOS arm64 and FreeBSD build with clang `-Werror`, which flags warnings the default Linux/gcc build ignores. `-Wdeprecated-declarations`: wrap deliberate uses of deprecated-but-still-bound APIs (for example pybind shims for deprecated overloads) in `DART_SUPPRESS_DEPRECATED_BEGIN`/`DART_SUPPRESS_DEPRECATED_END` (`dart/common/diagnostics.hpp`). `-Wpotentially-evaluated-expression`: `typeid(*smart_ptr)` evaluates the smart-pointer dereference, and `typeid(*smart_ptr.get())` still warns because the `.get()` call stays inside the operand — first bind the raw pointer to a local (`auto* raw = smart_ptr.get();`) and take `typeid(*raw)`.
  - MSVC does not zero-initialize heap allocations, so an unset struct field (e.g. an Assimp `mNumMaterials`/`mMaterialIndex` left at garbage) can surface as `std::bad_alloc` on Windows only while Linux/gcc happens to read zeroed memory. Initialize every field explicitly instead of relying on zeroed heap memory.
  - Example reorganizations can conflict in `examples/CMakeLists.txt`; reconcile any new example entries and keep the category layout aligned before pushing.
  - GitHub Actions API calls can return `HTTP 406` if you omit required headers; include an explicit `Accept` header.
  - `gh api` writes to stdout and does not support `--output`; redirect to a file when you need to search logs.
  - Windows CI and Windows wheels are pinned to `windows-2025-vs2026` because
    DART's Windows floor is Visual Studio 2026 and the Pixi tasks default to
    CMake's `Visual Studio 18 2026` generator. Do not move them back to
    `windows-latest` unless the generator and MSVC compatibility assumptions are
    updated together.
  - The asserts-enabled CI job uses a custom CMake configure (`CMAKE_BUILD_TYPE=None`) instead of pixi tasks; keep native-only collision toggles explicit unless the job also installs a reference-engine Pixi environment.
  - The Eigen over-alignment job forces `EIGEN_MAX_ALIGN_BYTES=64` and `EIGEN_MAX_STATIC_ALIGN_BYTES=64`; failures usually indicate allocator, placement-new, or storage code assuming a smaller Eigen alignment.
  - Deprecated headers that emit `#warning` fail under `-Werror=cpp` (e.g., use `dart/io/urdf/All.hpp` instead of deprecated `dart/io/urdf/urdf.hpp`).
  - dartpy test failures can show up as a Python abort with minimal traceback when a C++ `DART_ASSERT` triggers; rerun the single test locally and inspect the C++ assert.
  - Reference collision raycast tests require the relevant reference target and dependency to be available; skip those checks or use the `collision-reference` environment when FCL/Bullet/ODE are intentionally disabled in a native-only build.
  - `gh pr status --json ...` can error with `Unknown JSON field: ...` if you request unsupported fields; use `gh pr status` (no JSON) or `gh pr view --json ...`.
  - `gh run view --json ...` only accepts supported fields; prefer the `url` field instead of guessing `htmlURL`.
  - `gh run rerun --job` expects the job `databaseId` (not the numeric ID from the job URL); if `gh run view --json jobs` shows `id: null`, use `databaseId`. Suggested (Unverified): `gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'`.
  - Job log endpoints can return `log not found`/404 even after a failure; fall back to the run-level logs archive or re-run the job (see CI Monitoring (API)).
  - Review comment metadata is not exposed by `gh pr view --json`; Suggested (Unverified): `gh api /repos/<OWNER>/<REPO>/pulls/comments/<COMMENT_ID>`.
  - `gh pr checks` may show duplicate entries from older runs or workflows that still use both `push` and `pull_request`; compare the run URLs and focus on the newest one.
  - Newer runs can cancel older ones; confirm the run status/conclusion before spending time on job logs.
  - Workflow concurrency is per workflow/ref, not per commit. A queued or
    rerun workflow from an old head can keep the current head's workflow
    pending, especially for wheel publishing. Compare run `headSha` values for
    the branch, cancel stale old-head runs only after explicit approval, and use
    GitHub's force-cancel endpoint only when a normal cancel request leaves a
    stale queued run blocking the current head.
- zsh can produce parse errors when jq expressions or backtick characters are not fully quoted; quote `gh ... --jq` programs and use a here-doc or `--body-file` when PR bodies include backticks.
  - If `CI gz-physics` fails, reproduce locally with the Gazebo workflow in [build-system.md](build-system.md#gazebo-integration-feature).
  - CI jobs can sit in the queue for a long time; re-check the run list and wait for the PR run to start before assuming a failure.
  - Wheel publishing workflows may lag behind other jobs and stay queued longer; keep watching the PR run until all workflows complete.
  - Alt Linux bootstrap failures while fetching packages from `ftp.altlinux.org` are usually mirror/network flakes; the bootstrap retries package installation in-place, and repeated failures should be rerun after the mirror recovers.
  - The Alt Linux Docker job has a larger timeout than most PR jobs because
    Sisyphus metadata/package fetches can be slow enough to consume most of a
    two-hour runner window before the DART build starts.
  - Randomized stress tests can diverge across platforms if they rely on library-dependent distributions; prefer deterministic RNG transforms when portability matters.
  - `check-format` failures usually mean formatting drift; run the C++ formatter and commit any diffs before retrying CI. Suggested (Unverified): `pixi run lint-cpp`.
  - Local lint may fail if clang-format is missing or a stale CMake cache references an old version; clean the build directory (`rm -rf build/`) and reconfigure to pick up the pixi-provided clang-format.
  - Codecov patch failures usually mean new lines or branches are uncovered; add targeted tests and re-run coverage.
  - Codecov patch status can lag until coverage jobs complete; confirm Coverage (Debug) finished before acting.
  - Codecov `codecov/patch` is the required changed-line coverage gate. The
    aggregate `codecov/project` status reports full-repository drift without
    blocking PRs because the Debug coverage lane is intentionally bounded and
    capture-pruned; investigate aggregate drops as trend signals when patch and
    relevant component statuses are green.
  - CodeQL alerts under generated dependency paths like `build/**/_deps` need
    SARIF filtering before upload; CodeQL config `paths-ignore` is not enough.
    Confirm Code Scanning has processed the relevant C++ analysis for the target
    commit before treating the open-alert count as final.
  - Benchmark runners (`scripts/run_cpp_benchmark.py`,
    `scripts/run_performance_dashboard_benchmarks.py`) build a target but do not
    configure the tree. Run `pixi run config` (or a pixi task with
    `depends-on = ["config"]`) first; on a fresh checkout or CI runner they
    otherwise fail with `Build directory build/<env>/cpp/<type> does not exist`.
  - `gh-pages` stores generated static surfaces such as `/performance/` and
    `/community-signals/`. `.github/workflows/pages_deploy.yml` packages that
    branch after successful dashboard publisher runs and deploys it as a Pages
    artifact; if the repository Pages source is still configured for legacy
    branch publishing, GitHub's generated `pages-build-deployment` workflow may
    queue behind hosted runners or cancel older branch builds while `gh-pages`
    is moving.

## Common CI Failure Modes

- Formatting checks fail: run the C++ formatting task and re-run CI. Suggested (Unverified): `pixi run lint-cpp`.
- Linux `ASAN Tests` compile fails with `No space left on device`: treat this
  as hosted-runner disk exhaustion, not a failing DART test. The instrumented
  ASAN tree is large; keep the disk cleanup step at the start of the job, then
  rerun the failed check.
- Codecov patch failures: add targeted coverage for new lines or branches.
- Example builds fail because sample code references removed formats or enums; update the example to match the current API (e.g., `dart::io::ModelFormat`).
- Example or test links fail with `cannot find -ldart-<component>`: inspect the
  component target output directories, especially for optional components
  declared outside the core `dart/` subtree. In-tree consumers expect linked
  DART libraries under `${DART_BINARY_DIR}/lib`; see
  [build-system.md](build-system.md#optional-component-output-directories).
- Example links fail with a missing optional DART library after a prerequisite
  target was skipped: confirm the example only registers itself when every
  required CMake target exists. Target guards belong before
  `dart_add_example()` or `dart_build_gui_example()` registers the executable.
- Unit test crashes or segfaults: isolate the failing test from job logs, reproduce locally, and add a regression for the edge case.
- Job logs are missing or return 404: re-run the single job and/or download the run-level logs archive to inspect failures.
- Infrastructure failures (self-hosted runner "lost communication"): the
  `dartsim-mark*` self-hosted runners advertise the `ubuntu-latest` label as
  parity replacements for GitHub-hosted runners, so each one must be capped to a
  GitHub-hosted envelope — **4 vCPU / 16 GB** per runner — and the total kept
  within host capacity with headroom. An uncapped runner on an oversubscribed
  host starves its `Runner.Listener` heartbeat, and GitHub fails the job with
  "The self-hosted runner lost communication with the server" — a host-resource
  issue, not a code defect. Tell-tale: the job logs contain only the setup steps
  because the failing step (e.g. `Build wheel`) died mid-run and never produced a
  log. Re-run once the host is calm (`gh run rerun <RUN_ID> --failed`) and make
  sure every runner is resource-capped per the runner-setup gist
  (https://gist.github.com/jslee02/c0a6b0e4af18e4ecb2f2e8fb5b715f78).
- FreeBSD RTTI failures: `dynamic_cast` across shared library boundaries can fail silently; use type enums + `static_cast` instead.
- macOS ARM64 sporadic SEGFAULT: `alloca()` or VLAs may cause alignment violations; use `std::vector<T>` for proper alignment.
- RTD build failures: Sphinx extension compatibility issues; use defensive `.get(key, default)` patterns.
- Case-colliding files after branch merge: When merging `release-*` into `main`, PascalCase files from the release branch can collide with snake_case files in main on case-insensitive filesystems (macOS, Windows). Check for duplicates with `find tests -name "*.cpp" | sort -f | uniq -di` and remove the PascalCase version. Also verify new tests are registered in the appropriate `CMakeLists.txt`.
- Eigen over-alignment failures: reproduce with `pixi run test-eigen-overalignment`. These failures do not require AVX-512 hardware; the job forces Eigen's 64-byte static alignment contract at compile time.
- CodeQL generated dependency alerts: `paths-ignore` can miss findings emitted
  from C++ manual-build SARIF, especially under generated dependency build
  trees like `build/**/_deps`. Keep CodeQL analysis limited to first-party
  sources, drop SARIF results whose primary location is under generated or
  vendored paths before upload, confirm the Code Scanning analyses include the
  target commit and language, then require the open-alert count to reach zero.

## Fast Iteration Loop

- Identify the first failing step in the CI job log, then reproduce locally with the same build toggles.
- Run the smallest failing test or target, then push and re-run CI.
- If the failure is in the examples build, fix the first compile error in the example source before re-running CI.
- If the failure is formatting-related, run the C++ formatter before retrying CI.
- Success signal: the failing job completes without `-Werror` compile failures or Python aborts.

Suggested (Unverified):

```bash
gh run view <RUN_ID> --json status,conclusion,url
gh run view <RUN_ID> --job <JOB_ID> --log-failed
gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'
gh run rerun <RUN_ID> --job <DATABASE_ID>
pixi run lint-cpp
ctest --test-dir <BUILD_DIR> -R <TEST>
gh api -H "Accept: application/vnd.github+json" /repos/<OWNER>/<REPO>/actions/jobs/<JOB_ID>/logs > /tmp/<JOB_ID>.log
rg -n "FAILED|SegFault|Exception|\\bError\\b|✗" /tmp/<JOB_ID>.log
python -m pytest <TEST_PATH>::<TEST_NAME> -vv
```

## CI Monitoring (CLI)

Use the GitHub CLI to locate the latest run for your branch and watch it to completion.

Suggested (Unverified):

```bash
gh pr checks <PR_NUMBER>
gh run list --repo <OWNER>/<REPO> --branch <BRANCH> --limit <N>
gh run watch <RUN_ID> --interval 30 --repo <OWNER>/<REPO>
gh run view <RUN_ID> --json status,conclusion,updatedAt,url
gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'
gh run rerun <RUN_ID> --job <DATABASE_ID>
```

## CI Monitoring (API)

If the GitHub CLI is unavailable, use the GitHub Actions REST API to poll runs and job steps.

Suggested (Unverified):

```bash
python - <<'PY'
import json
import urllib.request

branch = "<BRANCH>"
url = f"https://api.github.com/repos/<OWNER>/<REPO>/actions/runs?branch={branch}&per_page=100"
headers = {"Accept": "application/vnd.github+json", "User-Agent": "codex-cli"}
req = urllib.request.Request(url, headers=headers)
with urllib.request.urlopen(req) as resp:
    data = json.load(resp)
for run in data.get("workflow_runs", []):
    print(run["name"], run["status"], run["conclusion"], run["html_url"])
PY
```

Suggested (Unverified):

```bash
gh api -H "Accept: application/vnd.github+json" /repos/<OWNER>/<REPO>/actions/runs/<RUN_ID>/logs > /tmp/<RUN_ID>.zip
unzip -q /tmp/<RUN_ID>.zip -d /tmp/<RUN_ID>
rg -n "FAILED|SegFault|Exception|\\bError\\b|✗" /tmp/<RUN_ID>
```

## Asserts-Enabled CI Build (no -DNDEBUG)

The asserts-enabled job uses a custom CMake configure with `CMAKE_BUILD_TYPE=None`
to keep assertions enabled outside a Debug build.

- Pass build toggles explicitly when bypassing pixi tasks. The default
  asserts-enabled job should keep FCL/Bullet/ODE and reference collision
  harnesses `OFF`; enable them only in a Pixi environment that installs those
  reference engines.
- Expect `-Werror=cpp`; any deprecated headers that emit `#warning` will fail the build.
- If a dartpy test aborts without a Python traceback, the C++ assert message is usually the first useful clue.

## Eigen Over-Alignment CI Build

The Eigen over-alignment job runs on Linux PRs and scheduled builds via
`pixi run test-eigen-overalignment`.

It configures a dedicated `eigen64-align` CMake tree with
`EIGEN_MAX_ALIGN_BYTES=64` and `EIGEN_MAX_STATIC_ALIGN_BYTES=64`, then builds and
runs the non-experimental C++ tests in a reduced no-GUI/no-dartpy configuration.
This catches allocator, placement-new, object-pool, and Eigen storage mistakes
that otherwise only appear on AVX-512-capable compiler/CPU combinations.

Suggested (Unverified):

```bash
DART_PARALLEL_JOBS=8 CTEST_PARALLEL_LEVEL=8 pixi run test-eigen-overalignment
```

## Next-Time Accelerators

- When running dartpy tests against an in-tree build, set `PYTHONPATH` and `DARTPY_RUNTIME_DIR` to the build output.
- If Ubuntu Debug Python tests appear stuck at the final dartpy module link,
  inspect the generated `build.ninja` edge before assuming pytest started. The
  Debug binding-layer symbol policy is documented in
  [python-bindings.md](python-bindings.md#debug-build-symbols).
- If the Debug Python step has already entered pytest but stays quiet, use the
  direct pytest isolation workflow in [testing.md](testing.md#next-time-accelerators)
  to identify the slow file or test.
- If a test requires an optional backend, guard it (skip) or ensure the backend toggle is enabled in the build configuration.
- If editor or IDE context references a path, verify it exists before making edits or writing guidance.
- Re-run only the failed CI job (via job databaseId) to keep feedback loops short.

## Workflow Architecture

### Core CI Workflows

| Workflow                      | Purpose                     | Platforms                   | Trigger                                                            | Doc-only skip      |
| ----------------------------- | --------------------------- | --------------------------- | ------------------------------------------------------------------ | ------------------ |
| `ci_lint.yml`                 | Lint + docs build           | Ubuntu                      | Any branch push, PR, manual                                        | No                 |
| `ci_ubuntu.yml`               | Build, test, coverage       | Ubuntu                      | Branch push core; PR/main full                                     | Yes                |
| `ci_macos.yml`                | Build, test                 | macOS                       | PR, main/release push, schedule                                    | Yes                |
| `ci_windows.yml`              | Build, test; hook smoke     | Windows                     | PR, main/release push, schedule; hook smoke when hook paths change | Yes                |
| `ci_freebsd.yml`              | Build, test (VM)            | FreeBSD                     | Schedule, manual                                                   | N/A                |
| `ci_altlinux.yml`             | Build, test (Docker)        | Alt Linux                   | Schedule, manual                                                   | N/A                |
| `ci_cuda.yml`                 | CUDA compile + smoke        | Ubuntu/GPU                  | Path-scoped PR; trusted GPU runtime                                | N/A                |
| `ci_gz_physics.yml`           | Gazebo integration          | Ubuntu                      | Release-branch push/PR; manual canary                              | Yes                |
| `ci_gz_dart6.yml`             | DART 6 Gazebo canary        | Ubuntu                      | Weekly schedule + manual (on main)                                 | N/A                |
| `ci_simd.yml`                 | SIMD multi-arch             | Ubuntu + macOS arm64 (NEON) | Branch/PR path-scoped, manual                                      | N/A                |
| `publish_dartpy.yml`          | Python wheels               | Multi-platform              | PR, main/release/tag push, schedule                                | Yes                |
| `codeql.yml`                  | Static security analysis    | Ubuntu                      | `main` push/PR (code paths), schedule, manual                      | Yes (path filters) |
| `benchmark_pr_comparison.yml` | Benchmark PR comparison     | Ubuntu                      | PR (label-gated)                                                   | N/A                |
| `performance_dashboard.yml`   | Performance dashboard       | Ubuntu                      | `main` push (path-scoped), schedule, manual                        | N/A                |
| `community_signals.yml`       | Community signals dashboard | Self-hosted Linux           | `main` push (path-scoped), schedule, manual                        | N/A                |
| `pages_deploy.yml`            | GitHub Pages deploy         | Self-hosted Linux           | Workflow run, manual                                               | N/A                |
| `update_lockfiles.yml`        | Pixi lockfile refresh PRs   | Ubuntu                      | Schedule, manual                                                   | N/A                |
| `cancel_branch_jobs.yml`      | Cancel superseded runs      | Ubuntu                      | PR closure, branch delete                                          | N/A                |

### CI Tiering Policy

Use CI tiers to reduce PR feedback cost without removing coverage from the
project's continuous validation surface.

| Tier                      | Required before merge | Examples                                                                                                                        |
| ------------------------- | --------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Core branch push          | No                    | Lint, Ubuntu core release tests (skipped once the branch has an open PR), path-scoped SIMD                                      |
| Required PR               | Yes                   | Lint/docs, core Linux, macOS, Windows, baseline dartpy wheels                                                                   |
| Conditional PR            | When affected         | SIMD-only CI, CUDA compile CI, path-filtered platform jobs for code changes                                                     |
| Release support PR        | Yes on release lines  | gz-physics compatibility on active DART 6 LTS PRs                                                                               |
| Main/release continuous   | After merge           | Full platform coverage on protected branches; full wheels on `main` and release tags; ASAN suite; CodeQL C++ analysis           |
| Scheduled/manual coverage | No                    | FreeBSD VM, Alt Linux repro, ASAN suite, CUDA packet benchmarks, gz-physics migration canaries, repeated full matrix, lockfiles |

Guardrails:

- Keep gz-physics compatibility required for release-line PRs that maintain the
  DART 6 support lane. On `main`/DART 7, use `ci_gz_physics.yml` as a manual
  migration canary when downstream compatibility evidence is needed.
- GitHub only fires `schedule:` triggers from the default branch, so cron
  entries inside `release-6.*` workflows never run. `ci_gz_dart6.yml` on main
  fills that gap: it checks out each active DART 6 release branch weekly and
  runs the branch's own `test-gz` (gz-physics + gz-sim from source) so
  downstream drift is caught between release-line merges.
- Treat core branch-push CI as early feedback only. It should be useful enough
  before a PR exists, but it is not a substitute for the required PR tier.
- Do not move a job from required PR coverage to continuous-only coverage
  without evidence that it is expensive, redundant for most PRs, or better
  suited to scheduled validation.
- The ASAN suite, CodeQL C++ analysis, and Alt Linux repro moved off the PR
  tier in July 2026 with that evidence recorded: ASAN spends ~2h of
  instrumented compile for ~2min of ctest, CodeQL C++ repeats a ~100min manual
  build per PR push, and distro-repro breakage is effectively never PR-local.
  All three keep running on `main` pushes and/or schedules; use
  workflow_dispatch on a branch when a PR needs that evidence before merge.
- Keep at least one dartpy wheel per supported OS in PR CI. Expanded Python
  version coverage can run on `main`, release tags, schedules, and manual
  dispatch.
- For hardware-gated accelerator paths, split trusted runtime coverage from
  untrusted fork coverage. Same-repository PRs, protected branch pushes, and
  manual dispatches may use the `ubuntu-latest-gpu` runner for CUDA runtime
  tests. Fork PRs must stay on a GitHub-hosted runner and compile the CUDA
  targets without running untrusted code on the long-lived GPU runner. If a
  hardware job remains queued, check runner labels before treating the queue as
  a code failure.
- Run the full dartpy wheel matrix when `publish_dartpy.yml` itself changes so
  workflow edits validate both PR and continuous wheel tiers before merge.
- Require stable aggregate check names for variable CI matrices. For dartpy
  wheels, branch protection should require `Wheels`; individual wheel legs may
  change by tier and should not be required directly.
- `main`, release, release-tag, scheduled, and manual workflows must preserve
  the full coverage that PR tiering skips.

### Design Principles

**Optimize for fast feedback on branches and PRs:**

- Feature branch pushes run a small core tier so contributors can get useful CI
  signal before opening a PR.
- Essential validations run on every PR
- Full matrix testing runs on main branch and releases
- Debug C++ builds run on every PR on Linux and macOS; Windows stays
  Release-only (see Conditional Execution Patterns)
- Keep gz-physics compatibility in the release-support PR tier so downstream
  Gazebo integration breakages are caught before release-line merges. For
  `main` PRs, run the manual gz-physics canary only when the change is relevant
  to downstream migration evidence.

**Efficient resource usage:**

- Compilation caching (sccache) reduces build time by 50-70%
- Path filtering prevents unnecessary workflow runs
- Conditional execution skips non-essential jobs on PRs
- Concurrency groups cancel superseded runs (see below)

**Concurrency configuration:**

CI workflows cancel superseded branch and PR validations, including protected
`main` and `release-*` branch pushes. Branch protection only needs the latest
commit on a ref; preserving obsolete post-merge validations can starve current
PR checks after a burst of merges. Durable evidence runs are still preserved:

- **scheduled jobs**: preserve every periodic validation run
- **manual dispatches and release tags**: preserve each requested validation run

GitHub's default concurrency queue keeps only one pending run per group. Shared
workflows therefore use stable groups for branch and PR validations so newer
runs cancel older pending or active work, and run-specific groups only for
durable schedule/manual/tag evidence.

```yaml
# Standard pattern for workflows that cancel branch/PR validations but preserve
# scheduled, manual, and release-tag evidence.
concurrency:
  group: >-
    ${{ (github.event_name == 'schedule'
        || github.event_name == 'workflow_dispatch'
        || startsWith(github.ref, 'refs/tags/v'))
        && format('{0}-{1}-{2}-{3}', github.workflow, github.ref, github.event_name, github.run_id)
        || format('{0}-{1}-{2}', github.workflow, github.ref, github.event_name) }}
  cancel-in-progress: >-
    ${{ github.event_name != 'schedule'
        && github.event_name != 'workflow_dispatch'
        && !startsWith(github.ref, 'refs/tags/v') }}

# For workflows with matrix jobs that serialize writes to fixed branches
# (e.g., update_lockfiles) or jobs that use a fixed external resource
# (e.g., the Docker-backed FreeBSD VM container), keep a stable group and
# request the full queue.
concurrency:
  group: ${{ github.workflow }}-${{ matrix.base }}
  queue: max
  cancel-in-progress: false
```

**Maintain full test coverage:**

- All tests run on at least one platform per PR
- Complete platform matrix on main branch
- Scheduled runs ensure periodic full validation

## Compilation Caching Strategy

### Why Caching Matters

DART compilation takes 15-25 minutes per build without caching. With proper caching:

- **First build**: Normal compilation time (populates cache)
- **Subsequent builds**: 50-70% faster (only changed files recompile)

### Compiler cache (sccache + ccache)

We standardized on sccache everywhere and automatically fall back to ccache when
it is the only launcher available. The detection logic lives in
`cmake/compiler_cache.cmake`, so **plain CMake invocations, pixi tasks, and CI
jobs all share the same configuration**. You can disable auto-detection with
`-DDART_DISABLE_COMPILER_CACHE=ON` or force a specific launcher via the
`DART_COMPILER_CACHE` cache variable/environment variable.

CI wires the launcher through two composite actions on Linux and macOS jobs
and on every wheel build (the Windows wheel configures with Ninja through
scikit-build-core, so the launcher applies there):

```yaml
- name: Setup pixi (CI) # installs pixi and mozilla-actions/sccache-action
  uses: ./.github/actions/setup-pixi-ci
  with:
    pixi-bin-path: ${{ runner.temp }}/pixi/bin/pixi

- name: Configure environment for compiler cache
  uses: ./.github/actions/configure-compiler-cache
```

`configure-compiler-cache` selects the launcher, exports
`DART_COMPILER_CACHE` and `CMAKE_{C,CXX}_COMPILER_LAUNCHER`, writes a
"Compiler cache" block to the job step summary (launcher, backend
availability, runner), and **fails the job on a GitHub-hosted runner when no
launcher could be configured** (`require-cache: "false"` opts a job out
deliberately). The GitHub Actions cache backend is detected through
`ACTIONS_RESULTS_URL` (cache service v2) or the legacy `ACTIONS_CACHE_URL`.

Self-hosted runners (the GPU runner) skip sccache and use a persistent ccache
directory under the runner tool cache instead; see the action for the
rationale.

The Windows `Tests (Release)` job is the exception: the pixi Windows tasks
configure a multi-config Visual Studio tree, and MSBuild ignores
`CMAKE_<LANG>_COMPILER_LAUNCHER` (CMake honors it for Makefile and Ninja
generators only), so that job still builds uncached (~100 min). Caching it
needs either a Ninja configure under a Visual Studio developer environment
or sccache's MSBuild integration (`CMAKE_VS_GLOBALS` with
`CLToolExe`/`CLToolPath`); the follow-up is tracked in
`docs/dev_tasks/ci-optimization/README.md`.

> **Regression history (May–September 2026):** the action gated sccache on
> `ACTIONS_CACHE_URL` only. GitHub's cache-service v2 migration removed that
> variable, so every hosted job logged `GitHub Actions cache URL is
unavailable; disabling sccache launcher.` and built cold for months while
> the workflows still read as cached. The fail-loud guard above exists so
> this cannot recur silently; when a job fails at "Configure environment for
> compiler cache", fix the cache setup rather than disabling the guard.

**Local builds:** Because the detection logic is inside CMake, you do not need
to wire anything up manually. If either `sccache` or `ccache` is on your PATH,
DART will automatically set `CMAKE_*_COMPILER_LAUNCHER` when you run `cmake`
directly or via `pixi`. pixi still forwards `CMAKE_*_COMPILER_LAUNCHER` to any
external CMake projects it drives (e.g., gz-physics) so nested builds benefit
from the same cache. CUDA-enabled dartpy Pixi builds keep
`CMAKE_CUDA_COMPILER_LAUNCHER` empty by default because of nvcc/fatbinary PTX
issues, but still allow C/CXX launchers unless
`DART_DISABLE_COMPILER_CACHE=ON` is set.

### Cache budget

GitHub gives the repository 10 GB of Actions cache with least-recently-used
eviction (entries unused for 7 days are also dropped). Compiler objects are
the only thing that earns that budget:

- **sccache objects**: the whole budget. PR runs (including fork PRs) read
  the base branch's entries and write only to their own pull-request scope;
  nothing a PR run writes reaches the `main` scope.
- **Not cached on purpose**: pixi environments (`setup-pixi-ci` defaults
  `cache: "false"`; `pixi install --locked` takes well under a minute and the
  entries were ~500–600 MB per platform) and CodeQL C++ TRAP databases
  (`trap-caching: false`; two ~3 GB entries once consumed 6 GB).
- apt package caches stay (a few MB).

Check usage with `gh api repos/dartsim/dart/actions/cache/usage` and the
largest entries with
`gh api "repos/dartsim/dart/actions/caches?sort=size_in_bytes&direction=desc"`.
If a new cache consumer is added, record it here and keep sccache first.

#### CI reliability notes

- The `.github/actions/configure-compiler-cache` action disables the sccache launcher on self-hosted or non-Ubuntu Linux runners and falls back to `ccache` when available (otherwise `env`) to avoid flaky `try_compile` failures; on GitHub-hosted runners it fails the job instead of building cold.
- In `.github/workflows/ci_macos.yml`, the "Setup sccache" step is best-effort (`sccache-continue-on-error: true`) so transient download timeouts don't fail the job; the compiler-cache guard then reports the missing launcher.

## MSVC Toolchain Policy

MSVC settings are centralized in `cmake/dart_defs.cmake` and invoked from the
root `CMakeLists.txt`. Keep new Windows compiler policy in those helpers
instead of appending to `CMAKE_CXX_FLAGS`.

The helpers own:

- runtime selection through `dart_configure_msvc_runtime_library()` and
  `CMAKE_MSVC_RUNTIME_LIBRARY`; this runs before dependency discovery so targets
  created during configure inherit the same runtime model
- compile-option policy through `dart_configure_msvc_toolchain()`; this also
  runs before dependency discovery so dependency-created targets inherit DART's
  MSVC directory options
- standard conformance and encoding options (`/EHsc`, `/permissive-`,
  `/Zc:twoPhase-`, `/utf-8`)
- multi-core compilation (`/MP` plus `/FS`)
- DART's documented warning suppressions and warnings-as-errors wiring
- Release/Debug tuning when `DART_MSVC_DEFAULT_OPTIONS=OFF`

**Two levels of parallelization:**

1. **Intra-target parallelization** (`/MP` flag):
   - Multiple cores compile within a single target
   - MSVC automatically uses all available CPU cores
   - Significantly speeds up compilation of large source files

2. **Inter-target parallelization** (`--parallel` flag):
   - Multiple targets build simultaneously
   - Configured in `pixi.toml`: `cmake --build build/... --parallel`
   - Visual Studio generator coordinates parallel target builds

**Important:** This is already optimally configured. Do not add redundant parallelization flags.

## Conditional Execution Patterns

### Debug Builds

Debug builds are slower, but Ubuntu keeps explicit Debug C++ and dartpy test
jobs in PR CI because they catch configuration-specific failures. macOS keeps
Debug C++ coverage, while dartpy is covered by macOS Release and Ubuntu Debug;
macOS arm64 Debug dartpy builds are too slow for PR feedback on GitHub-hosted
runners. Windows keeps Release-only tests to keep runtime acceptable.

**Behavior:**

- **PRs**: Release and Debug jobs run on Ubuntu/macOS when code paths change
- **Windows**: Release-only tests run on PRs, protected branch pushes,
  schedules, and manual dispatch
- **Doc-only changes**: Platform jobs are skipped by path filters; lint/docs still run

### Python Wheel Builds

Python 3.14 is the only supported dartpy interpreter, so `publish_dartpy.yml`
runs one static matrix (Ubuntu, macOS 15, Windows) for PRs, protected-branch
pushes, release tags, schedules, and manual dispatch. The `Wheels` aggregate
job is the stable required check; individual legs are not required directly.

When a second Python version is supported again, reintroduce the two-tier
matrix: PRs build the baseline interpreter per OS, while `main`, tags,
schedules, manual runs, and PRs that edit `publish_dartpy.yml` build the full
Python-version matrix (May 2026 measurement: six non-baseline legs cost
146.6 job-minutes per PR run).

**Behavior:**

- **PRs, main, tags, schedules, manual runs**: Ubuntu, macOS, and Windows wheels
- **Feature branch pushes**: Wheel builds are skipped by the core branch-push tier
- **Doc-only changes**: skipped by the shared code filter; `Wheels` still reports green

### Documentation Builds

Read the Docs owns documentation publishing. GitHub Actions validates that the
RTD-style documentation still renders once in `ci_lint.yml`, but it does not
deploy the generated site:

- Run `pixi run docs-build` to render the RTD site (including the C++ Doxygen
  bundle) locally.
- Committers can optionally run `pixi run api-docs-cpp` or `pixi run api-docs-py`
  if they need to inspect the standalone builders.
- RTD rebuilds automatically whenever `main` changes, keeping the hosted docs in
  sync without deploying from GitHub Actions.

### GitHub Pages Surfaces

GitHub Pages serves generated static dashboards and historical Doxygen snapshots
from the `gh-pages` branch:

- `/performance/` is updated by `.github/workflows/performance_dashboard.yml`.
- `/community-signals/` is updated by `.github/workflows/community_signals.yml`.
- Historical `/v6.*` Doxygen snapshots live on the same branch.

Community Signals refreshes weekly, on manual dispatch, and when files that own
the dashboard change. It does not run for unrelated commits to `main`.

The `gh-pages` branch is the storage surface, but deployment is owned by
`.github/workflows/pages_deploy.yml`: after the Community Signals or
Performance Dashboard publisher completes successfully, it checks out the
current branch tip, verifies the expected entry points, uploads the whole tree
as a GitHub Pages artifact, and deploys it through `actions/deploy-pages`.
The workflow also supports manual dispatch for exceptional historical-docs
updates. It uses a single `github-pages-deploy` concurrency group across
packaging and deployment, so rapid dashboard updates converge on the latest
`gh-pages` tip. Only successful publisher runs and manual dispatches can cancel
stale deployments; failed publisher runs use a run-specific no-op concurrency
group, skip packaging, and cannot cancel or replace a pending successful
deployment.

Repository Pages settings must use the GitHub Actions source for the deploy
workflow to own production. If the source is still "Deploy from a branch",
GitHub also creates its legacy `pages-build-deployment` workflow on every
`gh-pages` push; treat queued or cancelled legacy deployments as stale unless
the custom `Deploy GitHub Pages` workflow for the current `gh-pages` tip also
fails.

## Lint Check Strategy

**Dedicated workflow:** Linting is deterministic and platform-independent, so it
runs once in `ci_lint.yml`. The same workflow also runs one documentation build
so platform test jobs do not each rebuild docs.

**Key design:**

- `ci_lint.yml` runs lint on all branch pushes and PRs, including doc-only
  changes. The documentation build runs on PRs, protected branch pushes, and
  manual dispatch, but is skipped for ordinary feature branch pushes to keep
  the pre-PR tier small.
- FreeBSD CI (`ci_freebsd.yml`) and Alt Linux CI (`ci_altlinux.yml`) run on
  schedule/manual only to keep PR feedback fast; distro-repro breakage is
  effectively never PR-local
- Lint is removed from platform-specific workflows since it's covered by the dedicated job

**Shared code-change filter:** platform and integration workflows that use
`dorny/paths-filter` read `.github/filters/ci-code.yml` for their `changes`
job. Keep non-code skip policy there instead of copying path lists between
workflows.

```yaml
code:
  - "**"
  - "!.claude/**"
  - "!docs/**"
  - "!CHANGELOG.md"
```

```yaml
- uses: dorny/paths-filter@v4
  id: filter
  with:
    filters: .github/filters/ci-code.yml
    # Required so the `!` exclusions act as excludes.
    predicate-quantifier: some-with-excludes
```

The quantifier is not optional: under the action's default `some`, the
patterns are OR-ed and a negated pattern matches everything that is _not_
excluded, so `code` is always true. That is what happened from May to
September 2026, when every documentation-only and AI-harness PR still ran the
full platform and wheel matrix. `tests/test_ci_code_filter.py` (part of
`pixi run test-ai-infra`) fails when a workflow reads the filter without the
quantifier or when a literal exclusion no longer exists.

The filter excludes the AI tool surfaces (`.agents/`, `.claude/`, `.codex/`,
`.opencode/`), the documentation trees and root prose files, scheduled-only
and maintenance workflows, the distro container harnesses, repository
metadata that nothing builds from, and the AI-infrastructure Python files
that only `pixi run test-ai-infra` exercises. Markdown is excluded by tree or
by name, never with a blanket `*.md` glob: READMEs inside source trees are
build and test inputs (`pyproject.toml` embeds the root `README.md` as the
wheel's long description; the Python tests parse
`python/examples/demos/README.md`). Other inputs that look like metadata but
feed a build stay in: `package.xml` (CMake and `pyproject.toml` read the
version), `LICENSE` (packaged into the wheel), `tutorials/` (installed by the
root `CMakeLists.txt` into the package tree, like `data/` and `examples/`),
`codecov.yml`, `.gitattributes`, and `.gitignore`. Agent hook changes
(`.claude/hooks/**`) run the Windows hook smoke job through its own `hooks`
filter in `ci_windows.yml`.

Coverage guardrails:

- Do not add broad `.github/workflows/**` exclusions. Changes to build, test,
  packaging, and CI orchestration workflows should continue to run heavy CI.
- For path-scoped accelerator or backend workflows, include the build and test
  registration files that enable the path, not just the implementation
  directory. For example, a SIMD workflow needs the root and module
  `CMakeLists.txt` files plus benchmark/test registration files that can add or
  remove SIMD-covered targets.
- Keep `.github/filters/ci-code.yml` itself covered by the filter so changes to
  the trigger policy run the normal matrix.
- Prefer job-level `changes` gates over top-level `pull_request.paths-ignore`
  for required platform workflows so branch protection still receives stable
  workflow/check contexts.
- Use native `paths-ignore` only for workflows that cannot share the dorny
  filter file, such as CodeQL trigger configuration.

**Savings:** a documentation- or AI-harness-only PR runs Lint,
Documentation, the one-minute `changes` jobs, and the `Wheels` aggregate
(plus the CodeQL Python analysis for AI-harness changes, which `codeql.yml`'s
own `paths-ignore` does not cover) instead of the ~27-job platform and wheel
matrix; every required check reports skipped or green.

## Testing Strategy

### Test Coverage

**Per PR:**

- Ubuntu Release: Full tests + coverage
- Ubuntu Debug: Debug C++ and dartpy tests
- macOS Release: Full tests
- macOS Debug: Debug C++ tests
- Windows Release: Full tests
- Gazebo integration: required on release-line PRs; manual canary on `main`
  when downstream migration evidence is needed
- dartpy wheels: Baseline Python version on Ubuntu, macOS, and Windows

**Feature branch pushes:**

- Lint
- Ubuntu core release path: Release C++ tests, Release Python tests, examples,
  and install. Skipped once the branch has an open PR, because the
  pull_request event already runs the full tier for every push.
- SIMD multi-arch tests when SIMD paths change

**Release branch pushes and PRs:**

- Gazebo integration: gz-physics compatibility tests for the DART 6 support lane

**Main/release pushes and scheduled runs:**

- Repeat the PR validation on a fixed cadence
- Run the ASAN suite (`ASAN Tests`), CodeQL C++ analysis, and Alt Linux repro
- Ensure periodic full validation
- Run expanded dartpy wheel Python-version coverage

### Test Execution

GitHub Actions uses granular pixi tasks instead of `pixi run test-all` so CI
does not repeat local-only validation in every platform job:

- `pixi run check-lint` runs once in `ci_lint.yml`
- `pixi run docs-build` runs once in `ci_lint.yml` for PRs, protected branch
  pushes, and manual runs
- Platform jobs run explicit C++ and dartpy test tasks for their selected
  coverage slice
- Release jobs build examples and install where that platform covers the path

The Coverage (Debug) job's `coverage-report` task parallelizes the lcov
capture: object directories holding `.gcda` data are pruned so no listed
directory has a listed ancestor (geninfo scans each `--directory`
recursively), distributed round-robin across one capture process per job
slot, and the partial tracefiles are merged with `lcov -a`. Each `.gcda` is
captured exactly once, so the merged tracefile matches a serial capture. Set
`DART_PARALLEL_JOBS=1` to fall back to a single capture process when
debugging capture issues.

Use `pixi run test-all` for local pre-PR validation; avoid adding it to CI jobs
unless the duplicated lint/docs/build work is intentional.
On Linux hosts with a visible NVIDIA CUDA runtime, also run
`pixi run -e cuda test-all`; the CUDA-environment helper preserves the `cuda`
Pixi environment for nested tasks and executes the CUDA CTest + benchmark smoke
path automatically when a runtime is detected. The local CUDA Pixi config
auto-detects visible GPU compute capabilities for `DART_CUDA_ARCHITECTURES` so
runtime checks do not depend on PTX JIT compatibility with the installed driver.

## Monitoring and Maintenance

### Expected CI Times

**Baseline, code PR, compiler cache silently off (PR #3455, 2026-08-29):**

- Ubuntu Release Tests: 64 min (53 min build, 21 s ctest, 8 min dartpy)
- Ubuntu Coverage (Debug): 65 min (+33 min queue wait)
- Ubuntu Debug Tests: 53 min
- Ubuntu Headless Rendering: 51 min (two builds: examples + docking dartpy)
- macOS Release / Debug (arm64): 40 / 33 min
- Windows Tests (Release): 104 min (~100 min build, 32 s ctest)
- Wheels: Ubuntu 30, macOS 35, Windows 62 min
- 27 jobs, ~735 job-minutes, wall-clock to all-green ~107 min; the free plan
  caps concurrency at 20 jobs, so queue wait reached 35 min on some jobs.

**Warm compiler cache (after September 2026 fix):** record the measured
numbers here from the first two runs after the fix; expect the build-bound
steps to drop by 50-70%.

**Continuous additions on main/release pushes:**

- ASAN Tests: ~100 min (compile-dominated)
- CodeQL C++: ~90 min
- Performance Dashboard: ~50 min

### Cache Health

- Every job's step summary carries a "Compiler cache" block from
  `configure-compiler-cache` (launcher, backend availability, runner). A
  hosted job with `launcher: none` fails; do not weaken that guard.
- Repository usage: `gh api repos/dartsim/dart/actions/cache/usage` (10 GB
  cap; see Cache budget above for what may occupy it).
- Warm-run evidence: compare the build step duration of two consecutive runs
  of the same job; a warm run should be well under half of a cold run.
- Cache keys are content-hashed per object by sccache, so there is nothing to
  bust manually; clearing entries from the GitHub UI forces a cold rebuild.

### Maintenance Tasks

**Regular:**

- Review average CI times weekly
- Monitor cache hit rates
- Check for test flakiness

**Quarterly:**

- Update GitHub Actions versions
- Review and optimize cache sizes
- Evaluate new optimization opportunities

### Monitoring and Debugging from the GitHub CLI

#### Branch / PR Recon

Suggested (Unverified):

```bash
git status -sb
git fetch origin
git rev-list --left-right --count HEAD...origin/main
```

Map the current branch to a PR (Suggested (Unverified)):

```bash
gh pr list --head "$(git branch --show-current)" --json number,title,state,url,headRefName
gh pr view --json number,title,url,state,baseRefName,headRefName,author,labels,body
```

#### CI Triage

Suggested (Unverified):

```bash
gh run list --branch <BRANCH> -e pull_request -L 20
gh run watch <RUN_ID> --interval 30
gh run view --job <JOB_ID> --log-failed
gh pr checks <PR_NUMBER>
```

If a job behaves differently than expected, confirm which runner actually executed it (Suggested (Unverified)):

```bash
gh api repos/<ORG>/<REPO>/actions/jobs/<JOB_ID> --jq '{id: .id, status: .status, conclusion: .conclusion, runner_name: .runner_name, runner_group: .runner_group_name, labels: .labels, started_at: .started_at, completed_at: .completed_at}'
```

If `--log-failed` is missing context, list job step outcomes first:

Suggested (Unverified):

```bash
gh run view <RUN_ID> --json jobs --jq '.jobs[] | select(.databaseId==<JOB_ID>) | {steps:[.steps[] | {name: .name, conclusion: .conclusion}]}'
```

Suggested (Unverified):

```bash
gh pr checks <PR_NUMBER> --watch --interval 30 --fail-fast
```

Notes:

- If the `CI gz-physics` workflow fails, reproduce locally with the Gazebo workflow in [build-system.md](build-system.md#gazebo-integration-feature): `N=${DART_SAFE_JOBS:-$(python scripts/parallel_jobs.py)}` then `DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz`.
- Suggested (Unverified): If you create PRs from the command line, prefer `gh pr create --body-file <path>` over `--body "..."` when the body contains backticks; some shells (e.g., zsh) treat backticks as command substitution.

## Troubleshooting

### Slow CI Builds

**Check:**

1. The job's "Compiler cache" step summary: `launcher: sccache` and
   `GitHub Actions cache backend: available` are expected on hosted runners
2. Whether the build step duration matches the cold baseline in Expected CI
   Times (then the cache is not hitting) or the warm numbers
3. `gh api repos/dartsim/dart/actions/cache/usage` against the 10 GB budget

**Solutions:**

- Fix the cache setup rather than disabling the `require-cache` guard
- Verify `CMAKE_*_COMPILER_LAUNCHER` environment variables are set
- Check conditional execution logic

### Cache-Related Build Failures

**Symptoms:**

- Build succeeds locally but fails in CI
- Errors about missing headers or outdated objects

**Solutions:**

- Force cache bust by changing cache key
- Add dependency tracking to cache key (e.g., hash of `pixi.lock`)
- Run full clean build on schedule to catch issues

### sccache Failures and Flakiness

**Symptoms:**

- CMake `try_compile` fails with `sccache: error: while hashing the input file ... No such file or directory (os error 2)`
- The "Setup sccache" workflow step fails with `HttpError: Connect Timeout Error` (usually transient)

**What to check:**

1. Whether the job ran on a self-hosted runner vs GitHub-hosted (see the runner metadata snippet in [CI Triage](#ci-triage))
2. Whether `.github/actions/configure-compiler-cache` disabled sccache and selected `ccache` (or `env` if unavailable) (look for its stderr messages and `DART_COMPILER_CACHE` in logs)
3. On GitHub-hosted Linux runners and on wheel builds the launcher is required: `configure-compiler-cache` fails the job with `No compiler-cache launcher is configured` (or `sccache server is not using the GitHub Actions cache`) instead of building cold. A transient `503` from the cache service fails the job within a minute at that step; re-run it. Otherwise fix the setup (action version, exported `ACTIONS_RESULTS_URL`/`ACTIONS_RUNTIME_TOKEN`, sccache download) rather than passing `require-cache: "false"` or `DART_DISABLE_COMPILER_CACHE=ON`. macOS jobs opt out deliberately and only report the missing launcher in the step summary; the Windows `Tests (Release)` job does not run the action at all (see Compilation Caching Strategy).

**Related files:**

- `.github/actions/configure-compiler-cache/action.yml`
- `.github/workflows/ci_macos.yml`

### Platform-Specific Issues

**Debug-only failures:**

- Debug C++ jobs run on every code PR: `Debug Tests` and `Coverage (Debug)`
  on Linux, `Debug Tests (arm64)` on macOS; the Linux Debug job also builds
  and smoke-tests dartpy in Debug
- Check those PR jobs first; scheduled runs repeat the same tier
- Debug-specific issues (assertions, memory checks) surface there

**Single-platform failures:**

- Ensure test runs on at least one platform
- Review platform-specific conditionals
- Check if issue is in platform-specific code

### FreeBSD: RTTI Across Shared Libraries

`dynamic_cast` can fail silently across shared library boundaries on FreeBSD due to duplicate RTTI symbols. Prefer type enums + `static_cast` for polymorphic dispatch in cross-library APIs.

### FreeBSD: Ports Patches Depend on CMakeLists Context

The FreeBSD VM job applies the patches under `docker/freebsd/ports-patches/patch-*` to the source tree before configuring (`scripts/freebsd.py` `apply_ports_patches()` runs `patch -p0 -N`, guarded by a dry-run check). These patches carry fixed line numbers and surrounding context — for example, dropping `-O3` from `CMAKE_CXX_FLAGS_RELEASE` in the root `CMakeLists.txt`.

Reformatting or restructuring a patched `CMakeLists.txt` — a `gersemi` run, reordering blocks, or editing nearby lines — shifts that context so the hunks no longer match. Because the apply step is dry-run guarded, a non-matching patch is **silently skipped** rather than aborting, so the intended CMake change never lands and the failure surfaces later as wrong build flags or behavior on FreeBSD only. When you change the structure of a file targeted by `docker/freebsd/ports-patches/`, regenerate the affected patch against current source (`diff -u CMakeLists.txt.orig CMakeLists.txt`).

### macOS ARM64: Flaky SEGFAULTs in FCL and Sensor Tests

Certain tests (`FclPrimitiveContactMatrix` in Release, `SensorManager` in Debug) intermittently SEGFAULT on macOS ARM64 CI runners. These failures are non-deterministic — the same commit passes on push-triggered runs but can fail on PR-triggered runs.

**Symptoms:**

- SEGFAULT in `INTEGRATION_collision_FclPrimitiveContactMatrix` (Release only)
- SEGFAULT in `UNIT_sensor_SensorManager` (Debug only)
- Failures appear on PR-triggered runs but pass on push-triggered runs for the same commit

**Solution:** Re-run only the failed jobs:

```bash
gh run rerun <RUN_ID> --failed
```

These are pre-existing intermittent issues, not caused by PR changes. If you see them, verify the failing tests are in the known-flaky list above before re-running.

### ARM64: Memory Alignment

ARM64 requires strict alignment. Avoid `alloca()`, VLAs, and reinterpreting `std::vector<char>` as other types. Use `std::vector<T>` to guarantee `alignof(T)`.

### RTD: Sphinx Extension Compatibility

Sphinx extensions may break on newer versions. Use defensive patterns: `.get(key, default)`, `.pop(key, None)`, `getattr(obj, "attr", default)`.

## Best Practices

### When Adding New Tests

1. **Add to appropriate test suite** (unit, integration, Python)
2. **Verify test runs on CI** by checking workflow logs
3. **Keep tests fast** - slow tests impact developer productivity
4. **Avoid flaky tests** - use proper synchronization and timeouts

### When Modifying Workflows

1. **Validate YAML syntax** before pushing. Suggested (Unverified): `python3 -c "import yaml; yaml.safe_load(open('.github/workflows/<file>.yml')); print('valid')"`
2. **Test in a branch first** before merging to main
3. **Monitor impact** on CI times for next 5-10 PRs
4. **Document changes** in workflow comments
5. **Maintain backward compatibility** where possible

### When Adding Dependencies

1. **Update `pixi.toml`** with version constraints
2. **Run `pixi install`** to update lock file
3. **Consider cache impact** - large dependencies slow cache restore
4. **Test on all platforms** before merging

## Performance Optimization History

**Phase 1: Quick Wins** (Implemented)

- Centralized lint checks: 12-25 min saved
- Conditional Debug builds: 30-40 min saved
- Optimized wheel builds: 60-90 min saved
- Path filtering for docs: 20-30 min saved

**Phase 2: Caching** (Implemented)

- ccache/sccache: 30-50 min saved on subsequent builds
- MSVC multi-core verified as optimally configured

**Phase 3: Correctness of the cache and filter layers** (September 2026)

- Compiler cache re-enabled on hosted runners (it had been silently disabled
  since the Actions cache-service v2 migration) with a fail-loud guard
- Cache budget reallocated to sccache (pixi environment and CodeQL TRAP
  caches off)
- Shared code filter fixed (`predicate-quantifier: some-with-excludes`) and
  widened to AI tool surfaces, with a structural test
- Least-privilege `permissions` and `timeout-minutes` on every workflow/job

**Future opportunities:**

- Consolidate duplicate Release builds across Linux jobs (planned PR-2)
- Split test suites into parallel jobs

## Related Documentation

- [Testing Guide](testing.md) - Test suite organization and running tests
- [Contributing Guide](contributing.md) - Contribution workflow including CI requirements
- [Build System](build-system.md) - CMake configuration details

## References

- GitHub Actions workflows: `.github/workflows/`
- Build configuration: `pixi.toml`, `CMakeLists.txt`
- Test scripts: `scripts/test_all.py`
