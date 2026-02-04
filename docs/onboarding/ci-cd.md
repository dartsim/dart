# CI/CD System

## Overview

DART uses GitHub Actions for continuous integration and deployment. The CI system validates code quality, runs tests across multiple platforms, builds documentation, and publishes Python wheels.

## Quickstart

- Start here next time:
  - Local build/test entry points: [building.md](building.md) and [testing.md](testing.md)
  - Gazebo / gz-physics workflow: [build-system.md](build-system.md#gazebo-integration-feature)
  - PR template checklist: [`.github/PULL_REQUEST_TEMPLATE.md`](../../.github/PULL_REQUEST_TEMPLATE.md)
  - Asserts-enabled CI build (no `-DNDEBUG`): see [Asserts-Enabled CI Build](#asserts-enabled-ci-build-no--dndebug)
  - ASAN testing (memory errors): `pixi run test-asan` (runs in CI for `release-6.16`)
  - CI monitoring commands: see [CI Monitoring (CLI)](#ci-monitoring-cli) and [CI Monitoring (API)](#ci-monitoring-api)
  - Common CI failure fixes: see [Common CI Failure Modes](#common-ci-failure-modes)
- Fast CI fail-fast loop:
  - Suggested (Unverified): `gh pr checks <PR_NUMBER> --watch --interval 30 --fail-fast`
  - Suggested (Unverified): `gh run view --job <JOB_ID> --log-failed`
- Gotchas:
  - `gh run list` can show separate runs for `push` and `pull_request`; for PR gating, watch the `pull_request` run.
  - `gh run watch` is blocking and can run for a long time; use a persistent shell and re-run it if your terminal session times out.
  - `gh run view --job <JOB_ID> --log-failed` only works after the job completes; use the REST logs endpoint (or wait) when a run is still in progress.
  - If a PR is not mergeable due to conflicts, CI checks may be blocked or fail early (including AppVeyor); resolve conflicts locally and push before re-running CI.
  - FreeBSD VM tests can take over 1 hour to complete; this is expected, not a sign of failure.
  - FreeBSD VM startup can timeout (~5 min); this is transient—re-run the job.
  - `dynamic_cast` can fail silently on FreeBSD across shared library boundaries; use type enums + `static_cast`.
  - macOS ARM64 sporadic SEGFAULT from `alloca`/VLA alignment; use `std::vector<T>` instead.
  - Example reorganizations can conflict in `examples/CMakeLists.txt`; reconcile any new example entries and keep the category layout aligned before pushing.
  - GitHub Actions API calls can return `HTTP 406` if you omit required headers; include an explicit `Accept` header.
  - `gh api` writes to stdout and does not support `--output`; redirect to a file when you need to search logs.
  - The asserts-enabled CI job uses a custom CMake configure (`CMAKE_BUILD_TYPE=None`) instead of pixi tasks; pass required build toggles explicitly (e.g., Bullet collision).
  - Deprecated headers that emit `#warning` fail under `-Werror=cpp` (e.g., use `dart/utils/urdf/All.hpp` instead of deprecated `dart/utils/urdf/urdf.hpp`).
  - dartpy test failures can show up as a Python abort with minimal traceback when a C++ `DART_ASSERT` triggers; rerun the single test locally and inspect the C++ assert.
  - Bullet-backed raycast tests require Bullet to be built; skip or enable Bullet if the backend is intentionally disabled.
  - `gh pr status --json ...` can error with `Unknown JSON field: ...` if you request unsupported fields; use `gh pr status` (no JSON) or `gh pr view --json ...`.
  - `gh run view --json ...` only accepts supported fields; prefer the `url` field instead of guessing `htmlURL`.
  - `gh run rerun --job` expects the job `databaseId` (not the numeric ID from the job URL); if `gh run view --json jobs` shows `id: null`, use `databaseId`. Suggested (Unverified): `gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'`.
  - Job log endpoints can return `log not found`/404 even after a failure; fall back to the run-level logs archive or re-run the job (see CI Monitoring (API)).
  - Review comment metadata is not exposed by `gh pr view --json`; Suggested (Unverified): `gh api /repos/<OWNER>/<REPO>/pulls/comments/<COMMENT_ID>`.
  - `gh pr checks` may show duplicate entries when workflows run for both `push` and `pull_request` events; compare the run URLs and focus on the newest one.
  - Newer runs can cancel older ones; confirm the run status/conclusion before spending time on job logs.
- zsh can produce parse errors when jq expressions or backtick characters are not fully quoted; quote `gh ... --jq` programs and use a here-doc or `--body-file` when PR bodies include backticks.
  - If `CI gz-physics` fails, reproduce locally with the Gazebo workflow in [build-system.md](build-system.md#gazebo-integration-feature).
  - CI jobs can sit in the queue for a long time; re-check the run list and wait for the PR run to start before assuming a failure.
  - Wheel publishing workflows may lag behind other jobs and stay queued longer; keep watching the PR run until all workflows complete.
  - Randomized stress tests can diverge across platforms if they rely on library-dependent distributions; prefer deterministic RNG transforms when portability matters.
  - `check-format` failures usually mean formatting drift; run the C++ formatter and commit any diffs before retrying CI. Suggested (Unverified): `pixi run lint-cpp`.
  - Local lint may fail if clang-format is missing or a stale CMake cache references an old version; clean the build directory (`rm -rf build/`) and reconfigure to pick up the pixi-provided clang-format.
  - Codecov patch failures usually mean new lines or branches are uncovered; add targeted tests and re-run coverage.
  - Codecov patch status can lag until coverage jobs complete; confirm Coverage (Debug) finished before acting.

## Common CI Failure Modes

- Formatting checks fail: run the C++ formatting task and re-run CI. Suggested (Unverified): `pixi run lint-cpp`.
- Codecov patch failures: add targeted coverage for new lines or branches.
- Example builds fail because sample code references removed formats or enums; update the example to match the current API (e.g., `dart::io::ModelFormat`).
- Unit test crashes or segfaults: isolate the failing test from job logs, reproduce locally, and add a regression for the edge case.
- Job logs are missing or return 404: re-run the single job and/or download the run-level logs archive to inspect failures.
- Infrastructure failures (runner connectivity lost, self-hosted runner issues): re-run only the failed jobs with `gh run rerun <RUN_ID> --failed`.
- FreeBSD RTTI failures: `dynamic_cast` across shared library boundaries can fail silently; use type enums + `static_cast` instead.
- macOS ARM64 sporadic SEGFAULT: `alloca()` or VLAs may cause alignment violations; use `std::vector<T>` for proper alignment.
- RTD build failures: Sphinx extension compatibility issues; use defensive `.get(key, default)` patterns.
- Case-colliding files after branch merge: When merging `release-*` into `main`, PascalCase files from the release branch can collide with snake_case files in main on case-insensitive filesystems (macOS, Windows). Check for duplicates with `find tests -name "*.cpp" | sort -f | uniq -di` and remove the PascalCase version. Also verify new tests are registered in the appropriate `CMakeLists.txt`.

## Task Recap (General)

This task standardized naming and packaging for the experimental simulation module and ensured it installs cleanly as an optional component. Classic simulation APIs were kept independent of experimental code while the experimental module continues to rely on core DART facilities. A small behavior fix aligned World stepping with classic solver split-impulse cleanup. Changes were validated with lint plus full test and Gazebo integration runs, then monitored in CI.

## How We Worked (Repeatable Playbook)

- Sync with the target branch and inspect the diff before making edits.
- Keep legacy/public APIs isolated from experimental modules; verify dependencies flow one way.
- Update build/CI naming and docs when namespaces or component names change.
- Run lint before committing so formatter/codespell changes are captured.
- Run the smallest local validation first, then full test-all and Gazebo workflows when simulation/build changes land.
- Resolve merge conflicts before re-running CI so the PR remains mergeable.
- When review feedback cites behavioral differences, compare against the classic implementation and align or explain the deviation.
- Push each commit and monitor GitHub Actions until all jobs complete.

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

- Pass build toggles explicitly when bypassing pixi tasks (e.g., `DART_BUILD_COLLISION_BULLET=ON` if Bullet-backed tests are expected).
- Expect `-Werror=cpp`; any deprecated headers that emit `#warning` will fail the build.
- If a dartpy test aborts without a Python traceback, the C++ assert message is usually the first useful clue.

## Next-Time Accelerators

- When running dartpy tests against an in-tree build, set `PYTHONPATH` and `DARTPY_RUNTIME_DIR` to the build output.
- If a test requires an optional backend, guard it (skip) or ensure the backend toggle is enabled in the build configuration.
- If editor or IDE context references a path, verify it exists before making edits or writing guidance.
- Re-run only the failed CI job (via job databaseId) to keep feedback loops short.

## Workflow Architecture

### Core CI Workflows

| Workflow             | Purpose               | Platforms      | Trigger            | Doc-only skip |
| -------------------- | --------------------- | -------------- | ------------------ | ------------- |
| `ci_lint.yml`        | Lint (formatting)     | Ubuntu         | PR, push           | No            |
| `ci_ubuntu.yml`      | Build, test, coverage | Ubuntu         | PR, push, schedule | Yes           |
| `ci_macos.yml`       | Build, test           | macOS          | PR, push, schedule | Yes           |
| `ci_windows.yml`     | Build, test           | Windows        | PR, push, schedule | Yes           |
| `ci_freebsd.yml`     | Build, test (VM)      | FreeBSD        | Schedule, manual   | N/A           |
| `ci_altlinux.yml`    | Build, test (Docker)  | Alt Linux      | Schedule, manual   | N/A           |
| `ci_gz_physics.yml`  | Gazebo integration    | Ubuntu         | PR, push, schedule | Yes           |
| `publish_dartpy.yml` | Python wheels         | Multi-platform | Push, schedule     | Yes           |

### Design Principles

**Optimize for fast feedback on PRs:**

- Essential validations run on every PR
- Full matrix testing runs on main branch and releases
- Debug builds run on schedule (2x per week)

**Efficient resource usage:**

- Compilation caching (sccache) reduces build time by 50-70%
- Path filtering prevents unnecessary workflow runs
- Conditional execution skips non-essential jobs on PRs
- Concurrency groups cancel superseded runs (see below)

**Concurrency configuration:**

All CI workflows use concurrency groups to cancel in-progress jobs when new commits are pushed to the same branch, with exceptions:

- **main branch**: Never cancelled (ensure full test coverage)
- **scheduled jobs**: Never cancelled (ensure periodic validation completes)

```yaml
# Standard pattern for push/PR-triggered workflows
concurrency:
  group: ${{ github.workflow }}-${{ github.ref }}
  cancel-in-progress: ${{ github.event_name != 'schedule' && github.ref != 'refs/heads/main' }}

# For workflows with matrix that push to fixed branches (e.g., update_lockfiles)
concurrency:
  group: ${{ github.workflow }}-${{ matrix.base }}
  cancel-in-progress: false  # Queue instead of cancel/race
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
`cmake/CompilerCache.cmake`, so **plain CMake invocations, pixi tasks, and CI
jobs all share the same configuration**. You can disable auto-detection with
`-DDART_DISABLE_COMPILER_CACHE=ON` or force a specific launcher via the
`DART_COMPILER_CACHE` cache variable/environment variable.

**Linux/macOS setup** (see `.github/workflows/ci_ubuntu.yml` and `ci_macos.yml`):

```yaml
- name: Setup sccache
  uses: mozilla-actions/sccache-action@v0.0.9
  with:
    disable_annotations: true

- name: Configure environment for compiler cache
  uses: ./.github/actions/configure-compiler-cache
```

**Windows setup** (see `.github/workflows/ci_windows.yml`):

```yaml
- name: Setup sccache
  uses: mozilla-actions/sccache-action@v0.0.9
  with:
    disable_annotations: true

- name: Configure environment for compiler cache
  shell: powershell
  run: |
    echo "SCCACHE_GHA_ENABLED=true" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "SCCACHE_NO_DAEMON=1" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "DART_COMPILER_CACHE=sccache" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    if ($env:SCCACHE_PATH) {
      echo "CMAKE_C_COMPILER_LAUNCHER=$env:SCCACHE_PATH" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
      echo "CMAKE_CXX_COMPILER_LAUNCHER=$env:SCCACHE_PATH" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    } else {
      echo "CMAKE_C_COMPILER_LAUNCHER=sccache" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
      echo "CMAKE_CXX_COMPILER_LAUNCHER=sccache" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    }
    $ccacheDir = Join-Path $env:RUNNER_TEMP "ccache"
    New-Item -ItemType Directory -Force -Path $ccacheDir | Out-Null
    echo "CCACHE_BASEDIR=$env:GITHUB_WORKSPACE" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "CCACHE_DIR=$ccacheDir" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "CCACHE_COMPRESS=true" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "CCACHE_MAXSIZE=5G" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
```

> **Note:** `disable_annotations` disables the post-run `sccache --show-stats` call, which avoids occasional client/server version mismatches on GitHub-hosted runners.

**Local builds:** Because the detection logic is inside CMake, you do not need
to wire anything up manually. If either `sccache` or `ccache` is on your PATH,
DART will automatically set `CMAKE_*_COMPILER_LAUNCHER` when you run `cmake`
directly or via `pixi`. pixi still forwards `CMAKE_*_COMPILER_LAUNCHER` to any
external CMake projects it drives (e.g., gz-physics) so nested builds benefit
from the same cache.

#### CI reliability notes

- The `.github/actions/configure-compiler-cache` action may disable the sccache launcher on some Linux runners (e.g., self-hosted or non-Ubuntu) and fall back to `ccache` when available (otherwise `env`) to avoid flaky `try_compile` failures.
- In `.github/workflows/ci_macos.yml`, the "Setup sccache" step is best-effort (`continue-on-error: true`) so transient download timeouts don't fail the job.

## MSVC Multi-Core Compilation

**Critical configuration** (`CMakeLists.txt` line 282-284):

```cmake
# /MP - Multi-processor compilation (uses all available cores)
# /FS - Force synchronous PDB writes (prevents PDB conflicts in parallel builds with /MP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /permissive- /Zc:twoPhase- /MP /FS")
```

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

Debug builds are slower and primarily useful for debugging, not CI validation.

**Pattern** (see `ci_ubuntu.yml` and `ci_macos.yml`):

```yaml
build:
  name: ${{ matrix.build_type }}
  runs-on: ubuntu-latest
  if: matrix.build_type == 'Release' || github.event_name == 'schedule' || github.ref == 'refs/heads/main'
  strategy:
    matrix:
      build_type: ["Release", "Debug"]
```

**Behavior:**

- **PRs**: Only Release builds run
- **Main branch**: Both Release and Debug run
- **Scheduled runs**: Both Release and Debug run
- **Savings**: 30-40 minutes per PR

### Python Wheel Builds

Full wheel matrix is expensive (60-90 minutes) and only needed for releases.

**Pattern** (see `publish_dartpy.yml`):

```yaml
build_wheels:
  if: |
    matrix.release_only == false ||
    github.ref == 'refs/heads/main' ||
    startsWith(github.ref, 'refs/tags/') ||
    github.event_name == 'schedule'
```

**Behavior:**

- **PRs**: Only essential configurations build
- **Main branch**: Full matrix builds
- **Release tags**: Full matrix builds
- **Scheduled runs**: Full matrix builds

### Documentation Builds

Read the Docs now owns all documentation publishing. GitHub Actions no longer
generates or deploys the API references, so doc validation happens locally:

- Run `pixi run docs-build` to render the RTD site (including the C++ Doxygen
  bundle).
- Committers can optionally run `pixi run api-docs-cpp` or `pixi run api-docs-py`
  if they need to inspect the standalone builders.
- RTD rebuilds automatically whenever `main` changes, keeping the hosted docs in
  sync without consuming CI minutes.

## Lint Check Strategy

**Dedicated workflow:** Linting is deterministic and platform-independent, so it runs once in a dedicated `ci_lint.yml` workflow.

**Key design:**

- `ci_lint.yml` runs on ALL changes (including doc-only changes) to catch formatting issues early
- FreeBSD and Alt Linux CI (`ci_freebsd.yml`, `ci_altlinux.yml`) run on schedule/manual only to reduce maintenance burden
- Lint is removed from platform-specific workflows since it's covered by the dedicated job

**Doc-only skip patterns** (used by platform CI, NOT by lint CI):

```yaml
paths-ignore:
  - "docs/**"
  - ".readthedocs.yml"
  - "tutorials/**"
  - "**/*.md"
  - "**/*.rst"
  - "**/*.po"
  - "**/*.pot"
```

**Savings:** 12-25 minutes per PR on platform jobs for doc-only changes

## Testing Strategy

### Test Coverage

**Per PR:**

- Ubuntu Release: Full tests + coverage
- macOS Release: Full tests
- Windows Release: Full tests
- Gazebo integration: Integration tests

**Scheduled runs:**

- Add Debug builds for all platforms
- Ensure periodic full validation

### Test Execution

All tests run through `pixi run test-all`, which includes:

- Linting (check-lint)
- Unit tests (C++)
- Python tests (dartpy)
- Documentation build (optional)

**Best practice:** Don't duplicate steps already in `test-all`.

## Monitoring and Maintenance

### Expected CI Times

**Without caching (first run):**

- Ubuntu: 45-60 min
- macOS: 30-45 min
- Windows: 25-35 min
- Total: ~100-150 min

**With caching (subsequent runs):**

- Ubuntu: 20-30 min
- macOS: 15-25 min
- Windows: 15-20 min
- Total: ~30-60 min (50-70% reduction)

### Cache Health

**Monitor cache hit rates:**

- Target: >80% after first run
- Check cache statistics in workflow logs
- Adjust `max-size` if caches frequently exceed limits

**Cache invalidation:**

- Caches are specific to OS + build type
- Automatic invalidation on cache key change
- Manual cache clearing via GitHub UI if needed

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

- Suggested (Unverified): If the `CI gz-physics` workflow fails, reproduce locally with the Gazebo workflow in [build-system.md](build-system.md#gazebo-integration-feature); Linux example (2/3 cores): `N=$(( ( $(nproc) * 2 ) / 3 ))` then `DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz`.
- Suggested (Unverified): If you create PRs from the command line, prefer `gh pr create --body-file <path>` over `--body "..."` when the body contains backticks; some shells (e.g., zsh) treat backticks as command substitution.

## Troubleshooting

### Slow CI Builds

**Check:**

1. Cache hit rate in workflow logs
2. Whether ccache/sccache is being used (look for "compiler launcher" in logs)
3. If Debug builds are running on PRs (should only run on schedule)

**Solutions:**

- Clear GitHub Actions cache and rebuild
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
3. If this only happens on CI, treat sccache as optional and focus on correctness first; caching should not be required to pass CI

**Related files:**

- `.github/actions/configure-compiler-cache/action.yml`
- `.github/workflows/ci_macos.yml`

### Platform-Specific Issues

**Debug-only failures:**

- Debug builds still run on schedule
- Check scheduled workflow runs for failures
- Debug-specific issues (assertions, memory checks) caught there

**Single-platform failures:**

- Ensure test runs on at least one platform
- Review platform-specific conditionals
- Check if issue is in platform-specific code

### FreeBSD: RTTI Across Shared Libraries

`dynamic_cast` can fail silently across shared library boundaries on FreeBSD due to duplicate RTTI symbols. Prefer type enums + `static_cast` for polymorphic dispatch in cross-library APIs.

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

**Future opportunities:**

- Parallel test execution (CTest `--parallel`)
- Split test suites into parallel jobs
- Docker-based CI for faster dependency installation

## Related Documentation

- [Testing Guide](testing.md) - Test suite organization and running tests
- [Contributing Guide](contributing.md) - Contribution workflow including CI requirements
- [Build System](build-system.md) - CMake configuration details

## References

- GitHub Actions workflows: `.github/workflows/`
- Build configuration: `pixi.toml`, `CMakeLists.txt`
- Test scripts: `scripts/test_all.py`
