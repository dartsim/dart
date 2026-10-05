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
| `ci_ubuntu.yml`                   | CI Linux                     | PR, push, nightly               | AI checks, lint, Release/Debug build + test, no-OSG asserts gate; nightly adds ASan, coverage, and Eigen 64-byte alignment |
| `ci_macos.yml`                    | CI macOS                     | PR, push, nightly               | arm64 Release build + test; nightly adds Debug and install |
| `ci_windows.yml`                  | CI Windows                   | PR, push, nightly               | MSVC Release build + test |
| `ci_gz_physics.yml`               | CI gz-physics                | PR, push, nightly               | Gazebo/gz-physics downstream integration |
| `api_doc.yml`                     | API Documentation            | PR, push, nightly               | Doxygen API docs build (validation only; not published) |
| `ci_simd.yml`                     | CI SIMD Multi-Arch           | PR/push touching SIMD, nightly  | SIMD instruction-level matrix (scalar/SSE4.2/AVX/AVX2) on x86_64; NEON is covered by `ci_macos.yml` arm64 jobs |
| `ci_freebsd.yml`                  | CI FreeBSD (VM)              | nightly, dispatch               | FreeBSD build + test in a VM |
| `ci_toolchain.yml`                | CI Toolchain (Linux)         | nightly, dispatch               | Newest gcc/clang build + test |
| `codeql.yml`                      | CodeQL                       | nightly, dispatch               | Static security analysis |
| `publish_dartpy.yml`              | Publish dartpy               | nightly, version tags, dispatch | Build, repair, verify, and test wheels; publish from version tags |
| `nightly.yml`                     | Nightly                      | daily, on demand                | Everything above on `main`; files `nightly-failure` issues |
| `performance_dashboard_dart6.yml` | DART 6 Performance Dashboard | push, dispatch                  | Performance dashboard |
| `update_lockfiles.yml`            | Update Lock Files            | weekly                          | Pixi lockfile refresh PRs against `main` |

Required checks on `main`: `Release`, `Debug`, and
`Asserts enabled (no -DNDEBUG)` (CI Linux), `arm64-Release` (CI macOS),
`windows-Release` (CI Windows), `ubuntu-latest` (CI gz-physics),
`API Documentation`, and the two Read the Docs builds. Never require a
nightly-only job: it never reports on PRs, so it would block every merge.

## Caching

Build jobs restore an sccache compiler cache saved from `main` at most once a
day per configuration, and pixi environment caches are also written only from
`main`: PR runs read both but never write, which keeps the repository's 10 GB
Actions cache for main-branch entries. Each job prints `sccache --show-stats`.
CTest runs in parallel (`CTEST_PARALLEL_LEVEL`). The assertions gate builds
`ALL_NO_RUN`, which builds everything `ALL` does without running the tests.

## Nightly

`nightly.yml` runs every workflow in the index except the performance
dashboard and lockfile refresh against `main` each night at 08:00 UTC,
including the nightly-only jobs. It is scheduled directly on `main`, the
default branch, with no dispatcher. Run it on demand with
`gh workflow run nightly.yml --ref main`.

Its `report` job (`scripts/nightly_ci_report.py`) groups jobs by their
`nightly.yml` caller (`linux`, `macos`, `freebsd`, ...) and keeps at most one
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

For failing CI, inspect the exact run and job logs before changing code. Prefer
reproducing locally, but document when a hosted-platform failure cannot be
reproduced on the current machine.
