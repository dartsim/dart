# CI And Release-Branch Checks

Use GitHub Actions as the hosted source of truth after a PR is opened. Locally,
run the smallest gate that proves the touched surface, then broaden when shared
runtime, package, or downstream behavior changes.

## Workflow Index

All files live in `.github/workflows/` on this branch. "PR, push" means pull
requests and pushes to `release-*`; "nightly" means the `Nightly` workflow
below. Note that `gh pr checks` lists job-level check names (for example
`Release` under the `CI Linux` workflow); map a failing check to its workflow
via the run's workflow name shown here (`gh pr checks` exposes it in the
`workflow` JSON field).

| Workflow file                     | Workflow name                | Runs                            | Purpose |
| --------------------------------- | ---------------------------- | ------------------------------- | ------- |
| `ci_ubuntu.yml`                   | CI Linux                     | PR, push, nightly               | AI checks, lint, Release/Debug build + test, no-OSG asserts gate; nightly adds ASan, coverage, and Eigen 64-byte alignment |
| `ci_macos.yml`                    | CI macOS                     | PR, push, nightly               | arm64 Release build + test; nightly adds Debug |
| `ci_windows.yml`                  | CI Windows                   | PR, push, nightly               | MSVC Release build + test |
| `ci_gz_physics.yml`               | CI gz-physics                | PR, push, nightly               | Gazebo/gz-physics downstream integration |
| `api_doc.yml`                     | API Documentation            | PR, push, nightly               | Doxygen API docs build (validation only; not published) |
| `ci_simd.yml`                     | CI SIMD Multi-Arch           | PR/push touching SIMD, nightly  | SIMD instruction-level matrix (scalar/SSE4.2/AVX/AVX2) on x86_64; NEON is covered by `ci_macos.yml` arm64 jobs |
| `ci_freebsd.yml`                  | CI FreeBSD (VM)              | nightly, dispatch               | FreeBSD build + test in a VM |
| `ci_toolchain.yml`                | CI Toolchain (Linux)         | nightly, dispatch               | Newest gcc/clang build + test |
| `codeql.yml`                      | CodeQL                       | nightly, dispatch               | Static security analysis |
| `publish_dartpy.yml`              | Publish dartpy               | nightly, version tags, dispatch | Build, repair, verify, and test wheels; publish from version tags |
| `nightly.yml`                     | Nightly                      | daily, PRs that change CI       | Everything above on `release-6.20`; files `nightly-failure` issues |
| `performance_dashboard_dart6.yml` | DART 6 Performance Dashboard | push, call, dispatch            | Performance dashboard |
| `update_lockfiles.yml`            | Update Lock Files            | weekly                          | Pixi lockfile refresh PRs against `release-6.20` |

Required checks on `release-6.20`: `Release`, `Debug`, and
`Asserts enabled (no -DNDEBUG)` (CI Linux), `arm64-Release` (CI macOS),
`windows-Release` (CI Windows), `ubuntu-latest` (CI gz-physics),
`API Documentation`, and the two Read the Docs builds. Never require a
nightly-only job: it never reports on PRs, so it would block every merge.

## Nightly

`nightly.yml` runs every workflow in the index except the performance
dashboard and lockfile refresh against `release-6.20` each night at 08:00 UTC,
including the nightly-only jobs. GitHub fires schedules only on the default
branch, so the scheduled run on `main` just dispatches the real run on
`release-6.20`; changes to the schedule itself take effect once `main` is
fast-forwarded. Run it on demand with
`gh workflow run nightly.yml --ref release-6.20`.

Its `report` job (`scripts/nightly_ci_report.py`) groups jobs by their
`nightly.yml` caller (`linux`, `macos`, `freebsd`, ...) and keeps at most one
open `nightly-failure` issue per failing group. It opens the issue with log
excerpts, fixing steps, and a prompt for an AI agent; comments on it each
night the group still fails; and closes it on the first night the group
succeeds. PRs that change CI run the whole nightly matrix, with the report in
dry-run mode. Test the reporter with
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
