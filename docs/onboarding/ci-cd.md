# CI And Release-Branch Checks

Use GitHub Actions as the hosted source of truth after a PR is opened. Locally,
run the smallest gate that proves the touched surface, then broaden when shared
runtime, package, or downstream behavior changes.

## Workflow Index

All files live in `.github/workflows/` on this branch. For PR-triggered
workflows (the first eight rows), check names on a PR map to the workflow's
`name:` shown here; the remaining rows run on push, workflow call, dispatch,
or schedule (and CodeQL is currently pinned to older branches).

| Workflow file                     | Check name                  | Purpose                                          |
| --------------------------------- | --------------------------- | ------------------------------------------------ |
| `ci_ubuntu.yml`                   | CI Linux                    | Linux AI checks, lint, build, test, coverage     |
| `ci_macos.yml`                    | CI macOS                    | macOS lint, build, test                          |
| `ci_windows.yml`                  | CI Windows                  | Windows lint, build, test                        |
| `ci_freebsd.yml`                  | CI FreeBSD (VM)             | FreeBSD build + test in a VM                     |
| `ci_simd.yml`                     | CI SIMD Multi-Arch          | SIMD instruction-level matrix (scalar/SSE4.2/AVX/AVX2) on x86_64; NEON is covered by `ci_macos.yml` arm64 jobs |
| `ci_toolchain.yml`                | CI Toolchain (Linux)        | Alternate Linux toolchain build + test           |
| `ci_gz_physics.yml`               | CI gz-physics               | Gazebo/gz-physics downstream integration         |
| `api_doc.yml`                     | API Documentation           | Doxygen API docs build/publish                   |
| `codeql.yml`                      | CodeQL                      | Static security analysis — currently inactive here: its branch filter pins `release-6.17`/`release-6.16` |
| `publish_dartpy.yml`              | Publish dartpy              | Build, repair, verify, and publish Python wheels |
| `performance_dashboard_dart6.yml` | DART 6 Performance Dashboard | Performance dashboard (push/call/dispatch)      |
| `update_lockfiles.yml`            | Update Lock Files           | Scheduled pixi lockfile refresh PRs              |

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
