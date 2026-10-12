# Resume

## Current state (2026-10-12)

| Packet | Branch | State |
| --- | --- | --- |
| WP-PD.0 | `docs/demo-app-consolidation-plan` | Draft PR #3732 |
| WP-PD.1 | `build/imgui-docking` | Draft PR #3745, bottom of stack #3748 |
| WP-PD.2 | `refactor/demo-models-library` | Draft PR #3746, stacked on #3745 |
| WP-PD.3 | `refactor/gui-app-host` | Draft PR #3747, stacked on #3746 |
| WP-PD.4 | `feature/app-panels-contract` | Local, on WP-PD.3; gates and reviews running |
| WP-PD.8 | `feature/bind-resting-heightmap-voxel` | Draft PR #3737; voxel input hardening in progress |
| WP-PD.9 | `feature/bind-whole-body-ik` | Local; gates pass, in review |

- Related fixes: #3735 (API snapshots of non-arithmetic enums) and #3744
  (heightmap bounds).
- `main`'s Windows C++ CI fails on `ContactSurfaceProbe` since #3725. #3731
  fixes it; that failure is not caused by these packets.

## Immediate next step

1. Finish WP-PD.4's reviews, then add it to stack #3748.
2. Write WP-PD.5 (`dartpy.viewer`) on top of WP-PD.4.
3. Publish WP-PD.9 once two local reviews are clean.
4. Watch the GitHub reviews and CI of the open PRs.

## Recovery notes

- **Stale ImGui choice.** Existing build trees cache
  `DART_USE_SYSTEM_IMGUI=ON`. After WP-PD.1, `pixi run config` passes `OFF`
  explicitly. If configure still selects the old ImGui, check
  `DART_USE_SYSTEM_IMGUI` in `CMakeCache.txt` and
  `_deps/dart_imgui_tag.txt`.
- **Missing GTest.** If configure reports "Could NOT find GTest" while the
  Pixi `gtest` package is listed, the environment lost that package's headers
  and unversioned libraries. `pixi reinstall gtest` restores them.
- **Codex worktrees.** Lanes live in `../<slug>` on branches `codex/<slug>`.
  List them with `git worktree list`, and remove a worktree after its packet
  merges.
- **Parity runs.** Until WP-PD.17, run the Python app from `python/` with
  `PYTHONPATH=../build/default/cpp/Release/python/dartpy python -m examples.demos`.

## Verification commands

```bash
pixi run lint && pixi run check-lint
pixi run build && pixi run test
pixi run build-py-dev && pixi run test-py
pixi run test-gui
```

The [README](README.md#gates) lists the vanilla, GUI-off, install and wheel
gates for each packet.
