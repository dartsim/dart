# Resume

## Current state (2026-10-12)

| Packet | Branch | State |
| --- | --- | --- |
| WP-PD.0 | `docs/demo-app-consolidation-plan` | Draft PR #3732 |
| WP-PD.1 | `build/imgui-docking` | Local; gates pass, in review |
| WP-PD.2 | `refactor/demo-models-library` | Local; gates pass, in review; to be stacked on WP-PD.1 |
| WP-PD.8 | `feature/bind-resting-heightmap-voxel` | Local; gates pass, in review |
| WP-PD.9 | `codex/bind-whole-body-ik` | Codex lane running |

- `main`'s Windows C++ CI fails on `ContactSurfaceProbe` since #3725. #3731
  fixes it; that failure is not caused by these packets.
- `scripts/nanobind/api_surface.py --snapshot` fails on the current dartpy
  (a non-arithmetic enum). A separate fix keeps the API-diff gate usable.

## Immediate next step

1. Publish WP-PD.1 and WP-PD.2 as a stack once their reviews are clean.
2. Start WP-PD.3 (host move) as a Codex lane on top of WP-PD.2, with the
   spec built from the README's packet scope.
3. Commit and review WP-PD.9 when its lane finishes.

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
