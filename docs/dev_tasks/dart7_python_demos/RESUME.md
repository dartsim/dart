# Resume

## Current state (2026-10-12)

| Packet | Branch | State |
| --- | --- | --- |
| WP-PD.0 | `docs/demo-app-consolidation-plan` | Merged (#3732) |
| WP-PD.1 | `build/imgui-docking` | Ready PR #3745, bottom of stack #3748 |
| WP-PD.2 | `refactor/demo-models-library` | Ready PR #3746, stacked on #3745 |
| WP-PD.3 | `refactor/gui-app-host` | Ready PR #3747, stacked on #3746 |
| WP-PD.4 | `feature/app-panels-contract` | Ready PR #3758, stacked on #3747 |
| WP-PD.5 | `feature/dartpy-viewer` | Draft PR #3761, stacked on #3758 |
| WP-PD.6 | `feature/bind-demo-models` | Draft PR #3755, on #3746; off stack #3748 |
| WP-PD.7 | `feature/python-demo-catalog` | Draft PR #3762, stacked on #3761 |
| WP-PD.8 | `feature/bind-resting-heightmap-voxel` | Ready PR #3737, off stack #3748 |
| WP-PD.9 | `feature/bind-whole-body-ik` | Draft PR #3751 |
| WP-PD.10 (S1) | `feature/port-demo-scenes-s1` | Draft PR #3763, stacked on #3762 |
| WP-PD.11 (S2) | `feature/port-demo-scenes-s2` | Draft PR #3764, stacked on #3763 |
| WP-PD.12 (S3) | `codex/s3` | Local, in review |
| WP-PD.13 (S4) | `codex/s4` | In progress, on S3 |
| WP-PD.14 (S5) | `feature/port-demo-scenes-s5` | Draft PR #3765, on `feature/demo-scenes-base` |
| WP-PD.15 (S6a) | `codex/s6a` | Local, in review, on S5 |
| WP-PD.16 (S6b) | `codex/s6b` | In progress, on S6a |
| WP-PD.17 | — | Planned |
| WP-PD.18 | — | Planned |

- In stack #3748, #3745, #3746, #3747 and #3758 are ready for review; the later PRs are drafts.
- WP-PD.9 is off stack #3748.
- #3735 is merged: API snapshots read non-arithmetic enums.
- #3759 is merged: the release-target check skips stacked PR bases.
- #3744 (heightmap bounds) is ready for review, off the stack.
- `feature/demo-scenes-base` merges S2 (#3764) with #3755, #3737 and #3751;
  the later scene batches build on it.
- `main`'s Windows C++ CI failed on `ContactSurfaceProbe` from #3725 until
  #3741 fixed it. Merging `main` clears that failure from older PR heads.

## Immediate next step

1. Watch the reviews and CI of the open PRs.
2. Publish S3 (WP-PD.12) and S6a (WP-PD.15) after their reviews, then
   finish S4 (WP-PD.13) and S6b (WP-PD.16).
3. Finish WP-PD.17 (launch the catalog from the demos tasks), then WP-PD.18
   (remove the C++ app).

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
- **Process groups.** Codex lanes and review loops run as their own process
  groups, stopped only through their recorded PIDs.
- **Parity runs.** Until WP-PD.17, run the Python app from `python/` with
  `PYTHONPATH=../build/default/cpp/Release/python/dartpy pixi run python -m examples.demos`.

## Verification commands

```bash
pixi run lint && pixi run check-lint
pixi run build && pixi run test
pixi run build-py-dev && pixi run test-py
pixi run test-gui
```

The [README](README.md#gates) lists the vanilla, GUI-off, install and wheel
gates for each packet.
