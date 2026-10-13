# Resume

## Current state (2026-10-12)

The `codex/pd17` checkout includes the host, bindings and all scene ports
through WP-PD.16. WP-PD.17's task and documentation migration is local and
verified, with all requested checks and two clean local reviews. The table
below records the earlier published stack; it does not describe the full
content of this checkout.

| Packet | Branch | State |
| --- | --- | --- |
| WP-PD.0 | `docs/demo-app-consolidation-plan` | Draft PR #3732 |
| WP-PD.1 | `build/imgui-docking` | Draft PR #3745, bottom of stack #3748 |
| WP-PD.2 | `refactor/demo-models-library` | Draft PR #3746, stacked on #3745 |
| WP-PD.3 | `refactor/gui-app-host` | Draft PR #3747, stacked on #3746 |
| WP-PD.4 | `feature/app-panels-contract` | Local, on WP-PD.3; in review |
| WP-PD.5 | `feature/dartpy-viewer` | Local, on WP-PD.4; in review |
| WP-PD.8 | `feature/bind-resting-heightmap-voxel` | Draft PR #3737; voxel input hardening in review |
| WP-PD.9 | `feature/bind-whole-body-ik` | Draft PR #3751 |

- Related fixes: #3735 (API snapshots of non-arithmetic enums) and #3744
  (heightmap bounds).
- `main`'s Windows C++ CI failed on `ContactSurfaceProbe` from #3725 until
  #3741 fixed it. Merging `main` clears that failure from older PR heads.

## Immediate next step

1. Continue with WP-PD.18; WP-PD.17's command results and review evidence are
   recorded in [verification.md](verification.md).
2. Keep the C++ app and its CTests until WP-PD.18 removes them.
3. No commit or publication is authorized for this checkout.

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
- **Parity runs.** `pixi run demos -- …` now launches the Python catalog.
  For direct parity runs from `python/`, use
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
