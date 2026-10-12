# Resume

## Current state (2026-10-12)

- WP-PD.0 is in progress on the local branch
  `docs/demo-app-consolidation-plan`, cut from `origin/main` at `722f28ddd`.
  Nothing has been pushed.
- No other packet branch exists yet.

## Immediate next step

1. Finish WP-PD.0:
   - lint and check-lint;
   - two clean local reviews;
   - commit;
   - ask the maintainer to approve a push and a draft PR.
2. Start WP-PD.1 (docking ImGui) in the main checkout.
3. Run WP-PD.2 (model move) in parallel as a Codex lane in a worktree.

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
