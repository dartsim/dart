# DART 7 demo consolidation follow-ups

The demo consolidation (decision D31b) landed on `main` on 2026-10-13: one
Python demo catalog (`python -m examples.demos`, 37 scenes) runs on the private
native app host, the `demos`, `cycle-demos` and `capture` tasks launch it, and
the C++ `dart-demos` application is removed. This folder tracks only the
loose ends below. Retire it when they are done.

[RESUME](RESUME.md) has the handoff prompt and the exact next steps.

## What landed

| PR | Packet | Change |
| --- | --- | --- |
| #3732 | WP-PD.0 | Decision D31b and the plan |
| #3745 | WP-PD.1 | Bundled Dear ImGui docking release |
| #3746 | WP-PD.2, WP-PD.6 | GUI-free `dart-demo-models` library, bound as `dartpy._demos` (#3755) |
| #3747 | WP-PD.3 | Private demo host library in `dart/gui/app` |
| #3758 | WP-PD.4 | Declarative panels and the stricter scene contract |
| #3761 | WP-PD.5 | `dartpy.viewer`, the Python binding of the host |
| #3762 | WP-PD.7 | Python catalog on the native host |
| #3763 | WP-PD.10, WP-PD.11 | Scene batches S1 and S2 (with #3764) |
| #3737 | WP-PD.8 | Resting state, `HeightmapShapef` and `VoxelGridShape` bindings |
| #3744 | — | `HeightmapShape` height bounds fix |
| #3751 | WP-PD.9 | Whole-body IK, end effector, support and balance bindings |
| #3779 | — | Whole-body IK GC traversal before the hierarchy refresh |
| #3780 | WP-PD.12–WP-PD.18 | Scene batches S3–S6b, the Python `demos` tasks, and the C++ app's removal (merged into its branch as #3765, #3766, #3768, #3769, #3770, #3772 and #3773) |

Related: #3735 (API snapshots read non-arithmetic enums), #3759 (the
release-target check skips stacked PR bases) and #3767 (project-home status).

## Where the durable facts live

- Host and model architecture: [architecture](../../onboarding/architecture.md#python-demo-catalog).
- Scene rules and catalog layout: [examples README](../../../examples/README.md#adding-catalog-scenes).
- Callback ownership, errors and threads: [Python bindings](../../onboarding/python-bindings.md#viewer-app-callbacks).
- Catalog gates: [testing](../../onboarding/testing.md#demo-catalog-verification).
- M6b's remaining scope (`dart.render`, the final viewer API, migrating the
  interim `dartpy.gui` primitives) and the known host gaps:
  [DART 7 program](../dart7_python_first/README.md#demo-consolidation-d31b).
- Deferred CHANGELOG entries, which land with M6b's consolidation PR:
  [changelog drafts](../dart7_python_first/changelog-drafts.md).
- Packet evidence: [program verification](../dart7_python_first/verification.md#demo-consolidation-evidence).

## Remaining work

1. **Post-merge CI on `main`.** At the maintainer's request, the last merges
   landed after a clean Codex review but before CI finished. Check the CI runs
   of the merge commits `a25183069` (#3758), `4dc1eb1f4` (#3761), `b26d909b2`
   (#3762), `a1dfa943b` (#3763), `2d3c0c0d6` (#3779) and `e378f985f` (#3780).
   Fix any failure at its root cause in a focused follow-up PR.
2. **Stale ssik marker.** A Codex review that arrived after #3768 merged found
   that `ssik_ik_gui` keeps its cyan solved marker at the previous solution
   when a later solve finds no branches or fails: `update_solution()` in
   `python/examples/demos/scenes/ssik_ik_gui.py` moves the marker only when a
   valid pose exists. Hide or move it whenever the solution list is cleared,
   compare with the removed C++ scene in history
   (`examples/demos/scenes/ssik_ik_gui/SsikIkGuiScene.cpp` at `a1dfa943b`),
   and add a regression test to `python/tests/integration/test_demo_scenes_s6b.py`.
3. **Windows launch tasks.** The Windows variants of `pixi run demos`,
   `cycle-demos` and `capture` have not run on Windows yet.
4. **GitHub cleanup, with maintainer approval.**
   - Resolve the #3751 review thread on the GC traversal; #3779 fixed it.
   - The merged stack #3748 still lists its PRs.
   - The remote branch `feature/port-demo-scenes-s6a` was recreated by a push
     after #3766 merged; it carries nothing that is not on `main`.

## Constraints

- Follow `AGENTS.md`, the [PR lifecycle](../../onboarding/ai-tools.md#pr-lifecycle)
  and the gates in [testing](../../onboarding/testing.md#demo-catalog-verification).
- Keep C++ API and installed headers compatible; the host and model libraries
  stay private and uninstalled.
- New scenes are Python scenes on the native host; heavy per-step math stays
  native in `python/native/demo_models`.
