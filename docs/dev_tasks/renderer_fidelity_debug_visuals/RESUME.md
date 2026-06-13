# Resume: Renderer Fidelity + First-Class Debug Visuals (PR #2984)

## Last Session Summary

Implemented and verified the full first slice (A1/A3/A4/B1/B2/B3 of the
design doc), fixed all internal code-review findings, merged upstream `main`
(adapting to #2932's removal of the legacy world / example / extraction
test), opened draft PR
[#2984](https://github.com/dartsim/dart/pull/2984) (milestone DART 7.0), and
triggered `@codex review`. CI Lint then failed on a docs-policy check and
Codex submitted one P2 finding. Follow-up commits indexed the design doc,
merged latest `origin/main`, and removed unsupported body/joint/velocity
built-in panel controls while keeping the helpers available through
`debugProvider`. A later Codex P2 finding noted that descriptor-backed Python
GUI scenes did not propagate PBR values; the local follow-up exposes the
optional `MaterialDescriptor` PBR fields in dartpy and copies non-negative
`VisualAspect` values through `_world_render_bridge.py`. The next Codex review
flagged strict-symbol-visibility linkage for
`shapeUsesLitMaterialOverride`; the local follow-up exports the helper and
marks the test's local declaration with `DART_GUI_API`. A fourth Codex review
flagged dartpy parity for the new B3 debug helpers; the local follow-up exposes
the new `DebugDrawOptions` fields plus `make_joint_axis_debug_lines` and
`make_velocity_debug_lines`.

## Current Branch

`feature/gui-fidelity-debug-visuals` — PR #2984, targeting `main`. Verify the
current local and remote state with `git status --short --branch` and
`git log -4 --oneline` before continuing.

## Current Reality (2026-06-13 snapshot — verify fresh)

All four Codex P2 fixes are pushed (head `30d15865a81`, branch in sync with
origin) and Codex's re-review at 2026-06-12T22:49Z returned **"Didn't find
any major issues."** A further `@codex review` was posted at
2026-06-13T01:19Z and is awaiting a response. CI on this head: 12 pass,
18 pending, 0 failures, 2 skipping.

## Immediate Next Step

1. Wait for the pending CI matrix (`gh pr checks 2984`; resilient poll, not a
   single `--watch`) and for Codex's response to the 01:19Z trigger.
2. When CI is green and Codex stays clean: resolve the four addressed review
   threads via GraphQL and mark the draft ready for review (draft-ready fast
   path in `docs/onboarding/ai-tools.md`), both after maintainer approval.
3. Merge waits for branch protection / required checks.

## Open Review Findings (all addressed and pushed)

- **Codex P2** (review `4482029601`, `dart/gui/detail/panel.cpp:507`) has a
  local fix: the built-in panel now hides body/joint/velocity controls that
  cannot be backed by the current world-less static overlay path. The
  `DebugDrawOptions` fields and `BodyNode` helpers stay for `debugProvider`.
  Do NOT reply to the bot; fix silently, resolve the thread via GraphQL only
  after maintainer approval.
- **Codex P2** (review `4489497208`,
  `python/dartpy/dynamics/shape_frame.cpp:55`) has a local fix:
  descriptor-backed Python GUI scenes now propagate `VisualAspect` metallic,
  roughness, and reflectance values into `gui.MaterialDescriptor`. Do NOT reply
  to the bot; fix silently, resolve the thread via GraphQL only after the fix is
  pushed.
- **Codex P2** (review `4489544810`,
  `dart/gui/detail/renderable_factory.hpp:65`) has a local fix:
  `shapeUsesLitMaterialOverride` is exported consistently so
  `UNIT_gui_DebugVisuals` can link under DLL / strict-symbol-visibility builds.
  Do NOT reply to the bot; fix silently, resolve the thread via GraphQL only
  after the fix is pushed.
- **Codex P2** (review `4489608235`, `dart/gui/debug.hpp:108`) has a local
  fix: dartpy exposes the joint-axis/velocity debug option fields and helper
  functions. Do NOT reply to the bot; fix silently, resolve the thread via
  GraphQL only after the fix is pushed.

## CI State (snapshot, verify fresh)

- `Lint` job failed on head `5ae5b7a9d81` because `check_docs_policy.py`
  required the new design doc to be indexed in `docs/design/README.md`. Verify
  the current head with `pixi run python scripts/check_docs_policy.py`.
- Rest of the matrix was pending at session end: `gh pr checks 2984`.

## Context That Would Be Lost

- **Upstream #2932 changed the ground**: legacy `dart::simulation::World`
  (skeleton-hosting), `examples/gui_scene_diagnostics/`, and
  `tests/unit/gui/test_filament_scene_extraction.cpp` (6210 lines) are GONE
  on `main`. `extractRenderables(world)` returns `{}` and
  `extractDebugLines(world, ...)` ignores the world. `ApplicationOptions`
  lost its `world` member; scenes come from `makeSceneFromOptions` +
  `renderableProvider`. Our merge commit `9185368b215` documents the
  adaptation.
- **Tests** live in `tests/unit/gui/test_gui_debug_visuals.cpp`
  (`UNIT_gui_DebugVisuals`, 5 tests, links `dart-gui-core`). The PBR flow
  test uses `dart::gui::describeShapeFrame(*shapeNode)`;
  `shapeUsesLitMaterialOverride` is declared locally in the test because
  `detail/renderable_factory.hpp` pulls Filament math headers whose include
  dirs are PRIVATE to `dart-gui-core`.
- **Stub churn trap**: `pixi run generate-stubs` output does NOT match the
  committed `python/stubs/**` style on current main (~900-line diff in
  `gui/__init__.pyi`, plus stale-stub drift in `constraint.pyi` /
  `simulation.pyi` that is not ours to fix). Keep committed style; our
  genuine additions (VisualAspect PBR methods in `dynamics.pyi`) are already
  committed.
- **Verification evidence so far**: `pixi run lint`; `pixi run build`;
  `pixi run test-unit` (162/162); `UNIT_gui_DebugVisuals` 5/5 (run binary
  directly to confirm all five execute); dartpy PBR smoke (camelCase +
  snake_case); focused GUI Python coverage
  (`python/tests/unit/gui/test_gui_scene.py`, 24/24); headless
  `dartsim --headless --frames 1 --screenshot` renders correctly pre+post
  merge (PPM converted to PNG and inspected — A1/A3 must be judged visually
  per `docs/onboarding/gui-rendering.md`). Pre-merge, the provider path was
  verified visually end-to-end (12-line selection wireframe around the
  picked body rendered via `debugProvider`).
- **Policy reminders for this PR**: no AI attribution in commits/PRs; merge
  (never rebase) latest `main` before EVERY push; pushes / PR comments /
  thread resolution need explicit maintainer approval; never reply to bot
  review comments — the code change is the response; re-trigger
  `@codex review` at most once per review-fix round, only after an approved
  push that addressed its comments.
- Local visual artifacts from the session (not committed, regenerate if
  needed): `/tmp/dartsim_fidelity.png`, `/tmp/diag_zoom.png`,
  `/tmp/dartsim_postmerge.png`.

## How to Resume

```bash
cd /path/to/dart
git checkout feature/gui-fidelity-debug-visuals
git status && git log -4 --oneline
gh pr view 2984 --json state,statusCheckRollup | head -40
gh pr checks 2984
# Unaddressed Codex comments:
gh api repos/dartsim/dart/pulls/2984/comments \
  --jq '.[] | "\(.path):\(.line // .original_line) \(.body)"'
```

Then: push the validated local commits, re-trigger `@codex review`, and keep
watching `gh pr checks 2984` (long matrix; use a resilient poll, not a single
`--watch`).

## Validation Battery (this area)

```bash
pixi run lint
pixi run python scripts/check_docs_policy.py
pixi run ninja -C build/default/cpp/Release UNIT_gui_DebugVisuals dartsim
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_DebugVisuals$'
build/default/cpp/Release/bin/UNIT_gui_DebugVisuals   # confirm all 5 tests run
pixi run test-unit
build/default/cpp/Release/bin/dartsim --headless --frames 1 --width 1280 \
  --height 720 --screenshot /tmp/dartsim.ppm   # inspect visually
PYTHONPATH=build/default/cpp/Release/python pixi run python -c \
  "import dartpy as d; s=d.dynamics.Skeleton('x'); j,b=s.create_free_joint_and_body_node_pair(); \
   n=b.create_shape_node(d.dynamics.BoxShape([1,1,1])); v=n.create_visual_aspect(); \
   assert v.get_metallic()<0; v.set_metallic(.9); assert abs(v.get_metallic()-.9)<1e-12; print('OK')"
```
