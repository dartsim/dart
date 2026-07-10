# Verification Log — Fable 5 Visual Verification

## 2026-07-09 — Implementation + tests

**What changed:** offscreen `DebugScene` channel (C++ + nanobind + stubs);
`_debug_layers.py` (10 world-derived layers, label compositing, projection);
`_view_quality.py` (ViewReport, assess_view, select_viewpoints, framing);
`scripts/agent_capture.py`, `image_compose.py`, `evidence_select.py`,
`evidence_publish.py`; `_image_tools.py` composites; pixi tasks; docs/skill;
CHANGELOG; #2984 retrospective (`capture_2984.py`, 04-retrospective-2984.md).

**Checks run (all on Linux x86_64, NVIDIA RTX 5000 Ada, DISPLAY=:0):**

- `python/tests/unit/gui/test_view_quality.py` +
  `test_debug_layers.py` — 17 passed (geometry-only, no GPU).
- `python/tests/unit/gui/test_offscreen_render.py` — 4 passed (includes new
  debug-differs/clears and annotated-labels tests).
- `python/tests/unit/test_image_compose.py`, `test_evidence_select.py`,
  `test_evidence_publish.py` — 14 passed.
- C++ `UNIT_gui_OffscreenRenderer --gtest_filter='*DebugScene*'` — OK under
  llvmpipe (built with `DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS=ON`, flag
  restored OFF afterward).
- Live verification: occlusion wall detected (fraction 1.0) and reselection
  found clean views (score 0.977, occlusion 0); projection matched rendered
  centroid sub-pixel (110.0/98.6 vs 109.5/98.4); debug renders deterministic
  across repeated fresh-renderer calls; capture sidecars carry reproduce
  commands.
- Retrospective #2984: 3 claims covered, selection + publication dry run
  green; view-quality loop caught and fixed a too-far fidelity framing.

**Known gaps / classified failures:**

- PRE-EXISTING: `OffscreenRenderer.RendersDescriptorSceneDeterministically`
  fails on this host at baseline commit a70fc2ed5cb (before any change),
  both NVIDIA GL and llvmpipe Mesa 26.0.3 — same-renderer replay drift.
  Environment-specific; the smoke suite is opt-in
  (`DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS`). Not weakened, not caused here.
- `pixi run lint` — green (auto-fixes applied, exit 0).
- Combined suite after lint formatting: 59 passed (new tests + pre-existing
  image sheet/verdict/golden, verification bundle, trajectory tests that
  share `_image_tools.py`); `pixi run check-dartpy-import-layout` — passed.
