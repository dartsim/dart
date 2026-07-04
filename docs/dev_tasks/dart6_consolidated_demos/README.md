# DART 6 Consolidated Demos Application

## Goal

Replace the scattered `examples/*` and `python/examples/*` catalogs with:

- **`dart-demos`** (C++): one `dart::gui::osg` + ImGui application hosting all
  GUI demos as runtime-switchable scenes, selected from a categorized
  navigator. Modeled on DART 7 `examples/demos` (`dart-demos`) architecture but
  built on the DART 6 OSG/ImGui stack (no Filament backport).
- **`py-demos`** (Python, best effort): a consolidated `dartpy`-based demos
  runner using existing bindings only — no new Python bindings for demos.
- A small set of intentionally standalone examples (e.g. `hello_world`)
  retained; everything superseded is retired.

Quality bar (user-directed, high impact):

- Everything tunable on the fly at runtime; the app must **never crash** from
  scene switching, parameter changes, or interaction.
- ImGui workspace style/layout/scalability must be high standard — this app is
  the library's first impression.
- First-class visual debugging: simulation control, logging, scene
  tree/hierarchy + state inspection, mouse drag-force interaction, 3D gizmo
  manipulation, toggleable live profiling, RTF/FPS stats.

## Working Branches

- Task lane: DART 6.20 (`origin/release-6.20`), topic branches per phase.
- Docs + plan branch: `feature/dart6-consolidated-demos`.
- DART 7 reference: `origin/main` refs only (e.g.
  `git show origin/main:docs/design/demos_app.md`); do not depend on other
  local worktrees.

## Principles Digest (binding for this task)

- `docs/ai/principles.md`: evidence first; root-cause fixes; compatibility
  first; approval needed for pushes/PRs; scope control via this folder.
- `docs/ai/north-star.md` (6.20): compatibility lane. Examples are **not**
  installed package surface, but every retirement must come with evidence that
  no test, CI job, tutorial, or doc depends on the removed example. Public
  headers and package components stay untouched except additive `dart-demos`
  example wiring.
- DART 7 `docs/ai/north-star.md` + `docs/design/demos_app.md` (origin/main):
  architecture reference — scene-as-data registry, lazy factories, soft-fail
  scene switching, categorized navigator, transactional switches, headless
  cycle smoke, capture tooling. Use as reference evidence only; prove DART 6
  behavior directly.

## Key References

- DART 7 design doc: `origin/main:docs/design/demos_app.md`.
- DART 7 C++ app: `origin/main:examples/demos/` (registry.cpp, scenes.hpp,
  app/main.cpp). Historical commits contain the ported DART 6 catalog before
  its retirement — mine `git log origin/main -- examples/demos` for scene
  ports.
- DART 7 py-demos: `origin/main:python/examples/demos/`.
- Headless visual verification recipe: `examples/sleeping --headless` pattern
  (pbuffer + SingleThreaded + captureScreen); see PLAN.md verification section.

## Status

- 2026-07-04: task started. Recon of DART 6 examples inventory, `dart/gui/osg`
  capabilities, and DART 7 demos architecture in progress. Phased plan lands in
  `PLAN.md`.

## Decisions

- D1: C++ app builds on `dart::gui::osg::ImGuiViewer`; DART 7's `dart::gui`
  (Filament) is not backported.
- D2: py-demos uses existing `dartpy` bindings only; C++ demo features degrade
  gracefully in Python where bindings are missing (per task directive).
- D3: Cleanup keeps specialized examples that a consolidated GUI app cannot
  represent (minimum: `hello_world`); the retained set is finalized in Phase 5
  with per-example evidence.
