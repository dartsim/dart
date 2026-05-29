# Filament Offscreen Viewport Spike — Dev Task

## Current Status

- [ ] Phase 0: scope + success criteria (this doc)
- [ ] Phase 1: render one `dart::gui` scene `View` to an offscreen Filament
      `RenderTarget` backed by a color `Texture`, read it back, and assert it
      matches the on-screen swapchain render (parity gate)
- [ ] Phase 2: present that texture in a host window two ways and compare —
      (A) Filament swapchain into a host-owned native child window
      (`winId()`-style handle), (B) offscreen texture composited by the host
      compositor — and record the risk/effort of each
- [ ] Phase 3: write up findings as a decision input for the
      `dartsim_gui_toolkit_decisions.md` Qt trigger; then delete this folder

## Goal

Prove (or disprove) that Filament can render the dartsim scene **to an offscreen
texture** with on-screen parity, and determine which host-composite strategy is
lower risk. This is the prerequisite the toolkit decision (Decision 3 of
`docs/design/dartsim_gui_toolkit_decisions.md`) names as the hard ~80% of _both_
a future Qt viewport _and_ a web/streaming viewer — so it is the lowest-regret
rendering investment regardless of whether Qt is ever adopted.

## Why now (not a Qt port)

The 2026-05-28 re-investigation reaffirmed "keep GLFW + Dear ImGui; do not
migrate to Qt." This spike is **not** a Qt migration. It de-risks an option and
delivers standalone value (headless render fidelity, sensor cameras, future
remote viz) without committing to a toolkit change. The live viewport today is
`createSwapChain(nativeWindow)` only (`dart/gui/detail/render_context.cpp`); no
offscreen render-to-texture path exists yet (`screenshot.cpp` uses post-render
`readPixels`, which is not the same as compositing a live texture).

## Non-Goals (for this spike)

- No Qt dependency added to the build. The host-window comparison in Phase 2 can
  use a minimal GLFW/native harness; Qt is evaluated on paper from the result.
- No production viewport rewrite, no `PanelBuilder` changes, no model/view work.
- No new public `dart::gui` surface; keep all Filament types in `detail/`.
- No macOS Metal material path (Filament materials remain opengl+vulkan).

## Key Decisions

- **Parity gate first.** Phase 1 must show the offscreen render matches the
  swapchain render before any host-composite work; otherwise the composite
  question is moot.
- **Compare both composite strategies, decide later.** Strategy A (Filament owns
  a native child window inside the host) reuses the existing
  `createSwapChain(nativeWindow)` path and is likely lowest-risk; Strategy B
  (zero-copy offscreen texture into the host's GL/Vulkan context) is the more
  flexible but less-referenced path (Filament external-texture import is
  unplanned beyond Android). The spike measures, it does not pre-judge.
- **Keep it throwaway.** Land behind a disabled-by-default smoke target or a
  scratch example; this folder is deleted when Phase 3 writes up the finding.

## Success Criteria

1. A `dart::gui::detail` offscreen render produces a color buffer that matches
   the on-screen render within a small pixel tolerance (reuse the screenshot
   readback + the existing Filament scene-extraction test fixtures).
2. A one-page risk/effort comparison of Strategy A vs B with a recommendation.
3. A clear statement of what (if anything) this changes for the Qt trigger in
   `docs/design/dartsim_gui_toolkit_decisions.md` (it informs the trigger; it
   does not fire it on its own).

## Immediate Next Steps

1. Read `dart/gui/detail/render_context.cpp` (swapchain creation),
   `frame_renderer.cpp` (render loop), and `screenshot.cpp` (readback) to find
   the smallest insertion point for an offscreen `RenderTarget` + color
   `Texture`.
2. Stand up Phase 1 behind a disabled-by-default smoke target; reuse
   `tests/unit/gui/test_filament_scene_extraction.cpp` fixtures for parity.
3. Record results here; update `RESUME.md` before ending the session.

## Related

- `docs/design/dartsim_gui_toolkit_decisions.md` — Decision 3 (this spike's
  parent) and the 2026-05-28 re-investigation addendum
- `docs/design/renderer_realtime_and_scalability.md`,
  `docs/design/filament_fidelity_profile.md` — renderer-neutral camera/loop
- `dart/gui/detail/{render_context,frame_renderer,screenshot}.cpp` — source of
  truth for the current on-screen path
