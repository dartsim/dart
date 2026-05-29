# Resume: Filament Offscreen Viewport Spike

## Last Session Summary

Spike was scoped (not started) as a follow-up to the 2026-05-28 GUI toolkit
re-investigation, which reaffirmed "keep GLFW + Dear ImGui." Phase 0 (this doc)
defines the parity gate and the Strategy A vs B comparison. No code written.

## Current Branch

`feature/dartsim-gui-toolkit-revisit` — carries the toolkit-decision addendum,
the `dartsim/ui` CI boundary guard, and this spike plan. No implementation yet.

## Immediate Next Step

Read `dart/gui/detail/render_context.cpp`, `frame_renderer.cpp`, and
`screenshot.cpp` to find the smallest place to add an offscreen Filament
`RenderTarget` + color `Texture`, then stand up the Phase 1 parity check behind
a disabled-by-default smoke target.

## Context That Would Be Lost

- This is a decision-input spike, NOT a Qt port. The toolkit verdict is
  unchanged (keep ImGui+GLFW). Do not add a Qt build dependency.
- The live viewport is `createSwapChain(nativeWindow)` only; `screenshot.cpp`'s
  `readPixels` is post-render readback, not live-texture compositing.
- Parity gate (Phase 1) must pass before the host-composite comparison (Phase 2)
  is worth doing.
- On completion, extract the key finding into the toolkit decision doc and
  `git rm -r` this folder in the same PR (dev_tasks cleanup rule).

## How to Resume

```bash
git checkout feature/dartsim-gui-toolkit-revisit
git status && git log -3 --oneline
```

Then: read the three render-path files above and implement the Phase 1 offscreen
parity check.
