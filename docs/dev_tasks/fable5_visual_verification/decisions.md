# Decisions — Fable 5 Visual Verification

## 2026-07-09 — Offscreen debug channel reuses the viewer overlay path

- **Decision:** extend `OffscreenRenderer::render` with a `DebugScene`
  overload routed through `detail::refreshProviderDebugOverlay`
  (`materials.debugColor`), rather than converting debug lines to shaded
  renderables in Python.
- **Evidence:** overlay machinery already proven in the viewer
  (`dart/gui/detail/debug_overlay.cpp`); Python-side thin-box conversion
  would lose unlit always-on-top treatment. Verified by pixel-diff tests.
- **Tradeoff:** requires a C++ rebuild; kept minimal (one controller member,
  clear-on-plain-render for purity).
- **Revisit:** if a depth-tested debug material variant lands (#2984 B4).

## 2026-07-09 — Contacts layer built from `world.collide()` in Python

- **Decision:** mirror `extractContactDebugLines` marker/arrow construction
  in `_debug_layers.py` over `simulation.Contact` objects.
- **Evidence:** the C++ extractor requires `collision::CollisionResult`,
  which the DART 7 Python World does not expose (verified: TypeError);
  contacts from `world.collide()` are the objects agents already consume.
- **Revisit:** if a CollisionResult binding appears, dedupe to C++.

## 2026-07-09 — Labels composited in Python

- **Decision:** offscreen labels are drawn into the returned RGBA buffer via
  projection math + a 3x5 bitmap font; no Filament text rendering.
- **Evidence:** viewer labels are ImGui-pass-only (`ui_frame.cpp:143`);
  projection matches the render sub-pixel (verified against rendered
  centroid). Deterministic, dependency-free.

## 2026-07-09 — View metrics are geometry-first

- **Decision:** view-quality checks (coverage, subject size, occlusion,
  ambiguity) run on projections and CPU pick rays, not rendered pixels.
- **Evidence:** works without a GPU (tests run in 0.3 s headless);
  mirrors the repo's text-first policy; pixel checks remain in
  `image_verdict.py` as corroboration.

## 2026-07-09 — Evidence minimality: every artifact needs a claim

- **Decision:** `evidence_select` rejects artifacts that add no claim
  coverage; extra viewpoints earn inclusion via their own claims.
- **Tradeoff:** stricter than "keep distinct angles"; keeps PR evidence
  small and forces claims to be explicit.

## 2026-07-09 — Publication backends are approval-gated

- **Decision:** `evidence-publish` defaults to `manual` (web-editor
  placeholders per dart-pr.md); `gh-release` uploads only with `--yes`,
  under the shared-state-approval rule. gh CLI has no user-attachments API;
  release assets are the only scriptable GitHub-hosted option that renders
  inline images (videos stay links — only user-attachments get the player).

## 2026-07-09 — C++ overlay test uses tolerance, not bit-equality

- **Decision:** the DebugScene lifecycle test compares with a per-pixel
  budget (tolerance 4, noise ≤64 bytes, overlay ≥256 bytes) instead of
  exact equality.
- **Evidence:** the PRE-EXISTING `RendersDescriptorSceneDeterministically`
  fails at baseline commit a70fc2ed5cb on this host (NVIDIA and llvmpipe
  Mesa 26.0.3) — same-renderer replays drift a few levels (~6 bytes/76.8k).
  Python locks bit-exactness for fresh renderers where it holds.
