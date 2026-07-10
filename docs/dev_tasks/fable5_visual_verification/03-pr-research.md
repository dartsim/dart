# DART 6/7 Visual-Reasoning PR Research Summary

Merged-PR lineage for visual debugging/verification for AI agents, recovered
from PR bodies and metadata (July 2026). Base-branch key: `main` = DART 7,
`release-6.*` = DART 6 LTS. Media audit method: every body scanned for
`user-images.githubusercontent.com`, `github.com/user-attachments`, and
markdown image embeds.

## Headline finding

**None of the 13 PRs in the lineage embeds any visual artifact.** Every
visual PR cites local `/tmp/*.png` paths as "inspected"; #2984 states the
cause outright: _"attaching transient images requires the web PR editor, so
they are not committed."_ The tooling to render, capture, and judge images
exists — the publication link is the missing piece.

## Lineage (chronological)

| PR                                                                                                     | Branch       | What it added                                                                                                                                                                                                                                                                  | Verification shown in-body                                      |
| ------------------------------------------------------------------------------------------------------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------- |
| [#2647](https://github.com/dartsim/dart/pull/2647), [#2687](https://github.com/dartsim/dart/pull/2687) | main         | Filament renderer path, promoted to maintained                                                                                                                                                                                                                                 | build/tests                                                     |
| [#2774](https://github.com/dartsim/dart/pull/2774)                                                     | main         | offscreen render-to-texture parity gate (headless seed)                                                                                                                                                                                                                        | text parity numbers                                             |
| [#2690](https://github.com/dartsim/dart/pull/2690)                                                     | main         | collision visual-verification sandbox: contact/normal/depth overlays, broad-phase debug snapshots, gizmos, labels                                                                                                                                                              | benchmark/coverage tables; screenshot smoke pass/fail, no image |
| [#2836](https://github.com/dartsim/dart/pull/2836)                                                     | main         | modern viewer theme (+CUDA deformable toggle)                                                                                                                                                                                                                                  | "before/after confirm the look" prose, no image                 |
| [#2986](https://github.com/dartsim/dart/pull/2986)                                                     | main         | work-packet evidence harness (resolved-solver identity in packets)                                                                                                                                                                                                             | docs only                                                       |
| [#2984](https://github.com/dartsim/dart/pull/2984)                                                     | main         | per-shape PBR, mip chains, `DART_GUI_HIGH_FIDELITY` headless fidelity, `debugProvider` DebugScene seam, joint-axis/velocity debug primitives                                                                                                                                   | unit tests; local captures declined                             |
| [#2992](https://github.com/dartsim/dart/pull/2992)                                                     | main         | rigid-body verification workflow: 36→53-row navigator, capture packets, manifests, review_index.html                                                                                                                                                                           | manifest counts (2544 PNGs, 181/181 links) — zero images shown  |
| [#2994](https://github.com/dartsim/dart/pull/2994)                                                     | main         | golden trajectories for default World::step (text-first seed)                                                                                                                                                                                                                  | strong text: determinism, fault injection                       |
| [#3063](https://github.com/dartsim/dart/pull/3063)                                                     | main         | stabilized py-demos panels; runtime contact-solver selection                                                                                                                                                                                                                   | one `/tmp` screenshot "inspected"                               |
| [#3084](https://github.com/dartsim/dart/pull/3084)                                                     | main         | capture hardening (degenerate debug-geometry crash fix, capture routes, scene-state JSON)                                                                                                                                                                                      | timing tables; non-blank writes; no render shown                |
| [#3304](https://github.com/dartsim/dart/pull/3304)                                                     | release-6.20 | soft_bodies capture: headless flags, Y-up camera fix, two-sided/translucent soft meshes                                                                                                                                                                                        | 4 before/after `/tmp` PNGs "inspected", none shown              |
| [#3313](https://github.com/dartsim/dart/pull/3313)                                                     | main         | **keystone**: OffscreenRenderer + `dart.gui.render`, camera flags/turntable, StepMetrics, scene dump, trajectory golden/compare, image verdict/golden/sheet, dart-verify-sim skill; policy justified by two blind-judge A/B studies (text catches physics defects images miss) | tests + byte-identical goldens; no image                        |
| [#3314](https://github.com/dartsim/dart/pull/3314)                                                     | release-6.20 | DART 6 OSG port: setUpOffscreenViewer/captureOffscreen, `pixi run capture`, ported image tooling                                                                                                                                                                               | non-blank captures + verdict JSON; no image                     |
| [#3320](https://github.com/dartsim/dart/pull/3320)                                                     | main         | follow-ups: scene diff, verification bundle, image A/B study + round-2 packet tooling                                                                                                                                                                                          | 21 pytest; A/B machinery never exercised visibly                |

## Progression arc

"Render it" (Filament, #2647→#2687→#2774) → "render internals for
debugging" (#2690, #2984) → "capture it reproducibly" (#2992, #3063, #3084,
#3304) → "let an agent audit it, text-first" (#3313, #3314, #3320).

## Recurring gaps (drove the Fable 5 design)

1. **No PR embeds visual evidence** — the publication step was never built.
2. **Fixed/preset cameras** — #3313's named views are a fixed enumeration;
   nothing selects viewpoints adaptively per scene or detects bad views.
3. **Evidence is metadata, not the artifact** — counts, checksums, and
   "non-blank" assertions stand in for the images themselves.
4. **Evidence not tied to claims** — visual fixes verified by prose.
5. **Debug layers exist but never surface as evidence** — overlays from
   #2690/#2984 are viewer-only; the offscreen path had no debug channel,
   and no producer aggregates layers from a DART 7 World (body-frame/COM
   flags were dead, labels ImGui-only, no trajectory layer).
6. **Text-first oracle under-exploited visually** — captures were not paired
   with the step-metrics/trajectory text that validates the physics shown.

## Lessons applied

- Publication needs a scriptable GitHub-hosted backend plus the documented
  manual flow; media stays out of git either way.
- View adequacy must be measurable (coverage/crop, subject size, occlusion,
  ambiguity) so an agent can detect and replace bad views deterministically.
- Debug layers must reach the headless renderer and be composable per-World.
- Every artifact must carry its claim, rationale, and reproduction command.
- Keep text primary: view reports and capture sidecars are JSON first.

## Retrospective target

**#2984** — a purely visual feature set (PBR fidelity + debug visuals) whose
body explicitly declined to attach its capture artifacts, and whose deferred
follow-up ("re-wiring world-derived debug overlays" and depth-tested debug
material) is exactly what the Fable 5 offscreen debug channel + world-layer
composition implement. Runner-up candidates: #2690, #3304 (DART 6), #2992,
#2836, #3084, #3063.
