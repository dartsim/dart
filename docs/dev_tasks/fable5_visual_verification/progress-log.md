# Progress Log — Fable 5 Visual Verification

- 2026-07-09 — Researched the merged visual-reasoning PR lineage (13 PRs,
  DART 6 + 7); headline: zero PRs embed media. → `03-pr-research.md`.
- 2026-07-09 — Audited existing tooling (dart-verify-sim stack, debug
  producers, offscreen renderer, image tooling, bundling); identified the
  deltas. → `02-design.md` capability table.
- 2026-07-09 — Implemented the offscreen `DebugScene` channel
  (`dart/gui/offscreen.{hpp,cpp}`, nanobind, stubs) with overlay purity
  (clear on plain render); fixed the refresh/clear ordering bug found by the
  end-to-end pixel test.
- 2026-07-09 — Implemented `python/dartpy/_debug_layers.py` (10 layers,
  trajectory tracker, label compositing with verified projection math) and
  `python/dartpy/_view_quality.py` (ViewReport, occlusion via pick rays,
  deterministic viewpoint selection, focus framing).
- 2026-07-09 — Implemented `scripts/agent_capture.py` (deterministic capture
  harness + sidecars), `image_compose.py`, `evidence_select.py`,
  `evidence_publish.py`; pixi tasks; `_image_tools.py` composites.
- 2026-07-09 — Tests: 17 (view quality + layers) + 4 (offscreen) + 14
  (compose/select/publish) Python, 1 C++ overlay-lifecycle test; classified
  the pre-existing same-renderer determinism failure with baseline evidence.
- 2026-07-09 — #2984 retrospective: three-claim evidence packet
  (PBR sweep, fidelity side-by-side + diff heatmap, offscreen debug overlay),
  selection + PR-section generation with gh-release dry run.
  → `04-retrospective-2984.md`, `capture_2984.py`.
- 2026-07-09 — Docs: agent-sim-verification.md, dart-verify-sim skill,
  CHANGELOG entry, stubs.
