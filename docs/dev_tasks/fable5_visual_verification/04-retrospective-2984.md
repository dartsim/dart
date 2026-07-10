# Retrospective Visual Verification: PR #2984

Validation of the Fable 5 workflow against a representative merged PR
([#2984 — "Improve renderer fidelity and promote debug visuals into
dart::gui"](https://github.com/dartsim/dart/pull/2984), merged 2026-06-14).

## 1. What the PR changed, and why its verification was insufficient

#2984 added per-shape PBR material control (metallic/roughness/reflectance),
full texture mip chains, a `DART_GUI_HIGH_FIDELITY` headless fidelity mode,
the `debugProvider` DebugScene seam, and joint-axis/velocity debug-line
producers. It is an almost entirely _visual_ feature set.

Its PR body verified with unit tests and prose ("judged visually per
docs/onboarding/gui-rendering.md") and explicitly declined to attach the
captures: _"Local capture artifacts available on request
(`/tmp/dartsim_fidelity.png`, …); attaching transient images requires the web
PR editor, so they are not committed."_ Reviewers approved renderer-fidelity
claims without seeing a single rendered pixel. It also deferred "re-wiring
world-derived debug overlays" — debug visuals existed but nothing composed
them from a World, and the offscreen path could not draw them at all.

## 2. Camera views and debug layers added by this reconstruction

- **PBR sweep** — a deterministic descriptor grid (metallic rows ×
  roughness columns) with in-image captions, front orbit camera.
- **Fidelity pair** — the same scene/camera rendered by
  `OffscreenRenderer(high_fidelity=False)` vs `True`, framed with
  `dart.gui.frame_body(..., "box")` after `assess_view` showed a whole-scene
  fit left the subject too small to judge shading (the workflow's
  detect-and-reframe loop in action; the view report is saved beside the
  images). Composited side-by-side with labels plus a diff heatmap that
  localizes the change (edges, contact creases, shadows).
- **Debug overlay** — world-derived layers on the headless path:
  body frames, contact markers + normals, ball trajectory polyline,
  velocity arrows, and name labels — i.e. the follow-up #2984 deferred,
  now demonstrated offscreen.

## 3. Resulting artifacts

Generated (not committed — media stays out of git) by:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python docs/dev_tasks/fable5_visual_verification/capture_2984.py --out <dir>
pixi run evidence-select -- <dir>/candidates.json --out <dir>/selection.json
pixi run evidence-publish -- <dir>/selection.json --backend gh-release \
  --repo dartsim/dart --tag verification-media-2984 [--yes after approval] ...
```

| Artifact                                                             | Claim                                           | Selected because                                               |
| -------------------------------------------------------------------- | ----------------------------------------------- | -------------------------------------------------------------- |
| `pbr_sweep.png`                                                      | C1 per-shape PBR changes appearance             | only artifact covering C1; quality 0.90                        |
| `fidelity_compare.png` (+`fidelity_diff.png`, `fidelity_stats.json`) | C2 high-fidelity offscreen differs from default | side-by-side + quantified delta (68.9% pixels changed, max 70) |
| `debug_overlay.png` (+view report JSON)                              | C3 debug layers render offscreen                | auto-assessed view (score 0.80, no issues)                     |

`evidence_select` rejected nothing here because each artifact carries a
distinct claim; the earlier smoke run demonstrated redundancy pruning and
budget rejection (see `python/tests/unit/test_evidence_select.py`).

## 4. What the evidence proves and does not prove

**Proves:** metallic/roughness parameters visibly change materials (C1);
`high_fidelity=True` changes the offscreen output in the expected places —
AO in creases, smoother edges — with pixel statistics (C2); frames,
contacts, trajectories, velocities, and labels render on the headless path
composed from a live World (C3).

**Does not prove:** windowed-vs-offscreen parity (only offscreen modes were
compared); texture mip/anisotropy improvements (no textured asset);
metal response under image-based lighting (the neutral IBL has no specular
environment — #2984's own deferred A2 follow-up, visible as dark metallic
rows); cross-backend pixel equality (llvmpipe differs slightly from the
NVIDIA captures).

## 5. Improved PR-body verification section

The generated section (media links are `gh-release` dry-run predictions;
actual upload requires maintainer approval, or the manual web-editor flow
with the `manual` backend) lives at `<dir>/pr_section.md` after running the
commands above. It contains: environment/configuration, the three claims
with per-artifact captions, "what to look for" guidance, per-artifact
selection rationale, not-proven statements, limitations, and reproduction
commands — readable without reproducing the environment.

## Workflow lessons captured

- The detect→reframe loop caught a real inadequacy (fidelity pair too far)
  that would have shipped as weak evidence.
- Claims-first candidate manifests force every image to justify itself.
- The publication step remains the only part that needs a human: uploads
  mutate shared state and stay approval-gated by design.
