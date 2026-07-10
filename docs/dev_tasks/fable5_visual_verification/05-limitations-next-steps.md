# Limitations and Next Steps — Fable 5 Visual Verification

## Current limitations

- **Videos don't inline in PR bodies via release assets.** Only
  user-attachments URLs get GitHub's inline player, and those can only be
  minted through the web editor (no API). `gh-release` assets render inline
  for images but appear as links for MP4s; the `manual` backend remains the
  path to inline video.
- **Occlusion probes sample bounds corners + center.** Thin or concave
  occluders between sample rays can be missed; a denser sample grid (or
  depth-buffer readback) would tighten this.
- **Ambiguity metric is pairwise screen-box IoU.** It flags stacked/aligned
  bodies but not all degenerate views (e.g. symmetric shapes hiding a flip).
- **Labels are uppercase 3x5 bitmap text** — compact and dependency-free but
  not pretty; no leader lines or collision avoidance between labels.
- **Debug layers cover rigid-body worlds.** Multibody links/joint axes and
  deformable bodies reuse only some layers (frames/labels work off
  transforms; joint-axis and support-polygon layers need the skeleton-typed
  C++ producers that the DART 7 Python World does not expose).
- **Same-renderer replays drift a few pixel levels on some driver stacks**
  (pre-existing; documented in verification.md). Fresh-renderer captures are
  bit-stable; the capture harness uses fresh renderers.
- **`agent-capture` scene registry is small** (trajectory_record's three
  scenes + `module:callable` factories); py-demos scenes are reachable only
  through the existing `py-demo-capture` path.
- **View-quality thresholds are fixed constants** calibrated on primitive
  scenes; large mesh scenes may want per-scene overrides.

## Next steps

1. **Inline-video publication**: when GitHub exposes an attachments API (or
   a maintainer-run upload step is scripted via browser automation policy),
   upgrade `evidence-publish` to mint user-attachments URLs; keep the
   approval gate.
2. **Depth-buffer occlusion**: read back a depth/ID buffer from the
   offscreen renderer to replace ray sampling with exact per-pixel subject
   visibility (also enables auto-crop tightening).
3. **Camera paths**: keyframed orbit interpolation for videos beyond
   turntable/azimuth sweeps (dolly + target tracking of a moving body).
4. **Multibody/deformable layers**: bind skeleton-typed debug producers
   (joint axes, support polygon) for Python multibodies; wireframe layer for
   deformable surfaces.
5. **Expected-vs-actual overlays**: render a reference world state (e.g.
   from a golden trajectory) into the same frame as ghost geometry for
   direct expected/actual composites.
6. **CI hook**: an opt-in job that runs `agent-capture` + `image-verdict`
   on the built-in scenes and uploads artifacts to the workflow run (not the
   PR), giving reviewers clickable evidence without shared-state writes.
7. **DART 6 parity**: port view-quality assessment and the evidence
   selector/publisher (renderer-independent parts run as-is) to the
   release-6.* image tooling set.
