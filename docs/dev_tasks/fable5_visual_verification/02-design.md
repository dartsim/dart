# Fable 5 Visual Verification — Design

Grounded in the tooling audit (existing `dart-verify-sim` stack) and the
merged PR lineage (#2690 → #2984 → #2992 → #3063/#3084/#3090 → #3304/#3313/
#3314/#3320). Everything below composes with, and does not replace, the
text-first verification policy in `docs/onboarding/agent-sim-verification.md`.

## Capability deltas (audit result)

| Requirement        | Exists today                                                                                                                                                              | Missing                                                                                                                                                                         |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Camera control     | OrbitCamera (yaw/pitch/distance/target), named views, turntable, `--fit`, ProjectionOptions (fov/clip)                                                                    | region targeting, reframe-from-feedback, camera paths for video                                                                                                                 |
| View quality       | image_verdict: blank/contrast (+diff/SSIM vs reference)                                                                                                                   | coverage/crop, too-close/far, occlusion, ambiguity detection; auto-improve loop                                                                                                 |
| Debug layers       | producers: grid, world frame, frame triads, inertia, collision bounds, support polygon, joint axes, velocities, contact markers/normals/forces; viewer-only provider path | offscreen debug channel; world-level aggregation (nothing walks a World); body frames/COM (dead flags); trajectory traces; labels outside ImGui; before/after / diff composites |
| Evidence selection | verification_bundle (fixed roles, no ranking)                                                                                                                             | ranking, redundancy filtering, claims↔artifact links, recorded rationale, captions, size budget                                                                                 |
| GitHub publication | manual web-editor flow documented in dart-pr.md                                                                                                                           | scriptable upload backend, PR-body verification section generator                                                                                                               |

## Components

### 1. C++ — offscreen debug channel (surgical)

`OffscreenRenderer::render(descriptors, camera, const DebugScene& debug)`
overload. Implementation reuses the viewer's proven overlay path:
`detail::refreshProviderDebugOverlay(engine, scene, materials.debugColor,
debug.lines, debug.triangles, controller)` with a `DebugOverlayController`
member on `Impl`, cleared via `clearProviderDebugOverlay` when the debug scene
is empty and on destroy. Labels are NOT rendered in C++ (viewer renders them in
the ImGui pass, which offscreen lacks); they are composited in Python.
nanobind: optional `debug` kwarg on `OffscreenRenderer.render`.

### 2. dartpy — debug layer composition (`python/dartpy/_debug_layers.py`)

`dart.gui.debug_scene_for_world(world, options=None, *, layers=None,
contacts=None, trajectories=None, labels=...) -> DebugScene`

Aggregates per-World what the C++ loop never did, from existing producers:

- `grid`, `world_frame` — `extract_debug_lines(options)`
- `body_frames` — `make_frame_debug_lines` per body (fixes dead
  `draw_body_frames` at the composition layer)
- `coms` — axis markers at COM (pure Python; dead C++ flag)
- `inertia`, `collision_bounds`, `joint_axes`, `velocities`,
  `support_polygons` — existing `make_*` producers per body/skeleton
- `contacts` — built from `world.collide()` `simulation.Contact` objects
  (point cross + normal arrow + optional force), Python-side mirror of
  `extractContactDebugLines` (which needs a C++ CollisionResult that the
  DART 7 Python World does not expose)
- `trajectories` — NEW: polyline layer from recorded per-body position
  history (`TrajectoryTracker` helper that samples world state each step)
- `labels` — body-name/metadata `DebugLabelDescriptor`s at body origins

`dart.gui.render(world, camera, size, *, debug=..., annotate=...)` gains the
debug pass-through plus Python label compositing: project label anchors via
orbit-camera basis + perspective projection (math already exposed:
`camera_eye`, `make_orbit_camera_basis`, `PerspectiveProjection` params) and
draw text into the returned RGBA buffer with a small built-in bitmap font.

### 3. dartpy — view quality + adaptive viewpoints (`python/dartpy/_view_quality.py`)

`dart.gui.assess_view(world, camera, size, *, focus=None) -> ViewReport`
(dataclass, `to_json()`), deterministic and CPU-only (no render needed for
geometry checks; optional image for pixel checks):

- **coverage/crop**: project bounds corners of focus (or all) renderables to
  screen; fraction on-screen, subject bbox area fraction, margins
- **too close / too far**: subject screen-fraction thresholds
- **occlusion**: `pick_nearest_renderable` rays from eye to focus-body sample
  points; fraction blocked by other renderables
- **ambiguity**: projected-separation score (distinct bodies whose screen
  bboxes overlap heavily while depth-separated ⇒ ambiguous view axis)
- **pixel checks** (when an image is provided): reuse blank/contrast metrics

`dart.gui.select_viewpoints(world, size, *, focus=None, count=1,
azimuths=..., elevations=..., distance_scales=...) -> list[ViewChoice]`:
scores a deterministic candidate grid with `assess_view`, returns top-k
maximizing quality then angular diversity; each choice records camera
parameters + report + reason string (reproducibility: pure function of world
state and explicit candidate lists).

`frame_body(world, name, ...)` / `frame_region(center, radius, ...)`:
targeted reframing helpers (zoom/dolly/pan via target+distance).

### 4. scripts — capture, composites, evidence, publication

- `scripts/agent_capture.py` (new): deterministic capture harness for
  agent evidence: scene factory (same registry style as trajectory_record) or
  live world; explicit or auto-selected cameras; debug layer config; still /
  multi-view / turntable / camera-path video (PNG + ffmpeg MP4); writes a
  sidecar JSON (camera params, layers, world digest, sizes) so any capture is
  reproducible from its sidecar.
- `scripts/_image_tools.py` additions: `side_by_side`, `overlay_blend`,
  `diff_heatmap` composites; `scripts/image_compose.py` CLI for
  before/after, expected/actual, normal/debug, diff panels with labels.
- `scripts/evidence_select.py` (new): rank candidate artifacts against
  explicit claims. Inputs: candidate manifest (artifact + claim ids + view
  kind + quality/verdict JSONs). Scoring: claim coverage, view quality,
  redundancy penalty (pairwise similarity), size budget. Output: selected set
  with per-artifact rationale ("what it proves"), rejected-with-reasons, and
  a `dart.evidence_selection/v1` manifest for downstream use.
- `scripts/evidence_publish.py` (new): generate the PR-body
  "Visual verification" section from a selection manifest (what is shown,
  environment/config, what reviewers should observe, limitations, repro
  commands) with GitHub-hosted media links. Upload backends:
  - `manual` (default): emits the section with local-path placeholders plus
    the documented web-editor user-attachments instructions (dart-pr.md flow)
  - `gh-release`: uploads assets to a dedicated prerelease tag via
    `gh release upload` (scriptable; requires explicit `--yes` and is
    approval-gated per repo rule "shared state needs approval")
    Media never enters git history either way.

### 5. Tests

- `python/tests/unit/gui/test_view_quality.py` — synthetic worlds with known
  defects: occluder wall ⇒ occlusion flagged and reduced by reselection;
  off-target camera ⇒ crop flagged; far camera ⇒ too-far; selection is
  deterministic and improves the score.
- `python/tests/unit/gui/test_debug_layers.py` — layer composition counts
  (contact lines after `collide()`, frame triads per body, trajectory
  polyline growth, labels) without rendering.
- `python/tests/unit/gui/test_offscreen_render.py` — extend: debug render
  differs from plain render; label compositing writes pixels (display-gated
  like existing tests).
- `tests/unit/gui/test_offscreen_renderer.cpp` — extend: debug overload
  compiles/renders, empty scene clears overlay.
- script tests under `python/tests/unit/tools/` (pattern used for image
  tooling): compose/select/publish are pure-JSON testable without GPU.

## Decisions

1. **Contacts from `world.collide()` in Python** rather than binding a C++
   CollisionResult path: the DART 7 Python World returns
   `list[simulation.Contact]`; mirroring the marker/arrow construction in
   Python keeps one contact API for agents (evidence: type mismatch verified
   by direct call).
2. **Labels composited in Python** rather than adding text rendering to the
   Filament offscreen path: viewer labels are ImGui-pass-only today; a bitmap
   font over the returned buffer is deterministic and dependency-free.
3. **View metrics are geometry-first** (projection/ray math) with pixel checks
   as corroboration — mirrors the repo's text-first verification principle and
   keeps assessment usable without a GPU.
4. **Publication backends are opt-in and approval-gated**; the default
   generates the section + instructions without mutating shared state
   (Axiom 9).

## Non-goals

- New Filament materials/effects; mesh wireframe mode; screen-space text in
  C++; VLM-in-the-loop gating; committing media to git.
