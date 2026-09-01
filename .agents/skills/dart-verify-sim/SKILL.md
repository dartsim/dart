---
name: dart-verify-sim
description: "DART Verify Sim: text-first and visual checks for 3D scenes and physics (metrics, scene dump, trajectories, headless render, image verdict/golden)"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-verify-sim/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART Simulation Verification

Load this skill when verifying that a DART 3D scene or physics simulation is
correct — implementing, debugging, benchmarking, or reviewing dynamics,
collision, contact, or GUI output. Modern image-capable agents can inspect a
capture, but pixels do not expose solver state and machine image checks are not
semantic inspection. This tooling grounds visual reasoning without a GUI.

**Lead with text, corroborate with images.** Measured A/B evidence: per-step
metrics and trajectories detect nearly all seeded physics defects; a rendered
image alone misses static geometry defects (penetration, interpenetration).
Decide correctness from text; use images for scene comprehension and gross
dynamic failures.

## Applicability contract

Use this skill for any task whose claim depends on 3D structure or behavior:
model/scene loading, dynamics, collision/contact/constraints, simulation
stepping, GUI/rendering, or visual examples. First run a text oracle (metrics,
scene diff, trajectory/contact comparison, or focused behavioral test), then
corroborate it end to end with an assessed headless view and only the debug
layers needed by the claim. If rendering is unavailable or genuinely
irrelevant, record why and name the replacement evidence; never treat an image
as the sole correctness oracle. When the active agent accepts image input,
actually open and semantically inspect the selected capture; a passing view
report or `image-verdict` is not visual review.

## Full documentation

[`docs/onboarding/agent-sim-verification.md`](../../../docs/onboarding/agent-sim-verification.md)
— the durable guide. `docs/ai/verification.md` owns the gate policy;
`docs/onboarding/profiling.md` owns text-first profiling.

## Image-capable review loop

Every lane in the model-routing owner (`docs/ai/README.md` § "Model
Routing") is image-capable and supports native image input and
original-detail inspection; a model-upgrade audit re-verifies that property
and updates the owner, not this skill. Keep this loop capability-based.

1. State one claim, its expected visible observation, and the text oracle that
   decides correctness.
2. Capture one assessed, claim-tied view first. Add only the debug layers needed
   for the claim; use paired plain/debug views when an overlay could obscure the
   underlying scene. For a turntable or motion sequence, inspect at least the
   capture sidecar's start/middle/end frame targets; add intervening frames or a
   grid when a transient event is part of the claim.
3. Run `image-verdict` for artifact integrity, then open the selected local PNG
   with the active agent's native image viewer. Use original detail for small
   contacts, labels, bounds, or frame axes. Add a grid or another view only when
   motion, occlusion, or ambiguity requires it.
4. Record the visible observation separately from the text result. If they
   disagree, do not average them into a pass: report fail/uncertain, inspect the
   capture sidecar, reframe or recapture, and investigate the simulation state.
5. Close with pass/fail/uncertain, artifact path, view/layers, reproduction
   command, what the image shows, and what it does not prove. If native image
   review is unavailable, use `verification-bundle` for an image-capable
   reviewer and record that limitation.

## Quick commands

Text (primary):

- `world.compute_step_metrics()` — energy/momentum/penetration/contacts/residual
- `dartpy.dump_scene_json(world)` / `dump_scene_text(world)` — "what is in this
  world?" (glTF/USD-flavored hierarchy + flat index)
- `pixi run scene-diff` — structural JSON verdict for intended-vs-actual scene
  dumps
- `pixi run trajectory-record` / `pixi run trajectory-compare` — per-body TSV +
  contact JSONL; bit-exact or tolerance diff with first-divergence

Visual (corroboration):

- `dart.gui.render(world, camera=None, size=(w, h), debug=(...layers...))` →
  headless image with optional world-derived debug layers (`grid`,
  `world_frame`, `body_frames`, `coms`, `inertia_boxes`, `collision_bounds`,
  `velocities`, `contacts`, `labels`; `trajectories` additionally requires a
  sampled `dart.gui.TrajectoryTracker` via `debug_scene_for_world`);
  `dart.gui.render_annotated(...)` composites label text; `.png_bytes()` for
  notebooks; `dart.gui.orbit_camera(...)` / `look_at(...)`
- `dart.gui.assess_view(world, camera, size, focus=...)` → ViewReport with
  issues (`cropped`/`too-far`/`too-close`/`occluded`/`ambiguous`);
  `dart.gui.select_viewpoints(...)` picks deterministic best views;
  `dart.gui.frame_body`/`frame_region` reframe onto a subject. Assess first;
  fix flagged views before capturing evidence. Descriptor bounds, transformed
  corners, viewport FOV, and fit distance stay in the shared `dart::gui` core;
  Python performs only focus resolution and capture/search orchestration.
- viewer camera flags: `--view {three-quarter|front|side|top}`,
  `--camera-azimuth/-elevation/-distance/-target`, `--turntable N`, `--fit`
- `pixi run py-demo-capture` — headless PNG/MP4 capture from Python
- `pixi run agent-capture` — deterministic evidence harness: auto/explicit
  cameras, debug layers, stills/turntable/motion video, reproducible sidecar
- `pixi run image-compose` — side-by-side / blend / diff-heatmap composites
- `pixi run evidence-select` — claim-driven artifact selection with recorded
  rationale; `pixi run evidence-publish` — PR "Visual verification" section
  with a required semantic verdict and claim boundary plus GitHub-hosted media
  (manual placeholders by default; `gh-release` upload only with `--yes` +
  maintainer approval)
- `pixi run image-verdict` / `image-golden` / `image-sheet` — JSON verdict,
  golden diff, contact sheet (contrast is report-only; `--require-contrast` to
  gate); these are machine pixel checks, not semantic visual review
- `pixi run image-ab-study` — blind-judge detection deltas for single-view,
  multi-view, turntable, and annotated captures
- `pixi run image-ab-round2` — prepare a blinded round-2 packet and score
  completed judge observations

Opt-in:

- `pixi run render-golden-gate` — opt-in golden gate (backend-specific golden,
  curated locally with `-- --update`; not default CI)
- `pixi run rerun-trajectory` — rerun.io inspection (opt-in; graceful when
  `rerun-sdk` is absent)
- `pixi run verification-bundle` — package text evidence plus still/grid images
  for a provider-neutral VLM or reviewer call

Default capture for agent review: one ~1280 px frame, UI hidden, 3/4 view; add
a 9-frame grid for motion. Keep images as corroboration, never the sole oracle
for static geometry.

## DART 6 (release-6.20)

Equivalent capture over OpenSceneGraph: `dart::gui::osg::setUpOffscreenViewer`
/ `captureOffscreen` + dartpy bindings, `pixi run capture` (needs a real X
server or Xvfb), the ported image tooling, and `pixi run bm-boxes-headless` for
rendering-free determinism checksums.
