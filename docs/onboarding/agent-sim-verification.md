# Agent Simulation Verification

How an AI agent (or any contributor) checks that a DART 3D scene and physics
simulation behaves correctly, without a GUI. DART's domains — dynamics,
collision, robotics — need 3D understanding that language models lack natively,
so this page collects the text-first and visual verification tooling that makes
that understanding checkable.

**Principle (measured, see below): lead with text, corroborate with images.**
A/B evidence gathered while building this tooling showed per-step metrics and
trajectories detect ~all seeded physics defects, while a blind agent misses
static geometry defects (ground penetration, interpenetration) from a rendered
image entirely. Use images to confirm _what a scene looks like_ and gross
dynamic failures (explosions, tunneling); use text to decide correctness.

## Text verification (primary)

- **Step metrics** — `world.compute_step_metrics()` returns a `StepMetrics`
  with kinetic/potential/total energy, linear/angular momentum, active contact
  count, max penetration depth, iterations, and residual. Watch energy and
  momentum for conservation, penetration for contact health.
- **Scene dump** — `dartpy.dump_scene_json(world)` / `dump_scene_text(world)`
  answer "what is in this world?": a glTF/USD-flavored hierarchy (header with
  gravity, time step, units, solver rest tolerance; bodies with mass/inertia/
  pose; joints with type/position/limits; shapes with type/size/transform) plus
  a flat index for diffing. Reading it catches structural/units bugs (gravity
  sign, a joint initialized past its limit, a missing collision shape).
- **Trajectories and contacts** — `pixi run trajectory-record` writes a
  golden-style per-body TSV (frame/time/pos/vel/contact_count under a
  gravity/time-step/rest-tolerance header) plus an optional contact-event JSONL
  (body pair, position, normal, depth). `pixi run trajectory-compare` diffs two
  runs with a bit-exact checksum mode (determinism) or a per-column tolerance
  mode, reporting the first divergence. Derive an empirical drift band from
  DART's own runs rather than importing a threshold — no engine publishes one.
- **Scene diff** — `pixi run scene-diff` compares two
  `dump_scene_json` outputs structurally, with numeric tolerance and a
  machine-readable verdict. Use it when the question is "did the scene I built
  match the scene I intended?" before relying on a rendered image.
- **Profiling** — `docs/onboarding/profiling.md` covers the text-first step
  profiler and solver diagnostics for _where time goes_.

## Visual verification (corroboration)

- **Render** — `dart.gui.render(world, camera=None, size=(w, h))` returns a
  headless RGBA image (buffer protocol; `.png_bytes()` for inline notebook
  display). `camera=None` bounds-fits the world; build an explicit camera with
  `dart.gui.orbit_camera(...)` / `look_at(...)`. A world needs collision shapes
  to produce renderables.
- **Camera + capture** — the demos viewer takes `--camera-azimuth/-elevation/
-distance/-target`, `--view {three-quarter|front|side|top}`, `--turntable N`,
  and `--fit`; multi-view and turntable captures write `shot_<view>.ppm` /
  `shot_turnNNN.ppm`. `pixi run py-demo-capture` drives this from Python (PNG,
  and MP4 with `--video` when ffmpeg is present).
- **Image verdict / golden / contact sheet** — `pixi run image-verdict`
  emits a machine-readable JSON verdict (non-blank, report-only contrast,
  per-pixel diff with AA-ignore and a Blender-style two-number budget, optional
  Habitat relative-norm and numpy SSIM). `pixi run image-golden` adds the
  golden workflow (`--update`, retries). `pixi run image-sheet` assembles a
  labeled contact-sheet grid (>=200 px tiles). Contrast is scene-dependent, so
  it is reported but only gates pass/fail with `--require-contrast`.
- **A/B study reducer** — `pixi run image-ab-study` reduces blind-judge rows
  for single-view, multi-view, turntable, and annotated captures into detection
  deltas and false-positive rates. Treat its output as research evidence only
  when the rows came from a real blind protocol; unit fixtures are not evidence.
- **Recommended default capture**: one ~1280 px frame, UI hidden, from the 3/4
  view; add a 9-frame grid for motion questions. Avoid filmstrips (they read
  worst in the A/B). Keep images as corroboration, never the sole oracle for
  static geometry.

## Opt-in extras and gates

- **Golden gate** — `pixi run render-golden-gate` renders a canonical scene and
  compares it to a locally curated golden. Opt-in, never wired into default CI;
  goldens are backend-specific (llvmpipe vs a GPU-backed Filament differ), so
  curate per backend with `-- --update` (kept out of git under `*.png`; the
  sidecar records the backend). Default tolerance is Blender-style
  `--fail 0.016 --failpercent 1`; use `--retries` for known-flaky setups and
  record why in the owning plan.
- **rerun.io** — `pixi run rerun-trajectory` logs a trajectory/contacts to
  rerun for interactive or agent inspection (`--spawn` / `--save`). Opt-in and
  never a core or wheel dependency: install `rerun-sdk` yourself; absent it the
  tool prints an install hint and exits nonzero without a traceback.
- **VLM bundle** — `pixi run verification-bundle` packages scene JSON/text,
  metrics, trajectories, contacts, one still frame, and an optional grid into a
  manifest plus provider-neutral review prompt. The bundle is for an external
  VLM or reviewer call; it does not add a network dependency or make VLM output
  a default gate.

## DART 6 (release-6.20)

The DART 6 LTS branch has the equivalent capture path over OpenSceneGraph:
`dart::gui::osg::setUpOffscreenViewer` / `captureOffscreen` (and the dartpy
bindings), `pixi run capture`, and the same ported image verdict/golden/sheet
tooling. Headless capture there needs a real X server or Xvfb (conda-forge has
no linux-64 Xvfb build); its `docs/ai/verification.md` documents the workflow.
For physics determinism without rendering, `pixi run bm-boxes-headless` prints
per-step checksums for byte-exact build-to-build comparison.

## Requirements

GUI rendering needs a Filament-capable build (`DART_BUILD_GUI=ON`, the Linux
x86_64 default). The image tooling is stdlib-only with optional numpy (SSIM);
no new runtime or wheel dependencies. The `dart-verify-sim` skill points here.
