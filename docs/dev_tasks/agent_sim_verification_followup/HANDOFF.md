# Handoff: Agent 3D-Scene & Physics Verification — Follow-up

Read this top-to-bottom before doing anything. It is a self-contained resume
prompt for a fresh AI agent (Codex, Claude, or otherwise). The core task is
**shipped and merged**; this folder tracks the remaining follow-up work and
preserves the full context so you do not re-derive it.

## 1. Mission and current status

**Task:** improve DART's practices/tooling so an AI agent can understand and
verify 3D scenes and physics simulations headlessly — text-based and
visual-based — on both DART 7 (`main`) and DART 6 (`release-6.20`).

**Status: DONE and MERGED.** Two PRs shipped 2026-07-06:

- **PR #3313 → `main`** (DART 7, milestone DART 7.0), merged as
  `3cddea8cb9d`. 57 files.
- **PR #3314 → `release-6.20`** (DART 6, milestone DART 6.20.0), merged as
  `361c34401d4`. 21 files.

Both merges incorporated Codex bot-review responses on top of the original
work (added a CHANGELOG entry, a `test_render_golden_gate.py`, gui stub
updates, and hardened non-blank/corrupt-PNG and offscreen-backend validation).

**First step for you:** `git checkout main && git pull` (and for DART 6 work,
a worktree or checkout of `release-6.20`). Everything below is already in those
branches unless a section says "NOT done".

## 2. What shipped (durable homes)

Read these first — they are the source of truth and were written for you:

- **`docs/onboarding/agent-sim-verification.md`** — the how-to (owner doc): the
  full text-first + visual verification workflow and every `pixi run` command.
- **`docs/design/agent_sim_verification.md`** — the why (research + design
  record): the measured A/B evidence, the tolerance policy, rejected
  alternatives.
- **`.claude/skills/dart-verify-sim/SKILL.md`** — the consolidated skill
  (Codex sees `$dart-verify-sim`). Registered in `docs/ai/workflows.md` +
  `docs/ai/capabilities.json`.
- DART 6: **`release-6.20:docs/ai/verification.md`** "Visual Verification
  (Headless Capture)" section.

### DART 7 (`main`) capability inventory

Text (the primary oracle):

- `world.compute_step_metrics() -> StepMetrics` (energy, momentum, contacts,
  penetration, iterations, residual). C++: `dart/simulation/.../world.hpp`;
  binding: `python/dartpy/simulation/module_compute.cpp` / `module_world.cpp`.
- `dartpy.dump_scene_json(world)` / `dump_scene_text(world)` — glTF/USD-flavored
  hierarchy + flat index. Impl: `python/dartpy/_scene_dump.py`, wired via an
  `_install_scene_dump()` hook in `python/dartpy/__init__.py`.
- `pixi run trajectory-record` / `trajectory-compare` —
  `scripts/trajectory_record.py` (golden-style per-body TSV + contact JSONL),
  `scripts/trajectory_compare.py` (bit-exact checksum + per-column tolerance,
  reports first divergence).

Visual (corroboration):

- `dart.gui.render(world, camera=None, size)` → buffer-protocol `RenderedImage`
  with `.png_bytes()`. C++ `dart::gui::OffscreenRenderer` (pimpl over the
  Filament headless context) in `dart/gui/offscreen.{hpp,cpp}`; binding
  `python/dartpy/gui/offscreen.cpp`; Python world walk in
  `python/dartpy/_world_render_bridge.py`. `orbit_camera()` / `look_at()`
  helpers; `camera=None` bounds-fits.
- Viewer camera flags in `dart/gui/detail/scenes.cpp`: `--view
{three-quarter|front|side|top}`, `--camera-azimuth/-elevation/-distance/
-target`, `--turntable N`, `--fit`; multi-view/turntable capture loop in
  `dart/gui/detail/application.cpp`; shared public `sceneBoundingSphere` /
  `fitOrbitCamera` in `dart/gui/viewer.{hpp,cpp}`. Plumbed through
  `scripts/capture_py_demo.py`.
- `pixi run image-verdict` / `image-golden` / `image-sheet` —
  `scripts/image_verdict.py` (JSON verdict: non-blank, report-only contrast,
  per-pixel diff with AA-ignore + Blender two-number budget, optional numpy
  SSIM / Habitat relnorm), `image_golden.py`, `image_sheet.py`, shared
  `scripts/_image_tools.py`.

Opt-in:

- `pixi run render-golden-gate` — `scripts/render_golden_gate.py` (opt-in,
  not default-CI; goldens are backend-specific, curated locally with
  `-- --update`, gitignored under `*.png`).
- `pixi run rerun-trajectory` — `scripts/rerun_trajectory.py` (rerun.io,
  guarded import, graceful + nonzero exit when `rerun-sdk` absent; never a
  core/wheel dep).

### DART 6 (`release-6.20`) capability inventory

- `dart::gui::osg::setUpOffscreenViewer` / `captureOffscreen` /
  `defaultAgentCamera` in `dart/gui/osg/OffscreenViewer.{hpp,cpp}` (additive
  over `Viewer`); examples `sleeping` / `ssik_ik_gui` migrated onto it;
  `pixi run capture` task; `DART_ENABLE_GUI_OSG_SMOKE_TESTS` ctest.
- dartpy bindings `viewer.setUpOffscreen` / `captureOffscreen` +
  `dart.gui.osg.defaultAgentCamera` (camelCase — DART 6 dartpy is pybind11 +
  camelCase, unlike DART 7 nanobind + snake_case).
- Ported `image_verdict/golden/sheet` + `_image_tools` + tests.

## 3. Key decisions and evidence (do not re-litigate)

The maintainer interview (2026-07-05) fixed four calls; the A/B studies fixed
the rest. Full detail in `docs/design/agent_sim_verification.md`.

1. **Lead with text, corroborate with images** — measured, not assumed. Two
   blind-judge A/B studies: text metrics/trajectory detect ~all seeded physics
   defects (metrics 16/16); images miss _static geometry_ defects (ground
   penetration, interpenetration) entirely; single ~1280 px frames beat
   4-frame filmstrips; UI panels hurt comprehension.
2. **Near-parity on both branches** (DART 7 Filament + DART 6 OSG).
3. **Golden gates opt-in only, never default-CI** — llvmpipe-vs-GPU render
   variance makes strict pixel gates flake-prone. Tolerance policy: Blender
   two-number (`--fail 0.016 --failpercent 1`) + pixelmatch AA-ignore + Habitat
   relnorm option; contrast is report-only unless `--require-contrast`.
4. **rerun.io is the one opt-in richer-inspection extra**; meshcat/viser/
   Foxglove/k3d/pyrender were rejected (no automated-oracle story / redundant).
5. **No published conservation threshold exists** — derive an empirical drift
   band from DART's own runs; use bit-exact checksums for determinism.

## 4. Gotchas and lessons learned (operational gold — saves you hours)

- **Filament frame-skip drops one-shot captures.** A headless offscreen grab
  must NOT gate on `beginFrame()`'s pacing bool; flush between warmup frames
  and capture unconditionally, mirroring `dart/gui/detail/offscreen_parity.cpp`.
  This bit both the offscreen render and the camera multi-view loop.
- **Filament ignores `LIBGL_ALWAYS_SOFTWARE`** on this host and grabs the
  NVIDIA GPU. So rendered goldens are backend-specific — that is why the golden
  gate is opt-in and goldens are curated locally per backend.
- **The smoke `analyze_contrast` heuristic false-fails minimal scenes.** A
  bare-box render is legitimately flat; use `analyze_basic` (non-blank) for
  the render-API smoke (PLAN-012's gate is "nonblank", not shadow-contrast).
- **DART 6 dartpy layout differs.** Module is `dartpy.cpython-*.so` (not
  `_dartpy`); import with `PYTHONPATH=build/<env>/cpp/Release/python/dartpy`
  (the dir containing the `.so`) and do NOT also put the source `python/` dir
  ahead of it or it shadows the compiled module. Build dartpy with
  `pixi run build-py-dev` (separate from `pixi run build`, which only does C++).
- **DART 7 dartpy stubs drift.** Regenerating all stubs locally produces
  ~2600 lines of unrelated nanobind-version churn; there is no CI stub gate.
  Only the source-export line in `generate_stubs.py` was committed — see the
  deferred stub-regen item below.
- **conda-forge has no linux-64 Xvfb.** DART 6 headless capture needs a real X
  server (this host has one at `DISPLAY=:0`) or a system-installed Xvfb.
- **`check_collision_runtime_isolation.py` walked into `.claude/worktrees/`**
  (gitignored local worktrees) and false-flagged their test includes; fixed by
  skipping `.agents/.claude/.codex/.opencode`. If you add a new filesystem
  scanner, skip those dirs too.
- **The pre-commit guard lints `CLAUDE_PROJECT_DIR` (the main checkout).** When
  committing in a sibling worktree, commit with `git -C <worktree> ...` — the
  guard's cross-worktree exemption then applies.
- **Codex sandbox limits** (if you are Codex): `.git` is read-only (leave
  changes uncommitted for the orchestrator to commit); `pixi run lint` fails
  in-sandbox (Black process pool) — do not "fix" the lint tooling, let the
  orchestrator run lint; the sandbox can write the main working tree but not
  sibling worktrees.

## 5. Remaining / follow-up work (prioritized)

None of this blocks the shipped tooling; it extends or hardens it. Current
status is mixed: A, C, D, and E have local follow-up implementation; B still
needs real blind-judge rows before it has measured research evidence.

### A. dartpy stub regeneration (deferred housekeeping) — DONE locally

New DART 7 public symbols are absent from the committed `.pyi` stubs
(`StepMetrics`, `dump_scene_json/text`, `OffscreenRenderer`, `RenderedImage`,
`render`, `orbit_camera`, `look_at`, `sceneBoundingSphere`, `fitOrbitCamera`,
camera view presets). `python/dartpy/gui/__init__.pyi` got partial updates
during review, but a clean full pass is owed.

- **Local evidence:** `pixi run generate-stubs` was run and rejected because it
  produced broad nanobind formatting/signature churn. The retained patch is
  minimal: `StepMetrics`, `world.compute_step_metrics()`,
  `dump_scene_json/text`, and `MultibodyIntegrationFamily` appear in the
  committed stubs while the pre-existing GUI render stubs stay intact.
- **Gate:** `python/tests/unit/simulation/test_scene_dump.py` checks the
  committed stub symbols.

### B. Image A/B round 2 (multi-view + annotations) — PARTIAL

Round 1 used a fixed per-scene camera and left round 2 blocked on camera
control (WP-ASV.4), which is now shipped. Round 2 should measure, with the
same blind-judge protocol: multi-view grids vs single view vs turntable;
Set-of-Mark-style annotations (numbered bodies, ground-line, contact markers,
CoM/velocity trails) on vs off — the research predicts annotations recover the
static-geometry defects images missed in round 1.

- **Local progress:** `pixi run image-ab-round2 -- generate` prepares a
  blinded packet over real DART 7 `dart.gui.render` images for all four
  capture arms. The companion score command merges completed observations into
  the `pixi run image-ab-study` reducer. The durable design doc records the
  round-2 protocol and makes clear that fixture rows are not research evidence.
- **Remaining next step:** collect real blind-judge rows over seeded defects and
  clean controls for single-view, multi-view, turntable, and annotated captures;
  record results and fold conclusions into
  `docs/onboarding/agent-sim-verification.md`.
- **Completion evidence still needed:** measured detection deltas that either
  confirm or refute the multi-view/annotation hypothesis; updated or explicitly
  reaffirmed default-capture recommendation.

### C. Adopt the verdict/golden tooling in the solver plan gates — DONE locally

Solver plans (PLAN-081/082/083/104) require "headless Filament captures" as
completion gates. The image-verdict/golden tooling now makes those concrete.

- **Local evidence:** PLAN-083's acceptance criteria now names
  `pixi run py-demo-capture`, `pixi run image-verdict`, and opt-in
  backend-local `pixi run render-golden-gate` commands as the visual-gate
  template.

### D. Scene-dump coverage + a scene-diff verdict — DONE locally

`_scene_dump.py` covers box/sphere/capsule/cylinder/mesh; add remaining shape
types (cone, plane, heightmap, soft-body) as needed, and consider a
`dump_scene`-diff verdict (structural JSON diff) analogous to
`trajectory-compare` for "did the scene I built match the scene I intended".

- **Local evidence:** scene dumps defensively serialize cone, heightmap, and
  soft-body-like fields when those shape objects expose the expected attributes;
  `pixi run scene-diff` emits a structural JSON verdict for scene dumps.

### E. VLM-in-the-loop verification (research) — DONE locally

The whole point is agent consumption. A natural extension: a helper that
captures the recommended representation set (text bundle + one 1280 px frame +
optional grid) and packages it for a VLM verification call. Grounded in the
WP-ASV.1 memo (Claude 28×28 patch tokens, Set-of-Mark, contact sheets).

- **Local evidence:** `pixi run verification-bundle` packages primary text
  artifacts, one still frame, an optional grid, hashes, and a provider-neutral
  review prompt. The helper deliberately stops before model/provider invocation.

## 6. How to resume / operate

- Orient: `git checkout main && git pull`; read the three durable docs (§2) and
  invoke `$dart-verify-sim` (Codex) or the `dart-verify-sim` skill.
- Build DART 7 dartpy: `pixi run build-py-dev-docking`; tests via
  `PYTHONPATH=build/default/cpp/Release-docking/python:python
.pixi/envs/default/bin/python -m pytest python/tests/unit/...`.
- Gates: `pixi run test-all` (authoritative), `pixi run check-lint`,
  `check-api-boundaries`, `check-ai-commands`. Run `pixi run lint` before every
  commit.
- DART 6: work in a `release-6.20` checkout/worktree; build C++ with
  `pixi run build` and dartpy with `pixi run build-py-dev`; import with
  `PYTHONPATH=build/default/cpp/Release/python/dartpy`; the OSG offscreen path
  needs `DISPLAY=:0` (or Xvfb).
- **Dual-branch rule:** bug fixes touching both lines need PRs to BOTH `main`
  and the active `release-6.*`. New features can be single-branch.
- **GitHub mutations (push, PR, comments, re-triggers) require explicit
  maintainer approval.** Never reply inline to `[bot]` review comments; verify
  their claims locally and fix silently.

## 7. Provenance

Original multi-agent orchestration ran 2026-07-05/06: a maintainer decision
interview, 14 work packets (WP-ASV.1–14) across Codex executors and Claude
worktree teammates, an adversarial pre-PR review workflow, and `test-all`
before each PR. The original `docs/dev_tasks/agent_sim_verification/` tracking
folder was retired at completion (durable artifacts promoted to §2). This
follow-up folder should itself be deleted in the PR that closes out items A–E,
per `docs/dev_tasks/README.md`.
