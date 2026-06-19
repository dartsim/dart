# Resume: py-demos Framework

## Last Session Summary

M0 is complete: 155/155 scenes pass the full-catalog no-crash smoke, tier-2
panel coverage, scalable registry coverage, and the real-viewer render smoke
(see `02-m0-baseline.md`). The branch is now in M1: rigid body remains the
flagship domain, with AVBD as the modern solver track and boxed-LCP as the DART
6 contact baseline.

The first M1 contact-baseline increment makes `World.contact_solver_method`
writable after construction and exposes that choice in the flagship
`rigid_body` Python demo panel, replay state, and capture metadata. Manual
testing found realtime workflow rows should stay on the Sequential Impulse
rigid-body solver; SI-vs-IPC inspection belongs in `rigid_solver_compare` and
explicit IPC shelf scenes.

Interactive selection found one M0 gap: a click/force-drag with no cursor motion
could enqueue a zero-length debug overlay segment, trip Filament's empty-AABB
precondition, and then fail shutdown while destroying `dartDebugColor`.
Reproducer:

```bash
pixi run py-demos -- --scene rigid_body --headless --frames 4 \
  --width 640 --height 480 --screenshot /tmp/rigid_body.ppm \
  --scripted-force-drag 1:sphere_0_visual:0,0,0:2
```

The fix hardens both debug-line generation and the Filament renderable factory
against degenerate debug primitives.

The next M1 slice added scriptable capture-state restoration:
`py-demo-capture -- --scene-state-json '{"controls":{"contact_method_index":1}}'`
restores scene-owned replay state before the viewer starts. The `rigid_body`
workflow panel now advertises the boxed-LCP baseline capture command with
`--capture-label boxed_lcp`, so stateful A/B captures use distinct default
output directories and artifact stems. Default/CUDA captures resolved
`contact_solver_method=BOXED_LCP` in their manifests.

The contact-baseline path is now also a first-class workflow packet:

```bash
pixi run py-demo-capture -- --rigid-workflow --contact-baseline-only \
  --output-dir /tmp/dart_capture_rigid_contact_baseline
```

It captures the `rigid_body` Sequential Impulse and boxed-LCP contact-policy
variants as one review-indexed packet with per-row capture labels, state
metadata, and row rerun commands.

The current M1 slice curates the AVBD rigid-constraint showcase from the
existing `avbd_*` scenes instead of creating a new unified scene. The dedicated
packet command is:

```bash
pixi run py-demo-capture -- --rigid-workflow --avbd-showcase-only \
  --output-dir /tmp/dart_capture_rigid_avbd_showcase
```

It captures fixed-joint/contact, breakable-joint, spherical breakable-joint,
revolute-motor, and prismatic-motor AVBD rows as a complementary M1 showcase
beside the World contact-policy / boxed-LCP baseline. Default and CUDA packet
row captures for `avbd_rigid_revolute_motor` completed with
`solver=avbd_rigid_joints` in the workflow manifests.

The full reviewer-facing packet pass completed on PR #3084 head `fd8db58`:

- Default contact-baseline packet:
  `/tmp/dart_capture_rigid_contact_baseline_full_default_3084_fd8db58`
  (`real 32.58s`).
- CUDA contact-baseline packet:
  `/tmp/dart_capture_rigid_contact_baseline_full_cuda_3084_fd8db58`
  (`real 33.50s`).
- Default AVBD showcase packet:
  `/tmp/dart_capture_rigid_avbd_showcase_full_default_3084_fd8db58`
  (`real 35.37s`).
- CUDA AVBD showcase packet:
  `/tmp/dart_capture_rigid_avbd_showcase_full_cuda_3084_fd8db58`
  (`real 36.40s`).

All four workflow manifests report `status=complete`; the review indexes report
complete guidance, solver identity, and scene metrics with zero failed rows.
Visual inspection found the AVBD rows readable as docked visual evidence. The
contact-baseline screenshots are intentionally similar, so the review-index
labels, scene-state metadata, `contact_solver_method` solver identity, and
metrics are the distinguishing evidence. This is enough for the current PR
evidence; a unified comparison scene is deferred unless review asks for a
stronger single-scene visual.

A follow-up local improvement makes packet review pages report their M1 phase
maps for contact-baseline and AVBD-only packets instead of showing `phases 0`.
It also preserves `scene_state_json` in each row's `open live` command, so the
boxed-LCP contact-baseline row opens the same state that the capture used.
The live rigid workflow panel now advertises the matching boxed-LCP `py-demos`
command beside the boxed-LCP capture command, so the panel opens and captures
the same stateful baseline.
The `rigid_body` scene panel also has one-click Sequential Impulse and boxed-LCP
baseline preset buttons that set the contact solver and reset the scene through
the same replay/capture state path.

The next local UI follow-up adds an `Open contact comparison` button to the
`rigid_body` front-door panel. It routes through the existing panel
`request_scene_switch` hook to `rigid_contact_solver_compare`, giving users a
direct path from the baseline scene to the side-by-side Sequential
Impulse/boxed-LCP comparison row.

The fresh Codex review on PR #3084 head `e9ef404` found a valid state-override
edge case: `DART_DEMOS_SCENE=rigid_body pixi run py-demos --
--scene-state-json ...` rejected the override even though the environment
variable selects a single supported scene. The local fix resolves the
state-override target from explicit `--scene`, then `DART_DEMOS_SCENE`, then the
injected default `rigid_body` scene, while still rejecting true multi-scene
cycle runs.

A broad local M0 regression recheck completed on PR #3084 local head `bd0b8e7`:
default `pixi run py-demos-smoke --json-out
/tmp/py_demos_smoke_default_pr3084_local.json` passed 155/155 scenes in
`real 47.96s`, and CUDA `pixi run -e cuda py-demos-smoke --json-out
/tmp/py_demos_smoke_cuda_pr3084_local.json` passed 155/155 scenes in
`real 76.18s`.

PR #3084 merged to `main` as `0e6dcd0` on 2026-06-19. The current follow-up
branch adds a durable real-viewer regression guard for the original
zero-motion `rigid_body` scripted force-drag crash path, so the crash is covered
by pytest instead of relying only on manual PR evidence.

The same follow-up branch also advances the M1 `rigid_body` front door with
one-click material examples. The panel now has Default, Slide, and Bounce
presets that set friction/restitution and reset through the existing replay and
capture state path, giving reviewers quick rigid-body material cases without
leaving the flagship scene.

The latest local increment promotes those material examples into the workflow
capture helper: `py-demo-capture -- --rigid-workflow --material-examples-only`
captures the Default, Slide, and Bounce `rigid_body` variants as a
review-indexed packet with state metadata, capture labels, rerun commands, and
the same workflow guidance fields used by the contact-baseline and AVBD packets.

The full material-example packet pass completed on local head `2b887338002`:

- Default material-example packet:
  `/tmp/dart_capture_rigid_material_examples_full_default_2b887338002`
  (`real 63.16s`).
- CUDA material-example packet:
  `/tmp/dart_capture_rigid_material_examples_full_cuda_2b887338002`
  (`real 94.86s`).

Both workflow manifests report `status=complete`, three captured
`material_examples` rows (`default_material`, `slide_material`,
`bounce_material`), complete guidance, solver identity, scene metrics, and
friction/restitution state metadata for the Slide and Bounce variants.

The latest local UI slice makes the `rigid_body` front-door panel report the
active material preset (or `Custom`) beside the live friction/restitution values
and adds an `Open material mixing` route to `rigid_material_mixing`, matching
the existing contact-comparison route pattern.

## Current Branch

`fix/py-demos-selection-regression-guard` - branched from current `main` after
the #3084 merge. It currently contains the scripted-selection integration guard
plus the `rigid_body` material-example preset, packet, full-packet evidence, and
material-route UI increments. Keep any remaining edits narrow, keep the dev-task
handoff current, and validate the exact default/CUDA py-demos front doors before
publishing a follow-up PR.

## Immediate Next Step

**M1 is in progress.** For this branch, run the focused regression test:

```bash
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run python \
  -m pytest \
  python/tests/integration/test_demos_cycle.py::test_rigid_body_scripted_selection_force_drag_is_stable -q
```

Then keep the direct default and CUDA front-door commands in the validation set:
`pixi run py-demos -- --scene rigid_body --headless --frames 4 --width 640 --height 480 --screenshot /tmp/rigid_body.ppm --scripted-force-drag 1:sphere_0_visual:0,0,0:2`
and the matching `pixi run -e cuda py-demos` command.

For the material-preset panel increment, also run:

```bash
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run python \
  -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_rigid_body_panel_material_example_presets_reset_scene \
  python/tests/unit/test_py_demo_panels.py::test_rigid_body_panel_material_status_tracks_custom_sliders \
  python/tests/unit/test_py_demo_panels.py::test_rigid_body_panel_routes_to_material_mixing \
  -q
```

For the material-example workflow packet increment, also run:

```bash
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run python \
  -m pytest \
  python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_capture_material_examples_only \
  python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows \
  -q
pixi run py-demo-capture -- --rigid-workflow --material-examples-only \
  --dry-run --output-dir /tmp/dart_capture_rigid_material_examples_dry_run
```

After this follow-up, continue M1 by using the dedicated contact-baseline packet
for rigid-body SI vs boxed-LCP visual evidence and the dedicated AVBD showcase
packet for the modern rigid-constraint track.

Re-run any M0 guard:

```bash
PYTHONPATH=build/cuda/cpp/Release-docking/python:python .pixi/envs/cuda/bin/python \
  scripts/py_demos_smoke.py            # logic (build/step/providers/panels)
  # add --render for the real-viewer render sweep
```

## Decisions locked this session

- M1 solver: **AVBD flagship + DART 6 boxed-LCP baseline** (IPC deferred).
  Note the architecture reality in `01-milestones.md` (World rigid solvers are
  only SI+IPC; LCP is a `ContactSolverMethod`; AVBD is a separate track).
- First M1 increment: enhance the existing `rigid_body` scene before creating a
  new comparison scene. This keeps the baseline path visible in the flagship
  scene while AVBD showcase curation proceeds separately.
- AVBD showcase curation: keep M1 as two complementary packets for now. The
  AVBD packet is scoped to the existing `sx` rigid-constraint scenes and does
  not claim solver-enum parity with the World contact-policy baseline.
- Capture-state overrides belong to scenes that already expose
  `replay_restore_state`; unsupported scenes should fail clearly rather than
  silently pretending a panel state was applied.
- M0 triage: **quarantine to green** — non-runnable/planned scenes are marked
  skipped (tracked as deferred), not crashed and not force-fixed.
- Sibling worktrees (`task_6` memory allocator, `task_7` WP-091 enum) keep
  running in parallel — isolated worktrees, low overlap; watch WP-091's
  multibody-selector enum for a small rebase when it lands on main.
- Quarantine signal already partly exists: planned scenes carry
  `info["planned_status"]` / `info["planned_world_port"]`; PLAN-083 rows carry
  `info["plan083_smoke_command"]`.

## Context That Would Be Lost

- Build/run requires the pixi env interpreter + built binding, e.g.:
  `PYTHONPATH=build/cuda/cpp/Release-docking/python:python .pixi/envs/cuda/bin/python -m examples.demos --list`
  (plain `python` lacks numpy/dartpy; `pixi run py-demos` rebuilds + opens GUI).
- `--list`, `--cycle-scenes --frames N --headless`, `--screenshot`, `--out`,
  `--gpu/--no-gpu` are the runner CLI. Catalog = `python/examples/demos/registry.py`.
- The viewer is C++ (`dart::gui::run_demos`); Python provides scenes, `pre_step`
  hooks, `ScenePanel`s, `debug_provider` overlays, replay metadata.
- Codex is invoked as: `codex exec -s workspace-write -C <repo> -o <out.txt> "<prompt>"`.
  Verify Codex's _artifacts_ (files, test runs) — do not trust its summary text.
- M1 solver direction is confirmed (see `01-milestones.md`): AVBD flagship +
  DART 6 boxed-LCP reference baseline.

## How to Resume

```bash
git checkout fix/py-demos-selection-regression-guard
git status && git log -3 --oneline
# Verify build state:
ls build/cuda/cpp/Release-docking/python/dartpy/_dartpy*.so 2>/dev/null || echo "needs build"
```

Then: run the focused scripted-selection regression guard, the default/CUDA
manual front-door checks, and `pixi run lint` before any commit or approved
push.
