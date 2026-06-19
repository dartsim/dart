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

## Current Branch

`fix/py-demos-selection-crash` - published as PR #3084. The PR includes the
selection debug-overlay fix, scriptable capture-state restoration, labeled
stateful captures, and the AVBD showcase packet.

## Immediate Next Step

**M1 is in progress.** Keep the scripted selection repro above in the validation
set, use the labeled scripted capture-state path for rigid-body SI vs boxed-LCP
visual packets, and use the dedicated AVBD showcase packet for the modern
rigid-constraint track. The next useful slice is a reviewer-facing visual packet
pass: capture full SI/boxed-LCP and AVBD packets, inspect screenshots/review
indexes, then decide whether M1 needs a unified comparison scene or whether the
two complementary packets are enough for review.

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
git checkout fix/py-demos-selection-crash
git status && git log -3 --oneline
# Verify build state:
ls build/cuda/cpp/Release-docking/python/dartpy/_dartpy*.so 2>/dev/null || echo "needs build"
```

Then: run the py-demos panel/smoke guards, and proceed with reviewer-facing M1
visual packet capture for the boxed-LCP contact baseline and AVBD showcase.
