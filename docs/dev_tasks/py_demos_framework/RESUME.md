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

## Immediate Next Step

**M1 is in progress.** Verify the contact-baseline increment, then capture the
rigid-body SI vs boxed-LCP visual evidence and curate the AVBD constraint
showcase.

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
git checkout py-demos-framework
git status && git log -3 --oneline
# Verify build state:
ls build/cuda/cpp/Release-docking/python/dartpy/_dartpy*.so 2>/dev/null || echo "needs build"
```

Then: run focused C++/dartpy verification for the writable contact solver
method, run the py-demos panel/smoke guards, and proceed with M1 visual capture.
