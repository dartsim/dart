# Resume: py-demos Framework

## Last Session Summary

Mapped the architecture, built the cuda binding, and established the **M0
baseline: 155/155 scenes pass the full-catalog no-crash smoke** (verified
genuine — see `02-m0-baseline.md`). The catalog was unguarded, not broken, so
there are no fix-tasks. M0 pivoted to locking the guard into CI (delegated to
Codex) and deepening coverage (tier-2 panels). Orchestration model in use: this
session steers/diagnoses/verifies; Codex (`codex exec`) implements well-defined
offloaded tasks in this same working tree.

## Current Branch

`py-demos-framework` — off `main`, uncommitted. New files: the three dev-task
docs + `scripts/py_demos_smoke.py` (full-catalog no-crash smoke harness).

## Immediate Next Step

**M0 is complete** (no-crash + panels + scalable contract + real render, all
155/155, CI-guarded; commits on `py-demos-framework`). The cuda binding is built
at `build/cuda/cpp/Release-docking/python/dartpy/_dartpy*.so`.

Next is **M1**: the AVBD flagship + DART 6 LCP/SI baseline rigid showcase. First
design decision to resolve (see `01-milestones.md` "M1 architecture reality"):
how to present the AVBD solver *track* next to the in-`World` SI/IPC/BoxedLcp
baseline — side-by-side scenes vs a unified comparison scene. The flagship
`rigid_body` scene currently exposes only the in-`World` `SequentialImpulse`/`Ipc`
dropdown.

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
  Verify Codex's *artifacts* (files, test runs) — do not trust its summary text.
- M1 solver recommendation pending user confirmation (see `01-milestones.md`):
  AVBD flagship + DART 6 boxed-LCP reference baseline.

## How to Resume

```bash
git checkout py-demos-framework
git status && git log -3 --oneline
# Verify build state:
ls build/cuda/cpp/Release-docking/python/dartpy/_dartpy*.so 2>/dev/null || echo "needs build"
```

Then: finish/confirm the build, run the full-catalog no-crash smoke, record
failures in this folder, and proceed with M0 triage.
