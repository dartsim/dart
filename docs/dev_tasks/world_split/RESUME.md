# Resume: World Split

## Last Session Summary

The task folder now has the required status and resume files. Existing design
and migration notes describe the split between the stable classic world and the
experimental ECS-backed world.

## Current Branch

`docs/dart-next-workflow` — contains documentation and AI workflow updates.

## Immediate Next Step

Decide the Python binding module name for the experimental world, then update
`01_migration.md` and route implementation through the appropriate DART task
workflow.

## Context That Would Be Lost

- The current preferred name in the design note is
  `dartpy.simulation_experimental`.
- The main alternative still recorded is `dartpy.next`.
- The classic `dart::simulation::World` should remain stable during the DART 7
  transition.
- Experimental code should stay under `dart::simulation::experimental` and
  behind explicit experimental build surfaces.

## How To Resume

```bash
git checkout docs/dart-next-workflow
git status --short --branch
```

Then inspect:

- `docs/dev_tasks/world_split/README.md`
- `docs/dev_tasks/world_split/00_design.md`
- `docs/dev_tasks/world_split/01_migration.md`
- `docs/plans/dashboard.md`
