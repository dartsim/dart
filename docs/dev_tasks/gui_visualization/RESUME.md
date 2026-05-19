# Resume: GUI Visualization VSG Migration

## Current Status

`docs/dev_tasks/gui_visualization/README.md` is the task source of truth. The
recorded status is Phase 1-4 complete for the VSG foundation and ImGui
integration work.

This task is not part of the native-collision completion scope. This file
exists so active dev-task folders satisfy the repository documentation policy
that each task has both `README.md` and `RESUME.md`.

## How To Resume

```bash
git status --short --branch
sed -n '1,220p' docs/dev_tasks/gui_visualization/README.md
```

Before editing this task, read:

1. `docs/dev_tasks/gui_visualization/README.md`
2. `docs/onboarding/gui-rendering.md`
3. `docs/onboarding/testing.md`

Run `pixi run lint` before committing any changes.
