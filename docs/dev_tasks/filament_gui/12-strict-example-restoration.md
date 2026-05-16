# Strict Example Restoration

## Current Correction

Maintainer correction on 2026-05-16: there are many more examples that are not
fully restored, and `examples/fetch/` is the concrete warning case. A migrated
source file, successful build, headless screenshot, and runner coverage do not
prove parity with the historical GUI example.

Every pre-existing user-facing example remains parity pending until its current
`dart::gui` source has been checked against the pre-promotion historical source
and every user-visible behavior is either:

- Restored through public `dart::gui`.
- Superseded by the official Filament/GLFW/ImGui renderer direction.
- Recorded as a named public API gap with the missing API spelled out.

Do not mark an example complete because it is source-owned, uses
`ApplicationOptions::world`, or avoids legacy renderer types. Those are entry
conditions, not the restoration bar.

## Fetch Re-Open Rule

`examples/fetch/` stays open as the representative strict-audit case. Previous
Fetch checkpoints restored the MJCF world load, Bullet preference, robot/object
initialization, mocap weld reset, target-following pre-step callback, target
visual, work-area grid, panel controls, camera/run defaults, README, and marker
tests. That is still not enough to end the audit if another historical
user-visible behavior is found.

The next Fetch pass should compare the current source against
`520993d7301^:examples/fetch/main.cpp` again and challenge these details:

- The target affordance should match the historical interactive-frame behavior
  as closely as the public `dart::gui` selection/drag API allows.
- The panel should preserve the old user-facing title and whole-body-motion
  explanation; menu-bar/collapsible placement can be superseded only if the
  promoted panel API cannot express it without backend hooks.
- Any extra promoted controls, such as Step, are acceptable only if they do not
  replace or hide historical Play/Pause/Exit/Help/About affordances.
- If a gap is real, fix it before moving on; if it requires a new public
  renderer-neutral API, record that API gap explicitly in
  `11-example-parity-audit.md`.

The current Fetch repair addresses the first point by replacing the generic
3-axis target cross with two transparent green target bars on the same
selectable `SimpleFrame`.

## Active Queue

Use `11-example-parity-audit.md` as the live per-example table. The queue is
not limited to the current cursor. Examples already touched by recent
checkpoints may still need strict re-open if their historical controls, panels,
camera defaults, command-line options, README expectations, and marker guards
are not itemized.

Immediate order:

1. Update the active docs to reflect the latest pushed `rigid_cubes`
   checkpoint and this broader correction.
2. Re-open `examples/fetch/` for one more strict comparison and repair any
   concrete gap found through public `dart::gui`.
3. `examples/coupler_constraint/` was committed and pushed as
   `3945c65852c Restore coupler constraint controls` after mandatory lint and
   post-lint focused checks.
4. Audit and restore `examples/drag_and_drop/` next. The historical source used
   an `InteractiveFrame` at `(4, -4, 0)`, a child red box, X/Y/Z markers, a
   640x480 window, camera home from `(20, 17, 17)` to the origin, console
   instructions, and a README that described the interaction contract. Restore
   everything expressible through public `dart::gui`; record true
   InteractiveFrame-style rotation/handle manipulation as a public API gap
   rather than reintroducing OSG.
   This checkpoint was pushed as
   `1297aca1fe6 Restore drag and drop affordances`.
5. Audit and restore `examples/human_joint_limits/` next. The historical
   example depended on TinyDNN-backed custom arm/leg constraints and required
   ODE/Bullet; the current branch no longer carries TinyDNN. Restore the live
   world, joint-limit enforcement, default size, instructions, README, and
   marker guards now, and record the TinyDNN/custom neural-network constraint
   path as a named dependency/API follow-up unless a maintained replacement is
   added.
6. Continue through the remaining `Needs strict audit` and
   `Recent parity checkpoint; still subject to strict audit re-open` rows.

## Checkpoint Rule

For each reasonable checkpoint:

- Update this dev-task documentation before making scope or design changes.
- Run focused build/smoke validation for the example.
- Run `pixi run lint` before committing, even for docs-only checkpoints.
- Commit with a plain descriptive title.
- Push to the tracked remote branch without opening a pull request.
