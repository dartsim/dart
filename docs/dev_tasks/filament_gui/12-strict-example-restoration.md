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

2026-05-16 follow-up: after the human joint-limits checkpoint, maintainer
steering again called out that many more examples are still not fully restored,
with `examples/fetch/` as the concrete example. The active Fetch re-open must
challenge the earlier conclusion that two target bars plus translation are
enough: the old `InteractiveFrame` target exposed axis arrows, planar
translation tools, and rotation rings, and the old panel embedded the shared
viewer instructions. Restore the strongest public `dart::gui` replacement now
and keep exact mouse rotation-gizmo parity as a named public manipulation API
gap if it cannot be expressed without legacy renderer types.

Implementation state for this re-opened Fetch slice: the source uses one
transparent green line target handle with local axes/rings, adds
renderer-neutral keyboard actions for target rotation/reset, restores the
shared viewer help text in the panel, and now promotes selected-frame rotation
through public `dart::gui`. Ctrl-Shift-left drag rotates the selected target,
with X/Y/Z selecting local target axes as the renderer-neutral replacement for
the old `InteractiveFrame` mouse rotation tools.

Validation state: focused Fetch build/CTest, direct and pixi headless
screenshot smokes, Python C++ example-runner tests, aggregate `examples` build,
`git diff --check`, mandatory lint, and post-lint focused smoke checks have
passed locally for this slice.

2026-05-16 follow-up during the Atlas Puppet checkpoint: maintainer steering
again called out that many more examples are not fully restored and named
`examples/fetch/` specifically. Treat the previous Fetch restored state as
provisional only. After the current Atlas Puppet checkpoint is committed and
pushed, perform a fresh Fetch historical-source comparison and keep Fetch open
until every remaining user-visible behavior is restored or named as a public
`dart::gui` API gap.

## Active Queue

Use `11-example-parity-audit.md` as the live per-example table. The queue is
not limited to the current cursor. Examples already touched by recent
checkpoints may still need strict re-open if their historical controls, panels,
camera defaults, command-line options, README expectations, and marker guards
are not itemized.

Immediate order:

Current override after the Atlas Puppet checkpoint: re-open `examples/fetch`
again because maintainer steering explicitly identified it as still not fully
restored. Do not rely on earlier restored-state wording in the inventory.

1. Update the active docs to reflect the latest pushed
   `18dd2a70dd5 Audit headless simulation example` checkpoint and the active
   speed-test strict-audit slice.
2. Re-open `examples/fetch` now. The selected-frame rotation repair is
   implemented and locally validated. Pre-lint and post-lint focused checks
   passed; commit and push it before moving to the remaining strict-audit
   examples.
3. Audit and restore `examples/imgui/` next. The current source is a generic
   box/slider panel, while the historical example demonstrated panel
   extension, an empty target frame with drag controls, custom key handling,
   gravity controls, viewer help, and camera/run defaults. Restore everything
   expressible through public `dart::gui`; record headlight toggles,
   key-release callbacks, camera-inspector text, and pre/post render or
   post-step hooks as public API gaps.
   This checkpoint was pushed as
   `63f990d5251 Restore panel extension example`.
4. Audit `examples/csv_logger/` next. The historical source and README are
   byte-for-byte identical to the current files, so this slice should preserve
   the non-GUI command-line logger, add source-marker guards for the CLI/CSV
   contract, validate a short CSV run, and avoid adding `dart::gui` or renderer
   dependencies.
   Implementation and pre-lint validation are complete: focused build/CTest,
   direct `csv_logger --steps 3` CSV verification, Python C++ example-runner
   tests, and aggregate `examples` build passed.
   Post-lint focused build/CTest, direct CSV verification, and
   `git diff --check` also passed.
   This checkpoint was pushed as
   `7ec64c3609e Audit CSV logger example`.
5. Audit `examples/headless_simulation/` next. The historical source and
   README are byte-for-byte identical to the current files, so this slice
   should preserve the non-GUI deterministic batch-simulation workflow, add
   source-marker guards for the CLI/seed/progress contract, validate a short
   headless run, and avoid adding `dart::gui` or renderer dependencies.
   Implementation and pre-lint validation are complete: focused build/CTest,
   direct `headless_simulation --steps 3 --dt 0.002 --seed 17` output
   verification, Python C++ example-runner tests, and aggregate `examples`
   build passed.
   Post-lint focused build/CTest, direct output verification, and
   `git diff --check` also passed.
   This checkpoint was pushed as
   `18dd2a70dd5 Audit headless simulation example`.
6. Audit `examples/speed_test/` next. The historical source and README are
   byte-for-byte identical to the current files, so this slice should preserve
   the non-GUI timing benchmark workflow, add source-marker guards for scene
   loading, dynamics/kinematics benchmark loops, result summaries, and README,
   and avoid adding `dart::gui` or renderer dependencies. Validate with focused
   build/unit coverage; do not add quick-run controls or execute the long
   benchmark in this checkpoint.
   Implementation and pre-lint validation are complete: focused build/CTest,
   Python C++ example-runner tests, and aggregate `examples` build passed. The
   benchmark executable itself was not run because the preserved historical
   behavior is an intentionally long timing loop with no bounded mode.
   Post-lint focused build/CTest and `git diff --check` also passed.
   This checkpoint was pushed as
   `824c520659c Audit speed test example`.
7. Audit `examples/unified_loading/` next. The historical source and README
   are byte-for-byte identical to the current files, so this slice should
   preserve the non-GUI shared `ReadOptions` loading workflow, add
   source-marker guards for world/skeleton load toggles, format/root-joint
   parsing, package mappings, summary output, and README, validate a short
   direct load run, and avoid adding `dart::gui` or renderer dependencies.
   Implementation and pre-lint validation are complete: focused build/CTest,
   direct `unified_loading --no-world --format urdf --sdf-root-joint fixed`
   output verification, Python C++ example-runner tests, and aggregate
   `examples` build passed.
   Post-lint focused build/CTest, direct output verification, and
   `git diff --check` also passed.
8. `examples/coupler_constraint/` was committed and pushed as
   `3945c65852c Restore coupler constraint controls` after mandatory lint and
   post-lint focused checks.
9. Audit and restore `examples/drag_and_drop/` next. The historical source used
   an `InteractiveFrame` at `(4, -4, 0)`, a child red box, X/Y/Z markers, a
   640x480 window, camera home from `(20, 17, 17)` to the origin, console
   instructions, and a README that described the interaction contract. Restore
   everything expressible through public `dart::gui`; record true
   InteractiveFrame-style rotation/handle manipulation as a public API gap
   rather than reintroducing OSG.
   This checkpoint was pushed as
   `1297aca1fe6 Restore drag and drop affordances`.
10. `examples/human_joint_limits/` was committed and pushed as
    `cf0ed62209e Restore human joint limits live example`. The historical
    example depended on TinyDNN-backed custom arm/leg constraints and required
    ODE/Bullet; the current branch no longer carries TinyDNN. Restore the live
    world, joint-limit enforcement, default size, instructions, README, and
    marker guards now, and record the TinyDNN/custom neural-network constraint
    path as a named dependency/API follow-up unless a maintained replacement is
    added.
11. Continue through the remaining `Needs strict audit` and
    `Recent parity checkpoint; still subject to strict audit re-open` rows.
    The Atlas Simbicon checkpoint has been committed and pushed as
    `82e39c45558 Restore Atlas Simbicon controller example`; exact headlight,
    shadow-toggle, depth-camera, and native window-title parity remain named
    public API gaps. `examples/free_joint_cases/` was already restored and
    pushed as `f4963df00cd Restore free joint cases controls`, so the next
    strict-audit cursor is `examples/gui_scene_diagnostics/`.
12. `examples/gui_scene_diagnostics/` has been compared against
    `520993d7301^:examples/gui_scene_diagnostics`. It is a non-interactive
    descriptor diagnostic example, and the current behavior is preserved except
    for the intentional `dart::gui::experimental` to promoted `dart::gui`
    naming. Add marker coverage and validate direct diagnostic output before
    committing this audit checkpoint. Pre-lint focused build/CTest, direct
    diagnostic output verification, Python C++ example-runner tests, and
    aggregate `examples` build passed. Post-lint focused build/CTest, direct
    diagnostic output verification, and `git diff --check` also passed. This
    checkpoint was pushed as `b80b7809570 Audit GUI scene diagnostics example`.
13. The next strict-audit cursor is `examples/rerun/`, the final current
    `Needs strict audit` row. Compare it against
    `520993d7301^:examples/rerun`, document the itemized inventory, then
    preserve, restore, or explicitly name any public API gaps before coding
    past the slice.
14. `examples/rerun/` has been compared against
    `520993d7301^:examples/rerun`. It is an unchanged no-source placeholder,
    not a migrated GUI example. Preserve the CMake early-skip behavior, README
    scaffolding, absence of source files, and absence of renderer dependencies
    with marker coverage. Pre-lint focused build/CTest, aggregate `examples`
    build, no-`rerun`-executable verification, and Python C++ example-runner
    tests passed. Post-lint focused build/CTest, aggregate `examples` build,
    no-`rerun`-executable verification, and `git diff --check` also passed.
    This checkpoint was pushed as
    `6511db33821 Audit Rerun placeholder example`.
15. No rows remain with the exact `Needs strict audit` state. Continue through
    the rows still marked as recent checkpoints subject to strict-audit
    re-open. The next cursor is `examples/add_delete_skels/`: compare it
    against `520993d7301^:examples/add_delete_skels`, document the itemized
    inventory, then preserve, restore, or explicitly name any public API gaps.
16. `examples/add_delete_skels/` strict re-open found remaining gaps after the
    recent control checkpoint: README deleted, 640x480 default missing, and
    startup cubes added despite historical spawn-only-on-`q` behavior. Repair
    those and update marker coverage before committing this checkpoint.
    Implementation and pre-lint validation are complete: focused build/CTest,
    direct and pixi headless screenshot smokes, Python C++ example-runner
    tests, and aggregate `examples` build passed. Post-lint focused
    build/CTest, direct screenshot smoke, and `git diff --check` also passed.
    This checkpoint was pushed as
    `9fa571ed585 Restore add-delete skeleton defaults`.
17. The next strict re-open cursor is `examples/atlas_puppet/`: compare it
    against `520993d7301^:examples/atlas_puppet`, document the itemized
    inventory, then preserve, restore, or explicitly name any public API gaps.
18. `examples/atlas_puppet/` strict re-open found missing README/defaults and
    deeper remaining behavior gaps. Restore the README, 1280x960 run defaults,
    and camera home now; keep target activation/deactivation, support toggles,
    relaxed-pose/posture recovery, balance recovery, and default support
    polygon/COM visualization as explicit follow-up gaps until implemented.
19. The Atlas Puppet defaults checkpoint was pushed as
    `3ca9f65c9bb Restore Atlas Puppet defaults`. The active cursor is now the
    fresh `examples/fetch/` re-open requested by maintainer correction.
20. The Fetch target-affordance checkpoint was pushed as
    `6ea2c868547 Restore Fetch target affordance`. The active cursor is now
    `examples/biped_stand/`; compare it against
    `520993d7301^:examples/biped_stand` before coding.
21. The Biped Stand instruction checkpoint was pushed as
    `b11f7db7d6c Restore Biped Stand instructions`. The active cursor is now
    `examples/box_stacking/`; compare it against
    `520993d7301^:examples/box_stacking` before coding.
22. `examples/box_stacking/` strict re-open found remaining scene, panel, and
    README gaps after the earlier solver-controls checkpoint. Repair the stack
    height, floor thickness/color, historical panel labels/help, standalone
    README sections, and marker guards now. Keep OSG-specific menu-bar
    placement, headlight toggles, camera Eye/Center/Up readout, and key-release
    callbacks explicit as public `dart::gui` API follow-ups unless the missing
    renderer-neutral APIs are added.

## Checkpoint Rule

For each reasonable checkpoint:

- Update this dev-task documentation before making scope or design changes.
- Run focused build/smoke validation for the example.
- Run `pixi run lint` before committing, even for docs-only checkpoints.
- Commit with a plain descriptive title.
- Push to the tracked remote branch without opening a pull request.
