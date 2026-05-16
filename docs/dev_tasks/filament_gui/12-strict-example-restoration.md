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

2026-05-16 follow-up during the Simple Frames checkpoint: maintainer steering
again called out that there are many more incompletely restored examples, such
as `examples/fetch/`. This overrides any temptation to treat the remaining work
as only the current linear cursor. Finish the in-flight Simple Frames
checkpoint, push it for CI, then re-open Fetch as the next active example while
continuing to drive every table row with a recent checkpoint, partial
restoration, or named public API gap through this strict historical-source
standard.

2026-05-16 follow-up during the Hubo Puppet checkpoint: maintainer steering
again called out that many examples are still not fully restored, including
`examples/fetch/`. Finish the in-flight Hubo checkpoint, push it for CI, then
make `examples/fetch/` the next active strict-audit slice before resuming the
linear cursor at `examples/hybrid_dynamics/`.

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
    This earlier preservation decision has now been superseded by the
    maintenance-cost re-open in item 39.
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
23. The Box Stacking scene-parity checkpoint was pushed as
    `7af2ad1228d Restore box stacking scene parity`. The active cursor is now
    `examples/boxes/`; compare it against `520993d7301^:examples/boxes`
    before coding.
24. The Boxes instruction checkpoint was pushed as
    `68e31a339e4 Restore boxes example instructions`. The active cursor is now
    `examples/simple_frames/`; compare it against
    `520993d7301^:examples/simple_frames` before coding.
25. Maintainer steering during the Simple Frames checkpoint explicitly
    re-emphasized that many examples remain incomplete, with
    `examples/fetch/` named as the representative case. Complete and push the
    in-flight Simple Frames checkpoint, then re-open `examples/fetch/` with a
    fresh comparison against `520993d7301^:examples/fetch/main.cpp` before
    selecting another linear cursor.
26. The Simple Frames names/arrow checkpoint was pushed as
    `061bf811d76 Restore simple frames names and arrow`. The active cursor is
    now the fresh `examples/fetch/` re-open requested by maintainer correction;
    compare it against `520993d7301^:examples/fetch/main.cpp` before coding.
27. The Fetch panel-window checkpoint was pushed as
    `7a9c712a3f1 Restore Fetch panel window controls`. The active cursor is now
    `examples/g1_puppet/`; compare it against
    `520993d7301^:examples/g1_puppet` before coding.
28. `examples/g1_puppet/` strict re-open found remaining scene/default/README
    gaps after the earlier robot/IK checkpoint: missing README, zero gravity,
    smaller renamed ground, visual-bounds root placement instead of historical
    z `0.75`, missing XY grid geometry, and no restored 1280x960 camera home.
    Repair those through promoted `dart::gui` and source-owned DART geometry
    now. Restore the always-on support polygon as a source-owned public debug
    line overlay in this checkpoint; keep per-body articulated dragging as the
    remaining named public API follow-up.
    Implementation and validation are complete: focused build/CTest, direct G1
    headless screenshot, Python C++ example-runner tests, aggregate `examples`
    build, `git diff --check`, mandatory `pixi run lint`, and post-lint
    focused checks passed locally.
29. The G1 checkpoint was pushed as
    `1e691ce891b Restore G1 Puppet scene parity`. The active strict-audit
    cursor is now `examples/hubo_puppet/`; compare it against
    `520993d7301^:examples/hubo_puppet` before coding.
30. `examples/hubo_puppet/` strict re-open found remaining
    target/default/README gaps after the earlier robot/IK checkpoint: missing
    README, smaller renamed ground, all target frames visible and solved instead
    of number-key toggled, missing support-polygon overlay, missing 1280x960
    camera home, and missing renderer-neutral replacements for the historical
    `X`/`C` support toggles, `P` DOF print, and `T` relaxed-pose reset. Repair
    those now. Keep Shift movement amplification, hold/release `R` posture
    optimization with balance mode, exact COM marker colors, and historical
    camera roll/up-vector parity as named promoted API or solver follow-ups
    unless those surfaces are added in this checkpoint.
    Implementation and pre-lint validation are complete: focused build/CTest,
    direct Hubo headless screenshot, Python C++ example-runner tests, aggregate
    `examples` build, and `git diff --check` passed locally.
31. After the Hubo checkpoint is committed and pushed, the next strict-audit
    cursor is a fresh `examples/fetch/` re-open because maintainer steering
    explicitly named it again as an incompletely restored example. Compare the
    current source and README against `520993d7301^:examples/fetch`, update
    `11-example-parity-audit.md` before coding, then repair any newly found gap
    or record the promoted public API gap. Resume
    `examples/hybrid_dynamics/` after the Fetch re-open is handled.
32. The Hubo checkpoint was pushed as
    `f7d408fea9e Restore Hubo Puppet target controls`. The active strict-audit
    slice is now `examples/fetch/`; do the fresh historical-source comparison
    before any Fetch code edits.
33. Fresh Fetch comparison found a concrete target-visual drift: the historical
    source describes the dummy object at "the cross of the two transparent
    green bars," while the current source uses a custom line/ring/arrow target
    gizmo. Restore a selectable source-owned mesh cross with two transparent
    green bars, keep promoted selected-frame manipulation for drag/rotation,
    and update README/marker guards accordingly. Implementation and pre-lint
    validation are complete: focused build/CTest, direct and pixi headless
    screenshot smokes, Python C++ example-runner tests, aggregate `examples`
    build, and `git diff --check` passed locally. Mandatory `pixi run lint`,
    post-lint focused build/CTest, post-lint direct screenshot smoke, and
    post-lint `git diff --check` also passed. This checkpoint was pushed as
    `cfce8c0186f Restore Fetch green bar target`.
34. The active strict-audit cursor is now `examples/hybrid_dynamics/`; compare
    it against `520993d7301^:examples/hybrid_dynamics` before coding.
35. `examples/hybrid_dynamics/` strict re-open found concrete gaps: missing
    README, missing 640x480 `ApplicationOptions::runDefaults`, renamed and
    recolored loaded SKEL skeletons/body visuals, missing historical harness
    console messages, and missing source-marker coverage for those restored
    contracts. Restore those now; keep the historical `(0, 1, 0)` camera up
    vector as a named public camera API gap unless a promoted camera roll/up
    surface is added in this checkpoint.
    Implementation and pre-lint validation are complete: focused build/CTest,
    direct and pixi headless screenshot smokes, Python C++ example-runner tests,
    aggregate `examples` build, and `git diff --check` passed locally.
    Mandatory `pixi run lint`, post-lint focused build/CTest, post-lint direct
    screenshot smoke, and post-lint `git diff --check` also passed. This
    checkpoint was pushed as `0800c54ec18 Restore Hybrid Dynamics defaults`.
36. The active strict-audit cursor is now `examples/joint_constraints/`;
    compare it against `520993d7301^:examples/joint_constraints` before coding.
37. `examples/joint_constraints/` strict re-open found concrete gaps: missing
    README, missing 640x480 `ApplicationOptions::runDefaults`, renamed and
    recolored loaded SKEL skeletons/body visuals, missing historical
    perturbation and harness console messages, and missing source-marker
    coverage for those restored contracts. Restore those now; keep the
    historical `(0, 1, 0)` camera up vector as a named public camera API gap
    unless a promoted camera roll/up surface is added in this checkpoint.
    Implementation and validation are complete: focused build/CTest, direct
    and pixi headless screenshot smokes, Python C++ example-runner tests,
    aggregate `examples` build, `git diff --check`, mandatory
    `pixi run lint`, post-lint focused build/CTest, and post-lint direct
    screenshot smoke passed locally. This checkpoint was pushed as
    `cf13c6e5115 Restore Joint Constraints defaults`.
38. The active strict re-open cursor is now `examples/lcp_physics/`; compare it
    against
    `520993d7301^:examples/lcp_physics` before coding past that slice.
39. Before advancing the `examples/lcp_physics/` code slice, re-open the
    previous `examples/rerun/` placeholder decision. New maintainer steering is
    to optimize maintenance effort: if the audit still finds no source files,
    executable, concrete user workflow, or downstream dependency, remove
    `examples/rerun/` from the examples set instead of preserving no-source
    placeholder support. The audit confirms those conditions, so this slice
    removes the placeholder directory, build/list entries, old preservation
    wording, and marker test, and replaces them with a removal guard. This
    checkpoint was pushed as
    `b4ed7d9d62f Remove Rerun placeholder example`; local validation covered
    focused build/CTest, aggregate examples build, example-runner tests,
    absence checks, `git diff --check`, mandatory `pixi run lint`, and
    post-lint focused checks.
40. `examples/lcp_physics/` strict re-open found concrete gaps: scene counts
    and placement drifted from the historical source, live scenario/solver
    switching and Reset/timestep/gravity panel controls were missing, README
    command-line option coverage was incomplete, and marker tests covered only
    a subset of the contract. Restore the historical scene counts, names,
    placement, local scenario/solver controls, Reset/timestep/gravity controls,
    README option inventory, and marker guards now. Keep exact ImGui line plots
    and backend-specific debug metrics as a named public panel
    plotting/render-metrics API gap. Implementation and pre-lint validation are
    complete: focused `lcp_physics` and `UNIT_gui_FilamentSceneExtraction`
    build, focused CTest, direct `--list`, software-GL screenshot and `--out`
    image-sequence analyzer checks, pixi runner screenshot analyzer check,
    Python example-runner tests, aggregate `examples` build, and
    `git diff --check` passed locally. Mandatory `pixi run lint`, post-lint
    focused build/CTest, direct `--list`, software-GL screenshot analyzer
    check, and post-lint `git diff --check` also passed. This checkpoint was
    pushed as `e5523b4f5cd Restore LCP physics scene controls`.
41. The active strict re-open cursor is now `examples/mimic_pendulums/`;
    compare it against `520993d7301^:examples/mimic_pendulums` before coding
    past that slice.
42. `examples/mimic_pendulums/` strict re-open found concrete gaps: the live
    ODE-collision/PGS-solver and reset controls were reduced to launch-time
    flags/static text, the historical OSG grid was replaced with a ground box,
    the source renamed the three rigs after collecting diagnostics, and README
    plus marker coverage missed the full contract. Restore the live controls,
    source-owned grid, original rig names/base colors, README option inventory,
    and marker guards now.
43. `examples/mimic_pendulums/` now restores the live reset, ODE collision, and
    PGS solver controls; keeps original rig names with base-color legend;
    replaces the old OSG grid with a source-owned `LineSegmentShape` XY grid;
    reports reference/follower angles, position error, velocity error, and base
    drift; and documents shared capture flags in the README. Exact ImGui table
    and color-swatch layout remains a public panel API gap. Local validation
    includes focused builds, focused CTest, direct `--help`, software-GL
    screenshot and image-sequence analyzer checks, aggregate `build-examples`,
    Python C++ example-runner tests, mandatory `pixi run lint`, post-lint
    focused checks, and `git diff --check`.
44. The active strict re-open cursor is now `examples/mixed_chain/`; compare it
    against `520993d7301^:examples/mixed_chain` before coding past that slice.
45. `examples/mixed_chain/` strict re-open found concrete gaps after the prior
    parity checkpoint: the historical random startup pose became deterministic,
    640x480 defaults were not wired through public run options, console
    instructions are missing, there is no README, and marker coverage does not
    guard the full restored contract. Restore those items now while keeping the
    already-promoted pre-step impulse controls and keyboard actions.
46. `examples/mixed_chain/` now restores the random startup pose, 640x480 run
    default, console instructions, README/capture documentation, and marker
    coverage while keeping the promoted pre-step impulse controls, keyboard
    actions, panel buttons, and camera. Local validation includes focused
    builds, focused CTest, direct and pixi software-GL screenshot analyzer
    checks, image-sequence analyzer checks, aggregate `build-examples`, Python
    C++ example-runner tests, mandatory `pixi run lint`, post-lint focused
    checks, and `git diff --check`.
47. The active strict re-open cursor is now
    `examples/operational_space_control/`; compare it against
    `520993d7301^:examples/operational_space_control` before coding past that
    slice.
48. `examples/operational_space_control/` strict re-open found major scope
    drift: the promoted source loaded WAM, while the historical example loads
    KR5 plus KR5 ground. The current source restores KR5 robot/ground loading,
    the red `SphereShape(0.025)` target ball, controller math on the KR5 end
    effector, 640x480 defaults, camera home, console/panel instructions,
    README/capture docs, and marker coverage. 1/2/3 aliases were added to
    promoted axis-constrained drag so the historical constraint keys work
    without reintroducing OSG. The hidden shadow toggle and exact custom camera
    up vector remain public API gaps. Pre-lint validation includes focused
    builds, focused CTest, direct and pixi software-GL screenshot analyzer
    checks, image-sequence analyzer checks, aggregate `build-examples`, Python
    C++ example runner tests, and direct visual inspection of the KR5 capture.
    Mandatory `pixi run lint`, post-lint focused rebuild/CTest, post-lint
    direct screenshot analyzer check, and `git diff --check` also passed.
49. New maintainer steering after item 48: do not keep Rerun support merely as
    optional scaffolding. The `examples/rerun/` placeholder is already removed;
    this slice also removed stale Rerun-as-adjacent-tool planning language from
    the renderer selection and north-star migration notes. The remaining
    lowercase `rerun` references are CI retry workflow docs or intentional
    removal guards/history, not product support.
50. The active strict re-open cursor is now `examples/rigid_shapes/`. A prior
    parity checkpoint restored a substantial amount of behavior, but
    `11-example-parity-audit.md` still lacks the itemized historical-source
    inventory. Before coding past this slice, compare current source/README
    against `520993d7301^:examples/rigid_shapes`, then restore or classify the
    historical spawn/delete/contact controls, CLI options, camera/defaults,
    README, panel, and marker behavior.
51. `examples/rigid_shapes/` strict re-open found concrete gaps after the
    prior parity checkpoint: startup scene loading drifted from the historical
    `dart://sample/skel/shapes.skel` world to source-owned initial shapes, the
    random spawn behavior became deterministic, console instructions were not
    printed, the collision-detector printout was missing, and the historical
    README was absent. The current source restores those through public
    `dart::gui` and `dart::io`, preserving the promoted panel controls and
    example-owned `PointCloudShape` contact markers. Pre-lint validation
    includes focused builds, focused CTest, direct/pixi screenshot analyzer
    checks, direct CLI-options screenshot smoke, image-sequence analyzer check,
    aggregate `build-examples`, Python C++ example-runner tests, and direct
    visual inspection of the loaded SKEL scene.
    Mandatory `pixi run lint`, post-lint focused rebuild/CTest, post-lint
    direct screenshot analyzer check, and `git diff --check` also passed.
52. The active strict re-open cursor is now `examples/soft_bodies/`. A prior
    parity checkpoint restored source-owned playback controls, but
    `11-example-parity-audit.md` still lacks the itemized historical-source
    inventory. Before coding past this slice, compare current source/README
    against `520993d7301^:examples/soft_bodies`, then restore or classify the
    historical playback/step/reset controls, shifted-key shortcuts,
    camera/defaults, README, panel, and marker behavior.
53. `examples/soft_bodies/` strict re-open found concrete gaps after the prior
    parity checkpoint: the current source restores the historical
    `softBodies.skel` load, recorded playback history, bracket/brace controls,
    reset/latest controls, and promoted panel, but it does not set the
    historical 640x480 launch default, print the historical instruction block,
    or include the README. The current source restores those gaps through
    public `dart::gui` and `dart::io` while keeping the promoted playback panel
    and marker coverage. Pre-lint validation includes focused builds, focused
    CTest, direct and pixi software-GL screenshot analyzer checks,
    image-sequence analyzer check, aggregate `build-examples`, Python C++
    example-runner tests, and direct visual inspection of the rendered
    soft-body scene. Mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, post-lint direct screenshot basic analyzer check, and
    `git diff --check` also passed. The generic contrast analyzer is not a
    soft-bodies gate because the scene is low-shadow by design.

## Checkpoint Rule

For each reasonable checkpoint:

- Update this dev-task documentation before making scope or design changes.
- Run focused build/smoke validation for the example.
- Run `pixi run lint` before committing, even for docs-only checkpoints.
- Commit with a plain descriptive title.
- Push to the tracked remote branch without opening a pull request.
