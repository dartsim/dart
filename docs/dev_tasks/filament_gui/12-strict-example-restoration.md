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
   expressible through public `dart::gui`; the later local R24-15/R24-16/R24-18
   checkpoints restore camera-inspector text, key-release callbacks, and
   post-step plus pre/post-render hooks. The later local R24-17 lighting-state
   checkpoint restores headlight toggles through public `dart::gui`.
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
    `82e39c45558 Restore Atlas Simbicon controller example`; a later local
    follow-up restores depth-camera parity through public
    `RenderSettings::outputMode` after the headlight, native window-title, and
    shadow-toggle repairs.
    `examples/free_joint_cases/` was already restored and
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
    A later local checkpoint restores target activation/deactivation, support
    toggles, P/T diagnostics, and source-owned support-polygon/COM overlays;
    the following local checkpoint restores hold/release R whole-body
    posture/balance solver behavior through public DART whole-body IK APIs.
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
    placement, camera Eye/Center/Up readout, and key-release callbacks explicit
    as public `dart::gui` API follow-ups unless the missing renderer-neutral
    APIs are added. The later local R24-17 lighting-state checkpoint restores
    headlight toggles through public `dart::gui`.
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
    those now. Later local checkpoints restore
    hold/release `R` posture optimization with balance mode through public DART
    whole-body IK APIs and restore the source-owned analytical arm/leg IK
    methods plus Shift movement amplification through public uppercase keyboard
    shortcuts.
    A later checkpoint restores exact blue/red COM marker colors with
    source-owned DART line geometry. Historical camera roll/up-vector parity is
    restored by the later R24-14 public `OrbitCamera::up` checkpoint.
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
    contracts. Restore those now. The later R24-14 public `OrbitCamera::up`
    checkpoint restores the historical `(0, 1, 0)` camera up vector.
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
    coverage for those restored contracts. Restore those now. The later R24-14
    public `OrbitCamera::up` checkpoint restores the historical `(0, 1, 0)`
    camera up vector.
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
    README option inventory, and marker guards now. A later checkpoint restored
    source-owned render FPS, rendered/skipped frame counts, and rolling
    step-time summaries. Another local checkpoint restores the step-time plot
    through public `PanelBuilder::plotLines`. The next local checkpoint adds
    public `PanelContext::UiState` metrics and restores display/font debug
    diagnostics without backend hooks. Implementation and pre-lint validation
    are
    complete: focused `lcp_physics` and `UNIT_gui_FilamentSceneExtraction`
    build, focused CTest, direct `--list`, software-GL screenshot and `--out`
    image-sequence analyzer checks, pixi runner screenshot analyzer check,
    Python example-runner tests, aggregate `examples` build, and
    `git diff --check` passed locally. Mandatory `pixi run lint`, post-lint
    focused build/CTest, direct `--list`, software-GL screenshot analyzer
    check, and post-lint `git diff --check` also passed. This checkpoint was
    pushed as `e5523b4f5cd Restore LCP physics scene controls`. The
    panel-plotting follow-up validation also passed: focused `lcp_physics` and
    `UNIT_gui_FilamentSceneExtraction` build, focused CTest, mandatory
    `pixi run lint`, post-lint focused build/CTest, and post-lint
    LCP pixi headless screenshot analyzer smoke with 307200/307200 nonzero
    pixels. UI metrics follow-up validation passed: focused `lcp_physics` and
    `UNIT_gui_FilamentSceneExtraction` build, focused CTest, mandatory
    `pixi run lint`, post-lint focused build/CTest, and post-lint LCP pixi
    headless screenshot analyzer smoke with 307200/307200 nonzero pixels.
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
    drift; and documents shared capture flags in the README. Local validation
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
    without reintroducing OSG. The hidden shadow toggle is restored by the local
    R24-19 public `RenderSettings` checkpoint; the exact custom camera up vector
    is restored by the later R24-14 public `OrbitCamera::up` checkpoint.
    Pre-lint validation includes focused
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
54. Rerun support audit after the soft-bodies checkpoint: maintainer steering
    asked to remove Rerun maintenance surface unless a concrete use case or
    downstream user exists. Repository search found no remaining product
    CMake/package/API/runtime support and no path named Rerun. The remaining
    lowercase `rerun` references are CI retry workflow docs, generated
    command/skill guard patterns, marker tests that enforce the removed
    `examples/rerun/` placeholder, and historical dev-task notes. No additional
    product removal is needed now; future Rerun work should return only as a
    real sourced example or application feature with a concrete use case.
55. The active strict re-open cursor is now `examples/tinkertoy/`. A prior
    parity checkpoint restored substantial builder/keyboard/camera behavior,
    but the strict itemized inventory is still missing. Compare current
    source/README against `520993d7301^:examples/tinkertoy`, then restore or
    classify the historical construction controls, force controls, camera home,
    recording behavior, README, and marker coverage before moving past it.
56. `examples/tinkertoy/` strict re-open found concrete gaps after the prior
    checkpoints: the current source restores the initial assemblies, block
    shapes, add/delete controls, force application, target handle, gravity
    control, force coefficient control, Tab camera reset, and promoted
    selection movement, but it lacks the README, explicit 1280x720 defaults,
    historical panel layout/menu/help labels, historical console force/edit
    messages, and exact pick-point/normal behavior. The current source repairs
    those gaps by extending public `PanelContext` with selected hit
    point/normal, using that in Tinkertoy, restoring panel/README/default
    markers, and later restoring runtime Enter recording through public
    `ViewerLifecycleState` frame-output capture controls plus the exact About
    DART modal through public `PanelBuilder` modal helpers. The later local
    R24-17 lighting-state checkpoint restores the headlight toggle.
    Pre-lint validation includes focused builds, focused
    CTest, direct and pixi software-GL screenshot basic analyzer checks,
    image-sequence analyzer check, aggregate `build-examples`, and Python C++
    example-runner tests. Mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, post-lint direct screenshot basic analyzer check,
    `git diff --check`, and an API-boundary inventory diff check also passed.
57. The active strict re-open cursor is now `examples/vehicle/`. A prior
    parity checkpoint restored the historical command keys and camera default,
    but the strict itemized inventory is still missing. Compare current
    source/README against `520993d7301^:examples/vehicle`, then restore or
    classify historical command behavior, controller setup, camera/defaults,
    README, capture, and marker coverage before moving past it.
58. `examples/vehicle/` strict re-open found concrete gaps after the prior
    command-key checkpoint: current source had the world/controller, keyboard
    actions, camera, and panel controls, but still lacked the historical
    640x480 launch default, printed instruction block, README, and marker
    coverage for those restored surfaces. The current source repairs those via
    public `dart::gui::RunOptions`, source-owned console text, restored README
    run/capture/build/execute documentation, and marker guards. Pre-lint
    validation includes focused builds, focused CTest, direct and pixi
    software-GL screenshot basic analyzer checks, image-sequence analyzer
    coverage, aggregate `build-examples`, and Python C++ example-runner tests.
    Mandatory `pixi run lint`, post-lint focused rebuild/CTest, and post-lint
    direct screenshot basic analyzer check also passed.
59. The active strict re-open cursor is now `examples/wam_ikfast/`. A prior
    robot/IK checkpoint restored target-handle behavior, but the strict
    itemized inventory is still missing. Compare current source/README against
    `520993d7301^:examples/wam_ikfast`, then restore or classify historical WAM
    URDF loading, IK target behavior, keyboard/teleoperation, camera/defaults,
    README, capture, and marker coverage before moving past it.
60. `examples/wam_ikfast/` strict re-open found concrete gaps after the prior
    visual target checkpoint: the current source loaded the WAM and exposed a
    visible target, but it had dropped the historical local IKFast shared
    library target, `SharedLibraryIkFast` configuration, `ee`/`lh_target`
    setup, target activation workflow, `P` joint printing, `T` relaxed-posture
    reset, console instructions, 1280x960 launch default, camera home, README,
    and marker coverage. The current source repairs the feasible behavior
    through public `dart::gui` keyboard actions, IK handles, pre-step solving,
    run defaults, camera setup, restored README/capture docs, and marker guards.
    A later local follow-up restores the explicit `allowSimulation(false)`
    lifecycle with public `ApplicationOptions::simulateWorld`. Exact
    Alt/Ctrl/Shift parent-joint manipulation remains a documented public API
    gap. Rerun support remains intentionally removed: repository search still
    finds no
    product CMake/package/API/runtime surface or downstream use case, and
    remaining lowercase `rerun` references are CI retry docs, generated command
    guards, historical task notes, or tests that guard the removed placeholder.
    Pre-lint validation includes focused builds, focused CTest, direct and pixi
    software-GL screenshot basic analyzer checks, image-sequence analyzer
    coverage, aggregate `build-examples`, and Python C++ example-runner tests.
    Mandatory `pixi run lint`, post-lint focused rebuild/CTest, post-lint direct
    screenshot basic analyzer check, and `git diff --check` also passed.
61. The active strict re-open cursor is now `examples/fetch/`. Maintainer
    correction explicitly called out Fetch as not fully restored, so do not rely
    on earlier restored wording. Compare current source/README against
    `520993d7301^:examples/fetch`, then restore or classify historical target
    manipulation, panel/help behavior, camera/defaults, README, capture, and
    marker coverage before moving past it.
62. The Fetch re-open after WAM found source behavior still aligned with the
    historical source through public `dart::gui`; the concrete gap was README
    and marker coverage for promoted `--out` image-sequence capture. That local
    checkpoint is committed locally as `de1873ddab7` and passed focused
    build/CTest, mandatory lint, and direct/pixi Fetch `--out` smoke checks.
63. R24-1 is committed locally as `9d5dc251b43`. It adds the public
    `dart::gui::Gizmo` value API, registers render-only gizmos through
    `ApplicationOptions`, renders them privately as debug-line overlays, and
    attaches them to Atlas Puppet targets.
64. R24-2 is committed locally as `bb90788bbd2`. It wires X/Y/Z gizmo arrow
    handles to axis-drag math for `SimpleFrame` targets.
65. R24-3 is committed locally as `a5f776f6eb1`. It adds X/Y/Z rotation rings
    and wires them to target rotation.
66. R24-4 is committed locally as `2fb5c962e9e`. It adds XY/YZ/XZ plane
    handles and wires them to target translation.
67. R24-5 is committed locally as `dfb829b5307`. It adds hover/active
    highlighting for gizmo handles by feeding the hovered or actively dragged
    handle into renderer-neutral gizmo debug-line generation.
68. R24-7 is committed locally as `8778962b97a`. It replaces
    `examples/atlas_puppet/` source-owned IK target-handle geometry with public
    `dart::gui::Gizmo` affordances while preserving number-key target selection
    and keyboard nudges.
69. R24-8 applies the same no-bare-target-affordance rule to the remaining
    robot/IK examples:
    `examples/hubo_puppet`, `examples/g1_puppet`,
    `examples/operational_space_control`, and `examples/wam_ikfast` now use
    public `dart::gui::Gizmo` target affordances instead of source-owned
    target-handle or target-ball geometry. `Gizmo::isVisible` keeps inactive
    target gizmos out of rendering and picking while preserving number-key
    target selection and keyboard nudges. Local validation passed focused
    builds for the four migrated examples plus `UNIT_gui_FilamentSceneExtraction`,
    focused CTest, direct software-GL screenshot analyzer smokes for Hubo, G1,
    operational-space control, and WAM IKFast, aggregate `examples` build,
    mandatory `pixi run lint`, post-lint focused rebuild/CTest, post-lint
    direct screenshot smokes, stale-handle text scan, and `git diff --check`.
70. The current R24-9 implementation extends the same public-gizmo cleanup to
    `examples/imgui`: the panel-extension example now uses a public
    `dart::gui::Gizmo` for its target frame instead of a source-owned yellow
    line handle while preserving the panel, keydown, gravity, pre-step, camera,
    and README behavior from the strict audit. The private Filament startup
    gate now allows visible gizmo-only scenes instead of forcing examples to
    keep dummy renderable geometry. Validation passed focused build for
    `imgui` plus `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct
    software-GL screenshot analyzer smoke for `imgui`, aggregate `examples`
    build, mandatory `pixi run lint`, post-lint focused rebuild/CTest, and
    post-lint direct screenshot smoke.
71. The current R24-10 implementation extends the cleanup to
    `examples/tinkertoy`: the builder keeps its intentional source-owned axes
    and force-line geometry, but the user-movable force target now uses a
    public `dart::gui::Gizmo` instead of source-owned target-handle geometry.
    The panel and README describe gizmo dragging while preserving the existing
    selected-hit data, force application, add/delete controls, target
    reorientation, camera reset, and capture behavior. Validation passed
    focused build for `tinkertoy` plus `UNIT_gui_FilamentSceneExtraction`,
    focused CTest, direct software-GL screenshot analyzer smoke for
    `tinkertoy`, aggregate `examples` build, mandatory `pixi run lint`,
    post-lint focused rebuild/CTest, post-lint direct screenshot smoke,
    stale-handle text scan, and `git diff --check`.
72. The current R24-11 implementation re-opens `examples/drag_and_drop` now
    that public gizmos have axis, plane, and rotation handles. The example keeps
    its source-owned DART world, red child box, X/Y/Z marker boxes, run
    defaults, camera, panel, README, and selection text, but replaces the old
    source-owned line-handle geometry with a public `dart::gui::Gizmo` attached
    to the `interactive frame` `SimpleFrame`. Validation passed focused build
    for `drag_and_drop` plus `UNIT_gui_FilamentSceneExtraction`, focused CTest,
    direct 640x480 software-GL screenshot analyzer smoke for `drag_and_drop`,
    aggregate `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, post-lint direct screenshot smoke, stale-handle text scan,
    and `git diff --check`.
73. The current R24-12 implementation applies the same public-gizmo cleanup to
    `examples/fetch` without removing the historical Fetch-specific target cue:
    the transparent green bars remain source-owned scene geometry on the target
    frame, but mouse translation/rotation now uses a public
    `dart::gui::Gizmo` registered through `ApplicationOptions::gizmos`.
    Validation passed focused build for `fetch` plus
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct 1280x960
    software-GL screenshot analyzer smoke for `fetch`, aggregate `examples`
    build, mandatory `pixi run lint`, post-lint focused rebuild/CTest,
    post-lint direct screenshot smoke, stale-handle text scan, and
    `git diff --check`.
74. The current R24-13 implementation promotes a native window-title field on
    `dart::gui::RunOptions` and uses it from `examples/atlas_simbicon` to
    restore the historical `Atlas Simbicon` window title. This removes the
    window-title public API gap from Atlas Simbicon while keeping the
    then-remaining render-settings gaps explicit until later local checkpoints.
    Validation passed focused build for `atlas_simbicon` plus
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct 1280x960
    software-GL screenshot analyzer smoke for `atlas_simbicon`, aggregate
    `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, post-lint direct screenshot smoke, stale window-title/text
    scan, and `git diff --check`.
75. The current R24-14 implementation promotes `dart::gui::OrbitCamera::up` and
    routes orbit camera basis construction plus Filament `lookAt` calls through
    the configured up vector. This restores historical custom camera up-vector
    defaults for `hello_world`, `capsule_ground_contact`, `empty`, `imgui`,
    `hubo_puppet`, `hybrid_dynamics`, `joint_constraints`,
    `operational_space_control`, and `rigid_shapes` while keeping true render
    settings and example-specific solver/control gaps explicit. Validation
    passed focused build for `UNIT_gui_FilamentSceneExtraction`,
    `operational_space_control`, and `hybrid_dynamics`, focused CTest, direct
    software-GL screenshot analyzer smokes for `operational_space_control` and
    `hybrid_dynamics`, aggregate `examples` build, mandatory `pixi run lint`,
    post-lint focused rebuild/CTest, post-lint direct screenshot smokes, stale
    camera-up gap text scan, and `git diff --check`.
76. The current R24-15 implementation promotes
    `dart::gui::PanelContext::camera` as renderer-neutral camera inspection
    state with live eye, target, and up vectors. `examples/box_stacking` and
    `examples/imgui` use that public panel context to restore the historical
    Eye/Center/Up readouts without exposing backend viewer or camera objects.
    Validation passed focused build for `imgui`, `box_stacking`, and
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct software-GL
    screenshot analyzer smokes for `imgui` and `box_stacking`, aggregate
    `examples` build, mandatory `pixi run lint`, post-lint focused rebuild/CTest,
    post-lint direct screenshot smokes, stale camera-inspection gap text scan,
    and `git diff --check`.
77. The current R24-16 implementation promotes
    `dart::gui::KeyboardActionTrigger::Release`, teaches character shortcuts to
    distinguish shifted uppercase keys from lowercase keys, and restores the
    historical keydown/key-release callback messages in `box_stacking`, `empty`,
    and `imgui` without exposing backend event types. Validation passed focused
    build for `empty`, `imgui`, `box_stacking`, and
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct software-GL
    screenshot analyzer smokes for `empty`, `imgui`, and `box_stacking`,
    aggregate `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, post-lint direct screenshot smokes, stale key-release gap text
    scan, and `git diff --check`.
78. The current R24-17 implementation promotes
    `dart::gui::PanelContext::lighting` with renderer-neutral headlight state
    owned by the application frame, applies the state to Filament scene-light
    intensities before rendering, and restores historical `Headlights On/Off`
    panel checkboxes in `atlas_simbicon`, `box_stacking`, `imgui`, and
    `tinkertoy` without exposing backend viewer hooks. Validation passed
    focused build for those four examples plus
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct software-GL
    screenshot analyzer smokes for all four examples, aggregate `examples`
    build, mandatory `pixi run lint`, post-lint focused rebuild/CTest, and
    post-lint direct screenshot smokes.
79. The current R24-18 implementation promotes
    `dart::gui::ApplicationOptions::postStep`, `preRender`, and `postRender`
    alongside the existing `preStep` callback. The private Filament runner
    invokes post-step callbacks after each simulated world step and render
    callbacks around each successful frame-render attempt, restoring the
    historical lifecycle-hook demonstrations in `empty` and `imgui` without
    exposing backend world-node or viewer types. Validation passed focused
    build for `empty`, `imgui`, and `UNIT_gui_FilamentSceneExtraction`,
    focused CTest, direct software-GL screenshot analyzer smokes for `empty`
    and `imgui`, aggregate `examples` build, mandatory `pixi run lint`,
    post-lint focused rebuild/CTest, and post-lint direct screenshot smokes.
80. The current R24-19 implementation promotes
    `dart::gui::RenderSettings::shadowsEnabled`, exposes that renderer-neutral
    state to panels through `PanelContext::rendering`, exposes it to keyboard
    callbacks through `KeyboardActionContext::renderSettings`, and applies it
    to the private Filament view before rendering each frame. This restores the
    historical Atlas Simbicon `Shadow On/Off` panel checkbox and operational
    space control `s`/`S` shadow toggle without exposing backend shadow
    techniques. Validation passed focused build for `atlas_simbicon`,
    `operational_space_control`, and `UNIT_gui_FilamentSceneExtraction`,
    focused CTest, direct software-GL screenshot analyzer smokes for
    `atlas_simbicon` and `operational_space_control`, aggregate `examples`
    build, mandatory `pixi run lint`, post-lint focused rebuild/CTest, and
    post-lint direct screenshot smokes.
81. The current R24-20 implementation promotes
    `dart::gui::PanelBuilder::colorEdit` as a renderer-neutral RGBA color
    editing primitive and implements it in the private Filament ImGui panel
    bridge. `examples/heightmap` now restores the historical terrain color
    editor, and `examples/point_cloud` restores point-cloud and voxel-grid
    color editors without exposing backend widgets. Validation passed focused
    build for `heightmap`, `point_cloud`, and
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, direct software-GL
    screenshot analyzer smokes for `heightmap` and `point_cloud`, aggregate
    `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, and post-lint direct screenshot smokes.
82. The current R24-21 implementation promotes renderer-neutral
    `dart::gui::PanelBuilder` table and color-swatch primitives. The private
    Filament ImGui panel bridge maps them to `BeginTable`/`ColorButton`, while
    `examples/mimic_pendulums` uses them to restore the historical rig legend
    color swatches and mimic diagnostics table with a flat-text fallback for
    renderers without table support. Validation passed focused build for
    `mimic_pendulums` and `UNIT_gui_FilamentSceneExtraction`, focused CTest,
    direct software-GL screenshot analyzer smoke for `mimic_pendulums`,
    aggregate `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, and post-lint direct screenshot smoke.
83. The current R24-22 implementation maps
    `dart::gui::KeyboardShortcut::characterKey('?')` to shifted slash in the
    private Filament input bridge, while keeping unshifted slash distinct. This
    restores the historical Simulation Event Handler `?` help alias through
    public `dart::gui::KeyboardAction` instead of backend event types.
    Validation passed focused build for `simulation_event_handler` and
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, pixi software-GL
    screenshot analyzer smoke for `simulation_event_handler`, aggregate
    `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, and post-lint pixi screenshot smoke.
84. The current R24-23 implementation restores the historical
    `hardcoded_design` wireframe appearance by attaching source-owned
    `dart::dynamics::LineSegmentShape` edge overlays to each articulated link.
    This closes the OSG polygon-mode gap without exposing backend renderer
    state or adding a renderer-wide style setting that Filament does not
    provide. Validation passed focused build for `hardcoded_design` and
    `UNIT_gui_FilamentSceneExtraction`, focused CTest, pixi software-GL
    screenshot analyzer smoke for `hardcoded_design`, aggregate `examples`
    build, mandatory `pixi run lint`, post-lint focused rebuild/CTest, and
    post-lint pixi screenshot smoke.
85. The current R24-24 implementation restores the historical Heightmap and
    Point Cloud fine-grained grid controls without reviving OSG `GridVisual` or
    backend debug-overlay hooks. Both examples now consume a shared
    source-owned grid helper that rebuilds DART `LineSegmentShape` frames for
    grid plane, offset, line count, step size, major/minor/axis widths, and
    line colors. Validation passed focused build for `heightmap`,
    `point_cloud`, and `UNIT_gui_FilamentSceneExtraction`, focused CTest, pixi
    software-GL screenshot analyzer smokes for both examples, aggregate
    `examples` build, mandatory `pixi run lint`, post-lint focused
    rebuild/CTest, and post-lint pixi screenshot smokes.

## Checkpoint Rule

For each reasonable checkpoint:

- Update this dev-task documentation before making scope or design changes.
- Run focused build/smoke validation for the example.
- Run `pixi run lint` before committing, even for docs-only checkpoints.
- Commit with a plain descriptive title.
- Push to the tracked remote branch without opening a pull request.
