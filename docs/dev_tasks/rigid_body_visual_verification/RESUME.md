# Resume: Rigid-Body Visual Verification

## Current Handoff Snapshot

Latest continuation resumed from the pushed handoff commit `30d65d4229c` and
continued the interrupted row-29 capture-metrics slice for
`rigid_joint_motor_limits`. The branch had no associated GitHub PR, and the
only dirty implementation/test files were
`python/examples/demos/scenes/rigid_joint_motor_limits.py` and
`python/tests/integration/test_demos_cycle.py`.

Expected branch/worktree state after this local checkpoint:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Pushed branch head remains `30d65d4229c` unless a maintainer/user explicitly
  approves another push.
- The completed local checkpoint should include row-29 motor-limit capture
  metrics, tests, docs, and refreshed handoff state.
- Future pushes, PR creation, comments, review replies, CI retriggers, merges,
  or other GitHub mutations still require explicit maintainer/user approval.

Recent checkpoints:

- `7ef7a96dda0` (`Expose rigid collision cast capture metrics`)
- `b54c4f5dba3` (`Expose rigid executor equivalence capture metrics`)
- `a47d52b7aea` (`Hand off rigid friction threshold capture metrics`)
- `d16c6200850` (`Complete rigid friction threshold capture docs`)
- `c2beab542e9` (`Expose rigid spin roll capture metrics`)
- `dfef5306aac` (`Expose rigid stack stability capture metrics`)
- `b68f216ebe9` (`Expose rigid contact manipulation capture metrics`)
- `9947ab1285a` (`Expose rigid kinematic driver capture metrics`)
- `30d65d4229c` (`Document rigid visual verification handoff`)
- the local checkpoint containing this file (`Expose rigid joint motor limit capture metrics`)

Previous local checkpoint: `rigid_kinematic_driver` row capture metrics. The
checkpoint touches:

- `python/examples/demos/scenes/rigid_kinematic_driver.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test: row 23 now exposes
`SceneSetup.info["capture_metrics"]` for the IPC prescribed-motion kinematic
driver row and its sequential-impulse caveat lane. The payload exports row
identity, solver scope, selected executor, drive-speed/grip-friction controls,
lane order, lane solver/friction modes, world time, per-lane driver travel, box
travel, slip, speed ratio, support gap, contact count, status, step timing, and
compact history ranges. The focused test now asserts the capture hook is
present and that exported lane values and histories mirror the live controller
state.

Validation collected for the kinematic-driver slice:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc -q`
  reported `1 passed`.
- `pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_driver_metrics_1781232929`
  wrote a nonblank docked capture with 71 PNG frames and 72 scene-metrics
  events. The manifest recorded top-level ranges for IPC grip, IPC slip, and
  sequential-impulse caveat driver travel, box travel, slip, speed ratio,
  support gap, contact count, step timing, time step, and world time.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `11 passed`.
- `pixi run lint` passed.
- Bounded `DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 pixi run build`
  passed and reported `ninja: no work to do`.
- `git diff --check` passed.
- Contact-manipulation completion immediately before this slice:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal -q`
  reported `1 passed`; the real docked capture under
  `/tmp/dart_capture_contact_manipulation_metrics_1781232293` wrote 72
  scene-metrics events. The broader workflow/doc drift guard, `pixi run lint`,
  bounded build, and `git diff --check` were intentionally not run for that
  slice after the user explicitly requested handoff without further
  verification.
- Stack-stability completion immediately before contact manipulation:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest ... -q`
  reported `10 passed`; the real docked capture under
  `/tmp/dart_capture_stack_stability_metrics_1781231606` wrote
  24 scene-metrics events; `pixi run lint`, bounded build, and
  `git diff --check` passed.
- Spin/roll completion immediately before this slice:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest ... -q`
  reported `10 passed`; the real docked capture under
  `/tmp/dart_capture_spin_roll_metrics_1781230743` wrote 96 scene-metrics
  events; `pixi run lint` passed before the final spin/roll handoff edit.
- Friction-threshold completion immediately before this slice:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest ... -q`
  reported `7 passed`; the real docked capture under
  `/tmp/dart_capture_friction_threshold_metrics_1781229799` wrote 24
  scene-metrics events; `pixi run lint`, bounded build, and `git diff --check`
  passed.

Current local slice: `rigid_joint_motor_limits` row capture metrics. The local
checkpoint touches:

- `python/examples/demos/scenes/rigid_joint_motor_limits.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `CAPTURE_METRICS_INFO_KEY` wiring in
  `python/examples/demos/scenes/rigid_joint_motor_limits.py`.
- Scene-owned payload fields for row identity, solver
  `world_multibody_joint_actuators`, constraint
  `velocity_motor_position_limit_effort_cap`, time step, world time,
  user-facing controls, joint names, motor speed/position clamp metrics,
  position-limit angle/error/speed, capped-vs-reference force travel and
  acceleration gaps, step timing, the latest metrics snapshot, and compact
  history ranges.
- `_step_ms_history` replay capture/restore coverage and a panel line showing
  force travel gap, step timing, and world time.
- Focused assertions in
  `python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort`
  that the capture hook mirrors the controller state and exposes history maxima.

Validation collected for the motor-limit slice so far:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort -q`
  reported `1 passed` after fixing the assertions to compare latest top-level
  fields against latest metrics and history fields against max histories.
- `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_metrics_1781234533`
  wrote a nonblank docked capture with 71 PNG frames, 72 scene-metrics events,
  screenshot
  `/tmp/dart_capture_joint_motor_limits_metrics_1781234533/rigid_joint_motor_limits.png`,
  and manifest ranges for motor speed/position/error, position-limit angle and
  error, capped/open force acceleration, acceleration gap, force-position gap,
  step timing, time step, and world time.
- Latest captured payload details: row `rigid_joint_motor_limits`, solver
  `world_multibody_joint_actuators`, constraint
  `velocity_motor_position_limit_effort_cap`, command speed `0.55`, velocity
  limit `0.3`, position limit `0.35`, requested force `16.0`, effort limit
  `4.0`, motor speed `0.3`, motor speed error `0`, latest position-limit angle
  about `0.2633`, latest force-position gap about `0.3942`, latest
  limited/open force acceleration about `2.0/8.0`, acceleration gap about `6.0`,
  latest step time about `0.0720 ms`, and history sample count `73`.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `11 passed`.
- `pixi run lint` passed.
- Bounded `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run build`
  passed and reported `ninja: no work to do`.
- `git diff --check` passed.

Immediate next step for a fresh session:

1. Verify `git status -sb` and `git log --oneline --decorate -5`.
2. Continue the capture-metrics hardening pass. The likely next missing row is
   `rigid_joint_passive_parameters`.

## Last Session Summary

Created the first paired rigid solver comparison scene for Python `py-demos`.
`rigid_solver_compare` runs sequential impulse and rigid IPC in matched DART 7
Worlds, renders them side by side, exposes an executor selector, and plots
speed, wall clearance, position divergence, and step profile timing.

Focused Python demo tests pass, and `py-demo-capture` now applies the same
Linux software-Mesa defaults as GUI tests. The verified visual smoke command is:

```bash
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 \
    --width 960 --height 540 --show-ui
```

The second P0 slice, `rigid_friction_threshold`, is implemented and visually
captured. It shows three rigid IPC inclined-ramp lanes below, near, and above
`mu = tan(angle)`.

The executor-equivalence slice, `rigid_executor_equivalence`, is implemented and
visually captured. It runs matched DART 7 Worlds side by side with one shared
physics solver while the left world uses the sequential executor and the right
world uses a parallel executor. Its panel plots pose divergence, velocity
divergence, contact-count delta, and per-executor step timing so users can
distinguish executor performance from physics changes.

The contact-solver policy slice, `rigid_contact_solver_compare`, is implemented
and visually captured. It keeps the rigid-body solver family fixed to
sequential impulse, then compares the default sequential-impulse contact policy
against the opt-in boxed-LCP contact method on the same tilted multi-contact
plank. Its panel plots contact count, penetration depth, analytic corner
clearance, speed, angular speed, kinetic energy, pose divergence, and per-step
timing. The shared replay timeline now also shows this pose-divergence signal
and marks frames with coupled contacts or positive depth while scrubbing saved
states.

The restitution material-response slice, `rigid_restitution_ladder`, is
implemented and visually captured. It drops three matched rigid spheres with
low, medium, and high restitution, exposes solver/executor, launch height, and
restitution scale, and plots height, vertical velocity, contact count, rebound
height, mechanical-energy trend, and step timing without claiming exact energy
conservation.

The pair-material ownership slice, `rigid_material_mixing`, is implemented
immediately after `rigid_restitution_ladder`. It pins the row to sequential
impulse and uses swapped body/surface lanes: two bounce lanes for
`max(restitution)` and two flat sliding lanes for
`sqrt(friction_a * friction_b)`. The row computes expected pair coefficients
from public `RigidBody.restitution` and `RigidBody.friction`, and it avoids IPC
restitution claims or duplicating the incline stick/slip threshold row.

The third P0 slice, `rigid_stack_stability`, is implemented and visually
captured. It is a compact two-block top-heavy stack with sequential impulse and
rigid IPC side by side. A local probe showed taller IPC stacks are too slow for
the default GUI, so this scene uses a deliberately small stack to expose
mass-ratio jitter/drift without turning the live demo into a benchmark-only
surface.

The contact-observability slice, `rigid_contact_inspector`, is implemented and
visually captured. It uses static targets plus kinematic probes to keep stable
contacts available for direct `World.collide()` inspection: selected/total
contact count, representative point, normal, depth, local points, shape
indices, and depth histories. The inspector now covers representative public
shape families in one row: sphere/box, box/ground, capsule/sphere,
cylinder/sphere, mesh/sphere, and compound/sphere, including compound child
shape-index diagnostics.

The collision-query-options slice, `rigid_collision_query_options`, is
implemented and visually captured. It keeps four stable sphere overlap lanes
visible for direct `World.collide(options)` inspection: rigid/rigid,
rigid/link, same-multibody link/link, and cross-multibody link/link. Its panel
toggles the public include flags, selects a persistent ignored pair, reports
baseline, option-filtered, pair-ignored, and active contacts, marks filtered
lanes, and records replay snapshots for the query controls.

The current collision ignored-pair follow-up extends that same row instead of
adding a new verifier row. It separates body-kind option filtering from
persistent pair ignores, records the ignored-pair selector in replay snapshots,
exposes the same query counts and lane statuses through
`SceneSetup.info["capture_metrics"]`, and the docked capture wrote a nonblank
960x540 screenshot, 23 PNG frames, one dropped UI warmup frame, and docked
workspace evidence. A later 8-frame capture wrote eight scene-metrics events
for the same row and mirrored the rigid/link/cross-link lane set into the
manifest.

The collision-casts slice, `rigid_collision_casts`, is implemented and visually
captured. It uses public `CollisionGroup.raycast_result()` and
`CollisionGroup.sphere_cast_result()` queries to show nearest/all ray hits,
swept-sphere time of impact, sweep margin, first point/normal, and replayed
ray-offset/all-hit/sweep-radius controls without claiming World contact-solver
or CCD time-step behavior.

The contact-manipulation slice, `rigid_contact_manipulation`, is implemented
and visually captured. It runs matched sequential impulse and rigid IPC
table-push tasks side by side, with controls for executor, pusher launch speed,
table friction, and pusher mass. Its panel plots target travel, pusher-target
gap, contact/proximity evidence, target speed, lateral drift, goal error,
solver divergence, and per-step timing. This is a bounded task-like pusher row,
not a full articulated arm or gripper controller.

The kinematic-driver slice, `rigid_kinematic_driver`, is implemented and
visually captured. It shows IPC prescribed tangential motion carrying a box on a
kinematic support through friction, a zero-friction IPC slip baseline, and the
current sequential-impulse static-like caveat.

The normal prescribed-motion follow-up, `rigid_kinematic_normal_push`, is
implemented as a caveat verifier after `rigid_kinematic_driver`. It drives an
explicit normal kinematic paddle into a target and records IPC normal/heavy
penetration-caveat lanes next to a sequential-impulse push lane, with push
speed, target mass, driver travel, target travel, analytic gap, penetration
depth, contact count, speed ratio, step timing, capture metrics, and replayed
controls. Do not claim robust IPC normal kinematic manipulation from this row.

The constraints slice is implemented and visually captured. `rigid_fixed_joint`
now has perturb/reset controls and plots relative offset, relative orientation,
payload speed, and angular-speed histories. `rigid_limited_joints` now verifies
the locked directions for revolute and prismatic rigid-body joints while showing
the free hinge yaw and slider z-axis travel. `rigid_joint_motor_limits` now
covers the stable World multibody joint-actuator path for rigid links: velocity
motor clamping, a gravity-driven position stop, and capped versus reference
force response. It intentionally does not claim rigid-body joint motor support,
because a local probe showed rigid-body joint actuation is not stable enough for
a bounded public visual verifier yet.

The passive joint-parameter slice is implemented and visually captured as
`rigid_joint_passive_parameters`, immediately after the joint motor/limit row in
the curated workflow. It keeps contacts and gravity out of the scene while
showing World multibody spring/rest, damping, Coulomb friction, and armature
behavior in prismatic lanes. Its panel exposes spring stiffness, rest position,
damping, Coulomb friction, slip force, armature, acceleration-versus-expected
diagnostics, energy histories, and step-profile timing. The row is a passive
parameter verifier, not a motor, limit, or contact-load scene.

The current curation slice records the rigid visual-verification workflow in
`python/examples/demos/README.md`: baseline World rigid dynamics, restitution
material response, contact inspection, solver comparison, executor equivalence,
contact-solver policy, friction threshold, stack stability, contact
manipulation, kinematic drivers, joint constraints, joint motor/limit behavior,
and the existing IPC no-tunneling capability scene. It also records the
no-tunneling scope decision: do not add a separate thin-wall catalog row this
slice unless a future high-speed preset asks a distinct user question.

The durable evidence packet for this slice now lives in
`docs/plans/103-examples-strategy/rigid-body-visual-verification.md`. That
sidecar is the source of truth for scene order, capture commands, test evidence,
visual-smoke evidence, known limitations, and the no-tunneling decision.

A review-readiness pass then tightened the current branch: the controller-heavy
rigid rows now include user-facing controls in replay snapshots, and
`rigid_solver_compare` has a focused wall-response invariant test.

A front-door alignment pass made `rigid_body` the default Python `py-demos`
launch scene and moved the curated rigid visual-verification block to the front
of the global catalog and `World Rigid Body` category. The replay timeline
remains explicitly selectable as `replay_scrubber`.

The Demos navigator follow-up now prefixes the first 36 interactive
**World Rigid Body** viewer titles with workflow position and role, such as
`01/36 Baseline: World Rigid Body` and
`15/36 Solver family: Rigid Solver Compare`, while preserving stable
`py-demos --list` scene ids and titles for scripts. This turns the existing
curated order into an in-app breadcrumb instead of relying only on the README
and PLAN-103 sidecar.

The external-loads slice is implemented and visually captured as
`rigid_external_loads`, immediately after `rigid_body` in the curated workflow.
It keeps gravity and contacts out of the scene so users can inspect persistent
`apply_force()`/`apply_torque()` accumulators, mass-scaled acceleration,
inertia-scaled angular response, one-step pulse clearing, and the static-body
caveat. The row intentionally does not claim point-force, impulse, or contact
load semantics.

The time-step-sensitivity slice is implemented and visually captured as
`rigid_timestep_sensitivity`, immediately after `rigid_link_point_loads` in the
curated workflow. It keeps the displayed simulation-time advance matched across
fine, medium, and coarse lanes while changing each World's integration time
step and substep count. Its panel exposes solver, executor, base time step,
gravity scale, free-fall error, contact timing, clearance, coarse/fine error
ratio, and step-profile timing. The row is a parameter/convergence diagnostic,
not a solver correctness proof or exact contact-threshold claim.

The step-diagnostics slice is implemented as `rigid_step_diagnostics`,
immediately after `rigid_timestep_sensitivity` in the curated workflow. It runs
single-body, contact-pair, and small-stack Worlds under one selected
solver/executor so users can inspect `World.last_step_profile`,
`World.memory_diagnostics`, ECS entity/component counts, contact counts, and
frame-scratch usage without turning the row into another solver-family
comparison. It now also surfaces each top profile stage's domain,
backend-neutral acceleration mask, accelerated-backend flag, and accelerated
stage count so users can distinguish executor parallelism from actual
accelerator use without implying rigid CUDA support. If profiling is compiled
out, the panel still reports memory and contact diagnostics and marks profile
timing unavailable.

The contact-scale budget slice is implemented as
`rigid_contact_scale_budget`, immediately after `rigid_step_diagnostics` in the
curated workflow. It runs one-, four-, and nine-box contact workloads under one
selected solver, executor, frame budget, and friction value. Its panel exposes
contact-point count, bodies, contacts per body, wall time, per-contact cost, top
profile stage, frame-scratch peak usage, ECS counters, worker count,
dense/single wall-time ratio, and budget status. The row is a bounded live-GUI
performance diagnostic, not a benchmark suite or heavy IPC stress packet.

The free-flight slice is implemented and visually captured as
`rigid_free_flight`, immediately after `rigid_body` in the curated workflow. It
keeps contacts out of the scene while showing zero-gravity linear drift, a
gravity arc against analytic trajectory and momentum references, and
low/high-inertia spin lanes. Its panel exposes executor, launch speed, launch
angle, gravity scale, spin speed, inertia ratio, position error, momentum
residual, energy drift, spin momentum/energy ratios, contact count, and
step-profile timing. The row is a no-contact initial-state diagnostic, not a
load, restitution, contact, or solver-family row.

The loop-closure slice is implemented and visually captured as
`rigid_loop_closure`, after the rigid joint motor/limit row in the curated
workflow. It compares the same four-link endpoint POINT closure in residual-only
and solved modes under the variational rigid multibody path. The residual-only
lane remains a live diagnostic and visibly drifts under gravity, while the
solved lane keeps the endpoint target locked and plots residual, tip error, tip
height, joint speed, residual ratio, and step-profile timing. The row is an
endpoint-closure verifier, not a distance-closure or rigid-closure family sweep.

## Current Branch

`feature/rigid-body-gui-visual-verification` - local work in progress. The
branch has been pushed to origin through `b41b9279b60`
(`Expose collision query metrics in captures`). The earlier merge commit
`cd7600f8cda` merged latest `origin/main` into the feature branch before push.
The only manual merge conflict was `python/tests/integration/test_demos_cycle.py`;
the resolved file keeps both the local rigid visual-verification tests and the
new AVBD demo tests from `origin/main`. Post-merge validation passed: the
conflict-resolution `pixi run test-py` command ran the full Python suite and
reported `942 passed, 9 skipped`, and `pixi run lint` passed before the merge
commit was created and pushed.

The current distance-spring follow-up adds `rigid_distance_spring` as row 27/36
after the AVBD breakage lifecycle and before the one-DOF joint row. It shows
unsprung, soft, stiff, and off-center anchor lanes for the public
`World.add_rigid_body_distance_spring()` path, keeps IPC/multibody rejection
scope visible in the panel/docs, exports replay-state controls and
`capture_metrics`, and updates the registry, runner guide, README, PLAN-103
sidecar, changelog, dev-task notes, and PR draft. The focused behavior/replay/
category/order/viewer-title/guidance/docs/README/capture-command/panel guard
reported `17 passed` before and after lint. The real docked capture
`pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_1781220718`
wrote a nonblank 960x540 screenshot with docked-workspace detection, 71 PNG
frames, and 72 scene-metrics events for lanes `free/soft/stiff/offset`.

The previous loop-closure follow-up added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, changelog entry, and dev-task evidence. The focused
category/order/behavior/replay/panel command reported `5 passed`, and the
`rigid_loop_closure` visual capture wrote a nonblank 960x540 screenshot plus 71
nonblank PNG frames with final contacts at 0. `pixi run test-py` reported
`600 passed, 9 skipped`, `pixi run lint` passed, bounded `pixi run build`
passed with `DART safe jobs: 5`, and `git diff --check` was clean after the
loop-closure scene, registry, test, quickstart, workflow, changelog, and
dev-task updates.

The current passive joint-parameter follow-up has added the scene, registry
row, panel coverage, behavior coverage, replay-control restore checks,
quickstart docs, workflow docs, changelog entry, and dev-task evidence. The full
Python sweep through `pixi run test-py` reported `601 passed, 9 skipped`, and
the `rigid_joint_passive_parameters` visual capture wrote a nonblank 960x540
screenshot plus 119 nonblank PNG frames with final contacts at 0.

The current step-diagnostics follow-up has added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, changelog entry, and dev-task evidence. The focused
category/order/profile-memory/replay/panel command reported `5 passed`; the
visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG frames;
`pixi run test-py` reported `603 passed, 9 skipped`; `pixi run lint` passed;
and bounded `pixi run build` passed with no work remaining.

The current material-mixing follow-up has added the scene, registry row, panel
coverage, pair-rule behavior coverage, replay-control restore checks,
quickstart docs, workflow docs, changelog entry, and dev-task evidence. The
focused category/order/pair-rule/replay/panel command reported `5 passed`, and
the visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG
frames with final contacts at 0. `pixi run test-py` reported `604 passed, 9
skipped`, `pixi run lint` passed, and bounded `pixi run build` passed with no
work remaining.

The current curation-audit follow-up fixed the PLAN-103 sidecar's duplicate
workflow order number and added
`test_rigid_visual_verification_sidecar_matches_registry_order`. The test keeps
the numbered sidecar workflow unique, consecutive, backed by registered scene
ids, and aligned with the front-of-catalog World rigid-body verifier block while
keeping `rigid_ipc_tunnel` in the separate Rigid IPC capability shelf. The
sidecar validation snapshot is now a compact current-evidence summary instead
of a chronological list of stale per-slice pass counts.

The current README-sidecar drift guard adds
`test_rigid_visual_verification_readme_matches_sidecar_order`, keeping the
Python demo README quick workflow table in exact scene-id order with the
PLAN-103 sidecar. The capture-command companion
`test_rigid_visual_verification_capture_commands_match_workflow` keeps the
sidecar table commands, sidecar quick-refresh commands, and Python demo README
capture commands in the same ordered rigid workflow with matching frame budgets,
960x540 size, and `--show-ui`. The capture helper default now matches the
`py-demos` `rigid_body` front door, pinned by
`test_visual_capture_default_scene_matches_py_demos_front_door`. The focused
default/order/sidecar/README/capture/front-door guard command reported
`5 passed`.

The current backend-status follow-up keeps the next work inside the existing
`rigid_step_diagnostics` row instead of adding another catalog row. The panel
now reports top-stage domain, acceleration mask, backend-active status, and
accelerated-stage count from public `WorldStepStageProfile` metadata. The
focused step-diagnostics/panel command reported `2 passed`.

The current differentiable contact-gradient follow-up keeps the rigid workflow
from duplicating `diff_drone_liftoff` as a second numbered row. Instead, the
README and PLAN-103 sidecar now route rigid users to the Differentiable shelf
when forward contact looks correct but gradient-based optimization stalls. The
shared `diff_drone_liftoff` panel now exposes aware thrust, aware loss, aware
thrust-gradient, analytic loss, and height histories plus the public
`ANALYTIC`/`COMPLEMENTARITY_AWARE` mode labels. A refreshed full
`pixi run test-py` after the curation, plane-shape, and differentiable routing
follow-ups reported `605 passed, 9 skipped`.

The current contact-inspector plane-shape follow-up adds a true public
`CollisionShape.plane` plus sphere lane to `rigid_contact_inspector` and extends
`test_rigid_contact_inspector_reports_contact_manifolds` to verify the plane
shape while continuing to check finite normals, depths, local points, and
compound shape indices. The focused contact/order/sidecar/panel command
reported `4 passed`, and the visual capture wrote a nonblank 960x540 screenshot
plus 23 nonblank PNG frames.

The current screw-joint pitch follow-up adds `rigid_screw_joint_pitch` after
the passive-joint row and before loop closure in the curated World rigid-body
workflow. It uses zero, fine, coarse, and reverse World multibody screw-joint
lanes under gravity to expose pitch as axial translation per radian, including
rotation sign, axial travel, travel-per-radian, effective mass, and
expected-versus-actual acceleration diagnostics. The focused
behavior/replay/category/order/sidecar/README/capture-command/panel command
reported `8 passed`, and the visual capture wrote a nonblank 960x540
screenshot plus 95 nonblank PNG frames with final contacts at 0. The broader
`pixi run test-py` sweep reported `609 passed, 9 skipped`.

The current loop-closure family follow-up keeps the existing
`rigid_loop_closure` workflow row but expands it from a POINT-only endpoint
closure into POINT, DISTANCE, and RIGID public closure families crossed with
residual-only and solved policies. The panel now reports the family-specific
signal users need to choose the narrowest closure: tip error for POINT,
tether-length error for DISTANCE, orientation residual for RIGID, and residual
ratios for all three families. The focused loop-closure/replay/panel command
with category/order/sidecar/README/capture-command drift guards reported
`8 passed`, and the refreshed visual capture wrote a nonblank 960x540
`rigid_loop_closure` screenshot plus 71 nonblank PNG frames with final contacts
at 0. The broader `pixi run test-py` sweep still reported
`609 passed, 9 skipped`.

The current link center-of-mass follow-up adds `rigid_link_center_of_mass`
between `rigid_multibody_dynamics_terms` and `rigid_link_jacobian`. It keeps
visual link geometry centered while moving public `Link.center_of_mass` markers
through centered, mirrored, and high-inertia lanes so users can see zero
gravity torque, opposite acceleration signs, reflected mass, and inertia-scaled
acceleration in one contact-free World. The focused COM/replay/order/docs/panel
guard reported `17 passed`, and the docked visual capture wrote a nonblank
960x540 screenshot plus 71 PNG frames with final contacts at 0.

The current body-mode follow-up adds `rigid_body_modes` directly after the
`rigid_body` front door. It compares dynamic, static, and kinematic rigid-body
mode semantics in one contact-free World with shared solver/executor controls,
gravity and force controls, prescribed kinematic speed, static-drift history,
and kinematic path-error history. The focused
behavior/replay/category/order/sidecar/README/capture-command/panel command
reported `8 passed`; visual capture wrote a nonblank 960x540 screenshot plus
71 PNG frames with final contacts at 0; a standard-library PNG pixel probe
reported RGB variance `[2490.449, 2653.248, 2853.391]` and 3208 unique colors;
`pixi run test-py` reported `610 passed, 9 skipped`; `pixi run lint` passed;
the focused post-lint guard again reported `8 passed`; and `git diff --check`
was clean.

The previous free-flight follow-up added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, changelog entry, and dev-task evidence. A direct scene probe
verified contact-free Worlds, near-zero zero-gravity drift error, bounded
gravity-arc error and momentum residual, and inertia-scaled spin momentum and
energy ratios. The focused behavior/order/replay/panel command reported
`5 passed`, `py-demos --list` shows the row directly after `rigid_body`, and
the `rigid_free_flight` visual capture wrote nonblank 960x540 PNG frames.
`pixi run test-py` reported `599 passed, 9 skipped`, `pixi run lint` passed,
bounded `pixi run build` passed with `DART safe jobs: 5`, and
`git diff --check` was clean after the final free-flight evidence update.
The previous time-step-sensitivity follow-up added the scene, registry row,
panel coverage, behavior coverage, replay-control restore checks, quickstart
docs, workflow docs, changelog entry, and dev-task evidence. A direct scene
probe verified matched displayed simulation time, ordered pre-contact
free-fall error across fine/medium/coarse time steps, and recorded contact
timing/clearance. The focused behavior/order/replay/panel command reported
`5 passed`, `py-demos --list` shows the row directly after
`rigid_link_point_loads`, and the `rigid_timestep_sensitivity` visual capture
wrote nonblank 960x540 PNG frames. `pixi run test-py` reported
`598 passed, 9 skipped`, `pixi run lint` passed, bounded `pixi run build`
passed with `DART safe jobs: 5`, and `git diff --check` was clean after the
final time-step-sensitivity evidence update.
The previous external-loads follow-up added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, and dev-task evidence. A direct scene probe verified
mass-scaled linear acceleration, inertia-scaled angular acceleration, one-step
pulse clearing, and zero static drift. The focused behavior/order/replay/panel
command reported `5 passed`, `py-demos --list` shows the row directly after
`rigid_body`, and the `rigid_external_loads` visual capture wrote nonblank
960x540 PNG frames. `pixi run test-py` reported `597 passed, 9 skipped`,
`pixi run lint` passed, bounded `pixi run build` passed with
`DART safe jobs: 5`, and `git diff --check` was clean after the final
external-loads evidence update.
The previous front-door alignment follow-up updated the default launch scene,
catalog order, quickstart docs, workflow docs, and focused ordering/default
tests. The focused default/order/category command reported `3 passed`, and the
`rigid_body` visual capture wrote nonblank 960x540 PNG frames. `pixi run lint`
passed, bounded `pixi run build` passed with `DART safe jobs: 5`, and
`git diff --check` was clean after the final front-door evidence update.
The previous kinematic-driver follow-up has passed a direct scene probe,
focused behavior/order/replay/panel tests, full Python sweep, visual capture,
`pixi run lint`, bounded `pixi run build`, and `git diff --check`.
The previous collision-query-options follow-up has passed the direct scene
probe, focused filter/order/replay/panel tests, full Python sweep, visual
capture, `pixi run lint`, bounded `pixi run build`, and `git diff --check`.

## Immediate Next Step

Use the current handoff snapshot at the top of this file for live branch state.
After the local row-29 checkpoint is committed, continue the capture-metrics
hardening pass with
`rigid_joint_passive_parameters` unless a fresh audit finds a higher-priority
missing rigid-body GUI evidence gap.

## Context That Would Be Lost

- The broad goal is not complete. This is only the first P0 visual-debug slice.
- Two read-only research passes agreed that the highest-value next pattern is a
  side-by-side rigid solver comparison, followed by friction threshold,
  no-tunneling, and small stacking/mass-ratio scenes.
- A third read-only pass recommended friction threshold before no-tunneling
  because `rigid_ipc_tunnel` and `rigid_solver_compare` already cover the thin
  wall shape. It also recommended avoiding exact-threshold completion claims.
- A fourth read-only pass recommended the small resting-stack / mass-ratio
  verifier next because no-tunneling and constraints already have partial GUI
  coverage.
- Backend controls should compare executor performance/equivalence, not imply a
  separate physics mode.
- Contact-solver policy should stay separate from rigid-body solver family.
  `rigid_contact_solver_compare` is about sequential-impulse contacts versus
  boxed-LCP contacts under the same rigid solver, not IPC-vs-sequential rigid
  method-family behavior.
- Restitution should remain a relative material-response diagnostic. The
  default sequential-impulse lane ordering verifies higher restitution gives a
  stronger rebound in this setup; the energy plots are debugging signals, not a
  conservation guarantee.
- Contact inspection should remain query-level observability. Use it to debug
  contact packets before claiming solver-family differences. Broaden the
  existing contact-inspector row for public shape-family coverage instead of
  adding a duplicate collision-sandbox catalog row.
- Collision query options should remain a query-filtering diagnostic for
  `World.collide(options)`: rigid/rigid, rigid/link, same-multibody link/link,
  cross-multibody link/link, and persistent ignored pairs. It should not
  duplicate the shape-family manifold inspector or become a solver benchmark.
- Kinematic driver visuals should stay scoped to tangential prescribed motion
  under IPC: moving supports/conveyors, slip, and the sequential-impulse
  static-like caveat. Normal prescribed contact now belongs in
  `rigid_kinematic_normal_push`, which intentionally exposes the IPC
  penetration caveat next to an SI push lane instead of claiming robust IPC
  normal manipulation.
- The app front door should stay aligned with the curated rigid workflow:
  default `py-demos` launch opens `rigid_body`, while `replay_scrubber` remains
  an explicit scene for replay-specific debugging.
- The next distinct rigid-body feature gap is external loads: persistent
  `RigidBody.force`/`torque` accumulators and `apply_force`/`apply_torque`
  response. That gap is now covered by `rigid_external_loads`; keep any future
  point-force or impulse work separate unless it answers a distinct public API
  question.
- The no-contact rigid free-flight/initial-state gap is now covered by
  `rigid_free_flight`. Add a separate rigid energy/momentum accounting row only
  if it answers a distinct user question beyond the free-flight diagnostics.
  Keep contacts, restitution, load accumulators, and solver comparisons out of
  that row unless they become the distinct question.
- The explicit loop-closure gap is now covered by `rigid_loop_closure` as a
  public-family comparison row. POINT covers endpoint locking, DISTANCE covers
  tether length, and RIGID covers full-pose welds; add another row only for a
  distinct behavior such as compliance or breakage.
- The passive joint-parameter gap is now covered by
  `rigid_joint_passive_parameters` as a contact-free World multibody row. Add
  future passive rows only if they answer a distinct public question beyond
  spring/rest, damping, Coulomb friction, and armature.
- Sleeping/deactivation/island activation rows should stay deferred until
  public dartpy APIs expose those states; current branch audits found only
  internal island machinery and no public sleep/wake/deactivation surface.
- The current branch intentionally keeps `rigid_ipc_tunnel` as a focused IPC
  capability scene in the Rigid IPC shelf rather than adding another
  thin-wall comparison until there is a distinct user question not covered by
  `rigid_solver_compare`.
- Rigid-body joint motor visuals should stay deferred until that API behavior is
  stable enough to support bounded visual assertions. The current motor/limit
  row uses World multibody joints instead.
- Baseline local validation for the P0 workflow passed: focused rigid workflow
  checks, all documented visual captures, `pixi run lint`, `pixi run build`,
  the Python suite, `pixi run -e cuda test-all`, and `git diff --check`.
- Latest contact-solver follow-up validation passed: the 13-test focused rigid
  workflow refresh, `rigid_contact_solver_compare` visual capture,
  `pixi run lint`, `pixi run build`, and `git diff --check`.
- Latest restitution follow-up validation passed: the 14-test focused rigid
  workflow refresh, `rigid_restitution_ladder` visual capture, `pixi run lint`,
  `pixi run build`, and `git diff --check`.
- Latest joint motor/limit capture-metrics follow-up validation passed:
  focused motor-limit guard reported `1 passed`, the broader workflow/doc drift
  guard reported `11 passed`, the docked capture under
  `/tmp/dart_capture_joint_motor_limits_metrics_1781234533` wrote nonblank
  960x540 PNG frames plus 72 scene-metrics events, `pixi run lint` passed,
  bounded `pixi run build` passed with `DART safe jobs: 5`, and
  `git diff --check` passed.
- Latest loop-closure follow-up validation passed: the focused
  category/order/behavior/replay/panel command reported `5 passed`;
  `rigid_loop_closure` visual capture wrote a nonblank 960x540 screenshot and
  71 nonblank PNG frames with final contacts at 0; `pixi run test-py` reported
  `600 passed, 9 skipped`; `pixi run lint` passed; bounded
  `pixi run build` passed with `DART safe jobs: 5`; and `git diff --check` was
  clean.
- Latest passive joint-parameter follow-up validation passed so far:
  `pixi run test-py` reported `601 passed, 9 skipped`, and
  `rigid_joint_passive_parameters` visual capture wrote a nonblank 960x540
  screenshot plus 119 nonblank PNG frames with final contacts at 0.
  `pixi run lint` passed; bounded `pixi run build` passed with
  `DART safe jobs: 5`; and `git diff --check` was clean.
- Latest contact-inspector shape-family follow-up validation passed: the
  3-test focused contact/panel/replay command reported `3 passed`, the
  `rigid_contact_inspector` visual capture wrote nonblank 960x540 PNG frames,
  `pixi run lint` passed, bounded `pixi run build` passed with
  `DART safe jobs: 5`, and `git diff --check` was clean. The focused contact
  integration test verifies every lane has finite contact geometry and compound
  shape-index reporting.
- Current collision-query-options follow-up validation passed so far: direct
  scene probe reported four baseline contacts and four active contacts; the
  focused filter/order/replay/panel command reported `4 passed`;
  `pixi run test-py` reported `595 passed, 9 skipped`; and
  `rigid_collision_query_options` visual capture wrote nonblank 960x540 PNG
  frames. `pixi run lint`, bounded `pixi run build`, and `git diff --check`
  also passed after the scene, registry, test, replay-snapshot, and
  documentation updates.
- Current kinematic-driver follow-up validation passed so far: direct scene
  probe showed IPC grip carrying the box, IPC zero-friction slip, and the
  sequential static-like caveat; the focused behavior/order/replay/panel command
  reported `5 passed`; and `rigid_kinematic_driver` visual capture wrote
  nonblank 960x540 PNG frames. `pixi run test-py` reported `596 passed, 9
skipped`; `pixi run lint` passed; bounded `pixi run build` passed with
  `DART safe jobs: 5`; and `git diff --check` was clean after the final
  kinematic-driver evidence update.
- Current front-door alignment validation passed so far: the focused
  default/order/category command reported `3 passed`, `py-demos --list` shows
  `rigid_body` and the curated rigid verifier block at the front of the catalog,
  `rigid_body` visual capture wrote nonblank 960x540 PNG frames,
  `pixi run lint` passed, bounded `pixi run build` passed with
  `DART safe jobs: 5`, and `git diff --check` was clean after the final
  front-door evidence update.
- Current external-loads validation passed so far: direct scene probe reported
  the intended force, torque, pulse, and static-body metrics; focused
  behavior/order/replay/panel tests reported `5 passed`; `py-demos --list`
  shows `rigid_external_loads` directly after `rigid_body`; and
  `rigid_external_loads` visual capture wrote nonblank 960x540 PNG frames.
  `pixi run test-py` reported `597 passed, 9 skipped`; `pixi run lint` passed;
  bounded `pixi run build` passed with `DART safe jobs: 5`; and
  `git diff --check` was clean after the final evidence update.
- Current point-load validation passed so far: direct scene probe reported
  centered translation, world-space off-center lever-arm yaw acceleration,
  one-step pulse clearing, double-apply accumulation, and world/local frame
  separation; focused behavior/order/replay/panel tests reported `5 passed`;
  and visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG
  frames with final contacts at 0. `pixi run test-py` reported `602 passed, 9
skipped`; `pixi run lint` passed; and bounded `pixi run build` passed with
  `DART safe jobs: 5`.
- Current step-diagnostics validation passed so far: focused
  category/order/profile-memory/replay/panel tests reported `5 passed`;
  visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG
  frames; `pixi run test-py` reported `603 passed, 9 skipped`;
  `pixi run lint` passed; and bounded `pixi run build` passed with no work
  remaining. `git diff --check` was clean after the final documentation update.
- Current material-mixing validation passed so far: focused
  category/order/pair-rule/replay/panel tests reported `5 passed`, and visual
  capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG frames with
  final contacts at 0. `pixi run test-py` reported `604 passed, 9 skipped`;
  `pixi run lint` passed; and bounded `pixi run build` passed with no work
  remaining.
- Current multibody solver-family validation passed so far:
  `rigid_multibody_solver_family` is implemented after the screw-joint pitch
  row and before the loop-closure family row. The focused
  behavior/order/replay/panel guard reported `8 passed`; `pixi run test-py`
  reported `611 passed, 9 skipped`; visual capture wrote a nonblank 960x540
  screenshot plus 71 PNG frames with final contacts at 0 and RGB variance
  `[2505.408, 2673.722, 2866.646]`.
- Current multibody dynamics-terms validation passed so far:
  `rigid_multibody_dynamics_terms` is implemented after the screw-joint pitch
  row and before the solver-family row. The focused
  behavior/order/replay/panel guard reported `8 passed`; `pixi run test-py`
  reported `612 passed, 9 skipped`; visual capture wrote a nonblank 960x540
  screenshot plus 95 PNG frames with final contacts at 0 and RGB variance
  `[2496.525, 2663.351, 2868.943]`.
- Current link-Jacobian validation passed so far: `rigid_link_jacobian` is
  implemented after generalized dynamics terms and before the solver-family
  row. The focused behavior/order/replay/panel guard reported `8 passed`;
  `pixi run test-py` reported `613 passed, 9 skipped`;
  visual capture wrote a nonblank 960x540 screenshot plus 95 PNG frames with
  final contacts at 0 and RGB variance `[2483.852, 2653.334, 2866.583]`.
- Current contact-scale budget validation passed so far:
  `rigid_contact_scale_budget` is implemented after the step-diagnostics row and
  before the restitution ladder. The focused behavior/order/replay/panel guard
  reported `8 passed`; `pixi run test-py` reported `614 passed, 9 skipped`;
  `pixi run lint` passed; the focused post-lint guard again reported `8 passed`;
  and `git diff --check` was clean. Visual capture wrote a nonblank 960x540
  screenshot plus 71 PNG frames. A 72-frame controller probe reported
  one/four/nine-box contact points at 4, 16, and 36 with dense/single wall ratio
  `6.016` and RGB variance `[2502.671, 2667.095, 2860.303]`. The selected slice
  came from a fresh audit of remaining public rigid workflow gaps:
  arbitrary-point/contact Jacobian and sleep/island rows are deferred until
  stronger public APIs exist.
- Current collision-casts validation passed so far: `rigid_collision_casts` is
  implemented after the collision-query-options row and before the solver-family
  row. The focused behavior/order/replay/panel guard reported `7 passed`;
  `pixi run test-py` reported `615 passed, 9 skipped`; `pixi run lint` passed;
  and bounded `pixi run build` passed with `DART safe jobs: 3` and
  `ninja: no work to do`; `git diff --check` was clean after the final
  documentation update. Visual capture wrote a nonblank 960x540 screenshot plus
  47 PNG frames with final contacts at 0, docked UI detected, 4286 unique colors,
  and RGB variance `[2507.382, 2770.902, 2914.942]`.
- Current Demos navigator title validation passed so far: the focused
  list/sidecar/viewer-title/README guard reported `4 passed`; `pixi run lint`
  passed; the short docked capture command for `rigid_body` wrote a nonblank
  960x540 screenshot plus 7 PNG frames through the docked viewer path;
  `pixi run test-py` reported
  `616 passed, 9 skipped`; bounded `pixi run build` reported
  `ninja: no work to do`; and `git diff --check` was clean after the evidence
  update.
- The bounded contact-rich manipulation row is now implemented as
  `rigid_contact_manipulation`.
- A fuller articulated arm/gripper row remains deferred after the latest public
  API/runtime probe. Rigid-body joints are not supported by the IPC rigid-body
  solver, multibody link contacts do not expose material/friction controls, and
  scripted IPC two-jaw pinch settings that actually carry a target measured in
  the hundreds of milliseconds per step. Do not add this as a default live GUI
  row until the public API can express a stable, interactive gripper without
  overclaiming IK, actuator dynamics, or link-material behavior.
- The local review packet draft now lives at `PR_DRAFT.md` and follows the
  repository PR template for a `main`/DART 7.0 review. That draft-only slice
  did not perform a commit, push, or PR mutation.
- Fresh pre-review validation on 2026-06-10: `pixi run test-py` reported
  `615 passed, 9 skipped`; bounded `pixi run build` passed with
  `DART safe jobs: 3` and `ninja: no work to do`; `pixi run lint` passed; the
  focused post-lint guard reported `7 passed`; `git diff --check` was clean; and
  the current branch still has no associated PR.
- Fresh navigator-title validation on 2026-06-11: `pixi run test-py` reported
  `616 passed, 9 skipped`; bounded `pixi run build` passed with
  `DART safe jobs: 3` and `ninja: no work to do`; `pixi run lint` passed; the
  focused list/sidecar/viewer-title/README guard reported `4 passed`;
  `py-demo-capture` produced a nonblank docked `rigid_body` screenshot; and
  `git diff --check` was clean. The current branch still has no associated PR.
- Fresh breakage-lifecycle validation on 2026-06-11: the focused
  category/order/sidecar/viewer-title/README/capture-command/panel guard reported
  `8 passed`, and `py-demo-capture` produced a nonblank docked
  `rigid_joint_breakage` screenshot plus 47 PNG frames with final contacts at 0.
  `pixi run test-py` reported `617 passed, 9 skipped`; bounded
  `pixi run build` passed with `DART safe jobs: 3` and `ninja: no work to do`;
  `pixi run lint` passed; and `git diff --check` was clean after the evidence
  update.
- Fresh navigator-count drift validation on 2026-06-11: the focused
  viewer-title/docs-count/sidecar/README guard reported `4 passed` after fixing
  stale navigator-count examples to `15/36`; `pixi run lint` passed after adding
  the guard.
- Fresh frame-hierarchy validation on 2026-06-11: the focused
  category/order/viewer-title/docs-count/sidecar/README/capture-command/frame
  invariant/replay/panel guard reported `12 passed` before and after lint.
  `py-demo-capture` produced a nonblank docked `rigid_frame_hierarchy`
  screenshot plus 71 PNG frames with final contacts at 0. `pixi run lint`
  passed; bounded `pixi run build` reported `ninja: no work to do`; and
  `git diff --check` was clean after the evidence update.
- Fresh baseline-hardening validation on 2026-06-11: the focused
  baseline/body-mode/frame/breakage/replay/category/order/viewer-title/docs-count/
  sidecar/README/capture-command/panel guard reported `15 passed`. The docked
  `rigid_body` visual capture produced a nonblank 960x540 screenshot plus 23 PNG
  frames with final contacts at 0. `pixi run lint` passed, and bounded
  `pixi run build` reported `ninja: no work to do`.
- Fresh spin/roll coupling validation on 2026-06-11: the focused
  category/order/viewer-title/docs-count/sidecar/README/capture-command/spin-roll/
  replay/panel guard reported `10 passed`. The docked
  `rigid_spin_roll_coupling` visual capture produced a nonblank 960x540
  screenshot plus 95 PNG frames with final contacts at 0. `pixi run lint`
  passed; bounded `pixi run build` reported `ninja: no work to do`; and
  `git diff --check` was clean after the evidence update.
- Fresh in-viewer workflow-guide follow-up on 2026-06-11: a specialized UX
  audit found that the 36-row learning path was still mostly documented outside
  the app. The runner now injects a compact `Rigid Workflow` panel for numbered
  World Rigid Body rows, sourced from the PLAN-103 user questions, with inspect
  signals and previous/next route guidance. The focused
  category/order/viewer-title/guidance/docs-count/sidecar/README/capture-command/
  spin-roll/replay/panel guard reported `13 passed` before and after lint.
  Docked visual captures for `rigid_body`, `rigid_solver_compare`, and
  `rigid_loop_closure` each produced nonblank 960x540 screenshots plus 7 PNG
  frames with final contacts at 0. `pixi run lint` passed, and bounded
  `pixi run build` reported `ninja: no work to do`.
- Fresh checklist/contact-body hardening on 2026-06-11: the shared
  `Rigid Workflow` panel now presents each numbered row as `Question`,
  `Try first`, `Look for`, and `Do not infer` so the main control, healthy
  signal, and scope caveat are visible in-app. `rigid_collision_query_options`
  now surfaces public `Contact.body_a/body_b` `CollisionBody` kind, validity,
  and cast diagnostics for rigid/rigid, rigid/link, same-multibody link/link,
  and cross-multibody link/link lanes. The focused
  category/order/viewer-title/guidance/docs-count/sidecar/README/capture-command/
  query-options/spin-roll/replay/panel guard reported `14 passed` after lint.
  Docked visual captures for `rigid_body`, `rigid_solver_compare`,
  `rigid_loop_closure`, and `rigid_collision_query_options` produced nonblank
  960x540 screenshots.
- Fresh workflow-route navigation on 2026-06-11: Python panel contexts now
  expose `request_scene_switch(scene_id)`, and the shared `Rigid Workflow`
  panel renders previous/next routes plus a direct row selector as scene-switch
  rows. The focused panel route/docs guard reported `9 passed`; after rebuilding
  dartpy, the focused GUI lifecycle binding guard reported `2 passed` for
  `request_scene_switch()` and `request_scene_replay()` state transitions.
- Fresh workflow-search/restart hardening on 2026-06-11: Python panel contexts
  now also expose `request_scene_replay(scene_id)`, and the shared
  `Rigid Workflow` panel renders a restart command plus a text filter over row
  ids, questions, checklist text, and inspect signals. The filter now includes
  explicit `NN/MM` row-id tokens such as `15/36` so documented row-id search
  switches to the intended workflow row. The focused panel/stub guard reported
  `7 passed`.
- Fresh collision-casts capsule hardening on 2026-06-11:
  `rigid_collision_casts` now visualizes public swept-capsule
  time-of-impact queries alongside raycast and swept-sphere probes, with
  capsule offset/radius/height controls, margins, point/normal metrics, and
  replayed controls. The focused casts/replay guard reported `2 passed`; the
  docked visual capture produced a nonblank 960x540 screenshot plus 47 PNG
  frames with final contacts at 0.
- Fresh stack-stability verification hardening on 2026-06-11: the existing
  `rigid_stack_stability` invariant test now checks both solver lanes,
  per-lane metric histories, and finite top-x divergence while keeping the
  strict stability thresholds on IPC. The focused stack guard reported
  `3 passed`, and the docked visual capture produced a nonblank 960x540
  screenshot plus 23 PNG frames.

The latest follow-up promotes the stable `contact` scene id into row 18/36 as
`Rigid Link Contact`, directly after contact-solver policy and before the
friction-threshold row. It now shows multibody links dropping, friction-sliding,
and pushing a rigid target through the public World contact path, with executor,
friction/restitution, drop/slide/push controls, link contact counts, rebound,
slide travel, target travel, replay restore coverage, and docked visual capture.
Arm/gripper manipulation, direct contact impulse, sleep/island/deactivation,
and loop compliance remain blocked until public APIs or runtime evidence justify
GUI rows.

Fresh link-contact validation on 2026-06-11: the focused
order/guidance/docs/capture/contact/replay/panel guard reported `18 passed`;
the docked `contact` capture produced a nonblank 960x540 screenshot plus 71 PNG
frames; `pixi run lint`, bounded default `pixi run build`, and
`git diff --check` passed; and CUDA-host `pixi run -e cuda test-all` reported
all 7 stages passed, including 206 C++ tests, 68 simulation tests, 643 Python
items, documentation, and CUDA benchmark smoke.

Fresh capture-evidence hardening on 2026-06-11: `py-demo-capture` manifests now
record requested width/height/frames, converted frame count, and
`visual_evidence` for the screenshot and first UI-ready frame: dimensions,
nonzero pixels, unique RGB count, RGB/luminance variance, and
docked-workspace detection. `pixi run test-py` reported
`636 passed, 9 skipped`; a direct `pixi run python -m pytest` probe still needs
the built dartpy `PYTHONPATH` and is not the right entry point for this suite.

Fresh guided-replay timeline hardening on 2026-06-11: the shared bottom
`Replay` panel now accepts optional `replay_timeline` metadata so a scene can
feed the saved-state scrubber one diagnostic signal and one marker track from
recorded replay snapshots. `rigid_solver_compare` uses this to show position
divergence and near-wall marker frames without adding another row or panel. The
focused replay timeline and solver-comparison guard reported `5 passed`, and
`pixi run test-py` reported `640 passed, 9 skipped` on the post-lint tree.

Fresh related-evidence routing on 2026-06-11: the runner-owned `Rigid Workflow`
panel now keeps non-numbered shelves visible without changing the 36-row order.
Row 3/36 `rigid_free_flight` links to `floating_base` as a broader
floating-joint drift/spin example, row 32/36
`rigid_multibody_dynamics_terms` links to `articulated` as a broader two-link
arm example, row 15/36 `rigid_solver_compare` links to `rigid_ipc_tunnel` as a
focused IPC no-tunneling view, and row 17/36
`rigid_contact_solver_compare` links to `diff_drone_liftoff` as a
differentiable contact-gradient route. The same route table also links row
18/36 `contact` to `avbd_rigid_fixed_joint_contact`, row 26/36
`rigid_joint_breakage` to the AVBD fixed/spherical break-reset scenes, and row
29/36 `rigid_joint_motor_limits` to the AVBD revolute/prismatic motor scenes.
The PLAN-103 sidecar owns the route table, tests verify those scene ids remain
registered outside the numbered workflow, and `Find row` now indexes related
shelf names, scene ids, labels, and scope notes so searches such as
`floating_base`, `two-link arm`, `rigid_ipc_tunnel`, `contact gradient`,
`avbd fixed contact`, `avbd spherical`, and `avbd prismatic` route to the
numbered source row. When the match comes only from related evidence, the
search result labels the target scene and the tooltip explains the related
shelf. The sidecar and Python demo README also document docked
`py-demo-capture --show-ui` commands for every related target, with drift
coverage tying those commands to the route table.

Fresh capture-first IPC stack packet on 2026-06-11: `rigid_ipc_stack_packet`
was added to the non-numbered Rigid IPC shelf, outside the 36-row World Rigid
Body workflow, to answer what happens when a four-box IPC stack leaves the live
demo budget. Its panel reports frame-budget status, wall time, min clearance,
contact count, top drift, height error, max speed, and the
`bm_rigid_ipc_solver` benchmark owner. The PLAN-103 sidecar now has a dedicated
`Capture-First Rigid IPC Packets` table so future heavy packets stay separate
from numbered workflow rows. The focused catalog/sidecar/panel/packet guard
reported `12 passed`; the docked capture produced a nonblank 960x540 screenshot
plus 23 PNG frames, and a 24-step probe ended capture-first with min clearance
`0.000267` m, top drift `0.000360` m, and last-step wall time `805.087` ms.

Fresh scene-metrics capture hardening on 2026-06-11: `py-demo-capture` now
passes a runner-local metrics log path to `py-demos`; scenes that expose
`SceneSetup.info["capture_metrics"]` write per-frame `scene_metrics.jsonl`
events during the actual viewer loop, and the capture manifest summarizes the
stream with first/latest events, per-key presence counts, and top-level numeric
ranges. `rigid_ipc_stack_packet` exports its clearance, contact-count, drift,
height-error, speed, wall-time, frame-budget, solver, capture-first, and
`bm_rigid_ipc_solver` benchmark fields through that hook. The capture/runner
unit guard reported `25 passed`, the stack-focused guard reported `3 passed`,
and the real docked stack capture wrote 24 scene-metrics events with frame 24
ending capture-first at min clearance `0.000267` m and step wall time
`626.778` ms.

Fresh runtime-row capture metrics on 2026-06-11: the numbered runtime evidence
rows now use the same capture-metrics hook. `rigid_step_diagnostics` exports
solver/executor, time step, world time, and each lane's profile, memory, speed,
and contact counters; `rigid_contact_scale_budget` adds budget, friction,
dense/single wall-time ratio, and per-lane contact-budget metrics. The focused
guard
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads -q`
reported `2 passed`. Short real `py-demo-capture --show-ui` runs for both rows
wrote eight scene-metrics events each and mirrored the expected lane sets into
the manifests.

Fresh numbered workflow capture hardening on 2026-06-11: the capture test now
uses the docking build to run a real `rigid_contact_scale_budget --show-ui`
capture through the numbered World Rigid Body workflow. The new guard checks
the nonblank docked screenshot and first frame, confirms a `scene_metrics`
manifest summary exists, and verifies the latest metrics row exports
`single/medium/dense`. A companion factory-composition guard verifies the
actual numbered row gets `Rigid Workflow`, `Rigid Contact Scale Budget`, and
`Replay` panels in order.

Fresh post-push replay-contract refresh on 2026-06-11: the branch was committed
as `245095f1164` and pushed to
`origin/feature/rigid-body-gui-visual-verification`. The first full
`pixi run test-py` caught `rigid_ipc_stack_packet` missing shared replay-state
hooks; the packet now exposes capture/restore callbacks for its controls and
history, the focused replay/stack guard reported `2 passed`, and the refreshed
full Python sweep reported `648 passed, 9 skipped`.

Fresh workflow-search UX follow-up on 2026-06-11: a specialized UX audit found
that `Find row` ranked scope caveats before intent matches. The runner now
scores row ids, scene ids, labels, questions, and positive signals before scope
caveats, so `contact` and `solver` searches surface the intended workflow rows.
Related-shelf links now include the target shelf and scene id in the visible
row label.

Fresh kinematic normal-push caveat follow-up on 2026-06-11:
`rigid_kinematic_normal_push` is row 24/36 after the tangential kinematic-driver
row. The focused order/guidance/docs/replay/panel/normal-push guard reported
`17 passed`. The standard docked capture wrote a nonblank 960x540 screenshot,
71 PNG frames, and 72 scene-metrics events; frame 72 ended with IPC normal and
heavy lanes at `status=ipc penetration caveat` with about 0.125 m depth and
near-zero target travel, while the sequential-impulse lane reached
`status=pushed` with about 0.123 m target travel.

Fresh solver/contact comparison capture-metrics follow-up on 2026-06-11:
`rigid_solver_compare` now records method-family capture metrics for
sequential impulse vs IPC cases, and `rigid_contact_solver_compare` records
contact-policy capture metrics for sequential-impulse vs boxed-LCP contact
methods. The focused guard reported `2 passed`. The real docked captures wrote
24 and 72 scene-metrics events respectively, with nonblank 960x540 docked
screenshots and manifest summaries for case enums, controls, executor, world
time, and divergence histories.

Fresh baseline capture-metrics follow-up on 2026-06-11: `rigid_body` now records
the default first-run scene's solver/material controls, dynamic-body count,
current speed/height/energy/contact/timing diagnostics, and history ranges in
`py-demo-capture` manifests. The focused baseline guard reported `1 passed`, and
the real docked capture wrote a nonblank 960x540 screenshot, 23 PNG frames, and
24 scene-metrics events.

The specialized-agent audit for the next slice ranked row 18 `contact` first,
row 12 `rigid_contact_inspector` second, and row 5 `rigid_external_loads` third
for capture-metrics follow-up value/risk. Prefer `contact` next unless PR/CI
state or maintainer instruction changes the priority.

## How to Resume

```bash
git checkout feature/rigid-body-gui-visual-verification
git fetch origin main feature/rigid-body-gui-visual-verification
git status -sb
git log --oneline --decorate --max-count=5
```

If the branch is clean and matches origin after this handoff push, this slice is
done locally. Use `PR_DRAFT.md` only after maintainer approval to update or open
a PR. For future implementation work, refresh the focused curation guard before
editing:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_default_timeline_without_metadata \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_ignores_malformed_timeline_metadata \
    python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response -q
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics \
    python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics \
    python/tests/integration/test_demos_cycle.py::test_rigid_free_flight_preserves_initial_state_diagnostics \
    python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame \
    python/tests/integration/test_demos_cycle.py::test_rigid_external_loads_scale_force_and_torque_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_link_point_loads_show_lever_arm_and_frame_semantics \
    python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size \
    python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_inspector_reports_contact_manifolds \
    python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds \
    python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits \
    python/tests/integration/test_demos_cycle.py::test_rigid_restitution_ladder_orders_rebound_height \
    python/tests/integration/test_demos_cycle.py::test_rigid_material_mixing_applies_pair_rules \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy \
    python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_friction_threshold_separates_stick_and_slip_lanes \
    python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal \
    python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc \
    python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation \
    python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures \
    python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families \
    python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls -q
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/unit/collision/test_collision.py::test_continuous_capsule_cast_result -q
pixi run test-py -- python/tests/unit/test_py_demo_panels.py python/tests/integration/test_demos_cycle.py
pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_body_modes --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_free_flight --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_external_loads --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_point_loads --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_timestep_sensitivity --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_restitution_ladder --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_material_mixing --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_inspector --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_solver_compare --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene contact --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui
pixi run lint
DART_PARALLEL_JOBS=${DART_SAFE_JOBS:-5} \
    CTEST_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} \
    CMAKE_BUILD_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} \
    pixi run build
git diff --check
```

## 2026-06-11 Fixed/One-DOF Joint Capture Metrics Follow-Up

The current follow-up adds scene-owned `capture_metrics` hooks to the existing
`rigid_fixed_joint` and `rigid_limited_joints` rows. The captures now record
fixed relative-transform errors plus revolute/prismatic locked-axis errors in
`scene_metrics.jsonl`, matching the panel diagnostics without changing row
ordering or scope claims.

Validation so far:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform \
    python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions \
    python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow \
    python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_metrics_1781222114
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_metrics_1781222114
pixi run lint
pixi run build
```

Observed capture evidence:

- `rigid_fixed_joint`: nonblank 960x540 docked screenshot, 23 PNG frames, 24
  scene-metrics events, latest translation error
  `2.1673896011265015e-10`.
- `rigid_limited_joints`: nonblank 960x540 docked screenshot, 23 PNG frames, 24
  scene-metrics events, latest hinge radius/z and slider orthogonal errors
  `0.0`.

## 2026-06-11 Joint-Breakage Capture Metrics Follow-Up

The current follow-up adds scene-owned `capture_metrics` to the shared
`avbd_rigid_breakable_joint` builder and tags the numbered
`rigid_joint_breakage` row with its own row id. The metric stream records the
AVBD fixed break-force lifecycle: intact/broken status, captured-offset error,
payload release distance, payload speed, dynamic break-force value, and whether
the capture saw a broken state. This keeps the row AVBD-pinned and does not add
sequential-impulse or IPC breakage claims.

Validation so far:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage \
    python/tests/integration/test_demos_cycle.py::test_avbd_breakable_joint_demo_marks_and_resets_joint -q
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_breakage_metrics_1781223331
pixi run lint
DART_PARALLEL_JOBS=${DART_SAFE_JOBS:-5} CTEST_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} CMAKE_BUILD_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} pixi run build
```

Observed capture evidence:

- `rigid_joint_breakage`: nonblank 960x540 docked screenshot, 47 PNG frames,
  48 scene-metrics events, latest status `broken`, `saw_broken=1.0`, and latest
  `payload_release_distance=0.41319290960568955`.
