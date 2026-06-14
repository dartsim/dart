# PLAN-103 Sidecar: Rigid-Body Visual Verification

This packet owns the curated rigid-body `py-demos` visual-verification workflow
for PLAN-103. It records which scenes answer which user debugging questions,
how to regenerate visual evidence, and which automated tests keep the workflow
from drifting.

## Work Packet

#### WP-103.1 Rigid-body visual verification workflow

- Objective: the Python `py-demos` rigid-body category is a curated DART 7
  visual-verification workflow with reviewable screenshots, row guidance,
  scene metrics, resolved solver identity, and replay/debug evidence.
- Scope: `python/examples/demos/` rigid-body scenes, runner workflow metadata,
  `scripts/capture_py_demo.py`, focused Python tests, this PLAN-103 sidecar,
  `python/examples/demos/README.md`, and `CHANGELOG.md`.
- Non-goals: new C++ `dart-demos` rows, public APIs for unsupported
  sleep/island/loop-closure compliance behavior, benchmark replacement, or
  maintainer acceptance/PR publication without explicit approval.
- Acceptance evidence: default 36-row and optional extended workflow manifests
  report complete guidance, scene metrics, and resolved solver identity; static
  review indexes resolve their local assets; focused capture and docs drift
  tests pass; broad default and CUDA validation evidence is recorded before PR
  publication.
- Gates: `pixi run lint`, focused `python/tests/unit/test_capture_py_demo.py`
  capture-helper guards, focused `python/tests/integration/test_demos_cycle.py`
  workflow/docs drift guards, `pixi run test-all`, and CUDA `pixi run -e cuda
test-all` on a host with visible CUDA runtime.
- Dependencies: PLAN-103 policy in
  [`../103-examples-strategy.md`](../103-examples-strategy.md), DART 7
  evidence rules from PLAN-091/PR #2986, and current public `dartpy.simulation`
  rigid API availability.
- Evidence: current local evidence is recorded below and in the workflow capture
  manifests/review indexes named in this sidecar. PR publication and milestone
  setup are complete; maintainer acceptance and hosted review/CI remain external
  gates.

## Scope

The workflow covers DART 7 `World` rigid dynamics as an interactive Python
examples surface. It is a user-facing debugging path, not a replacement for
solver correctness tests, IPC fixture manifests, or benchmark packets.

In scope:

- side-by-side method-family comparison where the user naturally asks how
  sequential impulses differ from rigid IPC;
- focused material-response rows where the user needs to see a parameter trend
  without turning it into a solver-family comparison;
- raw contact-observability rows where the user needs contact pairs, points,
  normals, depths, local points, shape indices, body-kind query filtering, and
  persistent ignored pairs before comparing solvers;
- focused IPC scenes where the user question is a threshold or guarantee;
- capture commands that exercise the same docked Filament + ImGui workspace as
  the interactive viewer;
- focused pytest coverage for scene ordering, panels, and representative
  physical invariants.

Out of scope:

- new C++ `dart-demos` scenes while PLAN-103 keeps C++ examples frozen;
- claiming exact analytic thresholds for near-discontinuous contact behavior;
- live-GUI-heavy IPC stacks that are better handled as benchmark or
  capture-first packets until runtime improves;
- sleep/deactivation/island-state and loop-closure compliance rows until public
  dartpy exposes those surfaces.

Current public API audit: `RigidBody` exposes force/torque accumulators plus
direct linear/angular impulse and momentum accessors, `Link.apply_force()`
exposes one-shot point-load semantics, and
`Multibody.compute_impulse_response()` exposes joint-space impulse response.
The public direct rigid-body impulse surface is covered by
`rigid_external_loads`; there is still no public sleep/wake or island
activation surface and no public loop-closure compliance surface, so those
candidate GUI rows remain deferred.

## Curated Workflow

In the interactive `py-demos` viewer, the first 36 **World Rigid Body** rows
use navigator titles with their workflow position and role, for example
`01/36 Baseline: World Rigid Body` and
`15/36 Solver family: Rigid Solver Compare`. The CLI `--list` output keeps the
stable scene titles and ids for scripts. These rows also receive a runner-owned
`Rigid Workflow` panel with the row's maintained user question, workflow phase,
focus axis, what to try first, the main signals to inspect, the known
scope/limitation, and the previous/next numbered route, restart command,
direct row selector, and ranked row filter as scene-switch rows. Workflow
phases group the numbered rows into foundations, diagnostics,
contact/material/query basics, the solver decision path, contact behavior
cases, rigid constraints/joint mechanics, and multibody
dynamics/kinematics. The panel also shows the exact
per-row live `pixi run py-demos -- --scene ...` open command and paired
`pixi run py-demo-capture` command, with frame count, resolution, and
docked-UI mode so users can jump from the workflow row into the live viewer or
regenerate visual evidence from the same context. It also lists the full
numbered workflow packet command, the current row-range rerun command, a
current-row motion packet command with `--video --fps 24`, and the extended
related/IPC-shelf/packet command that generate `manifest.json` plus
`review_index.html`.
Rows that compare solvers, executors, contact policies, time-step
multipliers, workload sizes, or passive joint parameter families label the
comparison axis and held-fixed controls directly in both the panel and capture
metrics.
The workflow capture helper also writes `review_index.html` beside the
top-level manifest. That static contact sheet links every numbered row's
manifest, screenshot, frame directory, live open command, workflow-row rerun
command, direct capture command, maintained user question, try-first guidance,
healthy signal, scope note, and comparison/metrics summary so a reviewer can
scan all 36 captures without opening each scene folder. When workflow captures
request `--video`, the same review index links each row's MP4 motion artifact
if `ffmpeg` is available. The top-level manifest and review header also record
the exact workflow packet command that produced the artifact, and the manifest
publishes `workflow_phase_summary` for selected numbered rows. The review
header mirrors that summary as a `Workflow Phase Map` with row ranges, phase
counts, per-phase status, focus axes, and selected scene ids, so full and
row-range packets expose their logical route and capture state before the
reviewer opens individual cards. Unit coverage keeps captured row cards to one
screenshot thumbnail per row while checking that manifest, screenshot, frame,
and video links resolve from the static review sheet.
Rows with scene-owned Replay timeline metadata also export a JSON-safe
`scene_metadata.replay_timeline` summary, so the per-scene manifest and review
card name the exact Replay value track and whether it has signal and marker
tracks without serializing Python callables.
Direct single-scene captures for maintained rigid workflow rows also write a
`workflow_guidance` manifest block with the row number, role, workflow phase,
focus axis, user question, try-first action, inspect signals, healthy signal,
and scope note, so a single row capture remains self-describing outside a
workflow packet.
Optional related-evidence, direct Rigid IPC shelf, and capture-first packet
rows use the same manifest/review-index metadata fields, so extended packets
also explain the row's role, user question, first action, inspect signals,
healthy signal, and scope instead of only exposing a workflow group label.
The manifest records `guidance_complete`, `guidance_missing_count`, and
`guidance_missing_rows`, and the review index mirrors that state with a
guidance badge plus a warning block for any selected row missing required
self-description fields. Successful scene captures also promote a
machine-readable `resolved_solver_identity` block from
`scene_metrics.latest.metrics` into each per-scene manifest, and workflow
manifests record `resolved_solver_identity_complete`,
`resolved_solver_identity_count`, and missing-row details so DART 7 review
packets can prove which solver/contact/executor configuration actually ran.
The identity contract now requires both a solver-family field and a context
field such as executor, contact method, same-solver marker, or held-fixed
configuration. Workflow manifests also record `scene_metrics_complete`,
`scene_metrics_count`, and missing-row details so a screenshot packet cannot
look green while a captured row lacks runtime metrics. Non-dry-run workflow
packets return failure status if any captured row misses scene metrics or
resolved solver identity evidence, and `review_index.html` highlights those
rows alongside missing guidance or failed-row summaries. Review cards also
promote nested step-diagnostics lane metrics into a compact
backend-diagnostics line, so backend activity, accelerated-stage counts,
worker counts, top stages, and stage timings are visible without opening the
raw manifest JSON. Contact-query and collision-cast cards also promote
selected contact/depth, query-filter, ignored-pair, ray hit, swept-probe,
time-of-impact, and cast-margin values into `latest signals`, so reviewers can
scan the query rows without opening raw manifests.
The baseline, free-flight, body-mode, frame-hierarchy, external-load, and link
point-load cards also promote their defining physical signals into
`latest signals`, so the static packet shows first-screen speed/height/energy,
contact-free path/momentum residuals, mode semantics, transform residuals,
acceleration scaling, and lever-arm yaw response before a reviewer opens
per-row manifests.
The filter prioritizes row ids, scene
ids, labels, questions, workflow phases, focus axes, positive signals, and
explicit aliases such as
`RigidBodySolver`, `SI`, `boxed LCP`, `ContactSolverMethod`,
`contact solver policy`, `worker count`, `accelerated backend`,
`compute backend`, `CUDA`, `GPU`, `CUDA backend`, `GPU backend`,
`backend comparison`, `Taskflow executor`, `executor comparison`,
`semi-implicit`, `variational solver`, `joint damping`, `joint friction`,
`throughput`, `latency`, `raycast`, `swept sphere`, `friction coefficient`,
`coefficient of restitution`, `time-of-impact`, `CollisionQueryOptions`,
`generalized force`, `coriolis`, `compute_impulse_response`,
`RigidBody.apply_linear_impulse`, `stack jitter`, `closed chain`, `closed loop`,
`direct rigid body impulse`, `sleep wake`, `island activation`, and
`loop closure compliance` before scope caveats, so intent searches such as
`contact`, `solver`, `step profile`, `backend comparison`, `GPU backend`,
`variational solver`, `joint damping`, `raycast`, `friction coefficient`,
`CollisionQueryOptions`, `generalized force`, `compute_impulse_response`,
`closed chain`, `executor comparison`, or `sequential impulse` surface the
relevant debugging rows instead of early rows that only say what not to infer.
The search folds punctuation, underscores, hyphens, dotted API names,
CamelCase, compact API tokens, and simple plurals, so
`RigidBody.applyLinearImpulse`, `Multibody.computeImpulseResponse`, `ray-cast`,
`shape-cast`, `body kind filters`, and `resting contacts` route like the
documented snake_case or singular terms.
Search-result tooltips name the match source, such as maintained alias, row
number, user question, workflow phase, focus axis, related evidence, or scope
caveat, so the navigator is auditable before the user switches scenes.
Deferred public-API searches such as `sleep wake`, `island activation`, and
`loop closure compliance` additionally show a deferred API caveat in the
tooltip, guide payload, workflow manifest, and static review card before
opening or reviewing the closest current verifier row. The workflow manifest
also publishes a `deferred_api_caveat_summary`, and the review index shows a
top-level `Deferred API Caveats` table and badge so unsupported API routes are
visible during packet triage.
Backend-status terms route to
`rigid_step_diagnostics`, while executor terms route to the same-solver
`rigid_executor_equivalence` row. Direct impulse queries route to the public
`rigid_external_loads` row, multibody impulse-response API queries route to the
generalized dynamics row, while deferred
activation and compliance queries route to the closest current row while
preserving the row's caveat, so the workflow remains searchable without
claiming unsupported activation or compliance behavior.

| Order | Scene id                         | User question                                                            | Solver(s)                        | Controls and diagnostics                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Capture command                                                                                                      | Automated evidence                                                                                                                                                                                                                                                                                                                 | Known limitation                                                                                         |
| ----- | -------------------------------- | ------------------------------------------------------------------------ | -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| 1     | `rigid_body`                     | What is the baseline DART 7 World rigid-body path?                       | Selectable rigid solver          | Solver, friction, restitution, reset, force drag, max speed, min height, kinetic energy, contact count, step profile timing, replay snapshots for baseline controls, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                                             | `pixi run py-demo-capture -- --scene rigid_body --frames 180 --width 960 --height 540 --show-ui`                     | `test_rigid_body_baseline_reports_restartable_first_run_diagnostics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                     | Baseline front door; focused edge cases stay in the specialized verifier rows.                           |
| 2     | `rigid_body_modes`               | Which body mode should I choose?                                         | Selectable rigid solver          | Rigid-body mode-semantics comparison axis, held-fixed solver/executor/gravity/force/body-mass/time-step context, kinematic speed, dynamic height/x, static drift, kinematic path error, mode flags, force norm, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                     | `pixi run py-demo-capture -- --scene rigid_body_modes --frames 72 --width 960 --height 540 --show-ui`                | `test_rigid_body_modes_compare_dynamic_static_kinematic_semantics`, comparison-axis panel/capture coverage, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                               | Contact-free mode semantics row; no sleep/wake or island activation API claim.                           |
| 3     | `rigid_free_flight`              | Do initial velocity, gravity, and spin evolve?                           | Sequential impulse               | Executor, launch speed, launch angle, gravity scale, spin speed, spin inertia ratio, path error, momentum residual, energy drift, spin ratios, contact count, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                                       | `pixi run py-demo-capture -- --scene rigid_free_flight --frames 96 --width 960 --height 540 --show-ui`               | `test_rigid_free_flight_preserves_initial_state_diagnostics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                             | No-contact initial-state row; not a load, restitution, contact, or solver row.                           |
| 4     | `rigid_frame_hierarchy`          | Where is a sensor/tool frame on a moving body?                           | World frame hierarchy            | Executor, body yaw speed, path radius, local offset/yaw, parent name, body/sensor world pose, world-transform residual, relative-transform residual, orientation error, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                             | `pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 --width 960 --height 540 --show-ui`           | `test_rigid_frame_hierarchy_tracks_body_fixed_frame`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                                     | Kinematics/frame row only; not a force, contact, sensor model, or solver-family claim.                   |
| 5     | `rigid_external_loads`           | How do external loads and direct impulses move and spin bodies?          | Sequential impulse               | Executor, force magnitude, torque magnitude, direct linear impulse, direct angular impulse, heavy mass ratio, high inertia ratio, speed, acceleration versus expected, direct impulse momentum, angular response, static drift, step profile timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                             | `pixi run py-demo-capture -- --scene rigid_external_loads --frames 72 --width 960 --height 540 --show-ui`            | `test_rigid_external_loads_scale_force_and_torque_response`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                              | Contact-free zero-gravity load and direct rigid-body impulse row; no contact or solver-family claim.     |
| 6     | `rigid_link_point_loads`         | Do point forces create lever-arm torque?                                 | Sequential impulse               | Executor, force magnitude, point offset, yawed frame angle, centered/world-point/pulse/double/world-frame/local-frame lanes, acceleration versus expected, yaw acceleration, displacement, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                          | `pixi run py-demo-capture -- --scene rigid_link_point_loads --frames 72 --width 960 --height 540 --show-ui`          | `test_rigid_link_point_loads_show_lever_arm_and_frame_semantics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                         | Contact-free one-shot Link.apply_force row; not persistent rigid-body accumulator behavior.              |
| 7     | `rigid_timestep_sensitivity`     | How does time-step size change free fall/contact?                        | Selectable rigid solver          | Solver, executor, base time step, gravity scale, matched fine/medium/coarse lanes, free-fall error, contact timing, clearance, error ratio, step-profile timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                                                 | `pixi run py-demo-capture -- --scene rigid_timestep_sensitivity --frames 96 --width 960 --height 540 --show-ui`      | `test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                     | Parameter-sensitivity row; not a solver correctness proof or exact contact threshold.                    |
| 8     | `rigid_step_diagnostics`         | Where does a World step spend time and memory?                           | Selectable rigid solver          | Solver, executor, single/contact/stack lanes, profile stage count, wall/stage time, top stage domain/acceleration/backend status, worker count, ECS counters, contact count, frame-scratch usage, overflow/reset counters, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                       | `pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 72 --width 960 --height 540 --show-ui`          | `test_rigid_step_diagnostics_reports_profile_and_memory_counters`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                        | Profiling may be compiled out; memory/contact diagnostics remain visible.                                |
| 9     | `rigid_contact_scale_budget`     | How much contact fits in my frame budget?                                | Selectable rigid solver          | Solver, executor, frame budget, contact friction, one/four/nine-box contact workloads, wall time, per-contact cost, contact-point count, frame-scratch peak, entity/component counts, dense/single ratio, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                        | `pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 72 --width 960 --height 540 --show-ui`      | `test_rigid_contact_scale_budget_orders_contact_loads`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                                   | Bounded live-GUI budget row; not a benchmark suite or heavy IPC stress packet.                           |
| 10    | `rigid_restitution_ladder`       | How does restitution change bounce height?                               | Selectable rigid solver          | Solver, executor, launch height, restitution scale, height, vertical velocity, contact count, rebound height, energy trend, step profile timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | `pixi run py-demo-capture -- --scene rigid_restitution_ladder --frames 96 --width 960 --height 540 --show-ui`        | `test_rigid_restitution_ladder_orders_rebound_height`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                                    | Relative rebound diagnostic only; energy plots are not exact conservation claims.                        |
| 11    | `rigid_material_mixing`          | Which material owns bounce or friction response?                         | Sequential impulse               | Executor, impact speed, tangential speed, low/high restitution, low/high friction, swapped body/surface lanes, expected max restitution, geometric-mean friction, rebound, speed loss, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                              | `pixi run py-demo-capture -- --scene rigid_material_mixing --frames 72 --width 960 --height 540 --show-ui`           | `test_rigid_material_mixing_applies_pair_rules`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                                          | Pair-rule ownership row only; not an IPC restitution claim or incline stick/slip proof.                  |
| 12    | `rigid_contact_inspector`        | Which contact pairs and manifold fields exist?                           | Collision query                  | Shape pair, penetration, sphere/box, box/ground, plane/sphere, capsule/sphere, cylinder/sphere, mesh/sphere, compound/sphere, selected/total contacts, point, normal, depth, local points, shape indices, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                        | `pixi run py-demo-capture -- --scene rigid_contact_inspector --frames 24 --width 960 --height 540 --show-ui`         | `test_rigid_contact_inspector_reports_contact_manifolds`, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                                                                   | Query-focused row; not a solver-family benchmark or all-pairs native sandbox.                            |
| 13    | `rigid_collision_query_options`  | Which body-kind pairs does a query include?                              | Collision query options          | Rigid/rigid, rigid/link, same-multibody link/link, cross-multibody link/link lanes; include toggles, ignored-pair selector, baseline/option/active/filtered contacts, lane status, public `CollisionBody` kind/cast diagnostics, shape indices, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                  | `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui`   | `test_rigid_collision_query_options_filter_body_kinds`, `test_simulation_collision_query_can_ignore_specific_pairs`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                      | Query-filter and persistent ignored-pair row only; not a shape-family manifold inspector.                |
| 14    | `rigid_collision_casts`          | Where do rays and swept probes hit?                                      | Collision cast queries           | Ray lateral offset, all-hit toggle, swept-sphere radius, swept-capsule offset/radius/height, nearest/all ray targets, ray fractions, sphere/capsule time of impact, cast margins, first hit point/normal, hit-count histories, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                   | `pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui`           | `test_rigid_collision_casts_report_nearest_all_and_swept_hits`, `test_continuous_capsule_cast_result`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                    | Public collision-cast query row; not a contact-solver, no-tunneling, or CCD time-step guarantee.         |
| 15    | `rigid_solver_compare`           | How do the rigid method families differ visually?                        | Sequential impulse and IPC       | Comparison-axis label for rigid-body solver family; shared executor, launch speed, friction, restitution, speed, wall clearance, position divergence, step profile timing, solver-pair/case-order capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                    | `pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 --width 960 --height 540 --show-ui`            | `test_rigid_solver_compare_records_wall_response`, comparison-axis panel/capture coverage, Replay timeline coverage, registry/category ordering, capture metrics, and visual smoke capture.                                                                                                                                        | Generic thin-wall comparison; not the sole no-tunneling proof.                                           |
| 16    | `rigid_executor_equivalence`     | Does a parallel executor preserve the same physics?                      | Same solver in both worlds       | Comparison-axis label for executor-only equivalence; physics solver, launch speed, friction, restitution, pose divergence, velocity divergence, contact-count delta, per-executor step timing, fallback executor label, selected-solver capture metrics, and Replay timeline signal.                                                                                                                                                                                                                                                                                                                                                  | `pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 --width 960 --height 540 --show-ui`      | `test_rigid_executor_equivalence_keeps_parallel_rollout_matched`, comparison-axis panel/capture coverage, Replay timeline coverage, panel/category coverage, capture metrics, and visual smoke capture inspected.                                                                                                                  | Same-solver executor-equivalence row; not a solver-family comparison.                                    |
| 17    | `rigid_contact_solver_compare`   | What changes when contact solver policy changes?                         | Contact solver policies          | Comparison-axis label for contact solver method; executor, launch speed, friction, restitution, initial tilt, contact count, penetration depth, analytic corner clearance, speed, energy, divergence, step profile timing, contact-policy-pair capture metrics.                                                                                                                                                                                                                                                                                                                                                                       | `pixi run py-demo-capture -- --scene rigid_contact_solver_compare --frames 72 --width 960 --height 540 --show-ui`    | `test_rigid_contact_solver_compare_records_coupled_contact_policy`, comparison-axis panel/capture coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                       | Contact-policy row only; it does not compare IPC against sequential impulse.                             |
| 18    | `contact`                        | Do articulated links contact like rigid bodies?                          | World multibody link contact     | Executor, ground friction, ground restitution, drop height, slide speed, push speed, link drop/slide/pusher lanes, link/rigid contact counts, rebound, slide travel, target travel, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                 | `pixi run py-demo-capture -- --scene contact --frames 144 --width 960 --height 540 --show-ui`                        | `test_rigid_link_contact_exercises_multibody_contact_response`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                           | Multibody-link contact row only; not a contact-impulse or compliance inspector.                          |
| 19    | `rigid_friction_threshold`       | Where is the inclined-ramp stick/slip boundary?                          | IPC                              | Friction-threshold comparison axis, held-fixed IPC/executor/time step, ramp angle, controlled friction, threshold, lane drift, lane speed, clearance, step profile timing, diagnostic plots, and capture metrics.                                                                                                                                                                                                                                                                                                                                                                                                                     | `pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui`        | `test_rigid_friction_threshold_separates_stick_and_slip_lanes`, panel/category coverage, capture metrics, and visual smoke capture inspected.                                                                                                                                                                                      | Near-threshold behavior is tunable visual evidence, not an exact proof point.                            |
| 20    | `rigid_spin_roll_coupling`       | How does contact friction couple sliding and spin?                       | Sequential impulse               | Spin/roll initial-condition comparison axis, held-fixed sequential-impulse/executor/time step, contact friction, launch speed, backspin ratio, matched rolling/sliding/backspin/low-friction lanes, contact slip, roll ratio, spin change, travel, energy, contact count, step timing, and capture metrics.                                                                                                                                                                                                                                                                                                                           | `pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui`        | `test_rigid_spin_roll_coupling_converts_slip_to_roll`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                                                    | Spin/rolling visual diagnostic only; no public rolling-resistance or torsional-friction parameter claim. |
| 21    | `rigid_stack_stability`          | Does a top-heavy mass-ratio stack stay ordered?                          | Sequential impulse and IPC       | Rigid-body solver-family comparison axis, held-fixed executor/top mass/friction/time step, solver pair, max speed, top drift, analytic clearance/overlap, height error, top-x divergence, step profile, Replay timeline markers for instability frames, and capture metrics.                                                                                                                                                                                                                                                                                                                                                          | `pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui`           | `test_rigid_stack_stability_keeps_ipc_stack_ordered`, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture inspected.                                                                                                                                                   | Compact two-box stack only; taller IPC stacks remain benchmark/capture-first.                            |
| 22    | `rigid_contact_manipulation`     | Can a rigid pusher move an object through contact?                       | Sequential impulse and IPC       | Rigid-pusher contact-response comparison axis, held-fixed executor/table-goal/target-mass/time-step context, pusher launch speed, table friction, pusher mass, solver pair, target travel, pusher-target gap, contact/proximity, lateral drift, goal error, travel divergence, step profile timing, Replay timeline markers for contact/proximity/progress frames, and capture metrics.                                                                                                                                                                                                                                               | `pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui`      | `test_rigid_contact_manipulation_pushes_target_toward_goal`, comparison-axis panel/capture coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                              | Task-like pusher row only; not a full arm/gripper manipulation controller.                               |
| 23    | `rigid_kinematic_driver`         | Does prescribed motion carry objects by contact?                         | IPC plus SI caveat               | Prescribed tangential contact-response comparison axis, held-fixed executor/kinematic-support/box-mass/time-step context, driver speed, grip friction, IPC grip/slip lanes, sequential-impulse caveat lane, solver/case pair, driver travel, box travel, slip, speed ratio, support gap, contact evidence, step timing, Replay timeline markers for contact/carry/slip/caveat frames, and capture metrics.                                                                                                                                                                                                                            | `pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui`          | `test_rigid_kinematic_driver_carries_box_with_ipc`, comparison-axis panel/capture coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                                       | Tangential kinematic-driver row only; normal pushing is routed to the next row.                          |
| 24    | `rigid_kinematic_normal_push`    | Can prescribed normal motion push a target?                              | IPC plus SI comparison           | Prescribed normal-contact comparison axis, held-fixed executor/kinematic-paddle/zero-friction/time-step context, push speed, target mass, IPC normal/heavy lanes, sequential-impulse lane, driver travel, target travel, target-travel divergence, analytic gap, penetration depth, contact count, speed ratio, step timing, Replay timeline markers for contact/penetration/push/divergence frames, and capture metrics.                                                                                                                                                                                                             | `pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui`     | `test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat`, comparison-axis panel/capture coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                          | Normal kinematic-pusher caveat row only; IPC exposes penetration rather than a robust manipulation path. |
| 25    | `rigid_fixed_joint`              | Does a fixed joint preserve its captured pose?                           | Sequential rigid joints          | Fixed-relative-transform recovery comparison axis, held-fixed sequential-joint/static-base/payload-mass/captured-offset/time-step context, perturb/reset controls, relative offset error, relative orientation error, payload speed, angular speed, histories, and Replay timeline markers for pose-error/recovery frames.                                                                                                                                                                                                                                                                                                            | `pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui`               | `test_rigid_fixed_joint_verifier_restores_captured_transform`, comparison-axis panel/capture coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                                            | Fixed-pose row only; no motor, limit, or break-force claims.                                             |
| 26    | `rigid_joint_breakage`           | What happens when a fixed joint breaks?                                  | AVBD rigid joints                | Fixed break-force lifecycle comparison axis, held-fixed AVBD/static-base/payload-mass/captured-offset/time-step context, editable log-scale break-force threshold, broken/intact state, connector color, captured-offset error, payload speed, payload release distance, broken-state history, resettable breakage lifecycle, and Replay timeline markers for broken/released frames.                                                                                                                                                                                                                                                 | `pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui`            | `test_rigid_joint_breakage_marks_and_resets_breakage`, `test_rigid_joint_breakage_panel_edits_break_force_threshold`, comparison-axis panel/capture coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                     | AVBD-pinned editable-threshold breakage row; no sequential-impulse or IPC break-force parity claim.      |
| 27    | `rigid_distance_spring`          | How do rigid-body distance springs enforce rest length?                  | Sequential rigid AVBD rows       | Distance-spring response-family comparison axis, held-fixed executor/rest-length/payload-mass/time-step context, editable rest length, soft/stiff/off-center stiffness controls, initial stretch, gravity scale, unsprung/soft/stiff/off-center lanes, lane order, current length, stretch, absolute stretch, payload speed, angular speed, connector visuals, top-level latest signals for each lane's absolute stretch plus offset spin, step timing, and Replay timeline markers for high-stretch/off-center-spin frames.                                                                                                          | `pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui`           | `test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor`, `test_rigid_distance_spring_panel_edits_public_spring_parameters`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture. | World rigid-body distance-spring row only; IPC and multibody worlds reject this public API.              |
| 28    | `rigid_limited_joints`           | Do one-DOF joints keep only their free axis?                             | Sequential rigid joints          | One-DOF joint constraint-family comparison axis, held-fixed sequential-joint/static-base/payload-mass/z-axis/time-step context, perturb/reset controls, hinge and slider lane names, hinge radius/z error, hinge yaw, hinge angular speed, slider orthogonal error, slider travel, slider axis speed, histories, top-level latest signals for locked-axis errors and free-axis motion, and Replay timeline markers for locked-error/free-axis-motion frames.                                                                                                                                                                          | `pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui`            | `test_rigid_one_dof_joint_verifier_preserves_locked_directions`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                         | Revolute/prismatic constraint row only; public motor/limit behavior is out of scope.                     |
| 29    | `rigid_joint_motor_limits`       | Do joint motors and limits clamp commands?                               | World multibody joints           | World multibody actuator/limit comparison axis, held-fixed x-axis prismatic-rail/y-axis revolute-stop/carriage-mass/time-step context, command speed, velocity limit, position stop, requested force, effort cap, velocity/position/force lane names, motor speed, expected speed, speed error, limit angle/error, acceleration gap, force travel gap, step timing, histories, top-level latest signals for motor clamp/limit/force-gap behavior, and Replay timeline markers for velocity-clamp/position-stop/effort-cap frames.                                                                                                     | `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 --width 960 --height 540 --show-ui`        | `test_rigid_joint_motor_limits_clamp_commands_and_effort`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                               | Multibody joint-actuator row; rigid-body joint motor behavior is not claimed.                            |
| 30    | `rigid_joint_passive_parameters` | Do passive joint parameters shape motion?                                | World multibody joints           | Passive joint-parameter comparison axis, held-fixed World-prismatic/gravity-off/contact-free/link-mass/time-step context, executor, spring stiffness, rest position, damping, Coulomb friction, separate hold/slip/armature drive forces, armature, lane names, acceleration versus expected, spring/damped energy and ratio, stiction/slip status, slip speed, armature acceleration gap, armature position gap, step timing, top-level latest signals for energy/slip/armature behavior, and Replay timeline markers for damping/slip/armature-lag frames.                                                                          | `pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui` | `test_rigid_joint_passive_parameters_order_passive_response`, `test_rigid_joint_passive_parameters_panel_edits_drive_forces`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.            | Contact-free passive-parameter row; no motor, limit, or contact-load claims.                             |
| 31    | `rigid_screw_joint_pitch`        | Does screw pitch couple rotation and translation?                        | World multibody joints           | Screw pitch coupling comparison axis, held-fixed contact-free World screw-joint/z-axis/moving-mass/axial-inertia/time-step context, executor, pitch scale, gravity scale, zero/fine/coarse/reverse pitch lanes, lane names, angle, axial travel, coarse/fine travel gap, pitch ratios, effective mass, expected-versus-actual acceleration, top-level latest signals for zero-pitch contrast, pitch multipliers, travel gap, reverse-sign angle, and acceleration residual, and Replay timeline markers for pitch-spread/zero-pitch/reverse-sign frames.                                                                              | `pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui`         | `test_rigid_screw_joint_pitch_couples_rotation_and_translation`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                         | Contact-free screw-pitch row; no contact, motor, limit, or loop-closure claims.                          |
| 32    | `rigid_multibody_dynamics_terms` | What do generalized dynamics terms mean?                                 | World multibody dynamics         | Joint-space dynamics term-family comparison axis, held-fixed contact-free World dynamics/fixed-base/revolute-link/target-acceleration/impulse/gravity/time-step context, executor, target acceleration, joint impulse, distal mass scale, gravity scale, single-hinge/coupled/heavy lanes, mass matrix diagonal/coupling/conditioning, inverse-dynamics residual, impulse residual, torque norm, response norm, top-level latest signals for scalar mass, coupling, heavy-load torque gap, coupled/heavy response gap, response ratio, and residual checks, plus Replay timeline markers for response gap/coupling/heavy-load frames. | `pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui`  | `test_rigid_multibody_dynamics_terms_expose_generalized_terms`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                          | Contact-free joint-space dynamics row; not a Cartesian point-force or COM-Jacobian claim.                |
| 33    | `rigid_link_center_of_mass`      | How does a link center-of-mass offset change inertia and gravity torque? | World multibody inertial offsets | Link center-of-mass offset-family comparison axis, held-fixed contact-free World revolute-link/fixed-visual-geometry/link-mass/gravity/time-step context, executor, COM offset, gravity scale, link mass, high-inertia multiplier, centered/+X/-X/high-inertia lanes, gravity torque, mass matrix, hinge acceleration, expected acceleration, COM marker position, energy, top-level latest signals for centered/positive/negative gravity torque, mirrored angle sum, high-inertia mass and acceleration ratios, acceleration residual, and Replay timeline markers for mirrored-angle/centered/high-inertia-lag frames.             | `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui`       | `test_rigid_link_center_of_mass_offsets_gravity_torque`, comparison-axis panel/capture coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                 | Link inertial-offset row only; not arbitrary-point Jacobians, contact, IK, or operational-space control. |
| 34    | `rigid_link_jacobian`            | What does a link Jacobian map?                                           | World multibody kinematics       | Link-origin Jacobian mapping comparison axis, held-fixed contact-free World two-revolute-link/time-step/finite-difference context, motion speed, elbow phase, wrench controls, Jacobian term family, link-origin world/body Jacobian gap, `J qdot` twist, finite-difference velocity error, `J.T wrench` torque, power error, top-level latest signals for link speed, Jacobian gap, torques, and residuals, Replay timeline markers for high-twist/wrench-load/world-body-gap/residual-alert frames, and capture metrics.                                                                                                            | `pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui`             | `test_rigid_link_jacobian_maps_link_origin_twist_and_wrench`, comparison-axis panel coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                    | Link-origin kinematic/wrench map only; not arbitrary point, COM, contact, IK, or OSC.                    |
| 35    | `rigid_multibody_solver_family`  | Which multibody integration family supports solves?                      | Multibody solver families        | Multibody integration solve-policy comparison axis, held-fixed contact-free World point-closure/three-revolute-link/gravity/time-step context, executor, gravity scale, semi-implicit residual-only, variational residual-only, variational solved lanes, closure residual, tip error, tip height, joint speed, residual solve ratio, top-level latest signals for residual-only residual, solved residual, residual solve ratio, lane residuals, solved tip error, and max step time, Replay timeline markers for solve-advantage/residual-drift/solved-tight frames, step timing, and capture metrics.                              | `pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui`   | `test_rigid_multibody_solver_family_routes_solved_closures`, comparison-axis panel coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                     | Solver-family routing row; closure family selection remains in the next row.                             |
| 36    | `rigid_loop_closure`             | Which loop-closure family should I use?                                  | Variational rigid multibody      | Loop-closure family/policy comparison axis, held-fixed contact-free variational rigid multibody/four-revolute-link/gravity/time-step context, executor, gravity scale, POINT/DISTANCE/RIGID family lanes, residual-only versus solved policies, closure residual, tip error, distance error, orientation residual, residual ratio, joint speed, top-level latest signals for family residual ratios, distance-family distance/tip error, RIGID orientation error, and max step time, Replay timeline markers for solve-advantage/family-drift/distance-tip/rigid-orientation frames, step timing, and capture metrics.                | `pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui`              | `test_rigid_loop_closure_compares_closure_families`, comparison-axis panel coverage, latest-signal ordering coverage, Replay timeline coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                             | Public-family comparison row; not a compliance, breakage, or distance-family solver sweep.               |

## No-Tunneling Decision

Do not add a second thin-wall catalog row in this slice. The current user path
is:

1. `rigid_contact_inspector` for raw contact-pair observability;
2. `rigid_collision_query_options` for `World.collide(options)` body-kind
   filtering and persistent ignored pairs;
3. `rigid_collision_casts` for public raycast, swept-sphere, and swept-capsule
   hit queries;
4. `rigid_solver_compare` for broad method-family differences on a wall scene;
5. the Rigid IPC shelf's `rigid_ipc_tunnel` scene for the focused IPC
   no-tunneling capability check;
6. `rigid_friction_threshold`, `rigid_spin_roll_coupling`, and
   `rigid_stack_stability` for the next common contact failure modes.

Revisit this only if a future high-speed preset or regression asks a distinct
user question that neither `rigid_solver_compare` nor `rigid_ipc_tunnel`
answers.

`rigid_ipc_tunnel` remains an existing Rigid IPC capability scene rather than a
numbered World Rigid Body workflow row. Capture it directly when the user wants
the focused IPC no-tunneling view. Its scene-owned metrics report the fast
box's wall clearance, through-wall margin, velocity, contact count, step
timing, and barrier-held status so the related route is not screenshot-only
evidence:

```bash
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui
```

## Differentiable Contact-Gradient Route

Do not add duplicate numbered rigid rows for contact-gradient modes in this
slice. Keep the backward-pass diagnostics in the Differentiable shelf so the
mode distinction is explicit, but route rigid users there from the workflow
docs:

- `diff_drone_liftoff` answers "why does my contact optimization stall even
  though the collision response looks right?" It uses a rigid `World` with
  boxed-LCP contact and shows `ContactGradientMode.ANALYTIC` stalling at the
  clamping saddle while `ContactGradientMode.COMPLEMENTARITY_AWARE` escapes.
  Its scene-owned capture metrics report target/rest height, analytic versus
  complementarity-aware thrust/final-height/loss values, height and
  target-error gaps, fallback status, and compact history summaries.
- `diff_pre_contact_surrogate` answers "why is there no contact gradient before
  the objects touch?" It uses an approaching-but-not-touching rigid sphere and
  shows that `ContactGradientMode.ANALYTIC` remains contact-free while
  `ContactGradientMode.PRE_CONTACT_SURROGATE` adds a backward-only pre-contact
  Jacobian block without changing the forward step. Its scene-owned capture
  metrics report pre/post contact counts, forward-state parity,
  analytic-freefall error, surrogate block magnitude, vertical sensitivity, and
  fallback status.

Default builds with `DART_BUILD_DIFF=OFF` record finite fallback payloads;
diff-enabled builds should show the aware-mode saddle escape and the
pre-contact surrogate Jacobian block.

```bash
pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_pre_contact_surrogate --frames 24 --width 960 --height 540 --show-ui
```

## Related Evidence Routes

The in-viewer `Rigid Workflow` panel may route a numbered rigid row to a
non-numbered shelf only through this table. These scenes remain outside the
36-row World Rigid Body sequence.

| Source row                       | Related scene                          | Shelf                       | Panel label                                                                                                      | Scope note                                                                                                               |
| -------------------------------- | -------------------------------------- | --------------------------- | ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| `rigid_free_flight`              | `floating_base`                        | World Rigid Body            | Related shelf: World Rigid Body / floating_base - broader floating-joint row                                     | Broader floating-joint SE(3) drift/spin example; use the numbered row for baseline rigid-body initial-state diagnostics. |
| `rigid_multibody_dynamics_terms` | `articulated`                          | World Rigid Body            | Related shelf: World Rigid Body / articulated - broader two-link arm row                                         | Broader two-link arm example; use the numbered row for mass, inverse-dynamics, and impulse-response diagnostics.         |
| `rigid_solver_compare`           | `rigid_ipc_tunnel`                     | Rigid IPC                   | Related shelf: Rigid IPC / rigid_ipc_tunnel - focused no-tunneling view                                          | Focused IPC capability scene; not a broad solver comparison or general proof.                                            |
| `rigid_solver_compare`           | `rigid_ipc_edge_drop`                  | Rigid IPC                   | Related shelf: Rigid IPC / rigid_ipc_edge_drop - degenerate edge-contact view                                    | Focused IPC degenerate edge-contact capability scene; not a broad solver comparison or contact-manifold inspector.       |
| `rigid_contact_solver_compare`   | `diff_drone_liftoff`                   | Differentiable              | Related shelf: Differentiable / diff_drone_liftoff - contact-gradient route                                      | Analytic vs complementarity-aware clamping-contact optimization; not a solver row.                                       |
| `rigid_contact_solver_compare`   | `diff_pre_contact_surrogate`           | Differentiable              | Related shelf: Differentiable / diff_pre_contact_surrogate - pre-contact gradient route                          | Analytic vs pre-contact surrogate backward-only gradient for an approaching but not touching body; not a solver row.     |
| `contact`                        | `avbd_rigid_fixed_joint_contact`       | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_fixed_joint_contact - fixed-joint contact route          | Variational fixed-joint/contact capability scene; not a World contact-policy comparison.                                 |
| `rigid_joint_breakage`           | `avbd_rigid_breakable_joint`           | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_breakable_joint - free-rigid fixed break/reset           | Dedicated free-rigid fixed break/reset row; not sequential-impulse or IPC parity evidence.                               |
| `rigid_joint_breakage`           | `avbd_rigid_spherical_breakable_joint` | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_spherical_breakable_joint - spherical anchor break/reset | Dedicated free-rigid spherical anchor break/reset row; orientation remains intentionally free.                           |
| `rigid_joint_motor_limits`       | `avbd_rigid_revolute_motor`            | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_revolute_motor - free-rigid hinge motor                  | AVBD free-rigid revolute velocity motor; not a World multibody motor/limit comparison.                                   |
| `rigid_joint_motor_limits`       | `avbd_rigid_prismatic_motor`           | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_prismatic_motor - free-rigid slider motor                | AVBD free-rigid prismatic velocity motor; not a World multibody motor/limit comparison.                                  |

Capture every non-numbered related-evidence route with the docked UI visible:

These commands are also included after the numbered rows by
`py-demo-capture -- --rigid-workflow --include-related`.

```bash
pixi run py-demo-capture -- --scene floating_base --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene articulated --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_edge_drop --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_pre_contact_surrogate --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_fixed_joint_contact --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_breakable_joint --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_spherical_breakable_joint --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_revolute_motor --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_prismatic_motor --frames 72 --width 960 --height 540 --show-ui
```

## Capture-First Rigid IPC Packets

These packets are registered in non-numbered shelves only. They give users a
GUI/capture route for stress cases that currently exceed the default live
workflow budget, and they point to benchmark evidence instead of making
solver-performance parity claims.

| Scene id                       | User question                                                                             | Signals                                                                                                                                                    | Capture command                                                                                                 | Scope note                                                                                                 |
| ------------------------------ | ----------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `rigid_ipc_stack_packet`       | Can a four-box IPC stack stay separated, ordered, and finite beyond the live demo budget? | Friction, box count, frame-budget threshold, min clearance, contact count, top drift, height error, max speed, wall time, and benchmark pointer.           | pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui       | Capture-first stress packet; not a numbered workflow row and not a solver-performance parity claim.        |
| `rigid_ipc_heavy_stack_packet` | How does a taller, top-heavy IPC stack behave beyond the live demo budget?                | Friction, box count, top mass, frame-budget threshold, min clearance, contact count, top drift, height error, max speed, wall time, and benchmark pointer. | pixi run py-demo-capture -- --scene rigid_ipc_heavy_stack_packet --frames 12 --width 960 --height 540 --show-ui | Taller capture-first stress packet; not a numbered workflow row and not a solver-performance parity claim. |

Capture-first rigid IPC packets are also included after the numbered workflow,
optional related-evidence routes, and optional direct Rigid IPC shelf routes by
`py-demo-capture -- --rigid-workflow --include-packets`.

The matching benchmark owner remains `bm_rigid_ipc_solver`; use its
`BM_RigidWorldStep_SequentialImpulse` and `BM_RigidWorldStep_Ipc` rows for
same-scene per-step throughput tracking before promoting any heavier IPC stack
into the live workflow.
The capture helper also stores scene-owned metrics for rows and packets that
expose `SceneSetup.info["capture_metrics"]`; `rigid_step_diagnostics` and
`rigid_contact_scale_budget` summarize their profiling, memory, contact, and
budget metrics into `manifest.json` as first/latest events, per-key presence
counts, and top-level numeric ranges. The `contact` row uses the same hook for
multibody-link drop/slide/pusher lane summaries, the related World rows
`floating_base` and `articulated` mirror floating-joint drift/spin and two-link
arm speed/height/damping diagnostics, the direct Rigid IPC shelf rows
`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and `rigid_ipc_pile`
mirror basic barrier-settle, tangential slide, inclined slide, and multi-box
pile gap/speed/contact/timing evidence, `rigid_ipc_tunnel` mirrors
no-tunneling clearance, through-wall margin, velocity, contact, step-timing,
and barrier-held status, `rigid_ipc_edge_drop` mirrors degenerate edge-contact
barrier gap, tilt, angular speed, contact count, step timing, and edge-barrier
status, and `rigid_ipc_stack_packet` plus `rigid_ipc_heavy_stack_packet` mirror
clearance, contact, drift, height, wall-time, frame-budget, mass, and benchmark
values through the same schema. The
related `diff_drone_liftoff` route uses the schema for clamping-contact
gradient mode outcome, target/rest height, analytic versus aware
thrust/final-height/loss values, height/target-error gaps, fallback status, and
history summaries. The related `diff_pre_contact_surrogate` route uses the
schema for pre-contact counts, identical forward-state evidence,
analytic-freefall error, surrogate block magnitude, vertical sensitivity, and
fallback status. The related AVBD routes use the schema for fixed-joint
contact offset/clearance/contact counts, spherical breakage anchor/orientation
drift, free-rigid revolute motor speed tracking, and free-rigid prismatic motor
axis/drift tracking.

## Regenerating Visual Evidence

Run the capture commands in the workflow table from the repository root. The
helper prints the output directory, writes PNG frames, rejects blank captures,
and applies Linux software-Mesa defaults so the same command works on headless
dev hosts. Visual artifacts are intentionally not checked in.

Use the workflow mode when you want one command to plan or run every numbered
rigid verifier capture. It writes a top-level manifest that points at each
per-scene manifest plus `review_index.html`, a static contact sheet for scanning
all 36 screenshots and metric summaries from one page:

```bash
pixi run py-demo-capture -- --rigid-workflow --dry-run
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow
```

For long review packets, add `--continue-on-failure` when later rows should
still be captured after one row fails. The manifest records
`continue_on_failure=true` plus a `failed_rows` list, and
`review_index.html` shows a Failed Rows summary with workflow row-range rerun
commands that preserve the packet flags and absolute row numbering. The final
workflow status and process exit code still report failure if any selected row
failed.

```bash
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --continue-on-failure --output-dir /tmp/dart_capture_rigid_workflow_resilient
```

Add `--include-related` when the review packet should also capture the
non-numbered related evidence routes after the 36-row workflow:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related --output-dir /tmp/dart_capture_rigid_workflow_with_related
```

Add `--include-ipc-shelf` when the packet should also capture the direct
metric-backed Rigid IPC shelf scenes after the numbered rows and any related
evidence routes:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-ipc-shelf --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --output-dir /tmp/dart_capture_rigid_workflow_with_ipc_shelf
```

Add `--include-packets` when the packet should also include capture-first
rigid IPC evidence that is intentionally outside the live 36-row workflow.
Capture-first packets are appended after related evidence and direct Rigid IPC
shelf rows when those groups are requested:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-packets --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --output-dir /tmp/dart_capture_rigid_workflow_with_packets
```

Capture the direct Rigid IPC shelf routes with the docked UI visible:

These commands are also included after the numbered rows, and after related
evidence when present, by
`py-demo-capture -- --rigid-workflow --include-ipc-shelf`.

```bash
pixi run py-demo-capture -- --scene rigid_ipc --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_slide --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_incline --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 72 --width 960 --height 540 --show-ui
```

For targeted reruns after a failed or manually inspected row, keep the same
workflow packet but bound the row range. Row numbers stay absolute, so row 37
still writes under `scenes/37_<scene>` when related evidence is included. With
`--include-related --include-packets`, rows 48-49 are the two
capture-first stack packets. If `--include-ipc-shelf` is also requested, those
packet rows become 52-53.

```bash
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 17 --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 48 --workflow-end-row 49 --output-dir /tmp/dart_capture_rigid_workflow_packet_rerun
```

When a row range selects only part of a larger packet, the top-level manifest
keeps the requested packet shape in `include_related`, `include_ipc_shelf`, and
`include_packets`. The selected slice is reported separately through
`selected_include_related`, `selected_include_ipc_shelf`, and
`selected_include_packets`. The generated `review_index.html` header mirrors
that packet context with row-span, requested-groups, and selected-groups
badges.

For motion evidence, add `--video --fps 24` to the same docked capture path.
The helper encodes an MP4 when `ffmpeg` is available and still keeps the PNG
frames as inspectable evidence:

```bash
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 72 --width 960 --height 540 --show-ui --video --fps 24
```

The workflow packet path accepts the same motion flags and passes them to every
selected row. The generated review index links each per-row video artifact
beside the screenshot and frames directory when encoding succeeds:

```bash
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --video --fps 24 --output-dir /tmp/dart_capture_rigid_workflow_solver_motion
```

For a quick curated refresh:

```bash
pixi run py-demo-capture -- --scene rigid_body --frames 180 --width 960 --height 540 --show-ui
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
pixi run py-demo-capture -- --scene contact --frames 144 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui
```

## Validation Snapshot

Evidence recorded for this slice:

- Latest optional 53-row packet refresh:
  the current optional packet
  `build/captures/rigid_workflow_optional_signal_highlights_1781338541`
  selected rows 37-53 with related evidence, direct Rigid IPC shelf, and
  capture-first packet groups requested and selected. It reported
  `status=complete`, `capture_count=17`, `completed_count=17`,
  `failed_count=0`, `workflow_total_count=53`, `workflow_row_start=37`,
  `workflow_row_end=53`, `continue_on_failure=false`,
  `guidance_complete=true`, `scene_metrics_complete=true`,
  `scene_metrics_count=17`, `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=17`, and `failed_rows=[]`. Its
  `review_index.html` now promotes World related, focused Rigid IPC, direct IPC
  shelf, capture-first stack, differentiable contact-gradient, and AVBD related
  row health signals instead of only generic solver/executor/contact metadata.
  A read-only static review audit checked 86 local `href`/`src` targets with 0
  missing links. The optional latest-signal guard reported `9 passed`.

- Historical pre-surrogate packet refresh: after the workflow-command provenance and
  review-index link-normalization fix, the full numbered workflow packet
  `build/captures/rigid_workflow_rows_01_36_1781309127` completed rows 01-36
  with `status=complete`, `capture_count=36`, `completed_count=36`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`,
  `guidance_missing_count=0`, and `failed_rows=[]`. The optional extended
  packet `build/captures/rigid_workflow_optional_rows_37_52_1781309448`
  selected rows 37-52 with related evidence, direct Rigid IPC shelf, and
  capture-first packet groups requested; it reported `status=complete`,
  `capture_count=16`, `completed_count=16`, `failed_count=0`,
  `workflow_total_count=52`, `workflow_row_start=37`,
  `workflow_row_end=52`, `continue_on_failure=true`,
  `guidance_complete=true`, `guidance_missing_count=0`, and `failed_rows=[]`.
  The generated review indexes include top-level workflow commands and a
  read-only HTML asset audit found 0 missing local assets: 181/181 links for
  rows 01-36 and 81/81 links for rows 37-52. This packet is retained as
  historical pre-surrogate evidence only; the rows 37-53 packet above is the
  current optional maintainer-review artifact.

- Latest DART 7 harness alignment: `py-demo-capture` now writes validated
  `resolved_solver_identity` into successful per-scene manifests, requires
  solver-family plus context fields for workflow completeness, attaches
  identity and scene-metrics evidence to captured workflow rows, summarizes
  both identity and scene-metrics completeness in workflow manifests, and shows
  resolved-solver summaries or warning blocks in `review_index.html`.
  Focused guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_resolved_solver_identity_requires_solver_family_and_context python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_scene_metrics_are_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_solver_identity_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_can_continue_after_scene_failure python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row -q`
  reported `11 passed`. A full GUI packet regeneration attempt under
  `build/captures/rigid_workflow_rows_01_36_1781311276` failed at row 1 with
  `return_code=-7` before scene metrics were written; after the scoped
  headless engine-creation fix, the rows 01-36 packet and the current rows
  37-53 optional packet both regenerated successfully with explicit
  identity-complete and scene-metrics-complete fields.

- Latest backend-diagnostics review-card slice: workflow review cards now
  promote nested `rigid_step_diagnostics` lane profile data into a compact
  `backend diagnostics` line. Focused capture-helper guards reported
  `2 passed`, and the real row-8 packet
  `build/captures/rigid_workflow_backend_diagnostics_1781335057` completed
  with `status=complete`, `scene_metrics_complete=true`,
  `resolved_solver_identity_complete=true`, and a review card containing the
  backend/fallback/timing summary.

- Latest contact-query/cast review-card slice: workflow review cards now
  promote row 12-14 query metrics into `latest signals`. Focused
  capture-helper guards reported `2 passed`, and the real rows 12-14 packet
  `build/captures/rigid_workflow_query_signal_highlights_1781335507`
  completed with `status=complete`, `scene_metrics_complete=true`,
  `resolved_solver_identity_complete=true`, and review cards showing selected
  contact counts/depths, query-filter counts, ignored-pair state, ray hit
  fractions, swept-probe times of impact, and cast margins.

- Latest review-index provenance/link guard: the workflow manifest and
  `review_index.html` now record the exact top-level workflow packet command,
  and `_workflow_scene_manifest_summary` normalizes workspace-relative
  per-scene artifact paths before emitting screenshot/frame/video links.
  Focused capture-helper pytest reported `4 passed` for the workflow command,
  aggregate manifest, workspace-relative link-resolution, and video-link
  guards.

- Latest review-index UX guard: after fetching `origin/main` and confirming the
  feature branch was already up to date with the PR #2986 DART 7
  architecture/work-packet harness, the focused unit guard
  `python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests`
  reported `1 passed`. The guard asserts that generated workflow review cards
  render exactly one screenshot thumbnail for each captured row.

- Latest full numbered workflow packet evidence: after the DART 7 harness
  scene-metrics contract update, backend/executor search-routing follow-up,
  backend-diagnostics review-card summary, contact-query/collision-cast
  review-card summary, direct rigid-body impulse update, phase/focus static
  review summary, and deferred-API caveat summary, the current workflow packet
  `build/captures/rigid_workflow_rows_01_36_1781356342` captured rows 01-36
  with `status=complete`, `capture_count=36`, `completed_count=36`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`,
  `guidance_missing_count=0`, `scene_metrics_complete=true`,
  `scene_metrics_count=36`, `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=36`, and `failed_rows=[]`. The packet wrote
  `manifest.json`, `review_index.html`, 36 docked row screenshots, and 2544
  frame PNGs; a read-only review-index asset audit checked 181 local
  `href`/`src` targets with 0 missing links. The refreshed review index shows
  row-5 direct impulse signals, row-8 `backend diagnostics`, row 12-14
  contact-query/collision-cast `latest signals`, a `Workflow Phase Map`, and a
  top-level `Deferred API Caveats` table.

- Historical optional extended workflow packet evidence: the pre-surrogate workflow
  packet `build/captures/rigid_workflow_optional_rows_37_52_1781309448`
  selected rows 37-52 from the fully extended 52-row packet with related
  evidence, direct
  Rigid IPC shelf, and capture-first packet groups requested and selected. It
  reported `status=complete`, `capture_count=16`, `completed_count=16`,
  `failed_count=0`, `workflow_total_count=52`, `workflow_row_start=37`,
  `workflow_row_end=52`, `continue_on_failure=true`,
  `guidance_complete=true`, `guidance_missing_count=0`, and `failed_rows=[]`.
  The packet wrote `manifest.json`, `review_index.html`, 16 docked row
  screenshots, and 1004 frame PNGs; the first selected scene was
  `floating_base` and the last selected scene was
  `rigid_ipc_heavy_stack_packet`.

- Latest loop-closure family row evidence: the real workflow packet
  `build/captures/rigid_loop_closure_row_36_1781304923` captured row 36 with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and `guidance_complete=true`.
  The focused row/panel/docs-order/review-index pytest subset reported
  `6 passed`. Row 36 now exposes
  `loop_closure_family_policy_selection` as the comparison axis, held-fixed
  contact-free variational rigid multibody/four-link-chain/gravity/time-step
  context, closure family and policy lanes, Replay signal/markers, metric-key
  summary, and latest-signal ordering for POINT/DISTANCE/RIGID residual
  ratios, distance-family distance/tip error, RIGID orientation error, and
  solver. The per-scene capture wrote a nonblank docked screenshot with 2446
  unique colors and 71 PNG frames from the 72-frame workflow row capture.

- Latest multibody solver-family row evidence: the real workflow packet
  `build/captures/rigid_multibody_solver_family_row_35_1781304535` captured
  row 35 with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 35 now exposes
  `multibody_integration_solve_policy_family` as the comparison axis,
  held-fixed contact-free World point-closure/three-link-chain/gravity/time-step
  context, controls, lane order, Replay signal/markers, metric-key summary, and
  latest-signal ordering for residual-only residual, solved residual, residual
  solve ratio, lane residuals, solved tip error, max step time, and solver. The
  per-scene capture wrote a nonblank docked screenshot and 71 PNG frames from
  the 72-frame workflow row capture.

- Latest link-Jacobian row evidence: the real workflow packet
  `build/captures/rigid_link_jacobian_row_34_1781304169` captured row 34 with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and `guidance_complete=true`.
  The focused row/panel/docs-order/review-index pytest subset reported
  `6 passed`. Row 34 now exposes `link_origin_jacobian_mapping_family` as the
  comparison axis, held-fixed contact-free World
  two-revolute-link/time-step/finite-difference context, controls, Jacobian
  term order, Replay signal/markers, metric-key summary, and latest-signal
  ordering for link speed, world/body Jacobian gap, transpose-mapped torques,
  finite-difference residual, power residual, and solver. The per-scene capture
  wrote a nonblank docked screenshot and 95 PNG frames from the 96-frame
  workflow row capture.

- Latest link center-of-mass row evidence: the real workflow packet
  `build/captures/rigid_link_center_of_mass_row_33_1781303757` captured row 33
  with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 33 now exposes
  `link_center_of_mass_offset_family` as the comparison axis, held-fixed
  contact-free World revolute-link/fixed-visual-geometry/mass/gravity/time-step
  context, controls, lane order, Replay signal/markers, metric-key summary, and
  latest-signal ordering for centered/positive/negative gravity torque,
  mirrored angle sum, high-inertia mass and acceleration ratios, acceleration
  residual, and solver. The per-scene capture wrote a nonblank docked
  screenshot and 71 PNG frames from the 72-frame workflow row capture.

- Latest multibody dynamics-terms row evidence: the real workflow packet
  `build/captures/rigid_multibody_dynamics_terms_row_32_1781303198` captured
  row 32 with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 32 now exposes
  `joint_space_dynamics_term_family` as the comparison axis, held-fixed
  contact-free World dynamics/fixed-base/revolute-link/target-acceleration/
  impulse/gravity/time-step context, controls, lane order, Replay
  signal/markers, metric-key summary, and latest-signal ordering for scalar
  mass diagonal, coupled off-diagonal mass term, heavy-load torque gap,
  coupled/heavy response gap, heavy response ratio, inverse-dynamics residual,
  impulse residual, and solver. The per-scene capture wrote a nonblank docked
  screenshot and 95 PNG frames from the 96-frame workflow row capture.

- Latest screw-joint pitch coupling row evidence: the real workflow packet
  `build/captures/rigid_screw_joint_pitch_row_31_1781302747` captured row 31
  with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 31 now exposes
  `screw_pitch_coupling_family` as the comparison axis, held-fixed contact-free
  World screw-joint/z-axis/mass/inertia/time-step context, controls, lane
  order, Replay signal/markers, metric-key summary, and latest-signal ordering
  for zero-pitch axial travel, fine/coarse/reverse pitch, coarse/fine travel
  gap, reverse-sign angle, fine acceleration residual, and solver. The
  per-scene capture wrote a nonblank docked screenshot and 95 PNG frames from
  the 96-frame workflow row capture.

- Latest passive joint-parameter row evidence: the real workflow packet
  `build/captures/rigid_joint_passive_parameters_row_30_1781302360` captured
  row 30 with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 30 now exposes
  `passive_joint_parameter_family` as the comparison axis, held-fixed
  contact-free World-prismatic/link-mass/time-step context, controls, lane
  order, Replay signal/markers, metric-key summary, and latest-signal ordering
  for spring energy, damped energy, damped-energy ratio, slip speed, armature
  position gap, armature acceleration gap, solver, and executor. The per-scene
  capture wrote a nonblank docked screenshot and 119 PNG frames from the
  120-frame workflow row capture.

- Latest World multibody actuator/limit row evidence: the real workflow packet
  `build/captures/rigid_joint_motor_limits_row_29_1781301981` captured row 29
  with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 29 now exposes
  `world_multibody_actuator_limit_family` as the comparison axis, held-fixed
  x-axis-prismatic/y-axis-revolute/mass/time-step context, controls, lane
  order, Replay signal/markers, metric-key summary, and latest-signal ordering
  for motor speed, expected speed, speed error, limit angle/error, force travel
  gap, force acceleration gap, and solver. The per-scene capture wrote a
  nonblank docked screenshot and 95 PNG frames from the 96-frame workflow row
  capture. The successful capture was written under `build/captures/` after a
  previous `/tmp` attempt hit the host tmpfs quota.

- Latest one-DOF joint constraint-family row evidence: after merging
  `origin/main` and the PR #2986 DART 7 architecture harness locally, the real
  workflow packet `/tmp/dart_capture_rigid_limited_joints_row_28_1781301510`
  captured row 28 with `status=complete`, `capture_count=1`,
  `completed_count=1`, `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subset reported `6 passed`. Row 28 now exposes
  `one_dof_joint_constraint_family` as the comparison axis, held-fixed
  sequential-joint/static-base/z-axis/payload-mass/time-step context, controls,
  lane order, Replay signal/markers, metric-key summary, and latest signals for
  hinge radius/z error, slider orthogonal error, hinge yaw, slider travel,
  hinge angular speed, slider axis speed, and solver. The per-scene capture
  wrote a nonblank docked screenshot and 23 PNG frames from the 24-frame
  workflow row capture.

- Latest distance-spring response-family row evidence: the real workflow packet
  `/tmp/dart_capture_rigid_distance_spring_row_27_1781300614` captured row 27
  with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and `guidance_complete=true`.
  The focused row/panel/docs-order/review-index pytest subset reported
  `6 passed`. Row 27 now exposes `distance_spring_response_family` as the
  comparison axis, held-fixed executor/rest-length/payload/time-step context,
  controls, lane order, Replay signal/markers, metric-key summary, and latest
  signals for free, soft, stiff, and offset absolute stretch, offset angular
  speed, max sprung absolute stretch, and solver. The per-scene capture wrote a
  nonblank docked screenshot and 71 PNG frames from the 72-frame workflow row
  capture.

- Latest fixed-joint breakage lifecycle row evidence: the real workflow packet
  `/tmp/dart_capture_rigid_joint_breakage_row_26_1781299982` captured row 26
  with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and `guidance_complete=true`.
  The focused row/panel/docs-order/review-index pytest subset reported
  `7 passed`. Row 26 now exposes `fixed_break_force_lifecycle` as the
  comparison axis, held-fixed AVBD/static-base/payload-mass/offset context,
  controls, Replay signal/markers, metric-key summary, and latest signals for
  payload release distance, broken state, captured-offset error, payload speed,
  status, and solver. The per-scene capture wrote a nonblank docked screenshot
  and 47 PNG frames from the 48-frame workflow row capture.

- Latest fixed-joint verifier row evidence: the real workflow packet
  `/tmp/dart_capture_rigid_fixed_joint_row_25_1781299686` captured row 25 with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and `guidance_complete=true`.
  The focused row/panel/docs-order/review-index pytest subset reported
  `6 passed`. Row 25 now exposes `fixed_relative_transform_recovery` as the
  comparison axis, held-fixed sequential-joint/static-base/payload-mass/offset
  context, controls, Replay signal/markers, metric-key summary, and latest
  signals for fixed-joint translation error, orientation error, payload speed,
  angular speed, solver, and history samples. The per-scene capture wrote a
  nonblank docked screenshot and 23 PNG frames from the 24-frame workflow row
  capture.

- Latest contact/kinematic-push comparison mini-packet:
  the real workflow packet
  `/tmp/dart_capture_rigid_contact_kinematic_rows_22_24_1781299153`
  captured rows 22-24 with `status=complete`, `capture_count=3`,
  `completed_count=3`, `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/docs-order/review-index
  pytest subsets reported `6 passed` for the row-22 follow-up, `6 passed` for
  the row-23 follow-up, and `7 passed` for the row-24 follow-up. The review
  index keeps
  `rigid_contact_manipulation`, `rigid_kinematic_driver`, and
  `rigid_kinematic_normal_push` adjacent. Row 22 now exposes
  `rigid_pusher_contact_response` as the comparison axis, held-fixed matched
  table/goal context, controls, Replay signal/markers, metric-key summary, and
  latest signals that show travel divergence, both solver lanes' target travel,
  sequential-impulse contact evidence, IPC proximity gap, and solver pair. Row
  23 exposes `prescribed_tangential_contact_response` as the comparison axis,
  held-fixed tangential-support context, controls, Replay signal/markers,
  metric-key summary, and latest signals that show IPC grip box travel, grip
  speed ratio, low-friction slip, sequential-impulse caveat driver travel, IPC
  proximity gap, and solver pair. Row 24 exposes
  `prescribed_normal_contact_response` as the comparison axis, held-fixed
  normal-paddle context, controls, Replay signal/markers, metric-key summary,
  and latest signals that show target-travel divergence, SI target travel, IPC
  penetration depth, solver pair, case pair, and solver label. Per-scene
  captures wrote nonblank docked screenshots and 71 PNG frames for each row.
- Latest contact-failure comparison mini-packet:
  the real workflow packet
  `/tmp/dart_capture_rigid_contact_failure_rows_19_21_1781296166`
  captured rows 19-21 with `status=complete`, `capture_count=3`,
  `completed_count=3`, `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`. The focused row/panel/review-index pytest subset
  reported `5 passed`. The review index keeps `rigid_friction_threshold`,
  `rigid_spin_roll_coupling`, and `rigid_stack_stability` adjacent and exposes
  each row's comparison axis, held-fixed values, controls, guidance, commands,
  metric summaries, and latest signals. The row cards showed
  `friction_threshold_lane` with IPC/executor/time-step held fixed and threshold
  drift signals, `spin_roll_initial_condition` with sequential-impulse/
  executor/time-step held fixed and contact-slip/spin-change signals, and
  `rigid_body_solver_family` with executor/friction/top-mass/time-step held
  fixed plus solver pair, top-x divergence, and clearance signals. Per-scene
  captures wrote nonblank docked screenshots, 23 PNG frames for
  `rigid_friction_threshold`, 95 PNG frames for `rigid_spin_roll_coupling`, and
  23 PNG frames for `rigid_stack_stability`.
- Latest solver/contact-policy mini-packet evidence refresh:
  the real workflow packet
  `/tmp/dart_capture_rigid_workflow_solver_contact_rows_15_17_1781294600`
  captured rows 15-17 with `status=complete`, `capture_count=3`,
  `completed_count=3`, `failed_count=0`, `workflow_total_count=36`,
  `guidance_complete=true`, and `guidance_missing_count=0`. The review index
  links the three adjacent comparison rows and exposes their maintained
  questions, live open commands, workflow row rerun commands, direct capture
  commands, Replay tracks, metric-key summaries, held-fixed/control values, and
  latest decision-signal values for solver/executor/contact-policy comparisons.
  Per-scene manifests recorded docked nonblank screenshots, 23 PNG frames each
  for `rigid_solver_compare` and `rigid_executor_equivalence`, 71 PNG frames for
  `rigid_contact_solver_compare`, and scene-metric streams showing the
  `rigid_body_solver_family`, `executor`, and `contact_solver_method`
  comparison axes. The latest metrics reported sequential-impulse-vs-IPC
  solver pair, same-solver sequential-vs-parallel executor pair with max pose
  divergence about `2.22e-16`, and sequential-impulse-vs-boxed-LCP contact
  policy pair.
- Latest direct-capture workflow-guidance slice:
  direct `py-demo-capture -- --scene ...` manifests for maintained rigid
  workflow rows now include a `workflow_guidance` block with row number, role,
  user question, try-first action, inspect signals, healthy signal, and scope
  note. The focused
  `test_visual_capture_manifest_records_image_evidence` guard verifies the
  manifest block while preserving image, Replay metadata, and scene-metrics
  evidence; the focused direct-manifest plus rigid-workflow capture test subset
  reported `26 passed, 13 deselected`.
- Latest review-index row-rerun command slice:
  every workflow review card now includes the packet-preserving
  `workflow_rerun_command` for that selected row, not only the failed-row
  summary block. The focused
  `test_rigid_workflow_dry_run_writes_capture_plan` guard checks that a
  planned review card exposes `rerun workflow row`, the absolute
  `--workflow-start-row/--workflow-end-row` selector, and the row-specific
  `reruns/<row>_<scene>` output directory.
- Latest workflow failure-resilience slice:
  `py-demo-capture -- --rigid-workflow --continue-on-failure` now keeps
  capturing later selected rows after a row fails while still writing a final
  failed workflow manifest and returning a failing process exit code when any
  row failed. The top-level manifest records `continue_on_failure` and
  `failed_rows`; the review index summary records the failure mode and shows a
  Failed Rows block with workflow row-range rerun commands that preserve packet
  flags and absolute row numbering; and the in-viewer `Rigid Workflow` panel
  exposes a resilient extended-packet command for long 36-52 row review
  packets. The focused pytest covering failed-row summaries, workflow row
  rerun commands, fail-fast behavior,
  continue mode,
  workflow-only flag validation, panel command rendering, and README/PLAN-103
  documentation reported `11 passed`. The public dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --continue-on-failure --workflow-start-row 51 --workflow-end-row 52 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_continue_on_failure_dry_run_1781288150`
  reported `status=planned`, `dry_run=true`, `continue_on_failure=true`,
  `capture_count=2`, `workflow_total_count=52`, selected rows `51-52`,
  `guidance_complete=true`, and `guidance_missing_count=0`, with the two
  selected rows as `rigid_ipc_stack_packet` and
  `rigid_ipc_heavy_stack_packet`. The failed-row triage dry-run
  `/tmp/dart_capture_rigid_workflow_failed_rows_dry_run_1781289000` confirmed
  `failed_rows` length `0` for the planned packet and a review-index
  `failure mode` badge of `continue`.
  `/tmp/dart_capture_rigid_workflow_failed_row_rerun_dry_run_1781289350`
  confirmed planned rows 51 and 52 also carry `workflow_rerun_command` values
  preserving `--include-related --include-ipc-shelf --include-packets
--continue-on-failure` and their absolute row ids.
- Latest heavy Rigid IPC stack packet slice:
  `python -m py_compile python/examples/demos/scenes/rigid_ipc_stack_packet.py scripts/capture_py_demo.py python/tests/integration/test_demos_cycle.py python/tests/unit/test_py_demo_panels.py python/tests/unit/test_capture_py_demo.py`
  passed. The focused Python suite covering registry/category placement,
  non-numbered workflow placement, capture-first packet docs/spec sync, panel
  metrics, replay restore, and full extended-packet guidance reported
  `11 passed` before and after `pixi run lint`. The extended dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 51 --workflow-end-row 52 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_heavy_stack_dry_run_1781286611`
  reported `status=planned`, `capture_count=2`, `workflow_total_count=52`,
  selected rows `51-52`, `guidance_complete=true`, and
  `guidance_missing_count=0`; its review index showed
  `51/52 rigid_ipc_stack_packet` and
  `52/52 rigid_ipc_heavy_stack_packet` as capture-first packet rows. The real
  row-52 workflow capture
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 52 --workflow-end-row 52 --output-dir /tmp/dart_capture_rigid_workflow_heavy_stack_row52_1781286631`
  completed with `capture_count=1`, `completed_count=1`, `failed_count=0`,
  `workflow_total_count=52`, `guidance_complete=true`, and
  `guidance_missing_count=0`; the per-scene manifest for
  `rigid_ipc_heavy_stack_packet` recorded 11 PNG frames, a docked nonblank
  screenshot with 3539 unique RGB colors, 12 scene-metric events,
  `box_count=6.0`, `top_mass=4.25`, row `rigid_ipc_heavy_stack_packet`, and
  `status=capture-first`. `pixi run lint` passed and `git diff --check` was
  clean.
- Previous completion/retirement readiness audit: the maintained 36-row rigid
  workflow plus optional rows 37-52 packet now has durable sidecar coverage,
  Python README user guidance, sync/drift tests, full workflow capture evidence,
  optional packet capture evidence, and explicit public-API deferrals. The
  working dev-task folder was retired in the completing PR; future follow-ups
  should update this durable sidecar and the Python demo README instead of
  restoring task-local handoff notes.
- Latest workflow-search count follow-up: the in-viewer `Rigid Workflow` filter
  now counts the full ranked result set while still showing only the top six
  rows, so broad queries such as `contact` display a `Showing 6 of N matching
workflow rows` line before the selectable results. This keeps the navigator
  compact while making capped result sets visible. The focused panel/search and
  README-order guard reported `12 passed`.
- Latest workflow focus-axis follow-up: every numbered `Rigid Workflow` row now
  exposes a front-door `Focus axis` before the try/inspect checklist. The axis
  names the main comparison or debugging dimension, such as solver family,
  executor-only behavior, contact solver method, workload shape/backend
  profile, passive joint parameter family, or loop-closure family, and the axis
  text participates in search ranking and tooltip reasons. The focused
  panel/search guard reported `6 passed`; after docs updates, the broader
  panel/search/docs-order guard reported `15 passed`.
- Latest workflow phase follow-up: every numbered `Rigid Workflow` row now
  exposes a `Workflow phase` before the focus axis and try/inspect checklist.
  The phases group rows into foundations, diagnostics, contact/material/query
  basics, the solver decision path, contact behavior cases, rigid
  constraints/joint mechanics, and multibody dynamics/kinematics. Multi-token
  phase searches route to the first row in the matching phase and report
  `Search match: workflow phase` without changing broad single-token ordering.
  The focused phase guard reported `4 passed`, and the broader
  panel/search/docs-order guard reported `17 passed`.
- Latest review-prep follow-up: numbered workflow captures now export
  `workflow_phase` and `focus_axis` into single-scene `workflow_guidance`,
  workflow manifests, and static review cards. Missing numbered-row phase or
  focus metadata now fails the workflow guidance completeness audit. The
  focused capture-helper guard reported `4 passed`; a row-15 dry-run manifest
  reported `workflow_phase=4. Solver decision path` and
  `focus_axis=rigid-body solver family`, and its review card contained
  `<dt>phase</dt>` plus `<dt>focus axis</dt>`.
- Latest static review phase-map follow-up: workflow manifests now record
  `workflow_phase_summary` for selected numbered rows, and the review index
  header shows a `Workflow Phase Map` with row ranges, phase counts,
  per-phase status, focus axes, and scene ids before the per-row cards. This
  keeps full and targeted row-range packets reviewable by logical phase and
  capture state after the phase/focus metadata export. The focused
  capture-helper guard reported `3 passed`; a follow-up focused guard covering
  phase status/focus-axis display reported `4 passed`.
- Latest deferred-search UX follow-up: deferred public-API searches such as
  `sleep wake`, `island activation`, and `loop closure compliance` still route
  to the closest current verifier rows, but the result tooltip, guide payload,
  workflow manifest, static review card, and top-level review-index caveat map
  now show a `Deferred API caveat` note before reporting the search match. The
  focused panel/capture/docs guard reported `7 passed` for the latest summary
  update after the earlier tooltip/card guard reported `8 passed`.
- Latest API search follow-up: public dartpy now exposes direct
  `RigidBody.apply_linear_impulse()` and `RigidBody.apply_angular_impulse()`
  surfaces, while sleep/wake or island activation and loop-closure
  compliance/stiffness/damping remain unavailable. The in-viewer `Rigid
Workflow` search routes `direct rigid body impulse` to
  `rigid_external_loads`, and continues to route `sleep wake`,
  `island activation`, and `loop closure compliance` to the closest current
  rows with their explicit non-claim caveats.
- Latest optional extended-packet capture refresh:
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --output-dir /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053`
  completed the non-numbered related-evidence, direct Rigid IPC shelf, and
  capture-first packet rows as real docked captures. The top-level manifest
  reported `status=complete`, `capture_count=15`, `completed_count=15`,
  `failed_count=0`, `workflow_total_count=51`, selected rows `37-51`,
  requested and selected related/IPC-shelf/packet groups all `true`,
  `guidance_complete=true`, `guidance_missing_count=0`, and
  `elapsed_s=149.609`. The selected packet covered ten related-evidence rows
  (`floating_base`, `articulated`, `rigid_ipc_tunnel`, `rigid_ipc_edge_drop`,
  `diff_drone_liftoff`, and five AVBD rigid routes), four direct Rigid IPC shelf
  rows (`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`,
  `rigid_ipc_pile`), and the capture-first `rigid_ipc_stack_packet` row. Every
  per-scene manifest inspected had positive frame counts, docked screenshots,
  nontrivial unique-color counts, and scene-owned metric events. The review
  index at
  `/tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/review_index.html`
  showed the `37-51 / 51` row span, requested groups `numbered, related, ipc
shelf, packets`, selected groups `related, ipc shelf, packets`, guidance
  complete, representative route cards such as `37/51 floating_base`,
  `46/51 avbd_rigid_prismatic_motor`, `47/51 rigid_ipc`, and
  `51/51 rigid_ipc_stack_packet`.
- Latest Rigid IPC shelf capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `2 passed`. The direct Rigid IPC shelf scenes `rigid_ipc`,
  `rigid_ipc_slide`, `rigid_ipc_incline`, and `rigid_ipc_pile` now export
  scene-owned capture metrics for row identity, IPC solver label, scope,
  time-step/world-time, friction, status, contact counts, step timing, and
  scene-specific barrier gap, speed, travel, height, span, or pile summaries.
  The focused test also checks that each scene preserves shared replay controls
  through `replay_sync` and `replay_live_step_is_stateless`.
  The real docked capture
  `pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current`
  wrote a 960x540 screenshot with docked UI detected, 23 converted PNG frames,
  24 scene-metric events, latest row `rigid_ipc_pile`, scope
  `multi_box_barrier_pile`, 25 history samples, `box_count=3`, maximum history
  speed about `1.177` m/s, and minimum history clearance about `0.276` m.
- Latest rigid IPC edge-drop related-evidence follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_edge_drop_reports_degenerate_contact_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches -q`
  reported `9 passed`. The new `rigid_ipc_edge_drop` related route reports
  `row=rigid_ipc_edge_drop`, `related_source_row=rigid_solver_compare`,
  `solver=rigid_ipc`, `scope=degenerate_edge_contact_capability`,
  `min_barrier_gap <= 0.01`, and nonzero angular/tilt motion; the route table,
  docked capture command list, capture-helper related specs, panel route/search
  coverage, and unnumbered-route guard all include the edge-drop scene. The
  public dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run --output-dir /tmp/dart_capture_rigid_workflow_related_edge_drop_dry_run`
  reported `capture_count=46`, `40/46 rigid_ipc_edge_drop`, and final
  `46/46 avbd_rigid_prismatic_motor`. The row-range packet dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 47 --workflow-end-row 47 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run`
  reported `workflow_total_count=47` and `47/47 rigid_ipc_stack_packet`. The
  real docked capture
  `pixi run py-demo-capture -- --scene rigid_ipc_edge_drop --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_edge_drop_edge_barrier_current`
  wrote a nonblank 960x540 screenshot with docked UI detected, 71 converted PNG
  frames, latest status `edge-barrier`, minimum barrier gap about `0.000384` m,
  maximum tilt about `55.33` degrees, and maximum angular speed about
  `0.555` rad/s.
- Latest row-range workflow rerun follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q`
  reported `11 passed`. The public dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 47 --workflow-end-row 47 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run`
  wrote a workflow manifest with `capture_count=1`,
  `workflow_total_count=47`, `workflow_row_start=47`, `workflow_row_end=47`,
  and the selected row as `rigid_ipc_stack_packet` with
  `workflow_group=capture_first_packet`. The generated `review_index.html`
  contained the absolute `47/47 rigid_ipc_stack_packet` row.
- Latest capture-first packet bundle follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented -q`
  reported `8 passed`. The public dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run`
  wrote a workflow manifest with `include_related=true`,
  `include_packets=true`, `capture_count=47`, and the final row as
  `rigid_ipc_stack_packet` with `workflow_group=capture_first_packet`. The
  generated `review_index.html` contained the final
  `47/47 rigid_ipc_stack_packet` row.
- Latest related-evidence capture-bundle follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_include_related_requires_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q`
  reported `5 passed`. The public dry-run
  `pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run --output-dir /tmp/dart_capture_rigid_workflow_related_dry_run`
  wrote a workflow manifest with `include_related=true`, `capture_count=46`,
  the first related-evidence row at `captures[36]` as `floating_base`, and the
  final row as `avbd_rigid_prismatic_motor`. The generated
  `review_index.html` contained related-evidence cards and the final
  `46/46 avbd_rigid_prismatic_motor` row.
- Latest full workflow-capture follow-up:
  `pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_full_review_index_1781259714`
  completed all 36 numbered rigid verifier captures. The workflow manifest
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/manifest.json`
  reported
  `status=complete`, `capture_count=36`, `completed_count=36`,
  `failed_count=0`, and `elapsed_s=314.278`; every capture entry had
  `return_code=0`, `status=captured`, and `manifest_exists=true`. The first
  per-scene manifest was
  `scenes/01_rigid_body/manifest.json`, the last was
  `scenes/36_rigid_loop_closure/manifest.json`, and each row wrote a docked
  screenshot plus PNG frame sequence through the same capture helper path. The
  workflow manifest also pointed at
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/review_index.html`;
  that generated contact sheet contained 36 screenshot thumbnails and linked
  the first and last row screenshots plus comparison/held-fixed metric
  summaries for the solver, executor, contact-policy, time-step, workload, and
  passive-parameter rows.
- Latest distance-spring follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches -q`
  reported `17 passed` before and after `pixi run lint`. The real docked
  capture
  `pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_1781220718`
  wrote a nonblank 960x540 screenshot with docked-workspace detection, 71 PNG
  frames, and 72 scene-metrics events. Frame 72 mirrored the
  `free/soft/stiff/offset` lane set into `manifest.json`: the unsprung lane
  stayed at `0.330` m stretch, the soft/stiff lanes reduced absolute stretch to
  about `0.113` m and `0.125` m, and the off-center lane reached about
  `0.001` m stretch with `1.956` rad/s angular speed.
- Latest scene-metrics capture hardening follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py python/tests/unit/test_py_demo_gpu_toggle.py -q`
  reported `25 passed`, and
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals -q`
  reported `3 passed`. The real docked
  `pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui`
  run wrote `scene_metrics.jsonl` with 24 scene-owned metrics events and
  mirrored frame 24 into `manifest.json`: `status=capture-first`, min clearance
  `0.000267` m, top drift `0.000360` m, height error `-0.010816` m,
  max speed `0.0493` m/s, last-step wall time `626.778` ms, frame budget
  `33.3` ms, benchmark owner `bm_rigid_ipc_solver`, and `world_time=0.100` s.
  The same capture wrote a nonblank 960x540 screenshot, 23 PNG frames, one
  dropped UI warmup frame, docked-workspace detection, 3790 unique screenshot
  colors, and RGB variance `3394.733`. After `pixi run lint`, rerunning the
  capture/runner unit files reported `25 passed`, rerunning the 12-test
  registry/docs/panel guard reported `12 passed`, and `git diff --check` was
  clean.
- Latest runtime-row capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads -q`
  reported `2 passed`. `rigid_step_diagnostics` now exposes solver/executor,
  step size, world time, and per-lane profiling/memory/contact metrics through
  `SceneSetup.info["capture_metrics"]`; `rigid_contact_scale_budget` exports
  the same capture path plus frame budget, friction, dense/single wall-time
  ratio, and one/four/nine-box contact budget lane metrics. Short real
  `py-demo-capture --show-ui` smoke runs for both rows wrote eight
  scene-metrics events each, mirrored the expected lane sets
  (`single/contact/stack` and `single/medium/dense`) into `manifest.json`, and
  produced nonblank docked screenshots with 2186 and 2120 unique screenshot
  colors.
- Latest numbered workflow capture-composition follow-up:
  `PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release-docking/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_py_demo_capture_records_numbered_rigid_workflow_metrics_artifacts python/tests/unit/test_py_demo_panels.py::test_numbered_rigid_workflow_factory_combines_panels -q`
  reported `2 passed`. The real docked capture now exercises the numbered
  `rigid_contact_scale_budget` workflow path, verifies nonblank docked
  screenshot and first-frame evidence, requires `scene_metrics` in the
  manifest, and checks the latest metrics row exports the
  `single/medium/dense` lane set. The companion no-GUI guard keeps the actual
  composed panels ordered as `Rigid Workflow`, `Rigid Contact Scale Budget`,
  then `Replay`.
- Latest capture-evidence hardening follow-up: `py-demo-capture` manifests now
  record requested width/height/frames, converted frame count, and screenshot
  plus first UI-ready-frame evidence: dimensions, nonzero pixels, unique RGB
  count, RGB/luminance variance, and docked-workspace detection.
  `pixi run test-py` reported `636 passed, 9 skipped`, including the helper
  stats guard and deterministic capture-manifest schema guard.
- Latest link-contact follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  reported `18 passed`. It promotes the stable `contact` scene id into row
  18/36 as `Rigid Link Contact`, verifies multibody-link drop, slide, and pusher
  contact response plus replay controls, and keeps the registry, sidecar,
  README, capture commands, viewer-title numbering, workflow guidance, route
  controls, and panel coverage synchronized. The docked visual capture
  `pixi run py-demo-capture -- --scene contact --frames 72 --width 960 --height 540 --show-ui`
  wrote a nonblank 960x540 screenshot plus 71 PNG frames.
- Latest link-contact capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response -q`
  reported `1 passed`. The `contact` row now exports scene-owned capture
  metrics for the sequential-impulse rigid-body solver, multibody-link contact
  scope, executor/material/drop/slide/push controls, current drop/slide/pusher
  lane metrics, contact body kinds, and compact history ranges. The real
  docked capture
  `pixi run py-demo-capture -- --scene contact --frames 144 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_metrics_1781225144`
  wrote a nonblank 960x540 docked screenshot, 143 PNG frames, and 144
  scene-metrics events. The latest manifest metrics recorded
  `drop_max_contacts=1.0`, `slide_max_contacts=1.0`,
  `push_max_contacts=1.0`, `push_max_target_travel=0.07463822342673943`, and
  `drop_max_upward_velocity=1.1546369999999997`.
- Previous full Python sweep evidence: `pixi run test-py` reported
  `617 passed, 9 skipped` after the joint-breakage lifecycle follow-up.
- Current curation guard:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_capture_py_demo.py::test_visual_capture_default_scene_matches_py_demos_front_door -q`
  reported `5 passed`. The guard keeps the sidecar rows unique, consecutive,
  backed by registered scene ids, aligned with the front-of-catalog
  `World Rigid Body` registry block, mirrored by the Python demo README quick
  workflow table, backed by matching sidecar table / sidecar refresh / README
  capture specs (`--scene`, `--frames`, `--width 960`, `--height 540`, and
  `--show-ui`), aligned with the `py-demo-capture` default front door, and
  separate from the `Rigid IPC` `rigid_ipc_tunnel` capability shelf.
- Latest Demos navigator title follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_runner_list_prints_catalog python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `4 passed`. It keeps the interactive viewer catalog titles
  prefixed with `01/36 ...` through `36/36 ...` for the curated World Rigid
  Body workflow while preserving stable `py-demos --list` scene ids and titles
  for scripts. The short docked capture command for `rigid_body` wrote a
  nonblank 960x540 screenshot plus 7 PNG frames, `pixi run test-py` reported
  `616 passed, 9 skipped`, `pixi run lint` passed, bounded
  `pixi run build` reported `ninja: no work to do`, and `git diff --check` was
  clean.
- Latest navigator-count drift follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `4 passed`. It checks the viewer-title count against
  `RIGID_VISUAL_WORKFLOW_LABELS` and scans the durable sidecar plus Python demo
  README for stale two-digit navigator fractions, so examples such as
  `15/36 Solver family: Rigid Solver Compare` stay aligned with the current
  36-row workflow.
- Latest kinematic normal-push caveat follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  reported `17 passed`. It keeps `rigid_kinematic_normal_push` directly after
  the tangential kinematic-driver row, verifies IPC normal/heavy penetration
  caveat lanes against the sequential-impulse push lane, checks replayed
  controls and capture metrics, and preserves row guidance, sidecar/README
  order, capture-command drift, category, and panel coverage. The standard
  docked visual smoke command for `rigid_kinematic_normal_push` wrote a
  nonblank 960x540 screenshot plus 71 PNG frames and 72 scene-metrics events;
  frame 72 had IPC normal/heavy `status=ipc penetration caveat` with about
  0.125 m depth and near-zero target travel, while the sequential-impulse lane
  had `status=pushed` with about 0.123 m target travel.
- Latest API-gap audit follow-up: public dartpy currently exposes
  `RigidBody.apply_force()`, `RigidBody.apply_torque()`,
  `RigidBody.apply_linear_impulse()`, `RigidBody.apply_angular_impulse()`,
  rigid-body linear/angular momentum accessors, `Link.apply_force(..., point,
...)`, and `Multibody.compute_impulse_response()`, but no public sleep/wake
  or island activation surface or loop-closure compliance surface. The drift guard
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `4 passed`. This keeps those deferrals explicit and points future
  API additions back to this workflow.
- Latest joint-breakage follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps `rigid_joint_breakage` ordered after the
  fixed-joint row, verifies the AVBD break-force threshold, broken state,
  connector/panel coverage, payload release, and `reset_breakage()` lifecycle,
  and preserves sidecar/README order, capture-command drift, category, and
  numbered viewer-title coverage. The docked visual smoke command for
  `rigid_joint_breakage` wrote a nonblank 960x540 screenshot plus 47 PNG frames
  with final contacts at 0. The later capture-metrics follow-up records 48
  scene-metrics events with latest status `broken`, `saw_broken=1.0`, and
  `payload_release_distance=0.41319290960568955`.
- Latest collision-casts follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `7 passed` before and after lint. It keeps the cast row ordered after
  `World.collide(options)` query filtering and before solver comparison,
  verifies nearest versus all ray hits, hit fractions, point/normal reporting,
  swept-sphere time of impact, swept-capsule hit/miss behavior, finite capsule
  hit point/normal reporting, miss behavior from ray offset, sphere radius, and
  capsule offset, replay controls, scene-owned capture metrics, sidecar/README
  order, capture-command drift, category, and panel coverage.
  `pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_collision_casts_metrics_1781227741`
  wrote a nonblank 960x540 screenshot, 47 PNG frames, and 48 scene-metrics
  events. The manifest recorded numeric ranges for ray/sphere/capsule hit
  counts, TOI/fraction, margins, time step, and world time.
- Latest frame-hierarchy follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
  reported `12 passed` before and after lint. It keeps the workflow ordered as
  it stood then with `rigid_frame_hierarchy` after free flight and before external
  loads, verifies fixed-frame parent identity, local-to-world and relative
  transform residuals, replay-control snapshots, breakage panel reset behavior,
  sidecar/README order, capture-command drift, category, and numbered
  viewer-title coverage. The docked visual smoke command for
  `rigid_frame_hierarchy` wrote a nonblank 960x540 screenshot plus 71 PNG frames
  with final contacts at 0. `pixi run lint` passed; bounded `pixi run build`
  reported `ninja: no work to do`; and `git diff --check` was clean.
- Latest baseline-hardening follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_body_panel_resets_baseline_scene python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
  reported `15 passed`. It upgrades the default `rigid_body` first-run scene with
  selectable solver/material controls, explicit reset, force drag, contact,
  energy, height, speed, step-timing, and replay-control diagnostics while keeping
  focused edge cases in later rows. The docked visual smoke command for
  `rigid_body` wrote a nonblank 960x540 screenshot plus 23 PNG frames with final
  contacts at 0. `pixi run lint` passed and bounded `pixi run build` reported
  `ninja: no work to do`.
- Latest baseline capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics -q`
  reported `1 passed`. `rigid_body` now exports scene-owned capture metrics for
  solver, solver enum, friction, restitution, dynamic-body count, world time,
  current speed/height/energy/contact/step diagnostics, and history ranges. The
  real docked capture
  `pixi run py-demo-capture -- --scene rigid_body --frames 180 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_body_metrics_1781224228`
  wrote a nonblank 960x540 docked screenshot, 23 PNG frames, and 24
  scene-metrics events.
- Latest contact-inspector follow-up:
  `pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_inspector_reports_contact_manifolds python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `4 passed`. It covers sphere/box, box/ground, plane/sphere,
  capsule/sphere, cylinder/sphere, mesh/sphere, and compound/sphere lanes,
  including finite unit normals, depths, local points, public plane-shape
  coverage, and compound shape-index reporting.
- Latest step-diagnostics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `2 passed`. It keeps the row's profile, memory, contact, panel, and
  backend-neutral acceleration-status fields visible without claiming an
  accelerated rigid backend is active.
- Latest contact-scale budget follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the row ordered after step diagnostics and
  before restitution, verifies the one/four/nine-box contact workloads increase
  body, entity, component, and contact-point counts, and preserves replay
  controls, sidecar/README order, capture-command drift, category, and panel
  coverage.
- Latest screw-joint pitch follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the screw-pitch row ordered after passive
  joint parameters, verifies pitch-as-translation-per-radian coupling, checks
  expected acceleration against mass and axial inertia, and preserves replay
  controls, sidecar/README order, capture-command drift, category, and panel
  coverage.
- Latest multibody dynamics-terms follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the generalized dynamics row ordered after
  screw-joint pitch and before multibody solver-family routing, verifies
  fixed-base joint-space mass/inverse-mass consistency, inverse-dynamics
  residuals, joint-space impulse-response residuals, off-diagonal coupling,
  heavy-distal torque/response trends, replay controls, sidecar/README order,
  capture-command drift, category, and panel coverage.
- Latest link center-of-mass follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  reported `17 passed`. It inserts `rigid_link_center_of_mass` after
  generalized dynamics terms and before link-origin Jacobians, verifies
  centered, mirrored, and high-inertia COM offsets against public
  `Link.center_of_mass`, mass-matrix, gravity-force, acceleration-sign, and
  replay-control behavior, and preserves sidecar/README order, capture-command
  drift, category, numbered title, workflow-guide, and panel coverage.
  `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui`
  wrote a nonblank 960x540 screenshot plus 71 PNG frames with final contacts at 0.
- Latest link center-of-mass capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque -q`
  reported `1 passed`. `rigid_link_center_of_mass` now exports scene-owned
  capture metrics for row identity, inertial-offset scope, executor,
  COM/gravity/mass/inertia controls, lane order/count, per-lane link/joint/
  local-COM metadata, mirrored torque/angle/acceleration sums, reflected
  mass-matrix and acceleration ratios, COM marker position, energy, step
  timing, and compact histories.
  `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_center_of_mass_metrics_1781237623`
  wrote a nonblank 960x540 screenshot, 71 PNG frames, 72 scene-metrics events,
  and final contacts at 0. The latest manifest event recorded row
  `rigid_link_center_of_mass`, solver `world_multibody_inertial_offsets`,
  scope `contact_free_link_center_of_mass_offsets`, executor `Sequential`,
  lane count `4`, COM offset `0.18`, gravity scale `1.0`, link mass `2.0`,
  inertia scale `4.0`, mirrored positive/negative angles about
  `+/-0.44895`, mirrored torques about `+/-3.18162`, zero mirrored torque,
  angle, and acceleration sums, high/positive mass-matrix ratio about `2.948`,
  high/positive acceleration ratio about `0.3702`, 73 history samples, and max
  observed step time about `0.1534 ms`. The broader workflow/doc drift guard
  with row ordering, viewer-title numbering, sidecar/README/capture-command
  drift checks, adjacent row coverage, replay snapshot coverage, and high-value
  panel coverage reported `14 passed`; `pixi run lint`, bounded
  `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 CMAKE_BUILD_PARALLEL_LEVEL=2 pixi run build`,
  and `git diff --check` passed.
- Latest link center-of-mass Replay timeline follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
  reported `6 passed`. It verifies the `Mirrored COM angle spread` Replay
  timeline label, signal, mirrored-angle marker, centered-still marker,
  high-inertia-lag marker, quiet-frame behavior, and sidecar/README/capture
  command drift. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_center_of_mass_timeline_1781279455`
  wrote a nonblank 960x540 screenshot plus 71 PNG frames and
  72 scene-metrics events. The manifest recorded mirrored positive/negative
  angles about `+/-0.449`, high-inertia angle about `0.153`, centered angle
  `0.0`, mirrored torques about `+/-3.182`, high/positive acceleration ratio
  about `0.370`, high/positive mass-matrix ratio about `2.948`, and historical
  max positive/high-inertia angles about `0.449`/`0.153`.
- Latest link-Jacobian comparison-axis follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_jacobian_maps_link_origin_twist_and_wrench python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_link_jacobian_values -q`
  reported `6 passed`. It keeps the link-origin Jacobian row ordered after
  generalized dynamics terms and before multibody solver-family routing,
  verifies `get_world_jacobian(link) @ qdot` against finite-difference
  link-origin velocity, verifies `get_world_jacobian(link).T @ wrench` power
  consistency, verifies the `Link-origin speed` Replay timeline label and
  signal, covers high-twist, wrench-load, world/body-gap, residual-alert, and
  quiet-frame marker behavior, and now checks the comparison-axis panel text
  plus latest-signal ordering for link speed, world/body gap, finite-difference
  residual, torques, power residual, and solver. The real row-34 workflow
  capture
  `pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 34 --workflow-end-row 34 --output-dir build/captures/rigid_link_jacobian_row_34_1781304169`
  completed with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`, a
  nonblank 960x540 docked screenshot with 2138 unique colors, 95 PNG frames,
  and 96 scene-metrics events. The latest manifest event recorded
  `comparison_axis=link_origin_jacobian_mapping_family`, held-fixed
  `contacts=off`, `gravity=off`, `joint_family=two_revolute_links`,
  `link_length=0.55`, `finite_difference_eps=1e-6`,
  `solver=world_multibody_link_jacobian`, `time_step_ms=4.0`, controls
  `motion_speed=0.85`, `elbow_phase=0.72`, `wrench_force=1.35`,
  `wrench_angle_deg=28.0`, `wrench_moment=0.12`,
  `jacobian_terms=[world_jacobian_twist, finite_difference_velocity, jacobian_transpose_wrench, world_body_jacobian_gap]`,
  latest linear/angular speed about `0.5652/0.6190`, finite-difference error
  about `1.52e-7`, power error `0`, world/body Jacobian gap about `0.1272`,
  torques about `-0.8650/-0.2949`, historical max link speed about `0.6677`,
  historical max world/body gap about `0.2055`, and historical max absolute
  torques about `0.8650`/`0.2949`.
- Latest loop-closure comparison-axis follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_loop_closure_values -q`
  reported `6 passed`. It keeps one loop-closure workflow row, verifies the
  capture hook mirrors POINT, DISTANCE, and RIGID public families crossed with
  residual-only and solved policies, per-family residual ratios, controls, case
  metadata, compact histories, and Replay timeline value/marker metadata, and
  now checks the comparison-axis panel text plus latest-signal ordering for
  family residual ratios, distance-family distance/tip error, RIGID orientation
  error, and solver. The real row-36 workflow capture
  `pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 36 --workflow-end-row 36 --output-dir build/captures/rigid_loop_closure_row_36_1781304923`
  completed with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`, a
  nonblank 960x540 docked screenshot with 2446 unique colors, 71 PNG frames,
  and 72 scene-metrics events. The latest manifest event recorded
  `comparison_axis=loop_closure_family_policy_selection`, held-fixed
  `contacts=off`, `integration_family=variational integrator`,
  `joint_family=four_revolute_links`, `chain_links=4`, `link_length=0.56`,
  `link_mass=0.55`, `initial_bend=0.18`, `gravity_scale=1.0`,
  `solver=variational_rigid_multibody_loop_closure`, `time_step_ms=5.0`,
  controls `executor_index=0.0`, `gravity_scale=1.0`,
  `closure_family_lanes=[POINT, DISTANCE, RIGID]`,
  `closure_policy_lanes=[residual, solved]`, POINT/DISTANCE/RIGID residual
  ratios about `7.595e11`/`7.458e11`/`7.740e11`, rigid residual orientation
  error about `0.149`, solved orientation error near `2.8e-17`, distance solved
  distance error near `9.4e-15`, distance solved tip error about `0.439`, and
  max step time about `0.483 ms`.
- Latest body-mode follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the new early workflow row ordered after
  `rigid_body`, verifies dynamic integration under gravity/force, static no
  drift, kinematic prescribed-path tracking, replay controls, sidecar/README
  order, capture-command drift, category, and panel coverage.
- Latest multibody solver-family comparison-axis follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_multibody_solver_values -q`
  reported `6 passed`. It keeps the solver-family row ordered before the
  closure-family row, verifies the capture hook mirrors semi-implicit
  residual-only, variational residual-only, variational dynamic closure solving,
  residual solve ratio, controls, case metadata, compact histories, and Replay
  timeline value/marker metadata, and now checks the comparison-axis panel text
  plus latest-signal ordering for residual-only residual, solved residual,
  solve ratio, lane residuals, solved tip error, maximum step time, and solver.
  The real row-35 workflow capture
  `pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 35 --workflow-end-row 35 --output-dir build/captures/rigid_multibody_solver_family_row_35_1781304535`
  completed with `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`, a
  nonblank 960x540 docked screenshot with 3410 unique colors, 71 PNG frames,
  and 72 scene-metrics events. The latest manifest event recorded
  `comparison_axis=multibody_integration_solve_policy_family`, held-fixed
  `contacts=off`, `closure_family=point`,
  `joint_family=three_revolute_links`, `chain_links=3`, `link_length=0.55`,
  `link_mass=0.55`, `initial_bend=0.28`, `gravity_scale=1.0`,
  `solver=world_multibody_integration_family`, `time_step_ms=5.0`, controls
  `executor_index=0.0`, `gravity_scale=1.0`,
  `solver_family_lanes=[semi_residual, variational_residual, variational_solved]`,
  residual-only residual about `0.7642`, solved residual clamped at `1e-12`,
  residual solve ratio about `7.642e11`, semi-implicit residual about
  `0.7631`, variational residual about `0.7642`, solved tip error about
  `6.66e-16`, and max step time about `0.0982 ms`.
- Latest spin/roll coupling follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `10 passed`. It keeps the sequential-impulse spin/roll row ordered
  after `rigid_friction_threshold` and before stack stability, verifies matched
  rolling, slip-to-roll spin-up, backspin scrub, the low-friction slip baseline,
  replay controls, sidecar/README order, capture-command drift, category, and
  panel coverage. The docked visual smoke command wrote a nonblank 960x540
  `rigid_spin_roll_coupling` screenshot plus 95 PNG frames with final contacts
  at 0.
- Latest in-viewer workflow-guide follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  reported `13 passed` before and after lint. It keeps the runner-owned
  `Rigid Workflow` guide synchronized with the PLAN-103 user questions,
  verifies previous/next route labels for first, middle, and final numbered
  rows, and confirms non-numbered World Rigid Body rows do not receive the
  guide panel. Docked visual smoke commands for `rigid_body`,
  `rigid_solver_compare`, and `rigid_loop_closure` each wrote nonblank 960x540
  screenshots plus 7 PNG frames with final contacts at 0. `pixi run lint`
  passed, and bounded `pixi run build` reported `ninja: no work to do`.
- Latest checklist/contact-body hardening follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  reported `14 passed` after lint. It turns the shared workflow panel into a
  per-row checklist with `Question`, `Try first`, `Look for`, and
  `Do not infer`, while keeping scope text synchronized with this sidecar. It
  also verifies `rigid_collision_query_options` exposes public
  `Contact.body_a/body_b` `CollisionBody` kind, validity, and cast results for
  rigid/rigid, rigid/link, same-multibody link/link, and cross-multibody
  link/link lanes. Docked visual smoke commands for `rigid_body`,
  `rigid_solver_compare`, `rigid_loop_closure`, and
  `rigid_collision_query_options` wrote nonblank 960x540 screenshots; the first
  three produced 7 PNG frames each and the collision-query row produced 23 PNG
  frames, all with final contacts at 0.
- Latest collision ignored-pair follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/simulation/test_world.py::test_simulation_collision_query_can_ignore_specific_pairs python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `5 passed` after lint. `rigid_collision_query_options` now exposes
  a persistent ignored-pair selector next to the existing body-kind include
  toggles, and its metrics separate baseline, option-filtered, pair-ignored,
  and active contacts. The docked visual capture
  `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui`
  wrote a nonblank 960x540 screenshot with 2751 unique RGB values, one dropped
  UI warmup frame, docked workspace detection, and 23 PNG frames.
- Latest collision-query capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds -q`
  reported `1 passed`. The row now exposes `SceneSetup.info["capture_metrics"]`
  with the same baseline, option-filtered, pair-ignored, active counts,
  ignored-pair state, and lane statuses that the panel and replay snapshots use.
  A short docked capture
  `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_collision_query_metrics`
  wrote eight scene-metrics events, mirrored the
  `rigid_rigid/rigid_link/same_links/cross_links` lane set into the manifest,
  and produced nonblank docked screenshot evidence with 2749 unique RGB values.
- Latest workflow-route navigation follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `9 passed`. It exposes `PanelContext.request_scene_switch(scene_id)`
  to Python scene panels and turns the `Rigid Workflow` previous/next route text
  plus the direct row selector into in-viewer scene-switch controls.
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden python/tests/unit/gui/test_gui_scene.py::test_gui_camera_and_run_helpers -q`
  reported `2 passed` after rebuilding dartpy; it directly verifies the public
  `request_scene_switch()` and `request_scene_replay()` lifecycle bindings and
  their requested-scene/pending-scene status fields.
- Latest workflow-search/restart follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden -q`
  reported `7 passed`. It adds a `PanelContext.request_scene_replay(scene_id)`
  binding for panel authors, a `Rigid Workflow` restart command for the current
  row, and a text filter that matches workflow row ids, questions, checklist
  text, and inspect signals before requesting an in-viewer scene switch. The
  filter includes explicit `NN/MM` row-id tokens so searches such as `15/36`
  select the intended workflow row.
- Latest stack-stability verification hardening follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `3 passed`. It keeps strict stability thresholds on the IPC lane
  while verifying both stack lanes expose finite height, height error,
  clearance, drift, speed, step time, populated per-lane histories, and finite
  top-x solver divergence. The docked visual smoke command for
  `rigid_stack_stability` wrote a nonblank 960x540 screenshot plus 23 PNG
  frames with final contacts at 0.
- Latest stack-stability capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered -q`
  reported `1 passed`. It verifies row identity, solver pair, executor,
  controls, per-solver metrics, top-x divergence, and history ranges exported by
  `SceneSetup.info["capture_metrics"]`. The broader workflow/doc drift guard
  with row ordering, viewer-title numbering, sidecar/README/capture-command
  drift checks, replay snapshot coverage, and high-value panel coverage reported
  `10 passed`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_stack_stability_metrics_1781231606`
  wrote a nonblank 960x540 screenshot plus 23 PNG frames and 24 scene-metrics
  events. The manifest recorded numeric ranges for sequential-impulse and IPC
  height error, speed, clearance, top drift, step timing, top-x divergence, time
  step, and world time.
- Latest contact-manipulation capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal -q`
  reported `1 passed`. It verifies row identity, solver pair, executor,
  controls, per-solver pusher/target metrics, travel divergence, and compact
  history ranges exported by `SceneSetup.info["capture_metrics"]`. The real
  docked capture
  `pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_manipulation_metrics_1781232293`
  wrote a nonblank 960x540 screenshot plus 71 PNG frames and
  72 scene-metrics events. The manifest recorded numeric ranges for
  sequential-impulse and IPC target travel, pusher gap, contact count, target
  speed, lateral drift, goal error, step timing, travel divergence, time step,
  and world time. The broader workflow/doc drift guard, lint, bounded build,
  and `git diff --check` were not run after the user's explicit
  stop/no-further-verification instruction.
- Latest kinematic-driver capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc -q`
  reported `1 passed`. It verifies row identity, solver scope, executor,
  controls, lane order, lane solver/friction modes, per-lane driver/box/slip
  metrics, and compact history ranges exported by
  `SceneSetup.info["capture_metrics"]`. The broader workflow/doc drift guard
  with row ordering, viewer-title numbering, sidecar/README/capture-command
  drift checks, replay snapshot coverage, contact-manipulation coverage, and
  high-value panel coverage reported `11 passed`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_driver_metrics_1781232929`
  wrote a nonblank 960x540 screenshot plus 71 PNG frames and
  72 scene-metrics events. The manifest recorded numeric ranges for IPC grip,
  IPC slip, and sequential-impulse caveat driver travel, box travel, slip,
  speed ratio, support gap, contact count, step timing, time step, and world
  time. `pixi run lint`, bounded default `pixi run build`, and
  `git diff --check` passed.
- Latest joint motor/limit capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort -q`
  reported `1 passed`. It verifies row identity, World multibody actuator
  scope, velocity/position/force controls, joint names, latest motor clamp,
  position-stop, force-gap, acceleration-gap, and compact history fields
  exported by `SceneSetup.info["capture_metrics"]`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_metrics_1781234533`
  wrote a nonblank 960x540 screenshot plus 71 PNG frames and
  72 scene-metrics events. The manifest recorded numeric ranges for motor
  speed/position/error, position-limit angle and error, capped/open force
  acceleration, acceleration gap, force-position gap, step timing, time step,
  and world time. The broader workflow/doc drift guard with row ordering,
  viewer-title numbering, sidecar/README/capture-command drift checks, replay
  snapshot coverage, kinematic-driver coverage, and high-value panel coverage
  reported `11 passed`. `pixi run lint`, bounded default `pixi run build`, and
  `git diff --check` passed.
- Latest joint motor/limit Replay timeline follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
  reported `6 passed`. It verifies the `Force travel gap` Replay timeline
  label, signal, velocity-clamp markers, position-stop markers, effort-cap
  markers, quiet-frame behavior, and the sidecar/README/capture-command drift.
  The maintained workflow capture budget was raised to 96 frames because the
  older 72-frame packet did not reach the position stop. The real docked
  capture
  `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_timeline_1781276821`
  wrote a nonblank 960x540 screenshot plus 95 PNG frames and
  96 scene-metrics events. The manifest recorded motor speed and expected speed
  `0.3` m/s, motor speed error `0.0`, position-limit angle equal to the upper
  stop (`0.35` rad), position-limit error `0.0`, force travel gap about
  `0.6984` m, force acceleration gap about `6.0` m/s^2, and matching history
  extrema. `pixi run lint` and `git diff --check` passed.
- Latest passive joint-parameter capture-metrics handoff:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q`
  reported `1 passed`. It verifies row identity, World multibody passive
  parameter scope, controls, lane order, spring/rest/damping, Coulomb friction,
  stiction/slip, armature, acceleration, energy, and compact history fields
  exported by `SceneSetup.info["capture_metrics"]`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_metrics_1781235045`
  wrote a nonblank 960x540 screenshot plus 119 PNG frames and
  120 scene-metrics events. The manifest recorded numeric ranges for
  spring/damped energy, damped-energy ratio, stiction/slip, armature
  acceleration and position gaps, step timing, time step, and world time. The
  skipped broader workflow/doc drift guard later reported `11 passed`;
  `pixi run lint`, bounded default `pixi run build`, and `git diff --check`
  passed.
- Latest passive joint-parameter Replay timeline handoff:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
  reported `6 passed`. It verifies the `Armature position gap` Replay timeline
  label, signal, damping energy-separation markers, Coulomb slip markers,
  armature-lag markers, quiet-frame behavior, and sidecar/README/capture-command
  drift. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_timeline_1781277900`
  wrote a nonblank 960x540 screenshot plus 119 PNG frames and
  120 scene-metrics events. The manifest recorded spring energy about
  `2.1583`, damped energy about `1.3403`, damped-energy ratio about `0.6210`,
  stiction position `0.0`, slip position about `0.1742` m, slip speed about
  `0.7200` m/s, armature acceleration gap about `2.25` m/s^2, and armature
  position gap about `0.2614` m. `pixi run lint` passed and
  `git diff --check` was clean.
- Latest screw-joint pitch capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation -q`
  reported `1 passed`. It verifies row identity, World multibody screw-pitch
  scope, controls, lane order, pitch multipliers, joint names, zero/fine/coarse
  and reverse pitch metrics, acceleration residuals, effective mass, mass
  matrix, and compact history fields exported by
  `SceneSetup.info["capture_metrics"]`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_metrics_1781235714`
  wrote a nonblank 960x540 screenshot plus 95 PNG frames and
  96 scene-metrics events. The manifest recorded numeric ranges for fine,
  coarse, reverse, and zero pitch lanes, pitch ratios, axial travel, near-zero
  acceleration residuals, step timing, time step, and world time. The broader
  workflow/doc drift guard reported `12 passed`; `pixi run lint`, bounded
  default `pixi run build`, and `git diff --check` passed.
- Latest screw-joint pitch Replay timeline follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
  reported `6 passed`. It verifies the `Coarse/fine travel gap` Replay
  timeline label, signal, pitch-spread markers, zero-pitch contrast markers,
  reverse-sign markers, quiet-frame behavior, and sidecar/README/capture-command
  drift. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_timeline_1781278553`
  wrote a nonblank 960x540 screenshot plus 95 PNG frames and
  96 scene-metrics events. The manifest recorded fine/coarse/reverse angle
  signs, fine/coarse/reverse axial travel, coarse/fine travel gap about
  `0.1122` m, and travel-per-radian values `0.28`, `0.56`, and `-0.28`.
  `pixi run lint` passed and `git diff --check` was clean.
- Latest multibody dynamics-terms capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms -q`
  reported `1 passed`. It verifies row identity, World multibody dynamics
  scope, controls, lane order, joint names, target/impulse patterns, serialized
  per-lane mass/coupling/conditioning/residual/torque/response metrics,
  heavy-versus-coupled ratios, and compact history fields exported by
  `SceneSetup.info["capture_metrics"]`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_metrics_1781236627`
  wrote a nonblank 960x540 screenshot plus 95 PNG frames and
  96 scene-metrics events. The manifest recorded numeric ranges for target
  acceleration, joint impulse, heavy distal mass scale, gravity scale,
  single/coupled/heavy mass, coupling, conditioning, response, torque,
  residual, step timing, time step, and world time. The broader workflow/doc
  drift guard reported `13 passed`; `pixi run lint`, bounded default
  `pixi run build`, and `git diff --check` passed.
- Latest multibody dynamics-terms Replay timeline follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
  reported `6 passed`. It verifies the `Response norm gap` Replay timeline
  label, signal, response-separation marker, off-diagonal-coupling marker,
  heavy-load-torque marker, quiet-frame behavior, and sidecar/README/capture
  command drift. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_timeline_1781279136`
  wrote a nonblank 960x540 screenshot plus 95 PNG frames and
  96 scene-metrics events. The manifest recorded coupled response norm about
  `15.46`, heavy response norm about `8.63`, heavy/coupled response ratio about
  `0.558`, heavy-minus-coupled torque norm about `18.14`, coupled/heavy
  off-diagonal coupling about `0.357`/`1.427`, max impulse residual about
  `2.7e-14`, and historical max coupling about `0.373`.
- Latest guided-replay timeline follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_default_timeline_without_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_ignores_malformed_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response -q`
  reported `5 passed`. The shared bottom `Replay` panel now accepts optional
  `replay_timeline` metadata for a saved-state value track and marker track.
  `rigid_solver_compare` pilots it by plotting position divergence and marking
  near-wall frames during replay scrubbing while keeping long-form guidance in
  the `Rigid Workflow` panel. The post-lint full Python sweep,
  `pixi run test-py`, reported `640 passed, 9 skipped`.
- Latest post-push replay-contract refresh:
  the branch was committed as `245095f1164` and pushed to
  `origin/feature/rigid-body-gui-visual-verification`. A full
  `pixi run test-py` first caught `rigid_ipc_stack_packet` missing shared
  replay-state callbacks; after adding capture/restore hooks for its controls
  and history, the focused replay/stack guard reported `2 passed`, and the
  refreshed full Python sweep reported `648 passed, 9 skipped`.
- Latest contact-policy replay timeline follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata -q`
  reported `5 passed`. `rigid_contact_solver_compare` now feeds the shared
  replay timeline a pose-divergence value track plus marker frames for
  coupled-contact counts or positive depth so saved-state scrubbing can jump to
  the frames where contact-policy differences are visible.
- Latest solver/contact capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy -q`
  reported `2 passed`. `rigid_solver_compare` and
  `rigid_contact_solver_compare` now export scene-owned capture metrics for
  controls, executor, case solver/method enums, world time, and divergence
  histories. Real docked captures wrote 24 and 72 scene-metrics events,
  respectively, with nonblank 960x540 docked screenshots.
- Latest executor-equivalence capture-metrics follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_executor_equivalence_keeps_parallel_rollout_matched -q`
  reported `1 passed`. `rigid_executor_equivalence` now exports scene-owned
  capture metrics for same-solver identity, executor pair, controls,
  per-executor contact counts and step timing, pose/velocity/contact
  divergence, and compact history ranges. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_executor_equivalence_metrics_1781228299`
  wrote a nonblank 960x540 screenshot, 23 PNG frames, and 24 scene-metrics
  events with manifest ranges for divergence, contact counts, per-executor step
  timing, time step, and world time. The workflow/doc drift guard reported
  `7 passed`.
- Latest friction-threshold capture-metrics follow-up:
  `rigid_friction_threshold` now exports scene-owned capture metrics for IPC
  solver identity, selected executor, angle/friction controls, threshold
  friction, controlled threshold delta, per-lane drift/speed/clearance/status,
  step timing, and compact history ranges. The focused
  workflow/doc drift guard reported `7 passed`, and the real docked capture
  `pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_friction_threshold_metrics_1781229799`
  wrote a nonblank 960x540 screenshot, 23 PNG frames, and 24 scene-metrics
  events with manifest ranges for per-lane distance, speed, clearance,
  friction, controlled threshold delta, step timing, time step, and world time.
- Latest spin/roll capture-metrics follow-up:
  `rigid_spin_roll_coupling` now exports scene-owned capture metrics for
  sequential-impulse solver identity, selected executor, contact-friction/
  launch/backspin controls, contact count, per-lane slip, roll ratio,
  spin-delta, travel, energy, friction, status, step timing, and compact history
  ranges. The focused workflow/doc drift guard reported `10 passed`, and the
  real docked capture
  `pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_spin_roll_metrics_1781230743`
  wrote a nonblank 960x540 screenshot, 95 PNG frames, and 96 scene-metrics
  events with manifest ranges for slip, roll ratio, spin delta, travel,
  low-friction coefficient, matched-roll energy, contact count, step timing,
  time step, and world time.
- Latest related-evidence route follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `7 passed`. The runner-owned `Rigid Workflow` panel now exposes
  `Related shelf` routes from `rigid_free_flight` to `floating_base`, from
  `rigid_multibody_dynamics_terms` to `articulated`, from
  `rigid_solver_compare` to `rigid_ipc_tunnel`, from
  `rigid_contact_solver_compare` to `diff_drone_liftoff`, from `contact` to
  `avbd_rigid_fixed_joint_contact`, from `rigid_joint_breakage` to the AVBD
  fixed/spherical break/reset rows, and from `rigid_joint_motor_limits` to the
  AVBD revolute/prismatic motor rows, while tests keep those targets
  registered and non-numbered.
- Latest related-evidence search follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q`
  reported `6 passed`. `Find row` now indexes related shelf names, scene ids,
  labels, and scope notes only after core row fields fail to match, so
  `floating_base`, `two-link arm`, `rigid_ipc_tunnel`, `contact gradient`,
  `avbd fixed contact`, `avbd spherical`, and `avbd prismatic` route directly
  to their related evidence scenes without demoting broad intent searches such
  as `contact` or `solver`.
  Related-only search results label the matched target scene and their tooltips
  explain the related shelf. The sidecar and README also document docked capture
  commands for every non-numbered related target, and tests keep those commands
  synchronized with the route table.
- Latest capture-first IPC stack packet follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_registry_has_scenes python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals -q`
  reported `12 passed`. The docked `rigid_ipc_stack_packet` capture produced a
  nonblank 960x540 screenshot plus 23 PNG frames, dropped one warmup frame, and
  kept docked UI detected with 3762 unique screenshot colors and RGB variance
  `3394.279`. A 24-step probe ended capture-first with min clearance
  `0.000267` m, top drift `0.000360` m, max speed `0.0493` m/s, last-step wall
  time `805.087` ms, and max observed wall time `3508.939` ms, confirming why
  this packet remains outside the live numbered workflow.
- Latest non-numbered no-tunneling metrics follow-up:
  `rigid_ipc_tunnel` now exports scene-owned capture metrics for the related
  Rigid IPC shelf route: row identity, no-tunneling scope, IPC solver label,
  launch speed, wall/box extents, wall clearance, through-wall margin, box
  velocity, contact count, step timing, barrier status, and compact history
  extrema. The focused guard
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_tunnel_reports_no_tunneling_metrics -q`
  reported `1 passed`. The real docked capture
  `pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_tunnel_metrics_1781240644`
  wrote a nonblank 960x540 screenshot, 23 PNG frames, 24 scene-metrics events,
  final contacts `0`, latest status `barrier-held`, minimum clearance about
  `1.22e-6` m, minimum through-wall margin about `0.500001` m, max wall
  crossing `0.0`, and world time `0.24` s.
  The related route/docs drift guard reported `12 passed`; `pixi run lint`,
  bounded default `pixi run build`, and `git diff --check` passed.
- Latest non-numbered contact-gradient metrics follow-up:
  `diff_drone_liftoff` now exports scene-owned capture metrics for the related
  Differentiable shelf route from `rigid_contact_solver_compare`: row identity,
  boxed-LCP contact-gradient scope, optimized/fallback status, target/rest
  height, playhead/current height, analytic versus complementarity-aware
  thrust/final-height/loss values, height/target-error/thrust gaps, and compact
  history summaries. The focused guard
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_diff_drone_liftoff_reports_contact_gradient_metrics -q`
  reported `1 passed`. The default-build docked capture
  `pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_diff_drone_liftoff_metrics_1781241455`
  wrote a nonblank 960x540 screenshot, 95 PNG frames, 96 scene-metrics events,
  row `diff_drone_liftoff`, status `fallback`, `optimized=false`, target
  height `1.5`, rest/current/final heights about `0.19995`, target error
  `1.30005`, zero height/thrust gaps, and top-level numeric ranges for the
  manifest. This fallback status is expected in the default capture because
  `DART_BUILD_DIFF=OFF`. The related route/docs drift guard reported
  `17 passed`; `pixi run lint`, bounded default `pixi run build`, and
  `git diff --check` passed.
- Latest AVBD related-route metrics follow-up:
  `avbd_rigid_fixed_joint_contact`,
  `avbd_rigid_spherical_breakable_joint`, `avbd_rigid_revolute_motor`, and
  `avbd_rigid_prismatic_motor` now export scene-owned capture metrics for the
  non-numbered AVBD shelf routes linked from `contact`,
  `rigid_joint_breakage`, and `rigid_joint_motor_limits`. The payloads report
  row identity, related source row, solver/scope or actuator label,
  fixed-contact offset/clearance/contact counts, spherical breakage anchor and
  orientation drift, revolute speed tracking, prismatic axis/drift tracking,
  and compact history extrema. The focused AVBD guard
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_avbd_fixed_joint_contact_demo_exercises_contact_path python/tests/integration/test_demos_cycle.py::test_avbd_revolute_motor_demo_drives_hinge python/tests/integration/test_demos_cycle.py::test_avbd_prismatic_motor_demo_drives_slider python/tests/integration/test_demos_cycle.py::test_avbd_rigid_spherical_breakable_joint_demo_resets_anchor_only -q`
  reported `4 passed` before the docs update. Real docked captures wrote
  71 PNG frames and 72 scene-metrics events for each route under
  `/tmp/dart_capture_avbd_fixed_joint_contact_metrics_1781242847`,
  `/tmp/dart_capture_avbd_spherical_breakable_metrics_1781242878`,
  `/tmp/dart_capture_avbd_revolute_motor_metrics_1781242920`, and
  `/tmp/dart_capture_avbd_prismatic_motor_metrics_1781242958`. Latest payloads
  included fixed contact count `3`, max fixed contact count `4`, spherical
  status `broken`, spherical `saw_broken=1`, revolute measured speed
  `1.2000000000000077` rad/s, and prismatic measured speed
  `0.8000000000000007` m/s with zero orthogonal drift. The focused AVBD plus
  related-route/docs guard reported `15 passed`; `pixi run lint`, bounded
  default `pixi run build`, and `git diff --check` passed.
- Latest World related-route metrics follow-up: `floating_base` and
  `articulated` now export scene-owned capture metrics for the non-numbered
  World Rigid Body shelf routes linked from `rigid_free_flight` and
  `rigid_multibody_dynamics_terms`. The payloads report row identity, related
  source row, floating-joint drift/spin speed and position, compact two-link
  arm shoulder/wrist speed, forearm height, damping controls, joint positions,
  and compact history extrema. The focused World related-route guard with
  runner/panel related-route coverage reported `7 passed`. Real docked
  captures wrote 71 PNG frames and 72 scene-metrics events for each route under
  `/tmp/dart_capture_floating_base_metrics_1781250001` and
  `/tmp/dart_capture_articulated_metrics_1781250002`. Latest payloads included
  `floating_base` linear speed about `1.0145`, angular speed `2.0`, body x
  about `0.725`, and world time about `0.72`, plus `articulated` shoulder speed
  about `0.643`, wrist speed about `0.583`, max joint speed about `0.643`,
  forearm height about `-0.0257`, and world time about `0.072`.
  `pixi run lint`, bounded default `pixi run build`, and `git diff --check`
  passed.
- Latest fundamental-row capture-metrics follow-up: the early numbered workflow
  rows `rigid_body_modes`, `rigid_free_flight`, `rigid_frame_hierarchy`,
  `rigid_external_loads`, `rigid_link_point_loads`,
  `rigid_timestep_sensitivity`, `rigid_restitution_ladder`, and
  `rigid_material_mixing` now export scene-owned capture metrics. The payloads
  report row identity, solver/executor or scope, user-facing controls,
  per-lane metrics, compact history extrema, and top-level manifest-friendly
  fields for mode flags, free-flight residuals, frame residuals, force/torque
  response, point-load lever-arm response, time-step error ratio, restitution
  rebound, and pair-material mixing. `rigid_kinematic_normal_push` now uses the
  shared capture-metrics info key rather than a literal string, and
  `rigid_joint_breakage` remains covered through the shared AVBD breakable
  joint builder. The focused affected-row guard
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_external_loads_scale_force_and_torque_response python/tests/integration/test_demos_cycle.py::test_rigid_free_flight_preserves_initial_state_diagnostics python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_restitution_ladder_orders_rebound_height python/tests/integration/test_demos_cycle.py::test_rigid_material_mixing_applies_pair_rules python/tests/integration/test_demos_cycle.py::test_rigid_link_point_loads_show_lever_arm_and_frame_semantics python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage -q`
  reported `9 passed`. Real docked captures under
  `/tmp/dart_capture_*_fundamental_metrics_1781245202` wrote 72
  scene-metrics events for `rigid_body_modes`, `rigid_external_loads`,
  `rigid_frame_hierarchy`, `rigid_link_point_loads`, and
  `rigid_material_mixing`, plus 96 events for `rigid_free_flight`,
  `rigid_restitution_ladder`, and `rigid_timestep_sensitivity`. All eight
  manifests reported docked first-frame visual evidence and row-matched latest
  metrics, including dynamic body speed, free-flight residuals, frame
  residuals, load response, point-load yaw response, material mixing fields,
  ordered rebound heights, and coarse/fine timestep error ratio. The related
  workflow/docs drift guard reported `10 passed`; `pixi run lint`, bounded
  default `pixi run build` with `DART safe jobs: 4`, and `git diff --check`
  passed.
- Focused behavior tests across the packet cover the ordered rigid workflow:
  default front door, body modes, free flight, external and point loads,
  time-step sensitivity, step diagnostics, contact-scale frame-budget
  diagnostics, restitution, material mixing, contact query options, collision
  casts, solver and executor comparison, contact-solver policy, friction
  threshold, spin/roll coupling, stack stability, pusher manipulation,
  kinematic driver and normal-push caveat behavior, fixed-joint preservation,
  AVBD-pinned joint
  breakage and reset lifecycle, one-DOF joints, joint motor/limit clamping,
  passive joint parameters, screw-joint pitch coupling, generalized multibody
  dynamics terms, link-origin Jacobian mapping, multibody solver-family routing,
  loop-closure family comparison, replay-control snapshots, category ordering,
  and panel coverage. The
  differentiable contact-gradient route is covered by the `diff_drone_liftoff`
  panel test, which now checks height, thrust, loss, thrust-gradient,
  analytic-loss histories, the public gradient-mode labels, and by the focused
  capture-metrics test above.
- Visual evidence: every numbered workflow row has a recorded nonblank
  960x540 `py-demo-capture --show-ui` run. The most recent follow-ups wrote a
  nonblank `rigid_joint_breakage` screenshot plus 47 PNG frames with final
  contacts at 0, a nonblank `rigid_collision_casts` screenshot plus 47 PNG
  frames with final contacts at 0, docked UI detected, 4286 unique colors, and
  RGB variance `[2507.382, 2770.902, 2914.942]`, a
  nonblank `rigid_contact_scale_budget` screenshot plus 71 PNG frames with
  controller contact points at 4, 16, and 36 for the one-, four-, and nine-box
  lanes and dense/single wall ratio `6.016` (RGB variance
  `[2502.671, 2667.095, 2860.303]`, 2151 unique colors), a nonblank
  `rigid_spin_roll_coupling` screenshot plus 95 PNG frames with final contacts
  at 0, a
  nonblank 960x540 `rigid_link_jacobian` workflow-row screenshot plus 95 PNG
  frames with final contacts at 0, 96 scene-metrics events, and 2138 unique
  colors, a nonblank `rigid_multibody_dynamics_terms` screenshot plus 95 PNG
  frames with final contacts at 0 (RGB variance
  `[2496.525, 2663.351, 2868.943]`, 2869 unique colors), a nonblank
  `rigid_multibody_solver_family` workflow-row screenshot plus 71 PNG frames
  with 72 scene-metrics events, final contacts at 0, and 3410 unique colors, a
  nonblank `rigid_loop_closure` workflow-row screenshot plus 71 PNG frames
  with 72 scene-metrics events, final contacts at 0, and 2446 unique colors, a
  nonblank `rigid_body_modes` screenshot plus 71 frames, a nonblank
  `rigid_screw_joint_pitch` screenshot
  plus 95 frames, a nonblank `rigid_contact_inspector` screenshot plus 23
  frames, a nonblank `rigid_material_mixing` screenshot plus 71 frames, and a
  nonblank
  `rigid_joint_passive_parameters` screenshot plus 119 frames.
- Local quality gates: `pixi run lint` passed after the stack-stability
  capture-metrics follow-up, the focused
  category/order/viewer-title/docs-count/sidecar/README/capture-command/stack/
  replay/panel guard reported `10 passed`, bounded `pixi run build` passed with
  `DART safe jobs: 5` and `ninja: no work to do`, and `git diff --check` was
  clean after the evidence update.
- Local count-drift gate: the focused navigator-count guard reported
  `4 passed`, and `pixi run lint` passed after adding the guard.
- Broad validation evidence: default `pixi run test-all` passed all 6 wrapper
  groups at pushed commit `fbbd5de0005`, and CUDA
  `pixi run -e cuda test-all` passed all 7 groups at the same implementation
  state on a host with an NVIDIA RTX 4080 Laptop GPU.

## Follow-Ups

- Add a no-tunneling-specific regression or side-by-side row only if the
  existing IPC scene family is not enough to cover the next user question.
- Keep fuller articulated arm/gripper manipulation deferred for now. A fresh
  public-API probe found that rigid-body joints are not supported by the IPC
  rigid-body solver, multibody link contacts do not expose material/friction
  controls, and the behaviorally useful scripted IPC two-jaw pinch settings run
  at hundreds of milliseconds per step. Revisit only after the public API can
  express a stable, interactive gripper without overclaiming IK, actuator
  dynamics, or link-material behavior.
- Keep taller or heavier rigid IPC stacks benchmark/capture-first before making
  them part of the default live GUI path. The four-box and six-box top-heavy
  stack packets are in the Rigid IPC shelf; future larger packets still need
  benchmark/capture evidence before any live-workflow claim.
- Keep related-evidence routes in the runner and this sidecar synchronized.
  They remain labelled as related shelf links rather than numbered workflow
  rows, so non-numbered scenes such as `floating_base`, `articulated`,
  `rigid_ipc_tunnel`, and `diff_drone_liftoff` do not look like new rigid
  workflow rows.
- Keep new rigid visual rows in this packet, `python/examples/demos/README.md`,
  and the ordered demo registry in sync.
- Keep motor/limit wording on the World multibody joint actuator path until
  rigid-body joint motors have equally stable public behavior.
- Add future point-force or impulse rows only if they answer a distinct,
  bounded visual question that the `rigid_external_loads` direct impulse lanes
  do not answer.
- Add a separate `rigid_energy_momentum` row only if it answers a distinct user
  question beyond the free-flight diagnostics, without duplicating the
  time-step, load, restitution, or solver-comparison rows.
- Add additional loop-closure rows only for distinct behavior beyond public
  POINT/DISTANCE/RIGID family selection, such as compliance or breakage.
- Keep `rigid_joint_breakage` AVBD-pinned and scoped to break-force,
  broken-state, and reset lifecycle; do not imply sequential-impulse or IPC
  parity.
- Keep `rigid_multibody_dynamics_terms` scoped to joint-space generalized
  dynamics, `rigid_link_center_of_mass` scoped to public `Link.center_of_mass`
  inertial offsets and gravity torque, and `rigid_link_jacobian` scoped to
  link-origin world-Jacobian velocity and wrench-transpose mapping. Do not fold
  arbitrary point-force, COM-Jacobian, contact-Jacobian, IK, or
  operational-space controller claims into those rows.
- Keep `rigid_material_mixing` scoped to DART 7 World pair-material ownership:
  effective restitution is `max(e_a, e_b)`, effective friction is
  `sqrt(mu_a * mu_b)`, swapped mixed pairs are the signal, and the row should
  not grow into an IPC restitution claim or a duplicate stick/slip threshold.
- Keep differentiable contact-gradient mode UX routed to the Differentiable
  shelf. `diff_drone_liftoff` covers clamping-contact saddle escape, and
  `diff_pre_contact_surrogate` covers the public
  `ContactGradientMode.PRE_CONTACT_SURROGATE` approaching-but-not-touching
  backward-only signal without expanding the numbered rigid workflow.
