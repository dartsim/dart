# PLAN-103 Sidecar: Rigid-Body Visual Verification

This packet owns the curated rigid-body `py-demos` visual-verification workflow
for PLAN-103. It records which scenes answer which user debugging questions,
how to regenerate visual evidence, and which automated tests keep the workflow
from drifting.

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
- direct rigid-body impulse, sleep/deactivation/island-state, and
  loop-closure compliance rows until public dartpy exposes those surfaces.

Current public API audit: `RigidBody` exposes force/torque accumulators plus
linear and angular momentum, `Link.apply_force()` exposes one-shot point-load
semantics, and `Multibody.compute_impulse_response()` exposes joint-space
impulse response. There is still no public direct rigid-body impulse surface,
no public sleep/wake or island activation surface, and no public loop-closure
compliance surface, so those candidate GUI rows remain deferred.

## Curated Workflow

In the interactive `py-demos` viewer, the first 36 **World Rigid Body** rows
use navigator titles with their workflow position and role, for example
`01/36 Baseline: World Rigid Body` and
`15/36 Solver family: Rigid Solver Compare`. The CLI `--list` output keeps the
stable scene titles and ids for scripts. These rows also receive a runner-owned
`Rigid Workflow` panel with the row's maintained user question, what to try
first, the main signals to inspect, the known scope/limitation, and the
previous/next numbered route, restart command, direct row selector, and
ranked row filter as scene-switch rows. The filter prioritizes row ids, scene
ids, labels, questions, and positive signals before scope caveats, so intent
searches such as `contact` or `solver` surface the relevant debugging rows
instead of early rows that only say what not to infer.

| Order | Scene id                         | User question                                                            | Solver(s)                        | Controls and diagnostics                                                                                                                                                                                                                                | Capture command                                                                                                      | Automated evidence                                                                                                                                                                                                              | Known limitation                                                                                           |
| ----- | -------------------------------- | ------------------------------------------------------------------------ | -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| 1     | `rigid_body`                     | What is the baseline DART 7 World rigid-body path?                       | Selectable rigid solver          | Solver, friction, restitution, reset, force drag, max speed, min height, kinetic energy, contact count, step profile timing, replay snapshots for baseline controls, and capture metrics.                                                               | `pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui`                      | `test_rigid_body_baseline_reports_restartable_first_run_diagnostics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                  | Baseline front door; focused edge cases stay in the specialized verifier rows.                             |
| 2     | `rigid_body_modes`               | Which body mode should I choose?                                         | Selectable rigid solver          | Solver, executor, gravity scale, force magnitude, kinematic speed, dynamic height/x, static drift, kinematic path error, mode flags, force norm, and step timing.                                                                                       | `pixi run py-demo-capture -- --scene rigid_body_modes --frames 72 --width 960 --height 540 --show-ui`                | `test_rigid_body_modes_compare_dynamic_static_kinematic_semantics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                     | Contact-free mode semantics row; contact-driven prescribed motion is routed later.                         |
| 3     | `rigid_free_flight`              | Do initial velocity, gravity, and spin evolve?                           | Sequential impulse               | Executor, launch speed, launch angle, gravity scale, spin speed, spin inertia ratio, path error, momentum residual, energy drift, spin ratios, contact count, and step timing.                                                                          | `pixi run py-demo-capture -- --scene rigid_free_flight --frames 96 --width 960 --height 540 --show-ui`               | `test_rigid_free_flight_preserves_initial_state_diagnostics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                           | No-contact initial-state row; not a load, restitution, contact, or solver row.                             |
| 4     | `rigid_frame_hierarchy`          | Where is a sensor/tool frame on a moving body?                           | World frame hierarchy            | Executor, body yaw speed, path radius, local offset/yaw, parent name, body/sensor world pose, world-transform residual, relative-transform residual, orientation error, and step timing.                                                                | `pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 --width 960 --height 540 --show-ui`           | `test_rigid_frame_hierarchy_tracks_body_fixed_frame`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                                   | Kinematics/frame row only; not a force, contact, sensor model, or solver-family claim.                     |
| 5     | `rigid_external_loads`           | How do external loads move and spin bodies?                              | Sequential impulse               | Executor, force magnitude, torque magnitude, heavy mass ratio, high inertia ratio, speed, acceleration versus expected, angular response, static drift, and step profile timing.                                                                        | `pixi run py-demo-capture -- --scene rigid_external_loads --frames 72 --width 960 --height 540 --show-ui`            | `test_rigid_external_loads_scale_force_and_torque_response`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                            | Contact-free zero-gravity accumulator row; not a point-force or impulse verifier.                          |
| 6     | `rigid_link_point_loads`         | Do point forces create lever-arm torque?                                 | Sequential impulse               | Executor, force magnitude, point offset, yawed frame angle, centered/world-point/pulse/double/world-frame/local-frame lanes, acceleration versus expected, yaw acceleration, displacement, and step timing.                                             | `pixi run py-demo-capture -- --scene rigid_link_point_loads --frames 72 --width 960 --height 540 --show-ui`          | `test_rigid_link_point_loads_show_lever_arm_and_frame_semantics`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                       | Contact-free one-shot Link.apply_force row; not persistent rigid-body accumulator behavior.                |
| 7     | `rigid_timestep_sensitivity`     | How does time-step size change free fall/contact?                        | Selectable rigid solver          | Solver, executor, base time step, gravity scale, matched fine/medium/coarse lanes, free-fall error, contact timing, clearance, error ratio, and step-profile timing.                                                                                    | `pixi run py-demo-capture -- --scene rigid_timestep_sensitivity --frames 96 --width 960 --height 540 --show-ui`      | `test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                   | Parameter-sensitivity row; not a solver correctness proof or exact contact threshold.                      |
| 8     | `rigid_step_diagnostics`         | Where does a World step spend time and memory?                           | Selectable rigid solver          | Solver, executor, single/contact/stack lanes, profile stage count, wall/stage time, top stage domain/acceleration/backend status, worker count, ECS counters, contact count, frame-scratch usage, and overflow/reset counters.                          | `pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 72 --width 960 --height 540 --show-ui`          | `test_rigid_step_diagnostics_reports_profile_and_memory_counters`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                      | Profiling may be compiled out; memory/contact diagnostics remain visible.                                  |
| 9     | `rigid_contact_scale_budget`     | How much contact fits in my frame budget?                                | Selectable rigid solver          | Solver, executor, frame budget, contact friction, one/four/nine-box contact workloads, wall time, per-contact cost, contact-point count, frame-scratch peak, entity/component counts, and dense/single ratio.                                           | `pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 72 --width 960 --height 540 --show-ui`      | `test_rigid_contact_scale_budget_orders_contact_loads`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                                 | Bounded live-GUI budget row; not a benchmark suite or heavy IPC stress packet.                             |
| 10    | `rigid_restitution_ladder`       | How does restitution change bounce height?                               | Selectable rigid solver          | Solver, executor, launch height, restitution scale, height, vertical velocity, contact count, rebound height, energy trend, and step profile timing.                                                                                                    | `pixi run py-demo-capture -- --scene rigid_restitution_ladder --frames 96 --width 960 --height 540 --show-ui`        | `test_rigid_restitution_ladder_orders_rebound_height`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                                  | Relative rebound diagnostic only; energy plots are not exact conservation claims.                          |
| 11    | `rigid_material_mixing`          | Which material owns bounce or friction response?                         | Sequential impulse               | Executor, impact speed, tangential speed, low/high restitution, low/high friction, swapped body/surface lanes, expected max restitution, geometric-mean friction, rebound, speed loss, and step timing.                                                 | `pixi run py-demo-capture -- --scene rigid_material_mixing --frames 72 --width 960 --height 540 --show-ui`           | `test_rigid_material_mixing_applies_pair_rules`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                                        | Pair-rule ownership row only; not an IPC restitution claim or incline stick/slip proof.                    |
| 12    | `rigid_contact_inspector`        | Which contact pairs and manifold fields exist?                           | Collision query                  | Shape pair, penetration, sphere/box, box/ground, plane/sphere, capsule/sphere, cylinder/sphere, mesh/sphere, compound/sphere, selected/total contacts, point, normal, depth, local points, shape indices, and capture metrics.                          | `pixi run py-demo-capture -- --scene rigid_contact_inspector --frames 24 --width 960 --height 540 --show-ui`         | `test_rigid_contact_inspector_reports_contact_manifolds`, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                | Query-focused row; not a solver-family benchmark or all-pairs native sandbox.                              |
| 13    | `rigid_collision_query_options`  | Which body-kind pairs does a query include?                              | Collision query options          | Rigid/rigid, rigid/link, same-multibody link/link, cross-multibody link/link lanes; include toggles, ignored-pair selector, baseline/option/active/filtered contacts, lane status, public `CollisionBody` kind/cast diagnostics, and shape indices.     | `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui`   | `test_rigid_collision_query_options_filter_body_kinds`, `test_simulation_collision_query_can_ignore_specific_pairs`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.    | Query-filter and persistent ignored-pair row only; not a shape-family manifold inspector.                  |
| 14    | `rigid_collision_casts`          | Where do rays and swept probes hit?                                      | Collision cast queries           | Ray lateral offset, all-hit toggle, swept-sphere radius, swept-capsule offset/radius/height, nearest/all ray targets, ray fractions, sphere/capsule time of impact, cast margins, first hit point/normal, hit-count histories, and capture metrics.     | `pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui`           | `test_rigid_collision_casts_report_nearest_all_and_swept_hits`, `test_continuous_capsule_cast_result`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture. | Public collision-cast query row; not a contact-solver, no-tunneling, or CCD time-step guarantee.           |
| 15    | `rigid_solver_compare`           | How do the rigid method families differ visually?                        | Sequential impulse and IPC       | Executor, launch speed, friction, restitution, speed, wall clearance, position divergence, step profile timing, and capture metrics.                                                                                                                    | `pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 --width 960 --height 540 --show-ui`            | `test_rigid_solver_compare_records_wall_response`, replay-control snapshot coverage, registry/category ordering, panel coverage, capture metrics, and visual smoke capture.                                                     | Generic thin-wall comparison; not the sole no-tunneling proof.                                             |
| 16    | `rigid_executor_equivalence`     | Does a parallel executor preserve the same physics?                      | Same solver in both worlds       | Physics solver, launch speed, friction, restitution, pose divergence, velocity divergence, contact-count delta, per-executor step timing, fallback executor label, and capture metrics.                                                                 | `pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 --width 960 --height 540 --show-ui`      | `test_rigid_executor_equivalence_keeps_parallel_rollout_matched`, panel/category coverage, capture metrics, and visual smoke capture inspected.                                                                                 | Same-solver executor-equivalence row; not a solver-family comparison.                                      |
| 17    | `rigid_contact_solver_compare`   | What changes when contact solver policy changes?                         | Contact solver policies          | Executor, launch speed, friction, restitution, initial tilt, contact count, penetration depth, analytic corner clearance, speed, energy, divergence, step profile timing, and capture metrics.                                                          | `pixi run py-demo-capture -- --scene rigid_contact_solver_compare --frames 72 --width 960 --height 540 --show-ui`    | `test_rigid_contact_solver_compare_records_coupled_contact_policy`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                    | Contact-policy row only; it does not compare IPC against sequential impulse.                               |
| 18    | `contact`                        | Do articulated links contact like rigid bodies?                          | World multibody link contact     | Executor, ground friction, ground restitution, drop height, slide speed, push speed, link drop/slide/pusher lanes, link/rigid contact counts, rebound, slide travel, target travel, step timing, and capture metrics.                                   | `pixi run py-demo-capture -- --scene contact --frames 144 --width 960 --height 540 --show-ui`                        | `test_rigid_link_contact_exercises_multibody_contact_response`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                        | Multibody-link contact row only; not a contact-impulse or compliance inspector.                            |
| 19    | `rigid_friction_threshold`       | Where is the inclined-ramp stick/slip boundary?                          | IPC                              | Executor, ramp angle, controlled friction, threshold, lane drift, lane speed, clearance, step profile timing, diagnostic plots, and capture metrics.                                                                                                    | `pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui`        | `test_rigid_friction_threshold_separates_stick_and_slip_lanes`, panel/category coverage, capture metrics, and visual smoke capture inspected.                                                                                   | Near-threshold behavior is tunable visual evidence, not an exact proof point.                              |
| 20    | `rigid_spin_roll_coupling`       | How does contact friction couple sliding and spin?                       | Sequential impulse               | Executor, contact friction, launch speed, backspin ratio, matched rolling/sliding/backspin/low-friction lanes, contact slip, roll ratio, spin change, travel, energy, contact count, step timing, and capture metrics.                                  | `pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui`        | `test_rigid_spin_roll_coupling_converts_slip_to_roll`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                 | Spin/rolling visual diagnostic only; no public rolling-resistance or torsional-friction parameter claim.   |
| 21    | `rigid_stack_stability`          | Does a top-heavy mass-ratio stack stay ordered?                          | Sequential impulse and IPC       | Executor, top mass ratio, friction, max speed, top drift, analytic clearance/overlap, height error, divergence, step profile, and capture metrics.                                                                                                      | `pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui`           | `test_rigid_stack_stability_keeps_ipc_stack_ordered`, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture inspected.                                                                          | Compact two-box stack only; taller IPC stacks remain benchmark/capture-first.                              |
| 22    | `rigid_contact_manipulation`     | Can a rigid pusher move an object through contact?                       | Sequential impulse and IPC       | Executor, pusher launch speed, table friction, pusher mass, target travel, pusher-target gap, contact/proximity, lateral drift, goal error, travel divergence, step profile timing, and capture metrics.                                                | `pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui`      | `test_rigid_contact_manipulation_pushes_target_toward_goal`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                           | Task-like pusher row only; not a full arm/gripper manipulation controller.                                 |
| 23    | `rigid_kinematic_driver`         | Does prescribed motion carry objects by contact?                         | IPC plus SI caveat               | Driver speed, grip friction, executor, IPC grip/slip lanes, sequential-impulse caveat lane, driver travel, box travel, slip, speed ratio, support gap, step timing, and capture metrics.                                                                | `pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui`          | `test_rigid_kinematic_driver_carries_box_with_ipc`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                    | Tangential kinematic-driver row only; normal pushing is routed to the next row.                            |
| 24    | `rigid_kinematic_normal_push`    | Can prescribed normal motion push a target?                              | IPC plus SI comparison           | Push speed, target mass, executor, IPC normal/heavy lanes, sequential-impulse lane, driver travel, target travel, analytic gap, penetration depth, contact count, speed ratio, and step timing.                                                         | `pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui`     | `test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                       | Normal kinematic-pusher caveat row only; IPC exposes penetration rather than a robust manipulation path.   |
| 25    | `rigid_fixed_joint`              | Does a fixed joint preserve its captured pose?                           | Sequential rigid joints          | Perturb/reset controls, relative offset error, relative orientation error, payload speed, angular speed, and histories.                                                                                                                                 | `pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui`               | `test_rigid_fixed_joint_verifier_restores_captured_transform`, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                           | Fixed-pose row only; no motor, limit, or break-force claims.                                               |
| 26    | `rigid_joint_breakage`           | What happens when a fixed joint breaks?                                  | AVBD rigid joints                | Fixed AVBD break-force threshold diagnostics, broken/intact state, connector color, captured-offset error, payload speed, broken-state history, and resettable breakage lifecycle.                                                                      | `pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui`            | `test_rigid_joint_breakage_marks_and_resets_breakage`, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                                   | AVBD-pinned breakage row; no user-editable threshold, sequential-impulse, or IPC break-force parity claim. |
| 27    | `rigid_distance_spring`          | How do rigid-body distance springs enforce rest length?                  | Sequential rigid AVBD rows       | Executor, initial stretch, gravity scale, unsprung/soft/stiff/off-center lanes, current length, stretch, payload speed, angular speed, connector visuals, and step timing.                                                                              | `pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui`           | `test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor`, replay-control snapshot coverage, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                  | World rigid-body distance-spring row only; IPC and multibody worlds reject this public API.                |
| 28    | `rigid_limited_joints`           | Do one-DOF joints keep only their free axis?                             | Sequential rigid joints          | Perturb/reset controls, hinge radius/z error, hinge yaw, slider orthogonal error, slider travel, and histories.                                                                                                                                         | `pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui`            | `test_rigid_one_dof_joint_verifier_preserves_locked_directions`, workflow ordering, panel/category coverage, capture metrics, and visual smoke capture.                                                                         | Revolute/prismatic constraint row only; public motor/limit behavior is out of scope.                       |
| 29    | `rigid_joint_motor_limits`       | Do joint motors and limits clamp commands?                               | World multibody joints           | Speed command, velocity limit, position stop, requested force, effort cap, motor speed, limit overshoot, acceleration gap, and histories.                                                                                                               | `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui`        | `test_rigid_joint_motor_limits_clamp_commands_and_effort`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                              | Multibody joint-actuator row; rigid-body joint motor behavior is not claimed.                              |
| 30    | `rigid_joint_passive_parameters` | Do passive joint parameters shape motion?                                | World multibody joints           | Executor, spring stiffness, rest position, damping, Coulomb friction, drive force, armature, acceleration versus expected, energy histories, stiction/slip status, and step timing.                                                                     | `pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui` | `test_rigid_joint_passive_parameters_order_passive_response`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                           | Contact-free passive-parameter row; no motor, limit, or contact-load claims.                               |
| 31    | `rigid_screw_joint_pitch`        | Does screw pitch couple rotation and translation?                        | World multibody joints           | Executor, pitch scale, gravity scale, moving mass, axial inertia, zero/fine/coarse/reverse pitch lanes, angle, axial travel, pitch ratio, effective mass, and expected-versus-actual acceleration.                                                      | `pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui`         | `test_rigid_screw_joint_pitch_couples_rotation_and_translation`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                        | Contact-free screw-pitch row; no contact, motor, limit, or loop-closure claims.                            |
| 32    | `rigid_multibody_dynamics_terms` | What do generalized dynamics terms mean?                                 | World multibody dynamics         | Executor, target acceleration, joint impulse, distal mass scale, gravity scale, single-hinge/coupled/heavy lanes, mass matrix diagonal/coupling/conditioning, inverse-dynamics residual, impulse residual, torque norm, response norm, and step timing. | `pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui`  | `test_rigid_multibody_dynamics_terms_expose_generalized_terms`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                         | Contact-free joint-space dynamics row; not a Cartesian point-force or COM-Jacobian claim.                  |
| 33    | `rigid_link_center_of_mass`      | How does a link center-of-mass offset change inertia and gravity torque? | World multibody inertial offsets | Executor, COM offset, gravity scale, link mass, high-inertia multiplier, centered/+X/-X/high-inertia lanes, gravity torque, mass matrix, hinge acceleration, expected acceleration, COM marker position, energy, and step timing.                       | `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui`       | `test_rigid_link_center_of_mass_offsets_gravity_torque`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                                | Link inertial-offset row only; not arbitrary-point Jacobians, contact, IK, or operational-space control.   |
| 34    | `rigid_link_jacobian`            | What does a link Jacobian map?                                           | World multibody kinematics       | Motion speed, elbow phase, wrench force, wrench angle, wrench moment, link-origin world/body Jacobian gap, `J qdot` twist, finite-difference velocity error, `J.T wrench` torque, and power error.                                                      | `pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui`             | `test_rigid_link_jacobian_maps_link_origin_twist_and_wrench`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                           | Link-origin kinematic/wrench map only; not arbitrary point, COM, contact, IK, or OSC.                      |
| 35    | `rigid_multibody_solver_family`  | Which multibody integration family supports solves?                      | Multibody solver families        | Executor, gravity scale, semi-implicit residual-only, variational residual-only, variational solved lanes, closure residual, tip error, tip height, joint speed, residual solve ratio, and step timing.                                                 | `pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui`   | `test_rigid_multibody_solver_family_routes_solved_closures`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                            | Solver-family routing row; closure family selection remains in the next row.                               |
| 36    | `rigid_loop_closure`             | Which loop-closure family should I use?                                  | Variational rigid multibody      | Executor, gravity scale, POINT/DISTANCE/RIGID family lanes, residual-only versus solved policies, closure residual, tip error, distance error, orientation residual, residual ratio, joint speed, and step timing.                                      | `pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui`              | `test_rigid_loop_closure_compares_closure_families`, replay-control snapshot coverage, workflow ordering, panel/category coverage, and visual smoke capture.                                                                    | Public-family comparison row; not a compliance, breakage, or distance-family solver sweep.                 |

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
the focused IPC no-tunneling view:

```bash
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui
```

## Differentiable Contact-Gradient Route

Do not add a duplicate numbered rigid row for contact-gradient modes in this
slice. The existing `diff_drone_liftoff` scene already answers the next user
question after forward contact debugging: "why does my contact optimization
stall even though the collision response looks right?" It uses a rigid `World`
with boxed-LCP contact and shows `ContactGradientMode.ANALYTIC` stalling at the
clamping saddle while `ContactGradientMode.COMPLEMENTARITY_AWARE` escapes. Keep
it in the Differentiable shelf so the mode distinction is explicit, but route
rigid users there from the workflow docs.

```bash
pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui
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
| `rigid_contact_solver_compare`   | `diff_drone_liftoff`                   | Differentiable              | Related shelf: Differentiable / diff_drone_liftoff - contact-gradient route                                      | Analytic vs complementarity-aware clamping-contact optimization; not a solver row.                                       |
| `contact`                        | `avbd_rigid_fixed_joint_contact`       | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_fixed_joint_contact - fixed-joint contact route          | Variational fixed-joint/contact capability scene; not a World contact-policy comparison.                                 |
| `rigid_joint_breakage`           | `avbd_rigid_breakable_joint`           | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_breakable_joint - free-rigid fixed break/reset           | Dedicated free-rigid fixed break/reset row; not sequential-impulse or IPC parity evidence.                               |
| `rigid_joint_breakage`           | `avbd_rigid_spherical_breakable_joint` | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_spherical_breakable_joint - spherical anchor break/reset | Dedicated free-rigid spherical anchor break/reset row; orientation remains intentionally free.                           |
| `rigid_joint_motor_limits`       | `avbd_rigid_revolute_motor`            | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_revolute_motor - free-rigid hinge motor                  | AVBD free-rigid revolute velocity motor; not a World multibody motor/limit comparison.                                   |
| `rigid_joint_motor_limits`       | `avbd_rigid_prismatic_motor`           | AVBD Rigid Constraints (sx) | Related shelf: AVBD Rigid Constraints (sx) / avbd_rigid_prismatic_motor - free-rigid slider motor                | AVBD free-rigid prismatic velocity motor; not a World multibody motor/limit comparison.                                  |

Capture every non-numbered related-evidence route with the docked UI visible:

```bash
pixi run py-demo-capture -- --scene floating_base --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene articulated --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui
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

| Scene id                 | User question                                                                             | Signals                                                                                                                                          | Capture command                                                                                           | Scope note                                                                                          |
| ------------------------ | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `rigid_ipc_stack_packet` | Can a four-box IPC stack stay separated, ordered, and finite beyond the live demo budget? | Friction, box count, frame-budget threshold, min clearance, contact count, top drift, height error, max speed, wall time, and benchmark pointer. | pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui | Capture-first stress packet; not a numbered workflow row and not a solver-performance parity claim. |

The matching benchmark owner remains `bm_rigid_ipc_solver`; use its
`BM_RigidWorldStep_SequentialImpulse` and `BM_RigidWorldStep_Ipc` rows for
same-scene per-step throughput tracking before promoting any heavier IPC stack
into the live workflow.
The capture helper also stores scene-owned metrics for rows and packets that
expose `SceneSetup.info["capture_metrics"]`; `rigid_step_diagnostics` and
`rigid_contact_scale_budget` summarize their profiling, memory, contact, and
budget metrics into `manifest.json` as first/latest events, per-key presence
counts, and top-level numeric ranges. The `contact` row uses the same hook for
multibody-link drop/slide/pusher lane summaries, while `rigid_ipc_stack_packet`
mirrors clearance, contact, drift, height, wall-time, frame-budget, and
benchmark values through the same schema.

## Regenerating Visual Evidence

Run the capture commands in the workflow table from the repository root. The
helper prints the output directory, writes PNG frames, rejects blank captures,
and applies Linux software-Mesa defaults so the same command works on headless
dev hosts. Visual artifacts are intentionally not checked in.

For a quick curated refresh:

```bash
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
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui
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
  `RigidBody.apply_force()`, `RigidBody.apply_torque()`, rigid-body
  linear/angular momentum accessors, `Link.apply_force(..., point, ...)`, and
  `Multibody.compute_impulse_response()`, but no public direct rigid-body
  impulse surface, sleep/wake or island activation surface, or loop-closure
  compliance surface. The drift guard
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
  `pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_body_metrics_1781224228`
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
- Latest link-Jacobian follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_jacobian_maps_link_origin_twist_and_wrench python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the link-origin Jacobian row ordered after
  generalized dynamics terms and before multibody solver-family routing,
  verifies `get_world_jacobian(link) @ qdot` against finite-difference
  link-origin velocity, verifies `get_world_jacobian(link).T @ wrench` power
  consistency, replay controls, sidecar/README order, capture-command drift,
  category, and panel coverage.
- Latest loop-closure family follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps one loop-closure workflow row but expands the
  row from POINT-only endpoint closure into POINT, DISTANCE, and RIGID public
  families crossed with residual-only and solved policies.
- Latest body-mode follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the new early workflow row ordered after
  `rigid_body`, verifies dynamic integration under gravity/force, static no
  drift, kinematic prescribed-path tracking, replay controls, sidecar/README
  order, capture-command drift, category, and panel coverage.
- Latest multibody solver-family follow-up:
  `PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `8 passed`. It keeps the solver-family row ordered before the
  closure-family row, verifies semi-implicit and variational residual-only
  closure diagnostics, variational dynamic closure solving, residual solve
  ratio, replay controls, sidecar/README order, capture-command drift,
  category, and panel coverage.
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
  `avbd fixed contact`, `avbd spherical`, and `avbd prismatic` route to their
  numbered source rows without demoting broad intent searches such as
  `contact` or `solver`.
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
  analytic-loss histories, and the public gradient-mode labels.
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
  nonblank 960x540 `rigid_link_jacobian` screenshot plus 95 PNG frames with
  final contacts at 0 (RGB variance `[2483.852, 2653.334, 2866.583]`, 2721
  unique colors), a nonblank `rigid_multibody_dynamics_terms` screenshot plus 95 PNG
  frames with final contacts at 0 (RGB variance
  `[2496.525, 2663.351, 2868.943]`, 2869 unique colors), a nonblank
  `rigid_multibody_solver_family` screenshot plus 71 PNG frames with final
  contacts at 0 (RGB variance `[2505.408, 2673.722, 2866.646]`, 3407 unique
  colors), a nonblank `rigid_body_modes` screenshot plus 71 frames, a nonblank
  `rigid_loop_closure` screenshot plus 71 frames, a nonblank
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
- CUDA evidence: `pixi run -e cuda test-all` passed all 7 groups on a host with
  an NVIDIA RTX 4080 Laptop GPU before later Python demo replay, scene, focused
  test, and docs follow-ups. Re-run CUDA validation before merging if a later
  slice touches CUDA-facing code.

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
  them part of the default live GUI path. The first four-box stack packet is in
  the Rigid IPC shelf; future larger packets still need benchmark/capture
  evidence before any live-workflow claim.
- Keep related-evidence routes in the runner and this sidecar synchronized.
  They remain labelled as related shelf links rather than numbered workflow
  rows, so non-numbered scenes such as `floating_base`, `articulated`,
  `rigid_ipc_tunnel`, and `diff_drone_liftoff` do not look like new rigid
  workflow rows.
- Keep new rigid visual rows in this packet, `python/examples/demos/README.md`,
  and the ordered demo registry in sync.
- Keep motor/limit wording on the World multibody joint actuator path until
  rigid-body joint motors have equally stable public behavior.
- Add future point-force or impulse rows only if the public API supports a
  distinct, bounded visual question that the accumulator row does not answer.
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
- Keep differentiable contact-gradient mode UX routed to the existing
  `diff_drone_liftoff` scene unless users need a distinct rigid workflow row
  with additional public controls such as pre-contact surrogate mode.
