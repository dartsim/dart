# Rigid-Body Visual Verification - Dev Task

## Current Active Snapshot - 2026-06-12 Capture Metric Ownership

The latest continuation resumed from pushed full-stop handoff
`c552ce83e2b` and hardened the capture-evidence contract for the rigid visual
workflow. A live audit showed all 36 numbered rows already exposed capture
hooks, but two related routes did not identify their owning numbered source row
and the capture-first IPC stack packet did not identify its own row in the
metrics payload.

Code checkpoint before this evidence-refresh docs edit:
`601845c4197` (`Harden rigid visual capture metric ownership`).

Current local slice: self-describing capture metrics for every numbered,
related, and capture-first rigid visual route. The checkpoint touches:

- `python/examples/demos/scenes/rigid_ipc_tunnel.py`
- `python/examples/demos/scenes/rigid_ipc_stack_packet.py`
- `python/examples/demos/scenes/avbd_rigid_breakable_joint.py`
- `python/examples/demos/scenes/rigid_joint_breakage.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed:

- `rigid_ipc_tunnel` capture metrics now include
  `related_source_row: rigid_solver_compare`.
- `avbd_rigid_breakable_joint` capture metrics now include
  `related_source_row: rigid_joint_breakage`, while the numbered
  `rigid_joint_breakage` wrapper suppresses that field so its row identity
  remains direct.
- `rigid_ipc_stack_packet` capture metrics now include
  `row: rigid_ipc_stack_packet`.
- `test_rigid_visual_routes_publish_self_describing_capture_metrics` now
  guards that every numbered workflow row has a callable capture hook and row
  identity, every related route records its source row, and every capture-first
  IPC packet records row identity plus `capture_first`.

Validation collected so far:

- Focused route ownership guard:
  `test_rigid_visual_routes_publish_self_describing_capture_metrics`,
  `test_rigid_visual_workflow_related_evidence_routes_are_valid`,
  `test_rigid_ipc_tunnel_reports_no_tunneling_metrics`,
  `test_rigid_ipc_stack_packet_reports_capture_first_metrics`,
  `test_rigid_joint_breakage_marks_and_resets_breakage`, and
  `test_avbd_breakable_joint_demo_marks_and_resets_joint` reported
  `6 passed` before and after lint.
- Real docked capture evidence under
  `/tmp/dart_capture_metric_ownership_1781249077`:
  - `rigid_ipc_tunnel` wrote 23 PNG frames and 24 scene-metrics events; the
    screenshot was docked and nonblank with 2524 unique RGB values. Latest
    metrics recorded row `rigid_ipc_tunnel`, related source row
    `rigid_solver_compare`, status `barrier-held`, min tunnel margin about
    `0.500001`, max wall crossing `0.0`, and world time about `0.24`.
  - `rigid_ipc_stack_packet` wrote 23 PNG frames and 24 scene-metrics events;
    the screenshot was docked and nonblank with 2658 unique RGB values. Latest
    metrics recorded row `rigid_ipc_stack_packet`, `capture_first=true`,
    benchmark owner `bm_rigid_ipc_solver`, status `capture-first`, world time
    about `0.10`, and last step time about `674.9` ms.
  - `avbd_rigid_breakable_joint` wrote 71 PNG frames and 72 scene-metrics
    events; the screenshot was docked and nonblank with 2721 unique RGB values.
    Latest metrics recorded row `avbd_rigid_breakable_joint`, related source
    row `rigid_joint_breakage`, status `broken`, `saw_broken=1.0`, payload
    release distance about `0.365`, and world time about `0.288`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate next step:

1. Continue the next bounded rigid visual-verification slice, or push/open the
   PR only after explicit user approval. Do not comment, re-trigger CI, or
   otherwise mutate GitHub state without that approval.

## Critical Stop Handoff - 2026-06-12 Full Stop

The user explicitly stopped all further work and requested handoff docs only,
with no further verification. Fresh sessions should start from this boundary
and not continue implementation, validation, pushing, PR creation, comments, CI
retries, or other GitHub mutation unless the user explicitly asks for that exact
action.

- Branch at handoff edit: `feature/rigid-body-gui-visual-verification`.
- Worktree was clean before the docs handoff edit.
- Local checkpoint before this docs-only handoff: `5127bf7e6c73`
  (`Refresh rigid visual verification PR readiness`).
- Upstream checkpoint before this docs-only handoff: `4a2fb7a0714e`
  (`Refresh rigid visual verification stop handoff`).
- The branch was ahead of origin by three commits:
  - `5127bf7e6c7` `Refresh rigid visual verification PR readiness`
  - `22d1a02de55` `Attach shared replay to related rigid scenes`
  - `7c853bee1af` `Expose fundamental rigid workflow capture metrics`
- Latest checked GitHub state: no PR associated with this branch.
- No tests, lint, build, captures, `git diff --check`, or other verification
  were run after this docs-only handoff edit. Do not infer fresh verification
  for the handoff docs commit itself.
- If explicitly asked to publish next, push the branch and use `PR_DRAFT.md`.

## Current Active Snapshot - 2026-06-11 Local PR-Readiness Refresh

After the shared Replay follow-up checkpoint, the branch was refreshed against
origin for local PR readiness without pushing or mutating GitHub state.
`origin/main` is already merged into the branch, no local merge commit was
needed, bounded `pixi run build` passed with `DART safe jobs: 2` and
`ninja: no work to do`, and full `pixi run test-py` passed with
`950 passed, 10 skipped` using `DART safe jobs: 1`. The latest checked GitHub
state still reports no PR associated with this branch.

## Current Active Snapshot - 2026-06-11 Shared Replay Follow-Up

The latest continuation lands a local checkpoint named
`Attach shared replay to related rigid scenes` after starting from
`7c853bee1af8` (`Expose fundamental rigid workflow capture metrics`). It closes
the shared Replay panel gap exposed by `pixi run test-py`: `articulated`,
`floating_base`, `avbd_rigid_revolute_motor`,
`avbd_rigid_prismatic_motor`, and `rigid_ipc_tunnel` now mark their custom
`pre_step` callbacks as replay-stateless and provide `bridge.sync` as the
shared replay sync hook. The focused shared-replay invariant, replay-gap audit,
full Python suite, `pixi run lint`, and `git diff --check` passed for this
slice. No PR is associated with this branch in the latest checked GitHub state,
and no push or GitHub mutation has been performed for this follow-up.

## Current Active Snapshot - 2026-06-11 Fundamental Workflow Metrics

The latest continuation resumed from pushed handoff commit `4a2fb7a0714` and
added local scene-owned capture metrics to the early numbered World Rigid Body
fundamental rows. The local slice covers `rigid_body_modes`,
`rigid_free_flight`, `rigid_frame_hierarchy`, `rigid_external_loads`,
`rigid_link_point_loads`, `rigid_timestep_sensitivity`,
`rigid_restitution_ladder`, and `rigid_material_mixing`; it also normalizes
`rigid_kinematic_normal_push` to use the shared capture-metrics info key. The
focused affected-row guard reported `9 passed` so far, and inspected docked
captures under `/tmp/dart_capture_*_fundamental_metrics_1781245202` wrote
scene metrics for all eight rows. The related docs drift guard reported
`10 passed`; `pixi run lint`, bounded `pixi run build`, and `git diff --check`
passed locally before commit.

## Critical Stop Handoff - 2026-06-11 World Related Metrics

The user explicitly stopped further work and requested handoff-only docs with
no further verification. Fresh sessions should start from this boundary and not
continue implementation unless the user explicitly resumes it.

- Branch at handoff edit: `feature/rigid-body-gui-visual-verification`.
- Worktree was clean before the docs handoff edit.
- Local code checkpoint before this docs-only handoff: `7fb9163794d`
  (`Expose World related route capture metrics`).
- Origin at that moment: `550b37d8b68`
  (`Refresh rigid visual verification handoff`).
- The branch was ahead of origin by one code/docs checkpoint.
- Last checked GitHub state: no PR associated with this branch.
- No tests, lint, build, captures, or `git diff --check` were run after this
  handoff-only docs edit. Do not infer fresh verification for the handoff docs
  commit itself.
- This docs-only handoff commit is intended to be pushed immediately at the
  user's request, without a new verification pass.

## Current Status

- [x] Local PR-readiness refresh: fetched origin, confirmed `origin/main` is
      already merged, confirmed no PR is associated with the branch, ran
      bounded `pixi run build` (`DART safe jobs: 2`, `ninja: no work to do`),
      and ran full `pixi run test-py` (`950 passed, 10 skipped`, `DART safe
jobs: 1`). No push or GitHub mutation was performed.
- [x] Capture metric ownership hardening: `rigid_ipc_tunnel`,
      `avbd_rigid_breakable_joint`, and `rigid_ipc_stack_packet` now publish
      capture metrics with the missing related-source or row identity fields,
      and a registry-level invariant keeps every numbered rigid row,
      related-evidence route, and capture-first IPC packet self-identifying.
      The focused six-test route ownership guard reported `6 passed` before
      and after lint; real docked captures for the three affected scenes wrote
      nonblank screenshots, PNG frame sequences, and scene-metrics sidecars
      with the expected row/source ownership fields; `pixi run lint` and
      `git diff --check` passed.
- [x] Shared Replay panel follow-up: `articulated`, `floating_base`,
      `avbd_rigid_revolute_motor`, `avbd_rigid_prismatic_motor`, and
      `rigid_ipc_tunnel` now publish `replay_sync` plus
      `replay_live_step_is_stateless`, allowing the registered-scene replay
      invariant to attach the shared bottom `Replay` panel to every
      replay-capable non-opt-out world scene. The pre-stop focused invariant
      passed, the replay-gap audit printed `[]`, and full `pixi run test-py`
      reported `950 passed, 10 skipped`; the post-doc-refresh focused
      invariant, `pixi run lint`, and `git diff --check` also passed.
- [x] Recon: existing `py-demos` are the primary growing example surface.
- [x] First solver-comparison slice: `rigid_solver_compare` runs sequential
      impulse and rigid IPC side by side in one Python GUI scene with
      scene-owned capture metrics for solver family, controls, cases, and
      position divergence.
- [x] Executor-equivalence slice: `rigid_executor_equivalence` runs matched
      rigid Worlds side by side with the same physics solver and sequential vs
      parallel executors.
- [x] Executor-equivalence capture-metrics follow-up:
      `rigid_executor_equivalence` now publishes same-solver identity,
      sequential/parallel executor labels, controls, per-executor contact
      counts and step timing, pose/velocity/contact divergence, fallback status,
      and compact history ranges through the capture hook. The focused
      executor guard reported `1 passed`; the real docked 24-frame capture
      wrote 24 scene-metrics events and numeric ranges for divergence, contact
      counts, per-executor step time, and world time. The workflow/doc drift
      guard reported `7 passed`.
- [x] Friction-threshold capture-metrics follow-up:
      `rigid_friction_threshold` now publishes IPC
      solver identity, executor, angle/friction controls, threshold delta,
      per-lane drift/speed/clearance/friction/status, step timing, and compact
      history ranges through the capture hook. The focused workflow/doc drift
      guard reported `7 passed`; the real docked 24-frame capture wrote
      24 scene-metrics events under
      `/tmp/dart_capture_friction_threshold_metrics_1781229799`; the lint gate
      passed; and a bounded default build reported `ninja: no work to do`.
- [x] Contact-solver policy slice: `rigid_contact_solver_compare` runs matched
      Worlds with the same rigid-body solver and sequential-impulse vs boxed-LCP
      contact methods on a tilted multi-contact plank with scene-owned capture
      metrics for contact policy, controls, cases, and pose divergence.
- [x] Link-contact slice: `contact` is now the numbered
      `Rigid Link Contact` row, showing multibody links dropping onto ground,
      friction-sliding, and pushing a rigid target through the public World
      contact path.
- [x] Validation: default focused guards, docked `contact` visual capture,
      `pixi run lint`, bounded default `pixi run build`, `git diff --check`,
      and CUDA-host `pixi run -e cuda test-all` all pass for the current
      link-contact follow-up.
- [x] Capture-evidence hardening: `py-demo-capture` manifests now record
      requested dimensions, converted frame count, and screenshot/first
      UI-ready-frame statistics: nonzero pixels, unique RGB count, RGB and
      luminance variance, and docked-workspace detection. `pixi run test-py`
      reported `636 passed, 9 skipped` with the manifest schema guard.
- [x] Capture-metrics hardening: Python scenes can expose
      `SceneSetup.info["capture_metrics"]`; `py-demo-capture` records the
      per-frame `scene_metrics.jsonl` sidecar and summarizes scene-owned
      physics/runtime metrics into `manifest.json` as first/latest events,
      per-key presence counts, and numeric ranges. The focused capture/runner
      guard reported `25 passed`; the runtime-row guard reported `2 passed`
      after adding metrics for `rigid_step_diagnostics` and
      `rigid_contact_scale_budget`, and short real captures for both rows wrote
      eight scene-metrics events apiece.
- [x] Baseline capture-metrics follow-up: the default `rigid_body` front door
      now publishes solver, material controls, dynamic body count, world time,
      current diagnostics, and history ranges through the same capture hook. The
      focused baseline guard reported `1 passed`; the real docked capture wrote
      24 scene-metrics events.
- [x] Link-contact capture-metrics follow-up: the numbered `contact` row now
      publishes sequential-impulse solver identity, multibody-link contact
      scope, executor/material/drop/slide/push controls, current lane metrics,
      contact body kinds, and compact history ranges through the same capture
      hook. The focused link-contact guard reported `1 passed`; the real docked
      144-frame capture wrote 144 scene-metrics events and covered all three
      lane histories.
- [x] Contact-inspector capture-metrics follow-up:
      `rigid_contact_inspector` now publishes selected-pair `World.collide()`
      manifold fields through the capture hook: row, collision-query scope,
      pair/penetration controls, total/selected contact counts, depth ranges,
      representative point/normal/local points, shape indices, and compact
      history ranges. The focused manifold guard reported `1 passed`; the real
      docked 24-frame capture wrote 24 scene-metrics events and top-level
      numeric ranges for contact counts, depths, world time, and shape indices.
      The focused workflow/doc drift guard reported `4 passed`; `pixi run lint`,
      bounded default `pixi run build`, and `git diff --check` all passed.
- [x] Current pushed handoff refresh: runtime-code checkpoints
      `7ef7a96dda0` (`Expose rigid collision cast capture metrics`),
      `b54c4f5dba3` (`Expose rigid executor equivalence capture metrics`), and
      `a47d52b7aea` (`Hand off rigid friction threshold capture metrics`) are
      followed by a local docs/validation completion for the
      `rigid_friction_threshold` capture-metrics slice and a pushed
      `rigid_spin_roll_coupling` handoff. That handoff instruction is
      satisfied; the current continuation is local. Future pushes, PR creation,
      comments, review replies, CI retriggers, or other GitHub mutations still
      require explicit approval.
- [x] Solver/contact comparison capture-metrics follow-up:
      `rigid_solver_compare` and `rigid_contact_solver_compare` now publish the
      same capture hook so manifests preserve method-family/contact-policy case
      names, solver/method enums, controls, world time, executor, and
      divergence histories. The focused two-test guard reported `2 passed`; real
      docked captures wrote 24 and 72 scene-metrics events respectively.
- [x] Numbered workflow capture hardening: a real docked
      `rigid_contact_scale_budget` capture now verifies the numbered workflow
      path produces nonblank docked UI artifacts and `scene_metrics` lanes, and
      a factory-composition guard keeps the injected `Rigid Workflow`, scene,
      and `Replay` panels together.
- [x] Material-response slice: `rigid_restitution_ladder` shows matched
      low/medium/high restitution bounce lanes with solver/executor, launch
      height, rebound, contact, energy-trend, and timing diagnostics.
- [x] Pair-material slice: `rigid_material_mixing` shows swapped
      body/surface ownership lanes for DART 7 World rigid contact material
      rules: `max(restitution)` and `sqrt(friction product)`.
- [x] Second friction slice: `rigid_friction_threshold` turns the inclined-ramp
      stick/slip boundary into below-threshold, controlled, and above-threshold
      IPC lanes.
- [x] Spin/roll coupling slice: `rigid_spin_roll_coupling` shows matched
      rolling, no-spin sliding, backspin scrub, and low-friction slip lanes with
      contact-slip, roll-ratio, spin-change, travel, energy, contact-count, and
      step-timing diagnostics.
- [x] Spin/roll coupling capture-metrics follow-up:
      `rigid_spin_roll_coupling` now publishes sequential-impulse solver
      identity, executor, contact-friction/launch/backspin controls, contact
      count, per-lane slip/roll-ratio/spin-delta/travel/energy/friction/status,
      step timing, and compact history ranges through the capture hook. The
      focused workflow/doc drift guard reported `10 passed`; the real docked 96-frame
      capture wrote 96 scene-metrics events under
      `/tmp/dart_capture_spin_roll_metrics_1781230743`; `pixi run lint`
      completed before the final handoff edit. No later build, capture, test,
      or `git diff --check` was run because the user explicitly requested no
      further verification.
- [x] Third stack slice: `rigid_stack_stability` compares sequential impulse and
      rigid IPC on a compact top-heavy mass-ratio stack.
- [x] Stack-stability capture-metrics follow-up:
      `rigid_stack_stability` now publishes solver-pair identity, executor,
      top-mass/friction controls, per-solver speed, top drift, clearance,
      height error, status, step timing, top-x divergence, and compact history
      ranges through the capture hook. The focused workflow/doc drift guard
      reported `10 passed`; the real docked 24-frame capture wrote
      24 scene-metrics events under
      `/tmp/dart_capture_stack_stability_metrics_1781231606`; `pixi run lint`,
      bounded default `pixi run build`, and `git diff --check` passed.
- [x] Contact-observability slice: `rigid_contact_inspector` exposes raw
      `World.collide()` contact pairs, points, normals, depths, local points,
      shape indices, and selected-pair histories across representative public
      shape families, including true plane-shape and compound child shape
      indices.
- [x] Collision-query-options slice: `rigid_collision_query_options` exposes
      `World.collide(options)` body-kind filters across rigid/rigid,
      rigid/link, same-multibody link/link, and cross-multibody link/link
      lanes plus persistent ignored-pair diagnostics, including public
      `Contact.body_a/body_b` `CollisionBody` kind and cast diagnostics.
- [x] Collision ignored-pair follow-up: the same query-options row now
      separates unignored baseline contacts, option-filtered contacts,
      pair-ignored contacts, and final active contacts, with replayed
      ignored-pair controls, capture metrics, and a docked nonblank capture.
- [x] Collision-casts slice: `rigid_collision_casts` exposes public
      `CollisionGroup.raycast_result()` nearest/all-hit behavior and
      `CollisionGroup.sphere_cast_result()` /
      `CollisionGroup.capsule_cast_result()` time-of-impact queries with ray
      offset, all-hit, sphere/capsule sweep controls, margins, point/normal,
      history diagnostics, and scene-owned capture metrics.
- [x] Collision-casts capture-metrics follow-up: the same row now publishes row
      identity, query scope, ray/sphere/capsule controls, hit counts, first
      targets, ray fraction, sphere/capsule time of impact, margins,
      point/normal/center fields, serialized query metrics, and compact history
      ranges through the capture hook. The focused cast guard reported
      `1 passed`; the real docked 48-frame capture wrote 48 scene-metrics
      events and numeric ranges for hit counts, TOI/fraction, margins, and
      world time. The workflow/doc drift guard reported `7 passed`.
- [x] Constraint slice: `rigid_fixed_joint` and `rigid_limited_joints` expose
      perturb/reset controls plus bounded fixed, revolute, and prismatic
      constraint-error histories with scene-owned capture metrics.
- [x] Breakage lifecycle slice: `rigid_joint_breakage` exposes AVBD-pinned
      fixed-joint break-force, broken-state, connector color, payload release,
      reset behavior, and scene-owned capture metrics without claiming
      sequential-impulse or IPC parity.
- [x] Distance-spring slice: `rigid_distance_spring` shows the public
      `World.add_rigid_body_distance_spring()` path with unsprung, soft, stiff,
      and off-center anchor lanes, replay controls, capture metrics, and
      explicit IPC/multibody rejection scope.
- [x] Joint motor/limit slice: `rigid_joint_motor_limits` shows the stable
      World multibody joint-actuator path for rigid links with velocity motor,
      position stop, and effort-cap diagnostics.
- [x] Joint motor/limit capture-metrics follow-up:
      `rigid_joint_motor_limits` now publishes row identity, multibody actuator
      scope, controls, joint names, motor clamp, position-stop, force/effort
      cap, acceleration-gap, step-timing, latest-metric, and compact-history
      fields through the capture hook. The focused motor-limit guard reported
      `1 passed`, and a real docked 72-frame capture wrote 72 scene-metrics
      events under `/tmp/dart_capture_joint_motor_limits_metrics_1781234533`.
      The broader workflow/doc drift guard reported `11 passed`;
      `pixi run lint`, bounded default `pixi run build`, and
      `git diff --check` passed.
- [x] Passive joint-parameter slice: `rigid_joint_passive_parameters` shows the
      World multibody spring/rest, damping, Coulomb friction, and armature paths
      in contact-free prismatic lanes with acceleration and energy diagnostics.
- [x] Passive joint-parameter capture-metrics handoff:
      `rigid_joint_passive_parameters` now publishes row identity, passive
      joint-parameter scope, executor, spring/rest/damping, Coulomb friction,
      slip/stiction, armature, acceleration, energy, step-timing, lane order,
      per-lane metrics, and compact history fields through the capture hook.
      The focused passive-parameter guard reported `1 passed`, and the real
      docked 120-frame capture wrote 119 PNG frames and 120 scene-metrics
      events under
      `/tmp/dart_capture_joint_passive_parameters_metrics_1781235045`.
      The skipped broader workflow/doc drift guard later reported `11 passed`;
      `pixi run lint`, bounded default `pixi run build`, and
      `git diff --check` passed.
- [x] Screw-joint pitch slice: `rigid_screw_joint_pitch` shows zero, fine,
      coarse, and reverse World multibody screw-pitch lanes with gravity,
      mass/inertia controls, pitch-ratio, effective-mass, and acceleration
      diagnostics.
- [x] Screw-joint pitch capture-metrics follow-up:
      `rigid_screw_joint_pitch` now publishes row identity, screw-pitch scope,
      executor, pitch/gravity/mass/inertia controls, lane order, per-lane
      pitch, angle, axial travel, travel-per-radian, acceleration, expected
      acceleration, effective mass, mass matrix, step-timing, and compact
      history fields through the capture hook. The focused screw-pitch guard
      reported `1 passed`, and the real docked 96-frame capture wrote 95 PNG
      frames and 96 scene-metrics events under
      `/tmp/dart_capture_screw_joint_pitch_metrics_1781235714`.
      The broader workflow/doc drift guard reported `12 passed`;
      `pixi run lint`, bounded default `pixi run build`, and
      `git diff --check` passed.
- [x] Multibody dynamics-terms slice: `rigid_multibody_dynamics_terms` shows
      public joint-space dynamics accessors on contact-free fixed-base
      multibodies: mass matrix, inverse mass matrix, inverse dynamics, impulse
      response, coupling, conditioning, and residual diagnostics.
- [x] Multibody dynamics-terms capture-metrics follow-up:
      `rigid_multibody_dynamics_terms` now publishes row identity, joint-space
      dynamics scope, executor, target-acceleration/impulse/heavy-mass/gravity
      controls, lane order, per-lane joint names, target/impulse patterns,
      mass, inverse-mass, coupling, conditioning, inverse-dynamics residual,
      impulse residual, torque, response, heavy-versus-coupled ratios,
      step-timing, and compact history fields through the capture hook. The
      focused dynamics-terms guard reported `1 passed`, and the real docked
      96-frame capture wrote 95 PNG frames and 96 scene-metrics events under
      `/tmp/dart_capture_multibody_dynamics_terms_metrics_1781236627`.
      The broader workflow/doc drift guard reported `13 passed`;
      `pixi run lint`, bounded default `pixi run build`, and
      `git diff --check` passed.
- [x] Link center-of-mass slice: `rigid_link_center_of_mass` shows how
      `Link.center_of_mass` offsets change gravity torque, reflected mass, and
      hinge acceleration while the visual geometry stays fixed.
- [x] Link center-of-mass capture-metrics follow-up:
      `rigid_link_center_of_mass` now publishes row identity, inertial-offset
      scope, executor, COM/gravity/mass/inertia controls, lane order/count,
      per-lane link/joint/local-COM metadata, serialized lane metrics,
      mirrored torque/angle/acceleration sums, reflected mass-matrix and
      acceleration ratios, COM marker positions, energy, step timing, and
      compact histories through the capture hook. The focused row guard
      reported `1 passed`, and the real docked 72-frame capture wrote 71 PNG
      frames and 72 scene-metrics events under
      `/tmp/dart_capture_link_center_of_mass_metrics_1781237623`. The broader
      workflow/doc drift guard reported `14 passed`; `pixi run lint`, bounded
      default `pixi run build`, and `git diff --check` passed.
- [x] Link-Jacobian slice: `rigid_link_jacobian` shows the public
      link-origin `get_world_jacobian(link)` path on a contact-free two-link
      multibody, including `J qdot` twist, finite-difference velocity,
      `J.T wrench` torques, and power-consistency diagnostics.
- [x] Link-Jacobian capture-metrics follow-up:
      `rigid_link_jacobian` now publishes row identity, link-Jacobian scope,
      motion/wrench controls, joint names, link-origin position, twist
      components, finite-difference error, wrench force, transpose-mapped
      torques, joint-versus-wrench power, world/body Jacobian gap, and compact
      histories through the capture hook. The focused row guard reported
      `1 passed`, and the real docked 96-frame capture wrote 95 PNG frames and
      96 scene-metrics events under
      `/tmp/dart_capture_link_jacobian_metrics_1781238268`. The broader
      workflow/doc drift guard reported `14 passed`.
- [x] Multibody solver-family slice: `rigid_multibody_solver_family` shows
      semi-implicit residual-only, variational residual-only, and variational
      solved loop-closure lanes so users can see why dynamic closure solving is
      routed to the variational path.
- [x] Multibody solver-family capture-metrics follow-up:
      `rigid_multibody_solver_family` now publishes row identity, solver-family
      routing scope, executor/gravity controls, case order, integration family
      and closure policy labels, residuals, tip errors/heights, joint speeds,
      step timing, residual solve ratio, and compact histories through the
      capture hook. The focused row guard reported `1 passed`, and the real
      docked 72-frame capture wrote 71 PNG frames and 72 scene-metrics events
      under `/tmp/dart_capture_multibody_solver_family_metrics_1781239036`.
      The broader workflow/doc drift guard reported `15 passed`;
      `pixi run lint`, bounded default `pixi run build`, and
      `git diff --check` passed.
- [x] Loop-closure family slice: `rigid_loop_closure` compares POINT,
      DISTANCE, and RIGID public closure families on a variational rigid
      multibody chain, with residual-only versus solved policies plus
      residual, tip-error, distance-error, orientation, and residual-ratio
      diagnostics.
- [x] Loop-closure capture-metrics follow-up:
      `rigid_loop_closure` now publishes row identity, loop-closure family
      scope, executor/gravity controls, family and policy order, per-case
      family/policy labels, residuals, tip/distance/orientation errors, joint
      speeds, step timing, solved ratios, and compact histories through the
      capture hook. The focused row guard reported `1 passed`, and the real
      docked 72-frame capture wrote 71 PNG frames and 72 scene-metrics events
      under `/tmp/dart_capture_loop_closure_metrics_1781239815`. The broader
      workflow/doc drift guard reported `16 passed`. A pre-handoff lint,
      bounded default build, and `git diff --check` had passed before final
      handoff edits, but the user explicitly stopped further verification
      before post-handoff lint/diff refresh.
- [x] Contact-manipulation slice: `rigid_contact_manipulation` runs matched
      sequential impulse and rigid IPC table-push tasks with executor, launch
      speed, friction, pusher mass, travel, gap, contact/proximity, drift,
      goal-error, divergence, and step-timing diagnostics.
- [x] Contact-manipulation capture-metrics follow-up:
      `rigid_contact_manipulation` now publishes solver-pair identity,
      executor, launch/friction/pusher-mass controls, per-solver target travel,
      pusher gap, contact count, target speed, lateral drift, goal error, step
      timing, travel divergence, and compact history ranges through the capture
      hook. The focused contact-manipulation guard reported `1 passed`; the
      real docked 72-frame capture wrote 72 scene-metrics events under
      `/tmp/dart_capture_contact_manipulation_metrics_1781232293`. The broader
      workflow/doc drift guard, lint, bounded build, and `git diff --check`
      were not run after the user's explicit stop/no-further-verification
      instruction.
- [x] Kinematic-driver slice: `rigid_kinematic_driver` shows IPC prescribed
      tangential motion carrying a box through friction, a zero-friction slip
      baseline, and the current sequential-impulse static-like caveat.
- [x] Kinematic-driver capture-metrics follow-up:
      `rigid_kinematic_driver` now publishes IPC grip/slip lane identity, the
      sequential-impulse caveat lane, executor, drive-speed/grip-friction
      controls, per-lane solver/friction modes, driver travel, box travel,
      slip, speed ratio, support gap, contact count, status, step timing, and
      compact history ranges through the capture hook. The focused
      kinematic-driver guard reported `1 passed`; the real docked 72-frame
      capture wrote 72 scene-metrics events under
      `/tmp/dart_capture_kinematic_driver_metrics_1781232929`; and the focused
      workflow/doc drift guard reported `11 passed`. `pixi run lint`, bounded
      default `pixi run build`, and `git diff --check` passed.
- [x] Kinematic normal-push caveat slice:
      `rigid_kinematic_normal_push` shows explicit normal prescribed motion
      with IPC normal/heavy penetration-caveat lanes and a sequential-impulse
      push lane, with push speed, target mass, gap, depth, contact count,
      target travel, and capture metrics.
- [x] Front-door alignment: `rigid_body` is the default Python `py-demos`
      launch scene and the first visible row in the curated World rigid-body
      workflow.
- [x] In-viewer workflow guide: each numbered World Rigid Body row gets a
      compact runner-owned `Rigid Workflow` panel showing the maintained user
      question, a try-first checklist, the main signals to inspect, the
      maintained scope note, selectable previous/next workflow routes, and a
      restart command plus direct/searchable workflow-row selectors that match
      `NN/MM` row ids.
- [x] Baseline-hardening slice: `rigid_body` now keeps the simple falling-body
      first run while exposing solver/material controls, explicit reset,
      force-drag, contact, energy, height, speed, step-timing, replay-control
      diagnostics, and scene-owned capture metrics.
- [x] Body-mode slice: `rigid_body_modes` compares dynamic, static, and
      kinematic rigid-body mode semantics in one contact-free World, with
      shared solver/executor controls, gravity/force controls, prescribed-path
      speed, static drift, and kinematic path-error diagnostics.
- [x] Free-flight slice: `rigid_free_flight` shows contact-free initial-state
      evolution across zero-gravity drift, gravity arc, and low/high-inertia
      spin lanes with path, momentum, energy, contact-count, and step-profile
      diagnostics.
- [x] Frame-hierarchy slice: `rigid_frame_hierarchy` shows a body-fixed
      sensor/tool frame on a moving rigid body with local/world transform,
      relative-transform, parent-frame, orientation, and step-timing residuals.
- [x] External-loads slice: `rigid_external_loads` shows persistent
      force/torque accumulators, mass-scaled acceleration, inertia-scaled
      angular response, one-step pulse clearing, and the static-body caveat in
      a zero-gravity, contact-free World.
- [x] Point-load slice: `rigid_link_point_loads` shows one-shot
      `Link.apply_force()` at application points with centered translation,
      world-space off-center lever-arm yaw acceleration, one-step clearing,
      double-call accumulation, and world/local frame semantics.
- [x] Time-step-sensitivity slice: `rigid_timestep_sensitivity` shows matched
      fine/medium/coarse World time-step lanes with gravity scaling, free-fall
      error, contact timing, clearance, coarse/fine error ratio, and
      step-profile diagnostics.
- [x] Step-diagnostics slice: `rigid_step_diagnostics` shows single-body,
      contact-pair, and small-stack Worlds under one selected solver/executor
      with stage-profile domain, backend-neutral acceleration/backend-active
      status, ECS counter, contact-count, and frame-scratch memory diagnostics.
- [x] Contact-scale budget slice: `rigid_contact_scale_budget` shows
      one-, four-, and nine-box contact workloads under one selected
      solver/executor with frame-budget, friction, wall-time, per-contact,
      scratch, ECS counter, dense/single ratio, and budget-status diagnostics.
- [x] Replay follow-up: controller-heavy rigid verifier rows now store and
      restore their user-facing controls in shared replay snapshots.
- [x] Solver-comparison follow-up: `rigid_solver_compare` now has focused
      invariant coverage for wall response, histories, and solver divergence.
- [x] Guided replay timeline follow-up: the shared bottom `Replay` panel now
      accepts optional `replay_timeline` metadata for a saved-state diagnostic
      signal and event markers. `rigid_solver_compare` uses it to show position
      divergence and near-wall marker frames while replay scrubbing.
- [x] Contact-policy replay follow-up: `rigid_contact_solver_compare` now feeds
      the same replay timeline with pose-divergence values plus
      coupled-contact/depth marker frames.
- [x] Related-evidence route follow-up: the runner-owned `Rigid Workflow` panel
      now links `rigid_free_flight` and
      `rigid_multibody_dynamics_terms` to the broader `floating_base` and
      `articulated` World rows, `rigid_solver_compare` to the non-numbered
      Rigid IPC no-tunneling view, and `rigid_contact_solver_compare` to the
      differentiable contact-gradient route without changing the 36-row order;
      it also links `contact`, `rigid_joint_breakage`, and
      `rigid_joint_motor_limits` to their AVBD-specific rigid constraint
      variants.
- [x] Workflow-search UX follow-up: the `Rigid Workflow` panel's `Find row`
      filter now ranks row ids, scene ids, labels, questions, and positive
      signals ahead of scope caveats, so searches such as `contact` and
      `solver` route users to the intended rows instead of early rows that
      only say what not to infer. Related-shelf labels now show the target
      shelf and scene id in the visible row text.
- [x] Related-evidence search follow-up: `Find row` now indexes related-shelf
      scene ids, shelf names, labels, and scope notes, so searches such as
      `floating_base`, `two-link arm`, `rigid_ipc_tunnel`,
      `contact gradient`, `avbd fixed contact`, `avbd spherical`, and
      `avbd prismatic` route to the numbered row that owns the non-numbered
      shelf link. Related-only search results now label the matched target
      scene and explain the related shelf in the tooltip.
- [x] Related-evidence capture drift check: the PLAN-103 sidecar and Python
      demo README now document docked `py-demo-capture --show-ui` commands for
      every non-numbered related-evidence target, and integration coverage keeps
      those commands synchronized with the route table.
- [x] Related no-tunneling metrics follow-up: `rigid_ipc_tunnel` now exports
      scene-owned capture metrics for the non-numbered Rigid IPC shelf route:
      wall clearance, through-wall margin, box velocity, contact count, step
      timing, and barrier-held status. The focused no-tunneling metrics guard
      reported `1 passed`, and the real docked 24-frame capture wrote
      23 PNG frames plus 24 scene-metrics events under
      `/tmp/dart_capture_rigid_ipc_tunnel_metrics_1781240644`, with latest
      status `barrier-held`, min clearance about `1.22e-6` m, min tunnel
      margin about `0.500001` m, and max wall crossing `0.0`. The related
      route/docs drift guard reported `12 passed`; `pixi run lint`, bounded
      default `pixi run build`, and `git diff --check` passed.
- [x] Critical stop handoff refresh: after the no-tunneling checkpoint, the
      next continuation only inspected the remaining related-shelf gap and did
      not edit code. The maintainer/user then explicitly requested a
      handoff-only stop with no further verification. The interrupted analysis
      identified `diff_drone_liftoff` as the next plausible non-numbered
      related route for scene-owned contact-gradient metrics. No tests, lint,
      build, captures, or `git diff --check` were run after that handoff-only
      docs edit.
- [x] Related contact-gradient metrics follow-up:
      `diff_drone_liftoff` already owns the Differentiable shelf route from
      `rigid_contact_solver_compare` for analytic versus
      complementarity-aware contact-gradient behavior. It now exports
      scene-owned capture metrics for target/rest height, playhead/current
      height, analytic versus complementarity-aware thrust/final-height/loss
      values, height/target-error/thrust gaps, optimized/fallback status, and
      compact history summaries while staying non-numbered. The focused metrics
      guard reported `1 passed`; the default-build docked 96-frame capture
      wrote 95 PNG frames plus 96 scene-metrics events under
      `/tmp/dart_capture_diff_drone_liftoff_metrics_1781241455`, with status
      `fallback`, `optimized=false`, target height `1.5`, rest/current/final
      heights about `0.19995`, target error `1.30005`, and zero height/thrust
      gaps because `DART_BUILD_DIFF=OFF` in the default capture environment.
      The related route/docs drift guard reported `17 passed`; `pixi run lint`,
      bounded default `pixi run build`, and `git diff --check` passed.
- [x] Critical stop handoff refresh after the contact-gradient checkpoint:
      local `HEAD` was `949d08083a8` and origin still pointed at
      `377e55ce5cb` when the user explicitly stopped implementation and asked
      for handoff only with no further verification. This handoff records that
      no tests, lint, build, captures, or `git diff --check` were run after the
      docs-only edit. The next slice had only been inspected/planned; no code
      changes were successfully applied.
- [x] AVBD related-route metrics follow-up: keep these as
      non-numbered related evidence, and add scene-owned capture metrics to
      `avbd_rigid_fixed_joint_contact`,
      `avbd_rigid_spherical_breakable_joint`,
      `avbd_rigid_revolute_motor`, and
      `avbd_rigid_prismatic_motor`. `avbd_rigid_breakable_joint` already has a
      capture hook and was the implementation pattern. The four route hooks now
      report related-source identity, fixed-contact offset/clearance/contact
      counts, spherical anchor/orientation drift, revolute speed tracking,
      prismatic axis/drift tracking, and compact history extrema. The focused
      AVBD guard reported `4 passed`; real docked captures for all four routes
      wrote 71 PNG frames and 72 scene-metrics events apiece. Latest payloads
      reported fixed contact count `3`, spherical status `broken`, revolute
      speed about `1.2` rad/s, and prismatic speed about `0.8` m/s with zero
      orthogonal drift. The focused AVBD plus related-route/docs guard later
      reported `15 passed`; `pixi run lint`, bounded default `pixi run build`,
      and `git diff --check` passed.
- [x] World related-route metrics follow-up: keep `floating_base` and
      `articulated` as non-numbered World Rigid Body related evidence, and add
      scene-owned capture metrics so the routes linked from
      `rigid_free_flight` and `rigid_multibody_dynamics_terms` are not
      screenshot-only evidence. `floating_base` now reports related-source
      identity, floating-joint SE(3) drift/spin speed, position, spin command,
      and compact history extrema. `articulated` now reports related-source
      identity, DOFs, link count, shoulder/wrist speeds, forearm height,
      damping controls, joint positions, and compact history extrema. The
      focused related-route guard reported `7 passed`; real docked captures for
      both routes wrote 71 PNG frames and 72 scene-metrics events apiece.
      Latest payloads reported `floating_base` linear speed about `1.0145`,
      angular speed `2.0`, and body x about `0.725`, plus `articulated`
      shoulder speed about `0.643`, wrist speed about `0.583`, and forearm
      height about `-0.0257`. `pixi run lint`, bounded default
      `pixi run build`, and `git diff --check` passed.
- [x] Fundamental numbered-row capture-metrics follow-up:
      `rigid_body_modes`, `rigid_free_flight`, `rigid_frame_hierarchy`,
      `rigid_external_loads`, `rigid_link_point_loads`,
      `rigid_timestep_sensitivity`, `rigid_restitution_ladder`, and
      `rigid_material_mixing` now publish scene-owned capture metrics. The
      payloads report row identity, solver/executor or scope, controls,
      per-lane metrics, compact history extrema, and top-level manifest fields
      for mode flags, free-flight residuals, frame residuals, load response,
      point-load response, time-step error ratio, restitution rebound, and
      material mixing. `rigid_kinematic_normal_push` now uses the shared
      capture-metrics info key; `rigid_joint_breakage` remains covered through
      the shared AVBD builder. The focused affected-row guard reported
      `9 passed`. Real docked captures under
      `/tmp/dart_capture_*_fundamental_metrics_1781245202` wrote 72
      scene-metrics events for `rigid_body_modes`, `rigid_external_loads`,
      `rigid_frame_hierarchy`, `rigid_link_point_loads`, and
      `rigid_material_mixing`, plus 96 events for `rigid_free_flight`,
      `rigid_restitution_ladder`, and `rigid_timestep_sensitivity`; all
      manifests reported docked first-frame visual evidence. The related
      workflow/docs drift guard reported `10 passed`; `pixi run lint`, bounded
      default `pixi run build` with `DART safe jobs: 4`, and
      `git diff --check` passed.
- [x] Capture-first IPC stack packet: `rigid_ipc_stack_packet` lives in the
      non-numbered Rigid IPC shelf with frame-budget, wall-time, clearance,
      contact-count, drift, height-error, speed, and `bm_rigid_ipc_solver`
      benchmark-owner diagnostics for heavier stack evidence. The focused
      packet guard reported `12 passed`, the scene-metrics guard reported
      `3 passed`, and the docked capture wrote a nonblank 960x540 screenshot,
      23 PNG frames, and 24 scene-metrics events.
- [x] Visual smoke: `rigid_solver_compare` capture writes nonblank PNGs with
      the docked ImGui workspace and 24 scene-metrics events.
- [x] Visual smoke: `rigid_executor_equivalence` capture writes nonblank PNGs
      with the docked ImGui workspace and 24 scene-metrics events.
- [x] Visual smoke: `rigid_contact_solver_compare` capture writes nonblank PNGs
      with the docked ImGui workspace and 72 scene-metrics events.
- [x] Visual smoke: `contact` capture writes nonblank PNGs with the docked
      ImGui workspace and 144 scene-metrics events.
- [x] Visual smoke: `rigid_restitution_ladder` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_material_mixing` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_friction_threshold` capture writes nonblank PNGs
      with the docked ImGui workspace and 24 scene-metrics events.
- [x] Visual smoke: `rigid_spin_roll_coupling` capture writes nonblank PNGs with
      the docked ImGui workspace and 96 scene-metrics events.
- [x] Visual smoke: `rigid_stack_stability` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_contact_manipulation` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_kinematic_driver` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_kinematic_normal_push` capture writes nonblank PNGs
      with the docked ImGui workspace and 72 scene-metrics events.
- [x] Visual smoke: `rigid_contact_inspector` capture writes nonblank PNGs with
      the docked ImGui workspace and 24 scene-metrics events.
- [x] Visual smoke: `rigid_collision_query_options` capture writes nonblank
      PNGs with the docked ImGui workspace.
- [x] Visual smoke: `rigid_collision_casts` capture writes nonblank PNGs with
      the docked ImGui workspace and 48 scene-metrics events.
- [x] Visual smoke: `rigid_body` capture writes nonblank PNGs with the docked
      ImGui workspace and scene-owned baseline metrics.
- [x] Visual smoke: `rigid_body_modes` capture writes nonblank PNGs with the
      docked ImGui workspace.
- [x] Visual smoke: `rigid_free_flight` capture writes nonblank PNGs with the
      docked ImGui workspace.
- [x] Visual smoke: `rigid_frame_hierarchy` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_external_loads` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_timestep_sensitivity` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_step_diagnostics` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_contact_scale_budget` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_fixed_joint` capture writes nonblank PNGs with the
      docked ImGui workspace and scene-owned fixed-joint metrics.
- [x] Visual smoke: `rigid_joint_breakage` capture writes nonblank PNGs with the
      docked ImGui workspace and scene-owned breakage-lifecycle metrics.
- [x] Visual smoke: `rigid_distance_spring` capture writes nonblank PNGs with
      the docked ImGui workspace and scene-owned distance-spring metrics.
- [x] Visual smoke: `rigid_limited_joints` capture writes nonblank PNGs with the
      docked ImGui workspace and scene-owned one-DOF joint metrics.
- [x] Visual smoke: `rigid_joint_motor_limits` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_joint_passive_parameters` capture writes nonblank
      PNGs with the docked ImGui workspace.
- [x] Visual smoke: `rigid_screw_joint_pitch` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_multibody_dynamics_terms` capture writes nonblank
      PNGs with the docked ImGui workspace.
- [x] Visual smoke: `rigid_link_center_of_mass` capture writes nonblank PNGs
      with the docked ImGui workspace.
- [x] Visual smoke: `rigid_link_jacobian` capture writes nonblank PNGs with
      the docked ImGui workspace.
- [x] Visual smoke: `rigid_multibody_solver_family` capture writes nonblank
      PNGs with the docked ImGui workspace.
- [x] Visual smoke: `rigid_loop_closure` capture writes nonblank PNGs with the
      docked ImGui workspace.
- [x] Curated workflow: the Python demo README now orders the rigid visual
      verification scenes by user debugging question and lists capture commands.
- [x] No-tunneling scope decision: do not add a separate thin-wall catalog row
      this slice; keep `rigid_solver_compare` plus `rigid_ipc_tunnel` as the
      current user path, with `rigid_ipc_tunnel` documented as a related Rigid
      IPC shelf scene, unless a distinct high-speed preset is needed later.
- [x] Durable visual evidence packet: final curated rows, capture commands,
      test evidence, visual-smoke evidence, and limitations are promoted to the
      PLAN-103 sidecar.
- [x] Public collision-cast API guard: Python unit coverage now exercises
      `CollisionGroup.capsule_cast()` and `capsule_cast_result()` hit/miss
      behavior alongside the visual ray/sphere/capsule cast row.
- [x] Curation drift check: the PLAN-103 sidecar workflow order is now tested
      for unique consecutive numbering, registered scene ids, and alignment
      with the front-of-catalog World Rigid Body verifier block while keeping
      `rigid_ipc_tunnel` in the separate Rigid IPC capability shelf.
- [x] README-sidecar drift check: the Python demo README quick workflow table
      is now tested against the PLAN-103 sidecar order, keeping the durable
      evidence packet and user-facing quick path synchronized.
- [x] Capture-command drift check: the PLAN-103 sidecar table, sidecar refresh
      commands, and Python demo README capture commands are now tested for the
      same ordered rigid workflow, frame budgets, 960x540 size, and `--show-ui`.
- [x] Capture front-door alignment: `py-demo-capture` now defaults to the same
      `rigid_body` front door as `py-demos`.
- [x] Differentiable contact-gradient route: do not duplicate
      `diff_drone_liftoff` as a numbered rigid row this slice; route rigid users
      to the Differentiable shelf scene and expose thrust, loss, gradient, and
      height histories there.
- [x] Public API gap audit: direct rigid-body impulse, sleep/deactivation/island
      state, and loop-closure compliance rows remain deferred. The current
      public surface has `RigidBody.apply_force()`, `RigidBody.apply_torque()`,
      rigid-body linear/angular momentum, `Link.apply_force(..., point, ...)`,
      and `Multibody.compute_impulse_response()`, but no public direct
      rigid-body impulse surface, no public sleep/wake or island activation
      surface, and no public loop-closure compliance surface. The focused
      API-gap/docs guard reported `4 passed`.
- [x] Review packet draft: the local PR body draft lives in
      [`PR_DRAFT.md`](PR_DRAFT.md) and follows the repository pull-request
      template for a `main`/DART 7.0 review.
- [x] Post-push main merge/readiness refresh: merged latest `origin/main` into
      `feature/rigid-body-gui-visual-verification` as `cd7600f8cda`, resolved
      the `python/tests/integration/test_demos_cycle.py` conflict by keeping
      both the rigid workflow block and the new AVBD demo block, pushed the
      branch to origin, and refreshed validation. The conflict-resolution
      `pixi run test-py` command ran the full Python suite and reported
      `942 passed, 9 skipped`; `pixi run lint` also passed.
- [x] Current continuation checkpoint: after verifying the skipped passive-row
      gates, the branch records a local row-31 screw-joint pitch
      capture-metrics slice with focused test, docked capture evidence, broad
      workflow/doc drift guard, `pixi run lint`, bounded `pixi run build`, and
      `git diff --check`. The next fresh session should inspect the next likely
      capture-metrics row, `rigid_multibody_dynamics_terms`.
- [x] Stop/handoff checkpoint: after the row-31 checkpoint, feature work
      stopped on user instruction. No row-32 implementation edits were made.
      The next likely row was only inspected: `rigid_multibody_dynamics_terms`
      already has replay hooks, controller controls, per-lane dynamics metrics,
      and histories, but no capture-metrics hook yet. This docs-only handoff
      was not followed by tests, captures, lint, build, or `git diff --check`
      because the user explicitly requested no further verification.
- [x] Current continuation checkpoint: after the pushed handoff, the branch now
      records row-33 link center-of-mass capture metrics with focused test and
      fresh docked capture evidence. The broader workflow/doc drift guard
      reported `14 passed`; `pixi run lint`, bounded default
      `pixi run build`, and `git diff --check` passed. The next fresh session
      should continue with the next likely capture-metrics row,
      `rigid_link_jacobian`.
- [x] Current continuation checkpoint: after the row-33 local checkpoint, the
      branch now records row-34 link-Jacobian capture metrics with focused test
      and fresh docked capture evidence. Work stopped on explicit
      maintainer/user instruction to focus only on handoff without further
      verification. The next fresh session should treat final `pixi run lint`,
      bounded `pixi run build`, `git diff --check`, and any post-lint
      row-34/broad guard refresh as intentionally not completed for this
      checkpoint, then continue with the next likely capture-metrics row,
      `rigid_multibody_solver_family` only after those gates are refreshed.
- [x] Current continuation checkpoint: after the pushed row-34 handoff,
      `pixi run lint`, bounded default `pixi run build`, and
      `git diff --check` passed from the clean pushed state. The branch now
      records row-35 multibody solver-family capture metrics with focused test
      and fresh docked capture evidence. The broader workflow/doc drift guard
      reported `15 passed`; `pixi run lint`, bounded default
      `pixi run build`, and `git diff --check` passed. The next likely missing
      capture-metrics row is `rigid_loop_closure`.
- [x] Current continuation checkpoint: after the row-35 local checkpoint, the
      branch now records row-36 loop-closure capture metrics with focused test
      and fresh docked capture evidence. This completes the numbered workflow's
      current capture-metrics pass. The broader workflow/doc drift guard
      reported `16 passed`. Final handoff edits were made after the last known
      lint/build/diff evidence, and no further verification was run because the
      user explicitly requested handoff without additional verification. The
      next fresh session should audit remaining non-numbered shelf/PR-readiness
      gaps rather than assume another numbered row.

## Goal

Build a carefully curated rigid-body `py-demos` category that lets users verify
and understand DART 7 rigid-body behavior visually: contact, friction,
tunneling, stacking, constraints, restitution/material response, solver-family
differences, and backend-neutral performance.
Step diagnostics are part of that path: they make profiling and memory
observability visible without turning solver/executor controls into another
physics-method comparison.

## Key Decisions

- Python `py-demos` is the growing user-facing surface; do not add new C++
  demo scenes for this task while PLAN-103 keeps C++ `dart-demos` frozen.
- Prefer side-by-side solver comparisons only where users naturally ask what
  changes between method families. Use focused parameter controls for scenarios
  where the key question is a threshold or corner case.
- Keep backend/executor controls framed as performance and equivalence checks.
  Solver choice changes physics; executor choice should not.
- Keep contact-solver policy separate from rigid-body solver family. The
  boxed-LCP contact method is a narrow contact resolution policy for coupled
  rigid contacts, not another IPC-vs-sequential method-family comparison.
- Keep multibody-link contact as a dedicated row after contact policy. The
  stable `contact` scene id now answers whether articulated links participate
  in contact response through drop, slide, and pusher lanes without exposing
  solver-internal contact impulses or loop-compliance data.
- Treat the `World Rigid Body` category as the World-facade rigid debugging
  path. A scene can still pin IPC when the user question is a focused threshold
  or stability behavior rather than a solver-family comparison.
- Keep the restitution ladder as a relative material-response diagnostic. Its
  energy plots help users debug bounce behavior, but they are not an exact
  conservation claim.
- Put raw contact observability before solver comparison in the workflow. The
  contact inspector is a query-level diagnostic, so it should not make solver
  equivalence or performance claims.
- Keep collision-query option filtering next to the contact inspector. It is a
  public `World.collide(options)` body-kind and ignored-pair diagnostic, not a
  duplicate shape-family contact sandbox.
- Keep collision casts next to query filtering and before solver comparison.
  `rigid_collision_casts` is a public query diagnostic for ray hits, swept
  sphere probes, and elongated swept-capsule link/tool proxies. The row should
  not claim World contact-solver, no-tunneling, or CCD time-step guarantees.
- Keep link center-of-mass offsets between generalized dynamics terms and
  link-origin Jacobians. It shows inertial offsets and gravity torque from
  `Link.center_of_mass`; arbitrary-point Jacobians, COM Jacobians, contacts, IK,
  and operational-space control remain separate questions.
- Keep kinematic-driver wording on tangential prescribed motion. The public API
  documents IPC support for moving supports/conveyors and the sequential
  impulse static-like caveat. Normal prescribed motion now lives in
  `rigid_kinematic_normal_push` as a caveat verifier: SI pushes the target while
  IPC exposes penetration, so do not claim robust IPC normal kinematic
  manipulation.
- Heavy rigid IPC scenes remain capture/benchmark-first until runtime
  performance is suitable for the default live GUI experience. The first
  four-box stack packet is registered in the Rigid IPC shelf; future taller
  stacks still need benchmark/capture packets before any live-workflow claim.
- Do not add a second thin-wall comparison unless it shows a distinct user
  question. `rigid_solver_compare` already exposes side-by-side solver behavior
  on a wall scene, and `rigid_ipc_tunnel` remains the focused IPC
  no-tunneling capability check from the Rigid IPC shelf.
- Keep related-evidence routes in the runner and PLAN-103 sidecar synchronized.
  They are labelled as `Related shelf` links that name the target shelf and
  scene id, so non-numbered scenes such as `floating_base`, `articulated`,
  `rigid_ipc_tunnel`, and `diff_drone_liftoff` do not look like new workflow
  rows.
- Keep motor/limit wording on World multibody joint actuators until rigid-body
  joint motor behavior is stable enough to support bounded public claims.
- Keep passive joint parameters separate from motor/limit claims. The passive
  row covers spring/rest, damping, Coulomb friction, and armature on World
  multibody joints without contacts or gravity.
- Keep screw-joint pitch as a World multibody joint-family row. It covers
  translation-per-radian pitch coupling, gravity-driven rotation sign, mass and
  axial inertia effects, and effective-mass diagnostics without claiming
  contact, motor, limit, or loop-closure behavior.
- Keep generalized multibody dynamics terms as a contact-free joint-space row.
  It covers `mass_matrix`, `inverse_mass_matrix`, inverse dynamics, and
  impulse response for fixed-base revolute/prismatic-style multibodies without
  claiming Cartesian point-force, COM-Jacobian, contact impulse, or
  solver-family behavior.
- Keep link Jacobian mapping as a separate contact-free link-origin row. It
  covers `get_world_jacobian(link) @ qdot`, finite-difference link-origin
  velocity, and `get_world_jacobian(link).T @ wrench` power consistency without
  claiming arbitrary point Jacobians, COM Jacobians, contact Jacobians, IK, or
  operational-space controller stability.
- Keep multibody integration-family selection separate from closure-family
  selection. `rigid_multibody_solver_family` shows that semi-implicit can carry
  residual diagnostics while variational integration is the dynamic solve path;
  `rigid_loop_closure` then answers POINT/DISTANCE/RIGID family choice.
- Keep contact-gradient mode UX routed to `diff_drone_liftoff` until users need
  a distinct rigid workflow row beyond the existing analytic-versus-aware
  clamping-contact saddle escape.
- Keep `rigid_body` as the app front door while this branch is focused on DART
  7 rigid-body visual verification. It should stay a simple baseline scene with
  solver/material/reset/contact/step diagnostics; focused edge cases belong in
  later verifier rows. The replay timeline remains explicitly selectable as
  `replay_scrubber`.
- Keep the `Rigid Workflow` panel runner-owned. It mirrors the durable PLAN-103
  sidecar's row questions, compact inspect signals, and scope notes while adding
  a try-first checklist, restart command, selectable previous/next
  scene-switch rows, and direct/searchable workflow-row selectors, including
  `NN/MM` row-id search and ranked intent search, so the learning path and
  caveats are visible and navigable inside `py-demos` without duplicating guide
  text in every scene.
- Keep replay timeline diagnostics opt-in and scene-owned. The shared `Replay`
  panel can render a lightweight `replay_timeline` signal plus marker track from
  saved replay snapshots, but long-form row guidance stays in `Rigid Workflow`
  and row-specific panels.
- Keep body-mode semantics as an early contact-free row. `rigid_body_modes`
  answers whether a body is dynamic, static, or kinematic; deeper force
  accumulator behavior stays in `rigid_external_loads`, and contact-driven
  prescribed motion stays in `rigid_kinematic_driver` for tangential support
  and `rigid_kinematic_normal_push` for the normal-push caveat.
- Keep free flight as the no-contact initial-state row. It can show analytic
  drift/arc references, momentum residuals, energy drift, and inertia-scaled
  spin diagnostics, but it should not claim contact, load, restitution, or
  solver-family behavior.
- Keep frame hierarchy as a contact-free kinematic row. It covers body-fixed
  sensor/tool frames, local-to-world composition, relative transforms, and parent
  identity without claiming force, contact, sensor-model, or solver-family
  behavior.
- Keep external loads as a contact-free accumulator semantics row. It covers
  persistent `apply_force()`/`apply_torque()`, direct `force`/`torque`
  observability, pulse clearing, mass/inertia scaling, and static-body
  behavior; it does not claim point-force, impulse, or contact-load semantics.
- Keep link point loads as a separate contact-free one-shot semantics row. It
  covers `Link.apply_force(force, point, ...)`, lever-arm torque, pre-step
  accumulation, world-space application points, frame flags, and next-step
  consumption without claiming persistent rigid-body force/torque accumulators.
- Keep time-step sensitivity as a matched-World parameter/convergence row. It
  advances fine, medium, and coarse lanes by the same displayed simulation time
  per viewer frame with different substep counts; it is not a solver
  correctness proof or an exact contact-threshold claim.
- Keep step diagnostics as an observability row. It uses one selected
  solver/executor across single-body, contact-pair, and small-stack Worlds so
  profile stages, backend-neutral acceleration metadata, ECS counters,
  contacts, and frame-scratch memory are the user-facing signal instead of
  another solver-family comparison. Acceleration metadata is status evidence,
  not a claim that a rigid accelerated backend is active.
- Keep contact-scale budget as a bounded live-GUI performance row. It compares
  one-, four-, and nine-box contact workloads with public profiling and memory
  APIs; taller or heavier IPC contact stacks remain benchmark/capture-first
  until they are suitable for routine interactive use.
- Keep material mixing as a pair-rule ownership row. It uses swapped
  body/surface pairs to expose `max(restitution)` and
  `sqrt(friction product)` without claiming IPC restitution behavior or
  duplicating the restitution ladder or incline friction threshold.
- Keep spin/roll coupling as a sequential-impulse rotational-friction row. It
  explains how surface friction couples sliding speed and angular speed through
  contact slip and roll ratio, but it does not claim a public rolling-resistance,
  torsional-friction, IPC rolling-cone, or spinning-coin parameter surface.
- Keep loop closure as a public-family comparison row for residual-only versus
  solved behavior. POINT covers endpoint locking, DISTANCE covers tether length,
  and RIGID covers full-pose welds; add another row only for a distinct
  behavior such as compliance or breakage. The current public dartpy surface has
  no public loop-closure compliance surface.
- Keep `rigid_joint_breakage` AVBD-pinned and scoped to break-force,
  broken-state, and reset lifecycle through the explicit panel reset button; do
  not imply sequential-impulse or IPC parity.
- Keep direct rigid-body impulse as a future row, not a speculative current
  scene. `rigid_external_loads` covers force/torque accumulators,
  `rigid_link_point_loads` covers one-shot point-force semantics,
  `rigid_free_flight` covers momentum diagnostics, and
  `rigid_multibody_dynamics_terms` covers joint-space impulse response; there
  is currently no public direct rigid-body impulse surface.
- Defer sleeping/deactivation/island visual rows until public dartpy APIs expose
  those states; current branch evidence found no public sleep/wake or island
  activation surface.

## Durable Evidence Owner

The curated evidence matrix now lives in
[`../../plans/103-examples-strategy/rigid-body-visual-verification.md`](../../plans/103-examples-strategy/rigid-body-visual-verification.md).
Keep that sidecar as the source of truth for scene order, capture commands,
capture-first IPC packets, test evidence, visual-smoke evidence, limitations,
and the no-tunneling scope decision.

## Immediate Next Steps

1. Confirm `git status -sb` and `git log --oneline --decorate -5` on
   `feature/rigid-body-gui-visual-verification`.
2. Do not push, open a PR, comment, or mutate GitHub state unless the user
   explicitly approves it in the current turn.
3. The numbered workflow capture-metrics pass now covers the early
   fundamentals, the runtime/profiling rows, contact/query rows,
   solver/executor rows, manipulation/kinematic rows, and the final multibody
   rows. `rigid_joint_breakage` gets its hook from the shared AVBD breakable
   builder. Keep future payloads summary-oriented and manifest-friendly.
4. The focused `rigid_ipc_tunnel` no-tunneling route plus the
   `diff_drone_liftoff` contact-gradient route now have capture metrics. Keep
   both as related shelf routes, not new numbered rigid rows, unless a distinct
   user question appears.
5. The AVBD related-route metrics slice is implemented in code checkpoint
   `4d63d2b24b0c` for fixed-joint contact, spherical breakable joint, revolute
   motor, and prismatic motor. Keep these as non-numbered related shelf routes
   unless a distinct numbered workflow question appears.
6. The broader World related-route metrics slice is implemented in code
   checkpoint `7fb9163794d` for `floating_base` and `articulated`; keep both as
   non-numbered related shelf routes unless a distinct numbered workflow
   question appears.
7. Keep payloads summary-oriented: row identity, solver/contact scope,
   user-facing controls, current lane metrics, compact history ranges, and
   enough top-level numeric fields for manifest range summaries.
8. Refresh validation for any later code/docs changes, then use the local
   [`PR_DRAFT.md`](PR_DRAFT.md) when a maintainer approves opening a PR for the
   pushed branch.
9. Keep related-evidence routes synchronized between the runner-owned
   `Rigid Workflow` panel and the durable PLAN-103 sidecar if more
   non-numbered evidence shelves are added.
10. Revisit the direct impulse, sleep/deactivation/island, and loop-closure
    compliance deferrals when public dartpy APIs expose those surfaces.
11. Keep fuller articulated arm/gripper manipulation deferred until the public
    API/runtime can support it as an interactive verifier. The current audit
    found rigid-body joints are not IPC-supported, multibody link contacts lack
    material/friction controls, and scripted IPC two-jaw pinch settings that
    actually carry an object run at hundreds of milliseconds per step.
