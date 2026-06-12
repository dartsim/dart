# PR Draft: Rigid-Body Visual Verification Workflow

## Current Local Continuation - 2026-06-12

After the pushed full-stop handoff, the active goal was explicitly continued.
The latest local slice hardens capture metric ownership for the rigid visual
verification packet:

- `rigid_ipc_tunnel` now records
  `related_source_row: rigid_solver_compare`.
- `avbd_rigid_breakable_joint` now records
  `related_source_row: rigid_joint_breakage`; the numbered
  `rigid_joint_breakage` wrapper keeps only its direct row identity.
- `rigid_ipc_stack_packet` now records `row: rigid_ipc_stack_packet`.
- A registry-level invariant now verifies every sidecar-defined numbered row,
  related-evidence route, and capture-first IPC packet has a self-identifying
  capture metrics payload.
- Real docked captures under `/tmp/dart_capture_metric_ownership_1781249077`
  verified that the generated `manifest.json` and `scene_metrics.jsonl`
  artifacts contain the new ownership fields for `rigid_ipc_tunnel`,
  `rigid_ipc_stack_packet`, and `avbd_rigid_breakable_joint`.

## Current Stop Handoff - 2026-06-12

The latest local state is a handoff-only docs stop. The user explicitly
requested no further work and no additional verification. Before this docs edit,
the branch `feature/rigid-body-gui-visual-verification` was clean, at local
checkpoint `5127bf7e6c73` (`Refresh rigid visual verification PR readiness`),
and ahead of `origin/feature/rigid-body-gui-visual-verification`
(`4a2fb7a0714e`) by three commits:

- `5127bf7e6c7` `Refresh rigid visual verification PR readiness`
- `22d1a02de55` `Attach shared replay to related rigid scenes`
- `7c853bee1af` `Expose fundamental rigid workflow capture metrics`

Latest checked GitHub state still reported no PR associated with this branch.
No tests, lint, build, captures, `git diff --check`, push, PR creation, or other
GitHub mutation were performed after this stop-handoff docs edit.

## Summary

- Curates the Python `py-demos` front door around a 36-row World rigid-body
  visual-verification workflow.
- Adds focused rigid-body GUI scenes, panels, replay-state coverage, capture
  commands, and drift checks for solver, executor, contact, material, joint,
  multibody, profiling, and loop-closure debugging.
- Promotes the durable evidence matrix to the PLAN-103 sidecar so future agents
  can keep the user-facing workflow synchronized.

## Motivation / Problem

- DART 7 introduces new rigid-body solvers, executors, contact policies,
  diagnostics, and multibody APIs in parallel, but users need a coherent visual
  path to understand and debug them.
- The existing catalog had capability scenes, but not a carefully ordered
  first-run workflow that answers user questions before dropping into broad
  examples or planned ports.

## Changes / Key Changes

- Makes `rigid_body` the default `py-demos` and `py-demo-capture` front door,
  with solver/material controls, reset, contact, energy, height, speed,
  step-timing, force-drag, and replay-control diagnostics.
- Moves the curated World rigid-body verifier block to the front of the Python
  demo registry and documents the row order in `python/examples/demos/README.md`.
- Prefixes the interactive `Demos` navigator titles for the 36-row World Rigid
  Body workflow with their position and role while keeping `py-demos --list`
  titles and ids stable for scripts.
- Adds a runner-owned `Rigid Workflow` panel to numbered rigid rows with the
  maintained user question, inspect signals, restart command, selectable
  previous/next route, direct workflow-row selector, and searchable
  row-id/question/checklist/signal filter.
- Adds or upgrades focused visual-verification scenes for:
  - body modes, free flight, frame hierarchy, external loads, link point loads,
    time-step sensitivity, step diagnostics, and contact-scale budget;
  - restitution, material mixing, contact inspection, collision query options
    with persistent ignored-pair diagnostics, collision casts with swept-capsule
    queries, solver comparison, executor equivalence, contact-solver policy,
    multibody-link contact, friction threshold, spin/roll coupling, stack
    stability, contact manipulation, kinematic drivers, and normal-push caveat
    behavior;
  - fixed-joint preservation, AVBD-pinned break-force lifecycle, public
    rigid-body distance springs, one-DOF joints, motor/limit behavior, passive
    joint parameters, screw-joint pitch, generalized multibody dynamics terms,
    link center-of-mass offsets, link-origin Jacobians, multibody solver-family
    routing, and loop-closure families.
- Adds replay-state snapshot coverage for controller-heavy rigid verifier rows.
- Adds opt-in replay timeline metadata for the shared bottom `Replay` panel,
  piloted by `rigid_solver_compare` and `rigid_contact_solver_compare` with
  divergence values plus marker frames while scrubbing saved states.
- Adds related-evidence links from numbered rigid workflow rows to
  non-numbered shelves for broader World `floating_base`/`articulated`
  examples, focused IPC no-tunneling, differentiable contact-gradient
  debugging, and AVBD-specific rigid constraint variants. The `Find row` filter
  indexes those related shelf labels and scene ids, so users can search for the
  target evidence and land on the numbered source row; the search result labels
  the matched related scene and the docs provide docked capture commands for
  every related target.
- Adds a non-numbered `rigid_ipc_stack_packet` Rigid IPC shelf scene for
  capture-first heavier stack evidence with frame-budget, wall-time,
  clearance, contact-count, drift, height-error, speed, and benchmark-owner
  diagnostics.
- Adds scene-owned capture metrics to `py-demo-capture` manifests via
  `SceneSetup.info["capture_metrics"]`, including baseline first-run
  diagnostics, body-mode flags, free-flight residuals, frame residuals,
  force/torque accumulator response, point-load response, time-step error
  ratios, step-diagnostics, contact-scale budget, restitution rebound,
  pair-material mixing, contact inspection, collision casts, solver
  comparison, executor equivalence, contact-policy comparison,
  multibody-link contact, friction threshold, spin/roll coupling,
  stack stability, contact manipulation, kinematic driver,
  fixed/breakage/one-DOF joint constraint errors, motor/limit behavior,
  passive joint parameters, screw-joint pitch, generalized dynamics terms,
  link center-of-mass offsets, link-origin Jacobian mapping, multibody
  solver-family routing, loop-closure family selection, no-tunneling IPC,
  contact-gradient route outcome, AVBD related-route contact/breakage/motor
  metrics, and stack-packet physics/runtime fields in `scene_metrics.jsonl`
  and `manifest.json`. The manifest summarizes the full event stream with
  first/latest events, per-key presence counts, and top-level numeric ranges so
  mid-capture metric dropouts are visible.
- Adds drift tests that keep the registry, PLAN-103 sidecar, README quick table,
  and capture commands synchronized.
- Keeps heavy IPC stacks, arm/gripper manipulation, arbitrary-point/contact
  Jacobians, sleep/island/deactivation, and duplicate no-tunneling rows out of
  the numbered workflow until the public API or runtime evidence supports an
  interactive GUI row.

## Testing

- Local PR-readiness refresh:
  - `git fetch origin`
    - completed
  - `origin/main` ancestry check
    - already merged into the local branch; no merge commit needed
  - Latest checked GitHub state
    - no PR associated with `feature/rigid-body-gui-visual-verification`
  - bounded `pixi run build`
    - passed with `DART safe jobs: 2`, `ninja: no work to do`
  - full `pixi run test-py`
    - passed with `950 passed, 10 skipped` using `DART safe jobs: 1`
  - No push or GitHub mutation has been performed for this refresh.
- Latest capture metric ownership follow-up:
  - `rigid_ipc_tunnel`, `avbd_rigid_breakable_joint`, and
    `rigid_ipc_stack_packet` now publish self-identifying capture metrics for
    their related-route or capture-first roles.
  - Focused route ownership guard:
    `test_rigid_visual_routes_publish_self_describing_capture_metrics`,
    `test_rigid_visual_workflow_related_evidence_routes_are_valid`,
    `test_rigid_ipc_tunnel_reports_no_tunneling_metrics`,
    `test_rigid_ipc_stack_packet_reports_capture_first_metrics`,
    `test_rigid_joint_breakage_marks_and_resets_breakage`, and
    `test_avbd_breakable_joint_demo_marks_and_resets_joint`
    - `6 passed` before and after lint
  - `pixi run lint`
    - passed
  - `git diff --check`
    - passed
  - Real docked capture artifacts under
    `/tmp/dart_capture_metric_ownership_1781249077`
    - `rigid_ipc_tunnel`: 23 PNG frames, 24 scene-metrics events, docked
      nonblank screenshot with 2524 unique RGB values, row
      `rigid_ipc_tunnel`, related source row `rigid_solver_compare`, status
      `barrier-held`, min tunnel margin about `0.500001`, and max wall
      crossing `0.0`
    - `rigid_ipc_stack_packet`: 23 PNG frames, 24 scene-metrics events,
      docked nonblank screenshot with 2658 unique RGB values, row
      `rigid_ipc_stack_packet`, `capture_first=true`, benchmark
      `bm_rigid_ipc_solver`, and status `capture-first`
    - `avbd_rigid_breakable_joint`: 71 PNG frames, 72 scene-metrics events,
      docked nonblank screenshot with 2721 unique RGB values, row
      `avbd_rigid_breakable_joint`, related source row
      `rigid_joint_breakage`, status `broken`, and `saw_broken=1.0`
- Latest shared Replay follow-up:
  - `python/examples/demos/scenes/articulated.py`,
    `floating_base.py`, `avbd_rigid_revolute_motor.py`,
    `avbd_rigid_prismatic_motor.py`, and `rigid_ipc_tunnel.py` now publish
    `replay_sync: bridge.sync` and `replay_live_step_is_stateless: True`.
  - This lets the runner attach the shared bottom `Replay` panel to
    replay-capable scenes whose custom `pre_step` callbacks only run bridge
    stepping and derived metrics/history sampling.
  - Triggering failure: full `pixi run test-py` initially reported
    `python/tests/unit/test_py_demo_panels.py::test_registered_world_scenes_receive_shared_replay_controls`
    failed on `articulated`; a replay-gap audit then identified the full
    affected set above.
  - Pre-stop evidence: `py_compile` passed for the five edited files; the
    focused `test_registered_world_scenes_receive_shared_replay_controls`
    invariant passed; the replay-gap audit printed `[]`; and full
    `pixi run test-py` reported `950 passed, 10 skipped`.
  - Post-doc-refresh evidence: the focused shared-replay invariant reported
    `1 passed`; the replay-gap audit still printed `[]`; `pixi run lint`
    passed; and `git diff --check` passed.
  - No push or GitHub mutation has been performed for this follow-up.
- Latest fundamental numbered-row capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_body_modes.py`,
    `rigid_free_flight.py`, `rigid_frame_hierarchy.py`,
    `rigid_external_loads.py`, `rigid_link_point_loads.py`,
    `rigid_timestep_sensitivity.py`, `rigid_restitution_ladder.py`, and
    `rigid_material_mixing.py` now publish scene-owned capture metrics.
  - The payloads report row identity, solver/executor or scope, controls,
    per-lane metrics, compact history extrema, and top-level manifest fields
    for mode flags, free-flight residuals, frame residuals, load response,
    point-load response, time-step error ratio, restitution rebound, and
    material mixing.
  - `rigid_kinematic_normal_push.py` now uses the shared capture-metrics info
    key; `rigid_joint_breakage` remains covered through the shared AVBD
    breakable-joint builder.
  - Focused affected-row guard
    - `9 passed`
  - Focused workflow/docs drift guard
    - `10 passed`
  - Real docked captures under
    `/tmp/dart_capture_*_fundamental_metrics_1781245202`
    - `rigid_body_modes`, `rigid_external_loads`,
      `rigid_frame_hierarchy`, `rigid_link_point_loads`, and
      `rigid_material_mixing` wrote 72 scene-metrics events each.
    - `rigid_free_flight`, `rigid_restitution_ladder`, and
      `rigid_timestep_sensitivity` wrote 96 scene-metrics events each.
    - All eight manifests reported docked first-frame visual evidence and
      latest row-matched metrics; representative latest values include
      body-mode dynamic speed about `1.861`, free-flight arc error about
      `0.00414`, frame residuals `0.0`, light/heavy load acceleration about
      `4.0`/`1.0`, off-center point-load yaw acceleration about `-4.896`,
      material effective restitution `0.82`, ordered restitution rebound
      heights about `0.0792`/`0.1183`/`0.1526`, and a timestep coarse/fine
      error ratio about `1.0`.
  - `pixi run lint`
    - passed
  - bounded default `pixi run build`
    - passed with `DART safe jobs: 4`, `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest World related-route capture-metrics follow-up:
  - `python/examples/demos/scenes/floating_base.py` and
    `python/examples/demos/scenes/articulated.py` now publish scene-owned
    capture metrics for the non-numbered World Rigid Body shelf routes linked
    from `rigid_free_flight` and `rigid_multibody_dynamics_terms`.
  - The payloads report related source row, floating-joint drift/spin speed and
    position, compact two-link arm shoulder/wrist speed, forearm height,
    damping controls, joint positions, and compact history extrema.
  - `python/tests/integration/test_demos_cycle.py::test_world_related_evidence_routes_report_capture_metrics`
    asserts both capture hooks and finite manifest-friendly fields.
  - Focused World related-route guard with runner related-route and panel
    related-route coverage
    - `7 passed`
  - `pixi run py-demo-capture -- --scene floating_base --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_floating_base_metrics_1781250001`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `floating_base`, related source row `rigid_free_flight`, linear speed
      about `1.0145`, angular speed `2.0`, body x about `0.725`, and world
      time about `0.72`
  - `pixi run py-demo-capture -- --scene articulated --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_articulated_metrics_1781250002`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `articulated`, related source row `rigid_multibody_dynamics_terms`,
      shoulder speed about `0.643`, wrist speed about `0.583`, max joint speed
      about `0.643`, forearm height about `-0.0257`, and world time about
      `0.072`
  - `pixi run lint`
    - passed
  - bounded default `pixi run build`
    - passed, `ninja: no work to do`
  - `git diff --check`
    - passed
- Handoff-only stop after the World related-route checkpoint:
  - The user explicitly stopped further work and requested handoff-only docs
    with no further verification.
  - State at the start of the handoff edit: local `HEAD` `7fb9163794d`
    (`Expose World related route capture metrics`), origin `550b37d8b68`
    (`Refresh rigid visual verification handoff`), and no associated GitHub PR
    in the last checked state.
  - No tests, lint, build, captures, or `git diff --check` were run after this
    docs-only handoff edit.
- Handoff-only stop after the AVBD related-route checkpoint:
  - The user explicitly stopped further work and requested handoff-only docs
    with no further verification.
  - State at the start of the handoff edit: local `HEAD` `4d63d2b24b0c`
    (`Expose AVBD related route capture metrics`), origin
    `5f794e65d3f8` (`Document rigid visual verification handoff`), and no
    associated GitHub PR in the last checked state.
  - No tests, lint, build, captures, or `git diff --check` were run after this
    docs-only handoff edit.
- Latest AVBD related-route capture-metrics follow-up:
  - `python/examples/demos/scenes/avbd_rigid_fixed_joint_contact.py`,
    `avbd_rigid_spherical_breakable_joint.py`,
    `avbd_rigid_revolute_motor.py`, and
    `avbd_rigid_prismatic_motor.py` now publish scene-owned capture metrics for
    the non-numbered AVBD shelf routes linked from `contact`,
    `rigid_joint_breakage`, and `rigid_joint_motor_limits`.
  - The payloads report related source row, AVBD solver/scope or actuator,
    fixed-contact offset/clearance/contact counts, spherical breakage
    anchor/orientation drift, revolute speed tracking, prismatic axis/drift
    tracking, and compact history extrema.
  - `python/tests/integration/test_demos_cycle.py` extends the four existing
    AVBD route tests to assert the capture hooks and finite manifest-friendly
    fields.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_avbd_fixed_joint_contact_demo_exercises_contact_path python/tests/integration/test_demos_cycle.py::test_avbd_revolute_motor_demo_drives_hinge python/tests/integration/test_demos_cycle.py::test_avbd_prismatic_motor_demo_drives_slider python/tests/integration/test_demos_cycle.py::test_avbd_rigid_spherical_breakable_joint_demo_resets_anchor_only -q`
    - `4 passed`
  - `pixi run py-demo-capture -- --scene avbd_rigid_fixed_joint_contact --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_fixed_joint_contact_metrics_1781242847`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, latest
      contact count `3`, max contact count `4`, captured-offset error about
      `0.05649`, and payload speed about `0.1239`
  - `pixi run py-demo-capture -- --scene avbd_rigid_spherical_breakable_joint --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_spherical_breakable_metrics_1781242878`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, status
      `broken`, `saw_broken=1`, anchor-offset error about `0.4608`, orientation
      drift about `2.3637`, and payload speed about `2.923`
  - `pixi run py-demo-capture -- --scene avbd_rigid_revolute_motor --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_revolute_motor_metrics_1781242920`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events,
      measured speed about `1.2` rad/s, and near-zero speed error
  - `pixi run py-demo-capture -- --scene avbd_rigid_prismatic_motor --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_prismatic_motor_metrics_1781242958`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events,
      measured speed about `0.8` m/s, axis position about `0.708`, and zero
      orthogonal drift
  - Focused AVBD plus related-route/docs drift guard with panel related-route
    coverage
    - `15 passed`
  - `pixi run lint`
    - passed
  - bounded default `pixi run build`
    - passed, `ninja: no work to do`
  - `git diff --check`
    - passed
- Previous handoff-only stop:
  - The user explicitly stopped implementation after the
    `diff_drone_liftoff` contact-gradient checkpoint and requested handoff only
    with no further verification.
  - Branch state at the start of that handoff edit: local `HEAD`
    `949d08083a8` (`Expose contact-gradient route capture metrics`), origin
    `377e55ce5cb` (`Document rigid visual verification stop handoff`), and no
    associated GitHub PR in the last checked state.
  - No tests, lint, build, captures, or `git diff --check` were run after the
    docs-only handoff edit.
  - The next AVBD related-route metrics slice was inspected/planned but not
    implemented: `avbd_rigid_fixed_joint_contact`,
    `avbd_rigid_spherical_breakable_joint`,
    `avbd_rigid_revolute_motor`, and `avbd_rigid_prismatic_motor` still need
    capture metrics; `avbd_rigid_breakable_joint` already has the pattern.
- Latest contact-gradient related-shelf capture-metrics follow-up:
  - `python/examples/demos/scenes/diff_drone_liftoff.py` now publishes
    scene-owned capture metrics for the non-numbered Differentiable shelf route
    from `rigid_contact_solver_compare`: row identity, boxed-LCP
    contact-gradient scope, optimized/fallback status, target/rest height,
    playhead/current height, analytic versus complementarity-aware
    thrust/final-height/loss values, height/target-error/thrust gaps, and
    compact history summaries.
  - `python/tests/integration/test_demos_cycle.py::test_diff_drone_liftoff_reports_contact_gradient_metrics`
    asserts the capture hook mirrors the scene state, exposes finite
    manifest-friendly metrics, and verifies aware-mode escape when diff
    bindings are enabled while accepting the explicit fallback payload when
    `DART_BUILD_DIFF=OFF`.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_diff_drone_liftoff_reports_contact_gradient_metrics -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_diff_drone_liftoff_metrics_1781241455`
    - nonblank docked capture, 95 PNG frames, 96 scene-metrics events, status
      `fallback`, `optimized=false`, target height `1.5`, rest/current/final
      heights about `0.19995`, target error `1.30005`, and zero height/thrust
      gaps because the default build has `DART_BUILD_DIFF=OFF`
  - Focused related-route/docs drift guard with row ordering, viewer-title
    numbering, sidecar/README/capture-command drift checks,
    no-tunneling/stack-packet metrics, contact-gradient metrics, and panel
    related-route coverage
    - `17 passed`
  - `pixi run lint`
    - passed
  - bounded default `pixi run build`
    - passed, `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest no-tunneling related-shelf capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_ipc_tunnel.py` now publishes
    scene-owned capture metrics for the non-numbered focused Rigid IPC
    no-tunneling route: row identity, no-tunneling scope, IPC solver label,
    launch speed, wall/box extents, wall clearance, through-wall margin, box
    velocity, contact count, step timing, barrier status, and compact history
    extrema.
  - `python/tests/integration/test_demos_cycle.py::test_rigid_ipc_tunnel_reports_no_tunneling_metrics`
    asserts the capture hook mirrors live IPC state and that the fast box keeps
    positive through-wall margin.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_tunnel_reports_no_tunneling_metrics -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_tunnel_metrics_1781240644`
    - nonblank docked capture, 23 PNG frames, 24 scene-metrics events, final
      contacts `0`, status `barrier-held`, min clearance about `1.22e-6` m,
      min tunnel margin about `0.500001` m, max wall crossing `0.0`, and world
      time `0.24` s
  - Focused route/docs drift guard with row ordering, viewer-title numbering,
    docs-count, sidecar, related-evidence route/capture-command, README,
    capture-command, stack-packet, tunnel-metrics, and high-value panel
    coverage
    - `12 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 pixi run build`
    - passed, `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest loop-closure capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_loop_closure.py` now publishes
    scene-owned capture metrics for the row 36 loop-closure family verifier:
    row identity, family-selection scope, executor/gravity controls, family and
    policy order, per-case family/policy labels, residuals, tip/distance/
    orientation errors, joint speeds, step timing, residual ratios, and compact
    histories.
  - `python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families`
    now asserts the capture hook mirrors live controller metrics, controls,
    case metadata, residual ratios, and history maxima.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_metrics_1781239815`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `rigid_loop_closure`, solver
      `variational_rigid_multibody_loop_closure`, scope
      `point_distance_rigid_closure_family_selection`, executor `Sequential`,
      gravity scale `1.0`, six cases, POINT/DISTANCE/RIGID residual ratios
      about `7.595e11`/`7.458e11`/`7.740e11`, and manifest ranges for
      residuals, tip errors/heights, distance errors, orientation errors, joint
      speeds, step timing, residual ratios, and world time
  - Broad workflow/doc drift guard with row ordering, viewer-title numbering,
    sidecar/README/capture-command drift checks, passive-parameter coverage,
    screw-pitch coverage, dynamics-terms coverage, link center-of-mass
    coverage, link-Jacobian coverage, solver-family coverage, replay snapshot
    coverage, and high-value panel coverage
    - `16 passed`
  - `pixi run lint`
    - passed before the final handoff edits; a later lint result was not
      captured because the user explicitly stopped verification
  - `DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 pixi run build`
    - passed before the final handoff edits; `ninja: no work to do`
  - `git diff --check`
    - passed before the final handoff edits; not rerun after the explicit stop
      request
- Latest multibody solver-family capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_multibody_solver_family.py` now
    publishes scene-owned capture metrics for the row 35 solver-family routing
    verifier: row identity, solver-family scope, executor/gravity controls,
    case order, integration family and closure policy labels, residuals, tip
    errors/heights, joint speeds, step timing, residual solve ratio, and
    compact histories.
  - `python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures`
    now asserts the capture hook mirrors live controller metrics, controls,
    case metadata, residual solve ratio, and history maxima.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_solver_family_metrics_1781239036`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `rigid_multibody_solver_family`, solver
      `world_multibody_integration_family`, scope
      `multibody_closure_solve_routing`, executor `Sequential`, gravity scale
      `1.0`, three cases, residual-only residual about `0.7642`, solved
      residual clamped at `1e-12`, residual solve ratio about `7.642e11`, and
      manifest ranges for residuals, tip errors/heights, joint speeds, step
      timing, solve ratio, and world time
  - Broad workflow/doc drift guard with row ordering, viewer-title numbering,
    sidecar/README/capture-command drift checks, passive-parameter coverage,
    screw-pitch coverage, dynamics-terms coverage, link center-of-mass
    coverage, link-Jacobian coverage, replay snapshot coverage, and high-value
    panel coverage
    - `15 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run build`
    - passed, `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest link-Jacobian capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_link_jacobian.py` now publishes
    scene-owned capture metrics for the row 34 link-origin Jacobian verifier:
    row identity, kinematic/wrench-map scope, controls, joint names,
    link-origin position, twist components, finite-difference error, wrench
    force, transpose-mapped torques, joint-versus-wrench power, world/body
    Jacobian gap, and compact histories.
  - `python/tests/integration/test_demos_cycle.py::test_rigid_link_jacobian_maps_link_origin_twist_and_wrench`
    now asserts the capture hook mirrors live controller metrics, vectors, and
    history maxima.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_jacobian_maps_link_origin_twist_and_wrench -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_jacobian_metrics_1781238268`
    - nonblank docked capture, 95 PNG frames, 96 scene-metrics events, row
      `rigid_link_jacobian`, solver `world_multibody_link_jacobian`, scope
      `contact_free_link_origin_jacobian_wrench_map`, motion speed `0.85`,
      wrench force `1.35`, wrench moment `0.12`, two DoFs, six Jacobian rows,
      latest linear/angular speed about `0.5652/0.6190`, finite-difference
      error about `1.52e-7`, power error `0`, world/body Jacobian gap about
      `0.1272`, torques about `-0.8650/-0.2949`, matching joint/wrench power
      about `-0.4212`, and manifest ranges for speed, finite-difference error,
      power error, torque, world/body Jacobian gap, power, and world time
  - Broad workflow/doc drift guard with row ordering, viewer-title numbering,
    sidecar/README/capture-command drift checks, passive-parameter coverage,
    screw-pitch coverage, dynamics-terms coverage, link center-of-mass
    coverage, replay snapshot coverage, and high-value panel coverage
    - `14 passed`
- Latest link center-of-mass capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_link_center_of_mass.py` now publishes
    scene-owned capture metrics for the row 33 inertial-offset verifier: row
    identity, solver scope, executor, controls, lane order/count, per-lane
    link/joint/local-COM metadata, serialized lane metrics, mirrored
    torque/angle/acceleration sums, reflected mass-matrix and acceleration
    ratios, COM marker positions, energy, step timing, and compact histories.
  - `python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque`
    now asserts the capture hook mirrors live controller metrics and history
    maxima.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_center_of_mass_metrics_1781237623`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `rigid_link_center_of_mass`, solver
      `world_multibody_inertial_offsets`, scope
      `contact_free_link_center_of_mass_offsets`, executor `Sequential`, lane
      count `4`, COM offset `0.18`, gravity scale `1.0`, link mass `2.0`,
      inertia scale `4.0`, latest positive/negative angle about
      `+/-0.44895`, latest positive/negative gravity torque about
      `+/-3.18162`, zero mirrored torque/angle/acceleration sums,
      high-to-positive mass-matrix ratio about `2.948`, high-to-positive
      acceleration ratio about `0.3702`, 73 history samples, and manifest
      ranges for controls, mirrored response fields, mass matrices,
      acceleration ratios, step timing, time-step, and world-time fields
  - Broad workflow/doc drift guard with row ordering, viewer-title numbering,
    sidecar/README/capture-command drift checks, motor-limit coverage,
    passive-parameter coverage, screw-pitch coverage, dynamics-terms coverage,
    replay snapshot coverage, and high-value panel coverage
    - `14 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 CMAKE_BUILD_PARALLEL_LEVEL=2 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest multibody dynamics-terms capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_multibody_dynamics_terms.py` now
    publishes a scene-owned capture hook for the row 32 generalized dynamics
    verifier: controls, lane order, joint names, target/impulse patterns,
    mass/inverse-mass, coupling, conditioning, inverse-dynamics residual,
    impulse residual, torque norm, response norm, heavy-versus-coupled ratios,
    step timing, and compact histories.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_metrics_1781236627`
    - nonblank docked capture, 95 PNG frames, 96 scene-metrics events, row
      `rigid_multibody_dynamics_terms`, solver
      `world_multibody_dynamics_terms`, scope
      `contact_free_joint_space_dynamics`, executor `Sequential`, lane count
      `3`, target acceleration `2.2`, joint impulse `3.0`, heavy distal mass
      scale `4.0`, gravity scale `1.0`, latest coupled/heavy coupling about
      `0.3566/1.4265`, latest coupled/heavy response norm about
      `15.4636/8.6328`, latest coupled/heavy tau norm about
      `8.8632/27.0012`, heavy-to-coupled tau ratio about `3.0464`,
      heavy-to-coupled response ratio about `0.5583`, near-zero dynamics and
      impulse residuals, and manifest ranges for controls, mass, coupling,
      condition, response, torque, residual, step-timing, time-step, and
      world-time fields
  - Broad workflow/doc drift guard with row ordering, viewer-title numbering,
    sidecar/README/capture-command drift checks, motor-limit coverage,
    passive-parameter coverage, screw-pitch coverage, replay snapshot coverage,
    and high-value panel coverage
    - `13 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 CMAKE_BUILD_PARALLEL_LEVEL=2 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest screw-joint pitch capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_screw_joint_pitch.py` now publishes a
    scene-owned capture hook for the row 31 screw-pitch verifier: controls,
    lane order, pitch multipliers, joint names, zero/fine/coarse/reverse lane
    metrics, pitch ratios, angle, axial travel, acceleration residuals,
    effective mass, mass matrix, step timing, and compact histories.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_metrics_1781235714`
    - nonblank docked capture, 95 PNG frames, 96 scene-metrics events, row
      `rigid_screw_joint_pitch`, solver `world_multibody_screw_joint_pitch`,
      scope `contact_free_screw_pitch_lanes`, executor `Sequential`, lane count
      `4`, fine/coarse/reverse pitch `0.28/0.56/-0.28`, coarse/fine pitch
      ratio `2`, reverse/fine pitch ratio `-1`, latest fine/coarse/reverse
      angle about `-0.8317/-0.6162/0.8317`, latest fine/coarse/reverse axial
      travel about `-0.2329/-0.3451/-0.2329`, near-zero acceleration residuals
      on all nonzero pitch lanes, and manifest ranges for pitch, angle, axial
      travel, acceleration residual, step-timing, time-step, and world-time
      fields
  - Broad workflow/doc drift guard with row ordering, viewer-title numbering,
    sidecar/README/capture-command drift checks, motor-limit coverage,
    passive-parameter coverage, replay snapshot coverage, and high-value panel
    coverage
    - `12 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest passive joint-parameter capture-metrics handoff:
  - `python/examples/demos/scenes/rigid_joint_passive_parameters.py` now
    publishes a scene-owned capture hook for the row 30 passive parameter
    verifier: controls, lane order, spring/rest/damping, Coulomb friction,
    stiction/slip, armature, acceleration, energy, step timing, per-lane
    metrics, and compact histories.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_metrics_1781235045`
    - nonblank docked capture, 119 PNG frames, 120 scene-metrics events, row
      `rigid_joint_passive_parameters`, solver
      `world_multibody_passive_joint_parameters`, scope
      `contact_free_prismatic_lanes`, executor `Sequential`, lane count `6`,
      latest spring/damped energy about `2.1583/1.3403`, damped-energy ratio
      about `0.6210`, stiction position `0`, slip position about `0.17424`,
      slip speed about `0.72`, armature reference/heavy acceleration about
      `3.0/0.75`, armature acceleration gap about `2.25`, armature position
      gap about `0.26136`, latest step time about `0.0534 ms`, and manifest
      ranges for passive response, step-timing, time-step, and world-time
      fields
  - The skipped broader workflow/doc drift guard later reported `11 passed`;
    `pixi run lint`, bounded `pixi run build`, and `git diff --check` passed.
- Latest joint motor/limit capture-metrics follow-up:
  - `python/examples/demos/scenes/rigid_joint_motor_limits.py` now publishes a
    scene-owned capture hook for the row 29 multibody actuator verifier:
    controls, joint names, motor clamp, position stop, force/effort cap,
    acceleration gap, step timing, latest metrics, and compact histories.
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort -q`
    - `1 passed`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `11 passed`
  - `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_metrics_1781234533`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `rigid_joint_motor_limits`, solver `world_multibody_joint_actuators`,
      constraint `velocity_motor_position_limit_effort_cap`, command speed
      `0.55`, velocity limit `0.3`, position limit `0.35`, requested force
      `16.0`, effort limit `4.0`, motor speed `0.3`, motor speed error `0`,
      latest force-position gap about `0.3942`, latest limited/open
      acceleration about `2.0/8.0`, acceleration gap about `6.0`, and manifest
      ranges for motor, limit, force-gap, step-timing, time-step, and world-time
      fields
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Previous contact-manipulation capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_manipulation_metrics_1781232293`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, latest
      row `rigid_contact_manipulation`, solver `sequential_impulse_vs_ipc`,
      case solvers `SEQUENTIAL_IMPULSE`/`IPC`, pusher mass `10`, launch speed
      `1.2`, friction `0.18`, and numeric ranges for per-solver target travel,
      pusher gap, contact count, target speed, lateral drift, goal error, step
      timing, travel divergence, and world time
  - Not run after the user's explicit stop/no-further-verification instruction:
    broader workflow/doc drift guard, `pixi run lint`, bounded `pixi run build`,
    and `git diff --check`.
- Latest kinematic-driver capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc -q`
    - `1 passed`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `11 passed`
  - `pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_driver_metrics_1781232929`
    - nonblank docked capture, 71 PNG frames, 72 scene-metrics events, row
      `rigid_kinematic_driver`, solver
      `ipc_kinematic_driver_with_si_caveat`, IPC grip/slip lane solvers `IPC`,
      sequential caveat solver `SEQUENTIAL_IMPULSE`, and numeric ranges for
      per-lane driver travel, box travel, slip, speed ratio, support gap,
      contact count, step timing, time step, and world time
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest stack-stability capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered -q`
    - `1 passed`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `10 passed`
  - `pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_stack_stability_metrics_1781231606`
    - nonblank docked capture, 23 PNG frames, 24 scene-metrics events, latest
      row `rigid_stack_stability`, solver `sequential_impulse_vs_ipc`,
      case solvers `SEQUENTIAL_IMPULSE`/`IPC`, top mass ratio `20`, friction
      `0.8`, and numeric ranges for per-solver speed, clearance, height error,
      top drift, step timing, top-x divergence, and world time
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest spin/roll coupling capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `10 passed`
  - `pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_spin_roll_metrics_1781230743`
    - nonblank docked capture, 95 PNG frames, 96 scene-metrics events, latest
      row `rigid_spin_roll_coupling`, solver `sequential_impulse`, solver enum
      `SEQUENTIAL_IMPULSE`, per-lane slip/roll/spin/travel ranges, contact
      count range `0..4`, and low-friction coefficient `0`
  - `pixi run lint`
    - passed before the final handoff edit
  - No later build, capture, test, or `git diff --check` was run after the
    user's explicit stop/no-further-verification instruction.
- Latest friction-threshold capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_friction_threshold_separates_stick_and_slip_lanes python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `7 passed`
  - `pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_friction_threshold_metrics_1781229799`
    - nonblank docked capture, 23 PNG frames, 24 scene-metrics events, latest
      row `rigid_friction_threshold`, solver `ipc`, solver enum `IPC`,
      per-lane drift/speed/clearance/friction ranges, and
      `controlled_threshold_delta=0`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 pixi run build`
    - passed; `ninja: no work to do`
  - `git diff --check`
    - passed
- Latest contact-inspector capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_inspector_reports_contact_manifolds -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_contact_inspector --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_inspector_metrics_1781226572`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest row `rigid_contact_inspector`, solver
      `collision_query`, scope `shape_pair_manifold_fields`, lane count `7`,
      total contacts `21`, selected contacts `1`, max depth `0.08`, and
      selected max depth `0.04500000000000007`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_inspector_reports_contact_manifolds python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
    - `4 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build`
    - passed with safe parallelism; `ninja: no work to do`
- Latest link-contact capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene contact --frames 144 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_metrics_1781225144`
    - nonblank 960x540 screenshot, docked-workspace detection, 143 PNG
      frames, 144 scene-metrics events, latest solver
      `sequential_impulse_rigid_body`, scope `multibody_link_contact`,
      history contacts drop/slide/push `1.0/1.0/1.0`, push target travel
      `0.07463822342673943`, and drop max upward velocity
      `1.1546369999999997`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
    - `4 passed`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build`
    - passed with safe parallelism; `ninja: no work to do`
- Latest baseline capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics -q`
    - `1 passed`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
    - `4 passed`
  - `pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_body_metrics_1781224228`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest solver `Sequential impulse`, solver enum
      `SEQUENTIAL_IMPULSE`, dynamic-body count `5.0`, max speed
      `2.0129441124879746`, and minimum height `0.9956250000000001`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run build`
    - passed
- Latest solver/contact comparison capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy -q`
    - `2 passed`
  - `pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_solver_compare_metrics_1781223785`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest solver `sequential_impulse_vs_ipc`, case
      solvers `SEQUENTIAL_IMPULSE`/`IPC`, and max x-divergence
      `0.0012064618479162847`
  - `pixi run py-demo-capture -- --scene rigid_contact_solver_compare --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_solver_compare_metrics_1781223800`
    - nonblank 960x540 screenshot, docked-workspace detection, 71 PNG frames,
      72 scene-metrics events, latest contact policy
      `sequential_impulse_vs_boxed_lcp`, case methods
      `SEQUENTIAL_IMPULSE`/`BOXED_LCP`, and max pose divergence
      `0.003909716491843896`
- Latest joint-breakage capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_avbd_breakable_joint_demo_marks_and_resets_joint -q`
    - `2 passed`
  - Focused breakage workflow/doc drift suite after lint:
    - `7 passed`
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
    - `1 passed`
  - `pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_breakage_metrics_1781223331`
    - nonblank 960x540 screenshot, docked-workspace detection, 47 PNG frames,
      48 scene-metrics events, latest status `broken`, and
      `payload_release_distance=0.41319290960568955`
  - `pixi run lint`
    - passed
  - `DART_PARALLEL_JOBS=${DART_SAFE_JOBS:-5} CTEST_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} CMAKE_BUILD_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} pixi run build`
    - passed
- Latest fixed/one-DOF joint capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `8 passed`
  - `pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_metrics_1781222114`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest translation error
      `2.1673896011265015e-10`
  - `pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_metrics_1781222114`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest hinge/slider locked-axis errors `0.0`
  - `pixi run lint`
    - passed
  - `pixi run build`
    - passed
- Latest distance-spring follow-up:
  - `pixi run lint`
    - passed
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches -q`
    - `17 passed` before and after lint
  - `pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_1781220718`
    - nonblank 960x540 screenshot, docked-workspace detection, 71 PNG frames,
      72 scene-metrics events, and manifest lanes
      `free/soft/stiff/offset`
- Latest post-merge validation on pushed branch `cd7600f8cda`:
  - merged latest `origin/main` into
    `feature/rigid-body-gui-visual-verification`;
  - resolved the `python/tests/integration/test_demos_cycle.py` conflict by
    keeping both the rigid workflow test block and the new AVBD demo test block;
  - `pixi run test-py -- python/tests/integration/test_demos_cycle.py -k "world_rigid_visual_verification_scenes_are_ordered or rigid_visual_verification_sidecar_matches_registry_order or rigid_kinematic_normal_push_exposes_normal_pusher_caveat or avbd_empty_baseline_demo_steps_empty_world or avbd_demo2d_fracture_scene_breaks_and_resets_source_joints or avbd_fixed_joint_contact_demo_exercises_contact_path"`
    - `942 passed, 9 skipped`; the Pixi task forwarded this as a full Python
      suite run after rebuilding the updated native bindings.
  - `pixi run lint`
    - passed
- `pixi run test-py`
  - `942 passed, 9 skipped`; includes the capture manifest evidence guard for
    requested dimensions, converted frame count, screenshot stats, first
    UI-ready-frame stats, the shared replay timeline metadata guard, and the
    stack-packet replay-state contract.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_registered_world_scenes_receive_shared_replay_controls python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics -q`
  - `2 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_default_timeline_without_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_ignores_malformed_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response -q`
  - `5 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata -q`
  - `5 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `7 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q`
  - `6 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_registry_has_scenes python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals -q`
  - `12 passed` after lint
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py python/tests/unit/test_py_demo_gpu_toggle.py -q`
  - `25 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals -q`
  - `3 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads -q`
  - `2 passed`
- `PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release-docking/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_py_demo_capture_records_numbered_rigid_workflow_metrics_artifacts python/tests/unit/test_py_demo_panels.py::test_numbered_rigid_workflow_factory_combines_panels -q`
  - `2 passed`
- `pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_step_metrics`
  - nonblank 960x540 screenshot, 7 PNG frames, 8 scene-metrics events,
    manifest lanes `single/contact/stack`
- `pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_budget_metrics`
  - nonblank 960x540 screenshot, 7 PNG frames, 8 scene-metrics events,
    manifest lanes `single/medium/dense`
- `pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, 24 scene-metrics events,
    frame 24 `status=capture-first`, min clearance `0.000267` m, step wall time
    `626.778` ms, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `17 passed`
- `pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_normal_push_72`
  - nonblank 960x540 screenshot, 71 PNG frames, 72 scene-metrics events,
    frame 72 IPC normal/heavy `status=ipc penetration caveat` with about
    0.125 m depth and SI `status=pushed` with about 0.123 m target travel
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `18 passed`
- `pixi run py-demo-capture -- --scene contact --frames 72 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 71 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 71 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_body_panel_resets_baseline_scene python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
  - `15 passed`
- `pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
  - `12 passed` before and after lint
- `pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 71 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_runner_list_prints_catalog python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `4 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `4 passed`
- `pixi run py-demo-capture -- --scene rigid_body --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `8 passed`
- `pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 47 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `10 passed`
- `pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 95 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `13 passed` before and after lint
- `pixi run py-demo-capture -- --scene rigid_body --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_solver_compare --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_loop_closure --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `14 passed` after lint
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/simulation/test_world.py::test_simulation_collision_query_can_ignore_specific_pairs python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `5 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds -q`
  - `1 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `9 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden python/tests/unit/gui/test_gui_scene.py::test_gui_camera_and_run_helpers -q`
  - `2 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden -q`
  - `7 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls -q`
  - `2 passed`
- `pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 47 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_collision_casts_metrics_1781227741`
  - nonblank 960x540 screenshot, 47 PNG frames, 48 scene-metrics events,
    numeric ranges for ray/sphere/capsule hit counts, TOI/fraction, margins,
    time step, and world time
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_executor_equivalence_keeps_parallel_rollout_matched -q`
  - `1 passed`
- `pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_executor_equivalence_metrics_1781228299`
  - nonblank 960x540 screenshot, 23 PNG frames, 24 scene-metrics events,
    numeric ranges for pose/velocity divergence, contact-count delta,
    per-executor contact counts, per-executor step time, time step, and world
    time
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_executor_equivalence_keeps_parallel_rollout_matched python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `7 passed`
- `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_collision_query_metrics`
  - nonblank 960x540 screenshot, 7 PNG frames, 8 scene-metrics events,
    manifest lanes `rigid_rigid/rigid_link/same_links/cross_links`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `3 passed`
- `pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `7 passed` before and after lint
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_capture_py_demo.py::test_visual_capture_default_scene_matches_py_demos_front_door -q`
  - `5 passed`
- `pixi run lint`
  - passed
- `DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 pixi run build`
  - `ninja: no work to do`
- `git diff --check`
  - clean
- `DART_PARALLEL_JOBS=8 CTEST_PARALLEL_LEVEL=8 CMAKE_BUILD_PARALLEL_LEVEL=8 pixi run -e cuda test-all`
  - all 7 stages passed: lint, build, unit tests, simulation tests, Python
    tests, documentation, and CUDA tests/benchmark smoke
- Visual smoke: every numbered workflow row has a recorded nonblank
  `pixi run py-demo-capture -- --show-ui` run in the PLAN-103 sidecar.

## Breaking Changes

- [x] None

## Related Issues / PRs (backports)

- PLAN-103 examples strategy.
- No backport PR: this is DART 7 `main` example-catalog work.

---

#### Checklist

- [ ] Milestone set: `DART 7.0`
- [x] CHANGELOG.md updated
- [x] Add unit tests for new functionality
- [x] Document new methods and classes: N/A; scene docs and sidecar updated.
- [x] Add Python bindings (dartpy) if applicable: N/A; uses existing dartpy APIs.
