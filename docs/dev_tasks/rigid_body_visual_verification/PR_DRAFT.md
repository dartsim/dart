# PR Draft: Rigid-Body Visual Verification Workflow

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
  `SceneSetup.info["capture_metrics"]`, including step-diagnostics,
  contact-scale budget, baseline first-run diagnostics, contact inspection,
  collision casts, solver comparison, executor equivalence, contact-policy
  comparison, multibody-link contact, friction threshold, spin/roll coupling,
  stack stability, contact manipulation, kinematic driver,
  fixed/breakage/one-DOF joint constraint errors, motor/limit behavior,
  passive joint parameters, screw-joint pitch, generalized dynamics terms,
  link center-of-mass offsets, and stack-packet physics/runtime fields in
  `scene_metrics.jsonl` and `manifest.json`. The manifest summarizes the full
  event stream with first/latest events, per-key presence counts, and top-level
  numeric ranges so mid-capture metric dropouts are visible.
- Adds drift tests that keep the registry, PLAN-103 sidecar, README quick table,
  and capture commands synchronized.
- Keeps heavy IPC stacks, arm/gripper manipulation, arbitrary-point/contact
  Jacobians, sleep/island/deactivation, and duplicate no-tunneling rows out of
  the numbered workflow until the public API or runtime evidence supports an
  interactive GUI row.

## Testing

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
