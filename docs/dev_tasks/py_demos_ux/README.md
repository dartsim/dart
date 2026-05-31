# Python demos UX workspace

## Goal

Make `pixi run py-demos` a useful interactive examples workspace, not only a
headless smoke runner. The workspace should provide a stable docked layout,
global simulation controls, visual debugging artifacts, scalable scene
navigation, and a path for scene-specific controls.

## Current slice

- Add a docked `Simulation` toolbar and searchable `Demos` navigator around the
  Python demo catalog.
- Route `py-demos` and `py-demo-capture` through a docking-capable ImGui build
  without changing the normal `Release` build tree.
- Add `py-demo-capture` to produce PNG screenshots/frame sequences from the real
  Filament renderer and reject blank/noop captures.
- Expose Python `ScenePanel` callbacks through the existing renderer-neutral
  `PanelBuilder`/`PanelContext` abstraction.
- Add a first sx scene panel to `sx_rigid_ipc_slide` with solver metrics,
  friction control, speed plotting, and force-drag controls.
- Add scene panels to `sx_rigid_ipc_incline` and `sx_variational_chain` so the
  panel path covers both rigid IPC and variational-integrator examples.
- Add scene panels to `sx_rigid_ipc_pile` and `sx_rigid_ipc_tunnel`, plus
  shared IPC deformable diagnostics used by `ipc_deformable_friction_slide`.
- Add scene panels to the remaining core `Experimental` sx scenes:
  `sx_articulated`, `sx_floating_base`, `sx_contact`,
  `experimental_rigid_body_gui`, `sx_rigid_ipc`, and
  `sx_variational_tumbler`.
- Route sx bridge force-drag by `renderable_id` first, keep final-frame-name
  fallback, and support rigid-body force + torque without persistent force
  accumulation.
- Make scene panels stable docked panels by default, widen the right dock, and
  rebuild the default layout deterministically so stale ImGui layout state does
  not obscure the viewport.
- Add a toolbar `Replay` action that rebuilds the active demo scene and resumes
  from the beginning.
- Add a toolbar `Reset Layout` action that restores the default docked
  workspace after panels have been rearranged.
- Keep frame-output capture controlled by the shared lifecycle state so the
  toolbar state matches whether frame writing is active.
- Add a recorded-frame playback surface to the `Simulation` panel, with
  first, previous, play, next, and last controls over the recorded PPM
  sequence and a visible selected-frame artifact path.
- Make user-requested demo switches transactional: if a requested demo throws
  during factory startup, cannot create render state, fails its first frame, or
  returns after the startup budget, restore the previous active demo instead of
  leaving the workspace stuck on the broken target.
- Accept hyphenated scene aliases such as `sx-rigid-ipc-slide` in the Python
  runner validation path.
- Add scene panels to `diff_drone_liftoff` and
  `ipc_deformable_fem_buckle`, including replay/optimization summaries for the
  differentiable drone and compression/span/solver diagnostics for the FEM
  buckle scene.
- Add a clearance-focused scene panel to `ipc_deformable_fem_sphere` with
  sphere and ground clearance plots plus shared IPC deformable solver
  diagnostics.
- Add speed/clearance panels to `ipc_deformable_plate_friction` and
  `ipc_deformable_rod_friction` so the barrier-only friction demos expose live
  tangential-speed, clearance, friction-dissipation, and solver diagnostics.
- Add shape-specific diagnostics panels to `ipc_deformable_capsule_rod` and
  `ipc_deformable_trampoline`, exposing rod clearance/drape balance and
  center-height/sag/speed plots alongside shared IPC deformable diagnostics.
- Add shape-specific diagnostics panels to `ipc_deformable_drape` and
  `ipc_deformable_net`, exposing ground/step clearance, step drop, net sag, and
  lateral-sway plots alongside shared IPC deformable diagnostics.
- Add CG-solver diagnostics panels to `ipc_deformable_cg_solver` and
  `ipc_deformable_cg_contact`, exposing cantilever tip drop, FEM-cube ground
  clearance, solver path labels, and shared IPC deformable diagnostics.
- Add VBD diagnostics panels to `vbd_cloth`, `vbd_net`, and `vbd_beam`,
  exposing sag, sway/span, free-end height, node speed, and solver iteration
  metrics for the contact-free deformable VBD showcases.
- Add VBD diagnostics panels to `vbd_tilted_strand`,
  `vbd_obstacle_drape`, and `vbd_self_fold`, exposing TinyVBD stress metrics,
  obstacle clearance, layer separation, self-contact counts, and solver
  iteration metrics for the new VBD showcase scenes.
- Add replay and optimization panels to `diff_throw_to_target` and
  `diff_cartpole_trajopt`, exposing target error, trajectory/height traces,
  optimized decision variables, and playback stride/reset controls.
- Add direct GUI regression coverage for demo-sidebar search matching and
  first-appearance category grouping, replacing fragile source-only checks for
  the tree/list navigation behavior.
- Add imported-asset and scripted-boundary diagnostics panels to
  `ipc_deformable_obj_cloth`, `ipc_deformable_seg_strand`,
  `ipc_deformable_pt_particles`, and `ipc_deformable_scripted_dirichlet`,
  exposing sag/drop/clearance/sweep metrics alongside shared IPC solver
  diagnostics.
- Add FEM diagnostics panels to `ipc_deformable_fem_bar`,
  `ipc_deformable_fem_twist`, `ipc_deformable_fcr_twist`,
  `ipc_deformable_fem_drop`, `ipc_deformable_fem_box`, and
  `ipc_deformable_fem_msh`, exposing tip drop, twist span, clearance, and
  mean-speed metrics alongside shared IPC solver diagnostics.
- Move Python demo line-segment visual updates to snake_case dartpy APIs so
  panel tests and demo captures stay free of camelCase deprecation warnings.
- Add Control & Modern panels to `legged_balance`, `arm_push_box`,
  `cartpole_gym_env`, `cartpole_mpc`, and `sensor_descriptors`, exposing
  controller state, command signals, sensor-surface status, and live plots.
- Add legacy Control & IK panels to `operational_space_control`,
  `hybrid_dynamics`, `biped_stand`, `joint_constraints`, `vehicle`,
  `atlas_simbicon`, `g1_simbicon`, and `simbicon_duo`, exposing controller
  state, tracking/pose/command metrics, SIMBICON gait state, and live plots.
- Move the Control & Modern whole-step controllers that need the interactive
  viewer to `pre_step`, keeping `SceneSetup.step` documented as a headless
  runner escape hatch.
- Use a plane collision floor in `arm_push_box` so the contact-rich panel
  capture stays free of LCP fallback spam while preserving the visible floor.
- Fix legacy BodyNode force-drag to apply the mouse spring at the picked shape
  point instead of the body origin when a visual shape has a local offset, and
  add direct controller regression coverage for both BodyNode and external
  SimpleFrame-style force-drag paths.
- Make the `Demos` navigator group categories by first appearance across the
  whole catalog instead of relying on contiguous scene ordering, and show the
  current filtered/total scene count.
- Audit the remaining Python demo catalog after the custom-panel pass. The
  remaining no-panel scenes are starter/static visualization, basic rigid-body,
  basic collision/constraint, soft-body, and legacy asset examples; keep those
  lightweight until they gain scene-specific controls or metrics worth
  surfacing.

## Remaining work

- Add full viewer mouse-event injection coverage for force-drag if the Filament
  test harness grows synthetic pointer input.
- Decide whether dock layout persistence is needed after the default layout
  settles.

## Completion notes

Before completing this dev task, promote any durable design decisions to
`docs/design/demos_app.md` or `docs/plans/103-examples-strategy.md`, then remove
this directory in the completing PR.
