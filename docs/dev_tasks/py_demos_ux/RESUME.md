# Resume: Python demos UX workspace

## Done

- `pixi run py-demos` uses a docking-capable Python build in
  `build/default/cpp/Release-docking`.
- The default workspace docks `Simulation` at the top, `Demos` at the left,
  scene panels at the right, and diagnostics/status at the bottom.
- `Demos` has search, category counts, active-category expansion, and stable
  ImGui ids.
- `Simulation` has start/pause, step, reset, frame recording, status, and the
  active demo title.
- `pixi run py-demo-capture` captures real Filament PPM output, converts to PNG,
  rejects blank/noop captures, and can request MP4 encoding when `ffmpeg` is
  available.
- Python force-drag events include `renderable_id`.
- Python demo scenes can attach custom `ScenePanel` callbacks. The C++ binding
  renders those callbacks through DART's `PanelBuilder`/`PanelContext`
  abstraction and the demos host docks scene panels on the right.
- `sx_rigid_ipc_slide` has the first scene-specific panel: rigid IPC metadata,
  live friction slider, speed metric/plot, and generic sx bridge force-drag
  controls.
- `sx_rigid_ipc_incline` now has a scene panel with rigid IPC metadata, live
  friction slider, down-slope speed metric/plot, and generic sx bridge
  force-drag controls.
- `sx_rigid_ipc_pile` now has a scene panel with rigid IPC metadata, live
  friction slider, max-speed/height plots, and generic sx bridge force-drag
  controls.
- `sx_rigid_ipc_tunnel` now has a scene panel with rigid IPC metadata,
  no-tunneling clearance and velocity plots, and generic sx bridge force-drag
  controls.
- `sx_variational_chain` now has a scene panel with variational-integrator
  metadata, tip-height metric/plot, and generic sx bridge force-drag controls.
- `sx_articulated`, `sx_floating_base`, `sx_contact`,
  `experimental_rigid_body_gui`, `sx_rigid_ipc`, and
  `sx_variational_tumbler` now have custom scene panels. The tumbler panel
  includes invariant-drift diagnostics, and the rigid-body GUI scene now routes
  force-drag through `SxRenderBridge`.
- `ipc_deformable_friction_slide` now has an IPC deformable diagnostics panel
  backed by shared `IpcDeformableBridge.build_diagnostics_panel`.
- `Simulation` has a `Replay` action that rebuilds the active demo scene and
  resumes it from the beginning.
- `Simulation` has a `Reset Layout` action that restores the default docked
  workspace after panel rearrangement.
- Scene panels default to fixed-size right-docked panels, the default right
  dock is wider, and the default layout is rebuilt deterministically on startup
  before dockspace submission so ImGui places panels into real dock nodes
  instead of floating overlays.
- `dartpy.gui.is_docking_available()` exposes docking capability to tests, and
  an integration smoke now renders `py-demos --show-ui` in a docking-capable
  build and checks top/left/right/bottom dock regions around the central
  viewport.
- Frame-output recording is controlled by the viewer lifecycle state, so the
  toolbar `Stop Record` state now matches whether frames continue writing.
- Recorded-frame playback now lives in the `Simulation` panel when frame output
  is active. It tracks the recorded PPM sequence; offers first, previous, play,
  next, and last controls; and surfaces the selected frame artifact path.
- User-requested demo switches now keep a pending fallback to the previous
  active demo. If the requested demo throws during factory startup, cannot
  create render state, fails its first frame, or returns after the startup
  budget, the host restores the previous demo instead of leaving the workspace
  stuck on the broken target. Integration coverage now pins both throwing and
  slow-returning target factories.
- Python demo factories now run under the shared demo startup budget by default
  (`DART_DEMO_SCENE_STARTUP_TIMEOUT_MS`, default 5000 ms), with
  `DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS` available as a Python-specific
  override. A pure-Python factory stall raises a startup failure, allowing the
  existing transactional switch path to restore the previous demo instead of
  leaving tests stuck.
- Demo activation is visible in the docked UI: starting rows are marked,
  Simulation/Demos panels show startup or restored-previous-demo status, and
  Python factory exceptions now flow into the C++ transactional restore path.
- `py-demo-capture --switch-scene <id>` now drives the same demo-switch
  lifecycle after a bounded frame count, writes `manifest.json` and
  `events.jsonl`, and records target-demo observation or previous-demo restore
  events for stuck-switch debugging.
- Python scene validation accepts hyphenated aliases such as
  `sx-rigid-ipc-slide`.
- sx force-drag now resolves picked SimpleFrames by `renderable_id` first,
  falls back to the final SimpleFrame name, skips static targets, applies
  torque-aware rigid-body forces, and restores rigid force/torque buffers after
  the sx step.
- C++ force-drag no longer pauses the simulation loop, so Python one-shot force
  handlers can actually be consumed on the next step.
- C++ force-drag now records BodyNode-backed grab offsets in the BodyNode frame
  rather than the shape-frame descriptor, so offset visuals receive the mouse
  spring at the picked point and produce the expected torque.
- Active force-drag now has visible feedback: the DART status panel shows the
  dragged target and current force magnitude, and the viewport renders a spring
  line plus force arrow while the drag is active.
- `py-demo-capture --force-drag-target <renderable>` now drives the same
  force-drag controller path after a bounded frame count, records
  `force_drag_started`, `force_drag_updated`, and `force_drag_released` viewer
  events, and writes the scripted gesture into `manifest.json`. Scene teardown
  explicitly cancels active drags before renderables are destroyed so stale
  drag state cannot leak into a subsequent demo.
- Scripted force-drag now projects the target through the active viewport pane
  and picks it from framebuffer coordinates before starting the drag, so the
  headless proof covers the same viewport-coordinate hit testing used by real
  user drags. If the scripted target moves outside the active pane, the
  gesture cancels and logs `force_drag_target_unreachable` instead of leaving
  stale drag state active.
- `py-demo-capture --force-drag-pixel <x>,<y>` now drives a literal
  framebuffer-pixel force drag with `--force-drag-delta-pixels <dx>,<dy>`.
  This records `start_pixel` and `delta_pixels` in `events.jsonl` and
  `manifest.json`, so visual tests can reproduce a specific viewport click and
  drag without depending on a renderable name.
- Slider and plot labels now render above the control in scene panels, using
  hidden ImGui ids for the actual widgets, so narrow right-docked panels do not
  clip labels or waste horizontal plot/slider space. The bottom diagnostics
  dock is also smaller in the default layout to preserve more viewport area,
  and the docked `DART` panel puts live scene diagnostics before collapsible
  debug/help details.
- The Python demos workspace does not trust persisted dock layout as its source
  of truth. Startup rebuilds the deterministic default layout, and
  `Reset Layout` is the explicit recovery path after rearrangement, so stale
  `imgui.ini` state cannot hide controls on the next run.
- PLAN-103 now describes the current Python demos surface as interactive by
  default with headless/capture modes, updates the landed scene count to 79, and
  keeps `dartpy.gui.run_demos` scoped as a constrained examples host rather than
  a general Python-side scene authoring API.
- `UNIT_gui_FilamentSceneExtraction` now has direct controller coverage for
  external SimpleFrame-style force-drag callback routing and BodyNode
  shape-offset application points.
- `diff_drone_liftoff` now has a replay/optimization panel with target height,
  current frame height, playback stride, reset replay, analytic-vs-aware
  optimization summaries, and the aware height plot.
- `ipc_deformable_fem_buckle` now has a compression diagnostics panel with
  drive progress, span history plots, solver iterations, self-contact counts,
  and the shared IPC deformable diagnostics.
- `ipc_deformable_fem_sphere` now has a clearance diagnostics panel with
  sphere clearance, ground clearance, slab span, and shared IPC deformable
  diagnostics.
- `ipc_deformable_plate_friction` and `ipc_deformable_rod_friction` now have
  friction panels with tangential speed, obstacle clearance,
  friction-dissipation, and shared IPC deformable diagnostics.
- `ipc_deformable_capsule_rod` now has a rod-drape panel with capsule
  clearance, cloth sag, left/right balance, and shared IPC deformable
  diagnostics.
- `ipc_deformable_trampoline` now has a trampoline panel with center height,
  sag, vertical speed, and shared IPC deformable diagnostics.
- `ipc_deformable_drape` now has a step-drape panel with ground clearance, step
  clearance, step/off-step height delta, and shared IPC deformable diagnostics.
- `ipc_deformable_net` now has a net panel with sag, lateral sway/speed, and
  shared IPC deformable diagnostics.
- `ipc_deformable_cg_solver` now has an iterative-CG cantilever panel with tip
  drop, z-span, line-search counts, and shared IPC deformable diagnostics.
- `ipc_deformable_cg_contact` now has an incomplete-Cholesky CG contact panel
  with ground clearance, cube height/span, contact counts, and shared IPC
  deformable diagnostics.
- `vbd_cloth`, `vbd_net`, and `vbd_beam` now have VBD diagnostics panels with
  sag, sway/span, free-end height, mean node speed, and solver iteration
  metrics.
- `vbd_tilted_strand`, `vbd_obstacle_drape`, and `vbd_self_fold` now have VBD
  diagnostics panels with TinyVBD stress metrics, obstacle clearance, layer
  separation, self-contact counts, and solver iteration metrics.
- `diff_throw_to_target` and `diff_cartpole_trajopt` now have replay panels
  with playback stride/reset controls, target-error plots, trajectory traces,
  and optimized velocity/force summaries.
- Demo-sidebar search matching and first-appearance category grouping now have
  direct GUI regression tests, so the tree/list navigation behavior is pinned
  by behavior rather than source-string checks alone.
- `ipc_deformable_obj_cloth`, `ipc_deformable_seg_strand`,
  `ipc_deformable_pt_particles`, and `ipc_deformable_scripted_dirichlet` now
  have imported-asset/scripted-boundary panels with sag, tip-drop, ground
  clearance, fall distance, out-of-plane span, and shared IPC solver
  diagnostics.
- `ipc_deformable_fem_bar`, `ipc_deformable_fem_twist`,
  `ipc_deformable_fcr_twist`, `ipc_deformable_fem_drop`,
  `ipc_deformable_fem_box`, and `ipc_deformable_fem_msh` now have FEM
  diagnostics panels with tip drop, twist span, obstacle/ground clearance,
  free-end height, mean node speed, and shared IPC solver diagnostics.
- Python demo line-segment visuals now use snake_case dartpy APIs in the sx
  deformable bridge, IPC deformable bridge, and polyhedron scene so warnings-as
  errors panel tests stay clean.
- Shared IPC deformable diagnostics now include solver iteration, line-search,
  self-contact, converged-contact, and minimum active-distance metrics when the
  sx world exposes `last_deformable_solver_diagnostics`.
- The `Demos` navigator now groups categories by first appearance across the
  entire catalog rather than assuming each category is contiguous, and it shows
  the current filtered/total scene count beside the search field.
- `legged_balance`, `arm_push_box`, `cartpole_gym_env`, `cartpole_mpc`, and
  `sensor_descriptors` now have Control & Modern panels with controller state,
  command metrics, sensor-surface status, and live plots.
- `operational_space_control`, `hybrid_dynamics`, `biped_stand`, and
  `vehicle` now have legacy Control & IK panels with tracking error, command
  signals, pose error, wheel/steering state, and live plots.
- `joint_constraints`, `atlas_simbicon`, `g1_simbicon`, and `simbicon_duo`
  now have legacy locomotion panels with SPD sagittal-correction diagnostics
  and SIMBICON gait-state, pelvis-height, and balance-feedback plots.
- The shared SIMBICON panel has direct no-network unit coverage for Atlas + G1
  Duo diagnostics, so panel regressions are caught without fetching Unitree G1
  assets.
- `arm_push_box`, `cartpole_gym_env`, and `cartpole_mpc` now use
  `SceneSetup.pre_step` for interactive-viewer control. `SceneSetup.step` is
  documented as a headless whole-step escape hatch.
- `arm_push_box` now uses a plane collision floor plus a visual floor, avoiding
  LCP fallback warnings during panel capture while preserving the visible scene.
- The remaining no-panel catalog entries were audited after the custom-panel
  pass. They are starter/static visualization, basic rigid-body, basic
  collision/constraint, soft-body, and legacy asset examples, so they stay
  lightweight until they gain scene-specific controls or metrics worth
  surfacing.

## Validation

Docked visual proof:

```bash
pixi run py-demo-capture -- --scene sx_articulated --frames 2 --width 1280 --height 720 --output-dir /tmp/dart_py_demo_capture_ui_docked3 --show-ui
```

The command produced a nonblank docked UI screenshot.

Scene-panel visual proof:

```bash
pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --show-ui --frames 24 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_sx_controls
```

The command produced a nonblank docked UI screenshot and MP4 with the right
`Rigid IPC Slide` scene panel visible.

Additional scene-panel proof:

```bash
pixi run py-demo-capture -- --scene sx_rigid_ipc_incline --show-ui --frames 24 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_sx_incline_controls
pixi run py-demo-capture -- --scene sx_variational_chain --show-ui --frames 24 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_sx_variational_chain_controls
```

Both commands produced nonblank docked UI screenshots and MP4s with the right
scene panels visible.

Replay toolbar proof:

```bash
pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --show-ui --frames 4 --width 1280 --height 720 --output-dir /tmp/dart_py_demo_capture_replay_control
```

The screenshot at
`/tmp/dart_py_demo_capture_replay_control/sx_rigid_ipc_slide.png` was viewed
and showed the top `Simulation` toolbar with `Replay` between `Reset` and
`Stop Record`.

Reset-layout toolbar proof:

```bash
pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --show-ui --frames 4 --width 1280 --height 720 --output-dir /tmp/dart_py_demo_capture_reset_layout_control
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --show-ui --frames 4 --width 1280 --height 720 --output-dir /tmp/dart_py_demo_capture_no_persist_layout
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --show-ui --frames 8 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demos_true_docking_final
```

The screenshot at
`/tmp/dart_py_demo_capture_reset_layout_control/sx_rigid_ipc_slide.png` was
viewed and showed the top `Simulation` toolbar with `Reset Layout` between
`Replay` and `Stop Record`. The no-persistence proof screenshot at
`/tmp/dart_py_demo_capture_no_persist_layout/sx_rigid_ipc_slide.png` was
viewed and showed the deterministic docked workspace still places the Demos,
Simulation, scene panel, and DART diagnostics without overlap. The true-docking
proof screenshot at
`/tmp/dart_py_demos_true_docking_final/sx_rigid_ipc_slide.png` was viewed and showed
the panels attached to dock nodes with the viewport constrained between them,
rather than floating over the rendered scene. The top `Simulation` panel fits
recording playback without a first-run scrollbar, and the docked bottom `DART`
panel no longer duplicates the main pause/step controls.

Recording playback proof:

```bash
pixi run py-demo-capture -- --scene sx_articulated --show-ui --frames 6 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_recording_playback_panel
```

The screenshot at
`/tmp/dart_py_demo_capture_recording_playback_panel/sx_articulated.png` was
viewed and showed the `Simulation` panel's frame-playback row with
first, previous, play, next, and last controls and the selected PPM frame
filename.

Additional rigid/deformable panel proof:

```bash
pixi run py-demo-capture -- --scene sx_rigid_ipc_pile --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_sx_pile_panel
pixi run py-demo-capture -- --scene sx_rigid_ipc_tunnel --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_sx_tunnel_panel
pixi run py-demo-capture -- --scene ipc_deformable_friction_slide --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_deformable_friction_panel
```

The generated screenshots were viewed and showed the `Rigid IPC Pile`,
`Rigid IPC Tunnel`, and `IPC Friction Slide` panels with live plots and
diagnostics. Each command also produced an MP4.

Core sx panel and dock-layout proof:

```bash
pixi run py-demo-capture -- --scene experimental_rigid_body_gui --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_experimental_rigid_panel4
```

The screenshot at
`/tmp/dart_py_demo_capture_experimental_rigid_panel4/experimental_rigid_body_gui.png`
was viewed and showed the `Rigid Bodies sx` panel docked in a reserved right
column, with the 3D viewport unobscured, toolbar status unwrapped, and
frame-recording state displayed as `recording frames`.

Differentiable and FEM panel proof:

```bash
pixi run py-demo-capture -- --scene diff_drone_liftoff --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_diff_drone_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_buckle --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_fem_buckle_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_sphere --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_fem_sphere_panel
pixi run py-demo-capture -- --scene ipc_deformable_plate_friction --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_plate_friction_panel
pixi run py-demo-capture -- --scene ipc_deformable_rod_friction --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_rod_friction_panel
pixi run py-demo-capture -- --scene ipc_deformable_capsule_rod --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_capsule_rod_panel
pixi run py-demo-capture -- --scene ipc_deformable_trampoline --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_trampoline_panel
pixi run py-demo-capture -- --scene ipc_deformable_drape --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_drape_panel
pixi run py-demo-capture -- --scene ipc_deformable_net --show-ui --frames 20 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_net_panel
pixi run py-demo-capture -- --scene ipc_deformable_cg_solver --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_cg_solver_panel
pixi run py-demo-capture -- --scene ipc_deformable_cg_contact --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_cg_contact_panel
pixi run py-demo-capture -- --scene vbd_cloth --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vbd_cloth_panel
pixi run py-demo-capture -- --scene vbd_net --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vbd_net_panel
pixi run py-demo-capture -- --scene vbd_beam --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vbd_beam_panel
pixi run py-demo-capture -- --scene vbd_tilted_strand --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vbd_tilted_strand_panel
pixi run py-demo-capture -- --scene vbd_obstacle_drape --show-ui --frames 60 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vbd_obstacle_drape_panel
pixi run py-demo-capture -- --scene vbd_self_fold --show-ui --frames 60 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vbd_self_fold_panel
pixi run py-demo-capture -- --scene diff_throw_to_target --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_diff_throw_panel
pixi run py-demo-capture -- --scene diff_cartpole_trajopt --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_diff_cartpole_panel
pixi run py-demo-capture -- --scene ipc_deformable_obj_cloth --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_obj_cloth_panel
pixi run py-demo-capture -- --scene ipc_deformable_seg_strand --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_seg_strand_panel
pixi run py-demo-capture -- --scene ipc_deformable_pt_particles --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_pt_particles_panel
pixi run py-demo-capture -- --scene ipc_deformable_scripted_dirichlet --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_scripted_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_bar --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_fem_bar_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_twist --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_fem_twist_panel
pixi run py-demo-capture -- --scene ipc_deformable_fcr_twist --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_fcr_twist_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_drop --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_fem_drop_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_box --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_fem_box_panel
pixi run py-demo-capture -- --scene ipc_deformable_fem_msh --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_ipc_fem_msh_panel
pixi run py-demo-capture -- --scene legged_balance --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_legged_balance_panel
pixi run py-demo-capture -- --scene arm_push_box --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_arm_push_box_panel
pixi run py-demo-capture -- --scene cartpole_gym_env --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_cartpole_env_panel
pixi run py-demo-capture -- --scene cartpole_mpc --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_cartpole_mpc_panel
pixi run py-demo-capture -- --scene sensor_descriptors --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_sensor_descriptors_panel
pixi run py-demo-capture -- --scene operational_space_control --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_operational_space_panel
pixi run py-demo-capture -- --scene vehicle --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_vehicle_panel
pixi run py-demo-capture -- --scene joint_constraints --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_joint_constraints_panel
pixi run py-demo-capture -- --scene atlas_simbicon --show-ui --frames 12 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_atlas_simbicon_panel
pixi run py-demo-capture -- --scene sx_articulated --show-ui --frames 2 --width 1280 --height 720 --output-dir /tmp/dart_py_demo_capture_sidebar_grouping
```

All commands produced nonblank docked UI screenshots, PPM/PNG frame sequences,
and MP4s. Pixel checks over the final PPM frames showed nonblank viewport,
toolbar, and right-panel regions for each capture.

Scripted demo-switch proof:

```bash
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --switch-frame 2 --switch-scene sx_rigid_ipc_incline --show-ui --frames 10 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_scripted_switch
```

The event log recorded `requested_demo_switch`, `observed_target_demo`, and
`script_completed`; the final screenshot
`/tmp/dart_py_demo_capture_scripted_switch/sx_rigid_ipc_slide_to_sx_rigid_ipc_incline.png`
showed `Rigid IPC Inclined Slide (sx)` active in the docked workspace.

Scripted force-drag proof:

```bash
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --force-drag-target ipc_slide_box_visual --force-drag-frame 2 --force-drag-frames 8 --force-drag-delta 0.8,0,0.2 --show-ui --frames 14 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_force_drag
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --force-drag-target ipc_slide_box_visual --force-drag-frame 2 --force-drag-frames 8 --force-drag-delta 0.8,0,0.2 --show-ui --frames 14 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_force_drag_pointer_final
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --force-drag-pixel 465,388 --force-drag-delta-pixels 170,-40 --force-drag-frame 2 --force-drag-frames 8 --show-ui --frames 14 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_force_drag_pixel_final
```

The event log recorded `force_drag_started`, eight
`force_drag_updated` events, and `force_drag_released`; the viewed mid-drag
frame
`/tmp/dart_py_demo_capture_force_drag/png_frames/frame_000006.png` showed
`ipc_slide_box_visual` selected with active external-force status and force
magnitude in the docked workspace. The updated pointer-path proof produced the
same event sequence from viewport-coordinate picking, and the viewed frame
`/tmp/dart_py_demo_capture_force_drag_pointer_final/png_frames/frame_000006.png`
showed the selected target, spring/force overlay, and force magnitude in the
docked workspace. The pixel-path proof recorded `start_pixel` and
`delta_pixels` in the event log, and the viewed frame
`/tmp/dart_py_demo_capture_force_drag_pixel_final/png_frames/frame_000006.png`
showed the selected target, spring/force overlay, and force magnitude after a
literal framebuffer-pixel drag.

Focused checks:

```bash
pixi run python -m py_compile scripts/capture_py_demo.py python/tests/unit/test_capture_py_demo.py
pixi run python -m py_compile python/examples/demos/runner.py python/examples/demos/_sx_bridge.py python/examples/demos/scenes/sx_rigid_ipc_slide.py python/examples/demos/scenes/sx_rigid_ipc_incline.py python/examples/demos/scenes/sx_variational_chain.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/_ipc_deformable_bridge.py python/examples/demos/scenes/sx_rigid_ipc_pile.py python/examples/demos/scenes/sx_rigid_ipc_tunnel.py python/examples/demos/scenes/ipc_deformable_friction_slide.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/diff_drone_liftoff.py python/examples/demos/scenes/ipc_deformable_fem_buckle.py python/examples/demos/_ipc_deformable_bridge.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_fem_sphere.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_plate_friction.py python/examples/demos/scenes/ipc_deformable_rod_friction.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_capsule_rod.py python/examples/demos/scenes/ipc_deformable_trampoline.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_drape.py python/examples/demos/scenes/ipc_deformable_net.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_cg_solver.py python/examples/demos/scenes/ipc_deformable_cg_contact.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/_sx_bridge.py python/examples/demos/_ipc_deformable_bridge.py python/examples/demos/scenes/polyhedron_visual.py python/examples/demos/scenes/vbd_cloth.py python/examples/demos/scenes/vbd_net.py python/examples/demos/scenes/vbd_beam.py python/examples/demos/scenes/vbd_tilted_strand.py python/examples/demos/scenes/vbd_obstacle_drape.py python/examples/demos/scenes/vbd_self_fold.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/diff_throw_to_target.py python/examples/demos/scenes/diff_cartpole_trajopt.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_obj_cloth.py python/examples/demos/scenes/ipc_deformable_seg_strand.py python/examples/demos/scenes/ipc_deformable_pt_particles.py python/examples/demos/scenes/ipc_deformable_scripted_dirichlet.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/ipc_deformable_fem_bar.py python/examples/demos/scenes/ipc_deformable_fem_twist.py python/examples/demos/scenes/ipc_deformable_fcr_twist.py python/examples/demos/scenes/ipc_deformable_fem_drop.py python/examples/demos/scenes/ipc_deformable_fem_box.py python/examples/demos/scenes/ipc_deformable_fem_msh.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/runner.py python/examples/demos/scenes/legged_balance.py python/examples/demos/scenes/arm_push_box.py python/examples/demos/scenes/cartpole_gym_env.py python/examples/demos/scenes/cartpole_mpc.py python/examples/demos/scenes/sensor_descriptors.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/operational_space_control.py python/examples/demos/scenes/hybrid_dynamics.py python/examples/demos/scenes/biped_stand.py python/examples/demos/scenes/vehicle.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/scenes/joint_constraints.py python/examples/demos/scenes/_simbicon.py python/examples/demos/scenes/_simbicon_robots.py python/examples/demos/scenes/atlas_simbicon.py python/examples/demos/scenes/g1_simbicon.py python/examples/demos/scenes/simbicon_duo.py python/tests/unit/test_py_demo_panels.py
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_FilamentSceneExtraction$'
pixi run pytest python/tests/unit/test_capture_py_demo.py -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/unit/test_py_demo_panels.py -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/unit/test_py_demo_panels.py::test_scene_build_timeout_follows_demo_startup_budget_by_default python/tests/unit/test_py_demo_panels.py::test_scene_build_timeout_can_use_python_specific_override python/tests/unit/test_py_demo_panels.py::test_scene_build_timeout_disable_requires_python_specific_override -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/unit/test_py_demo_panels.py::test_simbicon_panel_reports_duo_robot_diagnostics_without_assets -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_show_ui_uses_docked_workspace_regions -q
PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_scripted_demo_switch_restores_previous_scene_on_startup_timeout -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_scripted_demo_switch_restores_previous_scene_when_python_factory_stalls -q
pixi run build-py-dev-docking
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_FilamentSceneExtraction$'
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --force-drag-target ipc_slide_box_visual --force-drag-frame 2 --force-drag-frames 8 --force-drag-delta 0.8,0,0.2 --show-ui --frames 14 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_force_drag_pointer_final
pixi run python -m py_compile scripts/capture_py_demo.py python/tests/unit/test_capture_py_demo.py
pixi run pytest python/tests/unit/test_capture_py_demo.py -q
env LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe timeout 180s pixi run py-demo-capture -- --scene sx_rigid_ipc_slide --force-drag-pixel 465,388 --force-drag-delta-pixels 170,-40 --force-drag-frame 2 --force-drag-frames 8 --show-ui --frames 14 --width 1280 --height 720 --video --output-dir /tmp/dart_py_demo_capture_force_drag_pixel_final
pixi run test-unit gui
pixi run lint
```

## Next

1. Consider process- or worker-isolated demo construction only if a future
   native/C++ factory can block indefinitely before returning to the C++ viewer
   loop; pure-Python stalls now fail through the shared startup-budget
   watchdog.
2. Add image thumbnail playback only if the UI renderer grows a texture-backed
   panel image primitive; the current playback surface controls and identifies
   recorded PPM frames from inside the workspace.
