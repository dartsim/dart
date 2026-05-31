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
  so stale ImGui state cannot leave panels floating over the viewport.
- Frame-output recording is controlled by the viewer lifecycle state, so the
  toolbar `Stop Record` state now matches whether frames continue writing.
- Recorded-frame playback now lives in the `Simulation` panel when frame output
  is active. It tracks the recorded PPM sequence; offers first, previous, play,
  next, and last controls; and surfaces the selected frame artifact path.
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
- `arm_push_box`, `cartpole_gym_env`, and `cartpole_mpc` now use
  `SceneSetup.pre_step` for interactive-viewer control. `SceneSetup.step` is
  documented as a headless whole-step escape hatch.
- `arm_push_box` now uses a plane collision floor plus a visual floor, avoiding
  LCP fallback warnings during panel capture while preserving the visible scene.

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
```

The screenshot at
`/tmp/dart_py_demo_capture_reset_layout_control/sx_rigid_ipc_slide.png` was
viewed and showed the top `Simulation` toolbar with `Reset Layout` between
`Replay` and `Stop Record`.

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
pixi run py-demo-capture -- --scene sx_articulated --show-ui --frames 2 --width 1280 --height 720 --output-dir /tmp/dart_py_demo_capture_sidebar_grouping
```

All commands produced nonblank docked UI screenshots, PPM/PNG frame sequences,
and MP4s. Pixel checks over the final PPM frames showed nonblank viewport,
toolbar, and right-panel regions for each capture.

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
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_FilamentSceneExtraction$'
pixi run pytest python/tests/unit/test_capture_py_demo.py -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/unit/test_py_demo_panels.py -q
PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
pixi run build-py-dev-docking
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_FilamentSceneExtraction$'
pixi run test-unit gui
pixi run lint
```

## Next

1. Audit the remaining Python demo catalog for any high-value scenes that still
   need custom panels or controls.
2. Add full viewer-input coverage for mouse force-drag once tests can inject
   pointer drags into the Filament viewer loop.
3. Add image thumbnail playback only if the UI renderer grows a texture-backed
   panel image primitive; the current playback surface controls and identifies
   recorded PPM frames from inside the workspace.
