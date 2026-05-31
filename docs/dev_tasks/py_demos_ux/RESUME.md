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
- Python scene validation accepts hyphenated aliases such as
  `sx-rigid-ipc-slide`.
- sx force-drag now resolves picked SimpleFrames by `renderable_id` first,
  falls back to the final SimpleFrame name, skips static targets, applies
  torque-aware rigid-body forces, and restores rigid force/torque buffers after
  the sx step.
- C++ force-drag no longer pauses the simulation loop, so Python one-shot force
  handlers can actually be consumed on the next step.

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

Focused checks:

```bash
pixi run python -m py_compile scripts/capture_py_demo.py python/tests/unit/test_capture_py_demo.py
pixi run python -m py_compile python/examples/demos/runner.py python/examples/demos/_sx_bridge.py python/examples/demos/scenes/sx_rigid_ipc_slide.py python/examples/demos/scenes/sx_rigid_ipc_incline.py python/examples/demos/scenes/sx_variational_chain.py python/tests/unit/test_py_demo_panels.py
pixi run python -m py_compile python/examples/demos/_ipc_deformable_bridge.py python/examples/demos/scenes/sx_rigid_ipc_pile.py python/examples/demos/scenes/sx_rigid_ipc_tunnel.py python/examples/demos/scenes/ipc_deformable_friction_slide.py python/tests/unit/test_py_demo_panels.py
pixi run pytest python/tests/unit/test_capture_py_demo.py -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/unit/test_py_demo_panels.py -q
PYTHONPATH=build/default/cpp/Release/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
PYTHONPATH=build/default/cpp/Release-docking/python:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
pixi run build-py-dev-docking
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_FilamentSceneExtraction$'
pixi run lint
```

## Next

1. Add custom panels to the highest-value differentiable and IPC deformable
   scenes, starting with `diff_drone_liftoff` and
   `ipc_deformable_fem_buckle`.
2. Add direct viewer-input coverage for mouse force-drag once tests can inject
   pointer drags into the Filament viewer loop.
3. Add recorded-frame playback only after the recording/playback data contract
   exists.
