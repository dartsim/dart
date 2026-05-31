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
- Make the `Demos` navigator group categories by first appearance across the
  whole catalog instead of relying on contiguous scene ordering, and show the
  current filtered/total scene count.

## Remaining work

- Add per-example controls for more high-value differentiable and IPC
  deformable scenes beyond the first drone/FEM/friction showcase slice.
- Add a recorded-frame playback surface once demos have a recording contract.
- Add direct pointer/input-driven force-drag interaction coverage if the viewer
  test harness grows mouse-event injection.
- Decide whether dock layout persistence is needed after the default layout
  settles.

## Completion notes

Before completing this dev task, promote any durable design decisions to
`docs/design/demos_app.md` or `docs/plans/103-examples-strategy.md`, then remove
this directory in the completing PR.
