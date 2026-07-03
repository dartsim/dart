# PLAN-101: dartsim GUI Simulator

- Operating state: `PLAN-101` in [`dashboard.md`](dashboard.md)
- Outcome: the `dartsim/` application is a standalone, general-purpose robotics
  multi-physics GUI simulator on the DART 7 World API, with a headless
  editor engine (object/selection/command/name managers, undo/redo, simulation
  control, record/replay) and a thin backend-hidden ImGui + Filament GUI that can
  design, run, record, and replay simulations.
- Architecture owner: [`../design/dartsim_gui_simulator.md`](../design/dartsim_gui_simulator.md)
  (see its "Implementation Status (As Built)" section)
- Developer overview: [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md)
  ("dartsim Editor")
- Delivered: v1 implemented (engine unit-tested; app builds and renders the
  DART 7 scene headless). Remaining follow-ups below. See `dashboard.md`
  for operating status.

This file owns scope, workstreams, sequencing, acceptance criteria, gates, open
gaps, and rationale. `dashboard.md` owns priority, status, horizon, dimension,
next step, and gate. The design doc owns durable architecture and tradeoffs.

## Why

DART's north star includes an "easy start" so a new researcher reaches a working
simulation with minimal friction. The dartsim app began as a thin viewer over
the legacy World with hardcoded example scenes. A standalone GUI simulator that
can
author, run, and replay simulations on the DART 7 World both serves that
"easy start" dimension and acts as a forcing function that validates the
DART 7 World on its path to the DART 7 clean-break API.

## Scope

In scope (architecture in [`../design/dartsim_gui_simulator.md`](../design/dartsim_gui_simulator.md)):

- Headless editor engine (`dartsim/engine`): `SimEngine` facade,
  `ObjectManager`, `SelectionManager`, `CommandManager` + `CommandStack`
  (undo/redo), `NameManager`, `SimulationController` (Edit/Simulation mode),
  `Recorder`/`Player`, `SceneIO`, `Logger`/`EventBus`.
- Thin GUI on the existing ImGui + Filament renderer: menu bar, Scene Tree,
  Inspector, Console/Logger, simulation controls + replay timeline, Viewport with
  selection and gizmos, plus a render bridge from the DART 7 World.
- Human-readable project file for designed scenes; binary per-frame recording for
  replay; separate UI layout/config.
- Primitive-shape authoring in v1 (box/sphere/cylinder/capsule/plane) via
  editor-side shape descriptors.

Out of scope for early waves (revisit with evidence; see design Non-Goals): Qt,
multi-process transport, URDF/SDF/MJCF import into the DART 7 World,
collision-shape authoring UI, sensors/actuators/controllers, promotion of the
engine to `dart::`/`dartpy`, differentiable/batched/GPU paths.

## Workstreams (phases)

All v1 phases below are implemented; see the design doc's "Implementation Status
(As Built)" section for what landed and how it was verified. Summary and
ordering:

1. **Engine skeleton**: managers, command stack, scene model over `sim::World`,
   event bus; headless unit tests. No UI changes.
2. **Render bridge + viewport**: render a DART 7 scene of primitives;
   picking and selection highlight.
3. **Editor UI**: Scene Tree, Inspector, menu bar; Create/Edit through commands;
   undo/redo wired to the Edit menu.
4. **Simulation control**: Edit/Simulation mode, Play/Pause/Step/Reset,
   time-step, real-time-factor and sim/wall/step readouts.
5. **Project save/load**: human-readable project format round-trip.
6. **Record & replay**: recorder, player, timeline scrubber.
7. **Hardening + co-evolution**: adopt simulation shape/loader APIs as they
   land; docs; headless screenshot CI for the app.

Phases 1–2 are independent enough to start in parallel once the scene-model
interface is agreed; phases 3–6 build sequentially on the engine.

Beyond v1, the post-MVP **workbench completion** (formerly tracked in
`docs/dev_tasks/dartsim_workbench_completion/`, now retired into the design doc)
brought the editor to feature parity: every panel/menu runs through tested
`dartsim/ui/*_actions` view-model seams, with viewport/tree/inspector selection
sync, a Create palette, a typed multi-selection Inspector, relationship edits, an
Edit/Simulation mode boundary with replay navigation, a Watch panel with
persisted presets, a Memory panel over DART 7 World allocator diagnostics,
editor-owned sensor/collision descriptors, and a four-view viewport layout. The
filtered `dartsim/engine/*` + `dartsim/ui/*_actions` surface holds ≥95% line
coverage. Remaining work is DART 7 API-gated (sensor output panes, joint render
layers/filters, shape/loader adoption per PLAN-050) and lives in the design
doc's as-built follow-ups.

## Acceptance Criteria

The initiative is complete (and durable output has moved to owner docs) when:

- The headless engine compiles and is exercised by focused unit tests for the
  command stack (execute/undo/redo), object manager (add/remove/reparent),
  selection manager, name uniqueness, and project round-trip — without linking
  any GUI/renderer code.
- The app can, on the DART 7 World: create primitive bodies and a simple
  multibody, select and edit them via the Inspector, undo/redo those edits, run
  the simulation with Play/Pause/Step/Reset, save and reload the scene from the
  human-readable project file, and record then scrub-replay a run.
- The GUI submits all edits as engine commands and reads state via queries/events
  only; no GUI code mutates `sim::World` or scene state directly.
- The simulator uses only `dart::simulation`; no legacy
  `dart::simulation::World` usage remains in the new paths.
- The renderer boundary stays backend-hidden (no Filament/ImGui types above the
  `dart::gui` boundary), consistent with PLAN-060.
- A headless screenshot smoke for the app runs under CTest (mirrors existing GUI
  example smokes).
- `pixi run lint`, `pixi run build`, and `pixi run test-unit`/`test-py` pass for
  touched surfaces; durable architecture stays in
  [`../design/dartsim_gui_simulator.md`](../design/dartsim_gui_simulator.md) and
  the dev-task folder is deleted in the PR that completes the work.

## Gate

See `dashboard.md` PLAN-101 for the operating gate. Objective-specific proof:
headless engine tests cover command/undo, object, selection, name, and
project-round-trip behavior; the app's editor loop (design → run → record →
replay) works on the DART 7 World; the engine has zero GUI/renderer
includes; the renderer stays backend-hidden; and a headless screenshot smoke is
CTest-gated.

## Open Gaps

- DART 7 World lacks public shape access, model-format loaders,
  contact/constraint data, joint limits/control, and sensors. These block the
  matching simulator features and are tracked as co-evolution with the
  DART 7 API effort (PLAN-041 and PLAN-042). The design doc's
  gap table records the interim approach for each. Do not revive the retired
  Skeleton/BodyNode `SimulationMode` dev task as a legacy dynamics API; the
  lifecycle decision now lives in the design doc, with import work routed
  through the DART 7 World and the editor scene model.
- Project file format (JSON vs XML) is decided at Phase 5 start (JSON leaning).
- Stepping thread model (GUI thread vs worker) is decided with responsiveness
  evidence during Phase 4.

## Current Evidence

- The dartsim entry (`dartsim/app/main.cpp`) calls `dartsim::ui::runEditor`; the
  earlier viewer called `dart::gui::runApplication` with hardcoded `ExampleScene`
  fixtures, still reachable via `--scene`. The editor starts an empty workspace
  by default; `--demo` seeds the sample box/sphere/arm scene.
- DART 7 World public surface (topology, kinematics, stepping, state,
  binary save/load) is documented in
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md)
  and
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md).
- Backend-hidden ImGui + Filament renderer with `PanelBuilder` (menus, tables,
  modals), gizmos, picking, and orbit camera exists under `dart/gui/`.
- Architecture pattern: a headless engine (command/object/selection/name
  managers) with a thin GUI client, as seen in Gazebo (`gz-sim` server vs GUI
  client) and Drake (computation/context vs visualizer).
