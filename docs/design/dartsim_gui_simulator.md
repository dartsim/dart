# dartsim GUI Simulator Architecture

Status: Implemented (v1)

This document is the durable architecture and rationale for the `dartsim/`
application: a general-purpose, standalone robotics multi-physics GUI simulator
built on the DART 7 World API (evolved from the earlier thin viewer). It owns the
design and tradeoffs only. Active priority, horizon, and gate live in
`docs/plans/dashboard.md` (PLAN-101); the landed developer overview lives in
`docs/onboarding/gui-rendering.md`.

## Implementation Status (As Built)

The v1 simulator is implemented and the durable layout differs from the early
sketch below in two ways worth recording:

- **Layout**: the simulator lives in the top-level `dartsim/` folder — a runtime
  executable distribution, not a library (nothing is installed or exported as a
  public dart:: component; runtime dependencies are kept minimal). It contains
  `dartsim/engine` (headless editor engine, no GUI dependency, unit-tested by
  `UNIT_dartsim_engine`), `dartsim/ui` (panels depending on `dart-gui` + the
  engine), and `dartsim/app` (the thin `main.cpp` entry calling
  `dartsim::ui::runEditor`). The engine and UI are separate libraries because the
  application entry directory is restricted to a single `main.cpp` by
  `dart_gui_add_application`.
- **Viewport render bridge**: rather than re-architecting the renderer, an
  additive `dart::gui::ApplicationOptions::renderableProvider` hook lets the
  editor append `RenderableDescriptor`s (from `ObjectManager::computeRenderItems`)
  to the scene each frame. The editor hosts an empty legacy world purely as a
  render canvas (never stepped); the DART 7 World drives the actual scene.
- **Docking workspace**: the editor enables an ImGui dockspace over the main
  viewport (`ApplicationOptions::dockingEnabled`, plumbed into `dart::gui` and
  submitted as `DockSpaceOverViewport` with a pass-through central node so the
  3D scene shows through). All docking code is guarded by `IMGUI_HAS_DOCK`, so
  it is a no-op with non-docking ImGui builds. Because the docking API ships
  only on ImGui's docking branch, launching the editor (`pixi run dartsim`)
  forces `DART_USE_SYSTEM_IMGUI=OFF` to fetch `v1.92.8-docking`; the library
  build, `--scene` fixtures, and GUI smoke tests keep using system ImGui. Because the GUI build links a single ImGui, the
  shared build directory flips between system and docking ImGui as different
  targets are built — the editor launchers reconfigure to docking on demand, and
  the backend-neutral `dart::gui::isDockingAvailable()` lets the editor log
  `Panel docking: enabled` (or a rebuild hint) so the active state is visible.
  On first run an
  IDE-style default layout is built (menu top, Scene Tree left, Inspector right,
  Console and Simulation tabbed at the bottom, viewport center) via a per-panel
  `dart::gui::DockSide` hint; afterwards the user's arrangement persists via
  `imgui.ini`.

What is built and verified: the headless engine (object/selection/command/name
managers, undo/redo, Edit/Simulation mode controller, record/replay, project
I/O) with unit tests; the editor UI (menu bar, Scene Tree, Inspector, Console,
simulation controls, replay timeline, a project path modal with native Browse
fallback, context-sensitive Create menu with starter examples, and frame
attach/detach relationship actions plus link hierarchy relationship actions);
direct native project file pickers for Open and Save As; inspector enum
choices for shape type and child link joint kind; viewport selection sync;
transform-gizmo action seams; and viewport rendering of the DART 7 scene
(verified headless with a non-blank smoke check plus a foreground-geometry
pixel check).

The post-MVP **workbench completion** brought `dartsim` to feature parity. The
durable UI architecture decision it established: every panel and menu drives the
engine through tested, backend-hidden view-model **action seams** under
`dartsim/ui/include/dartsim_ui/*_actions.hpp` (project, history, outliner,
inspector, memory, palette, relationship, simulation, console, watch, viewport),
each with its own `UNIT_dartsim_ui_*Actions` target, rather than anonymous
`editor.cpp` lambdas. New editor behavior is added as an action/view-model seam
first and then wired into `editor.cpp` as a thin view. The completed surface
covers project lifecycle (native picker plus in-app browser/manual-path
fallback, dirty-replacement confirmation, recent projects), undo/redo history
labels, viewport pick/selection sync and transform gizmos, a context-sensitive
Create palette, the typed Inspector with multi-selection, frame/link
relationship edits, an explicit Edit/Simulation mode boundary with in-mode
restart and first/prev/next/last replay navigation, a Watch panel with persisted
named presets, editor-owned camera/range/contact sensor and collision
descriptors, viewport camera presets/modes/lock with selection tracking,
view-only layer filters, and a single/four-view layout with per-pane camera
memory and all-pane fit/focus. The Memory panel exposes DART 7 World allocator,
frame-scratch, and ECS storage diagnostics through the same seam pattern.
Console automation drives the same seams. The filtered `dartsim/engine/*` plus
testable `dartsim/ui/*_actions` surface is held at ≥95% line coverage, measured
by `pixi run coverage-report-dartsim`.

The native file picker is the only OS/windowing dependency in `dartsim/ui`
(`project_file_dialog.cpp`, nativefiledialog-extended). It crosses the
`dart::gui` boundary as an opaque `void* parentNativeWindow` (no backend type),
and the renderer-backend boundary (no Filament/GLFW/ImGui/OpenGL/Vulkan/Metal in
`dartsim/engine` or `dartsim/ui`, and no `dart/gui` include in the headless
engine) is enforced by `scripts/check_api_boundaries.py`
(`dartsim-ui-backend-leak` and `dartsim-engine-backend-leak`).

Genuinely-deferred follow-ups — each gated on a capability that does not exist
yet, not on this app: runtime sensor output panes and joint render layers /
joint visibility filters (wait on the DART 7 World exposing sensor values
and joint render data); richer relationship inspectors and grouping (wait on
more authored object metadata); a Scene Tree context-menu popup affordance (waits
on the panel API exposing context menus); extracting the project file-dialog
orchestration out of `editor.cpp` into its own tested seam (the last untested
dialog-flow branch, tracked under PLAN-101); and co-evolution to adopt
simulation shape/loader APIs, replacing editor-side shape descriptors, as they
land (see PLAN-050). The `## Architecture Overview` and later sections record the
original rationale; treat this section as the authoritative as-built note where
they differ.

## Summary

`dartsim` becomes a desktop application with two clearly separated layers:

1. A **headless editor engine** (no rendering, no UI toolkit) that owns the
   editable scene over a DART 7 World plus an object manager, selection
   manager, command manager with undo/redo, name manager, simulation controller,
   and a record/replay subsystem.
2. A **thin GUI** built on the existing backend-hidden Dear ImGui + Filament
   renderer (`dart::gui`) that renders the viewport and panels and drives the
   engine exclusively through engine commands and queries.

The split follows the best-in-class separation seen in Gazebo (`gz-sim` server
vs GUI client) and Drake (computation/context vs visualizer): the GUI holds no
authoritative scene state; it observes engine state and submits commands.

The simulator targets the **DART 7 World API** only
(`dart::simulation`, alias `sim`; Python `dartpy.simulation`). That API is the
clean-break simulation surface (see
[`simulation_cpp_api.md`](simulation_cpp_api.md) and
[`simulation_python_api.md`](simulation_python_api.md)).

## Goals

- A standalone application that can **design** a simulation (author bodies,
  articulated multibodies, joints, frames, loop closures), **run** it
  (play/pause/step/reset over real physics), and **record and replay** the
  resulting motion.
- A discoverable desktop UI: menu bar, a **Scene Tree** of the world, a property
  **Inspector**, a **Console/Logger**, **simulation controls** with a replay
  timeline, and a 3D **Viewport** with selection and transform gizmos.
- A headless engine that is **independently testable** and is the single owner of
  scene state, selection, and the undo/redo history.
- Strict reliance on the **DART 7 World** as the physics/state model, so
  the simulator co-evolves with and validates the DART 7 clean-break simulation
  API.
- Stay **backend-hidden** per PLAN-060: no rendering-backend or UI-toolkit types
  leak across the engine boundary or any public surface.

## Non-Goals

These are out of scope for the architecture's first implementation waves and are
revisited only with new evidence. They are not permanent exclusions.

- No retained-mode toolkit (e.g. Qt) in v1. The GUI extends the existing ImGui +
  Filament renderer. This is a deferred, evidence-gated decision, not a permanent
  exclusion; the full toolkit/language/deploy rationale and the observable
  triggers that would reopen it live in
  [`dartsim_gui_toolkit_decisions.md`](dartsim_gui_toolkit_decisions.md).
- No multi-process client/server transport in v1. The engine and GUI live in one
  process behind an in-process interface; the boundary is kept clean enough that
  a transport could be added later.
- No URDF/SDF/MJCF import into the DART 7 World in early phases. The
  DART 7 World has no model-format loader yet (only binary save/load); see
  [DART 7 API gaps](#dart-7-api-gaps-and-co-evolution).
- No collision-shape authoring UI, sensors, or actuator/controller framework
  until the DART 7 World exposes the matching public concepts.
- No promotion of the editor engine into the `dart::` core library or `dartpy`
  in v1; it lives in the `dartsim/` application folder (revisit if reuse demand
  appears).
- No differentiable simulation, batched worlds, or GPU rendering paths.

## Constraints From The Current Codebase

These constraints come from the repository as it is today and shape every
decision below.

- **DART 7 World is topology + kinematics + stepping + state today.**
  Available: World lifecycle (design mode then simulation mode), `step()`,
  `step(n)`, `sync(WorldSyncStage::Kinematics)`, RigidBody/MultiBody/Link/Joint/
  Frame/LoopClosure creation and queries, joint position/velocity read/write,
  frame transform queries, loop-closure residual diagnostics, `StateSpace`
  metadata, and binary `saveBinary`/`loadBinary` ("DRT7" format). Not yet
  available: collision-geometry exposure, model-format loaders, contact/
  constraint data, joint limits/control, sensors, rendering integration, async
  stepping, and world-level collection views. The GUI must build on what exists
  and treat the rest as deferred integration points.
- **The renderer is backend-hidden and immediate-mode.** `dart::gui` wraps
  Filament behind `dart::gui::*` types and exposes an ImGui-style `PanelBuilder`
  (already supports menu bars, tables, modals, collapsing headers, sliders) plus
  gizmos, picking, and an orbit camera. There is no Qt anywhere in the build.
- **The renderer is currently wired to the legacy `dart::simulation::World`.**
  `ApplicationOptions::world` is a `dart::simulation::WorldPtr` and
  `PanelContext::world` is a `dart::simulation::World*`. Driving the viewport
  from a DART 7 World requires a rendering bridge that reads frame transforms
  and shape descriptions.
- **Scenes today are hardcoded C++ fixtures** (`ExampleScene` enum). A
  general-purpose simulator needs runtime-authored scenes with save/load.
- **Docs conventions are strict.** Durable architecture goes here in
  `docs/design/`; roadmap state goes in `docs/plans/dashboard.md`; active
  multi-session work goes in `docs/dev_tasks/<task>/` (requires `README.md` and
  `RESUME.md`, validated by `pixi run check-docs-policy`).

## Prior-Art Survey And Naming

A survey of standalone simulators and DCC tools informed the UI and architecture
(full notes summarized here; see References):

- **Gazebo / `gz-sim`**: headless server process + GUI client over `gz-transport`;
  state in an EntityComponentManager; GUI is composed of independent plugins
  (Entity Tree, Component Inspector, World Control). Strongest engine/GUI split.
- **NVIDIA Isaac Sim (Omniverse Kit)**: DCC-style workspace; the hierarchy panel
  is the **Stage** (USD scene graph); **Property** inspector with add-component;
  timeline-driven play/stop; **Stage Recorder** captures time-sampled motion for
  scrub-replay without re-simulation; UI is optional Kit extensions over a
  headless-capable core.
- **CoppeliaSim**: most feature-complete standalone robotics UI — **Scene
  Hierarchy** tree with drag re-parent and inline rename, object properties
  dialog, status-bar console, multilevel undo/redo, embedded scripting.
- **Webots**: **Scene Tree** of nodes/fields, explicit run modes
  (Pause/Real-time/Run/Fast) plus Step/Reset, human-readable `.wbt` world files.
- **MuJoCo `simulate`**: minimal native-UI viewer; control sliders bound directly
  to data; pause/reset/single-step; no formal outliner.
- **SAPIEN**: developer viewer with Scene Hierarchy (actors/articulations),
  Actor inspector, Articulation joint inspector with editable drive parameters.
- **RViz**: not a simulator, but its **Displays** tree of typed, configurable
  items with per-item property lists and an Add dialog is an influential pattern.
- **Drake**: clean separation of computation (Diagram + Context) from
  visualization (publisher systems to Meldis/MeshCat); JointSliders for runtime
  parameter control.
- **Blender**: the reference DCC layout — swappable editor areas, **Outliner**
  (collections/objects), tabbed **Properties** inspector, global undo/redo,
  unified selection across outliner and viewport.

**Naming of the world-tree panel.** The user rejected "simulation browser."
Across these tools, "browser" denotes an asset/model browser, not the scene
hierarchy, so "browser" would mislead. The chosen name is **Scene Tree** (used
by Webots; shortest and clearest; idiomatic in robotics; no engine jargon). The
accepted alternative label is **World Outliner** (Blender/Unreal convention,
evokes the DART/SDF "world" root). "Stage" is rejected (USD-specific) and
"Entity Tree" is rejected (ECS-flavored; the public model intentionally hides
the ECS).

## Architecture Overview

```
+------------------------------------------------------------------+
| dartsim (single process)                                         |
|                                                                  |
|  +-----------------------------+      +------------------------+  |
|  |   GUI layer (ui/)           |      |  Headless engine       |  |
|  |   ImGui + Filament          |      |  (engine/)             |  |
|  |                             | cmds |                        |  |
|  |  - Menu bar                 |----->|  CommandManager        |  |
|  |  - Scene Tree panel         |      |   + CommandStack       |  |
|  |  - Inspector panel          |<-----|  ObjectManager         |  |
|  |  - Console / Logger panel   | qry/ |  SelectionManager      |  |
|  |  - Sim controls + Timeline  | evts |  NameManager           |  |
|  |  - Viewport + gizmos        |      |  SimulationController   |  |
|  |  - Render bridge            |<-----|  Recorder / Player     |  |
|  +-----------------------------+ frame|  SceneIO               |  |
|                                 state |  Logger / EventBus     |  |
|                                       |  ----------------       |  |
|                                       |  sim::World              |  |
|                                       +------------------------+  |
+------------------------------------------------------------------+
```

Key rules:

- The GUI never mutates `sim::World` or scene model state directly. Every edit is
  a **Command** submitted to the `CommandManager`. Every read is a **query** on a
  manager or a per-frame **render snapshot**.
- The engine never includes any `dart::gui`, ImGui, or Filament header. It
  depends only on `dart::simulation` (and `dart::math`).
- The engine emits **events** (object added/removed/modified, selection changed,
  mode changed, frame stepped) that the GUI subscribes to in order to refresh
  panels. This keeps panels stateless mirrors of engine state.

## Headless Engine Design

All engine types live in the app-internal `dartsim` namespace, under
`dartsim/engine/`, compiled into the `dartsim_engine` static library. The
design keeps them
UI-agnostic so they could later be promoted to a library or bound to Python if
demand appears, but that promotion is explicitly a non-goal for v1.

### SimEngine (facade)

A single facade object that owns the `sim::World` and all managers,
and exposes a small surface to the GUI: submit a command, undo, redo, query the
scene model, query/modify selection, control simulation, start/stop recording,
load/save a project, and subscribe to events. The GUI holds one `SimEngine`.

### ObjectManager (scene model)

Owns the editable **scene model**: a tree of `SceneObject` nodes that wrap the
DART 7 World's public handles and carry editor-only metadata (display
name, visibility, expansion state, authoring parameters not yet representable in
the DART 7 World). Responsibilities:

- Create/destroy DART 7 simulation objects (RigidBody, MultiBody + Links/Joints,
  Frames, LoopClosures) and keep the scene tree in sync with World contents.
- Provide stable editor handles/IDs that survive undo/redo and World rebuilds
  (simulation handles can be invalidated by `World::clear()`), so the GUI and
  selection can refer to objects across operations.
- Maintain parent/child structure for the Scene Tree (world root, then bodies and
  multibodies, then links, joints, attached frames and shapes).

The scene model is the single bridge between editor concepts and the DART 7
World; it isolates the rest of the engine from API gaps and churn.

### SelectionManager

Owns the current selection set (single and multi-select), the active/primary
selection, and selection-change events. Both the Scene Tree and the Viewport read
and write selection here so the two stay in sync (Blender/RViz pattern).

### CommandManager + CommandStack (undo/redo)

The heart of editability. Every state-changing operation is a `Command` object
with `execute()`/`undo()` (and optional `redo()` when not symmetric). The
`CommandStack` holds the undo and redo stacks; the `CommandManager` runs commands,
pushes them, and exposes `undo()`, `redo()`, `canUndo()`, `canRedo()`, and
command labels for the Edit menu. Initial command catalog:

- `AddRigidBodyCommand`, `RemoveObjectCommand`
- `AddMultiBodyCommand`, `AddLinkCommand`, `AddJointCommand`
- `AddFrameCommand`, `AddLoopClosureCommand`
- `SetTransformCommand`, `SetVelocityCommand`, `SetMassCommand`,
  `SetInertiaCommand`
- `SetJointPositionCommand`, `SetJointTypeCommand`
- `RenameObjectCommand`, `ReparentObjectCommand`
- `SetTimeStepCommand`

Commands operate through the ObjectManager so the scene model, World, and name
table are mutated consistently and reversibly. Commands captured during **Edit
Mode** are undoable; live changes during **Simulation Mode** (e.g. dragging a
slider while stepping) are handled separately (see SimulationController) and are
not pushed as authoring history.

### NameManager

Guarantees unique, stable names within their scope, generates default names
(`Body 1`, `arm/link_2`), and validates renames before they reach the World
(which itself enforces owner-local uniqueness in the DART 7 API). Keeps
editor display names decoupled from World identifiers where the two must differ.

### SimulationController (Edit Mode vs Simulation Mode)

Owns the **mode** and the stepping loop:

- **Edit Mode**: World is in authored design state; topology edits are allowed; commands
  are undoable; the World is not advancing.
- **Simulation Mode**: the World has entered runtime simulation state; the
  controller advances it with `world.step()` on a fixed cadence with a
  real-time-factor target and reports sim time, wall time, and step count.
  Play/Pause are playback states within Simulation Mode; Step enters Simulation
  Mode for one or N ticks. **Reset** restores the authored Edit Mode state
  captured when Simulation Mode began (using a binary state snapshot), so running
  never destroys the design.

Stepping runs on the engine; whether it runs on the GUI thread or a worker thread
is an implementation detail behind the controller. The DART 7 World's
default synchronous stepping is the reference behavior; async stepping is not
used (it is deferred in the DART 7 API).

### Skeleton/BodyNode Lifecycle Placement

The retired `SimulationMode { Design, Simulation }` proposal for legacy
`dart::dynamics::Skeleton` / `BodyNode` does not become a new public dynamics
API. Lifecycle ownership is split between the DART 7 World and the editor
engine: `dart::simulation::World` owns the public design-to-simulation
transition through `enterSimulationMode()` / `isSimulationMode()`, while
`dartsim::SimulationController` owns the app-internal Edit vs Simulation UI
state, captures the authored `SceneModel`, and restores it on Reset. Legacy
Skeletons remain parser/translation inputs for `dart::simulation::io::addSkeleton`
and related import bridges only; future loader, shape, or project-import work
should extend the DART 7 World / `ObjectManager` scene-model surfaces rather
than adding lifecycle flags to `Skeleton` or `BodyNode`.

### Recorder / Player (record and replay)

- **Recorder**: while in Simulation Mode, captures a per-step **frame snapshot**
  of the world state into a recording buffer. Snapshots use the DART 7 World's
  binary serialization for fidelity and speed; the recording also stores time
  and step index per frame.
- **Player**: replays a recording by restoring frame snapshots in order, driven
  by a **timeline scrubber** in the GUI, without re-simulating (Isaac Sim Stage
  Recorder pattern). Supports play/pause, frame stepping, and seek.

Recordings can be saved to and loaded from disk independently of the scene
project file.

### SceneIO (project format + replay format)

- **Project file** (human-readable): the canonical saved scene. A diffable,
  versioned, line-oriented text document that describes the authored objects,
  their parameters, hierarchy, editor metadata, and a format version. Chosen over
  the binary format for design files because it is reviewable and hand-editable
  (Webots `.wbt` precedent). The loader is fail-closed: it rejects bad or newer
  headers, malformed numeric fields, duplicate object IDs, and nested or
  unterminated object blocks without partially mutating the output model. Saving
  a loaded scene should produce stable text so review diffs stay meaningful.
- **Replay file** (binary): a sequence of DART 7 World binary snapshots
  plus per-frame timing, optimized for fast capture and restore.
- **UI layout/config**: saved separately from scene data (RViz `.rviz`
  precedent), so workspace layout is portable and does not pollute scene diffs.

### Logger / EventBus

A small logging facility with severity levels that the engine and (later) user
scripts write to, and that the Console panel renders. The EventBus delivers
engine change notifications to GUI panels so they refresh without polling.

## Object And Scene Model Mapping

The Scene Tree presents this hierarchy, mapped onto DART 7 World public
concepts:

```
World (root)
├── Rigid Bodies
│   └── <RigidBody>            -> sim::RigidBody (transform, vel, mass, inertia, force)
├── MultiBodies
│   └── <MultiBody>            -> sim::MultiBody
│       ├── Links
│       │   └── <Link>         -> sim::Link (world transform, parent joint)
│       └── Joints
│           └── <Joint>        -> sim::Joint (type, axis, position, velocity, DOFs)
├── Frames
│   └── <Frame>               -> sim::FreeFrame / sim::FixedFrame
└── Loop Closures
    └── <LoopClosure>         -> sim::LoopClosure (family, offsets, residual)
```

Object properties surfaced in the Inspector map directly to the DART 7
public accessors documented in
[`simulation_cpp_api.md`](simulation_cpp_api.md):
transform/translation/rotation/quaternion, linear/angular velocity, mass,
inertia, force/torque (RigidBody); link/joint enumeration and counts (MultiBody);
joint type/axis/position/velocity/DOF count (Joint); loop-closure family/offsets
and residual diagnostics (LoopClosure).

Geometry: until the DART 7 World exposes collision/visual shapes, authored
bodies carry **editor-side shape descriptors** (primitive box/sphere/cylinder/
capsule/plane) used only for rendering and picking via the render bridge. When
the DART 7 World gains a public shape API, the descriptors migrate to it
and the editor-side copy is removed.

## GUI Design

### Window layout

A dockable workspace (ImGui docking) with a default layout:

- Top: **Menu bar**.
- Left: **Scene Tree** panel.
- Center: **Viewport** (3D scene, gizmos, selection highlight).
- Right: **Inspector** panel.
- Bottom: **Console/Logger** and **Simulation controls + Timeline**.

Layout is user-rearrangeable and persisted via the UI config file.

### Menu bar

- **File**: New, Open Project, Save / Save As, Import (disabled until loaders
  exist), Open Recording, Save Recording, Exit. Open Project and Save As invoke
  the native desktop file picker directly, parented to the GUI window. If the
  platform dialog backend is unavailable, or a selected path fails to load/save,
  the editor opens an in-app path modal with a Browse button. The file-dialog
  boundary remains injectable so path selection is testable without launching
  the renderer.
- **Edit**: Undo, Redo, frame attach/detach and link parent/root relationship
  actions, Delete, Rename, (later) Duplicate, Cut/Copy/Paste. Frame
  attach/detach goes through undoable commands that preserve world transforms
  and convert detached fixed frames to root free frames. Link relationship
  actions keep link hierarchy changes inside the owning multibody and reject
  cycles or cross-multibody parents.
- **Create**: a tested, context-sensitive action model for primitive rigid bodies
  (Box/Sphere/Cylinder/Capsule/Plane), MultiBody, root links, fixed/revolute/
  prismatic child links, frames, and starter example scenes. Example creation is
  grouped as single undoable transactions so users can explore and discard a
  preset without walking back each object. Fixed frames require an existing
  frame-like parent because the DART 7 World does not allow direct world
  attachment. Loop Closure and sensors remain deferred until the public API
  exposes the necessary concepts.
- **Simulate**: Play, Pause, Step, Reset, Set Time Step, (record) Start/Stop
  Recording.
- **View**: toggle panels, camera presets (Orbit/Top/Front/Side), show grid,
  contacts, frames, inertia, wireframe.
- **Help**: About, shortcuts.

Menu items invoke engine commands/controller actions; they never mutate state
directly.

### Panels (each is a stateless mirror over the engine)

- **Scene Tree**: hierarchical view from the ObjectManager; Add (+) typed-create
  dialog; inline rename; drag re-parent; per-row visibility toggle; multi-select;
  selection synced to SelectionManager; delete via command.
- **Inspector**: context-sensitive properties for the primary selection are
  generated from a typed action/view-model seam. Current fields cover transform
  translation, rigid-body mass, child-link joint position, editor-side shape
  dimensions, shape color, shape-type and joint-kind enum choices, child-link
  joint axes, Simulation Mode read-only state, and delete. Edits emit undoable
  commands with live apply; future fields should extend the metadata seam rather
  than adding direct `editor.cpp` branches.
- **Console/Logger**: severity-filtered log stream from the engine Logger. A
  future Python REPL slot is reserved (CoppeliaSim/Isaac precedent) but not in
  v1.
- **Simulation controls + Timeline**: Play/Pause/Step/Reset, time-step field,
  real-time-factor and sim/wall/step readouts; record toggle; a timeline
  scrubber bound to the Player for replay.
- **Viewport**: renders the DART 7 World through the render bridge; orbit
  camera; pick-select (single + box) writing to SelectionManager; transform
  gizmos that emit `SetTransformCommand` on release.

### Render bridge

A GUI-side adapter that, each frame, reads DART 7 frame world transforms
plus editor shape descriptors and produces the renderable set for `dart::gui`.
This replaces the legacy-World rendering path for the DART 7 scene and is
the only place that couples DART 7 World data to the renderer. It must stay
backend-hidden (no Filament types above the `dart::gui` boundary).

## File Formats

- **Project (`*.dartsim` or `*.json`/`*.xml`)**: human-readable, versioned.
  Top-level: format version, world settings (time step, gravity when exposed),
  and an ordered list of authored objects with their type, name, parameters
  (transform, dynamics, joint spec, shape descriptor), parent references, and
  editor metadata. Designed to be diff-friendly and forward-compatible via the
  version field. Exact schema is defined during Phase 1 implementation.
- **Recording (`*.dartrec` binary)**: header (format version, time step, frame
  count) followed by per-frame records (sim time, step index, DART 7 World
  binary snapshot via `saveBinary`). Restored with `loadBinary`.
- **UI layout/config**: ImGui `.ini`-style or a small app config; separate from
  scene data.

## DART 7 API Gaps And Co-Evolution

The simulator is a forcing function for the DART 7 World's promotion to the
DART 7 clean-break API. The following gaps block full simulator features and
should feed the DART 7 API design effort (see
PLAN-041 and PLAN-042):

| Simulator need                       | DART 7 World status       | Interim approach in dartsim                    |
| ------------------------------------ | ------------------------- | ---------------------------------------------- |
| Collision/visual shape access        | Deferred                  | Editor-side primitive shape descriptors        |
| Model loading (URDF/SDF/MJCF)        | Not present (binary only) | Defer Import menu; author primitives in v1     |
| Contact/constraint data for overlays | Deferred                  | No contact overlays until exposed              |
| Joint limits / control / actuators   | Deferred                  | Joint position/velocity editing only           |
| Sensors                              | Deferred                  | Out of scope until exposed                     |
| Gravity / world physics settings     | Verify in API             | Surface only settings the public API exposes   |
| World-level collection views         | Deferred                  | ObjectManager keeps its own ordered scene tree |

Each interim approach is built so that adopting the eventual public API is a
localized change (in the scene model / render bridge), not a GUI rewrite.

## Phasing

The v1 phases below are implemented (see
[Implementation Status](#implementation-status-as-built)). Roadmap state and any
remaining follow-ups live in `docs/plans/dashboard.md` (PLAN-101) and
`docs/plans/101-dartsim-gui-simulator.md`. At a high level:

1. **Engine skeleton** — managers, command stack, scene model over `sim::World`,
   events; headless unit tests; no UI changes.
2. **Render bridge + viewport** — render a DART 7 scene with
   primitives; selection/picking.
3. **Editor UI** — Scene Tree, Inspector, menu bar, Create/Edit via commands,
   undo/redo.
4. **Simulation control** — Edit/Simulation mode, Play/Pause/Step/Reset, RTF
   readouts.
5. **Project save/load** — human-readable project format round-trip.
6. **Record & replay** — recorder, player, timeline scrubber.
7. **Hardening + co-evolution** — adopt simulation shape/loader APIs as they
   land; documentation; headless screenshot CI for the new app.

## Risks And Open Questions

- **DART 7 API churn**: the DART 7 World is actively changing. The
  scene model and render bridge isolate the GUI from churn, but command/object
  APIs may need updates as the World API tightens.
- **Handle invalidation**: simulation handles can be invalidated (e.g.
  `World::clear()`); the ObjectManager must own stable editor IDs and rebind
  handles after undo/redo and Reset.
- **Project format choice (JSON vs XML)**: decided at Phase 5 start; JSON is the
  leaning default for tooling simplicity. Recorded here so the decision is not
  re-litigated silently.
- **Threading**: running stepping off the GUI thread improves responsiveness but
  needs a defined hand-off; v1 may keep stepping on the GUI thread behind the
  controller and revisit with evidence.
- **Engine location**: kept in the `dartsim/` application folder; if other
  apps or Python need the engine, revisit promotion to a library + bindings.

## References

- Experimental API design:
  [`simulation_cpp_api.md`](simulation_cpp_api.md),
  [`simulation_python_api.md`](simulation_python_api.md)
- GUI rendering policy: `docs/onboarding/gui-rendering.md` (PLAN-060,
  backend-hidden renderer)
- Prior art: Gazebo `gz-sim`, NVIDIA Isaac Sim (Omniverse Kit), CoppeliaSim,
  Webots, MuJoCo `simulate`, SAPIEN, RViz, Drake, Blender
