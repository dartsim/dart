# dartsim

`dartsim` is the standalone, general-purpose robotics multi-physics GUI
simulator built on DART's experimental World API. The DART libraries under
`dart/` remain the C++/Python library distribution; `dartsim` is distributed as
a runtime executable (via package managers), not as a library, so nothing here
is installed or exported as a public `dart::` component and runtime dependencies
are kept minimal.

## Layout

- `engine/` — headless editor engine (no GUI dependency): scene/object,
  selection, command (undo/redo, grouped macros), and name managers, a typed
  event bus, a logger, an Edit/Simulation mode controller, record/replay, and
  a human-readable project format. Unit-tested by `UNIT_dartsim_engine`.
- `ui/` — panel layer wiring the engine to the backend-hidden `dart::gui`
  visualization library (menu bar, Scene Tree, Inspector, Console, simulation
  controls, replay timeline).
- `app/` — the thin `main.cpp` executable entry calling `dartsim::ui::runEditor`.

Architecture and rationale: `docs/design/dartsim_gui_simulator.md`. Developer
overview: `docs/onboarding/gui-rendering.md`.

## Usage

Launch the editor as a dockable ImGui workspace:

```bash
pixi run dartsim
pixi run dartsim -- --width 1600 --height 900
```

The editor opens an **empty workspace** by default. Pass `--demo` to seed a
small sample scene (a box, a sphere, and a two-link revolute arm) for a quick
look at the panels:

```bash
pixi run dartsim -- --demo
```

Launching the editor with `pixi run dartsim` forces the ImGui docking branch
(`DART_USE_SYSTEM_IMGUI=OFF`) so the menu bar, Scene Tree, Inspector, Console,
and Simulation panels open in a default IDE layout and can be docked, split, and
rearranged, with the arrangement persisted in `imgui.ini`. Extra CLI flags pass
through after `--`. The Console panel logs `ImGui docking: enabled` on startup;
if it instead reports `UNAVAILABLE`, the binary was built against non-docking
ImGui — relaunch with `pixi run dartsim` to rebuild against the docking branch.

Because the GUI build links one ImGui, building the library or other GUI
examples (`pixi run build`, `pixi run config`, `pixi run ex <gui-example>`)
switches the shared build to system ImGui; the next editor launch transparently
rebuilds against the docking branch.

The editor inherits the shared `dart::gui` live performance HUD: press **F2**
(or launch with `--perf-hud`) to toggle an overlay with per-phase CPU/GPU frame
timing and the active renderer backend, which is useful for spotting cost in the
editor's per-frame scene extraction. Choose the renderer backend with
`--backend <name>` (or the `DART_FILAMENT_BACKEND` environment variable).

```bash
pixi run dartsim -- --perf-hud
```

Passing `--scene <name>` renders a built-in `dart::gui` scene fixture instead of
the editor (system ImGui, used by the headless GUI smoke tests):

```bash
pixi run ex dartsim --headless --frames 10 --screenshot /tmp/dartsim.ppm
pixi run ex dartsim --scene boxes
```

Historical GUI examples such as `hello_world`, `rigid_cubes`, `drag_and_drop`,
and `imgui` have their own executable names:

```bash
pixi run ex hello_world
pixi run ex rigid_cubes --headless --frames 10 --screenshot /tmp/rigid_cubes.ppm
```

The renderer implementation is private to `dart::gui`; code should include
`<dart/gui/application.hpp>` and call `dart::gui::runApplication(...)`, and use
`<dart/gui/panel.hpp>` for custom controls rather than including renderer or UI
toolkit headers directly.
