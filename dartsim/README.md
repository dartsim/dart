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
  event bus, a logger, an Edit/Run simulation controller, record/replay, and a
  human-readable project format. Unit-tested by `UNIT_dartsim_engine`.
- `ui/` — panel layer wiring the engine to the backend-hidden `dart::gui`
  visualization library (menu bar, Scene Tree, Inspector, Console, simulation
  controls, replay timeline).
- `app/` — the thin `main.cpp` executable entry calling `dartsim::ui::runEditor`.

Architecture and rationale: `docs/design/dartsim_gui_simulator.md`. Developer
overview: `docs/onboarding/gui-rendering.md`.

## Usage

```bash
pixi run ex dartsim
pixi run ex dartsim --headless --frames 10 --screenshot /tmp/dartsim.ppm
```

Passing `--scene <name>` renders a built-in `dart::gui` example fixture instead
of the editor (used by the headless GUI smoke tests):

```bash
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
