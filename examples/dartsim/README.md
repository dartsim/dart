# dartsim

`dartsim` is the application-level DART simulation viewer. The DART libraries
remain the project and library surface; `dartsim` is the executable used to run
interactive or headless GUI scenes.

## Usage

```bash
pixi run ex dartsim
pixi run ex dartsim --scene boxes
pixi run ex dartsim --headless --frames 10 --screenshot /tmp/dartsim.ppm
```

Historical GUI examples such as `hello_world`, `rigid_cubes`,
`drag_and_drop`, and `imgui` are restored as thin `dart::gui` launchers with
their own executable names:

```bash
pixi run ex hello_world
pixi run ex rigid_cubes --headless --frames 10 --screenshot /tmp/rigid_cubes.ppm
```

The renderer implementation is private to `dart::gui`; example code should
include `<dart/gui/application.hpp>` and call `dart::gui::runApplication(...)`.
