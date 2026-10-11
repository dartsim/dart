# dartpy Examples README

Most of dartpy's example scripts live in one consolidated package, `demos`,
so there is a single place to browse and run them. Three examples remain
standalone because they demonstrate something the consolidated catalog does
not.

## `demos`: the consolidated example catalog

`demos/` is a runnable package (`python -m examples.demos`) that bundles the
dartpy ports of the former single-purpose scripts (rigid cubes/chains/loops,
drag-and-drop, biped stand, operational-space control, atlas puppet,
contacts + point cloud) as selectable scenes in one keyboard-driven
viewer using the Filament-backed `dart.gui.Viewer`.
dartpy has no usable ImGui bindings, so unlike the C++
`dart-demos` host this is plain-keyboard navigation (number keys / n/p to
switch scenes, `h` for help) rather than a panel UI.

Run it:

    $ pixi run py-demos

or manually, with a GUI-enabled dartpy (`pixi run build-py-dev` builds one):

    $ cd python
    $ PYTHONPATH=../build/default/cpp/Release/python/dartpy python -m examples.demos

Useful flags (`python -m examples.demos --help` for the full list):

    --list            Print the catalog and exit.
    --scene <id>       Start on a specific scene.
    --cycle            Headless smoke test: step every scene and exit.
    --shot <path>      Capture a PNG; Linux needs no display.

Each scene's source lives under `demos/scenes/`.

## Standalone examples

- **`filament`** — a minimal world using the `dart.gui.Viewer`. Run it with
  `pixi run py-ex filament`, which builds a GUI-enabled dartpy first. The
  script's `--capture frame.png` option writes a PNG (without a display on
  Linux); capture dimensions are physical pixels.
- **`hello_world`** — the canonical minimal "first program"; start here if
  you are new to dartpy.
- **`ssik_analytical_ik`** — plugs an external
  [ssik](https://github.com/personalrobotics/ssik) analytical IK solver into
  DART's native `InverseKinematics.setPythonAnalytical` and drives it live
  from a draggable `InteractiveFrame` in a Filament `Viewer`. It stays
  standalone because it needs the optional `ssik` package; it is the Python
  counterpart of the C++ `ssik_ik_gui` scene in the `dart-demos` catalog. See
  `docs/readthedocs/dartpy/user_guide/examples.rst` for the full walkthrough.

## Execute Instructions

Run any example's `main.py` with a GUI-enabled dartpy on `PYTHONPATH`
(`pixi run build-py-dev` builds one; `pixi run test` or `test-py` reconfigures
that tree without the GUI):

    $ pixi run py-ex hello_world
    $ pixi run py-ex ssik_analytical_ik

or directly:

    $ PYTHONPATH=build/default/cpp/Release/python/dartpy python python/examples/hello_world/main.py

Follow the instructions detailed in the console, or see each example's own
docstring/README for details.
