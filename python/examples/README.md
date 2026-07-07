# dartpy Examples README

Most of dartpy's example scripts live in one consolidated package, `demos`,
so there is a single place to browse and run them. Two examples remain
standalone because they demonstrate something the consolidated catalog does
not.

## `demos`: the consolidated example catalog

`demos/` is a runnable package (`python -m examples.demos`) that bundles the
dartpy ports of the former single-purpose scripts (rigid cubes/chains/loops,
drag-and-drop, biped stand, operational-space control, atlas puppet,
contacts + point cloud) as selectable scenes in one keyboard-driven
`dart.gui.osg.Viewer`. dartpy has no usable ImGui bindings, so unlike the C++
`dart-demos` host this is plain-keyboard navigation (number keys / n/p to
switch scenes, `h` for help) rather than a panel UI.

Run it:

    $ pixi run py-demos

or manually:

    $ cd python
    $ PYTHONPATH=../build/default/cpp/Release/python/dartpy python -m examples.demos

Useful flags (`python -m examples.demos --help` for the full list):

    --list            Print the catalog and exit.
    --scene <id>       Start on a specific scene.
    --cycle            Headless smoke test: step every scene and exit.

Each scene's source lives under `demos/scenes/`.

## Standalone examples

- **`hello_world`** — the canonical minimal "first program"; start here if
  you are new to dartpy.
- **`ssik_analytical_ik`** — plugs an external
  [ssik](https://github.com/personalrobotics/ssik) analytical IK solver into
  DART's native `InverseKinematics.setPythonAnalytical` and drives it live
  from a draggable `InteractiveFrame` in an `ImGuiViewer`. It is kept
  standalone (not folded into `demos`) because it needs the ImGui viewer
  workspace and the optional `ssik` package that `demos` intentionally does
  without; it is the Python counterpart of the C++ `ssik_ik_gui` scene in
  the `dart-demos` catalog. See
  `docs/readthedocs/dartpy/user_guide/examples.rst` for the full walkthrough.

## Execute Instructions

Run any example's `main.py` with dartpy on `PYTHONPATH`:

    $ pixi run py-ex hello_world
    $ pixi run py-ex ssik_analytical_ik

or directly:

    $ PYTHONPATH=build/default/cpp/Release/python/dartpy python python/examples/hello_world/main.py

Follow the instructions detailed in the console, or see each example's own
docstring/README for details.
