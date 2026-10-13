# dartpy Examples README

Most of dartpy's example scripts live in one consolidated package, `demos`,
so there is a single place to browse and run them. Three examples remain
standalone because they demonstrate something the consolidated catalog does
not.

## `demos`: the consolidated example catalog

`demos/` is a runnable package (`python -m examples.demos`) that bundles the
dartpy ports of the former single-purpose scripts (rigid cubes/chains/loops,
drag-and-drop, biped stand, operational-space control, atlas puppet,
and fixed-base Atlas v5 and Unitree G1 inspection) as selectable scenes.
`dartpy.viewer.run()` runs the catalog on the same native app host as
`dart-demos`, with a categorized navigator, simulation toolbar, dockable
scene panels, and diagnostics. Python supplies worlds and callbacks; the
native host draws the UI and renders the worlds with Filament.

Run it:

    $ pixi run py-demos

or manually, with dartpy (`pixi run build-py-dev` builds it):

    $ cd python
    $ PYTHONPATH=../build/default/cpp/Release/python/dartpy python -m examples.demos

Useful flags (`python -m examples.demos --help` for the full list):

    --list-scenes                  Print the catalog and exit.
    --scene <id>                   Start on a specific scene.
    --cycle-scenes --frames 2       Build and step every scene without a graphics device.
    --headless --shot <path>        Capture a PNG; Linux needs no display.

Each scene's source lives under `demos/scenes/` and contributes a
`dartpy.viewer.Scene` whose factory returns a `SceneSetup`. Instructions
appear in its panel, and scene key actions also appear as buttons. The
catalog follows the C++ scene order and metadata; the existing Python
scene behavior is retained while further ports are developed.

The app host requires the bundled docking ImGui. If `dartpy.viewer` is absent,
rebuild with `-DDART_USE_SYSTEM_IMGUI=OFF` (`pixi run config` passes this option).
The humanoid scenes are registered when `ModelResourceRetriever` is available.
Prefetch their models with `pixi run fetch-robot-assets`; missing offline assets
are reported as skipped scenes and do not fail a scene cycle.

## Standalone examples

- **`filament`** — a minimal world using the `dart.gui.Viewer`. Run it with
  `pixi run py-ex filament`, which builds dartpy first. The
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

Run any example's `main.py` with dartpy on `PYTHONPATH`.
`pixi run build-py-dev` builds the all-in-one nanobind extension, which always
includes IO and `dartpy.gui`. `pixi run test` and `test-py` keep the GUI enabled
and skip device-dependent rendering tests:

    $ pixi run py-ex hello_world
    $ pixi run py-ex ssik_analytical_ik

or directly:

    $ PYTHONPATH=build/default/cpp/Release/python/dartpy python python/examples/hello_world/main.py

Follow the instructions detailed in the console, or see each example's own
docstring/README for details.
