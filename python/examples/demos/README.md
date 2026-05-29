# dart-demos (Python)

A headless scene-registry runner that hosts DART's Python demo scenes. This is
the Python-first surface from PLAN-103; C++ `dart-demos` (PLAN-102) stays frozen.

## Run

```bash
pixi run py-demos                                # default: run the first scene
pixi run py-demos -- --scene sx_articulated      # select a scene by id
pixi run py-demos -- --cycle-scenes --frames 4   # cycle through every scene
pixi run py-demos -- --list                      # print the scene catalog
```

Or directly (without pixi), from the repo root:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
    python -m examples.demos --scene hello_world --frames 5
```

## CLI

The runner mirrors C++ `dart-demos` to keep the cross-language UX consistent:

| Flag                | Meaning                                                    |
| ------------------- | ---------------------------------------------------------- |
| `--scene <id>`      | Select the initial scene (default: first registered)       |
| `--cycle-scenes`    | Advance through every scene for `--frames` frames and exit |
| `--frames N`        | Per-scene step budget (default 60 single, 4 cycle)         |
| `--screenshot PATH` | Phase 1: write a JSON state snapshot at PATH               |
| `--headless`        | Accepted no-op (Python demos are always headless)          |
| `--list`            | Print the catalog and exit                                 |

`--screenshot` writes a real non-blank PPM once Phase 2 wires the `dartpy.gui`
headless screenshot path; today it writes a deterministic JSON state snapshot
so the contract works end-to-end.

## Add a scene

1. Create `scenes/<name>.py` defining a module-level `SCENE` of type
   `PythonDemoScene` whose `build()` returns a `SceneSetup` (a `world` plus an
   optional custom `step(n)` and `info` dict).
2. Import it in `registry.py` and append it to the ordered list in
   `make_demo_scenes()`, placed within its category group.

That's the only change needed; no CMake edits, no test edits beyond the cycle
smoke (which iterates the registry).
