# DART Python Examples

Per PLAN-103, Python is DART's primary, growing example surface. The consolidated
home is the headless `dart-demos` Python app: one runner, one registry, many
scenes — see [`demos/README.md`](demos/README.md). C++ `dart-demos`
(PLAN-102) is frozen.

## Layout

- [`demos/`](demos/) — the headless scene-registry runner (`dart-demos`
  Python) and its scene modules; this is where new example content lands.
- A notebook gallery for Colab (PLAN-012) will live in `python/tutorials/` and
  import the scene modules from `demos/` (single source).

## Run

```bash
pixi run py-demos                                # run the first scene
pixi run py-demos -- --scene sx_articulated      # select a scene by id
pixi run py-demos -- --cycle-scenes --frames 4   # cycle every scene and exit
pixi run py-demos -- --list                      # print the catalog
```

Without pixi, from the repo root:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
    python -m examples.demos --scene hello_world --frames 5
```

## Add a scene

See [`demos/README.md`](demos/README.md): drop a `scenes/<name>.py` module
exposing a `SCENE` constant, append it in `demos/registry.py`. No CMake edits
are required.

## GUI and interactive viewer

`dartpy` exposes only headless GUI utilities (`dartpy.gui` — renderable
descriptors). There is no interactive viewer binding by design (PLAN-103); use
the C++ `dartsim` editor (PLAN-101) for interactive scene authoring.
