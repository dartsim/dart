# DART Python Examples

Per PLAN-103, Python is DART's primary, growing example surface. The consolidated
home is the `dart-demos` Python app: one runner, one registry, many scenes
hosted by the shared Filament viewer — see [`demos/README.md`](demos/README.md).
C++ `dart-demos` (PLAN-102) is now the smaller World-only C++ companion.

## Layout

- [`demos/`](demos/) — the scene-registry runner (`dart-demos` Python) and its
  scene modules; this is where new example content lands.
- A notebook gallery for Colab (PLAN-012) will live in `python/tutorials/` and
  import the scene modules from `demos/` (single source).

## Run

```bash
pixi run py-demos                                # run the first scene
pixi run py-demos -- --scene articulated         # select a scene by id
pixi run py-demos -- --cycle-scenes --frames 4   # cycle every scene and exit
pixi run py-demos -- --list                      # print the catalog
pixi run py-demo-capture -- --scene articulated --show-ui --video
```

Without pixi, from the repo root:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
    .pixi/envs/default/bin/python -m examples.demos --scene articulated --frames 5
```

## Add a scene

See [`demos/README.md`](demos/README.md): drop a `scenes/<name>.py` module
exposing a `SCENE` constant, append it in `demos/registry.py`. No CMake edits
are required.

## GUI and interactive viewer

`dartpy.gui.run_demos` opens the same Filament multi-scene viewer used by C++
`dart-demos`. Scene modules can attach custom `ScenePanel` callbacks for
per-example controls and diagnostics; the runner renders them through DART's
renderer-neutral `PanelBuilder`. Use `--headless` for deterministic
screenshot/frame capture and the C++ `dartsim` editor (PLAN-101) for scene
authoring.
