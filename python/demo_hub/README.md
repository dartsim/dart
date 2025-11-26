# DART Python Demo Hub

A consolidated Python demo browser built on `dartpy`. Scenes live under
`python/demo_hub/scenes/<name>` and are registered via `core/registry.py` so
you can browse and run them from one entrypoint.

## Run (GUI by default)

```bash
pixi run py-ex demo_hub                  # GUI shell (defaults to hello_world)
pixi run py-ex demo_hub -- --scene pendulum --dt 0.005

# Headless CLI
pixi run py-ex demo_hub -- --headless --list
pixi run py-ex demo_hub -- --headless --scene pendulum --steps 400
pixi run py-ex demo_hub -- --headless --scene pendulum --record /tmp/pendulum.jsonl
```

GLFW and PyOpenGL ship via the pixi environment. imgui/pyimgui is not yet
available for Python 3.14, so the launcher will fall back to headless mode
until a compatible wheel lands. Once available, add it with
`pixi add --pypi "imgui[glfw]"`.

For debugging, use the pixi helpers:

```bash
pixi run debug-py            # debugpy waiting for an IDE to attach
pixi run debug-py-native     # gdb/lldb for stepping into native code
```

## Adding scenes

1. Create `python/demo_hub/scenes/<your_scene>/scene.py` defining a `Scene`
   subclass with `metadata`, `setup()`, and `update()`.
2. Keep assets in that scene folder when possible.
3. The registry auto-discovers `*.scene` modules under `demo_hub.scenes` and
   registers any `Scene` subclasses that expose `metadata`.

## GUI scaffold

A minimal ImGui + GLFW shell lives in `demo_hub/gui/app.py` (no 3D rendering
yet; it drives scenes and shows their state). The GUI shell lets you switch
scenes, play/pause/step/reset, and toggle recording to a JSONL file (per-step
`export_state` payload).
