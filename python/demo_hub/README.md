# DART Python Demo Hub

A consolidated Python demo browser built on `dartpy`. Scenes live under
`python/demo_hub/scenes/<name>` and are registered via `core/registry.py` so
you can browse and run them from one entrypoint.

## Run (headless for now)

```bash
pixi run demo-hub                    # run default scene (hello_world)
pixi run demo-hub -- --list          # list scenes
pixi run demo-hub -- --scene hello_world --steps 300 --dt 0.002
```

For debugging, use the pixi helpers:

```bash
pixi run debug-py            # debugpy waiting for an IDE to attach
pixi run debug-py-native     # gdb/lldb for stepping into native code
```

## Adding scenes

1. Create `python/demo_hub/scenes/<your_scene>/scene.py` defining a `Scene`
   subclass with `metadata`, `setup()`, and `update()`.
2. Register it in `core/registry.py` (temporarily static; can be made
   dynamic/plug-in later).
3. Keep assets in that scene folder when possible.
