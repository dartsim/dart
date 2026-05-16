# Hello World Example

This example creates a single blue rigid box above a gray ground plane and
visualizes it through the promoted `dart::gui` runner.

Run it with:

```bash
pixi run ex hello_world
```

Headless capture:

```bash
pixi run ex hello_world --headless --frames 2 --screenshot /tmp/hello_world.ppm
```

Controls:

- Space: play/pause through the shared viewer control
- `n`: step one frame while paused
- Escape: exit

The default launch size is 640x480, matching the historical viewer example.
