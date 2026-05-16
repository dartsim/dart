# Rigid Loop Example

This example loads `dart://sample/skel/chain.skel`, bends the chain into a
closed-loop pose, connects `link 6` and `link 10` with a ball-joint constraint,
and visualizes the constrained dynamics through the promoted `dart::gui`
runner.

Run it with:

```bash
pixi run ex rigid_loop
```

Headless capture:

```bash
pixi run ex rigid_loop --headless --frames 2 --screenshot /tmp/rigid_loop.ppm
```

Controls:

- Space: play/pause
- `n`: step one frame while paused
- Escape: exit

The red links are connected by a ball-joint constraint to form the loop. The
default launch size is 640x480, matching the historical viewer example.
