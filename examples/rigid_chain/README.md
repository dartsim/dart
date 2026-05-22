# Rigid Chain Example

This example loads `dart://sample/skel/chain.skel`, gives the articulated chain
a random initial pose, applies damping before each simulation step, and
visualizes the result through the promoted `dart::gui` runner.

Run it with:

```bash
pixi run ex rigid_chain
```

Headless capture:

```bash
pixi run ex rigid_chain --headless --frames 2 --screenshot /tmp/rigid_chain.ppm
```

Controls:

- Space: play/pause
- `n`: step one frame while paused
- Escape: exit

The default launch size is 640x480, matching the historical viewer example.
