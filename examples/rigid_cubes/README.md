# Rigid Cubes Example

This example loads `dart://sample/skel/cubes.skel`, applies decaying directional
forces to the center cube, and visualizes the scene through the promoted
`dart::gui` runner.

Run it with:

```bash
pixi run ex rigid_cubes
```

Headless capture:

```bash
pixi run ex rigid_cubes --headless --frames 2 --screenshot /tmp/rigid_cubes.ppm
pixi run ex rigid_cubes --headless --frames 10 --out /tmp/rigid_cubes_frames
```

Controls:

- Space: play/pause
- `p`: playback/stop through the promoted lifecycle toggle
- `v`: toggle visualization marker state
- `1`: apply -X force
- `2`: apply +X force
- `3`: apply -Z force
- `4`: apply +Z force
- `n`: step one frame while paused
- Escape: exit

The default launch size is 640x480, matching the historical viewer example.
