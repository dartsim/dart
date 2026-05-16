# Heightmap Example

## Summary

- Goal: regenerate a DART heightmap interactively and compare heightmap contact
  alignment against a reference box.
- Concepts/APIs: `dart::dynamics::HeightmapShaped`, world-owned
  `SimpleFrame`s, collision heightmaps, and `dart::gui` panels.
- Expected output: an interactive heightmap surface, grid, reference markers,
  or an alignment scene with falling ball grids.
- Controls: use the panel to play, pause, step, exit, toggle the terrain/grid,
  and adjust heightmap resolution, size, and Z range. `--demo alignment` opens
  the contact-alignment scene.

## Run

From the source tree:

```bash
pixi run ex heightmap
```

Alignment mode:

```bash
pixi run ex heightmap --demo alignment
```

Headless capture is supported through the promoted `dart::gui` runner:

```bash
pixi run ex heightmap --headless --frames 2 --screenshot /tmp/heightmap.ppm
```

Common `dart::gui` flags such as `--width`, `--height`, `--gui-scale`,
`--hide-ui`, and `--out` also apply.
