# Polyhedron Visual Example

## Summary

- Goal: render a convex polyhedron from a vertex list.
- Concepts/APIs: `dart::gui::ApplicationOptions`, `ConvexMeshShape`,
  `LineSegmentShape`, source-owned DART worlds, and promoted GUI run defaults.
- Expected output: a translucent convex hull, dark wireframe, and reference
  grid in the Filament-backed `dart::gui` viewer.

## Run In Tree

From the repository root:

```bash
pixi run ex polyhedron_visual
```

Useful promoted runner options:

```bash
pixi run ex polyhedron_visual --headless --frames 2 --screenshot /tmp/polyhedron.ppm
pixi run ex polyhedron_visual --width 960 --height 540
```

The example defaults to the historical 640x480 viewer and preserves
command-line overrides handled by the shared `dart::gui` runner.

## Controls

Use the standard promoted viewer controls: left drag orbits, right or middle
drag pans, the wheel zooms, and Escape exits.
