# Collision Sandbox

Filament-backed native collision visual debugger for pair-by-pair contact and
broad-phase inspection.

The pair coverage table lists every registered case and marks unsupported rows
as placeholders.

Left-drag object gizmo arrows, planes, and rings to translate or rotate each
shape. The panel keeps pair, shape-size, filtering, contact-limit, and
broad-phase controls, while pose edits happen through the scene gizmos.
When a contact query produces contact points, the colliding object visuals are
rendered translucent so the point, normal, and depth overlays remain visible.
Minimal name tags are enabled by default for contact, distance, AABB,
broad-phase tree, candidate-pair, spatial-cell, and sweep-axis overlays; turn
off `Name Tags` in the panel when the scene needs less annotation.

Useful smoke command:

```bash
pixi run ex collision_sandbox --headless --frames 2 --pair sphere_box \
  --screenshot /tmp/collision_sandbox.ppm
```

Broad-phase overlays can be selected from the panel or headless command line:

```bash
pixi run ex collision_sandbox --headless --frames 2 --pair sphere_box \
  --broad-phase spatial_hash --screenshot /tmp/collision_sandbox_hash.ppm
```

Use `--filter-pair` to apply mismatched native collision group/mask bits and
inspect the filtered rendering state.
