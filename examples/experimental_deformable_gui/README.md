# Experimental Deformable GUI

Visual smoke scene for the experimental deformable-body slice. It renders a
spring net driven by `dart::simulation::experimental::World`, with a shaded
deformable surface mesh built through the reusable `dart::gui` deformable
surface renderable helper, optional point-mass/spring debug overlays, two fixed
anchor nodes, dynamic spring nodes, and an explicitly opted-in static ground
barrier.

Run the long-horizon headless check with:

```bash
pixi run ex experimental_deformable_gui -- --headless --frames 180 \
  --width 960 --height 540 --out /tmp/experimental_deformable_gui_frames \
  --screenshot /tmp/experimental_deformable_gui.ppm
```

Use the `Reset Scene` panel button or press `r` in the interactive window to
reset the scene.

The default view renders the surface mesh, point masses, and spring edges
together. Use `--deformable-view surface`, `--deformable-view points`, or
`--deformable-view combined` for headless comparisons, or toggle `Surface Mesh`,
`Point Masses`, and `Spring Edges` in the interactive panel.

## Drape showcase scene

`--deformable-scene-kind drape` builds a 572-node deformable mat suspended above
a raised box at the scene center. Under gravity the mat drapes over the box (a
finite-footprint ground-barrier step) and settles onto the surrounding ground,
folding against itself where it bends over the edges. It exercises the IPC
deformable-contact pipeline that has landed so far — self-contact and
ground-barrier repulsion resolved by the projected-Newton solve at sparse scale
(past the former 256-node dense cap) — and is a DART-native showcase, not a
faithful reproduction of the paper's codimensional/FEM mat figures.

```bash
pixi run ex experimental_deformable_gui -- --headless --frames 200 \
  --width 900 --height 600 --deformable-scene-kind drape \
  --deformable-view combined \
  --screenshot /tmp/experimental_deformable_drape.ppm \
  --diagnostics-json /tmp/experimental_deformable_drape.json
```

The default built-in scene (`--deformable-scene-kind net`, the hanging spring
net) is used when the flag is omitted.

The example can also replay the contact-free deformable scene-loader subset:

```bash
pixi run ex experimental_deformable_gui -- --headless \
  --frames 180 --width 960 --height 540 \
  --out /tmp/ipc_scene_replay_gui_combined \
  --screenshot /tmp/ipc_scene_replay_gui_combined.ppm \
  --deformable-scene /path/to/scene.txt \
  --diagnostics-json /tmp/deformable_scene_diagnostics.json \
  --deformable-view combined
```

For surface-only evidence:

```bash
pixi run ex experimental_deformable_gui -- --headless \
  --frames 180 --width 960 --height 540 \
  --out /tmp/ipc_scene_replay_gui_surface \
  --screenshot /tmp/ipc_scene_replay_gui_surface.ppm \
  --deformable-scene /path/to/scene.txt \
  --diagnostics-json /tmp/deformable_scene_diagnostics_surface.json \
  --deformable-view surface
```

For point-mass-only evidence:

```bash
pixi run ex experimental_deformable_gui -- --headless \
  --frames 180 --width 960 --height 540 \
  --out /tmp/ipc_scene_replay_gui_points \
  --screenshot /tmp/ipc_scene_replay_gui_points.ppm \
  --deformable-scene /path/to/scene.txt \
  --diagnostics-json /tmp/deformable_scene_diagnostics_points.json \
  --deformable-view points
```

Scene replay currently loads tetrahedral mesh state, generated spring edges,
and scripted Dirichlet/Neumann controls. It intentionally ignores or reports
unsupported metadata, contact, and friction directives until later IPC solver
slices land.
