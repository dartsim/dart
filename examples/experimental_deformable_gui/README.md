# Experimental Deformable GUI

Visual smoke scene for the experimental deformable-body slice. It renders a
spring net driven by `dart::simulation::experimental::World`, with a shaded
deformable surface mesh built through the reusable `dart::gui` deformable
surface renderable helper, optional point-mass/spring debug overlays, two fixed
anchor nodes, dynamic spring nodes, and an explicitly opted-in static ground
barrier.

Run the long-horizon headless check with:

```bash
pixi run ex experimental_deformable_gui --headless --frames 180 \
  --width 960 --height 540 --out /tmp/experimental_deformable_gui_frames \
  --screenshot /tmp/experimental_deformable_gui.ppm
```

Use the `Reset Scene` panel button or press `r` in the interactive window to
reset the scene.

The default view renders the surface mesh, point masses, and spring edges
together. Use `--deformable-view surface`, `--deformable-view points`, or
`--deformable-view combined` for headless comparisons, or toggle `Surface Mesh`,
`Point Masses`, and `Spring Edges` in the interactive panel.

The example can also replay the contact-free deformable scene-loader subset:

```bash
build/default/cpp/Release/bin/experimental_deformable_gui --headless \
  --frames 180 --width 960 --height 540 \
  --out /tmp/ipc_scene_replay_gui_combined \
  --screenshot /tmp/ipc_scene_replay_gui_combined.ppm \
  --deformable-scene /path/to/scene.txt \
  --diagnostics-json /tmp/deformable_scene_diagnostics.json \
  --deformable-view combined
```

Repeat the scene replay command with `--deformable-view surface` and
`--deformable-view points` and distinct `--out` directories when collecting PR
visual evidence.

Scene replay currently loads tetrahedral mesh state, generated spring edges,
and scripted Dirichlet/Neumann controls. It intentionally ignores or reports
unsupported metadata, contact, and friction directives until later IPC solver
slices land.
