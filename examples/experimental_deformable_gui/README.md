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
