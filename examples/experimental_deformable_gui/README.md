# Experimental Deformable GUI

Visual smoke scene for the experimental deformable-body slice. It renders a
spring net driven by `dart::simulation::experimental::World`, with two fixed
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
