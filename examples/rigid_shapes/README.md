# Rigid Shapes Example

## Summary

- Goal: inspect rigid collision shapes loaded from `shapes.skel` and spawn
  additional random rigid bodies at runtime.
- Concepts/APIs: SKEL loading, collision detector selection, max-contact
  configuration, ground-thickness adjustment, keyboard actions,
  example-owned contact-point markers, and public `dart::gui` panels.
- Expected output: the historical rigid-shape scene with optional spawned
  boxes, ellipsoids, cylinders, convex meshes, and red contact markers.
- Controls: `q` spawns a random cube, `w` spawns a random ellipsoid, `e`
  spawns a random cylinder, `r` spawns a random convex mesh, `a` deletes the
  latest spawned object, and `c` toggles contact points. Space toggles
  simulation and `n` steps once while paused.

## Run

From the repository root:

```bash
pixi run ex rigid_shapes
```

Rigid-shapes-specific options are parsed before the promoted viewer options:

```bash
pixi run ex rigid_shapes --collision-detector dart --max-contacts 128 --ground-thickness 0.1
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex rigid_shapes --headless --frames 2 --screenshot /tmp/rigid_shapes.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex rigid_shapes --headless --frames 3 --out /tmp/rigid_shapes_frames
```

## Build Instructions

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

Launch the executable from the build directory above:

```bash
./rigid_shapes
```

The standalone executable uses the same promoted `dart::gui` viewer and the
same shape spawning, contact-toggle, collision-detector, and capture controls.
