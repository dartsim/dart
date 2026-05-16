# Vehicle Example

## Summary

- Goal: drive a simple vehicle model with steering and wheel velocity control.
- Concepts/APIs: PD control, joint targets, keyboard input handling, and
  source-owned GUI panels.
- Expected output: a Filament-rendered car model with four wheels, ground, and
  obstacle boxes.
- Controls:
  - `w`: move forward
  - `s`: stop
  - `x`: move backward
  - `a`: rotate steering wheels to the left
  - `d`: rotate steering wheels to the right
  - Space: simulation on/off

## Run

From the repository root, use the in-tree runner:

```bash
pixi run ex vehicle
```

The example defaults to a 640x480 window. Shared GUI options such as
`--width`, `--height`, `--gui-scale`, `--screenshot`, and `--out` are supported:

```bash
pixi run ex vehicle --headless --frames 5 --screenshot /tmp/vehicle.ppm
pixi run ex vehicle --headless --frames 5 --out /tmp/vehicle_frames
```

## Build Instructions

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

From the build directory above:

```bash
./{generated_executable}
```

Follow the instructions detailed in the console.
