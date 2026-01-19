# Rigid Cubes Example

## Summary

- Goal: apply external forces to a simple rigid-cube scene in an OSG viewer.
- Concepts/APIs: `dart::io::readWorld`, `gui::Viewer`,
  `gui::RealTimeWorldNode`, custom event handling.
- Expected output: an OSG viewer with cubes falling under gravity; key presses
  nudge a cube with directional forces.

## Controls (Interactive Mode)

- Space: toggle simulation on/off.
- p: toggle playback/stop mode.
- v: toggle visualization markers.
- 1/2: apply negative/positive X force.
- 3/4: apply negative/positive Z force.

## Command-Line Options

```
--headless        Run without display window (for CI/testing)
--frames <n>      Run for n frames then exit (required with --headless)
--out <dir>       Output directory for captured frames
--width <n>       Viewport width (default: 640)
--height <n>      Viewport height (default: 480)
-h, --help        Show help
```

### Headless Mode Example

```bash
# Run for 100 frames and save to ./output/
./rigid_cubes --headless --frames 100 --out ./output/

# Run with custom resolution
./rigid_cubes --headless --frames 50 --out ./output/ --width 1920 --height 1080
```

## Build Instructions

This project requires DART with OSG support. From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

Launch the executable from the build directory:

```bash
./rigid_cubes
```

Follow the control instructions displayed in the console and viewer window.
