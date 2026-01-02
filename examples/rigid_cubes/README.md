# Rigid Cubes Example

## Summary

- Goal: apply external forces to a simple rigid-cube scene in an OSG viewer.
- Concepts/APIs: `dart::io::readWorld`, `gui::Viewer`,
  `gui::RealTimeWorldNode`, custom event handling.
- Expected output: an OSG viewer with cubes falling under gravity; key presses
  nudge a cube with directional forces.

## Controls

- Space: toggle simulation on/off.
- p: toggle playback/stop mode.
- v: toggle visualization markers.
- 1/2: apply negative/positive X force.
- 3/4: apply negative/positive Z force.

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
