# Soft Bodies Example

## Summary

- Goal: demonstrate soft-body simulation with recorded playback.
- Concepts/APIs: SKEL loading, soft-body state recording, renderer-neutral
  keyboard actions, public `dart::gui` panels, and headless capture.
- Expected output: the historical `softBodies.skel` scene rendered in the
  maintained Filament viewer.
- Controls: `[` and `]` move playback backward/forward one frame, `{` and `}`
  move backward/forward ten frames, `r` restarts playback, and `\` jumps to the
  latest recorded frame. Space toggles simulation and `n` steps once while
  paused.

## Run

From the repository root:

```bash
pixi run ex soft_bodies
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex soft_bodies --headless --frames 2 --screenshot /tmp/soft_bodies.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex soft_bodies --headless --frames 3 --out /tmp/soft_bodies_frames
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
./soft_bodies
```

The standalone executable uses the same promoted `dart::gui` viewer and the
same recorded-playback controls.
