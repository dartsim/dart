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

### Creating Video from Frames

Use ffmpeg to convert captured PNG frames to video:

```bash
# Basic MP4 (H.264)
ffmpeg -framerate 30 -i output/frame_%06d.png -c:v libx264 -pix_fmt yuv420p output.mp4

# High quality with CRF (lower = better quality, 18-23 recommended)
ffmpeg -framerate 60 -i output/frame_%06d.png -c:v libx264 -crf 18 -pix_fmt yuv420p output.mp4

# GIF (for documentation)
ffmpeg -framerate 30 -i output/frame_%06d.png -vf "fps=15,scale=480:-1" output.gif
```

### Performance Notes

**Headless rendering** uses OSG's pbuffer backend, which requires a GPU with
OpenGL support. Performance characteristics:

| Mode           | Typical FPS | Use Case               |
| -------------- | ----------- | ---------------------- |
| Interactive    | 60          | Development, debugging |
| Headless (GPU) | 100-500+    | CI, batch rendering    |

**Tips for faster batch rendering:**

- Use `--width`/`--height` to reduce resolution when high quality isn't needed
- Omit `--out` if you only need simulation validation (no PNG I/O overhead)
- Use `--seed` for deterministic comparisons between runs

**CI considerations:**

- Headless mode requires a display server or virtual framebuffer (Xvfb)
- CI jobs use `xvfb-run` wrapper (see `.github/workflows/ci_ubuntu.yml`)
- For true software rendering (no GPU), OSMesa support is planned but not yet
  implemented

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
