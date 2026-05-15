# Rigid Cubes Example

## Summary

- Goal: inspect a dynamic cube-grid scene in the maintained Filament path.
- Concepts/APIs: Filament GUI dynamic body descriptors, `dart::io::readWorld`,
  `gui::Viewer`, `gui::RealTimeWorldNode`, and legacy custom event handling.
- Expected output: a Filament viewer with colored cubes falling under gravity.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, click selects renderables,
  Ctrl-left drag or arrow/PageUp/PageDown keys move selectable items, and
  Escape exits.

## Legacy Controls (Standalone)

- Space: toggle simulation on/off.
- p: toggle playback/stop mode.
- v: toggle visualization markers.
- 1/2: apply negative/positive X force.
- 3/4: apply negative/positive Z force.

## Command-Line Options

These options apply to the standalone legacy OSG executable:

```
--headless        Run without display window (for CI/testing)
--frames <n>      Run for n frames then exit (required with --headless)
--out <dir>       Output directory for captured frames
--width <n>       Viewport width (default: 640)
--height <n>      Viewport height (default: 480)
-h, --help        Show help
```

The recommended in-tree runner uses the Filament scene in
`examples/filament_gui`, so prefer `--screenshot <path>` instead of the legacy
`--out <dir>` frame-recording option when running through `pixi run ex`.

## Run In Tree

From the repository root:

```bash
pixi run ex rigid_cubes
```

This builds and runs `examples/filament_gui --scene boxes`, so the recommended
visual path no longer depends on the legacy OSG viewer. On Linux without a
display, the runner automatically uses headless defaults.

### Legacy Headless Mode Example

```bash
# Run for 100 frames and save to ./output/
./rigid_cubes --headless --frames 100 --out ./output/

# Run with custom resolution
./rigid_cubes --headless --frames 50 --out ./output/ --width 1920 --height 1080
```

### Creating Video from Legacy Frames

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

This project requires DART with OSG support when building the standalone legacy
executable. From this directory:

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

This launches the legacy OSG viewer. Follow the control instructions displayed
in the console and viewer window for force controls and frame recording.
