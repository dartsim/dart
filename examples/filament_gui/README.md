# Filament GUI Example

## Summary

- Goal: exercise the experimental Filament + GLFW visualization path.
- Concepts/APIs: Filament renderer setup, GLFW windowing, DART world stepping,
  dynamic shadows, and bounded headless capture.
- Expected output: a Filament window with DART boxes falling onto a ground
  plane, additional primitive/mesh/plane visuals, dynamic shadows, and a small
  Dear ImGui status panel.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, a left click without dragging
  selects a renderable, arrow/PageUp/PageDown keys nudge selected free-joint
  bodies or `SimpleFrame` visuals, Ctrl-left drag moves selected frame
  renderables in a camera-facing plane, Escape exits, and `--frames` limits
  rendered frame count. Use
  `--screenshot <path>` to write a binary PPM capture, and `--headless` to use
  Filament's headless swap-chain path without creating a GLFW window. Use
  `--scene drag-and-drop` to run the interaction fixture.

## Notes

- Requires DART configured with `DART_BUILD_GUI_FILAMENT=ON`.
- Links against `dart-gui-experimental` for backend-hidden scene extraction and
  picking; it does not require the legacy OSG `dart-gui` target when
  `DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF`.
- Requires a Filament install tree with headers, libraries, and matching
  `matc`; set `Filament_ROOT` if it is not discoverable.
- This is an experimental implementation using the constrained
  `dart::gui::experimental` scene layer, not a stable public `dart::gui` API.
- The upstream Filament Linux archive currently links against libc++. In the
  default DART Pixi environment, configure needs a compatible Filament build or
  libc++/libc++abi libraries.
- The ImGui overlay is intentionally minimal. It renders the built-in status
  panel, but does not yet support the full Dear ImGui renderer-backend feature
  set.
- The example renders built-in primitives, TriMesh-backed `MeshShape` visuals,
  an imported WAM Collada mesh, the full WAM URDF skeleton, an Atlas DAE torso
  mesh, a full Atlas SDF robot fixture, a four-panel glTF/PBR environment
  layout, and a finite checker-textured `PlaneShape` proxy. Mesh rendering uses
  authored UVs, submesh material ranges, material colors,
  metallic/roughness factors, emissive color, and typed PNG/JPEG texture images
  for base color, metallic, roughness, combined metallic-roughness, normal,
  occlusion, and emissive maps when `MeshShape` provides them. The scene also
  loads the checked-in single- and multi-material glTF PBR fixtures and routes
  alpha-bearing solid, textured, and mesh visuals through transparent lit
  material variants. Broader robot/environment visual review is still future
  promotion work.
- The view enables HDR buffer quality, screen-space ambient occlusion, temporal
  anti-aliasing, FXAA, temporal dithering, and multi-sample anti-aliasing for
  windowed runs in the current visual smoke fixture.
- A simple 3D grid, world/body frame axes, contact markers, contact normals,
  and contact force vectors are generated as `dart-gui-experimental` debug
  descriptors and rendered as Filament line primitives. The debug-line material
  uses transparent blending for alpha-bearing overlay colors.
- Basic click-to-select uses renderer-independent DART descriptor picking and
  reports the selected shape in the built-in panel while drawing a bounds
  overlay around it. Selected free-joint bodies and `SimpleFrame` visuals can
  be moved with keyboard nudges or Ctrl-left camera-plane dragging through the
  same backend-hidden descriptor layer. The `--scene drag-and-drop` fixture
  uses that path for a legacy-style `SimpleFrame` anchor, child frame, and axis
  marker layout.

## Build Instructions

From the repository root:

```bash
DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  Filament_ROOT=<filament-install> \
  pixi run config
cmake --build build/default/cpp/Release --target dart_filament_gui
```

When using the upstream Linux archive, set `CMAKE_LIBRARY_PATH` to a prefix that
contains compatible libc++ and libc++abi libraries before configuring.

For explicit local fetching on Linux x86_64:

```bash
DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  pixi run config
```

To validate this example without the legacy OSG `dart-gui` target:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi>
CMAKE_LIBRARY_PATH="$LIBCXX_PREFIX/lib" \
  DART_BUILD_GUI_OVERRIDE=OFF \
  DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_FETCH_FILAMENT_OVERRIDE=ON \
  DART_USE_SYSTEM_FILAMENT_OVERRIDE=OFF \
  pixi run config OFF
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  cmake --build build/default/cpp/Release --target dart_filament_gui
```

## Execute Instructions

```bash
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ./build/default/cpp/Release/bin/filament_gui --frames 300
```

To run the interaction fixture:

```bash
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ./build/default/cpp/Release/bin/filament_gui --scene drag-and-drop --frames 300
```

For software-rendered local smoke checks:

```bash
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  LIBGL_ALWAYS_SOFTWARE=1 \
  MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  ./build/default/cpp/Release/bin/filament_gui --frames 10 \
  --screenshot /tmp/dart_filament_gui.ppm
```

For the CI-oriented explicit fetch smoke on Linux x86_64:

```bash
LIBCXX_PREFIX=<prefix-containing-libc++-and-libc++abi> \
  pixi run test-filament-gui-smoke
```

To register the opt-in headless CTest smoke check, configure with:

```bash
DART_BUILD_GUI_FILAMENT_OVERRIDE=ON \
  DART_ENABLE_FILAMENT_GUI_SMOKE_TESTS_OVERRIDE=ON \
  Filament_ROOT=<filament-install> \
  pixi run config
cmake --build build/default/cpp/Release --target dart_filament_gui
ctest --test-dir build/default/cpp/Release \
  -R 'EXAMPLE_filament_gui_headless_smoke|EXAMPLE_filament_gui_drag_and_drop_headless_smoke' \
  --output-on-failure
```
