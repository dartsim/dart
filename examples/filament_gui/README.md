# Filament GUI Example

## Summary

- Goal: exercise the experimental Filament + GLFW visualization path.
- User view: an interactive DART scene viewer for inspecting simulation
  renderables, camera movement, selection, debug overlays, dynamic shadows, and
  basic selected-object manipulation.
- Concepts/APIs: Filament renderer setup, GLFW windowing, DART world stepping,
  dynamic shadows, and bounded headless capture.
- Expected output: a Filament window with DART boxes falling onto a ground
  plane, additional primitive/mesh/plane visuals, dynamic shadows, and a small
  Dear ImGui status panel.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, a left click without dragging
  selects a renderable, arrow/PageUp/PageDown keys nudge selected free-joint
  bodies, `SimpleFrame` visuals, and robot IK targets, Ctrl-left drag moves the
  selected item in a camera-facing plane, Escape exits, and `--frames` limits
  rendered frame count. Use
  `--screenshot <path>` to write a binary PPM capture, and `--headless` to use
  Filament's headless swap-chain path without creating a GLFW window. Use
  `--hide-ui` or `--show-ui` to control the status panel in captures,
  `--gui-scale <factor>` to scale the render target and status panel, and
  `--profile` to print per-phase frame timings. The key light orbits by default
  for shading diagnostics; toggle it with `--orbit-light`, `--no-orbit-light`,
  or the panel checkbox, and tune the default 80-second full orbit with
  `--orbit-light-period <seconds>`.
  Use `--scene hello-world` to run the simplest dynamic-box fixture,
  `--scene boxes` to run the dynamic box-grid fixture,
  `--scene hardcoded-design` to run the hand-built skeleton fixture,
  `--scene rigid-chain` to run the SKEL-loaded chain fixture,
  `--scene rigid-loop` to run the constrained loop-chain fixture,
  `--scene mixed-chain` to run the mixed rigid/soft chain fixture,
  `--scene coupler-constraint` to run the paired mimic/coupler rig fixture,
  `--scene add-delete-skels` to run the deterministic add/delete skeletons
  fixture,
  `--scene vehicle` to run the SKEL-loaded vehicle visual fixture,
  `--scene hybrid-dynamics` to run the posed fullbody visual fixture,
  `--scene mimic-pendulums` to run the SDF-loaded mimic pendulum fixture,
  `--scene atlas-puppet` to run the Atlas IK target fixture,
  `--scene drag-and-drop` to run the interaction fixture,
  `--scene simple-frames` to run the frame-hierarchy fixture,
  `--scene soft-bodies` to run the soft-body SKEL fixture,
  `--scene point-cloud` to run the point-cloud/voxel-grid fixture,
  `--scene capsule-ground-contact` to run the capsule contact fixture,
  `--scene simulation-event-handler` to run the sensor-marker fixture,
  `--scene polyhedron` or `--scene heightmap` to inspect focused legacy
  visual fixtures, or `--scene g1` to load the Unitree G1 humanoid used in
  the project README animation.

## Notes

- Requires DART configured with `DART_BUILD_GUI_FILAMENT=ON`.
- Links against `dart-gui-experimental` for backend-hidden scene extraction and
  picking; it does not require the legacy OSG `dart-gui` target when
  `DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF`.
- Requires a Filament install tree with headers, libraries, and matching
  `matc`; set `Filament_ROOT` if it is not discoverable.
- This is an experimental implementation using the constrained
  `dart::gui::experimental` scene layer, not a stable public `dart::gui` API.
- The upstream Filament Linux archive currently links against libc++. The Linux
  Pixi environment includes libc++/libc++abi for that archive; outside Pixi,
  configure needs a compatible Filament build or those libraries.
- The ImGui overlay is intentionally minimal. It renders the built-in status
  panel, but does not yet support the full Dear ImGui renderer-backend feature
  set.
- The example renders built-in primitives, TriMesh-backed `MeshShape` visuals,
  an imported WAM Collada mesh, the full WAM URDF skeleton, an Atlas DAE torso
  mesh, a full Atlas SDF robot fixture, a four-panel glTF/PBR environment
  layout, and a finite checker-textured `PlaneShape` proxy. Mesh rendering uses
  descriptor triangle data for convex mesh, heightmap, soft mesh, and
  `MeshShape` visuals, while `MeshShape` rendering also uses descriptor-owned
  authored UVs, imported vertex normals, submesh material ranges, material
  colors, metallic/roughness factors, emissive color, and typed PNG/JPEG
  texture images for base color, metallic, roughness, combined
  metallic-roughness, normal, occlusion, and emissive maps when `MeshShape`
  provides them. Point-cloud rendering consumes descriptor-owned per-point
  colors when present. The scene also
  loads the checked-in single- and multi-material glTF PBR fixtures and routes
  alpha-bearing solid, textured, and mesh visuals through transparent lit
  material variants. Broader robot/environment visual review is still future
  promotion work.
- The view enables HDR buffer quality, screen-space ambient occlusion,
  multi-sample anti-aliasing, FXAA, and higher-resolution shadow maps for
  windowed runs in the current visual smoke fixture. Temporal anti-aliasing and
  temporal dithering are disabled so labels, wire meshes, and authored texture
  details stay crisp during visual review.
- Imported WAM and Atlas robot fixtures are included as static visual reference
  objects in the default scene. Their collision and gravity participation are
  disabled so they do not dominate example frame time.
- The `--scene hello-world` fixture renders the legacy hello-world example's
  single dynamic blue box and gray ground plane through the same
  descriptor-driven Filament path.
- The `--scene boxes` fixture renders the legacy boxes example's dynamic
  colored box grid and gray ground plane through the same descriptor-driven
  Filament path.
- The `--scene hardcoded-design` fixture renders the legacy hardcoded-design
  example's three-link skeleton through the same descriptor-driven Filament
  path. The standalone source remains comparison material for wireframe and
  direct key-controlled joint motion.
- The `--scene rigid-chain` fixture loads the legacy rigid-chain SKEL data and
  renders the chain's box-link descriptors through the same Filament path. The
  standalone source remains comparison material for custom per-step damping.
- The `--scene rigid-loop` fixture loads the legacy rigid-loop chain data,
  applies the loop-closing pose, marks the constrained links red, and renders
  the linked box descriptors through the same Filament path. The standalone
  source remains comparison material for damping and constraint setup.
- The `--scene mixed-chain` fixture loads the legacy mixed rigid/soft chain
  data and renders both box-link and soft-mesh descriptors through the same
  Filament path. The standalone source remains comparison material for
  keyboard-applied external forces.
- The `--scene coupler-constraint` fixture builds the legacy paired
  mimic/coupler rig layout and renders the four link boxes plus six guide lines
  through the same Filament path. The standalone source remains comparison
  material for the ImGui status overlay and reset controls.
- The `--scene add-delete-skels` fixture loads the legacy ground world and adds
  a deterministic set of cube skeletons through the same Filament path. The
  standalone source remains comparison material for live q/w add-delete
  controls.
- The `--scene vehicle` fixture loads the legacy vehicle SKEL world and renders
  the car body, wheel cylinders, ground, and obstacle boxes through the same
  descriptor-driven Filament path. The standalone source remains comparison
  material for live throttle and steering controls.
- The `--scene hybrid-dynamics` fixture loads the legacy fullbody SKEL world,
  applies the same initial humanoid pose, and renders the biped and ground
  boxes through the same descriptor-driven Filament path. The standalone source
  remains comparison material for scripted joint commands and harness toggling.
  The `biped_stand` runner reuses this fixture for its standing fullbody visual
  workflow while the standalone source remains comparison material for SPD
  perturbation controls.
- The `--scene mimic-pendulums` fixture loads the legacy mimic-pendulums SDF
  world and renders the three pendulum rigs, base poles, and ground through the
  same descriptor-driven Filament path. The standalone source remains
  comparison material for the ImGui solver/debug table.
- The `--scene atlas-puppet` fixture loads Atlas and exposes selectable hand
  and foot IK target frames through the same backend-hidden selection,
  keyboard-nudge, and Ctrl-left drag path. The standalone source remains
  comparison material for the OSG teleoperation widget and support-polygon
  visual.
- Windowed playback advances physics against wall-clock time with a bounded
  catch-up step budget. Headless captures remain deterministic and advance one
  simulation step for each rendered frame.
- A simple 3D grid, world/body frame axes, center-of-mass markers, inertia
  boxes, collision-shape bounds, contact markers, contact normals, contact
  force vectors, and support polygons are generated as `dart-gui-experimental`
  debug descriptors and rendered as Filament line primitives. The debug-line
  material uses transparent blending for alpha-bearing overlay colors.
- Basic click-to-select uses renderer-independent DART descriptor picking and
  reports the selected shape in the built-in panel while drawing a bounds
  overlay around it. Selected free-joint bodies and `SimpleFrame` visuals can
  be moved with keyboard nudges or Ctrl-left camera-plane dragging through the
  same backend-hidden descriptor layer. The `--scene drag-and-drop` fixture
  uses that path for a legacy-style `SimpleFrame` anchor, child frame, and axis
  marker layout.
- The `--scene simple-frames` fixture renders the legacy simple-frames
  example's `SimpleFrame` hierarchy, marker ellipsoids, and arrow marker
  through the same descriptor-driven Filament path.
- The `--scene soft-bodies` fixture loads the legacy `softBodies.skel` data
  through DART's regular IO path and renders the resulting soft meshes through
  backend-hidden descriptors.
- The `--scene point-cloud` fixture renders a colored point cloud and, when
  OctoMap is available, a voxel occupancy grid through backend-hidden
  descriptors.
- The `--scene capsule-ground-contact` fixture renders the legacy capsule-plane
  contact setup through backend-hidden capsule and ground descriptors, using
  ODE collision when available.
- The `--scene simulation-event-handler` fixture renders the legacy simulation
  event-handler body's falling boxes, sphere, ground, and sensor markers
  through backend-hidden descriptors.
- The `--scene heightmap` fixture renders a local heightmap surface and
  reference markers through descriptor-owned heightmap renderables. It carries
  the visual side of the legacy `heightmap` example while the standalone source
  remains as comparison material for panel-driven sculpting and contact
  alignment.
- The `--scene g1` fixture loads the remote Unitree G1 URDF through DART's
  normal resource retriever and renders it with Filament. Colored IK target
  markers are attached to both hands and feet; press `1`-`4` or click a marker,
  then Ctrl-left drag or use the arrow/PageUp/PageDown keys to move the target
  and apply IK. Override the package or robot source with `--g1-package-uri`,
  `--g1-robot-uri`, and `--g1-package-name` when testing local G1 assets.

## Quick Run

From the repository root:

```bash
pixi run ex filament_gui
```

This single command configures and builds the experimental example with
`DART_BUILD_GUI_FILAMENT=ON`, then runs `examples/filament_gui`. On Linux
x86_64 it uses the pinned explicit Filament fetch fallback by default, so it
does not require a Filament conda package. On platforms where that upstream
archive is unavailable, provide `Filament_ROOT` or a packaged Filament build and
set `DART_FETCH_FILAMENT_OVERRIDE=OFF`.

If `DISPLAY` is not set on Linux, the command automatically uses the headless
path, runs through `xvfb-run` when available, and writes a binary PPM screenshot
to `build/<pixi-env>/filament_gui_mvp.ppm`.

To run the interaction fixture:

```bash
pixi run ex filament_gui --scene drag-and-drop
```

To run the hello-world fixture:

```bash
pixi run ex filament_gui --scene hello-world
```

To run the boxes fixture:

```bash
pixi run ex filament_gui --scene boxes
```

To run the hardcoded-design fixture:

```bash
pixi run ex filament_gui --scene hardcoded-design
```

To run the rigid-chain fixture:

```bash
pixi run ex filament_gui --scene rigid-chain
```

To run the rigid-loop fixture:

```bash
pixi run ex filament_gui --scene rigid-loop
```

To run the mixed-chain fixture:

```bash
pixi run ex filament_gui --scene mixed-chain
```

To run the coupler-constraint fixture:

```bash
pixi run ex filament_gui --scene coupler-constraint
```

To run the add-delete-skels fixture:

```bash
pixi run ex filament_gui --scene add-delete-skels
```

To run the vehicle fixture:

```bash
pixi run ex filament_gui --scene vehicle
```

To run the hybrid-dynamics fixture:

```bash
pixi run ex filament_gui --scene hybrid-dynamics
```

To run the mimic-pendulums fixture:

```bash
pixi run ex filament_gui --scene mimic-pendulums
```

To run the atlas-puppet fixture:

```bash
pixi run ex filament_gui --scene atlas-puppet
```

To run the simple-frames fixture:

```bash
pixi run ex filament_gui --scene simple-frames
```

To run the soft-bodies fixture:

```bash
pixi run ex filament_gui --scene soft-bodies
```

To run the point-cloud fixture:

```bash
pixi run ex filament_gui --scene point-cloud
```

To run the capsule-ground-contact fixture:

```bash
pixi run ex filament_gui --scene capsule-ground-contact
```

To run the simulation-event-handler fixture:

```bash
pixi run ex filament_gui --scene simulation-event-handler
```

To run the convex polyhedron fixture:

```bash
pixi run ex filament_gui --scene polyhedron
```

To run the heightmap visual fixture:

```bash
pixi run ex filament_gui --scene heightmap
```

To run the Unitree G1 fixture:

```bash
pixi run ex filament_gui --scene g1 --gui-scale 2
```

To capture the local built-in fixtures for PR screenshots:

```bash
pixi run ex filament_gui --headless --scene all
```

The capture paths are `build/<pixi-env>/filament_gui_mvp.ppm` and
`build/<pixi-env>/filament_gui_hello_world.ppm`,
`build/<pixi-env>/filament_gui_boxes.ppm`,
`build/<pixi-env>/filament_gui_hardcoded_design.ppm`,
`build/<pixi-env>/filament_gui_rigid_chain.ppm`,
`build/<pixi-env>/filament_gui_rigid_loop.ppm`,
`build/<pixi-env>/filament_gui_mixed_chain.ppm`,
`build/<pixi-env>/filament_gui_coupler_constraint.ppm`,
`build/<pixi-env>/filament_gui_add_delete_skels.ppm`,
`build/<pixi-env>/filament_gui_vehicle.ppm`,
`build/<pixi-env>/filament_gui_hybrid_dynamics.ppm`,
`build/<pixi-env>/filament_gui_mimic_pendulums.ppm`,
`build/<pixi-env>/filament_gui_atlas_puppet.ppm`,
`build/<pixi-env>/filament_gui_drag_and_drop.ppm`,
`build/<pixi-env>/filament_gui_simple_frames.ppm`,
`build/<pixi-env>/filament_gui_soft_bodies.ppm`,
`build/<pixi-env>/filament_gui_point_cloud.ppm`,
`build/<pixi-env>/filament_gui_capsule_ground_contact.ppm`,
`build/<pixi-env>/filament_gui_simulation_event_handler.ppm`,
`build/<pixi-env>/filament_gui_polyhedron.ppm`, and
`build/<pixi-env>/filament_gui_heightmap.ppm` unless
`DART_FILAMENT_GUI_SCREENSHOT` is set. The G1 fixture intentionally stays out
of `--scene all` because it fetches a remote robot package by default.

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

The recommended repo-local entry point is:

```bash
pixi run ex filament_gui
```

The lower-level executable can also be run directly after configuring and
building:

```bash
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ./build/default/cpp/Release/bin/filament_gui --frames 300
```

To run the interaction fixture:

```bash
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ./build/default/cpp/Release/bin/filament_gui --scene drag-and-drop --frames 300
```

To profile the example by frame phase:

```bash
LD_LIBRARY_PATH="$LIBCXX_PREFIX/lib:${LD_LIBRARY_PATH:-}" \
  ./build/default/cpp/Release/bin/filament_gui --headless --profile --frames 120
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
  -R 'EXAMPLE_filament_gui_(headless|hello_world_headless|boxes_headless|hardcoded_design_headless|rigid_chain_headless|rigid_loop_headless|mixed_chain_headless|coupler_constraint_headless|add_delete_skels_headless|vehicle_headless|hybrid_dynamics_headless|mimic_pendulums_headless|atlas_puppet_headless|drag_and_drop_headless|simple_frames_headless|soft_bodies_headless|point_cloud_headless|capsule_ground_contact_headless|simulation_event_handler_headless|polyhedron_headless|heightmap_headless)_smoke' \
  --output-on-failure
```
