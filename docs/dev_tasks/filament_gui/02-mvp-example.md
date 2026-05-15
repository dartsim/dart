# Filament GUI Replacement - MVP Example

## Purpose

The first implementation should answer one question: can DART maintain a
Filament + GLFW + Dear ImGui viewer that meets the quality and packaging goals
without becoming a graphics-engine project?

It should not introduce a stable public API yet.

## Proposed target

- Directory: `examples/filament_gui/`
- Target: `dart_filament_gui`
- Build option: `DART_BUILD_GUI_FILAMENT=ON`
- Initial dependencies: `dart`, Filament, GLFW, Dear ImGui

Start example-local, then move reusable code into `dart/gui/experimental` only
after the API and lifetime boundaries become clear. The scene extraction and
picking helpers have now crossed that boundary; renderer/window/overlay code
remains local to the example.

## Implementation status

Implemented:

- `DART_BUILD_GUI_FILAMENT`, `DART_USE_SYSTEM_FILAMENT`, and
  `DART_FETCH_FILAMENT` CMake options.
- Example-local Filament discovery for install trees that provide `include/`,
  `lib/`, and `bin/matc`.
- Material compilation and embedding through Filament `matc`.
- A GLFW + Filament executable target that builds a DART world with two falling
  boxes and a ground plane.
- Filament lit material setup, tangent generation, explicit shadow participation
  for renderables, PCSS shadows, cascaded sun-light shadows, contact-shadow
  options, a neutral skybox, spherical harmonics indirect lighting, high-quality
  color grading, and anisotropic texture sampling in the example source.
- Explicit view-quality setup for HDR buffer quality, screen-space ambient
  occlusion, temporal anti-aliasing, FXAA, temporal dithering, and
  multi-sample anti-aliasing for windowed runs.
- Deterministic `--frames`, `--width`, `--height`, and `--screenshot`
  command-line parsing.
- Screenshot readback through Filament `readPixels`, currently written as a
  binary PPM file for visual smoke testing.
- `--headless` mode using Filament's headless swap-chain path for bounded
  screenshot smoke tests without creating a GLFW window.
- Orbit, pan, and zoom camera controls.
- A visible Dear ImGui panel rendered through a small Filament-native overlay
  path that converts ImGui draw vertices/indices into Filament buffers.
- A `dart-gui-experimental` scene extraction layer that describes visual shape
  nodes, world transforms, material colors, visibility, and shadow flags without
  depending on Filament.
- A graphics-free unit test for the experimental extraction layer.
- Filament renderable creation from extracted box, sphere, ellipsoid, cylinder,
  cone, capsule, TriMesh-backed mesh, and finite plane descriptors.
- A richer visual fixture with dynamic boxes, a ground plane, additional
  primitive visual shapes, a generated TriMesh visual, an imported WAM Collada
  mesh, a required Atlas DAE torso mesh, a full Atlas SDF robot fixture, and a
  multi-panel glTF/PBR environment fixture, plus a finite PlaneShape proxy for
  material/shadow inspection.
- The smoke fixture also requires the full WAM URDF skeleton to load through
  DART's normal `dart::io` and `dart-utils-urdf` path, adding a multi-link
  articulated robot asset with existing DAE/JPEG material data to the rendered
  scene.
- The smoke fixture requires the larger Atlas torso DAE mesh to load and create
  a Filament renderable, adding a second robot-asset mesh path to the fixture.
- The smoke fixture also requires the full Atlas SDF robot to load through
  DART's normal `dart::io` path and produce at least twenty visible mesh
  descriptors/renderables, adding a larger articulated robot fixture beyond the
  WAM asset.
- A procedural checker-textured Filament material applied to the finite plane
  proxy, proving texture coordinates, sampler binding, and textured PBR
  material compilation in the example.
- MeshShape texture-coordinate and submesh-range accessors plus Filament mesh
  rendering that uses authored UVs, submesh material assignments, material
  colors, and metallic/roughness factors when available.
- PNG/JPEG texture-image loading for typed `MeshShape` material maps, including
  base color, metallic, roughness, combined metallic-roughness, normal,
  occlusion, and emissive maps. The smoke fixture exercises the path with an
  imported textured WAM Collada mesh and a procedural checker-textured plane.
- Checked-in glTF PBR fixtures that use authored base-color,
  metallic-roughness, normal, occlusion, and emissive texture slots. The
  `UNIT_dynamics_MeshShape` test loads them through the real Assimp importer
  and verifies material factors, typed texture paths, alpha-bearing material
  color, UV metadata, and single-/multi-material submesh ranges. The Filament
  smoke scene also loads these fixtures.
- The smoke fixture uses the multi-material glTF PBR asset as a required
  four-panel environment layout and fails early if those panels are not
  extracted and rendered.
- Transparent lit and textured-lit material variants route alpha-bearing DART
  visual aspects and mesh material base-color alpha through Filament blending.
  The smoke fixture includes a translucent ellipsoid, a translucent
  checker-textured plane, and the alpha-bearing glTF PBR mesh.
- Backend-hidden debug line descriptors in `dart-gui-experimental` for a grid,
  world/body frames, center-of-mass markers, contact markers, contact normal
  arrows, and contact force arrows, with the Filament example translating those
  descriptors into line primitives.
- Renderer-independent picking bounds, nearest ray-hit helpers, and focused
  unit coverage in the experimental interaction layer.
- Basic click-to-select in the example, with the ImGui panel reporting the
  selected DART shape and the selected renderable temporarily highlighted.
- Renderer-independent selection bounds descriptors, with the example rendering
  a line overlay around the selected shape.
- Backend-hidden free-joint, simple-frame, and combined frame-renderable
  translation helpers, with C++ and Python tests. The example uses the combined
  helper for keyboard nudging of selected dynamic bodies and `SimpleFrame`
  visuals, covering the core data path needed by drag-and-drop style workflows.
- Backend-hidden plane intersection and plane-drag translation helpers, with
  C++ and Python tests, and example Ctrl-left dragging for selected dynamic
  bodies in a camera-facing plane.
- A selectable `--scene drag-and-drop` fixture that recreates the legacy
  drag-and-drop example's `SimpleFrame` anchor, child frame, and axis markers
  through the same backend-hidden extraction and frame-translation path.
- A selectable `--scene hardcoded-design` fixture that recreates the legacy
  hardcoded-design example's three-link manually constructed skeleton through
  backend-hidden renderable descriptors.
- A selectable `--scene rigid-chain` fixture that loads the legacy rigid-chain
  SKEL data and renders its box-link descriptors through the same
  backend-hidden extraction path.
- Built-in panel controls for pause/resume, single-step, and debug overlay
  toggles covering grid, world/body frames, center-of-mass markers, contacts,
  normal arrows, and force arrows.
- Backend-hidden viewer-runtime helpers in `dart-gui-experimental`, with the
  Filament example using them for bounded screenshots, camera placement, cursor
  tracking, directional nudging, viewer lifecycle state, screenshot storage,
  and perspective pick rays.
- `examples/gui_scene_diagnostics`, a non-rendering second example that consumes
  `dart-gui-experimental` descriptors, debug lines, run options, camera basis,
  and picking helpers.
- Constrained `dartpy.gui.experimental` bindings for scene descriptors,
  picking helpers, frame translation, debug-line descriptor generation,
  run options, orbit-camera helpers, orbit-camera controller helpers, and
  directional nudge helpers.
- A local Linux runtime smoke pass with the upstream Filament archive,
  compatible libc++/libc++abi libraries, and Mesa llvmpipe.

Blocked or incomplete:

- The current Pixi environment has no Filament package. The upstream Linux
  archive can be found with `Filament_ROOT`, but it depends on libc++ on Linux.
  A compatible libc++/libc++abi source is needed until Filament is packaged or
  built for the DART toolchain.
- The current ImGui overlay is intentionally example-local and MVP-level. It is
  accepted for the built-in status/debug panel, pause/step controls, and debug
  overlay toggles, but it should not be promoted as a stable user-extension API
  without DART-owned panel/tool abstractions and explicit support expectations
  for scissoring, multiple textures, docking, and user-provided image widgets.
- Screenshot capture exists for both the window swap chain and Filament's
  headless swap-chain path. Broader CI integration for headless capture is
  still pending.
- Broader human visual review with larger authored glTF/PBR environment scenes
  beyond the current WAM, Atlas, and PBR panel fixtures is still pending.
- Debug overlays are still MVP-level and need broader scenario coverage, but
  their grid/frame/center-of-mass/contact/force-arrow data is no longer
  Filament-only example code and the built-in panel can toggle the current
  overlay groups.
- Selection/manipulation is still MVP-level: shape picking, bounds overlays,
  body/simple-frame nudging, Ctrl-left camera-plane dragging, and a first
  drag-and-drop fixture exist, but broader interaction-heavy workflows are
  still pending.
- Camera, bounded-run math, screenshot storage, and viewer lifecycle state have
  moved into `dart-gui-experimental`; broader viewer API promotion is still
  pending.
- The second example is diagnostic-only; the Filament example now carries the
  first interaction-heavy fixture through `--scene drag-and-drop`, but broader
  example migration is still pending.
- The generated primitive meshes are sufficient for the current smoke fixture,
  but need visual polish before promotion.

## MVP scene

Start with a small scene built directly in code:

- ground plane
- two falling boxes or spheres connected to a `simulation::World`
- one static mesh or generated TriMesh visual
- one finite visual proxy for `PlaneShape`
- one textured primitive or mesh
- directional light
- default camera using DART's Z-up convention
- simple material showing color, roughness, and shadows
- visible shadow casting and receiving on the ground plane

Avoid file loading in the first pass unless needed to prove mesh import.

## MVP controls

- Orbit/pan/zoom camera controls
- Space: pause/resume
- `n`: step one frame while paused
- Left click without dragging: select the nearest visible renderable under the
  cursor
- Ctrl-left drag: move the selected free-joint body in a camera-facing plane
- Escape or window close: exit
- `--frames <n>`: render a bounded number of frames and exit
- `--width <n>` / `--height <n>`: configure viewport size
- `--headless`: render through Filament's headless swap chain without opening a
  GLFW window
- Minimal ImGui panel with simulation time, pause/play, step, and renderer info

## MVP acceptance criteria

The MVP is successful only if all of the following are true:

- Builds on at least Linux through the DART pixi/CMake workflow.
- Opens a window and renders a nonblank, correctly framed 3D scene.
- Renders visible dynamic shadows for the default body/ground fixture.
- Advances a real DART `simulation::World` and updates transforms.
- Shows a Dear ImGui panel without leaking ImGui into DART public headers.
- Exits deterministically with `--frames`.
- Does not require OSG or Raylib.
- Documents all package/tooling blockers found during implementation.

## Stretch criteria

These are useful but should not block the first MVP:

- screenshot output
- CI integration for headless/offscreen frame capture
- shadowed screenshot fixture for visual smoke testing
- broader glTF/PBR robot or environment fixtures with real authored normal,
  occlusion, emissive, metallic, roughness, and combined metallic-roughness maps
- simple shape picking
- richer debug-overlay controls and scenario coverage
- macOS and Windows builds

## Expected implementation sequence

1. Add CMake discovery for Filament and GLFW behind explicit experimental
   options.
2. Add a window-only Filament example that clears the screen and draws ImGui.
3. Add one Filament primitive with lighting and camera controls.
4. Add a DART `simulation::World` and transform synchronization.
5. Add `--frames` bounded execution and basic CI-friendly smoke behavior.
6. Record package, material compiler, platform, and headless findings in this
   task folder before promoting the code.
