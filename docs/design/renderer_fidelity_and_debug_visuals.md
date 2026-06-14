# Renderer Material/Lighting Fidelity And First-Class Debug Visuals

Durable analysis and recommended direction for two adjacent DART 7 GUI goals:

1. **Raise the visual fidelity** of the Filament-backed `dart::gui` renderer
   beyond what the post-process pipeline already does.
2. **Promote debug visualization to a `dart::gui` component feature** that every
   host (dartsim editor, demos, downstream apps) gets for free, instead of each
   example re-implementing it.

This doc records investigation findings and a prioritized direction; it is **not**
active task state (roadmap priority/status live in
[`../plans/dashboard.md`](../plans/dashboard.md), PLAN-090 / PLAN-060). User-facing
usage lives in [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md).

## Implementation Status

Landed (build + headless render + unit/extraction tests + dartpy smoke verified):

- **A1 — per-shape PBR.** `MaterialDescriptor` carries optional
  `metallic`/`roughness`/`reflectance`; `VisualAspect` gained matching
  setters/getters (sentinel `< 0` = renderer default) plumbed through extraction
  and a guarded post-creation override in `renderable_factory.cpp` (lit
  primitives only; asset meshes and unlit line/point/voxel renderables are
  excluded). The override is hashed into the render-resource version so runtime
  edits rebuild the renderable. dartpy bindings + stubs included.
- **A3 — texture mip chains.** `textures.cpp` allocates a full mip chain
  (`GEN_MIPMAPPABLE` usage) + `generateMipmaps`, with
  `LINEAR_MIPMAP_LINEAR` + 8× anisotropy.
- **A4 — capture fidelity opt-in.** `DART_GUI_HIGH_FIDELITY` makes the headless
  path use the windowed-quality passes (GTAO/bloom/MSAA/fog/contact shadows +
  sharper shadows) for GPU capture, without changing the llvmpipe CI default.
- **B1 — `ApplicationOptions::debugProvider`.** A `DebugScene`
  (lines/triangles/labels) provider merged each frame into the built-in overlay
  with the correct unlit/no-shadow treatment; a unit test covers provider
  propagation through `createDartScene`. (The `gui_scene_diagnostics` example
  that motivated this seam was retired together with the legacy world in the
  DART 7 simulation promotion, so the per-example wiring it carried no longer
  exists anywhere.)
- **B3 — new debug primitives.** `drawJointAxes` (revolute double-line /
  prismatic arrow) and `drawLinearVelocities`/`drawAngularVelocities` arrows,
  with `makeJointAxisDebugLines`/`makeVelocityDebugLines` helpers. The helpers
  operate on `dynamics::BodyNode` directly and are intended for `debugProvider`
  callbacks; the legacy-world `extractDebugLines` overload that auto-walked
  skeletons was retired with the legacy world, so world-derived overlays will
  be re-wired when debug extraction lands for the promoted
  `dart::simulation::World`.
- **B2 — built-in panel.** The built-in panel exposes only debug controls backed
  by the current overlay extraction path: grid/world-frame toggles, contact
  toggles, and a "Debug tuning" slider group for grid spacing, world-frame axis
  length, and contact-force scale. Body/joint/velocity controls stay out of the
  panel until world debug extraction is reintroduced for the promoted world (see
  B3 note), while applications can draw those helpers through `debugProvider`.

Remaining (deliberately deferred — see rationale):

- **A2 — IBL specular reflection cubemap.** Needs a roughness-prefiltered
  environment (offline `cmgen`/KTX asset or an `IBLPrefilterContext` pass) plus
  cubemap resource ownership in the render-environment teardown. Correct specular
  reflections cannot be visually verified on the headless llvmpipe path, so this
  is left for implementation + review on a real GPU rather than landed blind.
- **B4 — depth-tested debug variant + view layers.** Needs a new
  matc-compiled `debug_color_depth` material (a CMake reconfigure plus the
  embed/registration pipeline in `backend_sources.cmake`) and per-descriptor
  material selection / `View::setVisibleLayers` plumbing. Lowest-ROI item;
  scoped as a follow-up.

## Single Source Of Truth (scope boundaries)

To avoid overlap with existing design docs, this doc owns **material, lighting,
texture fidelity** and **debug-visualization-as-a-GUI-feature**. It explicitly
defers:

- Post-process dials (MSAA/TAA, SSAO, bloom), the Realtime/Balanced/Offline
  preset surface, offline temporal accumulation, and the camera-sensor seam
  (lens distortion, depth/segmentation outputs) →
  [`filament_fidelity_profile.md`](filament_fidelity_profile.md).
- Real-time loop (interpolation, render/sim decoupling, speed controls), GPU
  instancing, and large-scene scalability →
  [`renderer_realtime_and_scalability.md`](renderer_realtime_and_scalability.md).

The recommendations below are additive to those and reuse their seams (e.g. a
`FidelityProfile` is the natural home for the new fidelity toggles in Part A).

All proposals keep the backend-hidden rule: no Filament/GLFW/ImGui/OpenGL/Vulkan
type appears in public `dart/gui/*.hpp`
([`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md), `dart/gui/AGENTS.md`).

## Current State (evidence)

The renderer is already well past a basic viewer. `render_environment.cpp` sets
up PBR-neutral tone mapping + HIGH color grading, a 3-point light rig
(`createSceneLights`, key/fill/rim), a diffuse SH `IndirectLight`, a colored
`Skybox`, and — on the windowed path — GTAO, bloom, 4x MSAA, volumetric fog, PCF
shadows (4096 / 4 cascades), and screen-space contact shadows
(`render_environment.cpp:137-326`). FXAA + temporal dithering run on both paths.

Debug drawing is also partly centralized already: `dart/gui/debug.hpp` defines
renderer-neutral `DebugLineDescriptor` / `DebugTriangleDescriptor` /
`DebugLabelDescriptor` + `DebugDrawOptions`, and a built-in status panel
(`panel.cpp:377-517`, `renderBuiltInStatusPanel`) ships runtime checkboxes for
Grid, World frame, Contacts, contact Normals, and Contact forces. The
`DebugOverlayController`
(`debug_overlay.cpp`) renders lines + triangles through the unlit `debug_color`
material, and `ui_frame.cpp:139-187` draws `debugLabels` as 2D-projected ImGui
text (capped at 96, `kMaxDebugLabels`).

So the question is not "does debug viz exist" but **"what fidelity and what debug
capabilities are still missing, and what still forces per-example work."**

---

## Part A — Visual Fidelity Gaps And Recommendations

### A1. Per-shape PBR material parameters (highest ROI, surgical)

**Gap.** `MaterialDescriptor` (`renderable.hpp:152-158`) carries only
`rgba`, `visible`, `castsShadows`, `receivesShadows`. Extraction populates it from
`VisualAspect` as **color-only**: `renderable.cpp:181`
(`descriptor.material.rgba = visualAspect.getRGBA()`). Primitive shapes then get
hardcoded `kDefaultDielectricMetallic = 0.0f` / `kDefaultMatteRoughness = 0.82f`
(`renderable_factory.cpp:91-92`). PBR metallic/roughness exist **only** on loaded
meshes via `dynamics::mesh_material.hpp` (`metallicFactor`, `roughnessFactor`,
texture paths). Consequence: a procedurally created box/sphere/capsule — the bulk
of robotics scenes — can never be made to look like metal, glossy plastic, or
rubber through the public API; everything is matte dielectric.

**Recommendation.** Add `metallic`, `roughness`, `reflectance` (and optionally an
`emissive` color) to `MaterialDescriptor`, plumb them through
`renderable_factory.cpp` (it already accepts these as function arguments at
`:827-879`, so the change is mostly threading descriptor fields instead of
constants), and surface them on `dynamics::VisualAspect` as DART concepts
(e.g. `setMetallic`/`setRoughness`, or a small `VisualMaterial` with sensible
named presets: `Matte`, `Plastic`, `Metal`, `Rubber`, `Glass`). The `default_lit`
material already exposes all four parameters, so **no shader change is needed**.

### A2. Image-based specular reflections (env reflection cubemap)

**Gap.** `createNeutralIndirectLight` (`render_environment.cpp:94-113`) builds the
`IndirectLight` with `.irradiance(3, ...)` only — diffuse spherical harmonics, **no
`.reflections(cubemap)`**. The skybox is a flat color (`createNeutralSkybox`,
`:115-121`). So metallic/glossy surfaces have nothing meaningful to reflect; the
metallic path in A1 will look dull until there is an environment to mirror.

**Recommendation.** Provide a prefiltered reflection cubemap for the
`IndirectLight`: either bundle a small neutral studio KTX (matc/cmgen pipeline
already exists in `backend_sources.cmake`) or generate a cheap gradient cubemap at
startup from the existing skybox colors. This is what turns A1 from "tinted matte"
into "recognizably metal/glass." Pair with the fidelity profile so offline mode
can swap in a higher-resolution / HDR environment.

### A3. Texture mipmaps and filtering

**Gap.** Every texture is uploaded with `.levels(1)` (`textures.cpp:243`, `:402`,
`:424`) — a single mip level. The sampler requests 8x anisotropy
(`makeRepeatTextureSampler`, `:271`) but anisotropy without a mip chain does
almost nothing; textured surfaces (URDF/glTF assets, ground textures) alias and
shimmer at distance and grazing angles.

**Recommendation.** Allocate the full mip chain (`.levels(...)` from texture
dimensions), call `Texture::generateMipmaps`, and switch the sampler to
`MinFilter::LINEAR_MIPMAP_LINEAR`. Self-contained, no public-API change, and a
clear before/after on any textured asset.

### A4. Headless/windowed capture fidelity parity

**Gap.** `configureViewQuality` (`render_environment.cpp:137-183`) intentionally
strips GTAO, bloom, 4x MSAA, volumetric fog, and screen-space contact shadows on
the headless path, and `createSceneLights` halves the shadow atlas/cascades
(`:283-296`). That is the right default on the CI **software** rasterizer
(llvmpipe), but it means screenshots, frame-output sequences, and any future
headless sensor camera render at materially **lower** fidelity than the
interactive window — which undercuts the validation workflow that explicitly
relies on inspecting screenshots
([`gui-rendering.md` Validation](../onboarding/gui-rendering.md)).

**Recommendation.** Gate the heavy passes on **GPU availability**, not on the
headless flag, so headless-on-a-real-GPU (the synthetic-data / sensor case) gets
full fidelity while headless-on-llvmpipe stays light. This is the natural
implementation of the `FidelityProfile` Offline preset
([`filament_fidelity_profile.md`](filament_fidelity_profile.md)): the preset, not
`headless`, should decide quality; detect the active backend
(`render_context.cpp`) to pick a safe default.

### A5. Lower-priority fidelity items

- **Exposure / physically-based camera.** No exposure control today; tone mapping
  is fixed PBR-neutral. Expose exposure (EV) through the `FidelityProfile` for
  consistent brightness across scenes and sensor outputs.
- **Extended shading models.** `default_lit.mat` is the base `lit` model. A
  `clearCoat`/`anisotropy` variant would help painted metal, carbon-fiber, and
  brushed surfaces, but it is niche for robotics — defer until A1/A2 land.
- **Shadow-catcher ground.** The grid is debug geometry, not a lit ground plane;
  an optional large matte ground would improve "groundedness" of contact-rich
  scenes (complements the existing GTAO + contact-shadow work).

---

## Part B — Debug Visualization As A `dart::gui` Component Feature

The goal is "support debug visuals in the GUI component, not just per example."
Today the built-in overlay covers a **fixed** menu of world-derived overlays
(Part: Current State). The gaps are (a) apps cannot feed their **own** debug
geometry into the built-in overlay, (b) the toggle/tuning surface is limited, and
(c) several high-value debug primitive **types** do not exist.

### B1. An app `debugProvider` callback (the core "not per example" fix)

**Gap.** `ApplicationOptions` (`application.hpp:198-274`) has `renderableProvider`
(extra solid geometry) and `debugLabels` (2D text), but **no per-frame provider
for `DebugLineDescriptor` / `DebugTriangleDescriptor`**. An app that wants custom
debug geometry must either abuse `renderableProvider` (debug lines become real
lit/solid renderables, losing the always-on-top unlit debug treatment) or build
its own ImGui panel and call `extractDebugLines` itself. `gui_scene_diagnostics`
did exactly the latter (hand-rolled `DebugDrawOptions` + `extractDebugLines` +
`makeSelectionDebugLines` wiring) before it was retired with the legacy world —
that wiring is what should live once, in the component.

**Recommendation.** Add

```cpp
struct DebugScene {              // renderer-neutral, in dart/gui/debug.hpp
  std::vector<DebugLineDescriptor> lines;
  std::vector<DebugTriangleDescriptor> triangles;
  std::vector<DebugLabelDescriptor> labels;
};
// in ApplicationOptions:
std::function<DebugScene()> debugProvider;
```

The frame loop appends `debugProvider()` output to the built-in
`DebugOverlayController` overlay each frame (`debug_overlay.cpp` already merges
multiple line sets — `refreshDebugLineOverlay`). Now any host contributes debug
geometry through one DART concept, with the correct unlit/no-shadow/x-ray
treatment, and `debugLabels` becomes the `.labels` member instead of a separate
hook.

### B2. Richer built-in toggles + numeric tuning

**Gap.** `renderBuiltInStatusPanel` (`panel.cpp:467-491`) exposes ~10 booleans but
**none** of the numeric `DebugDrawOptions` fields (`debug.hpp:92-107`:
`gridSpacing`, `contactForceScale`, axis lengths, marker radii, …), and omits
`drawWorldFrame`'s siblings like `drawContactNormals`-vs-`drawSupportCentroids`
granularity. Tuning requires a code edit + rebuild.

**Recommendation.** Add a collapsible "Debug" section for supported scale/length
fields (grid spacing, world-frame axis length, and contact-force scale) to the
built-in panel, driven directly by the existing `DebugDrawOptions` struct so it
stays the single source of truth. Keep body/joint/velocity controls hidden from
the built-in panel until the promoted `dart::simulation::World` can provide the
body-node data those overlays require; applications can still expose those
helpers through `ApplicationOptions::debugProvider`.

### B3. Missing debug primitive types

`DebugDrawOptions` (`debug.hpp:80-108`) and the `make*`/`extract*` helpers cover
grid, frames, COM, inertia boxes, collision-shape bounds, support polygons,
contacts, contact normals, contact forces. High-value types **absent** today,
each expressible with the existing line+triangle primitives:

- **Arrowheads on vectors.** Contact normals/forces are plain line segments
  (`extractContactDebugLines`); a cone/triangle head (the gizmo code already
  builds triangle heads — `makeGizmoDebugTriangles`) makes direction/sign legible.
- **Velocity vectors** (linear + angular) per body — the most-requested dynamics
  debug aid and not derivable from contacts.
- **Applied external-force vectors** (incl. the mouse force-drag spring, which is
  currently a special-case selection line — `selection.hpp:85`).
- **Joint axes / DOF arrows** for revolute/prismatic joints.
- **Broad-phase AABBs** (distinct from per-shape `drawCollisionShapeBounds`),
  feeding the collision-sandbox broad-phase overlay the onboarding doc calls for.
- **Friction cones** at contacts; **trajectories / trails** for tracked frames;
  **momentum / COP** markers.

**Recommendation.** Extend `DebugDrawOptions` with the booleans (+ scales) and add
`make*DebugLines`/`...Triangles` helpers, reusing the gizmo arrowhead geometry.
Keep each helper renderer-neutral and unit-test it like the existing extractors
(`tests/unit/gui/`).

### B4. Depth handling and category visibility layers

**Gap.** The debug material is unlit, `depthWrite:false`, `depthCulling:false`,
`doubleSided` (`materials/debug_color.mat`) — i.e. **always X-ray / on top**. That
is right for selection highlights but wrong for, say, contact geometry you want
occluded by the bodies in front of it; there is no depth-tested debug variant.
There is also no cheap way to hide all debug for a clean screenshot.

**Recommendation.** Add a depth-tested debug material variant and a per-descriptor
"overlay vs depth-tested" flag, and put debug geometry on its own Filament view
layer (`View::setVisibleLayers`, as flagged in
[`renderer_realtime_and_scalability.md`](renderer_realtime_and_scalability.md) §LOD)
so a single toggle (and the capture path) can suppress debug without rebuilding
overlays.

### B5. 3D labels beyond the 2D-projection cap

**Gap.** Labels are 2D ImGui text projected from world positions, capped at 96
(`ui_frame.cpp:162`), with no depth sorting/occlusion. Fine for a handful of
annotations; it does not scale to per-contact or per-body labels.

**Recommendation.** Keep the 2D path as default (cheap, crisp) but make the cap a
`DebugDrawOptions` field and add optional depth-aware fade so dense labels degrade
gracefully; a true world-space text mesh is a larger effort — defer.

---

## Prioritized Roadmap

Ordered by return on a small, surgical change. Each is independently shippable.

| #     | Item                                               | Effort | ROI  | Public API change                      | Validation                                  |
| ----- | -------------------------------------------------- | ------ | ---- | -------------------------------------- | ------------------------------------------- |
| A1    | Per-shape PBR (metallic/roughness/reflectance)     | S–M    | High | `MaterialDescriptor` + `VisualAspect`  | scene-extraction test + headless screenshot |
| A3    | Texture mipmaps + trilinear                        | S      | High | none                                   | screenshot of textured asset                |
| B1    | `debugProvider` callback into built-in overlay     | S      | High | `ApplicationOptions` + `DebugScene`    | new gui unit test (`UNIT_gui_DebugVisuals`) |
| A2    | IBL specular reflection cubemap                    | M      | High | none (internal)                        | screenshot of A1 metal sphere               |
| B3    | Arrowheads + velocity/force/joint-axis debug types | M      | High | `DebugDrawOptions` + helpers           | per-helper unit tests                       |
| A4    | Capture fidelity = GPU-gated, not headless-gated   | M      | Med  | via `FidelityProfile`                  | offscreen-parity + GPU headless screenshot  |
| B2    | Supported numeric debug tuning in built-in panel   | S      | Med  | none                                   | manual/headless smoke                       |
| B4    | Depth-tested debug variant + view layers           | M      | Med  | per-descriptor flag                    | screenshot occluded-vs-xray                 |
| A5/B5 | Exposure, clearcoat, label scaling                 | M      | Low  | `FidelityProfile` / `DebugDrawOptions` | deferred                                    |

**Suggested first slice** (smallest change, biggest perceived jump): A1 + A3 + B1.
A1+A3 make the _scene_ look materially better with near-zero API risk; B1 removes
the per-example debug wiring and is the literal "support it in the component"
deliverable.

## Validation

Per [`gui-rendering.md`](../onboarding/gui-rendering.md), renderer changes need the
focused extraction test plus at least one bounded headless render — command
success is not sufficient for material/lighting/debug regressions:

```bash
cmake --build build/default/cpp/Release --target UNIT_gui_DebugVisuals
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_gui_DebugVisuals$'
pixi run ex dartsim --headless --frames 1 --width 1280 --height 720 \
  --screenshot /tmp/dartsim.ppm
pixi run gui-offscreen-parity        # for A4 / render-target work
```

Inspect the captured PPM for material, reflection, texture-aliasing, and debug
overlays; A1/A2/A3 must be judged visually, not by exit code.

## Non-Goals

- A DART-side scene graph / BVH (Filament already frustum-culls;
  [`renderer_realtime_and_scalability.md`](renderer_realtime_and_scalability.md)).
- Exposing Filament/GLFW/ImGui types in public headers (backend-hidden rule).
- The camera-sensor descriptor, lens distortion, and offline accumulation, which
  belong to [`filament_fidelity_profile.md`](filament_fidelity_profile.md).
