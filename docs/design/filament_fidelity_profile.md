# Filament Renderer Fidelity Profile And Camera-Sensor Seam

Durable design rationale for configuring DART's Filament renderer across a
real-time-first default and higher-fidelity offline use (synthetic-data
generation, photorealistic camera-sensor simulation, egocentric views, lens
distortion). This doc owns the design shape; roadmap priority and status live in
`docs/plans/dashboard.md`. Implementation had not started when this was written;
treat the types below as a proposed seam, not a current API.

## Goal And Constraints

- Real-time interaction by default; an explicit path to maximize image quality
  for offline rendering.
- The configuration surface stays a renderer-neutral DART concept. Public
  headers must not expose Filament/GLFW/ImGui/OpenGL/Vulkan types
  (`docs/onboarding/gui-rendering.md`).
- Reuse existing capture plumbing (screenshot / frame-output) rather than adding
  parallel paths.

## Core Idea: A Fidelity Profile

A single bundled "fidelity profile" in public options, mapped internally to
Filament `View`/`Renderer` settings in `dart::gui::detail`, with presets:

- **Realtime** (default): favor frame rate — lower/auto shadows, modest or no
  MSAA, no expensive post, dynamic resolution allowed.
- **Offline / high-fidelity**: favor image quality — high MSAA/TAA, full
  shadows, SSAO/bloom, temporal accumulation, and no frame skipping.

```cpp
namespace dart::gui {

enum class FidelityPreset { Realtime, Balanced, Offline };
enum class AntiAliasing  { None, Fxaa, Msaa4x, Taa };

struct FidelityProfile {
  FidelityPreset preset = FidelityPreset::Realtime;
  double resolutionScale = 1.0;     // dynamic-res / supersample factor
  AntiAliasing antiAliasing = AntiAliasing::Fxaa;
  int   shadowMapSize = 1024;       // 0 = shadows off
  bool  ambientOcclusion = false;
  bool  bloom = false;
  int   accumulationSamples = 1;    // >1 = offline temporal accumulation
  bool  allowFrameSkip = true;      // false for deterministic offline capture
};

// Camera-sensor description for synthetic data / sensor simulation. Attaches to
// a DART frame; intrinsics and distortion are DART concepts, not backend types.
struct CameraSensorDescriptor {
  std::string name;
  std::string mountFrame;                       // DART frame the camera rides
  Eigen::Isometry3d localPose = Eigen::Isometry3d::Identity();
  double horizontalFovDegrees = 60.0;
  int    width = 1280, height = 720;
  std::array<double, 5> distortionKoeff{};      // Brown-Conrady k1,k2,p1,p2,k3
  bool outputColor = true, outputDepth = false, outputSegmentation = false;
};

} // namespace dart::gui
```

## Internal Mapping (`dart::gui::detail`)

- `FidelityProfile` → `View::setAntiAliasing` /
  `setMultiSampleAntiAliasingOptions`, shadow type and shadow-map size,
  `setAmbientOcclusionOptions`, `setBloomOptions`,
  `setDynamicResolutionOptions`; `allowFrameSkip = false` forces the
  deterministic capture path already used by the headless screenshot harness.
- Presets are preset `FidelityProfile` values; Offline maxes quality, disables
  frame skip, and enables accumulation.
- `CameraSensorDescriptor` → one `View` + `Camera` per sensor; lens distortion is
  a post-process material parameterized by `distortionKoeff`; depth and
  segmentation are extra render targets selected by the `output*` flags. Per-
  sensor output reuses the existing screenshot/frame-output capture plumbing.

## Compatibility

`RenderSettings` already carries `shadowsEnabled` and `outputMode`; a
`FidelityProfile` supersedes those, with the old fields kept as thin shims during
transition. No public header gains a backend, windowing, or UI type.

## Why This Seam

Making the renderer fast and measurable is necessary but not sufficient: if the
fidelity controls are an afterthought, sensor-simulation features get bolted on
per-feature and leak backend details. Defining the profile plus the
camera-sensor descriptor seam keeps "best-effort real-time" and "best-fidelity
offline" on one code path with different settings, and keeps the public API
renderer-neutral.
