# Resume

Branch `feature/dart7-filament-m6a` is non-tracking from `main` `3d31d568794`.
It ports the tested Filament work (the earlier branch at `fc4a3d9cc31` plus its
resolved merge with `main`) as DART 7.0 milestone M6a. Pushing, opening a PR,
or any other GitHub mutation needs explicit maintainer approval.

## Current state

The OSG replacement, the native FTexture use-after-free repair, the C21
removal hygiene, and the interim camelCase `dartpy.gui` nanobind module are
implemented and locally qualified on Linux (see `verification.md`). M6b (the
DART 7.0 program) builds `dart.render` and `dart.viewer` on the C++
`dart::gui` backend and deletes the interim module before 7.0 rc.

## Next steps

1. Ask the maintainer to approve pushing the branch and opening a draft PR
   (`dart-pr`; plain title). Before every push, merge the latest `main` and
   rerun `pixi run test-all`, `pixi run test-gui`, and
   `pixi run -e gazebo test-gz`. After the DART 7.0 M0 rollover lands, merge
   it and retarget the CHANGELOG entry from main's unreleased section.
2. After publication, watch `CI Filament` (Linux/Windows/macOS). Windows uses
   Mesa WGL copied next to the test binaries; macOS uses Metal. Pixel bounds
   were tuned on RTX and Lavapipe; widen only with captured per-backend
   evidence.
3. M6a gates before the C2 checkpoint: the C22 headless-import audit, a
   separately cached Filament SDK stage for x86_64 and aarch64 in
   `docker/wheels/Dockerfile.manylinux_2_28`, native Windows/macOS wheels, and
   quiet-host performance on GPU and Lavapipe.
4. On a quiet host, run the OSG-baseline comparison with
   `scripts/compare_gui_renderers.py` for completed frames, setup, full RGBA,
   RSS and GPU memory on RTX and Lavapipe. Never stop other sessions' work;
   record the host state. The OSG reference (binary, its 50 libraries,
   `manifest.json`, and the OSG-capable `gui_render_benchmark.cpp`, SHA256
   `bdfe859c...`, which is in no git ref) exists only in
   `/tmp/dart-filament-osg-reference-before-removal` (102 MB), and `/tmp` is
   cleared at reboot. Move it to durable storage first; otherwise rebuild it
   from `ec43ea419fe` in an environment that still has OpenSceneGraph.
5. Retire this folder once publication and the remaining gates close; durable
   facts are already in the owner docs listed in `README.md`.

## Known follow-ups (not blockers)

- `config` and `config-demos` share `build/<env>/cpp/Release` (system versus
  bundled docking ImGui), so alternating `test-gui` and `demos` reconfigures
  that tree.
- A/B perf runs against OSG-era base revisions cannot build their OSG-linked
  `contact_benchmark` once the environment lacks OpenSceneGraph.
- Explicit `--width/--height` demo windows open at the origin instead of
  centered; `examples/demos/GuiBackend.hpp` is a thin alias layer that could be
  inlined; test-only `detail::readViewerPixelsRGBA` hooks are declared in the
  installed `Viewer.hpp`.
- A viewer must not be destroyed by its own event handler in C++ (Python keeps
  it alive for the duration of `frame()`/`run()`).
- A static `dart-gui` built with the private SDK does not install that SDK;
  configure warns and consumers must set `Filament_ROOT` (documented in
  `build.rst`). Installing the SDK would remove that step.
- A system SDK without `notices/` (the conda package) makes the first GUI
  configure download the 1.77.3 notices; offline configures fail, and
  `FindFilament` does not enforce the SDK version (only matc/runtime
  agreement).
- Windows `/MT` CRT selection in `FindFilament` and Windows Python task paths
  are untested.

## Recovery notes

- `pixi run config` builds the GUI and dartpy; `pixi run test-gui` runs the
  display-free GUI tests; native tests:
  `ctest --test-dir build/default/cpp/Release -L filament-native` (needs a
  desktop). Configure with `-DDART_ENABLE_GUI_SMOKE_TESTS=ON` to add
  `offscreen_capture_smoke`.
- Linux display-free runs can force Lavapipe with
  `VK_DRIVER_FILES=$CONDA_PREFIX/share/vulkan/icd.d/lvp_icd.x86_64.json`.
- The deterministic texture-lifetime regressions are
  `FilamentBackend.RetiredImGuiTextureLeavesNoCachedSamplerReferences`,
  `FilamentNative.RetiredImGuiTextureLeavesNoCachedSamplerReferences`, and
  `FilamentNative.ViewerSurvivesAtlasGrowthScaleChangesAndRepeatedWindows`.
- The pre-port branch `feature/filament-renderer-6.21` and its uncommitted
  merge stay in the original worktree as local-only reference.
