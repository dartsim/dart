# Resume

Branch `feature/dart7-filament-m6a` is non-tracking from `main` `3d31d568794`.
It ports the tested Filament work (the earlier branch at `fc4a3d9cc31` plus its
resolved merge with `main`) as DART 7.0 milestone M6a. It is published as
draft PR #3696 (milestone "DART 7.0.0"). Pushes in the original session were
maintainer-approved; PR edits, comments, review requests, marking ready, and
merging each need fresh maintainer approval.

## Current state

The OSG replacement, the native FTexture use-after-free repair, the C21
removal hygiene, the C22 typed `RenderingUnavailableError`, and the interim
camelCase `dartpy.gui` nanobind module are implemented and locally qualified on
Linux (see `verification.md`). The first CI rounds fixed Windows macro
collisions (`near`/`far`), an AppleClang `typeid` warning, a duplicate Filament
config import on case-insensitive filesystems, line-ending-dependent wheel
license tests, the `CI Filament` workflow's job-level contexts and targets, and
offline-cache demo cycling. The first Codex review's four findings (release
picks, dynamic triangle hashing, cursor window depth, point-cloud overall
color) are fixed with regressions. M6b (the DART 7.0 program) builds
`dart.render` and `dart.viewer` on the C++ `dart::gui` backend and deletes the
interim module before 7.0 rc.

## Next steps

1. Expected CI failures until resolved upstream: the release-target check
   fails while `main`'s release-management row still names 6.21.0 (it also
   skips the Linux and Windows C++ "Build and test" steps that run after
   lint); merge `main` once the DART 7.0 rollover lands and retarget the
   CHANGELOG entry. Perf A/B cannot build the OSG-based base revision and needs
   a maintainer decision.
2. Before every push, merge the latest `main` and rerun `pixi run test-all`,
   `pixi run test-gui`, and `pixi run -e gazebo test-gz`. Watch `CI Filament`
   (Linux/Windows/macOS); Windows uses Mesa WGL copied next to the test
   binaries and macOS uses Metal. Pixel bounds were tuned on RTX and Lavapipe;
   widen only with captured per-backend evidence.
3. After the DART 7.0 GIL finalization fix lands on `main`, apply its guard
   rules to the GUI binding callbacks and destruction notifications.
4. M6a gates before the C2 checkpoint: a separately cached Filament SDK stage
   for x86_64 and aarch64 in `docker/wheels/Dockerfile.manylinux_2_28`, native
   Windows/macOS wheels (the `Publish dartpy` rows have not yet run with
   Filament), and quiet-host performance on GPU and Lavapipe. The C22 slim
   import check runs in `Publish dartpy` for the abi3 Linux wheels.
5. On a quiet host, run the OSG-baseline comparison with
   `scripts/compare_gui_renderers.py` for completed frames, setup, full RGBA,
   RSS and GPU memory on RTX and Lavapipe. Never stop other sessions' work;
   record the host state. The OSG reference (binary, its 50 libraries,
   `manifest.json`, and the OSG-capable `gui_render_benchmark.cpp`, SHA256
   `bdfe859c...`, which is in no git ref) is preserved outside the repository
   on the original development host; otherwise rebuild it from `ec43ea419fe`
   in an environment that still has OpenSceneGraph.
6. Retire this folder once publication and the remaining gates close; durable
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
