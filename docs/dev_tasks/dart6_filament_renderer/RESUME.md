# Resume

M6a merged in [#3696](https://github.com/dartsim/dart/pull/3696) on 2026-10-11
at `b63cc07f6`. Main now uses Filament and the interim nanobind `dartpy.gui`.
The port originated on `feature/dart7-filament-m6a` from `main` `3d31d568794`,
using the earlier Filament source at `fc4a3d9cc31` and its resolved main merge.
The dated qualification history remains in `verification.md`.

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

The merged PR's `CI Filament` functional lanes passed on ubuntu-24.04,
windows-2022 and macos-15. Linux x86_64 cp312 stable-ABI wheel repair, audit,
and fresh cp312/cp313 smoke tests passed locally. The merged Dockerfile still
builds the SDK during the wheel build, and the merged evidence does not
qualify aarch64, Windows or macOS wheels. The C22 headless-import check passed
locally on the x86_64 stable-ABI wheel and is wired into `Publish dartpy`; the
aarch64 wheel and the publish runs themselves remain.

## Next steps

1. Land the DART 7.0 rollover (#3697), which resolves the release-target
   mismatch and puts the Filament entry in the DART 7.0.0 changelog section.
   #3696 is already merged and needs no further branch merge or PR update.
2. For approved follow-up pushes, merge the latest `main` and rerun `pixi run test-all`,
   `pixi run test-gui`, and `pixi run -e gazebo test-gz`. Watch `CI Filament`
   (Linux/Windows/macOS); Windows uses Mesa WGL copied next to the test
   binaries and macOS uses Metal. Pixel bounds were tuned on RTX and Lavapipe;
   widen only with captured per-backend evidence.
3. After the DART 7.0 GIL finalization fix lands on `main`, apply its guard
   rules to the GUI binding callbacks and destruction notifications.
4. Remaining release gates before the C2 checkpoint: a separately cached
   Filament SDK stage for x86_64 and aarch64 in
   `docker/wheels/Dockerfile.manylinux_2_28`, aarch64 and native Windows/macOS
   wheel qualification, and quiet-host performance on GPU and Lavapipe. The
   merged handoff recorded no Windows/macOS wheel runs with Filament; native
   GUI CI passes do not qualify those wheels. The C22 slim import check runs
   in `Publish dartpy` for the abi3 Linux wheels.
5. On a quiet host, measure Filament completed frames, setup, full RGBA,
   RSS and GPU memory on RTX and Lavapipe against an approved fast-profile
   disposition (decision D32). Record the host state; never stop other
   sessions' work.
6. Retire this folder once the remaining release gates close; durable
   facts are already in the owner docs listed in `README.md`.

## Known follow-ups (not blockers)

- `config` and `config-demos` share `build/<env>/cpp/Release` (system versus
  bundled docking ImGui), so alternating `test-gui` and `demos` reconfigures
  that tree.
- Historical contact benchmarks requiring the removed GUI component fail with
  an infrastructure error; use the `release-6.20` harness for comparisons.
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
- Windows `/MT` CRT selection in `FindFilament` and the Windows `test-gui` pixi
  task override have no recorded qualification; `CI Filament`'s Windows GUI
  ctest and Python GUI suites passed.

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
