# Resume: Demos App

## Last Session Summary

`dart-demos` consolidates DART's GUI examples into one window with a categorized,
runtime-switchable scene sidebar. The runtime scene-swap lives in `dart::gui`
(`dart::gui::runDemos`). **37 scenes** are migrated and verified headless.
Branch `demos-app`, checkpoint commits. Phases 1, 2, and 4 are essentially done;
Phase 3 (legacy fixture-catalog collapse) and Phase 5 (promote design) remain.

## Current Branch

`demos-app`. Key commits (newest last):

- PLAN-102 docs
- `feat(gui)`: runtime scene switching + `runDemos` host + sidebar + 3 seeds
- `feat(demos)`: migrate 21 simpler examples (24 scenes)
- `refactor(examples)`: remove 22 consolidated dirs
- `feat(demos)`: migrate physics/robot examples (34 scenes)
- `refactor(examples)`: remove 10 more dirs
- `feat(demos)`: migrate multi-file/special examples (39 scenes)
- `refactor(examples)`: remove last 5 dirs
- `refactor(demos)`: keep atlas_simbicon/collision_sandbox standalone (37
  scenes); wire `pixi run demos`, run_cpp_example, READMEs, CHANGELOG
- (pending) demos cycle CTest smoke

## What Is Done

- **Library**: `dart::gui::runDemos` + `ViewerLifecycleState::sceneSwitchRequested`
  - an outer scene loop in `dart/gui/detail/application.cpp` that rebuilds the
    scene-bound state (world/renderables/panels/handlers/camera) on switch while
    the window/engine/materials/imgui/lights persist; `destroySceneRenderables` /
    `destroyPersistentApplicationResources` split in application_teardown. The
    single-scene and legacy `--scene` paths are unchanged (verified).
- **App**: `examples/demos/` (`scenes/*.cpp`, `registry.cpp`, `scenes.hpp`,
  `app/main.cpp`) builds `dart-demos` with 37 scenes across 8 ordered
  categories. Soft-fails a scene whose factory throws (shows an empty world).
- **Removed** standalone dirs for every migrated GUI example; kept standalone:
  `hello_world` (template), `atlas_simbicon`/`collision_sandbox` (their
  controller/pair-registry sources are compiled by tests — verified passing),
  `wam_ikfast` (IKFast lib), `gui_scene_diagnostics` (headless inspector); and
  the 4 headless CLI examples.
- **Tooling/docs**: `pixi run demos`, `demos` in `run_cpp_example.py` (removed
  names redirect to `pixi run demos -- --scene <id>`), `examples/README.md`,
  `dartsim/README.md`, `CHANGELOG.md`.
- **Verified headless**: `dart-demos --cycle-scenes --headless` exits 0 across
  all 37 scenes; a single-scene `--screenshot` renders non-blank; the two
  example-source-reusing tests (`INTEGRATION_simulation_World`,
  `test_collision_sandbox_pair_registry`) pass.

## Remaining Work

- **Phase 3 (deferred — library-internal, risky):** collapse the
  `dart/gui/detail/scenes.{hpp,cpp}` `ExampleScene` fixture catalog. It is still
  used by `runApplication(argc,argv,defaultScene)` / `--scene`, the
  `EXAMPLE_dartsim_<scene>_headless_smoke` tests, `test_filament_scene_extraction`,
  `dartsim --scene`, and `run_cpp_example.py` `FILAMENT_ALL_SCENES`. To collapse
  it: split `examples/demos` into a `demos_scenes` library + `dart-demos` app so
  tests can link the registry; retarget the renderable-extraction test to the
  registry; reduce the in-library catalog to one `Mvp` self-smoke; update
  `dartsim`'s smoke gate (PLAN-101) and `run_cpp_example.py`. This is separable
  from the user-facing consolidation and was deferred to avoid a rushed, broad
  test-suite break.
- **Phase 5:** once Phase 3 lands, promote `01-design.md` ->
  `docs/design/demos_app.md` (register in `docs/design/README.md`), add a note to
  `docs/onboarding/gui-rendering.md`, delete this folder, and run
  `pixi run test-all`.
- Optional: split `examples/demos` into lib+app (needed for Phase 3 test
  linking); a single-scene non-blank analyzer smoke per category.

## Build / Run / Verify

- Build: `pixi run python scripts/cmake_build.py --build-dir build/default/cpp/Release --target dart-demos`
  (verify success by grepping the log for `error:|FAILED|undefined reference`;
  the wrapper exit code is unreliable).
- Run headless (live `DISPLAY=:0`, no xvfb needed), wrap in `pixi run bash -lc`:
  `LD_LIBRARY_PATH=$PWD/build/default/cpp/Release/lib:$CONDA_PREFIX/lib LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe build/default/cpp/Release/bin/dart-demos --cycle-scenes --headless`.
- Scene migration recipe: `int main` body -> `dart::gui::ApplicationOptions makeXxxScene()`;
  drop argc/argv, `runDefaults`, `DART_PROFILE_*`, instruction prints, CLI
  parsing; keep camera/panels/gizmos/ikHandles/keyboardActions/preStep/postStep
  and world/solver/collision settings. Asset scenes need `dart-io` (linked);
  collision backends are runtime-registered. Multi-file examples merge into one
  TU; if it has unused members, wrap helpers in a named (not anonymous)
  namespace to avoid `-Werror=unused-function`.

## How to Resume

```bash
git checkout demos-app
git status && git log -12 --oneline
```

Then read `01-design.md` and continue with Phase 3 (or land the demos CTest
smoke if that commit is not present).
