# Resume: Demos App

## Last Session Summary

Building the `dart-demos` app that consolidates DART's GUI examples into one
window with a categorized, runtime-switchable scene sidebar. Branch `demos-app`,
checkpoint commits. Foundation (runtime scene swap in `dart::gui` +
`dart::gui::runDemos` + sidebar) is done and verified headless. 24 scenes are
live; 22 standalone example dirs have been deleted (consolidated).

## Current Branch

`demos-app` — checkpoint commits landed:

1. PLAN-102 docs
2. `feat(gui)`: runtime scene switching + dart-demos host + 3 seed scenes
3. `feat(demos)`: migrate 21 GUI examples into scene modules (24 total)
4. `refactor(examples)`: remove 22 consolidated standalone dirs

## Immediate Next Step

Integrate the in-flight robot/physics migration wave (agents porting:
rigid_shapes, heightmap, lcp_physics, biped_stand, tinkertoy,
operational_space_control, point_cloud, fetch, atlas_puppet, hubo_puppet) — add
their decls to `examples/demos/scenes.hpp`, entries to `registry.cpp`, sources to
`examples/demos/CMakeLists.txt`, build `dart-demos`, cycle-test headless, commit,
then delete those standalone dirs.

## Remaining Work

- **Phase 2 tail still standalone**: g1_puppet, human_joint_limits (3 files),
  collision_sandbox (3 files; links `dart-collision-native`),
  experimental_rigid_body_gui (links `dart-simulation-experimental`),
  atlas_simbicon (9 files), wam_ikfast (needs `DART_WAM_IKFAST_LIB_PATH`),
  gui_scene_diagnostics (headless descriptor inspector — likely keep standalone
  or drop; not a visual scene).
- **Phase 3**: retarget `tests/unit/gui/test_filament_scene_extraction.cpp` +
  per-scene headless smokes to the demos registry; delete the
  `dart/gui/detail/scenes.{hpp,cpp}` `ExampleScene` example catalog + `--scene`
  example overloads; keep one tiny `dart-gui` MVP self-smoke.
- **Phase 4**: `scripts/run_cpp_example.py` (drop per-example GUI specs; add a
  `demos` target + `--cycle-scenes` smoke), pixi `demos` task, `examples/README.md`,
  `dartsim/README.md` cross-refs, dartpy docs, `CHANGELOG.md`.
- **Phase 5**: promote `01-design.md` -> `docs/design/demos_app.md` (register in
  `docs/design/README.md`), onboarding note, delete this dev-task folder; run
  `pixi run test-all`.

## Key Facts / Gotchas

- Build/run: GUI is configured in `build/default/cpp/Release`. Build a target with
  `pixi run python scripts/cmake_build.py --build-dir build/default/cpp/Release --target dart-demos`.
  Run headless (a live `DISPLAY=:0` exists, so no xvfb needed):
  `LD_LIBRARY_PATH=$PWD/build/default/cpp/Release/lib:$CONDA_PREFIX/lib LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe build/default/cpp/Release/bin/dart-demos --cycle-scenes --headless`.
  Wrap in `pixi run bash -lc '...'` for the env.
- `--cycle-scenes` renders a few frames of each scene then exits (switch-stress);
  `--scene <id>` selects the initial scene; both are parsed by `runDemos`.
- Scene migration recipe: `int main` body -> `dart::gui::ApplicationOptions makeXxxScene()`;
  drop argc/argv, `runDefaults`, `DART_PROFILE_*`, instruction prints, CLI parsing;
  keep camera/panels/gizmos/ikHandles/keyboardActions/preStep/postStep and
  world/solver/collision settings. Asset scenes need `dart-io` linked (already in
  `examples/demos/CMakeLists.txt`); collision backends are runtime-registered.
- The host soft-fails a scene whose factory throws (shows an empty world), so a
  missing asset can't crash the app.
- Verify build SUCCESS by grepping the log for `error:|FAILED|undefined reference`
  — the cmake_build wrapper's exit code is not reliable.

## How to Resume

```bash
git checkout demos-app
git status && git log -8 --oneline
```

Then read `01-design.md`, continue from "Immediate Next Step".
