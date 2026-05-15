# Filament GUI Replacement - Completion Audit

## Objective

Promote Filament with GLFW3 and Dear ImGui as DART's only maintained renderer
under `dart::gui`, replacing the legacy OpenSceneGraph renderer and removing
the Raylib experiment from supported build, example, tutorial, Python, and
documentation surfaces.

## Current Decision

Filament with GLFW3 and Dear ImGui is the maintained `dart::gui` renderer in
this branch. `DART_BUILD_GUI` now builds the Filament-backed GUI component,
`dart-gui` owns the promoted renderer-independent GUI declarations under
`dart::gui`, and `dartsim` is the application-level visual runner. The old
`filament_gui` / `dart_filament_gui` user-facing names are migration
compatibility rejections, not official examples.

The OpenSceneGraph implementation, Raylib experiment, legacy C++/Python GUI
examples and tutorials, legacy dartpy GUI bindings/stubs, OSG/Raylib dependency
discovery, and active OSG/Raylib build/test references were removed from the
supported source tree.

## Completion Checklist

| Requirement                                     | Evidence                                                                                                                                                                                                             | Status                                                                   |
| ----------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| Keep the public direction as `dart::gui`.       | `dart-gui` is now a real public component; installed aggregate headers expose DART-owned GUI concepts without OSG types, while old experimental includes remain compatibility shims.                                 | Satisfied.                                                               |
| Avoid multiple maintained renderer backends.    | `DART_BUILD_GUI` resolves to Filament. OSG and Raylib build paths, dependency discovery, examples, tutorials, and Python legacy bindings were removed.                                                               | Satisfied.                                                               |
| Make Filament the default development renderer. | Linux x86_64 defaults to the pinned Filament fetch path when needed, and `pixi run config` configures Filament GUI by default.                                                                                       | Satisfied for the maintained Linux development path.                     |
| Preserve a package-manager or fallback path.    | `DART_FETCH_FILAMENT` provides the current fallback because conda-forge does not yet publish Filament.                                                                                                               | Satisfied for source builds; conda packaging remains external follow-up. |
| Keep examples out of backend internals.         | The GUI application CMake helper now rejects direct Filament includes and non-minimal example sources for `examples/dartsim`; backend implementation files live under `dart/gui/experimental/detail/filament`.       | Satisfied for the official runner.                                       |
| Remove legacy C++ GUI surfaces.                 | OSG headers/sources, render nodes, legacy GUI tests, legacy GUI examples, and legacy GUI tutorials were deleted; `tutorials/CMakeLists.txt` now reports the removal.                                                 | Satisfied.                                                               |
| Remove legacy Python GUI surfaces.              | Legacy dartpy GUI binding files, stubs, examples, tutorials, and tests were deleted; `dartpy.gui` now exposes the experimental Filament-backed descriptor/helpers only.                                              | Satisfied.                                                               |
| Remove OSG/Raylib build and packaging hooks.    | OSG/Raylib finder files, Raylib fetch logic, OSG package dependencies, and stale CI/script options were removed; legacy Raylib cache options now fail fast.                                                          | Satisfied.                                                               |
| Update user-facing docs.                        | Building, build-system, architecture, testing, Python, GUI rendering, examples, tutorials, ReadTheDocs, and changelog text now identify Filament as the maintained path and mark legacy OSG/Raylib paths as removed. | Satisfied.                                                               |
| Verify rendering path still works.              | Local build, unit tests, Python tests, and a headless screenshot smoke passed; the generated PPM was nonblank and did not emit JPEG-version mismatch warnings.                                                       | Satisfied locally.                                                       |

## Verified Commands

Recent local evidence for this promotion branch:

```bash
pixi run config
cmake --build build/default/cpp/Release --target dart-gui dartsim dartpy examples --parallel 4
cmake --build build/default/cpp/Release --target UNIT_gui_FilamentSceneExtraction --parallel 4
ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_gui_FilamentSceneExtraction$'
PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-} pixi run python -m pytest python/tests/unit/test_run_cpp_example.py python/tests/unit/gui/test_gui_scene.py -q
LIBCXX_PREFIX=/home/jeongseok/dev/jslee02/dartsim/dart/task_2/.pixi/envs/default pixi run ex dartsim --headless --frames 1 --width 320 --height 240 --screenshot /tmp/dart_gui_promote.ppm
pixi run python dart/gui/experimental/detail/filament/testing/analyze_headless_smoke.py /tmp/dart_gui_promote.ppm --width 320 --height 240 --mode basic
git grep -n -E 'tutorials/(biped|collisions|dominoes|multi-pendulum|wholebody-ik)|tutorial_[a-zA-Z0-9_/-]' -- docs examples python tutorials CMakeLists.txt
```

## Remaining Follow-Up Scope

These are not blockers for replacing the official renderer, but they remain
useful follow-on work:

- Validate macOS and Windows source builds on CI workers.
- Add ImGui Docking as the default GUI shell.
- Embed the Filament 3D scene as a dockable window widget.
- Expand the current headless screenshot path into richer offscreen rendering.
- Restore the historical `--out <dir>` image-sequence workflow in the active
  branch, then add automated video capture as a later capture workflow.
- Track external Filament packaging so source builds do not need the pinned
  archive fallback long term.
