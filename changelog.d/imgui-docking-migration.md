Section: Breaking Changes

  * The installed bundled Dear ImGui headers under
    `dart/external/imgui` use the docking layout; rebuild applications that
    include them. `DART_IMGUI_DOCKING` is removed. With a system ImGui that
    lacks docking, dart-gui still builds, but
    dartpy omits `dartpy.viewer`.
    [#3745](https://github.com/dartsim/dart/pull/3745),
    [#3761](https://github.com/dartsim/dart/pull/3761),
    [#3928](https://github.com/dartsim/dart/pull/3928)
