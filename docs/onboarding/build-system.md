# Build System Notes

DART uses CMake through Pixi tasks. Dependency cleanup must preserve
exported CMake package behavior unless a maintainer explicitly accepts a
compatibility break.

Important checks:

- `pixi run config`
- `pixi run build`
- `pixi run test`
- `pixi run -e gazebo test-gz` for Gazebo/gz-physics compatibility surfaces

When moving optional dependencies out of the default environment, verify both
dependency-present and dependency-absent configure paths where practical.

For DART 6.21, one `find_package(DART CONFIG REQUIRED ...)` exports three
primary C++ libraries:

| Component | Target | Contents |
| --------- | ------ | -------- |
| `dart` | `dart` | Headless core and enabled collision backends |
| `io` | `dart-io` | SKEL, SDF, MJCF and URDF loading |
| `gui` | `dart-gui` | OSG and ImGui integration |

The default pybind11 `dartpy` module includes all three libraries. The opt-in
nanobind binder currently includes core and IO; its existing GUI limitation
remains during the binder transition. See [Python bindings](python-bindings.md)
for the binder contracts and wheel checks.

## DART 6.21 Headless Core

`DART_BUILD_IO=OFF`, `DART_BUILD_GUI=OFF`, and `DART_BUILD_DARTPY=OFF` configure
the standalone headless `dart` library. The complete GUI directory is skipped,
including ImGui and graphics discovery. The legacy `DART_BUILD_GUI_OSG=OFF`
also disables GUI; set both GUI options to `ON` when re-enabling it in a reused
build directory.

Optional Bullet and ODE implementations compile into core. Their legacy CMake
components and targets forward to `dart` for rebuilt consumers; the former
collision shared-library filenames are no longer installed. `DART_SKIP_BULLET`
and `DART_SKIP_ODE` disable the respective backends, including bundled downloads.
The FCL default and public detector classes remain unchanged.

Run `pixi run test-package-core` for a core-only build, isolated installation,
and external world/factory consumers. See the
[package checks](../../tools/package/README.md) for static builds, legacy
components, and parent-project embedding.

## DART 6.21 IO

`dart-io` contains the SKEL, SDF, MJCF and URDF loaders. Enable it with
`DART_BUILD_IO=ON` (the default); it requires tinyxml2 and urdfdom. Set
`DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF` for a headless core plus IO
build.

Use `find_package(DART CONFIG REQUIRED COMPONENTS io)` and link `dart-io`.
The `dart::utils` namespace and `dart/utils/...` header paths are preserved.
Legacy `utils` / `utils-urdf` components and `dart-utils` / `dart-utils-urdf`
targets forward to `dart-io`; applications linked to their former shared
libraries must rebuild.

## DART 6.21 GUI

`dart-gui` combines OSG and ImGui and depends directly on core. It supports
`DART_BUILD_IO=OFF` when Python is disabled. `DART_BUILD_GUI=ON` requires
OpenSceneGraph and OpenGL; `DART_USE_SYSTEM_IMGUI` selects the existing system
or bundled ImGui configuration.

Use `find_package(DART CONFIG REQUIRED COMPONENTS gui)` and link `dart-gui`.
OSG and ImGui compatibility headers keep their current paths. Legacy
`gui-osg` / `external-imgui` components and `dart-gui-osg` /
`dart-external-imgui` targets forward to `dart-gui`; applications linked to
their former shared libraries must rebuild.
