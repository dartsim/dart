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

On `main`, one `find_package(DART CONFIG REQUIRED ...)` exports three
primary C++ libraries:

| Component | Target | Contents |
| --------- | ------ | -------- |
| `dart` | `dart` | Headless core and enabled collision backends |
| `io` | `dart-io` | SKEL, SDF, MJCF and URDF loading |
| `gui` | `dart-gui` | Filament GUI and ImGui integration |

The nanobind `dartpy` module includes all three libraries. See
[Python bindings](python-bindings.md) for the binder contracts and wheel checks.

Legacy library targets also remain buildable through
`cmake --build <build-directory> --target <legacy-target>`; they build the
corresponding canonical library.

Gazebo compatibility covers rebuilding released `gz-physics` and `gz-sim`
without source changes. Existing binary plugins must be rebuilt against the
new library layout.

## Headless Core

`DART_BUILD_IO=OFF`, `DART_BUILD_GUI=OFF`, and `DART_BUILD_DARTPY=OFF` configure
the standalone headless `dart` library. The complete GUI directory is skipped,
including ImGui and graphics discovery. The legacy `DART_BUILD_GUI_OSG=OFF`
also selects this headless build (it caches `DART_BUILD_GUI=OFF`), while
`DART_BUILD_GUI_OSG=ON` fails because DART 7 removed the OSG GUI; set
`DART_BUILD_GUI=ON` to re-enable the GUI in a reused build directory.

Optional Bullet and ODE implementations compile into core. Their legacy CMake
components and targets forward to `dart` for rebuilt consumers; the former
collision shared-library filenames are no longer installed. `DART_SKIP_BULLET`
and `DART_SKIP_ODE` disable the respective backends, including bundled downloads.
The FCL default and public detector classes remain unchanged.

Run `pixi run test-package-core` for a core-only build, isolated installation,
and external world/factory consumers. See the
[package checks](../../tools/package/README.md) for static builds, legacy
components, and parent-project embedding.

## IO

`dart-io` contains the SKEL, SDF, MJCF and URDF loaders. Enable it with
`DART_BUILD_IO=ON` (the default); it requires tinyxml2 and urdfdom. Set
`DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF` for a headless core plus IO
build.

Use `find_package(DART CONFIG REQUIRED COMPONENTS io)` and link `dart-io`.
The `dart::utils` namespace and `dart/utils/...` header paths are preserved.
Legacy `utils` / `utils-urdf` components and `dart-utils` / `dart-utils-urdf`
targets forward to `dart-io`; applications linked to their former shared
libraries must rebuild.

`DART_BUILD_UTILS_ASSETS=ON` adds the optional online model retriever to
`dart-io` and requires `DART_BUILD_IO=ON`, libcurl and OpenSSL. Its
`utils-assets` component and `dart-utils-assets` target forward to `dart-io`.

## GUI

Main uses Filament after M6a merged in #3696. The `gui-osg` component fails
with migration text instead of forwarding to `dart-gui`; see the
[DART 7.0 project home](../dev_tasks/dart7_python_first/README.md#m6a-interface).

`dart-gui` combines the Filament GUI and ImGui and depends directly on core. It
supports `DART_BUILD_IO=OFF` when Python is disabled. `DART_BUILD_GUI=ON` (the
default) requires Filament 1.77.3, GLFW 3.4, libpng, libjpeg, and Dear ImGui
1.92.8 or newer. `DART_USE_SYSTEM_FILAMENT=ON` (the default) uses an installed
Filament SDK; `OFF` uses DART's pinned private SDK, which Linux builds from
source with Clang 20 or newer and Ninja, and macOS arm64 and 64-bit Windows
download as prebuilt archives. `DART_USE_SYSTEM_IMGUI` selects the system or
bundled ImGui configuration.

Use `find_package(DART CONFIG REQUIRED COMPONENTS gui)`, link `dart-gui`, and
include `dart/gui/gui.hpp`. ImGui compatibility headers keep their
`dart/external/imgui/` paths. The legacy `external-imgui` component and
`dart-external-imgui` target forward to `dart-gui`; applications linked to the
former ImGui shared library must rebuild. The OSG `gui-osg` component,
`dart-gui-osg` target, and `dart/gui/osg` headers are removed; requesting
`gui-osg` reports the migration instead of forwarding to `dart-gui`. The
[visualization guide](../readthedocs/dart/user_guide/visualization.rst) covers
porting.
