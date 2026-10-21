# Build System Notes

DART uses CMake through Pixi tasks. Dependency cleanup must preserve
exported CMake package behavior unless a maintainer explicitly accepts a
compatibility break.

Shared compiler policy lives in `cmake/dart_defs.cmake`. MSVC setup runs before
dependency discovery; GCC/Clang warning and build-type settings run afterward
through `dart_configure_compiler_options()`. The shared MSVC C++ parsing options
feed directory builds, exported header requirements, and the isolated header
probe, with each caller retaining its existing compiler and language guards.
The shared parsing list excludes warning and code-generation settings. The
probe keeps its isolated flags, which the infrastructure checker independently
validates.

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

## Test Tooling

With `BUILD_TESTING=ON` and the default `DART_USE_SYSTEM_GOOGLETEST=OFF` and
`DART_USE_SYSTEM_GOOGLEBENCHMARK=OFF`, DART fetches GoogleTest 1.17.0 and Google
Benchmark 1.10.0 over HTTPS with pinned SHA-256 hashes. GoogleTest uses its release
archive; Benchmark uses its tag archive because that release has no attached
source archive. Fetched test tooling is not installed. On CMake 3.24 and newer,
archives extract with current timestamps so changing an archive rebuilds its
objects; older CMake versions need a fresh build directory after an update.
The pins live in `tests/CMakeLists.txt` and `tests/benchmark/CMakeLists.txt`.
To update a pin, download the new archive with `curl -fLO` and compute its hash
with `sha256sum`.

`DART_USE_SYSTEM_GOOGLETEST=ON` and
`DART_USE_SYSTEM_GOOGLEBENCHMARK=ON` select installed packages instead, as the
default Pixi tasks do. The supported system floors remain Ubuntu 24.04's
GoogleTest 1.14 and Google Benchmark 1.8.3. For offline configures, pre-seed
`FETCHCONTENT_SOURCE_DIR_GOOGLETEST` and `FETCHCONTENT_SOURCE_DIR_BENCHMARK`
with local source trees, or use the system switches. The Linux CI
fetched-test-tooling job configures both switches `OFF`, builds the C++ tests
and benchmarks, runs CTest, and checks that installation excludes test tooling.

## Package Version Compatibility

The installed `DARTConfigVersion.cmake` accepts plain version requests from
6.0 through the installed DART version, including requests for an earlier
major version. Requests below 6.0 are incompatible. `EXACT` requires the full
installed version. Version ranges follow CMake's `AnyNewerVersion` rules with
the same 6.0 floor; an excluded upper bound such as `6.10...<7` rejects DART 7
and newer. The version file retains CMake's pointer-size suitability check.
When a request for an earlier major resolves to DART, the package prints a
short status message unless `QUIET` is set.

`package.xml` normally supplies every C++ version value. The CMake cache
variable `DART_VERSION_OVERRIDE` defaults to empty and is **CI-only; never use
it for packaging**. A nonempty override must be `X.Y.Z` and at least the
`package.xml` version. It supplies `DART_VERSION`, its major/minor/patch
values, generated headers, package files and the `MAJOR.MINOR` SOVERSION.
For example, configure a compatibility build with
`-DDART_VERSION_OVERRIDE=7.0.0`; reset with `-DDART_VERSION_OVERRIDE=`.

The Gazebo compatibility tasks read the `main | Development` release target
from [release management](release-management.md), fail if it cannot be read,
and set the override when that target is newer than `package.xml`. This
exercises the upcoming DART version before release packaging updates the
metadata. Historical comparison sources without override support keep their
own version.

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
`DART_BUILD_IO=ON` (the default); it requires tinyxml2, urdfdom and sdformat
14 (at least 14.8), 15 or 16. Pixi uses sdformat 16.1 or newer within major 16
and urdfdom 6 by default. SDF loading uses sdformat's validation, schema conversion and
frame resolution; tinyxml2 remains required by the other XML loaders.
CMake selects major 16, then 15, then 14 when several SDKs are available,
using versioned or unversioned package names. Installed IO components record
the selected package and major and require that same major, so a consumer
cannot silently link a different sdformat ABI. Core builds with IO disabled
do not discover sdformat.

Set
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
Shared installs link libcurl and OpenSSL privately; only static installs make
consumers find them.

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
download as prebuilt archives. The bundled ImGui, the default, is the Dear
ImGui v1.92.8 docking release, which configure fetches with Git; its installed
`dart/external/imgui/` headers use the docking layout. For offline configures,
pass a local checkout of that tag with `FETCHCONTENT_SOURCE_DIR_DART_IMGUI`.
`DART_USE_SYSTEM_IMGUI=ON` uses an installed ImGui instead; a vanilla system
ImGui builds `dart-gui` without docking.

Use `find_package(DART CONFIG REQUIRED COMPONENTS gui)`, link `dart-gui`, and
include `dart/gui/gui.hpp`. ImGui compatibility headers keep their
`dart/external/imgui/` paths; include ImGui as `<dart/external/imgui/imgui.h>`.
For target-based consumers, if the install prefix also holds another ImGui,
such as a Conda environment's, a bare `<imgui.h>` reaches the bundled header
only when `dart-gui` precedes the other DART targets in `target_link_libraries`;
otherwise it finds that ImGui, whose struct layout differs from the docking
release. The legacy variables `DART_INCLUDE_DIRS` and `DART_LIBRARIES` also
support GUI consumers:
with bundled ImGui and a requested `gui` component, `DART_INCLUDE_DIRS` puts
`<prefix>/include/dart/external/imgui` before `<prefix>/include`, so
`include_directories(${DART_INCLUDE_DIRS})` selects the docking header for a
bare `<imgui.h>`. System-ImGui builds and component requests without `gui`
keep their existing include directories. The legacy
`external-imgui` component and `dart-external-imgui` target forward to
`dart-gui`; applications linked to the former ImGui shared library must
rebuild. The OSG `gui-osg` component, `dart-gui-osg` target, and `dart/gui/osg`
headers are removed; requesting `gui-osg` reports the migration instead of
forwarding to `dart-gui`. The
[visualization guide](../readthedocs/dart/user_guide/visualization.rst) covers
porting.
