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

## Dependency Sources

`DART_USE_SYSTEM_DEPENDENCIES=ON` seeds the defaults of
`DART_USE_SYSTEM_GOOGLETEST`, `DART_USE_SYSTEM_GOOGLEBENCHMARK`, and
`DART_USE_SYSTEM_TRACY` to use installed packages. The umbrella defaults to
`OFF`; other dependency switches keep their existing defaults. ODE and Bullet
already default to system packages. Each option keeps its cached value:
set the umbrella on the first configure of a fresh
build tree, or explicitly set each per-dependency switch when reconfiguring.
An explicit per-dependency `-D` always wins over the umbrella.

For offline builds, set `FETCHCONTENT_FULLY_DISCONNECTED=ON` and either use
installed dependencies through their system switches or pre-seed source
trees with `FETCHCONTENT_SOURCE_DIR_<NAME>` (uppercase name). DART stops with
an actionable error if a fetch needs a missing source tree, including with
CMake older than 3.30; on 3.30 and newer it also enables CMP0170. Existing
populated source directories in the build tree are accepted. The stable
FetchContent names are `googletest`, `benchmark`, `tracy`, `nanobind`,
`dart_imgui`, `dart_filament_sdk`, `fmt`, `ode`, and `bullet`. The umbrella
does not select system nanobind, ImGui, Filament, or fmt; set those switches
separately when needed. Pixi configure tasks use the umbrella;
`pixi run -e profile config-tracy` selects the profile environment's Tracy.
To fetch Tracy instead, override the umbrella with `DART_USE_SYSTEM_TRACY=OFF`.

`DART_USE_SYSTEM_ODE=OFF` fetches ODE 0.16.6 from its GitHub release tag,
builds a shared, double-precision library with system libccd and without the
libccd box-cylinder collider, and installs ODE's headers, library and CMake
package alongside DART. Installed `collision-ode` consumers must include that
prefix in `CMAKE_PREFIX_PATH`; libccd remains a system dependency. System ODE
must also use double precision; configuration rejects single-precision packages.
For a nonstandard prefix on Linux, include its library directory in
`LD_LIBRARY_PATH` or configure an install RPATH.
The compatibility policy minimum needed by ODE under CMake 4 is scoped to the
dependency's configure, without changing DART's policy settings.

`cmake --build <build-directory> --target dart-uninstall` removes files listed
in DART's install manifest, including dependencies installed by that build.
With system dependencies, `uninstall` forwards to `dart-uninstall`. Fetched
ODE defines its own `uninstall` target, which retains ODE's upstream behavior;
it expects an ODE sub-build install manifest and does not uninstall DART.
Use `dart-uninstall` to remove the complete DART installation in that case.
Uninstall uses the prefix recorded by the last installation and honors `DESTDIR`.

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

`DART_USE_SYSTEM_BULLET=OFF` fetches Bullet 3.25 and builds and installs only
the shared `BulletCollision` and `LinearMath` libraries and their headers,
with double precision enabled. DART installs the headers directly under
`<prefix>/include`, not upstream's `include/bullet`. Bullet 3, extras, demos, Python bindings and
upstream unit tests are disabled. The CMake 4 policy minimum is scoped to
Bullet's configure. This is a collision-only SDK; the installed pkg-config
file lists only these two libraries, and upstream's full Bullet CMake package
is not installed.

The installed DART finder requires only Bullet's headers, `BulletCollision`
and `LinearMath`, so this collision-only SDK also supports DART consumers
without BulletDynamics or BulletSoftBody installed.

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
