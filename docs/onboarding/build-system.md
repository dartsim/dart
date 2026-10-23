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
CMake older than 3.30; on 3.30 and newer it also enables CMP0170. The check
runs right after each `FetchContent_MakeAvailable`, so a registered
dependency provider (CMake 3.24+) can still fulfill a request without a source
tree; source-consuming dependencies such as the Filament SDK keep enforcing
their own SDK and license requirements. Existing
populated source directories in the build tree are accepted. The stable
FetchContent names are `googletest`, `benchmark`, `tracy`, `nanobind`,
`dart_imgui`, `dart_filament_sdk`, `ode`, and `bullet`. The umbrella
does not select system nanobind, ImGui, or Filament; set those switches
separately when needed. Pixi configure tasks use the umbrella;
`pixi run -e profile config-tracy` selects the profile environment's Tracy.
To fetch Tracy instead, override the umbrella with `DART_USE_SYSTEM_TRACY=OFF`.
System Tracy must be 0.11.1 or newer when it reports a version (CMake
variables, or the selected `Tracy::TracyClient` target's `common/TracyVersion.hpp`);
upstream installs no version file, so a versionless package is accepted.

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

## Eigen Compatibility

DART supports Eigen >= 3.4.0 and < 6. Every Pixi environment pins Eigen 3.4.0
through the `eigen34` feature; the toolchain CI lane rewrites that pin to build
and test Eigen 5 with warnings as errors.
The installed CMake package records the major used to build DART and rejects
a consumer using a different major; pkg-config consumers must match majors
themselves. Eigen 3.4.90 development snapshots are treated as Eigen 3.
Eigen 3 and 5 are ABI-incompatible for DART's public Eigen types: select the
same major or rebuild DART and its consumers.

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
By default a missing optional dependency (Bullet, ODE) only warns and drops its
backend. Gazebo-facing packagers, whose gz-physics needs both, should pass
`-DDART_REQUIRE_OPTIONAL_DEPENDENCIES=ON` so configure fails instead; the error
names the dependency and the component, and an explicit `DART_SKIP_<NAME>=ON`
still disables one on purpose. The Gazebo pixi task (`config-dart-gz`) and
the compat lanes set it.

With system Bullet 3.17 or newer, `DART_BULLET_FLOAT64` (default `ON`) links
the `-float64` libraries installed beside the single-precision ones, because
single-precision Bullet's absolute GJK tolerances lose contacts of
centimeter-scale shapes ([gz-physics#1117](https://github.com/gazebosim/gz-physics/issues/1117)).
DART rejects mixed library variants and conflicting scalar precision in linked
project-built Bullet components. Filename variants and scalar ABI are recorded
separately: unsuffixed libraries can use either precision. Imported libraries
must pass exactly one of the single/double C++ link probes, covering both
Collision and LinearMath; project-built components use their compile definitions.
Installed CMake components verify the recorded scalar ABI and filename variant.
C-only consumers use an isolated CXX probe without enabling CXX in the parent;
unsupported toolchain or target contexts remain unverified. Cross-directory
link scopes also remain unverified because their target names can bind to
directory-local imports or aliases; the finder rejects them before probe export
and fallback metadata serialization. Unsupported potentially active target
generator expressions, ambiguous or failed probes, and unknown linked-header
versions fail dependency validation rather than claiming compatibility. Inactive
install/platform branches and absent optional targets contribute nothing.
The installed version floor comes from the linked target's headers: 3.17 when
DART uses its newer heightfield constructor, otherwise 3.05.

Empty single-config builds keep their empty configuration when selecting
`dart.pc` libraries and use CMake's `NOCONFIG` imported-file mappings.

`dart.pc` requires `bullet-float64` when pkg-config finds that module and otherwise
publishes the selected libraries and include directories in its public flags.
Paths are escaped, and compile/link sysroots are removed at path boundaries.
Absolute library search paths from interface link options precede ordinary link
directories, matching CMake's ordering. The fallback accepts joined `-L` and
MSVC `LIBPATH` search options; unsupported interface options report an error
instead of publishing incomplete flags. A matching `bullet-float64.pc` module
supplies its own link flags and bypasses that fallback.
On Linux, sysroot library flags retain the exact selected filename using `-l:`;
unsupported platform/sysroot library serialization reports an error.
Scalar ABI checks retain normal interface targets and mapped link options in an
isolated target graph, independently of pkg-config serialization.
The two precisions
share GNU unique symbols, such as the static in `btTransform::getIdentity()`,
even across `RTLD_LOCAL` plugins, so a process must load only one Bullet
precision.

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
`DART_BUILD_IO=ON` (the default); it requires tinyxml2 10.0.0 or newer (the
Ubuntu 24.04 LTS floor), urdfdom and sdformat 14 (at least 14.8), 15 or 16.
Pixi uses sdformat 16.1 or newer within major 16 and urdfdom 6 by default.
SDF loading uses sdformat's validation, schema conversion and frame resolution;
tinyxml2 remains required by the other XML loaders.
CMake selects major 16, then 15, then 14 when several SDKs are available,
using versioned or unversioned package names. Installed IO components record
the selected package and major and require that same major, so a consumer
cannot silently link a different sdformat ABI. Core builds with IO disabled
do not discover sdformat.

Source builds using urdfdom below 5.1 should load only trusted URDF: those
versions can interpret URDF text as a printf format string in parser error
logging (fixed upstream in urdfdom 5.1.0, ros/urdfdom#243 and #244). Pixi
builds use urdfdom 6 and are unaffected. Linux wheels bundle urdfdom 6.0.1
and urdfdom_headers 3.0.1, with sdformat rebuilt against them.

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
ImGui v1.92.8 docking release, which configure fetches as a SHA256-pinned
tag archive, unpatched; its installed `dart/external/imgui/` headers use the
docking layout. For offline configures, extract that archive and pass the
folder that contains `imgui.h` with `FETCHCONTENT_SOURCE_DIR_DART_IMGUI`.
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
