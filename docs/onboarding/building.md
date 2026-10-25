# Building DART

Use Pixi tasks from the repository root.

```bash
pixi run config
pixi run build
pixi run build-py-dev
```

`pixi run config` configures the default CMake build under `build/`.
`pixi run build` builds the C++ libraries. `pixi run build-py-dev` builds the
Python bindings, including `dartpy.gui`, in the development tree.

Source builds on `main` require the Ubuntu 24.04 LTS toolchain floor: GCC 13.2,
LLVM Clang 18, Apple Clang 17 (Xcode 16.3), MSVC 19.32 (Visual Studio 2022
17.2), and CMake 3.28. IO builds require tinyxml2 10.0.0 or newer, the
Ubuntu 24.04 LTS version. `DART_BUILD_UTILS_ASSETS=ON` builds require OpenSSL
Crypto 3.0.13 or newer, also the Ubuntu 24.04 LTS version. Installed consumers
(`find_package(DART)`) still only need CMake 3.22.1. `release-6.20` keeps
Ubuntu 22.04. The platform table is in
`docs/readthedocs/dart/developer_guide/build.rst`. Stock Ubuntu 24.04 packages
plus OSRF's sdformat build `dart` and `dart-io`, as the nightly apt lane checks;
see the [support tiers](build-system.md#support-tiers).

Pixi installs `sccache` and DART automatically falls back to `ccache` when that
is the available launcher. Configure tasks leave compiler caching enabled by
default so repeated CMake builds can reuse cached C and C++ translation units.
Set `DART_COMPILER_CACHE=sccache` or `DART_COMPILER_CACHE=ccache` to request a
specific launcher; if the requested launcher is unavailable, DART disables
compiler caching instead of falling back to a different launcher. Set
`DART_DISABLE_COMPILER_CACHE=ON` when you need to compare uncached build
behavior or debug a cache-specific toolchain issue.

For package or dependency changes, validate both the default configure/build
path and any feature environment affected by the change.

`DART_BUILD_IO=ON` (the default) requires urdfdom 4.0.0 or newer for URDF
parsing. The default Pixi environment uses urdfdom 6. Ubuntu 22.04's apt
urdfdom (3.0) is below the floor: use Pixi, build urdfdom 4 or newer, or set
`DART_BUILD_IO=OFF`.

`DART_BUILD_UTILS_ASSETS=ON` (the Pixi tasks enable it) requires libcurl 8.5.0
or newer, the Ubuntu 24.04 LTS version.

Builds without `DART_USE_SYSTEM_DEPENDENCIES=ON`, which the Pixi tasks set,
fetch and build FCL and libccd at configure time and install them with DART;
set `DART_USE_SYSTEM_FCL=ON` to use an installed FCL instead. See the
[build system notes](build-system.md#dependency-sources).

## Filament GUI

`pixi run config` builds the Filament GUI (`DART_BUILD_GUI=ON`, the default)
and dartpy into `build/<env>/cpp/Release`. `pixi run test`, `test-py`, and
`test-all` leave out GUI tests that need a graphics device; `pixi run test-gui`
runs all GUI tests except native-window tests in the same tree. CTest uses
`gui-device` for the device-dependent exclusion; device-free GUI tests remain
in the default gates. See [testing guidance](testing.md) for native-window
tests.

```bash
pixi run demos        # Build dartpy and open the Python demo catalog
pixi run cycle-demos  # Replace every demo scene without a graphics context
pixi run capture      # Render a demo scene to a PNG without a window
pixi run test-gui     # GUI gate; see testing.md
```

`demos`, `cycle-demos`, and `capture` run `python -m examples.demos`
against the built dartpy and depend on `pixi run build-py-dev`.
That build configures through `pixi run config-py`, whose bundled ImGui is the
Dear ImGui docking release that configure fetches as a pinned tag archive. `capture` keeps
its output relative to the repository root (by default, `capture.png`).
The catalog has no C++ executable target; `dart-gui-app` is a private library
linked into dartpy and tests. Demo implementation lives entirely in Python,
with model helpers in `python/examples/demos/models/`.
With `-DDART_USE_SYSTEM_IMGUI=ON`, a vanilla ImGui retains `dart-gui` but
compiles out the app host and `dartpy.viewer`. Set
`FETCHCONTENT_SOURCE_DIR_DART_IMGUI` to the folder that contains `imgui.h` in
the extracted docking tag archive for an offline bundled-ImGui build.
`pixi run ex <example>` builds and runs one C++ example; `py-ex` and
`agent-capture` also depend on `pixi run build-py-dev`. For a build without
graphics dependencies, `pixi run config-core` configures the headless core in
`build/<env>/cpp/core`; see [build system notes](build-system.md).

`DART_USE_SYSTEM_FILAMENT=ON` (the default) uses an installed Filament 1.77.3
SDK; a Linux SDK must use libstdc++. `OFF` uses a private pinned 1.77.3 SDK:
Linux builds it from downloaded sources with Clang 20 or newer and Ninja, and
macOS arm64 and Windows x64 download the upstream release archive. The GUI also
needs GLFW 3.4 (fetched unless `DART_USE_SYSTEM_GLFW=ON`, which Pixi and macOS
select),
libpng, libjpeg, and Dear ImGui 1.92.8 or newer (bundled by default).

## Packaging DART 7

This section is for distribution and package-manager maintainers; the
[build system notes](build-system.md) hold the details behind it. A standalone
source build fetches some dependencies at configure time, while Pixi and
package builds use installed ones.

### Dependency switches

Each dependency that DART can fetch has a `DART_USE_SYSTEM_<X>` switch.
`DART_USE_SYSTEM_DEPENDENCIES=ON` (default `OFF`) defaults the first five
switches below to `ON`. Pixi's configure tasks set it, and package builds
should too. Each switch
keeps its cached value, so set the umbrella on the first configure of a fresh
build tree; an explicit `-D` always wins.

| Switch | Used when | Standalone default | With the umbrella | `OFF` fetches |
| --- | --- | --- | --- | --- |
| `DART_USE_SYSTEM_FCL` | Always | `OFF`; `ON` with a multi-configuration generator | `ON` | FCL 0.7.0 with the Linux wheels' GJK fix and a double-precision libccd 2.1, both shared; needs the `patch` program |
| `DART_USE_SYSTEM_GOOGLETEST` | `BUILD_TESTING=ON`, the default | `OFF` | `ON` | GoogleTest 1.17.0 |
| `DART_USE_SYSTEM_GOOGLEBENCHMARK` | `BUILD_TESTING=ON` | `OFF` | `ON` | Google Benchmark 1.10.0 |
| `DART_USE_SYSTEM_TRACY` | `DART_BUILD_PROFILE=ON` and `DART_PROFILE_TRACY=ON` | `OFF` | `ON` | Tracy 0.14.1, built with `TRACY_ENABLE=ON` |
| `DART_USE_SYSTEM_GLFW` | `DART_BUILD_GUI=ON`, the default | `OFF`; `ON` on macOS, which rejects `OFF` | `ON` | GLFW 3.4, built static (X11 only on Linux) |
| `DART_USE_SYSTEM_IMGUI` | `DART_BUILD_GUI=ON` | `OFF` | `OFF` | Dear ImGui v1.92.8-docking |
| `DART_USE_SYSTEM_FILAMENT` | `DART_BUILD_GUI=ON` | `ON` | `ON` | Filament 1.77.3: Linux builds the SDK from source with Clang 20 or newer and Ninja; macOS arm64 and 64-bit Windows download the release SDK |
| `DART_USE_SYSTEM_NANOBIND` | `DART_BUILD_DARTPY=ON` | `OFF` | `OFF` | nanobind 3.1.0 |
| `DART_USE_SYSTEM_ODE` | `DART_SKIP_ODE=OFF`, the default | `ON` | `ON` | ODE 0.16.6, shared and double precision, with the fetched libccd when FCL is fetched and the system one otherwise |
| `DART_USE_SYSTEM_BULLET` | `DART_SKIP_BULLET=OFF`, the default | `ON` | `ON` | Bullet 3.25, only shared double-precision `BulletCollision` and `LinearMath` |
| `DART_USE_SYSTEM_FMT` | Never | `ON` | `ON` | Nothing; a deprecated no-op through 7.x |

The wheel builds set their switches in `pyproject.toml` and
`scripts/wheel_build.py`. Each fetch downloads one archive and checks its
SHA-256:

| FetchContent name | Archive | SHA-256 |
| --- | --- | --- |
| `googletest` | `https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz` | `65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c` |
| `benchmark` | `https://github.com/google/benchmark/archive/refs/tags/v1.10.0.tar.gz` | `acc5c7c0fbed855c4aaa2d7c3c36fb82b0c15c28f4e151e3f679c2fc0aa404af` |
| `tracy` | `https://github.com/wolfpld/tracy/archive/refs/tags/v0.14.1.tar.gz` | `bf4af567e9c7524d07f3caa745fad02fb33bd5694f11910750382d1efbb251c1` |
| `glfw` | `https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip` | `b5ec004b2712fd08e8861dc271428f048775200a2df719ccf575143ba749a3e9` |
| `dart_imgui` | `https://github.com/ocornut/imgui/archive/refs/tags/v1.92.8-docking.tar.gz` | `ca0653454ed371b7a87e9b0bc29a5d15c9be7f7c0fbe778042fc48c71df1d3d8` |
| `dart_filament_sdk` (Linux) | `https://codeload.github.com/google/filament/tar.gz/refs/tags/v1.77.3` | `6f0904a5092760a04831a6be4fb53884488d6deded8c3e6aa541ba711c50e61b` |
| `dart_filament_sdk` (macOS arm64) | `https://github.com/google/filament/releases/download/v1.77.3/filament-v1.77.3-mac.tgz` | `c1ef146a7efa504fcdc3aa06aa2db9b2f89d91436fcb87f2b4c15c95862ffeb8` |
| `dart_filament_sdk` (Windows x64) | `https://github.com/google/filament/releases/download/v1.77.3/filament-v1.77.3-windows.tgz` | `e8705d817b7a884fa2d185080ceb80dd42c4ba339930d49dedc79b5761215285` |
| `nanobind` | `https://files.pythonhosted.org/packages/c7/1f/e28fae715401e8f57400e72aeaff4fde12046d31828164f34f61f0816da8/nanobind-3.1.0.tar.gz` | `6661e3dc1434d78781ccf7913442d90361dab395c3424efadb63bbb6ddea7ea3` |
| `ode` | `https://github.com/odedevs/ode/archive/refs/tags/0.16.6.tar.gz` | `91fa10c811db757c113ba5ced02e578587f84331d51041b9fd4cadb1522fcef6` |
| `bullet` | `https://github.com/bulletphysics/bullet3/archive/refs/tags/3.25.tar.gz` | `c45afb6399e3f68036ddb641c6bf6f552bf332d5ab6be62f7e6c54eda05ceb77` |
| `dart_libccd` | `https://github.com/danfis/libccd/archive/refs/tags/v2.1.tar.gz` | `542b6c47f522d581fbf39e51df32c7d1256ac0c626e7c2b41f1040d4b9d50d1e` |
| `dart_fcl` | `https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.tar.gz` | `90409e940b24045987506a6b239424a4222e2daf648c86dd146cbcb692ebdcbc` |

DART never fetches its other dependencies: Eigen, assimp,
octomap, tinyxml2, urdfdom, sdformat, libpng, libjpeg, libcurl, OpenSSL,
Python and NumPy always come from the system.

### Offline builds

debhelper and vcpkg configure with `FETCHCONTENT_FULLY_DISCONNECTED=ON`, and
Homebrew's `std_cmake_args` add a dependency provider that refuses
FetchContent. Such a build takes each dependency it reaches in the switch table
from its system switch or from a pre-seeded source tree: download the archive,
check its
SHA-256 (CMake does not check a seeded tree), extract it, and pass the
archive's top-level directory as `FETCHCONTENT_SOURCE_DIR_<NAME>`, with the
FetchContent name in uppercase, such as `FETCHCONTENT_SOURCE_DIR_DART_IMGUI`.
A seeded dependency also bypasses Homebrew's provider.

When a source tree is missing, the configure stops. CMake 3.30 and newer stop
inside FetchContent under policy CMP0170, which DART enables. With older CMake,
or an empty seeded directory, DART's own check stops it and names both
remedies:

```text
Cannot fetch googletest with FETCHCONTENT_FULLY_DISCONNECTED=ON: pre-seed
FETCHCONTENT_SOURCE_DIR_GOOGLETEST, or set DART_USE_SYSTEM_GOOGLETEST=ON to
use an installed package.
```

The umbrella leaves these to the packager:

- `BUILD_TESTING` defaults to `ON`, so a configure needs GoogleTest and
  Benchmark even when no test is built. `-DBUILD_TESTING=OFF` needs neither.
- The GUI fetches the bundled ImGui unless `DART_USE_SYSTEM_IMGUI=ON` finds
  ImGui 1.92.8 or newer. Without the docking branch, `dart-gui` builds without
  its demo app host and dartpy without `dartpy.viewer`.
- dartpy fetches nanobind unless `DART_USE_SYSTEM_NANOBIND=ON` finds nanobind
  3.1.x through `nanobind_DIR` or `python -m nanobind --cmake_dir`.
- The GUI installs Filament's license and notices. It copies them from the SDK
  root (`Filament_ROOT`, or the directory above the SDK's include directory)
  when that holds Filament's `LICENSE` and a `notices/` tree, as the SDK that
  Linux builds with `DART_USE_SYSTEM_FILAMENT=OFF` does. Otherwise configure
  downloads each missing file from `raw.githubusercontent.com` at `v1.77.3`
  and checks its SHA-256, even with `FETCHCONTENT_FULLY_DISCONNECTED=ON`;
  offline, configure stops with CMake's own error for the failed download.
  conda-forge's `filament` package has no `notices/` tree, and
  `cmake/DARTFilamentNotices.cmake` lists the files.

### Fetched dependencies in the installation

- GoogleTest and Benchmark are build-only and are not installed.
- nanobind links statically into the dartpy module.
- The bundled ImGui compiles into `dart-gui`, and its headers install under
  `include/dart/external/imgui/`. A system ImGui instead becomes a requirement
  of the installed `gui` component.
- Fetched GLFW is static and private to `dart-gui`, with its symbols hidden in
  shared builds. Only static builds install it, with its CMake package,
  because their exported `dart-gui` links it.
- The private Filament SDK links into `dart-gui` and is not installed;
  consumers of a static `dart-gui` set `Filament_ROOT` to it. Every GUI build
  installs Filament's license and notices under `share/licenses/dart/filament`.
- Fetched ODE installs `libode`, its headers under `include/ode`, `ode.pc`,
  `ode-config` and its CMake package beside DART, and installed
  `collision-ode` consumers need that prefix in `CMAKE_PREFIX_PATH`. It brings
  its own `uninstall` target; `dart-uninstall` removes the whole installation.
- Fetched Bullet installs `BulletCollision`, `LinearMath` and their headers
  directly under `include/`, plus a `bullet.pc` narrowed to those two
  libraries (not with MSVC), but no Bullet CMake package. An installed DART
  finds these files before the consumer's search paths.
- Fetched FCL and libccd install their shared libraries, headers, CMake
  packages and pkg-config files beside DART. An installed DART finds them
  before the consumer's search paths, and `libdart`'s `$ORIGIN` RUNPATH
  (`@loader_path` on macOS) loads them without `LD_LIBRARY_PATH`.
- Tracy links only into profiling builds (`DART_PROFILE_TRACY=ON`), which
  packages leave off.

Fetched ODE, Bullet, FCL and libccd keep their upstream library names. A
distribution that also packages them keeps `DART_USE_SYSTEM_ODE` and
`DART_USE_SYSTEM_BULLET` at their `ON` defaults, and sets the umbrella or
`DART_USE_SYSTEM_FCL=ON`, instead of shipping second copies.

### Scalar precision

ODE must use double precision: configure stops on a single-precision package.
FCL's GJK and EPA compute in libccd's precision, which must also be double:
compiling the FCL detector stops on `CCD_SINGLE` unless
`DART_ALLOW_SINGLE_PRECISION_LIBCCD=ON`. Bullet can be either. With system
Bullet 3.17 or newer, `DART_BULLET_FLOAT64` (default `ON`) links the
`-float64` libraries installed beside the single-precision ones, and the
installed CMake package and `dart.pc` require the same libraries. A process
must load only one Bullet precision ([details](build-system.md#headless-core)).
gz-physics' own Bullet plugins find Bullet through CMake's `FindBullet`, which
selects the unsuffixed libraries; `-DDART_BULLET_FLOAT64=OFF` keeps DART on
those.

| Channel | Bullet | ODE | libccd |
| --- | --- | --- | --- |
| DART's fetched copies | Double | Double | Double |
| Debian and Ubuntu | `-float64` beside single; DART links double | Double, except on armel, mips and mipsel | Double |
| conda-forge and Pixi | `-float64` beside single; DART links double | Double | Double (`libccd-double`) |
| Homebrew | Single in `lib`; double in `lib/bullet/double` without the suffix, so DART links single (unverified) | Double | Double |
| vcpkg | Single; double with the `double-precision` feature | Double on 64-bit targets | Single; double with `ccd[double-precision]` |
| FreeBSD packages | Not checked | Double with the `ode-double` package | Single |
| Linux wheels | Double | Double | Double |

### Support tiers

Ubuntu 24.04 LTS is the oldest supported platform. Its packages plus OSRF's
sdformat build `dart` and `dart-io` headless (Tier 1); the GUI and dartpy need
DART's fallbacks there (Tier 2); Pixi, conda-forge and the wheels build
everything (Tier 3). The [support tiers](build-system.md#support-tiers)
describe each tier and the nightly Tier 1 lane, whose configure is a working
offline package configure:

```bash
cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DDART_USE_SYSTEM_DEPENDENCIES=ON \
    -DDART_REQUIRE_OPTIONAL_DEPENDENCIES=ON \
    -DDART_BUILD_GUI=OFF -DDART_BUILD_DARTPY=OFF \
    -DDART_BUILD_UTILS_ASSETS=ON \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON
```

### Ubuntu 24.04 packages

The [library requirements](../readthedocs/dart/developer_guide/build.rst) list
DART 7's floors. Ubuntu 24.04 provides them as follows:

| Dependency | Ubuntu 24.04 |
| --- | --- |
| CMake | 3.28.3, `cmake` |
| Compiler | GCC 13.3, `build-essential` |
| Eigen | 3.4.0, `libeigen3-dev` |
| FCL, libccd | 0.7.0 and 2.1, `libfcl-dev`, `libccd-dev` |
| assimp | 5.3.1, `libassimp-dev` |
| octomap, optional | 1.9.7, `liboctomap-dev` |
| Bullet, optional | 3.24, `libbullet-dev` |
| ODE, optional | 0.16.2, `libode-dev` |
| tinyxml2 (`dart-io`) | 10.0.0, `libtinyxml2-dev` |
| urdfdom (`dart-io`) | 4.0.0, `liburdfdom-dev`, `liburdfdom-headers-dev` |
| sdformat, optional | None; OSRF's `libsdformat14-dev` |
| libcurl, OpenSSL Crypto (`DART_BUILD_UTILS_ASSETS`) | 8.5.0 and 3.0.13, `libcurl4-openssl-dev`, `libssl-dev` |
| GoogleTest, Benchmark (system) | 1.14.0 and 1.8.3, `libgtest-dev`, `libbenchmark-dev` |
| Filament (GUI) | None |
| GLFW (GUI) | 3.3.10; `DART_USE_SYSTEM_GLFW=OFF` fetches 3.4 |
| libpng, libjpeg (GUI) | 1.6.43, libjpeg-turbo 2.1.5 |
| Dear ImGui, system (GUI) | 1.90.1; use the bundled one |
| Python, NumPy (dartpy) | 3.12.3, 1.26.4 |
| nanobind (dartpy) | 1.9.2; use the fetched one |

### Removed dependencies

DART 7 uses none of spdlog, fmt, Boost, OpenSceneGraph, pybind11 (nanobind
replaces it) or TinyXML 1, and DART 6.20 had already stopped using GLUT,
IPOPT, NLopt, pagmo, FLANN and lodepng; recipes can drop all of them.
`DART_USE_SYSTEM_FMT` and `DART_SKIP_spdlog` remain as no-ops,
`DART_USE_SYSTEM_PYBIND11` is gone, and `DART_BUILD_GUI_OSG=ON` stops configure
(`OFF` still selects a headless build). URDF stays on urdfdom, with no
in-house reader; SDF stays on sdformat; and the asset retriever stays on
libcurl and OpenSSL.

### Recipe notes

These notes say what each channel's recipe needs for DART 7; check the
recipe's current state before applying them. For every package:

- Pass `-DDART_USE_SYSTEM_DEPENDENCIES=ON` on a fresh build tree, and
  `-DBUILD_TESTING=OFF` unless the package runs DART's tests with the
  channel's GoogleTest and Benchmark.
- gz-physics requests `collision-bullet`, `collision-ode`, `utils` and
  `utils-urdf`, so Gazebo-facing packages pass
  `-DDART_REQUIRE_OPTIONAL_DEPENDENCIES=ON`: a missing Bullet, ODE or sdformat
  then stops configure instead of dropping its feature.
- A channel without sdformat 14.8 to 16 passes `-DDART_SKIP_SDFORMAT=ON`. The
  strict switch needs it, and it keeps an sdformat that happens to be
  installed out of the package; `dart-io` then has no SDF support and no
  `io-sdf` component.
- Every build type except `Debug`, debhelper's `None` included, defaults
  `DART_TREAT_WARNINGS_AS_ERRORS` to `ON`; the Gazebo lanes and the conda-forge
  and vcpkg recipes pass `OFF`. `-DDART_BUILD_EXAMPLES=OFF` skips the C++
  examples, which MSVC builds by default.
- The development package depends on what the installed CMake package and
  `dart.pc` require: assimp, Eigen, FCL and libccd, octomap when found, Bullet
  and ODE when their backends are built, and tinyxml2 for `io`. Static
  installs add urdfdom, sdformat, libcurl and OpenSSL for `io`, and Filament,
  GLFW, libpng and libjpeg for `gui`; a system ImGui is required either way.
- DART installs its `LICENSE` and `NOTICE` under `share/licenses/dart`.

**conda-forge.**

- Drop `-DDART_BUILD_GUI_OSG=ON`, which stops a DART 7 configure, and the
  `fmt`, `spdlog`, `libboost-devel`, `openscenegraph` and `pybind11` host
  dependencies.
- The GUI needs `filament` 1.77.3 (Pixi pins its Linux `x11` build), `glfw`
  3.4, `libpng` and `libjpeg-turbo`, and its configure downloads Filament's
  notices (see [Offline builds](#offline-builds)).
- `DART_USE_SYSTEM_IMGUI=ON` needs `imgui` 1.92.8 or newer. Without the
  docking branch, the demo app host and `dartpy.viewer` drop out; the bundled
  ImGui instead needs its archive, for example as a second source passed
  through `FETCHCONTENT_SOURCE_DIR_DART_IMGUI`.
- SDF needs `libsdformat` (Pixi uses 16). dartpy needs Python 3.12 or newer,
  NumPy 1.26.4 or newer and `nanobind` 3.1 with `DART_USE_SYSTEM_NANOBIND=ON`
  (or its archive). `DART_BUILD_UTILS_ASSETS=ON`, as in Pixi, needs `libcurl`
  and `openssl`, and tests with the umbrella need `gtest` and `benchmark`.
- `bullet-cpp`, `libode` and `libccd-double` already meet the precision
  requirements.

**Homebrew.**

- `std_cmake_args` pass `-DBUILD_TESTING=OFF` and the provider that refuses
  FetchContent, so every fetched dependency needs a formula dependency or a
  resource passed through `FETCHCONTENT_SOURCE_DIR_<NAME>`.
- Without a Filament 1.77.3 formula, pass `-DDART_BUILD_GUI=OFF`, which also
  makes the ImGui resource unnecessary. Without an sdformat 14.8 to 16
  dependency, also pass `-DDART_SKIP_SDFORMAT=ON`.
- Drop `flann`, `fmt`, `ipopt`, `nlopt`, `open-scene-graph` and `spdlog`.
- `DART_BULLET_FLOAT64` finds only `-float64`-suffixed libraries, so a
  double-precision Bullet installed under other names is not selected by it.

**vcpkg.**

- `vcpkg_cmake_configure` passes `FETCHCONTENT_FULLY_DISCONNECTED=ON`.
- `dart-io`, and with it the `utils` and `utils-urdf` features that the
  `gz-physics` port requests, needs a `urdfdom` port at 4.0.0 or newer.
- DART's build stops on a single-precision libccd, so depend on
  `ccd[double-precision]` if the `fcl` port does not.
- Without a Filament 1.77.3 port, pass `-DDART_BUILD_GUI=OFF`. An `sdformat`
  port at 14.8 to 16 can back SDF; otherwise pass `-DDART_SKIP_SDFORMAT=ON`.
- Windows builds are static, so the installed package also requires urdfdom,
  and sdformat, libcurl and OpenSSL when they are enabled.
- The `spdlog` feature and the `fmt` and `lodepng` dependencies no longer
  apply.

**Debian and OSRF.**

- debhelper's CMake build system, which pybuild also drives, passes
  `FETCHCONTENT_FULLY_DISCONNECTED=ON` and `CMAKE_BUILD_TYPE=None`.
- The [Tier 1 configure](#support-tiers) is the reference. Build-Depends:
  `libassimp-dev`, `libbullet-dev`, `libccd-dev`, `libeigen3-dev`,
  `libfcl-dev`, `liboctomap-dev`, `libode-dev`, `libtinyxml2-dev`,
  `liburdfdom-dev`, `liburdfdom-headers-dev` and `pkg-config`, plus
  `libcurl4-openssl-dev` and `libssl-dev` for the asset retriever and OSRF's
  `libsdformat14-dev` for SDF. Without sdformat 14.8 to 16, a package passes
  `-DDART_SKIP_SDFORMAT=ON`.
- Tests need `libgtest-dev` and `libbenchmark-dev`; a package that sets only
  `DART_USE_SYSTEM_GOOGLETEST=ON` still fetches Benchmark, and FCL unless it
  also sets `DART_USE_SYSTEM_FCL=ON`. The umbrella covers both. `test_Distance`
  aborts on the archive's FCL, which is built with assertions, and the Tier 1
  lane excludes it.
- dartpy needs the GUI, whose Filament SDK Ubuntu 24.04 lacks, and nanobind
  3.1, where 24.04 has 1.9.2. `DART_USE_SYSTEM_PYBIND11` no longer exists.

**gz_dartsim_vendor.**

- With `BUILD_TESTING=OFF`, as `ament_vendor` passes, and the umbrella on,
  DART fetches nothing. `DART_BUILD_GUI_OSG=FALSE` still selects a headless
  build, and `DART_BUILD_EXTRAS` has no effect.
- It declares no sdformat, so pass `-DDART_SKIP_SDFORMAT=ON`, and
  `-DDART_REQUIRE_OPTIONAL_DEPENDENCIES=ON` to keep the Bullet and ODE backends
  that gz-physics needs from dropping out.
- `fmt`, Boost and `lz4` are not DART 7 dependencies. DART 7 needs
  `tinyxml2` 10.0.0 and `liburdfdom-dev` 4.0.0 or newer, which Ubuntu 24.04
  has.
