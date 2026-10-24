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
`docs/readthedocs/dart/developer_guide/build.rst`.

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
needs GLFW 3.4, libpng, libjpeg, and Dear ImGui 1.92.8 or newer (bundled by
default).
