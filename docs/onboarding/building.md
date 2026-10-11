# Building DART 6

Use Pixi tasks from the repository root.

```bash
pixi run config
pixi run build
pixi run build-py-dev
```

`pixi run config` configures the default CMake build under `build/`.
`pixi run build` builds the C++ libraries. `pixi run build-py-dev` builds the
Python bindings, including `dartpy.gui`, in the development tree.

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

## Filament GUI

`pixi run config` builds the Filament GUI (`DART_BUILD_GUI=ON`, the default)
and dartpy into `build/<env>/cpp/Release`. `pixi run test`, `test-py`, and
`test-all` leave out GUI tests that need a graphics device; `pixi run test-gui`
runs all GUI tests except native-window tests in the same tree. CTest uses
`gui-device` for the device-dependent exclusion; device-free GUI tests remain
in the default gates. See [testing guidance](testing.md) for native-window
tests.

```bash
pixi run demos        # Build dart-demos and open a native window
pixi run cycle-demos  # Replace every demo scene without a graphics context
pixi run capture      # Render a demo scene to a PNG without a window
pixi run test-gui     # GUI gate; see testing.md
```

`demos`, `cycle-demos`, and `capture` configure through
`pixi run config-demos`, which selects the bundled docking ImGui; alternating
with tasks that use `config` reconfigures the tree. `pixi run ex <example>`
builds and runs one GUI example. Python GUI tasks (`py-demos`, `py-ex`,
`agent-capture`) depend on `pixi run build-py-dev`. For a build without
graphics dependencies, `pixi run config-core` configures the headless core in
`build/<env>/cpp/core`; see [build system notes](build-system.md).

`DART_USE_SYSTEM_FILAMENT=ON` (the default) uses an installed Filament 1.77.3
SDK; a Linux SDK must use libstdc++. `OFF` uses a private pinned 1.77.3 SDK:
Linux builds it from downloaded sources with Clang 20 or newer and Ninja, and
macOS arm64 and Windows x64 download the upstream release archive. The GUI also
needs GLFW 3.4, libpng, libjpeg, and Dear ImGui 1.92.8 or newer (bundled by
default).
