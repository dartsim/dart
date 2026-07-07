# Building DART 6.20

Use Pixi tasks from the repository root.

```bash
pixi run config
pixi run build
pixi run build-py-dev
```

`pixi run config` configures the default CMake build under `build/`.
`pixi run build` builds the C++ libraries. `pixi run build-py-dev` builds the
Python bindings in the development tree.

Pixi installs `sccache` and DART automatically falls back to `ccache` when that
is the available launcher. Configure tasks leave compiler caching enabled by
default so repeated CMake builds can reuse cached C and C++ translation units.
Set `DART_COMPILER_CACHE=sccache` or `DART_COMPILER_CACHE=ccache` to request a
specific launcher. Set `DART_DISABLE_COMPILER_CACHE=ON` when you need to compare
uncached build behavior or debug a cache-specific toolchain issue.

For package or dependency changes, validate both the default configure/build
path and any feature environment affected by the change.
