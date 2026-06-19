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

For package or dependency changes, validate both the default configure/build
path and any feature environment affected by the change.
