# Python Bindings

DART Python bindings are built through the Pixi/CMake workflow.

```bash
pixi run build-py-dev
pixi run test-py
```

Run Python tests when dependency, package, or target changes can alter dartpy
imports, linked components, or installed package behavior.

## Wheels

Wheel packaging is Pixi-managed on this branch. `wheel-build-core`,
`wheel-repair-<platform>-core`, `wheel-verify-core`, and `wheel-test-core`
implement the pipeline, and the `py310-wheel`..`py313-wheel` environments
expose `wheel-build` / `wheel-repair` / `wheel-verify` / `wheel-test` per
Python version. `.github/workflows/publish_dartpy.yml` runs the same tasks
for released wheels; reproduce a CI wheel step locally with
`pixi run -e <pyXY-wheel> wheel-build` and friends.
