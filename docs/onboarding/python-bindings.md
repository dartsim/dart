# Python Bindings

DART Python bindings are built through the Pixi/CMake workflow.

```bash
pixi run build-py-dev
pixi run test-py
```

Run Python tests when dependency, package, or target changes can alter dartpy
imports, linked components, or installed package behavior.
